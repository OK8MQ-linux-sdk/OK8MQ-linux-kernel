/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/poll.h>
 

struct gpio_input_pin {
        int             gpio;
        int             active_low;
        int             debounce_interval;
        const char      *desc;
};


struct gpio_input_platform_data {
        struct gpio_input_pin   *pins;
        int                     npins;
};

struct gpio_input_dev {
	struct gpio_input_pin 	*pin;
	int 			timer_debounce;     /* in msecs */
	int 			state;
	int			ready;

	wait_queue_head_t 	outq;
	struct mutex 		mutex;
	struct timer_list 	timer;
	struct work_struct 	work;
	struct list_head 	di_list;
		
	dev_t    		di_dev;
	struct cdev 		di_cdev;
	struct device 		*di_device;
	char      		di_name[32];
};

static int gpio_major;
static struct list_head gpio_input_devs = LIST_HEAD_INIT(gpio_input_devs);
static struct class *gpio_input_class;

static void gpio_input_report_val(struct gpio_input_dev *g_in_dev)
{
	struct gpio_input_pin *pins = g_in_dev->pin;
	int state = (gpio_get_value_cansleep(pins->gpio) ? 1 : 0) ^ pins->active_low;
	if(g_in_dev->state == state)
		return;

	mutex_lock(&g_in_dev->mutex);
	g_in_dev->state = state;
	g_in_dev->ready = 1; 
	mutex_unlock(&g_in_dev->mutex);

	wake_up_interruptible(&g_in_dev->outq);
}

static void gpio_input_work_func(struct work_struct *work)
{
	struct gpio_input_dev *data =
		container_of(work, struct gpio_input_dev, work);

	gpio_input_report_val(data);
}

static void gpio_input_timer(struct timer_list *t)
{
	struct gpio_input_dev *data = from_timer(data, t, timer);

	schedule_work(&data->work);
}

static irqreturn_t gpio_input_isr(int irq, void *dev_id)
{
	struct gpio_input_dev *g_in_dev = dev_id;
	struct gpio_input_pin *pins = g_in_dev->pin;

	BUG_ON(irq != gpio_to_irq(pins->gpio));

	if (g_in_dev->timer_debounce)
		mod_timer(&g_in_dev->timer,
			jiffies + msecs_to_jiffies(g_in_dev->timer_debounce));
	else
		schedule_work(&g_in_dev->work);

	return IRQ_HANDLED;
}

static int gpio_input_setup_key(struct gpio_input_dev *g_in_dev)
{
	struct gpio_input_pin *pins = g_in_dev->pin;
	struct device *dev = g_in_dev->di_device;
	const char *desc = pins->desc ? pins->desc : "gpio_input";
	unsigned long irqflags;
	int irq, error;

	INIT_WORK(&g_in_dev->work, gpio_input_work_func);
	mutex_init(&g_in_dev->mutex);
	init_waitqueue_head(&g_in_dev->outq);

	error = gpio_request(pins->gpio, desc);
	if (error < 0) {
		dev_err(dev, "failed to request GPIO %d, error %d\n",
			pins->gpio, error);
		goto fail1;
	}

	error = gpio_direction_input(pins->gpio);
	if (error < 0) {
		dev_err(dev, "failed to configure"
			" direction for GPIO %d, error %d\n",
			pins->gpio, error);
		goto fail2;
	}
	g_in_dev->state = (gpio_get_value_cansleep(pins->gpio) ? 1 : 0) ^ pins->active_low;

	if (pins->debounce_interval) {
		error = gpio_set_debounce(pins->gpio,
					  pins->debounce_interval * 1000);
		/* use timer if gpiolib doesn't provide debounce */
		if (error < 0) {
			g_in_dev->timer_debounce = pins->debounce_interval;
			timer_setup(&g_in_dev->timer, gpio_input_timer, 0);
		}
	}

	irq = gpio_to_irq(pins->gpio);
	if (irq < 0) {
		error = irq;
		dev_err(dev, "Unable to get irq number for GPIO %d, error %d\n",
			pins->gpio, error);
		goto fail2;
	}

	irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;
	/*
	 * If platform has specified that the button can be disabled,
	 * we don't want it to share the interrupt line.
	 */

	/*
	 * If platform has specified that the button can wake up the system,
	 * for example, the power key which usually use to wake up the system
	 * from suspend, we add the IRQF_EARLY_RESUME flag to this irq, so
	 * that the power key press can be handled and reported as early as
	 * possible. Some platform like Android need to get the power key
	 * event early to reume some devcies like framebuffer and etc.
	 */

	error = request_any_context_irq(irq, gpio_input_isr, irqflags, desc, g_in_dev);
	if (error < 0) {
		dev_err(dev, "Unable to claim irq %d; error %d\n",
			irq, error);
		goto fail2;
	}

	return 0;

fail2:
	if(g_in_dev->timer_debounce)
        	del_timer_sync(&g_in_dev->timer);
        cancel_work_sync(&g_in_dev->work);
	gpio_free(pins->gpio);
fail1:
	return error;
}

static int gpio_input_open(struct inode *inodp,struct file *filp)
{
	struct cdev *cdev = inodp->i_cdev;
        struct gpio_input_dev *g_in_dev = container_of(cdev, struct gpio_input_dev, di_cdev);

        filp->private_data = g_in_dev;
	
	mutex_lock(&g_in_dev->mutex);
        g_in_dev->ready = 1;
        mutex_unlock(&g_in_dev->mutex);
	
	return 0;
}
static int gpio_input_release(struct inode *inodp,struct file *filp)
{
	return 0;
}
ssize_t gpio_input_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	struct gpio_input_dev *g_in_dev = filp->private_data;
	ssize_t retval;

	if(filp->f_flags & O_NONBLOCK) {
		retval = put_user(g_in_dev->state, (int __user *)buf);
		goto out;
	}

	if (wait_event_interruptible(g_in_dev->outq, g_in_dev->ready))
 	{
		return - ERESTARTSYS;
	}	
 
	mutex_lock(&g_in_dev->mutex);
	retval = put_user(g_in_dev->state, (int __user *)buf);
	g_in_dev->ready = 0; 
	mutex_unlock(&g_in_dev->mutex);
out:
 	
 	return retval;
}
static unsigned int gpio_input_poll(struct file *filp, poll_table *wait)
 
{
	struct gpio_input_dev *g_in_dev = filp->private_data;
	unsigned int mask = 0;

	poll_wait(filp, &g_in_dev->outq, wait);

	if(g_in_dev->ready)
	{
		mask |= POLLIN | POLLRDNORM;
	}

	return mask;
} 


static struct file_operations gpio_input_fops = {
	.owner 		= THIS_MODULE,
	.open 		= gpio_input_open,
	.release 	= gpio_input_release,
	.read 		= gpio_input_read,
	.poll		= gpio_input_poll,
};

static ssize_t gpio_state_show(struct device *dev,
		                struct device_attribute *attr, char *buf)
{
	struct gpio_input_dev *g_in_dev = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", g_in_dev->state);
}

static DEVICE_ATTR(state, 0444, gpio_state_show, NULL);

static struct attribute *gpio_class_attrs[] = {
	&dev_attr_state.attr,
	NULL,
};

static const struct attribute_group gpio_group = {
	.attrs = gpio_class_attrs,
};

static const struct attribute_group *gpio_groups[] = {
	&gpio_group,
	NULL,
};

static void gpio_input_cleanup_devs(void)
{
        struct gpio_input_dev *cur, *n;

        list_for_each_entry_safe(cur, n, &gpio_input_devs, di_list) {
		int irq = gpio_to_irq(cur->pin->gpio);
                free_irq(irq, cur);
                if (cur->timer_debounce)
                        del_timer_sync(&cur->timer);
                cancel_work_sync(&cur->work);
                gpio_free(cur->pin->gpio);
		if (cur->di_device) {
			device_remove_groups(cur->di_device, gpio_groups);
                	cdev_del(&cur->di_cdev);
			device_del(cur->di_device);
		}
                list_del(&cur->di_list);
                kfree(cur);
        }
}

/*
 * Handlers for alternative sources of platform_data
 */

#ifdef CONFIG_OF
/*
 * Translate OpenFirmware node properties into platform_data
 */
static struct gpio_input_platform_data *
gpio_inputs_get_devtree_pdata(struct device *dev)
{
        struct device_node *node, *pp;
        struct gpio_input_platform_data *pdata;
        struct gpio_input_pin *pin;
        int error;
        int npins;
        int i;

        node = dev->of_node;
        if (!node)
                return ERR_PTR(-ENODEV);

        npins = of_get_child_count(node);
        if (npins == 0)
                return ERR_PTR(-ENODEV);

        pdata = devm_kzalloc(dev,
                             sizeof(*pdata) + npins * sizeof(*pin),
                             GFP_KERNEL);
        if (!pdata)
                return ERR_PTR(-ENOMEM);

        pdata->pins = (struct gpio_input_pin *)(pdata + 1);
        pdata->npins = npins;


        i = 0;
        for_each_child_of_node(node, pp) {
                enum of_gpio_flags flags;

                pin = &pdata->pins[i++];

                pin->gpio = of_get_gpio_flags(pp, 0, &flags);
                if (pin->gpio < 0) {
                        error = pin->gpio;
                        if (error != -ENOENT) {
                                if (error != -EPROBE_DEFER)
                                        dev_err(dev,
                                                "Failed to get gpio flags, error: %d\n",
                                                error);
                                return ERR_PTR(error);
                        }
                } else {
                        pin->active_low = flags & OF_GPIO_ACTIVE_LOW;
                }


                if (!gpio_is_valid(pin->gpio)) {
                        dev_err(dev, "Found pin without gpios\n");
                        return ERR_PTR(-EINVAL);
                }

                pin->desc = of_get_property(pp, "label", NULL);

                if (of_property_read_u32(pp, "debounce-interval",
                                         &pin->debounce_interval))
                        pin->debounce_interval = 5;
        }

        if (pdata->npins == 0)
                return ERR_PTR(-EINVAL);

        return pdata;
}

static const struct of_device_id gpio_input_of_match[] = {
        { .compatible = "gpio-input", },
        { },
};
MODULE_DEVICE_TABLE(of, gpio_input_of_match);

#else

static inline struct gpio_keys_platform_data *
gpio_keys_get_devtree_pdata(struct device *dev)
{
        return ERR_PTR(-ENODEV);
}

#endif


static int gpio_input_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gpio_input_platform_data *pdata = pdev->dev.platform_data;
	int i;
	int result = 0;
	dev_t major_dev;
	
        if (!pdata) {
                pdata = gpio_inputs_get_devtree_pdata(dev);
                if (IS_ERR(pdata))
                        return PTR_ERR(pdata);
        }

	gpio_input_class = class_create(THIS_MODULE, "gpio-input");
        if (IS_ERR(gpio_input_class)) {
                printk(KERN_ERR "class_create() failed for gpio_input_class\n");
                goto out_err1;
        }

	result = alloc_chrdev_region(&major_dev, 0, pdata->npins, "gpio_input");
	gpio_major = MAJOR(major_dev);
	if (result < 0) {
                printk(KERN_ERR "alloc_chrdev_region() failed for gpio input\n");
                goto out_err2;
        }

 	for (i = 0 ; i < pdata->npins; i++) {
                struct gpio_input_dev *cur = kzalloc(sizeof(struct gpio_input_dev),
                                              GFP_KERNEL);
                if (!cur) {
                        printk(KERN_ERR "Unable to alloc gpio input dev\n");
                        result = -ENOMEM;
                        goto out_err3;
                }

                cur->pin  = &pdata->pins[i];
                cur->di_dev = MKDEV(gpio_major, i);
                snprintf(cur->di_name, 32, "gpio_input_%d", i);
                cdev_init(&cur->di_cdev, &gpio_input_fops);
                result = cdev_add(&cur->di_cdev, cur->di_dev, 1);
                if (result) {
                        kfree(cur);
                        goto out_err3;
                }

		cur->di_device = device_create(gpio_input_class, NULL, cur->di_dev,
                                                cur, cur->di_name);
                if (IS_ERR(cur->di_device)) {
                        printk(KERN_ERR "device_create failed for %s\n",
                               cur->di_name);
                        cdev_del(&cur->di_cdev);
                        kfree(cur);
                        goto out_err3;
                }

		if(device_add_groups(cur->di_device, gpio_groups)) {
			printk(KERN_ERR "device_add_groups() failed\n");
			cdev_del(&cur->di_cdev);
			device_del(cur->di_device);
                        kfree(cur);
		}

		result = gpio_input_setup_key(cur);
		if (result) {
                        cdev_del(&cur->di_cdev);
			device_del(cur->di_device);
                        kfree(cur);
			goto out_err3;
		}

                list_add_tail(&cur->di_list, &gpio_input_devs);
		
        }


	return 0;

 out_err3:
	gpio_input_cleanup_devs();
	unregister_chrdev_region(major_dev, pdata->npins);
 out_err2:
	class_destroy(gpio_input_class);
 out_err1:
	return result;
}

static int gpio_input_remove(struct platform_device *pdev)
{
	struct gpio_input_platform_data *pdata = pdev->dev.platform_data;

	gpio_input_cleanup_devs();

	if (gpio_input_class)
                class_destroy(gpio_input_class);

	if (gpio_major)
                unregister_chrdev_region(MKDEV(gpio_major, 0), pdata->npins);
	return 0;
}


static int gpio_input_suspend(struct device *dev)
{

	return 0;
}

static int gpio_input_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops gpio_input_pm_ops = {
	.suspend	= gpio_input_suspend,
	.resume		= gpio_input_resume,
};

static struct platform_driver gpio_input_device_driver = {
	.probe		= gpio_input_probe,
	.remove		= gpio_input_remove,
	.driver		= {
		.name	= "gpio-input",
		.pm	= &gpio_input_pm_ops,
		.of_match_table = of_match_ptr(gpio_input_of_match),
	}
};

static int __init gpio_input_init(void)
{
	return platform_driver_register(&gpio_input_device_driver);
}

static void __exit gpio_input_exit(void)
{
	platform_driver_unregister(&gpio_input_device_driver);
}

module_init(gpio_input_init);
module_exit(gpio_input_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Phil Blundell <pb@handhelds.org>");
MODULE_DESCRIPTION("Keyboard driver for CPU GPIOs");
MODULE_ALIAS("platform:gpio-input");
