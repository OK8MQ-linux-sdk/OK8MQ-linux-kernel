/*
 * Analog Devices ADV7511 HDMI transmitter driver
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/slab.h>
#include <video/of_videomode.h>
#include <video/videomode.h>
#include <linux/string.h>

#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

#include "lt8912.h"
#define HDMI_1280_720	1
#define HDMI_1920_1080	2
#define HDMI_640_480	3
#define LVDS_1280_800	4
#define CUSTOM_MODE	5
#define MATCH_ALL	99

static int lt8912_write_reg(struct i2c_client *client, u8 reg, u8 val);
static int sync_mode = 0;
struct lt8912_reg_cfg {
	u8 reg;
	u8 val;
	int sleep_in_ms;
};

static struct videomode vm = {
	.pixelclock = 74250000,
        .hactive = 1280,
        .hfront_porch = 220,
        .hback_porch = 110,
        .hsync_len = 40,

        .vactive = 720,
        .vfront_porch = 2,
        .vback_porch = 5,
        .vsync_len = 5,
};

static const struct display_timing rad_default_timing = {
        .pixelclock = { 66000000, 132000000, 132000000 },
        .hactive = { 1080, 1080, 1080 },
        .hfront_porch = { 20, 20, 20 },
        .hsync_len = { 2, 2, 2 },
        .hback_porch = { 34, 34, 34 },
        .vactive = { 1920, 1920, 1920 },
        .vfront_porch = { 10, 10, 10 },
        .vsync_len = { 2, 2, 2 },
        .vback_porch = { 4, 4, 4 },
};

static struct lt8912_reg_cfg lt8912_init_setup[] = {
	{0x02, 0xf7, 0}, //lvds pll reset.
	{0x08, 0xff, 0},
	{0x09, 0xff, 0},
	{0x0a, 0xff, 0},
	{0x0b, 0x7c, 0},
	{0x0c, 0xff, 0},
	{0x31, 0xb1, 0},  
	{0x32, 0xb1, 0}, 
	{0x33, 0x17, 0},  
	{0x37, 0x00, 0},
	{0x38, 0x22, 0},
	{0x60, 0x82, 0},
	{0x39, 0x45, 0},
	{0x3a, 0x00, 0}, //20180718
	{0x3b, 0x00, 0},
	{0x44, 0x31, 0},
	{0x55, 0x44, 0},
	{0x57, 0x01, 0},
	{0x5a, 0x02, 0},
	{0x3e, 0xd6, 0},  //0xde.  //0xf6 = pn swap
	{0x3f, 0xd4, 0},
	{0x41, 0x3c, 0},
};

static struct lt8912_reg_cfg lt8912_mipi_basic_set[] = {
	{0x12, 0x04, 0}, 
	{0x13, 0x00, 0},  //0x02 = mipi 2 lane
	{0x14, 0x00, 0}, 
	{0x15, 0x00, 0},
	{0x1a, 0x03, 0}, 
	{0x1b, 0x03, 0}, 
};

static void lt8912_custom_mode(struct i2c_client *cec_dev) {

        uint8_t hs_mode = 0x01; //0x10
        uint8_t time_PR_SOT = 0x0a; //0x11
        uint8_t hsync_width = vm.hsync_len; //0x18
        uint8_t vsync_width = vm.vsync_len; //0x19
        uint8_t xres_low = vm.hactive & 0xff; //0x1c
        uint8_t xres_high = vm.hactive >> 8; //0x1d
        uint8_t write_fifo_lenth = 0x2f; //0x2f
        uint8_t xres_total_low = (vm.hactive + vm.hfront_porch + vm.hback_porch + vm.hsync_len) & 0xff; //0x34
        uint8_t xres_total_high = (vm.hactive + vm.hfront_porch + vm.hback_porch + vm.hsync_len) >> 8; //0x35
        uint8_t yres_total_low = (vm.vactive + vm.vfront_porch + vm.vback_porch + vm.vsync_len) & 0xff; //0x36
        uint8_t yres_total_high = (vm.vactive + vm.vfront_porch + vm.vback_porch + vm.vsync_len) >> 8; //0x37
        uint8_t v_back_porch_low = vm.vback_porch & 0xff; //0x38
        uint8_t v_back_porch_high = vm.vback_porch >> 8; //0x39
        uint8_t v_front_porch_low = vm.vfront_porch & 0xff; //0x3a
        uint8_t v_front_porch_high = vm.vfront_porch >> 8; //0x3b
        uint8_t h_back_porch_low = vm.hback_porch & 0xff; //0x3c
        uint8_t h_back_porch_high = vm.hback_porch >> 8; //0x3d
        uint8_t h_front_porch_low = vm.hfront_porch & 0xff; //0x3e
        uint8_t h_front_porch_high = vm.hfront_porch >> 8; //0x3f

        lt8912_write_reg(cec_dev, 0x10, hs_mode);
        lt8912_write_reg(cec_dev, 0x11, time_PR_SOT);
        lt8912_write_reg(cec_dev, 0x18, hsync_width);
        lt8912_write_reg(cec_dev, 0x19, vsync_width);
        lt8912_write_reg(cec_dev, 0x1c, xres_low);
        lt8912_write_reg(cec_dev, 0x1d, xres_high);
        lt8912_write_reg(cec_dev, 0x2f, write_fifo_lenth);
        lt8912_write_reg(cec_dev, 0x34, xres_total_low);
        lt8912_write_reg(cec_dev, 0x35, xres_total_high);
        lt8912_write_reg(cec_dev, 0x36, yres_total_low);
        lt8912_write_reg(cec_dev, 0x37, yres_total_high);
        lt8912_write_reg(cec_dev, 0x38, v_back_porch_low);
        lt8912_write_reg(cec_dev, 0x39, v_back_porch_high);
        lt8912_write_reg(cec_dev, 0x3a, v_front_porch_low);
        lt8912_write_reg(cec_dev, 0x3b, v_front_porch_high);
        lt8912_write_reg(cec_dev, 0x3c, h_back_porch_low);
        lt8912_write_reg(cec_dev, 0x3d, h_back_porch_high);
        lt8912_write_reg(cec_dev, 0x3e, h_front_porch_low);
        lt8912_write_reg(cec_dev, 0x3f, h_front_porch_high);
}

static struct lt8912_reg_cfg lt8912_ddsconfig[] = {	
	{0x4e, 0xff, 0},
	{0x4f, 0x56, 0},
	{0x50, 0x69, 0},
	{0x51, 0x80, 0},
	{0x1f, 0x5e, 0},
	{0x20, 0x01, 0},
	{0x21, 0x2c, 0},
	{0x22, 0x01, 0},
	{0x23, 0xfa, 0},
	{0x24, 0x00, 0},
	{0x25, 0xc8, 0},
	{0x26, 0x00, 0},
	{0x27, 0x5e, 0},
	{0x28, 0x01, 0},
	{0x29, 0x2c, 0},
	{0x2a, 0x01, 0},
	{0x2b, 0xfa, 0},
	{0x2c, 0x00, 0},
	{0x2d, 0xc8, 0},
	{0x2e, 0x00, 0},
	{0x42, 0x64, 0},
	{0x43, 0x00, 0},
	{0x44, 0x04, 0},
	{0x45, 0x00, 0},
	{0x46, 0x59, 0},
	{0x47, 0x00, 0},
	{0x48, 0xf2, 0},
	{0x49, 0x06, 0},
	{0x4a, 0x00, 0},
	{0x4b, 0x72, 0},
	{0x4c, 0x45, 0},
	{0x4d, 0x00, 0},
	{0x52, 0x08, 0},
	{0x53, 0x00, 0},
	{0x54, 0xb2, 0},
	{0x55, 0x00, 0},
	{0x56, 0xe4, 0},
	{0x57, 0x0d, 0},
	{0x58, 0x00, 0},
	{0x59, 0xe4, 0},
	{0x5a, 0x8a, 0},
	{0x5b, 0x00, 0},
	{0x5c, 0x34, 0},
	{0x1e, 0x4f, 0},
	{0x51, 0x00, 0},
};	


static struct lt8912_reg_cfg lt8912_rxlogicres[] = {	
	//lt8912_rxlogicres
	{0x03, 0x7f, 10},
	{0x03, 0xff, 0},
};

static struct lt8912_reg_cfg lt8912_rxlogicres2[] = {	
	//lt8912_rxlogicres
	{0x51, 0x80, 10},
	{0x51, 0x00, 0},
};

static struct lt8912_reg_cfg lt8912_lvds_bypass_cfg[] = {	
//	{0x44, 0x30, 0},
//	{0x51, 0x05, 0},
	{0x50, 0x24, 0},//cp=50uA
	{0x51, 0x2d, 0},//Pix_clk as reference,second order passive LPF PLL
	{0x52, 0x04, 0},//loopdiv=0;use second-order PLL
	{0x69, 0x0e, 0},//CP_PRESET_DIV_RATIO
	{0x69, 0x8e, 0},
	{0x6a, 0x00, 0},
	{0x6c, 0xb8, 0},//RGD_CP_SOFT_K_EN,RGD_CP_SOFT_K[13:8]
	{0x6b, 0x51, 0},
	{0x04, 0xfb, 0},//core pll reset
	{0x04, 0xff, 0},
	{0x7f, 0x00, 0},//disable scaler
	{0xa8, 0x13, 0},//0x13 : JEIDA, 0x33:VSEA

	{0x03, 0x7f, 0},//lvds pll reset
	{0x03, 0xff, 10},
	{0x05, 0xfb, 0},
	{0x05, 0xff, 10},
};


static struct drm_display_mode forlinx_mipi7_mode[5] = {
	{
		.clock = 74250,
		.hdisplay = 1280,
		.hsync_start = 1280 + 110,
		.hsync_end = 1280 + 110 + 40,
		.htotal = 1280 + 110 + 40 + 220,
		.vdisplay = 720,
		.vsync_start = 720 + 5,
		.vsync_end = 720 + 5 + 5,
		.vtotal = 720 + 5 + 5 + 20,
		.vrefresh = 60,
	},
	{
		.clock = 148500,
		.hdisplay = 1920,
		.hsync_start = 1920 + 88,
		.hsync_end = 1920 + 88 + 44,
		.htotal = 1920 + 88 + 44 + 148,
		.vdisplay = 1080,
		.vsync_start = 1080 + 4,
		.vsync_end = 1080 + 4 + 5,
		.vtotal = 1080 + 4 + 5 + 36,
		.vrefresh = 60,
	},
	{
		.clock = 25000,
		.hdisplay = 640,
		.hsync_start = 640 + 8,
		.hsync_end = 640 + 8 + 96,
		.htotal = 640 + 8 + 96 + 40,
		.vdisplay = 480,
		.vsync_start = 480 + 33,
		.vsync_end = 480 + 33 + 2,
		.vtotal = 480 + 33 + 2 + 10,
		.vrefresh = 60,
	},
	{
		.clock = 74250,
		.hdisplay = 1280,
		.hsync_start = 1280 + 64,
		.hsync_end = 1280 + 64 + 136,
		.htotal = 1280 + 64 + 136 + 200,
		.vdisplay = 800,
		.vsync_start = 800 + 1,
		.vsync_end = 800 + 1 + 3,
		.vtotal = 800 + 1 + 3 + 24,
		.vrefresh = 60,
	},
	{
		.name = "lt8912_custom",
		.clock = 74250,
		.hdisplay = 1280,
		.hsync_start = 1280 + 64,
		.hsync_end = 1280 + 64 + 136,
		.htotal = 1280 + 64 + 136 + 200,
		.vdisplay = 800,
		.vsync_start = 800 + 1,
		.vsync_end = 800 + 1 + 3,
		.vtotal = 800 + 1 + 3 + 24,
		.vrefresh = 60,
	}
};

struct panel_desc {
	const struct drm_display_mode *modes;
	unsigned int num_modes;
	const struct display_timing *timings;
	unsigned int num_timings;

	unsigned int bpc;

	/**
	 * @width: width (in millimeters) of the panel's active display area
	 * @height: height (in millimeters) of the panel's active display area
	 */
	struct {
		unsigned int width;
		unsigned int height;
	} size;

	/**
	 * @prepare: the time (in milliseconds) that it takes for the panel to
	 *           become ready and start receiving video data
	 * @enable: the time (in milliseconds) that it takes for the panel to
	 *          display the first valid frame after starting to receive
	 *          video data
	 * @disable: the time (in milliseconds) that it takes for the panel to
	 *           turn the display off (no content is visible)
	 * @unprepare: the time (in milliseconds) that it takes for the panel
	 *             to power itself down completely
	 */
	struct {
		unsigned int prepare;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
	} delay;

	u32 bus_format;
	u32 bus_flags;
};

struct panel_desc_dsi {
	struct panel_desc desc;

	unsigned long flags;
	enum mipi_dsi_pixel_format format;
	unsigned int lanes;
};

static const struct panel_desc_dsi forlinx_mipi7 = {
	.desc = {
		.modes = forlinx_mipi7_mode,
		.num_modes = 5,
		.bpc = 8,
		.size = {
			.width = 156,
			.height = 90,
		},
		.bus_format = MEDIA_BUS_FMT_RGB888_1X24,
	},
	.flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE | MIPI_DSI_MODE_LPM,
	.format = MIPI_DSI_FMT_RGB888,
	.lanes = 4,
};

static int panel_simple_get_fixed_modes(struct lt8912 *lt8912, struct drm_connector *connector)
{
	struct drm_device *drm = lt8912->bridge.dev;
	struct drm_display_mode *mode;
	unsigned int i, num = 0;


	for (i = 0; i < forlinx_mipi7.desc.num_modes; i++) {
		const struct drm_display_mode *m = &forlinx_mipi7.desc.modes[i];
		if(lt8912->match_mode != (i + 1) && lt8912->match_mode != MATCH_ALL) 
			continue;
		mode = drm_mode_duplicate(drm, m);
		if (!mode) {
			dev_err(drm->dev, "failed to add mode %ux%u@%u\n",
					m->hdisplay, m->vdisplay, m->vrefresh);
			continue;
		}

		mode->type |= DRM_MODE_TYPE_DRIVER;

		if (forlinx_mipi7.desc.num_modes == 1)
			mode->type |= DRM_MODE_TYPE_PREFERRED;
		if(strncmp("lt8912_custom", mode->name, 13)) {
			drm_mode_set_name(mode);
		}
		drm_mode_probed_add(connector, mode);
		num++;
	}

	connector->display_info.bpc = forlinx_mipi7.desc.bpc;
	connector->display_info.width_mm = forlinx_mipi7.desc.size.width;
	connector->display_info.height_mm = forlinx_mipi7.desc.size.height;
	if (forlinx_mipi7.desc.bus_format)
		drm_display_info_set_bus_formats(&connector->display_info,
				&forlinx_mipi7.desc.bus_format, 1);
	connector->display_info.bus_flags = forlinx_mipi7.desc.bus_flags;

	return num;
}

static int lt8912_get_modes(struct lt8912 *lt8912,
		struct drm_connector *connector)
{
	return panel_simple_get_fixed_modes(lt8912, connector);
}

static int lt8912_read_reg(struct i2c_client *client, u8 reg, u8 *val)
{
	struct i2c_msg msg[2];
	u8 buf[1];
	int ret;

	buf[0] = reg;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 1;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0)
		return ret;

	*val = buf[0];
	return 0;
}

static int lt8912_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	struct i2c_msg msg;
	u8 buf[2];
	int ret;

	buf[0] = reg;
	buf[1] = val;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
	//	printk("%s: error: reg=%x, val=%x\n",
	//			__func__, reg, val);
		return ret;
	}
	return 0;
}

static void lt8912_write_array(struct i2c_client *client, struct lt8912_reg_cfg reg_cfg[], int cnt){
	int i = 0;
	for (i = 0; i < cnt; i++){
		lt8912_write_reg(client, reg_cfg[i].reg, reg_cfg[i].val);
		if (reg_cfg[i].sleep_in_ms != 0)
			msleep(reg_cfg[i].sleep_in_ms);
	}
}

#define LT8912_REG_CHIP_REVISION_0 (0x00)
#define LT8912_REG_CHIP_REVISION_1 (0x01)

#define LT8912_VAL_CHIP_REVISION_0 (0x12)
#define LT8912_VAL_CHIP_REVISION_1 (0xB2)

static int lt8912_check_dds(struct lt8912 *pdata)
{
	u8 reg_920c = 0;
	u8 reg_920d = 0;
	u8 reg_920e = 0;
	u8 i;
	//int connected = 0;

	if (!pdata) {
		pr_err("%s: invalid platform data\n", __func__);
		return -EINVAL;
	}
	
	pr_debug("%s: enter\n", __func__);
	/* Check if DDS clock stable
	 * if clock untable, need to reset DDS module. 
	 * if dds clock stable, then enable lvds output.
	 * 
	 */

	for(i = 0; i < 10; i++){
	lt8912_read_reg(pdata->i2c_cec, 0x0c, &reg_920c);	
	lt8912_read_reg(pdata->i2c_cec, 0x0e, &reg_920d);
	lt8912_read_reg(pdata->i2c_cec, 0x0f, &reg_920e);

        pr_debug("%s: DDS:%x,%x,%x\n", __func__,reg_920c,reg_920d,reg_920e);
	msleep(5);
	
	}

	lt8912_write_reg(pdata->i2c_main, 0x02, 0xf7);//lvds pll reset
	lt8912_write_reg(pdata->i2c_main, 0x02, 0xff);
	lt8912_write_reg(pdata->i2c_main, 0x03, 0xcb);//scaler module reset
	lt8912_write_reg(pdata->i2c_main, 0x03, 0xfb);//lvds tx module reset
	lt8912_write_reg(pdata->i2c_main, 0x03, 0xff);

    	lt8912_write_reg(pdata->i2c_main, 0x44, 0x30); //enbale lvds output


	return 0;
}

static void lt8912_mode_set(struct lt8912 *lt8912,
		const struct drm_display_mode *mode,
		const struct drm_display_mode *adj_mode)
{
	u8 val;
	drm_mode_copy(&lt8912->curr_mode, adj_mode);

	lt8912_read_reg(lt8912->i2c_main, LT8912_REG_CHIP_REVISION_0, &val);
	if (val != LT8912_VAL_CHIP_REVISION_0)
	{
		printk("check chip revision not match reg = 0x%x, val = 0x%x\n", LT8912_REG_CHIP_REVISION_0, val);
	}

	lt8912_read_reg(lt8912->i2c_main, LT8912_REG_CHIP_REVISION_1, &val);
	if (val != LT8912_VAL_CHIP_REVISION_1)
	{
		printk("check chip revision not match reg = 0x%x, val = 0x%x\n", LT8912_REG_CHIP_REVISION_1, val);
	}
	lt8912_write_array(lt8912->i2c_main, lt8912_init_setup, ARRAY_SIZE(lt8912_init_setup));
	lt8912_write_array(lt8912->i2c_cec, lt8912_mipi_basic_set, ARRAY_SIZE(lt8912_mipi_basic_set));
	
	lt8912_custom_mode(lt8912->i2c_cec);

	lt8912_write_array(lt8912->i2c_cec, lt8912_ddsconfig, ARRAY_SIZE(lt8912_ddsconfig));
	lt8912_write_array(lt8912->i2c_main, lt8912_rxlogicres, ARRAY_SIZE(lt8912_rxlogicres));
	lt8912_write_array(lt8912->i2c_cec, lt8912_rxlogicres2, ARRAY_SIZE(lt8912_rxlogicres2));
	lt8912_write_array(lt8912->i2c_main, lt8912_lvds_bypass_cfg, ARRAY_SIZE(lt8912_lvds_bypass_cfg));
	lt8912_check_dds(lt8912);
}

/* Connector funcs */
static struct lt8912 *connector_to_lt8912(struct drm_connector *connector)
{
	return container_of(connector, struct lt8912, connector);
}

static int lt8912_connector_get_modes(struct drm_connector *connector)
{
	struct lt8912 *adv = connector_to_lt8912(connector);

	return lt8912_get_modes(adv, connector);
}

	static enum drm_mode_status
lt8912_connector_mode_valid(struct drm_connector *connector,
		struct drm_display_mode *mode)
{
	return MODE_OK;
}

static struct drm_connector_helper_funcs lt8912_connector_helper_funcs = {
	.get_modes = lt8912_connector_get_modes,
	.mode_valid = lt8912_connector_mode_valid,
};

	static enum drm_connector_status
lt8912_connector_detect(struct drm_connector *connector, bool force)
{
	return connector_status_connected;
}

static const struct drm_connector_funcs lt8912_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = lt8912_connector_detect,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

/* Bridge funcs */
static struct lt8912 *bridge_to_lt8912(struct drm_bridge *bridge)
{
	return container_of(bridge, struct lt8912, bridge);
}

static void lt8912_bridge_enable(struct drm_bridge *bridge)
{
}

static void lt8912_bridge_disable(struct drm_bridge *bridge)
{
}

static void lt8912_bridge_mode_set(struct drm_bridge *bridge,
		const struct drm_display_mode *mode,
		const struct drm_display_mode *adj_mode)
{
	struct lt8912 *adv = bridge_to_lt8912(bridge);

	lt8912_mode_set(adv, mode, adj_mode);
}

static bool lt8912_bridge_mode_fixup(struct drm_bridge *bridge,
		const struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode)
{

	return true;
}

static int lt8912_bridge_attach(struct drm_bridge *bridge)
{
	struct lt8912 *adv = bridge_to_lt8912(bridge);
	struct device *dev = &adv->i2c_main->dev;
	struct mipi_dsi_host *host;
	struct mipi_dsi_device *dsi;
	int ret = 0;

	const struct mipi_dsi_device_info info = { 
		.type = "lt8912_dsi",
		.channel = 0,
		.node = NULL,
	};

	if (!bridge->encoder) {
		DRM_ERROR("Parent encoder object not found");
		return -ENODEV;
	}

	adv->connector.polled = 0;

	ret = drm_connector_init(bridge->dev, &adv->connector,
			&lt8912_connector_funcs,
			DRM_MODE_CONNECTOR_HDMIA);
	if (ret) {
		DRM_ERROR("Failed to initialize connector with drm\n");
		return ret;
	}
	drm_connector_helper_add(&adv->connector,
			&lt8912_connector_helper_funcs);
	drm_connector_attach_encoder(&adv->connector, bridge->encoder);

	adv->host_node = of_graph_get_remote_node(dev->of_node, 0, 0);
	if (!adv->host_node)
		return -ENODEV;

	of_node_put(adv->host_node);

	host = of_find_mipi_dsi_host_by_node(adv->host_node);
	if (!host) {
		dev_err(dev, "failed to find dsi host\n");
		return -EPROBE_DEFER;
	}

	dsi = mipi_dsi_device_register_full(host, &info);
	if (IS_ERR(dsi)) {
		dev_err(dev, "failed to create dsi device\n");
		ret = PTR_ERR(dsi);
	}

	adv->dsi = dsi;

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO |
		MIPI_DSI_MODE_EOT_PACKET | MIPI_DSI_MODE_VIDEO_HSE;
	if(sync_mode == 1) {
		dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_SYNC_PULSE;
	} else {
		dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_BURST;
	}

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "failed to attach dsi to host\n");
	}

	return ret;
}

static const struct drm_bridge_funcs lt8912_bridge_funcs = {
	.enable = lt8912_bridge_enable,
	.disable = lt8912_bridge_disable,
	.mode_set = lt8912_bridge_mode_set,
	.mode_fixup = lt8912_bridge_mode_fixup,
	.attach = lt8912_bridge_attach,
};


static const struct i2c_device_id lt8912_i2c_ids[] = {
	{ "lt8912" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lt8912_i2c_ids);

static const struct of_device_id lt8912_of_ids[] = {
	{ .compatible = "lontium,lt8912-CUSTOM" , .data = (void*)CUSTOM_MODE},
	{ }
};
MODULE_DEVICE_TABLE(of, lt8912_of_ids);
static int setup_ok = 0;
static int lt8912_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct lt8912 *lt8912;
	struct device *dev = &i2c->dev;
	unsigned int main_i2c_addr = i2c->addr;
	unsigned int cec_i2c_addr = main_i2c_addr + 1;
	int ret;
	const struct of_device_id *of_id =
                        of_match_device(lt8912_of_ids, &i2c->dev);
	if (!dev->of_node)
		return -EINVAL;

	lt8912 = devm_kzalloc(dev, sizeof(*lt8912), GFP_KERNEL);
	if (!lt8912)
		return -ENOMEM;

	lt8912->i2c_main = i2c;
	lt8912->addr_cec = cec_i2c_addr;

	lt8912->i2c_cec = i2c_new_dummy(i2c->adapter, cec_i2c_addr);
	if (!lt8912->i2c_cec) {
		printk("i2c_new_dummy error\n");
	}

	lt8912->bridge.funcs = &lt8912_bridge_funcs;
	lt8912->bridge.of_node = dev->of_node;
	if(!of_id)	
		lt8912->match_mode = MATCH_ALL;
	else
		lt8912->match_mode = (long)of_id->data;
	if(lt8912->match_mode == CUSTOM_MODE) {
	       if(setup_ok == 0) {	
			struct device_node *timings;
			timings = of_get_child_by_name(dev->of_node, "display-timings");
	        	if (timings) {
        	        	of_node_put(timings);
                		ret = of_get_videomode(dev->of_node, &vm, 0);
				if(ret < 0) {
					printk("custom mode get videomode error %d\n", ret);
					return ret;
				}
	        	} else {
				printk("custom mode no display-timings use default\n");
        	        	videomode_from_timing(&rad_default_timing, &vm);
	        	}
	       }
		forlinx_mipi7_mode[CUSTOM_MODE - 1].clock = vm.pixelclock / 1000;
                forlinx_mipi7_mode[CUSTOM_MODE - 1].hdisplay = vm.hactive;
                forlinx_mipi7_mode[CUSTOM_MODE - 1].hsync_start = vm.hactive + vm.hfront_porch;
                forlinx_mipi7_mode[CUSTOM_MODE - 1] .hsync_end = vm.hactive +vm.hsync_len + vm.hfront_porch;
                forlinx_mipi7_mode[CUSTOM_MODE - 1].htotal = vm.hactive +vm.hfront_porch + vm.hback_porch + vm.hsync_len;
                forlinx_mipi7_mode[CUSTOM_MODE - 1].vdisplay = vm.vactive;
                forlinx_mipi7_mode[CUSTOM_MODE - 1].vsync_start = vm.vactive +vm.vfront_porch;
                forlinx_mipi7_mode[CUSTOM_MODE - 1].vsync_end = vm.vactive +vm.vsync_len + vm.vfront_porch;
                forlinx_mipi7_mode[CUSTOM_MODE - 1].vtotal = vm.vactive +vm.vfront_porch + vm.vback_porch + vm.vsync_len;
                forlinx_mipi7_mode[CUSTOM_MODE - 1].vrefresh = 60;

	}
	drm_bridge_add(&lt8912->bridge);

	return 0;
}

static int lt8912_remove(struct i2c_client *i2c)
{
	struct lt8912 *lt8912 = i2c_get_clientdata(i2c);

	mipi_dsi_detach(lt8912->dsi);
	mipi_dsi_device_unregister(lt8912->dsi);

	drm_bridge_remove(&lt8912->bridge);

	return 0;
}

static void get_mode_frome_env(char *options)
{
	char *opt;
	while ((opt = strsep(&options, ",")) != NULL) {
		if (!*opt)
			continue;
		if (!strncmp(opt, "xres=", 5)) {
			vm.hactive = simple_strtoul(opt + 5, NULL, 0);
		} else if (!strncmp(opt, "yres=", 5)) {
			vm.vactive = simple_strtoul(opt + 5, NULL, 0);
		} else if (!strncmp(opt, "pixclock=", 9)) {
			vm.pixelclock = simple_strtoul(opt + 9, NULL, 0);
		} else if (!strncmp(opt, "left_margin=", 12)) {
			vm.hback_porch = simple_strtoul(opt + 12, NULL, 0);
		} else if (!strncmp(opt, "right_margin=", 13)) {
			vm.hfront_porch = simple_strtoul(opt + 13, NULL, 0);
		} else if (!strncmp(opt, "upper_margin=", 13)) {
			vm.vback_porch = simple_strtoul(opt + 13, NULL, 0);
		} else if (!strncmp(opt, "lower_margin=", 13)) {
			vm.vfront_porch = simple_strtoul(opt + 13, NULL, 0);
		} else if (!strncmp(opt, "hsync_len=", 10)) {
			vm.hsync_len = simple_strtoul(opt + 10, NULL, 0);
		} else if (!strncmp(opt, "vsync_len=", 10)) {
			vm.vsync_len = simple_strtoul(opt + 10, NULL, 0);
		} else if (!strncmp(opt, "sync_mode", 9)) {
			sync_mode = 1;

		} 
	}
}

static int __init custom_video_mode_setup(char *options)
{
	printk("custom_video_mode options = %s\n", options);
	get_mode_frome_env(options);
	setup_ok = 1;
	return 1;
}
__setup("custom_video_mode=", custom_video_mode_setup);

static struct mipi_dsi_driver lt8912_dsi_driver = {
	.driver.name = "lt8912_dsi",
};

static struct i2c_driver lt8912_driver = {
	.driver = {
		.name = "lt8912",
		.of_match_table = lt8912_of_ids,
	},
	.id_table = lt8912_i2c_ids,
	.probe = lt8912_probe,
	.remove = lt8912_remove,
};

static int __init lt8912_init(void)
{
	if (IS_ENABLED(CONFIG_DRM_MIPI_DSI))
		mipi_dsi_driver_register(&lt8912_dsi_driver);

	return i2c_add_driver(&lt8912_driver);
}
module_init(lt8912_init);

static void __exit lt8912_exit(void)
{
	i2c_del_driver(&lt8912_driver);

	if (IS_ENABLED(CONFIG_DRM_MIPI_DSI))
		mipi_dsi_driver_unregister(&lt8912_dsi_driver);
}
module_exit(lt8912_exit);

MODULE_AUTHOR("www.forlinx.com");
MODULE_DESCRIPTION("lt8912 HDMI transmitter driver");
MODULE_LICENSE("GPL");
