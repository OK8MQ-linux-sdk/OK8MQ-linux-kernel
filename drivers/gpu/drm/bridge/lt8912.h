/*
 * Analog Devices ADV7511 HDMI transmitter driver
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef __DRM_LT8912_H__
#define __DRM_LT8912_H__

#include <linux/hdmi.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_crtc_helper.h>
#include <drm/drm_mipi_dsi.h>

struct lt8912 {
	struct i2c_client *i2c_main;
	struct i2c_client *i2c_cec;

	u32 addr_cec;
	u32 addr_main;

	struct drm_display_mode curr_mode;

	struct drm_bridge bridge;
	struct drm_connector connector;

	struct device_node *host_node;
	struct mipi_dsi_device *dsi;
	u8 num_dsi_lanes;
	u8 channel_id;
	bool use_timing_gen;
	int match_mode;
};

#endif
