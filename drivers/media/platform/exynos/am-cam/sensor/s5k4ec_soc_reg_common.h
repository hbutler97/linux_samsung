/*
 * Samsung Exynos SoC series module-camera driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef __S5K4EC_SOC_REG_COMMON_H
#define __S5K4EC_SOC_REG_COMMON_H

#define ARR_SIZE(a) (sizeof(a) / sizeof((a)[0]))

uint32_t s5k4ec_reg_set_sw_reset[] = {
	0xFCFCD000,
	0x00100001,
	0x10300000,
	0x00140001,
};

uint32_t s5k4ec_reg_set_stream_on[] = {
	0xFCFCD000,
	0x002A0266,
	0x0F120000,
	0x002A026A,
	0x0F120001,
	0x002A0268,
	0x0F120001,
	0x002A026E,
	0x0F120000,
	0x002A026A,
	0x0F120001,
	0x002A0270,
	0x0F120001,

	0x002A024E,
	0x0F120001,
	0x002A023E,
	0x0F120001,
	0x0F120001,

	0x002A01A8,
	0x0F12AAAA,

	0x002A147C,
	0x0F120180,
	0x002A1482,
	0x0F120180,
};

uint32_t s5k4ec_reg_set_stream_off[] = {
	0x00287000,
	0x002A023E,
	0x0F120000,
	0x0F120001,
};

struct s5k4ec_reg s5k4ec_reg_sw_reset = {
	.reg = s5k4ec_reg_set_sw_reset,
	.size = ARR_SIZE(s5k4ec_reg_set_sw_reset)
};

struct s5k4ec_reg s5k4ec_reg_stream_on = {
	.reg = s5k4ec_reg_set_stream_on,
	.size = ARR_SIZE(s5k4ec_reg_set_stream_on)
};

struct s5k4ec_reg s5k4ec_reg_stream_off = {
	.reg = s5k4ec_reg_set_stream_off,
	.size = ARR_SIZE(s5k4ec_reg_set_stream_off)
};

/* ============================================ */
/* ETC						*/
/* ============================================ */
uint32_t s5k4ec_reg_set_contrast_default[] = {
	0x002A0232,
	0x0F120000,
};

struct s5k4ec_reg s5k4ec_reg_contrast_default = {
	.reg = s5k4ec_reg_set_contrast_default,
	.size = ARR_SIZE(s5k4ec_reg_set_contrast_default)
};

uint32_t s5k4ec_reg_set_effect_none[] = {
	0x002A023C,
	0x0F120000,
};

struct s5k4ec_reg s5k4ec_reg_effect_none = {
	.reg = s5k4ec_reg_set_effect_none,
	.size = ARR_SIZE(s5k4ec_reg_set_effect_none)
};

uint32_t s5k4ec_reg_set_brightness_default[] = {
	0x002A0230,
	0x0F12003C,
};

struct s5k4ec_reg s5k4ec_reg_brightness_default = {
	.reg = s5k4ec_reg_set_brightness_default,
	.size = ARR_SIZE(s5k4ec_reg_set_brightness_default)
};

uint32_t s5k4ec_reg_set_iso_auto[] = {
	0x002A0938,
	0x0F120000,

	0x0F120014,
	0x0F1200D2,
	0x0F120384,
	0x0F1207D0,
	0x0F121388,
	0x002A0230,
	0x0F120000,
	0x002A1484,
	0x0F12003C,

	0x002A0F2A,
	0x0F120001,
	0x002A04E6,
	0x0F12077F,

	0x002A04D0,
	0x0F120000,
	0x0F120000,
	0x0F120001,
	0x002A06C2,
	0x0F120200,
};

struct s5k4ec_reg s5k4ec_reg_iso_auto = {
	.reg = s5k4ec_reg_set_iso_auto,
	.size = ARR_SIZE(s5k4ec_reg_set_iso_auto)
};

uint32_t s5k4ec_reg_set_metering_center[] = {
	0x002A1492,
	0x0F120000,
	0x0F120101,
	0x0F120101,
	0x0F120000,
	0x0F120101,
	0x0F120101,
	0x0F120101,
	0x0F120101,
	0x0F120201,
	0x0F120303,
	0x0F120303,
	0x0F120102,
	0x0F120201,
	0x0F120403,
	0x0F120304,
	0x0F120102,
	0x0F120201,
	0x0F120403,
	0x0F120304,
	0x0F120102,
	0x0F120201,
	0x0F120403,
	0x0F120304,
	0x0F120102,
	0x0F120201,
	0x0F120303,
	0x0F120303,
	0x0F120102,
	0x0F120201,
	0x0F120202,
	0x0F120202,
	0x0F120102,
};

struct s5k4ec_reg s5k4ec_reg_metering_center = {
	.reg = s5k4ec_reg_set_metering_center,
	.size = ARR_SIZE(s5k4ec_reg_set_metering_center)
};

uint32_t s5k4ec_reg_set_saturation_default[] = {
	0x002A0234,
	0x0F120000,
};

struct s5k4ec_reg s5k4ec_reg_saturation_default = {
	.reg = s5k4ec_reg_set_saturation_default,
	.size = ARR_SIZE(s5k4ec_reg_set_saturation_default)
};

uint32_t s5k4ec_reg_set_scene_none[] = {
	0x002A04E6,
	0x0F12077F,

	0x002A0234,
	0x0F120000,

	0x002A0232,
	0x0F120000,

	0x002A0230,
	0x0F12003C,

	0x002A0A28,
	0x0F126024,
	0x002A0ADE,
	0x0F126024,
	0x002A0B94,
	0x0F126024,
	0x002A0C4A,
	0x0F126024,
	0x002A0D00,
	0x0F126024,

	0x002A0938,
	0x0F120000,
};

struct s5k4ec_reg s5k4ec_reg_scene_none = {
	.reg = s5k4ec_reg_set_scene_none,
	.size = ARR_SIZE(s5k4ec_reg_set_scene_none)
};

uint32_t s5k4ec_reg_set_sharpness_default[] = {
	0x002A0A28,
	0x0F126024,
	0x002A0ADE,
	0x0F126024,
	0x002A0B94,
	0x0F126024,
	0x002A0C4A,
	0x0F126024,
	0x002A0D00,
	0x0F126024,
};

struct s5k4ec_reg s5k4ec_reg_sharpness_default = {
	.reg = s5k4ec_reg_set_sharpness_default,
	.size = ARR_SIZE(s5k4ec_reg_set_sharpness_default)
};

uint32_t s5k4ec_reg_set_ae_unlock[] = {
	/* AE Unlock */
	0x002A2C5E,
	0x0F120001,
};

struct s5k4ec_reg s5k4ec_reg_ae_unlock = {
	.reg = s5k4ec_reg_set_ae_unlock,
	.size = ARR_SIZE(s5k4ec_reg_set_ae_unlock)
};

uint32_t s5k4ec_reg_set_awb_unlock[] = {
	/* AWB Unlock */
	0x002A2C66,
	0x0F120001,
};

struct s5k4ec_reg s5k4ec_reg_awb_unlock = {
	.reg = s5k4ec_reg_set_awb_unlock,
	.size = ARR_SIZE(s5k4ec_reg_set_awb_unlock)
};

uint32_t s5k4ec_reg_set_awb_mode_auto[] = {
	0x002A04E6,
	0x0F12077F,
};

struct s5k4ec_reg s5k4ec_reg_awb_mode_auto = {
	.reg = s5k4ec_reg_set_awb_mode_auto,
	.size = ARR_SIZE(s5k4ec_reg_set_awb_mode_auto)
};

uint32_t s5k4ec_reg_set_fps_auto[] = {
	0xFCFCD000,
	0x00287000,
	0x002A02BE,
	0x0F120000,
	0x0F120001,
	0x0F12029A,
	0x0F12014A,
	0x002A02D0,
	0x0F120000,
	0x0F120000,
	0x002A0266,
	0x0F120000,
	0x002A026A,
	0x0F120001,
	0x002A024E,
	0x0F120001,
};

struct s5k4ec_reg s5k4ec_reg_fps_auto = {
	.reg = s5k4ec_reg_set_fps_auto,
	.size = ARR_SIZE(s5k4ec_reg_set_fps_auto)
};

#endif
