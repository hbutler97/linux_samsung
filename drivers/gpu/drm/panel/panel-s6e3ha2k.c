/*
 * MIPI-DSI based s6e3ha2k AMOLED 5.7 inch panel driver.
 *
 * Copyright (c) 2014 Samsung Electronics Co., Ltd
 *
 * Donghwa Lee <dh09.lee@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include "panel-drv.h"

static void s6e3ha2k_test_key_on_f0(struct exynos_panel *ctx)
{
	panel_helper_dcs_write(ctx, 0xf0, 0x5a, 0x5a);
}

static void s6e3ha2k_reg_f2(struct exynos_panel *ctx)
{
	panel_helper_dcs_write(ctx, 0xF2,
	0x67, 0x41, 0xC3, 0x06, 0x0A);
}

static void s6e3ha2k_reg_f9(struct exynos_panel *ctx)
{
	/* Bypass mic encoder logic in DDI */
	panel_helper_dcs_write(ctx, 0xF9,
	0x29);
}

static void s6e3ha2k_sleep_out(struct exynos_panel *ctx)
{
	panel_helper_dcs_write(ctx, 0x11);
}

static void s6e3ha2k_te_start_setting(struct exynos_panel *ctx)
{
	panel_helper_dcs_write(ctx, 0xB9,
	0x10, 0x09, 0xFF, 0x00, 0x09);
}

static void s6e3ha2k_te_on(struct exynos_panel *ctx)
{
	panel_helper_dcs_write(ctx, 0x35);
}

static void s6e3ha2k_touch_hsync(struct exynos_panel *ctx)
{
	panel_helper_dcs_write(ctx, 0xBD,
	0x30, 0x22, 0x02, 0x16, 0x02, 0x16);
}

static void s6e3ha2k_pentile_control(struct exynos_panel *ctx)
{
	panel_helper_dcs_write(ctx, 0xC0,
	0x30, 0x00, 0xD8, 0xD8);
}

static void s6e3ha2k_column_address(struct exynos_panel *ctx)
{
	panel_helper_dcs_write(ctx, 0x2A,
	0x00, 0x00, 0x05, 0x9F);
}

static void s6e3ha2k_gamma_condition_set(struct exynos_panel *ctx)
{
	panel_helper_dcs_write(ctx, 0xCA,
	0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x80, 0x80, 0x80, 0x80,
	0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
	0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
	0x80, 0x80, 0x80, 0x00, 0x00);
}

static void s6e3ha2k_aid_set(struct exynos_panel *ctx)
{
	panel_helper_dcs_write(ctx, 0xB2,
	0x03, 0x10);
}

static void s6e3ha2k_elvss_set(struct exynos_panel *ctx)
{
	panel_helper_dcs_write(ctx, 0xB6,
	0x9C, 0x0A);
}

static void s6e3ha2k_gamma_update(struct exynos_panel *ctx)
{
	panel_helper_dcs_write(ctx, 0xF7,
	0x03);
}

static void s6e3ha2k_acl_off(struct exynos_panel *ctx)
{
	panel_helper_dcs_write(ctx, 0x55,
	0x00);
}

static void s6e3ha2k_acl_opr(struct exynos_panel *ctx)
{
	panel_helper_dcs_write(ctx, 0xB5,
	0x40);
}

static void s6e3ha2k_hbm_off(struct exynos_panel *ctx)
{
	panel_helper_dcs_write(ctx, 0xB4,
	0x04);
}

static void s6e3ha2k_test_global(struct exynos_panel *ctx)
{
	panel_helper_dcs_write(ctx, 0xB0,
	0x07);
}

static void s6e3ha2k_test(struct exynos_panel *ctx)
{
	panel_helper_dcs_write(ctx, 0xB8,
	0x19);
}

static void s6e3ha2k_test_key_off_f0(struct exynos_panel *ctx)
{
	panel_helper_dcs_write(ctx, 0xF0,
	0xA5, 0xA5);
}

static void s6e3ha2k_enable_up_scaling(struct exynos_panel *ctx)
{
	panel_helper_dcs_write(ctx, 0xBA, 0x00); /* Up Scaling in DDI */
}

static void s6e3ha2k_set_src_size(struct exynos_panel *ctx)
{
	panel_helper_dcs_write(ctx, 0x2A, /* 1080(0x437), Column (caset) */
	0x00, 0x00, 0x04, 0x37);
	panel_helper_dcs_write(ctx, 0x2B, /* 1920(0x77F), row (paset) */
	0x00, 0x00, 0x07, 0x7F);
}

static void s6e3ha2k_enable_scl(struct exynos_panel *ctx)
{
	s6e3ha2k_test_key_on_f0(ctx);
	s6e3ha2k_enable_up_scaling(ctx);
	s6e3ha2k_set_src_size(ctx);
	s6e3ha2k_test_key_off_f0(ctx);
}

static int s6e3ha2k_init(struct exynos_panel *ctx)
{
	DRM_DEBUG_KMS("%s +\n", __func__);

	if (!panel_helper_is_connected(ctx))
		return -EBUSY;
#if 1
	/* This is temporary code for test */
	panel_helper_dcs_write(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	panel_helper_dcs_write(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
#endif
	s6e3ha2k_test_key_on_f0(ctx);

	s6e3ha2k_reg_f2(ctx);

	/* If mic is enabled, Below command should be called */
	if (!ctx->attr.mic_bypass)
		s6e3ha2k_reg_f9(ctx);

	s6e3ha2k_sleep_out(ctx);
	msleep(10);

	s6e3ha2k_test_key_on_f0(ctx);

	s6e3ha2k_te_start_setting(ctx);

	s6e3ha2k_reg_f2(ctx);

	s6e3ha2k_te_on(ctx);

	s6e3ha2k_touch_hsync(ctx);

	s6e3ha2k_pentile_control(ctx);

	s6e3ha2k_column_address(ctx);

	s6e3ha2k_gamma_condition_set(ctx);

	s6e3ha2k_aid_set(ctx);

	s6e3ha2k_elvss_set(ctx);

	s6e3ha2k_gamma_update(ctx);

	s6e3ha2k_acl_off(ctx);

	s6e3ha2k_acl_opr(ctx);

	s6e3ha2k_hbm_off(ctx);

	s6e3ha2k_test_global(ctx);

	s6e3ha2k_test(ctx);

	s6e3ha2k_test_key_off_f0(ctx);

	if (ctx->attr.up_scale)
		s6e3ha2k_enable_scl(ctx);

	DRM_DEBUG_KMS("%s -\n", __func__);
	return 0;
}

static int s6e3ha2k_disp_off(struct exynos_panel *ctx)
{
	DRM_DEBUG_KMS("%s +\n", __func__);

	panel_helper_dcs_write(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	panel_helper_dcs_write(ctx, MIPI_DCS_SET_DISPLAY_OFF);

	DRM_DEBUG_KMS("%s -\n", __func__);
	return 0;
}

static int s6e3ha2k_power_off(struct exynos_panel *ctx)
{
	return 0;
}

static int s6e3ha2k_power_on(struct exynos_panel *ctx)
{
	return 0;
}

static int s6e3ha2k_disp_on(struct exynos_panel *ctx)
{
	panel_helper_dcs_write(ctx, MIPI_DCS_SET_DISPLAY_ON);
	return 0;
}

static const struct exynos_panel_funcs s6e3ha2k_panel_funcs = {
	.power_on = s6e3ha2k_power_on,
	.power_off = s6e3ha2k_power_off,
	.init = s6e3ha2k_init,
	.disp_on = s6e3ha2k_disp_on,
	.disp_off = s6e3ha2k_disp_off,
};

static int s6e3ha2k_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct exynos_panel *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(struct exynos_panel), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->funcs = &s6e3ha2k_panel_funcs;

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ret = panel_helper_probe(ctx, dsi);
	if (ret < 0)
		goto error;

	return ret;
error:
	kfree(ctx);
	return ret;
}

static int s6e3ha2k_remove(struct mipi_dsi_device *dsi)
{
	panel_helper_remove(dsi);

	return 0;
}

static struct of_device_id s6e3ha2k_of_match[] = {
	{ .compatible = "samsung,s6e3ha2k" },
	{},
};
MODULE_DEVICE_TABLE(of, s6e3ha2k_of_match);

static struct mipi_dsi_driver s6e3ha2k_driver = {
	.probe = s6e3ha2k_probe,
	.remove = s6e3ha2k_remove,
	.driver = {
		.name = "panel_s6e3ha2k",
		.owner = THIS_MODULE,
		.of_match_table = s6e3ha2k_of_match,
	},
};
module_mipi_dsi_driver(s6e3ha2k_driver);

MODULE_AUTHOR("Donghwa Lee <dh09.lee@samsung.com>");
MODULE_AUTHOR("Hyungwon Hwang <human.hwang@samsung.com>");
MODULE_DESCRIPTION("MIPI-DSI based s6e3ha2k AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");
