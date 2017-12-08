/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include "panel-drv.h"

/* Private Functions */
int __panel_helper_dcs_write(struct exynos_panel *ctx, const u8 cmd,
		const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	DRM_DEBUG_KMS("%s +\n", __func__);

	ret = mipi_dsi_dcs_write(dsi, cmd, data, len);
	if (ret < 0)
		dev_err(ctx->dev, "error %zd writing dcs seq: %*ph\n", ret,
							(int)len, data);
	DRM_DEBUG_KMS("%s -\n", __func__);
	return ret;
}

static int panel_get_brightness(struct backlight_device *bl_dev)
{
	DRM_DEBUG_KMS("%s \n", __func__);
	return bl_dev->props.brightness;
}
static int panel_set_brightness(struct backlight_device *bl_dev)
{
	DRM_DEBUG_KMS("%s \n", __func__);
	return 0;
}

static const struct backlight_ops panel_bl_ops = {
	.get_brightness = panel_get_brightness,
	.update_status = panel_set_brightness,
};

/* Write a display_off command and wait */
static int panel_disable(struct drm_panel *panel)
{
	struct exynos_panel *ctx = panel_to_context(panel);

	DRM_DEBUG_KMS("%s +\n", __func__);

	if (ctx->funcs->disp_off)
		ctx->funcs->disp_off(ctx);
	ctx->bl_dev->props.power = FB_BLANK_NORMAL;

	msleep(ctx->delay.disable);

	DRM_DEBUG_KMS("%s -\n", __func__);
	return 0;
}

/* Wait and Power off */
static int panel_unprepare(struct drm_panel *panel)
{
	struct exynos_panel *ctx = panel_to_context(panel);

	DRM_DEBUG_KMS("%s +\n", __func__);

	ctx->bl_dev->props.power = FB_BLANK_POWERDOWN;
	msleep(ctx->delay.unprepare);

	if (ctx->funcs->power_off)
		ctx->funcs->power_off(ctx);

	if(!IS_ERR_OR_NULL(ctx->gpio.power))
		gpiod_set_value(ctx->gpio.power, 0);

	DRM_DEBUG_KMS("%s -\n", __func__);
	return 0;
}

/* Reset and Wait */
static int panel_ready(struct drm_panel *panel)
{
	struct exynos_panel *ctx = panel_to_context(panel);
	struct device *dev = ctx->dev;
	int ret, cnt = 100;

	DRM_DEBUG_KMS("%s +\n", __func__);
	gpiod_set_value(ctx->gpio.reset, 0);
	msleep(ctx->delay.reset);
	/*gpiod_set_value(ctx->gpio.reset, 1);*/

	if(!IS_ERR_OR_NULL(ctx->gpio.ready)) {
		do {
			ret = gpiod_get_value(ctx->gpio.ready);
			msleep(1);
			cnt--;
		} while(!ret && cnt);

		if (cnt <= 0)
			dev_err(dev, "%s failed to ready\n", __func__);
	}

	DRM_DEBUG_KMS("%s -\n", __func__);
	return 0;
}

/* Power on and Wait */
static int panel_prepare(struct drm_panel *panel)
{
	struct exynos_panel *ctx = panel_to_context(panel);

	DRM_DEBUG_KMS("%s +\n", __func__);

	if(!IS_ERR_OR_NULL(ctx->gpio.power))
		gpiod_set_value(ctx->gpio.power, 1);

	if (ctx->funcs->power_on)
		ctx->funcs->power_on(ctx);

	msleep(ctx->delay.prepare);
	ctx->bl_dev->props.power = FB_BLANK_NORMAL;

	DRM_DEBUG_KMS("%s -\n", __func__);
	return 0;
}

/* Write and init and Wait and Write a display_on */
static int panel_enable(struct drm_panel *panel)
{
	struct exynos_panel *ctx = panel_to_context(panel);
	int ret = 0;

	DRM_DEBUG_KMS("%s +\n", __func__);

	if (ctx->funcs->init)
		ret = ctx->funcs->init(ctx);

	msleep(ctx->delay.enable);
	if (ctx->funcs->disp_on)
		ctx->funcs->disp_on(ctx);

	ctx->bl_dev->props.power = FB_BLANK_UNBLANK;

	DRM_DEBUG_KMS("%s -\n", __func__);
	return ret;
}

static int panel_get_modes(struct drm_panel *panel)
{
	struct drm_connector *connector = panel->connector;
	struct exynos_panel *ctx = panel_to_context(panel);
	struct drm_display_mode *mode;

	DRM_DEBUG_KMS("%s +\n", __func__);

	mode = drm_mode_create(connector->dev);
	if (!mode) {
		DRM_ERROR("failed to create a new display mode\n");
		return 0;
	}

	drm_display_mode_from_videomode(&ctx->vm, mode);
	mode->vrefresh = 60;
	mode->width_mm = ctx->width_mm;
	mode->height_mm = ctx->height_mm;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	DRM_DEBUG_KMS("%s -\n", __func__);
	return 1;
}

static const struct drm_panel_funcs panel_drm_funcs = {
	.disable = panel_disable,
	.unprepare = panel_unprepare,
	.prepare = panel_prepare,
	.ready = panel_ready,
	.enable = panel_enable,
	.get_modes = panel_get_modes,
};

static int panel_backlight_create(struct exynos_panel *panel)
{
	struct backlight_device *bl;
	static int idx = 0;
	char name[32] = {'\n'};

	sprintf(name, "%s_%d", panel->dev->driver->name, idx++);

	bl = backlight_device_register(name, panel->dev,
			panel, &panel_bl_ops, NULL);
	if (IS_ERR(bl)) {
		dev_err(panel->dev, "failed to register backlight device\n");
		return PTR_ERR(bl);
	}

	bl->props.max_brightness = MAX_BRIGHTNESS;
	bl->props.brightness = DEFAULT_BRIGHTNESS;
	bl->props.power = FB_BLANK_POWERDOWN;

	panel->bl_dev = bl;

	return 0;
}

int panel_parse_dt(struct exynos_panel *ctx)
{
	struct device *dev = ctx->dev;
	struct device_node *np = dev->of_node;
	int ret;

	DRM_DEBUG_KMS("%s +\n", __func__);

	ctx->gpio.reset = devm_gpiod_get(dev, "reset");
	if (IS_ERR(ctx->gpio.reset)) {
		dev_err(dev, "cannot get reset %ld\n",
			PTR_ERR(ctx->gpio.reset));
	} else {
		ret = gpiod_direction_output(ctx->gpio.reset, 1);
		if (ret < 0) {
			dev_err(dev, "cannot configure reset %d\n", ret);
			return ret;
		}
	}

	ctx->gpio.power = devm_gpiod_get(dev, "power");
	if (IS_ERR(ctx->gpio.power)) {
		dev_err(dev, "cannot get power %ld\n",
			PTR_ERR(ctx->gpio.power));
	} else {
		ret = gpiod_direction_output(ctx->gpio.power, 0);
		if (ret < 0) {
			dev_err(dev, "cannot configure power %d\n", ret);
			return ret;
		}
	}

	ctx->gpio.ready = devm_gpiod_get(dev, "ready");
	if (IS_ERR(ctx->gpio.ready)) {
		dev_err(dev, "cannot get ready %ld\n",
			PTR_ERR(ctx->gpio.ready));
	} else {

		ret = gpiod_direction_input(ctx->gpio.ready);
		if (ret < 0) {
			dev_err(dev, "cannot configure ready %d\n", ret);
			return ret;
		}
	}

	of_property_read_u32(np, "reset-delay", &ctx->delay.reset);
	of_property_read_u32(np, "lane-on-delay", &ctx->delay.prepare);
	of_property_read_u32(np, "lane-off-delay", &ctx->delay.unprepare);
	of_property_read_u32(np, "disp-on-delay", &ctx->delay.enable);
	of_property_read_u32(np, "disp-off-delay", &ctx->delay.disable);

	ret = of_get_videomode(np, &ctx->vm, 0);
	if (ret < 0)
		return ret;

	of_property_read_u32(np, "panel-width-mm", &ctx->width_mm);
	of_property_read_u32(np, "panel-height-mm", &ctx->height_mm);
	of_property_read_u32(np, "panel-up-scale", &ctx->attr.up_scale);
	of_property_read_u32(np, "panel-mic-bypass", &ctx->attr.mic_bypass);

	DRM_INFO("%s: hactive(%d), vactive(%d)\n", __func__,
			ctx->vm.hactive, ctx->vm.vactive);

	DRM_DEBUG_KMS("%s -\n", __func__);
	return 0;
}

static void set_maximum_return_packet_size(struct exynos_panel *ctx, int size)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	const struct mipi_dsi_host_ops *ops = dsi->host->ops;
	u8 buf[] = {size, 0};
	struct mipi_dsi_msg msg = {
		.channel = dsi->channel,
		.type = MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE,
		.tx_len = sizeof(buf),
		.tx_buf = buf
	};
	int ret;

	if (!ops || !ops->transfer)
		ret = -EIO;
	else
		ret = ops->transfer(dsi->host, &msg);

	if (ret < 0) {
		dev_err(ctx->dev,
			"error %d setting maximum return packet size to %d\n",
			ret, size);
	}
}

static int panel_try_read(struct exynos_panel *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	int ret;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0)
		dev_err(ctx->dev, "error %d reading dcs seq(%#x)\n", ret, cmd);

	return ret;
}

/***************************/
/* Public Helper Functions */
/***************************/
bool panel_helper_is_connected(struct exynos_panel *ctx)
{
	u8 id[3];
	int ret;

	memset(id, 0, sizeof(id));
	set_maximum_return_packet_size(ctx, ARRAY_SIZE(id));
	ret = panel_try_read(ctx, MIPI_DCS_GET_DISPLAY_ID, id, ARRAY_SIZE(id));
	if (ret < 0 || id[0] == 0) {
		dev_err(ctx->dev, "read id failed, ret %d\n", ret);
		return false;
	}

	dev_info(ctx->dev, "ID: 0x%2x, 0x%2x, 0x%2x\n", id[0], id[1], id[2]);
	return true;
}

int panel_helper_probe(struct exynos_panel *ctx, struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	int ret;

	DRM_DEBUG_KMS("%s +\n", __func__);

	mipi_dsi_set_drvdata(dsi, ctx);
	ctx->dev = dev;

	ret = panel_parse_dt(ctx);
	if (ret < 0)
		return ret;

	ret = panel_backlight_create(ctx);
	if (ret < 0)
		return ret;

	drm_panel_init(&ctx->panel);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &panel_drm_funcs;

	ret = drm_panel_add(&ctx->panel);
	if (ret < 0)
		goto remove_bl;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		goto remove_panel;

	DRM_DEBUG_KMS("%s -\n", __func__);
	return ret;

remove_panel:
	drm_panel_remove(&ctx->panel);
remove_bl:
	backlight_device_unregister(ctx->bl_dev);
	return ret;
}

int panel_helper_remove(struct mipi_dsi_device *dsi)
{
	struct exynos_panel *ctx = mipi_dsi_get_drvdata(dsi);

	DRM_DEBUG_KMS("%s +\n", __func__);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
	backlight_device_unregister(ctx->bl_dev);

	kfree(ctx);

	DRM_DEBUG_KMS("%s -\n", __func__);
	return 0;
}
