/*
 * Copyright (C) 2011 Samsung Electronics Co.Ltd
 * Authors:
 * Seung-Woo Kim <sw0312.kim@samsung.com>
 *	Inki Dae <inki.dae@samsung.com>
 *	Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * Based on drivers/media/video/s5p-tv/hdmi_drv.c
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include "exynos_drm_hdmi.h"

#include <linux/kernel.h>
#include <linux/wait.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/io.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/hdmi.h>
#include <linux/component.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#include "regs-hdmi.h"

#include <drm/exynos_drm.h>

#include "exynos_drm_drv.h"
#include "exynos_drm_crtc.h"

//#define HDMI_FORCE_PATTERN

int hdmi_dbg=3;
module_param(hdmi_dbg, int, S_IRUGO | S_IWUSR);

static inline struct hdmi_device *encoder_to_hdmi(struct drm_encoder *e)
{
	return container_of(e, struct hdmi_device, encoder);
}

static inline struct hdmi_device *connector_to_hdmi(struct drm_connector *c)
{
	return container_of(c, struct hdmi_device, connector);
}

void s5p_v4l2_int_src_hdmi_hpd(struct hdmi_device *hdev)
{
	int ret = 0;

	ret = pinctrl_select_state(hdev->pinctrl, hdev->pin_int);
	if (ret)
		hdmi_err("failed to set hdmi hpd interrupt");
}

void s5p_v4l2_int_src_ext_hpd(struct hdmi_device *hdev)
{
	int ret = 0;

	ret = pinctrl_select_state(hdev->pinctrl, hdev->pin_ext);
	if (ret)
		hdmi_err("failed to set external hpd interrupt");
}

bool hdmi_match_timings(const struct v4l2_dv_timings *t1,
			  const struct v4l2_dv_timings *t2,
			  unsigned pclock_delta)
{
	if (t1->type != t2->type)
		return false;
	if (t1->bt.width == t2->bt.width &&
	    t1->bt.height == t2->bt.height &&
	    t1->bt.interlaced == t2->bt.interlaced &&
	    t1->bt.polarities == t2->bt.polarities &&
	    t1->bt.pixelclock >= t2->bt.pixelclock - pclock_delta &&
	    t1->bt.pixelclock <= t2->bt.pixelclock + pclock_delta &&
	    t1->bt.hfrontporch == t2->bt.hfrontporch &&
	    t1->bt.vfrontporch == t2->bt.vfrontporch &&
	    t1->bt.vsync == t2->bt.vsync &&
	    t1->bt.vbackporch == t2->bt.vbackporch &&
	    (!t1->bt.interlaced ||
		(t1->bt.il_vfrontporch == t2->bt.il_vfrontporch &&
		 t1->bt.il_vsync == t2->bt.il_vsync &&
		 t1->bt.il_vbackporch == t2->bt.il_vbackporch)))
		return true;
	return false;
}

static int hdmi_find_phy_conf(struct hdmi_device *hdmi_dev, u32 pixel_clock)
{
	int i;
	for (i = 0; i < hdmi_pre_cnt; ++i)
		if(hdmi_config[i].dv_timings.bt.pixelclock == pixel_clock)
			return i;

	DRM_DEBUG_KMS("Could not find phy config for %d\n", pixel_clock);
	return -EINVAL;
}

static const struct hdmi_config *hdmi_timing2conf(struct v4l2_dv_timings *timings)
{
	int i;

	for (i = 0; i < hdmi_pre_cnt; ++i)
		if (hdmi_match_timings(&hdmi_config[i].dv_timings,
					timings, 0))
			break;

	if (i == hdmi_pre_cnt)
		return NULL;

	return &hdmi_config[i];
}

static struct i2c_client *hdmi_ddc;
void hdmi_attach_ddc_client(struct i2c_client *ddc)
{
	if (ddc){
		hdmi_ddc = ddc;
	}
}

static enum drm_connector_status exynos_hdmi_detect(struct drm_connector *connector,
				bool force)
{
	int state = 0;
	struct hdmi_device *hdmi_dev = connector_to_hdmi(connector);

	state = gpio_get_value(hdmi_dev->res.gpio_hpd);

	DRM_INFO("%s: %s\n", __func__, (state)?"connected":"disconnected");

	if(state)
		return connector_status_connected;
	else
		return connector_status_disconnected;
}

static void exynos_hdmi_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static struct drm_connector_funcs exynos_hdmi_connector_funcs = {
	.dpms = drm_atomic_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = exynos_hdmi_detect,
	.destroy = exynos_hdmi_connector_destroy,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int exynos_hdmi_get_modes(struct drm_connector *connector)
{
	struct hdmi_device *hdmi_dev = connector_to_hdmi(connector);
	struct edid *edid;
	int ret=0;

	DRM_DEBUG_KMS(" +\n");

	if (!hdmi_dev->ddc_port)
		return -ENODEV;

	edid = drm_get_edid(connector, hdmi_dev->ddc_port->adapter);
	if (!edid)
		return -ENODEV;

	hdmi_dev->dvi_mode = !drm_detect_hdmi_monitor(edid);
	DRM_DEBUG_DRIVER("%s : width[%d] x height[%d]\n",
		(hdmi_dev->dvi_mode ? "dvi monitor" : "hdmi monitor"),
		edid->width_cm, edid->height_cm);

	drm_mode_connector_update_edid_property(connector, edid);

	ret = drm_add_edid_modes(connector, edid);

	return ret;
}

static int exynos_hdmi_mode_valid(struct drm_connector *connector,
			struct drm_display_mode *mode)
{
	struct hdmi_device *hdmi_dev = connector_to_hdmi(connector);
	int ret;

	DRM_DEBUG_DRIVER("xres=%d, yres=%d, refresh=%d, intl=%d clock=%d\n",
		mode->hdisplay, mode->vdisplay, mode->vrefresh,
		(mode->flags & DRM_MODE_FLAG_INTERLACE) ? true :
		false, mode->clock * 1000);

	ret = hdmi_find_phy_conf(hdmi_dev, mode->clock * 1000);
	if (ret < 0)
		return MODE_BAD;

	return MODE_OK;
}

static struct drm_encoder *exynos_hdmi_best_encoder(struct drm_connector *connector)
{
	struct hdmi_device *hdmi_dev = connector_to_hdmi(connector);

	return &hdmi_dev->encoder;
}

static struct drm_connector_helper_funcs exynos_hdmi_connector_helper_funcs = {
	.get_modes = exynos_hdmi_get_modes,
	.mode_valid = exynos_hdmi_mode_valid,
	.best_encoder = exynos_hdmi_best_encoder,
};

static int exynos_hdmi_create_connector(struct drm_encoder *encoder)
{
	struct hdmi_device *hdmi_dev = encoder_to_hdmi(encoder);
	struct drm_connector *connector = &hdmi_dev->connector;
	int ret;

	DRM_DEBUG_KMS("+\n");

	connector->interlace_allowed = true;
	connector->polled = DRM_CONNECTOR_POLL_HPD;

	ret = drm_connector_init(hdmi_dev->drm_dev, connector,
			&exynos_hdmi_connector_funcs, DRM_MODE_CONNECTOR_HDMIA);
	if (ret) {
		DRM_ERROR("Failed to initialize connector with drm\n");
		return ret;
	}

	drm_connector_helper_add(connector, &exynos_hdmi_connector_helper_funcs);
	drm_connector_register(connector);
	drm_mode_connector_attach_encoder(connector, encoder);

	DRM_DEBUG_KMS("-\n");

	return 0;
}

static bool exynos_hdmi_mode_fixup(struct drm_encoder *encoder,
			    const struct drm_display_mode *mode,
			    struct drm_display_mode *adjusted_mode)
{
	struct hdmi_device *hdmi_dev = encoder_to_hdmi(encoder);
	struct drm_connector *connector = &hdmi_dev->connector;
	struct drm_display_mode *m;
	int mode_ok;

	DRM_DEBUG_KMS("+\n");

	drm_mode_set_crtcinfo(adjusted_mode, 0);

	mode_ok = exynos_hdmi_mode_valid(connector, adjusted_mode);

	/* just return if user desired mode exists. */
	if (mode_ok == MODE_OK){
		return true;
	}

	/*
	 * otherwise, find the most suitable mode among modes and change it
	 * to adjusted_mode.
	 */
	list_for_each_entry(m, &connector->modes, head) {
		mode_ok = exynos_hdmi_mode_valid(connector, m);

		if (mode_ok == MODE_OK) {
			DRM_INFO("desired mode doesn't exist so\n");
			DRM_INFO("use the most suitable mode among modes.\n");

			DRM_INFO("Adjusted Mode: [%d]x[%d] [%d]Hz\n",
				m->hdisplay, m->vdisplay, m->vrefresh);

			drm_mode_copy(adjusted_mode, m);
			break;
		}
	}

	DRM_DEBUG_KMS("-\n");

	return true;
}

static void exynos_hdmi_mode_set(struct drm_encoder *encoder,
			  struct drm_display_mode *mode,
			  struct drm_display_mode *adjusted_mode)
{
	struct hdmi_device *hdmi_dev = encoder_to_hdmi(encoder);
	struct drm_display_mode *m = adjusted_mode;

	DRM_DEBUG_KMS("xres=%d, yres=%d, refresh=%d, intl=%s\n",
		m->hdisplay, m->vdisplay,
		m->vrefresh, (m->flags & DRM_MODE_FLAG_INTERLACE) ?
		"INTERLACED" : "PROGRESSIVE");

	drm_mode_copy(&hdmi_dev->current_mode, m);
	hdmi_dev->cea_video_id = drm_match_cea_mode(mode);
}

static int exynos_hdmi_runtime_resume(struct device *dev)
{
	struct hdmi_device *hdev = dev_get_drvdata(dev);
	struct hdmi_resources *res = &hdev->res;
	int ret = 0;

	hdmi_dbg(3, "start\n");

	clk_prepare_enable(res->hdmi);
	clk_prepare_enable(res->pixel);
	clk_prepare_enable(res->tmds);

	hdmi_phy_sw_reset(hdev);
	hdmiphy_set_isolation(hdev, 1);
	hdmiphy_set_power(hdev, 1);

	ret = hdmi_conf_apply(hdev);
	if (ret)
		goto fail;

	hdmi_dbg(3, "end\n");

	return 0;
fail:
	clk_disable_unprepare(res->hdmi);
	clk_disable_unprepare(res->pixel);
	clk_disable_unprepare(res->tmds);
	hdmi_err("poweron failed\n");

	return ret;
}

static int exynos_hdmi_runtime_suspend(struct device *dev)
{
	struct hdmi_device *hdev = dev_get_drvdata(dev);
	struct hdmi_resources *res = &hdev->res;

	hdmi_dbg(3, "start\n");

	hdmiphy_set_isolation(hdev, 0);
	hdmiphy_set_power(hdev, 0);
	hdmi_phy_sw_reset(hdev);

	/* turn clocks off */
	clk_disable_unprepare(res->pixel);
	clk_disable_unprepare(res->tmds);
	clk_disable_unprepare(res->hdmi);

	hdmi_dbg(3, "end\n");

	return 0;
}

static const struct dev_pm_ops exynos_hdmi_pm_ops = {
	SET_RUNTIME_PM_OPS(exynos_hdmi_runtime_suspend, exynos_hdmi_runtime_resume,
			   NULL)
};

static int hdmi_s_power(struct hdmi_device *hdev, int on)
{
	int ret = 0;

	/* If runtime PM is not implemented, hdmi_runtime_resume
	 * and hdmi_runtime_suspend functions are directly called.
	 */
	if (on) {
#ifdef CONFIG_PM_RUNTIME
		ret = pm_runtime_get_sync(hdev->dev);
#else
		exynos_hdmi_runtime_resume(hdev->dev);
#endif
	} else {
#ifdef CONFIG_PM_RUNTIME
		ret = pm_runtime_put_sync(hdev->dev);
#else
		exynos_hdmi_runtime_suspend(hdev->dev);
#endif
	}

	/* only values < 0 indicate errors */
	return IS_ERR_VALUE(ret) ? ret : 0;
}

static int hdmi_streamon(struct hdmi_device *hdev)
{
	const struct hdmi_config *conf = hdev->cur_conf;

	hdmi_dbg(3, "start\n");

	hdmi_reg_init(hdev);

	/* setting core registers */
	hdmi_timing_apply(hdev, conf);

	if (hdev->dvi_mode)
		hdmi_set_dvi_mode(hdev);
	else
		hdmi_avi_packet_config(hdev);

	/* enable HDMI and timing generator */
	hdmi_enable(hdev, 1);

#ifdef HDMI_FORCE_PATTERN
	hdmi_video_pattern_enable(hdev, conf);
#endif

	hdev->streaming = HDMI_STREAMING;

	/* change the HPD interrupt: External -> Internal */
	disable_irq(hdev->ext_irq);
	cancel_delayed_work(&hdev->hpd_work_ext);
	hdmi_reg_set_int_hpd(hdev);
	enable_irq(hdev->int_irq);
	hdmi_info("HDMI interrupt changed to internal\n");

	/* start HDCP if enabled */
	if (hdev->hdcp_info.hdcp_enable) {
		hdcp_start(hdev);
	}

	hdmi_dbg(3, "end\n");

	return 0;
}

static void hdmi_hpd_changed(struct hdmi_device *hdev, int state)
{
	if (state) {
		/* We don't read the EDID */
		hdev->dvi_mode = 0;
		hdev->cur_timings = hdmi_config[HDMI_DEFAULT_TIMINGS_IDX].dv_timings;
		hdev->cur_conf = hdmi_timing2conf(&hdev->cur_timings);
	}

	hdmi_info("%s\n", state ? "plugged" : "unplugged");

#ifdef HDMI_FORCE_PATTERN
	hdmi_s_power(hdev, 1);
	hdmi_streamon(hdev);
#endif
}

static void hdmi_hpd_work_ext(struct work_struct *work)
{
	int state = 0;
	struct hdmi_device *hdmi_dev = container_of(work, struct hdmi_device,
						hpd_work_ext.work);

	state = gpio_get_value(hdmi_dev->res.gpio_hpd);
	hdmi_hpd_changed(hdmi_dev, state);

	if (hdmi_dev->drm_dev)
		drm_helper_hpd_irq_event(hdmi_dev->drm_dev);
}

static void hdmi_hpd_work(struct work_struct *work)
{
	struct hdmi_device *hdmi_dev = container_of(work, struct hdmi_device,
						hpd_work);

	hdmi_hpd_changed(hdmi_dev, 0);
}

irqreturn_t hdmi_irq_handler_ext(int irq, void *dev_data)
{
	struct hdmi_device *hdev = dev_data;

	hdmi_dbg(4, "start\n");

	queue_delayed_work(hdev->hpd_wq_ext, &hdev->hpd_work_ext, 0);

	hdmi_dbg(4, "end\n");

	return IRQ_HANDLED;
}

static void exynos_hdmi_enable(struct drm_encoder *encoder)
{
	struct hdmi_device *hdmi_dev = encoder_to_hdmi(encoder);

	if (hdmi_dev->powered)
		return;

	hdmi_dev->powered = true;

	DRM_DEBUG_KMS(" is called\n");

	hdmi_s_power(hdmi_dev, 1);

	hdmi_streamon(hdmi_dev);
}

static void exynos_hdmi_disable(struct drm_encoder *encoder)
{
	struct hdmi_device *hdmi_dev = encoder_to_hdmi(encoder);
	struct drm_crtc *crtc = encoder->crtc;
	const struct drm_crtc_helper_funcs *funcs = NULL;

	if (!hdmi_dev->powered)
		return;

	DRM_DEBUG_KMS(" is called\n");

	hdcp_stop(hdmi_dev);
	/*
	 * The SFRs of VP and Mixer are updated by Vertical Sync of
	 * Timing generator which is a part of HDMI so the sequence
	 * to disable TV Subsystem should be as following,
	 *	VP -> Mixer -> HDMI
	 *
	 * Below codes will try to disable Mixer and VP(if used)
	 * prior to disabling HDMI.
	 */
	if (crtc)
		funcs = crtc->helper_private;
	if (funcs && funcs->disable)
		(*funcs->disable)(crtc);

	hdmi_s_power(hdmi_dev, 0);

	hdmi_dev->powered = false;
}

static struct drm_encoder_helper_funcs exynos_hdmi_encoder_helper_funcs = {
	.mode_fixup	= exynos_hdmi_mode_fixup,
	.mode_set	= exynos_hdmi_mode_set,
	.enable		= exynos_hdmi_enable,
	.disable	= exynos_hdmi_disable,
};

static struct drm_encoder_funcs exynos_hdmi_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static void exynos_hdmi_resources_cleanup(struct hdmi_device *hdmi_dev)
{
	struct hdmi_resources *res = &hdmi_dev->res;

	DRM_DEBUG_DRIVER("HDMI resource cleanup\n");

	/* put clocks */
	if (!IS_ERR_OR_NULL(res->hdmi))
		clk_put(res->hdmi);
	if (!IS_ERR_OR_NULL(res->pixel))
		clk_put(res->pixel);
	if (!IS_ERR_OR_NULL(res->tmds))
		clk_put(res->tmds);
	memset(res, 0, sizeof *res);
}

static int exynos_hdmi_resources_init(struct hdmi_device *hdmi_dev)
{
	struct device *dev = hdmi_dev->dev;
	struct hdmi_resources *res = &hdmi_dev->res;

	DRM_DEBUG_DRIVER("HDMI resource init\n");

	memset(res, 0, sizeof *res);
	/* get clocks, power */

	res->hdmi = clk_get(dev, "hdmi");
	if (IS_ERR_OR_NULL(res->hdmi)) {
		DRM_ERROR("failed to get clock 'pclk_hdmi'\n");
		goto fail;
	}
	res->pixel = clk_get(dev, "hdmi_pixel");
	if (IS_ERR_OR_NULL(res->pixel)) {
		DRM_ERROR("failed to get clock 'hdmi_pixel'\n");
		goto fail;
	}
	res->tmds = clk_get(dev, "hdmi_tmds");
	if (IS_ERR_OR_NULL(res->tmds)) {
		DRM_ERROR("failed to get clock 'hdmi_tmds'\n");
		goto fail;
	}

	return 0;
fail:
	DRM_ERROR("HDMI resource init - failed\n");
	exynos_hdmi_resources_cleanup(hdmi_dev);
	return -ENODEV;

}


static struct of_device_id exynos_hdmi_match_types[] = {
	{ .compatible = "samsung,exynos8-hdmi"},
	{ }
};
MODULE_DEVICE_TABLE (of, exynos_hdmi_match_types);

static int exynos_hdmi_bind(struct device *dev, struct device *master, void *data)
{
	struct drm_device *drm_dev = data;
	struct hdmi_device *hdmi_dev = dev_get_drvdata(dev);
	struct drm_encoder *encoder = &hdmi_dev->encoder;
	int ret, pipe;

	hdmi_dev->drm_dev = drm_dev;

	pipe = exynos_drm_crtc_get_pipe_from_type(drm_dev, DECON_OUT_HDMI);
	if (pipe < 0)
		return pipe;

	encoder->possible_crtcs = 1 << pipe;

	DRM_DEBUG_DRIVER("possible_crtcs = 0x%x\n", encoder->possible_crtcs);

	drm_encoder_init(drm_dev, encoder, &exynos_hdmi_encoder_funcs,
			 DRM_MODE_ENCODER_TMDS, NULL);

	drm_encoder_helper_add(encoder, &exynos_hdmi_encoder_helper_funcs);

	ret = exynos_hdmi_create_connector(encoder);
	if (ret) {
		DRM_ERROR("failed to create connector ret = %d\n", ret);
		drm_encoder_cleanup(encoder);
		return ret;
	}

	return 0;
}

static void exynos_hdmi_unbind(struct device *dev, struct device *master, void *data)
{
}

static const struct component_ops exynos_hdmi_component_ops = {
	.bind	= exynos_hdmi_bind,
	.unbind = exynos_hdmi_unbind,
};

int exynos_hdmi_set_gpio(struct hdmi_device *hdmi_dev)
{
	struct device *dev = hdmi_dev->dev;
	struct hdmi_resources *res = &hdmi_dev->res;
	int ret = 0;

	if (of_get_property(dev->of_node, "gpios", NULL) != NULL) {
		/* HPD */
		res->gpio_hpd = of_get_gpio(dev->of_node, 0);
		if (res->gpio_hpd < 0) {
			dev_err(dev, "failed to get gpio hpd\n");
			return -ENODEV;
		}
		if (gpio_request(res->gpio_hpd, "hdmi-hpd")) {
			dev_err(dev, "failed to request hdmi-hpd\n");
			ret = -ENODEV;
		} else {
			gpio_direction_input(res->gpio_hpd);
			dev_info(dev, "success request GPIO for hdmi-hpd\n");
		}
		/* Level shifter */
		res->gpio_ls = of_get_gpio(dev->of_node, 1);
		if (res->gpio_ls < 0) {
			dev_info(dev, "There is no gpio for level shifter\n");
			return 0;
		}
		if (gpio_request(res->gpio_ls, "hdmi-ls")) {
			dev_err(dev, "failed to request hdmi-ls\n");
			ret = -ENODEV;
		} else {
			gpio_direction_output(res->gpio_ls, 1);
			gpio_set_value(res->gpio_ls, 1);
			dev_info(dev, "success request GPIO for hdmi-ls\n");
		}
		/* DCDC */
		res->gpio_dcdc = of_get_gpio(dev->of_node, 2);
		if (res->gpio_dcdc < 0) {
			dev_err(dev, "failed to get gpio dcdc\n");
			return -ENODEV;
		}
		if (gpio_request(res->gpio_dcdc, "hdmi-dcdc")) {
			dev_err(dev, "failed to request hdmi-dcdc\n");
			ret = -ENODEV;
		} else {
			gpio_direction_output(res->gpio_dcdc, 1);
			gpio_set_value(res->gpio_dcdc, 1);
			dev_info(dev, "success request GPIO for hdmi-dcdc\n");
		}
	}

	return ret;
}

int exynos_hdmi_get_sysreg_addr(struct hdmi_device *hdmi_dev)
{
	struct device_node *hdmiphy_sys;

	hdmiphy_sys = of_get_child_by_name(hdmi_dev->dev->of_node, "hdmiphy-sys");
	if (!hdmiphy_sys) {
		DRM_ERROR("No sys-controller interface for hdmiphy\n");
		return -ENODEV;
	}
	hdmi_dev->pmu_regs = of_iomap(hdmiphy_sys, 0);
	if (hdmi_dev->pmu_regs == NULL) {
		DRM_ERROR("Can't get hdmiphy pmu control register\n");
		return -ENOMEM;
	}

	return 0;
}

static int exynos_hdmi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hdmi_device *hdmi_dev;
	struct resource *res;
	int ret;

	DRM_INFO("%s +\n", __func__);

	hdmi_dev = devm_kzalloc(dev, sizeof(struct hdmi_device), GFP_KERNEL);
	if (!hdmi_dev)
		return -ENOMEM;

	platform_set_drvdata(pdev, hdmi_dev);

	hdmi_dev->dev = dev;

	/* mapping HDMI registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		DRM_ERROR("get hdmi memory resource failed.\n");
		return -ENXIO;
	}
	hdmi_dev->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(hdmi_dev->regs)) {
		DRM_ERROR("failed to claim register region for hdmi\n");
		ret = PTR_ERR(hdmi_dev->regs);
		return ret;
	}

	/* mapping HDMIPHY_APB registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		DRM_ERROR("get hdmiphy memory resource failed.\n");
		return -ENXIO;
	}
	hdmi_dev->phy_regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(hdmi_dev->phy_regs)) {
		DRM_ERROR("failed to claim register region for hdmiphy\n");
		ret = PTR_ERR(hdmi_dev->phy_regs);
		return ret;
	}

	/* mapping HDCP_OTP registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!res) {
		DRM_ERROR("get hdcp_otp memory resource failed.\n");
		return -ENXIO;
	}
	hdmi_dev->otp_regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(hdmi_dev->otp_regs)) {
		DRM_ERROR("failed to claim register region for hdcp_otp\n");
		ret = PTR_ERR(hdmi_dev->otp_regs);
		return ret;
	}

	/* Internal hpd */
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		DRM_ERROR("get internal interrupt resource failed.\n");
		return -ENXIO;
	}
	ret = devm_request_irq(dev, res->start, hdmi_irq_handler,
			0, "hdmi-int", hdmi_dev);
	if (ret) {
		DRM_ERROR("request int interrupt failed.\n");
		return ret;
	} else {
		hdmi_dev->int_irq = res->start;
		disable_irq(hdmi_dev->int_irq);
		DRM_INFO("success request hdmi-int irq : %d\n", hdmi_dev->int_irq);
	}

	/* DDC i2c driver */
	if (i2c_add_driver(&ddc_driver)) {
		DRM_ERROR("failed to register ddc i2c driver\n");
		return -ENOENT;
	}
	hdmi_dev->ddc_port = hdmi_ddc;

	hdmi_dev->hpd_wq = create_singlethread_workqueue("hdmi_hpd");
	if (!hdmi_dev->hpd_wq) {
		hdmi_err("failed to create workqueue for internal hpd\n");
		goto fail_ddc;
	}
	INIT_WORK(&hdmi_dev->hpd_work, hdmi_hpd_work);

	hdmi_dev->hpd_wq_ext = create_singlethread_workqueue("hdmi_hpd_ext");
	if (!hdmi_dev->hpd_wq_ext) {
		hdmi_err("failed to create workqueue for internal hpd_ext\n");
		goto fail_ddc;
	}
	INIT_DELAYED_WORK(&hdmi_dev->hpd_work_ext, hdmi_hpd_work_ext);

	/* setting the clocks */
	ret = exynos_hdmi_resources_init(hdmi_dev);
	if (ret) {
		DRM_ERROR("hdmi_resources_init failed\n");
		return ret;
	}

	/* HDMI pin control */
	hdmi_dev->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(hdmi_dev->pinctrl)) {
		DRM_ERROR("could not set hdmi internal hpd pins\n");
		goto fail_clk;
	} else {
		hdmi_dev->pin_int = pinctrl_lookup_state(hdmi_dev->pinctrl, "hdmi_hdmi_hpd");
		if(IS_ERR(hdmi_dev->pin_int)) {
			DRM_ERROR("could not get hdmi internal hpd pin state\n");
			goto fail_clk;
		}
		hdmi_dev->pin_ext = pinctrl_lookup_state(hdmi_dev->pinctrl, "hdmi_ext_hpd");
		if(IS_ERR(hdmi_dev->pin_ext)) {
			DRM_ERROR("could not get hdmi external hpd pin state\n");
			goto fail_clk;
		}
	}

	/* setting the GPIO */
	ret = exynos_hdmi_set_gpio(hdmi_dev);
	if (ret) {
		DRM_ERROR("failed to get GPIO\n");
		goto fail_clk;
	}
	/* External hpd */
	hdmi_dev->ext_irq = gpio_to_irq(hdmi_dev->res.gpio_hpd);
	ret = devm_request_irq(dev, hdmi_dev->ext_irq, hdmi_irq_handler_ext,
			IRQ_TYPE_EDGE_BOTH, "hdmi-ext", hdmi_dev);
	if (ret) {
		DRM_ERROR("request ext interrupt failed.\n");
		goto fail_gpio;
	} else {
		DRM_INFO("success request hdmi-ext irq : %d\n", hdmi_dev->ext_irq);
	}

	mutex_init(&hdmi_dev->mutex);
	hdmi_dev->cur_timings =
		hdmi_config[HDMI_DEFAULT_TIMINGS_IDX].dv_timings;
	hdmi_dev->cur_conf = &hdmi_config[HDMI_DEFAULT_TIMINGS_IDX];

	/* default audio configuration : disable audio */
	hdmi_dev->audio_enable = 0;
	hdmi_dev->audio_channel_count = 2;
	hdmi_dev->sample_rate = DEFAULT_SAMPLE_RATE;
	hdmi_dev->color_range = HDMI_RGB709_0_255;
	hdmi_dev->bits_per_sample = DEFAULT_BITS_PER_SAMPLE;
	hdmi_dev->audio_codec = DEFAULT_AUDIO_CODEC;

	/* default aspect ratio is 16:9 */
	hdmi_dev->aspect = HDMI_ASPECT_RATIO_16_9;

	/* default HDMI streaming is stoped */
	hdmi_dev->streaming = HDMI_STOP;

	pm_runtime_enable(dev);

	/* work after booting */
	queue_delayed_work(hdmi_dev->hpd_wq_ext, &hdmi_dev->hpd_work_ext,
					msecs_to_jiffies(8000));

	/* mapping SYSTEM registers */
	ret = exynos_hdmi_get_sysreg_addr(hdmi_dev);
	if (ret){
		goto fail_mutex;
	}

	ret = hdcp_prepare(hdmi_dev);
	if (ret)
		goto fail_mutex;

	ret = component_add(&pdev->dev, &exynos_hdmi_component_ops);
	if (ret){
		return ret;
	}

	DRM_INFO("%s -\n", __func__);

	return 0;

fail_mutex:
	mutex_destroy(&hdmi_dev->mutex);
fail_gpio:
	gpio_free(hdmi_dev->res.gpio_hpd);
	gpio_free(hdmi_dev->res.gpio_ls);
	gpio_free(hdmi_dev->res.gpio_dcdc);
fail_clk:
	exynos_hdmi_resources_cleanup(hdmi_dev);
fail_ddc:
	i2c_del_driver(&ddc_driver);
	hdmi_err("probe failed\n");
	return ret;

}

static int exynos_hdmi_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hdmi_device *hdmi_dev = platform_get_drvdata(pdev);

	DRM_INFO("%s +\n", __func__);

	cancel_delayed_work(&hdmi_dev->hpd_work_ext);

	pm_runtime_disable(dev);
	free_irq(hdmi_dev->ext_irq, hdmi_dev);
	free_irq(hdmi_dev->int_irq, hdmi_dev);
	mutex_destroy(&hdmi_dev->mutex);
	iounmap(hdmi_dev->regs);
	exynos_hdmi_resources_cleanup(hdmi_dev);
	i2c_del_driver(&ddc_driver);

	component_del(&pdev->dev, &exynos_hdmi_component_ops);

	kfree(hdmi_dev);

	DRM_INFO("%s -\n", __func__);

	return 0;
}

struct platform_driver hdmi_driver = {
	.probe = exynos_hdmi_probe,
	.remove = exynos_hdmi_remove,
	.driver = {
		   .name = "exynos-hdmi",
		   .owner = THIS_MODULE,
		   .pm	= &exynos_hdmi_pm_ops,
		   .of_match_table = exynos_hdmi_match_types,
	},
};
