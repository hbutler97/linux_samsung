/* linux/drivers/gpu/drm/exynos-hq/exynos_drm_decon.c
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Samsung Exynos DECON driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/component.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>
#include <linux/exynos_iovmm.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include <linux/types.h>

#include "exynos_drm_drv.h"
#include "exynos_drm_crtc.h"
#include "exynos_drm_fb.h"
#include "exynos_drm_plane.h"
#include "exynos_drm_decon.h"
#include "exynos_drm_dsi.h"
#include "exynos_drm_vpp.h"

#include "regs-decon.h"

struct decon_context *decon_drvdata[MAX_DECON_CNT];
EXPORT_SYMBOL(decon_drvdata);

#ifndef MHZ
#define MHZ	(1000*1000)
#endif

#define RGB_IMG_WIDTH_MIN      16
#define IMG_WIDTH_MAX          4096
#define RGB_IMG_HEIGHT_MIN     8
#define IMG_HEIGHT_MAX         4096
#define SCALED_WIDTH_MIN       16
#define SCALED_WIDTH_MAX       4096
#define SCALED_HEIGHT_MIN      8
#define SCALED_HEIGHT_MAX      4096

static const char * const decon_clks_name[] = {
	"disp_pll",
	"decon_pclk",
	"eclk_user",
	"eclk_leaf",
	"vclk_user",
	"vclk_leaf",
};

static const char * const decon_t_clks_name[] = {
	"hdmi_pixel",
	"decon_pclk",
	"eclk_user",
	"eclk_leaf",
	"p_eclk_user",
	"p_eclk_leaf",
};

static const uint32_t decon_formats[] = {
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_RGBA8888,
	DRM_FORMAT_BGRA8888,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_RGBX8888,
	DRM_FORMAT_BGRX8888,
	DRM_FORMAT_RGBA5551,
	DRM_FORMAT_RGB565,
};

static int exynos_decon_resume(struct device *dev);
static int exynos_decon_suspend(struct device *dev);

static inline void decon_set_bits(struct decon_context *ctx, u32 reg, u32 mask,
				  u32 val)
{
	DRM_DEBUG_KMS("%s \n", __func__);
	val = (val & mask) | (readl(ctx->regs + reg) & ~mask);
	writel(val, ctx->regs + reg);
}

static int decon_enable_vblank(struct exynos_drm_crtc *crtc)
{
	struct decon_context *ctx = crtc->ctx;

	DRM_DEBUG_KMS("%s +\n", __func__);

	if (test_bit(BIT_SUSPENDED, &ctx->flags))
		return -EPERM;

	DRM_DEBUG_KMS("%s -\n", __func__);
	return 0;
}

static void decon_disable_vblank(struct exynos_drm_crtc *crtc)
{
	struct decon_context *ctx = crtc->ctx;

	DRM_DEBUG_KMS("%s +\n", __func__);

	if (test_bit(BIT_SUSPENDED, &ctx->flags))
		return;

	clear_bit(BIT_IRQS_ENABLED, &ctx->flags);
	DRM_DEBUG_KMS("%s -\n", __func__);
}

static void decon_atomic_begin(struct exynos_drm_crtc *crtc)
{
	struct decon_context *ctx = crtc->ctx;

	DRM_DEBUG_KMS("%s +\n", __func__);

	if (test_bit(BIT_SUSPENDED, &ctx->flags))
		return;

	DRM_DEBUG_KMS("%s -\n", __func__);
}

#define BIT_VAL(x, e, s) (((x) & ((1 << ((e) - (s) + 1)) - 1)) << (s))
#define COORDINATE_X(x) BIT_VAL((x), 23, 12)
#define COORDINATE_Y(x) BIT_VAL((x), 11, 0)

static enum decon_pixel_format decon_translate_pixfmt(uint32_t pixel_format)
{
	DRM_DEBUG_KMS("%s +\n", __func__);

	switch (pixel_format) {
	case DRM_FORMAT_ARGB8888:
		return DECON_PIXEL_FORMAT_ARGB_8888;
	case DRM_FORMAT_ABGR8888:
		return DECON_PIXEL_FORMAT_ABGR_8888;
	case DRM_FORMAT_RGBA8888:
		return DECON_PIXEL_FORMAT_RGBA_8888;
	case DRM_FORMAT_BGRA8888:
		return DECON_PIXEL_FORMAT_BGRA_8888;
	case DRM_FORMAT_XRGB8888:
		return DECON_PIXEL_FORMAT_XRGB_8888;
	case DRM_FORMAT_XBGR8888:
		return DECON_PIXEL_FORMAT_XBGR_8888;
	case DRM_FORMAT_RGBX8888:
		return DECON_PIXEL_FORMAT_RGBX_8888;
	case DRM_FORMAT_BGRX8888:
		return DECON_PIXEL_FORMAT_BGRX_8888;
	case DRM_FORMAT_RGBA5551:
		return DECON_PIXEL_FORMAT_RGBA_5551;
	case DRM_FORMAT_RGB565:
		return DECON_PIXEL_FORMAT_RGB_565;
	default:
		DRM_ERROR("0x%x is not supported format\n", pixel_format);
	}

	DRM_DEBUG_KMS("%s -\n", __func__);
	return -EINVAL;
}

u32 wincon(u32 transp_len, u32 a0, u32 a1,
	int plane_alpha, enum decon_blending blending)
{
	u32 data = 0;
	int is_plane_alpha = (plane_alpha < 255 && plane_alpha > 0) ? 1 : 0;

	DRM_DEBUG_KMS("%s +\n", __func__);

	if (transp_len == 1 && blending == DECON_BLENDING_PREMULT)
		blending = DECON_BLENDING_COVERAGE;

	if (is_plane_alpha) {
		if (transp_len) {
			if (blending != DECON_BLENDING_NONE)
				data |= WIN_CONTROL_ALPHA_MUL_F;
		} else {
			if (blending == DECON_BLENDING_PREMULT)
				blending = DECON_BLENDING_COVERAGE;
		}
	}

	if (transp_len > 1)
		data |= WIN_CONTROL_ALPHA_SEL_F(W_ALPHA_SEL_F_BYAEN);

	switch (blending) {
	case DECON_BLENDING_NONE:
		data |= WIN_CONTROL_FUNC_F(PD_FUNC_COPY);
		break;

	case DECON_BLENDING_PREMULT:
		if (!is_plane_alpha) {
			data |= WIN_CONTROL_FUNC_F(PD_FUNC_SOURCE_OVER);
		} else {
			/* need to check the eq: it is SPEC-OUT */
			data |= WIN_CONTROL_FUNC_F(PD_FUNC_LEGACY2);
		}
		break;

	case DECON_BLENDING_COVERAGE:
	default:
		data |= WIN_CONTROL_FUNC_F(PD_FUNC_LEGACY);
		break;
	}

	data |= WIN_CONTROL_ALPHA0_F(a0) | WIN_CONTROL_ALPHA1_F(a1);
	data |= WIN_CONTROL_EN_F;

	DRM_DEBUG_KMS("%s -\n", __func__);
	return data;
}

static int decon_check_limitation(struct exynos_drm_plane_state *state)
{
	if ((state->src.w < RGB_IMG_WIDTH_MIN) || (state->src.w > IMG_WIDTH_MAX))
		return -EINVAL;
	if ((state->src.h < RGB_IMG_HEIGHT_MIN) || (state->src.h > IMG_HEIGHT_MAX))
		return -EINVAL;
	if ((state->crtc.w < SCALED_WIDTH_MIN) || (state->src.w > SCALED_WIDTH_MAX))
		return -EINVAL;
	if ((state->crtc.h < SCALED_HEIGHT_MIN) || (state->src.h > SCALED_HEIGHT_MAX))
		return -EINVAL;

	return 0;
}

static void decon_convert_vpp_params(struct vpp_params_info *vpp_params,
				struct exynos_drm_plane_state *state,
				struct decon_lcd *lcd_info)
{
	struct drm_framebuffer *fb = state->base.fb;

	vpp_params->src.x = state->src.x;
	vpp_params->src.y = state->src.y;
	vpp_params->src.w = state->src.w;
	vpp_params->src.h = state->src.h;
	vpp_params->src.f_w = fb->pitches[0] / (fb->bits_per_pixel / 8);
	vpp_params->src.f_h = fb->height;
	vpp_params->dst.x = state->crtc.x;
	vpp_params->dst.y = state->crtc.y;
	vpp_params->dst.w = state->crtc.w;
	vpp_params->dst.h = state->crtc.h;
	vpp_params->dst.f_w = lcd_info->xres;
	vpp_params->dst.f_h = lcd_info->yres;
	if ((state->src.w != state->crtc.w) || (state->src.h != state->crtc.h))
		vpp_params->is_scale = true;
	else
		vpp_params->is_scale = false;
	vpp_params->format = decon_translate_pixfmt(fb->pixel_format);
	vpp_params->addr[0] = exynos_drm_fb_dma_addr(fb, 0);
	vpp_params->addr[1] = exynos_drm_fb_dma_addr(fb, 1);
	vpp_params->addr[2] = exynos_drm_fb_dma_addr(fb, 2);
	vpp_params->h_ratio = (state->src.w << 20) / state->crtc.w;
	vpp_params->v_ratio = (state->src.h << 20) / state->crtc.h;
	if ((vpp_params->h_ratio != (1 << 20)) ||
			(vpp_params->v_ratio != (1 << 20)))
		vpp_params->is_scale = true;
	else
		vpp_params->is_scale = false;
}

static void decon_update_plane(struct exynos_drm_crtc *crtc,
			       struct exynos_drm_plane *plane)
{
	struct exynos_drm_plane_state *state =
				to_exynos_plane_state(plane->base.state);
	struct decon_context *ctx = crtc->ctx;
	struct decon_window_regs win_regs;
	struct vpp_params_info vpp_params;
	int ret;

	DRM_DEBUG_KMS("%s +\n", __func__);

	if (test_bit(BIT_SUSPENDED, &ctx->flags))
		return;

	ret = decon_check_limitation(state);
	if (ret) {
		DRM_ERROR("not supported coordinates\n");
		return;
	}

	memset(&win_regs, 0, sizeof(struct decon_window_regs));
	win_regs.wincon = wincon(0x8, 0xFF, 0xFF, 0xFF, DECON_BLENDING_NONE);
	win_regs.start_pos = win_start_pos(state->crtc.x, state->crtc.y);
	win_regs.end_pos = win_end_pos(state->crtc.x, state->crtc.y,
			state->crtc.w, state->crtc.h);
	DRM_DEBUG_KMS("win %d idma %d xres %d yres %d start_pos %x end_pos %x\n",
			plane->win_idx, plane->idma_idx,
			state->crtc.w, state->crtc.h,
			win_regs.start_pos, win_regs.end_pos);
	win_regs.colormap = 0x000000;
	win_regs.pixel_count = state->crtc.w * state->crtc.h;
	/* FILL ME */
	win_regs.whole_w = ctx->lcd_info->xres;
	win_regs.whole_h = ctx->lcd_info->yres;
	win_regs.offset_x = state->crtc.x;
	win_regs.offset_y = state->crtc.y;
	win_regs.type = plane->idma_idx;
	decon_reg_set_window_control(ctx->id, plane->win_idx, &win_regs, false);

	memset(&vpp_params, 0, sizeof(struct vpp_params_info));
	decon_convert_vpp_params(&vpp_params, state, ctx->lcd_info);
	exynos_vpp_set_config(plane->idma_idx, &vpp_params);

	decon_reg_update_req_window(ctx->id, plane->win_idx);

	decon_reg_start(ctx->id, &ctx->psr);

	DRM_DEBUG_KMS("%s -\n", __func__);
}

static void decon_disable_plane(struct exynos_drm_crtc *crtc,
				struct exynos_drm_plane *plane)
{
	struct decon_context *ctx = crtc->ctx;

	DRM_INFO("%s +\n", __func__);

	if (test_bit(BIT_SUSPENDED, &ctx->flags))
		return;

	DRM_INFO("%s -\n", __func__);
}

static void decon_atomic_flush(struct exynos_drm_crtc *crtc)
{
	struct decon_context *ctx = crtc->ctx;

	DRM_DEBUG_KMS("%s +\n", __func__);

	if (test_bit(BIT_SUSPENDED, &ctx->flags))
		return;

	if (ctx->psr.psr_mode == DECON_MIPI_COMMAND_MODE)
		set_bit(BIT_WIN_UPDATED, &ctx->flags);

	DRM_DEBUG_KMS("%s -\n", __func__);
}

static void decon_enable(struct exynos_drm_crtc *crtc)
{
	struct decon_context *ctx = crtc->ctx;
	struct decon_param p;
#ifdef WINMAP
	struct decon_window_regs win_regs;
#endif
	DRM_INFO("%s +\n", __func__);

	if (!test_and_clear_bit(BIT_SUSPENDED, &ctx->flags))
		return;

#if defined(CONFIG_PM_RUNTIME)
	pm_runtime_get_sync(ctx->dev);
#else
	exynos_decon_resume(ctx->dev);
#endif

	set_bit(BIT_CLKS_ENABLED, &ctx->flags);

	if (ctx->pinctrl && ctx->decon_te_on) {
		if (pinctrl_select_state(ctx->pinctrl, ctx->decon_te_on)) {
			DRM_ERROR("failed to turn on Decon_TE\n");
			return;
		}
	}

	memcpy(&p.psr, &ctx->psr, sizeof(struct decon_mode_info));
	p.lcd_info = ctx->lcd_info;
	p.disp_ss_regs = ctx->ss_regs;
	decon_reg_init(ctx->id, ctx->psr.out_idx, &p);
#ifdef WINMAP
	memset(&win_regs, 0, sizeof(struct decon_window_regs));
	win_regs.wincon = wincon(0x8, 0xFF, 0xFF, 0xFF, DECON_BLENDING_NONE);
	win_regs.wincon |= WIN_CONTROL_EN_F;
	win_regs.start_pos = win_start_pos(0, 0);
	win_regs.end_pos = win_end_pos(0, 0, ctx->lcd_info->xres, ctx->lcd_info->yres);
	win_regs.colormap = 0x00FF00;
	win_regs.pixel_count = ctx->lcd_info->xres * ctx->lcd_info->yres;
	win_regs.whole_w = ctx->lcd_info->xres;
	win_regs.whole_h = ctx->lcd_info->yres;
	win_regs.offset_x = 0;
	win_regs.offset_y = 0;
	decon_reg_set_window_control(ctx->id, 0, &win_regs, true);
	decon_reg_update_req_window(ctx->id, 0);
#endif
	enable_irq(ctx->irq);
	decon_reg_set_int(ctx->id, &p.psr, 1);
	decon_reg_start(ctx->id, &p.psr);

#if 0
	/* if vblank was enabled status, enable it again. */
	if (test_and_clear_bit(BIT_IRQS_ENABLED, &ctx->flags))
		decon_enable_vblank(ctx->crtc);

	decon_commit(ctx->crtc);
#endif
	DRM_INFO("%s -\n", __func__);
}

static void decon_disable(struct exynos_drm_crtc *crtc)
{
	struct decon_context *ctx = crtc->ctx;
	struct decon_param p;

	DRM_DEBUG_KMS("%s +\n", __func__);

	if (test_bit(BIT_SUSPENDED, &ctx->flags))
		return;

	memcpy(&p.psr, &ctx->psr, sizeof(struct decon_mode_info));
	p.disp_ss_regs = ctx->ss_regs;

	decon_reg_set_int(ctx->id, &p.psr, 0);
	disable_irq(ctx->irq);

	decon_reg_stop(ctx->id, 0, &p.psr);
	decon_reg_clear_int_all(ctx->id);

	clear_bit(BIT_CLKS_ENABLED, &ctx->flags);

#if defined(CONFIG_PM_RUNTIME)
	pm_runtime_put_sync(ctx->dev);
#else
	exynos_decon_suspend(ctx->dev);
#endif

	set_bit(BIT_SUSPENDED, &ctx->flags);

	DRM_DEBUG_KMS("%s -\n", __func__);
}

void decon_te_irq_handler(struct exynos_drm_crtc *crtc)
{
	struct decon_context *ctx = crtc->ctx;

	if (!ctx->pinctrl)
		return;

	if (!test_bit(BIT_CLKS_ENABLED, &ctx->flags))
		return;

	clear_bit(BIT_WIN_UPDATED, &ctx->flags);

	drm_crtc_handle_vblank(&ctx->crtc->base);
}

static void decon_clear_channels(struct exynos_drm_crtc *crtc)
{
	struct decon_context *ctx = crtc->ctx;
	int i, ret;

	DRM_DEBUG_KMS("%s +\n", __func__);

	for (i = 0; i < ctx->clk_cnt; i++) {
		ret = clk_prepare_enable(ctx->clks[i]);
		if (ret < 0)
			goto err;
	}

	/* TODO: wait for possible vsync */
	msleep(50);

	DRM_DEBUG_KMS("%s -\n", __func__);
err:
	while (--i >= 0)
		clk_disable_unprepare(ctx->clks[i]);
}

static struct exynos_drm_crtc_ops decon_crtc_ops = {
	.enable			= decon_enable,
	.disable		= decon_disable,
	.enable_vblank		= decon_enable_vblank,
	.disable_vblank		= decon_disable_vblank,
	.atomic_begin		= decon_atomic_begin,
	.update_plane		= decon_update_plane,
	.disable_plane		= decon_disable_plane,
	.atomic_flush		= decon_atomic_flush,
	.te_handler		= decon_te_irq_handler,
};

static int decon_bind(struct device *dev, struct device *master, void *data)
{
	struct decon_context *ctx = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	struct exynos_drm_private *priv = drm_dev->dev_private;
	struct exynos_drm_plane *exynos_plane;
	struct exynos_dsi *dsi;
	struct decon_lcd *lcd_info;
	int i, ret, type;

	DRM_INFO("%s +\n", __func__);

	lcd_info = kzalloc(sizeof(struct decon_lcd), GFP_KERNEL);
	if (!lcd_info) {
		DRM_ERROR("could not allocate decon_lcd for wb\n");
		return -ENOMEM;
	}
	ctx->drm_dev = drm_dev;
	ctx->pipe = priv->pipe++;

	DRM_INFO("pipe = %d\n", ctx->pipe);

	for (i = 0; i < ctx->nr_wins; i++) {
		type = (i == 0) ?
			DRM_PLANE_TYPE_PRIMARY : DRM_PLANE_TYPE_OVERLAY;
		ctx->configs[i].pixel_formats = decon_formats;
		ctx->configs[i].num_pixel_formats = ARRAY_SIZE(decon_formats);
		ctx->configs[i].zpos = i;
		ctx->configs[i].type = type;
		ret = exynos_plane_init(drm_dev, &ctx->planes[i], i,
					1 << ctx->pipe, &ctx->configs[i]);
		if (ret)
			goto err_plane;
	}

	exynos_plane = &ctx->planes[0];
	ctx->crtc = exynos_drm_crtc_create(drm_dev, &exynos_plane->base,
					ctx->pipe, ctx->psr.out_type,
					&decon_crtc_ops, ctx);
	if (IS_ERR(ctx->crtc)) {
		ret = PTR_ERR(ctx->crtc);
		goto err;
	}

	DRM_INFO("crtc id = %d\n", ctx->crtc->base.base.id);
	decon_clear_channels(ctx->crtc);
	if (ctx->id != 2) {
		dsi = get_dsim_drvdata(ctx->id);
		ctx->lcd_info = &dsi->lcd_info;
	} else {
		ctx->lcd_info = lcd_info;
		ctx->lcd_info->width = 1920;
		ctx->lcd_info->height = 1080;
		ctx->lcd_info->xres = 1920;
		ctx->lcd_info->yres = 1080;
		ctx->lcd_info->vfp = 4;
		ctx->lcd_info->vbp = 36;
		ctx->lcd_info->hfp = 88;
		ctx->lcd_info->hbp = 148;
		ctx->lcd_info->vsa = 5;
		ctx->lcd_info->hsa = 44;
		ctx->lcd_info->fps = 60;
	}

	DRM_INFO("lcd xres(%d), yres(%d)\n",
			ctx->lcd_info->xres, ctx->lcd_info->yres);

	DRM_INFO("%s -\n", __func__);

	return ret;
err_plane:
	exynos_drm_gem_deinit_iommu(drm_dev, dev);
err:
	priv->pipe--;
	return ret;
}

static void decon_unbind(struct device *dev, struct device *master, void *data)
{
	struct decon_context *ctx = dev_get_drvdata(dev);

	DRM_INFO("%s +\n", __func__);

	exynos_drm_gem_deinit_iommu(ctx->drm_dev, dev);

	decon_disable(ctx->crtc);

	DRM_INFO("%s -\n", __func__);
}

static const struct component_ops decon_component_ops = {
	.bind	= decon_bind,
	.unbind = decon_unbind,
};

static irqreturn_t decon_irq_handler(int irq, void *dev_id)
{
	struct decon_context *ctx = dev_id;
	u32 val;
	int i;

	if (!test_bit(BIT_CLKS_ENABLED, &ctx->flags))
		goto out;

	val = exynos_decon_reg_get_interrupt_and_clear(ctx->id);
	val &= INTERRUPT_FRAME_DONE_INT_EN | INTERRUPT_DISPIF_VSTATUS_INT_EN;

	if (val) {
		if (ctx->psr.psr_mode == DECON_VIDEO_MODE)
			drm_crtc_handle_vblank(&ctx->crtc->base);

		for (i = 0; i < ctx->nr_wins; i++) {
			struct exynos_drm_plane *plane = &ctx->planes[i];

			if (!plane->pending_fb)
				continue;

			exynos_drm_crtc_finish_update(ctx->crtc, plane);
		}
	}

out:
	return IRQ_HANDLED;
}

static int exynos_decon_suspend(struct device *dev)
{
	struct decon_context *ctx = dev_get_drvdata(dev);
	int i = ctx->clk_cnt;

	DRM_DEBUG_KMS("%s +\n", __func__);
	while (--i >= 0)
		clk_disable_unprepare(ctx->clks[i]);

	DRM_DEBUG_KMS("%s -\n", __func__);
	return 0;
}

void exynos_decon_clocks_info(struct decon_context *ctx)
{
	DRM_DEBUG_KMS("%s: %ld Mhz\n", __clk_get_name(ctx->clks[1]),
				clk_get_rate(ctx->clks[1]) / MHZ);
	DRM_DEBUG_KMS("%s: %ld Mhz\n", __clk_get_name(ctx->clks[3]),
				clk_get_rate(ctx->clks[3]) / MHZ);
	DRM_DEBUG_KMS("%s: %ld Mhz\n", __clk_get_name(ctx->clks[5]),
				clk_get_rate(ctx->clks[5]) / MHZ);
}

#define CLK_CON_MUX_PHYCLK_DISP1_HDMIPHY_PIXEL_CLKO_USER	0x0220
#define CLK_CON_MUX_SCLK_DECON1_ECLK1  0x0230
static int exynos_decon_resume(struct device *dev)
{
	struct decon_context *ctx = dev_get_drvdata(dev);
	int i, ret;
	struct decon_clocks clks;

	DRM_INFO("%s +\n", __func__);

	if (ctx->id == 2) {
		decon_cmu_write(2, CLK_CON_MUX_PHYCLK_DISP1_HDMIPHY_PIXEL_CLKO_USER, 0x201000);
		decon_cmu_write(2, CLK_CON_MUX_SCLK_DECON1_ECLK1, 0x201000);
	}

	for (i = 0; i < ctx->clk_cnt; i++) {
		ret = clk_prepare_enable(ctx->clks[i]);
		if (ret < 0)
			goto err;
	}

	decon_reg_get_clock_ratio(&clks, ctx->lcd_info);
	if (ctx->id != 2) {
		/* VCLK */
		clk_set_rate(ctx->clks[0], clks.decon[CLK_ID_DPLL] * MHZ);
		clk_set_rate(ctx->clks[5], clks.decon[CLK_ID_VCLK] * MHZ);

		/* ECLK */
		clk_set_rate(ctx->clks[2], clks.decon[CLK_ID_ECLK] * MHZ);
		clk_set_rate(ctx->clks[3], clks.decon[CLK_ID_ECLK] * MHZ);

		exynos_decon_clocks_info(ctx);
	} else {
		clk_set_parent(ctx->clks[2], ctx->clks[0]);
		clk_set_rate(ctx->clks[4], clks.decon[CLK_ID_ECLK] * MHZ);
		clk_set_rate(ctx->clks[5], clks.decon[CLK_ID_ECLK] * MHZ);
	}

	DRM_INFO("%s -\n", __func__);

	return 0;
err:
	while (--i >= 0)
		clk_disable_unprepare(ctx->clks[i]);

	return ret;
}

static const struct dev_pm_ops exynos_decon_pm_ops = {
	SET_RUNTIME_PM_OPS(exynos_decon_suspend, exynos_decon_resume,
			   NULL)
};

static const struct of_device_id exynos_decon_driver_dt_match[] = {
	{
		.compatible = "samsung,exynos8-decon",
	},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_decon_driver_dt_match);

static int exynos_decon_get_disp_ss_addr(struct decon_context *ctx)
{
	DRM_DEBUG_KMS("%s +\n", __func__);

	if (of_have_populated_dt()) {
		struct device_node *nd;

		nd = of_find_compatible_node(NULL, NULL,
				"samsung,exynos8-disp_ss");
		if (!nd) {
			DRM_ERROR("failed find compatible node(sysreg-disp)");
			return -ENODEV;
		}

		ctx->ss_regs = of_iomap(nd, 0);
		if (!ctx->ss_regs) {
			DRM_ERROR("Failed to get sysreg-disp address.");
			return -ENOMEM;
		}
	} else {
		DRM_ERROR("failed have populated device tree");
		return -EIO;
	}

	DRM_DEBUG_KMS("%s -\n", __func__);
	return 0;
}

static int exynos_decon_get_pinctrl(struct decon_context *ctx)
{
	DRM_DEBUG_KMS("%s +\n", __func__);

	ctx->pinctrl = devm_pinctrl_get(ctx->dev);
	if (IS_ERR(ctx->pinctrl)) {
		DRM_INFO("failed to get decon0 pinctrl\n");
		ctx->pinctrl = NULL;
		return -EINVAL;
	} else {
		ctx->decon_te_on = pinctrl_lookup_state(ctx->pinctrl, "decon_te_on");
		if (IS_ERR(ctx->decon_te_on)) {
			DRM_ERROR("failed to get decon_te_on pin state\n");
			ctx->decon_te_on = NULL;
		}
		ctx->decon_te_off = pinctrl_lookup_state(ctx->pinctrl, "decon_te_off");
		if (IS_ERR(ctx->decon_te_off)) {
			DRM_ERROR("failed to get decon_te_off pin state\n");
			ctx->decon_te_off = NULL;
		}
	}

	DRM_DEBUG_KMS("%s -\n", __func__);

	return 0;
}

static void exynos_decon_get_clk_count(struct decon_context *ctx)
{
	if (ctx->id != 2)
		ctx->clk_cnt = ARRAY_SIZE(decon_clks_name);
	else
		ctx->clk_cnt = ARRAY_SIZE(decon_t_clks_name);
}

static void exynos_decon_parse_dt(struct decon_context *ctx)
{
	struct device_node *node = ctx->dev->of_node;
	u32 res[MAX_WINDOW_CNT];
	int i;

	ctx->id = of_alias_get_id(node, "decon");
	DRM_INFO("decon%d probe start...\n", ctx->id);

	of_property_read_u32(node, "nr_wins", &ctx->nr_wins);
	DRM_INFO("nr_wins(%d)\n", ctx->nr_wins);

	of_property_read_u32_array(node, "connected_idmas",
			res, ctx->nr_wins);
	for (i = 0; i < ctx->nr_wins; ++i) {
		ctx->planes[i].win_idx = i;
		ctx->planes[i].idma_idx = res[i];
		DRM_INFO("idma(%d) --> window(%d)\n", ctx->planes[i].idma_idx,
				ctx->planes[i].win_idx);
	}

	of_property_read_u32(node, "psr_mode", &ctx->psr.psr_mode);
	of_property_read_u32(node, "trig_mode", &ctx->psr.trig_mode);
	of_property_read_u32(node, "dsi_mode", &ctx->psr.dsi_mode);
	of_property_read_u32(node, "out_type", &ctx->psr.out_type);
	of_property_read_u32(node, "out_idx", &ctx->psr.out_idx);
	DRM_INFO("PSR Info: psr(%d) trig(%d) dsi(%d) out(%d)\n",
				ctx->psr.psr_mode, ctx->psr.trig_mode,
				ctx->psr.dsi_mode, ctx->psr.out_type);
}

static int exynos_decon_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id;
	struct device *dev = &pdev->dev;
	struct decon_context *ctx;
	struct resource *res;
	int ret = 0;
	int i;

	DRM_INFO("%s +\n", __func__);

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = dev;

	exynos_decon_parse_dt(ctx);

	__set_bit(BIT_SUSPENDED, &ctx->flags);

	of_id = of_match_device(exynos_decon_driver_dt_match, &pdev->dev);
	exynos_decon_get_clk_count(ctx);

	if (ctx->id != 2) {
		for (i = 0; i < ctx->clk_cnt; i++) {
			struct clk *clk;

			clk = devm_clk_get(ctx->dev, decon_clks_name[i]);
			if (IS_ERR(clk))
				return PTR_ERR(clk);

			ctx->clks[i] = clk;
		}
	} else {
		for (i = 0; i < ctx->clk_cnt; i++) {
			struct clk *clk;

			clk = devm_clk_get(ctx->dev, decon_t_clks_name[i]);
			if (IS_ERR(clk))
				return PTR_ERR(clk);

			ctx->clks[i] = clk;
		}
	}

	decon_drvdata[ctx->id] = ctx;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "cannot find IO resource\n");
		return -ENXIO;
	}

	ret = exynos_decon_get_disp_ss_addr(ctx);
	if (ret)
		return ret;

	ctx->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(ctx->regs)) {
		dev_err(dev, "ioremap failed\n");
		return PTR_ERR(ctx->regs);
	}

	/* FOR VPP, VG0 default setting */
	if (ctx->id != 2) {
		res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "vsync");
		if (!res) {
			dev_err(dev, "cannot find IRQ resource\n");
			return -ENXIO;
		}

		ret = devm_request_irq(dev, res->start, decon_irq_handler, 0,
				"drm_decon", ctx);
		if (ret < 0) {
			dev_err(dev, "lcd_sys irq request failed\n");
			return ret;
		}

		ctx->irq = res->start;
		disable_irq(ctx->irq);
	} else {
		ctx->cmu_regs = ioremap(0x13F00000, 0x1000);	//CMU_DISP1_BASE

		/* 1: VStatus irq */
		res = platform_get_resource(pdev, IORESOURCE_IRQ, 1);
		ret = devm_request_irq(dev, res->start, decon_irq_handler, 0,
				"drm_decon", ctx);
		if (ret) {
			dev_err(dev, "failed to install irq\n");
			return ret;
		}
		ctx->irq = res->start;
		disable_irq(ctx->irq);

		/* 2: FrameDone irq */
		res = platform_get_resource(pdev, IORESOURCE_IRQ, 2);
		ret = devm_request_irq(dev, res->start, decon_irq_handler, 0,
				"drm_decon", ctx);
		if (ret < 0) {
			dev_err(dev, "lcd_sys irq request failed\n");
			return ret;
		}

		/* 3: Extra Interrupts: Resource Conflict irq */
		res = platform_get_resource(pdev, IORESOURCE_IRQ, 3);
		ret = devm_request_irq(dev, res->start, decon_irq_handler, 0,
				"drm_decon", ctx);
		if (ret) {
			dev_err(dev, "failed to install irq\n");
			return ret;
		}
	}

	set_bit(BIT_CLKS_ENABLED, &ctx->flags);

	if (ctx->id != 2) {
		ret = exynos_decon_get_pinctrl(ctx);
		if (ret)
			return ret;
	}
	platform_set_drvdata(pdev, ctx);

	pm_runtime_enable(dev);

	ret = component_add(dev, &decon_component_ops);
	if (ret)
		goto err_disable_pm_runtime;

	DRM_INFO("%s -\n", __func__);

	return 0;

err_disable_pm_runtime:
	pm_runtime_disable(dev);
	return ret;
}

static int exynos_decon_remove(struct platform_device *pdev)
{
	DRM_DEBUG_KMS("%s +\n", __func__);
	pm_runtime_disable(&pdev->dev);

	component_del(&pdev->dev, &decon_component_ops);

	DRM_DEBUG_KMS("%s -\n", __func__);
	return 0;
}

struct platform_driver exynos_decon_driver = {
	.probe		= exynos_decon_probe,
	.remove		= exynos_decon_remove,
	.driver		= {
		.name	= "exynos8-decon",
		.pm	= &exynos_decon_pm_ops,
		.of_match_table = exynos_decon_driver_dt_match,
	},
};
