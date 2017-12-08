/* linux/drivers/gpu/drm/exynos-hq/exynos_drm_vpp.c
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Samsung Exynos VPP driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/spinlock.h>
#include <drm/drmP.h>

#include "exynos_drm_vpp.h"
#include "exynos_drm_gem.h"

struct exynos_vpp *vpp_drvdata[MAX_VPP_CNT];

void exynos_vpp_dump(struct exynos_vpp *vpp)
{
	DRM_INFO("=== VPP%d SFR DUMP ===\n", vpp->id);

	print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 32, 4,
			vpp->res.regs, 0x90, false);
	print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 32, 4,
			vpp->res.regs + 0xA00, 0x8, false);
	print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 32, 4,
			vpp->res.regs + 0xA48, 0x10, false);
	print_hex_dump(KERN_INFO, "", DUMP_PREFIX_ADDRESS, 32, 4,
			vpp->res.regs + 0xB00, 0x100, false);
}

static int exynos_vpp_check_scale_ratio(struct vpp_params_info *p)
{
	u32 sc_up_max_w, sc_up_max_h;
	u32 sc_down_min_w, sc_down_min_h;

	if (p->is_rot) {
		sc_up_max_w = p->dst.h << 1;
		sc_up_max_h = p->dst.w << 1;
		sc_down_min_w = p->src.h >> 3;
		sc_down_min_h = p->src.w >> 3;
	} else {
		sc_up_max_w = p->dst.w << 1;
		sc_up_max_h = p->dst.h << 1;
		sc_down_min_w = p->src.w >> 3;
		sc_down_min_h = p->src.h >> 3;
	}

	if (p->src.w > sc_up_max_w || p->src.h > sc_up_max_h) {
		DRM_ERROR("unsupported max(2x) up scale ratio\n");
		goto err;
	}

	if (p->dst.w < sc_down_min_w || p->dst.h < sc_down_min_h) {
		DRM_ERROR("unsupported min(1/8x) down scale ratio\n");
		goto err;
	}

	return 0;
err:
	DRM_ERROR("src w(%d) h(%d), dst w(%d) h(%d), rot(%d)\n",
			p->src.w, p->src.h, p->dst.w, p->dst.h, p->is_rot);
	return -EINVAL;
}

static int exynos_vpp_check_format(struct exynos_vpp *vpp,
		struct vpp_params_info *p)
{
	if ((vpp->id == 0 || vpp->id == 1 || vpp->id == 4 || vpp->id == 5) &&
			(p->format >= DECON_PIXEL_FORMAT_NV16)) {
		DRM_ERROR("Unsupported YUV format%d in G%d \n",
				p->format, vpp->id);
		return -EINVAL;
	}

	if (vpp->id != 6 && vpp->id != 7) { /* !VGR case */
		if (p->is_comp) {
			DRM_ERROR("Unsupported AFBC format decoding in DPP%d\n",
					vpp->id);
			return -EINVAL;
		}

		if ((p->is_scale) || (p->is_rot)) {
			DRM_ERROR("Unsupported Scailing in vpp%d, s(%d), r(%d)\n",
					vpp->id, p->is_scale, p->is_rot);
			return -EINVAL;
		}
	}

	switch (p->format) {
	case DECON_PIXEL_FORMAT_ARGB_8888:
	case DECON_PIXEL_FORMAT_ABGR_8888:
	case DECON_PIXEL_FORMAT_RGBA_8888:
	case DECON_PIXEL_FORMAT_BGRA_8888:
	case DECON_PIXEL_FORMAT_XRGB_8888:
	case DECON_PIXEL_FORMAT_XBGR_8888:
	case DECON_PIXEL_FORMAT_RGBX_8888:
	case DECON_PIXEL_FORMAT_BGRX_8888:
	case DECON_PIXEL_FORMAT_RGB_565:
	case DECON_PIXEL_FORMAT_NV16:
	case DECON_PIXEL_FORMAT_NV61:
	case DECON_PIXEL_FORMAT_NV12:
	case DECON_PIXEL_FORMAT_NV12M:
	case DECON_PIXEL_FORMAT_NV21:
	case DECON_PIXEL_FORMAT_NV21M:
	case DECON_PIXEL_FORMAT_NV12N:
		break;
	default:
		DRM_ERROR("Unsupported Format\n");
		return -EINVAL;
	}

	return 0;
}

static int exynos_vpp_check_size(struct vpp_params_info *p,
		struct vpp_img_format *vi)
{
	struct decon_frame *src = &p->src;
	struct decon_frame *dst = &p->dst;
	struct vpp_size_constraints vc;

	vpp_constraints_params(&vc, vi);

	if ((!check_align(src->x, src->y, vc.src_mul_x, vc.src_mul_y)) ||
	   (!check_align(src->f_w, src->f_h, vc.src_mul_w, vc.src_mul_h)) ||
	   (!check_align(src->w, src->h, vc.img_mul_w, vc.img_mul_h)) ||
	   (!check_align(dst->w, dst->h, vc.sca_mul_w, vc.sca_mul_h))) {
		DRM_ERROR("Alignment error\n");
		goto err;
	}

	if (src->w > vc.src_w_max || src->w < vc.src_w_min ||
		src->h > vc.src_h_max || src->h < vc.src_h_min) {
		DRM_ERROR("Unsupported source size\n");
		goto err;
	}

	if (dst->w > vc.sca_w_max || dst->w < vc.sca_w_min ||
		dst->h > vc.sca_h_max || dst->h < vc.sca_h_min) {
		DRM_ERROR("Unsupported dest size\n");
		goto err;
	}

	return 0;
err:
	DRM_ERROR("offset x : %d, offset y: %d\n", src->x, src->y);
	DRM_ERROR("src_mul_x : %d, src_mul_y : %d\n", vc.src_mul_x, vc.src_mul_y);
	DRM_ERROR("src f_w : %d, src f_h: %d\n", src->f_w, src->f_h);
	DRM_ERROR("src_mul_w : %d, src_mul_h : %d\n", vc.src_mul_w, vc.src_mul_h);
	DRM_ERROR("src w : %d, src h: %d\n", src->w, src->h);
	DRM_ERROR("img_mul_w : %d, img_mul_h : %d\n", vc.img_mul_w, vc.img_mul_h);
	DRM_ERROR("dst w : %d, dst h: %d\n", dst->w, dst->h);
	DRM_ERROR("sca_mul_w : %d, sca_mul_h : %d\n", vc.sca_mul_w, vc.sca_mul_h);
	DRM_ERROR("rotation : %d, color_format : %d\n",
				p->is_rot, p->format);

	return -EINVAL;
}

static void exynos_vpp_get_prop(struct exynos_vpp *vpp,
		struct vpp_img_format *vi, struct vpp_params_info *p)
{
	vi->vgr = is_vgr(vpp);
	vi->normal = VPP_ROT_NORMAL;
	vi->rot = p->is_rot;
	vi->scale = p->is_scale;
	vi->format = p->format;
	vi->afbc_en = p->is_comp;
	vi->yuv = is_yuv(p->format);
	vi->yuv422 = is_yuv422(p->format);
	vi->yuv420 = is_yuv420(p->format);
	vi->wb = false;
}

static int exynos_vpp_check_limitation(struct exynos_vpp *vpp,
		struct vpp_params_info *p)
{
	int ret;
	struct vpp_img_format vi;

	ret = exynos_vpp_check_scale_ratio(p);
	if (ret) {
		DRM_ERROR("failed to set vpp%d scale information\n", vpp->id);
		return -EINVAL;
	}

	exynos_vpp_get_prop(vpp, &vi, p);

	ret = exynos_vpp_check_format(vpp, p);
	if (ret)
		return -EINVAL;

	if (p->is_comp && p->is_rot) {
		DRM_ERROR("rotation is not supported in case of compression");
		return -EINVAL;
	}

	ret = exynos_vpp_check_size(p, &vi);
	if (ret)
		return -EINVAL;

	if (!is_vgr(vpp) && (p->rot > VPP_ROT_NORMAL)) {
		DRM_ERROR("vpp-%d can't rotate\n", vpp->id);
		return -EINVAL;
	}

	return 0;
}

static void exynos_vpp_print_params(struct vpp_params_info *vpp_params)
{
	DRM_DEBUG_KMS("vpp params info\n");
	DRM_DEBUG_KMS("src: x(%d), y(%d), w(%d), h(%d), f_w(%d), f_h(%d)\n",
			vpp_params->src.x, vpp_params->src.y,
			vpp_params->src.w, vpp_params->src.h,
			vpp_params->src.f_w, vpp_params->src.f_h);
	DRM_DEBUG_KMS("dst: x(%d), y(%d), w(%d), h(%d), f_w(%d), f_h(%d)\n",
			vpp_params->dst.x, vpp_params->dst.y,
			vpp_params->dst.w, vpp_params->dst.h,
			vpp_params->dst.f_w, vpp_params->dst.f_h);
	DRM_DEBUG_KMS("scale(%d), h ratio(%d), v ratio(%d)\n",
			vpp_params->is_scale,
			vpp_params->h_ratio, vpp_params->v_ratio);
	DRM_DEBUG_KMS("format(%d), DV addr(0x%llx)\n",
			vpp_params->format, vpp_params->addr[0]);
}

static int exynos_vpp_runtime_suspend(struct device *dev)
{
	struct exynos_vpp *vpp = dev_get_drvdata(dev);

	DRM_DEBUG_KMS("%s: vpp(%d) +\n", __func__, vpp->id);
	clk_disable_unprepare(vpp->res.gate);
	DRM_DEBUG_KMS("%s -\n", __func__);

	return 0;
}

static int exynos_vpp_runtime_resume(struct device *dev)
{
	struct exynos_vpp *vpp = dev_get_drvdata(dev);
	int ret;

	DRM_DEBUG_KMS("%s: vpp(%d) +\n", __func__, vpp->id);

	ret = clk_prepare_enable(vpp->res.gate);
	if (ret) {
		DRM_ERROR("failed to enable vpp%d clock\n", vpp->id);
		return ret;
	}
	DRM_DEBUG_KMS("%s -\n", __func__);

	return 0;
}

static const struct dev_pm_ops exynos_vpp_pm_ops = {
	.runtime_suspend	= exynos_vpp_runtime_suspend,
	.runtime_resume		= exynos_vpp_runtime_resume,
};

int exynos_vpp_set_config(int id, struct vpp_params_info *vpp_params)
{
	struct exynos_vpp *vpp = get_vpp_drvdata(id);
	int ret = 0;

	mutex_lock(&vpp->lock);

	if (vpp->state == VPP_STATE_OFF) {
#if defined(CONFIG_PM_RUNTIME)
		pm_runtime_get_sync(vpp->dev);
#else
		exynos_vpp_runtime_resume(vpp->dev);
#endif
		vpp_reg_init(id);
		enable_irq(vpp->res.irq);
	}

	ret = exynos_vpp_check_limitation(vpp, vpp_params);
	if (ret)
		goto err;

	vpp_reg_configure_params(id, vpp_params);

	exynos_vpp_print_params(vpp_params);

	vpp->state = VPP_STATE_ON;

err:
	mutex_unlock(&vpp->lock);
	return ret;
}

int exynos_vpp_stop(int id)
{
	struct exynos_vpp *vpp = get_vpp_drvdata(id);
	int ret = 0;

	mutex_lock(&vpp->lock);

	if (vpp->state == VPP_STATE_OFF) {
		DRM_INFO("vpp%d is already disabled\n", vpp->id);
		goto err;
	}

	disable_irq(vpp->res.irq);
	vpp_reg_deinit(vpp->id, false);

#if defined(CONFIG_PM_RUNTIME)
	pm_runtime_put_sync(vpp->dev);
#else
	exynos_vpp_runtime_suspend(vpp->dev);
#endif
	DRM_DEBUG_KMS("vpp%d is stopped\n", vpp->id);

	vpp->state = VPP_STATE_OFF;
err:
	mutex_unlock(&vpp->lock);
	return ret;

}

static irqreturn_t exynos_vpp_irq_handler(int irq, void *priv)
{
	struct exynos_vpp *vpp = priv;
	u32 vpp_irq = 0;

	spin_lock(&vpp->slock);
	if (vpp->state == VPP_STATE_OFF)
		goto end;

	vpp_irq = vpp_reg_get_irq_status(vpp->id);
	vpp_reg_clear_irq(vpp->id, vpp_irq);

end:
	spin_unlock(&vpp->slock);
	return IRQ_HANDLED;
}

static const struct of_device_id exynos_vpp_driver_dt_match[] = {
	{
		.compatible = "samsung,exynos8-vpp",
	},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_vpp_driver_dt_match);

static int exynos_vpp_bind(struct device *dev, struct device *master,
				void *data)
{
	struct drm_device *drm_dev = data;

	DRM_INFO("%s +\n", __func__);

	return exynos_drm_gem_init_iommu(drm_dev, dev);
}

static void exynos_vpp_unbind(struct device *dev, struct device *master,
				void *data)
{
	struct drm_device *drm_dev = data;

	DRM_DEBUG_KMS("%s +\n", __func__);

	exynos_drm_gem_deinit_iommu(drm_dev, dev);

	DRM_DEBUG_KMS("%s -\n", __func__);
}

static const struct component_ops exynos_vpp_component_ops = {
	.bind	= exynos_vpp_bind,
	.unbind	= exynos_vpp_unbind,
};

static int exynos_vpp_probe(struct platform_device *pdev)
{
	struct exynos_vpp *vpp;
	struct device *dev = &pdev->dev;
	struct resource *res;
	const struct of_device_id *of_id;
	int ret;

	DRM_DEBUG_KMS("%s +\n", __func__);

	vpp = devm_kzalloc(dev, sizeof(*vpp), GFP_KERNEL);
	if (!vpp)
		return -ENOMEM;

	vpp->id = of_alias_get_id(dev->of_node, "vpp");
	DRM_INFO("vpp(%d) probe start..\n", vpp->id);

	vpp->dev = dev;
	vpp_drvdata[vpp->id] = vpp;

	spin_lock_init(&vpp->slock);
	mutex_init(&vpp->lock);

	of_id = of_match_device(exynos_vpp_driver_dt_match, &pdev->dev);

	vpp->res.gate = devm_clk_get(dev, "vpp_clk");
	if (IS_ERR_OR_NULL(vpp->res.gate)) {
		DRM_ERROR("failed to get vpp%d clock\n", vpp->id);
		return PTR_ERR(vpp->res.gate);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		DRM_ERROR("cannot find IO resource\n");
		return -ENXIO;
	}

	DRM_INFO("res: start(0x%x), end(0x%x)\n",
			(u32)res->start, (u32)res->end);

	vpp->res.regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(vpp->res.regs)) {
		DRM_ERROR("ioremap failed\n");
		return PTR_ERR(vpp->res.regs);
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		DRM_ERROR("failed to get irq resource\n");
		return -ENOENT;
	}
	DRM_INFO("irq no = %lld\n", res->start);

	vpp->res.irq = res->start;
	ret = devm_request_irq(dev, res->start, exynos_vpp_irq_handler, 0,
			pdev->name, vpp);
	if (ret) {
		DRM_ERROR("failed to install VPP%d irq\n", vpp->id);
		return -EINVAL;
	}
	disable_irq(vpp->res.irq);

	platform_set_drvdata(pdev, vpp);
	pm_runtime_enable(dev);

	vpp->state = VPP_STATE_OFF;

	DRM_INFO("vpp%d is probed successfully\n", vpp->id);

	return component_add(dev, &exynos_vpp_component_ops);
}

static int exynos_vpp_remove(struct platform_device *pdev)
{
	DRM_DEBUG_KMS("%s +\n", __func__);

	pm_runtime_disable(&pdev->dev);
	component_del(&pdev->dev, &exynos_vpp_component_ops);

	return 0;
}

struct platform_driver exynos_vpp_driver = {
	.probe		= exynos_vpp_probe,
	.remove		= exynos_vpp_remove,
	.driver		= {
		.name	= "exynos8-vpp",
		.pm	= &exynos_vpp_pm_ops,
		.of_match_table = exynos_vpp_driver_dt_match,
	},
};
