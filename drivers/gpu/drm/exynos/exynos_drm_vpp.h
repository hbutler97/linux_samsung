/* linux/drivers/gpu/drm/exynos-hq/vpp_core.h
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * header file for Samsung EXYNOS5 SoC series VPP driver

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef VPP_H_
#define VPP_H_

#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/smc.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>
#include <linux/pm_qos.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/exynos_iovmm.h>
#include <linux/mutex.h>
#include <media/exynos_mc.h>
#include <soc/samsung/bts.h>

#include "exynos_drm_decon.h"
#include "regs-vpp.h"

#define MAX_VPP_CNT	8

#define YUV_SRC_OFFSET_MULTIPLE	2
#define YUV_SRC_SIZE_MULTIPLE	2
#define YUV_SRC_SIZE_MUL_HEIGHT	1
#define YUV_SRC_WIDTH_MAX	65534
#define YUV_SRC_HEIGHT_MAX	8190
#define YUV_SRC_WIDTH_MIN	32
#define YUV_SRC_HEIGHT_MIN	16
#define RGB_SRC_OFFSET_MULTIPLE	1
#define RGB_SRC_SIZE_MULTIPLE	1
#define RGB_SRC_WIDTH_MAX	65535
#define RGB_SRC_HEIGHT_MAX	8191
#define RGB_SRC_WIDTH_MIN	16
#define RGB_SRC_HEIGHT_MIN	8

#define ROT1_RGB_SRC_WIDTH_MIN	32
#define ROT1_RGB_SRC_HEIGHT_MIN	16
#define ROT1_YUV_SRC_WIDTH_MIN	64
#define ROT1_YUV_SRC_HEIGHT_MIN	32
#define ROT2_RGB_SRC_WIDTH_MIN	16
#define ROT2_RGB_SRC_HEIGHT_MIN	32
#define ROT2_YUV_SRC_WIDTH_MIN	32
#define ROT2_YUV_SRC_HEIGHT_MIN	64

#define YUV_IMG_SIZE_MULTIPLE	2
#define RGB_IMG_SIZE_MULTIPLE	1
#define IMG_WIDTH_MAX		4096
#define IMG_HEIGHT_MAX		4096
#define RGB_IMG_WIDTH_MIN	16
#define RGB_IMG_HEIGHT_MIN	8
#define YUV_IMG_WIDTH_MIN	32
#define YUV_IMG_HEIGHT_MIN	16

#define IMG_SIZE_MULTIPLE	2

#define ROT1_RGB_IMG_WIDTH_MIN	32
#define ROT1_RGB_IMG_HEIGHT_MIN	16
#define ROT1_YUV_IMG_WIDTH_MIN	64
#define ROT1_YUV_IMG_HEIGHT_MIN	32
#define ROT2_RGB_IMG_WIDTH_MIN	16
#define ROT2_RGB_IMG_HEIGHT_MIN	32
#define ROT2_YUV_IMG_WIDTH_MIN	32
#define ROT2_YUV_IMG_HEIGHT_MIN	64

#define ROT3_RGB_IMG_WIDTH_MAX	4096
#define ROT3_RGB_IMG_HEIGHT_MAX	4096
#define ROT3_YUV_IMG_WIDTH_MAX	4096
#define ROT3_YUV_IMG_HEIGHT_MAX	4096
#define ROT4_RGB_IMG_WIDTH_MAX	2048
#define ROT4_RGB_IMG_HEIGHT_MAX	2048
#define ROT4_YUV_IMG_WIDTH_MAX	2048
#define ROT4_YUV_IMG_HEIGHT_MAX	2048

#define BLK_WIDTH_MAX		4096
#define BLK_HEIGHT_MAX		4096
#define BLK_WIDTH_MIN		144
#define BLK_HEIGHT_MIN		10
#define ROT3_RGB_BLK_WIDTH_MAX	4096
#define ROT3_RGB_BLK_HEIGHT_MAX	4096
#define ROT3_RGB_BLK_WIDTH_MIN	128
#define ROT3_RGB_BLK_HEIGHT_MIN	16
#define ROT4_RGB_BLK_WIDTH_MAX	2048
#define ROT4_RGB_BLK_HEIGHT_MAX	2048

#define SCALED_SIZE_MULTIPLE	1
#define SCALED_WIDTH_MAX	4096
#define SCALED_HEIGHT_MAX	4096
#define SCALED_WIDTH_MIN	16
#define SCALED_HEIGHT_MIN	8

#define PRE_RGB_WIDTH		1
#define PRE_RGB_HEIGHT		1
#define PRE_YUV_WIDTH		2
#define PRE_YUV_HEIGHT		2
#define PRE_ROT1_YUV_HEIGHT	1

#define SRC_SIZE_MULTIPLE	2
#define SRC_ROT1_MUL_Y		1
#define SRC_ROT2_MUL_Y		2

#define DST_SIZE_MULTIPLE	1
#define DST_SIZE_WIDTH_MIN	16
#define DST_SIZE_WIDTH_MAX	8191
#define DST_SIZE_HEIGHT_MIN	8
#define DST_SIZE_HEIGHT_MAX	8191
#define DST_OFFSET_MULTIPLE	1
#define DST_IMGAGE_MULTIPLE	1
#define DST_IMG_WIDTH_MIN	16
#define DST_IMG_HEIGHT_MIN	8
#define DST_IMG_MAX		4096

#define check_align(width, height, align_w, align_h)\
	(IS_ALIGNED(width, align_w) && IS_ALIGNED(height, align_h))
#define is_vgr(vpp) ((vpp->id == 6) || (vpp->id == 7))
#define is_yuv(format) ((format >= DECON_PIXEL_FORMAT_NV16) \
			&& (format < DECON_PIXEL_FORMAT_MAX))
#define is_yuv422(format) ((format >= DECON_PIXEL_FORMAT_NV16) \
			&& (format <= DECON_PIXEL_FORMAT_YVU422_3P))
#define is_yuv420(format) ((format >= DECON_PIXEL_FORMAT_NV12) \
			&& (format <= DECON_PIXEL_FORMAT_YVU420M))

enum vpp_state {
	VPP_STATE_ON,
	VPP_STATE_OFF,
};

struct vpp_size_constraints {
	u32		src_mul_w;
	u32		src_mul_h;
	u32		src_w_min;
	u32		src_w_max;
	u32		src_h_min;
	u32		src_h_max;
	u32		img_mul_w;
	u32		img_mul_h;
	u32		img_w_min;
	u32		img_w_max;
	u32		img_h_min;
	u32		img_h_max;
	u32		blk_w_min;
	u32		blk_w_max;
	u32		blk_h_min;
	u32		blk_h_max;
	u32		blk_mul_w;
	u32		blk_mul_h;
	u32		src_mul_x;
	u32		src_mul_y;
	u32		sca_w_min;
	u32		sca_w_max;
	u32		sca_h_min;
	u32		sca_h_max;
	u32		sca_mul_w;
	u32		sca_mul_h;
	u32		dst_mul_w;
	u32		dst_mul_h;
	u32		dst_w_min;
	u32		dst_w_max;
	u32		dst_h_min;
	u32		dst_h_max;
	u32		dst_mul_x;
	u32		dst_mul_y;
};

struct vpp_img_format {
	u32		vgr;
	u32		normal;
	u32		rot;
	u32		scale;
	u32		format;
	u32		afbc_en;
	u32		protection;
	u32		yuv;
	u32		yuv422;
	u32		yuv420;
	u32		wb;
};

struct vpp_size_param {
	u32		src_x;
	u32		src_y;
	u32		src_fw;
	u32		src_fh;
	u32		src_w;
	u32		src_h;
	u32		dst_w;
	u32		dst_h;
	u32		fr_w;
	u32		fr_h;
	u32		fr_yx;
	u32		fr_yy;
	u32		fr_cx;
	u32		fr_cy;
	u32		vpp_h_ratio;
	u32		vpp_v_ratio;
	u32		rot;
	u32		block_w;
	u32		block_h;
	u32		block_x;
	u32		block_y;
	u64		phys_addr0;
	u64		phys_addr1;
	u64		phys_addr2;
	u32		phys_addr_len0;
	u32		phys_addr_len1;
	u32		phys_addr_len2;
	u64		addr0;
	u64		addr1;
	u64		addr2;
};

struct vpp_resources {
	struct clk *gate;
};

struct vpp_fraction {
	u32 y_x;
	u32 y_y;
	u32 c_x;
	u32 c_y;
	u32 w;
	u32 h;
};

struct vpp_minlock_entry {
	bool rotation;
	u32 scalefactor;
	u32 dvfsfreq;
};

struct vpp_minlock_table {
	struct list_head node;
	u16 width;
	u16 height;
	int num_entries;
	struct vpp_minlock_entry entries[0];
};

struct vpp_phys_addr {
	unsigned long phy_addr[MAX_BUF_PLANE_CNT];
	unsigned int phy_addr_len[MAX_BUF_PLANE_CNT];
};

struct vpp_params_info {
	struct decon_frame src;
	struct decon_frame dst;
	struct decon_win_rect block;
	bool is_rot;
	bool is_comp;
	bool is_scale;
	bool is_block;
	enum decon_pixel_format format;
	enum vpp_rotate rot;
	dma_addr_t addr[3];
	enum vpp_csc_eq eq_mode;
	int h_ratio;
	int v_ratio;
};

struct exynos_vpp_resource {
	struct clk *gate;
	void __iomem *regs;
	int irq;
};

struct exynos_vpp {
	int id;
	spinlock_t slock;
	struct mutex lock;
	enum vpp_state state;
	struct device *dev;
	struct exynos_vpp_resource res;
};

extern struct exynos_vpp *vpp_drvdata[MAX_VPP_CNT];

static inline struct exynos_vpp *get_vpp_drvdata(u32 id)
{
	if (id < 0 || id > MAX_VPP_CNT)
		return NULL;
	else
		return vpp_drvdata[id];
}

static inline u32 vpp_read(u32 id, u32 reg_id)
{
	struct exynos_vpp *vpp = get_vpp_drvdata(id);
	return readl(vpp->res.regs + reg_id);
}

static inline u32 vpp_read_mask(u32 id, u32 reg_id, u32 mask)
{
	u32 val = vpp_read(id, reg_id);
	val &= (~mask);
	return val;
}

static inline void vpp_write(u32 id, u32 reg_id, u32 val)
{
	struct exynos_vpp *vpp = get_vpp_drvdata(id);
	writel(val, vpp->res.regs + reg_id);
}

static inline void vpp_write_mask(u32 id, u32 reg_id, u32 val, u32 mask)
{
	u32 old = vpp_read(id, reg_id);

	val = (val & mask) | (old & ~mask);
	vpp_write(id, reg_id, val);
}

void vpp_reg_init(u32 id);
void vpp_reg_configure_params(u32 id, struct vpp_params_info *p);
u32 vpp_reg_get_irq_status(u32 id);
void vpp_reg_clear_irq(u32 id, u32 irq);
void vpp_constraints_params(struct vpp_size_constraints *vc,
		struct vpp_img_format *vi);
int vpp_reg_deinit(u32 id, bool reset);

int exynos_vpp_set_config(int id, struct vpp_params_info *vpp_params);

#endif
