/*
 * Samsung Exynos SoC series am-cam driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef FIMC_IS_CSI_H_
#define FIMC_IS_CSI_H_

#include <linux/interrupt.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "fimc-is-type.h"

#define CSI_VIRTUAL_CH_0	0
#define CSI_VIRTUAL_CH_1	1
#define CSI_VIRTUAL_CH_2	2
#define CSI_VIRTUAL_CH_3	3
#define CSI_VIRTUAL_CH_MAX	4

#define CSI_DATA_LANES_1	0
#define CSI_DATA_LANES_2	1
#define CSI_DATA_LANES_3	2
#define CSI_DATA_LANES_4	3

#define CSI_MODE_CH0_ONLY	0
#define CSI_MODE_DT_ONLY	1
#define CSI_MODE_VC_ONLY	2
#define CSI_MODE_VC_DT		3

#define HW_FORMAT_YUV420_8BIT	0x18
#define HW_FORMAT_YUV420_10BIT	0x19
#define HW_FORMAT_YUV422_8BIT	0x1E
#define HW_FORMAT_YUV422_10BIT	0x1F
#define HW_FORMAT_RGB565	0x22
#define HW_FORMAT_RGB666	0x23
#define HW_FORMAT_RGB888	0x24
#define HW_FORMAT_RAW6		0x28
#define HW_FORMAT_RAW7		0x29
#define HW_FORMAT_RAW8		0x2A
#define HW_FORMAT_RAW10 	0x2B
#define HW_FORMAT_RAW12 	0x2C
#define HW_FORMAT_RAW14 	0x2D
#define HW_FORMAT_USER		0x30
#define HW_FORMAT_UNKNOWN	0x3F

/*
 * This enum will be used for masking each interrupt masking.
 * The irq_ids params which masked by shifting this bit(id)
 * was sended to csi_hw_irq_msk.
 */
enum csis_hw_irq_id {
	CSIS_IRQ_ID			= 0,
	CSIS_IRQ_CRC			= 1,
	CSIS_IRQ_ECC			= 2,
	CSIS_IRQ_WRONG_CFG		= 3,
	CSIS_IRQ_OVERFLOW_VC		= 4,
	CSIS_IRQ_LOST_FE_VC		= 5,
	CSIS_IRQ_LOST_FS_VC		= 6,
	CSIS_IRQ_SOT_VC			= 7,
	CSIS_IRQ_FRAME_END_VC		= 8,
	CSIS_IRQ_FRAME_START_VC		= 9,
	CSIS_IRQ_LINE_END_VC		= 10,
	CSIS_IRQ_DMA_FRM_START_VC	= 11,
	CSIS_IRQ_DMA_FRM_END_VC		= 12,
	CSIS_IRQ_ABORT_ERROR		= 13,
	CSIS_IRQ_ABORT_DONE		= 14,
	CSIS_IRQ_OTF_OVERLAP		= 15,
	CSIS_IRQ_END,
};

/*
 * This enum will be used for current error status by reading interrupt source.
 */
enum csis_hw_err_id {
	CSIS_ERR_ID = 0,
	CSIS_ERR_CRC = 1,
	CSIS_ERR_ECC = 2,
	CSIS_ERR_WRONG_CFG = 3,
	CSIS_ERR_OVERFLOW_VC = 4,
	CSIS_ERR_LOST_FE_VC = 5,
	CSIS_ERR_LOST_FS_VC = 6,
	CSIS_ERR_SOT_VC = 7,
	CSIS_ERR_OTF_OVERLAP = 8,
	CSIS_ERR_DMA_ERR_DMAFIFO_FULL = 9,
	CSIS_ERR_DMA_ERR_TRXFIFO_FULL = 10,
	CSIS_ERR_DMA_ERR_BRESP_ERR = 11,
	CSIS_ERR_INVALID_CODE_HS = 12,
	CSIS_ERR_SOT_SYNC_HS = 13,
	CSIS_ERR_MAL_CRC = 14,
	CSIS_ERR_DMA_ABORT_DONE = 15,
	CSIS_ERR_END
};

/*
 * This enum will be used in csi_hw_s_control api to set specific functions.
 */
enum csis_hw_control_id {
	CSIS_CTRL_INTERLEAVE_MODE,
	CSIS_CTRL_LINE_RATIO,
	CSIS_CTRL_BUS_WIDTH,
	CSIS_CTRL_DMA_ABORT_REQ,
	CSIS_CTRL_ENABLE_LINE_IRQ,
};

enum fimc_is_csi_state {
	CSIS_START_STREAM
};

#define EXPECT_FRAME_START	0
#define EXPECT_FRAME_END	1

struct fimc_is_sensor_cfg {
	u32 width;
	u32 height;
	u32 framerate;
	u32 settle;
	int mode;
	u32 lanes;
	u32 mipi_speed;
};

struct fimc_is_device_csi {
	struct platform_device		*pdev;
	void __iomem			*regs;
	resource_size_t 		regs_size;
	int				irq;

	/* for vci setting */
	u32				active_vci;
	u32				vcis;
	struct fimc_is_vci		*vci;

	/* image configuration */
	u32				mode;
	u32				lanes;
	u32				mipi_speed;

	struct fimc_is_sensor_cfg	cfg;
	struct fimc_is_image		image;

	unsigned long			state;

	/* for DMA feature */
	struct am_cam_framemgr		*framemgr;
	struct tasklet_struct		tasklet_csis_str;
	struct tasklet_struct		tasklet_csis_end;

	/* pointer address from device sensor */
	struct v4l2_subdev		**subdev;
	struct phy			*phy;
};

int __must_check fimc_is_csi_probe(void *parent);
int __must_check fimc_is_csi_open(struct v4l2_subdev *subdev, struct am_cam_framemgr *framemgr);
int __must_check fimc_is_csi_close(struct v4l2_subdev *subdev);

int csi_s_power(struct v4l2_subdev *subdev, int on);
int csi_s_stream(struct v4l2_subdev *subdev, int enable);
int csi_s_buffer(struct v4l2_subdev *subdev, void *buf, unsigned int *size);
int csi_s_buffer_addr(struct v4l2_subdev *subdev, void *buf, unsigned int *size);
int csi_s_buffer_finish(struct v4l2_subdev *subdev, void *buf, unsigned int *size);
void csi_dma_set_buffer_start(struct v4l2_subdev *subdev);

#endif
