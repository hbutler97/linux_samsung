/*
 * Samsung Exynos SoC series am-cam driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef AM_CAM_VIDEO_H_
#define AM_CAM_VIDEO_H_

#include <linux/version.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-dev.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-ion.h>

#define EXYNOS_VIDEONODE_MODULE_AM_CAM	200
#define AM_CAM_MAX_INSTANCES		10
#define AM_CAM_MAX_BUFS			16
#define AM_CAM_MAX_PLANES		4

enum am_cam_video_state {
	AM_CAM_VIDEO_CLOSE,
	AM_CAM_VIDEO_OPEN,
	AM_CAM_VIDEO_S_INPUT,
	AM_CAM_VIDEO_S_FORMAT,
	AM_CAM_VIDEO_S_BUFS,
	AM_CAM_VIDEO_STOP,
	AM_CAM_VIDEO_START,
};

struct am_cam_video;

struct am_cam_queue {
	struct vb2_queue		*vbq;

	u32				buf_maxcount;
	u32				buf_rdycount;
	u32				buf_refcount;
	ulong				buf_dva[AM_CAM_MAX_BUFS][AM_CAM_MAX_PLANES];
	ulong				buf_kva[AM_CAM_MAX_BUFS][AM_CAM_MAX_PLANES];
	ulong				buf_box[AM_CAM_MAX_BUFS][AM_CAM_MAX_PLANES];

	/* for debugging */
	u32				buf_req;
	u32				buf_pre;
	u32				buf_que;
	u32				buf_com;
	u32				buf_dqe;

	u32				id;
	unsigned long			state;
};

struct am_cam_video_refcount {
	atomic_t			refcount;
	struct am_cam_video		*video;
	int				(*first)(struct am_cam_video *video);
	int				(*final)(struct am_cam_video *video);
};

struct am_cam_video_ctx {
	struct am_cam_queue		queue;
	struct mutex			lock;
	u32				instance;
	u32				refcount;
	unsigned long			state;

	void				*device;
	void				*subdev;
	struct am_cam_video		*video;
};

struct am_cam_video {
	u32				id;
	atomic_t			refcount;
	struct mutex			lock;
	struct video_device		vd;
	struct am_cam_video_refcount	open_cnt;

	const struct vb2_ops		*vb2_ops;
	const struct vb2_mem_ops	*vb2_mem_ops;
	void				*alloc_ctx;
};

struct am_cam_buf {
	struct vb2_buffer		*vb2_buf;
	struct v4l2_buffer		*v4l2_buf;
	struct timeval			timestamp;
	dma_addr_t			dvaddr[AM_CAM_MAX_PLANES];
	void				*kvaddr[AM_CAM_MAX_PLANES];
};

int am_cam_video_probe(struct am_cam_video *video, struct v4l2_device *v4l2_dev);

#endif
