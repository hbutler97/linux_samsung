/*
 * Samsung Exynos SoC series am-cam driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef AM_CAM_DEVICE_H_
#define AM_CAM_DEVICE_H_

#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "am-cam-video.h"

struct am_cam_device {
	struct device				*dev;
	struct platform_device			*pdev;

	struct v4l2_device			v4l2_dev;
	struct v4l2_subdev			subdev_sen;
	struct v4l2_subdev			subdev_csi;

	struct pinctrl				*pinctrl;

	struct am_cam_video			video;
};

int am_cam_device_open(struct am_cam_device *device);
int am_cam_device_close(struct am_cam_device *device);

#endif
