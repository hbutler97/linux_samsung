/*
 * Samsung Exynos SoC series module-camera driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include "sensor_module.h"

extern struct sensor_module_ops *mod_ops;

int sensor_module_init(int mode)
{
	int bRet = 0;

	bRet = mod_ops->init(mode);
	return bRet;
}

int sensor_module_stream_on(void)
{

	mod_ops->stream_on();
	return 0;
}

int sensor_module_stream_off(void)
{

	mod_ops->stream_off();
	return 0;
}
