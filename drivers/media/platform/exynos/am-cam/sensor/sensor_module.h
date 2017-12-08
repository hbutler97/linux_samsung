/*
 * Samsung Exynos SoC series module-camera driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef __SENSOR_MODULE_H
#define __SENSOR_MODULE_H

//#define DEBUG_LOG
#ifdef DEBUG_LOG
#define sen_debug(fmt, args...) printk(KERN_INFO "[FIMCIS_DEBUG]" fmt, ##args)
#else
#define sen_debug(fmt, args...)
#endif
#define sen_info(fmt, args...) printk(KERN_INFO "[SEN_INFO]" fmt, ##args)
#define sen_err(fmt, args...) printk(KERN_INFO "[SEN_ERR]" fmt, ##args)


struct sensor_module_ops {
	int (*init)(int mode);
	int (*stream_on)(void);
	int (*stream_off)(void);
};

int sensor_module_init(int mode);
int sensor_module_stream_on(void);
int sensor_module_stream_off(void);
#endif
