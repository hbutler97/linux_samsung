/*
 * Samsung Exynos SoC series module-camera driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef __S5K4EC_SOC_H
#define __S5K4EC_SOC_H

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/wait.h>

int s5k4ec_init(int mode);
int s5k4ec_stream_on(void);
int s5k4ec_stream_off(void);

struct s5k4ec_dev {
	uint32_t	mode;
	uint32_t	stream_on;

	uint32_t	i2c_addr;
};

struct s5k4ec_reg {
	uint32_t *reg;
	uint32_t size;
};

int s5k4ec_set_reg(struct s5k4ec_reg *reg_set);
#endif
