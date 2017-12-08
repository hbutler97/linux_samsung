/*
 * Samsung Exynos SoC series module-camera driver
 *
 * Copyright (c) 2016 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/io.h>

#include "fimc_is_hw_regs.h"

uint32_t update_field(uint32_t reg_value, struct fimc_is_field *field, uint32_t val)
{
	uint32_t field_mask = (1 << field->bit_width) - 1;

	/* bit clear */
	reg_value &= ~(field_mask << field->bit_start);

	/* setting value */
	reg_value |= (val & field_mask) << (field->bit_start);

	return reg_value;
};

void set_field(unsigned long reg, struct fimc_is_field *field, uint32_t val)
{
	uint32_t field_mask = (1 << field->bit_width) - 1;
	uint32_t reg_value = *(unsigned int*)reg;

	/* bit clear */
	reg_value &= ~(field_mask << field->bit_start);

	/* setting value */
	reg_value |= (val & field_mask) << (field->bit_start);

	*(unsigned int*)reg = reg_value;
};

