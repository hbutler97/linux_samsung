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
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/time.h>

#include "sensor_module.h"
#include "s5k4ec_soc.h"
#include "../i2c/i2c_api.h"

#include "s5k4ec_soc_reg_common.h"
#include "s5k4ec_soc_reg_1280x720.h"
#include "s5k4ec_soc_reg_1920x1080.h"
#include "s5k4ec_soc_reg_2560x1440.h"

static const struct sensor_module_ops s5k4ec_ops = {
	.init = s5k4ec_init,
	.stream_on = s5k4ec_stream_on,
	.stream_off = s5k4ec_stream_off
};

const struct sensor_module_ops *mod_ops = &s5k4ec_ops;

struct s5k4ec_dev sen_dev;

int parsing_reg_set(struct s5k4ec_reg *register_set)
{
	struct reg_data_set {
		uint32_t size;
		uint8_t *data;
	};

	struct reg_data_set *regset_table;
	struct reg_data_set *regset;
	struct reg_data_set *end_regset;
	uint8_t *regset_data;
	uint8_t *dst_ptr;
	const uint32_t *end_src_ptr;
	int flag_copied;
	int init_reg_2_array_size = register_set->size;
	int init_reg_2_size = init_reg_2_array_size * sizeof(uint32_t);
	const uint32_t *src_ptr = register_set->reg;
	uint32_t src_value;
	int err;

	sen_debug("%s : start\n", __func__);

	regset_data = kmalloc(init_reg_2_size, GFP_KERNEL);
	if (regset_data == NULL)
		return -ENOMEM;

	regset_table = kmalloc((uint32_t)sizeof(struct reg_data_set) * init_reg_2_size, GFP_KERNEL);
	if (regset_table == NULL) {
		kfree(regset_data);
		return -ENOMEM;
	}

	dst_ptr = regset_data;
	regset = regset_table;
	end_src_ptr = &register_set->reg[init_reg_2_array_size - 1];

	src_value = *src_ptr++;
	while (src_ptr <= end_src_ptr) {
		/* initial value for a regset */
		regset->data = dst_ptr;
		flag_copied = 0;
		*dst_ptr++ = src_value >> 24;
		*dst_ptr++ = src_value >> 16;
		*dst_ptr++ = src_value >> 8;
		*dst_ptr++ = src_value;

		/* check subsequent values for a data flag (starts with
		   0x0F12) or something else */
		do {
			src_value = *src_ptr++;
			if ((src_value & 0xFFFF0000) != 0x0F120000) {
				/* src_value is start of next regset */
				regset->size = dst_ptr - regset->data;
				regset++;
				break;
			}
			/* copy the 0x0F12 flag if not done already */
			if (!flag_copied) {
				*dst_ptr++ = src_value >> 24;
				*dst_ptr++ = src_value >> 16;
				flag_copied = 1;
			}
			/* copy the data part */
			*dst_ptr++ = src_value >> 8;
			*dst_ptr++ = src_value;
		} while (src_ptr < end_src_ptr);
	}
	sen_debug("%s : finished creating table\n", __func__);

	end_regset = regset;
	sen_debug("%s : first regset = %p, last regset = %p, count = %d\n",
			__func__, regset_table, regset, end_regset - regset_table);
	sen_debug("%s : regset_data = %p, end = %p, dst_ptr = %p\n", __func__,
			regset_data, regset_data + (init_reg_2_size * sizeof(uint32_t)),
			dst_ptr);

	regset = regset_table;
	sen_debug("%s : start writing init reg 2 bursts\n", __func__);
	do {
		if (regset->size > 4) {
			/* write the address packet */
			err = i2c_write_burst(sen_dev.i2c_addr, regset->data, 4);
			if (err)
				break;
			/* write the data in a burst */
			err = i2c_write_burst(sen_dev.i2c_addr, regset->data + 4,
					regset->size-4);

		} else
			err = i2c_write_burst(sen_dev.i2c_addr, regset->data,
					regset->size);
		if (err)
			break;
		regset++;
	} while (regset < end_regset);

	sen_debug("%s : finished writing init reg 2 bursts\n", __func__);

	kfree(regset_data);
	kfree(regset_table);

	if (err) {
		sen_err("%s: write cmd failed\n", __func__);
		goto p_err;
	}

p_err:
	return err;
}

int s5k4ec_init(int mode)
{
	uint16_t rev;
	int i2c_ret = 0;
	struct timeval st, ed;

	sen_dev.mode = mode;
	sen_dev.stream_on = 0;
	sen_dev.i2c_addr = 0xAC;

	do_gettimeofday(&st);
	sen_info("am-cam-SEN-0 %X\n", sen_dev.i2c_addr);
	/* enter read mode */
	i2c_ret = i2c_write_addr16data16(sen_dev.i2c_addr, 0x002C, 0x7000);

	if (i2c_ret != 0)
	    return i2c_ret;

	sen_info("am-cam-SEN-1\n");
	i2c_ret = i2c_write_addr16data16(sen_dev.i2c_addr, 0x002E, 0x01A6);

	if (i2c_ret != 0)
	    return i2c_ret;

	i2c_ret = i2c_read_addr16data16(sen_dev.i2c_addr, 0x0F12, &rev);

    if (i2c_ret != 0)
	    return i2c_ret;

	sen_info("%s : revision %08X\n", __func__, rev);

	/* restore write mode */
	i2c_write_addr16data16(sen_dev.i2c_addr, 0x0028, 0x7000);

	msleep(10);

	s5k4ec_set_reg(&s5k4ec_reg_sw_reset);

	msleep(10);

	//parsing_reg_set(&s5k4ec_reg_set_1280x720);
	parsing_reg_set(&s5k4ec_reg_set_1920x1080);

	// contrast: default
	s5k4ec_set_reg(&s5k4ec_reg_contrast_default);

	// effect: none
	s5k4ec_set_reg(&s5k4ec_reg_effect_none);

	// ev: brightness default
	s5k4ec_set_reg(&s5k4ec_reg_brightness_default);

	// iso: auto
	s5k4ec_set_reg(&s5k4ec_reg_iso_auto);

	// metering: center
	s5k4ec_set_reg(&s5k4ec_reg_metering_center);

	// saturation: default
	s5k4ec_set_reg(&s5k4ec_reg_saturation_default);

	// scene mode: none
	s5k4ec_set_reg(&s5k4ec_reg_scene_none);

	// sharpness: default
	s5k4ec_set_reg(&s5k4ec_reg_sharpness_default);

	// aeawb: unlock
	s5k4ec_set_reg(&s5k4ec_reg_ae_unlock);
	s5k4ec_set_reg(&s5k4ec_reg_awb_unlock);

	// awb: auto
	s5k4ec_set_reg(&s5k4ec_reg_awb_mode_auto);

	// fps: auto
	s5k4ec_set_reg(&s5k4ec_reg_fps_auto);
	do_gettimeofday(&ed);

	sen_info("Sensor init time: %ld msec\n", (ed.tv_usec-st.tv_usec)/1000);

	return 0;
}

int s5k4ec_stream_on(void)
{
	uint32_t time_out = 0;

	if (sen_dev.stream_on == 1)
		sen_info("alreay stream on\n");

	s5k4ec_set_reg(&s5k4ec_reg_stream_on);

	/* wait stream on complete */
	i2c_write_addr16data16(sen_dev.i2c_addr, 0x002C, 0x7000);
	do {
		uint16_t val;

		i2c_write_addr16data16(sen_dev.i2c_addr, 0x002E, 0x0240);
		i2c_read_addr16data16(sen_dev.i2c_addr, 0x0F12, &val);
		if (val == 0x00)
			break;
		msleep(1);
		time_out += 1;
	} while (time_out < 33);

	sen_dev.stream_on = 1;

	return 0;
}

int s5k4ec_stream_off(void)
{
	uint32_t time_out = 0;

	if (sen_dev.stream_on == 0)
		sen_info("alreay stream off\n");

	s5k4ec_set_reg(&s5k4ec_reg_stream_off);

	/* wait stream off complete */
	i2c_write_addr16data16(sen_dev.i2c_addr, 0x002C, 0x7000);
	do {
		uint16_t val;

		i2c_write_addr16data16(sen_dev.i2c_addr, 0x002E, 0x0240);
		i2c_read_addr16data16(sen_dev.i2c_addr, 0x0F12, &val);
		if (val == 0x00)
			break;
		msleep(1);
		time_out += 1;
	} while (time_out < 33);

	sen_dev.stream_on = 0;
	return 0;
}

int s5k4ec_set_reg(struct s5k4ec_reg *reg_set)
{
	int ret = 0;
	int i = 0;
	uint16_t addr = 0;
	uint16_t val = 0;


	for (i = 0; i < reg_set->size; i++) {
		addr = (reg_set->reg[i] & 0xFFFF0000) >> 16;
		val = (reg_set->reg[i] & 0xFFFF);
		ret = i2c_write_addr16data16(sen_dev.i2c_addr, addr, val);
		if (ret) {
			sen_err("[%s] i2c write fail!!\n", __func__);
			break;
		}
	}

	return ret;
}
