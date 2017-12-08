/*
* Samsung Exynos5 SoC series FIMC-IS driver
 *
 * exynos5 fimc-is vender functions
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef FIMC_IS_VENDER_H
#define FIMC_IS_VENDER_H

#include <linux/types.h>
#include <linux/platform_device.h>

#define FIMC_IS_PATH_LEN 100

struct fimc_is_vender {
	char fw_path[FIMC_IS_PATH_LEN];
	char request_fw_path[FIMC_IS_PATH_LEN];
	char setfile_path[FIMC_IS_PATH_LEN];
	char request_setfile_path[FIMC_IS_PATH_LEN];
	void *private_data;
};

int fimc_is_vender_probe(struct fimc_is_vender *vender);
int fimc_is_vender_dt(struct device_node *np);
int fimc_is_vender_fw_prepare(struct fimc_is_vender *vender);
int fimc_is_vender_fw_load(struct fimc_is_vender *vender);
int fimc_is_vender_cal_load(struct fimc_is_vender *vender, void *module_data);
int fimc_is_vender_module_sel(struct fimc_is_vender *vender, void *module_data);
int fimc_is_vender_fw_sel(struct fimc_is_vender *vender);
int fimc_is_vender_setfile_sel(struct fimc_is_vender *vender, char *setfile_name);
int fimc_is_vender_gpio_on_sel(struct fimc_is_vender *vender, u32 scenario, u32 *gpio_scneario);
int fimc_is_vender_gpio_on(struct fimc_is_vender *vender, u32 scenario, u32 gpio_scenario);
int fimc_is_vender_gpio_off_sel(struct fimc_is_vender *vender, u32 scenario, u32 *gpio_scneario);
int fimc_is_vender_gpio_off(struct fimc_is_vender *vender, u32 scenario, u32 gpio_scenario);
int fimc_is_vender_set_torch(u32 aeflashMode);
#endif