/*
 * Copyright (c) 2016 Samsung Electronics Co., Ltd.
 *	      http://www.samsung.com/
 *
 * EXYNOS - mapping cpu onto physical core
 * Author: Park Bumgyu <bumgyu.park@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/smc.h>

u64 get_auto_cpu_hwid(u32 cpu)
{
	return exynos_smc(SMC_CMD_CPUMAP, cpu, 0, 0);
}
