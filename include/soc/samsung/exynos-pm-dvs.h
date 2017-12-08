/*
 * linux/soc/samsung/exynos-pm_dvs.h
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_EXYNOS_PM_DVS_H
#define __LINUX_EXYNOS_PM_DVS_H

#define EXYNOS_PM_DVS_MODULE_NAME "exynos-pm-dvs"

struct exynos_dvs_info
{
        struct list_head node;
        struct regulator *regulator;
        const char *id;
        u32 dvs_en;
        u32 set_mode;
        u32 suspend_volt;
        u32 init_volt;
        u32 volt_range;
        u32 init_mode;
        u32 call_mode;
};

#ifdef CONFIG_SOC_EXYNOS8890
extern int pmic_set_mode(struct regulator *rdev, unsigned int mode);
#else
static inline int pmic_set_mode(struct regulator *rdev, unsigned int mode)
{
	return 0;
}
#endif

extern int is_cp_aud_enabled(void);
#endif /* __LINUX_EXYNOS_PM_DVS_H */
