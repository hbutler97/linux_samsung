/* linux/drivers/soc/samsung/exynos-pm-dvs.c
 *
 * Copyright (c) 2015 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Samsung SOC series dynamic voltage control driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 2 of the License,
 * or (at your option) any later version.
 */

#include <linux/of.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include <soc/samsung/exynos-pm-dvs.h>

static LIST_HEAD(regulator_list);

static int exynos_pm_dvs_probe(struct platform_device *pdev)
{
	struct device_node *np = NULL;
	int ret = 0;

	for_each_child_of_node(pdev->dev.of_node, np) {
		struct exynos_dvs_info *di;

		/* skip unmanaged regulator */
		if (!of_device_is_available(np))
			continue;

		di = kzalloc(sizeof(*di), GFP_KERNEL);
		if (!di){
			dev_err(&pdev->dev, "%s: failed to callocate memory for exynos_dvs_info~\n",
					__func__);
			return -ENOMEM;
		}

		ret = of_property_read_string(np,"regulator_name",&di->id);
		if (ret) {
			dev_err(&pdev->dev, "failed to get regulator_name \n");
			kfree(di);
			return ret;
		}

		di->regulator = regulator_get(&pdev->dev,  di->id);
		if (IS_ERR(di->regulator)) {
			dev_err(&pdev->dev, "failed to get resource %s \n", di->id);
			kfree(di);
			return -EINVAL;
		}

		ret = of_property_read_u32(np,"dvs_en",&di->dvs_en);
		if (ret) {
			dev_err(&pdev->dev, "failed to get regulator_name \n");
			kfree(di);
			return ret;
		}

		ret = of_property_read_u32(np,"set_mode_en",&di->set_mode);
		if (ret) {
			dev_err(&pdev->dev, "failed to get regulator_name \n");
			kfree(di);
			return ret;
		}

		if (di->dvs_en) {
			ret = of_property_read_u32(np,"suspend_volt",(u32 *)&di->suspend_volt);
			if(ret) {
				dev_err(&pdev->dev, "%s failed to get suspend_volt \n", di->id);
				kfree(di);
				return ret;
			}

			ret = of_property_read_u32(np,"init_volt",(u32 *)&di->init_volt);
			if(ret) {
				dev_err(&pdev->dev, "%s failed to get init_volt \n", di->id);
				kfree(di);
				return ret;
			}

			ret = of_property_read_u32(np,"volt_range_step",(u32 *)&di->volt_range);
			if(ret) {
				dev_err(&pdev->dev, "%s failed to get volt_range_step \n", di->id);
				kfree(di);
				return ret;
			}
		}

		if (di->set_mode) {
			ret = of_property_read_u32(np,"call_mode",(u32 *)&di->call_mode);
			if(ret) {
				dev_err(&pdev->dev, "%s failed to get call_mode \n", di->id);
				kfree(di);
				return ret;
			}

			ret = of_property_read_u32(np,"init_mode",(u32 *)&di->init_mode);
			if(ret) {
				dev_err(&pdev->dev, "%s failed to get init_mode \n", di->id);
				kfree(di);
				return ret;
			}
		}

		list_add_tail(&di->node, &regulator_list);
	}

	dev_info(&pdev->dev, "driver probe success!");
	return 0;
}

static int exynos_pm_dvs_remove(struct platform_device *pdev)
{
	struct exynos_dvs_info *di;

	list_for_each_entry(di, &regulator_list, node) {
		regulator_put(di->regulator);
		kfree(di);
	}
	return 0;
}

#ifdef CONFIG_PM
static int exynos_pm_dvs_suspend_late(struct device *dev)
{
	struct exynos_dvs_info *di;
	int ret = 0;

	list_for_each_entry(di, &regulator_list, node) {
		if (di->dvs_en) {
			ret = regulator_set_voltage(di->regulator, di->suspend_volt, di->suspend_volt + di->volt_range);
			if (ret) {
				dev_err(dev, "%s	failed to regulator set_voltage ", di->id);
				break;
			}
		}

		if (di->set_mode && is_cp_aud_enabled()) {
			dev_info(dev, "set call mode:%d \n",di->call_mode);
			ret = pmic_set_mode(di->regulator, di->call_mode);
			if (ret) {
				dev_err(dev, "%s	failed to regulator set_mode ", di->id);
				break;
			}
		}
	}
	return ret;
}

static int exynos_pm_dvs_resume_early(struct device *dev)
{
	struct exynos_dvs_info *di;
	int ret = 0;

	list_for_each_entry(di, &regulator_list, node) {
		if (di->dvs_en) {
			regulator_set_voltage(di->regulator, di->init_volt, di->init_volt + di->volt_range);
			if(ret) {
				dev_err(dev, "%s	failed to regulator restore voltage ", di->id);
				break;
			}
		}

		if (di->set_mode && is_cp_aud_enabled()) {
			dev_info(dev, "set init mode:%d \n",di->init_mode);
			ret = pmic_set_mode(di->regulator, di->init_mode);
			if (ret) {
				dev_err(dev, "%s	failed to regulator set_mode ", di->id);
				break;
			}
		}
	}
	return ret;
}

static const struct dev_pm_ops exynos_pm_dvs_pm = {
	.suspend_late = exynos_pm_dvs_suspend_late,
	.resume_early = exynos_pm_dvs_resume_early,
};
#endif

static const struct of_device_id exynos_pm_dvs_match[] = {
	{
		.compatible = "samsung,exynos-pm-dvs",
	},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_pm_dvs);

static struct platform_device_id exynos_pm_dvs_driver_ids[] = {
	{
		.name		= EXYNOS_PM_DVS_MODULE_NAME,
	},
	{},
};
MODULE_DEVICE_TABLE(platform, exynos_pm_dvs_driver_ids);


static struct platform_driver exynos_pm_dvs_driver = {
	.driver	= {
		.name	= EXYNOS_PM_DVS_MODULE_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &exynos_pm_dvs_pm,
#endif
		.of_match_table = of_match_ptr(exynos_pm_dvs_match),
	},
	.probe	= exynos_pm_dvs_probe,
	.remove	= exynos_pm_dvs_remove,
	.id_table = exynos_pm_dvs_driver_ids,
};

module_platform_driver(exynos_pm_dvs_driver);

MODULE_DESCRIPTION("Samsung soc series dynamic voltage control driver");
MODULE_LICENSE("GPL");
