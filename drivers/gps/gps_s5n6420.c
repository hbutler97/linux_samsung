/*
 * Driver for GPS-S5N6420
 *
 * Copyright (c) 2015 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/module.h>

#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/unistd.h>
#include <linux/bug.h>

#if defined(CONFIG_64BIT)
#include <asm-generic/gpio.h>
#else
#include <mach/gpio.h>
#endif /* CONFIG_64BIT */
#include <linux/sec_sysfs.h>

static unsigned int gps_pwr_on;
static unsigned int gps_reset;

static int gps_s5n6420_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node;
	int ret;

	node = dev->of_node;
        printk(KERN_INFO "%s: gps_s5n6420_probe  START.......\n", __FUNCTION__);
        if (!node)
		return -ENODEV;
        printk(KERN_INFO "%s: gps_s5n6420_probe  START.2......\n", __FUNCTION__);

	gps_pwr_on = of_get_gpio(node, 1);
	if (gpio_is_valid(gps_pwr_on))
	{
		ret = gpio_request(gps_pwr_on, "GPS_PWR_EN");
		if (ret) {
			dev_err(dev, "cannot get GPS_PWR_EN GPIO\n");
			return ret;
		}
		gpio_direction_output(gps_pwr_on, 0);
		gpio_export(gps_pwr_on, 1);
		gpio_export_link(dev, "GPS_PWR_EN", gps_pwr_on);
	}
	else
	{
		dev_warn(dev, "GPIO not specified in DT (of_get_gpio returned %d)\n", gps_pwr_on);
		return -ENOENT;
	}

	gps_reset = of_get_gpio(node, 0);
	if (gpio_is_valid(gps_reset))
	{
		ret = gpio_request(gps_reset, "GPS_RESET");
		if (ret) {
			dev_err(dev, "cannot get GPS_RESET GPIO\n");
			return ret;
		}
		gpio_direction_output(gps_reset, 0);
		gpio_export(gps_reset, 1);
		gpio_export_link(dev, "GPS_RESET", gps_reset);
	}
	else
	{
		gpio_unexport(gps_pwr_on);
		gpio_free(gps_pwr_on);

		dev_warn(dev, "GPIO not specified in DT (of_get_gpio returned %d)\n", gps_reset);
		return -ENOENT;
	}

	return 0;
}

static int gps_s5n6420_remove(struct platform_device *pdev)
{
	gpio_unexport(gps_reset);
	gpio_free(gps_reset);

	gpio_unexport(gps_pwr_on);
	gpio_free(gps_pwr_on);

	return 0;
}

static const struct of_device_id gps_s5n6420_of_match[] = {
	{ .compatible = "samsung,gps-s5n6420", },
	{ },
};
MODULE_DEVICE_TABLE(of, gps_s5n6420_of_match);

static struct platform_driver gps_s5n6420_driver = {
	.driver		= {
		.name	= "gps-s5n6420",
                .owner  = THIS_MODULE,
		.of_match_table =of_match_ptr(gps_s5n6420_of_match),
	},
        .probe          = gps_s5n6420_probe,
        .remove         = gps_s5n6420_remove,
};

module_platform_driver(gps_s5n6420_driver);

static int __init gps_s5n6420_init(void)
{
        return platform_driver_register(&gps_s5n6420_driver);
}

static void __exit gps_s5n6420_exit(void)
{
	platform_driver_unregister(&gps_s5n6420_driver);
}

device_initcall(gps_s5n6420_init);
module_exit(gps_s5n6420_exit);
