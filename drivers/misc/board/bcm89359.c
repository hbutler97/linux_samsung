/*
 *
 *  Broadcom Bluetooth driver
 *
 *  Copyright (C) 2016 Samsung Electronics Co., Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/rfkill.h>
#include <linux/serial_core.h>
#include <linux/wakelock.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/serial_s3c.h>
#include <soc/samsung/exynos-powermode.h>

extern s3c_wake_peer_t s3c2410_serial_wake_peer[CONFIG_SERIAL_SAMSUNG_UARTS];

struct bt_gpio {
	int bt_reg_on;
	int bt_dev_wake;
	int bt_host_wake;
	int irq;
} bt_gpio;

int idle_ip_index;
int bt_uart_port;
struct rfkill *bt_rfkill;

struct bt_lpm {
	int host_wake;
	int dev_wake;

	struct hrtimer enter_lpm_timer;
	ktime_t enter_lpm_delay;

	struct uart_port *uport;

	struct wake_lock host_wake_lock;
	struct wake_lock dev_wake_lock;
} bt_lpm;

static int bcm89359_bt_rfkill_set_power(void *data, bool blocked)
{
	if (!blocked) {
		if (irq_set_irq_wake(bt_gpio.irq, 1)) {
			pr_err("bluetooth: irq_wake enable failed.\n");
			return -1;
		}
		gpio_set_value(bt_gpio.bt_reg_on, 1);
		gpio_set_value(bt_gpio.bt_dev_wake, 1);
		exynos_update_ip_idle_status(idle_ip_index, 0);
		pr_info("bluetooth: power on.\n");
	} else {
		if (gpio_get_value(bt_gpio.bt_reg_on)
				&& irq_set_irq_wake(bt_gpio.irq, 0)) {
			pr_err("bluetooth: irq_wake disable failed.\n");
			return -1;
		}
		exynos_update_ip_idle_status(idle_ip_index, 1);
		gpio_set_value(bt_gpio.bt_reg_on, 0);
		pr_info("bluetooth: power off.\n");
	}
	return 0;
}

static const struct rfkill_ops bcm89359_bt_rfkill_ops = {
	.set_block = bcm89359_bt_rfkill_set_power,
};

static void set_wake_locked(int wake)
{
	if (wake)
		wake_lock(&bt_lpm.dev_wake_lock);

	gpio_set_value(bt_gpio.bt_dev_wake, wake);
	bt_lpm.dev_wake = wake;
}

static enum hrtimer_restart enter_lpm(struct hrtimer *timer)
{
	if (bt_lpm.uport != NULL)
		set_wake_locked(0);

	if (bt_lpm.host_wake == 0)
		exynos_update_ip_idle_status(idle_ip_index, 1);

	wake_lock_timeout(&bt_lpm.dev_wake_lock, HZ/2);
	return HRTIMER_NORESTART;
}

void bcm_bt_lpm_exit_lpm_locked(struct uart_port *uport)
{
	bt_lpm.uport = uport;
	uport->hw_stopped = 0;
	hrtimer_try_to_cancel(&bt_lpm.enter_lpm_timer);
	exynos_update_ip_idle_status(idle_ip_index, 0);
	set_wake_locked(1);
	hrtimer_start(&bt_lpm.enter_lpm_timer, bt_lpm.enter_lpm_delay,
			HRTIMER_MODE_REL);
}

static void update_host_wake_locked(int host_wake)
{
	if (host_wake == bt_lpm.host_wake)
		return;

	bt_lpm.host_wake = host_wake;
	if (host_wake) {
		exynos_update_ip_idle_status(idle_ip_index, 0);
		wake_lock(&bt_lpm.host_wake_lock);
	} else {
		wake_lock_timeout(&bt_lpm.host_wake_lock, HZ);
	}

	if (bt_lpm.dev_wake == 0)
		exynos_update_ip_idle_status(idle_ip_index, 1);
}

static irqreturn_t host_wake_isr(int irq, void *dev)
{
	int host_wake;

	host_wake = gpio_get_value(bt_gpio.bt_host_wake);

	if (!bt_lpm.uport) {
		bt_lpm.host_wake = host_wake;
		return IRQ_HANDLED;
	}

	update_host_wake_locked(host_wake);

	return IRQ_HANDLED;
}

static int bcm89359_bluetooth_probe(struct platform_device *pdev)
{
	int rc = 0;
	int ret;

	dev_info(&pdev->dev, "bcm89359_bluetooth_probe()\n");

	bt_gpio.bt_reg_on = of_get_gpio(pdev->dev.of_node, 0);

	if (!gpio_is_valid(bt_gpio.bt_reg_on)) {
		dev_err(&pdev->dev, "bluetooth: failed to get bt_reg_on.\n");
		return -EINVAL;
	}

	bt_gpio.bt_dev_wake = of_get_gpio(pdev->dev.of_node, 1);

	if (!gpio_is_valid(bt_gpio.bt_dev_wake)) {
		dev_err(&pdev->dev, "bluetooth: failed to get bt_dev_wake.\n");
		return -EINVAL;
	}

	bt_gpio.bt_host_wake = of_get_gpio(pdev->dev.of_node, 2);

	if (!gpio_is_valid(bt_gpio.bt_host_wake)) {
		dev_err(&pdev->dev, "bluetooth: failed to get bt_host_wake.\n");
		return -EINVAL;
	}

	gpio_direction_input(bt_gpio.bt_host_wake);
	gpio_direction_output(bt_gpio.bt_dev_wake, 0);
	gpio_direction_output(bt_gpio.bt_reg_on, 0);

	bt_rfkill = rfkill_alloc("bcm89359 Bluetooth", &pdev->dev,
			RFKILL_TYPE_BLUETOOTH, &bcm89359_bt_rfkill_ops, NULL);

	if (unlikely(!bt_rfkill)) {
		dev_err(&pdev->dev, "bluetooth: bt_rfkill alloc failed.\n");
		return -ENOMEM;
	}

	rfkill_init_sw_state(bt_rfkill, 0);
	rc = rfkill_register(bt_rfkill);

	if (unlikely(rc)) {
		dev_err(&pdev->dev, "bluetooth: bt_rfkill register failed.\n");
		return -1;
	}

	rfkill_set_sw_state(bt_rfkill, 1);

        hrtimer_init(&bt_lpm.enter_lpm_timer,
			CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	bt_lpm.enter_lpm_delay = ktime_set(1, 0);
	bt_lpm.enter_lpm_timer.function = enter_lpm;

	bt_lpm.host_wake = 0;
	wake_lock_init(&bt_lpm.host_wake_lock, WAKE_LOCK_SUSPEND, "BT_host_wake");
	wake_lock_init(&bt_lpm.dev_wake_lock, WAKE_LOCK_SUSPEND, "BT_bt_wake");

	if (of_property_read_u32(pdev->dev.of_node, "uart-port", &bt_uart_port)) {
		dev_warn(&pdev->dev, "bluetooth: uart port number is not specified, assuming port 1.");
		bt_uart_port = 1;
	}

	s3c2410_serial_wake_peer[bt_uart_port] =
			(s3c_wake_peer_t) bcm_bt_lpm_exit_lpm_locked;

	bt_gpio.irq = gpio_to_irq(bt_gpio.bt_host_wake);

	ret = request_irq(bt_gpio.irq, host_wake_isr,
		IRQF_TRIGGER_FALLING  | IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND,
		"bt_host_wake", NULL);

	if (ret) {
		dev_err(&pdev->dev, "bluetooth: bt_host_wake irq failed :%d\n",ret);
		rfkill_unregister(bt_rfkill);
		rfkill_destroy(bt_rfkill);
	}

	idle_ip_index = exynos_get_idle_ip_index("bluetooth");
	exynos_update_ip_idle_status(idle_ip_index, 1);

	return 0;
}

static const struct of_device_id exynos_bluetooth_match[] = {
	{
		.compatible = "broadcom,bcm89359",
	},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_bluetooth_match);

static struct platform_driver bcm89359_bluetooth_driver = {
	.probe		= bcm89359_bluetooth_probe,
	.driver		= {
			   .name  = "bcm89359_bluetooth",
			   .owner = THIS_MODULE,
			   .of_match_table = exynos_bluetooth_match,
			},
};

static int __init bt_init(void)
{
	int ret = platform_driver_register(&bcm89359_bluetooth_driver);

	if (ret)
		pr_err("bluetooth driver registrer failed\n");

	return ret;
}

module_init(bt_init);

static void __exit bt_exit(void)
{
	platform_driver_unregister(&bcm89359_bluetooth_driver);
}

module_exit(bt_exit);

MODULE_AUTHOR("Jung Ick Guack <ji.guack@samsung.com>");
MODULE_AUTHOR("Youngmin Nam <youngmin.nam@samsung.com>");
MODULE_DESCRIPTION("Broadcom bluetooth driver");
MODULE_LICENSE("GPL");
