/*
 * Samsung Exynos SoC series am-cam driver
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
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <video/videonode.h>
#include <media/exynos_mc.h>
#include <asm/cacheflush.h>
#include <asm/pgtable.h>
#include <linux/firmware.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/videodev2.h>
#include <linux/videodev2_exynos_camera.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/pm_qos.h>
#include <linux/bug.h>
#include <linux/v4l2-mediabus.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/exynos_iovmm.h>

#include "am-cam-config.h"
#include "am-cam-device.h"
#include "am-cam-framemgr.h"
#include "hw/fimc-is-csi.h"
#include "hw/fimc_is_hw_regs.h"

#include "exynos-fimc-is.h"
#include "exynos-fimc-is-sensor.h"

struct pm_qos_request exynos_isp_qos_int;
struct pm_qos_request exynos_isp_qos_cam;

void i2c_init(struct device *dev);
void i2c_deinit(struct device *dev);
int sensor_module_init(int mode);
int sensor_module_stream_on(void);
int sensor_module_stream_off(void);

int i2c_ret = 0;

static unsigned int probed = 0;
unsigned int am_cam_device_get_probed(void)
{
	return probed;
}

void am_cam_device_set_probed(void)
{
	probed = 1;
}

int init_pwr(void)
{
	unsigned long base;

	unsigned int offset = 0;
	unsigned int val = 0;
	unsigned long addr = 0;

	/* HACK */
	base = (unsigned long) ioremap_nocache(EXYNOS_REG_PMU_BASE, 0x10000);

	/* CAM0 */
	/* Check if CAM0_STATUS is Power off */
	offset = 0x4024;
	addr = base + offset;
	val = *(unsigned int *)addr;
	am_cam_info("CAM0 POWER STATUS : %X\n", val);
	if (val == 0x0) {
		offset = 0x4028;
		addr = base + offset;
		val = *(unsigned int *)addr;
		val |= 0x1;
		*(unsigned int *)addr = val;

		offset = 0x4020;
		addr = base + offset;
		val = *(unsigned int *)addr;
		val |= 0xf;
		*(unsigned int *)addr = val;

		offset = 0x4024;
		addr = base + offset;
		val = *(unsigned int *)addr;
		if (val != 0xF) {
			am_cam_err("CAM0 Power on fail!\n");
			return -1;
		}
	}

	/* CAM1 */
	/* Check if CAM1_STATUS is Power off */
	offset = 0x40A4;
	addr = base + offset;
	val = *(unsigned int *)addr;
	am_cam_info("CAM1 POWER STATUS : %X\n", val);
	if (val == 0x0) {
		offset = 0x40A8;
		addr = base + offset;
		val = *(unsigned int *)addr;
		val |= 0x1;
		*(unsigned int *)addr = val;

		offset = 0x40A0;
		addr = base + offset;
		val = *(unsigned int *)addr;
		val |= 0xf;
		*(unsigned int *)addr = val;

		offset = 0x40A4;
		addr = base + offset;
		val = *(unsigned int *)addr;
		if (val != 0xF) {
			am_cam_err("CAM1 Power on fail!\n");
			return -1;
		}
	}

	/* ISP0 */
	/* Check if ISP0_STATUS is Power off */
	offset = 0x4144;
	addr = base + offset;
	val = *(unsigned int *)addr;
	am_cam_info("ISP0 POWER STATUS : %X\n", val);
	if (val == 0x0) {
		offset = 0x4148;
		addr = base + offset;
		val = *(unsigned int *)addr;
		val |= 0x1;
		*(unsigned int *)addr = val;

		offset = 0x4140;
		addr = base + offset;
		val = *(unsigned int *)addr;
		val |= 0xf;
		*(unsigned int *)addr = val;

		offset = 0x4144;
		addr = base + offset;
		val = *(unsigned int *)addr;
		if (val != 0xF) {
			am_cam_err("ISP0 Power on fail!\n");
			return -1;
		}
	}

	/* ISP1 */
	/* Check if ISP1_STATUS is Power off */
	offset = 0x4164;
	addr = base + offset;
	val = *(unsigned int *)addr;
	am_cam_info("ISP1 POWER STATUS : %X\n", val);
	if (val == 0x0) {
		offset = 0x4168;
		addr = base + offset;
		val = *(unsigned int *)addr;
		val |= 0x1;
		*(unsigned int *)addr = val;

		offset = 0x4160;
		addr = base + offset;
		val = *(unsigned int *)addr;
		val |= 0xf;
		*(unsigned int *)addr = val;

		offset = 0x4164;
		addr = base + offset;
		val = *(unsigned int *)addr;
		if (val != 0xF) {
			am_cam_err("ISP1 Power on fail!\n");
			return -1;
		}
	}

	iounmap((void*)base);

	return 0;
}

int ispcpu_sysmmu_bypass(void)
{
	unsigned long addr_ispcpu = (unsigned long) ioremap_nocache(SYSMMU_REG_ISPCPU, 0x4);
	unsigned long addr_vra = (unsigned long) ioremap_nocache(SYSMMU_REG_VRA, 0x4);
	unsigned long addr_isp_a = (unsigned long) ioremap_nocache(SYSMMU_REG_IS_A, 0x4);
	unsigned long addr_isp_b = (unsigned long) ioremap_nocache(SYSMMU_REG_IS_B, 0x4);
	unsigned long addr_mcsc = (unsigned long) ioremap_nocache(SYSMMU_REG_MC_SCALER, 0x4);
	unsigned long addr_isp_c = (unsigned long) ioremap_nocache(SYSMMU_REG_IS_C, 0x4);

	*(unsigned int *)addr_ispcpu = 0;
	*(unsigned int *)addr_isp_a = 0;
	*(unsigned int *)addr_isp_b = 0;
	*(unsigned int *)addr_isp_c = 0;
	*(unsigned int *)addr_vra = 0;
	*(unsigned int *)addr_mcsc = 0;

	iounmap((void*)addr_ispcpu);
	iounmap((void*)addr_isp_a);
	iounmap((void*)addr_isp_b);
	iounmap((void*)addr_isp_c);
	iounmap((void*)addr_vra);
	iounmap((void*)addr_mcsc);

	return 0;

}

int clk_off(struct device *dev)
{
	exynos_fimc_is_clk_off(dev);

	exynos_fimc_is_sensor_mclk_off(dev, 0, 0);

	return 0;
}
int clk_setting(struct device *dev)
{
	exynos_fimc_is_clk_get(dev);
	exynos_fimc_is_clk_cfg(dev);
	exynos_fimc_is_clk_on(dev);

	return 0;
}

int mclk_on(struct device *dev)
{
	exynos_fimc_is_sensor_iclk_cfg(dev, 0, 0);
	exynos_fimc_is_sensor_iclk_on(dev, 0, 0);
	exynos_fimc_is_sensor_mclk_on(dev, 0, 0);

	return 0;
}

int sensor_power_on(struct am_cam_device *device)
{
	int ret = 0;
	struct device *dev;
	struct device_node *dnode;
	int gpio_reset = 0;
	struct pinctrl_state *pin1;
	struct regulator *regulator = NULL;

	// for debugging
	volatile unsigned long gpc0 =
		(volatile unsigned long)ioremap_nocache(0x136D0100, 0x1000);
	volatile unsigned long gpc1 =
		(volatile unsigned long)ioremap_nocache(0x136D0140, 0x1000); /* I2C0 : GPC2 */
	volatile unsigned long gpc2 =
		(volatile unsigned long)ioremap_nocache(0x136D0180, 0x1000); /* GPK0 */

	dev = &device->pdev->dev;
	dnode = dev->of_node;

	gpio_reset = of_get_named_gpio(dnode, "gpio_reset", 0);
	if (!gpio_is_valid(gpio_reset)) {
		am_cam_err("failed to get PIN_RESET\n");
		return -EINVAL;
	}
/*probe*/
	device->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(device->pinctrl)) {
		am_cam_err("devm_pinctrl_get is fail");
		goto p_err;
	}

	pin1 = pinctrl_lookup_state(device->pinctrl, "pin1");
	if (IS_ERR_OR_NULL(pin1)) {
		am_cam_err("pinctrl_lookup_state(pin1) is failed");
		goto p_err;
	}
/**/
	gpio_request_one(gpio_reset, GPIOF_OUT_INIT_LOW, "CAM_GPIO_OUTPUT_LOW");
	gpio_free(gpio_reset);

	am_cam_info("reset low : %X %X\n",
			readl((volatile void *)(gpc0 + 0)),
			readl((volatile void *)(gpc0 + 4)));

	//AVDD28_MCAM_SENSOR - VLDO11
	regulator = regulator_get(dev, "VDDA28_CAMSEN");
	if (IS_ERR_OR_NULL(regulator)) {
		pr_err("regulator_get(VDDA28_CAMSEN) fail\n");
		return PTR_ERR(regulator);
	}

	ret = regulator_set_voltage(regulator, 2800000, 2800000);
	if(ret) {
		am_cam_err("regulator_set_voltage(VDDA28_CAMSEN) is fail(%d)\n", ret);
	}

	if (!regulator_is_enabled(regulator)) {
		ret = regulator_enable(regulator);
		if (ret) {
			pr_err("regulator_enable(VDDA28_CAMSEN) fail\n");
			regulator_put(regulator);
			return ret;
		}
	}

	regulator_put(regulator);
	//VDD28_DRV - VLDO12
	regulator = regulator_get(dev, "VDD28_CAMAF");
	if (IS_ERR_OR_NULL(regulator)) {
		pr_err("regulator_get(VDD28_CAMAF) fail\n");
		return PTR_ERR(regulator);
	}

	ret = regulator_set_voltage(regulator, 2800000, 2800000);
	if(ret) {
		am_cam_err("regulator_set_voltage(VDD28_CAMAF) is fail(%d)\n", ret);
	}

	if (!regulator_is_enabled(regulator)) {
		ret = regulator_enable(regulator);
		if (ret) {
			pr_err("regulator_enable(VDD28_CAMAF) fail\n");
			regulator_put(regulator);
			return ret;
		}
	}

	regulator_put(regulator);
	//SVDDD18 - VLDO8
	regulator = regulator_get(dev, "VDD18_CAMIO");
	if (IS_ERR_OR_NULL(regulator)) {
		pr_err("regulator_get(VDD18_CAMIO) fail\n");
		return PTR_ERR(regulator);
	}

	ret = regulator_set_voltage(regulator, 1800000, 1800000);
	if(ret) {
		am_cam_err("regulator_set_voltage(VDD18_CAMIO) is fail(%d)\n", ret);
	}

	if (!regulator_is_enabled(regulator)) {
		ret = regulator_enable(regulator);
		if (ret) {
			pr_err("regulator_enable(VDD18_CAMIO) fail\n");
			regulator_put(regulator);
			return ret;
		}
	}

	regulator_put(regulator);

	//SVDDD12 - BUCK2 GNDB2
	regulator = regulator_get(dev, "VDD12_CAMCORE");
	if (IS_ERR_OR_NULL(regulator)) {
		am_cam_err("regulator_get(VDD12_CAMCORE) fail\n");
		return PTR_ERR(regulator);
	}

	ret = regulator_set_voltage(regulator, 1200000, 1200000);
	if(ret) {
		am_cam_err("regulator_set_voltage(VDD12_CAMCORE) is fail(%d)\n", ret);
	}

	if (!regulator_is_enabled(regulator)) {
		ret = regulator_enable(regulator);
		if (ret) {
			am_cam_err("regulator_enable(VDD12_CAMCORE) fail\n");
			regulator_put(regulator);
			return ret;
		}
	}

	regulator_put(regulator);

	{
		unsigned long value;
		value = readl((volatile void *)gpc2);
		value &= ~0xF000;
		value |= 0x1000;
		writel(value, (volatile void *)gpc2);
		writel(0x8, (volatile void *)(gpc2 + 0x4));
	}

	// pin function
	ret = pinctrl_select_state(device->pinctrl, pin1);
	if (ret < 0) {
		pr_err("pinctrl_select_state(pin1) is fail(%d)\n", ret);
		return ret;
	}

	am_cam_info("I2C : %X\n", readl((const volatile void *)gpc1));
	am_cam_info("MCLK : %X %X\n", readl((const volatile void *)gpc2),
			readl((const volatile void *)(gpc2 + 0x4)));

	// reset high
	gpio_request_one(gpio_reset, GPIOF_OUT_INIT_HIGH, "GPIOF_OUT_INIT_HIGH");
	gpio_free(gpio_reset);

	am_cam_info("reset high : %X %X\n",
			readl((const volatile void *)gpc0),
			readl((const volatile void *)(gpc0 + 4)));

#if 0	
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_reset, "sen_rst low", PIN_OUTPUT, 0, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "VDDAF_2.8V_CAM", PIN_REGULATOR, 1, 2000);
	SET_PIN_VOLTAGE(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "VDDA_2.9V_CAM", PIN_REGULATOR, 1, 0, 2950000);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "VDDIO_1.8V_CAM", PIN_REGULATOR, 1, 0);
	SET_PIN_VOLTAGE(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "VDDD_1.2V_CAM", PIN_REGULATOR, 1, 0, 1200000);
	SET_PIN_VOLTAGE(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "VDDD_RET_1.0V_COMP", PIN_REGULATOR, 1, 0, 1000000);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "VDDIO_1.8V_COMP", PIN_REGULATOR, 1, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "VDDA_1.8V_COMP", PIN_REGULATOR, 1, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "VDDD_CORE_1.0V_COMP", PIN_REGULATOR, 1, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "VDDD_NORET_0.9V_COMP", PIN_REGULATOR, 1, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "VDDD_CORE_0.8V_COMP", PIN_REGULATOR, 1, 0);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_none, "pin", PIN_FUNCTION, 2, 2000);
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_prep_reset, "prep_rst high", PIN_OUTPUT, 1, 300); //cap issue: 0->300
	SET_PIN(pdata, SENSOR_SCENARIO_NORMAL, GPIO_SCENARIO_ON, gpio_reset, "sen_rst high", PIN_OUTPUT, 1, 2000);
#endif

p_err:
	return ret;
}

int sensor_power_off(struct am_cam_device *device)
{
	int ret = 0;
	struct device *dev;
	struct device_node *dnode;
	int gpio_reset = 0;
	struct pinctrl_state *pin0;
	struct regulator *regulator = NULL;

	// for debugging
	volatile unsigned long gpc0 =
		(volatile unsigned long)ioremap_nocache(0x136D0100, 0x1000);
	volatile unsigned long gpc1 =
		(volatile unsigned long)ioremap_nocache(0x136D0140, 0x1000); //I2C0 : GPC2
	volatile unsigned long gpc2 =
		(volatile unsigned long)ioremap_nocache(0x136D0180, 0x1000);

	dev = &device->pdev->dev;
	dnode = dev->of_node;

	gpio_reset = of_get_named_gpio(dnode, "gpio_reset", 0);
	if (!gpio_is_valid(gpio_reset)) {
		am_cam_err("failed to get PIN_RESET\n");
		return -EINVAL;
	}
/*probe*/
	device->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(device->pinctrl)) {
		am_cam_err("devm_pinctrl_get is fail");
		goto p_err;
	}

	pin0 = pinctrl_lookup_state(device->pinctrl, "pin0");
	if (IS_ERR_OR_NULL(pin0)) {
		am_cam_err("pinctrl_lookup_state(pin0) is failed");
		goto p_err;
	}
/**/
	gpio_request_one(gpio_reset, GPIOF_OUT_INIT_LOW, "CAM_GPIO_OUTPUT_LOW");
	gpio_free(gpio_reset);

	am_cam_info("reset low : %X %X\n",
			readl((volatile void *)(gpc0 + 0)),
			readl((volatile void *)(gpc0 + 4)));

	//AVDD28_MCAM_SENSOR - VLDO11
	regulator = regulator_get(dev, "VDDA28_CAMSEN");
	if (IS_ERR_OR_NULL(regulator)) {
		pr_err("regulator_get(VDDA28_CAMSEN) fail\n");
		return PTR_ERR(regulator);
	}

	if (regulator_is_enabled(regulator)) {
		ret = regulator_disable(regulator);
		if (ret) {
			pr_err("regulator_enable(VDDA28_CAMSEN) fail\n");
			regulator_put(regulator);
			return ret;
		}
	}

	regulator_put(regulator);

	//VDD28_DRV - VLDO12
	regulator = regulator_get(dev, "VDD28_CAMAF");
	if (IS_ERR_OR_NULL(regulator)) {
		pr_err("regulator_get(VDD28_CAMAF) fail\n");
		return PTR_ERR(regulator);
	}

	if (regulator_is_enabled(regulator)) {
		ret = regulator_disable(regulator);
		if (ret) {
			pr_err("regulator_enable(VDD28_CAMAF) fail\n");
			regulator_put(regulator);
			return ret;
		}
	}

	regulator_put(regulator);

	//SVDDD18 - VLDO8
	regulator = regulator_get(dev, "VDD18_CAMIO");
	if (IS_ERR_OR_NULL(regulator)) {
		pr_err("regulator_get(VDD18_CAMIO) fail\n");
		return PTR_ERR(regulator);
	}

	if (regulator_is_enabled(regulator)) {
		ret = regulator_disable(regulator);
		if (ret) {
			pr_err("regulator_enable(VDD18_CAMIO) fail\n");
			regulator_put(regulator);
			return ret;
		}
	}

	regulator_put(regulator);

	//SVDDD12 - BUCK2 GNDB2
	regulator = regulator_get(dev, "VDD12_CAMCORE");
	if (IS_ERR_OR_NULL(regulator)) {
		am_cam_err("regulator_get(VDD12_CAMCORE) fail\n");
		return PTR_ERR(regulator);
	}

	if (regulator_is_enabled(regulator)) {
		ret = regulator_disable(regulator);
		if (ret) {
			am_cam_err("regulator_enable(VDD12_CAMCORE) fail\n");
			regulator_put(regulator);
			return ret;
		}
	}

	regulator_put(regulator);

//	{
//		unsigned long value;
//		value = readl((volatile void *)gpc2);
//		value &= ~0xF000;
//		value |= 0x1000;
//		writel(value, (volatile void *)gpc2);
//		writel(0x8, (volatile void *)(gpc2 + 0x4));
//	}

	// pin function
	ret = pinctrl_select_state(device->pinctrl, pin0);
	if (ret < 0) {
		pr_err("pinctrl_select_state(pin0) is fail(%d)\n", ret);
		return ret;
	}

	am_cam_info("I2C : %X\n", readl((const volatile void *)gpc1));
	am_cam_info("MCLK : %X %X\n", readl((const volatile void *)gpc2),
			readl((const volatile void *)(gpc2 + 0x4)));

	am_cam_info("reset low : %X %X\n",
			readl((const volatile void *)gpc0),
			readl((const volatile void *)(gpc0 + 4)));

p_err:
	return ret;
}

void __am_cam_fault_handler(struct am_cam_device *device)
{

	struct fimc_is_device_csi *csi;
	struct am_cam_framemgr *framemgr;

	csi = (struct fimc_is_device_csi *)v4l2_get_subdevdata(&device->subdev_csi);
	if (!csi) {
		am_cam_err("csi is NULL");
		return;;
	}

	framemgr = csi->framemgr;
	am_cam_frame_print_all(framemgr);
}

static int __attribute__((unused)) am_cam_fault_handler(struct iommu_domain *domain,
		struct device *dev,
		unsigned long fault_addr,
		int fault_flag,
		void *token)
{
	struct am_cam_device *device;
	pr_err("<AM-CAM FAULT HANDLER>\n");
	pr_err("Device virtual(0x%X) is invalid access\n", (u32)fault_addr);

	device = dev_get_drvdata(dev);

	__am_cam_fault_handler(device);

	return -EINVAL;
}

static int am_cam_device_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct am_cam_device *device;
	struct device *dev;

	dev = &pdev->dev;

	device = kzalloc(sizeof(struct am_cam_device), GFP_KERNEL);
	if (!device) {
		probe_err("device is NULL");
		return -ENOMEM;
	}

	device->pdev = pdev;

	ret = v4l2_device_register(dev, &device->v4l2_dev);
	if (ret) {
		probe_err("v4l2_device_register is fail\n");
		goto p_err;
	}

	ret = am_cam_video_probe(&device->video, &device->v4l2_dev);
	if (ret) {
		probe_err("am_cam_video_probe is fail\n");
		goto p_err;
	}

	ret = fimc_is_csi_probe(device);
	if (ret) {
		probe_err("fimc_is_csi_probe is fail\n");
		goto p_err;
	}

#if defined(CONFIG_PM_RUNTIME)
	pm_runtime_enable(dev);
#endif
	device->dev = dev;

	dev_set_drvdata(dev, device);

	iovmm_set_fault_handler(dev, am_cam_fault_handler, NULL);

p_err:
	probe_info("%s():%d\n", __func__, ret);
	return ret;
}

static int am_cam_device_remove(struct platform_device *pdev)
{
	return 0;
}


int am_cam_device_open(struct am_cam_device *device)
{
	int ret = 0;
	struct device *dev;
	struct fimc_is_device_csi *csi;
	struct am_cam_framemgr *framemgr;

	csi = (struct fimc_is_device_csi *)v4l2_get_subdevdata(&device->subdev_csi);
	if (!csi) {
		am_cam_err("csi is NULL");
		return -EINVAL;
	}

	dev = &device->pdev->dev;

#if defined(CONFIG_PM_DEVFREQ)
	pm_qos_add_request(&exynos_isp_qos_cam, PM_QOS_CAM_THROUGHPUT, 670000);
	pm_qos_add_request(&exynos_isp_qos_int, PM_QOS_DEVICE_THROUGHPUT, 640000);
#endif
#if defined(CONFIG_PM_RUNTIME)
	ret = pm_runtime_get_sync(device->dev);
	if (ret)
		am_cam_err("runtime get_sync is fail(%d)", ret);
#else
#endif

	if (am_cam_device_get_probed() == 0) {

		am_cam_info("1. init power+\n");
		init_pwr();
		am_cam_info("1. init power-\n");

		/* am_cam_info("2. sysmmu off+\n"); */
		/* ispcpu_sysmmu_bypass(); */
		/* am_cam_info("2. sysmmu off-\n"); */

		am_cam_info("3. phy power on+\n");
		csi_s_power(&device->subdev_csi, 1);
		am_cam_info("3. phy power on-\n");

		am_cam_info("4. clk setting+\n");

		clk_setting(dev);
		am_cam_info("4. clk setting-\n");

		am_cam_info("5. mclk on+\n");
		mclk_on(dev);
		am_cam_info("5. mclk on-\n");

		am_cam_info("6. csi init+\n");

		framemgr = kzalloc(sizeof(struct am_cam_framemgr), GFP_KERNEL);
		spin_lock_init(&framemgr->slock);
		am_cam_frame_init(framemgr);

		ret = fimc_is_csi_open(&device->subdev_csi, framemgr);
		if (ret) {
			am_cam_err("fimc_is_csi_open fail \n");
		}

		am_cam_info("6. csi init-\n");

		am_cam_info("7. sensor power on+\n");
		sensor_power_on(device);
		am_cam_info("7. sensor power on-\n");

		am_cam_info("8. i2c init+\n");
		i2c_init(dev);
		am_cam_info("8. i2c init-\n");

		am_cam_info("9. sensor init+\n");
		i2c_ret = sensor_module_init(0);
		am_cam_info("9. sensor init-\n");

        if (i2c_ret != 0) {
#if defined(CONFIG_PM_DEVFREQ)
	        pm_qos_remove_request(&exynos_isp_qos_cam);
	        pm_qos_remove_request(&exynos_isp_qos_int);
#endif
#if defined(CONFIG_PM_RUNTIME)
	        ret = pm_runtime_put_sync(device->dev);
	        if (ret)
		        am_cam_err("runtime put_sync is fail(%d)", ret);
#else
#endif

	    am_cam_info("%s():sensor_module_init failed %d\n", __func__, i2c_ret);
	    return i2c_ret;
}
		am_cam_device_set_probed();
	}

	am_cam_info("%s():%d\n", __func__, ret);

	return ret;
}

int am_cam_device_close(struct am_cam_device *device)
{
	int ret = 0;
	struct device *dev;

	am_cam_info("%s():%d\n", __func__, ret);

	dev = &device->pdev->dev;

/* HACK */
#if 0
	clk_off(dev);

	ret = fimc_is_csi_close(&device->subdev_csi);
	if (ret) {
		am_cam_err("fimc_is_csi_close fail \n");
	}

	i2c_deinit(dev);

	sensor_power_off(device);
#endif

#if defined(CONFIG_PM_DEVFREQ)
	pm_qos_remove_request(&exynos_isp_qos_cam);
	pm_qos_remove_request(&exynos_isp_qos_int);
#endif
#if defined(CONFIG_PM_RUNTIME)
	ret = pm_runtime_put_sync(device->dev);
	if (ret)
		am_cam_err("runtime put_sync is fail(%d)", ret);
#else
#endif
	return ret;
}

static int am_cam_suspend(struct device *dev)
{
	return 0;
}

static int am_cam_resume(struct device *dev)
{
	return 0;
}

static int am_cam_runtime_suspend(struct device *dev)
{
	return 0;
}

static int am_cam_runtime_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops am_cam_pm_ops = {
	.suspend		= am_cam_suspend,
	.resume			= am_cam_resume,
	.runtime_suspend	= am_cam_runtime_suspend,
	.runtime_resume		= am_cam_runtime_resume,
};

static const struct of_device_id exynos_am_cam_match[] = {
	{
		.compatible = "samsung,exynos-am-cam",
	},
	{}
};
MODULE_DEVICE_TABLE(of, exynos_am_cam_match);

static struct platform_driver am_cam_driver = {
	.probe		= am_cam_device_probe,
	.remove		= am_cam_device_remove,
	.driver = {
		.name	= "exynos-am-cam",
		.owner	= THIS_MODULE,
		.pm	= &am_cam_pm_ops,
		.of_match_table = of_match_ptr(exynos_am_cam_match)
	}
};

static int __init am_cam_init(void)
{
	int ret = platform_driver_register(&am_cam_driver);
	if (ret)
		probe_err("platform_driver_register failed: %d\n", ret);

	return ret;
}
device_initcall(am_cam_init);

static void __exit am_cam_exit(void)
{
	platform_driver_unregister(&am_cam_driver);
}
module_exit(am_cam_exit);

MODULE_AUTHOR("Pilsun Jang<pilsun.jang@samsung.com>");
MODULE_DESCRIPTION("Exynos Automotive Camera driver");
MODULE_LICENSE("GPL");
