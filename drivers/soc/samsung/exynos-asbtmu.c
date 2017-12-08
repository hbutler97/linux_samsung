/*
 * exynos_tmu.c - Samsung EXYNOS TMU (Thermal Management Unit)
 *
 *  Copyright (C) 2014 Samsung Electronics
 *  Bartlomiej Zolnierkiewicz <b.zolnierkie@samsung.com>
 *  Lukasz Majewski <l.majewski@samsung.com>
 *
 *  Copyright (C) 2011 Samsung Electronics
 *  Donggeun Kim <dg77.kim@samsung.com>
 *  Amit Daniel Kachhap <amit.kachhap@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/threads.h>
#include <linux/slab.h>
#include <linux/platform_data/exynos_thermal.h>

/* Exynos generic registers */
#define EXYNOS_TMU_REG_TRIMINFO		0x0
#define EXYNOS_TMU_REG_TRIMINFO1	0x4
#define EXYNOS_TMU_REG_CONTROL		0x20
#define EXYNOS_TMU_REG_STATUS		0x28
#define EXYNOS_TMU_REG_CURRENT_TEMP	0x40
#define EXYNOS_TMU_REG_INTEN		0x110
#define EXYNOS_TMU_REG_INTSTAT		0x74
#define EXYNOS_TMU_REG_INTCLEAR		0x78

#define EXYNOS_TMU_REF_VOLTAGE_SHIFT	24
#define EXYNOS_TMU_REF_VOLTAGE_MASK	0x1f
#define EXYNOS_TMU_BUF_SLOPE_SEL_MASK	0xf
#define EXYNOS_TMU_BUF_SLOPE_SEL_SHIFT	8
#define EXYNOS_TMU_CORE_EN_SHIFT	0

#define EXYNOS_TMU_TRIP_MODE_SHIFT	13
#define EXYNOS_TMU_TRIP_MODE_MASK	0x7
#define EXYNOS_TMU_THERM_TRIP_EN_SHIFT	12

#define EXYNOS_TMU_INTEN_RISE0_SHIFT	0
#define EXYNOS_TMU_INTEN_FALL0_SHIFT	16

#define EXYNOS_EMUL_TIME	0x57F0
#define EXYNOS_EMUL_TIME_MASK	0xffff
#define EXYNOS_EMUL_TIME_SHIFT	16
#define EXYNOS_EMUL_DATA_SHIFT	7
#define EXYNOS_EMUL_DATA_MASK	0x1FF
#define EXYNOS_EMUL_ENABLE	0x1

#define EXYNOS_THD_TEMP_RISE7_6			0x50
#define EXYNOS_THD_TEMP_FALL7_6			0x60
#define EXYNOS_TMU_INTEN_RISE0_SHIFT		0
#define EXYNOS_TMU_INTEN_RISE1_SHIFT		1
#define EXYNOS_TMU_INTEN_RISE2_SHIFT		2
#define EXYNOS_TMU_INTEN_RISE3_SHIFT		3
#define EXYNOS_TMU_INTEN_RISE4_SHIFT		4
#define EXYNOS_TMU_INTEN_RISE5_SHIFT		5
#define EXYNOS_TMU_INTEN_RISE6_SHIFT		6
#define EXYNOS_TMU_INTEN_RISE7_SHIFT		7

#define EXYNOS_TMU_CALIB_SEL_SHIFT		(23)
#define EXYNOS_TMU_CALIB_SEL_MASK		(0x1)
#define EXYNOS_TMU_TEMP_MASK			(0x1ff)
#define EXYNOS_TMU_TRIMINFO_85_P0_SHIFT		(9)
#define EXYNOS_TRIMINFO_ONE_POINT_TRIMMING	(0)
#define EXYNOS_TRIMINFO_TWO_POINT_TRIMMING	(1)
#define EXYNOS_TMU_T_BUF_VREF_SEL_SHIFT		(18)
#define EXYNOS_TMU_T_BUF_VREF_SEL_MASK		(0x1F)
#define EXYNOS_TMU_T_BUF_SLOPE_SEL_SHIFT	(18)
#define EXYNOS_TMU_T_BUF_SLOPE_SEL_MASK		(0xF)

#define EXYNOS_TMU_REG_INTPEND			(0x118)
#define EXYNOS_TMU_REG_EMUL_CON			(0x160)

#define MCELSIUS	1000

struct exynos_asb_tmu_platform_data {
	u8 gain;
	u8 reference_voltage;

	u32 efuse_value;
	u8 first_point_trim;
	u8 second_point_trim;
	u8 default_temp_offset;

	u32 cal_type;
	u32 cal_mode;
};


/**
 * struct exynos_asb_tmu_data : A structure to hold the private data of the TMU
	driver
 * @id: identifier of the one instance of the TMU controller.
 * @pdata: pointer to the tmu platform/configuration data
 * @base: base address of the single instance of the TMU controller.
 * @base_second: base address of the common registers of the TMU controller.
 * @irq: irq number of the TMU controller.
 * @soc: id of the SOC type.
 * @irq_work: pointer to the irq work structure.
 * @lock: lock to implement synchronization.
 * @temp_error1: fused value of the first point trim.
 * @temp_error2: fused value of the second point trim.
 * @regulator: pointer to the TMU regulator structure.
 * @reg_conf: pointer to structure to register with core thermal.
 * @tmu_initialize: SoC specific TMU initialization method
 * @tmu_control: SoC specific TMU control method
 * @tmu_read: SoC specific TMU temperature read method
 * @tmu_set_emulation: SoC specific TMU emulation setting method
 * @tmu_clear_irqs: SoC specific TMU interrupts clearing method
 */
struct exynos_asb_tmu_data {
	int id;
	/* Throttle hotplug related variables */
	struct exynos_asb_tmu_platform_data *pdata;
	void __iomem *base;
	struct mutex lock;
	u16 temp_error1, temp_error2;

	int (*tmu_initialize)(struct platform_device *pdev);
	void (*tmu_control)(struct platform_device *pdev, bool on);
	int (*tmu_read)(struct exynos_asb_tmu_data *data);
};

/*
 * Calculate a temperature value from a temperature code.
 * The unit of the temperature is degree Celsius.
 */
static int code_to_temp(struct exynos_asb_tmu_data *data, u16 temp_code)
{
	struct exynos_asb_tmu_platform_data *pdata = data->pdata;
	int temp;

	switch (pdata->cal_type) {
	case TYPE_TWO_POINT_TRIMMING:
		temp = (temp_code - data->temp_error1) *
			(pdata->second_point_trim - pdata->first_point_trim) /
			(data->temp_error2 - data->temp_error1) +
			pdata->first_point_trim;
		break;
	case TYPE_ONE_POINT_TRIMMING:
		temp = temp_code - data->temp_error1 + pdata->first_point_trim;
		break;
	default:
		temp = temp_code - pdata->default_temp_offset;
		break;
	}

	return temp;
}

static int exynos_asb_tmu_initialize(struct platform_device *pdev)
{
	struct exynos_asb_tmu_data *data = platform_get_drvdata(pdev);
	int ret;

	mutex_lock(&data->lock);
	ret = data->tmu_initialize(pdev);
	mutex_unlock(&data->lock);

	return ret;
}

static u32 get_con_reg(struct exynos_asb_tmu_data *data, u32 con)
{
	struct exynos_asb_tmu_platform_data *pdata = data->pdata;

	con &= ~(EXYNOS_TMU_REF_VOLTAGE_MASK << EXYNOS_TMU_REF_VOLTAGE_SHIFT);
	con |= pdata->reference_voltage << EXYNOS_TMU_REF_VOLTAGE_SHIFT;

	con &= ~(EXYNOS_TMU_BUF_SLOPE_SEL_MASK << EXYNOS_TMU_BUF_SLOPE_SEL_SHIFT);
	con |= (pdata->gain << EXYNOS_TMU_BUF_SLOPE_SEL_SHIFT);

	return con;
}

static void exynos_asb_tmu_control(struct platform_device *pdev, bool on)
{
	struct exynos_asb_tmu_data *data = platform_get_drvdata(pdev);

	mutex_lock(&data->lock);
	data->tmu_control(pdev, on);
	mutex_unlock(&data->lock);
}

static int exynos8890_tmu_initialize(struct platform_device *pdev)
{
	struct exynos_asb_tmu_data *data = platform_get_drvdata(pdev);
	struct exynos_asb_tmu_platform_data *pdata = data->pdata;
	unsigned int trim_info, tmu_con;

	/* Disable TMU core */
	tmu_con = readl(data->base + EXYNOS_TMU_REG_CONTROL);
	tmu_con &= ~(1 << EXYNOS_TMU_CORE_EN_SHIFT);
	writel(tmu_con, data->base + EXYNOS_TMU_REG_CONTROL);

	/* Check tmu core ready status */
	trim_info = readl(data->base + EXYNOS_TMU_REG_TRIMINFO);

	/* Check thermal calibration type */
	pdata->cal_type = (trim_info >> EXYNOS_TMU_CALIB_SEL_SHIFT)
			& EXYNOS_TMU_CALIB_SEL_MASK;

	/* Check temp_error1 and error2 value */
	data->temp_error1 = trim_info & EXYNOS_TMU_TEMP_MASK;
	data->temp_error2 = (trim_info >> EXYNOS_TMU_TRIMINFO_85_P0_SHIFT)
				& EXYNOS_TMU_TEMP_MASK;

	if (!data->temp_error1)
		data->temp_error1 = pdata->efuse_value & EXYNOS_TMU_TEMP_MASK;
	if (!data->temp_error2)
		data->temp_error2 = (pdata->efuse_value >>
					EXYNOS_TMU_TRIMINFO_85_P0_SHIFT)
					& EXYNOS_TMU_TEMP_MASK;

	return 0;
}

static void exynos8890_tmu_control(struct platform_device *pdev, bool on)
{
	struct exynos_asb_tmu_data *data = platform_get_drvdata(pdev);
	unsigned int con, trim_info, trim_info1;
	unsigned int t_buf_vref_sel, t_buf_slope_sel;

	trim_info = readl(data->base + EXYNOS_TMU_REG_TRIMINFO);
	trim_info1 = readl(data->base + EXYNOS_TMU_REG_TRIMINFO1);

	/* Save fuse buf_vref_sel, calib_sel value to TRIMINFO and 1 register */
	t_buf_vref_sel = (trim_info >> EXYNOS_TMU_T_BUF_VREF_SEL_SHIFT)
				& (EXYNOS_TMU_T_BUF_VREF_SEL_MASK);
	t_buf_slope_sel = (trim_info1 >> EXYNOS_TMU_T_BUF_SLOPE_SEL_SHIFT)
				& (EXYNOS_TMU_T_BUF_SLOPE_SEL_MASK);

	con = get_con_reg(data, readl(data->base + EXYNOS_TMU_REG_CONTROL));

	if (on) {
		con |= (t_buf_vref_sel << EXYNOS_TMU_REF_VOLTAGE_SHIFT);
		con |= (t_buf_slope_sel << EXYNOS_TMU_BUF_SLOPE_SEL_SHIFT);
		con |= (1 << EXYNOS_TMU_CORE_EN_SHIFT);
	} else {
		con &= ~(1 << EXYNOS_TMU_CORE_EN_SHIFT);
	}

	writel(con, data->base + EXYNOS_TMU_REG_CONTROL);
}

static int exynos8890_tmu_read(struct exynos_asb_tmu_data *data)
{
	u32 code;
	int temp;

	code = readl(data->base + EXYNOS_TMU_REG_CURRENT_TEMP) &
		EXYNOS_TMU_TEMP_MASK;
	temp = code_to_temp(data, code);

	return temp;
}

static const struct of_device_id exynos_asb_tmu_match[] = {
	{ .compatible = "samsung,exynos8890-tmu", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, exynos_asb_tmu_match);

static int exynos_of_sensor_conf(struct device_node *np,
				 struct exynos_asb_tmu_platform_data *pdata)
{
	u32 value;
	int ret;

	of_node_get(np);

	ret = of_property_read_u32(np, "samsung,tmu_gain", &value);
	pdata->gain = (u8)value;
	of_property_read_u32(np, "samsung,tmu_reference_voltage", &value);
	pdata->reference_voltage = (u8)value;

	of_property_read_u32(np, "samsung,tmu_efuse_value",
			     &pdata->efuse_value);

	of_property_read_u32(np, "samsung,tmu_first_point_trim", &value);
	pdata->first_point_trim = (u8)value;
	of_property_read_u32(np, "samsung,tmu_second_point_trim", &value);
	pdata->second_point_trim = (u8)value;
	of_property_read_u32(np, "samsung,tmu_default_temp_offset", &value);
	pdata->default_temp_offset = (u8)value;

	of_property_read_u32(np, "samsung,tmu_cal_type", &pdata->cal_type);

	of_node_put(np);
	return 0;
}

static int exynos_map_dt_data(struct platform_device *pdev)
{
	struct exynos_asb_tmu_data *data = platform_get_drvdata(pdev);
	struct exynos_asb_tmu_platform_data *pdata;
	struct resource res;

	if (!data || !pdev->dev.of_node)
		return -ENODEV;

	data->id = of_alias_get_id(pdev->dev.of_node, "tmuctrl");
	if (data->id < 0)
		data->id = 0;

	if (of_address_to_resource(pdev->dev.of_node, 0, &res)) {
		dev_err(&pdev->dev, "failed to get Resource 0\n");
		return -ENODEV;
	}

	data->base = devm_ioremap(&pdev->dev, res.start, resource_size(&res));
	if (!data->base) {
		dev_err(&pdev->dev, "Failed to ioremap memory\n");
		return -EADDRNOTAVAIL;
	}

	pdata = devm_kzalloc(&pdev->dev,
			     sizeof(struct exynos_asb_tmu_platform_data),
			     GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	exynos_of_sensor_conf(pdev->dev.of_node, pdata);
	data->pdata = pdata;

	data->tmu_initialize = exynos8890_tmu_initialize;
	data->tmu_control = exynos8890_tmu_control;
	data->tmu_read = exynos8890_tmu_read;

	return 0;
}

static ssize_t
exynos_thermal_temp(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct exynos_asb_tmu_data *data = platform_get_drvdata(pdev);
	int temp;
	int len = 0;

	temp = data->tmu_read(data);

	len += snprintf(&buf[len], PAGE_SIZE, "%d\n", temp);

	return len;
}

static DEVICE_ATTR(temp, S_IRUGO, exynos_thermal_temp, NULL);

static struct attribute *exynos_thermal_sensor_attributes[] = {
	&dev_attr_temp.attr,
	NULL
};

static const struct attribute_group exynos_thermal_sensor_attr_group = {
	.attrs = exynos_thermal_sensor_attributes,
};

static int exynos_asb_tmu_probe(struct platform_device *pdev)
{
	struct exynos_asb_tmu_data *data;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(struct exynos_asb_tmu_data),
					GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	platform_set_drvdata(pdev, data);
	mutex_init(&data->lock);

	ret = exynos_map_dt_data(pdev);
	if (ret)
		goto err_sensor;

	ret = exynos_asb_tmu_initialize(pdev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to initialize TMU\n");
		goto err_sensor;
	}

	exynos_asb_tmu_control(pdev, true);

	ret = sysfs_create_group(&pdev->dev.kobj, &exynos_thermal_sensor_attr_group);
	if (ret)
		dev_err(&pdev->dev, "cannot create thermal sensor attributes\n");

	return 0;

err_sensor:
	return ret;
}

static int exynos_asb_tmu_remove(struct platform_device *pdev)
{
	exynos_asb_tmu_control(pdev, false);

	return 0;
}

static struct platform_driver exynos_asb_tmu_driver = {
	.driver = {
		.name   = "exynos-asbtmu",
		.of_match_table = exynos_asb_tmu_match,
	},
	.probe = exynos_asb_tmu_probe,
	.remove	= exynos_asb_tmu_remove,
};

module_platform_driver(exynos_asb_tmu_driver);

MODULE_DESCRIPTION("EXYNOS ASB TMU Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:exynos-asbtmu");
