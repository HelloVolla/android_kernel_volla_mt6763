/*
 *  Copyright (C) 2016 Richtek Technology Corp.
 *  Patrick Chang <patrick_chang@richtek.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/of.h>
#include "inc/rt5081_pmu.h"

struct rt5081_ldo_regulator_struct {
	unsigned char vol_reg;
	unsigned char vol_mask;
	unsigned char vol_shift;
	unsigned char enable_reg;
	unsigned char enable_bit;
};

static struct rt5081_ldo_regulator_struct rt5081_ldo_regulators = {
	.vol_reg = RT5081_PMU_REG_LDOVOUT,
	.vol_mask = (0x0F),
	.vol_shift = (0),
	.enable_reg = RT5081_PMU_REG_LDOVOUT,
	.enable_bit = (1 << 7),
};

#define rt5081_ldo_min_uV (1600000)
#define rt5081_ldo_max_uV (4000000)
#define rt5081_ldo_step_uV (200000)
#define rt5081_ldo_id 0
#define rt5081_ldo_type REGULATOR_VOLTAGE

struct rt5081_pmu_ldo_data {
	struct regulator_desc *desc;
	struct regulator_dev *regulator;
	struct rt5081_pmu_chip *chip;
	struct device *dev;
};

struct rt5081_pmu_ldo_platform_data {
	uint8_t cfg;
};

static irqreturn_t rt5081_pmu_ldo_oc_irq_handler(int irq, void *data)
{
	pr_info("%s: IRQ triggered\n", __func__);
	return IRQ_HANDLED;
}

static struct rt5081_pmu_irq_desc rt5081_ldo_irq_desc[] = {
	RT5081_PMU_IRQDESC(ldo_oc),
};

static void rt5081_pmu_ldo_irq_register(struct platform_device *pdev)
{
	struct resource *res;
	int i, ret = 0;

	for (i = 0; i < ARRAY_SIZE(rt5081_ldo_irq_desc); i++) {
		if (!rt5081_ldo_irq_desc[i].name)
			continue;
		res = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
				rt5081_ldo_irq_desc[i].name);
		if (!res)
			continue;
		ret = devm_request_threaded_irq(&pdev->dev, res->start, NULL,
				rt5081_ldo_irq_desc[i].irq_handler,
				IRQF_TRIGGER_FALLING,
				rt5081_ldo_irq_desc[i].name,
				platform_get_drvdata(pdev));
		if (ret < 0) {
			dev_dbg(&pdev->dev, "request %s irq fail\n", res->name);
			continue;
		}
		rt5081_ldo_irq_desc[i].irq = res->start;
	}
}

static int rt5081_ldo_list_voltage(struct regulator_dev *rdev,
		unsigned selector)
{
	int vout = 0;

	vout = rt5081_ldo_min_uV + selector * rt5081_ldo_step_uV;
	if (vout > rt5081_ldo_max_uV)
		return -EINVAL;
	return vout;
}

static int rt5081_ldo_set_voltage_sel(
		struct regulator_dev *rdev, unsigned selector)
{
	struct rt5081_pmu_ldo_data *info = rdev_get_drvdata(rdev);
	const int count = rdev->desc->n_voltages;
	u8 data;

	if (selector > count)
		return -EINVAL;

	data = (u8)selector;
	data <<= rt5081_ldo_regulators.vol_shift;

	return rt5081_pmu_reg_update_bits(info->chip,
		rt5081_ldo_regulators.vol_reg,
		rt5081_ldo_regulators.vol_mask, data);
}

static int rt5081_ldo_get_voltage_sel(struct regulator_dev *rdev)
{
	struct rt5081_pmu_ldo_data *info = rdev_get_drvdata(rdev);
	int ret;

	ret = rt5081_pmu_reg_read(info->chip,
		rt5081_ldo_regulators.vol_reg);

	if (ret < 0)
		return ret;

	return (ret&rt5081_ldo_regulators.vol_mask)>>
		rt5081_ldo_regulators.vol_shift;
}

static int rt5081_ldo_enable(struct regulator_dev *rdev)
{
	struct rt5081_pmu_ldo_data *info = rdev_get_drvdata(rdev);

	return rt5081_pmu_reg_set_bit(info->chip,
		rt5081_ldo_regulators.enable_reg,
		rt5081_ldo_regulators.enable_bit);
}

static int rt5081_ldo_disable(struct regulator_dev *rdev)
{
	struct rt5081_pmu_ldo_data *info = rdev_get_drvdata(rdev);

	return rt5081_pmu_reg_clr_bit(info->chip,
		rt5081_ldo_regulators.enable_reg,
		rt5081_ldo_regulators.enable_bit);
}

static int rt5081_ldo_is_enabled(struct regulator_dev *rdev)
{
	struct rt5081_pmu_ldo_data *info = rdev_get_drvdata(rdev);
	int ret;

	ret = rt5081_pmu_reg_read(info->chip,
		rt5081_ldo_regulators.enable_reg);
	if (ret < 0)
		return ret;
	return ret&rt5081_ldo_regulators.enable_bit ? 1 : 0;
}

static struct regulator_ops rt5081_ldo_regulator_ops = {
	.list_voltage = rt5081_ldo_list_voltage,
	.set_voltage_sel = rt5081_ldo_set_voltage_sel,
	.get_voltage_sel = rt5081_ldo_get_voltage_sel,
	.enable = rt5081_ldo_enable,
	.disable = rt5081_ldo_disable,
	.is_enabled = rt5081_ldo_is_enabled,
};

static struct regulator_desc rt5081_ldo_regulator_desc = {
	.id = 0,
	.name = "rt5081_ldo",
	.n_voltages = 13,
	.ops = &rt5081_ldo_regulator_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
};

static inline struct regulator_dev *rt5081_ldo_regulator_register(
		struct regulator_desc *desc, struct device *dev,
		struct regulator_init_data *init_data, void *driver_data)
{
	struct regulator_config config = {
		.dev = dev,
		.init_data = init_data,
		.driver_data = driver_data,
	};
	return regulator_register(desc, &config);
}

static inline int rt_parse_dt(struct device *dev,
		struct rt5081_pmu_ldo_platform_data *pdata,
		struct rt5081_pmu_ldo_platform_data *mask)
{
	struct device_node *np = dev->of_node;
	uint32_t val;

	if (of_property_read_u32(np, "ldo_oms", &val) == 0) {
		mask->cfg |= (0x1  <<  6);
		pdata->cfg |= (val  <<  6);
	}

	mask->cfg |= (0x01 << 5);
	if (of_property_read_u32(np, "ldo_vrc", &val) == 0) {
		mask->cfg |= (0x3  <<  1);
		pdata->cfg |= (val  <<  1);
		pdata->cfg |= (1  <<  5);
	}

	if (of_property_read_u32(np, "ldo_vrc_lt", &val) == 0) {
		mask->cfg = 0x3  <<  3;
		pdata->cfg = val  <<  3;
	}
	return 0;
}

static struct regulator_init_data *rt_parse_init_data(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct device_node *sub_np;
	struct regulator_init_data *init_data;

	sub_np = of_get_child_by_name(np, "rt5081_ldo");
	if (!sub_np) {
		dev_dbg(dev, "no rt5081_ldo sub node\n");
		return NULL;
	}
	init_data = of_get_regulator_init_data(dev, sub_np, NULL);
	if (init_data) {
		dev_info(dev,
			"regulator_name = %s, min_uV = %d, max_uV = %d\n",
			init_data->constraints.name,
			init_data->constraints.min_uV,
			init_data->constraints.max_uV);
	} else {
		dev_dbg(dev, "no init data rt5081_init\n");
		return NULL;
	}
	return init_data;
}

static int ldo_apply_dts(struct rt5081_pmu_chip *chip,
		struct rt5081_pmu_ldo_platform_data *pdata,
		struct rt5081_pmu_ldo_platform_data *mask)
{
	return rt5081_pmu_reg_update_bits(chip, RT5081_PMU_REG_LDOCFG,
			mask->cfg, pdata->cfg);
}

static int rt5081_pmu_ldo_probe(struct platform_device *pdev)
{
	struct rt5081_pmu_ldo_data *ldo_data;
	struct regulator_init_data *init_data = NULL;
	bool use_dt = pdev->dev.of_node;
	struct rt5081_pmu_ldo_platform_data pdata, mask;
	int ret;

	ldo_data = devm_kzalloc(&pdev->dev, sizeof(*ldo_data), GFP_KERNEL);
	if (!ldo_data)
		return -ENOMEM;

	memset(&pdata, 0, sizeof(pdata));
	memset(&mask, 0, sizeof(mask));
	if (use_dt)
		rt_parse_dt(&pdev->dev, &pdata, &mask);

	init_data = rt_parse_init_data(&pdev->dev);
	if (init_data == NULL) {
		dev_dbg(&pdev->dev, "no init data\n");
		return -EINVAL;
	}

	ldo_data->chip = dev_get_drvdata(pdev->dev.parent);
	ldo_data->dev = &pdev->dev;
	ldo_data->desc = &rt5081_ldo_regulator_desc;
	platform_set_drvdata(pdev, ldo_data);

	ret = ldo_apply_dts(ldo_data->chip, &pdata, &mask);
	if (ret < 0)
		goto probe_err;

	ldo_data->regulator = rt5081_ldo_regulator_register(ldo_data->desc,
			&pdev->dev, init_data, ldo_data);
	if (IS_ERR(ldo_data->regulator)) {
		dev_dbg(&pdev->dev, "fail to register ldo regulator %s\n",
			ldo_data->desc->name);
		goto probe_err;
	}

	rt5081_pmu_ldo_irq_register(pdev);

	dev_info(&pdev->dev, "%s successfully\n", __func__);
	return ret;
probe_err:
	dev_info(&pdev->dev, "%s: register mtk regulator failed\n", __func__);
	return ret;
}

static int rt5081_pmu_ldo_remove(struct platform_device *pdev)
{
	struct rt5081_pmu_ldo_data *ldo_data = platform_get_drvdata(pdev);

	dev_info(ldo_data->dev, "%s successfully\n", __func__);
	return 0;
}

static const struct of_device_id rt_ofid_table[] = {
	{ .compatible = "richtek,rt5081_pmu_ldo", },
	{ },
};
MODULE_DEVICE_TABLE(of, rt_ofid_table);

static const struct platform_device_id rt_id_table[] = {
	{ "rt5081_pmu_ldo", 0},
	{ },
};
MODULE_DEVICE_TABLE(platform, rt_id_table);

static struct platform_driver rt5081_pmu_ldo = {
	.driver = {
		.name = "rt5081_pmu_ldo",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(rt_ofid_table),
	},
	.probe = rt5081_pmu_ldo_probe,
	.remove = rt5081_pmu_ldo_remove,
	.id_table = rt_id_table,
};
module_platform_driver(rt5081_pmu_ldo);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Patrick Chang <patrick_chang@richtek.com>");
MODULE_DESCRIPTION("Richtek RT5081 PMU Vib LDO");
MODULE_VERSION("1.0.1_G");

/*
 * Revision Note
 * 1.0.1
 * (1) Remove force OSC on/off for enable/disable LDO
 *
 * 1.0.0
 * Initial release
 */
