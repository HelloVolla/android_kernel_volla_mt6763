/*
 *  Copyright (C) 2016 Richtek Technology Corp.
 *  patrick_chang <patrick_chang@richtek.com>
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
#include <linux/of.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include "inc/rt5081_pmu.h"

struct rt5081_dsv_regulator_struct {
	unsigned char vol_reg;
	unsigned char vol_mask;
	unsigned char vol_shift;
	unsigned char enable_reg;
	unsigned char enable_bit;
};

static struct rt5081_dsv_regulator_struct rt5081_dsv_regulators[] = {
	{
		.vol_reg = RT5081_PMU_REG_DBVPOS,
		.vol_mask = (0x3F),
		.vol_shift = 0,
		.enable_reg = RT5081_PMU_REG_DBCTRL2,
		.enable_bit = (1 << 6),
	},
	{
		.vol_reg = RT5081_PMU_REG_DBVNEG,
		.vol_mask = (0x3F),
		.vol_shift = 0,
		.enable_reg = RT5081_PMU_REG_DBCTRL2,
		.enable_bit = (1 << 3),
	},
};

#define rt5081_dsv_min_uV (4000000)
#define rt5081_dsv_max_uV (6000000)
#define rt5081_dsv_step_uV (50000)

#define rt5081_dsvp_min_uV rt5081_dsv_min_uV
#define rt5081_dsvp_max_uV rt5081_dsv_max_uV
#define rt5081_dsvp_step_uV rt5081_dsv_step_uV

#define rt5081_dsvn_min_uV rt5081_dsv_min_uV
#define rt5081_dsvn_max_uV rt5081_dsv_max_uV
#define rt5081_dsvn_step_uV rt5081_dsv_step_uV

enum {
	RT5081_DSV_POS,
	RT5081_DSV_NEG,
};

struct dsv_regulator {
	struct regulator_desc *desc;
	struct regulator_dev *regulator;
};

struct rt5081_pmu_dsv_data {
	struct dsv_regulator *dsvp;
	struct dsv_regulator *dsvn;
	struct rt5081_pmu_chip *chip;
	struct device *dev;
	int id;
};

struct rt5081_pmu_dsv_platform_data {
	union {
		uint8_t raw;
		struct {
			uint8_t db_ext_en:1;
			uint8_t reserved:3;
			uint8_t db_periodic_fix:1;
			uint8_t db_single_pin:1;
			uint8_t db_freq_pm:1;
			uint8_t db_periodic_mode:1;
		};
	} db_ctrl1;

	union {
		uint8_t raw;
		struct {
			uint8_t db_startup:1;
			uint8_t db_vneg_20ms:1;
			uint8_t db_vneg_disc:1;
			uint8_t reserved:1;
			uint8_t db_vpos_20ms:1;
			uint8_t db_vpos_disc:1;
		};
	} db_ctrl2;

	union {
		uint8_t raw;
		struct {
			uint8_t vbst:6;
			uint8_t delay:2;
		} bitfield;
	} db_vbst;

	uint8_t db_vpos_slew;
	uint8_t db_vneg_slew;
};

static irqreturn_t rt5081_pmu_dsv_vneg_ocp_irq_handler(int irq, void *data)
{
	/* Use pr_info()  instead of dev_info */
	pr_info("%s: IRQ triggered\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5081_pmu_dsv_vpos_ocp_irq_handler(int irq, void *data)
{
	pr_info("%s: IRQ triggered\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5081_pmu_dsv_bst_ocp_irq_handler(int irq, void *data)
{
	pr_info("%s: IRQ triggered\n", __func__);
	return IRQ_HANDLED;
}

static irqreturn_t rt5081_pmu_dsv_vneg_scp_irq_handler(int irq, void *data)
{
	struct rt5081_pmu_dsv_data *dsv_data = data;
	int ret;

	pr_info("%s: IRQ triggered\n", __func__);
	ret = rt5081_pmu_reg_read(dsv_data->chip, RT5081_PMU_REG_DBSTAT);
	if (ret&0x40)
		regulator_notifier_call_chain(
			dsv_data->dsvn->regulator, REGULATOR_EVENT_FAIL, NULL);
	return IRQ_HANDLED;
}

static irqreturn_t rt5081_pmu_dsv_vpos_scp_irq_handler(int irq, void *data)
{
	struct rt5081_pmu_dsv_data *dsv_data = data;
	int ret;

	pr_info("%s: IRQ triggered\n", __func__);
	ret = rt5081_pmu_reg_read(dsv_data->chip, RT5081_PMU_REG_DBSTAT);
	if (ret&0x80)
		regulator_notifier_call_chain(
			dsv_data->dsvp->regulator, REGULATOR_EVENT_FAIL, NULL);
	return IRQ_HANDLED;
}

static struct rt5081_pmu_irq_desc rt5081_dsv_irq_desc[] = {
	RT5081_PMU_IRQDESC(dsv_vneg_ocp),
	RT5081_PMU_IRQDESC(dsv_vpos_ocp),
	RT5081_PMU_IRQDESC(dsv_bst_ocp),
	RT5081_PMU_IRQDESC(dsv_vneg_scp),
	RT5081_PMU_IRQDESC(dsv_vpos_scp),
};

static void rt5081_pmu_dsv_irq_register(struct platform_device *pdev)
{
	struct resource *res;
	int i, ret = 0;

	for (i = 0; i < ARRAY_SIZE(rt5081_dsv_irq_desc); i++) {
		if (!rt5081_dsv_irq_desc[i].name)
			continue;
		res = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
				rt5081_dsv_irq_desc[i].name);
		if (!res)
			continue;
		ret = devm_request_threaded_irq(&pdev->dev, res->start, NULL,
				rt5081_dsv_irq_desc[i].irq_handler,
				IRQF_TRIGGER_FALLING,
				rt5081_dsv_irq_desc[i].name,
				platform_get_drvdata(pdev));
		if (ret < 0) {
			dev_dbg(&pdev->dev, "request %s irq fail\n", res->name);
			continue;
		}
		rt5081_dsv_irq_desc[i].irq = res->start;
	}
}

static int rt5081_dsv_list_voltage(struct regulator_dev *rdev,
		unsigned selector)
{
	int vout = 0;

	vout = rt5081_dsv_min_uV + selector * rt5081_dsv_step_uV;
	if (vout > rt5081_dsv_max_uV)
		return -EINVAL;
	return vout;
}

static int rt5081_dsv_set_voltage_sel(
		struct regulator_dev *rdev, unsigned selector)
{
	struct rt5081_pmu_dsv_data *info = rdev_get_drvdata(rdev);
	const int count = rdev->desc->n_voltages;
	u8 data;

	if (selector > count)
		return -EINVAL;

	data = (u8)selector;
	data <<= rt5081_dsv_regulators[rdev->desc->id].vol_shift;

	return rt5081_pmu_reg_update_bits(info->chip,
		rt5081_dsv_regulators[rdev->desc->id].vol_reg,
		rt5081_dsv_regulators[rdev->desc->id].vol_mask, data);
}

static int rt5081_dsv_get_voltage_sel(struct regulator_dev *rdev)
{
	struct rt5081_pmu_dsv_data *info = rdev_get_drvdata(rdev);
	int ret;

	ret = rt5081_pmu_reg_read(info->chip,
		rt5081_dsv_regulators[rdev->desc->id].vol_reg);

	if (ret < 0)
		return ret;

	return (ret&rt5081_dsv_regulators[rdev->desc->id].vol_mask)>>
		rt5081_dsv_regulators[rdev->desc->id].vol_shift;
}

static int rt5081_dsv_enable(struct regulator_dev *rdev)
{
	struct rt5081_pmu_dsv_data *info = rdev_get_drvdata(rdev);

	pr_info("%s, id = %d\n", __func__, rdev->desc->id);
	return rt5081_pmu_reg_set_bit(info->chip,
		rt5081_dsv_regulators[rdev->desc->id].enable_reg,
		rt5081_dsv_regulators[rdev->desc->id].enable_bit);
}

static int rt5081_dsv_disable(struct regulator_dev *rdev)
{
	struct rt5081_pmu_dsv_data *info = rdev_get_drvdata(rdev);

	pr_info("%s, id = %d\n", __func__, rdev->desc->id);
	return rt5081_pmu_reg_clr_bit(info->chip,
		rt5081_dsv_regulators[rdev->desc->id].enable_reg,
		rt5081_dsv_regulators[rdev->desc->id].enable_bit);
}

static int rt5081_dsv_is_enabled(struct regulator_dev *rdev)
{
	struct rt5081_pmu_dsv_data *info = rdev_get_drvdata(rdev);
	int ret;

	ret = rt5081_pmu_reg_read(info->chip,
		rt5081_dsv_regulators[rdev->desc->id].enable_reg);
	if (ret < 0)
		return ret;
	return ret&rt5081_dsv_regulators[rdev->desc->id].enable_bit ? 1 : 0;
}

struct dbctrl_bitfield_desc {
	const char *name;
	uint8_t shift;
};

static const struct dbctrl_bitfield_desc dbctrl1_desc[] = {
	{ "db_ext_en", 0 },
	{ "db_periodic_fix", 4 },
	{ "db_single_pin", 5 },
	{ "db_freq_pm", 6 },
	{ "db_periodic_mode", 7 },
};

static const struct dbctrl_bitfield_desc dbctrl2_desc[] = {
	{ "db_startup", 0 },
	{ "db_vneg_20ms", 1 },
	{ "db_vneg_disc", 2 },
	{ "db_vpos_20ms", 4 },
	{ "db_vpos_disc", 5 },
};

static struct regulator_ops rt5081_dsv_regulator_ops = {
	.list_voltage = rt5081_dsv_list_voltage,
	.set_voltage_sel = rt5081_dsv_set_voltage_sel,
	.get_voltage_sel = rt5081_dsv_get_voltage_sel,
	.enable = rt5081_dsv_enable,
	.disable = rt5081_dsv_disable,
	.is_enabled = rt5081_dsv_is_enabled,
};

static struct regulator_desc rt5081_dsv_regulator_desc[] = {
	{
		.id = RT5081_DSV_POS,
		.name = "rt5081_dsv_pos",
		.n_voltages = 41,
		.ops = &rt5081_dsv_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.id = RT5081_DSV_NEG,
		.name = "rt5081_dsv_neg",
		.n_voltages = 41,
		.ops = &rt5081_dsv_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	}
};

static inline struct regulator_dev *rt5081_dsv_regulator_register(
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
		struct rt5081_pmu_dsv_platform_data *pdata,
		struct rt5081_pmu_dsv_platform_data *mask)
{
	struct device_node *np = dev->of_node;
	int i;
	uint32_t val;

	for (i = 0; i < ARRAY_SIZE(dbctrl1_desc); i++) {
		if (of_property_read_u32(np, dbctrl1_desc[i].name, &val) == 0) {
			mask->db_ctrl1.raw |= (1 << dbctrl1_desc[i].shift);
			pdata->db_ctrl1.raw |= (val << dbctrl1_desc[i].shift);
		}
	}

	for (i = 0; i < ARRAY_SIZE(dbctrl2_desc); i++) {
		if (of_property_read_u32(np, dbctrl2_desc[i].name, &val) == 0) {
			mask->db_ctrl2.raw |= (1 << dbctrl2_desc[i].shift);
			pdata->db_ctrl2.raw |= (val << dbctrl2_desc[i].shift);
		}
	}

	if (of_property_read_u32(np, "db_delay", &val) == 0) {
		mask->db_vbst.bitfield.delay = 0x3;
		pdata->db_vbst.bitfield.delay = val;
	}

	if (of_property_read_u32(np, "db_vbst", &val) == 0) {
		if (val >= 4000 && val <= 6150) {
			mask->db_vbst.bitfield.vbst = 0x3f;
			pdata->db_vbst.bitfield.vbst = (val - 4000) / 50;
		}
	}

	if (of_property_read_u32(np, "db_vpos_slew", &val) == 0) {
		mask->db_vpos_slew = 0x3  <<  6;
		pdata->db_vpos_slew = val  <<  6;
	}

	if (of_property_read_u32(np, "db_vneg_slew", &val) == 0) {
		mask->db_vneg_slew = 0x3  <<  6;
		pdata->db_vneg_slew = val  <<  6;
	}
	return 0;
}

static struct regulator_init_data *rt_parse_regulator_init_data(
				struct device *dev, const char *node_name)
{
	struct regulator_init_data *init_data;
	struct device_node *np = dev->of_node;
	struct device_node *sub_np;

	sub_np = of_get_child_by_name(np, node_name);
	if (!sub_np) {
		dev_dbg(dev, "no child node %s", node_name);
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
		dev_dbg(dev, "no init data for %s\n", node_name);
		return NULL;
	}

	return init_data;
}

static int dsv_apply_dts(struct rt5081_pmu_chip *chip,
		struct rt5081_pmu_dsv_platform_data *pdata,
		struct rt5081_pmu_dsv_platform_data *mask)
{
	int ret = 0;

	if (mask->db_ctrl1.raw)
		ret = rt5081_pmu_reg_update_bits(chip, RT5081_PMU_REG_DBCTRL1,
				mask->db_ctrl1.raw, pdata->db_ctrl1.raw);
	if (ret < 0)
		return ret;
	if (mask->db_ctrl2.raw)
		ret = rt5081_pmu_reg_update_bits(chip, RT5081_PMU_REG_DBCTRL2,
				mask->db_ctrl2.raw, pdata->db_ctrl2.raw);
	if (ret < 0)
		return ret;
	if (mask->db_vbst.raw)
		ret = rt5081_pmu_reg_update_bits(chip, RT5081_PMU_REG_DBVBST,
				mask->db_vbst.raw, pdata->db_vbst.raw);
	if (ret < 0)
		return ret;
	if (mask->db_vpos_slew)
		ret = rt5081_pmu_reg_update_bits(chip, RT5081_PMU_REG_DBVPOS,
				mask->db_vpos_slew, pdata->db_vpos_slew);
	if (ret < 0)
		return ret;
	if (mask->db_vneg_slew)
		ret = rt5081_pmu_reg_update_bits(chip, RT5081_PMU_REG_DBVNEG,
				mask->db_vneg_slew, pdata->db_vneg_slew);
	if (ret < 0)
		return ret;

	return ret;
}

static int rt5081_pmu_dsv_probe(struct platform_device *pdev)
{
	struct rt5081_pmu_dsv_data *dsv_data;
	struct regulator_init_data *init_data_p = NULL;
	struct regulator_init_data *init_data_n = NULL;
	bool use_dt = pdev->dev.of_node;
	struct rt5081_pmu_dsv_platform_data pdata, mask;
	int ret;

	dev_info(&pdev->dev, "Probing....\n");
	dsv_data = devm_kzalloc(&pdev->dev, sizeof(*dsv_data), GFP_KERNEL);
	if (!dsv_data)
		return -ENOMEM;
	dsv_data->dsvp = devm_kzalloc(&pdev->dev,
			sizeof(struct dsv_regulator), GFP_KERNEL);
	dsv_data->dsvn = devm_kzalloc(&pdev->dev,
			sizeof(struct dsv_regulator), GFP_KERNEL);

	memset(&pdata, 0, sizeof(pdata));
	memset(&mask, 0, sizeof(mask));

	if (use_dt)
		rt_parse_dt(&pdev->dev, &pdata, &mask);
	init_data_p = rt_parse_regulator_init_data(&pdev->dev, "rt5081_dsvp");
	init_data_n = rt_parse_regulator_init_data(&pdev->dev, "rt5081_dsvn");
	if ((init_data_p == NULL) || (init_data_n == NULL)) {
		dev_dbg(&pdev->dev, "no init data\n");
		return -EINVAL;
	}

	dsv_data->chip = dev_get_drvdata(pdev->dev.parent);
	dsv_data->dev = &pdev->dev;
	dsv_data->dsvp->desc = &rt5081_dsv_regulator_desc[RT5081_DSV_POS];
	dsv_data->dsvn->desc = &rt5081_dsv_regulator_desc[RT5081_DSV_NEG];
	platform_set_drvdata(pdev, dsv_data);

	ret = dsv_apply_dts(dsv_data->chip, &pdata, &mask);
	if (ret < 0)
		goto reg_apply_dts_fail;

	dsv_data->dsvp->regulator = rt5081_dsv_regulator_register(
		dsv_data->dsvp->desc, &pdev->dev, init_data_p, dsv_data);
	if (IS_ERR(dsv_data->dsvp->regulator)) {
		dev_dbg(&pdev->dev, "fail to register dsv regulator %s\n",
			dsv_data->dsvp->desc->name);
		goto reg_dsvp_register_fail;
	}

	dsv_data->dsvn->regulator = rt5081_dsv_regulator_register(
		dsv_data->dsvn->desc, &pdev->dev, init_data_n, dsv_data);
	if (IS_ERR(dsv_data->dsvn->regulator)) {
		dev_dbg(&pdev->dev, "fail to register dsv regulator %s\n",
			dsv_data->dsvp->desc->name);
		goto reg_dsvn_register_fail;
	}

	rt5081_pmu_dsv_irq_register(pdev);
	dev_info(&pdev->dev, "%s successfully\n", __func__);
	return ret;
reg_apply_dts_fail:
reg_dsvn_register_fail:
	regulator_unregister(dsv_data->dsvp->regulator);
reg_dsvp_register_fail:
	dev_info(&pdev->dev, "%s failed\n", __func__);
	return ret;
}

static int rt5081_pmu_dsv_remove(struct platform_device *pdev)
{
	struct rt5081_pmu_dsv_data *dsv_data = platform_get_drvdata(pdev);

	dev_info(dsv_data->dev, "%s successfully\n", __func__);
	return 0;
}

static const struct of_device_id rt_ofid_table[] = {
	{ .compatible = "richtek,rt5081_pmu_dsv", },
	{ },
};
MODULE_DEVICE_TABLE(of, rt_ofid_table);

static const struct platform_device_id rt_id_table[] = {
	{ "rt5081_pmu_dsv", 0},
	{ },
};
MODULE_DEVICE_TABLE(platform, rt_id_table);

static struct platform_driver rt5081_pmu_dsv = {
	.driver = {
		.name = "rt5081_pmu_dsv",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(rt_ofid_table),
	},
	.probe = rt5081_pmu_dsv_probe,
	.remove = rt5081_pmu_dsv_remove,
	.id_table = rt_id_table,
};
module_platform_driver(rt5081_pmu_dsv);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Patrick Chang <patrick_chang@richtek.com>");
MODULE_DESCRIPTION("Richtek RT5081 PMU DSV");
MODULE_VERSION("1.0.0_G");
