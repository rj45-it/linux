// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal regulator driver for the Samsung S2MPB03 camera PMIC (7 LDOs).
 * Register layout taken from the downstream android driver.
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define S2MPB03_REG_LDO1_CTRL	0x03
#define S2MPB03_REG_LDO_SLEW1	0x0a
#define S2MPB03_REG_LDO_SLEW2	0x0b

#define S2MPB03_VSEL_MASK	0x3f
#define S2MPB03_EN_MASK		0x80

#define S2MPB03_LDO(n, _min, _step)				\
{								\
	.name = "s2mpb03-ldo" #n,				\
	.of_match = of_match_ptr("s2mpb03-ldo" #n),		\
	.regulators_node = of_match_ptr("regulators"),		\
	.id = n - 1,						\
	.ops = &s2mpb03_ops,					\
	.type = REGULATOR_VOLTAGE,				\
	.owner = THIS_MODULE,					\
	.min_uV = _min,						\
	.uV_step = _step,					\
	.n_voltages = S2MPB03_VSEL_MASK + 1,			\
	.vsel_reg = S2MPB03_REG_LDO1_CTRL + n - 1,		\
	.vsel_mask = S2MPB03_VSEL_MASK,				\
	.enable_reg = S2MPB03_REG_LDO1_CTRL + n - 1,		\
	.enable_mask = S2MPB03_EN_MASK,				\
	.enable_time = 150,					\
}

static const struct regulator_ops s2mpb03_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
};

static const struct regulator_desc s2mpb03_regs[] = {
	S2MPB03_LDO(1, 700000, 12500),
	S2MPB03_LDO(2, 700000, 12500),
	S2MPB03_LDO(3, 700000, 25000),
	S2MPB03_LDO(4, 700000, 12500),
	S2MPB03_LDO(5, 1800000, 25000),
	S2MPB03_LDO(6, 1800000, 25000),
	S2MPB03_LDO(7, 1800000, 25000),
};

static const struct regmap_config s2mpb03_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = S2MPB03_REG_LDO_SLEW2,
};

static int s2mpb03_probe(struct i2c_client *i2c)
{
	struct regulator_config cfg = { .dev = &i2c->dev };
	struct regulator_dev *rdev;
	struct regmap *map;
	int i, ret;

	map = devm_regmap_init_i2c(i2c, &s2mpb03_regmap);
	if (IS_ERR(map))
		return PTR_ERR(map);
	cfg.regmap = map;

	/* same init as downstream: LDO2 remote sense off, LDO5/6 slew, LDO2 discharge */
	ret = regmap_update_bits(map, S2MPB03_REG_LDO_SLEW1, 0x01, 0x00);
	if (ret)
		return dev_err_probe(&i2c->dev, ret, "no response from PMIC\n");
	regmap_update_bits(map, S2MPB03_REG_LDO_SLEW2, 0xf0, 0x00);
	regmap_update_bits(map, S2MPB03_REG_LDO_SLEW2, 0x02, 0x02);

	for (i = 0; i < ARRAY_SIZE(s2mpb03_regs); i++) {
		rdev = devm_regulator_register(&i2c->dev, &s2mpb03_regs[i], &cfg);
		if (IS_ERR(rdev))
			return dev_err_probe(&i2c->dev, PTR_ERR(rdev),
					     "failed to register ldo%d\n", i + 1);
	}

	return 0;
}

static const struct of_device_id s2mpb03_of_match[] = {
	{ .compatible = "samsung,s2mpb03pmic" },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mpb03_of_match);

static struct i2c_driver s2mpb03_driver = {
	.driver = {
		.name = "s2mpb03-regulator",
		.of_match_table = s2mpb03_of_match,
	},
	.probe = s2mpb03_probe,
};
module_i2c_driver(s2mpb03_driver);

MODULE_DESCRIPTION("Samsung S2MPB03 camera PMIC regulator driver");
MODULE_LICENSE("GPL");
