/*
 * drivers/regulator/ricoh568-regulator.c
 *
 * Regulator driver for RICOH568 power management chip.
 *
 * Copyright (C) 2012-2016 RICOH COMPANY,LTD
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
 
/*#define DEBUG			1*/
/*#define VERBOSE_DEBUG		1*/
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/ricoh568.h>
#include <linux/regulator/ricoh568-regulator.h>
#include <linux/of.h>
#include <linux/regulator/of_regulator.h>

struct ricoh568_regulator {
	int		id;
	int		sleep_id;
	/* Regulator register address.*/
	u8		reg_en_reg;
	u8		en_bit;
	u8		reg_disc_reg;
	u8		disc_bit;
	u8		vout_reg;
	u8		vout_mask;
	u8		vout_reg_cache;
	u8		sleep_reg;

	/* chip constraints on regulator behavior */
	int			min_uV;
	int			max_uV;
	int			step_uV;
	int			nsteps;

	/* regulator specific turn-on delay */
	u16			delay;

	/* used by regulator core */
	struct regulator_desc	desc;

	/* Device */
	struct device		*dev;
};

//static unsigned int ricoh568_suspend_status = 0;

static inline struct device *to_ricoh568_dev(struct regulator_dev *rdev)
{
	return rdev_get_dev(rdev)->parent->parent;
}
/*
static int ricoh568_regulator_enable_time(struct regulator_dev *rdev)
{
	struct ricoh568_regulator *ri = rdev_get_drvdata(rdev);

	return ri->delay;
}
*/
static int ricoh568_reg_is_enabled(struct regulator_dev *rdev)
{
	struct ricoh568_regulator *ri = rdev_get_drvdata(rdev);
	struct device *parent = to_ricoh568_dev(rdev);
	uint8_t control;
	int ret;

	ret = ricoh568_read(parent, ri->reg_en_reg, &control);
	if (ret < 0) {
		dev_err(&rdev->dev, "Error in reading the control register\n");
		return ret;
	}
	return (((control >> ri->en_bit) & 1) == 1);
}

static int ricoh568_reg_enable(struct regulator_dev *rdev)
{
	struct ricoh568_regulator *ri = rdev_get_drvdata(rdev);
	struct device *parent = to_ricoh568_dev(rdev);
	int ret;
	ret = ricoh568_set_bits(parent, ri->reg_en_reg, (1 << ri->en_bit));
	if (ret < 0) {
		dev_err(&rdev->dev, "Error in updating the STATE register\n");
		return ret;
	}
	udelay(ri->delay);
	return ret;
}

static int ricoh568_reg_disable(struct regulator_dev *rdev)
{
	struct ricoh568_regulator *ri = rdev_get_drvdata(rdev);
	struct device *parent = to_ricoh568_dev(rdev);
	int ret;
	ret = ricoh568_clr_bits(parent, ri->reg_en_reg, (1 << ri->en_bit));
	if (ret < 0)
		dev_err(&rdev->dev, "Error in updating the STATE register\n");

	return ret;
}

static int ricoh568_list_voltage(struct regulator_dev *rdev, unsigned index)
{
	struct ricoh568_regulator *ri = rdev_get_drvdata(rdev);

	return ri->min_uV + (ri->step_uV * index);
}

static int __ricoh568_set_s_voltage(struct device *parent,
		struct ricoh568_regulator *ri, int min_uV, int max_uV)
{
	int vsel;
	int ret;

	if ((min_uV < ri->min_uV) || (max_uV > ri->max_uV))
		return -EDOM;

	vsel = (min_uV - ri->min_uV + ri->step_uV - 1)/ri->step_uV;
	if (vsel > ri->nsteps)
		return -EDOM;

	ret = ricoh568_update(parent, ri->sleep_reg, vsel, ri->vout_mask);
	if (ret < 0)
		dev_err(ri->dev, "Error in writing the sleep register\n");
	return ret;
}

static int __ricoh568_set_voltage(struct device *parent,
		struct ricoh568_regulator *ri, int min_uV, int max_uV,
		unsigned *selector)
{
	int vsel;
	int ret;
	uint8_t vout_val;

	if ((min_uV < ri->min_uV) || (max_uV > ri->max_uV))
		return -EDOM;

	vsel = (min_uV - ri->min_uV + ri->step_uV - 1)/ri->step_uV;
	if (vsel > ri->nsteps)
		return -EDOM;

	if (selector)
		*selector = vsel;

	vout_val = (ri->vout_reg_cache & ~ri->vout_mask) |
				(vsel & ri->vout_mask);
	ret = ricoh568_write(parent, ri->vout_reg, vout_val);
	if (ret < 0)
		dev_err(ri->dev, "Error in writing the Voltage register\n");
	else
		ri->vout_reg_cache = vout_val;

	return ret;
}

static int ricoh568_set_voltage(struct regulator_dev *rdev,
		int min_uV, int max_uV, unsigned *selector)
{
	struct ricoh568_regulator *ri = rdev_get_drvdata(rdev);
	struct device *parent = to_ricoh568_dev(rdev);

//	if(ricoh568_suspend_status)
//		return -EBUSY;

	return __ricoh568_set_voltage(parent, ri, min_uV, max_uV, selector);
}

static int ricoh568_set_suspend_voltage(struct regulator_dev *rdev,
		int uV)
{
	struct ricoh568_regulator *ri = rdev_get_drvdata(rdev);
	struct device *parent = to_ricoh568_dev(rdev);

	return __ricoh568_set_s_voltage(parent, ri, uV, uV);
}

static int ricoh568_get_voltage(struct regulator_dev *rdev)
{
	struct ricoh568_regulator *ri = rdev_get_drvdata(rdev);
	struct device *parent = to_ricoh568_dev(rdev);
	uint8_t vsel;
	int ret;

	ret = ricoh568_read(parent, ri->vout_reg, &vsel);
	return ricoh568_list_voltage(rdev,vsel);
}

static unsigned int ricoh568_dcdc_get_mode(struct regulator_dev *rdev)
{
	struct ricoh568_regulator *ri = rdev_get_drvdata(rdev);
	struct device *parent = to_ricoh568_dev(rdev);
	int ret;
	uint8_t control;
	u8 mask = 0x30;
	
	ret = ricoh568_read(parent, ri->reg_en_reg,&control);
        if (ret < 0) {
                return ret;
        }
	control=(control & mask) >> 4;
	switch (control) {
	case 1:
		return REGULATOR_MODE_FAST;
	case 0:
		return REGULATOR_MODE_NORMAL;
	case 2:
		return REGULATOR_MODE_STANDBY;
	case 3:
		return REGULATOR_MODE_NORMAL;
	default:
		return -1;
	}

}
static int ricoh568_dcdc_set_mode(struct regulator_dev *rdev, unsigned int mode)
{
	struct ricoh568_regulator *ri = rdev_get_drvdata(rdev);
	struct device *parent = to_ricoh568_dev(rdev);
	int ret;
	uint8_t control;
	
	ret = ricoh568_read(parent, ri->reg_en_reg,&control);
	switch(mode)
	{
	case REGULATOR_MODE_FAST:
		return ricoh568_write(parent, ri->reg_en_reg, ((control & 0xcf) | 0x10));
	case REGULATOR_MODE_NORMAL:
		return ricoh568_write(parent, ri->reg_en_reg, (control & 0xcf));
	case REGULATOR_MODE_STANDBY:
		return ricoh568_write(parent, ri->reg_en_reg, ((control & 0xcf) | 0x20));	
	default:
		printk("error:pmu_568 only powersave pwm psm mode\n");
		return -EINVAL;
	}
	

}

static int ricoh568_dcdc_set_voltage_time_sel(struct regulator_dev *rdev,   unsigned int old_selector,
				     unsigned int new_selector)
{
	int old_volt, new_volt;
	
	old_volt = ricoh568_list_voltage(rdev, old_selector);
	if (old_volt < 0)
		return old_volt;
	
	new_volt = ricoh568_list_voltage(rdev, new_selector);
	if (new_volt < 0)
		return new_volt;

	return DIV_ROUND_UP(abs(old_volt - new_volt)*2, 14000);
}
static int ricoh568_dcdc_set_suspend_mode(struct regulator_dev *rdev, unsigned int mode)
{
	struct ricoh568_regulator *ri = rdev_get_drvdata(rdev);
	struct device *parent = to_ricoh568_dev(rdev);
	int ret;
	uint8_t control;
	
	ret = ricoh568_read(parent, ri->reg_en_reg,&control);
	switch(mode)
	{
	case REGULATOR_MODE_FAST:
		return ricoh568_write(parent, ri->reg_en_reg, ((control & 0x3f) | 0x40));
	case REGULATOR_MODE_NORMAL:
		return ricoh568_write(parent, ri->reg_en_reg, (control & 0x3f));
	case REGULATOR_MODE_STANDBY:
		return ricoh568_write(parent, ri->reg_en_reg, ((control & 0x3f) | 0x80));	
	default:
		printk("error:pmu_568 only powersave pwm psm mode\n");
		return -EINVAL;
	}
	

}
static int ricoh568_reg_suspend_enable(struct regulator_dev *rdev)
{
	struct ricoh568_regulator *ri = rdev_get_drvdata(rdev);
	struct device *parent = to_ricoh568_dev(rdev);
	int ret;
	ret = ricoh568_set_bits(parent, (0x16 + ri->id), (0xf << 0));
	if (ret < 0) {
		dev_err(&rdev->dev, "Error in updating the STATE register\n");
		return ret;
	}
	udelay(ri->delay);
	return ret;
}

static int ricoh568_reg_suspend_disable(struct regulator_dev *rdev)
{
	struct ricoh568_regulator *ri = rdev_get_drvdata(rdev);
	struct device *parent = to_ricoh568_dev(rdev);
	int ret;
	ret = ricoh568_clr_bits(parent, (0x16 + ri->id), (0xf <<0));
	if (ret < 0)
		dev_err(&rdev->dev, "Error in updating the STATE register\n");

	return ret;
}

static struct regulator_ops ricoh568_ops = {
	.list_voltage			= ricoh568_list_voltage,
	.set_voltage			= ricoh568_set_voltage,
	.get_voltage			= ricoh568_get_voltage,
	.set_suspend_voltage = ricoh568_set_suspend_voltage,
	.set_voltage_time_sel = ricoh568_dcdc_set_voltage_time_sel,
	.get_mode = ricoh568_dcdc_get_mode,
	.set_mode = ricoh568_dcdc_set_mode,
	.enable				= ricoh568_reg_enable,
	.disable				= ricoh568_reg_disable,
	.set_suspend_mode = ricoh568_dcdc_set_suspend_mode,
	.set_suspend_enable				= ricoh568_reg_suspend_enable,
	.set_suspend_disable				= ricoh568_reg_suspend_disable,
	.is_enabled			= ricoh568_reg_is_enabled,
};

#define RICOH568_REG(_id, _en_reg, _en_bit, _disc_reg, _disc_bit, _vout_reg, \
		_vout_mask, _ds_reg, _min_uv, _max_uv, _step_uV, _nsteps,    \
		_ops, _delay)		\
{								\
	.reg_en_reg	= _en_reg,				\
	.en_bit		= _en_bit,				\
	.reg_disc_reg	= _disc_reg,				\
	.disc_bit	= _disc_bit,				\
	.vout_reg	= _vout_reg,				\
	.vout_mask	= _vout_mask,				\
	.sleep_reg	= _ds_reg,				\
	.min_uV		= _min_uv,			\
	.max_uV		= _max_uv ,			\
	.step_uV	= _step_uV,				\
	.nsteps		= _nsteps,				\
	.delay		= _delay,				\
	.id		= RICOH568_ID_##_id,			\
	.sleep_id	= RICOH568_DS_##_id,			\
	.desc = {						\
		.name = ricoh568_rails(_id),			\
		.id = RICOH568_ID_##_id,			\
		.n_voltages = _nsteps,				\
		.ops = &_ops,					\
		.type = REGULATOR_VOLTAGE,			\
		.owner = THIS_MODULE,				\
	},							\
}

static struct ricoh568_regulator ricoh568_regulator_data[] = {
  	RICOH568_REG(DC1, 0x2C, 0, 0x2C, 1, 0x36, 0xFF, 0x3B,
			600000, 3500000, 12500, 0xE8, ricoh568_ops, 500),

  	RICOH568_REG(DC2, 0x2E, 0, 0x2E, 1, 0x37, 0xFF, 0x3C,
			600000, 3500000, 12500, 0xE8, ricoh568_ops, 500),

  	RICOH568_REG(DC3, 0x30, 0, 0x30, 1, 0x38, 0xFF, 0x3D,
			600000, 3500000, 12500, 0xE8, ricoh568_ops, 500),

  	RICOH568_REG(DC4, 0x32, 0, 0x32, 1, 0x39, 0xFF, 0x3E,
			600000, 3500000, 12500, 0xE8, ricoh568_ops, 500),
			
  	RICOH568_REG(LDO1, 0x44, 0, 0x46, 0, 0x4C, 0x7F, 0x58,
			900000, 3500000, 25000, 0x68, ricoh568_ops, 500),

	RICOH568_REG(LDO2, 0x44, 1, 0x46, 1, 0x4D, 0x7F, 0x59,
			900000, 3500000, 25000, 0x68, ricoh568_ops, 500),

  	RICOH568_REG(LDO3, 0x44, 2, 0x46, 2, 0x4E, 0x7F, 0x5A,
			900000, 3500000, 25000, 0x68, ricoh568_ops, 500),

  	RICOH568_REG(LDO4, 0x44, 3, 0x46, 3, 0x4F, 0x7F, 0x5B,
			900000, 3500000, 25000, 0x68, ricoh568_ops, 500),

  	RICOH568_REG(LDO5, 0x44, 4, 0x46, 4, 0x50, 0x7F, 0x5C,
			600000, 3500000, 25000, 0x74, ricoh568_ops, 500),
};

static inline struct ricoh568_regulator *find_regulator_info(int id)
{
	struct ricoh568_regulator *ri;
	int i;

	for (i = 0; i < ARRAY_SIZE(ricoh568_regulator_data); i++) {
		ri = &ricoh568_regulator_data[i];
		if (ri->desc.id == id)
			return ri;
	}
	return NULL;
}

#ifdef CONFIG_OF
static struct of_regulator_match ricoh568_regulator_matches[] = {
	{ .name	= "ricoh568_dc1",},
	{ .name = "ricoh568_dc2",},
	{ .name = "ricoh568_dc3",},
	{ .name = "ricoh568_dc4",},
	{ .name = "ricoh568_ldo1",},
	{ .name = "ricoh568_ldo2",},
	{ .name = "ricoh568_ldo3",},
	{ .name = "ricoh568_ldo4",},
	{ .name = "ricoh568_ldo5",},
};
#endif

#ifdef CONFIG_OF
static int ricoh568_regulator_dt_init(struct platform_device *pdev,
				    struct regulator_config *config,
				    int regidx)
{
	struct device_node *nproot, *np;
	int rcount;
	nproot = of_node_get(pdev->dev.parent->of_node);
	if (!nproot)
		return -ENODEV;
	np = of_find_node_by_name(nproot, "regulators");
	if (!np) {
		dev_err(&pdev->dev, "failed to find regulators node\n");
		return -ENODEV;
	}

	rcount = of_regulator_match(&pdev->dev, np,
				&ricoh568_regulator_matches[regidx], 1);
	of_node_put(np);
	if (rcount < 0)
		return -ENODEV;
	config->init_data = ricoh568_regulator_matches[regidx].init_data;
	config->of_node = ricoh568_regulator_matches[regidx].of_node;

	return 0;
}
#else
#define ricoh568_regulator_dt_init(x, y, z)	(-1)
#endif

static int ricoh568_regulator_probe(struct platform_device *pdev)
{
	struct ricoh568_regulator *ri = NULL;
	struct regulator_dev *rdev;
	struct regulator_config config = { };
	int err,id=0;
	printk("%s,line=%d\n", __func__,__LINE__);
	
	rdev = devm_kzalloc(&pdev->dev, RICOH568_NUM_REGULATOR *
				sizeof(*rdev), GFP_KERNEL);
	if (!rdev) {
		dev_err(&pdev->dev, "Mmemory alloc failed\n");
		return -ENOMEM;
	}

	for (id = 0; id < RICOH568_NUM_REGULATOR; ++id) {

	ri = find_regulator_info(id);
	if (!ri) {
		dev_err(&pdev->dev, "invalid regulator ID specified\n");
		err = -EINVAL;
	}

	ri->dev = &pdev->dev;
	config.dev = &pdev->dev;
	config.driver_data = ri;

	config.of_node = ricoh568_regulator_matches[id].of_node;
	printk("%s,line=%d: %s\n", __func__,__LINE__, ricoh568_regulator_matches[id].name);

	err = ricoh568_regulator_dt_init(pdev, &config, id);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to regulator dt init\n");
	}
#if 0
	rdev = regulator_register(&ri->desc, &config);
	if (IS_ERR_OR_NULL(rdev)) {
		dev_err(&pdev->dev, "failed to register regulator %s\n",
				ri->desc.name);
		return PTR_ERR(rdev);
	}
#endif
	}

	platform_set_drvdata(pdev, rdev);
	return 0;
}

static int ricoh568_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);

	regulator_unregister(rdev);
	return 0;
}

static struct platform_driver ricoh568_regulator_driver = {
	.driver	= {
		.name	= "ricoh568-regulator",
		.owner	= THIS_MODULE,
	},
	.probe		= ricoh568_regulator_probe,
	.remove		= ricoh568_regulator_remove,
};

static int __init ricoh568_regulator_init(void)
{

	return platform_driver_register(&ricoh568_regulator_driver);
}
subsys_initcall_sync(ricoh568_regulator_init);

static void __exit ricoh568_regulator_exit(void)
{
	platform_driver_unregister(&ricoh568_regulator_driver);
}
module_exit(ricoh568_regulator_exit);

MODULE_DESCRIPTION("RICOH568 regulator driver");
MODULE_ALIAS("platform:ricoh568-regulator");
MODULE_AUTHOR("Leozhang <leozhang@reds.ricoh.com>");
MODULE_LICENSE("GPL");
