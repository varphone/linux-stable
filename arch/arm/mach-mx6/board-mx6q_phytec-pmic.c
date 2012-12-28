/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */


/*
 * mx6_pmic_da9063.c  --  i.MX6 phyFLEX driver for pmic da9063
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/da9063/core.h>
#include <linux/mfd/da9063/pdata.h>
#include <linux/mfd/da9063/registers.h>
#include <mach/irqs.h>
#include <mach/gpio.h>

#include <linux/io.h>
#include "crm_regs.h"
#include "regs-anadig.h"
#include "cpu_op-mx6.h"

extern struct cpu_op *(*get_cpu_op)(int *op);

struct da9063_pdata da9063_data;

#define DA9063_LDO(max, min, rname, num_consumers, consumers) \
{\
	.constraints = {\
		.name		= (rname), \
		.max_uV		= (max) * 1000,\
		.min_uV		= (min) * 1000,\
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE\
		|REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,\
		.valid_modes_mask = REGULATOR_MODE_NORMAL,\
	},\
	.num_consumer_supplies = (num_consumers), \
	.consumer_supplies = (consumers), \
}


/* CPU */
static struct regulator_consumer_supply vdd_core_consumers[] = {
	{
		.supply = "VDDCORE",
	}
};
/* SOC */
static struct regulator_consumer_supply vdd_soc_consumers[] = {
	{
		.supply = "VDDSOC",
	},
};
static struct regulator_consumer_supply vdd_ddr3_consumers[] = {
	{
		.supply = "VDD_DDR3",
	},
};
static struct regulator_consumer_supply vdd_3v3_consumers[] = {
	{
		.supply = "VDD_3V3",
	},
};
static struct regulator_consumer_supply vdd_buckmem_consumers[] = {
	{
		.supply = "VDD_BUCKMEM",
	},
};
static struct regulator_consumer_supply vdd_eth_consumers[] = {
	{
		.supply = "VDD_ETH",
	},
};
static struct regulator_consumer_supply vdd_eth_io_consumers[] = {
	{
		.supply = "VDD_ETH_IO",
	},
};
static struct regulator_consumer_supply vdd_snvs_consumers[] = {
	{
		.supply = "VDD_MX6_SNVS",
	},
};
static struct regulator_consumer_supply vdd_pmic_io_consumers[] = {
	{
		.supply = "VDD_3V3_PMIC_IO",
	},
};
static struct regulator_consumer_supply vdd_sd0_consumers[] = {
	{
		.supply = "VDD_SD0",
	},
};
static struct regulator_consumer_supply vdd_sd1_consumers[] = {
	{
		.supply = "VDD_SD1",
	},
};
static struct regulator_consumer_supply vdd_high_consumers[] = {
	{
		.supply = "VDD_MX6_HIGH",
	},
};


/* currently the suspend_mv here takes no effects for DA9063
preset-voltage have to be done in the latest stage during
suspend*/
static struct regulator_init_data da9063_regulators_init[] = {
	/* BUCKS */
	{
		.constraints = {
			.name		= "VDDCORE",
			.max_uV		= 1280000,
			.min_uV		= 725000,
			.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
		},
		.num_consumer_supplies = ARRAY_SIZE(vdd_core_consumers),
		.consumer_supplies = vdd_core_consumers,
	}, {
		.constraints = {
			.name		= "VDDSOC",
			.max_uV		= 1280000,
			.min_uV		= 725000,
			.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_MODE,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
		},
		.num_consumer_supplies = ARRAY_SIZE(vdd_soc_consumers),
		.consumer_supplies = vdd_soc_consumers,
	}, {
		.constraints = {
			.name		= "VDD_DDR3",
			.max_uV		= 1500000,
			.min_uV		= 1500000,
			.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
			.valid_modes_mask = 0,
		},
		.num_consumer_supplies = ARRAY_SIZE(vdd_ddr3_consumers),
		.consumer_supplies = vdd_ddr3_consumers,
	}, {
		.constraints = {
			.name		= "VDD_3V3",
			.max_uV		= 3300000,
			.min_uV		= 3300000,
			.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
			.valid_modes_mask = 0,
		},
		.num_consumer_supplies = ARRAY_SIZE(vdd_3v3_consumers),
		.consumer_supplies = vdd_3v3_consumers,
	}, {
		.constraints = {
			.name		= "VDD_BUCKMEM",
			.max_uV		= 3300000,
			.min_uV		= 3300000,
			.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
			.valid_modes_mask = 0,
		},
		.num_consumer_supplies = ARRAY_SIZE(vdd_buckmem_consumers),
		.consumer_supplies = vdd_buckmem_consumers,
	}, {
		.constraints = {
			.name		= "VDD_ETH",
			.max_uV		= 1200000,
			.min_uV		= 1200000,
			.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
			.valid_modes_mask = 0,
		},
		.num_consumer_supplies = ARRAY_SIZE(vdd_eth_consumers),
		.consumer_supplies = vdd_eth_consumers,
	},

	/* LDO */
	{
		.constraints = {
			.name		= "VDD_ETH_IO",
			.max_uV		= 3300000,
			.min_uV		= 3300000,
			.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
			.valid_modes_mask = 0,
		},
		.num_consumer_supplies = ARRAY_SIZE(vdd_eth_io_consumers),
		.consumer_supplies = vdd_eth_io_consumers,
	}, {
		.constraints = {
			.name		= "VDD_MX6_SNVS",
			.max_uV		= 3300000,
			.min_uV		= 3300000,
			.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
			.valid_modes_mask = 0,
		},
		.num_consumer_supplies = ARRAY_SIZE(vdd_snvs_consumers),
		.consumer_supplies = vdd_snvs_consumers,
	}, {
		.constraints = {
			.name		= "VDD_3V3_PMIC_IO",
			.max_uV		= 3300000,
			.min_uV		= 3300000,
			.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
			.valid_modes_mask = 0,
		},
		.num_consumer_supplies = ARRAY_SIZE(vdd_pmic_io_consumers),
		.consumer_supplies = vdd_pmic_io_consumers,
	}, {
		.constraints = {
			.name		= "VDD_SD0",
			.max_uV		= 3300000,
			.min_uV		= 3300000,
			.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
			.valid_modes_mask = 0,
		},
		.num_consumer_supplies = ARRAY_SIZE(vdd_sd0_consumers),
		.consumer_supplies = vdd_sd0_consumers,
	}, {
		.constraints = {
			.name		= "VDD_SD1",
			.max_uV		= 3300000,
			.min_uV		= 3300000,
			.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
			.valid_modes_mask = 0,
		},
		.num_consumer_supplies = ARRAY_SIZE(vdd_sd1_consumers),
		.consumer_supplies = vdd_sd1_consumers,
	}, {
		.constraints = {
			.name		= "VDD_MX6_HIGH",
			.max_uV		= 3000000,
			.min_uV		= 3000000,
			.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
			.valid_modes_mask = 0,
		},
		.num_consumer_supplies = ARRAY_SIZE(vdd_high_consumers),
		.consumer_supplies = vdd_high_consumers,
	},
};

struct da9063_regulator_data da9063_regulators_data[] = {
	/* BUCKs */
	{
		.id = DA9063_ID_BCORE1,
		.initdata = &da9063_regulators_init[0],
		.dvc_base_uV = 725000,
		.dvc_max_uV = 1280000,
	}, {
		.id = DA9063_ID_BCORE2,
		.initdata = &da9063_regulators_init[1],
		.dvc_base_uV = 725000,
		.dvc_max_uV = 1280000,
	}, {
		.id = DA9063_ID_BPRO,
		.initdata = &da9063_regulators_init[2],
		.dvc_base_uV = 1500000,
		.dvc_max_uV = 1500000,
	}, {
		.id = DA9063_ID_BPERI,
		.initdata = &da9063_regulators_init[3],
		.dvc_base_uV = 3300000,
		.dvc_max_uV = 3300000,
	}, {
		.id = DA9063_ID_BMEM,
		.initdata = &da9063_regulators_init[4],
		.dvc_base_uV = 3300000,
		.dvc_max_uV = 3300000,
	}, {
		.id = DA9063_ID_BIO,
		.initdata = &da9063_regulators_init[5],
		.dvc_base_uV = 1200000,
		.dvc_max_uV = 1200000,
	},

	/* LDOs */
	{
		.id = DA9063_ID_LDO4,
		.initdata = &da9063_regulators_init[6],
		.dvc_base_uV = 3300000,
		.dvc_max_uV = 3300000,
	}, {
		.id = DA9063_ID_LDO5,
		.initdata = &da9063_regulators_init[7],
		.dvc_base_uV = 3300000,
		.dvc_max_uV = 3300000,
	}, {
		.id = DA9063_ID_LDO6,
		.initdata = &da9063_regulators_init[8],
		.dvc_base_uV = 3300000,
		.dvc_max_uV = 3300000,
	}, {
		.id = DA9063_ID_LDO9,
		.initdata = &da9063_regulators_init[9],
		.dvc_base_uV = 3300000,
		.dvc_max_uV = 3300000,
	}, {
		.id = DA9063_ID_LDO10,
		.initdata = &da9063_regulators_init[10],
		.dvc_base_uV = 3300000,
		.dvc_max_uV = 3300000,
	}, {
		.id = DA9063_ID_LDO11,
		.initdata = &da9063_regulators_init[11],
		.dvc_base_uV = 3000000,
		.dvc_max_uV = 3000000,
	},
};

struct da9063_regulators_pdata da9063_regulators = {
	.n_regulators = ARRAY_SIZE(da9063_regulators_data),
	.regulator_data = da9063_regulators_data,
};

static int da9063_init(struct da9063 *da9063)
{
	unsigned int reg;

	printk("\t\tDA9063 Initi function call!\n");

	// ANATOP regulators
	struct regulator *cpu_regulator = regulator_get(NULL, "cpu_vddgp");
	struct regulator *soc_regulator = regulator_get(NULL, "cpu_vddsoc");
	struct regulator *pu_regulator = regulator_get(NULL, "cpu_vddvpu");

	// Set to maximum possible voltage values
	regulator_set_voltage(cpu_regulator, 1300000, 1300000);
	regulator_set_voltage(soc_regulator, 1300000, 1300000);
	regulator_set_voltage(pu_regulator, 1300000, 1300000);

	return 0;
}

struct da9063_pdata __initdata da9063_data = {
	.bcores_merged = 0,
	.bmem_bio_merged = 0,
	.key_power = 0,
	.t_offset = 0,
	.regulators_pdata = &da9063_regulators,
	.init = da9063_init,
};

#define DA9063_IRQ_PIN	IMX_GPIO_NR(4, 17)
struct i2c_board_info da9063_i2c = {
	I2C_BOARD_INFO("da9063", 0x58),
	.irq = gpio_to_irq(DA9063_IRQ_PIN),
	.platform_data = &da9063_data,
};

int __init mx6_phyflex_init_da9063(void)
{
	printk("::: mx6_phyflex_init_da9063\n");
	return i2c_register_board_info(0, &da9063_i2c, 1);
}
