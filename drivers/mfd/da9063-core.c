
/* da9063-core.c - Core MFD device driver for DA9063
 * Copyright (C) 2012  Dialog Semiconductor Ltd.
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>

#include <linux/mfd/da9063/core.h>
#include <linux/mfd/da9063/pdata.h>
#include <linux/mfd/da9063/registers.h>

#include <linux/proc_fs.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>


static struct resource da9063_regulators_resources[] = {
	{
		.name	= "LDO_LIM",
		.start	= DA9063_IRQ_LDO_LIM,
		.end	= DA9063_IRQ_LDO_LIM,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource da9063_rtc_resources[] = {
	{
		.name	= "ALARM",
		.start	= DA9063_IRQ_ALARM,
		.end	= DA9063_IRQ_ALARM,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "TICK",
		.start	= DA9063_IRQ_TICK,
		.end	= DA9063_IRQ_TICK,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct resource da9063_onkey_resources[] = {
	{
		.start	= DA9063_IRQ_ONKEY,
		.end	= DA9063_IRQ_ONKEY,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource da9063_hwmon_resources[] = {
	{
		.start	= DA9063_IRQ_ADC_RDY,
		.end	= DA9063_IRQ_ADC_RDY,
		.flags	= IORESOURCE_IRQ,
	},
};


static struct mfd_cell da9063_devs[] = {
	{
		.name		= DA9063_DRVNAME_REGULATORS,
		.num_resources	= ARRAY_SIZE(da9063_regulators_resources),
		.resources	= da9063_regulators_resources,
	},
	{
		.name		= DA9063_DRVNAME_HWMON,
		.num_resources	= ARRAY_SIZE(da9063_hwmon_resources),
		.resources	= da9063_hwmon_resources,
	},
	{
		.name		= DA9063_DRVNAME_ONKEY,
		.num_resources	= ARRAY_SIZE(da9063_onkey_resources),
		.resources	= da9063_onkey_resources,
	},
	{
		.name		= DA9063_DRVNAME_RTC,
		.num_resources	= ARRAY_SIZE(da9063_rtc_resources),
		.resources	= da9063_rtc_resources,
	},
};

int da9063_reg_read(struct da9063 *da9063, u16 reg)
{
	u8 val;
	int ret;

	mutex_lock(&da9063->io_mutex);
	ret = da9063_read_device(da9063, DA9063_I2C_REG(reg), 1, &val);
	mutex_unlock(&da9063->io_mutex);

	if (ret)
		return ret;

	return val;
}

int da9063_reg_write(struct da9063 *da9063, u16 reg, u8 val)
{
	int ret;

	mutex_lock(&da9063->io_mutex);
	ret = da9063_write_device(da9063, DA9063_I2C_REG(reg), 1, &val);
	mutex_unlock(&da9063->io_mutex);

	return ret;
}

int da9063_block_read(struct da9063 *da9063, u16 reg, int bytes, u8 *dst)
{
	int ret;

	mutex_lock(&da9063->io_mutex);
	ret = da9063_read_device(da9063, DA9063_I2C_REG(reg), bytes, dst);
	mutex_unlock(&da9063->io_mutex);

	return ret;
}

int da9063_block_write(struct da9063 *da9063, u16 reg, int bytes, const u8 *src)
{
	int ret;

	mutex_lock(&da9063->io_mutex);
	ret = da9063_write_device(da9063, DA9063_I2C_REG(reg), bytes, src);
	mutex_unlock(&da9063->io_mutex);

	return ret;
}

int da9063_reg_update(struct da9063 *da9063, u16 reg, u8 mask, u8 val)
{
	u8 orig, new;
	int ret;

	mutex_lock(&da9063->io_mutex);

	ret = da9063_read_device(da9063, DA9063_I2C_REG(reg), 1, &orig);
	if (ret)
		goto unlock_out;

	new = (orig & ~mask) | (val & mask);
	if (new == orig)
		goto unlock_out;

	ret = da9063_write_device(da9063, DA9063_I2C_REG(reg), 1, &new);
unlock_out:
	mutex_unlock(&da9063->io_mutex);

	return ret;
}

int da9063_reg_set_bits(struct da9063 *da9063, u16 reg, u8 mask)
{
	return da9063_reg_update(da9063, reg, mask, mask);
}

int da9063_reg_clear_bits(struct da9063 *da9063, u16 reg, u8 mask)
{
	return da9063_reg_update(da9063, reg, mask, 0);
}

int da9063_device_init(struct da9063 *da9063, unsigned int irq)
{
	struct da9063_pdata *pdata = da9063->dev->platform_data;
	int ret = 0;

	mutex_init(&da9063->io_mutex);

	if (pdata == NULL) {
		dev_err(da9063->dev, "Platform data not specified.\n");
		return -EINVAL;
	}
	da9063->irq_base = pdata->irq_base;
	da9063->chip_irq = irq;

	if (pdata->init != NULL) {
		ret = pdata->init(da9063);
		if (ret != 0) {
			dev_err(da9063->dev,
				"Platform initialization failed.\n");
			return ret;
		}
	}

	ret = da9063_irq_init(da9063);
	if (ret) {
		dev_err(da9063->dev, "Cannot initialize interrupts.\n");
		return ret;
	}

	ret = mfd_add_devices(da9063->dev, -1, da9063_devs,
			      ARRAY_SIZE(da9063_devs), NULL, da9063->irq_base);
	if (ret)
		dev_err(da9063->dev, "Cannot add MFD cells\n");

	dev_info(da9063->dev, "Device detected DA9063\n" );

	return ret;
}

void da9063_device_exit(struct da9063 *da9063)
{
	mfd_remove_devices(da9063->dev);
}

MODULE_DESCRIPTION("Core MFD device driver for Dialog DA9063");
MODULE_AUTHOR("S Twiss <stwiss.opensource@diasemi.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DA9063_DRVNAME_CORE);
