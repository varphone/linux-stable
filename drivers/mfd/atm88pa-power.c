/*
 * drivers/mfd/atm88pa-power.c
 *
 * Power Management subsystem of ATMEGA88PA MFD driver
 *
 * Copyright (C) 2017 NanJing No.55 Research Institute Technology Development CO.,LTD.
 *
 * Author: Varphone Wong <varphone@qq.com>
 *
 */
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/mfd/atm88pa.h>
#include <linux/mfd/atm88pa-private.h>

#ifdef CONFIG_MFD_ATM88PA_POWEROFF_USE_GPIO
static int poweroff_gpio;
static int poweroff_level;

static void atm88pa_power_off_prepare(void)
{
	gpio_set_value(poweroff_gpio, poweroff_level);
}
#else
static struct atm88pa *poweroff_atm88pa;

static void atm88pa_power_off_prepare(void)
{
	int ret;

	ret = atm88pa_write(poweroff_atm88pa, ATM88PA_REG_PWR_CTRL, 0x55);
	if (ret < 0) {
		dev_warn(poweroff_atm88pa->dev, "write power off control failed, err: %d\n", ret);
	}
}
#endif

static void atm88pa_power_off(void)
{
	/* Nothing to do */
}

int atm88pa_power_register(struct atm88pa *atm)
{
#ifdef CONFIG_MFD_ATM88PA_POWEROFF_USE_GPIO
	int flags, ret;
	struct atm88pa_power *pwr = &atm->power;

	flags = pwr->active_low ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW;
	ret = devm_gpio_request_one(atm->dev, pwr->gpio, flags, "POWER OFF");
	if (ret) {
		dev_err(atm->dev, "request gpio: %d failed, err: %d\n", pwr->gpio, ret);
		return ret;
	}

	poweroff_gpio = pwr->gpio;
	poweroff_level = pwr->active_low ? 0 : 1;
#else
	poweroff_atm88pa = atm;
#endif
	/* Replace power off routine */
	pm_power_off = atm88pa_power_off;
	pm_power_off_prepare = atm88pa_power_off_prepare;

	dev_info(atm->dev, "power subsystem registered.\n");

	return 0;
}

int atm88pa_power_unregister(struct atm88pa *atm)
{
	int ret;
	struct atm88pa_power *pwr = &atm->power;

	if (pm_power_off == atm88pa_power_off)
		pm_power_off = NULL;

	dev_info(atm->dev, "power subsystem unregistered.\n");

	return 0;
}

