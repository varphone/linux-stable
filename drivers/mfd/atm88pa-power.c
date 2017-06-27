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
#include <linux/mfd/atm88pa-private.h>

static int poweroff_gpio;
static int poweroff_level;

static void atm88pa_power_off(void)
{
	gpio_set_value(poweroff_gpio, poweroff_level);
}

int atm88pa_power_register(struct atm88pa *atm)
{
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

