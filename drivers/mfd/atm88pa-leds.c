/*
 * drivers/mfd/atm88pa-leds.c
 *
 * LED subsystem of ATMEGA88PA MFD driver
 *
 * Copyright (C) 2017 NanJing No.55 Research Institute Technology Development CO.,LTD.
 *
 * Author: Varphone Wong <varphone@qq.com>
 *
 */
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/mfd/atm88pa.h>
#include <linux/mfd/atm88pa-private.h>

static void atm88pa_leds_set(struct led_classdev *led,
			     enum led_brightness brightness)
{
	struct atm88pa *atm = dev_to_atm88pa(led->dev->parent);

	if (strcmp(led->name, "keypad-bl") == 0) {
		atm88pa_write(atm, ATM88PA_REG_IND_LIGHT, brightness);
	}
	else if (strcmp(led->name, "camera-sl") == 0) {
		atm88pa_write(atm, ATM88PA_REG_SUP_LIGHT, brightness);
	}
	else if (strcmp(led->name, "fault") == 0) {
		atm88pa_write(atm, ATM88PA_REG_FAULT, brightness);
	}
}

static enum led_brightness atm88pa_leds_get(struct led_classdev *led)
{
	struct atm88pa *atm = dev_to_atm88pa(led->dev->parent);
	int ret;

	if (strcmp(led->name, "keypad-bl") == 0) {
		ret = atm88pa_read(atm, ATM88PA_REG_IND_LIGHT);
	}
	else if (strcmp(led->name, "camera-sl") == 0) {
		ret = atm88pa_read(atm, ATM88PA_REG_SUP_LIGHT);
	}
	else if (strcmp(led->name, "fault") == 0) {
		ret = atm88pa_read(atm, ATM88PA_REG_FAULT);
	}
	return ret;
}

int atm88pa_leds_register(struct atm88pa *atm)
{
	int i, ret;
	struct led_classdev *led;

	for (i = 0; i < ARRAY_SIZE(atm->leds); i++) {
		led = &atm->leds[i];
		/* Skip undefined */
		if (led->name == NULL)
			continue;
		led->brightness_get = atm88pa_leds_get;
		led->brightness_set = atm88pa_leds_set;
		ret = devm_led_classdev_register(atm->dev, led);
		if (ret < 0) {
			dev_err(atm->dev, "register led: %s failed, err: %d\n",
				led->name, ret);
			return ret;
		}
		dev_info(atm->dev, "led: %s registered.\n", led->name);
	}

	return 0;
}

int atm88pa_leds_unregister(struct atm88pa *atm)
{
	int i, ret;
	struct led_classdev *led;

	for (i = 0; i < ARRAY_SIZE(atm->leds); i++) {
		led = &atm->leds[i];
		/* Skip undefined */
		if (led->name == NULL)
			continue;
		devm_led_classdev_unregister(atm->dev, led);
		dev_info(atm->dev, "led: %s unregistered.\n", led->name);
	}

	return 0;
}
