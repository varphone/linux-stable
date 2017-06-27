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
#include <linux/mfd/atm88pa-private.h>

static void atm88pa_leds_set(struct led_classdev *led,
			     enum led_brightness brightness)
{
}

static enum led_brightness atm88pa_leds_get(struct led_classdev *led)
{
	return 0;
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
