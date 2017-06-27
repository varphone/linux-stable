/*
 * drivers/mfd/atm88pa-keypad.c
 *
 * Keypad subsystem of ATMEGA88PA MFD driver
 *
 * Copyright (C) 2017 NanJing No.55 Research Institute Technology Development CO.,LTD.
 *
 * Author: Varphone Wong <varphone@qq.com>
 *
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/mfd/atm88pa.h>
#include <linux/mfd/atm88pa-private.h>

static void atm88pa_keypad_process_keys(struct atm88pa_keypad *kp, u8 new_keys)
{
	int i;
	u8 diff_keys = kp->old_keys ^ new_keys;
	u8 down;
	u16 code;

	/* Skip unchanged */
	if (diff_keys == 0)
		return;

	/* Save current keys state */
	kp->old_keys = new_keys;

	/* Remap the bit to key code */
	for (i = 0; i < ATM88PA_MAX_KEYS; i++) {
		if (((diff_keys >> i) & 0x1) != 0) {
			down = (new_keys >> i) & 0x1;
			code = kp->keymap[i];
			dev_dbg(&kp->idev->dev, "key 0x%02x %s\n",
			        code, down ? "down" : "up");
			input_report_key(kp->idev, code, down);
			input_sync(kp->idev);
		}
	}
}

void atm88pa_keypad_update(struct atm88pa *atm)
{
	int ret;

	ret = atm88pa_read(atm, ATM88PA_REG_KEYS);
	if (ret < 0) {
		dev_warn(atm->dev, "read keys failed, err: %d\n", ret);
	} else {
		atm88pa_keypad_process_keys(&atm->keypad, (u8)ret);
	}
}

int atm88pa_keypad_register(struct atm88pa *atm)
{
	int i, ret;
	struct atm88pa_keypad *kp = &atm->keypad;
	struct input_dev *idev;

	idev = devm_input_allocate_device(atm->dev);
	if (idev == NULL) {
		dev_err(atm->dev, "allocate input device failed.\n");
		return -ENOMEM;
	}

	idev->name = "atm88pa-keypad";
	idev->phys = kp->phys;
	idev->evbit[0] = BIT(EV_KEY);

	/* Fill key bits */
	for (i = 0; i < ARRAY_SIZE(kp->keymap); i++) {
		__set_bit(kp->keymap[i], idev->keybit);
	}

	/* Clear reserved key */
	__clear_bit(KEY_RESERVED, idev->keybit);

	/* Setup key repeat */
	if (kp->repeat)
		__set_bit(EV_REP, idev->evbit);

	ret = input_register_device(idev);
	if (ret) {
		dev_err(atm->dev, "register input device %s failed, err: %d\n",
			idev->name, ret);
		return ret;
	}

	kp->idev = idev;

	dev_info(atm->dev, "%s registered.\n", kp->idev->name);

	return 0;
}

int atm88pa_keypad_unregister(struct atm88pa *atm)
{
	int ret;
	struct atm88pa_keypad *kp = &atm->keypad;

	if (kp->idev) {
		input_unregister_device(kp->idev);
		dev_info(atm->dev, "%s unregistered.\n", kp->idev->name);
		kp->idev = NULL;
	}

	return 0;
}

