/*
 * include/mfd/atm88pa-private.h
 *
 * Private defines of ATMEGA88PA MFD driver
 *
 * Copyright (C) 2017 NanJing No.55 Research Institute Technology Development CO.,LTD.
 *
 * Author: Varphone Wong <varphone@qq.com>
 *
 */
#ifndef MFD_ATM88PA_PRIVATE_H
#define MFD_ATM88PA_PRIVATE_H

#include <linux/input.h>
#include <linux/leds.h>
#include <linux/miscdevice.h>
#include <linux/hrtimer.h>

#define ATM88PA_MAX_KEYS	8
#define ATM88PA_MAX_LEDS	4
#define ATM88PA_MAX_DATAS	7

/* Keypad of ATM88PA */
struct atm88pa_keypad {
	struct input_dev	*idev;
	int			debounce_time; /* Debounce time */
	int			repeat; /* Repeat delay */
	u8			old_keys; /* Prev keys state */
	const char		*phys; /* Physical path to the device in the system hierarchy */
	u16			keymap[ATM88PA_MAX_KEYS]; /* keymaping */
};

/* Power Management of ATM88PA */
struct atm88pa_power {
	int			gpio; /* GPIO Number */
	int			active_low; /* Active with low level */
};

/* ATM88PA */
struct atm88pa {
	struct device		*dev; /* Device of the this */
	struct i2c_client	*i2c; /* I2C client */
	struct atm88pa_keypad	keypad; /* Keypad */
	struct led_classdev	leds[ATM88PA_MAX_LEDS]; /* Leds */
	struct miscdevice	misc; /* Misc device */
	struct atm88pa_power	power; /* Power off */
	u8			old_status; /* Prev status value */
	u16			chip_ver; /* Cached chip version */
	struct hrtimer		timer;
	struct work_struct	timer_work;
	struct workqueue_struct *timer_wq;
};

#define dev_to_i2c_client(d)	container_of(d, struct i2c_client, dev)
#define dev_to_atm88pa(d)	i2c_get_clientdata(dev_to_i2c_client(d))
#define miscdev_to_atm88pa(d)	container_of(dev_get_drvdata(d), struct atm88pa, misc)

extern int atm88pa_read(struct atm88pa *atm, u8 reg);
extern int atm88pa_read_word(struct atm88pa *atm, u8 reg);
extern int atm88pa_write(struct atm88pa *atm, u8 reg, u8 val);
extern int atm88pa_write_word(struct atm88pa *atm, u8 reg, u16 val);

extern int atm88pa_get_sw_ver(struct atm88pa *atm);
extern void atm88pa_update_status(struct atm88pa *atm);

extern int atm88pa_irq_init(struct atm88pa *atm);
extern int atm88pa_irq_exit(struct atm88pa *atm);
extern int atm88pa_irq_resume(struct atm88pa *atm);
extern int atm88pa_irq_suspend(struct atm88pa *atm);

extern int atm88pa_keypad_init(struct atm88pa *atm);
extern int atm88pa_keypad_reset(struct atm88pa *atm);
extern void atm88pa_keypad_update(struct atm88pa *atm);
extern void atm88pa_keypad_simulate_key(struct atm88pa *atm, int key, int value);

extern int atm88pa_keypad_register(struct atm88pa *atm);
extern int atm88pa_keypad_unregister(struct atm88pa *atm);

extern int atm88pa_leds_register(struct atm88pa *atm);
extern int atm88pa_leds_unregister(struct atm88pa *atm);

extern int atm88pa_power_register(struct atm88pa *atm);
extern int atm88pa_power_unregister(struct atm88pa *atm);

#endif /* MFD_ATM88PA_PRIVATE_H */

