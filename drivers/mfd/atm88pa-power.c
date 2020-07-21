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

volatile unsigned long virt_addr, phys_addr;
volatile unsigned long *GPIO5_DR, *GPIO5_GDIR;

static void atm88pa_power_off_prepare(void)
{
	phys_addr = 0x020AC000;
	virt_addr =(unsigned long)ioremap(phys_addr, 0x1000);
	GPIO5_DR = (unsigned long *)(virt_addr + 0x00);
	GPIO5_GDIR = (unsigned long *)(virt_addr + 0x04);
	*GPIO5_GDIR |= (0X01 << 20);
	*GPIO5_DR &= ~(0X01 << 20);
	iounmap((void *)virt_addr);
	printk(KERN_INFO "Power Off by GPIO %d, level %d\n",
	       poweroff_gpio, poweroff_level);
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
	printk(KERN_INFO "Power Off by ATM88PA I2C\n");
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
	dev_info(atm->dev, "Power Off use GPIO %d, level %d\n",
	         poweroff_gpio, poweroff_level);
#else
	poweroff_atm88pa = atm;
	dev_info(atm->dev, "Power Off use I2C\n");
#endif

	/* Replace power off routine */
	pm_power_off = atm88pa_power_off;

        /* To prevent the locks held bugs */
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

