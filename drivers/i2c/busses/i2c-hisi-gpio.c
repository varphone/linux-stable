/*
 * Bitbanging I2C bus driver using the GPIO API
 *
 * Copyright (C) 2007 Atmel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/i2c-gpio.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>

#include <asm/gpio.h>
#include <asm/io.h>

#define DRV_NAME		"i2c-hisi-gpio"

#define I2C_GPIO_BASE		0x20210000
#define I2C_GPIO_DIR		IO_ADDRESS(I2C_GPIO_BASE + 0x400)

#define I2C_SCL_DIR_REG		IO_ADDRESS(I2C_GPIO_BASE + 0x400)
#define I2C_SCL_SHIFT_NUM	0x5
#define I2C_SCL_MASK		(1 << 5)    /* GPIO12 0_5 */
#define I2C_SCL_REG		IO_ADDRESS(I2C_GPIO_BASE + 0x80)  /* 0x80 */
#define I2C_SCL_MUXCTRL_REG	IO_ADDRESS(0x200f0000 + 0x19c)

#define I2C_SDA_DIR_REG		IO_ADDRESS(I2C_GPIO_BASE + 0x400)
#define I2C_SDA_SHIFT_NUM	0x4
#define I2C_SDA_MASK		(1 << 4)    /* GPIO12 0_4 */
#define I2C_SDA_REG		IO_ADDRESS(I2C_GPIO_BASE + 0x40)  /* 0x40 */
#define I2C_SDA_MUXCTRL_REG	IO_ADDRESS(0x200f0000 + 0x198)

#define HW_REG(reg)		*((volatile unsigned int *)(reg))
#define DELAY(us)		time_delay_us(us)

static spinlock_t gpioi2c_lock;

#if 0
unsigned char gpio_i2c_read(unsigned char devaddress, unsigned char address)
{
	int ret;
	unsigned char val;
	struct i2c_adapter *adap;
	struct i2c_msg msgs[2];
	spin_lock(&gpioi2c_lock);
	adap = i2c_get_adapter(0);
	msgs[0].addr = devaddress >> 1;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &address;
	msgs[1].addr = devaddress >> 1;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = &val;
	ret = i2c_transfer(adap, msgs, 2);
	spin_unlock(&gpioi2c_lock);
	if (ret < 0)
		return 0xff;
	return val;

}
#else
unsigned char gpio_i2c_read(unsigned char devaddress, unsigned char address)
{
	int ret;
	struct i2c_adapter *adap;
	union i2c_smbus_data smdata;
	spin_lock(&gpioi2c_lock);
	adap = i2c_get_adapter(0);
	ret = i2c_smbus_xfer(adap, devaddress >> 1, 0, I2C_SMBUS_READ,
			     address, I2C_SMBUS_BYTE_DATA, &smdata);
	spin_unlock(&gpioi2c_lock);
	if (ret < 0)
		return 0xff;
	return smdata.byte;
}
#endif
EXPORT_SYMBOL(gpio_i2c_read);

/* Read a byte from 16 bit register */
unsigned char gpio_i2c_read_ex(unsigned char devaddress, unsigned short address)
{
	int ret;
	struct i2c_adapter *adap;
	union i2c_smbus_data smdata;
	unsigned char command = (address >> 8) & 0xff;
	spin_lock(&gpioi2c_lock);
	adap = i2c_get_adapter(0);
	smdata.byte = address & 0xff;
	ret = i2c_smbus_xfer(adap, devaddress >> 1, 0, I2C_SMBUS_WRITE,
			     command, I2C_SMBUS_BYTE_DATA, &smdata);
	ret = i2c_smbus_xfer(adap, devaddress >> 1, 0, I2C_SMBUS_READ,
			     command, I2C_SMBUS_BYTE, &smdata);
	spin_unlock(&gpioi2c_lock);
	if (ret < 0)
		return 0xff;
	return smdata.byte;
}
EXPORT_SYMBOL(gpio_i2c_read_ex);

#if 0
void gpio_i2c_write(unsigned char devaddress, unsigned char address, unsigned char data)
{
	int ret;
	unsigned char buf[2];
	struct i2c_adapter *adap = i2c_get_adapter(0);
	struct i2c_msg msgs[1];
	spin_lock(&gpioi2c_lock);
	adap = i2c_get_adapter(0);
	msgs[0].addr = devaddress >> 1;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = buf;
	buf[0] = address;
	buf[1] = data;
	ret = i2c_transfer(adap, msgs, 1);
	(void)ret;
	spin_unlock(&gpioi2c_lock);
}
#else
void gpio_i2c_write(unsigned char devaddress, unsigned char address, unsigned char data)
{
	int ret;
	struct i2c_adapter *adap;
	union i2c_smbus_data smdata;
	spin_lock(&gpioi2c_lock);
	adap = i2c_get_adapter(0);
	smdata.byte = data;
	ret = i2c_smbus_xfer(adap, devaddress >> 1, 0, I2C_SMBUS_WRITE,
			     address, I2C_SMBUS_BYTE_DATA, &smdata);
	spin_unlock(&gpioi2c_lock);
}
#endif
EXPORT_SYMBOL(gpio_i2c_write);

/* Write a byte to 16 bit register */
void gpio_i2c_write_ex(unsigned char devaddress, unsigned short address, unsigned char data)
{
	int ret;
	struct i2c_adapter *adap;
	union i2c_smbus_data smdata;
	unsigned char command = (address >> 8) & 0xff;
	spin_lock(&gpioi2c_lock);
	adap = i2c_get_adapter(0);
	smdata.block[0] = 2;
	smdata.block[1] = address & 0xff;
	smdata.block[2] = data;
	ret = i2c_smbus_xfer(adap, devaddress >> 1, 0, I2C_SMBUS_WRITE,
			     command, I2C_SMBUS_BLOCK_DATA, &smdata);
	spin_unlock(&gpioi2c_lock);
}
EXPORT_SYMBOL(gpio_i2c_write_ex);

static void time_delay_us(unsigned int usec)
{
    volatile int i, j;
    for (i = 0; i < usec * 2; i++)
    {
        for (j = 0; j < 50*6; j++)
        {
            ;
        }
    }
}

/* if state == true, set input, else set output and change to 0 */
static void hisi_i2c_setsda_dir(void *data, int state)
{
	volatile long val;
	val  = readl(I2C_SDA_DIR_REG);
	if (state) {
		val &= (~I2C_SDA_MASK);
		writel(val, I2C_SDA_DIR_REG);
	}
	else {
		val |= I2C_SDA_MASK;
		writel(val, I2C_SDA_DIR_REG);
		writel(0, I2C_SDA_REG);
	}
}

/* if state == true, set input, else set output and change to 0 */
static void hisi_i2c_setscl_dir(void *data, int state)
{
	volatile long val;
	val  = readl(I2C_SCL_DIR_REG);
	if (state) {
		val &= (~I2C_SCL_MASK);
		writel(val, I2C_SCL_DIR_REG);
	} else {
		val |= I2C_SCL_MASK;
		writel(val, I2C_SCL_DIR_REG);
		writel(0, I2C_SCL_REG);
	}
}

/*
 * Toggle SDA by changing the output value of the pin. This is only
 * valid for pins configured as open drain (i.e. setting the value
 * high effectively turns off the output driver.)
 */
static void hisi_i2c_setsda_val(void *data, int state)
{
	volatile long val;
	val  = readl(I2C_SDA_DIR_REG);
	val |= I2C_SDA_MASK;
	writel(val, I2C_SDA_DIR_REG);
	writel(state ? I2C_SDA_MASK : 0, I2C_SDA_REG);
}

/*
 * Toggle SCL by changing the output value of the pin. This is used
 * for pins that are configured as open drain and for output-only
 * pins. The latter case will break the i2c protocol, but it will
 * often work in practice.
 */
static void hisi_i2c_setscl_val(void *data, int state)
{
	volatile long val;

	val  = readl(I2C_SCL_DIR_REG);
	val |= I2C_SCL_MASK;
	writel(val, I2C_SCL_DIR_REG);
	writel(state ? I2C_SCL_MASK : 0, I2C_SCL_REG);
}

static int hisi_i2c_getsda(void *data)
{
	volatile long val;
	val = readl(I2C_SDA_REG);
	return val ? 1 : 0;
}

static int hisi_i2c_getscl(void *data)
{
	volatile long val;
	val = readl(I2C_SCL_REG);
	return val ? 1 : 0;
}

static void hisi_i2c_startup(void* data)
{
	volatile long val;

	/* Pin ctrl */
	writel(0, I2C_SCL_MUXCTRL_REG);
	writel(0, I2C_SDA_MUXCTRL_REG);
	/* Pull high */
	hisi_i2c_setscl_val(NULL, 1);
	hisi_i2c_setsda_val(NULL, 1);
}

static int __devinit hisi_i2c_probe(struct platform_device *pdev)
{
	struct i2c_algo_bit_data *bit_data;
	struct i2c_adapter *adap;
	int ret;

	ret = -ENOMEM;
	adap = kzalloc(sizeof(struct i2c_adapter), GFP_KERNEL);
	if (!adap)
		goto err_alloc_adap;

	bit_data = kzalloc(sizeof(struct i2c_algo_bit_data), GFP_KERNEL);
	if (!bit_data)
		goto err_alloc_bit_data;

	bit_data->setsda = hisi_i2c_setsda_dir;
	bit_data->setscl = hisi_i2c_setscl_dir;
	bit_data->getsda = hisi_i2c_getsda;
	bit_data->getscl = hisi_i2c_getscl;
	bit_data->udelay = 2;			/* 100 kHz */
	bit_data->timeout = HZ / 10;		/* 100 ms */
	bit_data->data = NULL;

	adap->owner = THIS_MODULE;
	snprintf(adap->name, sizeof(adap->name), "i2c-hisi-gpio");
	adap->algo_data = bit_data;
	adap->class = I2C_CLASS_HWMON | I2C_CLASS_SPD;
	adap->dev.parent = &pdev->dev;

	/*
	 * If "dev->id" is negative we consider it as zero.
	 * The reason to do so is to avoid sysfs names that only make
	 * sense when there are multiple adapters.
	 */
	adap->nr = (pdev->id != -1) ? pdev->id : 0;
	ret = i2c_bit_add_numbered_bus(adap);
	if (ret)
		goto err_add_bus;

	platform_set_drvdata(pdev, adap);
	hisi_i2c_startup(NULL);

	spin_lock_init(&gpioi2c_lock);

	dev_info(&pdev->dev, "probed.\n");

	return 0;

err_add_bus:
	kfree(bit_data);
err_alloc_bit_data:
	kfree(adap);
err_alloc_adap:
	return ret;
}

static int __devexit hisi_i2c_remove(struct platform_device *pdev)
{
	struct hisi_i2c_platform_data *pdata;
	struct i2c_adapter *adap;

	adap = platform_get_drvdata(pdev);
	pdata = pdev->dev.platform_data;

	i2c_del_adapter(adap);
	kfree(adap->algo_data);
	kfree(adap);

	return 0;
}

static struct platform_driver hisi_i2c_driver = {
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= hisi_i2c_probe,
	.remove		= __devexit_p(hisi_i2c_remove),
};

static int __init hisi_i2c_init(void)
{
	int ret;

	ret = platform_driver_register(&hisi_i2c_driver);
	if (ret)
		printk(KERN_ERR "%s: probe failed: %d\n", DRV_NAME, ret);
	return ret;
}

static void __exit hisi_i2c_exit(void)
{
	platform_driver_unregister(&hisi_i2c_driver);
}

module_init(hisi_i2c_init);
module_exit(hisi_i2c_exit);

MODULE_AUTHOR("Varphone Wong (varphone@qq.com)");
MODULE_DESCRIPTION("Bitbanging I2C driver for Hi3XXX, Hi6XXX");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:"DRV_NAME);
