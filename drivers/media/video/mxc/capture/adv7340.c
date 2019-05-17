/*
 * Copyright 2005-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file adv7340.c
 *
 * @brief Analog Device adv7340 video decoder functions
 *
 * @ingroup Camera
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/videodev2.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-int-device.h>
#include "mxc_v4l2_capture.h"

static int adv7340_probe(struct i2c_client *adapter,
			 const struct i2c_device_id *id);

static int adv7340_detach(struct i2c_client *client);

static const struct i2c_device_id adv7340_id[] = {
	{ "adv7340", 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, adv7340_id);

static struct i2c_driver adv7340_i2c_driver = {
	.driver =
		{
			.owner = THIS_MODULE,
			.name = "adv7340",
		},
	.probe = adv7340_probe,
	.remove = adv7340_detach,
	.id_table = adv7340_id,
};

/*!
 * Maintains the information on the current state of the sensor.
 */
struct sensor {
	struct i2c_client *i2c_client
} adv7340_data;

/***********************************************************************
 * I2C transfert.
 ***********************************************************************/

/*! Read one register from a adv7340 i2c slave device.
 *
 *  @param *reg		register in the device we wish to access.
 *
 *  @return		       0 if success, an error code otherwise.
 */
static inline int adv7340_read(struct i2c_client *client, u8 reg)
{
	int ret;
	union i2c_smbus_data data;
	ret = i2c_smbus_xfer(client->adapter, client->addr, client->flags,
			     I2C_SMBUS_READ, reg, I2C_SMBUS_BYTE_DATA, &data);

	if (ret < 0) {
		dev_err(&client->dev, "%s:read reg error: reg=%2x\n", __func__,
			reg);
		return -1;
	}
	return data.byte;
}

static s32 adv7340_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	union i2c_smbus_data data;
	int err, i;
	data.byte = val;
	for (i = 0; i < 3; i++) {
		err = i2c_smbus_xfer(client->adapter, client->addr,
				     client->flags, I2C_SMBUS_WRITE, reg,
				     I2C_SMBUS_BYTE_DATA, &data);
		if (!err)
			break;
	}
	if (err < 0)
		dev_err(&client->dev, "%s:write reg error:reg=%2x,val=%2x\n",
			__func__, reg, val);

	return err;
}

/*
clolor
D6 17 02 
D6 00 FC 
D6 01 00 
D6 80 11 
D6 82 C1 
D6 87 80 
D6 88 10 
D6 8A 0C 
D6 8C 00 
D6 8D 00 
D6 8E 02 
D6 8F 2A
*/
static void adv7340_registers_init(void)
{
	adv7340_write_reg(adv7340_data.i2c_client, 0x17, 0x02);
	adv7340_write_reg(adv7340_data.i2c_client, 0x00, 0xFC);
	adv7340_write_reg(adv7340_data.i2c_client, 0x01, 0x00);
	adv7340_write_reg(adv7340_data.i2c_client, 0x80, 0x11);
	adv7340_write_reg(adv7340_data.i2c_client, 0x82, 0xC1);
	adv7340_write_reg(adv7340_data.i2c_client, 0x87, 0x80);
	adv7340_write_reg(adv7340_data.i2c_client, 0x88, 0x10);
	adv7340_write_reg(adv7340_data.i2c_client, 0x8A, 0x0c);
	adv7340_write_reg(adv7340_data.i2c_client, 0x8C, 0x00);
	adv7340_write_reg(adv7340_data.i2c_client, 0x8D, 0x00);
	adv7340_write_reg(adv7340_data.i2c_client, 0x8E, 0x02);
	adv7340_write_reg(adv7340_data.i2c_client, 0x8F, 0x2A);
}

/*! adv7340 I2C attach function.
 *
 *  @param *adapter	struct i2c_adapter *.
 *
 *  @return		Error code indicating success or failure.
 */

/*!
 * adv7340 I2C probe function.
 * Function set in i2c_driver struct.
 * Called by insmod.
 *
 *  @param *adapter	I2C adapter descriptor.
 *
 *  @return		Error code indicating success or failure.
 */
static int adv7340_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret = 0;

	pr_err("In adv7340_probe\n");
	/* Set initial values for the sensor struct. */
	adv7340_data.i2c_client = client;
	adv7340_registers_init();

	return ret;
}

/*!
 * adv7340 I2C detach function.
 * Called on rmmod.
 *
 *  @param *client	struct i2c_client*.
 *
 *  @return		Error code indicating success or failure.
 */
static int adv7340_detach(struct i2c_client *client)
{
	return 0;
}

/*!
 * adv7340 init function.
 * Called on insmod.
 *
 * @return    Error code indicating success or failure.
 */
static __init int adv7340_init(void)
{
	u8 err = 0;

	pr_err("In adv7340_init\n");

	/* Tells the i2c driver what functions to call for this driver. */
	err = i2c_add_driver(&adv7340_i2c_driver);
	if (err != 0)
		pr_err("%s: driver registration failed, error=%d \n", __func__,
		       err);

	return err;
}

/*!
 * adv7340 cleanup function.
 * Called on rmmod.
 *
 * @return   Error code indicating success or failure.
 */
static void __exit adv7340_clean(void)
{
	i2c_del_driver(&adv7340_i2c_driver);
}

module_init(adv7340_init);
module_exit(adv7340_clean);

MODULE_AUTHOR("Wooshang wooshang@126.com");
MODULE_DESCRIPTION("Anolog Device adv7340 video decoder driver");
MODULE_LICENSE("GPL");
