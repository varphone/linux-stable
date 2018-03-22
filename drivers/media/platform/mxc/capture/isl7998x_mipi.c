/*
 * Copyright (C) 2015-2016 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <linux/mipi_csi2.h>
#include <linux/v4l2-controls.h>
#include <linux/miscdevice.h>
#include <media/v4l2-chip-ident.h>
#include "v4l2-int-device.h"
#include "mxc_v4l2_capture.h"

#define CHIP_ID_79985		0x85
#define CHIP_ID_79987		0x87

#define SENSOR_NUM 4

unsigned int g_isl7998x_width = 720;
unsigned int g_isl7998x_field_height = 288;
unsigned int g_isl7998x_frame_height = 576;

/*!
 * Maintains the information on the current state of the sesor.
 */
static int isl7998x_started = 0;
static int isl7998x_state[SENSOR_NUM];
static struct sensor_data isl7998x_data[SENSOR_NUM];
static unsigned int chip_id = 0;

static int g_isl7998x_ch1_std = -1;
static int g_isl7998x_ch2_std = -1;
static int g_isl7998x_ch3_std = -1;
static int g_isl7998x_ch4_std = -1;
module_param_named(ch1_std, g_isl7998x_ch1_std, int, S_IRUSR | S_IWUSR);
module_param_named(ch2_std, g_isl7998x_ch2_std, int, S_IRUSR | S_IWUSR);
module_param_named(ch3_std, g_isl7998x_ch3_std, int, S_IRUSR | S_IWUSR);
module_param_named(ch4_std, g_isl7998x_ch4_std, int, S_IRUSR | S_IWUSR);

static int g_isl7998x_ch1_rst_en = 0;
static int g_isl7998x_ch2_rst_en = 0;
static int g_isl7998x_ch3_rst_en = 0;
static int g_isl7998x_ch4_rst_en = 0;
module_param_named(ch1_rst_en, g_isl7998x_ch1_rst_en, int, S_IRUSR | S_IWUSR);
module_param_named(ch2_rst_en, g_isl7998x_ch2_rst_en, int, S_IRUSR | S_IWUSR);
module_param_named(ch3_rst_en, g_isl7998x_ch3_rst_en, int, S_IRUSR | S_IWUSR);
module_param_named(ch4_rst_en, g_isl7998x_ch4_rst_en, int, S_IRUSR | S_IWUSR);

static int g_isl7998x_sync_ch = 0;
module_param_named(sync_ch, g_isl7998x_sync_ch, int, S_IRUSR | S_IWUSR);

static int g_isl7998x_reset_gpio = -1;
static enum of_gpio_flags g_isl7998x_reset_gpio_flags = 0;
static int g_isl7998x_power_gpio = -1;
static enum of_gpio_flags g_isl7998x_power_gpio_flags = 0;

/* IRQ: INT, MPP1, MPP2 */
static unsigned int g_isl7998x_irqs[3];
static int g_isl7998x_irq_gpios[3];
static const char *g_isl7998x_irq_gpio_names[] = {
	"isl7998x-int-gpio",
	"isl7998x-mpp1-gpio",
	"isl7998x-mpp2-gpio",
	NULL
};

static struct mutex g_isl7998x_lock;

#define ISL7998X_LOCK()		mutex_lock(&g_isl7998x_lock)
#define ISL7998X_UNLOCK()	mutex_unlock(&g_isl7998x_lock)

static int isl7998x_probe(struct i2c_client *adapter,
				const struct i2c_device_id *device_id);
static int isl7998x_remove(struct i2c_client *client);

static int ioctl_dev_init(struct v4l2_int_device *s);

static const struct i2c_device_id isl7998x_id[] = {
	{"isl7998x_mipi", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, isl7998x_id);

static struct i2c_driver isl7998x_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "isl7998x_mipi",
		  },
	.probe  = isl7998x_probe,
	.remove = isl7998x_remove,
	.id_table = isl7998x_id,
};


/*! Read one register from a ISL7998x i2c slave device.
 *
 *  @param *reg		register in the device we wish to access.
 *
 *  @return		       0 if success, an error code otherwise.
 */
static inline int isl7998x_read_reg(u8 reg)
{
	int val;
	int retries = 3;

	while (retries-- > 0) {
		val = i2c_smbus_read_byte_data(isl7998x_data[0].i2c_client, reg);
		if (val >= 0)
			break;
	}
	if (val < 0) {
		dev_info(&isl7998x_data[0].i2c_client->dev,
			"%s:read reg error: reg=%2x\n", __func__, reg);
		return -1;
	}
	return val;
}

/*! Write one register of a ISL7998x i2c slave device.
 *
 *  @param *reg		register in the device we wish to access.
 *
 *  @return		       0 if success, an error code otherwise.
 */
static int isl7998x_write_reg(u8 reg, u8 val)
{
	s32 ret;
	int retries = 3;

	while (retries-- > 0) {
		ret = i2c_smbus_write_byte_data(isl7998x_data[0].i2c_client, reg, val);
		if (ret >= 0)
			break;
	}
	if (ret < 0) {
		dev_info(&isl7998x_data[0].i2c_client->dev,
			"%s:write reg error:reg=%2x,val=%2x\n", __func__,
			reg, val);
		return -1;
	}
	return 0;
}

/* Reset decoder only
 * @param channel	The decoder index, from 1 to 4
 * @return 0 for success, negative for error
 */
static int isl7998x_reset_channel(int channel)
{
	if (channel < 1)
		return -EINVAL;

	isl7998x_write_reg(0xFF, 0x00);
	/* Reset the decoder's logic */
	isl7998x_write_reg(0x02, 1 << (channel -1));
	/* Wait for reset */
	msleep(10);
	/* Exit reset state */
	isl7998x_write_reg(0x02, 0x00);
	isl7998x_write_reg(0xFF, 0x00);

	return 0;
}

/* Set power of the chip
 * @param on		0 = Power off, 1 = Power on
 */
static void isl7998x_chip_power(int on)
{
	/* Skip if gpio not set */
	if (g_isl7998x_power_gpio < 0)
		return;

	/* Power ? */
	if (g_isl7998x_power_gpio_flags & OF_GPIO_ACTIVE_LOW)
		gpio_direction_output(g_isl7998x_power_gpio, on ? 0 : 1);
	else
		gpio_direction_output(g_isl7998x_power_gpio, on ? 1 : 0);
}

/* Reset the chip
 * @param reset		0 = Leave reset, 1 = Enter reset
 */
static void isl7998x_chip_reset(int reset)
{
	/* Skip if gpio not set */
	if (g_isl7998x_reset_gpio < 0)
		return;

	/* Reset ? */
	if (g_isl7998x_reset_gpio_flags & OF_GPIO_ACTIVE_LOW)
		gpio_direction_output(g_isl7998x_reset_gpio, reset ? 0 : 1);
	else
		gpio_direction_output(g_isl7998x_reset_gpio, reset ? 1 : 0);

}

static int isl7998x_hardware_init(struct sensor_data *sensor)
{
	int retval = 0;
	void *mipi_csi2_info;
	u32 mipi_reg;
	int i, lanes;
	u8 reg;

	mipi_csi2_info = mipi_csi2_get_info();

	/* initial mipi dphy */
	if (!mipi_csi2_info) {
		printk(KERN_ERR "%s() in %s: Fail to get s_mipi_csi2_info!\n",
		       __func__, __FILE__);
		return -1;
	}

	if (!mipi_csi2_get_status(mipi_csi2_info))
		mipi_csi2_enable(mipi_csi2_info);

	if (!mipi_csi2_get_status(mipi_csi2_info)) {
		pr_err("Can not enable mipi csi2 driver!\n");
		return -1;
	}

	lanes = mipi_csi2_set_lanes(mipi_csi2_info);
	if (lanes > 1) {
		pr_err("ISL7998x doesn't support lanes = %d.\n", lanes + 1);
		return -1;
	}

	/* Only reset MIPI CSI2 HW at sensor initialize */
	/* 13.5MHz pixel clock (720*480@30fps) * 16 bits per pixel (YUV422) = 216Mbps mipi data rate for each camera */
	mipi_csi2_reset(mipi_csi2_info, (216 * SENSOR_NUM) / (lanes + 1));

	if (sensor->pix.pixelformat == V4L2_PIX_FMT_UYVY) {
		for (i=0; i<SENSOR_NUM; i++)
			mipi_csi2_set_datatype(mipi_csi2_info, i, MIPI_DT_YUV422);
	} else
		pr_err("currently this sensor format can not be supported!\n");

	/* Reset the chip */
	isl7998x_write_reg(0xFF, 0x00);
	isl7998x_write_reg(0x02, 0x80);
	msleep(10);

	// Init the isl7998x
	if (chip_id == CHIP_ID_79985) {
		// Page 0
		isl7998x_write_reg(0xFF, 0x00);
		isl7998x_write_reg(0x03, 0x00);
		if (lanes == 1)
			isl7998x_write_reg(0x0B, 0x41);
		else
			isl7998x_write_reg(0x0B, 0x40);
		isl7998x_write_reg(0x0D, 0xC9);
		isl7998x_write_reg(0x0E, 0xC9);
		isl7998x_write_reg(0x10, 0x01);
		isl7998x_write_reg(0x11, 0x03);
		isl7998x_write_reg(0x12, 0x00);
		isl7998x_write_reg(0x13, 0x00);
		isl7998x_write_reg(0x14, 0x00);
		isl7998x_write_reg(0xFF, 0x00);

		// Page 1
		isl7998x_write_reg(0xFF, 0x01);
		isl7998x_write_reg(0x07, 0x12); // VA_HI[5:4]=1, HA_HI[1:0]=2
		isl7998x_write_reg(0x08, 0x12); // VD_LO
		isl7998x_write_reg(0x09, 0x20); // VA_LO
		isl7998x_write_reg(0x0A, 0x0A); // HD_LO
		isl7998x_write_reg(0x0B, 0xD0); // HA_LO
		isl7998x_write_reg(0x2F, 0xE6); // CCS Blue Color
		isl7998x_write_reg(0x33, 0xF2); // 50Hz Free run, Medium YNR
		isl7998x_write_reg(0x3D, 0x00); // Data conversion
		isl7998x_write_reg(0xFF, 0x01);

		// Page 2
		isl7998x_write_reg(0xFF, 0x02);
		isl7998x_write_reg(0x07, 0x12);
		isl7998x_write_reg(0x08, 0x12);
		isl7998x_write_reg(0x09, 0x20);
		isl7998x_write_reg(0x0A, 0x0A);
		isl7998x_write_reg(0x0B, 0xD0);
		isl7998x_write_reg(0x2F, 0xE6);
		isl7998x_write_reg(0x33, 0xF2);
		isl7998x_write_reg(0x3D, 0x00);
		isl7998x_write_reg(0xFF, 0x02);

		// Page 3
		isl7998x_write_reg(0xFF, 0x03);
		isl7998x_write_reg(0x07, 0x12);
		isl7998x_write_reg(0x08, 0x12);
		isl7998x_write_reg(0x09, 0x20);
		isl7998x_write_reg(0x0A, 0x0A);
		isl7998x_write_reg(0x0B, 0xD0);
		isl7998x_write_reg(0x2F, 0xE6);
		isl7998x_write_reg(0x33, 0xF2);
		isl7998x_write_reg(0x3D, 0x00);
		isl7998x_write_reg(0xFF, 0x03);

		// Page 4
		isl7998x_write_reg(0xFF, 0x04);
		isl7998x_write_reg(0x07, 0x12);
		isl7998x_write_reg(0x08, 0x12);
		isl7998x_write_reg(0x09, 0x20);
		isl7998x_write_reg(0x0A, 0x0A);
		isl7998x_write_reg(0x0B, 0xD0);
		isl7998x_write_reg(0x2F, 0xE6);
		isl7998x_write_reg(0x33, 0xF2);
		isl7998x_write_reg(0x3D, 0x00);
		isl7998x_write_reg(0xFF, 0x04);

		// Page 5
		isl7998x_write_reg(0xFF, 0x05);
		isl7998x_write_reg(0x01, 0x85);
		isl7998x_write_reg(0x02, 0xA0);
		isl7998x_write_reg(0x03, 0x08);
		isl7998x_write_reg(0x04, 0xE4);
		isl7998x_write_reg(0x05, 0x60);
		isl7998x_write_reg(0x06, 0x00);
		isl7998x_write_reg(0x07, 0x46);
		isl7998x_write_reg(0x08, 0x02);
		isl7998x_write_reg(0x09, 0x00);
		isl7998x_write_reg(0x0A, 0x68);
		isl7998x_write_reg(0x0B, 0x02);
		isl7998x_write_reg(0x0C, 0x00);
		isl7998x_write_reg(0x0D, 0x06);
		isl7998x_write_reg(0x0E, 0x00);
		isl7998x_write_reg(0x0F, 0x00);
		isl7998x_write_reg(0x10, 0x05);
		isl7998x_write_reg(0x11, 0xA0);
		isl7998x_write_reg(0x12, 0x76);
		isl7998x_write_reg(0x13, 0x2F);
		isl7998x_write_reg(0x14, 0x0E);
		isl7998x_write_reg(0x15, 0x36);
		isl7998x_write_reg(0x16, 0x12);
		isl7998x_write_reg(0x17, 0xF6);
		isl7998x_write_reg(0x18, 0x00);
		isl7998x_write_reg(0x19, 0x17);
		isl7998x_write_reg(0x1A, 0x0A);
		isl7998x_write_reg(0x1B, 0x61);
		isl7998x_write_reg(0x1C, 0x7A);
		isl7998x_write_reg(0x1D, 0x0F);
		isl7998x_write_reg(0x1E, 0x8C);
		isl7998x_write_reg(0x1F, 0x02);
		isl7998x_write_reg(0x20, 0x00);
		isl7998x_write_reg(0x21, 0x0C);
		isl7998x_write_reg(0x22, 0x00);
		isl7998x_write_reg(0x23, 0x00);
		isl7998x_write_reg(0x24, 0x00);
		isl7998x_write_reg(0x25, 0xF0);
		isl7998x_write_reg(0x26, 0x00);
		isl7998x_write_reg(0x27, 0x00);
		isl7998x_write_reg(0x28, 0x01);
		isl7998x_write_reg(0x29, 0x0E);
		isl7998x_write_reg(0x2A, 0x00);
		isl7998x_write_reg(0x2B, 0x19);
		isl7998x_write_reg(0x2C, 0x18);
		isl7998x_write_reg(0x2D, 0xF1);
		isl7998x_write_reg(0x2E, 0x00);
		isl7998x_write_reg(0x2F, 0xF1);
		isl7998x_write_reg(0x30, 0x00);
		isl7998x_write_reg(0x31, 0x00);
		isl7998x_write_reg(0x32, 0x00);
		isl7998x_write_reg(0x33, 0xC0);
		isl7998x_write_reg(0x34, 0x18);
		isl7998x_write_reg(0x35, 0x00);
		isl7998x_write_reg(0x36, 0x00);
		if (lanes == 1)
			isl7998x_write_reg(0x00, 0x02);
		else
			isl7998x_write_reg(0x00, 0x01);
		isl7998x_write_reg(0xFF, 0x05);
	} else if (chip_id == CHIP_ID_79987) {
		// Page 5
		isl7998x_write_reg(0xFF, 0x05);
		reg = isl7998x_read_reg(0x00);
		reg &= ~0x80;
		isl7998x_write_reg(0x00, reg);  /* clear PowerDown */

		// Page 0
		isl7998x_write_reg(0xFF, 0x00);
		isl7998x_write_reg(0x02, 0x1F);  /* set Reset */

		// Default setting
		isl7998x_write_reg(0xFF, 0x00);
		isl7998x_write_reg(0x03, 0x00);
		isl7998x_write_reg(0x0C, 0xC9);
		isl7998x_write_reg(0x0D, 0xC9);
		isl7998x_write_reg(0x0E, 0xC9);
		isl7998x_write_reg(0x10, 0x01);
		isl7998x_write_reg(0x11, 0x03);
		isl7998x_write_reg(0x12, 0x00);
		isl7998x_write_reg(0x13, 0x00);
		isl7998x_write_reg(0x14, 0x00);

		isl7998x_write_reg(0xFF, 0x05);
		isl7998x_write_reg(0x00, 0x02);
		isl7998x_write_reg(0x01, 0x85);
		isl7998x_write_reg(0x02, 0xA0);
		isl7998x_write_reg(0x03, 0x18);
		isl7998x_write_reg(0x05, 0x40);
		isl7998x_write_reg(0x06, 0x40);
		isl7998x_write_reg(0x10, 0x05);
		isl7998x_write_reg(0x11, 0xA0);
		isl7998x_write_reg(0x20, 0x00);
		isl7998x_write_reg(0x21, 0x0C);
		isl7998x_write_reg(0x22, 0x00);
		isl7998x_write_reg(0x23, 0x00);
		isl7998x_write_reg(0x24, 0x00);
		isl7998x_write_reg(0x25, 0xF0);
		isl7998x_write_reg(0x26, 0x00);
		isl7998x_write_reg(0x27, 0x00);
		isl7998x_write_reg(0x2A, 0x00);
		isl7998x_write_reg(0x2B, 0x19);
		isl7998x_write_reg(0x2C, 0x18);
		isl7998x_write_reg(0x2D, 0xF1);
		isl7998x_write_reg(0x2E, 0x00);
		isl7998x_write_reg(0x2F, 0xF1);
		isl7998x_write_reg(0x30, 0x00);
		isl7998x_write_reg(0x31, 0x00);
		isl7998x_write_reg(0x32, 0x00);
		isl7998x_write_reg(0x33, 0xC0);
		isl7998x_write_reg(0x34, 0x18);
		isl7998x_write_reg(0x36, 0x00);

		// Page 0
		isl7998x_write_reg(0xFF, 0x00);
		isl7998x_write_reg(0x02, 0x10);  /* clear CReset_CH1 ~ Reset_CH4 */

		isl7998x_write_reg(0xFF, 0x0F);
		isl7998x_write_reg(0x08, 0x14);
		isl7998x_write_reg(0x2F, 0xE6);
		isl7998x_write_reg(0x33, 0x85);
		isl7998x_write_reg(0x45, 0x11);
		isl7998x_write_reg(0xE7, 0x00);

		// Page 0
		isl7998x_write_reg(0xFF, 0x00);
		isl7998x_write_reg(0x07, ((g_isl7998x_sync_ch & 0x03) << 5) | 0x02);
		isl7998x_write_reg(0x08, 0x1F);
		isl7998x_write_reg(0x09, 0x43);
		isl7998x_write_reg(0x0A, 0x4F);
		if (lanes == 1)
			isl7998x_write_reg(0x0B, 0x41);
		else
			isl7998x_write_reg(0x0B, 0x40);

		// Page 1
		isl7998x_write_reg(0xFF, 0x01);
		isl7998x_write_reg(0x02, 0x48); // 27MHz Input
		isl7998x_write_reg(0x07, 0x12); // VA_HI[5:4]=1, HA_HI[1:0]=2
		isl7998x_write_reg(0x08, 0x18); // VD_LO
		isl7998x_write_reg(0x09, 0x20); // VA_LO
		isl7998x_write_reg(0x0A, 0x08); // HD_LO
		isl7998x_write_reg(0x0B, 0xD0); // HA_LO
		isl7998x_write_reg(0x0C, 0xCC); // CNTRL1
		if (g_isl7998x_ch1_std >= 0)
			isl7998x_write_reg(0x1C, g_isl7998x_ch1_std);
		else
			isl7998x_write_reg(0x1C, 0x07); // Standard Auto Detection
		isl7998x_write_reg(0x1D, 0xFF); // Recognize All Standards
		isl7998x_write_reg(0x28, 0x08); // Use search VMODE
		isl7998x_write_reg(0x2F, 0xE6); // CCS Blue Color
		isl7998x_write_reg(0x33, 0xF2); // 50Hz Free run, Medium YNR
		isl7998x_write_reg(0x3B, 0x04); // Power Down Short Detection
		isl7998x_write_reg(0x3D, 0x00); // Data conversion
		isl7998x_write_reg(0xFF, 0x01);

		// Page 2
		isl7998x_write_reg(0xFF, 0x02);
		isl7998x_write_reg(0x02, 0x48); // 27MHz Input
		isl7998x_write_reg(0x07, 0x12); // VA_HI[5:4]=1, HA_HI[1:0]=2
		isl7998x_write_reg(0x08, 0x18); // VD_LO
		isl7998x_write_reg(0x09, 0x20); // VA_LO
		isl7998x_write_reg(0x0A, 0x08); // HD_LO
		isl7998x_write_reg(0x0B, 0xD0); // HA_LO
		isl7998x_write_reg(0x0C, 0xCC); // CNTRL1
		if (g_isl7998x_ch2_std >= 0)
			isl7998x_write_reg(0x1C, g_isl7998x_ch2_std);
		else
			isl7998x_write_reg(0x1C, 0x07); // Standard Auto Detection
		isl7998x_write_reg(0x1D, 0xFF); // Recognize All Standards
		isl7998x_write_reg(0x28, 0x08); // Use search VMODE
		isl7998x_write_reg(0x2F, 0xE6); // CCS Blue Color
		isl7998x_write_reg(0x33, 0xF2); // 50Hz Free run, Medium YNR
		isl7998x_write_reg(0x3B, 0x04); // Power Down Short Detection
		isl7998x_write_reg(0x3D, 0x00); // Data conversion
		isl7998x_write_reg(0xFF, 0x02);

		// Page 3
		isl7998x_write_reg(0xFF, 0x03);
		isl7998x_write_reg(0x02, 0x48); // 27MHz Input
		isl7998x_write_reg(0x07, 0x12); // VA_HI[5:4]=1, HA_HI[1:0]=2
		isl7998x_write_reg(0x08, 0x18); // VD_LO
		isl7998x_write_reg(0x09, 0x20); // VA_LO
		isl7998x_write_reg(0x0A, 0x08); // HD_LO
		isl7998x_write_reg(0x0B, 0xD0); // HA_LO
		isl7998x_write_reg(0x0C, 0xCC); // CNTRL1
		if (g_isl7998x_ch3_std >= 0)
			isl7998x_write_reg(0x1C, g_isl7998x_ch3_std);
		else
			isl7998x_write_reg(0x1C, 0x07); // Standard Auto Detection
		isl7998x_write_reg(0x1D, 0xFF); // Recognize All Standards
		isl7998x_write_reg(0x28, 0x08); // Use search VMODE
		isl7998x_write_reg(0x2F, 0xE6); // CCS Blue Color
		isl7998x_write_reg(0x33, 0xF2); // 50Hz Free run, Medium YNR
		isl7998x_write_reg(0x3B, 0x04); // Power Down Short Detection
		isl7998x_write_reg(0x3D, 0x00); // Data conversion
		isl7998x_write_reg(0xFF, 0x03);

		// Page 4
		isl7998x_write_reg(0xFF, 0x04);
		isl7998x_write_reg(0x02, 0x48); // 27MHz Input
		isl7998x_write_reg(0x07, 0x12); // VA_HI[5:4]=1, HA_HI[1:0]=2
		isl7998x_write_reg(0x08, 0x18); // VD_LO
		isl7998x_write_reg(0x09, 0x20); // VA_LO
		isl7998x_write_reg(0x0A, 0x08); // HD_LO
		isl7998x_write_reg(0x0B, 0xD0); // HA_LO
		isl7998x_write_reg(0x0C, 0xCC); // CNTRL1
		if (g_isl7998x_ch4_std >= 0)
			isl7998x_write_reg(0x1C, g_isl7998x_ch4_std);
		else
			isl7998x_write_reg(0x1C, 0x07); // Standard Auto Detection
		isl7998x_write_reg(0x1D, 0xFF); // Recognize All Standards
		isl7998x_write_reg(0x28, 0x08); // Use search VMODE
		isl7998x_write_reg(0x2F, 0xE6); // CCS Blue Color
		isl7998x_write_reg(0x33, 0xF2); // 50Hz Free run, Medium YNR
		isl7998x_write_reg(0x3B, 0x04); // Power Down Short Detection
		isl7998x_write_reg(0x3D, 0x00); // Data conversion
		isl7998x_write_reg(0xFF, 0x04);

		// Page 5
		isl7998x_write_reg(0xFF, 0x05);
		if (lanes == 1)
			isl7998x_write_reg(0x00, 0x02);
		else
			isl7998x_write_reg(0x00, 0x01);
//		isl7998x_write_reg(0x01, 0x05);  //For field mode
		isl7998x_write_reg(0x01, 0x25);  //For frame mode
		isl7998x_write_reg(0x02, 0xA0);
		isl7998x_write_reg(0x03, 0x10);
		isl7998x_write_reg(0x04, 0xE4);
		isl7998x_write_reg(0x05, 0x00);
		isl7998x_write_reg(0x06, 0x60);
		isl7998x_write_reg(0x07, 0x2B);
		isl7998x_write_reg(0x08, 0x02);
		isl7998x_write_reg(0x09, 0x00);
		isl7998x_write_reg(0x0A, 0x62);
		isl7998x_write_reg(0x0B, 0x02);
		isl7998x_write_reg(0x0C, 0x36);
		isl7998x_write_reg(0x0D, 0x00);
		isl7998x_write_reg(0x0E, 0x6C);
		isl7998x_write_reg(0x0F, 0x00);
		isl7998x_write_reg(0x10, 0x05);
		isl7998x_write_reg(0x11, 0xA0);
		isl7998x_write_reg(0x12, 0x77);
		isl7998x_write_reg(0x13, 0x17);
		isl7998x_write_reg(0x14, 0x08);
		isl7998x_write_reg(0x15, 0x38);
		isl7998x_write_reg(0x16, 0x14);
		isl7998x_write_reg(0x17, 0xF6);
		isl7998x_write_reg(0x18, 0x00);
		isl7998x_write_reg(0x19, 0x17);
		isl7998x_write_reg(0x1A, 0x0A);
		isl7998x_write_reg(0x1B, 0x71);
		isl7998x_write_reg(0x1C, 0x7A);
		isl7998x_write_reg(0x1D, 0x0F);
		isl7998x_write_reg(0x1E, 0x8C);
		isl7998x_write_reg(0x23, 0x0A);
		isl7998x_write_reg(0x24, 0x03);
		isl7998x_write_reg(0x25, 0xc0);
		isl7998x_write_reg(0x26, 0x08);
		isl7998x_write_reg(0x28, 0x01);
		isl7998x_write_reg(0x29, 0x0E);
		isl7998x_write_reg(0x2A, 0x00);
		isl7998x_write_reg(0x2B, 0x00);
		isl7998x_write_reg(0x38, 0x03);
		isl7998x_write_reg(0x39, 0xC0);
		isl7998x_write_reg(0x3A, 0x06);
		isl7998x_write_reg(0x3B, 0xB3);
		isl7998x_write_reg(0x3C, 0x00);
		isl7998x_write_reg(0x3D, 0xF1);

		/* change by BKang */
		isl7998x_write_reg(0x06, 0x00);
		isl7998x_write_reg(0x35, 0x00);

		// Page 0
		isl7998x_write_reg(0xFF, 0x00);
		isl7998x_write_reg(0x02, 0x00);  /* clear Reset */
	}

	msleep(10);

	if (mipi_csi2_info) {
		i = 0;

		/* wait for mipi sensor ready */
		mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
		while (((mipi_reg & 0x700) != 0x300) && (i < 10)) {
			mipi_reg = mipi_csi2_dphy_status(mipi_csi2_info);
			i++;
			msleep(50);
		}

		if (i >= 10) {
			pr_err("mipi csi2 can not receive sensor clk! MIPI_CSI_PHY_STATE = 0x%x.\n", mipi_reg);
			return -1;
		}

		i = 0;

		/* wait for mipi stable */
		mipi_reg = mipi_csi2_get_error1(mipi_csi2_info);
		while ((mipi_reg != 0x0) && (i < 10)) {
			mipi_reg = mipi_csi2_get_error1(mipi_csi2_info);
			i++;
			msleep(10);
		}

		if (i >= 10) {
			pr_err("mipi csi2 can not reveive data correctly! MIPI_CSI_ERR1 = 0x%x.\n", mipi_reg);
			return -1;
		}
	}

	return retval;
}

/* True if all channel power offed */
static int isl7998x_can_reset(void)
{
	int i = 0;
	for (; i < SENSOR_NUM; i++) {
		if (isl7998x_state[i])
			return false;
	}
	return true;
}

/* --------------- IOCTL functions from v4l2_int_ioctl_desc --------------- */

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	struct sensor_data *sensor = s->priv;

	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	memset(p, 0, sizeof(*p));
	p->u.bt656.clock_curr = sensor->mclk;
	pr_debug("   clock_curr=mclk=%d\n", sensor->mclk);
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.bt_sync_correct = 1;  /* Indicate external vsync */

	return 0;
}

/*!
 * ioctl_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct sensor_data *sensor = s->priv;

	sensor->on = on;

	/* Reset channel state if power off */
	if (!on)
		isl7998x_state[sensor->v_channel] = false;

	if (isl7998x_can_reset())
		isl7998x_started = 0;

	return 0;
}

/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor_data *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_debug("   type is not " \
			"V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
			a->type);
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor_data *sensor = s->priv;

	f->fmt.pix = sensor->pix;

	return 0;
}

static int ioctl_try_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor_data *sensor = s->priv;

	if (sensor->i2c_client != NULL) {
		g_isl7998x_width =  f->fmt.pix.width;
		if (chip_id == CHIP_ID_79987)
			g_isl7998x_frame_height =  f->fmt.pix.height;
		else if (chip_id == CHIP_ID_79985)
			g_isl7998x_field_height =  f->fmt.pix.height;
	}
	sensor->pix.width = g_isl7998x_width;
	if (chip_id == CHIP_ID_79987)
		sensor->pix.height = g_isl7998x_frame_height;
	else if (chip_id == CHIP_ID_79985)
		sensor->pix.height = g_isl7998x_field_height;

	ioctl_dev_init(s);
	return 0;
}

static int isl7998x_get_brightness(int channel)
{
	int ret = 0;
	isl7998x_write_reg(0xFF, channel);
	ret = isl7998x_read_reg(0x10);
	return ret;
}

static int isl7998x_get_contrast(int channel)
{
	int ret = 0;
	isl7998x_write_reg(0xFF, channel);
	ret = isl7998x_read_reg(0x11);
	return ret;
}

static int isl7998x_get_hue(int channel)
{
	int ret = 0;
	isl7998x_write_reg(0xFF, channel);
	ret = isl7998x_read_reg(0x15);
	return ret;
}

static int isl7998x_get_saturation(int channel)
{
	int ret = 0;
	isl7998x_write_reg(0xFF, channel);
	ret = isl7998x_read_reg(0x13);
	return ret;
}

static int isl7998x_get_sharpness(int channel)
{
	int ret = 0;
	isl7998x_write_reg(0xFF, channel);
	ret = isl7998x_read_reg(0x12);
	if (ret > 0)
		ret = ret & 0x3f;
	return ret;
}

static int isl7998x_set_brightness(int value, int channel)
{
	int ret = 0;
	isl7998x_write_reg(0xFF, channel);
	ret = isl7998x_write_reg(0x10, value);
	return ret;
}

static int isl7998x_set_contrast(int value, int channel)
{
	int ret = 0;
	isl7998x_write_reg(0xFF, channel);
	ret = isl7998x_write_reg(0x11, value);
	return ret;
}

static int isl7998x_set_hue(int value, int channel)
{
	int ret = 0;
	isl7998x_write_reg(0xFF, channel);
	ret = isl7998x_write_reg(0x15, value);
	return ret;
}

static int isl7998x_set_saturation(int value, int channel)
{
	int ret = 0;
	isl7998x_write_reg(0xFF, channel);
	ret  = isl7998x_write_reg(0x13, value);
	ret |= isl7998x_write_reg(0x14, value);
	return ret;
}

static int isl7998x_set_sharpness(int value, int channel)
{
	int ret = 0;
	isl7998x_write_reg(0xFF, channel);
	value = value & 0x3f;
	ret = isl7998x_write_reg(0x12, value);
	return ret;
}

/*!
 * ioctl_g_ctrl - V4L2 sensor interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int ioctl_g_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	struct sensor_data *sensor = s->priv;
	int ret = 0;
	int vch = sensor->v_channel + 1; /* The channel in isl7998x is base on 1 */
	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		vc->value = isl7998x_get_brightness(vch);
		break;
	case V4L2_CID_CONTRAST:
		vc->value = isl7998x_get_contrast(vch);
		break;
	case V4L2_CID_HUE:
		vc->value = isl7998x_get_hue(vch);
		break;
	case V4L2_CID_SATURATION:
		vc->value = isl7998x_get_saturation(vch);
		break;
	case V4L2_CID_SHARPNESS:
		vc->value = isl7998x_get_sharpness(vch);
		break;
	default:
		pr_debug("  type is unknow - %d\n",vc->id);
		ret = -EINVAL;
		break;
	}
	return ret;
}

/*!
 * ioctl_s_ctrl - V4L2 sensor interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int ioctl_s_ctrl(struct v4l2_int_device *s, struct v4l2_control *vc)
{
	struct sensor_data *sensor = s->priv;
	int ret = 0;
	int vch = sensor->v_channel + 1; /* The channel in isl7998x is base on 1 */
	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		ret = isl7998x_set_brightness(vc->value, vch);
		break;
	case V4L2_CID_CONTRAST:
		ret = isl7998x_set_contrast(vc->value, vch);
		break;
	case V4L2_CID_HUE:
		ret = isl7998x_set_hue(vc->value, vch);
		break;
	case V4L2_CID_SATURATION:
		ret = isl7998x_set_saturation(vc->value, vch);
		break;
	case V4L2_CID_SHARPNESS:
		ret = isl7998x_set_sharpness(vc->value, vch);
		break;
	default:
		pr_debug("  tpye is unknow - %d\n",vc->id);
		ret = -EINVAL;
		break;
	}
	return ret;
}

/*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{
	struct sensor_data *sensor = s->priv;

	if (fsize->index > 0)
		return -EINVAL;

	fsize->pixel_format = sensor->pix.pixelformat;
	fsize->discrete.width = sensor->pix.width;
	fsize->discrete.height = sensor->pix.height;
	return 0;
}

/*!
 * ioctl_enum_frameintervals - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_frameintervals(struct v4l2_int_device *s,
					 struct v4l2_frmivalenum *fival)
{
	if (fival->index > 0)
		return -EINVAL;

	if (fival->pixel_format == 0 || fival->width == 0 ||
			fival->height == 0) {
		pr_warning("Please assign pixelformat, width and height.\n");
		return -EINVAL;
	}

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;
	fival->discrete.denominator = 25;

	return 0;
}

/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	((struct v4l2_dbg_chip_ident *)id)->match.type =
					V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name,
		"ovisl7998x_mipi_decoder");

	return 0;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	return 0;
}

/*!
 * ioctl_enum_fmt_cap - V4L2 sensor interface handler for VIDIOC_ENUM_FMT
 * @s: pointer to standard V4L2 device structure
 * @fmt: pointer to standard V4L2 fmt description structure
 *
 * Return 0.
 */
static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
			      struct v4l2_fmtdesc *fmt)
{
	struct sensor_data *sensor = s->priv;

	if (fmt->index > 0) /* only 1 pixelformat support so far */
		return -EINVAL;

	fmt->pixelformat = sensor->pix.pixelformat;

	return 0;
}

/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct sensor_data *sensor = s->priv;
	int ret = 0;
	void *mipi_csi2_info;
	int rst_en = 0;

	ISL7998X_LOCK();

	if (!isl7998x_can_reset()) {
		printk(KERN_NOTICE "isl7998x_mipi: in used, skip reset\n");
		goto done;
	}

	sensor->on = true;

	if (!isl7998x_started) {
		mipi_csi2_info = mipi_csi2_get_info();

		/* enable mipi csi2 */
		if (mipi_csi2_info)
			mipi_csi2_enable(mipi_csi2_info);
		else {
			printk(KERN_ERR "%s() in %s: Fail to get mipi_csi2_info!\n",
			       __func__, __FILE__);
			ret = -EPERM;
			goto done;
		}

		ret = isl7998x_hardware_init(sensor);
		isl7998x_started = 1;
	}

done:
	/* Reset the decoder, the sensor->v_channel is 0 ~ 3,
	 * The isl7998x_reset_channel() need from 1 ~ 4 */
	switch (sensor->v_channel) {
	case 0:
		rst_en = g_isl7998x_ch1_rst_en;
		break;
	case 1:
		rst_en = g_isl7998x_ch2_rst_en;
		break;
	case 2:
		rst_en = g_isl7998x_ch3_rst_en;
		break;
	case 3:
		rst_en = g_isl7998x_ch4_rst_en;
		break;
	default:
		break;
	}

	if (rst_en)
		isl7998x_reset_channel(sensor->v_channel+1);

	isl7998x_state[sensor->v_channel] = true;

	ISL7998X_UNLOCK();

	return ret;
}

/*!
 * ioctl_dev_exit - V4L2 sensor interface handler for vidioc_int_dev_exit_num
 * @s: pointer to standard V4L2 device structure
 *
 * Delinitialise the device when slave detaches to the master.
 */
static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	struct sensor_data *sensor = s->priv;
	void *mipi_csi2_info;

	ISL7998X_LOCK();

	if (!isl7998x_can_reset()) {
		printk(KERN_NOTICE "isl7998x_mipi: in used, skip reset\n");
		goto done;
	}

	if (isl7998x_started) {
		mipi_csi2_info = mipi_csi2_get_info();

		/* disable mipi csi2 */
		if (mipi_csi2_info)
			if (mipi_csi2_get_status(mipi_csi2_info))
				mipi_csi2_disable(mipi_csi2_info);

		isl7998x_started = 0;
	}

done:
	ISL7998X_UNLOCK();

	return 0;
}

/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
static struct v4l2_int_ioctl_desc isl7998x_ioctl_desc[] = {
	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func *) ioctl_dev_init},
	{vidioc_int_dev_exit_num, ioctl_dev_exit},
	{vidioc_int_s_power_num, (v4l2_int_ioctl_func *) ioctl_s_power},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func *) ioctl_g_ifparm},
/*	{vidioc_int_g_needs_reset_num,
				(v4l2_int_ioctl_func *)ioctl_g_needs_reset}, */
/*	{vidioc_int_reset_num, (v4l2_int_ioctl_func *)ioctl_reset}, */
	{vidioc_int_init_num, (v4l2_int_ioctl_func *) ioctl_init},
	{vidioc_int_enum_fmt_cap_num,
				(v4l2_int_ioctl_func *) ioctl_enum_fmt_cap},
	{vidioc_int_try_fmt_cap_num,
				(v4l2_int_ioctl_func *)ioctl_try_fmt_cap},
	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func *) ioctl_g_fmt_cap},
/*	{vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func *) ioctl_s_fmt_cap}, */
	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func *) ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func *) ioctl_s_parm},
/*	{vidioc_int_queryctrl_num, (v4l2_int_ioctl_func *)ioctl_queryctrl}, */
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *) ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *) ioctl_s_ctrl},
	{vidioc_int_enum_framesizes_num,
				(v4l2_int_ioctl_func *) ioctl_enum_framesizes},
	{vidioc_int_enum_frameintervals_num,
				(v4l2_int_ioctl_func *) ioctl_enum_frameintervals},
	{vidioc_int_g_chip_ident_num,
				(v4l2_int_ioctl_func *) ioctl_g_chip_ident},
};

static struct v4l2_int_slave isl7998x_slave[SENSOR_NUM] = {
	{
	.ioctls = isl7998x_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(isl7998x_ioctl_desc),
	},

	{
	.ioctls = isl7998x_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(isl7998x_ioctl_desc),
	},

	{
	.ioctls = isl7998x_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(isl7998x_ioctl_desc),
	},

	{
	.ioctls = isl7998x_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(isl7998x_ioctl_desc),
	}
};

static struct v4l2_int_device isl7998x_int_device[SENSOR_NUM] = {
	{
		.module = THIS_MODULE,
		.name = "isl7998x",
		.type = v4l2_int_type_slave,
		.u = {
			.slave = &isl7998x_slave[0],
		},
	}, 

	{
		.module = THIS_MODULE,
		.name = "isl7998x",
		.type = v4l2_int_type_slave,
		.u = {
			.slave = &isl7998x_slave[1],
		},
	}, 

	{
		.module = THIS_MODULE,
		.name = "isl7998x",
		.type = v4l2_int_type_slave,
		.u = {
			.slave = &isl7998x_slave[2],
		},
	}, 

	{
		.module = THIS_MODULE,
		.name = "isl7998x",
		.type = v4l2_int_type_slave,
		.u = {
			.slave = &isl7998x_slave[3],
		},
	}
};

static ssize_t isl7998x_get_channels_status_attr(struct device *dev,
						 struct device_attribute *attr,
						 char *buf)
{
	int values[4];

	isl7998x_write_reg(0xFF, 0x00);
	values[0] = isl7998x_read_reg(0x1B);
	values[1] = isl7998x_read_reg(0x1C);
	values[2] = isl7998x_read_reg(0x1D);
	values[3] = isl7998x_read_reg(0x1E);
	isl7998x_write_reg(0xFF, 0x00);

	return scnprintf(buf, PAGE_SIZE, "0x%02X,0x%02X,0x%02X,0x%02X\n",
			 values[0], values[1], values[2],values[3]);
}

static ssize_t isl7998x_get_device_interrupt_status_attr(struct device *dev,
							 struct device_attribute *attr,
							 char *buf)
{
	int value;

	isl7998x_write_reg(0xFF, 0x00);
	value = isl7998x_read_reg(0x10);
	isl7998x_write_reg(0xFF, 0x00);

	return scnprintf(buf, PAGE_SIZE, "0x%02X\n", value);
}

static ssize_t isl7998x_get_mipi_csi_errors_attr(struct device *dev,
						 struct device_attribute *attr,
						 char *buf)
{
	void *mipi_csi2_info = mipi_csi2_get_info();
	unsigned int values[2];

	values[0] = mipi_csi2_get_error1(mipi_csi2_info);
	values[1] = mipi_csi2_get_error2(mipi_csi2_info);

	return scnprintf(buf, PAGE_SIZE, "0x%08X,0x%08X\n", values[0], values[1]);
}

static ssize_t isl7998x_get_mipi_csi_phy_status_attr(struct device *dev,
						     struct device_attribute *attr,
						     char *buf)
{
	void *mipi_csi2_info = mipi_csi2_get_info();
	unsigned int value;

	value = mipi_csi2_dphy_status(mipi_csi2_info);

	return scnprintf(buf, PAGE_SIZE, "0x%08X\n", value);
}

DEVICE_ATTR(channels_status, S_IRUGO, isl7998x_get_channels_status_attr, NULL);
DEVICE_ATTR(device_interrupt_status, S_IRUGO, isl7998x_get_device_interrupt_status_attr, NULL);
DEVICE_ATTR(mipi_csi_errors, S_IRUGO, isl7998x_get_mipi_csi_errors_attr, NULL);
DEVICE_ATTR(mipi_csi_phy_status, S_IRUGO, isl7998x_get_mipi_csi_phy_status_attr, NULL);

static struct attribute *isl7998x_attrs[] = {
	&dev_attr_channels_status.attr,
	&dev_attr_device_interrupt_status.attr,
	&dev_attr_mipi_csi_errors.attr,
	&dev_attr_mipi_csi_phy_status.attr,
	NULL
};

static const struct attribute_group isl7998x_attr_group = {
	.attrs = isl7998x_attrs,
};

static const struct attribute_group *isl7998x_attr_groups[] = {
	&isl7998x_attr_group,
	NULL,
};

static int isl7998x_misc_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int isl7998x_misc_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long isl7998x_misc_ioctl(struct file *file, unsigned int cmd,
			        unsigned long arg)
{
	return 0;
}

static struct file_operations isl7998x_misc_fops = {
	.owner		= THIS_MODULE,
	.open		= isl7998x_misc_open,
	.release	= isl7998x_misc_release,
	.unlocked_ioctl	= isl7998x_misc_ioctl,
};

static struct miscdevice isl7998x_misc = {
	.name	= "isl7998x",
	.minor	= MISC_DYNAMIC_MINOR,
	.fops	= &isl7998x_misc_fops,
	.groups	= isl7998x_attr_groups
};

static int isl7998x_misc_register(struct i2c_client *client)
{
	int ret;
	isl7998x_misc.parent = &client->dev;

	ret = misc_register(&isl7998x_misc);
	if (ret) {
		dev_err(&client->dev, "misc: %s register failed, err: %d\n",
			isl7998x_misc.name, ret);
		return ret;
	}

	dev_info(&client->dev, "misc: %s registered.\n", isl7998x_misc.name);

	return 0;
}

static int isl7998x_misc_unregister(struct i2c_client *client)
{
	int ret;

	ret = misc_deregister(&isl7998x_misc);
	if (ret) {
		dev_err(&client->dev, "misc: %s unregister failed, err: %d\n",
			isl7998x_misc.name, ret);
		return ret;
	}

	dev_info(&client->dev, "misc: %s unregistered.\n", isl7998x_misc.name);

	return 0;
}

/*!
 * isl7998x I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int isl7998x_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	int retval;
	int i;

	/* Set initial values for the sensor struct. */
	memset(&isl7998x_state, 0, sizeof(isl7998x_state));
	memset(&isl7998x_data[0], 0, sizeof(isl7998x_data[0]));
	isl7998x_data[0].sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(isl7998x_data[0].sensor_clk)) {
		/* assuming clock enabled by default */
		isl7998x_data[0].sensor_clk = NULL;
		dev_err(dev, "clock-frequency missing or invalid\n");
		return PTR_ERR(isl7998x_data[0].sensor_clk);
	}

	retval = of_property_read_u32(dev->of_node, "mclk",
					&(isl7998x_data[0].mclk));
	if (retval) {
		dev_err(dev, "mclk missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "mclk_source",
					(u32 *) &(isl7998x_data[0].mclk_source));
	if (retval) {
		dev_err(dev, "mclk_source missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "csi_id",
					&(isl7998x_data[0].csi));
	if (retval) {
		dev_err(dev, "csi id missing or invalid\n");
		return retval;
	}

	g_isl7998x_reset_gpio = of_get_named_gpio_flags(dev->of_node,
							"reset-gpios", 0,
							&g_isl7998x_reset_gpio_flags);

	g_isl7998x_power_gpio = of_get_named_gpio_flags(dev->of_node,
							"power-gpios", 0,
							&g_isl7998x_power_gpio_flags);

	if (g_isl7998x_reset_gpio > 0) {
		if (gpio_request(g_isl7998x_reset_gpio, "isl7998x-reset") < 0)
			g_isl7998x_reset_gpio = -1;
	}

	if (g_isl7998x_power_gpio > 0) {
		if (gpio_request(g_isl7998x_power_gpio, "isl7998x-power") < 0)
			g_isl7998x_power_gpio = -1;
	}

	for (i = 0; i < 3; i++) {
		g_isl7998x_irq_gpios[i] = of_get_named_gpio(dev->of_node, "irq-gpios", i);
		if (g_isl7998x_irq_gpios[i] <= 0) {
			dev_warn(dev, "irq gpio of %d not found.", i);
			continue;
		}

		if (gpio_request(g_isl7998x_irq_gpios[i], g_isl7998x_irq_gpio_names[i]) < 0) {
			dev_warn(dev, "request gpio %d failed!",
				 g_isl7998x_irq_gpios[i]);
			g_isl7998x_irq_gpios[i] = -1;
		}
		else {
			gpio_direction_input(g_isl7998x_irq_gpios[i]);
		}
	}

	mutex_init(&g_isl7998x_lock);

	/* Power on the chip */
	isl7998x_chip_power(1);

	/* Leave reset state */
	isl7998x_chip_reset(0);

	clk_prepare_enable(isl7998x_data[0].sensor_clk);

	isl7998x_data[0].i2c_client = client;
	isl7998x_data[0].pix.pixelformat = V4L2_PIX_FMT_UYVY;
	isl7998x_data[0].streamcap.capturemode = 0;
	isl7998x_data[0].streamcap.timeperframe.denominator = 25;
	isl7998x_data[0].streamcap.timeperframe.numerator = 1;
	isl7998x_data[0].is_mipi = 1;

	isl7998x_write_reg(0xFF, 0x00);
	chip_id = isl7998x_read_reg(0x00);
	if ((chip_id != CHIP_ID_79985) && (chip_id != CHIP_ID_79987)) {
		pr_warning("isl7998x is not found, chip id reg 0x00 = 0x%x.\n", chip_id);
		clk_disable_unprepare(isl7998x_data[0].sensor_clk);
		return -ENODEV;
	}

	isl7998x_data[0].pix.width = g_isl7998x_width;
	if (chip_id == CHIP_ID_79987) {
		isl7998x_data[0].pix.height = g_isl7998x_frame_height;
		isl7998x_data[0].is_mipi_interlaced = 1;
	} else if (chip_id == CHIP_ID_79985)
		isl7998x_data[0].pix.height = g_isl7998x_field_height;

	memcpy(&isl7998x_data[1], &isl7998x_data[0], sizeof(struct sensor_data));
	memcpy(&isl7998x_data[2], &isl7998x_data[0], sizeof(struct sensor_data));
	memcpy(&isl7998x_data[3], &isl7998x_data[0], sizeof(struct sensor_data));

	isl7998x_data[1].i2c_client = NULL;
	isl7998x_data[2].i2c_client = NULL;
	isl7998x_data[3].i2c_client = NULL;

	isl7998x_data[0].ipu_id = 0;
	isl7998x_data[0].csi = 0;
	isl7998x_data[0].v_channel = 0;

	isl7998x_data[1].ipu_id = 0;
	isl7998x_data[1].csi = 1;
	isl7998x_data[1].v_channel = 1;

	isl7998x_data[2].ipu_id = 1;
	isl7998x_data[2].csi = 0;
	isl7998x_data[2].v_channel = 2;

	isl7998x_data[3].ipu_id = 1;
	isl7998x_data[3].csi = 1;
	isl7998x_data[3].v_channel = 3;

	isl7998x_int_device[0].priv = &isl7998x_data[0];
	isl7998x_int_device[1].priv = &isl7998x_data[1];
	isl7998x_int_device[2].priv = &isl7998x_data[2];
	isl7998x_int_device[3].priv = &isl7998x_data[3];
	v4l2_int_device_register(&isl7998x_int_device[0]);
	v4l2_int_device_register(&isl7998x_int_device[1]);
	v4l2_int_device_register(&isl7998x_int_device[2]);
	retval = v4l2_int_device_register(&isl7998x_int_device[3]);

	clk_disable_unprepare(isl7998x_data[0].sensor_clk);

	isl7998x_misc_register(client);

	pr_info("isl7998x_mipi is found\n");
	return retval;
}

/*!
 * isl7998x I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int isl7998x_remove(struct i2c_client *client)
{
	isl7998x_misc_unregister(client);

	v4l2_int_device_unregister(&isl7998x_int_device[3]);
	v4l2_int_device_unregister(&isl7998x_int_device[2]);
	v4l2_int_device_unregister(&isl7998x_int_device[1]);
	v4l2_int_device_unregister(&isl7998x_int_device[0]);

	return 0;
}

/*!
 * isl7998x init function
 *
 * @return  Error code indicating success or failure
 */
static __init int isl7998x_init(void)
{
	u8 err;

	err = i2c_add_driver(&isl7998x_i2c_driver);
	if (err != 0)
		pr_err("%s:driver registration failed, error=%d\n",
			__func__, err);

	return err;
}

/*!
 * ISL7998x cleanup function
 *
 * @return  Error code indicating success or failure
 */
static void __exit isl7998x_clean(void)
{
	i2c_del_driver(&isl7998x_i2c_driver);
}

module_init(isl7998x_init);
module_exit(isl7998x_clean);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("ISL7998x Video Decoder Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
