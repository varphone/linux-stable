/*
 *  Backlight driver for MTY065X Military Projector
 *
 *  Copyright (c) 2017		Varphone Wong
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/slab.h>
#include <linux/delay.h>

/* Max led pwm level number */
#define MTY065X_LED_PWM_LEVEL_MAX	10

/* The default mappings for led pwm level */
static u16 mty065x_led_pwm_level_default_map[] = {
	41, 60, 80, 110, 140, 180, 220, 260, 350, 600,
};

/* Max test pattern number */
#define MTY065X_TEST_PATTERN_MAX	9

/* Chip ID */
#define MTY065X_CHIP_ID_R		0xc0

/* Input Source Select */
#define MTY065X_INPUT_SELECT_R		0x06
#define MTY065X_INPUT_SELECT_W		0x05

/* External Video Source Format Select */
#define MTY065X_INPUT_FORMAT_R		0x08
#define MTY065X_INPUT_FORMAT_W		0x07

/* External Input Image Size */
#define MTY065X_INPUT_SIZE_R		0x2f
#define MTY065X_INPUT_SIZE_W		0x2e

/* Image Crop */
#define MTY065X_IMAGE_CROP_R		0x11
#define MTY065X_IMAGE_CROP_W		0x10

/* Display Image Curtain */
#define MTY065X_IMAGE_CURTAIN_R		0x17
#define MTY065X_IMAGE_CURTAIN_W		0x16

/* Image Freeze */
#define MTY065X_IMAGE_FREEZE_R		0x1b
#define MTY065X_IMAGE_FREEZE_W		0x1a

/* Display Size */
#define MTY065X_DISPLAY_SIZE_R		0x13
#define MTY065X_DISPLAY_SIZE_W		0x12

/* Keystone Correction Control */
#define MTY065X_KS_CORRECTION_R		0x89
#define MTY065X_KS_CORRECTION_W		0x88

/* Keystone Projection Pitch Angle */
#define MTY065X_KS_PROJECTION_R		0xbc
#define MTY065X_KS_PROJECTION_W		0xbb

/* RGB LED Control */
#define MTY065X_RGB_LED_CTRL_R		0x53
#define MTY065X_RGB_LED_CTRL_W		0x52

/* RGB LED PWM */
#define MTY065X_RGB_LED_PWM_R		0x55
#define MTY065X_RGB_LED_PWM_W		0x54

/* RGB LED MAX PWM */
#define MTY065X_RGB_LED_MAX_PWM_R	0x5d
#define MTY065X_RGB_LED_MAX_PWM_W	0x5c

/* Test Pattern Select */
#define MTY065X_TEST_PATTERN_R		0x0c
#define MTY065X_TEST_PATTERN_W		0x0b

/* Image Curtain */
struct image_curtain {
	u8	enable:1;
	u8	color:3;
	u8	rsv:4;
} __attribute__((packed));

struct dmdres {
	u16	x;
	u16	y;
	u16	w;
	u16	h;
};

/* Keystone correction control */
struct ks_correction {
	u8	enable:1;	/* Enable control */
	u8	rsv:7;		/* Resevered */
	u16	ratio;		/* Optical Throw Ratio */
	s16	offset;		/* Optical DMD Offset */
} __attribute__((packed));

/* Keystone Projection */
struct ks_projection {
	s16	angle;		/* Pitch Angle */
} __attribute__((packed));

/* RGB LED PWM */
struct rgb_led_pwm {
	u16	red;		/* Red LED PWM */
	u16	green;		/* Green LED PWM */
	u16	blue;		/* Blue LED PWM */
} __attribute__((packed));

struct mty065x_props {
	u16			display_size[4];
	u16			image_crop[4];
	struct image_curtain	image_curtain;
	u8			image_freeze;
	u8			input_format;
	u8			input_select;
	u16			input_size[2];
	struct ks_correction 	ks_correction;
	struct ks_projection	ks_projection;
	u16			led_pwm_level_map[MTY065X_LED_PWM_LEVEL_MAX+1];
	u8			rgb_led_ctrl;
	struct rgb_led_pwm	rgb_led_pwm;
	struct rgb_led_pwm	rgb_led_max_pwm;
	u8			test_pattern;
};

struct mty065x {
	struct device		*dev;
	struct i2c_client	*i2c;
	struct backlight_device	*bl;
	u16			chip_id;
	struct mty065x_props	props;
	int			props_saved;
	struct mty065x_props	saved_props;
};

#define dev_to_i2c_client(d)	container_of(d, struct i2c_client, dev)
#define dev_to_mty065x(d)	i2c_get_clientdata(dev_to_i2c_client(d))

static void mty065x_get_led_pwm_level(struct mty065x *mty, int *level);
static void mty065x_set_led_pwm_level(struct mty065x *mty, int level);
static void mty065x_set_rgb_led_ctrl_value(struct mty065x *mty, u8 value);

static void mty065x_set_backlight(struct mty065x *data, int brightness)
{
	mty065x_set_led_pwm_level(data, brightness);
	/* Turn the rgb led power if brightness == 0 */
	if (brightness == 0)
		mty065x_set_rgb_led_ctrl_value(data, 0);
	else
		mty065x_set_rgb_led_ctrl_value(data, 7);
}

static int mty065x_update_status(struct backlight_device *dev)
{
	struct backlight_properties *props = &dev->props;
	struct mty065x *mty = bl_get_data(dev);
	int power = max(props->power, props->fb_blank);
	int brightness = props->brightness;

	if (power)
		brightness = 0;

	mty065x_set_backlight(mty, brightness);

	return 0;
}

static int mty065x_get_brightness(struct backlight_device *dev)
{
	struct mty065x *mty = bl_get_data(dev);
	int level;

	mty065x_get_led_pwm_level(mty, &level);

	return level;
}

/* Check if given framebuffer device is the one bound to this backlight;
   return 0 if not, !=0 if it is. */
static int mty065x_check_fb(struct backlight_device *dev, struct fb_info *fbi)
{
	if (fbi) {
		if (of_machine_is_compatible("myzr,myimx6q")) {
			/* LVDS0 and LVDS1 only */
			if (strncmp(fbi->fix.id, "DISP4 BG", 8) == 0)
				return 1;
		} else if (of_machine_is_compatible("myzr,myimx6u")) {
			/* LVDS0 and LVDS1 only */
			if (strncmp(fbi->fix.id, "DISP3 BG", 8) == 0)
				return 1;
		}
	}
	return 0;
}

static const struct backlight_ops bl_ops = {
	.get_brightness		= mty065x_get_brightness,
	.update_status		= mty065x_update_status,
	.check_fb		= mty065x_check_fb,
};

static void mty065x_get_display_size(struct mty065x *mty)
{
	int ret;
	u16 size[2];

	ret = i2c_smbus_read_i2c_block_data(mty->i2c, MTY065X_DISPLAY_SIZE_R, 4, (u8*)&size[0]);
	if (ret < 0) {
		dev_warn(mty->dev, "read display size failed, err: %d\n", ret);
	} else {
		mty->props.display_size[0] = le16_to_cpu(size[0]);
		mty->props.display_size[1] = le16_to_cpu(size[1]);
	}
}

static void mty065x_set_display_size_value(struct mty065x *mty, u16 size[4])
{
	int ret;
	u16 data[4];

	dev_dbg(mty->dev, "display size: %u %u\n",
		size[0], size[1]);

	data[0] = cpu_to_le16(size[0]);
	data[1] = cpu_to_le16(size[1]);

	ret = i2c_smbus_write_i2c_block_data(mty->i2c, MTY065X_DISPLAY_SIZE_W,
					     4, (u8*)&data[0]);
	if (ret < 0) {
		dev_warn(mty->dev, "write display size failed, err: %d\n", ret);
	}
}

static void mty065x_set_display_size(struct mty065x *mty)
{
	mty065x_set_display_size_value(mty, mty->props.display_size);
}

static void mty065x_get_image_crop(struct mty065x *mty)
{
	int ret;
	u16 crop[4];

	ret = i2c_smbus_read_i2c_block_data(mty->i2c, MTY065X_IMAGE_CROP_R,
					    8, (u8*)&crop[0]);
	if (ret < 0) {
		dev_warn(mty->dev, "read image crop failed, err: %d\n", ret);
	} else {
		mty->props.image_crop[0] = le16_to_cpu(crop[0]);
		mty->props.image_crop[1] = le16_to_cpu(crop[1]);
		mty->props.image_crop[2] = le16_to_cpu(crop[2]);
		mty->props.image_crop[3] = le16_to_cpu(crop[3]);
	}
}

static void mty065x_set_image_crop_value(struct mty065x *mty, u16 crop[4])
{
	int ret;
	u16 data[4];

	dev_dbg(mty->dev, "image crop: %u,%u %u,%u\n",
		crop[0], crop[1], crop[2], crop[3]);

	data[0] = cpu_to_le16(crop[0]);
	data[1] = cpu_to_le16(crop[1]);
	data[2] = cpu_to_le16(crop[2]);
	data[3] = cpu_to_le16(crop[3]);

	ret = i2c_smbus_write_i2c_block_data(mty->i2c, MTY065X_IMAGE_CROP_W,
					     8, (u8*)&data[0]);
	if (ret < 0) {
		dev_warn(mty->dev, "write image crop failed, err: %d\n", ret);
	}
}

static void mty065x_set_image_crop(struct mty065x *mty)
{
	mty065x_set_image_crop_value(mty, mty->props.image_crop);
}

static void mty065x_get_image_curtain(struct mty065x *mty)
{
	int ret;

	ret = i2c_smbus_read_byte_data(mty->i2c, MTY065X_IMAGE_CURTAIN_R);
	if (ret < 0) {
		dev_warn(mty->dev, "read image curtain failed, err: %d\n", ret);
	} else {
		mty->props.image_curtain.enable = ret & 0x01;
		mty->props.image_curtain.color = (ret >> 1) & 0x07;
	}
}

static void mty065x_set_image_curtain(struct mty065x *mty)
{
	int ret;
	u8 *val = (u8*)&mty->props.image_curtain;

	ret = i2c_smbus_write_byte_data(mty->i2c, MTY065X_IMAGE_CURTAIN_W, *val);
	if (ret < 0) {
		dev_warn(mty->dev, "write image curtain failed, err: %d\n", ret);
	}
}

static void mty065x_get_image_freeze(struct mty065x *mty)
{
	int ret;

	ret = i2c_smbus_read_byte_data(mty->i2c, MTY065X_IMAGE_FREEZE_R);
	if (ret < 0 ) {
		dev_warn(mty->dev, "read image freeze failed, err: %d\n", ret);
	} else {
		mty->props.image_freeze = ret & 0xff;
	}
}

static void mty065x_set_image_freeze_value(struct mty065x *mty, u8 freeze)
{
	int ret;

	ret = i2c_smbus_write_byte_data(mty->i2c, MTY065X_IMAGE_FREEZE_W, freeze);
	if (ret < 0 ) {
		dev_warn(mty->dev, "write image freeze failed, err: %d\n", ret);
	}
}

static void mty065x_set_image_freeze(struct mty065x *mty)
{
	mty065x_set_image_freeze_value(mty, mty->props.image_freeze);
}

static void mty065x_get_input_format(struct mty065x *mty)
{
	int ret;

	ret = i2c_smbus_read_byte_data(mty->i2c, MTY065X_INPUT_FORMAT_R);
	if (ret < 0 ) {
		dev_warn(mty->dev, "read input format failed, err: %d\n", ret);
	} else {
		mty->props.input_format = ret & 0xff;
	}
}

static void mty065x_set_input_format(struct mty065x *mty)
{
	int ret;

	dev_dbg(mty->dev, "input format: %d\n", mty->props.input_format);

	ret = i2c_smbus_write_byte_data(mty->i2c, MTY065X_INPUT_FORMAT_W, mty->props.input_format);
	if (ret < 0 ) {
		dev_warn(mty->dev, "write input format failed, err: %d\n", ret);
	}
}

static void mty065x_get_input_select(struct mty065x *mty)
{
	int ret;

	ret = i2c_smbus_read_byte_data(mty->i2c, MTY065X_INPUT_SELECT_R);
	if (ret < 0 ) {
		dev_warn(mty->dev, "read input select failed, err: %d\n", ret);
	} else {
		mty->props.input_select = ret & 0xff;
	}
}

static void mty065x_set_input_select_value(struct mty065x *mty, u8 sel)
{
	int ret;

	dev_dbg(mty->dev, "input select: %d\n", sel);

	ret = i2c_smbus_write_byte_data(mty->i2c, MTY065X_INPUT_SELECT_W, sel);
	if (ret < 0 ) {
		dev_warn(mty->dev, "write input select failed, err: %d\n", ret);
	}
}

static void mty065x_set_input_select(struct mty065x *mty)
{
	mty065x_set_input_select_value(mty, mty->props.input_select);
}

static void mty065x_get_input_size(struct mty065x *mty)
{
	int ret;
	u16 data[2];

	ret = i2c_smbus_read_i2c_block_data(mty->i2c, MTY065X_INPUT_SIZE_R,
					    sizeof(data), (u8*)&data[0]);
	if (ret < 0) {
		dev_warn(mty->dev, "read input size failed, err: %d\n", ret);
	} else {
		mty->props.input_size[0] = le16_to_cpu(data[0]);
		mty->props.input_size[1] = le16_to_cpu(data[1]);
	}
}

static void mty065x_set_input_size_value(struct mty065x *mty, u16 size[2])
{
	int ret;
	u16 data[2];

	dev_dbg(mty->dev, "input size: %u x %u\n",
		size[0], size[1]);

	data[0] = cpu_to_le16(size[0]);
	data[1] = cpu_to_le16(size[1]);

	ret = i2c_smbus_write_i2c_block_data(mty->i2c, MTY065X_INPUT_SIZE_W,
					     sizeof(data), (u8*)&data[0]);
	if (ret < 0) {
		dev_warn(mty->dev, "write input size failed, err: %d\n", ret);
	}
}

static void mty065x_set_input_size(struct mty065x *mty)
{
	mty065x_set_input_size_value(mty, mty->props.input_size);
}

static void mty065x_get_ks_correction(struct mty065x *mty)
{
	int ret;
	u8* data = (u8*)&mty->props.ks_correction;
	
	ret = i2c_smbus_read_i2c_block_data(mty->i2c, MTY065X_KS_CORRECTION_R,
					    sizeof(struct ks_correction), data);
	if (ret < 0) {
		dev_warn(mty->dev, "read keystone correction failed, err: %d\n", ret);
	}
}

static void mty065x_set_ks_correction(struct mty065x *mty)
{
	int ret;
	u8* data = (u8*)&mty->props.ks_correction;
	
	ret = i2c_smbus_write_i2c_block_data(mty->i2c, MTY065X_KS_CORRECTION_W,
					    sizeof(struct ks_correction), data);
	if (ret < 0) {
		dev_warn(mty->dev, "write keystone correction failed, err: %d\n", ret);
	}
}

static void mty065x_get_ks_projection(struct mty065x *mty)
{
	int ret;
	u8* data = (u8*)&mty->props.ks_projection;
	
	ret = i2c_smbus_read_i2c_block_data(mty->i2c, MTY065X_KS_PROJECTION_R,
					    sizeof(struct ks_projection), data);
	if (ret < 0) {
		dev_warn(mty->dev, "read keystone projection failed, err: %d\n", ret);
	}
}

static void mty065x_set_ks_projection(struct mty065x *mty)
{
	int ret;
	u8* data = (u8*)&mty->props.ks_projection;
	
	ret = i2c_smbus_write_i2c_block_data(mty->i2c, MTY065X_KS_PROJECTION_W,
					    sizeof(struct ks_projection), data);
	if (ret < 0) {
		dev_warn(mty->dev, "write keystone projection failed, err: %d\n", ret);
	}
}

static void mty065x_get_rgb_led_ctrl(struct mty065x *mty)
{
	int ret;

	ret = i2c_smbus_read_byte_data(mty->i2c, MTY065X_RGB_LED_CTRL_R);
	if (ret < 0) {
		dev_warn(mty->dev, "read rgb led control failed, err: %d\n", ret);
	} else {
		mty->props.rgb_led_ctrl = ret & 0xff;
	}
}

static void mty065x_set_rgb_led_ctrl_value(struct mty065x *mty, u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(mty->i2c, MTY065X_RGB_LED_CTRL_W,
					value);
	if (ret < 0) {
		dev_warn(mty->dev, "write rgb led control failed, err: %d\n", ret);
	}
}

static void mty065x_set_rgb_led_ctrl(struct mty065x *mty)
{
	mty065x_set_rgb_led_ctrl_value(mty, mty->props.rgb_led_ctrl);
}

static void mty065x_get_rgb_led_pwm(struct mty065x *mty);
static void mty065x_set_rgb_led_pwm(struct mty065x *mty);

static void mty065x_get_led_pwm_level(struct mty065x *mty, int *level)
{
	int i, v;
	u16 *map;

	mty065x_get_rgb_led_pwm(mty);
	v = mty->props.rgb_led_pwm.red;
	if (v == 0) {
		*level = 0;
		return;
	}

	map = mty->props.led_pwm_level_map;
	for (i = 0; i < MTY065X_LED_PWM_LEVEL_MAX -1; i++) {
		if (v >= map[i] && v < map[i+1])
			break;
	}

	*level = i;
}

static void mty065x_set_led_pwm_level(struct mty065x *mty, int level)
{
	int pwm;
	level = level > MTY065X_LED_PWM_LEVEL_MAX ? MTY065X_LED_PWM_LEVEL_MAX : level;

	/* Skip level 0 */
	if (level <= 0)
		return;

	pwm = mty->props.led_pwm_level_map[level -1];
	mty->props.rgb_led_pwm.red = pwm;
	mty->props.rgb_led_pwm.green = pwm;
	mty->props.rgb_led_pwm.blue = pwm;
	mty065x_set_rgb_led_pwm(mty);
}

static void mty065x_get_rgb_led_pwm(struct mty065x *mty)
{
	int ret;
	u8* data = (u8*)&mty->props.rgb_led_pwm;

	ret = i2c_smbus_read_i2c_block_data(mty->i2c, MTY065X_RGB_LED_PWM_R,
					    sizeof(struct rgb_led_pwm), data);
	if (ret < 0) {
		dev_warn(mty->dev, "read rgb led pwm failed, err: %d\n", ret);
	}
}

static void mty065x_set_rgb_led_pwm(struct mty065x *mty)
{
	int ret;
	u8* data = (u8*)&mty->props.rgb_led_pwm;

	ret = i2c_smbus_write_i2c_block_data(mty->i2c, MTY065X_RGB_LED_PWM_W,
					    sizeof(struct rgb_led_pwm), data);
	if (ret < 0) {
		dev_warn(mty->dev, "write rgb_led_pwm failed, err: %d\n", ret);
	}
}

static void mty065x_get_rgb_led_max_pwm(struct mty065x *mty)
{
	int ret;
	u8* data = (u8*)&mty->props.rgb_led_max_pwm;

	ret = i2c_smbus_read_i2c_block_data(mty->i2c, MTY065X_RGB_LED_PWM_R,
					    sizeof(struct rgb_led_pwm), data);
	if (ret < 0) {
		dev_warn(mty->dev, "read rgb_led_max_pwm failed, err: %d\n", ret);
	}
}

static void mty065x_set_rgb_led_max_pwm(struct mty065x *mty)
{
	int ret;
	u8* data = (u8*)&mty->props.rgb_led_max_pwm;

	ret = i2c_smbus_write_i2c_block_data(mty->i2c, MTY065X_RGB_LED_MAX_PWM_W,
					     sizeof(struct rgb_led_pwm), data);
	if (ret < 0) {
		dev_warn(mty->dev, "write rgb_led_max_pwm failed, err: %d\n", ret);
	}
}

static void mty065x_get_test_pattern(struct mty065x *mty)
{
	int ret;
	u8 block[6];

	ret = i2c_smbus_read_i2c_block_data(mty->i2c, MTY065X_TEST_PATTERN_R,
					    sizeof(block), block);
	if (ret < 0) {
		dev_warn(mty->dev, "read test pattern failed, err: %d\n", ret);
	} else {
		if (block[0] != 0) {
			mty->props.test_pattern = (block[0] & 0x0f) + 1;
		}
	}
}

static void mty065x_set_test_pattern_value(struct mty065x *mty, u8 pattern)
{
	int len;
	int ret;
	u8 block[6];

	dev_dbg(mty->dev, "test pattern: %u\n", pattern);

	switch (pattern) {
	case 1: /* Solid Field */
		block[0] = 0x80 | 0x00;
		block[1] = 0x10 | 0x00; /* Red/Black */
		len = 2;
		break;
	case 2: /* Horizontal Ramp */
		block[0] = 0x80 | 0x01;
		block[1] = 0x70 | 0x00; /* White/Black */
		block[2] = 0x00; /* Start Value */
		block[3] = 0xff; /* Stop Value */
		len = 4;
		break;
	case 3: /* Vertical Ramp */
		block[0] = 0x80 | 0x02;
		block[1] = 0x70 | 0x00; /* White/Black */
		block[2] = 0x00; /* Start Value */
		block[3] = 0xff; /* Stop Value */
		len = 4;
		break;
	case 4: /* Horizontal Lines */
		block[0] = 0x80 | 0x03;
		block[1] = 0x20 | 0x00; /* Green/Black */
		block[2] = 4;	/* Foreground Line Width */
		block[3] = 8;	/* Background Line Width */
		len = 4;
		break;
	case 5: /* Diagonal Lines */
		block[0] = 0x80 | 0x04;
		block[1] = 0x00 | 0x07; /* Black/White */
		block[2] = 15;	/* Horizontal Spacing */
		block[3] = 15;	/* Vertical Spacing */
		len = 4;
		break;
	case 6: /* Vertical Lines */
		block[0] = 0x80 | 0x05;
		block[1] = 0x70 | 0x00; /* White/Black */
		block[2] = 8;	/* Foreground Line Width */
		block[3] = 4;	/* Background Line Width */
		len = 4;
		break;
	case 7: /* Horizontal & Veritcal Grid */
		block[0] = 0x80 | 0x06;
		block[1] = 0x70 | 0x00; /* White/Black */
		block[2] = 2;	/* Horizontal Foreground Line Width */
		block[3] = 8;	/* Horizontal Background Line Width */
		block[4] = 2;	/* Vertical Foreground Line Width */
		block[5] = 8;	/* Vertical Background Line Width */
		len = 6;
		break;
	case 8: /* Checkboard */
		block[0] = 0x80 | 0x07;
		block[1] = 0x70 | 0x00;
		block[2] = 16;	/* Horizontal Checkers (LSB) */
		block[3] = 0;	/* Horizontal Checkers (MSB) */
		block[4] = 12;	/* Vertical Checkers (LSB) */
		block[5] = 0;	/* Vertical Checkers (MSB) */
		len = 6;
		break;
	case 9: /* Color Bars */
		block[0] = 0x80 | 0x08;
		len = 1;
		break;
	default: /* Nothing */
		block[0] = 0x00;
		block[1] = 0x00;
		block[2] = 0x00;
		block[3] = 0x00;
		block[4] = 0x00;
		block[5] = 0x00;
		len = 6;
		break;
	}

	/* Update test pattern registers */
	ret = i2c_smbus_write_i2c_block_data(mty->i2c, MTY065X_TEST_PATTERN_W,
					     len, block);
	if (ret < 0) {
		dev_warn(mty->dev, "write test pattern failed, err: %d\n", ret);
	}
}

static void mty065x_set_test_pattern(struct mty065x *mty)
{
	mty065x_set_test_pattern_value(mty, mty->props.test_pattern);
}

static void mty065x_current(struct mty065x *mty)
{
	mty065x_get_image_curtain(mty);
	mty065x_get_image_freeze(mty);
	mty065x_get_input_size(mty);
	mty065x_get_test_pattern(mty);
}

static int mty065x_configure(struct mty065x *mty)
{
	u16 crop[4];
	u16 size[2];

	/* Freeze the image first */
	mty065x_set_image_freeze_value(mty, 1);
	mty065x_set_image_curtain(mty);

	/* The image crop for test pattern must equal display size */
	if (mty->props.input_select) {
		crop[0] = 0;
		crop[1] = 0;
		crop[2] = mty->props.display_size[0];
		crop[3] = mty->props.display_size[1];

		size[0] = mty->props.display_size[0];
		size[1] = mty->props.display_size[1];

		mty065x_set_image_crop_value(mty, crop);
		mty065x_set_display_size_value(mty, size);
		mty065x_set_input_format(mty);
		/* Input Source Select must be the last to apply the settings */
		mty065x_set_input_select_value(mty, 1);
		mty065x_set_test_pattern(mty);
	} else {
		/* Fix image crop if changed by test pattern */
		if (mty->props.image_crop[2] != mty->props.input_size[0] ||
		    mty->props.image_crop[3] != mty->props.input_size[1]) {
			mty->props.image_crop[2] = mty->props.input_size[0];
			mty->props.image_crop[3] = mty->props.input_size[1];
		}

		mty065x_set_image_crop(mty);
		mty065x_set_input_size(mty);
		mty065x_set_display_size(mty);
		mty065x_set_input_format(mty);
		/* Input Source Select must be the last to apply the settings */
		mty065x_set_input_select(mty);
		mty065x_set_test_pattern_value(mty, 0);
	}

	/* Unfreeze the image after set */
	mty065x_set_image_freeze_value(mty, 0);

	return 0;
}

static void mty065x_restore_props(struct mty065x *mty)
{
	memcpy(&mty->props, &mty->saved_props, sizeof(mty->props));
	mty065x_configure(mty);
}

static void mty065x_save_props(struct mty065x *mty)
{
	memcpy(&mty->saved_props, &mty->props, sizeof(mty->saved_props));
	mty->props_saved = 1;
}

static ssize_t mty065x_get_chip_id_attr(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int ret;
	struct mty065x* mty = dev_to_mty065x(dev);

	ret = i2c_smbus_read_byte_data(mty->i2c, MTY065X_CHIP_ID_R);
	
	return scnprintf(buf, PAGE_SIZE, "0x%02x\n", ret);
}

static ssize_t mty065x_get_display_size_attr(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct mty065x* mty = dev_to_mty065x(dev);

	mty065x_get_display_size(mty);
	return scnprintf(buf, PAGE_SIZE, "%u %u\n",
			 mty->props.display_size[0],
			 mty->props.display_size[1]);

}

static ssize_t mty065x_set_display_size_attr(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct mty065x* mty = dev_to_mty065x(dev);
	unsigned int size[2];

	if (sscanf(buf, "%u %u", &size[0], &size[1]) == 2) {
		mty->props.display_size[0] = size[0];
		mty->props.display_size[1] = size[1];
		mty065x_set_display_size(mty);
	}

	return count;
}

static ssize_t mty065x_get_image_crop_attr(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct mty065x* mty = dev_to_mty065x(dev);

	mty065x_get_image_crop(mty);
	return scnprintf(buf, PAGE_SIZE, "%u %u %u %u\n",
			 mty->props.image_crop[0],
			 mty->props.image_crop[1],
			 mty->props.image_crop[2],
			 mty->props.image_crop[3]);

}

static ssize_t mty065x_set_image_crop_attr(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct mty065x* mty = dev_to_mty065x(dev);
	unsigned int crop[4];

	if (sscanf(buf, "%u %u %u %u",
		   &crop[0], &crop[1], &crop[2], &crop[3]) == 4) {
		mty->props.image_crop[0] = crop[0];
		mty->props.image_crop[1] = crop[1];
		mty->props.image_crop[2] = crop[2];
		mty->props.image_crop[3] = crop[3];
		mty065x_set_image_crop(mty);
	}

	return count;
}

static ssize_t mty065x_get_image_curtain_attr(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct mty065x* mty = dev_to_mty065x(dev);

	mty065x_get_image_curtain(mty);
	return scnprintf(buf, PAGE_SIZE, "%u %u\n",
			 mty->props.image_curtain.enable,
			 mty->props.image_curtain.color);

}

static ssize_t mty065x_set_image_curtain_attr(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
	struct mty065x* mty = dev_to_mty065x(dev);
	unsigned int enable, color;

	if (sscanf(buf, "%u %u", &enable, &color) == 2) {
		mty->props.image_curtain.enable = enable;
		mty->props.image_curtain.color = color;
		mty065x_set_image_curtain(mty);
	}

	return count;
}

static ssize_t mty065x_get_image_freeze_attr(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct mty065x* mty = dev_to_mty065x(dev);

	mty065x_get_image_freeze(mty);
	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 mty->props.image_freeze);
}

static ssize_t mty065x_set_image_freeze_attr(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct mty065x* mty = dev_to_mty065x(dev);
	unsigned int freeze;

	if (sscanf(buf, "%u", &freeze) == 1) {
		mty->props.image_freeze = freeze;
		mty065x_set_image_freeze(mty);
	}

	return count;
}

static ssize_t mty065x_get_input_format_attr(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct mty065x* mty = dev_to_mty065x(dev);

	mty065x_get_input_format(mty);
	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 mty->props.input_format);

}

static ssize_t mty065x_set_input_format_attr(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct mty065x* mty = dev_to_mty065x(dev);
	unsigned int fmt;

	if (sscanf(buf, "%u", &fmt) == 1) {
		mty->props.input_format = fmt;
		mty065x_set_input_format(mty);
	}

	return count;
}

static ssize_t mty065x_get_input_select_attr(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct mty065x* mty = dev_to_mty065x(dev);

	mty065x_get_input_select(mty);
	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 mty->props.input_select);

}

static ssize_t mty065x_set_input_select_attr(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct mty065x* mty = dev_to_mty065x(dev);
	unsigned int select;

	if (sscanf(buf, "%u", &select) == 1) {
		mty->props.input_select = select;
		mty065x_configure(mty);
	}

	return count;
}

static ssize_t mty065x_get_input_size_attr(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct mty065x* mty = dev_to_mty065x(dev);

	mty065x_get_input_size(mty);
	return scnprintf(buf, PAGE_SIZE, "%u %u\n",
			 mty->props.input_size[0],
			 mty->props.input_size[1]);

}

static ssize_t mty065x_set_input_size_attr(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct mty065x* mty = dev_to_mty065x(dev);
	unsigned int size[2];

	if (sscanf(buf, "%u %u", &size[0], &size[1]) == 2) {
		mty->props.input_size[0] = size[0];
		mty->props.input_size[1] = size[1];
		mty065x_set_input_size(mty);
	}

	return count;
}

static ssize_t mty065x_get_ks_correction_attr(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct mty065x* mty = dev_to_mty065x(dev);

	mty065x_get_ks_correction(mty);
	return scnprintf(buf, PAGE_SIZE, "%u %u %d\n",
			 mty->props.ks_correction.enable,
			 mty->props.ks_correction.ratio,
			 mty->props.ks_correction.offset);

}

static ssize_t mty065x_set_ks_correction_attr(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
	struct mty065x* mty = dev_to_mty065x(dev);
	unsigned int enable;
	unsigned int ratio;
	int offset;

	if (sscanf(buf, "%u %u %d", &enable, &ratio, &offset) == 3) {
		mty->props.ks_correction.enable = enable;
		mty->props.ks_correction.ratio = ratio;
		mty->props.ks_correction.offset = offset;
		mty065x_set_ks_correction(mty);
		return count;
	}

	return -EINVAL;
}

static ssize_t mty065x_get_ks_projection_attr(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct mty065x* mty = dev_to_mty065x(dev);

	mty065x_get_ks_projection(mty);
	return scnprintf(buf, PAGE_SIZE, "%d\n", mty->props.ks_projection.angle);

}

static ssize_t mty065x_set_ks_projection_attr(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
	struct mty065x* mty = dev_to_mty065x(dev);
	int angle;

	if (sscanf(buf, "%d", &angle) == 1) {
		mty->props.ks_projection.angle = angle;
		mty065x_set_ks_projection(mty);
		return count;
	}

	return -EINVAL;
}

static ssize_t mty065x_get_led_pwm_level_attr(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct mty065x* mty = dev_to_mty065x(dev);
	int level;

	mty065x_get_led_pwm_level(mty, &level);
	return scnprintf(buf, PAGE_SIZE, "%d\n", level);
}

static ssize_t mty065x_set_led_pwm_level_attr(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
	struct mty065x* mty = dev_to_mty065x(dev);
	int level;

	if (sscanf(buf, "%d", &level) == 1) {
		if (level < 0) { /* Sequence toggle levels */
			mty065x_get_led_pwm_level(mty, &level);
			if (++level > MTY065X_LED_PWM_LEVEL_MAX)
				level = 0;
		}
		mty065x_set_led_pwm_level(mty, level);
		return count;
	}

	return -EINVAL;
}

static ssize_t mty065x_get_led_pwm_level_map_attr(struct device *dev,
						  struct device_attribute *attr,
						  char *buf)
{
	int i, n, s, t;
	char *p;
	struct mty065x* mty = dev_to_mty065x(dev);

	s = PAGE_SIZE;
	t = 0;
	p = buf;

	/* The level 0 if special, actual level from 1 */
	for (i = 0; i < MTY065X_LED_PWM_LEVEL_MAX; i++) {
		n = scnprintf(p, s, "%u ", mty->props.led_pwm_level_map[i]);
		s -= n;
		p += n;
		t += n;
	}
	return t + scnprintf(p, s, "\n");
}

static ssize_t mty065x_set_led_pwm_level_map_attr(struct device *dev,
						  struct device_attribute *attr,
						  const char *buf, size_t count)
{
	int i, n;
	struct mty065x* mty = dev_to_mty065x(dev);
	unsigned int map[10];

	n = sscanf(buf, "%u %u %u %u %u %u %u %u %u %u",
		   &map[0], &map[1], &map[2], &map[3], &map[4],
		   &map[5], &map[6], &map[7], &map[8], &map[9]);
	for (i = 0; i < n && i < MTY065X_LED_PWM_LEVEL_MAX; i++) {
		mty->props.led_pwm_level_map[i] = map[i];
	}

	return count;
}

static ssize_t mty065x_get_rgb_led_ctrl_attr(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct mty065x* mty = dev_to_mty065x(dev);

	mty065x_get_rgb_led_ctrl(mty);
	return scnprintf(buf, PAGE_SIZE, "%u\n", mty->props.rgb_led_ctrl);
}

static ssize_t mty065x_set_rgb_led_ctrl_attr(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct mty065x* mty = dev_to_mty065x(dev);
	unsigned int ctrl;

	if (sscanf(buf, "%u", &ctrl) == 1) {
		mty->props.rgb_led_ctrl = ctrl;
		mty065x_set_rgb_led_ctrl(mty);
		return count;
	}

	return -EINVAL;
}

static ssize_t mty065x_get_rgb_led_pwm_attr(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct mty065x* mty = dev_to_mty065x(dev);

	mty065x_get_rgb_led_pwm(mty);
	return scnprintf(buf, PAGE_SIZE, "%u %u %u\n",
			 mty->props.rgb_led_pwm.red,
			 mty->props.rgb_led_pwm.green,
			 mty->props.rgb_led_pwm.blue);

}

static ssize_t mty065x_set_rgb_led_pwm_attr(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct mty065x* mty = dev_to_mty065x(dev);
	unsigned int red, green, blue;

	if (sscanf(buf, "%u %u %u", &red, &green, &blue) == 3) {
		mty->props.rgb_led_pwm.red = red;
		mty->props.rgb_led_pwm.green = green;
		mty->props.rgb_led_pwm.blue = blue;
		mty065x_set_rgb_led_pwm(mty);
		return count;
	}

	return -EINVAL;
}

static ssize_t mty065x_get_rgb_led_max_pwm_attr(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct mty065x* mty = dev_to_mty065x(dev);

	mty065x_get_rgb_led_max_pwm(mty);
	return scnprintf(buf, PAGE_SIZE, "%u %u %u\n",
			 mty->props.rgb_led_max_pwm.red,
			 mty->props.rgb_led_max_pwm.green,
			 mty->props.rgb_led_max_pwm.blue);

}

static ssize_t mty065x_set_rgb_led_max_pwm_attr(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct mty065x* mty = dev_to_mty065x(dev);
	unsigned int red, green, blue;

	if (sscanf(buf, "%u %u %u", &red, &green, &blue) == 3) {
		mty->props.rgb_led_max_pwm.red = red;
		mty->props.rgb_led_max_pwm.green = green;
		mty->props.rgb_led_max_pwm.blue = blue;
		mty065x_set_rgb_led_max_pwm(mty);
		return count;
	}

	return -EINVAL;
}

static ssize_t mty065x_get_test_pattern_attr(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct mty065x* mty = dev_to_mty065x(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 mty->props.test_pattern);

}

static ssize_t mty065x_set_test_pattern_attr(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct mty065x* mty = dev_to_mty065x(dev);
	int pattern;

	if (sscanf(buf, "%d", &pattern) == 1) {
		if (pattern == -1) { /* Sequence toggle patterns */
			if (++mty->props.test_pattern > MTY065X_TEST_PATTERN_MAX)
				mty->props.test_pattern = 0;
		} else {
			mty->props.test_pattern = pattern;
		}
		mty065x_configure(mty);
	}

	return count;
}


static void mty065x_parse_dt(struct mty065x *mty);
static ssize_t mty065x_set_restore_attr(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct mty065x* mty = dev_to_mty065x(dev);
	int mode;

	if (sscanf(buf, "%d", &mode) == 1) {
		if (mode == 1 && mty->props_saved)
			mty065x_restore_props(mty);
		else if (mode == 2) { /* Force reload dt configs */
			mty065x_parse_dt(mty);
			mty065x_configure(mty);
		}
	}

	return count;
}

static ssize_t mty065x_get_save_attr(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct mty065x* mty = dev_to_mty065x(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 mty->props_saved);

}

static ssize_t mty065x_set_save_attr(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct mty065x* mty = dev_to_mty065x(dev);
	int yes;

	if (sscanf(buf, "%d", &yes) == 1) {
		if (yes == 1)
			mty065x_save_props(mty);
	}

	return count;
}

DEVICE_ATTR(chip_id, S_IRUGO, mty065x_get_chip_id_attr, NULL);
DEVICE_ATTR(display_size, 0644, mty065x_get_display_size_attr, mty065x_set_display_size_attr);
DEVICE_ATTR(image_crop, 0644, mty065x_get_image_crop_attr, mty065x_set_image_crop_attr);
DEVICE_ATTR(image_curtain, 0644, mty065x_get_image_curtain_attr, mty065x_set_image_curtain_attr);
DEVICE_ATTR(image_freeze, 0644, mty065x_get_image_freeze_attr, mty065x_set_image_freeze_attr);
DEVICE_ATTR(input_format, 0644, mty065x_get_input_format_attr, mty065x_set_input_format_attr);
DEVICE_ATTR(input_select, 0644, mty065x_get_input_select_attr, mty065x_set_input_select_attr);
DEVICE_ATTR(input_size, 0644, mty065x_get_input_size_attr, mty065x_set_input_size_attr);
DEVICE_ATTR(ks_correction, 0644, mty065x_get_ks_correction_attr, mty065x_set_ks_correction_attr);
DEVICE_ATTR(ks_projection, 0644, mty065x_get_ks_projection_attr, mty065x_set_ks_projection_attr);
DEVICE_ATTR(led_pwm_level, 0644, mty065x_get_led_pwm_level_attr, mty065x_set_led_pwm_level_attr);
DEVICE_ATTR(led_pwm_level_map, 0644, mty065x_get_led_pwm_level_map_attr, mty065x_set_led_pwm_level_map_attr);
DEVICE_ATTR(rgb_led_ctrl, 0644, mty065x_get_rgb_led_ctrl_attr, mty065x_set_rgb_led_ctrl_attr);
DEVICE_ATTR(rgb_led_pwm, 0644, mty065x_get_rgb_led_pwm_attr, mty065x_set_rgb_led_pwm_attr);
DEVICE_ATTR(rgb_led_max_pwm, 0644, mty065x_get_rgb_led_max_pwm_attr, mty065x_set_rgb_led_max_pwm_attr);
DEVICE_ATTR(test_pattern, 0644, mty065x_get_test_pattern_attr, mty065x_set_test_pattern_attr);
DEVICE_ATTR(restore, 0644, NULL, mty065x_set_restore_attr);
DEVICE_ATTR(save, 0644, mty065x_get_save_attr, mty065x_set_save_attr);

static struct attribute *mty065x_attrs[] = {
	&dev_attr_chip_id.attr,
	&dev_attr_display_size.attr,
	&dev_attr_image_crop.attr,
	&dev_attr_image_curtain.attr,
	&dev_attr_image_freeze.attr,
	&dev_attr_input_format.attr,
	&dev_attr_input_select.attr,
	&dev_attr_input_size.attr,
	&dev_attr_ks_correction.attr,
	&dev_attr_ks_projection.attr,
	&dev_attr_led_pwm_level.attr,
	&dev_attr_led_pwm_level_map.attr,
	&dev_attr_rgb_led_ctrl.attr,
	&dev_attr_rgb_led_pwm.attr,
	&dev_attr_rgb_led_max_pwm.attr,
	&dev_attr_test_pattern.attr,
	&dev_attr_restore.attr,
	&dev_attr_save.attr,
	NULL,
};

static struct attribute_group mty065x_attr_group = {
	.attrs = mty065x_attrs,
};

#if defined(CONFIG_OF)
static void mty065x_parse_dt(struct mty065x *mty)
{
	int i;
	int ret;
	struct device_node *node = mty->i2c->dev.of_node;

	if (of_property_read_u16_array(node, "display-size", mty->props.display_size, 2)) {
		mty->props.display_size[0] = 854;
		mty->props.display_size[1] = 480;
	}

	if (of_property_read_u16_array(node, "image-crop", mty->props.image_crop, 4)) {
		mty->props.image_crop[0] = 0;
		mty->props.image_crop[1] = 0;
		mty->props.image_crop[2] = 1280;
		mty->props.image_crop[3] = 720;
	}

	if (of_property_read_u8_array(node, "input-format", &mty->props.input_format, 1))
		mty->props.input_format = 0x41;

	if (of_property_read_u8_array(node, "input-select", &mty->props.input_select, 1))
		mty->props.input_select = 0x00;

	if (of_property_read_u16_array(node, "input-size", mty->props.input_size, 2)) {
		mty->props.input_size[0] = mty->props.image_crop[2];
		mty->props.input_size[1] = mty->props.image_crop[3];
	}

	/* FIXME: Add led_pwm_level_map support to dts */
	for (i = 0; i < MTY065X_LED_PWM_LEVEL_MAX; i++) {
		mty->props.led_pwm_level_map[i] =
			mty065x_led_pwm_level_default_map[i];
	}
}
#else
static void mty065x_parse_dt(struct mty065x *mty)
{
	mty->props.display_sizep[0] = 854;
	mty->props.display_sizep[1] = 480;
	mty->props.image_crop[0] = 0;
	mty->props.image_crop[1] = 0;
	mty->props.image_crop[2] = 1280;
	mty->props.image_crop[3] = 720;
	mty->props.input_format = 0x41;
	mty->props.input_select = 0;
	mty->props.input_size[0] = 1280;
	mty->props.input_size[1] = 720;

	/* FIXME: Add led_pwm_level_map support to dts */
	for (i = 0; i < MTY065X_LED_PWM_LEVEL_MAX; i++) {
		mty->props.led_pwm_level_map[i] =
			mty065x_led_pwm_level_default_map[i];
	}
}
#endif

static int mty065x_bl_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct backlight_properties props;
	struct mty065x *mty;
	int ret = 0;

	mty = devm_kzalloc(&client->dev, sizeof(struct mty065x), GFP_KERNEL);
	if (!mty)
		return -ENOMEM;

	i2c_set_clientdata(client, mty);

	mty->dev = &client->dev;
	mty->i2c = client;

	ret = i2c_smbus_read_byte_data(mty->i2c, MTY065X_CHIP_ID_R);
	if (ret < 0) {
		dev_err(mty->dev, "MTY065X not found.\n");
		return -ENODEV;
	}

	mty->chip_id = ret;

	mty065x_current(mty);
	mty065x_parse_dt(mty);

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = MTY065X_LED_PWM_LEVEL_MAX;

	mty->bl = devm_backlight_device_register(&client->dev, "mty065x-bl",
						&client->dev, mty, &bl_ops,
						&props);
	if (IS_ERR(mty->bl)) {
		ret = PTR_ERR(mty->bl);
		goto err_blreg;
	}

	mty065x_get_led_pwm_level(mty, &mty->bl->props.brightness);
	mty->bl->props.power = FB_BLANK_UNBLANK;

	ret = sysfs_create_group(&mty->bl->dev.kobj, &mty065x_attr_group);
	if (ret) {
		dev_err(mty->dev, "failed to register sysfs, err: %d\n", ret);
		goto err_sysfs;
	}

	mty065x_configure(mty);
	backlight_update_status(mty->bl);

	dev_info(mty->dev, "registered\n");

	return 0;
	
err_sysfs:
	devm_backlight_device_unregister(mty->dev, mty->bl);
err_blreg:
	mty->bl = NULL;
	return ret;
}

static int mty065x_bl_remove(struct i2c_client *client)
{
	struct mty065x *mty = i2c_get_clientdata(client);

	i2c_set_clientdata(client, NULL);
	
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mty065x_bl_suspend(struct device *dev)
{
	struct mty065x *mty = dev_get_drvdata(dev);

	mty065x_set_backlight(mty, 0);

	return 0;
}

static int mty065x_bl_resume(struct device *dev)
{
	struct mty065x *mty = dev_get_drvdata(dev);

	backlight_update_status(mty->bl);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(mty065x_bl_pm_ops, mty065x_bl_suspend, mty065x_bl_resume);

static const struct i2c_device_id mty065x_bl_id[] = {
	{ "mty065x-bl", 0 },
	{ },
};

#ifdef CONFIG_OF
static const struct of_device_id mty065x_bl_of_match[] = {
	{ .compatible = "tdc,mty065x-bl", },
	{ }
};
MODULE_DEVICE_TABLE(of, mty065x_of_match);
#endif

static struct i2c_driver mty065x_bl_driver = {
	.driver = {
		.name		= "mty065x-bl",
		.owner		= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table	= of_match_ptr(mty065x_bl_of_match),
#endif
		.pm		= &mty065x_bl_pm_ops,
	},
	.probe		= mty065x_bl_probe,
	.remove		= mty065x_bl_remove,
	.id_table	= mty065x_bl_id,
};

static int __init mty065x_bl_init(void)
{
	return i2c_add_driver(&mty065x_bl_driver);
}

static void __exit mty065x_bl_exit(void)
{
	i2c_del_driver(&mty065x_bl_driver);
}

#if defined(MODULE)
module_init(mty065x_bl_init);
#else
/* Use late_initcall() to wait after mxcfb loaded. */
late_initcall(mty065x_bl_init);
#endif
module_exit(mty065x_bl_exit);

MODULE_AUTHOR("Varphone Wong");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Backlight driver for MTY065X Military Projector");

