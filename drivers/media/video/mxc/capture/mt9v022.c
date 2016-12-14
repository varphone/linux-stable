/*
 * Copyright 2004-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mt9v022.c
 *
 * @brief mt9v022 camera driver functions
 *
 * @ingroup Camera
 */

#define DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <media/v4l2-int-device.h>
#include <linux/v4l2-mediabus.h>
#include <linux/fsl_devices.h>
#include <media/v4l2-chip-ident.h>
#include "mxc_v4l2_capture.h"
#include "mt9v022.h"


/*!
 * Holds the current frame rate.
 */
static int reset_frame_rate = MT9V022_FRAME_RATE;

/* MT9M001 has only one fixed colorspace per pixelcode */
struct mt9v022_datafmt {
	enum v4l2_mbus_pixelcode	code;
	enum v4l2_colorspace		colorspace;
};

/* Find a data format by a pixel code in an array */
static const struct mt9v022_datafmt *mt9v022_find_datafmt(
	enum v4l2_mbus_pixelcode code, const struct mt9v022_datafmt *fmt,
	int n)
{
	int i;
	for (i = 0; i < n; i++)
		if (fmt[i].code == code)
			return fmt + i;

	return NULL;
}

static const struct mt9v022_datafmt mt9v022_colour_fmts[] = {
	/*
	 * Order important: first natively supported,
	 * second supported with a GPIO extender
	 */
	{V4L2_MBUS_FMT_SBGGR10_1X10, V4L2_COLORSPACE_SRGB},
	{V4L2_MBUS_FMT_SBGGR8_1X8, V4L2_COLORSPACE_SRGB},
};

static const struct mt9v022_datafmt mt9v022_monochrome_fmts[] = {
	/* Order important - see above */
	{V4L2_MBUS_FMT_Y10_1X10, V4L2_COLORSPACE_JPEG},
	{V4L2_MBUS_FMT_Y8_1X8, V4L2_COLORSPACE_JPEG},
};


struct mt9v022_data {
	const struct fsl_mxc_camera_platform_data *platform_data;
	struct v4l2_int_device *v4l2_int_device;
	struct i2c_client *i2c_client;
	struct v4l2_pix_format pix;
	struct v4l2_captureparm streamcap;
//new add ======
	struct v4l2_rect rect;	/* Sensor window */
//=====
	bool on;

	/* control settings */
	int brightness;
	int hue;
	int contrast;
	int saturation;
	int red;
	int green;
	int blue;
	int ae_mode;

	u32 mclk;
	u8 mclk_source;
	int csi;

	void (*io_init)(void);
	int num_fmts;   //new add=====
	int model;	/* V4L2_IDENT_MT9M001 or V4L2_IDENT_MT9M112 code
			 * from v4l2-chip-ident.h */
	int chip_control;
	const struct mt9v022_datafmt *fmt;
	const struct mt9v022_datafmt *fmts;  //new add======
	unsigned int gain;
	unsigned int exposure;
	unsigned short y_skip_top;	/* Lines to skip at the top */
	unsigned char autoexposure;

};

/*
 * Function definitions
 */

static struct mt9v022_data *to_mt9v022(const struct i2c_client *client)
{
	return i2c_get_clientdata(client);
}

static struct mt9v022_data *v4l2_to_mt9v022(const struct v4l2_int_device *dev)
{
	return dev->priv;
}

static int reg_read(struct i2c_client *client, const u8 reg)
{
	int ret;

	ret = swab16(i2c_smbus_read_word_data(client, reg & 0xff));

	dev_dbg(&client->dev, "read  reg.%03x -> %04x\n", reg, ret);
	return ret;
}

static int reg_write(struct i2c_client *client, const u8 reg,
			     const u16 data)
{
	int ret;

	ret = i2c_smbus_write_word_data(client, reg & 0xff,
						swab16(data));
	dev_dbg(&client->dev, "write reg.%03x = %04x -> %d\n", reg, data, ret);
	return ret;
}

static int reg_set(struct i2c_client *client, const u8 reg,
			   const u16 data)
{
	int ret;

	ret = reg_write(client, reg, ret | data);
	return ret;
}

static int reg_clear(struct i2c_client *client, const u8 reg,
			     const u16 data)
{
	int ret;

	ret = reg_read(client, reg);
	return reg_write(client, reg, ret & ~data);
}


static int mt9v022_sensor_init(struct i2c_client *client)
{
	struct mt9v022_data *mt9v022 = to_mt9v022(client);
	int ret;

	/*
	 * Almost the default mode: master, parallel, simultaneous, and an
	 * undocumented bit 0x200, which is present in table 7, but not in 8,
	 * plus snapshot mode to disable scan for now
	 */
//	mt9v022->chip_control |= 0x10;
	ret = reg_write(client, MT9V022_CHIP_CONTROL, mt9v022->chip_control);
	if (!ret)
		ret = reg_write(client, MT9V022_READ_MODE, 0x300);

	/* All defaults */
	if (!ret)
		/* AEC, AGC on */
		ret = reg_set(client, MT9V022_AEC_AGC_ENABLE, 0x3);
	if (!ret)
		ret = reg_write(client, MT9V022_ANALOG_GAIN, 16);
	if (!ret)
		ret = reg_write(client, MT9V022_TOTAL_SHUTTER_WIDTH, 480);
	if (!ret)
		ret = reg_write(client, MT9V022_MAX_TOTAL_SHUTTER_WIDTH, 480);
	if (!ret)
		/* default - auto */
		ret = reg_clear(client, MT9V022_BLACK_LEVEL_CALIB_CTRL, 1);
	if (!ret)
		ret = reg_write(client, MT9V022_DIGITAL_TEST_PATTERN, 0);

	return ret;
}


/* --------------- IOCTL functions from v4l2_int_ioctl_desc --------------- */

/*!
 * ioctl_g_ifparm - V4L2 sensor interface handler for vidioc_int_g_ifparm_num
 * s: pointer to standard V4L2 device structure
 * p: pointer to standard V4L2 vidioc_int_g_ifparm_num ioctl structure
 *
 * Gets slave interface parameters.
 * Calculates the required xclk value to support the requested
 * clock parameters in p.  This value is returned in the p
 * parameter.
 *
 * vidioc_int_g_ifparm returns platform-specific information about the
 * interface settings used by the sensor.
 *
 * Given the image capture format in pix, the nominal frame period in
 * timeperframe, calculate the required xclk frequency.
 *
 * Called on open.
 */
static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	struct mt9v022_data *mt9v022 = v4l2_to_mt9v022(s);

	pr_debug("In mt9v022:ioctl_g_ifparm\n");

	if (s == NULL) {
        printk("::: ioctl_g_ifparm (END, error)\n");

		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	memset(p, 0, sizeof(*p));
	p->u.bt656.clock_curr = mt9v022->mclk;
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_10BIT;
	p->u.bt656.clock_min = MT9V022_CLK_MIN;
	p->u.bt656.clock_max = MT9V022_CLK_MAX;

	return 0;
}

/*!
 * Sets the camera power.
 *
 * s  pointer to the camera device
 * on if 1, power is to be turned on.  0 means power is to be turned off
 *
 * ioctl_s_power - V4L2 sensor interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 * This is called on suspend and resume.
 */
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct mt9v022_data *sensor = s->priv;

	pr_debug("In mt9v022:ioctl_s_power\n");
#if 0
	if (on)
		gpio_sensor_active();
	else
		gpio_sensor_inactive();
#endif
	return 0;
}

static unsigned long mt9v022_get_cyclesperframe(struct mt9v022_data *mt9v022)
{
	struct i2c_client *client = mt9v022->i2c_client;
	unsigned int a, q, tmp;
	unsigned long f;

	a = reg_read(client, MT9V022_WINDOW_WIDTH) + 1;
	tmp = reg_read(client, MT9V022_HORIZONTAL_BLANKING);
	if (tmp < 19)
		tmp = 19;
	q = 244 + tmp - 19;

	f = reg_read(client, MT9V022_WINDOW_HEIGHT) + 1;
	f += reg_read(client, MT9V022_VERTICAL_BLANKING) + 1;
	f *= (a + q);

	return f;
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
	struct mt9v022_data *mt9v022 = v4l2_to_mt9v022(s);
	int ret = 0;
	struct v4l2_captureparm *cparm = &a->parm.capture;

	/* s->priv points to mt9v022_data */

	pr_debug("In mt9v022:ioctl_g_parm\n");

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n");
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = mt9v022->streamcap.capability;

		mt9v022->streamcap.timeperframe.denominator = mt9v022->mclk;
		mt9v022->streamcap.timeperframe.numerator =
			mt9v022_get_cyclesperframe(mt9v022);

		cparm->timeperframe =
				mt9v022->streamcap.timeperframe;
		cparm->capturemode = mt9v022->streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_err("   type is not V4L2_BUF_TYPE_VIDEO_CAPTURE " \
			"but %d\n", a->type);
		ret = -EINVAL;
		break;

	default:
		pr_err("   type is unknown - %d\n", a->type);
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
	struct mt9v022_data *mt9v022 = v4l2_to_mt9v022(s);
	int ret = 0;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	uint32_t clock_rate;
	/* s->priv points to mt9v022_data */

	pr_debug("In mt9v022:ioctl_s_parm\n");

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n");

		clock_rate = mt9v022_get_cyclesperframe(mt9v022) *
			cparm->timeperframe.denominator /
			cparm->timeperframe.numerator;
		set_mclk_rate(&clock_rate, 1);
		mt9v022->mclk = clock_rate;

		mt9v022->streamcap.timeperframe.denominator = mt9v022->mclk;
		mt9v022->streamcap.timeperframe.numerator =
			mt9v022_get_cyclesperframe(mt9v022);

		/* Check that new capture mode is supported. */
		if ((cparm->capturemode != 0) &&
		    !(cparm->capturemode & V4L2_MODE_HIGHQUALITY)) {
			pr_err("ERROR: mt9v022: ioctl_s_parm: " \
				"unsupported capture mode\n");
			ret  = -EINVAL;
		} else {
			mt9v022->streamcap.capturemode =
						cparm->capturemode;
		      /* Call any camera functions to match settings. */
		      /* Right now this camera only supports 1 mode. */
		}
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_err("   type is not V4L2_BUF_TYPE_VIDEO_CAPTURE " \
			"but %d\n", a->type);
		ret = -EINVAL;
		break;

	default:
		pr_err("   type is unknown - %d\n", a->type);
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
	struct mt9v022_data *sensor = s->priv;
	/* s->priv points to mt9v022_data */

	pr_debug("In mt9v022:ioctl_g_fmt_cap.\n");
	pr_debug("   Returning size of %dx%d\n",
		sensor->pix.width, sensor->pix.height);

	f->fmt.pix = sensor->pix;

	return 0;
}

/*!
 * ioctl_queryctrl - V4L2 sensor interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the video_control[] array.  Otherwise, returns -EINVAL if the
 * control is not supported.
 */
static int ioctl_queryctrl(struct v4l2_int_device *s, struct v4l2_queryctrl *qc)
{
	pr_debug("In mt9v022:ioctl_queryctrl\n");

	return 0;
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
	struct mt9v022_data *mt9v022 = v4l2_to_mt9v022(s);
	struct i2c_client *client = mt9v022->i2c_client;
	int data;
    
	pr_debug("In mt9v022:ioctl_g_ctrl\n");

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		pr_debug("   V4L2_CID_BRIGHTNESS\n");
		vc->value = 0;
		break;
	case V4L2_CID_CONTRAST:
		pr_debug("   V4L2_CID_CONTRAST\n");
		vc->value = 0;
		break;
	case V4L2_CID_SATURATION:
		pr_debug("   V4L2_CID_SATURATION\n");
		vc->value = 0;
		break;
	case V4L2_CID_HUE:
		pr_debug("   V4L2_CID_HUE\n");
		vc->value = 0;
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		pr_debug(
			"   V4L2_CID_AUTO_WHITE_BALANCE\n");
		vc->value = 0;
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		pr_debug(
			"   V4L2_CID_DO_WHITE_BALANCE\n");
		vc->value = 0;
		break;
	case V4L2_CID_RED_BALANCE:
		pr_debug("   V4L2_CID_RED_BALANCE\n");
		vc->value = 0;
		break;
	case V4L2_CID_BLUE_BALANCE:
		pr_debug("   V4L2_CID_BLUE_BALANCE\n");
		vc->value = 0;
		break;
	case V4L2_CID_GAMMA:
		pr_debug("   V4L2_CID_GAMMA\n");
		vc->value = 0;
		break;
	case V4L2_CID_EXPOSURE:
		pr_debug("   V4L2_CID_EXPOSURE\n");
		vc->value = 0;
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		pr_debug("   V4L2_CID_EXPOSURE_AUTO\n");
		vc->value = mt9v022->autoexposure;
		break;
	case V4L2_CID_AUTOGAIN:
		pr_debug("   V4L2_CID_AUTOGAIN\n");
		vc->value = 0;
		break;
	case V4L2_CID_GAIN:
		pr_debug("   V4L2_CID_GAIN\n");
		vc->value = 0;
		break;
	case V4L2_CID_HFLIP:
		pr_debug("   V4L2_CID_HFLIP\n");
		vc->value = 0;
		break;
	case V4L2_CID_VFLIP:
		pr_debug("   V4L2_CID_VFLIP\n");
		vc->value = 0;
		break;
	default:
		pr_debug("   Default case\n");
		printk("::: ioctl_g_ctrl (END, error=-EINVAL)\n");
		return -EINVAL;
		break;
	}

	return 0;
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
	struct mt9v022_data *mt9v022 = v4l2_to_mt9v022(s);
	struct i2c_client *client = mt9v022->i2c_client;
	int retval = 0;

	pr_debug("In mt9v022:ioctl_s_ctrl %d\n",
		vc->id);

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		pr_debug("   V4L2_CID_BRIGHTNESS\n");
		break;
	case V4L2_CID_CONTRAST:
		pr_debug("   V4L2_CID_CONTRAST\n");
		break;
	case V4L2_CID_SATURATION:
		pr_debug("   V4L2_CID_SATURATION\n");
		break;
	case V4L2_CID_HUE:
		pr_debug("   V4L2_CID_HUE\n");
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		pr_debug(
			"   V4L2_CID_AUTO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		pr_debug(
			"   V4L2_CID_DO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_RED_BALANCE:
		pr_debug("   V4L2_CID_RED_BALANCE\n");
		break;
	case V4L2_CID_BLUE_BALANCE:
		pr_debug("   V4L2_CID_BLUE_BALANCE\n");
		break;
	case V4L2_CID_GAMMA:
		pr_debug("   V4L2_CID_GAMMA\n");
		break;
	case V4L2_CID_EXPOSURE:
		pr_debug("   V4L2_CID_EXPOSURE\n");
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		pr_debug("   V4L2_CID_EXPOSURE_AUTO\n");
		break;
	case V4L2_CID_AUTOGAIN:
		pr_debug("   V4L2_CID_AUTOGAIN\n");
		break;
	case V4L2_CID_GAIN:
		pr_debug("   V4L2_CID_GAIN\n");
		break;
	case V4L2_CID_HFLIP:
		pr_debug("   V4L2_CID_HFLIP\n");
		break;
	case V4L2_CID_VFLIP:
		pr_debug("   V4L2_CID_VFLIP\n");
		break;
	default:
		pr_debug("   Default case\n");
		retval = -EINVAL;
		break;
	}

	pr_debug("   ioctl_s_ctrl (END, ret=%d)\n", retval);
    
	return retval;
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
	struct mt9v022_data *mt9v022 = v4l2_to_mt9v022(s);

	if (fmt->index > 1)
		return -EINVAL;

	fmt->pixelformat = mt9v022->pix.pixelformat;

	return 0;
}

static int ioctl_cropcap(struct v4l2_int_device *s,struct v4l2_cropcap *a)
{
	a->bounds.left			= MT9V022_COLUMN_SKIP;
	a->bounds.top			= MT9V022_ROW_SKIP;
	a->bounds.width			= MT9V022_MAX_WIDTH;
	a->bounds.height		= MT9V022_MAX_HEIGHT;
	a->defrect			= a->bounds;
	a->type				= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator	= 1;
	a->pixelaspect.denominator	= 1;
	return 0;
}
static int ioctl_g_corp(struct v4l2_int_device *s, struct v4l2_crop *a)
{
	struct mt9v022_data *mt9v022 = v4l2_to_mt9v022(s);
	struct i2c_client *client = mt9v022->i2c_client;

	a->c	= mt9v022->rect;
	a->type	= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	return 0;

}

 static inline void soc_camera_limit_side(int *start, int *length,
         unsigned int start_min,
        unsigned int length_min, unsigned int length_max)
{    
     if (*length < length_min)
         *length = length_min;
     else if (*length > length_max)
         *length = length_max;
 
     if (*start < start_min)
         *start = start_min;
     else if (*start > start_min + length_max - *length)
         *start = start_min + length_max - *length;
}   

static int ioctl_s_corp(struct v4l2_int_device *s, struct v4l2_crop *a)
{
	struct mt9v022_data *mt9v022 = v4l2_to_mt9v022(s);
	struct i2c_client *client = mt9v022->i2c_client;
struct v4l2_rect rect = a->c;
	int ret;

	/* Bayer format - even size lengths */
	if (mt9v022->fmts == mt9v022_colour_fmts) {
		rect.width	= ALIGN(rect.width, 2);
		rect.height	= ALIGN(rect.height, 2);
		/* Let the user play with the starting pixel */
	}

	soc_camera_limit_side(&rect.left, &rect.width,
		     MT9V022_COLUMN_SKIP, MT9V022_MIN_WIDTH, MT9V022_MAX_WIDTH);

	soc_camera_limit_side(&rect.top, &rect.height,
		     MT9V022_ROW_SKIP, MT9V022_MIN_HEIGHT, MT9V022_MAX_HEIGHT);

	/* Like in example app. Contradicts the datasheet though */
	ret = reg_read(client, MT9V022_AEC_AGC_ENABLE);
	if (ret >= 0 && ret & 1) /* Autoexposure */
		ret = reg_write(client, MT9V022_MAX_TOTAL_SHUTTER_WIDTH,
					rect.height + mt9v022->y_skip_top + 43);
	/* Setup frame format: defaults apart from width and height */
	if (!ret)
		ret = reg_write(client, MT9V022_COLUMN_START, rect.left);
	if (!ret)
		ret = reg_write(client, MT9V022_ROW_START, rect.top);
	if (!ret) {
		if (is_mt9v024()) {
			ret =  reg_write(client, MT9V022_HORIZONTAL_BLANKING,
				rect.width > 690 - 61 ? 61 :
				690 - rect.width);
		} else {
			ret = reg_write(client, MT9V022_HORIZONTAL_BLANKING,
				rect.width > 660 - 43 ? 43 :
				660 - rect.width);
		}
	}

	if (!ret)
		ret = reg_write(client, MT9V022_VERTICAL_BLANKING, 45);
	if (!ret)
		ret = reg_write(client, MT9V022_WINDOW_WIDTH, rect.width);
	if (!ret)
		ret = reg_write(client, MT9V022_WINDOW_HEIGHT,
				rect.height + mt9v022->y_skip_top);

	if (ret < 0)
		return ret;

	dev_dbg(&client->dev, "Frame %dx%d pixel\n", rect.width, rect.height);

	mt9v022->rect = rect;

	return 0;
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
	struct mt9v022_data *mt9v022 = v4l2_to_mt9v022(s);

	fsize->pixel_format = mt9v022->pix.pixelformat;
	fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
	fsize->stepwise.min_height = MT9V022_MIN_HEIGHT;
	fsize->stepwise.min_width = MT9V022_MIN_WIDTH;
	fsize->stepwise.max_height = MT9V022_MAX_HEIGHT;
	fsize->stepwise.max_width = MT9V022_MAX_WIDTH;
	fsize->stepwise.step_width = fsize->stepwise.step_height = 1;
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
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name, "mt9v022");

	return 0;
}


/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{
    printk("::: ioctl_init\n");
    
	pr_debug("In mt9v022:ioctl_init\n");

    printk("::: ioctl_init (END)\n");

	return 0;
}

/*
 * Interface active, can use i2c. If it fails, it can indeed mean, that
 * this wasn't our capture interface, so, we wait for the right one
 */
static int mt9v022_video_probe(struct mt9v022_data *mt9v022)
{
	struct i2c_client *client = mt9v022->i2c_client;
	s32 data;
	int ret;

	/* Read out the chip version register */
	data = reg_read(client, MT9V022_CHIP_VERSION);
	printk("data=%x\n",data);
	/* must be 0x1311, 0x1313 or 0x1324 */
	switch (data) {
	case 0x1324:
		mt9v022->model = V4L2_IDENT_MT9V024;
		break;
	case 0x1311:
	case 0x1313:
			mt9v022->model = V4L2_IDENT_MT9V022IX7ATC;
		break;
	default:
		ret = -ENODEV;
		dev_info(&client->dev, "No MT9V02x found, ID register 0x%x\n", data);
		goto ei2c;
	}

	/* Soft reset */
	ret = reg_write(client, MT9V022_RESET, 1);
	if (ret < 0)
		goto ei2c;
	/* 15 clock cycles */
	udelay(200);
	if (reg_read(client, MT9V022_RESET)) {
		dev_err(&client->dev, "Resetting MT9V02x failed!\n");
		if (ret > 0)
			ret = -EIO;
		goto ei2c;
	}

#ifdef ENABLE_LVDS_OUT
	reg_write(client, 0xb3, 0);
	reg_write(client, 0xb1, 0);
#ifdef LVDS_OUT_8BIT
	reg_write(client, 0xb6, 0);
#else
	reg_write(client, 0xb6, 1);
#endif
#endif
	int color = 1;
	/* Set monochrome or colour sensor type */
	if (color) {
		switch (mt9v022->model) {
		case V4L2_IDENT_MT9V022IX7ATC:
			ret = reg_write(client,
				MT9V022_PIXEL_OPERATION_MODE, 0x4 | 0x11);
			break;
		case V4L2_IDENT_MT9V024:
			ret = reg_write(client,
				MT9V022_PIXEL_OPERATION_MODE, 0x02 | 0x100);
			break;
		}

		mt9v022->fmts = mt9v022_colour_fmts;
	} else {
		switch (mt9v022->model) {
		case V4L2_IDENT_MT9V022IX7ATM:
			ret = reg_write(client,
				MT9V022_PIXEL_OPERATION_MODE, 0x11);
			break;
		case V4L2_IDENT_MT9V024:
			ret = reg_write(client,
				MT9V022_PIXEL_OPERATION_MODE, 0x100);
			break;
		}

		mt9v022->fmts = mt9v022_monochrome_fmts;
	}

	if (ret < 0)
		goto ei2c;

	mt9v022->num_fmts = 0;
#if 0
	/*
	 * This is a 10bit sensor, so by default we only allow 10bit.
	 * The platform may support different bus widths due to
	 * different routing of the data lines.
	 */
	if (icl->query_bus_param)
		flags = icl->query_bus_param(icl);
	else
		flags = SOCAM_DATAWIDTH_10;

	if (flags & SOCAM_DATAWIDTH_10)
		mt9v022->num_fmts++;
	else
		mt9v022->fmts++;

	if (flags & SOCAM_DATAWIDTH_8)
		mt9v022->num_fmts++;

#else
	mt9v022->num_fmts++;
#endif


	mt9v022->fmt = &mt9v022->fmts[0];

	dev_info(&client->dev, "Detected a MT9V02x chip ID %x, %s sensor\n",
		 data, mt9v022->model == V4L2_IDENT_MT9V022IX7ATM ?
		 "monochrome" : "colour");

	ret = mt9v022_sensor_init(client);

ei2c:
	return ret;
}

/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	uint32_t clock_rate = MT9V022_MCLK;
	struct mt9v022_data *mt9v022 = v4l2_to_mt9v022(s);

	pr_err("In mt9v022:ioctl_dev_init\n");

	set_mclk_rate(&clock_rate, 1);                        // TODO: unhardcode '0' csi

	mt9v022->mclk = clock_rate;

	mt9v022_video_probe(mt9v022);

	return 0;
}

/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
static struct v4l2_int_ioctl_desc mt9v022_ioctl_desc[] = {

	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func *)ioctl_dev_init},

	/*!
	 * Delinitialise the dev. at slave detach.
	 * The complement of ioctl_dev_init.
	 */
/*	{vidioc_int_dev_exit_num, (v4l2_int_ioctl_func *) ioctl_dev_exit}, */

	{vidioc_int_s_power_num, (v4l2_int_ioctl_func *) ioctl_s_power},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func *) ioctl_g_ifparm},
/*	{vidioc_int_g_needs_reset_num,
				(v4l2_int_ioctl_func *) ioctl_g_needs_reset}, */
/*	{vidioc_int_reset_num, (v4l2_int_ioctl_func *) ioctl_reset}, */
	{vidioc_int_init_num, (v4l2_int_ioctl_func *) ioctl_init},

	/*!
	 * VIDIOC_ENUM_FMT ioctl for the CAPTURE buffer type.
	 */
	{vidioc_int_enum_fmt_cap_num,
				(v4l2_int_ioctl_func *) ioctl_enum_fmt_cap},

	/*!
	 * VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.
	 * This ioctl is used to negotiate the image capture size and
	 * pixel format without actually making it take effect.
	 */
/*	{vidioc_int_try_fmt_cap_num,
				(v4l2_int_ioctl_func *) ioctl_try_fmt_cap}, */

	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func *) ioctl_g_fmt_cap},

	/*!
	 * If the requested format is supported, configures the HW to use that
	 * format, returns error code if format not supported or HW can't be
	 * correctly configured.
	 */
/*	{vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_s_fmt_cap}, */

	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func *) ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func *) ioctl_s_parm},
/*	{vidioc_int_queryctrl_num, (v4l2_int_ioctl_func *) ioctl_queryctrl}, */
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *) ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *) ioctl_s_ctrl},
    {vidioc_int_cropcap_num,(v4l2_int_ioctl_func *) ioctl_cropcap},
    {vidioc_int_g_crop_num,(v4l2_int_ioctl_func *)  ioctl_g_corp},
    {vidioc_int_s_crop_num,(v4l2_int_ioctl_func *)  ioctl_s_corp},

	{vidioc_int_enum_framesizes_num, (v4l2_int_ioctl_func *)ioctl_enum_framesizes},
	{vidioc_int_g_chip_ident_num,(v4l2_int_ioctl_func *)ioctl_g_chip_ident},
};

static struct v4l2_int_slave mt9v022_slave = {
	.ioctls = mt9v022_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(mt9v022_ioctl_desc),
};

static struct v4l2_int_device mt9v022_int_device = {
	.module = THIS_MODULE,
	.name = "mt9v022",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &mt9v022_slave,
		},
};

/*!
 * mt9v022 I2C probe function
 * Function set in i2c_driver struct.
 * Called by insmod mt9v022_camera.ko.
 *
 * @return  Error code indicating success or failure
 */
static int mt9v022_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int retval;
	struct mt9v022_data *mt9v022;

	pr_debug("In mt9v022_probe  device id is %s\n", id->name);

	mt9v022 = kzalloc(sizeof(struct mt9v022_data), GFP_KERNEL);
	if (!mt9v022)
		return -ENOMEM;

	/* Set initial values for the sensor struct. */
	mt9v022->i2c_client = client;
	i2c_set_clientdata(client, mt9v022);
	mt9v022->platform_data = client->dev.platform_data;
	mt9v022->csi = mt9v022->platform_data->csi;
	mt9v022->io_init = mt9v022->platform_data->io_init;

	pr_debug("   client name is %s\n", client->name);
	mt9v022->pix.pixelformat = V4L2_PIX_FMT_UYVY;
	mt9v022->pix.width = MT9V022_MAX_WIDTH;
	mt9v022->pix.height = MT9V022_MAX_HEIGHT;
	mt9v022->streamcap.capability = 0; /* No higher resolution or frame
						* frame rate changes supported.
						*/
	mt9v022->streamcap.timeperframe.denominator = MT9V022_FRAME_RATE;
	mt9v022->streamcap.timeperframe.numerator = 1;
	mt9v022->chip_control = MT9V022_CHIP_CONTROL_DEFAULT;
	mt9v022->rect.left	= MT9V022_COLUMN_SKIP;
	mt9v022->rect.top	= MT9V022_ROW_SKIP;
	mt9v022->rect.width	= MT9V022_MAX_WIDTH;
	mt9v022->rect.height	= MT9V022_MAX_HEIGHT;

	mt9v022_int_device.priv = mt9v022;

	pr_debug("   type is %d (expect %d)\n",
			mt9v022_int_device.type, v4l2_int_type_slave);
	pr_debug("   num ioctls is %d\n",
			mt9v022_int_device.u.slave->num_ioctls);

	/* This function attaches this structure to the /dev/video0 device.
	 * The pointer in priv points to the mt9v022_data structure here.*/
	retval = v4l2_int_device_register(&mt9v022_int_device);

	if (mt9v022->io_init)
		mt9v022->io_init();

	return retval;
}

/*!
 * Function set in i2c_driver struct.
 * Called on rmmod mt9v022_camera.ko
 */
static int mt9v022_remove(struct i2c_client *client)
{
	pr_debug("In mt9v022_remove\n");

	v4l2_int_device_unregister(&mt9v022_int_device);

	return 0;
}

static const struct i2c_device_id mt9v022_id[] = {
	{"mt9v022", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, mt9v022_id);

static struct i2c_driver mt9v022_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "mt9v022",
		   },
	.probe = mt9v022_probe,
	.remove = mt9v022_remove,
	.id_table = mt9v022_id,
/* To add power management add .suspend and .resume functions */
};


/*!
 * MT9M001 init function.
 * Called by insmod mt9v022_camera.ko.
 *
 * @return  Error code indicating success or failure
 */
static __init int mt9v022_init(void)
{
	u8 err;

	/* Tells the i2c driver what functions to call for this driver. */
	err = i2c_add_driver(&mt9v022_i2c_driver);
	if (err != 0)
		pr_err("%s: driver registration failed, error=%d \n",
				__func__, err);

	return err;
}

/*!
 * MT9M001 cleanup function.
 * Called on rmmod mt9v022_camera.ko
 *
 * @return  Error code indicating success or failure
 */
static void __exit mt9v022_clean(void)
{
	i2c_del_driver(&mt9v022_i2c_driver);

//	gpio_sensor_inactive();
}

module_init(mt9v022_init);
module_exit(mt9v022_clean);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Mt9m00001 Camera Driver");
MODULE_LICENSE("GPL");
