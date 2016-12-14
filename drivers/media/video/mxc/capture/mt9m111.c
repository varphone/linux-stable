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
 * @file mt9m111.c
 *
 * @brief mt9m111 camera driver functions
 *
 * @ingroup Camera
 */


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
#include "mt9m111.h"

#ifdef MT9M111_DEBUG
static u16 testpattern;
#endif

/*!
 * Holds the current frame rate.
 */
static int reset_frame_rate = MT9M111_FRAME_RATE;

/* MT9M111 has only one fixed colorspace per pixelcode */
struct mt9m111_datafmt {
	enum v4l2_mbus_pixelcode	code;
	enum v4l2_colorspace		colorspace;
};

struct mt9m111_data {
	const struct fsl_mxc_camera_platform_data *platform_data;
	struct v4l2_int_device *v4l2_int_device;
	struct i2c_client *i2c_client;
	struct v4l2_pix_format pix;
	struct v4l2_captureparm streamcap;
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

	int model;	/* V4L2_IDENT_MT9M111 or V4L2_IDENT_MT9M112 code
			 * from v4l2-chip-ident.h */
	enum mt9m111_context context;
	struct v4l2_rect rect;
	const struct mt9m111_datafmt *fmt;
	unsigned int gain;
	unsigned char autoexposure;
	unsigned char datawidth;
	unsigned int powered:1;
	unsigned int hflip:1;
	unsigned int vflip:1;
	unsigned int swap_rgb_even_odd:1;
	unsigned int swap_rgb_red_blue:1;
	unsigned int swap_yuv_y_chromas:1;
	unsigned int swap_yuv_cb_cr:1;
	unsigned int autowhitebalance:1;

};

extern void gpio_sensor_active(void);
extern void gpio_sensor_inactive(void);

#define reg_read(reg) mt9m111_reg_read(client, MT9M111_##reg)
#define reg_write(reg, val) mt9m111_reg_write(client, MT9M111_##reg, (val))
#define reg_set(reg, val) mt9m111_reg_set(client, MT9M111_##reg, (val))
#define reg_clear(reg, val) mt9m111_reg_clear(client, MT9M111_##reg, (val))

/*
 * Function definitions
 */

static struct mt9m111_data *to_mt9m111(const struct i2c_client *client)
{
	return i2c_get_clientdata(client);
}

static struct mt9m111_data *v4l2_to_mt9m111(const struct v4l2_int_device *dev)
{
	return dev->priv;
}

static int reg_page_map_set(struct i2c_client *client, const u16 reg)
{
	int ret;
	u16 page;
	static int lastpage = -1;	/* PageMap cache value */

	page = (reg >> 8);
	if (page == lastpage)
		return 0;
	if (page > 2)
		return -EINVAL;

	ret = i2c_smbus_write_word_data(client, MT9M111_PAGE_MAP, swab16(page));
	if (!ret)
		lastpage = page;
	return ret;
}

static int mt9m111_reg_read(struct i2c_client *client, const u16 reg)
{
	int ret;

	ret = reg_page_map_set(client, reg);
	if (!ret)
		ret = swab16(i2c_smbus_read_word_data(client, reg & 0xff));

	dev_dbg(&client->dev, "read  reg.%03x -> %04x\n", reg, ret);
	return ret;
}

static int mt9m111_reg_write(struct i2c_client *client, const u16 reg,
			     const u16 data)
{
	int ret;

	ret = reg_page_map_set(client, reg);
	if (!ret)
		ret = i2c_smbus_write_word_data(client, reg & 0xff,
						swab16(data));
	dev_dbg(&client->dev, "write reg.%03x = %04x -> %d\n", reg, data, ret);
	return ret;
}

static int mt9m111_reg_set(struct i2c_client *client, const u16 reg,
			   const u16 data)
{
	int ret;

	ret = mt9m111_reg_read(client, reg);
	if (ret >= 0)
		ret = mt9m111_reg_write(client, reg, ret | data);
	return ret;
}

static int mt9m111_reg_clear(struct i2c_client *client, const u16 reg,
			     const u16 data)
{
	int ret;

	ret = mt9m111_reg_read(client, reg);
	return mt9m111_reg_write(client, reg, ret & ~data);
}

static int mt9m111_set_context(struct i2c_client *client,
			       enum mt9m111_context ctxt)
{
	int valB = MT9M111_CTXT_CTRL_RESTART | MT9M111_CTXT_CTRL_DEFECTCOR_B
		| MT9M111_CTXT_CTRL_RESIZE_B | MT9M111_CTXT_CTRL_CTRL2_B
		| MT9M111_CTXT_CTRL_GAMMA_B | MT9M111_CTXT_CTRL_READ_MODE_B
		| MT9M111_CTXT_CTRL_VBLANK_SEL_B
		| MT9M111_CTXT_CTRL_HBLANK_SEL_B;
	int valA = MT9M111_CTXT_CTRL_RESTART;

	if (ctxt == HIGHPOWER)
		return reg_write(CONTEXT_CONTROL, valB);
	else
		return reg_write(CONTEXT_CONTROL, valA);
}

static int mt9m111_setup_rect(struct i2c_client *client,
			      struct v4l2_rect *rect)
{
	struct mt9m111_data *mt9m111 = to_mt9m111(client);
	int ret, is_raw_format;
	int width = rect->width;
	int height = rect->height;

	if (mt9m111->fmt->code == V4L2_MBUS_FMT_SBGGR8_1X8 ||
	    mt9m111->fmt->code == V4L2_MBUS_FMT_SBGGR10_2X8_PADHI_LE)
		is_raw_format = 1;
	else
		is_raw_format = 0;

	ret = reg_write(COLUMN_START, rect->left);
	if (!ret)
		ret = reg_write(ROW_START, rect->top);

	if (is_raw_format) {
		if (!ret)
			ret = reg_write(WINDOW_WIDTH, width);
		if (!ret)
			ret = reg_write(WINDOW_HEIGHT, height);
	} else {
		if (!ret)
			ret = reg_write(REDUCER_XZOOM_B, MT9M111_MAX_WIDTH);
		if (!ret)
			ret = reg_write(REDUCER_YZOOM_B, MT9M111_MAX_HEIGHT);
		if (!ret)
			ret = reg_write(REDUCER_XSIZE_B, width);
		if (!ret)
			ret = reg_write(REDUCER_YSIZE_B, height);
		if (!ret)
			ret = reg_write(REDUCER_XZOOM_A, MT9M111_MAX_WIDTH);
		if (!ret)
			ret = reg_write(REDUCER_YZOOM_A, MT9M111_MAX_HEIGHT);
		if (!ret)
			ret = reg_write(REDUCER_XSIZE_A, width);
		if (!ret)
			ret = reg_write(REDUCER_YSIZE_A, height);
	}

	return ret;
}

static int mt9m111_setup_pixfmt(struct i2c_client *client, u16 outfmt)
{
	int ret;
	u16 mask = MT9M111_OUTFMT_PROCESSED_BAYER | MT9M111_OUTFMT_RGB |
		MT9M111_OUTFMT_BYPASS_IFP | MT9M111_OUTFMT_SWAP_RGB_EVEN |
		MT9M111_OUTFMT_RGB565 | MT9M111_OUTFMT_RGB555 |
		MT9M111_OUTFMT_SWAP_YCbCr_Cb_Cr |
		MT9M111_OUTFMT_SWAP_YCbCr_C_Y;

	ret = reg_read(OUTPUT_FORMAT_CTRL2_A);
	if (ret >= 0)
		ret = reg_write(OUTPUT_FORMAT_CTRL2_A, (ret & ~mask) | outfmt);
	if (!ret)
		ret = reg_read(OUTPUT_FORMAT_CTRL2_B);
	if (ret >= 0)
		ret = reg_write(OUTPUT_FORMAT_CTRL2_B, (ret & ~mask) | outfmt);

	return ret;
}

static int mt9m111_setfmt_bayer8(struct i2c_client *client)
{
	return mt9m111_setup_pixfmt(client, MT9M111_OUTFMT_PROCESSED_BAYER |
				    MT9M111_OUTFMT_RGB);
}

static int mt9m111_setfmt_bayer10(struct i2c_client *client)
{
	return mt9m111_setup_pixfmt(client, MT9M111_OUTFMT_BYPASS_IFP);
}

static int mt9m111_setfmt_rgb565(struct i2c_client *client)
{
	struct mt9m111_data *mt9m111 = to_mt9m111(client);
	int val = 0;

	if (mt9m111->swap_rgb_red_blue)
		val |= MT9M111_OUTFMT_SWAP_YCbCr_Cb_Cr;
	if (mt9m111->swap_rgb_even_odd)
		val |= MT9M111_OUTFMT_SWAP_RGB_EVEN;
	val |= MT9M111_OUTFMT_RGB | MT9M111_OUTFMT_RGB565;

	return mt9m111_setup_pixfmt(client, val);
}

static int mt9m111_setfmt_rgb555(struct i2c_client *client)
{
	struct mt9m111_data *mt9m111 = to_mt9m111(client);
	int val = 0;

	if (mt9m111->swap_rgb_red_blue)
		val |= MT9M111_OUTFMT_SWAP_YCbCr_Cb_Cr;
	if (mt9m111->swap_rgb_even_odd)
		val |= MT9M111_OUTFMT_SWAP_RGB_EVEN;
	val |= MT9M111_OUTFMT_RGB | MT9M111_OUTFMT_RGB555;

	return mt9m111_setup_pixfmt(client, val);
}

static int mt9m111_setfmt_yuv(struct i2c_client *client)
{
	struct mt9m111_data *mt9m111 = to_mt9m111(client);
	int val = 0;

	if (mt9m111->swap_yuv_cb_cr)
		val |= MT9M111_OUTFMT_SWAP_YCbCr_Cb_Cr;
	if (mt9m111->swap_yuv_y_chromas)
		val |= MT9M111_OUTFMT_SWAP_YCbCr_C_Y;

	return mt9m111_setup_pixfmt(client, val);
}

static int mt9m111_enable(struct i2c_client *client)
{
	struct mt9m111_data *mt9m111 = to_mt9m111(client);
	int ret;

	ret = reg_set(RESET, MT9M111_RESET_CHIP_ENABLE);
	if (!ret)
		mt9m111->powered = 1;
	return ret;
}

static int mt9m111_reset(struct i2c_client *client)
{
	int ret;

	ret = reg_set(RESET, MT9M111_RESET_RESET_MODE);
	if (!ret)
		ret = reg_set(RESET, MT9M111_RESET_RESET_SOC);
	if (!ret)
		ret = reg_clear(RESET, MT9M111_RESET_RESET_MODE
				| MT9M111_RESET_RESET_SOC);

	return ret;
}

static int mt9m111_set_flip(struct i2c_client *client, int flip, int mask)
{
	struct mt9m111_data *mt9m111 = to_mt9m111(client);
	int ret;

	if (mt9m111->context == HIGHPOWER) {
		if (flip)
			ret = reg_set(READ_MODE_B, mask);
		else
			ret = reg_clear(READ_MODE_B, mask);
	} else {
		if (flip)
			ret = reg_set(READ_MODE_A, mask);
		else
			ret = reg_clear(READ_MODE_A, mask);
	}

	return ret;
}

static int mt9m111_get_global_gain(struct i2c_client *client)
{
	int data;

	data = reg_read(GLOBAL_GAIN);
	if (data >= 0)
		return (data & 0x2f) * (1 << ((data >> 10) & 1)) *
			(1 << ((data >> 9) & 1));
	return data;
}

static int mt9m111_set_global_gain(struct i2c_client *client, int gain)
{
	struct mt9m111_data *mt9m111 = to_mt9m111(client);
	u16 val;

	if (gain > 63 * 2 * 2)
		return -EINVAL;

	mt9m111->gain = gain;
	if ((gain >= 64 * 2) && (gain < 63 * 2 * 2))
		val = (1 << 10) | (1 << 9) | (gain / 4);
	else if ((gain >= 64) && (gain < 64 * 2))
		val = (1 << 9) | (gain / 2);
	else
		val = gain;

	return reg_write(GLOBAL_GAIN, val);
}

static int mt9m111_set_autoexposure(struct i2c_client *client, int on)
{
	struct mt9m111_data *mt9m111 = to_mt9m111(client);
	int ret;

	if (on)
		ret = reg_set(OPER_MODE_CTRL, MT9M111_OPMODE_AUTOEXPO_EN);
	else
		ret = reg_clear(OPER_MODE_CTRL, MT9M111_OPMODE_AUTOEXPO_EN);

	if (!ret)
		mt9m111->autoexposure = on;

	return ret;
}

static int mt9m111_set_autowhitebalance(struct i2c_client *client, int on)
{
	struct mt9m111_data *mt9m111 = to_mt9m111(client);
	int ret;

	if (on)
		ret = reg_set(OPER_MODE_CTRL, MT9M111_OPMODE_AUTOWHITEBAL_EN);
	else
		ret = reg_clear(OPER_MODE_CTRL, MT9M111_OPMODE_AUTOWHITEBAL_EN);

	if (!ret)
		mt9m111->autowhitebalance = on;

	return ret;
}


#ifdef MT9M111_DEBUG
/*!
 * Set sensor to test mode, which will generate test pattern.
 *
 * @return none
 */
static void mt9m111_test_pattern(bool flag)
{
    printk("::: mt9m111_test_pattern\n");
    
	u16 data;

	/* switch to sensor registers */
	mt9m111_write_reg(MT9M111I_ADDR_SPACE_SEL, MT9M111I_SEL_SCA);

	if (flag == true) {
		testpattern = MT9M111S_OUTCTRL_TEST_MODE;

		data = mt9m111_read_reg(MT9M111S_ROW_NOISE_CTRL) & 0xBF;
		mt9m111_write_reg(MT9M111S_ROW_NOISE_CTRL, data);

		mt9m111_write_reg(MT9M111S_TEST_DATA, 0);

		/* changes take effect */
		data = MT9M111S_OUTCTRL_CHIP_ENABLE | testpattern | 0x3000;
		mt9m111_write_reg(MT9M111S_OUTPUT_CTRL, data);
	} else {
		testpattern = 0;

		data = mt9m111_read_reg(MT9M111S_ROW_NOISE_CTRL) | 0x40;
		mt9m111_write_reg(MT9M111S_ROW_NOISE_CTRL, data);

		/* changes take effect */
		data = MT9M111S_OUTCTRL_CHIP_ENABLE | testpattern | 0x3000;
		mt9m111_write_reg(MT9M111S_OUTPUT_CTRL, data);
	}

    printk("::: mt9m111_test_pattern (END)\n");
}
#endif

static int mt9m111_sensor_init(struct i2c_client *client)
{
	struct mt9m111_data *mt9m111 = to_mt9m111(client);
	int ret;

	mt9m111->context = HIGHPOWER;
	ret = mt9m111_enable(client);
	if (!ret)
		ret = mt9m111_reset(client);
	if (!ret)
		ret = mt9m111_set_context(client, mt9m111->context);
	if (!ret)
		ret = mt9m111_set_autoexposure(client, mt9m111->autoexposure);
	if (!ret)
		ret = mt9m111_setfmt_yuv(client);
	if (ret)
		dev_err(&client->dev, "mt9m111 init failed: %d\n", ret);
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
	struct mt9m111_data *mt9m111 = v4l2_to_mt9m111(s);

	pr_debug("In mt9m111:ioctl_g_ifparm\n");

	if (s == NULL) {
        printk("::: ioctl_g_ifparm (END, error)\n");
    
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	memset(p, 0, sizeof(*p));
	p->u.bt656.clock_curr = mt9m111->mclk;
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.clock_min = MT9M111_CLK_MIN;
	p->u.bt656.clock_max = MT9M111_CLK_MAX;

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
	struct mt9m111_data *sensor = s->priv;

	pr_debug("In mt9m111:ioctl_s_power\n");

	sensor->powered = on;

	if (on)
		gpio_sensor_active();
	else
		gpio_sensor_inactive();

	return 0;
}

static unsigned long mt9m111_get_cyclesperframe(struct mt9m111_data *mt9m111)
{
	struct i2c_client *client = mt9m111->i2c_client;
	unsigned int a, q;
	unsigned long f;

	a = mt9m111_reg_read(client, MT9M111_WINDOW_WIDTH) + 8;
	if (mt9m111->context == HIGHPOWER)
		q = mt9m111_reg_read(client, MT9M111_HORIZONTAL_BLANKING_B);
	else
		q = mt9m111_reg_read(client, MT9M111_HORIZONTAL_BLANKING_A);

	f = mt9m111_reg_read(client, MT9M111_WINDOW_HEIGHT);

	if (mt9m111->context == HIGHPOWER)
		f += mt9m111_reg_read(client, MT9M111_VERTICAL_BLANKING_B);
	else
		f += mt9m111_reg_read(client, MT9M111_VERTICAL_BLANKING_A);

	f *= (a + q);

	/* One pixel clock cycle = 2 or 4 master clock cycles */
	if (mt9m111->context == HIGHPOWER)
		f *= 2;
	else
		f *= 4;

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
	struct mt9m111_data *mt9m111 = v4l2_to_mt9m111(s);
	int ret = 0;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	/* s->priv points to mt9m111_data */

	pr_debug("In mt9m111:ioctl_g_parm\n");

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n");
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

		mt9m111->streamcap.timeperframe.denominator = mt9m111->mclk;
		mt9m111->streamcap.timeperframe.numerator =
			mt9m111_get_cyclesperframe(mt9m111);

		cparm->capability = mt9m111->streamcap.capability;
		cparm->timeperframe =
				mt9m111->streamcap.timeperframe;
		cparm->capturemode = mt9m111->streamcap.capturemode;
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
	struct mt9m111_data *mt9m111 = v4l2_to_mt9m111(s);
	int ret = 0;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	uint32_t clock_rate;

	pr_debug("In mt9m111:ioctl_s_parm\n");

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n");

		clock_rate = mt9m111_get_cyclesperframe(mt9m111) *
			cparm->timeperframe.denominator /
			cparm->timeperframe.numerator;

		if (clock_rate > MT9M111_CLK_MAX)
			clock_rate = MT9M111_CLK_MAX;

		set_mclk_rate(&clock_rate, 0);
		mt9m111->mclk = clock_rate;

		mt9m111->streamcap.timeperframe.denominator = mt9m111->mclk;
		mt9m111->streamcap.timeperframe.numerator =
			mt9m111_get_cyclesperframe(mt9m111);

		/* Check that new capture mode is supported. */
		if ((cparm->capturemode != 0) &&
		    !(cparm->capturemode & V4L2_MODE_HIGHQUALITY)) {
			pr_err("ERROR: mt9m111: ioctl_s_parm: " \
				"unsupported capture mode\n");
			ret  = -EINVAL;
		} else {
			mt9m111->streamcap.capturemode =
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
	struct mt9m111_data *sensor = s->priv;
	/* s->priv points to mt9m111_data */

	pr_debug("In mt9m111:ioctl_g_fmt_cap.\n");
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
	pr_debug("In mt9m111:ioctl_queryctrl\n");

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
	struct mt9m111_data *mt9m111 = v4l2_to_mt9m111(s);
	struct i2c_client *client = mt9m111->i2c_client;
	int data;
    
	pr_debug("In mt9m111:ioctl_g_ctrl\n");

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
		vc->value = mt9m111->autowhitebalance;
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
		vc->value = mt9m111->autoexposure;
		break;
	case V4L2_CID_AUTOGAIN:
		pr_debug("   V4L2_CID_AUTOGAIN\n");
		vc->value = 0;
		break;
	case V4L2_CID_GAIN:
		pr_debug("   V4L2_CID_GAIN\n");
		data = mt9m111_get_global_gain(client);
		if (data < 0)
			return data;
		vc->value = data;
		break;
	case V4L2_CID_HFLIP:
		pr_debug("   V4L2_CID_HFLIP\n");
		if (mt9m111->context == HIGHPOWER)
			data = reg_read(READ_MODE_B);
		else
			data = reg_read(READ_MODE_A);

		if (data < 0)
			return -EIO;
		vc->value = !!(data & MT9M111_RMB_MIRROR_COLS);
		break;
	case V4L2_CID_VFLIP:
		pr_debug("   V4L2_CID_VFLIP\n");
		if (mt9m111->context == HIGHPOWER)
			data = reg_read(READ_MODE_B);
		else
			data = reg_read(READ_MODE_A);

		if (data < 0)
			return -EIO;
		vc->value = !!(data & MT9M111_RMB_MIRROR_ROWS);
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
	struct mt9m111_data *mt9m111 = v4l2_to_mt9m111(s);
	struct i2c_client *client = mt9m111->i2c_client;
	int retval = 0;

	pr_debug("In mt9m111:ioctl_s_ctrl %d\n",
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
		retval = mt9m111_set_autowhitebalance(client, vc->value);
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
		retval = mt9m111_set_autoexposure(client, vc->value);
		break;
	case V4L2_CID_AUTOGAIN:
		pr_debug("   V4L2_CID_AUTOGAIN\n");
		break;
	case V4L2_CID_GAIN:
		pr_debug("   V4L2_CID_GAIN\n");
		retval = mt9m111_set_global_gain(client, vc->value);
		break;
	case V4L2_CID_HFLIP:
		pr_debug("   V4L2_CID_HFLIP\n");
		mt9m111->hflip = vc->value;
		retval = mt9m111_set_flip(client, vc->value,
					MT9M111_RMB_MIRROR_COLS);
		break;
	case V4L2_CID_VFLIP:
		pr_debug("   V4L2_CID_VFLIP\n");
		mt9m111->vflip = vc->value;
		retval = mt9m111_set_flip(client, vc->value,
					MT9M111_RMB_MIRROR_ROWS);
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
	struct mt9m111_data *mt9m111 = v4l2_to_mt9m111(s);

	if (fmt->index > 1)
		return -EINVAL;

	fmt->pixelformat = mt9m111->pix.pixelformat;

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
	struct mt9m111_data *mt9m111 = v4l2_to_mt9m111(s);

	fsize->pixel_format = mt9m111->pix.pixelformat;
	fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
	fsize->stepwise.min_height = 0;
	fsize->stepwise.min_width = 0;
	fsize->stepwise.max_height = MT9M111_MAX_HEIGHT;
	fsize->stepwise.max_width = MT9M111_MAX_WIDTH;
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
static int ioctl_g_chip_ident(struct v4l2_int_device *s, struct v4l2_dbg_chip_ident *id)
{
	id->ident = V4L2_IDENT_MT9M111;

	id->match.type = V4L2_CHIP_MATCH_I2C_DRIVER;
	strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name, "mt9m111");

	return 0;
}


/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{
    printk("::: ioctl_init\n");
    
	pr_debug("In mt9m111:ioctl_init\n");

    printk("::: ioctl_init (END)\n");

	return 0;
}

/*
 * Interface active, can use i2c. If it fails, it can indeed mean, that
 * this wasn't our capture interface, so, we wait for the right one
 */
static int mt9m111_video_probe(struct mt9m111_data *mt9m111)
{
	struct i2c_client *client = mt9m111->i2c_client;
	s32 data;
	int ret;

	mt9m111->autoexposure = 1;
	mt9m111->autowhitebalance = 1;

	mt9m111->swap_rgb_even_odd = 1;
	mt9m111->swap_rgb_red_blue = 1;

	data = reg_read(CHIP_VERSION);

	switch (data) {
	case 0x143a: /* MT9M111 or MT9M131 */
		mt9m111->model = V4L2_IDENT_MT9M111;
		dev_info(&client->dev,
			"Detected a MT9M111/MT9M131 chip ID %x\n", data);
		break;
	case 0x148c: /* MT9M112 */
		mt9m111->model = V4L2_IDENT_MT9M112;
		dev_info(&client->dev, "Detected a MT9M112 chip ID %x\n", data);
		break;
	default:
		ret = -ENODEV;
		dev_err(&client->dev,
			"No MT9M111/MT9M112/MT9M131 chip detected register read %x\n",
			data);
		goto ei2c;
	}

	ret = mt9m111_sensor_init(client);

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
	uint32_t clock_rate = MT9M111_MCLK;
	struct mt9m111_data *mt9m111 = v4l2_to_mt9m111(s);

	pr_debug("In mt9m111:ioctl_dev_init\n");

	gpio_sensor_active();

	set_mclk_rate(&clock_rate, 0);                        // TODO: unhardcode '0' csi

	mt9m111->mclk = clock_rate;

	mt9m111_video_probe(mt9m111);

	return 0;
}

/*!
 * This structure defines all the ioctls for this module and links them to the
 * enumeration.
 */
static struct v4l2_int_ioctl_desc mt9m111_ioctl_desc[] = {

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
	{vidioc_int_enum_framesizes_num,
				(v4l2_int_ioctl_func *)ioctl_enum_framesizes},
	{vidioc_int_g_chip_ident_num,
				(v4l2_int_ioctl_func *)ioctl_g_chip_ident},
};

static struct v4l2_int_slave mt9m111_slave = {
	.ioctls = mt9m111_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(mt9m111_ioctl_desc),
};

static struct v4l2_int_device mt9m111_int_device = {
	.module = THIS_MODULE,
	.name = "mt9m111",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &mt9m111_slave,
		},
};

/*!
 * mt9m111 I2C probe function
 * Function set in i2c_driver struct.
 * Called by insmod mt9m111_camera.ko.
 *
 * @return  Error code indicating success or failure
 */
static int mt9m111_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int retval;
	struct mt9m111_data *mt9m111;

	pr_debug("In mt9m111_probe  device id is %s\n", id->name);

	mt9m111 = kzalloc(sizeof(struct mt9m111_data), GFP_KERNEL);
	if (!mt9m111)
		return -ENOMEM;

	/* Set initial values for the sensor struct. */
	mt9m111->i2c_client = client;
	i2c_set_clientdata(client, mt9m111);
	mt9m111->platform_data = client->dev.platform_data;
	mt9m111->csi = mt9m111->platform_data->csi;
	mt9m111->io_init = mt9m111->platform_data->io_init;

	pr_debug("   client name is %s\n", client->name);
	mt9m111->pix.pixelformat = V4L2_PIX_FMT_UYVY;
	mt9m111->pix.width = MT9M111_MAX_WIDTH;
	mt9m111->pix.height = MT9M111_MAX_HEIGHT;
	mt9m111->streamcap.capability = 0; /* No higher resolution or frame
						* frame rate changes supported.
						*/
	mt9m111->streamcap.timeperframe.denominator = MT9M111_FRAME_RATE;
	mt9m111->streamcap.timeperframe.numerator = 1;


	mt9m111_int_device.priv = mt9m111;

	pr_debug("   type is %d (expect %d)\n",
			mt9m111_int_device.type, v4l2_int_type_slave);
	pr_debug("   num ioctls is %d\n",
			mt9m111_int_device.u.slave->num_ioctls);

	/* This function attaches this structure to the /dev/video0 device.
	 * The pointer in priv points to the mt9m111_data structure here.*/
	retval = v4l2_int_device_register(&mt9m111_int_device);

	if (mt9m111->io_init)
		mt9m111->io_init();


	return retval;
}

/*!
 * Function set in i2c_driver struct.
 * Called on rmmod mt9m111_camera.ko
 */
static int mt9m111_remove(struct i2c_client *client)
{
	pr_debug("In mt9m111_remove\n");

	v4l2_int_device_unregister(&mt9m111_int_device);

	return 0;
}

static const struct i2c_device_id mt9m111_id[] = {
	{"mt9m111", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, mt9m111_id);

static struct i2c_driver mt9m111_i2c_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "mt9m111",
		   },
	.probe = mt9m111_probe,
	.remove = mt9m111_remove,
	.id_table = mt9m111_id,
/* To add power management add .suspend and .resume functions */
};


/*!
 * MT9M111 init function.
 * Called by insmod mt9m111_camera.ko.
 *
 * @return  Error code indicating success or failure
 */
static __init int mt9m111_init(void)
{
	u8 err;

	/* Tells the i2c driver what functions to call for this driver. */
	err = i2c_add_driver(&mt9m111_i2c_driver);
	if (err != 0)
		pr_err("%s: driver registration failed, error=%d \n",
				__func__, err);

	return err;
}

/*!
 * MT9M111 cleanup function.
 * Called on rmmod mt9m111_camera.ko
 *
 * @return  Error code indicating success or failure
 */
static void __exit mt9m111_clean(void)
{
	i2c_del_driver(&mt9m111_i2c_driver);

	gpio_sensor_inactive();
}

module_init(mt9m111_init);
module_exit(mt9m111_clean);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Mt9m111 Camera Driver");
MODULE_LICENSE("GPL");
