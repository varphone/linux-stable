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
#include "mxc_v4l2_capture.h"
#include "mt9m111.h"

#ifdef MT9M111_DEBUG
static u16 testpattern;
#endif

static mt9m111_conf mt9m111_device;

/*!
 * Holds the current frame rate.
 */
static int reset_frame_rate = MT9M111_FRAME_RATE;

struct sensor {
	const struct mt9m111_platform_data *platform_data;
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

} mt9m111_data;

extern void gpio_sensor_active(void);
extern void gpio_sensor_inactive(void);

static int mt9m111_probe(struct i2c_client *client,
			 const struct i2c_device_id *id);
static int mt9m111_remove(struct i2c_client *client);

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

/*
 * Function definitions
 */

#ifdef MT9M111_DEBUG
static inline int mt9m111_read_reg(u8 reg)
{
    printk("::: mt9m111_read_reg\n");
    
	int val = i2c_smbus_read_word_data(mt9m111_data.i2c_client, reg);
	if (val != -1)
		val = cpu_to_be16(val);

    printk("::: mt9m111_read_reg (END, ret=%d)\n", val);
	return val;
}
#endif

/*!
 * Writes to the register via I2C.
 */
static inline int mt9m111_write_reg(u8 reg, u16 val)
{
    printk("::: mt9m111_write_reg\n");
    
	pr_debug("In mt9m111_write_reg (0x%x, 0x%x)\n", reg, val);
	pr_debug("   write reg %x val %x.\n", reg, val);

    int ret = i2c_smbus_write_word_data(mt9m111_data.i2c_client,
					 reg, cpu_to_be16(val));
    
    printk("::: mt9m111_write_reg (END, ret=%d)\n", ret);
    
	return ret;
}

/*!
 * Initialize mt9m111_sensor_lib
 * Libarary for Sensor configuration through I2C
 *
 * @param       coreReg       Core Registers
 * @param       ifpReg        IFP Register
 *
 * @return status
 */
static u8 mt9m111_sensor_lib(mt9m111_coreReg *coreReg, mt9m111_IFPReg *ifpReg)
{
    printk("::: mt9m111_sensor_lib\n");
    
	u8 reg;
	u16 data;
	u8 error = 0;

	pr_debug("In mt9m111_sensor_lib\n");

	/*
	 * setup to IFP registers
	 */
	reg = MT9M111I_ADDR_SPACE_SEL;
	data = ifpReg->addrSpaceSel;
	mt9m111_write_reg(reg, data);

	/* Operation Mode Control */
	reg = MT9M111I_MODE_CONTROL;
	data = ifpReg->modeControl;
	mt9m111_write_reg(reg, data);

	/* Output format */
	reg = MT9M111I_FORMAT_CONTROL;
	data = ifpReg->formatControl;	/* Set bit 12 */
	mt9m111_write_reg(reg, data);

	/* AE limit 4 */
	reg = MT9M111I_SHUTTER_WIDTH_LIMIT_AE;
	data = ifpReg->gainLimitAE;
	mt9m111_write_reg(reg, data);

	reg = MT9M111I_OUTPUT_FORMAT_CTRL2;
	data = ifpReg->outputFormatCtrl2;
	mt9m111_write_reg(reg, data);

	reg = MT9M111I_AE_SPEED;
	data = ifpReg->AESpeed;
	mt9m111_write_reg(reg, data);

	/* output image size */
	reg = MT9M111i_H_PAN;
	data = 0x8000 | ifpReg->HPan;
	mt9m111_write_reg(reg, data);

	reg = MT9M111i_H_ZOOM;
	data = 0x8000 | ifpReg->HZoom;
	mt9m111_write_reg(reg, data);

	reg = MT9M111i_H_SIZE;
	data = 0x8000 | ifpReg->HSize;
	mt9m111_write_reg(reg, data);

	reg = MT9M111i_V_PAN;
	data = 0x8000 | ifpReg->VPan;
	mt9m111_write_reg(reg, data);

	reg = MT9M111i_V_ZOOM;
	data = 0x8000 | ifpReg->VZoom;
	mt9m111_write_reg(reg, data);

	reg = MT9M111i_V_SIZE;
	data = 0x8000 | ifpReg->VSize;
	mt9m111_write_reg(reg, data);

	reg = MT9M111i_H_PAN;
	data = ~0x8000 & ifpReg->HPan;
	mt9m111_write_reg(reg, data);
#if 0
	reg = MT9M111I_UPPER_SHUTTER_DELAY_LIM;
	data = ifpReg->upperShutterDelayLi;
	mt9m111_write_reg(reg, data);

	reg = MT9M111I_SHUTTER_60;
	data = ifpReg->shutter_width_60;
	mt9m111_write_reg(reg, data);

	reg = MT9M111I_SEARCH_FLICK_60;
	data = ifpReg->search_flicker_60;
	mt9m111_write_reg(reg, data);
#endif

	/*
	 * setup to sensor core registers
	 */
	reg = MT9M111I_ADDR_SPACE_SEL;
	data = coreReg->addressSelect;
	mt9m111_write_reg(reg, data);

	/* enable changes and put the Sync bit on */
	reg = MT9M111S_OUTPUT_CTRL;
	data = MT9M111S_OUTCTRL_SYNC | MT9M111S_OUTCTRL_CHIP_ENABLE | 0x3000;
	mt9m111_write_reg(reg, data);

	/* min PIXCLK - Default */
	reg = MT9M111S_PIXEL_CLOCK_SPEED;
	data = coreReg->pixelClockSpeed;
	mt9m111_write_reg(reg, data);

	/* Setup image flipping / Dark rows / row/column skip */
	reg = MT9M111S_READ_MODE;
	data = coreReg->readMode;
	mt9m111_write_reg(reg, data);

	/* zoom 0 */
	reg = MT9M111S_DIGITAL_ZOOM;
	data = coreReg->digitalZoom;
	mt9m111_write_reg(reg, data);

	/* min H-blank */
	reg = MT9M111S_HOR_BLANKING;
	data = coreReg->horizontalBlanking;
	mt9m111_write_reg(reg, data);

	/* min V-blank */
	reg = MT9M111S_VER_BLANKING;
	data = coreReg->verticalBlanking;
	mt9m111_write_reg(reg, data);

	reg = MT9M111S_SHUTTER_WIDTH;
	data = coreReg->shutterWidth;
	mt9m111_write_reg(reg, data);

	reg = MT9M111S_SHUTTER_DELAY;
	data = ifpReg->upperShutterDelayLi;
	mt9m111_write_reg(reg, data);

	/* changes become effective */
	reg = MT9M111S_OUTPUT_CTRL;
	data = MT9M111S_OUTCTRL_CHIP_ENABLE | 0x3000;
	mt9m111_write_reg(reg, data);

    printk("::: mt9m111_sensor_lib (END, error=%d)\n", error);
    
	return error;
}

/*!
 * MT9M111 frame rate calculate
 *
 * @param frame_rate       int *
 * @param mclk             int
 * @return  None
 */
static void mt9m111_rate_cal(int *frame_rate, int mclk)
{
    printk("::: mt9m111_rate_cal\n");
    
	int num_clock_per_row;
	int max_rate = 0;

	pr_debug("In mt9m111_rate_cal\n");

	num_clock_per_row = (MT9M111_MAX_WIDTH + 114 + MT9M111_HORZBLANK_MIN)
			* 2;
	max_rate = mclk / (num_clock_per_row *
			   (MT9M111_MAX_HEIGHT + MT9M111_VERTBLANK_DEFAULT));

	if ((*frame_rate > max_rate) || (*frame_rate == 0)) {
		*frame_rate = max_rate;
	}

	mt9m111_device.coreReg->verticalBlanking
	    = mclk / (*frame_rate * num_clock_per_row) - MT9M111_MAX_HEIGHT;

	reset_frame_rate = *frame_rate;

    printk("::: mt9m111_rate_cal (END)\n");
}

/*!
 * MT9M111 sensor configuration
 */
void mt9m111_config(void)
{
    printk("::: mt9m111_config\n");
    
	pr_debug("In mt9m111_config\n");

	mt9m111_device.coreReg->addressSelect = MT9M111I_SEL_SCA;
	mt9m111_device.ifpReg->addrSpaceSel = MT9M111I_SEL_IFP;

	mt9m111_device.coreReg->windowHeight = MT9M111_WINHEIGHT;
	mt9m111_device.coreReg->windowWidth = MT9M111_WINWIDTH;
	mt9m111_device.coreReg->zoomColStart = 0;
	mt9m111_device.coreReg->zomRowStart = 0;
	mt9m111_device.coreReg->digitalZoom = 0x0;

	mt9m111_device.coreReg->verticalBlanking = MT9M111_VERTBLANK_DEFAULT;
	mt9m111_device.coreReg->horizontalBlanking = MT9M111_HORZBLANK_MIN;
	mt9m111_device.coreReg->pixelClockSpeed = 0;
	mt9m111_device.coreReg->readMode = 0xd0a1;

	mt9m111_device.ifpReg->outputFormatCtrl2 = 0;
	mt9m111_device.ifpReg->gainLimitAE = 0x300;
	mt9m111_device.ifpReg->AESpeed = 0x80;

	/* here is the default value */
	mt9m111_device.ifpReg->formatControl = 0xc800;
	mt9m111_device.ifpReg->modeControl = 0x708e;
	mt9m111_device.ifpReg->awbSpeed = 0x4514;
	mt9m111_device.coreReg->shutterWidth = 0xf8;

	/* output size */
	mt9m111_device.ifpReg->HPan = 0;
	mt9m111_device.ifpReg->HZoom = MT9M111_MAX_WIDTH;
	mt9m111_device.ifpReg->HSize = MT9M111_MAX_WIDTH;
	mt9m111_device.ifpReg->VPan = 0;
	mt9m111_device.ifpReg->VZoom = MT9M111_MAX_HEIGHT;
	mt9m111_device.ifpReg->VSize = MT9M111_MAX_HEIGHT;
    
    printk("::: mt9m111_config (END)\n");
}

/*!
 * mt9m111 sensor set saturtionn
 *
 * @param saturation   int

 * @return  Error code of 0.
 */
static int mt9m111_set_saturation(int saturation)
{
    printk("::: mt9m111_set_saturation\n");
    
	u8 reg;
	u16 data;
	pr_debug("In mt9m111_set_saturation(%d)\n",
		saturation);

	switch (saturation) {
	case 150:
		mt9m111_device.ifpReg->awbSpeed = 0x6D14;
		break;
	case 100:
		mt9m111_device.ifpReg->awbSpeed = 0x4514;
		break;
	case 75:
		mt9m111_device.ifpReg->awbSpeed = 0x4D14;
		break;
	case 50:
		mt9m111_device.ifpReg->awbSpeed = 0x5514;
		break;
	case 37:
		mt9m111_device.ifpReg->awbSpeed = 0x5D14;
		break;
	case 25:
		mt9m111_device.ifpReg->awbSpeed = 0x6514;
		break;
	default:
		mt9m111_device.ifpReg->awbSpeed = 0x4514;
		break;
	}

	reg = MT9M111I_ADDR_SPACE_SEL;
	data = mt9m111_device.ifpReg->addrSpaceSel;
	mt9m111_write_reg(reg, data);

	/* Operation Mode Control */
	reg = MT9M111I_AWB_SPEED;
	data = mt9m111_device.ifpReg->awbSpeed;
	mt9m111_write_reg(reg, data);

    printk("::: mt9m111_set_saturation (END)\n");
    
	return 0;
}

/*!
 * mt9m111 sensor set Auto Exposure measurement window mode configuration
 *
 * @param ae_mode      int
 * @return  Error code of 0 (no Error)
 */
static int mt9m111_set_ae_mode(int ae_mode)
{
    printk("::: mt9m111_set_ae_mode\n");
    
	u8 reg;
	u16 data;

	pr_debug("In mt9m111_set_ae_mode(%d)\n",
		ae_mode);

	/* Currently this driver only supports auto and manual exposure
	 * modes. */
	if ((ae_mode > 1) || (ae_mode << 0))
    {
        printk("::: mt9m111_set_ae_mode (END, error=-EPERM)\n");
    
		return -EPERM;
    }

	/*
	 * The auto exposure is set in bit 14.
	 * Other values are set for:
	 *  -on the fly defect correction is on (bit 13).
	 *  -aperature correction knee enabled (bit 12).
	 *  -ITU_R BT656 synchronization codes are embedded in the image (bit 7)
	 *  -AE measurement window is weighted sum of large and center windows
	 *     (bits 2-3).
	 *  -auto white balance is on (bit 1).
	 *  -normal color processing (bit 4 = 0).
	 */
	/* V4L2_EXPOSURE_AUTO = 0; needs register setting of 0x708E */
	/* V4L2_EXPOSURE_MANUAL = 1 needs register setting of 0x308E */
	mt9m111_device.ifpReg->modeControl &= 0x3fff;
	mt9m111_device.ifpReg->modeControl |= (ae_mode & 0x03) << 14;
	mt9m111_data.ae_mode = ae_mode;

	reg = MT9M111I_ADDR_SPACE_SEL;
	data = mt9m111_device.ifpReg->addrSpaceSel;
	mt9m111_write_reg(reg, data);

	reg = MT9M111I_MODE_CONTROL;
	data = mt9m111_device.ifpReg->modeControl;
	mt9m111_write_reg(reg, data);

    printk("::: mt9m111_set_ae_mode (END)\n");
    
	return 0;
}

/*!
 * mt9m111 sensor get AE measurement window mode configuration
 *
 * @param ae_mode      int *
 * @return  None
 */
static void mt9m111_get_ae_mode(int *ae_mode)
{
    printk("::: mt9m111_get_ae_mode\n");
    
	pr_debug("In mt9m111_get_ae_mode(%d)\n", *ae_mode);

	if (ae_mode != NULL) {
		*ae_mode = (mt9m111_device.ifpReg->modeControl & 0xc) >> 2;
	}

    printk("::: mt9m111_get_ae_mode (END)\n");
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
    printk("::: ioctl_g_ifparm\n");
    
	pr_debug("In mt9m111:ioctl_g_ifparm\n");

	if (s == NULL) {
        printk("::: ioctl_g_ifparm (END, error)\n");
    
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	memset(p, 0, sizeof(*p));
	p->u.bt656.clock_curr = MT9M111_MCLK;
	p->if_type = V4L2_IF_TYPE_BT656;
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	p->u.bt656.clock_min = MT9M111_CLK_MIN;
	p->u.bt656.clock_max = MT9M111_CLK_MAX;

    printk("::: ioctl_g_ifparm (END)\n");
    
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
    printk("::: ioctl_s_power\n");
    
	struct sensor *sensor = s->priv;

	pr_debug("In mt9m111:ioctl_s_power\n");

	sensor->on = on;

	if (on)
		gpio_sensor_active();
	else
		gpio_sensor_inactive();

    printk("::: ioctl_s_power (END)\n");

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
    printk("::: ioctl_g_parm\n");
    
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
		cparm->capability = mt9m111_data.streamcap.capability;
		cparm->timeperframe =
				mt9m111_data.streamcap.timeperframe;
		cparm->capturemode = mt9m111_data.streamcap.capturemode;
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

    printk("::: ioctl_g_parm (END, ret=%d)\n", ret);
    
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
    printk("::: ioctl_s_parm\n");
    
	int ret = 0;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	/* s->priv points to mt9m111_data */

	pr_debug("In mt9m111:ioctl_s_parm\n");

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n");

		/* Check that the new frame rate is allowed.
		 * Changing the frame rate is not allowed on this
		 *camera. */
		if (cparm->timeperframe.denominator !=
		    mt9m111_data.streamcap.timeperframe.denominator) {
			pr_err("ERROR: mt9m111: ioctl_s_parm: " \
			       "This camera does not allow frame rate "
			       "changes.\n");
			ret = -EINVAL;
		} else {
			mt9m111_data.streamcap.timeperframe =
						cparm->timeperframe;
		      /* Call any camera functions to match settings. */
		}

		/* Check that new capture mode is supported. */
		if ((cparm->capturemode != 0) &&
		    !(cparm->capturemode & V4L2_MODE_HIGHQUALITY)) {
			pr_err("ERROR: mt9m111: ioctl_s_parm: " \
				"unsupported capture mode\n");
			ret  = -EINVAL;
		} else {
			mt9m111_data.streamcap.capturemode =
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

    printk("::: ioctl_s_parm (END, ret=%d)\n", ret);
    
	return 0;                 // TODO: wtf, must return 'ret'
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
    printk("::: ioctl_g_fmt_cap\n");
    
	struct sensor *sensor = s->priv;
	/* s->priv points to mt9m111_data */

	pr_debug("In mt9m111:ioctl_g_fmt_cap.\n");
	pr_debug("   Returning size of %dx%d\n",
		sensor->pix.width, sensor->pix.height);

	f->fmt.pix = sensor->pix;

    printk("::: ioctl_g_fmt_cap (END)\n");

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
    printk("::: ioctl_queryctrl\n");
    
	pr_debug("In mt9m111:ioctl_queryctrl\n");

    printk("::: ioctl_queryctrl (END)\n");

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
    printk("::: ioctl_g_ctrl\n");
    
	pr_debug("In mt9m111:ioctl_g_ctrl\n");

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		pr_debug("   V4L2_CID_BRIGHTNESS\n");
		vc->value = mt9m111_data.brightness;
		break;
	case V4L2_CID_CONTRAST:
		pr_debug("   V4L2_CID_CONTRAST\n");
		vc->value = mt9m111_data.contrast;
		break;
	case V4L2_CID_SATURATION:
		pr_debug("   V4L2_CID_SATURATION\n");
		vc->value = mt9m111_data.saturation;
		break;
	case V4L2_CID_HUE:
		pr_debug("   V4L2_CID_HUE\n");
		vc->value = mt9m111_data.hue;
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
		vc->value = mt9m111_data.red;
		break;
	case V4L2_CID_BLUE_BALANCE:
		pr_debug("   V4L2_CID_BLUE_BALANCE\n");
		vc->value = mt9m111_data.blue;
		break;
	case V4L2_CID_GAMMA:
		pr_debug("   V4L2_CID_GAMMA\n");
		vc->value = 0;
		break;
	case V4L2_CID_EXPOSURE:
		pr_debug("   V4L2_CID_EXPOSURE\n");
		vc->value = mt9m111_data.ae_mode;
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
        printk("::: ioctl_g_ctrl (END, error=-EPERM)\n");
		return -EPERM;
		break;
	}

    printk("::: ioctl_g_ctrl (END)\n");
    
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
    printk("::: ioctl_s_ctrl\n");
    
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
		retval = mt9m111_set_saturation(vc->value);
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
		retval = mt9m111_set_ae_mode(vc->value);
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
		retval = -EPERM;
		break;
	}

    printk("::: ioctl_s_ctrl (END, ret=%d)\n", retval);
    
	return retval;
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

/*!
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
    printk("::: ioctl_dev_init\n");
    
	uint32_t clock_rate = MT9M111_MCLK;

	pr_debug("In mt9m111:ioctl_dev_init\n");

	gpio_sensor_active();

	set_mclk_rate(&clock_rate, 0);                        // TODO: unhardcode '0' csi
	mt9m111_rate_cal(&reset_frame_rate, clock_rate);
	mt9m111_sensor_lib(mt9m111_device.coreReg, mt9m111_device.ifpReg);

    printk("::: ioctl_dev_init (END)\n");
    
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
/*	{vidioc_int_enum_fmt_cap_num,
				(v4l2_int_ioctl_func *) ioctl_enum_fmt_cap}, */

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
    printk("::: mt9m111_probe\n");
    
	int retval;

	pr_debug("In mt9m111_probe  device id is %s\n", id->name);

	/* Set initial values for the sensor struct. */
	memset(&mt9m111_data, 0, sizeof(mt9m111_data));
	mt9m111_data.i2c_client = client;
	pr_debug("   client name is %s\n", client->name);
	mt9m111_data.pix.pixelformat = V4L2_PIX_FMT_UYVY;
	mt9m111_data.pix.width = MT9M111_MAX_WIDTH;
	mt9m111_data.pix.height = MT9M111_MAX_HEIGHT;
	mt9m111_data.streamcap.capability = 0; /* No higher resolution or frame
						* frame rate changes supported.
						*/
	mt9m111_data.streamcap.timeperframe.denominator = MT9M111_FRAME_RATE;
	mt9m111_data.streamcap.timeperframe.numerator = 1;

	mt9m111_int_device.priv = &mt9m111_data;

	pr_debug("   type is %d (expect %d)\n",
		mt9m111_int_device.type, v4l2_int_type_slave);
	pr_debug("   num ioctls is %d\n",
		mt9m111_int_device.u.slave->num_ioctls);

	/* This function attaches this structure to the /dev/video0 device.
	 * The pointer in priv points to the mt9m111_data structure here.*/
	retval = v4l2_int_device_register(&mt9m111_int_device);

    printk("::: mt9m111_probe (END, ret=%d)\n", retval);
    
	return retval;
}

/*!
 * Function set in i2c_driver struct.
 * Called on rmmod mt9m111_camera.ko
 */
static int mt9m111_remove(struct i2c_client *client)
{
    printk("====:: mt9m111_remove\n");
    
	pr_debug("In mt9m111_remove\n");

	v4l2_int_device_unregister(&mt9m111_int_device);

    printk("====:: mt9m111_remove (END)\n");
    
	return 0;
}

/*!
 * MT9M111 init function.
 * Called by insmod mt9m111_camera.ko.
 *
 * @return  Error code indicating success or failure
 */
static __init int mt9m111_init(void)
{
    printk("::: mt9m111_init\n");
    
	u8 err;

	pr_debug("In mt9m111_init\n");

	/* Allocate memory for state structures. */
	mt9m111_device.coreReg = (mt9m111_coreReg *)
				kmalloc(sizeof(mt9m111_coreReg), GFP_KERNEL);
	if (!mt9m111_device.coreReg)
    {
        printk("::: mt9m111_init (END, error-0)\n");
		return -1;
    }
	memset(mt9m111_device.coreReg, 0, sizeof(mt9m111_coreReg));

	mt9m111_device.ifpReg = (mt9m111_IFPReg *)
				kmalloc(sizeof(mt9m111_IFPReg), GFP_KERNEL);
	if (!mt9m111_device.ifpReg) {
		kfree(mt9m111_device.coreReg);
		mt9m111_device.coreReg = NULL;
        printk("::: mt9m111_init (END, error-1)\n");
		return -1;
	}
	memset(mt9m111_device.ifpReg, 0, sizeof(mt9m111_IFPReg));

	/* Set contents of the just created structures. */
	mt9m111_config();

	/* Tells the i2c driver what functions to call for this driver. */
	err = i2c_add_driver(&mt9m111_i2c_driver);
	if (err != 0)
		pr_err("%s:driver registration failed, error=%d \n",
		       __func__, err);

    printk("::: mt9m111_init (END, ret=%d)\n", err);
    
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
    printk("::: mt9m111_clean\n");
    
	pr_debug("In mt9m111_clean()\n");

	i2c_del_driver(&mt9m111_i2c_driver);
	gpio_sensor_inactive();

	if (mt9m111_device.coreReg) {
		kfree(mt9m111_device.coreReg);
		mt9m111_device.coreReg = NULL;
	}

	if (mt9m111_device.ifpReg) {
		kfree(mt9m111_device.ifpReg);
		mt9m111_device.ifpReg = NULL;
	}

    printk("::: mt9m111_clean (END)\n");
}

module_init(mt9m111_init);
module_exit(mt9m111_clean);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Mt9m111 Camera Driver");
MODULE_LICENSE("GPL");
