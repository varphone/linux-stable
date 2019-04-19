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
 * @file tvp5158.c
 *
 * @brief Texas Instruments TVP5158A/AM1 video decoder driver
 *
 * @ingroup Decoder
 */

#include "mxc_v4l2_capture.h"
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fsl_devices.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-int-device.h>
#include <media/tvp5158.h>
#include "tvp5158_reg.h"

static struct regulator *dvddio_regulator;
static struct regulator *dvdd_regulator;
static struct regulator *avdd_regulator;
static struct regulator *pvdd_regulator;
static struct fsl_mxc_tvin_platform_data *tvin_plat;

extern void gpio_sensor_active(void);
extern void gpio_sensor_inactive(void);

static int tvp5158_probe(struct i2c_client *adapter,
			 const struct i2c_device_id *id);
static int tvp5158_detach(struct i2c_client *client);

static const struct i2c_device_id tvp5158_id[] = {
    {"tvp5158", 0}, {},
};

MODULE_DEVICE_TABLE(i2c, tvp5158_id);

static struct i2c_driver tvp5158_i2c_driver = {
    .driver =
	{
	    .owner = THIS_MODULE,
	    .name = "tvp5158",
	},
    .probe = tvp5158_probe,
    .remove = tvp5158_detach,
    .id_table = tvp5158_id,
};

/*!
 * Maintains the information on the current state of the sensor.
 */
struct tvp5158 {
	struct sensor_data sen;
	v4l2_std_id std_id;
	int agc;
	int agc_gain;
	int input;
};

static struct tvp5158 tvp5158_data;

/*! List of input video formats supported. The video formats is corresponding
 * with v4l2 id in video_fmt_t
 */
typedef enum {
	TVP5158_NTSC = 0,   /*!< Locked on (M) NTSC video signal. */
	TVP5158_PAL,	/*!< (B, G, H, I, N)PAL video signal. */
	TVP5158_PAL_M,      /*!< (B, G, H, I, N)PAL video signal. */
	TVP5158_PAL_60,     /*!< (B, G, H, I, N)PAL video signal. */
	TVP5158_NOT_LOCKED, /*!< Not locked on a signal. */
} video_fmt_idx;

/*! Number of video standards supported (including 'not locked' signal). */
#define TVP5158_STD_MAX (TVP5158_NOT_LOCKED)

/*! Video format structure. */
typedef struct {
	int v4l2_id;       /*!< Video for linux ID. */
	char name[16];     /*!< Name (e.g., "NTSC", "PAL", etc.) */
	u16 raw_width;     /*!< Raw width. */
	u16 raw_height;    /*!< Raw height. */
	u16 active_width;  /*!< Active width. */
	u16 active_height; /*!< Active height. */
} video_fmt_t;

/*! Description of video formats supported.
 *
 *  PAL: raw=720x625, active=720x576.
 *  NTSC: raw=720x525, active=720x480.
 */
static video_fmt_t video_fmts[] = {
    {
	/*! 0 NTSC */
	.v4l2_id = V4L2_STD_NTSC,
	.name = "NTSC",
	.raw_width = 720,     /* SENS_FRM_WIDTH */
	.raw_height = 525,    /* SENS_FRM_HEIGHT */
	.active_width = 720,  /* ACT_FRM_WIDTH plus 1 */
	.active_height = 480, /* ACT_FRM_WIDTH plus 1 */
    },
    {
	/*! 1 (B, G, H, I, N) PAL */
	.v4l2_id = V4L2_STD_PAL,
	.name = "PAL",
	.raw_width = 720,
	.raw_height = 625,
	.active_width = 720,
	.active_height = 576,
    },
    {
	/*! 2 (M) PAL */
	.v4l2_id = V4L2_STD_PAL_M,
	.name = "PAL-M",
	.raw_width = 720,     /* SENS_FRM_WIDTH */
	.raw_height = 525,    /* SENS_FRM_HEIGHT */
	.active_width = 720,  /* ACT_FRM_WIDTH plus 1 */
	.active_height = 480, /* ACT_FRM_WIDTH plus 1 */
    },
    {
	/*! 3 PAL 60 */
	.v4l2_id = V4L2_STD_PAL_60,
	.name = "PAL 60",
	.raw_width = 720,     /* SENS_FRM_WIDTH */
	.raw_height = 525,    /* SENS_FRM_HEIGHT */
	.active_width = 720,  /* ACT_FRM_WIDTH plus 1 */
	.active_height = 480, /* ACT_FRM_WIDTH plus 1 */
    },

    {
	/*! 4 Unlocked standard */
	.v4l2_id = V4L2_STD_ALL,
	.name = "Autodetect",
	.raw_width = 720,
	.raw_height = 625,
	.active_width = 720,
	.active_height = 576,
    },
};

/*!* Standard index of tvp5158. */
static video_fmt_idx video_idx = TVP5158_PAL;

/*! @brief This mutex is used to provide mutual exclusion.
 *
 *  Create a mutex that can be used to provide mutually exclusive
 *  read/write access to the globally accessible data structures
 *  and variables that were defined above.
 */
static DEFINE_MUTEX(mutex);

#define IF_NAME "tvp5158"

/* supported controls */
/* This hasn't been fully implemented yet.
 * This is how it should work, though. */
static struct v4l2_queryctrl tvp5158_qctrl[] = {
    {
	.id = V4L2_CID_BRIGHTNESS,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Brightness",
	.minimum = 0,	 /* check this value */
	.maximum = 255,       /* check this value */
	.step = 1,	    /* check this value */
	.default_value = 128, /* check this value */
	.flags = 0,
    },
    {
	.id = V4L2_CID_CONTRAST,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Contrast",
	.minimum = 0,	 /* check this value */
	.maximum = 255,       /* check this value */
	.step = 1,	    /* check this value */
	.default_value = 128, /* check this value */
	.flags = 0,
    },
    {
	.id = V4L2_CID_SATURATION,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Saturation",
	.minimum = 0,	 /* check this value */
	.maximum = 255,       /* check this value */
	.step = 1,	    /* check this value */
	.default_value = 128, /* check this value */
	.flags = 0,
    },
    {
	.id = V4L2_CID_HUE,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Hue",
	.minimum = -128,    /* check this value */
	.maximum = 127,     /* check this value */
	.step = 1,	  /* check this value */
	.default_value = 0, /* check this value */
	.flags = 0,
    },
    {
	.id = V4L2_CID_AUTOGAIN,
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.name = "AGC",
	.minimum = 0,
	.maximum = 1,
	.step = 1,
	.default_value = 1,
    },
    {
	.id = V4L2_CID_GAIN,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Gain",
	.minimum = 0,
	.maximum = 0x3fff,
	.step = 1,
	.default_value = 0x086A,
    },
    {
	.id = V4L2_CID_MXC_SWITCH_INPUT,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Switch input",
	.minimum = 0,
	.maximum = 3,
	.step = 1,
	.default_value = 0,
    },
    {
	.id = V4L2_CID_MXC_SEND_I2C_CMD,
	.type = V4L2_CTRL_TYPE_INTEGER,
	.name = "Send I2C cmd",
	.minimum = 0,
	.maximum = 65536,
	.step = 1,
	.default_value = 0,
    },
    {
	.id = V4L2_CID_MXC_VIDEO_SIGNAL,
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.name = "Video Signal",
	.minimum = 0,
	.maximum = 1,
	.step = 1,
	.default_value = 1,
    }};


/***********************************************************************
 * I2C transfert.
 ***********************************************************************/

/*! Read one register from a tvp5158 i2c slave device.
 *
 *  @param *reg		register in the device we wish to access.
 *
 *  @return		       0 if success, an error code otherwise.
 */

static inline int tvp5158_read(u8 addr)
{
	struct i2c_client *c = tvp5158_data.sen.i2c_client;
	u8 buffer[1];
	int rc;

	buffer[0] = addr;
	if (1 != (rc = i2c_master_send(c, buffer, 1))) {

		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"%s:read reg error: addr=%2x\n", __func__, addr);
		return -1;
	}

	msleep(10);

	if (1 != (rc = i2c_master_recv(c, buffer, 1))) {

		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"%s:read reg error: addr=%2x\n", __func__, addr);
		return -1;
	}

	return (buffer[0]);
}

/*! Read n data from the registers of tvp5158 i2c slave device.
 *
 *  @param *addr	register in the device we wish to access.
 *
 *  @param *num		number of data wish to access.
 *
 *  @param *buf		buffer to contain the data.
 *
 *  @return		    0 if success, -1 otherwise.
 */

static inline int tvp5158_read_bytes(u8 addr, u8 num, u8 *buf)
{
	struct i2c_client *c = tvp5158_data.sen.i2c_client;
	u8 buffer[1];
	int rc;

	buffer[0] = addr;
	if (1 != (rc = i2c_master_send(c, buffer, 1))) {

		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"%s:read reg error: addr=%2x\n", __func__, addr);
		return -1;
	}

	msleep(10);

	if (num != (rc = i2c_master_recv(c, buf, num))) {

		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"%s:read reg error: addr=%2x\n", __func__, addr);
		return -1;
	}

	return 0;
}

/*! Write one register of a tvp5158 i2c slave device.
 *
 *  @param *reg		register in the device we wish to access.
 *
 *  @return		       0 if success, an error code otherwise.
 */
static inline int tvp5158_write_reg(u8 addr, u8 value)
{
	struct i2c_client *c = tvp5158_data.sen.i2c_client;
	unsigned char buffer[2];
	int rc;

	buffer[0] = addr;
	buffer[1] = value;
	if (2 != (rc = i2c_master_send(c, buffer, 2))) {
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"%s:write reg error:addr=%2x,value=%2x\n", __func__,
			addr, value);
		return -1;
	}

	return 0;
}

static void tvp5158_dump_reg_range(int from, int to)
{

	int i;
	int tmp;

	if ((from > to) || (from < 0x00) || (to > 0xff)) {
		return;
	}

	for (i = from; i <= to; i++) {
		tmp = tvp5158_read(i);
		if (tmp < 0) {
			printk("reg[0x%02x]:read error   ", i);
		} else {
			printk("reg[0x%02x]:0x%02x   ", i, tmp);
		}
		if ((3 == (i % 4))) {
			printk("\n");
		}
	}
}

static int tvp5158_ofm_channel_select1(int chan)
{
	int tmp = 0x00;

	printk(KERN_INFO "tvp5158: switch to camera: %d\n", chan);

	switch (chan) {
	case 0:
		tmp = 0xE4;
		break;
	case 1:
		tmp = 0xE1;
		break;
	case 2:
		tmp = 0xC6;
		break;
	case 3:
		tmp = 0x27;
		break;
	}

	tvp5158_write_reg(TVP5158_OFM_CHL_SEL_1, tmp);

	return 0;
}

static int tvp5158_chip_init(void)
{
	int tmp;

	/* Clear lost lock detect */
	tvp5158_write_reg(TVP5158_CLEAR_LOCK_DETECT, 0x01);

	tvp5158_ofm_channel_select1(tvp5158_data.input);

	/* Defaut video standard selection is: autoswitch
	 * 0x02 for PAL; 0x00 for autoswitch
	 */
	tmp = tvp5158_read(TVP5158_VIDEO_STD_SEL);
	tmp = 0x02;
	tvp5158_write_reg(TVP5158_VIDEO_STD_SEL, tmp);

	/* Enable pix clk & video port
	 * use this register to enable the  pix clk & video port
	 */
	tmp = tvp5158_read(TVP5158_OFM_MODE_CTL);
	tmp |= 0x05;
	tvp5158_write_reg(TVP5158_OFM_MODE_CTL, tmp);

	/* make Decoder 0&1 Readable
	 * use this register to make Decoder core 0&1 Readable
	 */
	tmp = 0x13;
	tvp5158_write_reg(TVP5158_DECODER_READ_ENABLE, tmp);

	/* to improve the image quality
	 * setp1: enable the noise reduction module
	 */
	tmp = tvp5158_read(TVP5158_NR_CTL);
	tmp &= 0xFE;
	tvp5158_write_reg(TVP5158_NR_CTL, tmp);

	tmp = tvp5158_read(TVP5158_CHROMA_PROC_CTL_2);
	tmp &= 0xFC;
	tmp |= 0x03;
	tvp5158_write_reg(TVP5158_CHROMA_PROC_CTL_2, tmp);

	/* luma comb/trap filter bypassed */
	tmp = tvp5158_read(TVP5158_LUMA_PROC_CTL_2);
	tmp &= 0xCF;
	tmp |= 0x80;
	tvp5158_write_reg(TVP5158_LUMA_PROC_CTL_2, tmp);

	/* decrease the Chrominance Saturation */
	tmp = tvp5158_read(TVP5158_CHROMA_SATURATION);
	tmp = 0x70; // default is 0x80
	tvp5158_write_reg(TVP5158_CHROMA_SATURATION, tmp);

	/* Setup blue screen */
	//tvp5158_write_reg(TVP5158_DECODER_WRITE_ENABLE, 0x0f);
	tmp = tvp5158_read(TVP5158_OUT_FMT_CTL_2);
	tvp5158_write_reg(TVP5158_OUT_FMT_CTL_2, tmp | 0x04);
	tvp5158_write_reg(TVP5158_BLUE_SCREEN_Y_CTL, 0x10);
	tvp5158_write_reg(TVP5158_BLUE_SCREEN_CB_CTL, 0x80);
	tvp5158_write_reg(TVP5158_BLUE_SCREEN_CR_CTL, 0x80);
	tvp5158_write_reg(TVP5158_BLUE_SCREEN_LSB_CTL, 0x00);
	
	return 0;
}

/***********************************************************************
 * mxc_v4l2_capture interface.
 ***********************************************************************/

/*!
 * Return attributes of current video standard.
 * Since this device autodetects the current standard, this function also
 * sets the values that need to be changed if the standard changes.
 * There is no set std equivalent function.
 *
 *  @return		None.
 */
static int tvp5158_get_std(v4l2_std_id *std)
{
	int err = 0;
	int tmp = 0;
	u8 autoswitch = 0;
	u8 video_std = 0;
	u8 video_std_tmp = 0;
	int idx = 0;
	u8 buf[4];

	dev_dbg(&tvp5158_data.sen.i2c_client->dev, "In tvp5158_get_std\n");

	/*check weather the video stream present or not*/
	if (tvp5158_read_bytes(TVP5158_STATUS_REG_2, 2, buf) < 0) {
		video_std = 0;
		err = -EIO;
		goto next;
	} else {
		if (!((buf[0] & 0x80) || (buf[1] & 0x80))) {
			err = -EAGAIN;
		}
	}

	/* Read the AD_RESULT to get the autodetected output video standard */
	if ((tmp = tvp5158_read(TVP5158_VIDEO_STD_STATUS)) < 0) {
		video_std = 0;
		err = -EIO;
		goto next;
	} else {
		autoswitch = tmp & 0x80;
	}

	if (0 != autoswitch) {
		video_std = tmp & 0x07;
	} else {
		if ((tmp = tvp5158_read(TVP5158_VIDEO_STD_SEL)) < 0) {
			video_std = 0;
			err = -EIO;
			goto next;
		} else {
			video_std = tmp & 0x07;
		}
	}

next:
	mutex_lock(&mutex);
	if (0x01 == video_std) {
		/* (M, J) NTSC: 3.58MHz subcarrier, 525 lines, 60 fields NTSC*/
		*std = V4L2_STD_NTSC;
		idx = TVP5158_NTSC;
	} else if (0x02 == video_std) {
		/* (B, D, G, H, I, N) PAL: 4.43MHz subcarrier, 625 lines, 50
		 * feilds PAL*/
		*std = V4L2_STD_PAL;
		idx = TVP5158_PAL;
	} else if (0x03 == video_std) {
		/*(M) PAL: 3.58MHz subcarier, 525 lines, 60 filed PAL(only used
		 * in Brazil) */
		*std = V4L2_STD_PAL_M;
		idx = TVP5158_PAL_M;
	} else if (0x04 == video_std) {
		/*(Combination-N) PAL: ???*/
		*std = V4L2_STD_PAL_Nc;
		idx = TVP5158_PAL;
	} else if (0x05 == video_std) {
		/* NTSC 4.43: 4.43MHz subcarrier, 525 lines, 60 fields NTSC */
		*std = V4L2_STD_NTSC_443;
		idx = TVP5158_NTSC;
	} else if (0x07 == video_std) {
		/*PAL 60: 4.43MHz subcarrier, 525 lines, 60 fields PAL */
		*std = V4L2_STD_PAL_60;
		idx = TVP5158_PAL_60;
	} else {
		/* Reserved */
		*std = V4L2_STD_ALL;
		idx = TVP5158_NOT_LOCKED;
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"Got invalid video standard!\n");
	}
	mutex_unlock(&mutex);

	/* Assumes autodetect is on. */
	if (*std != tvp5158_data.std_id) {
		video_idx = idx;
		tvp5158_data.std_id = *std;
		tvp5158_data.sen.pix.width = video_fmts[video_idx].raw_width;
		tvp5158_data.sen.pix.height = video_fmts[video_idx].raw_height;
	}

	return err;
}

/***********************************************************************
 * IOCTL Functions from v4l2_int_ioctl_desc.
 ***********************************************************************/

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
 * Called on open.
 */
static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	dev_dbg(&tvp5158_data.sen.i2c_client->dev, "tvp5158:ioctl_g_ifparm\n");

	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	/* Initialize structure to 0s then set any non-0 values. */
	memset(p, 0, sizeof(*p));
	p->if_type = V4L2_IF_TYPE_BT656; /* This is the only possibility. */
	// BandWidth = 8 bit
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
	// Horizontal Synchronizing signal inverse ?
	p->u.bt656.nobt_hs_inv = 1;
	// Use Bt synchronisation codes for sync correction
	p->u.bt656.bt_sync_correct = 1;

	/* tvp5158 has a dedicated clock so no clock settings needed. */

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
 * This is called on open, close, suspend and resume.
 */
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct tvp5158 *sensor = s->priv;

	int tmp;

	dev_dbg(&tvp5158_data.sen.i2c_client->dev, "tvp5158:ioctl_s_power\n");

	if (on && !sensor->sen.on) {
		gpio_sensor_active();

		tmp = 0x00;
		if (tvp5158_write_reg(TVP5158_PWR_CTL, tmp) < 0) {
		}

		/*
		 * FIXME:Additional 400ms to wait the chip to be stable?
		 * This is a workaround for preview scrolling issue.
		 */
	} else if (!on && sensor->sen.on) {
		gpio_sensor_inactive();

		// turn this off make the I2C communication lost,to restart you
		// have to set the CVBS__RST(GPIO_1_15)
		tmp = 0xF1;
		if (tvp5158_write_reg(TVP5158_PWR_CTL, tmp) < 0) {
		}
	}

	sensor->sen.on = on;

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
	struct tvp5158 *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;

	dev_dbg(&tvp5158_data.sen.i2c_client->dev, "In tvp5158:ioctl_g_parm\n");

	switch (a->type) {
	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n");
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->sen.streamcap.capability;
		cparm->timeperframe = sensor->sen.streamcap.timeperframe;
		cparm->capturemode = sensor->sen.streamcap.capturemode;
		break;

	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		break;

	default:
		pr_debug("ioctl_g_parm:type is unknown %d\n", a->type);
		break;
	}

	return 0;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 *
 * This driver cannot change these settings.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	dev_dbg(&tvp5158_data.sen.i2c_client->dev, "In tvp5158:ioctl_s_parm\n");

	switch (a->type) {
	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		break;
	}

	return 0;
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
	struct tvp5158 *sensor = s->priv;
	int err;

	dev_dbg(&tvp5158_data.sen.i2c_client->dev, "tvp5158:ioctl_g_fmt_cap\n");

	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   Returning size of %dx%d\n", sensor->sen.pix.width,
			 sensor->sen.pix.height);
		f->fmt.pix = sensor->sen.pix;
		break;

	case V4L2_BUF_TYPE_PRIVATE: {
		v4l2_std_id std;
		if ((err = tvp5158_get_std(&std)) < 0) {
			return err;
		}
		f->fmt.pix.pixelformat = (u32)std;
	} break;

	default:
		f->fmt.pix = sensor->sen.pix;
		break;
	}

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
	int i;

	dev_dbg(&tvp5158_data.sen.i2c_client->dev, "tvp5158:ioctl_queryctrl\n");

	for (i = 0; i < ARRAY_SIZE(tvp5158_qctrl); i++)
		if (qc->id && qc->id == tvp5158_qctrl[i].id) {
			memcpy(qc, &(tvp5158_qctrl[i]), sizeof(*qc));
			return 0;
		}

	return -EINVAL;
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
	int ret = 0;
	int gain = 0;
	int tmp = 0;
	u8 buf[4];

	dev_dbg(&tvp5158_data.sen.i2c_client->dev, "In tvp5158:ioctl_g_ctrl\n");

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"   V4L2_CID_BRIGHTNESS\n");
		if ((tmp = tvp5158_read(TVP5158_LUMA_BRIGHT)) < 0) {
			ret = -EIO;
		} else {
			tvp5158_data.sen.brightness = tmp;
			vc->value = tvp5158_data.sen.brightness;
		}
		break;
	case V4L2_CID_CONTRAST:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"   V4L2_CID_CONTRAST\n");
		if ((tmp = tvp5158_read(TVP5158_LUMA_CONTRAST)) < 0) {
			ret = -EIO;
		} else {
			tvp5158_data.sen.contrast = tmp;
			vc->value = tvp5158_data.sen.contrast;
		}
		break;
	case V4L2_CID_SATURATION:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"   V4L2_CID_SATURATION\n");
		if ((tmp = tvp5158_read(TVP5158_CHROMA_SATURATION)) < 0) {
			ret = -EIO;
		} else {
			tvp5158_data.sen.saturation = tmp;
			vc->value = tvp5158_data.sen.saturation;
		}
		break;
	case V4L2_CID_HUE:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev, "   V4L2_CID_HUE\n");
		if ((tmp = tvp5158_read(TVP5158_CHROMA_HUE)) < 0) {
			ret = -EIO;
		} else {
			tvp5158_data.sen.hue = tmp;
			vc->value = tvp5158_data.sen.hue;
		}
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"   V4L2_CID_AUTO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"   V4L2_CID_DO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_RED_BALANCE:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"   V4L2_CID_RED_BALANCE\n");
		//		vc->value = tvp5158_data.sen.red;
		break;
	case V4L2_CID_BLUE_BALANCE:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"   V4L2_CID_BLUE_BALANCE\n");
		//		vc->value = tvp5158_data.sen.blue;
		break;
	case V4L2_CID_GAMMA:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"   V4L2_CID_GAMMA\n");
		break;
	case V4L2_CID_EXPOSURE:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"   V4L2_CID_EXPOSURE\n");
		//		vc->value = tvp5158_data.sen.ae_mode;
		break;
	case V4L2_CID_AUTOGAIN:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"   V4L2_CID_AUTOGAIN\n");
		if ((tmp = tvp5158_read(TVP5158_AFE_GAIN_CONTROL)) < 0) {
			ret = -EIO;
		} else {
			tvp5158_data.agc = (tmp & 0x01);
			vc->value = tvp5158_data.agc;
		}
		break;
	case V4L2_CID_GAIN:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"   V4L2_CID_GAIN\n");
		if (1 == tvp5158_data.agc) {
			if ((tmp = tvp5158_read(TVP5158_AGC_GAIN_STATUS_LSB)) <
			    0) {
				ret = -EIO;
			} else {
				gain = tmp;
				if ((tmp = tvp5158_read(
					 TVP5158_AGC_GAIN_STATUS_MSB)) < 0) {
					ret = -EIO;
				} else {
					gain |= (tmp & 0x3F) << 0x08;
				}
			}
		} else {
			if ((tmp = tvp5158_read(TVP5158_AFE_FINE_GAIN_LSB)) <
			    0) {
				ret = -EIO;
			} else {
				gain = tmp;
				if ((tmp = tvp5158_read(
					 TVP5158_AFE_FINE_GAIN_MSB)) < 0) {
					ret = -EIO;
				} else {
					gain |= (tmp & 0x3F) << 0x08;
				}
			}
		}
		if (0 == ret) {
			tvp5158_data.agc_gain = gain;
			vc->value = tvp5158_data.agc_gain;
		}
		break;
	case V4L2_CID_HFLIP:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"   V4L2_CID_HFLIP\n");
		break;
	case V4L2_CID_VFLIP:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"   V4L2_CID_VFLIP\n");
		break;
	case V4L2_CID_MXC_SWITCH_INPUT:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"   V4L2_CID_MXC_SWITCH_INPUT\n");

		if ((tmp = tvp5158_read(TVP5158_OFM_CHL_SEL_1)) < 0) {
			ret = -EIO;
		} else {
			tvp5158_data.input = tmp & 0x03;
			vc->value = tvp5158_data.input;
		}
		break;

	case V4L2_CID_MXC_SEND_I2C_CMD:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"   V4L2_CID_MXC_SEND_I2C_CMD\n");

		if ((tmp = tvp5158_read((vc->value) & 0xFF)) < 0) {
			ret = -EIO;
		} else {
			vc->value = tmp & 0xFF;
		}
		break;

	case V4L2_CID_MXC_VIDEO_SIGNAL:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"   V4L2_CID_MXC_VIDEO_SIGNAL\n");

		tmp = tvp5158_data.input;

		/* check whether the Signal present */
		if (tvp5158_read_bytes(TVP5158_STATUS_REG_2, 4, buf) < 0) {
			ret = -EIO;
			break;
		} else {
			vc->value = !!(buf[tmp] & 0x80);
		}
		break;

	default:
		printk(KERN_WARNING "V4L2_G_CTRL: %d unsupported.\n", vc->id);
		vc->value = 0;
		ret = -EPERM;
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
	int retval = 0;
	u8 tmp;
	u8 buf[4];

	dev_dbg(&tvp5158_data.sen.i2c_client->dev, "In tvp5158:ioctl_s_ctrl\n");

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"   V4L2_CID_BRIGHTNESS\n");
		tmp = vc->value;
		if (tvp5158_write_reg(TVP5158_LUMA_BRIGHT, tmp) < 0) {
			retval = -EIO;
		} else {
			tvp5158_data.sen.brightness = vc->value;
		}
		break;
	case V4L2_CID_CONTRAST:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"   V4L2_CID_CONTRAST\n");
		tmp = vc->value;
		if (tvp5158_write_reg(TVP5158_LUMA_CONTRAST, tmp) < 0) {
			retval = -EIO;
		} else {
			tvp5158_data.sen.contrast = vc->value;
		}
		break;
	case V4L2_CID_SATURATION:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"   V4L2_CID_SATURATION\n");
		tmp = vc->value;
		if (tvp5158_write_reg(TVP5158_CHROMA_SATURATION, tmp) < 0) {
			retval = -EIO;
		} else {
			tvp5158_data.sen.saturation = vc->value;
		}
		break;
	case V4L2_CID_HUE:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev, "   V4L2_CID_HUE\n");
		tmp = vc->value;
		if (tvp5158_write_reg(TVP5158_CHROMA_HUE, tmp) < 0) {
			retval = -EIO;
		} else {
			tvp5158_data.sen.hue = vc->value;
		}
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"   V4L2_CID_AUTO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"   V4L2_CID_DO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_RED_BALANCE:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"   V4L2_CID_RED_BALANCE\n");
		break;
	case V4L2_CID_BLUE_BALANCE:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"   V4L2_CID_BLUE_BALANCE\n");
		break;
	case V4L2_CID_GAMMA:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"   V4L2_CID_GAMMA\n");
		break;
	case V4L2_CID_EXPOSURE:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"   V4L2_CID_EXPOSURE\n");
		break;
	case V4L2_CID_AUTOGAIN:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"   V4L2_CID_AUTOGAIN\n");

		if ((tmp = tvp5158_read(TVP5158_AFE_GAIN_CONTROL)) < 0) {
			retval = -EIO;
			break;
		}

		if (0 != vc->value) {
			tmp = (tvp5158_read(TVP5158_AFE_GAIN_CONTROL) & 0xFE) |
			      0x01;
		} else {
			tmp = (tvp5158_read(TVP5158_AFE_GAIN_CONTROL) & 0xFE) |
			      0x00;
		}

		if (tvp5158_write_reg(TVP5158_AFE_GAIN_CONTROL, tmp) < 0) {
			retval = -EIO;
		} else {
			tvp5158_data.agc = vc->value;
		}

		break;
	case V4L2_CID_GAIN:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"   V4L2_CID_GAIN\n");
		tmp = vc->value;
		if (tvp5158_write_reg(TVP5158_AFE_FINE_GAIN_LSB, tmp & 0xFF) <
		    0) {
			retval = -EIO;
		} else if (tvp5158_write_reg(TVP5158_AFE_FINE_GAIN_MSB,
					     (tmp >> 0x08) & 0x3F) < 0) {
			retval = -EIO;
		} else {
			tvp5158_data.agc_gain = vc->value;
		}
		break;
	case V4L2_CID_HFLIP:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"   V4L2_CID_HFLIP\n");
		break;
	case V4L2_CID_VFLIP:
		dev_dbg(&tvp5158_data.sen.i2c_client->dev,
			"   V4L2_CID_VFLIP\n");
		break;
	case V4L2_CID_MXC_SWITCH_INPUT:
		tvp5158_data.input = vc->value;
		tvp5158_ofm_channel_select1(tvp5158_data.input);
		break;

	case V4L2_CID_MXC_SEND_I2C_CMD:
		if (tvp5158_write_reg(((vc->value) >> 8) & 0xFF,
				      (vc->value) & 0xFF) < 0) {
			retval = -EIO;
		}
		break;

	default:
		printk(KERN_WARNING "V4L2_S_CTRL: %d unsupported.\n", vc->id);
		retval = -EPERM;
		break;
	}

	return retval;
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
	if (fsize->index >= 1)
		return -EINVAL;

	fsize->discrete.width = video_fmts[video_idx].active_width;
	fsize->discrete.height = video_fmts[video_idx].active_height;

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
	       "tvp5158_decoder");
	((struct v4l2_dbg_chip_ident *)id)->ident = V4L2_IDENT_TVP5158;

	return 0;
}

/*!
 * ioctl_init - V4L2 sensor interface handler for VIDIOC_INT_INIT
 * @s: pointer to standard V4L2 device structure
 */
static int ioctl_init(struct v4l2_int_device *s)
{
	dev_dbg(&tvp5158_data.sen.i2c_client->dev, "In tvp5158:ioctl_init\n");
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
	return tvp5158_chip_init();
}

/*!
 * This structure defines all the ioctls for this module.
 */
static struct v4l2_int_ioctl_desc tvp5158_ioctl_desc[] =
    {

	{vidioc_int_dev_init_num, (v4l2_int_ioctl_func *)ioctl_dev_init},

	/*!
	 * Delinitialise the dev. at slave detach.
	 * The complement of ioctl_dev_init.
	 */
	/*	{vidioc_int_dev_exit_num, (v4l2_int_ioctl_func
	   *)ioctl_dev_exit}, */

	{vidioc_int_s_power_num, (v4l2_int_ioctl_func *)ioctl_s_power},
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func *)ioctl_g_ifparm},
	/*	{vidioc_int_g_needs_reset_num,
					(v4l2_int_ioctl_func
	   *)ioctl_g_needs_reset}, */
	/*	{vidioc_int_reset_num, (v4l2_int_ioctl_func *)ioctl_reset}, */
	{vidioc_int_init_num, (v4l2_int_ioctl_func *)ioctl_init},

	/*!
	 * VIDIOC_ENUM_FMT ioctl for the CAPTURE buffer type.
	 */
	/*	{vidioc_int_enum_fmt_cap_num,
					(v4l2_int_ioctl_func
	   *)ioctl_enum_fmt_cap}, */

	/*!
	 * VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.
	 * This ioctl is used to negotiate the image capture size and
	 * pixel format without actually making it take effect.
	 */
	/*	{vidioc_int_try_fmt_cap_num,
					(v4l2_int_ioctl_func
	   *)ioctl_try_fmt_cap}, */

	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_g_fmt_cap},

	/*!
	 * If the requested format is supported, configures the HW to use that
	 * format, returns error code if format not supported or HW can't be
	 * correctly configured.
	 */
	/*	{vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func
	   *)ioctl_s_fmt_cap}, */

	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func *)ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func *)ioctl_s_parm},
	{vidioc_int_queryctrl_num, (v4l2_int_ioctl_func *)ioctl_queryctrl},
	{vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *)ioctl_g_ctrl},
	{vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *)ioctl_s_ctrl},
	{vidioc_int_enum_framesizes_num,
	 (v4l2_int_ioctl_func *)ioctl_enum_framesizes},
	{vidioc_int_g_chip_ident_num,
	 (v4l2_int_ioctl_func *)ioctl_g_chip_ident},
};

static struct v4l2_int_slave tvp5158_slave = {
    .ioctls = tvp5158_ioctl_desc, .num_ioctls = ARRAY_SIZE(tvp5158_ioctl_desc),
};

static struct v4l2_int_device tvp5158_int_device = {
    .module = THIS_MODULE,
    .name = "tvp5158",
    .type = v4l2_int_type_slave,
    .u =
	{
	    .slave = &tvp5158_slave,
	},
};

/*! tvp5158 I2C attach function.
 *
 *  @param *adapter	struct i2c_adapter *.
 *
 *  @return		Error code indicating success or failure.
 */

/*!
 * tvp5158 I2C probe function.
 * Function set in i2c_driver struct.
 * Called by insmod.
 *
 *  @param *adapter	I2C adapter descriptor.
 *
 *  @return		Error code indicating success or failure.
 */
static int tvp5158_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int tmp;
	int ret = 0;
	int chip_id_msb, chip_id_lsb, rom_ver, ram_ver_msb, ram_ver_lsb;
	tvin_plat = client->dev.platform_data;

	if (tvin_plat->dvddio_reg) {
		dvddio_regulator =
		    regulator_get(&client->dev, tvin_plat->dvddio_reg);
		if (!IS_ERR_VALUE((unsigned long)dvddio_regulator)) {
			regulator_set_voltage(dvddio_regulator, 3300000,
					      3300000);
			if (regulator_enable(dvddio_regulator) != 0)
				return -ENODEV;
		}
	}

	if (tvin_plat->dvdd_reg) {
		dvdd_regulator =
		    regulator_get(&client->dev, tvin_plat->dvdd_reg);
		if (!IS_ERR_VALUE((unsigned long)dvdd_regulator)) {
			regulator_set_voltage(dvdd_regulator, 1800000, 1800000);
			if (regulator_enable(dvdd_regulator) != 0)
				return -ENODEV;
		}
	}

	if (tvin_plat->avdd_reg) {
		avdd_regulator =
		    regulator_get(&client->dev, tvin_plat->avdd_reg);
		if (!IS_ERR_VALUE((unsigned long)avdd_regulator)) {
			regulator_set_voltage(avdd_regulator, 1800000, 1800000);
			if (regulator_enable(avdd_regulator) != 0)
				return -ENODEV;
		}
	}

	if (tvin_plat->pvdd_reg) {
		pvdd_regulator =
		    regulator_get(&client->dev, tvin_plat->pvdd_reg);
		if (!IS_ERR_VALUE((unsigned long)pvdd_regulator)) {
			regulator_set_voltage(pvdd_regulator, 1800000, 1800000);
			if (regulator_enable(pvdd_regulator) != 0)
				return -ENODEV;
		}
	}

	if (tvin_plat->io_init)
		tvin_plat->io_init();

	if (tvin_plat->reset)
		tvin_plat->reset();

	if (tvin_plat->pwdn)
		tvin_plat->pwdn(0);

	msleep(1);

	/* Set initial values for the sensor struct. */
	memset(&tvp5158_data, 0, sizeof(tvp5158_data));
	tvp5158_data.sen.i2c_client = client;
	tvp5158_data.sen.streamcap.timeperframe.denominator = 30;
	tvp5158_data.sen.streamcap.timeperframe.numerator = 1;
	tvp5158_data.input = TVP5158_COMPOSITE0;
	tvp5158_data.std_id = V4L2_STD_ALL;
	video_idx = TVP5158_NOT_LOCKED;
	tvp5158_data.sen.pix.width = video_fmts[video_idx].raw_width;
	tvp5158_data.sen.pix.height = video_fmts[video_idx].raw_height;
	tvp5158_data.sen.pix.pixelformat = V4L2_PIX_FMT_UYVY; /* YUV422 */
	tvp5158_data.sen.pix.priv = 1; /* 1 is used to indicate TV in */
	tvp5158_data.sen.on = true;

	gpio_sensor_active();

	dev_dbg(&tvp5158_data.sen.i2c_client->dev,
		"%s:tvp5158 probe i2c address is 0x%02X\n", __func__,
		tvp5158_data.sen.i2c_client->addr);

	/*! Read the revision ID of the tvin chip */
	// 2015-05-28
	chip_id_msb = tvp5158_read(TVP5158_CHIP_ID_MSB);
	chip_id_lsb = tvp5158_read(TVP5158_CHIP_ID_LSB);
	rom_ver = tvp5158_read(TVP5158_ROM_VER);
	ram_ver_msb = tvp5158_read(TVP5158_RAM_VER_MSB);
	ram_ver_lsb = tvp5158_read(TVP5158_RAM_VER_LSB);

	if (chip_id_msb >= 0 || chip_id_lsb >= 0 || rom_ver >= 0 ||
	    ram_ver_msb >= 0 || ram_ver_lsb >= 0) {
		printk("%s:tvp%2X%2X detected!, Firmware Version is "
		       "v%2X.%2X.%2X\n",
		       __func__, chip_id_msb, chip_id_lsb, rom_ver, ram_ver_msb,
		       ram_ver_lsb);
	} else {
		printk("%s:Couldn't read infomation from the chip\n", __func__);
	}

	pr_debug("   type is %d (expect %d)\n", tvp5158_int_device.type,
		 v4l2_int_type_slave);
	pr_debug("   num ioctls is %d\n",
		 tvp5158_int_device.u.slave->num_ioctls);

	/* This function attaches this structure to the /dev/video0 device.
	 * The pointer in priv points to the mt9v111_data structure here.*/
	tvp5158_int_device.priv = &tvp5158_data;
	ret = v4l2_int_device_register(&tvp5158_int_device);

	return ret;
}

/*!
 * tvp5158 I2C detach function.
 * Called on rmmod.
 *
 *  @param *client	struct i2c_client*.
 *
 *  @return		Error code indicating success or failure.
 */
static int tvp5158_detach(struct i2c_client *client)
{
	dev_dbg(&tvp5158_data.sen.i2c_client->dev,
		"%s:Removing %s video decoder @ 0x%02X from adapter %s\n",
		__func__, IF_NAME, client->addr << 1, client->adapter->name);

	if (dvddio_regulator) {
		regulator_disable(dvddio_regulator);
		regulator_put(dvddio_regulator);
	}

	if (dvdd_regulator) {
		regulator_disable(dvdd_regulator);
		regulator_put(dvdd_regulator);
	}

	if (avdd_regulator) {
		regulator_disable(avdd_regulator);
		regulator_put(avdd_regulator);
	}

	if (pvdd_regulator) {
		regulator_disable(pvdd_regulator);
		regulator_put(pvdd_regulator);
	}

	v4l2_int_device_unregister(&tvp5158_int_device);

	return 0;
}

/*!
 * tvp5158 init function.
 * Called on insmod.
 *
 * @return    Error code indicating success or failure.
 */
static __init int tvp5158_init(void)
{
	u8 err = 0;

	/* Tells the i2c driver what functions to call for this driver. */
	err = i2c_add_driver(&tvp5158_i2c_driver);
	if (err != 0)
		pr_err("%s:driver registration failed, error=%d \n", __func__,
		       err);

	return err;
}

/*!
 * tvp5158 cleanup function.
 * Called on rmmod.
 *
 * @return   Error code indicating success or failure.
 */
static void __exit tvp5158_clean(void)
{
	dev_dbg(&tvp5158_data.sen.i2c_client->dev, "In tvp5158_clean\n");
	i2c_del_driver(&tvp5158_i2c_driver);
	gpio_sensor_inactive();
}

module_init(tvp5158_init);
module_exit(tvp5158_clean);

MODULE_AUTHOR("CETC55");
MODULE_DESCRIPTION("Texas Instruments TVP5150A video decoder driver");
MODULE_LICENSE("GPL");
