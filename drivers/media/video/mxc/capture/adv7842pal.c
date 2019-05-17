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
 * @file adv7842pal.c
 *
 * @brief adv7842pal camera driver functions
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

static struct regulator *dvddio_regulator;
static struct regulator *dvdd_regulator;
static struct regulator *avdd_regulator;
static struct regulator *pvdd_regulator;
static struct fsl_mxc_tvin_platform_data *tvin_plat;

/*!
 * Holds the current frame rate.
 */

struct sensor {
	struct sensor_data sen;
	v4l2_std_id std_id;
	struct i2c_client *i2c_sdp; //0x90
	struct i2c_client *i2c_sdp_io; //0x94
	struct i2c_client *i2c_avlink; //0x84
	struct i2c_client *i2c_cp; //0x44
	struct i2c_client *i2c_vdp; //0x48
	struct i2c_client *i2c_afe; //0x4c
	struct i2c_client *i2c_hdmi; //0x68
	struct i2c_client *i2c_ksv; //0x64
	struct i2c_client *i2c_edid; //0x6c
	struct i2c_client *i2c_infoframe; //0x7c
	struct i2c_client *i2c_cec; //0x80
} adv7842_pal;

/*! List of input video formats supported. The video formats is corresponding
 * with v4l2 id in video_fmt_t
 */
typedef enum {
	ADV7842_NTSC = 0, /*!< Locked on (M) NTSC video signal. */
	ADV7842_PAL, /*!< (B, G, H, I, N)PAL video signal. */
	ADV7842_NOT_LOCKED, /*!< Not locked on a signal. */
} video_fmt_idx;

/*! Number of video standards supported (including 'not locked' signal). */
#define ADV7842_STD_MAX (ADV7842_PAL + 1)

/*! Video format structure. */
typedef struct {
	int v4l2_id; /*!< Video for linux ID. */
	char name[16]; /*!< Name (e.g., "NTSC", "PAL", etc.) */
	u16 raw_width; /*!< Raw width. */
	u16 raw_height; /*!< Raw height. */
	u16 active_width; /*!< Active width. */
	u16 active_height; /*!< Active height. */
} video_fmt_t;

/*! Description of video formats supported.
 *
 *  PAL: raw=720x625, active=720x576.
 *  NTSC: raw=720x525, active=720x480.
 */
static video_fmt_t video_fmts[] = {
	{
		/*! (B, G, H, I, N) PAL */
		.v4l2_id = V4L2_STD_PAL,
		.name = "PAL",
		.raw_width = 720,
		.raw_height = 625,
		.active_width = 720,
		.active_height = 576,
	},
	{
		/*! NTSC */
		.v4l2_id = V4L2_STD_NTSC,
		.name = "NTSC",
		.raw_width = 720, /* SENS_FRM_WIDTH */
		.raw_height = 525, /* SENS_FRM_HEIGHT */
		.active_width = 720, /* ACT_FRM_WIDTH plus 1 */
		.active_height = 480, /* ACT_FRM_WIDTH plus 1 */
	},
	{
		/*! Unlocked standard */
		.v4l2_id = V4L2_STD_ALL,
		.name = "Autodetect",
		.raw_width = 720,
		.raw_height = 625,
		.active_width = 720,
		.active_height = 576,
	},
};

/*!* Standard index of ADV7180. */
static video_fmt_idx video_idx = ADV7842_PAL;

extern void gpio_sensor_active(void);
extern void gpio_sensor_inactive(void);

static DEFINE_MUTEX(mutex);

#define IF_NAME "adv7842pal"

/*
 * Function definitions
 */

static struct sensor *to_adv7842pal(const struct i2c_client *client)
{
	return i2c_get_clientdata(client);
}

static struct sensor *v4l2_to_adv7842pal(const struct v4l2_int_device *dev)
{
	return dev->priv;
}

static int reg_read(struct i2c_client *client, const u8 reg)
{
	int ret;

	ret = swab16(i2c_smbus_read_word_data(client, reg & 0xff));

	dev_err(&client->dev, "read  reg.%03x -> %04x\n", reg, ret);
	return ret;
}

static int reg_write(struct i2c_client *client, const u8 reg, const u16 data)
{
	int ret;

	ret = i2c_smbus_write_word_data(client, reg & 0xff, swab16(data));
	dev_err(&client->dev, "write reg.%03x = %04x -> %d\n", reg, data, ret);
	return ret;
}

static int reg_set(struct i2c_client *client, const u8 reg, const u16 data)
{
	int ret;

	ret = reg_write(client, reg, ret | data);
	return ret;
}

/* ----------------------------------------------------------------------- */
static s32 adv_smbus_read_byte_data_check(struct i2c_client *client, u8 command,
					  bool check)
{
#if 1
	union i2c_smbus_data data;
	if (!i2c_smbus_xfer(client->adapter, client->addr, client->flags,
			    I2C_SMBUS_READ, command, I2C_SMBUS_BYTE_DATA,
			    &data))
		return data.byte;
#else
	u8 buffer[1];
	int rc;

	buffer[0] = command;
	if (1 != (rc = i2c_master_send(client, buffer, 1))) {
		dev_err(&client->dev, "send reg error: addr=%2x\n", command);
		return -1;
	}

	msleep(10);
	if (1 != (rc = i2c_master_recv(client, buffer, 1))) {
		dev_err(&client->dev, "read reg error: addr=%2x\n", command);
		return -1;
	}
	return buffer[0];

#endif
	return i2c_smbus_read_byte_data(client, command);

	if (check)
		v4l_err(client, "error reading %02x, %02x\n", client->addr,
			command);
	return -EIO;
}

static s32 adv_smbus_read_byte_data(struct i2c_client *client, u8 command)
{
	int i;

	for (i = 0; i < 3; i++) {
		int ret = adv_smbus_read_byte_data_check(client, command, true);

		if (ret >= 0) {
			if (i)
				v4l_err(client, "read ok after %d retries\n",
					i);
			return ret;
		}
	}
	v4l_err(client, "read failed\n");
	return -EIO;
}

static s32 adv_smbus_write_byte_data(struct i2c_client *client, u8 command,
				     u8 value)
{
#if 1
	union i2c_smbus_data data;
	int err, i;
	data.byte = value;
	for (i = 0; i < 3; i++) {
		err = i2c_smbus_xfer(client->adapter, client->addr,
				     client->flags, I2C_SMBUS_WRITE, command,
				     I2C_SMBUS_BYTE_DATA, &data);
		if (!err)
			break;
	}
	if (err < 0)
		v4l_err(client, "error writing %02x, %02x, %02x\n",
			client->addr, command, value);
	return err;
#else
	u8 buffer[2];
	int rc;

	buffer[0] = command;
	buffer[1] = value;
	if (2 != (rc = i2c_master_send(client, buffer, 2))) {
		dev_err(&client->dev, "write reg error:addr=%2x,value=%2x\n",
			command, value);
		return -1;
	}

	return 0;
	return i2c_smbus_write_byte_data(client, command, value);

#endif
}

static void adv_smbus_write_byte_no_check(struct i2c_client *client, u8 command,
					  u8 value)
{
	union i2c_smbus_data data;
	data.byte = value;

	i2c_smbus_xfer(client->adapter, client->addr, client->flags,
		       I2C_SMBUS_WRITE, command, I2C_SMBUS_BYTE_DATA, &data);
}

static s32 adv_smbus_write_i2c_block_data(struct i2c_client *client, u8 command,
					  unsigned length, const u8 *values)
{
	union i2c_smbus_data data;

	if (length > I2C_SMBUS_BLOCK_MAX)
		length = I2C_SMBUS_BLOCK_MAX;
	data.block[0] = length;
	memcpy(data.block + 1, values, length);
	return i2c_smbus_xfer(client->adapter, client->addr, client->flags,
			      I2C_SMBUS_WRITE, command,
			      I2C_SMBUS_I2C_BLOCK_DATA, &data);
}

static int reg_clear(struct i2c_client *client, const u8 reg, const u16 data)
{
	int ret;

	ret = reg_read(client, reg);
	return reg_write(client, reg, ret & ~data);
}

/* ----------------------------------------------------------------------- */
static inline int io_read(struct i2c_client *client, u8 reg)
{
	return adv_smbus_read_byte_data(client, reg);
}

static inline int io_write(struct i2c_client *client, u8 reg, u8 val)
{
	return adv_smbus_write_byte_data(client, reg, val);
}

static inline int io_write_and_or(struct i2c_client *client, u8 reg, u8 mask,
				  u8 val)
{
	return io_write(client, reg, (io_read(client, reg) & mask) | val);
}

static inline int sdp_read(struct i2c_client *client, u8 reg)
{
	struct sensor *data = to_adv7842pal(client);

	return adv_smbus_read_byte_data(data->i2c_sdp, reg);
}

// 0x90
static inline int sdp_write(struct i2c_client *client, u8 reg, u8 val)
{
	struct sensor *data = to_adv7842pal(client);

	return adv_smbus_write_byte_data(data->i2c_sdp, reg, val);
}

static inline int sdpio_read(struct i2c_client *client, u8 reg)
{
	struct sensor *data = to_adv7842pal(client);

	return adv_smbus_read_byte_data(data->i2c_sdp_io, reg);
}
// 0x94
static inline int sdpio_write(struct i2c_client *client, u8 reg, u8 val)
{
	struct sensor *data = to_adv7842pal(client);

	return adv_smbus_write_byte_data(data->i2c_sdp_io, reg, val);
}

static inline int afe_read(struct i2c_client *client, u8 reg)
{
	struct sensor *data = to_adv7842pal(client);

	return adv_smbus_read_byte_data(data->i2c_afe, reg);
}

// 0x4c
static inline int afe_write(struct i2c_client *client, u8 reg, u8 val)
{
	struct sensor *data = to_adv7842pal(client);

	return adv_smbus_write_byte_data(data->i2c_afe, reg, val);
}

static inline int afe_write_and_or(struct i2c_client *client, u8 reg, u8 mask,
				   u8 val)
{
	return afe_write(client, reg, (afe_read(client, reg) & mask) | val);
}

static inline int cp_read(struct i2c_client *client, u8 reg)
{
	struct sensor *data = to_adv7842pal(client);

	return adv_smbus_read_byte_data(data->i2c_cp, reg);
}
// 0x44
static inline int cp_write(struct i2c_client *client, u8 reg, u8 val)
{
	struct sensor *data = to_adv7842pal(client);

	return adv_smbus_write_byte_data(data->i2c_cp, reg, val);
}

static inline int cp_write_and_or(struct i2c_client *client, u8 reg, u8 mask,
				  u8 val)
{
	return cp_write(client, reg, (cp_read(client, reg) & mask) | val);
}

static void adv7842pal_reset(struct i2c_client *client)
{
	/* 40 FF 80; I2C reset */
	io_write(client, 0xff, 0x80);
	mdelay(5);
}

static void adv7842pal_unregister_clients(struct i2c_client *client)
{
	struct sensor *data = to_adv7842pal(client);

	if (data->i2c_avlink)
		i2c_unregister_device(data->i2c_avlink);
	if (data->i2c_cec)
		i2c_unregister_device(data->i2c_cec);
	if (data->i2c_infoframe)
		i2c_unregister_device(data->i2c_infoframe);
	if (data->i2c_sdp_io)
		i2c_unregister_device(data->i2c_sdp_io);
	if (data->i2c_sdp)
		i2c_unregister_device(data->i2c_sdp);
	if (data->i2c_afe)
		i2c_unregister_device(data->i2c_afe);
	if (data->i2c_ksv)
		i2c_unregister_device(data->i2c_ksv);
	if (data->i2c_edid)
		i2c_unregister_device(data->i2c_edid);
	if (data->i2c_hdmi)
		i2c_unregister_device(data->i2c_hdmi);
	if (data->i2c_cp)
		i2c_unregister_device(data->i2c_cp);
	if (data->i2c_vdp)
		i2c_unregister_device(data->i2c_vdp);

	data->i2c_sdp = NULL;
	data->i2c_sdp_io = NULL;
	data->i2c_avlink = NULL;
	data->i2c_cec = NULL;
	data->i2c_infoframe = NULL;
	data->i2c_afe = NULL;
	data->i2c_ksv = NULL;
	data->i2c_edid = NULL;
	data->i2c_hdmi = NULL;
	data->i2c_cp = NULL;
	data->i2c_vdp = NULL;
}

static struct i2c_client *adv7842pal_dummy_client(struct i2c_client *client,
						  const char *desc, u8 addr,
						  u8 io_reg)
{
	struct i2c_client *cp;
	if (addr == 0) {
		printk("no %s i2c addr configured\n", desc);
		return NULL;
	}

	io_write(client, io_reg, addr);
	cp = i2c_new_dummy(client->adapter, io_read(client, io_reg) >> 1);
	if (!cp)
		printk("register %s on i2c addr 0x%x failed\n", desc, addr);

	return cp;
}

static int adv7842pal_register_clients(struct i2c_client *client)
{
	struct sensor *data = to_adv7842pal(client);

	data->i2c_sdp = adv7842pal_dummy_client(client, "sdp", 0x90, 0xf1);
	data->i2c_sdp_io =
		adv7842pal_dummy_client(client, "sdp_io", 0x94, 0xf2);
	data->i2c_avlink =
		adv7842pal_dummy_client(client, "avlink", 0x84, 0xf3);
	data->i2c_cec = adv7842pal_dummy_client(client, "cec", 0x80, 0xf4);
	data->i2c_infoframe =
		adv7842pal_dummy_client(client, "infoframe", 0x7c, 0xf5);
	data->i2c_afe = adv7842pal_dummy_client(client, "afe", 0x4c, 0xf8);
	data->i2c_ksv = adv7842pal_dummy_client(client, "ksv", 0x64, 0xf9);
	data->i2c_edid = adv7842pal_dummy_client(client, "edid", 0x6c, 0xfa);
	data->i2c_hdmi = adv7842pal_dummy_client(client, "hdmi", 0x68, 0xfb);
	data->i2c_cp = adv7842pal_dummy_client(client, "cp", 0x44, 0xfd);
	data->i2c_vdp = adv7842pal_dummy_client(client, "vdp", 0x48, 0xfe);

	if (!data->i2c_avlink || !data->i2c_cec || !data->i2c_infoframe ||
	    !data->i2c_sdp_io || !data->i2c_sdp || !data->i2c_afe ||
	    !data->i2c_ksv || !data->i2c_edid || !data->i2c_hdmi ||
	    !data->i2c_cp || !data->i2c_vdp) {
		pr_err("camera map i2c address failed\n");
		return -1;
	}

	/*
	40 00 01
	40 01 00
	40 03 00
	40 04 62
	40 0C 40
	40 15 80
	40 19 83
	40 33 40
	*/
	io_write(client, 0x00, 0x01);
	io_write(client, 0x01, 0x00);
	io_write(client, 0x03, 0x00);
	io_write(client, 0x04, 0x62);
	io_write(client, 0x0c, 0x40);
	io_write(client, 0x15, 0x80);
	io_write(client, 0x19, 0x83);
	io_write(client, 0x33, 0x40);

	/*
	4C 0C 1F
	4C 12 63
	4C 00 0E
	4C 02 80
	4C 03 A0
	*/
	afe_write(client, 0x0c, 0x1f);
	afe_write(client, 0x12, 0x63);
	afe_write(client, 0x00, 0x0e);
	afe_write(client, 0x02, 0x80);
	afe_write(client, 0x03, 0xa0);

	/*
	94 7A A5; sdp_io
	94 7B 8F
	94 60 01
	94 97 00
	94 AA 05
	94 B0 CC
	94 B2 6C
	*/
	sdpio_write(client, 0x7a, 0xa5);
	sdpio_write(client, 0x7b, 0x8f);
	sdpio_write(client, 0x60, 0x01);
	sdpio_write(client, 0x97, 0x00);
	sdpio_write(client, 0xaa, 0x05);
	sdpio_write(client, 0xb0, 0xcc);
	sdpio_write(client, 0xb2, 0x6c);

	/*
	sdp
	90 00 01
	90 01 00
	90 03 E3
	90 04 E8
	90 05 C4
	90 06 11
	90 12 05 -> 90 12 00 by lwx ain new board of 2016/12/7
	90 A7 00
	*/
	sdp_write(client, 0x00, 0x01);
	sdp_write(client, 0x01, 0x00);
	sdp_write(client, 0x03, 0xe3);
	sdp_write(client, 0x04, 0xe8);
	sdp_write(client, 0x05, 0xc4);
	sdp_write(client, 0x06, 0x11);
	sdp_write(client, 0x12, 0x00);
	sdp_write(client, 0xa7, 0x00);

	return 0;
}

/*!
 * Return attributes of current video standard.
 * Since this device autodetects the current standard, this function also
 * sets the values that need to be changed if the standard changes.
 * There is no set std equivalent function.
 *
 *  @return		None.
 */
static void adv7842_get_std(v4l2_std_id *std)
{
	int idx;
	*std = V4L2_STD_PAL; // we only support PAL now!
	idx = ADV7842_PAL;

	/* This assumes autodetect which this device uses. */
	if (*std != adv7842_pal.std_id) {
		video_idx = idx;
		adv7842_pal.std_id = *std;
		adv7842_pal.sen.pix.width = video_fmts[video_idx].raw_width;
		adv7842_pal.sen.pix.height = video_fmts[video_idx].raw_height;
	}
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
	// struct sensor *sensor = v4l2_to_adv7842pal(s);

	pr_debug("In adv7842pal:ioctl_g_ifparm\n");

	if (s == NULL) {
		printk("::: ioctl_g_ifparm (END, error)\n");
		pr_err("   ERROR!! no slave device set!\n");
		return -1;
	}

	/* Initialize structure to 0s then set any non-0 values. */
	memset(p, 0, sizeof(*p));
	p->if_type = V4L2_IF_TYPE_BT656; /* This is the only possibility. */
	p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
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
	struct sensor *sensor = s->priv;

	pr_debug("In adv7842pal:ioctl_s_power\n");

#if 0
	if (on)
		gpio_sensor_active();
	else
		gpio_sensor_inactive();
#endif
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
	int ret = 0;
	struct sensor *sensor = v4l2_to_adv7842pal(s);
	struct v4l2_captureparm *cparm = &a->parm.capture;

	/* s->priv points to adv7842pal_data */

	pr_debug("In adv7842pal:ioctl_g_parm\n");

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n");
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->sen.streamcap.capability;
		cparm->timeperframe = sensor->sen.streamcap.timeperframe;
		cparm->capturemode = sensor->sen.streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_err("   type is not V4L2_BUF_TYPE_VIDEO_CAPTURE "
		       "but %d\n",
		       a->type);
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
	/* s->priv points to adv7842pal_data */
	struct sensor *sensor = s->priv;

	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   Returning size of %dx%d\n", sensor->sen.pix.width,
			 sensor->sen.pix.height);
		f->fmt.pix = sensor->sen.pix;
		break;

	case V4L2_BUF_TYPE_PRIVATE: {
		v4l2_std_id std;
		adv7842_get_std(&std);
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
	pr_debug("In adv7842pal:ioctl_queryctrl\n");

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
	struct sensor *sensor = v4l2_to_adv7842pal(s);
	struct i2c_client *client = sensor->sen.i2c_client;
	int value = 0;
	s8 tmp;

	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		value = sdp_read(client, 0x14);
		tmp = sdp_read(client, 0x17);
		value = value << 2 | (tmp >> 2 & 0x3);
		vc->value = (50 * value / 0x1ff) + 50;
		break;
	case V4L2_CID_CONTRAST:
		value = sdp_read(client, 0x13);
		tmp = sdp_read(client, 0x17);
		value = value << 2 | (tmp & 0x3);
		vc->value = value * 100 / 0x3ff;
		break;
	case V4L2_CID_SATURATION:
		value = sdp_read(client, 0x15);
		tmp = sdp_read(client, 0x17);
		value = value << 2 | (tmp >> 4 & 0x3);
		vc->value = value * 100 / 0x3ff;
		break;
	case V4L2_CID_HUE:
		value = sdp_read(client, 0x16);
		tmp = sdp_read(client, 0x17);
		vc->value = 0;
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		pr_debug("   V4L2_CID_AUTO_WHITE_BALANCE\n");
		vc->value = 0;
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		pr_debug("   V4L2_CID_DO_WHITE_BALANCE\n");
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
		vc->value = 0;
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
	struct sensor *sensor = v4l2_to_adv7842pal(s);
	struct i2c_client *client = sensor->sen.i2c_client;
	int retval = 0;
	int value = 0;
	s8 tmp;

	pr_debug("In adv7842pal:ioctl_s_ctrl %d\n", vc->id);
	switch (vc->id) {
	case V4L2_CID_BRIGHTNESS:
		value = (vc->value - 50) * 0x1ff / 50;
		retval = sdp_write(client, 0x14, 0xff & (value >> 2));
		tmp = sdp_read(client, 0x17);
		retval = sdp_write(client, 0x17, tmp | ((0x3 & value) << 2));
		break;

	case V4L2_CID_CONTRAST:
		value = (vc->value * 0x3ff) / 100;
		retval = sdp_write(client, 0x13, (0xff & value >> 2));
		tmp = sdp_read(client, 0x17);
		retval = sdp_write(client, 0x17, tmp | (0x3 & value));

		break;
	case V4L2_CID_SATURATION:
		value = (vc->value * 0x3ff) / 100;
		retval = sdp_write(client, 0x15, (0xff & value >> 2));
		tmp = sdp_read(client, 0x17);
		retval = sdp_write(client, 0x17, tmp | ((0x3 & value) << 4));

		break;
	case V4L2_CID_HUE:
		// value = (vc->value - 50) * 90 / 50;
		// retval = cp_write(client, 0x3d, 0xff & value);
		retval = -1;
		break;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		pr_debug("   V4L2_CID_AUTO_WHITE_BALANCE\n");
		break;
	case V4L2_CID_DO_WHITE_BALANCE:
		pr_debug("   V4L2_CID_DO_WHITE_BALANCE\n");
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
	struct sensor *sensor = v4l2_to_adv7842pal(s);

	if (fmt->index > 1)
		return -EINVAL;

	fmt->pixelformat = sensor->sen.pix.pixelformat;

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
	       "adv7842pal_decoder");
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
 * ioctl_dev_init - V4L2 sensor interface handler for vidioc_int_dev_init_num
 * @s: pointer to standard V4L2 device structure
 *
 * Initialise the device when slave attaches to the master.
 */
static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct sensor *sensor = v4l2_to_adv7842pal(s);
	struct i2c_client *client = sensor->sen.i2c_client;

	pr_err("caeram ioctl_dev_init...\n");

	return 0;
}

/*!
 * This structure defines all the ioctls for this module.
 */
static struct v4l2_int_ioctl_desc adv7842pal_ioctl_desc[] = {

	{ vidioc_int_dev_init_num, (v4l2_int_ioctl_func *)ioctl_dev_init },

	/*!
	 * Delinitialise the dev. at slave detach.
	 * The complement of ioctl_dev_init.
	 */
	/* {vidioc_int_dev_exit_num, (v4l2_int_ioctl_func *)ioctl_dev_exit}, */

	{ vidioc_int_s_power_num, (v4l2_int_ioctl_func *)ioctl_s_power },
	{ vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func *)ioctl_g_ifparm },
	/* {vidioc_int_g_needs_reset_num,
	  (v4l2_int_ioctl_func *)ioctl_g_needs_reset}, */
	/* {vidioc_int_reset_num, (v4l2_int_ioctl_func *)ioctl_reset}, */
	{ vidioc_int_init_num, (v4l2_int_ioctl_func *)ioctl_init },

	/*!
	 * VIDIOC_ENUM_FMT ioctl for the CAPTURE buffer type.
	 */
	/* {vidioc_int_enum_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_enum_fmt_cap}, */

	/*!
	 * VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.
	 * This ioctl is used to negotiate the image capture size and
	 * pixel format without actually making it take effect.
	 */
	/* {vidioc_int_try_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_try_fmt_cap}, */

	{ vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_g_fmt_cap },

	/*!
	 * If the requested format is supported, configures the HW to use that
	 * format, returns error code if format not supported or HW can't be
	 * correctly configured.
	 */
	/* {vidioc_int_s_fmt_cap_num, (v4l2_int_ioctl_func *)ioctl_s_fmt_cap}, */

	{ vidioc_int_g_parm_num, (v4l2_int_ioctl_func *)ioctl_g_parm },
	{ vidioc_int_s_parm_num, (v4l2_int_ioctl_func *)ioctl_s_parm },
	{ vidioc_int_queryctrl_num, (v4l2_int_ioctl_func *)ioctl_queryctrl },
	{ vidioc_int_g_ctrl_num, (v4l2_int_ioctl_func *)ioctl_g_ctrl },
	{ vidioc_int_s_ctrl_num, (v4l2_int_ioctl_func *)ioctl_s_ctrl },
	{ vidioc_int_enum_framesizes_num,
	  (v4l2_int_ioctl_func *)ioctl_enum_framesizes },
	{ vidioc_int_g_chip_ident_num,
	  (v4l2_int_ioctl_func *)ioctl_g_chip_ident },
};

static struct v4l2_int_slave adv7842pal_slave = {
	.ioctls = adv7842pal_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(adv7842pal_ioctl_desc),
};

static struct v4l2_int_device adv7842pal_int_device = {
	.module = THIS_MODULE,
	.name = "adv7842pal",
	.type = v4l2_int_type_slave,
	.u =
		{
			.slave = &adv7842pal_slave,
		},
};

/*!
 * adv7842pal I2C probe function
 * Function set in i2c_driver struct.
 * Called by insmod adv7842pal_camera.ko.
 *
 * @return  Error code indicating success or failure
 */
static int adv7842pal_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
#if 1
	int rev;
	int ret = 0;
	tvin_plat = client->dev.platform_data;

	pr_err("In adv7842pal_probe\n");

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
	memset(&adv7842_pal, 0, sizeof(adv7842_pal));
	adv7842_pal.sen.i2c_client = client;
	i2c_set_clientdata(client, &adv7842_pal);
	adv7842_pal.sen.streamcap.timeperframe.denominator = 25;
	adv7842_pal.sen.streamcap.timeperframe.numerator = 1;
	adv7842_pal.std_id = V4L2_STD_ALL;
	video_idx = ADV7842_NOT_LOCKED;
	adv7842_pal.sen.pix.width = video_fmts[video_idx].raw_width;
	adv7842_pal.sen.pix.height = video_fmts[video_idx].raw_height;
	adv7842_pal.sen.pix.pixelformat = V4L2_PIX_FMT_UYVY; /* YUV422 */
	adv7842_pal.sen.pix.priv = 1; /* 1 is used to indicate TV in */
	adv7842_pal.sen.on = true;
	adv7842_pal.sen.csi = 1;
	adv7842_pal.sen.mclk_source = 1;

	/*
	gpio_sensor_active();
	*/

	dev_err(&adv7842_pal.sen.i2c_client->dev,
		"%s:adv7842 probe i2c address is 0x%02X\n", __func__,
		adv7842_pal.sen.i2c_client->addr);

	adv7842pal_reset(client); // 复位adv7842pal
	/* i2c access to adv7842? */
	rev = adv_smbus_read_byte_data_check(client, 0xea, false) << 8 |
	      adv_smbus_read_byte_data_check(client, 0xeb, false);

	if (rev != 0x2012) {
		dev_err(&adv7842_pal.sen.i2c_client->dev,
			"not an adv7842pal on address 0x%x (rev=0x%04x)\n",
			client->addr, rev);
		return -ENODEV;
	}
	dev_err(&adv7842_pal.sen.i2c_client->dev,
		"find adv7842pal on address 0x%x (rev=0x%04x)\n", client->addr,
		rev);

	if (adv7842pal_register_clients(client) < 0) {
		printk(KERN_ERR "failed to create all i2c clients\n");
		return -ENOMEM;
	}

	/* This function attaches this structure to the /dev/video0 device.
	 * The pointer in priv points to the mt9v111_data structure here.*/

	adv7842pal_int_device.priv = &adv7842_pal;
	ret = v4l2_int_device_register(&adv7842pal_int_device);

	return ret;
#else
	int retval;
	pr_debug("In adv7842pal_probe  device id is %s\n", id->name);

	/* Set initial values for the sensor struct. */
	adv7842_pal.sen.i2c_client = client;
	i2c_set_clientdata(client, adv7842pal);
	adv7842pal->platform_data = client->dev.platform_data;
	adv7842pal->csi = adv7842pal->platform_data->csi;
	adv7842pal->io_init = adv7842pal->platform_data->io_init;

	pr_debug("client name is %s\n", client->name);
	adv7842pal->pix.pixelformat = V4L2_PIX_FMT_UYVY;
	adv7842pal->pix.width = adv7842pal_MAX_WIDTH;
	adv7842pal->pix.height = adv7842pal_MAX_HEIGHT;
	adv7842pal->streamcap.capability = 0; /* No higher resolution or frame
						* frame rate changes supported.
						*/
	adv7842pal->streamcap.timeperframe.denominator = 25;
	adv7842pal->streamcap.timeperframe.numerator = 1;

	adv7842pal_int_device.priv = adv7842pal;

	pr_debug("   type is %d (expect %d)\n", adv7842pal_int_device.type,
		 v4l2_int_type_slave);
	pr_debug("   num ioctls is %d\n",
		 adv7842pal_int_device.u.slave->num_ioctls);

	adv7842pal_reset(client);

	/* i2c access to adv7842? */
	rev = adv_smbus_read_byte_data_check(client, 0xea, false) << 8 |
	      adv_smbus_read_byte_data_check(client, 0xeb, false);

	if (rev != 0x2012) {
		dev_err(&client->dev, "got rev=0x%04x on first read attempt\n",
			rev);
		rev = adv_smbus_read_byte_data_check(client, 0xea, false) << 8 |
		      adv_smbus_read_byte_data_check(client, 0xeb, false);
	}
	if (rev != 0x2012) {
		dev_err(&client->dev,
			"not an adv7842 on address 0x%x (rev=0x%04x)\n",
			client->addr, rev);
		return -ENODEV;
	}
	dev_err(&client->dev, "find adv7842pal on address 0x%x (rev=0x%04x)\n",
		client->addr, rev);

	if (adv7842pal_register_clients(client) < 0) {
		printk(KERN_ERR "failed to create all i2c clients\n");
		return -ENOMEM;
	}

	/* This function attaches this structure to the /dev/video0 device.
	 * The pointer in priv points to the adv7842pal_data structure here.*/
	retval = v4l2_int_device_register(&adv7842pal_int_device);

	if (adv7842pal->io_init)
		adv7842pal->io_init();

	return retval;
#endif
}

/*!
 * Function set in i2c_driver struct.
 * Called on rmmod adv7842pal_camera.ko
 */
static int adv7842pal_remove(struct i2c_client *client)
{
	pr_debug("In adv7842pal_remove\n");

	v4l2_int_device_unregister(&adv7842pal_int_device);

	return 0;
}

static const struct i2c_device_id adv7842pal_id[] = {
	{ "adv7842pal", 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, adv7842pal_id);

static struct i2c_driver adv7842pal_i2c_driver = {
	.driver =
		{
			.owner = THIS_MODULE,
			.name = "adv7842pal",
		},
	.probe = adv7842pal_probe,
	.remove = adv7842pal_remove,
	.id_table = adv7842pal_id,
	/* To add power management add .suspend and .resume functions */
};

/*!
 * adv7842pal init function.
 * Called by insmod adv7842pal_camera.ko.
 *
 * @return  Error code indicating success or failure
 */
static __init int adv7842pal_init(void)
{
	u8 err;

	/* Tells the i2c driver what functions to call for this driver. */
	err = i2c_add_driver(&adv7842pal_i2c_driver);
	if (err != 0)
		pr_err("%s: driver registration failed, error=%d \n", __func__,
		       err);

	return err;
}

/*!
 * adv7842pal cleanup function.
 * Called on rmmod adv7842pal_camera.ko
 *
 * @return  Error code indicating success or failure
 */
static void __exit adv7842pal_clean(void)
{
	adv7842pal_unregister_clients(adv7842_pal.sen.i2c_client);
	i2c_del_driver(&adv7842pal_i2c_driver);
	gpio_sensor_inactive();
}

module_init(adv7842pal_init);
module_exit(adv7842pal_clean);

MODULE_AUTHOR("lwx wooshang@126.com");
MODULE_DESCRIPTION("ADV7842 Camera Driver");
MODULE_LICENSE("GPL");
