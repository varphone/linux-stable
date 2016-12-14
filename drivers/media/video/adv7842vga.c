/*
 * Driver for adv7842vga6 FPGA Image Sensor
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/videodev2.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/log2.h>

#include <media/v4l2-subdev.h>
#include <media/v4l2-chip-ident.h>
#include <media/soc_camera.h>

/*
 * ADV7842VGA and ADV7842VGA i2c address 0x48, 0x4c, 0x58, 0x5c
 * The platform has to define ctruct i2c_board_info objects and link to them
 * from struct soc_camera_link
 */

static char *sensor_type;
module_param(sensor_type, charp, S_IRUGO);
MODULE_PARM_DESC(sensor_type, "Sensor type: \"colour\" or \"monochrome\"");

/* Progressive scan, master, defaults */
#define ADV7842VGA_CHIP_CONTROL_DEFAULT	0x188

#define ADV7842VGA_MAX_WIDTH		1280
#define ADV7842VGA_MAX_HEIGHT		1024
#define ADV7842VGA_MIN_WIDTH		48
#define ADV7842VGA_MIN_HEIGHT		32
#define ADV7842VGA_COLUMN_SKIP		0
#define ADV7842VGA_ROW_SKIP		0


/* ADV7842VGA has only one fixed colorspace per pixelcode */
struct adv7842_vga_datafmt {
	enum v4l2_mbus_pixelcode	code;
	enum v4l2_colorspace		colorspace;
};


static const struct adv7842_vga_datafmt adv7842vga_colour_fmt = {
	/*
	 * Order important: first natively supported,
	 * second supported with a GPIO extender
	 */
	.code = V4L2_MBUS_FMT_YUYV8_2X8,
    .colorspace = V4L2_COLORSPACE_JPEG,
};


struct adv7842_vga {
	struct v4l2_subdev subdev;
	struct v4l2_rect rect;	/* Sensor window */
	const struct adv7842_vga_datafmt *fmt;
	int num_fmts;
	int model;	    /* V4L2_IDENT_MT9V02x codes from v4l2-chip-ident.h */
	u16 chip_control;
	unsigned short y_skip_top;	/* Lines to skip at the top */

	/* i2c clients */
	struct i2c_client *i2c_sdp;     //0x90
	struct i2c_client *i2c_sdp_io;  //0x94
	struct i2c_client *i2c_avlink; //0x84
	struct i2c_client *i2c_cp;     //0x44
	struct i2c_client *i2c_vdp;   //0x48
	struct i2c_client *i2c_afe;   //0x4c
	struct i2c_client *i2c_hdmi;  //0x68
	struct i2c_client *i2c_ksv; 
	struct i2c_client *i2c_edid; //0x6c
	struct i2c_client *i2c_infoframe; //0x7c
	struct i2c_client *i2c_cec;  //0x80


};

/* ----------------------------------------------------------------------- */
static struct adv7842_vga *to_adv7842vga(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct adv7842_vga, subdev);
}

/* ----------------------------------------------------------------------- */
static s32 adv_smbus_read_byte_data_check(struct i2c_client *client, u8 command, bool check)
{
#if 1
	union i2c_smbus_data data;
	if (!i2c_smbus_xfer(client->adapter, client->addr, client->flags,
			    I2C_SMBUS_READ, command,
			    I2C_SMBUS_BYTE_DATA, &data))
		return data.byte;
#else
	u8 buffer[1];
	int rc;

	buffer[0] = command;
	if (1 != (rc = i2c_master_send(client, buffer, 1))) {
		dev_dbg(&client->dev, "send reg error: addr=%2x\n",  command);
		return -1;
	}

	msleep(10);
	if (1 != (rc = i2c_master_recv(client, buffer, 1))) {
		dev_dbg(&client->dev, "read reg error: addr=%2x\n", command);
		return -1;
	}
	return buffer[0];

#endif
    return i2c_smbus_read_byte_data(client, command);

	if (check)
		v4l_err(client, "error reading %02x, %02x\n", client->addr, command);
	return -EIO;
}

static s32 adv_smbus_read_byte_data(struct i2c_client *client, u8 command)
{
	int i;

	for (i = 0; i < 3; i++) {
		int ret = adv_smbus_read_byte_data_check(client, command, true);

		if (ret >= 0) {
			if (i)
				v4l_err(client, "read ok after %d retries\n", i);
			return ret;
		}
	}
	v4l_err(client, "read failed\n");
	return -EIO;
}

static s32 adv_smbus_write_byte_data(struct i2c_client *client, u8 command, u8 value)
{
#if 1
	union i2c_smbus_data data;
	int err, i;
	data.byte = value;
	for (i = 0; i < 3; i++) {
		err = i2c_smbus_xfer(client->adapter, client->addr,
				     client->flags,
				     I2C_SMBUS_WRITE, command,
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
		dev_dbg(&client->dev,"write reg error:addr=%2x,value=%2x\n", command, value);
		return -1;
	}

	return 0;
    return i2c_smbus_write_byte_data(client, command, value);

#endif
}

static void adv_smbus_write_byte_no_check(struct i2c_client *client,
					  u8 command, u8 value)
{

	union i2c_smbus_data data;
	data.byte = value;

	i2c_smbus_xfer(client->adapter, client->addr,
		       client->flags,
		       I2C_SMBUS_WRITE, command,
		       I2C_SMBUS_BYTE_DATA, &data);
}

static s32 adv_smbus_write_i2c_block_data(struct i2c_client *client, u8 command, unsigned length, const u8 *values)
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

/* ----------------------------------------------------------------------- */
static inline int io_read(struct i2c_client *client, u8 reg)
{
	return adv_smbus_read_byte_data(client, reg);
}

static inline int io_write(struct i2c_client *client, u8 reg, u8 val)
{
	return adv_smbus_write_byte_data(client, reg, val);
}

static inline int io_write_and_or(struct i2c_client *client, u8 reg, u8 mask, u8 val)
{
	return io_write(client, reg, (io_read(client, reg) & mask) | val);
}

static inline int afe_read(struct i2c_client *client, u8 reg)
{
    struct adv7842_vga *vga = to_adv7842vga(client);

	return adv_smbus_read_byte_data(vga->i2c_afe, reg);
}

// 0x4c
static inline int afe_write(struct i2c_client *client, u8 reg, u8 val)
{
    struct adv7842_vga *vga = to_adv7842vga(client);

	return adv_smbus_write_byte_data(vga->i2c_afe, reg, val);
}

static inline int afe_write_and_or(struct i2c_client *client, u8 reg, u8 mask, u8 val)
{
	return afe_write(client, reg, (afe_read(client, reg) & mask) | val);
}

static inline int cp_read(struct i2c_client *client, u8 reg)
{
    struct adv7842_vga *vga = to_adv7842vga(client);

	return adv_smbus_read_byte_data(vga->i2c_cp, reg);
}
// 0x44
static inline int cp_write(struct i2c_client *client, u8 reg, u8 val)
{
    struct adv7842_vga *vga = to_adv7842vga(client);

	return adv_smbus_write_byte_data(vga->i2c_cp, reg, val);
}

static inline int cp_write_and_or(struct i2c_client *client, u8 reg, u8 mask, u8 val)
{
	return cp_write(client, reg, (cp_read(client, reg) & mask) | val);
}

static void adv7842vga_rewrite_i2c_addresses(struct i2c_client *client)  // struct adv7842_platform_data *pdata)
{
/*
40 F1 90 ; SDP map
40 F2 94 ; SDPIO map
40 F3 84 ; AVLINK map
40 F4 80 ; CEC map
40 F5 7C ; INFOFRAME map
40 F8 4C ; AFE map
40 F9 64 ; KSV map
40 FA 6C ; EDID map
40 FB 68 ; HDMI map
40 FD 44 ; CP map
40 FE 48 ; VDP map
*/
    int value;
//	io_write(client, 0xf1, pdata->i2c_sdp);
//	io_write(client, 0xf2, pdata->i2c_sdp_io);
	io_write(client, 0xf3, 0x84);
	io_write(client, 0xf4, 0x80);
	io_write(client, 0xf5, 0x7c);

	io_write(client, 0xf8, 0x4c);
//	io_write(client, 0xf9, pdata->i2c_repeater << 1);
	io_write(client, 0xfa, 0x6c);
	io_write(client, 0xfb, 0x68);

	io_write(client, 0xfd, 0x44);
	io_write(client, 0xfe, 0x48);

//just for test

    value = io_read(client, 0xf3);
    printk(KERN_ERR "(40 F8 4C ; AFE map):%x", value);
}


static void adv7842vga_reset(struct i2c_client *client)
{
/* 40 FF 80 ; I2C reset */
	io_write(client, 0xff, 0x80);
	mdelay(5);
}

static int adv7842vga_init(struct i2c_client *client)
{
	//struct adv7842_vga *adv7842vga = to_adv7842vga(client);

    printk("adv7842vga_init ok..\n");
    return 0;
}

static int adv7842vga_s_stream(struct v4l2_subdev *sd, int enable)
{
    //TODO
	return 0;
}

static int adv7842vga_set_bus_param(struct soc_camera_device *icd,
				 unsigned long flags)
{
	struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
//	struct adv7842_vga *adv7842vga = to_adv7842vga(client);
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	unsigned int width_flag = flags & SOCAM_DATAWIDTH_MASK;
	int ret;
	u16 pixclk = 0;

	/* Only one width bit may be set */
	if (!is_power_of_2(width_flag))
		return -EINVAL;

	if (icl->set_bus_param) {
		ret = icl->set_bus_param(icl, width_flag);
		if (ret)
			return ret;
	} else {
		/*
		 * Without board specific bus width settings we only support the
		 * sensors native bus width
		 */
		if (width_flag != SOCAM_DATAWIDTH_16) {
            dev_err(&client->dev, "We need SOCAM_DATAWIDTH_16 but target is:%u\n", width_flag);
			return -EINVAL;
        }
	}

	flags = soc_camera_apply_sensor_flags(icl, flags);

	if (flags & SOCAM_PCLK_SAMPLE_FALLING)
		pixclk |= 0x10;

	if (!(flags & SOCAM_HSYNC_ACTIVE_HIGH))
		pixclk |= 0x1;

	if (!(flags & SOCAM_VSYNC_ACTIVE_HIGH))
		pixclk |= 0x2;

#if 0 
    // 写入时钟和主从模式，可能不需要该
	ret = reg_write(client, ADV7842VGA_PIXCLK_FV_LV, pixclk);
	if (ret < 0)
		return ret;

	if (!(flags & SOCAM_MASTER))
		adv7842vga->chip_control &= ~0x8;

	ret = reg_write(client, ADV7842VGA_CHIP_CONTROL, adv7842vga->chip_control);
	if (ret < 0)
		return ret;
	dev_dbg(&client->dev, "Calculated pixclk 0x%x, chip control 0x%x\n", pixclk, adv7842vga->chip_control);

#endif

	return 0;
}

static unsigned long adv7842vga_query_bus_param(struct soc_camera_device *icd)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	unsigned int flags = SOCAM_MASTER | SOCAM_SLAVE |
		SOCAM_PCLK_SAMPLE_RISING | SOCAM_PCLK_SAMPLE_FALLING |
		SOCAM_HSYNC_ACTIVE_HIGH | SOCAM_HSYNC_ACTIVE_LOW |
		SOCAM_VSYNC_ACTIVE_HIGH | SOCAM_VSYNC_ACTIVE_LOW |
		SOCAM_DATA_ACTIVE_HIGH;

	if (icl->query_bus_param)
		flags |= icl->query_bus_param(icl) & SOCAM_DATAWIDTH_MASK;
	else
		flags |= SOCAM_DATAWIDTH_16;

	return soc_camera_apply_sensor_flags(icl, flags);
}

static int adv7842vga_s_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct adv7842_vga *adv7842vga = to_adv7842vga(client);
	struct v4l2_rect rect = a->c;

#if 0
	/* Bayer format - even size lengths */
	if (adv7842vga->fmts == adv7842vga_colour_fmts) {
		rect.width	= ALIGN(rect.width, 2);
		rect.height	= ALIGN(rect.height, 2);
		/* Let the user play with the starting pixel */
	}
#endif

	soc_camera_limit_side(&rect.left, &rect.width,
		     ADV7842VGA_COLUMN_SKIP, ADV7842VGA_MIN_WIDTH, ADV7842VGA_MAX_WIDTH);

	soc_camera_limit_side(&rect.top, &rect.height,
		     ADV7842VGA_ROW_SKIP, ADV7842VGA_MIN_HEIGHT, ADV7842VGA_MAX_HEIGHT);
#if 0 // maybe wo do not support crop
	/* Like in example app. Contradicts the datasheet though */
	ret = reg_read(client, ADV7842VGA_AEC_AGC_ENABLE);
	if (ret >= 0 && ret & 1) /* Autoexposure */
		ret = reg_write(client, ADV7842VGA_MAX_TOTAL_SHUTTER_WIDTH,
					rect.height + adv7842vga->y_skip_top + 43);
	/* Setup frame format: defaults apart from width and height */
	if (!ret)
		ret = reg_write(client, ADV7842VGA_COLUMN_START, rect.left);
	if (!ret)
		ret = reg_write(client, ADV7842VGA_ROW_START, rect.top);
	if (!ret) {
		if (is_mt9v024()) {
			ret =  reg_write(client, ADV7842VGA_HORIZONTAL_BLANKING,
				rect.width > 690 - 61 ? 61 :
				690 - rect.width);
		} else {
			ret = reg_write(client, ADV7842VGA_HORIZONTAL_BLANKING,
				rect.width > 660 - 43 ? 43 :
				660 - rect.width);
		}
	}

	if (!ret)
		ret = reg_write(client, ADV7842VGA_VERTICAL_BLANKING, 45);
	if (!ret)
		ret = reg_write(client, ADV7842VGA_WINDOW_WIDTH, rect.width);
	if (!ret)
		ret = reg_write(client, ADV7842VGA_WINDOW_HEIGHT,
				rect.height + adv7842vga->y_skip_top);

	if (ret < 0)
		return ret;

	dev_dbg(&client->dev, "Frame %dx%d pixel\n", rect.width, rect.height);
#endif
	adv7842vga->rect = rect;

	return 0;
}

static int adv7842vga_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct adv7842_vga *adv7842vga = to_adv7842vga(client);

	a->c	= adv7842vga->rect;
	a->type	= V4L2_BUF_TYPE_VIDEO_CAPTURE;

	return 0;
}

static int adv7842vga_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	a->bounds.left			= ADV7842VGA_COLUMN_SKIP;
	a->bounds.top			= ADV7842VGA_ROW_SKIP;
	a->bounds.width			= ADV7842VGA_MAX_WIDTH;
	a->bounds.height		= ADV7842VGA_MAX_HEIGHT;
	a->defrect			= a->bounds;
	a->type				= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator	= 1;
	a->pixelaspect.denominator	= 1;

	return 0;
}

static int adv7842vga_g_fmt(struct v4l2_subdev *sd,
			 struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct adv7842_vga *adv7842vga = to_adv7842vga(client);

	mf->width	= adv7842vga->rect.width;
	mf->height	= adv7842vga->rect.height;
	mf->code	= adv7842vga->fmt->code;
	mf->colorspace	= adv7842vga->fmt->colorspace;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int adv7842vga_s_fmt(struct v4l2_subdev *sd,
			 struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct adv7842_vga *adv7842vga = to_adv7842vga(client);
	struct v4l2_crop a = {
		.c = {
			.left	= adv7842vga->rect.left,
			.top	= adv7842vga->rect.top,
			.width	= mf->width,
			.height	= mf->height,
		},
	};
	int ret;

	/*
	 * The caller provides a supported format, as verified per call to
	 * icd->try_fmt(), datawidth is from our supported format list
	 */
	switch (mf->code) {
	case V4L2_MBUS_FMT_YUYV8_2X8:
		if (adv7842vga->model != V4L2_IDENT_NONE)
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	/* No support for scaling on this camera, just crop. */
	ret = adv7842vga_s_crop(sd, &a);
	if (!ret) {
		mf->width	= adv7842vga->rect.width;
		mf->height	= adv7842vga->rect.height;
		mf->colorspace	= adv7842vga->fmt->colorspace;
	}
	return ret;
}

static int adv7842vga_try_fmt(struct v4l2_subdev *sd,
			   struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct adv7842_vga *adv7842vga = to_adv7842vga(client);
	int align = mf->code == V4L2_MBUS_FMT_YUYV8_2X8;

	v4l_bound_align_image(&mf->width, ADV7842VGA_MIN_WIDTH,
		ADV7842VGA_MAX_WIDTH, align,
		&mf->height, ADV7842VGA_MIN_HEIGHT + adv7842vga->y_skip_top,
		ADV7842VGA_MAX_HEIGHT + adv7842vga->y_skip_top, align, 0);

	mf->colorspace	= adv7842vga->fmt->colorspace;
	return 0;
}

static int adv7842vga_g_chip_ident(struct v4l2_subdev *sd,
				struct v4l2_dbg_chip_ident *id)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct adv7842_vga *adv7842vga = to_adv7842vga(client);
	if (id->match.type != V4L2_CHIP_MATCH_I2C_ADDR)
		return -EINVAL;

	if (id->match.addr != client->addr)
		return -ENODEV;

	id->ident	= adv7842vga->model;
	id->revision	= 0;
	strcpy(id->match.name, "adv7842vga");
	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int adv7842vga_g_register(struct v4l2_subdev *sd,
			      struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (reg->match.type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xff)
		return -EINVAL;

	if (reg->match.addr != client->addr)
		return -ENODEV;

	reg->size = 2;
	reg->val = io_read(client, reg->reg);

	if (reg->val > 0xffff)
		return -EIO;

	return 0;
}

static int adv7842vga_s_register(struct v4l2_subdev *sd,
			      struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (reg->match.type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xff)
		return -EINVAL;

	if (reg->match.addr != client->addr)
		return -ENODEV;

	if (io_write(client, reg->reg, reg->val) < 0)
		return -EIO;

	return 0;
}
#endif

static struct soc_camera_ops adv7842vga_ops = {
	.set_bus_param		= adv7842vga_set_bus_param,
	.query_bus_param	= adv7842vga_query_bus_param,
//	.controls		= adv7842vga_controls,
//	.num_controls		= ARRAY_SIZE(adv7842vga_controls),
};


/*
 * Interface active, can use i2c. If it fails, it can indeed mean, that
 * this wasn't our capture interface, so, we wait for the right one
 */
static int adv7842vga_video_probe(struct soc_camera_device *icd,
			       struct i2c_client *client)
{
	struct adv7842_vga *adv7842vga = to_adv7842vga(client);
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	s32 data;
	int ret;
//	unsigned long flags;

	if (!icd->dev.parent ||
	    to_soc_camera_host(icd->dev.parent)->nr != icd->iface)
		return -ENODEV;
/*
40 00 16 ; VID_STD=010110b for UXGA60
40 01 82 ; Prim_Mode to graphics input
40 02 F6 ; Auto input color space, Limited Range RGB Output
40 03 80 ; 16 bit SDR 422
40 05 28 ; Disable AV Codes
40 0C 40 ; Power up Part
40 14 3F ; Max Drive Strength 
40 15 B0 ; Disable Tristate of Pins except for Audio pins
40 19 81 ; LLC DLL Phase
40 33 40 ; LLC DLL Enable
*/  
    io_write(client, 0x00, 0x16);
    io_write(client, 0x01, 0x82);
    io_write(client, 0x02, 0xf6);
    io_write(client, 0x03, 0x80);
    io_write(client, 0x05, 0x28);
    io_write(client, 0x0c, 0x40);
    io_write(client, 0x14, 0x3f);
    io_write(client, 0x15, 0xb0);
    io_write(client, 0x19, 0x81);
    io_write(client, 0x33, 0x40);
/*
4C 05 0F ; AA Filters Enable
4C 06 20 ; AA Filters BW 146MHz
4C 07 FF ; AA Filters BW 146MHz
*/
    afe_write(client, 0x05, 0x0f);
    afe_write(client, 0x06, 0x20);
    afe_write(client, 0x07, 0xff);
/*
44 73 EA ; Set manual gain of 0x2A8
44 74 8A ; Set manual gain of 0x2A8
44 75 A2 ; Set manual gain of 0x2A8
44 76 A8 ; Set manual gain of 0x2A8
44 85 0B ; Disable Autodetectmode for Sync_Source for CH1. Force CH1 to use HS&VS
44 C3 39 ; ADI recommended write
*/
    cp_write(client, 0x73, 0xea);
    cp_write(client, 0x74, 0x8a);
    cp_write(client, 0x75, 0xa2);
    cp_write(client, 0x76, 0xa8);
    cp_write(client, 0x85, 0x0b);
    cp_write(client, 0xc3, 0x39);

/*
4C 0C 1F ; ADI recommended write
4C 12 63 ; ADI recommended write
4C 00 80 ; ADC power Up
4C 02 00 ; Ain_Sel to 000. (Ain 1,2,3)
4C C8 35 ; Good DLL phase for evaluation board
*/
    afe_write(client, 0x0c, 0x1f);
    afe_write(client, 0x12, 0x63);
    afe_write(client, 0x00, 0x80);
    afe_write(client, 0x02, 0x00);
    afe_write(client, 0xc8, 0x35);


    adv7842vga->num_fmts = 1;
	adv7842vga->fmt = &adv7842vga_colour_fmt;
    adv7842vga->model = V4L2_IDENT_NONE;
	ret = adv7842vga_init(client);
	if (ret < 0)
		dev_err(&client->dev, "Failed to initialise the camera\n");

ei2c:
	return ret;
}

static void adv7842vga_video_remove(struct soc_camera_device *icd)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);

	dev_dbg(&icd->dev, "Video removed: %p, %p\n",
		icd->dev.parent, icd->vdev);
	if (icl->free_bus)
		icl->free_bus(icl);
}

static int adv7842vga_g_skip_top_lines(struct v4l2_subdev *sd, u32 *lines)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct adv7842_vga *adv7842vga = to_adv7842vga(client);

	*lines = adv7842vga->y_skip_top;

	return 0;
}

static struct v4l2_subdev_core_ops adv7842vga_subdev_core_ops = {
//	.g_ctrl		= adv7842vga_g_ctrl,
//	.s_ctrl		= adv7842vga_s_ctrl,
	.g_chip_ident	= adv7842vga_g_chip_ident,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= adv7842vga_g_register,
	.s_register	= adv7842vga_s_register,
#endif
};

static int adv7842vga_enum_fmt(struct v4l2_subdev *sd, unsigned int index, enum v4l2_mbus_pixelcode *code)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct adv7842_vga *vga = to_adv7842vga(client);

	if (index >= vga->num_fmts)
		return -EINVAL;

	*code = vga->fmt->code;
	return 0;
}

static struct v4l2_subdev_video_ops adv7842vga_subdev_video_ops = {
	.s_stream	= adv7842vga_s_stream,
	.s_mbus_fmt	= adv7842vga_s_fmt,
	.g_mbus_fmt	= adv7842vga_g_fmt,
	.try_mbus_fmt	= adv7842vga_try_fmt,
	.s_crop		= adv7842vga_s_crop,
	.g_crop		= adv7842vga_g_crop,
	.cropcap	= adv7842vga_cropcap,
	.enum_mbus_fmt	= adv7842vga_enum_fmt,
};

static struct v4l2_subdev_sensor_ops adv7842vga_subdev_sensor_ops = {
	.g_skip_top_lines	= adv7842vga_g_skip_top_lines,
};

static struct v4l2_subdev_ops adv7842vga_subdev_ops = {
	.core	= &adv7842vga_subdev_core_ops,
	.video	= &adv7842vga_subdev_video_ops,
	.sensor	= &adv7842vga_subdev_sensor_ops,
};


static void adv7842vga_unregister_clients(struct i2c_client *client)
{
	struct adv7842_vga *vga = to_adv7842vga(client);

	if (vga->i2c_avlink)
		i2c_unregister_device(vga->i2c_avlink);
	if (vga->i2c_cec)
		i2c_unregister_device(vga->i2c_cec);
	if (vga->i2c_infoframe)
		i2c_unregister_device(vga->i2c_infoframe);
	if (vga->i2c_sdp_io)
		i2c_unregister_device(vga->i2c_sdp_io);
	if (vga->i2c_sdp)
		i2c_unregister_device(vga->i2c_sdp);
	if (vga->i2c_afe)
		i2c_unregister_device(vga->i2c_afe);
	if (vga->i2c_ksv)
		i2c_unregister_device(vga->i2c_ksv);
	if (vga->i2c_edid)
		i2c_unregister_device(vga->i2c_edid);
	if (vga->i2c_hdmi)
		i2c_unregister_device(vga->i2c_hdmi);
	if (vga->i2c_cp)
		i2c_unregister_device(vga->i2c_cp);
	if (vga->i2c_vdp)
		i2c_unregister_device(vga->i2c_vdp);

	vga->i2c_sdp = NULL;
	vga->i2c_sdp_io = NULL;
	vga->i2c_avlink = NULL;
	vga->i2c_cec = NULL;
	vga->i2c_infoframe = NULL;
	vga->i2c_afe = NULL;
	vga->i2c_ksv = NULL;
	vga->i2c_edid = NULL;
	vga->i2c_hdmi = NULL;
	vga->i2c_cp = NULL;
	vga->i2c_vdp = NULL;
}

static struct i2c_client *adv7842vga_dummy_client(struct i2c_client *client , const char *desc, u8 addr, u8 io_reg)
{
	struct i2c_client *cp;
	if (addr == 0) {
		printk("no %s i2c addr configured\n", desc);
		return NULL;
	}

	io_write(client, io_reg, addr);
	cp = i2c_new_dummy(client->adapter, io_read(client, io_reg)>>1);
	if (!cp)
		printk("register %s on i2c addr 0x%x failed\n", desc, addr);

	return cp;
}

static int adv7842vga_register_clients(struct i2c_client *client)
{
	struct adv7842_vga *vga = to_adv7842vga(client);

	vga->i2c_sdp = adv7842vga_dummy_client(client, "sdp", 0x90, 0xf1);
	vga->i2c_sdp_io = adv7842vga_dummy_client(client, "sdp_io", 0x94, 0xf2);
	vga->i2c_avlink = adv7842vga_dummy_client(client, "avlink", 0x84, 0xf3);
	vga->i2c_cec = adv7842vga_dummy_client(client, "cec", 0x80, 0xf4);
	vga->i2c_infoframe = adv7842vga_dummy_client(client, "infoframe", 0x7c, 0xf5);
	vga->i2c_afe = adv7842vga_dummy_client(client, "afe", 0x4c, 0xf8);
	vga->i2c_ksv = adv7842vga_dummy_client(client, "ksv", 0x64, 0xf9);
	vga->i2c_edid = adv7842vga_dummy_client(client, "edid", 0x6c, 0xfa);
	vga->i2c_hdmi = adv7842vga_dummy_client(client, "hdmi", 0x68, 0xfb);
	vga->i2c_cp = adv7842vga_dummy_client(client, "cp", 0x44, 0xfd);
	vga->i2c_vdp = adv7842vga_dummy_client(client, "vdp", 0x48, 0xfe);

	if (!vga->i2c_avlink ||
	    !vga->i2c_cec ||
	    !vga->i2c_infoframe ||
	    !vga->i2c_sdp_io ||
	    !vga->i2c_sdp ||
	    !vga->i2c_afe ||
	    !vga->i2c_ksv ||
	    !vga->i2c_edid ||
	    !vga->i2c_hdmi ||
	    !vga->i2c_cp ||
	    !vga->i2c_vdp)
		return -1;

	return 0;
}

static int adv7842vga_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	struct adv7842_vga *adv7842vga;
	struct soc_camera_device *icd = client->dev.platform_data;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct soc_camera_link *icl;
	int ret, rev;
	printk(KERN_ERR "we in probe\n");
	if (!icd) {
		dev_err(&client->dev, "ADV7842VGA: missing soc-camera data!\n");
		return -EINVAL;
	}

	icl = to_soc_camera_link(icd);
	if (!icl) {
		dev_err(&client->dev, "ADV7842VGA driver needs platform data\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_warn(&adapter->dev,
			 "I2C-Adapter doesn't support I2C_FUNC_SMBUS_WORD\n");
		return -EIO;
	}
    
	adv7842vga = kzalloc(sizeof(struct adv7842_vga), GFP_KERNEL);
	if (!adv7842vga)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&adv7842vga->subdev, client, &adv7842vga_subdev_ops);

    adv7842vga_reset(client);

	/* i2c access to adv7842? */
	rev = adv_smbus_read_byte_data_check(client, 0xea, false) << 8 |
		adv_smbus_read_byte_data_check(client, 0xeb, false);

	if (rev != 0x2012) {
		v4l2_info(&adv7842vga->subdev, "got rev=0x%04x on first read attempt\n", rev);
		rev = adv_smbus_read_byte_data_check(client, 0xea, false) << 8 |
			adv_smbus_read_byte_data_check(client, 0xeb, false);
	}
	if (rev != 0x2012) {
		v4l2_info(&adv7842vga->subdev, "not an adv7842 on address 0x%x (rev=0x%04x)\n",  client->addr, rev);
		return -ENODEV;
	}
	v4l2_info(&adv7842vga->subdev, "find adv7842vga on address 0x%x (rev=0x%04x)\n",  client->addr, rev);

	if (adv7842vga_register_clients(client) < 0) {
		printk(KERN_ERR "failed to create all i2c clients\n");
		kfree(adv7842vga);
        return -ENOMEM;
	}

	adv7842vga->chip_control = ADV7842VGA_CHIP_CONTROL_DEFAULT;
	icd->ops		= &adv7842vga_ops;
	/*
	 * ADV7842VGA _really_ corrupts the first read out line.
	 * TODO: verify on i.MX31
	 */
	adv7842vga->y_skip_top	= 0;
	adv7842vga->rect.left	= ADV7842VGA_COLUMN_SKIP;
	adv7842vga->rect.top	= ADV7842VGA_ROW_SKIP;
	adv7842vga->rect.width	= ADV7842VGA_MAX_WIDTH;
	adv7842vga->rect.height	= ADV7842VGA_MAX_HEIGHT;

	ret = adv7842vga_video_probe(icd, client);
	if (ret) {
		icd->ops = NULL;
		kfree(adv7842vga);
	}

	return ret;
}

static int adv7842vga_remove(struct i2c_client *client)
{
	struct adv7842_vga *vga = to_adv7842vga(client);
	struct soc_camera_device *icd = client->dev.platform_data;

	icd->ops = NULL;
    adv7842vga_unregister_clients(client);
	adv7842vga_video_remove(icd);
	kfree(vga);

	return 0;
}
static const struct i2c_device_id adv7842vga_id[] = {
	{ "adv7842vga", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, adv7842vga_id);

static struct i2c_driver adv7842vga_i2c_driver = {
	.driver = {
		.name = "adv7842vga",
	},
	.probe		= adv7842vga_probe,
	.remove		= adv7842vga_remove,
	.id_table	= adv7842vga_id,
};

static int __init adv7842vga_module_init(void)
{
	return i2c_add_driver(&adv7842vga_i2c_driver);
}

static void __exit adv7842vga_module_exit(void)
{
	i2c_del_driver(&adv7842vga_i2c_driver);
}

module_init(adv7842vga_module_init);
module_exit(adv7842vga_module_exit);

MODULE_DESCRIPTION("FPGA video driver");
MODULE_AUTHOR("wooshang <wooshang@126.com>");
MODULE_LICENSE("GPL");
