/*
 * File: lt8918.c
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Lontium LT8918 MIPI DSI/CSI-2 Transmitter
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Project       : i2c
 * Author        : Varphone Wong <varphone@qq.com>
 * File Created  : 2019-11-23 06:15:37 Saturday, 23 November
 * Last Modified : 2019-11-23 06:16:21 Saturday, 23 November
 * Modified By   : Varphone Wong <varphone@qq.com>
 * ---------------------------------------------------------------------------
 * Copyright (C) 2012-2019 CETC55, Technology Development CO.,LTD.
 * Copyright (C) 2012-2019 Varphone Wong, Varphone.com.
 * All rights reserved.
 * ---------------------------------------------------------------------------
 * HISTORY:
 */
#include <linux/bug.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "lt8918_regs.h"

struct lt8918_mode_info {
	u32 code;   /* Media bus code */
	u32 width;  /* Pixel width */
	u32 height; /* Pixel height */
	u32 fps;    /* Frame rate */
	u32 field;  /* Field */
};

struct lt8918 {
	struct v4l2_subdev base;
	struct media_pad pads[2];

	struct device* dev;
	struct i2c_client* i2c_client;
	struct regmap* regmap;
	int reset_gpio;
	bool is_power_on;
	u8 curr_bank;
	enum lt8918_mode curr_mode;
	struct v4l2_fract curr_frame_interval;
};

static const struct regmap_config lt8918_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.use_single_rw = true,
};

static struct lt8918_mode_info lt8918_mode_infos[] = {
	/* LT8918_MODE_PAL */
	{ MEDIA_BUS_FMT_UYVY8_2X8, 720, 576, 25, V4L2_FIELD_NONE },
	/* LT8918_MODE_NTSC */
	{ MEDIA_BUS_FMT_UYVY8_2X8, 720, 480, 30, V4L2_FIELD_NONE },
	/* LT8918_MODE_720P25 */
	{ MEDIA_BUS_FMT_UYVY8_2X8, 1280, 720, 25, V4L2_FIELD_NONE },
	/* LT8918_MODE_720P30 */
	{ MEDIA_BUS_FMT_UYVY8_2X8, 1280, 720, 30, V4L2_FIELD_NONE },
	/* LT8918_MODE_720P50 */
	{ MEDIA_BUS_FMT_UYVY8_2X8, 1280, 720, 50, V4L2_FIELD_NONE },
	/* LT8918_MODE_720P60 */
	{ MEDIA_BUS_FMT_UYVY8_2X8, 1280, 720, 60, V4L2_FIELD_NONE },
	/* LT8918_MODE_1080P25 */
	{ MEDIA_BUS_FMT_UYVY8_2X8, 1920, 1080, 25, V4L2_FIELD_NONE },
	/* LT8918_MODE_1080P30 */
	{ MEDIA_BUS_FMT_UYVY8_2X8, 1920, 1080, 30, V4L2_FIELD_NONE },
	/* LT8918_MODE_1080P50 */
	{ MEDIA_BUS_FMT_UYVY8_2X8, 1920, 1080, 50, V4L2_FIELD_NONE },
	/* LT8918_MODE_1080P60 */
	{ MEDIA_BUS_FMT_UYVY8_2X8, 1920, 1080, 60, V4L2_FIELD_NONE },
	// END
	{ 0, 0, 0, 0, 0 },
};

static int mode_for_format(struct v4l2_subdev_format* fmt, struct v4l2_fract* fi)
{
	struct lt8918_mode_info *info;
	int i;
	for (i = 0; i < LT8918_MODE_MAX; i++) {
		info = &lt8918_mode_infos[i];
		if (fmt->format.width == info->width &&
		    fmt->format.height == info->height &&
		    fi->denominator == info->fps) {
			return i;
		}
	}
	return LT8918_MODE_720P25;
}

static inline int lt8918_read_reg(struct lt8918 *self, u16 addr, u8* val)
{
	int err = 0;
	u32 reg_val = 0;
	err = regmap_read(self->regmap, addr, &reg_val);
	*val = reg_val & 0xFF;
	return err;
}

static inline int lt8918_write_reg(struct lt8918 *self, u16 addr, u8 val)
{
	return regmap_write(self->regmap, addr, val);
}

static int lt8918_write_table(struct lt8918* self,
			      const struct lt8918_reg_table* table)
{
	return regmap_multi_reg_write(self->regmap, table->regs, table->num_regs);
}

static inline void lt8918_set_bank(struct lt8918* self, u8 bank)
{
	regmap_write(self->regmap, 0xff, bank);
}

static void lt8918_hardware_reset(struct lt8918* self)
{
	gpio_set_value(self->reset_gpio, 0);
	usleep_range(1000, 2000);
	gpio_set_value(self->reset_gpio, 1);
	usleep_range(1000, 2000);
	regmap_write(self->regmap, 0xff, 0x60);
	regmap_write(self->regmap, 0xee, 0x01);
	usleep_range(1000, 2000);
	self->is_power_on = true;
}

static void lt8918_hardware_shutdown(struct lt8918* self)
{
	gpio_set_value(self->reset_gpio, 0);
	self->is_power_on = false;
}

static unsigned int lt8918_get_chip_id(struct lt8918* self)
{
	struct regmap* regmap = self->regmap;
	u32 ids[4];

	regmap_write(regmap, 0xff, 0x60);
	regmap_read(regmap, 0x00, &ids[0]);
	regmap_read(regmap, 0x01, &ids[1]);
	regmap_read(regmap, 0x02, &ids[2]);
	ids[3] = 0;
	return (ids[3] << 24) | (ids[2] << 16) | (ids[1] << 8) | (ids[0] & 0xff);
}

static void lt8918_log_status(struct lt8918* self)
{
	struct device* dev = self->dev;
	struct regmap* regmap = self->regmap;
	u8 vals[64] __maybe_unused;
	u16 sval;
	u32 ival;
	u32 ivals[4];

	// Switch to bank 0x80
	regmap_write(regmap, 0xFF, 0x80);

	regmap_read(regmap, 0x52, &ivals[0]);
	regmap_read(regmap, 0x53, &ivals[1]);
	sval = (ivals[0] << 8) | ivals[1];
	dev_dbg(dev, "HACT = %u (0x%02x%02x)\n", sval, ivals[0], ivals[1]);

	regmap_read(regmap, 0x4A, &ivals[0]);
	regmap_read(regmap, 0x4B, &ivals[1]);
	sval = ((ivals[0] & 0x0F) << 8) | ivals[1];
	dev_dbg(dev, "HFP = %u (0x%02x%02x)\n", sval, ivals[0], ivals[1]);

	regmap_read(regmap, 0x44, &ivals[0]);
	regmap_read(regmap, 0x45, &ivals[1]);
	sval = ((ivals[0] & 0x0F) << 8) | ivals[1];
	dev_dbg(dev, "HSA = %u (0x%02x%02x)\n", sval, ivals[0], ivals[1]);

	regmap_read(regmap, 0x48, &ivals[0]);
	regmap_read(regmap, 0x49, &ivals[1]);
	sval = ((ivals[0] & 0x0F) << 8) | ivals[1];
	dev_dbg(dev, "HBP = %u (0x%02x%02x)\n", sval, ivals[0], ivals[1]);

	regmap_read(regmap, 0x4E, &ivals[0]);
	regmap_read(regmap, 0x4F, &ivals[1]);
	sval = (ivals[0] << 8) | ivals[1];
	dev_dbg(dev, "HTATOL = %u (0x%02x%02x)\n", sval, ivals[0], ivals[1]);

	regmap_read(regmap, 0x50, &ivals[0]);
	regmap_read(regmap, 0x51, &ivals[1]);
	sval = ((ivals[0] & 0x0F) << 8) | ivals[1];
	dev_dbg(dev, "VACT = %u (0x%02x%02x)\n", sval, ivals[0], ivals[1]);

	regmap_read(regmap, 0x47, &ivals[0]);
	dev_dbg(dev, "VFP = %u\n", ivals[0]);

	regmap_read(regmap, 0x43, &ivals[0]);
	dev_dbg(dev, "VSA = %u\n", ivals[0]);

	regmap_read(regmap, 0x46, &ivals[0]);
	dev_dbg(dev, "VBP = %u\n", ivals[0]);

	regmap_read(regmap, 0x4C, &ivals[0]);
	regmap_read(regmap, 0x4D, &ivals[1]);
	sval = ((ivals[0] & 0x0F) << 8) | ivals[1];
	dev_dbg(dev, "VTATOL = %u (0x%02x%02x)\n", sval, ivals[0], ivals[1]);

	// Switch to bank 0x80
	regmap_write(regmap, 0xFF, 0x80);

	// Select byteclk to read
	regmap_write(regmap, 0x33, 0x08);
	usleep_range(48000, 52000);
	regmap_read(regmap, 0x30, &ivals[0]);
	regmap_read(regmap, 0x31, &ivals[1]);
	regmap_read(regmap, 0x32, &ivals[2]);
	ival = ((ivals[0] & 0x0F) << 16) | (ivals[1] << 8) | ivals[2];
	dev_dbg(dev, "BYTECLK = %ukHz\n", ival);

	// Select pixclk to read
	regmap_write(regmap, 0x33, 0x0C);
	usleep_range(48000, 52000);
	regmap_read(regmap, 0x30, &ivals[0]);
	regmap_read(regmap, 0x31, &ivals[1]);
	regmap_read(regmap, 0x32, &ivals[2]);
	ival = ((ivals[0] & 0x0F) << 16) | (ivals[1] << 8) | ivals[2];
	dev_dbg(dev, "PIXCLK = %ukHz\n", ival);
}

static int lt8918_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index < 0 || code->index >= LT8918_MODE_MAX)
		return -EINVAL;
	code->code = lt8918_mode_infos[code->index].code;
	return 0;
}

static int lt8918_enum_framesizes(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct lt8918_mode_info* info;
	if (fse->index < 0 || fse->index >= LT8918_MODE_MAX)
		return -EINVAL;
	info = &lt8918_mode_infos[fse->index];
	fse->code = info->code;
	fse->min_width = fse->max_width = info->width;
	fse->min_height = fse->max_height = info->height;
	return 0;
}

static int lt8918_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_frame_interval_enum *fie)
{
	struct lt8918_mode_info* info;
	if (fie->index < 0 || fie->index >= LT8918_MODE_MAX)
		return -EINVAL;
	info = &lt8918_mode_infos[fie->index];
	fie->code = info->code;
	fie->width = info->width;
	fie->height = info->height;
	fie->interval.denominator = info->fps;
	fie->interval.numerator = 1;
	return 0;
}

static int lt8918_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct lt8918 *self = v4l2_get_subdevdata(sd);
	struct lt8918_mode_info* info = &lt8918_mode_infos[self->curr_mode];
	fmt->format.width = info->width;
	fmt->format.height = info->height;
	fmt->format.code = info->code;
	fmt->format.field = info->field;
	return 0;
}

static int lt8918_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct lt8918 *self = v4l2_get_subdevdata(sd);
	int new_mode = mode_for_format(fmt, &self->curr_frame_interval);
	if (self->curr_mode != new_mode) {
		// Update new mode
		self->curr_mode = new_mode;
	}
	return 0;
}

static int lt8918_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				 struct v4l2_mbus_frame_desc *fd)
{
	return 0;
}

static int lt8918_set_frame_desc(struct v4l2_subdev *sd,
				 unsigned int pad,
				 struct v4l2_mbus_frame_desc *fd)
{
	return 0;
}

static int lt8918_set_power(struct v4l2_subdev *sd, int on)
{
	struct lt8918 *self = v4l2_get_subdevdata(sd);
	if (!on && self->is_power_on)
		lt8918_hardware_shutdown(self);
	else if (on && !self->is_power_on)
		lt8918_hardware_reset(self);
	return 0;
}

static int lt8918_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	return 0;
}

static int lt8918_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parm)
{
	return 0;
}

static int lt8918_s_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

static int lt8918_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *interval)
{
	struct lt8918 *self = v4l2_get_subdevdata(sd);
	interval->interval = self->curr_frame_interval;
	return 0;
}

static int lt8918_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *interval)
{
	struct lt8918 *self = v4l2_get_subdevdata(sd);
	self->curr_frame_interval = interval->interval;
	return 0;
}

static int lt8918_link_setup(struct media_entity *entity,
			     const struct media_pad *local,
			     const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct lt8918 *self = v4l2_get_subdevdata(sd);

	return 0;
}

static const struct v4l2_subdev_pad_ops lt8918_pad_ops = {
	.enum_mbus_code         = lt8918_enum_mbus_code,
	.enum_frame_size        = lt8918_enum_framesizes,
	.enum_frame_interval    = lt8918_enum_frame_interval,
	.get_fmt                = lt8918_get_fmt,
	.set_fmt                = lt8918_set_fmt,
	.get_frame_desc         = lt8918_get_frame_desc,
	.set_frame_desc         = lt8918_set_frame_desc,
};

static const struct v4l2_subdev_core_ops lt8918_core_ops = {
	.s_power                = lt8918_set_power,
};

static const struct v4l2_subdev_video_ops lt8918_video_ops = {
	.s_parm                 = lt8918_s_parm,
	.g_parm                 = lt8918_g_parm,
	.s_stream               = lt8918_s_stream,
	.g_frame_interval       = lt8918_g_frame_interval,
	.s_frame_interval       = lt8918_s_frame_interval,
};

static const struct v4l2_subdev_ops lt8918_subdev_ops = {
	.core  = &lt8918_core_ops,
	.pad   = &lt8918_pad_ops,
	.video = &lt8918_video_ops,
};

static const struct media_entity_operations lt8918_sd_media_ops = {
	.link_setup = lt8918_link_setup,
};

static int lt8918_parse_dt(struct lt8918 *self)
{
	struct device *dev = &self->i2c_client->dev;
	int ret = 0;

	/* Setup the reset gpio */
	self->reset_gpio = of_get_named_gpio(dev->of_node, "reset-gpios", 0);
	if (!gpio_is_valid(self->reset_gpio)) {
		dev_err(dev, "No sensor reset pin available\n");
		return -ENODEV;
	}

	ret = devm_gpio_request_one(dev, self->reset_gpio, GPIOF_OUT_INIT_HIGH,
				    "lt8918_rst");
	if (ret < 0) {
		dev_err(dev, "Failed to request sensor reset gpio: %d\n", ret);
	}

	return ret;
}

static int lt8918_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct v4l2_subdev* sd = NULL;
	struct lt8918 *self = NULL;
	u32 chip_id = 0;
	int ret = 0;

	self = devm_kzalloc(dev, sizeof(*self), GFP_KERNEL);
	if (!self)
		return -ENOMEM;

	self->regmap = devm_regmap_init_i2c(client, &lt8918_regmap_config);
	if (IS_ERR(self->regmap)) {
		dev_err(dev, "Regmap failed: %ld\n", PTR_ERR(self->regmap));
		return -ENODEV;
	}

	self->dev = dev;
	self->i2c_client = client;
	self->curr_mode = LT8918_MODE_720P25;
	self->curr_frame_interval.denominator = lt8918_mode_infos[self->curr_mode].fps;
	self->curr_frame_interval.numerator = 1;

	if (lt8918_parse_dt(self) < 0)
		return -EINVAL;

	lt8918_hardware_reset(self);

	chip_id = lt8918_get_chip_id(self);
	if (chip_id != LT8918_CHIP_ID) {
		dev_err(dev, "Chip Not Found!\n");
		return -ENODEV;
	}

	dev_info(dev, "Found, Chip Id: 0x%08X\n", chip_id);

	sd = &self->base;
	v4l2_i2c_subdev_init(sd, self->i2c_client, &lt8918_subdev_ops);

	sd->flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.function = MEDIA_ENT_F_IF_VID_DECODER;
	self->pads[0].flags = MEDIA_PAD_FL_SINK;
	self->pads[1].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, 2, self->pads);
	if (ret < 0) {
		v4l2_err(sd, "Failed to init media entity, err: %d\n", ret);
		goto err_media_entity_pads_init;
	}

	sd->entity.ops = &lt8918_sd_media_ops;
	ret = v4l2_async_register_subdev(sd);
	if (ret < 0) {
		v4l2_err(sd, "Failed to register subdev, err: %d\n", ret);
		goto err_v4l2_async_register_subdev;
	}

	i2c_set_clientdata(client, self);
	v4l2_set_subdevdata(sd, self);

	lt8918_write_table(self, &lt8918_mode_tables[self->curr_mode]);

	return 0;

err_v4l2_async_register_subdev:
	media_entity_cleanup(&sd->entity);
err_media_entity_pads_init:
	return ret;
}

static int lt8918_remove(struct i2c_client *client)
{
	struct lt8918 *self = i2c_get_clientdata(client);
	struct v4l2_subdev *sd = &self->base;
	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	return 0;
}

static const struct i2c_device_id lt8918_id[] = {
	{},
};

MODULE_DEVICE_TABLE(i2c, lt8918_id);

static const struct of_device_id lt8918_of_match[] = {
	{ .compatible = "lontium,lt8918" },
	{ /* sentinel */ }
};

static struct i2c_driver lt8918_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LT8918_DRIVER_NAME,
		.of_match_table = of_match_ptr(lt8918_of_match),
	},
	.probe = lt8918_probe,
	.remove = lt8918_remove,
	.id_table = lt8918_id,
};

module_i2c_driver(lt8918_i2c_driver);

MODULE_AUTHOR("CETC55, Technology Development CO.,LTD.");
MODULE_AUTHOR("Varphone Wong <varphone@qq.com>");
MODULE_DESCRIPTION("Lontium LT8918 MIPI DSI/CSI-2 Transmitter Driver");
MODULE_ALIAS("i2c:" LT8918_DRIVER_NAME);
MODULE_LICENSE("GPL");
