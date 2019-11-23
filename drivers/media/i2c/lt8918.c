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
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "lt8918_regs.h"

#define LT8918_DRIVER_NAME "lt8918"

struct lt8918 {
	struct v4l2_subdev base;
	struct media_pad pads[2];

	struct i2c_client* i2c_client;
};

static int lt8918_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	return 0;
}

static int lt8918_enum_framesizes(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	return 0;
}

static int lt8918_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_frame_interval_enum *fie)
{
	return -EINVAL;
}

static int lt8918_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	return 0;
}

static int lt8918_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
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

static int lt8918_link_setup(struct media_entity *entity,
			     const struct media_pad *local,
			     const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct lt8918 *self = v4l2_get_subdevdata(sd);

	if (WARN_ON(self == NULL))
		return 0;

	if (!(flags & MEDIA_LNK_FL_ENABLED)) {
		return 0;
	}

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
	.s_power	= lt8918_set_power,
};

static const struct v4l2_subdev_video_ops lt8918_video_ops = {
	.s_parm		= lt8918_s_parm,
	.g_parm		= lt8918_g_parm,
	.s_stream	= lt8918_s_stream,
};

static const struct v4l2_subdev_ops lt8918_subdev_ops = {
	.core	= &lt8918_core_ops,
	.pad	= &lt8918_pad_ops,
	.video	= &lt8918_video_ops,
};

static const struct media_entity_operations lt8918_sd_media_ops = {
	.link_setup = lt8918_link_setup,
};

static int lt8918_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct v4l2_subdev* sd = NULL;
	struct lt8918 *self = NULL;
	int ret = 0;

	self = devm_kzalloc(dev, sizeof(*self), GFP_KERNEL);
	if (!self)
		return -ENOMEM;

	self->i2c_client = client;

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
