/*
 * File: hi3xxx-vicap.c
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Video Input Capture driver for Hi3XXX Platforms
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Project       : hi3xxx
 * Author        : Varphone Wong <varphone@qq.com>
 * File Created  : 2019-11-20 04:06:33 Wednesday, 20 November
 * Last Modified : 2019-11-20 04:27:23 Wednesday, 20 November
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

#include "hi3xxx-vicap.h"
#include "hi3xxx-media-dev.h"

static int hi3xxx_vicap_enum_mbus_code(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_mbus_code_enum *code)
{
	return 0;
}

static int hi3xxx_vicap_enum_framesizes(struct v4l2_subdev *sd,
					struct v4l2_subdev_pad_config *cfg,
					struct v4l2_subdev_frame_size_enum *fse)
{
	return 0;
}

static int hi3xxx_vicap_enum_frame_interval(struct v4l2_subdev *sd,
					    struct v4l2_subdev_pad_config *cfg,
					    struct v4l2_subdev_frame_interval_enum *fie)
{
	return -EINVAL;
}

static int hi3xxx_vicap_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	return 0;
}

static int hi3xxx_vicap_set_fmt(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_format *fmt)
{
	return 0;
}

static int hi3xxx_vicap_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				       struct v4l2_mbus_frame_desc *fd)
{
	return 0;
}

static int hi3xxx_vicap_set_frame_desc(struct v4l2_subdev *sd,
				       unsigned int pad,
				       struct v4l2_mbus_frame_desc *fd)
{
	return 0;
}

static int hi3xxx_vicap_set_power(struct v4l2_subdev *sd, int on)
{
	return 0;
}

static int hi3xxx_vicap_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	return 0;
}

static int hi3xxx_vicap_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parm)
{
	// FIXME:
	return 0;
}

static int hi3xxx_vicap_s_stream(struct v4l2_subdev *sd, int enable)
{
	// FIXME:
	return 0;
}

static int hi3xxx_vicap_link_setup(struct media_entity *entity,
				   const struct media_pad *local,
				   const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct hi3xxx_vicap *self = v4l2_get_subdevdata(sd);

	if (WARN_ON(self == NULL))
		return 0;

	if (!(flags & MEDIA_LNK_FL_ENABLED)) {
		return 0;
	}

	return 0;
}

static const struct v4l2_subdev_pad_ops hi3xxx_vicap_pad_ops = {
	.enum_mbus_code         = hi3xxx_vicap_enum_mbus_code,
	.enum_frame_size        = hi3xxx_vicap_enum_framesizes,
	.enum_frame_interval    = hi3xxx_vicap_enum_frame_interval,
	.get_fmt                = hi3xxx_vicap_get_fmt,
	.set_fmt                = hi3xxx_vicap_set_fmt,
	.get_frame_desc         = hi3xxx_vicap_get_frame_desc,
	.set_frame_desc         = hi3xxx_vicap_set_frame_desc,
};

static const struct v4l2_subdev_core_ops hi3xxx_vicap_core_ops = {
	.s_power	= hi3xxx_vicap_set_power,
};

static const struct v4l2_subdev_video_ops hi3xxx_vicap_video_ops = {
	.s_parm		= hi3xxx_vicap_s_parm,
	.g_parm		= hi3xxx_vicap_g_parm,
	.s_stream	= hi3xxx_vicap_s_stream,
};

static const struct v4l2_subdev_ops hi3xxx_vicap_subdev_ops = {
	.core	= &hi3xxx_vicap_core_ops,
	.pad	= &hi3xxx_vicap_pad_ops,
	.video	= &hi3xxx_vicap_video_ops,
};

static const struct media_entity_operations hi3xxx_vicap_sd_media_ops = {
	.link_setup = hi3xxx_vicap_link_setup,
};

static int hi3xxx_vicap_add_to_async_subdevs(struct hi3xxx_vicap *self)
{
	struct hi3xxx_media* parent = self->parent;
	struct device *dev = &self->pdev->dev;
	struct device_node *node = dev->of_node;
	struct device_node *local = NULL;
	struct device_node *remote = NULL;
	struct hi3xxx_async_subdev* asd = NULL;
	int ret = 0;

	local = of_graph_get_next_endpoint(node, local);
	if (!local) {
		dev_err(dev, "Could not parse the endpoint\n");
		ret = -EINVAL;
		goto err_of_graph_get_next_endpoint;
	}

	remote = of_graph_get_remote_port_parent(local);
	if (!remote) {
		dev_err(dev, "Remote device at %s not found\n",
			local->full_name);
		ret = -EINVAL;
		goto err_of_graph_get_remote_port_parent;
	}

	asd = devm_kzalloc(dev, sizeof(*asd), GFP_KERNEL);
	if (!asd) {
		ret = -ENOMEM;
		goto err_devm_kzalloc;
	}

	asd->base.match_type = V4L2_ASYNC_MATCH_OF;
	asd->base.match.of.node = remote;
	asd->sd = &self->base;

	parent->async_subdevs[parent->num_async_subdevs++] = asd;

err_devm_kzalloc:
	of_node_put(remote);
err_of_graph_get_remote_port_parent:
	of_node_put(local);
err_of_graph_get_next_endpoint:
	return ret;
}

static int hi3xxx_vicap_parse_dt(struct hi3xxx_vicap *self)
{
	struct hi3xxx_media* parent = self->parent;
	struct device *dev = &self->pdev->dev;
	struct device_node *node = dev->of_node;
	int ret = 0;

	self->id = of_alias_get_id(node, "videv");

	return ret;
}

int hi3xxx_vicap_register(struct hi3xxx_vicap *self)
{
	struct device *dev = &self->pdev->dev;
	struct v4l2_subdev *sd;
	int ret = 0;

	ret = hi3xxx_vicap_parse_dt(self);
	if (ret < 0)
		return ret;

	if (self->id >= HI3XXX_VICAP_MAX_DEVS || self->id < 0) {
		dev_err(dev, "Invalid driver data or device id (%d)\n",
			self->id);
		return -EINVAL;
	}

	sd = &self->base;
	v4l2_subdev_init(sd, &hi3xxx_vicap_subdev_ops);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, sizeof(sd->name), "hi3xxx-vicap.%d", self->id);

	sd->entity.function = MEDIA_ENT_F_IO_V4L;
	self->pads[0].flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&sd->entity, 1, self->pads);
	if (ret) {
		v4l2_err(sd, "Failed to init media entity: %d\n", ret);
		goto err_media_entity_pads_init;
	}

	sd->entity.ops = &hi3xxx_vicap_sd_media_ops;
	ret = v4l2_device_register_subdev(&self->parent->v4l2_dev, sd);
	if (ret < 0) {
		v4l2_err(sd, "Failed to async register subdev: %d\n", ret);
		goto err_v4l2_device_register_subdev;
	}

	ret = hi3xxx_vicap_add_to_async_subdevs(self);
	if (ret < 0)
		goto err_hi3xxx_vicap_add_to_async_subdevs;

	v4l2_set_subdevdata(sd, self);
	platform_set_drvdata(self->pdev, self);

	dev_dbg(dev, "hi3xxx_vicap.%d registered successfully\n", self->id);

	return 0;
err_hi3xxx_vicap_add_to_async_subdevs:
	v4l2_device_unregister_subdev(sd);
err_v4l2_device_register_subdev:
	media_entity_cleanup(&sd->entity);
err_media_entity_pads_init:
	return ret;
}

int hi3xxx_vicap_unregister(struct hi3xxx_vicap *self)
{
	struct v4l2_subdev *sd = &self->base;

	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_set_subdevdata(sd, NULL);
	platform_set_drvdata(self->pdev, NULL);

	return 0;
}
