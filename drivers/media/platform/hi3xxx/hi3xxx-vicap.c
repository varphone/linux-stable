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
#include <linux/clk.h>
#include <linux/clk-provider.h>
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
#include <linux/pm_runtime.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/media-device.h>

#include "hi3xxx-vicap.h"

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

static const struct media_entity_operations hi3xxx_vicap_sd_media_ops = {
	.link_setup = hi3xxx_vicap_link_setup,
};

static const struct v4l2_subdev_internal_ops hi3xxx_vicap_sd_internal_ops = {
	.registered = hi3xxx_vicap_subdev_registered,
	.unregistered = hi3xxx_vicap_subdev_unregistered,
};

int hi3xxx_vicap_initialize_subdev(struct hi3xxx_vicap *self)
{
	struct v4l2_subdev *sd = &self->v4l2_sd;
	int ret;

	v4l2_subdev_init(sd, &hi3xxx_vicap_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, sizeof(sd->name), "mxc_isi.%d", self->id);

	sd->entity.function = MEDIA_ENT_F_IO_V4L;
	self->sink_pads[0].flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&sd->entity, 1, &self->sink_pads);
	if (ret)
		return ret;

	sd->entity.ops = &hi3xxx_vicap_sd_media_ops;
	sd->internal_ops = &hi3xxx_vicap_sd_internal_ops;
	v4l2_set_subdevdata(sd, self);

	return 0;
}

void hi3xxx_vicap_unregister_subdev(struct hi3xxx_vicap *self)
{
	struct v4l2_subdev *sd = &self->v4l2_sd;

	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_set_subdevdata(sd, NULL);
}

static int hi3xxx_vicap_parse_dt(struct hi3xxx_vicap *self)
{
	struct device *dev = &self->pdev->dev;
	struct device_node *node = dev->of_node;
	int ret = 0;

	self->id = of_alias_get_id(node, "vicap");

	return 0;
}

static int hi3xxx_vicap_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hi3xxx_vicap *self;
	struct resource *res;
	const struct of_device_id *of_id;
	int ret = 0;

	self = devm_kzalloc(dev, sizeof(*self), GFP_KERNEL);
	if (!self)
		return -ENOMEM;

	self->pdev = pdev;

	ret = mxc_isi_parse_dt(self);
	if (ret < 0)
		return ret;

	if (self->id >= HI3XXX_VICAP_MAX_DEVS || self->id < 0) {
		dev_err(dev, "Invalid driver data or device id (%d)\n",
			self->id);
		return -EINVAL;
	}

	ret = hi3xxx_vicap_initialize_subdev(self);
	if (ret < 0) {
		dev_err(dev, "failed to init cap subdev (%d)\n", ret);
		goto err_clk;
	}

	platform_set_drvdata(pdev, self);

	dev_dbg(dev, "hi3xxx_vicap.%d registered successfully\n", self->id);

	return 0;

err_clk:
	mxc_isi_unregister_capture_subdev(self);
	return ret;
}

static int hi3xxx_vicap_remove(struct platform_device *pdev)
{
	struct hi3xxx_vicap *hi3xxx_vicap = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	hi3xxx_vicap_unregister_subdev(hi3xxx_vicap);

	return 0;
}

static const struct of_device_id hi3xxx_vicap_of_match[] = {
	{.compatible = "hisilicon,hi3xxx-vicap", .data = NULL },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, hi3xxx_vicap_of_match);

static struct platform_driver hi3xxx_vicap_driver = {
	.probe		= hi3xxx_vicap_probe,
	.remove		= hi3xxx_vicap_remove,
	.driver = {
		.of_match_table = hi3xxx_vicap_of_match,
		.name		= HI3XXX_VICAP_DRIVER_NAME,
	}
};

module_platform_driver(hi3xxx_vicap_driver);

MODULE_AUTHOR("CETC55, Technology Development CO.,LTD.");
MODULE_AUTHOR("Varphone Wong <varphone@qq.com>");
MODULE_DESCRIPTION("Video Input driver for Hi3XXX Platforms");
MODULE_ALIAS("platform:" HI3XXX_VICAP_DRIVER_NAME);
MODULE_LICENSE("GPL");
