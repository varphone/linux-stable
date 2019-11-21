/*
 * File: hi3xxx-media.c
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * <Add your brief here>
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Project       : hi3xxx
 * Author        : Varphone Wong <varphone@qq.com>
 * File Created  : 2019-11-20 03:09:09 Wednesday, 20 November
 * Last Modified : 2019-11-20 03:10:10 Wednesday, 20 November
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

#include "hi3xxx-media.h"

static int subdev_notifier_bound(struct v4l2_async_notifier *notifier,
				 struct v4l2_subdev *sd,
				 struct v4l2_async_subdev *asd)
{
	struct hi3xxx_media *self = container_of(notifier, struct hi3xxx_media, subdev_notifier);
	printk(KERN_INFO "notifier=%p, sd=%p, asd=%p\n", notifier, sd, asd);
	return 0;
}

static int subdev_notifier_complete(struct v4l2_async_notifier *notifier)
{
	struct hi3xxx_media *self = container_of(notifier, struct hi3xxx_media, subdev_notifier);
	printk(KERN_INFO "self=%p, notifier=%p\n", self, notifier);
	v4l2_device_register_subdev_nodes(&self->v4l2_dev);
	media_device_register(&self->mdev);
	return 0;
}

static void subdev_notify(struct v4l2_subdev *sd, unsigned int notification, void *arg)
{
	printk(KERN_INFO "sd=%p, notification=%u, arg=%p\n", sd, notification, arg);
}

static int hi3xxx_media_link_notify(struct media_link *link,
				    unsigned int flags,
				    unsigned int notification)
{
	return 0;
}

static const struct media_device_ops hi3xxx_media_ops = {
	.link_notify = hi3xxx_media_link_notify,
};

static int hi3xxx_media_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct v4l2_device *v4l2_dev;
	struct hi3xxx_media *self;
	int ret;

	self = devm_kzalloc(dev, sizeof(*self), GFP_KERNEL);
	if (!self)
		return -ENOMEM;

	self->pdev = pdev;
	platform_set_drvdata(pdev, self);

	/* Register media device  */
	strlcpy(self->media_dev.model, "Hi3XXX Media Controller",
		sizeof(self->media_dev.model));
	self->media_dev.ops = &hi3xxx_media_ops;
	self->media_dev.dev = dev;

	/* Register v4l2 device */
	v4l2_dev = &self->v4l2_dev;
	v4l2_dev->mdev = &self->media_dev;
	v4l2_dev->notify = subdev_notify;
	strlcpy(v4l2_dev->name, "hi3xxx-media", sizeof(v4l2_dev->name));

	media_device_init(&self->media_dev);

	ret = v4l2_device_register(dev, &self->v4l2_dev);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "Failed to register v4l2_device: %d\n", ret);
		goto err_v4l2_device_register;
	}

	self->num_sensors = 0;
	self->link_status = 0;

	v4l2_async_notifier_init(&self->subdev_notifier);

	self->subdev_notifier.bound = subdev_notifier_bound;
	self->subdev_notifier.complete = subdev_notifier_complete;

	ret = v4l2_async_notifier_register(&self->v4l2_dev,
					   &self->subdev_notifier);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "Failed to register v4l2_async_notifier: %d\n", ret);
		goto err_v4l2_device_register;
	}

	return 0;

err_v4l2_async_notifier_register:
	v4l2_device_unregister(&self->v4l2_dev);
err_v4l2_device_register:
	media_device_cleanup(&self->media_dev);
	return ret;
}

static int hi3xxx_media_remove(struct platform_device *pdev)
{
	struct hi3xxx_media *self = platform_get_drvdata(pdev);

	if (!self)
		return 0;

	v4l2_async_notifier_unregister(&self->subdev_notifier);

	v4l2_device_unregister(&self->v4l2_dev);
	media_device_unregister(&self->media_dev);
	media_device_cleanup(&self->media_dev);

	return 0;
}

static const struct of_device_id hi3xxx_media_of_match[] = {
	{ .compatible = "hisilicon,hi3xxx-media",},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, hi3xxx_media_of_match);


static struct platform_driver hi3xxx_media_driver = {
	.driver = {
		.name = HI3XXX_MEDIA_DRIVER_NAME,
		.of_match_table	= hi3xxx_media_of_match,
	},
	.probe = hi3xxx_media_probe,
	.remove = hi3xxx_media_remove,
};

module_platform_driver(hi3xxx_media_driver);

MODULE_AUTHOR("CETC55, Technology Development CO.,LTD.");
MODULE_AUTHOR("Varphone Wong <varphone@qq.com>");
MODULE_DESCRIPTION("Media Controller driver for Hisilicon Hi3XXX Platforms");
MODULE_ALIAS("platform:" HI3XXX_MEDIA_DRIVER_NAME);
MODULE_LICENSE("GPL");
