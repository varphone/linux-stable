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

#include "hi3xxx-media-dev.h"
#include "hi3xxx-vicap.h"

static int hi3xxx_media_register_nodes(struct hi3xxx_media *self, struct device_node *parent)
{
	int id = 0;
	int ret = 0;
	struct device_node *node;
	struct hi3xxx_vicap *vicap;

	for_each_available_child_of_node(parent, node) {
		struct platform_device *pdev;

		pdev = of_find_device_by_node(node);
		if (!pdev)
			continue;

		if (strcmp(node->name, "vicap") == 0) {
			id = of_alias_get_id(node, "videv");
			if (id < 0 || id > HI3XXX_MEDIA_MAX_VICAPS) {
				ret = -EINVAL;
			}
			else {
				vicap = &self->vicaps[id];
				vicap->parent = self;
				vicap->pdev = pdev;
				ret = hi3xxx_vicap_register(vicap);
			}
		}

		put_device(&pdev->dev);
		if (ret < 0)
			break;
	}

	return ret;
}

static void hi3xxx_media_unregister_nodes(struct hi3xxx_media *self, struct device_node *parent)
{
	int id = 0;
	int ret = 0;
	struct device_node *child;
	struct hi3xxx_vicap *vicap;

	for_each_available_child_of_node(parent, child) {
		struct platform_device *pdev;

		pdev = of_find_device_by_node(child);
		if (!pdev)
			continue;

		if (strcmp(child->name, "vicap") == 0) {
			id = of_alias_get_id(child, "videv");
			if (id < 0 || id > HI3XXX_MEDIA_MAX_VICAPS) {
				ret = -EINVAL;
			}
			else {
				vicap = &self->vicaps[id];
				ret = hi3xxx_vicap_unregister(vicap);
			}
		}

		put_device(&pdev->dev);
		of_node_put(child);

		if (ret < 0)
			break;
	}
}

static int subdev_notifier_bound(struct v4l2_async_notifier *notifier,
				 struct v4l2_subdev *sd,
				 struct v4l2_async_subdev *asd)
{
	struct hi3xxx_async_subdev* xasd = (struct hi3xxx_async_subdev*)asd;
	int i;

	for (i = 0; i < xasd->num_links; i++) {
		xasd->links[i].remote_sd = sd;
	}

	return 0;
}

static struct v4l2_subdev* find_refer_subdev(struct device_node* node,
	struct hi3xxx_async_subdev **async_subdevs, int num_async_subdevs)
{
	struct hi3xxx_async_subdev *xasd;
	int i;

	for (i = 0; i < num_async_subdevs; i++) {
		xasd = async_subdevs[i];
		if (xasd->base.match.of.node != node)
			continue;
		return xasd->links[0].remote_sd;
	}
	return NULL;
}

static int subdev_notifier_complete(struct v4l2_async_notifier *notifier)
{
	struct hi3xxx_media *self = container_of(notifier, struct hi3xxx_media, subdev_notifier);
	struct hi3xxx_async_subdev* xasd = NULL;
	struct hi3xxx_async_link* link = NULL;
	struct device* dev = &self->pdev->dev;
	struct media_entity* source;
	struct media_entity* sink;
	struct media_pad* source_pad;
	struct media_pad* sink_pad;
	struct v4l2_subdev* sd;
	int i, j, ret;

	for (i = 0; i < self->num_async_subdevs; i++) {
		xasd = self->async_subdevs[i];
		for (j = 0; j < xasd->num_links; j++) {
			link = &xasd->links[j];
			if (link->local_refer) {
				link->local_sd =
					find_refer_subdev(link->local_refer,
							  self->async_subdevs,
							  self->num_async_subdevs);
			}
			if (link->local_sd == NULL)
				continue;
			ret = media_create_pad_link(
				&link->remote_sd->entity,
				link->remote_port,
				&link->local_sd->entity,
				link->local_port,
				MEDIA_LNK_FL_IMMUTABLE/*|MEDIA_LNK_FL_ENABLED*/);
			if (ret < 0) {
				dev_err(dev, "Failed to create pad link, err: %d\n",
					ret);
				return ret;
			}
			if (link->local_sd->entity.pads[link->local_port].flags & MEDIA_PAD_FL_SOURCE) {
				source = &link->local_sd->entity;
				source_pad = &link->local_sd->entity.pads[link->local_port];
				sink = &link->remote_sd->entity;
				sink_pad = &link->remote_sd->entity.pads[link->remote_port];
			}
			else {
				sink = &link->local_sd->entity;
				sink_pad = &link->local_sd->entity.pads[link->local_port];
				source = &link->remote_sd->entity;
				source_pad = &link->remote_sd->entity.pads[link->remote_port];
			}
			dev_info(dev, "link_setup on %s\n", source->name);
			media_entity_call(source, link_setup, source_pad,
					  sink_pad, MEDIA_LNK_FL_ENABLED);
		}
	}

	ret = v4l2_device_register_subdev_nodes(&self->v4l2_dev);
	if (ret < 0) {
		dev_err(dev, "Failed to register subdev nodes, err: %d\n",
			ret);
		return ret;
	}
	ret = media_device_register(&self->media_dev);
	if (ret < 0) {
		dev_err(dev, "Failed to register media device, err: %d\n",
			ret);
		return ret;
	}

	dev_dbg(dev, "Media Device registered!\n");

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
	printk(KERN_INFO "link=%p, flags=%u, notification=%u\n", link, flags, notification);
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

	self->link_status = 0;
	self->num_async_subdevs = 0;

	ret = hi3xxx_media_register_nodes(self, dev->of_node);

	self->subdev_notifier.subdevs =
		(struct v4l2_async_subdev**)(self->async_subdevs);
	self->subdev_notifier.num_subdevs = self->num_async_subdevs;
	self->subdev_notifier.bound = subdev_notifier_bound;
	self->subdev_notifier.complete = subdev_notifier_complete;

	ret = v4l2_async_notifier_register(&self->v4l2_dev,
					   &self->subdev_notifier);
	if (ret < 0) {
		v4l2_err(v4l2_dev, "Failed to register v4l2_async_notifier: %d\n", ret);
		goto err_v4l2_async_notifier_register;
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

	hi3xxx_media_unregister_nodes(self, pdev->dev.of_node);
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
