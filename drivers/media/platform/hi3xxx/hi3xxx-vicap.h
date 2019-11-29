/*
 * File: hi3xxx-vicap.h
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Video Input Capture driver for Hi3XXX Platforms
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Project       : hi3xxx
 * Author        : Varphone Wong <varphone@qq.com>
 * File Created  : 2019-11-20 06:49:42 Wednesday, 20 November
 * Last Modified : 2019-11-20 06:49:52 Wednesday, 20 November
 * Modified By   : Varphone Wong <varphone@qq.com>
 * ---------------------------------------------------------------------------
 * Copyright (C) 2012-2019 CETC55, Technology Development CO.,LTD.
 * Copyright (C) 2012-2019 Varphone Wong, Varphone.com.
 * All rights reserved.
 * ---------------------------------------------------------------------------
 * HISTORY:
 */
#ifndef HI3XXX_VICAP_H
#define HI3XXX_VICAP_H

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#define HI3XXX_VICAP_DRIVER_NAME "hi3xxx-vicap"
#define HI3XXX_VICAP_MAX_DEVS    8

struct hi3xxx_media;

struct hi3xxx_vicap {
	struct v4l2_subdev base;
	struct hi3xxx_media *parent;
	struct platform_device *pdev;
	struct v4l2_device *v4l2_dev;
	struct media_pad pads[1];
	struct mutex lock;
	u32 id;
};

/* Register the VICAP device
 * @note the `parent` and `pdev` must be by caller
 */
int hi3xxx_vicap_register(struct hi3xxx_vicap *self);
int hi3xxx_vicap_unregister(struct hi3xxx_vicap *self);

#endif /* HI3XXX_VICAP_H */
