/*
 * File: hi3xxx-media.h
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * V4L2 Media Device for Hisilicon Hi3XXX Platforms
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Project       : hisilicon
 * Author        : Varphone Wong <varphone@qq.com>
 * File Created  : 2019-11-20 03:01:59 Wednesday, 20 November
 * Last Modified : 2019-11-20 03:03:09 Wednesday, 20 November
 * Modified By   : Varphone Wong <varphone@qq.com>
 * ---------------------------------------------------------------------------
 * Copyright (C) 2012-2019 CETC55, Technology Development CO.,LTD.
 * Copyright (C) 2012-2019 Varphone Wong, Varphone.com.
 * All rights reserved.
 * ---------------------------------------------------------------------------
 * HISTORY:
 */
#ifndef HI3XXX_MEDIA_H
#define HI3XXX_MEDIA_H

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/bug.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/list.h>
#include <linux/mfd/syscon.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-subdev.h>

#define HI3XXX_MEDIA_DRIVER_NAME "hi3xxx-media"
#define HI3XXX_MEDIA_MAX_SENSORS 8

/*
 * The subdevices' group IDs.
 */
#define GRP_ID_HI3XXX_MEDIA_SENSOR       (1 << 8)
#define GRP_ID_HI3XXX_MEDIA_MIPI_CSI2    (1 << 9)
#define GRP_ID_HI3XXX_MEDIA_PARALLEL_CSI (1 << 10)

enum hi3xxx_media_subdev_index {
	IDX_SENSOR,
	IDX_MIPI_CSI2,
	IDX_PARALLEL_CSI,
	IDX_MAX,
};

struct hi3xxx_media {
	struct media_device media_dev;
	struct v4l2_device v4l2_dev;
	struct platform_device *pdev;

	int link_status;
	int num_sensors;

	struct v4l2_async_notifier subdev_notifier;
	struct v4l2_async_subdev *async_subdevs[HI3XXX_MEDIA_MAX_SENSORS];
};

#endif /* HI3XXX_MEDIA_H */
