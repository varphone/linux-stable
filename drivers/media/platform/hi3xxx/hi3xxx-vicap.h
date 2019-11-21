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

#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>

#define HI3XXX_VICAP_MAX_DEVS 8

struct hi3xxx_vicap {
	struct mutex lock;
	struct platform_device *pdev;
	struct v4l2_device *v4l2_dev;
	struct v4l2_subdev v4l2_sd;
	struct media_pad sink_pads[1];
};

#endif /* HI3XXX_VICAP_H */
