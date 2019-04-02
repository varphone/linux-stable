/*
 * ISL7998X - Four Channels MIPI/CSI-2 Reciever
 *
 * Copyright (C) 2019 NanJing No.55 Research Institute
 *                    Technology Development CO.,LTD
 *
 * Copyright (C) 2019 Varphone Wong <varphone@qq.com>
 *
 * All rights reserved.
 *
 */

#ifndef __ISL7998X_REGS_H__
#define __ISL7998X_REGS_H__

enum isl7998x_mode_id {
	ISL7998X_MODE_NTSC_720_240,
	ISL7998X_MODE_NTSC_720_480,
	ISL7998X_MODE_PAL_720_288,
	ISL7998X_MODE_PAL_720_576,
	ISL7998X_MODE_START_STREAM,
	ISL7998X_MODE_STOP_STREAM,
	ISL7998X_MODE_TEST_PATTERN,
	ISL7998X_NUM_MODES,
};

enum isl7998x_frame_rate {
	ISL7998X_25_FPS = 0,
	ISL7998X_30_FPS,
	ISL7998X_50_FPS,
	ISL7998X_60_FPS,
	ISL7998X_NUM_FRAMERATES,
};

struct isl7998x_mode_info {
	enum isl7998x_mode_id id;
	u32 fps;
	u32 width;
	u32 height;
	const struct reg_sequence *regs;
	u32 num_regs;
};

static const int isl7998x_framerates[] = {
	[ISL7998X_25_FPS] = 25,
	[ISL7998X_30_FPS] = 30,
	[ISL7998X_50_FPS] = 50,
	[ISL7998X_60_FPS] = 60,
};

/* ISL7998X_MODE_NTSC_720_240
 * Lanes=2
 */
__maybe_unused
static struct reg_sequence isl7998x_ntsc_720_240[] = {
	{0xFF, 0x00, 0},
};

/* ISL7998X_MODE_NTSC_720_480
 * Lanes=2
 */
__maybe_unused
static struct reg_sequence isl7998x_ntsc_720_480[] = {
	{0xFF, 0x00, 0},
};

/* ISL7998X_MODE_PAL_720_288
 * Lanes=2
 */
__maybe_unused
static struct reg_sequence isl7998x_pal_720_288[] = {
	{0xFF, 0x00, 0},
};

/* ISL7998X_MODE_PAL_720_576
 * Lanes=2
 */
__maybe_unused
static struct reg_sequence isl7998x_pal_720_576[] = {
	{0xFF, 0x05, 0},
	{0x00, 0x02, 0}, /* Clear MIPI PWDN */
	{0xFF, 0x00, 0},
	{0x02, 0x1F, 0}, /* Set Reset */
	/* Default settings */
	{0xFF, 0x00, 0},
	{0x03, 0x0C, 0},
	{0x0C, 0x00, 0},
	{0x0D, 0x00, 0},
	{0x0E, 0x00, 0},
	{0x10, 0x00, 0},
	{0x11, 0x00, 0},
	{0x12, 0x00, 0},
	{0x13, 0x00, 0},
	{0x14, 0x00, 0},
	{0x15, 0xFF, 0},
	/* Page 5 */
	{0xFF, 0x05, 0},
	{0x00, 0x02, 0},
	{0x01, 0x85, 0},
	{0x02, 0xA0, 0},
	{0x03, 0x18, 0},
	{0x04, 0xE4, 0},
	{0x05, 0x40, 0},
	{0x06, 0x40, 0},
	{0x10, 0x05, 0},
	{0x11, 0xA0, 0},
	{0x20, 0x00, 0},
	{0x21, 0x0C, 0},
	{0x2A, 0x00, 0},
	{0x2B, 0x00, 0},
	{0x2C, 0x01, 0},
	{0x2D, 0x04, 0},
	{0x2E, 0x01, 0},
	{0x2F, 0x05, 0},
	{0x30, 0x00, 0},
	{0x31, 0x00, 0},
	{0x32, 0x00, 0},
	{0x33, 0x00, 0},
	{0x34, 0x06, 0},
	{0x35, 0x0F, 0},
	{0x36, 0x00, 0},
	/* Page 0 */
	{0xFF, 0x00, 0},
	{0x02, 0x10, 0}, /* Clear CH1~CH4 Reset */
	/* Page 1/2/3/4 */
	{0xFF, 0x0F, 0},
	{0x44, 0x00, 0},
	{0x45, 0x11, 0},
	{0xE7, 0x00, 0},
	/* Page 0 */
	{0xFF, 0x00, 0},
	{0x07, 0x12, 0},
	{0x08, 0x1F, 0},
	{0x09, 0x43, 0},
	{0x0A, 0x4F, 0},
	{0x0B, 0x40, 0}, /* 2 Lanes */
	/* Page 1 */
	{0xFF, 0x01, 0},
	{0x02, 0x48, 0}, /* 27MHz Input */
	{0x07, 0x12, 0}, /* VA_HI[5:4]=1, HA_HI[1:0]=2 */
	{0x08, 0x18, 0}, /* VD_LO */
	{0x09, 0x20, 0}, /* VA_LO */
	{0x0A, 0x08, 0}, /* HD_LO */
	{0x0B, 0xD0, 0}, /* HA_LO */
	{0x0C, 0xCC, 0},
	{0x1C, 0x0F, 0},
	{0x1D, 0xFF, 0},
	{0x28, 0x00, 0},
	{0x29, 0xE0, 0},
	{0x2F, 0xE6, 0},
	{0x33, 0xC5, 0},
	{0x3B, 0x04, 0},
	{0x3D, 0x08, 0},
	{0xFF, 0x01, 0},
	/* Page 5 */
	{0xFF, 0x05, 0},
	{0x00, 0x01, 0},
	{0x01, 0x25, 0},  /* Frame Mode */
	{0x02, 0xA0, 0},
	{0x03, 0x18, 0},
	{0x04, 0xE4, 0},
	{0x05, 0x40, 0},
	{0x06, 0x00, 0},
	{0x08, 0x08, 0},
	{0x09, 0x00, 0},
	{0x0A, 0x62, 0},
	{0x0B, 0x02, 0},
	{0x0C, 0x36, 0},
	{0x0D, 0x00, 0},
	{0x0E, 0x6C, 0},
	{0x0F, 0x00, 0},
	{0x10, 0x05, 0},
	{0x11, 0xA0, 0},
	{0x12, 0x76, 0},
	{0x13, 0x17, 0},
	{0x14, 0x0E, 0},
	{0x15, 0x36, 0},
	{0x16, 0x12, 0},
	{0x17, 0xF6, 0},
	{0x18, 0x00, 0},
	{0x19, 0x03, 0},
	{0x1A, 0x0A, 0},
	{0x1B, 0x61, 0},
	{0x1C, 0x7A, 0},
	{0x1D, 0x0F, 0},
	{0x1E, 0x8C, 0},
	{0x1F, 0x06, 0},
	{0x28, 0x01, 0},
	{0x29, 0x0E, 0},
	{0x2A, 0x00, 0},
	{0x2B, 0x00, 0},
	{0x34, 0x18, 0},
	{0x35, 0x00, 0},
	{0x38, 0x03, 0},
	{0x39, 0xC0, 0},
	{0x3A, 0x06, 0},
	{0x3B, 0xB3, 0},
	{0x3C, 0x01, 0},
	{0x3D, 0x21, 0},
	/* Page 0 */
	{0xFF, 0x00, 0},
	{0x02, 0x00, 0},  /* Clear Reset */
};

/* ISL7998X_MODE_START_STREAM */
__maybe_unused
static struct reg_sequence isl7998x_start_stream[] = {
	{0xFF, 0x00, 0},
};

/* ISL7998X_MODE_STOP_STREAM */
__maybe_unused
static struct reg_sequence isl7998x_stop_stream[] = {
	{0xFF, 0x00, 0},
};

/* ISL7998X_MODE_TEST_PATTERN */
__maybe_unused
static struct reg_sequence isl7998x_test_pattern[] = {
	{0xFF, 0x00, 0},
};

__maybe_unused
static struct isl7998x_mode_info isl7998x_mode_tables[] = {
	{ /* ISL7998X_MODE_NTSC_720_240 */
		ISL7998X_MODE_NTSC_720_240, 60, 720, 240,
		isl7998x_ntsc_720_240, ARRAY_SIZE(isl7998x_ntsc_720_240)
	},
	{ /* ISL7998X_MODE_NTSC_720_480 */
		ISL7998X_MODE_NTSC_720_480, 30, 720, 480,
		isl7998x_ntsc_720_480, ARRAY_SIZE(isl7998x_ntsc_720_480)
	},
	{ /* ISL7998X_MODE_PAL_720_288 */
		ISL7998X_MODE_PAL_720_288, 50, 720, 288,
		isl7998x_pal_720_288, ARRAY_SIZE(isl7998x_pal_720_288)
	},
	{ /* ISL7998X_MODE_PAL_720_576 */
		ISL7998X_MODE_PAL_720_576, 25, 720, 576,
		isl7998x_pal_720_576, ARRAY_SIZE(isl7998x_pal_720_576)
	},
	{ /* ISL7998X_MODE_START_STREAM */
		ISL7998X_MODE_START_STREAM, 0, 0, 0,
		isl7998x_start_stream, ARRAY_SIZE(isl7998x_start_stream)
	},
	{ /* ISL7998X_MODE_STOP_STREAM */
		ISL7998X_MODE_STOP_STREAM, 0, 0, 0,
		isl7998x_stop_stream, ARRAY_SIZE(isl7998x_stop_stream)
	},
	{ /* ISL7998X_MODE_TEST_PATTERN */
		ISL7998X_MODE_TEST_PATTERN, 0, 0, 0,
		isl7998x_test_pattern, ARRAY_SIZE(isl7998x_test_pattern)
	},
};

#endif /* __ISL7998X_REGS_H__ */
