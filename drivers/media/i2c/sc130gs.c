// SPDX-License-Identifier: GPL-2.0
/*
 * sc130gs driver
 *
 * Copyright (C) 2020 Fuzhou Rockchip Electronics Co., Ltd.
 * Copyright (C) 2022 Fuzhou Full-V Smart Photoelectric Co., Ltd.
 *
 * V0.0X01.0X00 first version,adjust sc130gs.
 */

//#define DEBUG
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>
#include <linux/rk-preisp.h>

#define HAS_RKLASER_SYSFS		1

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x00)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define V4L2_CID_EXT_TRIGGER		(V4L2_CID_IMAGE_SOURCE_CLASS_BASE + 0x1001)

#define MIPI_FREQ_186M			186000000 // 371.25Mbps/lane
#define MIPI_FREQ_216M			432000000 // 432.00Mbps/lane
#define MIPI_FREQ_297M			297000000 // 594.00Mbps/lane
#define MIPI_FREQ_432M			432000000 // 864.00Mbps/lane

#define BITS_PER_SAMPLE			10
#define SC130GS_LANES			4
#define SC130GS_MAX_PIXEL_RATE		(MIPI_FREQ_432M * 2 / BITS_PER_SAMPLE * SC130GS_LANES)
#define OF_CAMERA_HDR_MODE		"rockchip,camera-hdr-mode"

#define SC130GS_XVCLK_FREQ		27000000

#define CHIP_ID				0x0130
#define SC130GS_REG_CHIP_ID		0x3107

#define SC130GS_REG_CTRL_MODE		0x0100
#define SC130GS_MODE_SW_STANDBY		0x0
#define SC130GS_MODE_STREAMING		BIT(0)

#define SC130GS_EXPOSURE_MIN		0
#define SC130GS_EXPOSURE_STEP		1 // Normal=1,HDR2=2,HDR3=3
#define SC130GS_VTS_MAX			0x7fff

//long exposure
#define SC130GS_REG_EXP_LONG_H		0x3e01    //[7:0]
#define SC130GS_REG_EXP_LONG_L		0x3e02    //[7:4]

#define SC130GS_REG_AGAIN_FINE		0x3e09

#define SC130GS_REG_CDGAIN		0x3e08
#define SC130GS_CGAIN_MASK		0x0c
#define SC130GS_DGAIN_MASK		0xe0

#define SC130GS_GAIN_MIN		0x0
#define SC130GS_GAIN_MAX		(64 * 64)
#define SC130GS_GAIN_STEP		1
#define SC130GS_GAIN_DEFAULT		0x0

#define SC130GS_SOFTWARE_RESET_REG	0x0103
#define SC130GS_REG_TEST_PATTERN	0x5040
#define SC130GS_TEST_PATTERN_ENABLE	0x80

#define SC130GS_REG_VTS			0x320e
#define SC130GS_FLIP_REG		0x3221
#define SC130GS_FLIP_MASK		0x60
#define SC130GS_MIRROR_MASK		0x06

#define REG_DELAY			0xFFFE
#define REG_NULL			0xFFFF

#define SC130GS_REG_VALUE_08BIT		1
#define SC130GS_REG_VALUE_16BIT		2
#define SC130GS_REG_VALUE_24BIT		3

// #define LONG_FRAME_MAX_EXP		4297
// #define SHORT_FRAME_MAX_EXP		260

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define SC130GS_NAME			"sc130gs"

#define SC130GS_DEF_MODE_GROUP		0
#define SC130GS_DEF_MODE_ID		0

static int default_group = SC130GS_DEF_MODE_GROUP;
module_param_named(group, default_group, int, 0644);
MODULE_PARM_DESC(group, "Default Mode Group (0-2)");

static int default_mode = SC130GS_DEF_MODE_ID;
module_param_named(mode, default_mode, int, 0644);
MODULE_PARM_DESC(mode, "Default Mode Id (0-1)");

static const char * const sc130gs_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define SC130GS_NUM_SUPPLIES ARRAY_SIZE(sc130gs_supply_names)

enum sc130gs_max_pad {
	PAD0,
	PAD1,
	PAD2,
	PAD3,
	PAD_MAX,
};

struct regval {
	u16 addr;
	u8 val;
};

struct sc130gs_mode {
	u32 bus_fmt;
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
	u32 hdr_mode;
	u32 mipi_freq_idx;
	u32 bpp;
	u32 vc[PAD_MAX];
};

struct sc130gs_mode_group {
	int num;
	const struct sc130gs_mode *modes;
};

struct sc130gs {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[SC130GS_NUM_SUPPLIES];
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;
	struct pinctrl_state	*pins_sleep;
	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*anal_gain;
	struct v4l2_ctrl	*digi_gain;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*test_pattern;
	struct v4l2_ctrl	*pixel_rate;
	struct v4l2_ctrl	*link_freq;
	struct v4l2_ctrl	*ext_trigger;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct sc130gs_mode *cur_mode;
	u32			cfg_num;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	bool			has_init_exp;
	u32			cur_vts;
	struct preisp_hdrae_exp_s init_hdrae_exp;
};

#define to_sc130gs(sd) container_of(sd, struct sc130gs, subdev)

/*
 * Xclk 27Mhz linear 10bit 1280x1024 120fps 504Mbps/lane
 */
static const struct regval sc130gs_linear_10_1280x1024_120fps_regs[] = {
	{0x0103, 0x01}, // soft reset
	{REG_DELAY, 5},
	{0x0100, 0x00},
	{0x3000, 0x00},
	{0x3001, 0x00},
	{0x3018, 0x70},
	{0x3019, 0x00},
	{0x3022, 0x10},
	{0x302b, 0x80},
	{0x3030, 0x04},
	{0x3031, 0x0a},
	{0x3034, 0x01},
	{0x3035, 0xd2},
	{0x3038, 0x44},
	{0x3039, 0x14},
	{0x303a, 0x32},
	{0x303b, 0x02},
	{0x303c, 0x04},
	{0x303f, 0x11},
	{0x3202, 0x00},
	{0x3203, 0x00},
	{0x3205, 0x8b},
	{0x3206, 0x02},
	{0x3207, 0x04},
	{0x320a, 0x04},
	{0x320b, 0x00},
	{0x320c, 0x03},
	{0x320d, 0x1e},
	{0x320e, 0x02},
	{0x320f, 0x0e},
	{0x3211, 0x0c},
	{0x3213, 0x04},
	{0x3300, 0x20},
	{0x3302, 0x0c},
	{0x3306, 0x48},
	{0x3308, 0x50},
	{0x330a, 0x01},
	{0x330b, 0x20},
	{0x330e, 0x1a},
	{0x3310, 0xf0},
	{0x3311, 0x10},
	{0x3319, 0xe8},
	{0x3333, 0x90},
	{0x3334, 0x30},
	{0x3348, 0x02},
	{0x3349, 0xee},
	{0x334a, 0x02},
	{0x334b, 0xe8},
	{0x335d, 0x00},
	{0x3380, 0xff},
	{0x3382, 0xe0},
	{0x3383, 0x0a},
	{0x3384, 0xe4},
	{0x3400, 0x53},
	{0x3416, 0x31},
	{0x3518, 0x07},
	{0x3519, 0xc8},
	{0x3620, 0x23},
	{0x3621, 0x08},
	{0x3622, 0x06},
	{0x3623, 0x14},
	{0x3624, 0x20},
	{0x3625, 0x00},
	{0x3627, 0x01},
	{0x3630, 0x63},
	{0x3632, 0x74},
	{0x3633, 0x62},
	{0x3634, 0xff},
	{0x3635, 0x44},
	{0x3638, 0x82},
	{0x3639, 0x74},
	{0x363a, 0x24},
	{0x363b, 0x00},
	{0x3640, 0x03},
	{0x3658, 0x9a},
	{0x3663, 0x88},
	{0x3664, 0x07},
	{0x3c00, 0x41},
	{0x3d08, 0x00},
	{0x3e01, 0x20},
	{0x3e02, 0x50},
	{0x3e03, 0x0b},
	{0x3e08, 0x03},
	{0x3e09, 0x20},
	{0x3e0e, 0x00},
	{0x3e0f, 0x14},
	{0x3e14, 0xb0},
	{0x3f08, 0x04},
	{0x4501, 0xc0},
	{0x4502, 0x16},
	{0x5000, 0x01},
	{0x5b00, 0x02},
	{0x5b01, 0x03},
	{0x5b02, 0x01},
	{0x5b03, 0x01},
	{0x0100, 0x01},
	{REG_NULL, 0x00},
};

/*
 * Xclk 27Mhz linear 10bit 1280x1024 210fps 594Mbps/lane
 */
static const struct regval sc130gs_linear_10_1280x1024_210fps_regs[] = {
	{0x0103, 0x01}, // soft reset
	// {REG_DELAY, 20},
	{0x0100, 0x00},
	{0x3000, 0x00},
	{0x3001, 0x00},
	{0x3018, 0x70},
	{0x3019, 0x00},
	{0x3022, 0x10},
	{0x302b, 0x80},
	{0x3030, 0x01},
	{0x3031, 0x0a},
	{0x3034, 0x01},
	{0x3035, 0xd2},
	{0x3038, 0x4b},
	{0x3039, 0x10},
	{0x303a, 0x34},
	{0x303b, 0x0e},
	{0x303c, 0x05},
	{0x303f, 0x11},
	{0x3202, 0x00},
	{0x3203, 0x00},
	{0x3205, 0x8b},
	{0x3206, 0x02},
	{0x3207, 0x04},
	{0x3208, 0x05},
	{0x3209, 0x00},
	{0x320a, 0x04},
	{0x320b, 0x00},
	{0x320c, 0x03},
	{0x320d, 0x0c},
	{0x320e, 0x02},
	{0x320f, 0x0f},
	{0x3210, 0x00},
	{0x3211, 0x0c},
	{0x3212, 0x00},
	{0x3213, 0x04},
	{0x3234, 0x00},
	{0x3300, 0x20},
	{0x3302, 0x0c},
	{0x3306, 0x48},
	{0x3308, 0x50},
	{0x330a, 0x01},
	{0x330b, 0x10},
	{0x330e, 0x1a},
	{0x3310, 0xf0},
	{0x3311, 0x10},
	{0x3319, 0xe8},
	{0x3333, 0x90},
	{0x3334, 0x30},
	{0x3348, 0x02},
	{0x3349, 0xee},
	{0x334a, 0x02},
	{0x334b, 0xe0},
	{0x335d, 0x00},
	{0x3380, 0xff},
	{0x3382, 0xe0},
	{0x3383, 0x0a},
	{0x3384, 0xe4},
	{0x3400, 0x53},
	{0x3416, 0x31},
	{0x3518, 0x07},
	{0x3519, 0xc8},
	{0x3620, 0x24},
	{0x3621, 0x0a},
	{0x3622, 0x06},
	{0x3623, 0x14},
	{0x3624, 0x20},
	{0x3625, 0x00},
	{0x3626, 0x00},
	{0x3627, 0x01},
	{0x3630, 0x63},
	{0x3632, 0x74},
	{0x3633, 0x63},
	{0x3634, 0xff},
	{0x3635, 0x44},
	{0x3638, 0x82},
	{0x3639, 0x74},
	{0x363a, 0x24},
	{0x363b, 0x00},
	{0x3640, 0x03},
	{0x3658, 0x9a},
	{0x3663, 0x88},
	{0x3664, 0x06},
	{0x3c00, 0x41},
	{0x3d08, 0x00},
	{0x3225, 0x02},
	{0x3e01, 0x20},
	{0x3e02, 0x50},
	{0x3e03, 0x0b},
	{0x3e08, 0x02},
	{0x3e09, 0x20},
	{0x3e0e, 0x00},
	{0x3e0f, 0x15},
	{0x3e14, 0xb0},
	{0x3f08, 0x04},
	{0x4501, 0xc0},
	{0x4502, 0x16},
	{0x5000, 0x01},
	{0x5b00, 0x02},
	{0x5b01, 0x03},
	{0x5b02, 0x01},
	{0x5b03, 0x01},
	{0x0100, 0x01},
	{0x363a, 0x24},
	{0x3630, 0x63},
	{REG_NULL, 0x00},
};

/*
 * Xclk 27Mhz linear 10bit 1280x1024 210fps 594Mbps/lane external trigger
 */
static const struct regval sc130gs_linear_10_1280x1024_210fps_slave_regs[] = {
	{0x0103, 0x01}, // soft reset
	// {REG_DELAY, 20},
	{0x0100, 0x00},
	{0x3000, 0x00},
	{0x3001, 0x00},
	{0x3018, 0x70},
	{0x3019, 0x00},
	{0x3022, 0x10},
	{0x302b, 0x80},
	{0x3030, 0x01},
	{0x3031, 0x0a},
	{0x3034, 0x01},
	{0x3035, 0xd2},
	{0x3038, 0x4b},
	{0x3039, 0x10},
	{0x303a, 0x34},
	{0x303b, 0x0e},
	{0x303c, 0x05},
	{0x303f, 0x11},
	{0x3202, 0x00},
	{0x3203, 0x00},
	{0x3205, 0x8b},
	{0x3206, 0x02},
	{0x3207, 0x04},
	{0x3208, 0x05},
	{0x3209, 0x00},
	{0x320a, 0x04},
	{0x320b, 0x00},
	{0x320c, 0x03},
	{0x320d, 0x0c},
	{0x320e, 0x02},
	{0x320f, 0x0f},
	{0x3210, 0x00},
	{0x3211, 0x0c},
	{0x3212, 0x00},
	{0x3213, 0x04},
	{0x3234, 0x00},
	{0x3300, 0x20},
	{0x3302, 0x0c},
	{0x3306, 0x48},
	{0x3308, 0x50},
	{0x330a, 0x01},
	{0x330b, 0x10},
	{0x330e, 0x1a},
	{0x3310, 0xf0},
	{0x3311, 0x10},
	{0x3319, 0xe8},
	{0x3333, 0x90},
	{0x3334, 0x30},
	{0x3348, 0x02},
	{0x3349, 0xee},
	{0x334a, 0x02},
	{0x334b, 0xe0},
	{0x335d, 0x00},
	{0x3380, 0xff},
	{0x3382, 0xe0},
	{0x3383, 0x0a},
	{0x3384, 0xe4},
	{0x3400, 0x53},
	{0x3416, 0x31},
	{0x3518, 0x07},
	{0x3519, 0xc8},
	{0x3620, 0x24},
	{0x3621, 0x0a},
	{0x3622, 0x06},
	{0x3623, 0x14},
	{0x3624, 0x20},
	{0x3625, 0x00},
	{0x3626, 0x00},
	{0x3627, 0x01},
	{0x3630, 0x63},
	{0x3632, 0x74},
	{0x3633, 0x63},
	{0x3634, 0xff},
	{0x3635, 0x44},
	{0x3638, 0x82},
	{0x3639, 0x74},
	{0x363a, 0x24},
	{0x363b, 0x00},
	{0x3640, 0x03},
	{0x3658, 0x9a},
	{0x3663, 0x88},
	{0x3664, 0x06},
	{0x3c00, 0x41},
	{0x3d08, 0x00},
	{0x3e01, 0x20},
	{0x3e02, 0x50},
	{0x3e03, 0x0b},
	{0x3e08, 0x02},
	{0x3e09, 0x20},
	{0x3e0e, 0x00},
	{0x3e0f, 0x15},
	{0x3e14, 0xb0},
	{0x3f08, 0x04},
	{0x4501, 0xc0},
	{0x4502, 0x16},
	{0x5000, 0x01},
	{0x5b00, 0x02},
	{0x5b01, 0x03},
	{0x5b02, 0x01},
	{0x5b03, 0x01},
	// External Trigger
	{0x3234, 0xa3},
	// GAIN < 4
	{0x363a, 0x24},
	{0x3630, 0x63},
	//
	{0x0100, 0x01},
	{REG_NULL, 0x00},
};

/*
 * The width and height must be configured to be
 * the same as the current output resolution of the sensor.
 * The input width of the isp needs to be 16 aligned.
 * The input height of the isp needs to be 8 aligned.
 * If the width or height does not meet the alignment rules,
 * you can configure the cropping parameters with the following function to
 * crop out the appropriate resolution.
 * struct v4l2_subdev_pad_ops {
 *	.get_selection
 * }
 */

static const struct sc130gs_mode modes_1280_1024_210_10bit_bg10[] = {
	{
		/* linear modes */
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.width = 1280,
		.height = 1024,
		.max_fps = {
			.numerator = 10000,
			.denominator = 2100000,
		},
		.exp_def = 0x20b / 2,
		.hts_def = 0x30c * 2, // REG{0x320c,0x320d}
		.vts_def = 0x20f * 2, // REG{0x320e,0x320f}
		.reg_list = sc130gs_linear_10_1280x1024_210fps_regs,
		.hdr_mode = NO_HDR,
		.mipi_freq_idx = 3, // 594.00 Mbps
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
};

static const struct sc130gs_mode modes_1280_1024_120_10bit[] = {
	{
		/* linear modes */
		.bus_fmt = MEDIA_BUS_FMT_Y10_1X10,
		.width = 1280,
		.height = 1024,
		.max_fps = {
			.numerator = 10000,
			.denominator = 1200000,
		},
		.exp_def = 0x20a / 2,
		.hts_def = 0x31e * 2, // REG{0x320c,0x320d}
		.vts_def = 0x20e * 2, // REG{0x320e,0x320f}
		.reg_list = sc130gs_linear_10_1280x1024_120fps_regs,
		.hdr_mode = NO_HDR,
		.mipi_freq_idx = 2, // 504.00 Mbps
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
};

static const struct sc130gs_mode modes_1280_1024_210_10bit[] = {
	{
		/* linear modes */
		.bus_fmt = MEDIA_BUS_FMT_Y10_1X10,
		.width = 1280,
		.height = 1024,
		.max_fps = {
			.numerator = 10000,
			.denominator = 2100000,
		},
		.exp_def = 0x20b / 2,
		.hts_def = 0x30c * 2, // REG{0x320c,0x320d}
		.vts_def = 0x20f * 2, // REG{0x320e,0x320f}
		.reg_list = sc130gs_linear_10_1280x1024_210fps_regs,
		.hdr_mode = NO_HDR,
		.mipi_freq_idx = 3, // 594.00 Mbps
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
};

static const struct sc130gs_mode modes_1280_1024_210_10bit_slave[] = {
	{
		/* linear modes */
		.bus_fmt = MEDIA_BUS_FMT_Y10_1X10,
		.width = 1280,
		.height = 1024,
		.max_fps = {
			.numerator = 10000,
			.denominator = 2100000,
		},
		.exp_def = 0x20b / 2,
		.hts_def = 0x30c * 2, // REG{0x320c,0x320d}
		.vts_def = 0x20f * 2, // REG{0x320e,0x320f}
		.reg_list = sc130gs_linear_10_1280x1024_210fps_regs,
		.hdr_mode = NO_HDR,
		.mipi_freq_idx = 3, // 594.00 Mbps
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
};

static const struct sc130gs_mode_group supported_mode_groups[] = {
	{ ARRAY_SIZE(modes_1280_1024_210_10bit_bg10), modes_1280_1024_210_10bit_bg10 },
	{ ARRAY_SIZE(modes_1280_1024_120_10bit), modes_1280_1024_120_10bit },
	{ ARRAY_SIZE(modes_1280_1024_210_10bit), modes_1280_1024_210_10bit },
	{ ARRAY_SIZE(modes_1280_1024_210_10bit_slave), modes_1280_1024_210_10bit_slave },
};

static const s64 link_freq_items[] = {
	MIPI_FREQ_186M,
	MIPI_FREQ_216M,
	MIPI_FREQ_297M,
	MIPI_FREQ_432M,
};

static const char * const sc130gs_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1"
};

/* Write registers up to 4 at a time */
static int sc130gs_write_reg(struct i2c_client *client, u16 reg, u32 len,
			     u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;
	int ret = 0;

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	ret = i2c_master_send(client, buf, len + 2);

	if (reg != 0x0103) {
		if (ret != len + 2) {
			dev_info(&client->dev,
				 "sc130gs_write_reg(0x%04hx, %u, %u) = %d\n",
				 reg, len, val, ret);
			return -EIO;
		}
	}

	return 0;
}

static int sc130gs_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
		ret |= sc130gs_write_reg(client, regs[i].addr,
					 SC130GS_REG_VALUE_08BIT, regs[i].val);
		ret = ret == 0x0103 ? 0 : ret;
		if (ret != 0) {
			dev_info(&client->dev,
				 "sc130gs_write_reg(0x%04hx, 0x%02hhx) @ %d\n",
				 regs[i].addr, regs[i].val, i);
		}
	}
	return ret;
}

/* Read registers up to 4 at a time */
static int sc130gs_read_reg(struct i2c_client *client, u16 reg,
			    unsigned int len, u32 *val)
{
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;

	if (len > 4 || !len)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

static int sc130gs_get_reso_dist(const struct sc130gs_mode *mode,
				 struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct sc130gs_mode *
sc130gs_find_best_fit(struct sc130gs *sc130gs, struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	const struct sc130gs_mode_group *mg =
		&supported_mode_groups[default_group];
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < sc130gs->cfg_num; i++) {
		dist = sc130gs_get_reso_dist(&mg->modes[i], framefmt);
		if ((cur_best_fit_dist == -1 || dist <= cur_best_fit_dist) &&
		    (mg->modes[i].bus_fmt == framefmt->code)) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &mg->modes[cur_best_fit];
}

static void sc130gs_change_mode(struct sc130gs *sc130gs,
				const struct sc130gs_mode *mode)
{
	sc130gs->cur_mode = mode;
	sc130gs->cur_vts = sc130gs->cur_mode->vts_def;
	dev_info(&sc130gs->client->dev,
		 "set fmt: cur_mode: %dx%d@%d/%d, hdr: %d\n", mode->width,
		 mode->height, mode->max_fps.numerator,
		 mode->max_fps.denominator, mode->hdr_mode);
}

static int sc130gs_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct sc130gs *sc130gs = to_sc130gs(sd);
	const struct sc130gs_mode *mode;
	s64 h_blank, vblank_def;
	u64 pixel_rate = 0;

	mutex_lock(&sc130gs->mutex);

	mode = sc130gs_find_best_fit(sc130gs, fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&sc130gs->mutex);
		return -ENOTTY;
#endif
	} else {
		sc130gs_change_mode(sc130gs, mode);
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(sc130gs->hblank, h_blank, h_blank, 1,
					 h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(sc130gs->vblank, vblank_def,
					 SC130GS_VTS_MAX - mode->height, 1,
					 vblank_def);
		__v4l2_ctrl_s_ctrl(sc130gs->link_freq, mode->mipi_freq_idx);
		pixel_rate = (u32)link_freq_items[mode->mipi_freq_idx] /
			     mode->bpp * 2 * SC130GS_LANES;
		__v4l2_ctrl_s_ctrl_int64(sc130gs->pixel_rate, pixel_rate);
	}

	mutex_unlock(&sc130gs->mutex);

	return 0;
}

static int sc130gs_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct sc130gs *sc130gs = to_sc130gs(sd);
	const struct sc130gs_mode *mode = sc130gs->cur_mode;

	mutex_lock(&sc130gs->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&sc130gs->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = mode->bus_fmt;
		fmt->format.field = V4L2_FIELD_NONE;
		if (fmt->pad < PAD_MAX && mode->hdr_mode != NO_HDR)
			fmt->reserved[0] = mode->vc[fmt->pad];
		else
			fmt->reserved[0] = mode->vc[PAD0];
	}
	mutex_unlock(&sc130gs->mutex);

	return 0;
}

static int sc130gs_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct sc130gs *sc130gs = to_sc130gs(sd);

	if (code->index != 0)
		return -EINVAL;
	code->code = sc130gs->cur_mode->bus_fmt;

	return 0;
}

static int sc130gs_enum_frame_sizes(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	struct sc130gs *sc130gs = to_sc130gs(sd);
	const struct sc130gs_mode_group *mg =
		&supported_mode_groups[default_group];

	if (fse->index >= sc130gs->cfg_num)
		return -EINVAL;

	if (fse->code != mg->modes[fse->index].bus_fmt)
		return -EINVAL;

	fse->min_width = mg->modes[fse->index].width;
	fse->max_width = mg->modes[fse->index].width;
	fse->max_height = mg->modes[fse->index].height;
	fse->min_height = mg->modes[fse->index].height;

	return 0;
}

static int sc130gs_enable_test_pattern(struct sc130gs *sc130gs, u32 pattern)
{
	u32 val = 0;
	int ret = 0;

	ret = sc130gs_read_reg(sc130gs->client, SC130GS_REG_TEST_PATTERN,
			       SC130GS_REG_VALUE_08BIT, &val);
	if (pattern)
		val |= SC130GS_TEST_PATTERN_ENABLE;
	else
		val &= ~SC130GS_TEST_PATTERN_ENABLE;
	ret |= sc130gs_write_reg(sc130gs->client, SC130GS_REG_TEST_PATTERN,
				 SC130GS_REG_VALUE_08BIT, val);
	return ret;
}

static int sc130gs_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct sc130gs *sc130gs = to_sc130gs(sd);
	const struct sc130gs_mode *mode = sc130gs->cur_mode;

	mutex_lock(&sc130gs->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&sc130gs->mutex);

	return 0;
}

static int sc130gs_g_mbus_config(struct v4l2_subdev *sd,
				 struct v4l2_mbus_config *config)
{
	struct sc130gs *sc130gs = to_sc130gs(sd);
	const struct sc130gs_mode *mode = sc130gs->cur_mode;
	u32 val = 0;

	if (mode->hdr_mode == NO_HDR)
		val = 1 << (SC130GS_LANES - 1) | V4L2_MBUS_CSI2_CHANNEL_0 |
		      V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	if (mode->hdr_mode == HDR_X2)
		val = 1 << (SC130GS_LANES - 1) | V4L2_MBUS_CSI2_CHANNEL_0 |
		      V4L2_MBUS_CSI2_CONTINUOUS_CLOCK |
		      V4L2_MBUS_CSI2_CHANNEL_1;

	config->type = V4L2_MBUS_CSI2;
	config->flags = val;

	return 0;
}

static void sc130gs_get_module_inf(struct sc130gs *sc130gs,
				   struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, SC130GS_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, sc130gs->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, sc130gs->len_name, sizeof(inf->base.lens));
}

static void sc130gs_get_gain_reg(u32 val, u32 *again_reg, u32 *again_fine_reg,
				 u32 *dgain_reg)
{
	// FIXME:
	*again_reg = 0x00;
	*again_fine_reg = 0x20;
	*dgain_reg = 0x00;
}

static int sc130gs_set_hdrae(struct sc130gs *sc130gs,
			     struct preisp_hdrae_exp_s *ae)
{
	// FIXME:
	return 0;
}

static long sc130gs_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct sc130gs *sc130gs = to_sc130gs(sd);
	struct rkmodule_hdr_cfg *hdr_cfg;
	const struct sc130gs_mode *mode;
	const struct sc130gs_mode_group *mg =
		&supported_mode_groups[default_group];
	long ret = 0;
	u64 pixel_rate = 0;
	u32 i, h, w, stream;

	switch (cmd) {
	case PREISP_CMD_SET_HDRAE_EXP:
		if (sc130gs->cur_mode->hdr_mode == HDR_X2)
			ret = sc130gs_set_hdrae(sc130gs, arg);
		break;
	case RKMODULE_GET_MODULE_INFO:
		sc130gs_get_module_inf(sc130gs, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		hdr_cfg->esp.mode = HDR_NORMAL_VC;
		hdr_cfg->hdr_mode = sc130gs->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		w = sc130gs->cur_mode->width;
		h = sc130gs->cur_mode->height;
		for (i = 0; i < sc130gs->cfg_num; i++) {
			if (w == mg->modes[i].width &&
			    h == mg->modes[i].height &&
			    mg->modes[i].hdr_mode == hdr_cfg->hdr_mode) {
				sc130gs_change_mode(sc130gs, &mg->modes[i]);
				break;
			}
		}
		if (i == sc130gs->cfg_num) {
			dev_err(&sc130gs->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr_cfg->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			mode = sc130gs->cur_mode;
			if (sc130gs->streaming) {
				ret = sc130gs_write_array(sc130gs->client,
							  mode->reg_list);
				if (ret)
					return ret;
			}
			w = mode->hts_def - mode->width;
			h = mode->vts_def - mode->height;
			__v4l2_ctrl_modify_range(sc130gs->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(sc130gs->vblank, h,
						 SC130GS_VTS_MAX - mode->height,
						 1, h);
			__v4l2_ctrl_s_ctrl(sc130gs->link_freq,
					   mode->mipi_freq_idx);
			pixel_rate = (u32)link_freq_items[mode->mipi_freq_idx] /
				     mode->bpp * 2 * SC130GS_LANES;
			__v4l2_ctrl_s_ctrl_int64(sc130gs->pixel_rate,
						 pixel_rate);
			dev_info(&sc130gs->client->dev, "sensor mode: %d\n",
				 mode->hdr_mode);
		}
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (stream)
			ret = sc130gs_write_reg(sc130gs->client,
						SC130GS_REG_CTRL_MODE,
						SC130GS_REG_VALUE_08BIT,
						SC130GS_MODE_STREAMING);
		else
			ret = sc130gs_write_reg(sc130gs->client,
						SC130GS_REG_CTRL_MODE,
						SC130GS_REG_VALUE_08BIT,
						SC130GS_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long sc130gs_compat_ioctl32(struct v4l2_subdev *sd, unsigned int cmd,
				   unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	struct rkmodule_hdr_cfg *hdr;
	struct preisp_hdrae_exp_s *hdrae;
	long ret = 0;
	u32 cg = 0;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = sc130gs_ioctl(sd, cmd, inf);
		if (!ret) {
			if (copy_to_user(up, inf, sizeof(*inf))) {
				kfree(inf);
				return -EFAULT;
			}
		}
		kfree(inf);
		break;
	case RKMODULE_AWB_CFG:
		cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
		if (!cfg) {
			ret = -ENOMEM;
			return ret;
		}

		if (copy_from_user(cfg, up, sizeof(*cfg))) {
			kfree(cfg);
			return -EFAULT;
		}
		ret = sc130gs_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = sc130gs_ioctl(sd, cmd, hdr);
		if (!ret) {
			if (copy_to_user(up, hdr, sizeof(*hdr))) {
				kfree(hdr);
				return -EFAULT;
			}
		}
		kfree(hdr);
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		if (copy_from_user(hdr, up, sizeof(*hdr))) {
			kfree(hdr);
			return -EFAULT;
		}
		ret = sc130gs_ioctl(sd, cmd, hdr);
		kfree(hdr);
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		hdrae = kzalloc(sizeof(*hdrae), GFP_KERNEL);
		if (!hdrae) {
			ret = -ENOMEM;
			return ret;
		}

		if (copy_from_user(hdrae, up, sizeof(*hdrae))) {
			kfree(hdrae);
			return -EFAULT;
		}
		ret = sc130gs_ioctl(sd, cmd, hdrae);
		kfree(hdrae);
		break;
	case RKMODULE_SET_CONVERSION_GAIN:
		if (copy_from_user(&cg, up, sizeof(cg)))
			return -EFAULT;
		ret = sc130gs_ioctl(sd, cmd, &cg);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		if (copy_from_user(&stream, up, sizeof(u32)))
			return -EFAULT;
		ret = sc130gs_ioctl(sd, cmd, &stream);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __sc130gs_start_stream(struct sc130gs *sc130gs)
{
	int ret;

	ret = sc130gs_write_array(sc130gs->client, sc130gs->cur_mode->reg_list);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_handler_setup(&sc130gs->ctrl_handler);
	if (ret)
		return ret;

	/* In case these controls are set before streaming */
	if (sc130gs->has_init_exp && sc130gs->cur_mode->hdr_mode != NO_HDR) {
		ret = sc130gs_ioctl(&sc130gs->subdev, PREISP_CMD_SET_HDRAE_EXP,
				    &sc130gs->init_hdrae_exp);
		if (ret) {
			dev_err(&sc130gs->client->dev,
				"init exp fail in hdr mode\n");
			return ret;
		}
	}

	return sc130gs_write_reg(sc130gs->client, SC130GS_REG_CTRL_MODE,
				 SC130GS_REG_VALUE_08BIT,
				 SC130GS_MODE_STREAMING);
}

static int __sc130gs_stop_stream(struct sc130gs *sc130gs)
{
	sc130gs->has_init_exp = false;
	return sc130gs_write_reg(sc130gs->client, SC130GS_REG_CTRL_MODE,
				 SC130GS_REG_VALUE_08BIT,
				 SC130GS_MODE_SW_STANDBY);
}

static int sc130gs_s_stream(struct v4l2_subdev *sd, int on)
{
	struct sc130gs *sc130gs = to_sc130gs(sd);
	struct i2c_client *client = sc130gs->client;
	int ret = 0;

	mutex_lock(&sc130gs->mutex);
	on = !!on;
	if (on == sc130gs->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __sc130gs_start_stream(sc130gs);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__sc130gs_stop_stream(sc130gs);
		pm_runtime_put(&client->dev);
	}

	sc130gs->streaming = on;

unlock_and_return:
	mutex_unlock(&sc130gs->mutex);

	return ret;
}

static int sc130gs_s_power(struct v4l2_subdev *sd, int on)
{
	struct sc130gs *sc130gs = to_sc130gs(sd);
	struct i2c_client *client = sc130gs->client;
	int ret = 0;

	mutex_lock(&sc130gs->mutex);

	/* If the power state is not modified - no work to do. */
	if (sc130gs->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret |= sc130gs_write_reg(sc130gs->client,
					 SC130GS_SOFTWARE_RESET_REG,
					 SC130GS_REG_VALUE_08BIT, 0x01);
		usleep_range(100, 200);
		ret |= sc130gs_write_reg(sc130gs->client, 0x303f,
					 SC130GS_REG_VALUE_08BIT, 0x01);

		sc130gs->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		sc130gs->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&sc130gs->mutex);

	return ret;
}

static int __sc130gs_power_on(struct sc130gs *sc130gs)
{
	int ret;
	struct device *dev = &sc130gs->client->dev;

	if (!IS_ERR_OR_NULL(sc130gs->pins_default)) {
		ret = pinctrl_select_state(sc130gs->pinctrl,
					   sc130gs->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(sc130gs->xvclk, SC130GS_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (27MHz)\n");
	if (clk_get_rate(sc130gs->xvclk) != SC130GS_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 27MHz\n");
	ret = clk_prepare_enable(sc130gs->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}
	if (!IS_ERR(sc130gs->reset_gpio))
		gpiod_set_value_cansleep(sc130gs->reset_gpio, 1);

	ret = regulator_bulk_enable(SC130GS_NUM_SUPPLIES, sc130gs->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(sc130gs->reset_gpio))
		gpiod_set_value_cansleep(sc130gs->reset_gpio, 0);

	usleep_range(500, 1000);
	if (!IS_ERR(sc130gs->pwdn_gpio))
		gpiod_set_value_cansleep(sc130gs->pwdn_gpio, 0);
	usleep_range(2000, 4000);

	return 0;

disable_clk:
	clk_disable_unprepare(sc130gs->xvclk);

	return ret;
}

static void __sc130gs_power_off(struct sc130gs *sc130gs)
{
	int ret;
	struct device *dev = &sc130gs->client->dev;

	if (!IS_ERR(sc130gs->pwdn_gpio))
		gpiod_set_value_cansleep(sc130gs->pwdn_gpio, 1);
	clk_disable_unprepare(sc130gs->xvclk);
	if (!IS_ERR(sc130gs->reset_gpio))
		gpiod_set_value_cansleep(sc130gs->reset_gpio, 1);
	if (!IS_ERR_OR_NULL(sc130gs->pins_sleep)) {
		ret = pinctrl_select_state(sc130gs->pinctrl,
					   sc130gs->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(SC130GS_NUM_SUPPLIES, sc130gs->supplies);
}

static int sc130gs_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc130gs *sc130gs = to_sc130gs(sd);

	return __sc130gs_power_on(sc130gs);
}

static int sc130gs_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc130gs *sc130gs = to_sc130gs(sd);

	__sc130gs_power_off(sc130gs);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int sc130gs_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct sc130gs *sc130gs = to_sc130gs(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct sc130gs_mode_group *mg =
		&supported_mode_groups[default_group];
	const struct sc130gs_mode *def_mode = &mg->modes[default_mode];

	mutex_lock(&sc130gs->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&sc130gs->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int
sc130gs_enum_frame_interval(struct v4l2_subdev *sd,
			    struct v4l2_subdev_pad_config *cfg,
			    struct v4l2_subdev_frame_interval_enum *fie)
{
	struct sc130gs *sc130gs = to_sc130gs(sd);
	const struct sc130gs_mode_group *mg = &supported_mode_groups[default_group];

	if (fie->index >= sc130gs->cfg_num)
		return -EINVAL;

	if (fie->code != mg->modes[fie->index].bus_fmt)
		return -EINVAL;

	fie->code = mg->modes[fie->index].bus_fmt;
	fie->width = mg->modes[fie->index].width;
	fie->height = mg->modes[fie->index].height;
	fie->interval = mg->modes[fie->index].max_fps;
	fie->reserved[0] = mg->modes[fie->index].hdr_mode;

	return 0;
}

static const struct dev_pm_ops sc130gs_pm_ops = {
	SET_RUNTIME_PM_OPS(sc130gs_runtime_suspend,
			   sc130gs_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops sc130gs_internal_ops = {
	.open = sc130gs_open,
};
#endif

static const struct v4l2_subdev_core_ops sc130gs_core_ops = {
	.s_power = sc130gs_s_power,
	.ioctl = sc130gs_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = sc130gs_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops sc130gs_video_ops = {
	.s_stream = sc130gs_s_stream,
	.g_frame_interval = sc130gs_g_frame_interval,
	.g_mbus_config = sc130gs_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops sc130gs_pad_ops = {
	.enum_mbus_code = sc130gs_enum_mbus_code,
	.enum_frame_size = sc130gs_enum_frame_sizes,
	.enum_frame_interval = sc130gs_enum_frame_interval,
	.get_fmt = sc130gs_get_fmt,
	.set_fmt = sc130gs_set_fmt,
};

static const struct v4l2_subdev_ops sc130gs_subdev_ops = {
	.core	= &sc130gs_core_ops,   /* v4l2_subdev_core_ops sc130gs_core_ops */
	.video	= &sc130gs_video_ops,  /* */
	.pad	= &sc130gs_pad_ops,    /* */
};

static int sc130gs_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sc130gs *sc130gs =
		container_of(ctrl->handler, struct sc130gs, ctrl_handler);
	struct i2c_client *client = sc130gs->client;
	s64 max;
	u32 again, again_fine, dgain;
	int ret = 0;
	u32 val;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = sc130gs->cur_mode->height + ctrl->val - 3;
		__v4l2_ctrl_modify_range(sc130gs->exposure,
					 sc130gs->exposure->minimum, max,
					 sc130gs->exposure->step,
					 sc130gs->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		if (sc130gs->cur_mode->hdr_mode != NO_HDR ||
		    sc130gs->ext_trigger->val)
			goto out_ctrl;
		val = ctrl->val << 1;
		ret = sc130gs_write_reg(sc130gs->client, SC130GS_REG_EXP_LONG_L,
					SC130GS_REG_VALUE_08BIT,
					(val << 4 & 0XF0));
		ret |= sc130gs_write_reg(sc130gs->client,
					 SC130GS_REG_EXP_LONG_H,
					 SC130GS_REG_VALUE_08BIT,
					 (val >> 8 & 0XFF));
		dev_dbg(&client->dev, "set exposure 0x%x\n", val);
		break;

	case V4L2_CID_ANALOGUE_GAIN:
		if (sc130gs->cur_mode->hdr_mode != NO_HDR ||
		    sc130gs->ext_trigger->val)
			goto out_ctrl;
		ret = sc130gs_read_reg(sc130gs->client, SC130GS_REG_CDGAIN,
				       SC130GS_REG_VALUE_08BIT, &val);
		if (ret)
			break;
		sc130gs_get_gain_reg(ctrl->val, &again, &again_fine, &dgain);
		dev_dbg(&client->dev,
			"recv:%d set again 0x%x, again_fine 0x%x, dgain 0x%x\n",
			ctrl->val, again, again_fine, dgain);

		val &= ~(SC130GS_CGAIN_MASK | SC130GS_DGAIN_MASK);
		val |= ((dgain << 2) & SC130GS_CGAIN_MASK);
		val |= ((again << 5) & SC130GS_DGAIN_MASK);
		ret |= sc130gs_write_reg(sc130gs->client,
					 SC130GS_REG_AGAIN_FINE,
					 SC130GS_REG_VALUE_08BIT, again_fine);
		ret |= sc130gs_write_reg(sc130gs->client, SC130GS_REG_CDGAIN,
					 SC130GS_REG_VALUE_08BIT, val);
		break;
	case V4L2_CID_VBLANK:
		ret = sc130gs_write_reg(sc130gs->client, SC130GS_REG_VTS,
					SC130GS_REG_VALUE_16BIT,
					ctrl->val + sc130gs->cur_mode->height);
		dev_dbg(&client->dev, "set vblank 0x%x\n", ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = sc130gs_enable_test_pattern(sc130gs, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = sc130gs_read_reg(sc130gs->client, SC130GS_FLIP_REG,
				       SC130GS_REG_VALUE_08BIT, &val);
		if (ret)
			break;
		if (ctrl->val)
			val |= SC130GS_MIRROR_MASK;
		else
			val &= ~SC130GS_MIRROR_MASK;
		ret |= sc130gs_write_reg(sc130gs->client, SC130GS_FLIP_REG,
					 SC130GS_REG_VALUE_08BIT, val);
		break;
	case V4L2_CID_VFLIP:
		ret = sc130gs_read_reg(sc130gs->client, SC130GS_FLIP_REG,
				       SC130GS_REG_VALUE_08BIT, &val);
		if (ret)
			break;
		if (ctrl->val)
			val |= SC130GS_FLIP_MASK;
		else
			val &= ~SC130GS_FLIP_MASK;
		ret |= sc130gs_write_reg(sc130gs->client, SC130GS_FLIP_REG,
					 SC130GS_REG_VALUE_08BIT, val);
		break;
	case V4L2_CID_EXT_TRIGGER:
		if (ctrl->val)
			val = 0xa3;
		else
			val = 0x00;
		ret |= sc130gs_write_reg(sc130gs->client, 0x3234,
					 SC130GS_REG_VALUE_08BIT, val);
		if (ctrl->val) {
			ret |= sc130gs_write_reg(sc130gs->client, 0x3225,
						 SC130GS_REG_VALUE_08BIT, 0x02);
			ret |= sc130gs_write_reg(sc130gs->client, 0x3e01,
						 SC130GS_REG_VALUE_08BIT, 0x00);
			ret |= sc130gs_write_reg(sc130gs->client, 0x3e02,
						 SC130GS_REG_VALUE_08BIT, 0x00);
		}
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

out_ctrl:
	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops sc130gs_ctrl_ops = {
	.s_ctrl = sc130gs_set_ctrl,
};


static const struct v4l2_ctrl_config sc130gs_ctrl_ext_trigger = {
	.ops = &sc130gs_ctrl_ops,
	.id = V4L2_CID_EXT_TRIGGER,
	.name = "Ext Trigger",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = false,
	.max = true,
	.step = 1,
	.def = false,
};

static int sc130gs_initialize_controls(struct sc130gs *sc130gs)
{
	const struct sc130gs_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;
	u64 pixel_rate = 0;

	handler = &sc130gs->ctrl_handler;
	mode = sc130gs->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &sc130gs->mutex;

	sc130gs->link_freq =
		v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				       ARRAY_SIZE(link_freq_items) - 1, 0,
				       link_freq_items);
	__v4l2_ctrl_s_ctrl(sc130gs->link_freq, mode->mipi_freq_idx);

	/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
	pixel_rate = (u32)link_freq_items[mode->mipi_freq_idx] / mode->bpp * 2 *
		     SC130GS_LANES;
	sc130gs->pixel_rate =
		v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE, 0,
				  SC130GS_MAX_PIXEL_RATE, 1, pixel_rate);

	h_blank = mode->hts_def - mode->width;
	sc130gs->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					    h_blank, h_blank, 1, h_blank);
	if (sc130gs->hblank)
		sc130gs->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	sc130gs->vblank =
		v4l2_ctrl_new_std(handler, &sc130gs_ctrl_ops, V4L2_CID_VBLANK,
				  vblank_def, SC130GS_VTS_MAX - mode->height, 1,
				  vblank_def);

	exposure_max = mode->vts_def - 3;
	sc130gs->exposure =
		v4l2_ctrl_new_std(handler, &sc130gs_ctrl_ops, V4L2_CID_EXPOSURE,
				  SC130GS_EXPOSURE_MIN, exposure_max,
				  SC130GS_EXPOSURE_STEP, mode->exp_def);

	sc130gs->anal_gain =
		v4l2_ctrl_new_std(handler, &sc130gs_ctrl_ops,
				  V4L2_CID_ANALOGUE_GAIN, SC130GS_GAIN_MIN,
				  SC130GS_GAIN_MAX, SC130GS_GAIN_STEP,
				  SC130GS_GAIN_DEFAULT);

	sc130gs->test_pattern = v4l2_ctrl_new_std_menu_items(
		handler, &sc130gs_ctrl_ops, V4L2_CID_TEST_PATTERN,
		ARRAY_SIZE(sc130gs_test_pattern_menu) - 1, 0, 0,
		sc130gs_test_pattern_menu);

	v4l2_ctrl_new_std(handler, &sc130gs_ctrl_ops, V4L2_CID_HFLIP, 0, 1, 1,
			  0);
	v4l2_ctrl_new_std(handler, &sc130gs_ctrl_ops, V4L2_CID_VFLIP, 0, 1, 1,
			  0);
	sc130gs->ext_trigger =
		v4l2_ctrl_new_custom(handler, &sc130gs_ctrl_ext_trigger, NULL);

	if (handler->error) {
		ret = handler->error;
		dev_err(&sc130gs->client->dev, "Failed to init controls(%d)\n",
			ret);
		goto err_free_handler;
	}

	sc130gs->subdev.ctrl_handler = handler;
	sc130gs->has_init_exp = false;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int sc130gs_check_sensor_id(struct sc130gs *sc130gs,
				  struct i2c_client *client)
{
	struct device *dev = &sc130gs->client->dev;
	u32 id = 0;
	int ret;

	ret = sc130gs_read_reg(client, SC130GS_REG_CHIP_ID,
		SC130GS_REG_VALUE_16BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%04x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected SC%04x sensor\n", CHIP_ID);

	return 0;
}

static int sc130gs_configure_regulators(struct sc130gs *sc130gs)
{
	unsigned int i;

	for (i = 0; i < SC130GS_NUM_SUPPLIES; i++)
		sc130gs->supplies[i].supply = sc130gs_supply_names[i];

	return devm_regulator_bulk_get(&sc130gs->client->dev,
				       SC130GS_NUM_SUPPLIES,
				       sc130gs->supplies);
}

#ifdef HAS_RKLASER_SYSFS
static int sc130gs_get_temperature(struct sc130gs *sc130gs,
				  struct i2c_client *client)
{
	int ret;
	int v[2] = {0, 0};

	mutex_lock(&sc130gs->mutex);
	ret = sc130gs_read_reg(client, 0x4C10,
		SC130GS_REG_VALUE_08BIT, &v[0]);
	ret = sc130gs_read_reg(client, 0x4C11,
		SC130GS_REG_VALUE_08BIT, &v[1]);
	mutex_unlock(&sc130gs->mutex);
	return (v[0] << 3) | (v[1] & 0x7);
}

static ssize_t sysfs_sensor_temp_raw_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc130gs *sc130gs = to_sc130gs(sd);
	return sprintf(buf, "%d\n", sc130gs_get_temperature(sc130gs, client));
}

static struct device_attribute attributes[] = {
	__ATTR(sensor_temp_raw, S_IRUSR, sysfs_sensor_temp_raw_show, NULL),
};

static int add_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto undo;
	return 0;
undo:
	for (i--; i >= 0 ; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s: failed to create sysfs interface\n", __func__);
	return -ENODEV;
}

static void remove_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++) {
		device_remove_file(dev, attributes + i);
	}
}
#endif

static int sc130gs_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct sc130gs *sc130gs;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	sc130gs = devm_kzalloc(dev, sizeof(*sc130gs), GFP_KERNEL);
	if (!sc130gs)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &sc130gs->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &sc130gs->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &sc130gs->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &sc130gs->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	sc130gs->cfg_num = supported_mode_groups[default_group].num;
	sc130gs->cur_mode = &supported_mode_groups[default_group].modes[default_mode];
	sc130gs->client = client;

	sc130gs->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(sc130gs->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	sc130gs->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(sc130gs->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	sc130gs->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(sc130gs->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	sc130gs->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(sc130gs->pinctrl)) {
		sc130gs->pins_default =
			pinctrl_lookup_state(sc130gs->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(sc130gs->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		sc130gs->pins_sleep =
			pinctrl_lookup_state(sc130gs->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(sc130gs->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = sc130gs_configure_regulators(sc130gs);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&sc130gs->mutex);

	sd = &sc130gs->subdev;
	v4l2_i2c_subdev_init(sd, client, &sc130gs_subdev_ops);
	ret = sc130gs_initialize_controls(sc130gs);
	if (ret)
		goto err_destroy_mutex;

	ret = __sc130gs_power_on(sc130gs);
	if (ret)
		goto err_free_handler;

	ret = sc130gs_check_sensor_id(sc130gs, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &sc130gs_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	sc130gs->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &sc130gs->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(sc130gs->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 sc130gs->module_index, facing,
		 SC130GS_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);
#ifdef HAS_RKLASER_SYSFS
	add_sysfs_interfaces(dev);
#endif
	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__sc130gs_power_off(sc130gs);
err_free_handler:
	v4l2_ctrl_handler_free(&sc130gs->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&sc130gs->mutex);

	return ret;
}

static int sc130gs_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc130gs *sc130gs = to_sc130gs(sd);

#ifdef HAS_RKLASER_SYSFS
	remove_sysfs_interfaces(&client->dev);
#endif

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&sc130gs->ctrl_handler);
	mutex_destroy(&sc130gs->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__sc130gs_power_off(sc130gs);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id sc130gs_of_match[] = {
	{ .compatible = "smartsens,sc130gs" },
	{ },
};
MODULE_DEVICE_TABLE(of, sc130gs_of_match);
#endif

static const struct i2c_device_id sc130gs_match_id[] = {
	{ "smartsens,sc130gs", 0 },
	{ },
};

static struct i2c_driver sc130gs_i2c_driver = {
	.driver = {
		.name = SC130GS_NAME,
		.pm = &sc130gs_pm_ops,
		.of_match_table = of_match_ptr(sc130gs_of_match),
	},
	.probe		= &sc130gs_probe,
	.remove		= &sc130gs_remove,
	.id_table	= sc130gs_match_id,
};

#ifdef CONFIG_ROCKCHIP_THUNDER_BOOT
module_i2c_driver(sc130gs_i2c_driver);
#else
static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&sc130gs_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&sc130gs_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);
#endif

MODULE_DESCRIPTION("Smartsens sc130gs sensor driver");
MODULE_LICENSE("GPL v2");
