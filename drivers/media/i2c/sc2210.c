// SPDX-License-Identifier: GPL-2.0
/*
 * sc2210 driver
 *
 * Copyright (C) 2020 Fuzhou Rockchip Electronics Co., Ltd.
 *
 * V0.0X01.0X00 first version,adjust sc2210.
 * V0.0X01.0X01 add set flip ctrl.
 * V0.0X01.0X02 1.fixed time limit error
 *		2.fixed gain conversion function
 *		3.fixed test pattern error
 *		4.add quick stream on/off
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

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x02)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define MIPI_FREQ_186M			186000000 // 371.25Mbps/lane
#define MIPI_FREQ_297M			297000000 // 594.00Mbps/lane
#define MIPI_FREQ_432M			432000000 // 864.00Mbps/lane

#define BITS_PER_SAMPLE			10
#define SC2210_LANES			4
#define SC2210_MAX_PIXEL_RATE		(MIPI_FREQ_432M * 2 / BITS_PER_SAMPLE * SC2210_LANES)
#define OF_CAMERA_HDR_MODE		"rockchip,camera-hdr-mode"

#define SC2210_XVCLK_FREQ		27000000

#define CHIP_ID				0x2210
#define SC2210_REG_CHIP_ID		0x3107

#define SC2210_REG_CTRL_MODE		0x0100
#define SC2210_MODE_SW_STANDBY		0x0
#define SC2210_MODE_STREAMING		BIT(0)

#define	SC2210_EXPOSURE_MIN		0
#define	SC2210_EXPOSURE_STEP		1 // Normal=1,HDR2=2,HDR3=3
#define SC2210_VTS_MAX			0xffff

//long exposure
#define SC2210_REG_EXP_LONG_H		0x3e00    //[3:0]
#define SC2210_REG_EXP_LONG_M		0x3e01    //[7:0]
#define SC2210_REG_EXP_LONG_L		0x3e02    //[7:4]

//short exposure
#define SC2210_REG_EXP_SF_H		0x3e04    //[7:0]
#define SC2210_REG_EXP_SF_L		0x3e05    //[7:4]

//long frame and normal gain reg
#define SC2210_REG_AGAIN		0x3e08
#define SC2210_REG_AGAIN_FINE		0x3e09

#define SC2210_REG_DGAIN		0x3e06
#define SC2210_REG_DGAIN_FINE		0x3e07

//short fram gain reg
#define SC2210_SF_REG_AGAIN		0x3e12
#define SC2210_SF_REG_AGAIN_FINE	0x3e13

#define SC2210_SF_REG_DGAIN		0x3e10
#define SC2210_SF_REG_DGAIN_FINE	0x3e11

#define SC2210_GAIN_MIN			0x0
#define SC2210_GAIN_MAX			(44 * 32 * 64)
#define SC2210_GAIN_STEP		1
#define SC2210_GAIN_DEFAULT		0x4

//group hold
#define SC2210_GROUP_UPDATE_ADDRESS	0x3812
#define SC2210_GROUP_UPDATE_START_DATA	0x00
#define SC2210_GROUP_UPDATE_LAUNCH	0x30

#define SC2210_SOFTWARE_RESET_REG	0x0103
#define SC2210_REG_TEST_PATTERN		0x4501
#define SC2210_TEST_PATTERN_ENABLE	0x08

#define SC2210_REG_VTS			0x320e
#define SC2210_FLIP_REG			0x3221
#define SC2210_FLIP_MASK		0x60
#define SC2210_MIRROR_MASK		0x06
#define REG_NULL			0xFFFF

#define SC2210_REG_VALUE_08BIT		1
#define SC2210_REG_VALUE_16BIT		2
#define SC2210_REG_VALUE_24BIT		3

#define LONG_FRAME_MAX_EXP		4297
#define SHORT_FRAME_MAX_EXP		260

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define SC2210_NAME			"sc2210"

#define SC2210_DEF_MODE_ID		0

static const char * const sc2210_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define SC2210_NUM_SUPPLIES ARRAY_SIZE(sc2210_supply_names)

enum sc2210_max_pad {
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

struct sc2210_mode {
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

struct sc2210 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[SC2210_NUM_SUPPLIES];
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
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct sc2210_mode *cur_mode;
	u32			cfg_num;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	bool			has_init_exp;
	u32			cur_vts;
	struct preisp_hdrae_exp_s init_hdrae_exp;
};

#define to_sc2210(sd) container_of(sd, struct sc2210, subdev)

/*
 * Xclk 27Mhz linear 10bit 1920*544 180fps 594Mbps/lane
 */
static const struct regval sc2210_linear_10_1920x544_180fps_regs[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x36e9, 0x80},
	{0x36f9, 0x80},
	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x300a, 0x2c},
	{0x300f, 0x00},
	{0x3018, 0x73},
	{0x3019, 0x00},
	{0x301a, 0xf0},
	{0x301c, 0x78},
	{0x301f, 0x5c},
	{0x3031, 0x0a},
	{0x3032, 0x20},
	{0x3038, 0x22},
	{0x3106, 0x81},
	{0x3201, 0x04},
	{0x3203, 0x04},
	{0x3205, 0x8b},
	{0x3207, 0x43},
	{0x3208, 0x07},
	{0x3209, 0x80},
	{0x320a, 0x04},
	{0x320b, 0x38},
	{0x320c, 0x04},
	{0x320d, 0x4c},
	{0x320e, 0x04},
	{0x320f, 0xb0},
	{0x3211, 0x04},
	{0x3213, 0x04},
	{0x3215, 0x11},
	{0x3220, 0x13},
	{0x3221, 0x00},
	{0x3222, 0x00},
	{0x3225, 0x04},
	{0x322e, 0x00},
	{0x322f, 0x02},
	{0x3230, 0x00},
	{0x3231, 0x01},
	{0x3248, 0x0c},
	{0x3000, 0x00},
	{0x3249, 0x18},
	{0x3250, 0x00},
	{0x3253, 0x04},
	{0x3301, 0x14},
	{0x3302, 0x13},
	{0x3304, 0x48},
	{0x3305, 0x00},
	{0x3306, 0x88},
	{0x3308, 0x20},
	{0x3309, 0x60},
	{0x330a, 0x01},
	{0x330b, 0x18},
	{0x330d, 0x58},
	{0x330e, 0x70},
	{0x3314, 0x92},
	{0x331e, 0x39},
	{0x331f, 0x51},
	{0x3320, 0x09},
	{0x3332, 0x54},
	{0x334c, 0x10},
	{0x3350, 0x54},
	{0x3358, 0x54},
	{0x335c, 0x54},
	{0x335d, 0x60},
	{0x335e, 0x02},
	{0x335f, 0x04},
	{0x3364, 0x16},
	{0x3366, 0x92},
	{0x3367, 0x01},
	{0x337c, 0x06},
	{0x337d, 0x0a},
	{0x337e, 0x80},
	{0x3390, 0x08},
	{0x3391, 0x18},
	{0x3392, 0x38},
	{0x3393, 0x14},
	{0x3394, 0x14},
	{0x3395, 0x14},
	{0x3396, 0x08},
	{0x3397, 0x18},
	{0x3398, 0x38},
	{0x3399, 0x14},
	{0x339a, 0x30},
	{0x339b, 0x30},
	{0x339c, 0x30},
	{0x339e, 0x54},
	{0x33a0, 0x54},
	{0x33a2, 0x08},
	{0x33a4, 0x54},
	{0x33a8, 0x54},
	{0x33aa, 0x54},
	{0x33b0, 0x0f},
	{0x33b9, 0x11},
	{0x33e0, 0xc8},
	{0x33e1, 0x08},
	{0x33e2, 0x18},
	{0x33e3, 0x10},
	{0x33e4, 0x08},
	{0x33e5, 0x10},
	{0x33e6, 0x08},
	{0x33e7, 0x04},
	{0x33e8, 0x18},
	{0x33e9, 0x10},
	{0x33ea, 0x08},
	{0x33eb, 0x18},
	{0x33ec, 0x10},
	{0x33ed, 0x08},
	{0x33ee, 0xc8},
	{0x33ef, 0x08},
	{0x33f4, 0x18},
	{0x33f5, 0x10},
	{0x33f6, 0x08},
	{0x33f7, 0x10},
	{0x33f8, 0x08},
	{0x33f9, 0x04},
	{0x33fa, 0x18},
	{0x33fb, 0x10},
	{0x33fc, 0x08},
	{0x33fd, 0x18},
	{0x33fe, 0x10},
	{0x33ff, 0x08},
	{0x360f, 0x01},
	{0x3622, 0xf7},
	{0x3625, 0x0a},
	{0x3627, 0x82},
	{0x3630, 0xb8},
	{0x3631, 0x00},
	{0x3632, 0xd8},
	{0x3633, 0x43},
	{0x3635, 0x20},
	{0x3638, 0x27},
	{0x363a, 0x80},
	{0x363b, 0x02},
	{0x363e, 0x22},
	{0x3670, 0x4a},
	{0x3671, 0xf7},
	{0x3672, 0xf7},
	{0x3673, 0xf7},
	{0x3674, 0xd0},
	{0x3675, 0x90},
	{0x3676, 0x8a},
	{0x367a, 0x40},
	{0x367b, 0x78},
	{0x367c, 0x40},
	{0x367d, 0x78},
	{0x3690, 0x43},
	{0x3691, 0x54},
	{0x3692, 0x54},
	{0x369c, 0x40},
	{0x369d, 0x78},
	{0x36b5, 0x40},
	{0x36b6, 0x78},
	{0x36c0, 0x9f},
	{0x36c1, 0x9f},
	{0x36c2, 0x9f},
	{0x36cc, 0x22},
	{0x36cd, 0x22},
	{0x36ce, 0x28},
	{0x36d0, 0x20},
	{0x36d1, 0x40},
	{0x36d2, 0x78},
	{0x36ea, 0x35},
	{0x36eb, 0x04},
	{0x36ec, 0x0a},
	{0x36ed, 0x14},
	{0x36fa, 0x35},
	{0x36fb, 0x04},
	{0x36fc, 0x00},
	{0x36fd, 0x14},
	{0x3901, 0x02},
	{0x3902, 0x45},
	{0x3904, 0x08},
	{0x3905, 0x98},
	{0x3907, 0x01},
	{0x3908, 0x11},
	{0x391b, 0x87},
	{0x391d, 0x2c},
	{0x391f, 0x00},
	{0x3933, 0x28},
	{0x3934, 0x2c},
	{0x3940, 0x6f},
	{0x3942, 0x08},
	{0x3943, 0x2c},
	{0x3958, 0x04},
	{0x3959, 0x02},
	{0x3980, 0x61},
	{0x3987, 0x0b},
	{0x3990, 0x00},
	{0x3991, 0x00},
	{0x3992, 0x00},
	{0x3993, 0x00},
	{0x3994, 0x00},
	{0x3995, 0x00},
	{0x3996, 0x00},
	{0x3997, 0x00},
	{0x3998, 0x00},
	{0x3999, 0x00},
	{0x399a, 0x00},
	{0x399b, 0x00},
	{0x399c, 0x00},
	{0x399d, 0x00},
	{0x399e, 0x00},
	{0x399f, 0x00},
	{0x39a0, 0x00},
	{0x39a1, 0x00},
	{0x39a2, 0x03},
	{0x39a3, 0x30},
	{0x39a4, 0x03},
	{0x39a5, 0x60},
	{0x39a6, 0x03},
	{0x39a7, 0xa0},
	{0x39a8, 0x03},
	{0x39a9, 0xb0},
	{0x39aa, 0x00},
	{0x39ab, 0x00},
	{0x39ac, 0x00},
	{0x39ad, 0x20},
	{0x39ae, 0x00},
	{0x39af, 0x40},
	{0x39b0, 0x00},
	{0x39b1, 0x60},
	{0x39b2, 0x00},
	{0x39b3, 0x00},
	{0x39b4, 0x08},
	{0x39b5, 0x14},
	{0x39b6, 0x20},
	{0x39b7, 0x38},
	{0x39b8, 0x38},
	{0x39b9, 0x20},
	{0x39ba, 0x14},
	{0x39bb, 0x08},
	{0x39bc, 0x08},
	{0x39bd, 0x10},
	{0x39be, 0x20},
	{0x39bf, 0x30},
	{0x39c0, 0x30},
	{0x39c1, 0x20},
	{0x39c2, 0x10},
	{0x39c3, 0x08},
	{0x39c4, 0x00},
	{0x39c5, 0x80},
	{0x39c6, 0x00},
	{0x39c7, 0x80},
	{0x39c8, 0x00},
	{0x39c9, 0x00},
	{0x39ca, 0x80},
	{0x39cb, 0x00},
	{0x39cc, 0x00},
	{0x39cd, 0x00},
	{0x39ce, 0x00},
	{0x39cf, 0x00},
	{0x39d0, 0x00},
	{0x39d1, 0x00},
	{0x39e2, 0x05},
	{0x39e3, 0xeb},
	{0x39e4, 0x07},
	{0x39e5, 0xb6},
	{0x39e6, 0x00},
	{0x39e7, 0x3a},
	{0x39e8, 0x3f},
	{0x39e9, 0xb7},
	{0x39ea, 0x02},
	{0x39eb, 0x4f},
	{0x39ec, 0x08},
	{0x39ed, 0x00},
	{0x3e00, 0x00},
	{0x3e01, 0x4a},
	{0x3e02, 0x80},
	{0x3e04, 0x00},
	{0x3e05, 0xc0},
	{0x3e06, 0x00},
	{0x3e07, 0x80},
	{0x3e08, 0x03},
	{0x3e09, 0x40},
	{0x3e10, 0x00},
	{0x3e11, 0x80},
	{0x3e12, 0x03},
	{0x3e13, 0x40},
	{0x3e14, 0x31},
	{0x3e16, 0x00},
	{0x3e17, 0x80},
	{0x3e18, 0x00},
	{0x3e19, 0x80},
	{0x3e1b, 0x3a},
	{0x3e22, 0x00},
	{0x3e23, 0x00},
	{0x3e24, 0xed},
	{0x3e26, 0x40},
	{0x3e50, 0x00},
	{0x3e51, 0x0c},
	{0x3e52, 0x00},
	{0x3e53, 0x00},
	{0x3e54, 0xc9},
	{0x3e56, 0x00},
	{0x3e57, 0x80},
	{0x3e58, 0x03},
	{0x3e59, 0x40},
	{0x3f05, 0x18},
	{0x3f08, 0x10},
	{0x4401, 0x1a},
	{0x4407, 0xc0},
	{0x4418, 0x68},
	{0x4500, 0x18},
	{0x4501, 0xa4},
	{0x4503, 0xc0},
	{0x4505, 0x12},
	{0x4509, 0x10},
	{0x4825, 0x36},
	{0x4837, 0x1b},
	{0x4853, 0xf0},
	{0x5000, 0x0e},
	{0x550f, 0x20},
	{0x5900, 0x01},
	{0x5901, 0x00},
	{0x36e9, 0x20},
	{0x36f9, 0x30},
	// {0x0100, 0x01},
	// 1920x544@180fps
	{0x3200, 0x00},
	{0x3201, 0x04},
	{0x3202, 0x01},
	{0x3203, 0x10},
	{0x3204, 0x07},
	{0x3205, 0x8b},
	{0x3206, 0x03},
	{0x3207, 0x37},
	{0x3208, 0x07},
	{0x3209, 0x80},
	{0x320a, 0x02},
	{0x320b, 0x20},
	{0x3210, 0x00},
	{0x3211, 0x04},
	{0x3212, 0x00},
	{0x3213, 0x04},
	{0x320e, 0x02},
	{0x320f, 0x58},
	{0x3e01, 0x25},
	{0x3e02, 0x40},
	//
	{0x0100, 0x01},
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz linear 12bit 1920*1080 60fps 432Mbps/lane
 */
static const struct regval sc2210_linear_12_1920x1080_60fps_regs[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},

	{0x36e9, 0x80},
	{0x36f9, 0x80},

	{0x3e01, 0x4f},
	{0x3e02, 0xc0},

	{0x3305, 0x01},
	{0x3306, 0x78},
	{0x330b, 0x00},

	{0x3907, 0x01},
	{0x3908, 0x11},
	{0x3622, 0x03},//cmp gbw
	{0x3627, 0x02},// bit[7] cmp gbw

	{0x3635, 0x42},
	{0x363b, 0x00},
	{0x363e, 0x30},
	{0x3308, 0x14},
	{0x3301, 0x58},

	///////digital updates start////////////
	// 05-23-2018

	//0x33e0,0xa0,
	{0x33e1, 0x40},
	{0x33e2, 0x38},
	{0x33e3, 0x38},
	{0x33e4, 0x20},
	{0x33e5, 0x20},
	{0x33e6, 0x20},
	{0x33e7, 0x10},

	{0x33e8, 0x20},
	{0x33e9, 0x20},
	{0x33ea, 0x10},
	{0x33eb, 0x10},
	{0x33ec, 0x10},
	{0x33ed, 0x08}, 


	//0x33e0,0xa0,
	{0x33e1, 0x08},

	{0x33e2, 0x18},
	{0x33e3, 0x18}, //10
	{0x33e4, 0x18}, //0c

	{0x33e5, 0x10},//18
	{0x33e6, 0x06},//18
	{0x33e7, 0x02},//18

	{0x33e8, 0x18},
	{0x33e9, 0x10},
	{0x33ea, 0x0c},

	{0x33eb, 0x10},
	{0x33ec, 0x04},
	{0x33ed, 0x02},

	{0x33ee, 0xa0},
	{0x33ef, 0x08},
	{0x33f4, 0x18},
	{0x33f5, 0x10},
	{0x33f6, 0x0c},
	{0x33f7, 0x10},
	{0x33f8, 0x06},
	{0x33f9, 0x02},
	{0x33fa, 0x18},
	{0x33fb, 0x10},
	{0x33fc, 0x0c},
	{0x33fd, 0x10},
	{0x33fe, 0x04},
	{0x33ff, 0x02},

	{0x3231, 0x02}, // 1st frame exp correction 20190105
	{0x4500, 0x18}, // digital test mode default set : black->white  20190105

	//0x3e1b,0x2a,//0x3e03 03 mode support fine gain 40-->7f
	//0x3e25,0x03,//blc 1x gain threshold
	//0x3e26,0x40,//blc 1x gain threshold
	//0x3e16,0x00,
	//0x3e17,0xac,//dcg gain value
	//0x3e18,0x00,
	//0x3e19,0xac,//dcg open point

	{0x5000, 0x06},

	//end

	//blc
	{0x391b, 0x8c}, //[3:0] blc update set

	///////digital updates end////////////
	{0x3622, 0xe3},
	{0x3627, 0x82},
	{0x3306, 0x30},
	{0x330b, 0xe0},
	{0x3302, 0x10},
	{0x33b9, 0x0e},
	{0x3638, 0x27},
	{0x3301, 0x48},

	{0x320c, 0x0e},
	{0x320d, 0xc4},
	{0x3622, 0x03},
	{0x3627, 0x02},
	{0x3633, 0x64},
	{0x3306, 0x70},
	{0x3301, 0xf8},

	{0x3038, 0x22},

	{0x3635, 0x20},  //lag optimization
	{0x3308, 0x30},

	//DVP mode : close mipi
	//0x3018,0x6f,
	//0x3019,0xff,
	//0x301c,0xb4,

	//0x3001,0xff,
	//0x3000,0xcf,  //PCB issue  right value is 0x0f ˳��Ҫ��

	//0x3002,0xff,
	//0x301c,0x94,
	//0x3018,0x2f,
	//0x301a,0xf8,
	//0x3c00,0x45,
	//0x3030,0x02,
	//0x303f,0x81,
	//0x3033,0xa0,

	//0x3641,0x05,

	//mipi
	{0x3018, 0x33},//[7:5] lane_num-1 [4] digital mipi pad en 1:mipi 0:DVP
	{0x3019, 0x0c},//[3:0] lane disable
	{0x3031, 0x0c},//[3:0] bitmode
	{0x300f, 0xff},
	{0x3001, 0x07},//DVP oen [7:0]->DATA[7:0]  not reused :[2:0] dat[2:0] oen  0705
	{0x3000, 0xc0},//DVP oen [3:0]->DATA[11:8]
	{0x3002, 0xc0},//[6] oen href [5] oen pclk  not reused href  0705
	{0x300a, 0x2c},//[6] oen efsync [2] oen fsync
	{0x300f, 0x00},

	{0x4603, 0x00},//[0] data_fifo mipi mode
	{0x4837, 0x15},//[7:0] pclk period * 2
	{0x3033, 0x20},

	{0x36ea, 0x3a}, //729Mbps->486Mbps
	//mipi end
	//dvp 25fps
	{0x3301, 0x80},
	{0x3305, 0x00},
	{0x3306, 0xa0},
	{0x3308, 0x20},
	{0x330a, 0x01},
	{0x330b, 0x60}, //[18,a8]

	{0x3622, 0x07},
	{0x363e, 0x2e},
	{0x3630, 0xa8},
	{0x3631, 0x00},

	//0x320c,0x07,
	//0x320d,0x62,
	{0x3e01, 0x10},

	//144M pclk 25fps 576Mcnt   72Msysclk
	//0x36f9,0x14,
	{0x36fa, 0x38},
	{0x36fb, 0x17},
	{0x36fc, 0x01},
	{0x36fd, 0x14},

	//0x36e9,0x00, //864Mbps
	{0x36ea, 0x38},
	{0x36eb, 0x07},
	{0x36ec, 0x03},
	{0x36ed, 0x14},

	//0x3106,0x81,

	{0x3633, 0x32},

	{0x363e, 0x28},
	{0x3301, 0x20},

	{0x3630, 0xa5},

	//1119
	{0x3638, 0x24},
	{0x3306, 0xa0},
	{0x330b, 0x18},

	{0x3633, 0x33},

	{0x3366, 0x92},

	{0x4418, 0x0d}, //8.6k fullwell 
	{0x3e09, 0x40},
	{0x4418, 0x1a},

	//preprecharge
	{0x3314, 0x92},
	{0x330e, 0x43}, //[43,80]
	{0x334c, 0x10},

	{0x335d, 0x60}, //1214
	{0x337f, 0x33}, //new auto precharge  330e in 33f0   [7:6] 11: close div_rst 00:open div_rst
	{0x3368, 0x04},
	{0x3369, 0x00},
	{0x336a, 0x00},
	{0x336b, 0x00},
	{0x3367, 0x10},
	{0x330e, 0x60},

	{0x3635, 0x40},

	{0x4509, 0x20}, //for more adc range in digital processing
	{0x330b, 0x44}, //[,64]
	{0x3306, 0x98}, //[8c,]
	{0x3638, 0x22},
	{0x3304, 0x48},
	{0x331e, 0x41},
	{0x3309, 0x98},
	{0x331f, 0x91},

	//11bit
	{0x4501, 0xb4},
	{0x4509, 0x20}, //0105
	{0x4418, 0x34},
	{0x330a, 0x00},
	{0x330b, 0xd0},
	{0x3306, 0x70},

	{0x3638, 0x24},

	//60fps
	{0x320c, 0x04}, //0x528 for 25fps
	{0x320d, 0x37},
	{0x320e, 0x04},
	{0x320f, 0x58},

	{0x3207, 0x3f},
	{0x3213, 0x04},

	{0x3f08, 0x08},

	{0x330a, 0x01},
	{0x330b, 0x40},
	{0x3306, 0x90},

	{0x3304, 0x58},
	{0x331e, 0x49},
	{0x3309, 0x98},
	{0x331f, 0x89},

	{0x3632, 0x88}, // optimize row noise from 363e[5] 1225

	{0x363e, 0x22}, //logic
	{0x3301, 0x20},
	{0x3622, 0xf7},
	{0x363a, 0x80},
	{0x3633, 0x43},

	//20190110
	{0x4837, 0x12},

	//0x36e9,0x00,
	//0x36ea,0x38,
	//0x36eb,0x07,
	//0x36ec,0x03,
	//0x36ed,0x14,
	//mipiclk=vco=27*4*8=864
	//mipipclk=108
	//
	//0x36f9,0x14,
	//0x36fa,0x38,
	//0x36fb,0x17,
	//0x36fc,0x01,
	//0x36fd,0x14,
	//vco=27/1.5*2*4*8=1152
	//sysclk=vco/2/8=72
	//countclk=vco/2=576
	//countclk:sysclk=8:1

	//201190306
	{0x3635, 0x20},//txvdd bypass
	{0x330e, 0x68},//short exp lag
	{0x301f, 0x01},//setting id

	//init
	{0x3e00, 0x00},//max exp=vts-4, min exp=0
	{0x3e01, 0x45},
	{0x3e02, 0x40},
	{0x3e03, 0x0b},
	{0x3e06, 0x00},
	{0x3e07, 0x80},
	{0x3e08, 0x03},
	{0x3e09, 0x40},
	{0x363e, 0x22},
	{0x3301, 0x20},
	{0x3622, 0xf7},
	{0x363a, 0x80},
	{0x3633, 0x43},

	//digital logic
	{0x3e1b, 0x3a},//fine gain 0x40~0x7f
	{0x3e26, 0x40},
	//0x3622 auto logic read 0x3680 for auto value
	{0x360f, 0x01},//[0] 3622 auto en
	{0x367a, 0x40},//gain0 (dcg_on)
	{0x367b, 0x7f},//gain1(max analog gain)
	{0x3671, 0xf7},//sel0
	{0x3672, 0xf7},//sel1
	{0x3673, 0x07},//sel2
	//0x363a auto logic read 0x368c for auto value
	{0x3670, 0x40},//[1]3630 auto en [3] 3633 auto en, [6] 363a auto en
	{0x36b5, 0x40},//gain0 (dcg_on)              
	{0x36b6, 0x7f},//gain1 (max analog gain) 
	{0x36c0, 0x80},//sel0
	{0x36c1, 0x9f},//sel1
	{0x36c2, 0x9f},//sel2
	//0x363e auto logic read 0x36c2 for auto value
	{0x36d0, 0x20},//0x36d0[5]:363e auto en, 0x36d0[4]:363d auto en
	{0x36d1, 0x40}, //gain0
	{0x36d2, 0x7f},//gain1
	{0x36cc, 0x20},//sel 0
	{0x36cd, 0x20},//sel 1		//20190826	blacksun
	{0x36ce, 0x30}, //sel 2
	//0x3633 auto logic read 0x3683 for auto value
	{0x3670, 0x48},
	{0x369c, 0x40},//gain0
	{0x369d, 0x7f},//gain1
	{0x3690, 0x42},//sel0
	{0x3691, 0x43},//sel1
	{0x3692, 0x54},//sel2
	//0x3301 auto logic read 0x33f2 for auto value
	{0x3364, 0x16}, //0x3364[4]:0x3301 auto en
	{0x3390, 0x10},//dcg disable gain0
	{0x3391, 0x30},//dcg disable gain1
	{0x3392, 0x40},//dcg disable gain2
	{0x3301, 0x0a},//dcg disable sel0	
	{0x3393, 0x0a},//dcg disable sel1	
	{0x3394, 0x0a},//dcg disable sel2	
	{0x3395, 0x0a},//dcg disable sel3	[05,
	{0x3396, 0x08},//dcg enable gain0
	{0x3397, 0x30},//dcg enable gain1
	{0x3398, 0x3f},//dcg enable gain2
	{0x3399, 0x30},//sel0	[14,
	{0x339a, 0x30},//sel1	[14,
	{0x339b, 0x30},//sel2	[14,
	{0x339c, 0x30},//sel3	[14,


	//20190620 high temp
	{0x3933, 0x28},//blc_max
	{0x3934, 0xa6},
	{0x3940, 0x70},
	{0x3942, 0x08},
	{0x3943, 0xbc},
	{0x3990, 0x00},//kh
	{0x3991, 0x00},
	{0x3992, 0x00},
	{0x3993, 0x00},
	{0x3994, 0x00},
	{0x3995, 0x00},
	{0x3996, 0x00},
	{0x3997, 0x00},
	{0x3998, 0x00},
	{0x3999, 0x00},
	{0x399a, 0x00},
	{0x399b, 0x00},
	{0x399c, 0x00},
	{0x399d, 0x00},
	{0x399e, 0x00},
	{0x399f, 0x00},
	{0x39a0, 0x00},
	{0x39a1, 0x00},
	{0x39a2, 0x03},//kv
	{0x39a3, 0x30},
	{0x39a4, 0x03},
	{0x39a5, 0x60},
	{0x39a6, 0x03},
	{0x39a7, 0xa0},
	{0x39a8, 0x03},
	{0x39a9, 0xb0},
	{0x39aa, 0x00},
	{0x39ab, 0x00},
	{0x39ac, 0x00},
	{0x39ad, 0x20},
	{0x39ae, 0x00},
	{0x39af, 0x70},
	{0x39b0, 0x00},
	{0x39b1, 0xa0},
	{0x39b2, 0x00},
	{0x39b3, 0xf0},
	{0x39b4, 0x08},//posh
	{0x39b5, 0x14},
	{0x39b6, 0x20},
	{0x39b7, 0x38},
	{0x39b8, 0x38},
	{0x39b9, 0x20},
	{0x39ba, 0x14},
	{0x39bb, 0x08},
	{0x39bc, 0x08},//posv
	{0x39bd, 0x10},
	{0x39be, 0x20},
	{0x39bf, 0x30},
	{0x39c0, 0x30},
	{0x39c1, 0x20},
	{0x39c2, 0x10},
	{0x39c3, 0x08},
	{0x39c4, 0x00},//set Temp_thre
	{0x39c5, 0x80},
	{0x39c6, 0x00},//top,btm.lft,rgt
	{0x39c7, 0x80},
	{0x39c8, 0x00},
	{0x39c9, 0x00},
	{0x39ca, 0x80},
	{0x39cb, 0x00},
	{0x39cc, 0x00},
	{0x39cd, 0x00},
	{0x39ce, 0x00},
	{0x39cf, 0x00},
	{0x39d0, 0x00},
	{0x39d1, 0x00},
	{0x39e2, 0x05},//DC_alpha
	{0x39e3, 0xeb},
	{0x39e4, 0x07},
	{0x39e5, 0xb6},
	{0x39e6, 0x00},
	{0x39e7, 0x3a},
	{0x39e8, 0x3f},
	{0x39e9, 0xb7},
	{0x39ea, 0x02},
	{0x39eb, 0x4f},
	{0x39ec, 0x08},
	{0x39ed, 0x00},
	{0x3980, 0x61},
	{0x3987, 0x0b},
	{0x3e14, 0x31},//agc_chg_flg
	{0x3207, 0x3f},//
	{0x391b, 0x87},//20190615 fix
	{0x391f, 0x00},
	{0x39ae, 0x00},//20190902 shading
	{0x39af, 0x40},
	{0x39b0, 0x00},
	{0x39b1, 0x60},
	{0x39b2, 0x00},
	{0x39b3, 0x00},

	//20190826
	{0x3632, 0xc8},//pump driver
	{0x3630, 0xc5},//blacksun

	//window
	//0x3200,0x00,
	{0x3201, 0x04},
	//0x3202,0x00,
	{0x3203, 0x04},
	{0x3204, 0x07},
	{0x3205, 0x8b},
	{0x3206, 0x04},
	{0x3207, 0x43},
	//0x3208,0x07,
	//0x3209,0x80,
	//0x320a,0x04,
	//0x320b,0x38,
	//0x3210,0x00,
	{0x3211, 0x04},
	//0x3212,0x00,
	{0x3213, 0x04},

	//0x36e9,0x24,//27/2*2*4*8=864
	//0x36ea,0x38,
	//0x36eb,0x07,
	//0x36ec,0x13,//mipiclk=432, mipipclk=54
	//0x36ed,0x14,
	//0x36f9,0x14,//27/1.5*2*4*6=864
	{0x36fa, 0x3a},
	{0x36fb, 0x15},//sclk=72
	{0x36fc, 0x01},//countclk=432
	{0x36fd, 0x14},

	//default
	//0x320c,0x04,//1079
	//0x320d,0x37,
	//0x320e,0x04,//1112
	//0x320f,0x58,
	//0x3106,0x01,
	//0x36e9,0x24,//27/2*2*4*8=864
	//0x36ea,0x38,
	//0x36eb,0x07,
	//0x36ec,0x13,//mipiclk=432, mipipclk=54
	//0x36ed,0x14,
	//0x4837,0x25,
	//0x36f9,0x14,//27/1.5*2*4*6=864
	//0x36fa,0x3a,
	//0x36fb,0x15,//sclk=72
	//0x36fc,0x01,//countclk=432
	//0x36fd,0x14,
	//0x3314,0x92,
	//0x330e,0x68,//[,65]
	//0x3367,0x10,
	//0x331e,0x49,
	//0x331f,0x89,
	//0x3304,0x58,
	//0x3309,0x98,
	//0x3305,0x00,
	//0x3306,0x90,//[86,ce]
	//0x330a,0x01,
	//0x330b,0x40,//[119,1a1]
	//0x3301,0x20,
	//0x330b,0x40,
	{0x3306, 0xb0},//[9c,c5]
	{0x330b, 0x68},//[150,181]
	{0x330e, 0x48},//[,4d]

	{0x301f, 0x20},
	//for mipi 4 lane
	{0x3018, 0x73},
	{0x3019, 0x00},

	{0x3106, 0x81},//pll1-->sclk
	//0x36e9,0x24,//27/2*2*4*8=864
	//0x36ea,0x38,
	{0x36eb, 0x0e},//sclk=864/2/6=72
	{0x36ec, 0x13},//mipiclk=864/2=432, mipipclk=54
	//0x36ed,0x14,
	{0x4837, 0x25},

	//short exp v_line(0x3301)
	{0x336d, 0x03},//beat sync
	{0x335e, 0x02},
	{0x335f, 0x06},
	{0x337c, 0x08},
	{0x337d, 0x0e},
	{0x33a2, 0x0a},
	{0x3630, 0xa2},//rst clamp, sig clamp: blacksun/v_line
	{0x3625, 0x0a},//smear
	{0x363b, 0x02},//hvdd
	{0x3632, 0xd8},//pump driver add 1
	{0x4800, 0x24},//non-continue mode
	{0x3253, 0x04},//power save mode
	{0x3905, 0xd8},//one channel blc
	{0x391b, 0x83},//Blc out range stable 2lsb
	{0x5000, 0x0e},//otp en
	{0x550f, 0x20},//otp addr
	{0x4407, 0xc0},//otp addr
	{0x4401, 0x1a},//otp r_pgm0_en
	{0x3958, 0x02},//hdr short exp:r_update_op_s_o
	{0x3959, 0x04},//hdr short exp:r_update_op_m_o
	//0x331e,0x49,
	//0x331f,0x89,
	//0x3304,0x58,//4c,
	//0x3309,0x98,//8c,

	//init
	//0x3e00,0x00,
	{0x3e01, 0x45},
	{0x3e02, 0x40},
	//0x3e06,0x00,
	//0x3e07,0x80,
	//0x3e08,0x03,
	//0x3e09,0x40,

	{0x36e9, 0x24},
	{0x36f9, 0x14},

	{0x0100, 0x01},

	//[gain<2]
	//0x363e,0x20,
	//0x3301,0x18,
	//0x3622,0xf7,
	//0x363a,0x80,
	//0x3633,0x43,
	//
	//[2=<gain<64]
	//0x363e,0x20,
	//0x3301,0x30,//201190306
	//0x3622,0xf7
	//0x363a,0x9f,
	//0x3633,0x43,
	//
	//[gain>=64]
	//0x363e,0x30,
	//0x3301,0x30,//201190306
	//0x3622,0x07
	//0x363a,0x9f,
	//0x3633,0x54,
	//
	{REG_NULL, 0x00},
};

/*
 * Xclk 27Mhz linear 10bit 1920*1080 90fps 594Mbps/lane
 */
static const struct regval sc2210_linear_10_1920x1080_90fps_regs[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x36e9, 0x80},
	{0x36f9, 0x80},
	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x300a, 0x2c},
	{0x300f, 0x00},
	{0x3018, 0x73},
	{0x3019, 0x00},
	{0x301a, 0xf0},
	{0x301c, 0x78},
	{0x301f, 0x4d},
	{0x3031, 0x0a},
	{0x3032, 0x20},
	{0x3038, 0x22},
	{0x3106, 0x81},
	{0x3201, 0x04},
	{0x3203, 0x04},
	{0x3205, 0x8b},
	{0x3207, 0x43},
	{0x3208, 0x07},
	{0x3209, 0x80},
	{0x320a, 0x04},
	{0x320b, 0x38},
	{0x320c, 0x04},
	{0x320d, 0x4c},
	{0x320e, 0x04},
	{0x320f, 0xb0},
	{0x3211, 0x04},
	{0x3213, 0x04},
	{0x3215, 0x11},
	{0x3220, 0x13},
	{0x3221, 0x00},
	{0x3222, 0x00},
	{0x3225, 0x04},
	{0x322e, 0x00},
	{0x322f, 0x02},
	{0x3230, 0x00},
	{0x3231, 0x01},
	{0x3248, 0x0c},
	{0x3000, 0x00},
	{0x3249, 0x18},
	{0x3250, 0x00},
	{0x3253, 0x04},
	{0x3301, 0x14},
	{0x3302, 0x13},
	{0x3304, 0x48},
	{0x3305, 0x00},
	{0x3306, 0x88},
	{0x3308, 0x20},
	{0x3309, 0x60},
	{0x330a, 0x01},
	{0x330b, 0x18},
	{0x330d, 0x58},
	{0x330e, 0x70},
	{0x3314, 0x92},//3-exp SHDR should modify to 0x93
	{0x331e, 0x39},
	{0x331f, 0x51},
	{0x3320, 0x09},
	{0x3332, 0x54},
	{0x334c, 0x10},
	{0x3350, 0x54},
	{0x3358, 0x54},
	{0x335c, 0x54},
	{0x335d, 0x60},
	{0x335e, 0x02},
	{0x335f, 0x04},
	{0x3364, 0x16},
	{0x3366, 0x92},
	{0x3367, 0x01},
	{0x337c, 0x06},
	{0x337d, 0x0a},
	{0x337e, 0x80},
	{0x3390, 0x08},
	{0x3391, 0x18},
	{0x3392, 0x38},
	{0x3393, 0x14},
	{0x3394, 0x14},
	{0x3395, 0x14},
	{0x3396, 0x08},
	{0x3397, 0x18},
	{0x3398, 0x38},
	{0x3399, 0x14},
	{0x339a, 0x30},
	{0x339b, 0x30},
	{0x339c, 0x30},
	{0x339e, 0x54},
	{0x33a0, 0x54},
	{0x33a2, 0x08},
	{0x33a4, 0x54},
	{0x33a8, 0x54},
	{0x33aa, 0x54},
	{0x33b0, 0x0f},
	{0x33b9, 0x11},
	{0x33e0, 0xc8},
	{0x33e1, 0x08},
	{0x33e2, 0x18},
	{0x33e3, 0x10},
	{0x33e4, 0x08},
	{0x33e5, 0x10},
	{0x33e6, 0x08},
	{0x33e7, 0x04},
	{0x33e8, 0x18},
	{0x33e9, 0x10},
	{0x33ea, 0x08},
	{0x33eb, 0x18},
	{0x33ec, 0x10},
	{0x33ed, 0x08},
	{0x33ee, 0xc8},
	{0x33ef, 0x08},
	{0x33f4, 0x18},
	{0x33f5, 0x10},
	{0x33f6, 0x08},
	{0x33f7, 0x10},
	{0x33f8, 0x08},
	{0x33f9, 0x04},
	{0x33fa, 0x18},
	{0x33fb, 0x10},
	{0x33fc, 0x08},
	{0x33fd, 0x18},
	{0x33fe, 0x10},
	{0x33ff, 0x08},
	{0x360f, 0x01},
	{0x3622, 0xf7},
	{0x3625, 0x0a},
	{0x3627, 0x82},
	{0x3630, 0xb8},
	{0x3631, 0x00},
	{0x3632, 0xd8},
	{0x3633, 0x43},
	{0x3635, 0x20},
	{0x3638, 0x27},
	{0x363a, 0x80},
	{0x363b, 0x02},
	{0x363e, 0x22},
	{0x3670, 0x4a},
	{0x3671, 0xf7},
	{0x3672, 0xf7},
	{0x3673, 0xf7},
	{0x3674, 0xd0},
	{0x3675, 0x90},
	{0x3676, 0x8a},
	{0x367a, 0x40},
	{0x367b, 0x78},
	{0x367c, 0x40},
	{0x367d, 0x78},
	{0x3690, 0x43},
	{0x3691, 0x54},
	{0x3692, 0x54},
	{0x369c, 0x40},
	{0x369d, 0x78},
	{0x36b5, 0x40},
	{0x36b6, 0x78},
	{0x36c0, 0x9f},
	{0x36c1, 0x9f},
	{0x36c2, 0x9f},
	{0x36cc, 0x22},
	{0x36cd, 0x22},
	{0x36ce, 0x28},
	{0x36d0, 0x20},
	{0x36d1, 0x40},
	{0x36d2, 0x78},
	{0x36ea, 0xf5},
	{0x36eb, 0x04},
	{0x36ec, 0x0a},
	{0x36ed, 0x04},
	{0x36fa, 0xf5},
	{0x36fb, 0x04},
	{0x36fc, 0x00},
	{0x36fd, 0x04},
	{0x3901, 0x02},
	{0x3902, 0x45},
	{0x3904, 0x08},
	{0x3905, 0x98},
	{0x3907, 0x01},
	{0x3908, 0x11},
	{0x391b, 0x87},
	{0x391d, 0x2c},
	{0x391f, 0x00},
	{0x3933, 0x28},
	{0x3934, 0x2c},
	{0x3940, 0x6f},
	{0x3942, 0x08},
	{0x3943, 0x2c},
	{0x3958, 0x04},
	{0x3959, 0x02},
	{0x3980, 0x61},
	{0x3987, 0x0b},
	{0x3990, 0x00},
	{0x3991, 0x00},
	{0x3992, 0x00},
	{0x3993, 0x00},
	{0x3994, 0x00},
	{0x3995, 0x00},
	{0x3996, 0x00},
	{0x3997, 0x00},
	{0x3998, 0x00},
	{0x3999, 0x00},
	{0x399a, 0x00},
	{0x399b, 0x00},
	{0x399c, 0x00},
	{0x399d, 0x00},
	{0x399e, 0x00},
	{0x399f, 0x00},
	{0x39a0, 0x00},
	{0x39a1, 0x00},
	{0x39a2, 0x03},
	{0x39a3, 0x30},
	{0x39a4, 0x03},
	{0x39a5, 0x60},
	{0x39a6, 0x03},
	{0x39a7, 0xa0},
	{0x39a8, 0x03},
	{0x39a9, 0xb0},
	{0x39aa, 0x00},
	{0x39ab, 0x00},
	{0x39ac, 0x00},
	{0x39ad, 0x20},
	{0x39ae, 0x00},
	{0x39af, 0x40},
	{0x39b0, 0x00},
	{0x39b1, 0x60},
	{0x39b2, 0x00},
	{0x39b3, 0x00},
	{0x39b4, 0x08},
	{0x39b5, 0x14},
	{0x39b6, 0x20},
	{0x39b7, 0x38},
	{0x39b8, 0x38},
	{0x39b9, 0x20},
	{0x39ba, 0x14},
	{0x39bb, 0x08},
	{0x39bc, 0x08},
	{0x39bd, 0x10},
	{0x39be, 0x20},
	{0x39bf, 0x30},
	{0x39c0, 0x30},
	{0x39c1, 0x20},
	{0x39c2, 0x10},
	{0x39c3, 0x08},
	{0x39c4, 0x00},
	{0x39c5, 0x80},
	{0x39c6, 0x00},
	{0x39c7, 0x80},
	{0x39c8, 0x00},
	{0x39c9, 0x00},
	{0x39ca, 0x80},
	{0x39cb, 0x00},
	{0x39cc, 0x00},
	{0x39cd, 0x00},
	{0x39ce, 0x00},
	{0x39cf, 0x00},
	{0x39d0, 0x00},
	{0x39d1, 0x00},
	{0x39e2, 0x05},
	{0x39e3, 0xeb},
	{0x39e4, 0x07},
	{0x39e5, 0xb6},
	{0x39e6, 0x00},
	{0x39e7, 0x3a},
	{0x39e8, 0x3f},
	{0x39e9, 0xb7},
	{0x39ea, 0x02},
	{0x39eb, 0x4f},
	{0x39ec, 0x08},
	{0x39ed, 0x00},
	{0x3e00, 0x00},
	{0x3e01, 0x4a},
	{0x3e02, 0x80},
	{0x3e04, 0x00},
	{0x3e05, 0xc0},
	{0x3e06, 0x00},
	{0x3e07, 0x80},
	{0x3e08, 0x03},
	{0x3e09, 0x40},
	{0x3e10, 0x00},
	{0x3e11, 0x80},
	{0x3e12, 0x03},
	{0x3e13, 0x40},
	{0x3e14, 0x31},
	{0x3e16, 0x00},
	{0x3e17, 0x80},
	{0x3e18, 0x00},
	{0x3e19, 0x80},
	{0x3e1b, 0x3a},
	{0x3e22, 0x00},
	{0x3e23, 0x00},
	{0x3e24, 0xed},
	{0x3e26, 0x40},
	{0x3e50, 0x00},
	{0x3e51, 0x0c},
	{0x3e52, 0x00},
	{0x3e53, 0x00},
	{0x3e54, 0xc9},
	{0x3e56, 0x00},
	{0x3e57, 0x80},
	{0x3e58, 0x03},
	{0x3e59, 0x40},
	{0x3f05, 0x18},
	{0x3f08, 0x10},
	{0x4401, 0x1a},
	{0x4407, 0xc0},
	{0x4418, 0x68},
	{0x4500, 0x18},
	{0x4501, 0xa4},
	{0x4503, 0xc0},
	{0x4505, 0x12},
	{0x4509, 0x10},
	{0x4825, 0x36},
	{0x4837, 0x1b},
	{0x4853, 0xf0},
	{0x5000, 0x0e},
	{0x550f, 0x20},
	{0x5900, 0x01},
	{0x5901, 0x00},
	{0x36e9, 0x53},
	{0x36f9, 0x3b},
	{0x0100, 0x01},
	{REG_NULL, 0x00},
};

/*
 * Xclk 27Mhz hdr 2to1 12bit 1920*1080 30fps 432Mbps/lane
 */
static __maybe_unused const struct regval sc2210_hdr2_12_1920x1080_30fps_regs[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x36e9, 0x80},
	{0x36f9, 0x94},
	{0x3001, 0x07},
	{0x3002, 0xc0},
	{0x300a, 0x2c},
	{0x300f, 0x00},
	{0x3018, 0x73},
	{0x3019, 0x00},
	{0x301f, 0x22},
	{0x3031, 0x0c},
	{0x3033, 0x20},
	{0x3038, 0x22},
	{0x3207, 0x3f},
	{0x320c, 0x04},
	{0x320d, 0x37},
	{0x320e, 0x08},
	{0x320f, 0xb0},
	{0x3213, 0x04},
	{0x3220, 0x53},
	{0x3231, 0x02},
	{0x3235, 0x11},
	{0x3236, 0x5e},
	{0x3250, 0x3f},
	{0x3301, 0x20},
	{0x3302, 0x10},
	{0x3304, 0x58},
	{0x3305, 0x00},
	{0x3306, 0x90},
	{0x3308, 0x20},
	{0x3309, 0x98},
	{0x330a, 0x01},
	{0x330b, 0x40},
	{0x330e, 0x68},
	{0x3314, 0x94},
	{0x331e, 0x49},
	{0x331f, 0x89},
	{0x334c, 0x10},
	{0x335d, 0x60},
	{0x3364, 0x16},
	{0x3366, 0x92},
	{0x3367, 0x10},
	{0x3000, 0xc0},
	{0x3368, 0x04},
	{0x3369, 0x00},
	{0x336a, 0x00},
	{0x336b, 0x00},
	{0x337f, 0x33},
	{0x3390, 0x10},
	{0x3391, 0x30},
	{0x3392, 0x40},
	{0x3393, 0x20},
	{0x3394, 0x20},
	{0x3395, 0x20},
	{0x3396, 0x08},
	{0x3397, 0x30},
	{0x3398, 0x3f},
	{0x3399, 0x30},
	{0x339a, 0x30},
	{0x339b, 0x30},
	{0x339c, 0x30},
	{0x33b9, 0x0e},
	{0x33e1, 0x08},
	{0x33e2, 0x18},
	{0x33e3, 0x18},
	{0x33e4, 0x18},
	{0x33e5, 0x10},
	{0x33e6, 0x06},
	{0x33e7, 0x02},
	{0x33e8, 0x18},
	{0x33e9, 0x10},
	{0x33ea, 0x0c},
	{0x33eb, 0x10},
	{0x33ec, 0x04},
	{0x33ed, 0x02},
	{0x33ee, 0xa0},
	{0x33ef, 0x08},
	{0x33f4, 0x18},
	{0x33f5, 0x10},
	{0x33f6, 0x0c},
	{0x33f7, 0x10},
	{0x33f8, 0x06},
	{0x33f9, 0x02},
	{0x33fa, 0x18},
	{0x33fb, 0x10},
	{0x33fc, 0x0c},
	{0x33fd, 0x10},
	{0x33fe, 0x04},
	{0x33ff, 0x02},
	{0x360f, 0x01},
	{0x3622, 0xf7},
	{0x3627, 0x02},
	{0x3630, 0xa5},
	{0x3631, 0x00},
	{0x3632, 0x88},
	{0x3633, 0x43},
	{0x3635, 0x20},
	{0x3638, 0x24},
	{0x363a, 0x80},
	{0x363b, 0x00},
	{0x363e, 0x22},
	{0x3670, 0x48},
	{0x3671, 0xf7},
	{0x3672, 0xf7},
	{0x3673, 0x07},
	{0x367a, 0x40},
	{0x367b, 0x7f},
	{0x3690, 0x43},
	{0x3691, 0x43},
	{0x3692, 0x54},
	{0x369c, 0x40},
	{0x369d, 0x7f},
	{0x36b5, 0x40},
	{0x36b6, 0x7f},
	{0x36c0, 0x80},
	{0x36c1, 0x9f},
	{0x36c2, 0x9f},
	{0x36cc, 0x22},
	{0x36cd, 0x28},
	{0x36ce, 0x30},
	{0x36d0, 0x20},
	{0x36d1, 0x40},
	{0x36d2, 0x7f},
	{0x36ea, 0x38},
	{0x36eb, 0x07},
	{0x36ec, 0x13},
	{0x36ed, 0x14},
	{0x36fa, 0x38},
	{0x36fb, 0x17},
	{0x36fc, 0x01},
	{0x36fd, 0x14},
	{0x3907, 0x01},
	{0x3908, 0x11},
	{0x391b, 0x87},
	{0x391f, 0x00},
	{0x3933, 0x28},
	{0x3934, 0xa6},
	{0x3940, 0x70},
	{0x3942, 0x08},
	{0x3943, 0xbc},
	{0x3980, 0x61},
	{0x3987, 0x0b},
	{0x3990, 0x00},
	{0x3991, 0x00},
	{0x3992, 0x00},
	{0x3993, 0x00},
	{0x3994, 0x00},
	{0x3995, 0x00},
	{0x3996, 0x00},
	{0x3997, 0x00},
	{0x3998, 0x00},
	{0x3999, 0x00},
	{0x399a, 0x00},
	{0x399b, 0x00},
	{0x399c, 0x00},
	{0x399d, 0x00},
	{0x399e, 0x00},
	{0x399f, 0x00},
	{0x39a0, 0x00},
	{0x39a1, 0x00},
	{0x39a2, 0x03},
	{0x39a3, 0x30},
	{0x39a4, 0x03},
	{0x39a5, 0x60},
	{0x39a6, 0x03},
	{0x39a7, 0xa0},
	{0x39a8, 0x03},
	{0x39a9, 0xb0},
	{0x39aa, 0x00},
	{0x39ab, 0x00},
	{0x39ac, 0x00},
	{0x39ad, 0x20},
	{0x39ae, 0x00},
	{0x39af, 0x70},
	{0x39b0, 0x00},
	{0x39b1, 0xa0},
	{0x39b2, 0x00},
	{0x39b3, 0xf0},
	{0x39b4, 0x08},
	{0x39b5, 0x14},
	{0x39b6, 0x20},
	{0x39b7, 0x38},
	{0x39b8, 0x38},
	{0x39b9, 0x20},
	{0x39ba, 0x14},
	{0x39bb, 0x08},
	{0x39bc, 0x08},
	{0x39bd, 0x10},
	{0x39be, 0x20},
	{0x39bf, 0x30},
	{0x39c0, 0x30},
	{0x39c1, 0x20},
	{0x39c2, 0x10},
	{0x39c3, 0x08},
	{0x39c4, 0x00},
	{0x39c5, 0x80},
	{0x39c6, 0x00},
	{0x39c7, 0x80},
	{0x39c8, 0x00},
	{0x39c9, 0x00},
	{0x39ca, 0x80},
	{0x39cb, 0x00},
	{0x39cc, 0x00},
	{0x39cd, 0x00},
	{0x39ce, 0x00},
	{0x39cf, 0x00},
	{0x39d0, 0x00},
	{0x39d1, 0x00},
	{0x39e2, 0x05},
	{0x39e3, 0xeb},
	{0x39e4, 0x07},
	{0x39e5, 0xb6},
	{0x39e6, 0x00},
	{0x39e7, 0x3a},
	{0x39e8, 0x3f},
	{0x39e9, 0xb7},
	{0x39ea, 0x02},
	{0x39eb, 0x4f},
	{0x39ec, 0x08},
	{0x39ed, 0x00},
	{0x3e00, 0x00},
	{0x3e01, 0x45},
	{0x3e02, 0x40},
	{0x3e03, 0x0b},
	{0x3e06, 0x00},
	{0x3e07, 0x80},
	{0x3e08, 0x03},
	{0x3e09, 0x40},
	{0x3e12, 0x03},
	{0x3e13, 0x40},
	{0x3e14, 0x31},
	{0x3e1b, 0x3a},
	{0x3e23, 0x00},
	{0x3e24, 0x8a},
	{0x3e26, 0x40},
	{0x3e53, 0x00},
	{0x3e54, 0x00},
	{0x3f05, 0x18},
	{0x3f08, 0x08},
	{0x4418, 0x34},
	{0x4500, 0x18},
	{0x4501, 0xb4},
	{0x4503, 0xc0},
	{0x4505, 0x12},
	{0x4509, 0x20},
	{0x4603, 0x00},
	{0x4837, 0x25},
	{0x4850, 0xab},
	{0x4851, 0x5b},
	{0x4853, 0xfd},
	{0x5000, 0x06},
	{0x36e9, 0x00},
	{0x36f9, 0x14},
	{0x0100, 0x01},
	{REG_NULL, 0x00},
};

/*
 * Xclk 27Mhz hdr2 10bit 1920*1080 45fps 594Mbps/lane
 */
static const struct regval sc2210_hdr2_10_1920x1080_45fps_regs[] = {
	{0x0103, 0x01},
	{0x0100, 0x00},
	{0x36e9, 0x80},
	{0x36f9, 0x80},
	{0x3001, 0x00},
	{0x3002, 0x00},
	{0x300a, 0x2c},
	{0x300f, 0x00},
	{0x3018, 0x73},
	{0x3019, 0x00},
	{0x301a, 0xf0},
	{0x301c, 0x78},
	{0x301f, 0x5e},
	{0x3031, 0x0a},
	{0x3032, 0x20},
	{0x3038, 0x22},
	{0x3106, 0x81},
	{0x3201, 0x04},
	{0x3203, 0x04},
	{0x3205, 0x8b},
	{0x3207, 0x43},
	{0x3208, 0x07},
	{0x3209, 0x80},
	{0x320a, 0x04},
	{0x320b, 0x38},
	{0x320c, 0x04},
	{0x320d, 0x4c},
	{0x320e, 0x09},
	{0x320f, 0x60},
	{0x3211, 0x04},
	{0x3213, 0x04},
	{0x3215, 0x11},
	{0x3220, 0x53},
	{0x3221, 0x00},
	{0x3222, 0x00},
	{0x3225, 0x04},
	{0x322e, 0x00},
	{0x322f, 0x02},
	{0x3000, 0x00},
	{0x3230, 0x00},
	{0x3231, 0x01},
	{0x3248, 0x0c},
	{0x3249, 0x18},
	{0x3250, 0x3f},
	{0x3253, 0x04},
	{0x3301, 0x14},
	{0x3302, 0x13},
	{0x3304, 0x48},
	{0x3305, 0x00},
	{0x3306, 0x88},
	{0x3308, 0x20},
	{0x3309, 0x60},
	{0x330a, 0x01},
	{0x330b, 0x18},
	{0x330d, 0x58},
	{0x330e, 0x70},
	{0x3314, 0x92},
	{0x331e, 0x39},
	{0x331f, 0x51},
	{0x3320, 0x09},
	{0x3332, 0x54},
	{0x334c, 0x10},
	{0x3350, 0x54},
	{0x3358, 0x54},
	{0x335c, 0x54},
	{0x335d, 0x60},
	{0x335e, 0x02},
	{0x335f, 0x04},
	{0x3364, 0x16},
	{0x3366, 0x92},
	{0x3367, 0x01},
	{0x337c, 0x06},
	{0x337d, 0x0a},
	{0x337e, 0x80},
	{0x3390, 0x08},
	{0x3391, 0x18},
	{0x3392, 0x38},
	{0x3393, 0x14},
	{0x3394, 0x14},
	{0x3395, 0x14},
	{0x3396, 0x08},
	{0x3397, 0x18},
	{0x3398, 0x38},
	{0x3399, 0x14},
	{0x339a, 0x30},
	{0x339b, 0x30},
	{0x339c, 0x30},
	{0x339e, 0x54},
	{0x33a0, 0x54},
	{0x33a2, 0x08},
	{0x33a4, 0x54},
	{0x33a8, 0x54},
	{0x33aa, 0x54},
	{0x33b0, 0x0f},
	{0x33b9, 0x11},
	{0x33e0, 0xc8},
	{0x33e1, 0x08},
	{0x33e2, 0x18},
	{0x33e3, 0x10},
	{0x33e4, 0x08},
	{0x33e5, 0x10},
	{0x33e6, 0x08},
	{0x33e7, 0x04},
	{0x33e8, 0x18},
	{0x33e9, 0x10},
	{0x33ea, 0x08},
	{0x33eb, 0x18},
	{0x33ec, 0x10},
	{0x33ed, 0x08},
	{0x33ee, 0xc8},
	{0x33ef, 0x08},
	{0x33f4, 0x18},
	{0x33f5, 0x10},
	{0x33f6, 0x08},
	{0x33f7, 0x10},
	{0x33f8, 0x08},
	{0x33f9, 0x04},
	{0x33fa, 0x18},
	{0x33fb, 0x10},
	{0x33fc, 0x08},
	{0x33fd, 0x18},
	{0x33fe, 0x10},
	{0x33ff, 0x08},
	{0x360f, 0x01},
	{0x3622, 0xf7},
	{0x3625, 0x0a},
	{0x3627, 0x82},
	{0x3630, 0xb8},
	{0x3631, 0x00},
	{0x3632, 0xd8},
	{0x3633, 0x43},
	{0x3635, 0x20},
	{0x3638, 0x27},
	{0x363a, 0x80},
	{0x363b, 0x02},
	{0x363e, 0x22},
	{0x3670, 0x4a},
	{0x3671, 0xf7},
	{0x3672, 0xf7},
	{0x3673, 0xf7},
	{0x3674, 0xd0},
	{0x3675, 0x90},
	{0x3676, 0x8a},
	{0x367a, 0x40},
	{0x367b, 0x78},
	{0x367c, 0x40},
	{0x367d, 0x78},
	{0x3690, 0x43},
	{0x3691, 0x54},
	{0x3692, 0x54},
	{0x369c, 0x40},
	{0x369d, 0x78},
	{0x36b5, 0x40},
	{0x36b6, 0x78},
	{0x36c0, 0x9f},
	{0x36c1, 0x9f},
	{0x36c2, 0x9f},
	{0x36cc, 0x22},
	{0x36cd, 0x22},
	{0x36ce, 0x28},
	{0x36d0, 0x20},
	{0x36d1, 0x40},
	{0x36d2, 0x78},
	{0x36ea, 0x35},
	{0x36eb, 0x04},
	{0x36ec, 0x0a},
	{0x36ed, 0x14},
	{0x36fa, 0x35},
	{0x36fb, 0x04},
	{0x36fc, 0x00},
	{0x36fd, 0x14},
	{0x3901, 0x02},
	{0x3902, 0x45},
	{0x3904, 0x08},
	{0x3905, 0x98},
	{0x3907, 0x01},
	{0x3908, 0x11},
	{0x391b, 0x87},
	{0x391d, 0x2c},
	{0x391f, 0x00},
	{0x3933, 0x28},
	{0x3934, 0x2c},
	{0x3940, 0x6f},
	{0x3942, 0x08},
	{0x3943, 0x2c},
	{0x3958, 0x04},
	{0x3959, 0x02},
	{0x3980, 0x61},
	{0x3987, 0x0b},
	{0x3990, 0x00},
	{0x3991, 0x00},
	{0x3992, 0x00},
	{0x3993, 0x00},
	{0x3994, 0x00},
	{0x3995, 0x00},
	{0x3996, 0x00},
	{0x3997, 0x00},
	{0x3998, 0x00},
	{0x3999, 0x00},
	{0x399a, 0x00},
	{0x399b, 0x00},
	{0x399c, 0x00},
	{0x399d, 0x00},
	{0x399e, 0x00},
	{0x399f, 0x00},
	{0x39a0, 0x00},
	{0x39a1, 0x00},
	{0x39a2, 0x03},
	{0x39a3, 0x30},
	{0x39a4, 0x03},
	{0x39a5, 0x60},
	{0x39a6, 0x03},
	{0x39a7, 0xa0},
	{0x39a8, 0x03},
	{0x39a9, 0xb0},
	{0x39aa, 0x00},
	{0x39ab, 0x00},
	{0x39ac, 0x00},
	{0x39ad, 0x20},
	{0x39ae, 0x00},
	{0x39af, 0x40},
	{0x39b0, 0x00},
	{0x39b1, 0x60},
	{0x39b2, 0x00},
	{0x39b3, 0x00},
	{0x39b4, 0x08},
	{0x39b5, 0x14},
	{0x39b6, 0x20},
	{0x39b7, 0x38},
	{0x39b8, 0x38},
	{0x39b9, 0x20},
	{0x39ba, 0x14},
	{0x39bb, 0x08},
	{0x39bc, 0x08},
	{0x39bd, 0x10},
	{0x39be, 0x20},
	{0x39bf, 0x30},
	{0x39c0, 0x30},
	{0x39c1, 0x20},
	{0x39c2, 0x10},
	{0x39c3, 0x08},
	{0x39c4, 0x00},
	{0x39c5, 0x80},
	{0x39c6, 0x00},
	{0x39c7, 0x80},
	{0x39c8, 0x00},
	{0x39c9, 0x00},
	{0x39ca, 0x80},
	{0x39cb, 0x00},
	{0x39cc, 0x00},
	{0x39cd, 0x00},
	{0x39ce, 0x00},
	{0x39cf, 0x00},
	{0x39d0, 0x00},
	{0x39d1, 0x00},
	{0x39e2, 0x05},
	{0x39e3, 0xeb},
	{0x39e4, 0x07},
	{0x39e5, 0xb6},
	{0x39e6, 0x00},
	{0x39e7, 0x3a},
	{0x39e8, 0x3f},
	{0x39e9, 0xb7},
	{0x39ea, 0x02},
	{0x39eb, 0x4f},
	{0x39ec, 0x08},
	{0x39ed, 0x00},
	{0x3e00, 0x00},
	{0x3e01, 0x8c},
	{0x3e02, 0x00},
	{0x3e04, 0x08},
	{0x3e05, 0xc0},
	{0x3e06, 0x00},
	{0x3e07, 0x80},
	{0x3e08, 0x03},
	{0x3e09, 0x40},
	{0x3e10, 0x00},
	{0x3e11, 0x80},
	{0x3e12, 0x03},
	{0x3e13, 0x40},
	{0x3e14, 0x31},
	{0x3e16, 0x00},
	{0x3e17, 0x80},
	{0x3e18, 0x00},
	{0x3e19, 0x80},
	{0x3e1b, 0x3a},
	{0x3e22, 0x00},
	{0x3e23, 0x00},
	{0x3e24, 0x92},
	{0x3e26, 0x40},
	{0x3e50, 0x00},
	{0x3e51, 0x0c},
	{0x3e52, 0x00},
	{0x3e53, 0x00},
	{0x3e54, 0xc9},
	{0x3e56, 0x00},
	{0x3e57, 0x80},
	{0x3e58, 0x03},
	{0x3e59, 0x40},
	{0x3f05, 0x18},
	{0x3f08, 0x10},
	{0x4401, 0x1a},
	{0x4407, 0xc0},
	{0x4418, 0x68},
	{0x4500, 0x18},
	{0x4501, 0xa4},
	{0x4503, 0xc0},
	{0x4505, 0x12},
	{0x4509, 0x10},
	{0x4814, 0x2a},
	{0x4825, 0x36},
	{0x4837, 0x1b},
	{0x4851, 0xab},
	{0x4853, 0xf8},
	{0x5000, 0x0e},
	{0x550f, 0x20},
	{0x5900, 0x01},
	{0x5901, 0x00},
	{0x36e9, 0x20},
	{0x36f9, 0x30},
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
static const struct sc2210_mode supported_modes[] = {
	{
		/* linear modes */
		.bus_fmt = MEDIA_BUS_FMT_Y10_1X10,
		.width = 1920,
		.height = 544,
		.max_fps = {
			.numerator = 10000,
			.denominator = 1800000,
		},
		.exp_def = 0x0255 / 2,
		.hts_def = 0x044c * 2, // REG{0x320c,0x320d}
		.vts_def = 0x0258, // REG{0x320e,0x320f}
		.reg_list = sc2210_linear_10_1920x544_180fps_regs,
		.hdr_mode = NO_HDR,
		.mipi_freq_idx = 1, // 594.00 Mbps
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
	{
		/* linear modes */
		.bus_fmt = MEDIA_BUS_FMT_Y12_1X12,
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.exp_def = 0x0455 / 2,
		.hts_def = 0x0437 * 2, // REG{0x320c,0x320d}
		.vts_def = 0x0458, // REG{0x320e,0x320f}
		.reg_list = sc2210_linear_12_1920x1080_60fps_regs,
		.hdr_mode = NO_HDR,
		.mipi_freq_idx = 1,
		.bpp = 12,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
	{
		/* linear modes */
		.bus_fmt = MEDIA_BUS_FMT_Y10_1X10,
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 900000,
		},
		.exp_def = 0x04ad / 2,
		.hts_def = 0x044c * 2, // REG{0x320c,0x320d}
		.vts_def = 0x04b0, // REG{0x320e,0x320f}
		.reg_list = sc2210_linear_10_1920x1080_90fps_regs,
		.hdr_mode = NO_HDR,
		.mipi_freq_idx = 1,
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
	{
		/* 2 to 1 hdr */
		.bus_fmt = MEDIA_BUS_FMT_Y12_1X12,
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x08ad / 2,
		.hts_def = 0x0437 * 2, // REG{0x320c,0x320d}
		.vts_def = 0x08b0, // REG{0x320e,0x320f}
		.reg_list = sc2210_hdr2_12_1920x1080_30fps_regs,
		.hdr_mode = HDR_X2,
		.mipi_freq_idx = 1,
		.bpp = 12,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_1,
		.vc[PAD1] = V4L2_MBUS_CSI2_CHANNEL_0,//L->csi wr0
		.vc[PAD2] = V4L2_MBUS_CSI2_CHANNEL_1,
		.vc[PAD3] = V4L2_MBUS_CSI2_CHANNEL_1,//M->csi wr2
	},
#if 0
	{
		/* 2 to 1 hdr */
		.bus_fmt = MEDIA_BUS_FMT_Y10_1X10,
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 450000,
		},
		.exp_def = 0x0440 / 2,
		.hts_def = 0x044c * 2, // REG{0x320c,0x320d}
		.vts_def = 0x0960, // REG{0x320e,0x320f}
		.reg_list = sc2210_hdr2_10_1920x1080_45fps_regs,
		.hdr_mode = HDR_X2,
		.mipi_freq_idx = 1,
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_1,
		.vc[PAD1] = V4L2_MBUS_CSI2_CHANNEL_0, //L->csi wr0
		.vc[PAD2] = V4L2_MBUS_CSI2_CHANNEL_1,
		.vc[PAD3] = V4L2_MBUS_CSI2_CHANNEL_1, //S->csi wr2
	},
#endif
};

static const s64 link_freq_items[] = {
	MIPI_FREQ_186M,
	MIPI_FREQ_297M,
	MIPI_FREQ_432M,
};

static const char * const sc2210_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1"
};

/* Write registers up to 4 at a time */
static int sc2210_write_reg(struct i2c_client *client, u16 reg,
			    u32 len, u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

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

	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

static int sc2210_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
		ret |= sc2210_write_reg(client, regs[i].addr,
			SC2210_REG_VALUE_08BIT, regs[i].val);
	}
	return ret;
}

/* Read registers up to 4 at a time */
static int sc2210_read_reg(struct i2c_client *client,
			    u16 reg,
			    unsigned int len,
			    u32 *val)
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

static int sc2210_get_reso_dist(const struct sc2210_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct sc2210_mode *
sc2210_find_best_fit(struct sc2210 *sc2210, struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < sc2210->cfg_num; i++) {
		dist = sc2210_get_reso_dist(&supported_modes[i], framefmt);
		if ((cur_best_fit_dist == -1 || dist <= cur_best_fit_dist) &&
			(supported_modes[i].bus_fmt == framefmt->code)) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static void sc2210_change_mode(struct sc2210 *sc2210, const struct sc2210_mode *mode)
{
	sc2210->cur_mode = mode;
	sc2210->cur_vts = sc2210->cur_mode->vts_def;
	dev_info(&sc2210->client->dev, "set fmt: cur_mode: %dx%d@%d/%d, hdr: %d\n",
		mode->width, mode->height, mode->max_fps.numerator,
		mode->max_fps.denominator, mode->hdr_mode);
}

static int sc2210_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct sc2210 *sc2210 = to_sc2210(sd);
	const struct sc2210_mode *mode;
	s64 h_blank, vblank_def;
	u64 pixel_rate = 0;

	mutex_lock(&sc2210->mutex);

	mode = sc2210_find_best_fit(sc2210, fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&sc2210->mutex);
		return -ENOTTY;
#endif
	} else {
		sc2210_change_mode(sc2210, mode);
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(sc2210->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(sc2210->vblank, vblank_def,
					 SC2210_VTS_MAX - mode->height,
					 1, vblank_def);
		__v4l2_ctrl_s_ctrl(sc2210->link_freq, mode->mipi_freq_idx);
		pixel_rate = (u32)link_freq_items[mode->mipi_freq_idx] /
			mode->bpp * 2 * SC2210_LANES;
		__v4l2_ctrl_s_ctrl_int64(sc2210->pixel_rate, pixel_rate);
	}

	mutex_unlock(&sc2210->mutex);

	return 0;
}

static int sc2210_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct sc2210 *sc2210 = to_sc2210(sd);
	const struct sc2210_mode *mode = sc2210->cur_mode;

	mutex_lock(&sc2210->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&sc2210->mutex);
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
	mutex_unlock(&sc2210->mutex);

	return 0;
}

static int sc2210_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct sc2210 *sc2210 = to_sc2210(sd);

	if (code->index != 0)
		return -EINVAL;
	code->code = sc2210->cur_mode->bus_fmt;

	return 0;
}

static int sc2210_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct sc2210 *sc2210 = to_sc2210(sd);

	if (fse->index >= sc2210->cfg_num)
		return -EINVAL;

	if (fse->code != supported_modes[fse->index].bus_fmt)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int sc2210_enable_test_pattern(struct sc2210 *sc2210, u32 pattern)
{
	u32 val = 0;
	int ret = 0;

	ret = sc2210_read_reg(sc2210->client, SC2210_REG_TEST_PATTERN,
			      SC2210_REG_VALUE_08BIT, &val);
	if (pattern)
		val |= SC2210_TEST_PATTERN_ENABLE;
	else
		val &= ~SC2210_TEST_PATTERN_ENABLE;
	ret |= sc2210_write_reg(sc2210->client, SC2210_REG_TEST_PATTERN,
				SC2210_REG_VALUE_08BIT, val);
	return ret;
}

static int sc2210_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct sc2210 *sc2210 = to_sc2210(sd);
	const struct sc2210_mode *mode = sc2210->cur_mode;

	mutex_lock(&sc2210->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&sc2210->mutex);

	return 0;
}

static int sc2210_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *config)
{
	struct sc2210 *sc2210 = to_sc2210(sd);
	const struct sc2210_mode *mode = sc2210->cur_mode;
	u32 val = 0;

	if (mode->hdr_mode == NO_HDR)
		val = 1 << (SC2210_LANES - 1) |
		V4L2_MBUS_CSI2_CHANNEL_0 |
		V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	if (mode->hdr_mode == HDR_X2)
		val = 1 << (SC2210_LANES - 1) |
		V4L2_MBUS_CSI2_CHANNEL_0 |
		V4L2_MBUS_CSI2_CONTINUOUS_CLOCK |
		V4L2_MBUS_CSI2_CHANNEL_1;

	config->type = V4L2_MBUS_CSI2;
	config->flags = val;

	return 0;
}

static void sc2210_get_module_inf(struct sc2210 *sc2210,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, SC2210_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, sc2210->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, sc2210->len_name, sizeof(inf->base.lens));
}

static void sc2210_get_gain_reg(u32 val, u32 *again_reg, u32 *again_fine_reg,
	u32 *dgain_reg, u32 *dgain_fine_reg)
{
	u8 u8Reg0x3e09 = 0x40, u8Reg0x3e08 = 0x03, u8Reg0x3e07 = 0x80, u8Reg0x3e06 = 0x00;
	u32 aCoarseGain = 0;
	u32 aFineGain = 0;
	u32 dCoarseGain = 0;
	u32 dFineGain = 0;
	u32 again = 0;
	u32 dgain = 0;

	if (val <= 2764) {
		again = val;
		dgain = 128;
	} else {
		again = 2764;
		dgain = val * 128 / again;
	}

	//again
	if (again <= 174) {
		//a_gain < 2.72x
		for (aCoarseGain = 1; aCoarseGain <= 2; aCoarseGain = aCoarseGain * 2) {
			//1,2,4,8,16
			if (again < (64 * 2 * aCoarseGain))
				break;
		}

		aFineGain = again / aCoarseGain;
	} else {
		for (aCoarseGain = 1; aCoarseGain <= 8; aCoarseGain = aCoarseGain * 2) {
			//1,2,4,8
			if (again < (64 * 2 * aCoarseGain * 272 / 100))
				break;
		}
		aFineGain = 100 * again / aCoarseGain / 272;
	}
	for ( ; aCoarseGain >= 2; aCoarseGain = aCoarseGain / 2)
		u8Reg0x3e08 = (u8Reg0x3e08 << 1) | 0x01;

	u8Reg0x3e09 = aFineGain;

	//dcg = 2.72	-->		2.72*1024=2785.28
	u8Reg0x3e08 = (again > 174) ? (u8Reg0x3e08 | 0x20) : (u8Reg0x3e08 & 0x1f);

	//------------------------------------------------------
	//dgain
	for (dCoarseGain = 1; dCoarseGain <= 16; dCoarseGain = dCoarseGain * 2) {
		//1,2,4,8,16
		if (dgain < (256 * dCoarseGain))
			break;
	}
	dFineGain = dgain / dCoarseGain;

	for ( ; dCoarseGain >= 2; dCoarseGain = dCoarseGain / 2)
		u8Reg0x3e06 = (u8Reg0x3e06 << 1) | 0x01;

	u8Reg0x3e07 = dFineGain;

	*again_reg = u8Reg0x3e08;
	*again_fine_reg = u8Reg0x3e09;
	*dgain_reg = u8Reg0x3e06;
	*dgain_fine_reg = u8Reg0x3e07;

}

static int sc2210_set_hdrae(struct sc2210 *sc2210,
			     struct preisp_hdrae_exp_s *ae)
{
	struct i2c_client *client = sc2210->client;
	u32 l_exp_time, m_exp_time, s_exp_time;
	u32 l_a_gain, m_a_gain, s_a_gain;
	u32 l_again, l_again_fine, l_dgain, l_dgain_fine;
	u32 s_again, s_again_fine, s_dgain, s_dgain_fine;
	int ret = 0;

	if (!sc2210->has_init_exp && !sc2210->streaming) {
		sc2210->init_hdrae_exp = *ae;
		sc2210->has_init_exp = true;
		dev_dbg(&client->dev, "sc2210 is not streaming, save hdr ae!\n");
		return ret;
	}

	l_exp_time = ae->long_exp_reg;
	m_exp_time = ae->middle_exp_reg;
	s_exp_time = ae->short_exp_reg;
	l_a_gain = ae->long_gain_reg;
	m_a_gain = ae->middle_gain_reg;
	s_a_gain = ae->short_gain_reg;
	dev_dbg(&client->dev,
		"rev exp req: L_exp: 0x%x, 0x%x, M_exp: 0x%x, 0x%x S_exp: 0x%x, 0x%x\n",
		l_exp_time, m_exp_time, s_exp_time,
		l_a_gain, m_a_gain, s_a_gain);

	if (sc2210->cur_mode->hdr_mode == HDR_X2) {
		l_a_gain = m_a_gain;
		l_exp_time = m_exp_time;
	}
	sc2210_get_gain_reg(l_a_gain, &l_again, &l_again_fine, &l_dgain, &l_dgain_fine);
	sc2210_get_gain_reg(s_a_gain, &s_again, &s_again_fine, &s_dgain, &s_dgain_fine);

	l_exp_time = l_exp_time << 1;
	s_exp_time = s_exp_time << 1;

	if (l_exp_time > LONG_FRAME_MAX_EXP)
		l_exp_time = LONG_FRAME_MAX_EXP;

	if (s_exp_time > SHORT_FRAME_MAX_EXP)
		s_exp_time = SHORT_FRAME_MAX_EXP;

	ret |= sc2210_write_reg(sc2210->client,
					SC2210_REG_EXP_LONG_L,
					SC2210_REG_VALUE_08BIT,
					(l_exp_time << 4 & 0XF0));
	ret |= sc2210_write_reg(sc2210->client,
					SC2210_REG_EXP_LONG_M,
					SC2210_REG_VALUE_08BIT,
					(l_exp_time >> 4 & 0XFF));
	ret |= sc2210_write_reg(sc2210->client,
					SC2210_REG_EXP_LONG_H,
					SC2210_REG_VALUE_08BIT,
					(l_exp_time >> 12 & 0X0F));

	ret |= sc2210_write_reg(sc2210->client,
					SC2210_REG_AGAIN,
					SC2210_REG_VALUE_08BIT,
					l_again);
	ret |= sc2210_write_reg(sc2210->client,
					SC2210_REG_AGAIN_FINE,
					SC2210_REG_VALUE_08BIT,
					l_again_fine);
	ret |= sc2210_write_reg(sc2210->client,
					SC2210_REG_DGAIN,
					SC2210_REG_VALUE_08BIT,
					l_dgain);
	ret |= sc2210_write_reg(sc2210->client,
					SC2210_REG_DGAIN_FINE,
					SC2210_REG_VALUE_08BIT,
					l_dgain_fine);

	ret |= sc2210_write_reg(sc2210->client,
					SC2210_REG_EXP_SF_L,
					SC2210_REG_VALUE_08BIT,
					(s_exp_time << 4 & 0XF0));
	ret |= sc2210_write_reg(sc2210->client,
					SC2210_REG_EXP_SF_H,
					SC2210_REG_VALUE_08BIT,
					(s_exp_time >> 4 & 0XFF));

	ret |= sc2210_write_reg(sc2210->client,
					SC2210_SF_REG_AGAIN,
					SC2210_REG_VALUE_08BIT,
					s_again);
	ret |= sc2210_write_reg(sc2210->client,
					SC2210_SF_REG_AGAIN_FINE,
					SC2210_REG_VALUE_08BIT,
					s_again_fine);
	ret |= sc2210_write_reg(sc2210->client,
					SC2210_SF_REG_DGAIN,
					SC2210_REG_VALUE_08BIT,
					s_dgain);
	ret |= sc2210_write_reg(sc2210->client,
					SC2210_SF_REG_DGAIN_FINE,
					SC2210_REG_VALUE_08BIT,
					s_dgain_fine);
	if (ret)
		return ret;
	return 0;
}

static long sc2210_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct sc2210 *sc2210 = to_sc2210(sd);
	struct rkmodule_hdr_cfg *hdr_cfg;
	const struct sc2210_mode *mode;
	long ret = 0;
	u64 pixel_rate = 0;
	u32 i, h, w, stream;

	switch (cmd) {
	case PREISP_CMD_SET_HDRAE_EXP:
		ret = sc2210_set_hdrae(sc2210, arg);
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		if (sc2210->streaming) {
			ret = sc2210_write_array(sc2210->client, sc2210->cur_mode->reg_list);
			if (ret)
				return ret;
		}
		w = sc2210->cur_mode->width;
		h = sc2210->cur_mode->height;
		for (i = 0; i < sc2210->cfg_num; i++) {
			if (w == supported_modes[i].width &&
			h == supported_modes[i].height &&
			supported_modes[i].hdr_mode == hdr_cfg->hdr_mode) {
				sc2210_change_mode(sc2210, &supported_modes[i]);
				break;
			}
		}

		if (i == sc2210->cfg_num) {
			dev_err(&sc2210->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr_cfg->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			mode = sc2210->cur_mode;
			w = mode->hts_def - mode->width;
			h = mode->vts_def - mode->height;
			__v4l2_ctrl_modify_range(sc2210->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(sc2210->vblank, h,
				SC2210_VTS_MAX - mode->height,
				1, h);
			__v4l2_ctrl_s_ctrl(sc2210->link_freq, mode->mipi_freq_idx);
			pixel_rate = (u32)link_freq_items[mode->mipi_freq_idx] /
				mode->bpp * 2 * SC2210_LANES;
			__v4l2_ctrl_s_ctrl_int64(sc2210->pixel_rate,
						 pixel_rate);
			dev_info(&sc2210->client->dev,
				"sensor mode: %d\n", mode->hdr_mode);
		}
		break;
	case RKMODULE_GET_MODULE_INFO:
		sc2210_get_module_inf(sc2210, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		hdr_cfg->esp.mode = HDR_NORMAL_VC;
		hdr_cfg->hdr_mode = sc2210->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (stream)
			ret = sc2210_write_reg(sc2210->client, SC2210_REG_CTRL_MODE,
				SC2210_REG_VALUE_08BIT, SC2210_MODE_STREAMING);
		else
			ret = sc2210_write_reg(sc2210->client, SC2210_REG_CTRL_MODE,
				SC2210_REG_VALUE_08BIT, SC2210_MODE_SW_STANDBY);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long sc2210_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
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

		ret = sc2210_ioctl(sd, cmd, inf);
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
		ret = sc2210_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = sc2210_ioctl(sd, cmd, hdr);
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
		ret = sc2210_ioctl(sd, cmd, hdr);
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
		ret = sc2210_ioctl(sd, cmd, hdrae);
		kfree(hdrae);
		break;
	case RKMODULE_SET_CONVERSION_GAIN:
		if (copy_from_user(&cg, up, sizeof(cg)))
			return -EFAULT;
		ret = sc2210_ioctl(sd, cmd, &cg);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		if (copy_from_user(&stream, up, sizeof(u32)))
			return -EFAULT;
		ret = sc2210_ioctl(sd, cmd, &stream);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __sc2210_start_stream(struct sc2210 *sc2210)
{
	int ret;

	ret = sc2210_write_array(sc2210->client, sc2210->cur_mode->reg_list);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_handler_setup(&sc2210->ctrl_handler);
	if (ret)
		return ret;

	/* In case these controls are set before streaming */
	if (sc2210->has_init_exp && sc2210->cur_mode->hdr_mode != NO_HDR) {
		ret = sc2210_ioctl(&sc2210->subdev, PREISP_CMD_SET_HDRAE_EXP,
			&sc2210->init_hdrae_exp);
		if (ret) {
			dev_err(&sc2210->client->dev,
				"init exp fail in hdr mode\n");
			return ret;
		}
	}

	return sc2210_write_reg(sc2210->client, SC2210_REG_CTRL_MODE,
		SC2210_REG_VALUE_08BIT, SC2210_MODE_STREAMING);
}

static int __sc2210_stop_stream(struct sc2210 *sc2210)
{
	sc2210->has_init_exp = false;
	return sc2210_write_reg(sc2210->client, SC2210_REG_CTRL_MODE,
		SC2210_REG_VALUE_08BIT, SC2210_MODE_SW_STANDBY);
}

static int sc2210_s_stream(struct v4l2_subdev *sd, int on)
{
	struct sc2210 *sc2210 = to_sc2210(sd);
	struct i2c_client *client = sc2210->client;
	int ret = 0;

	mutex_lock(&sc2210->mutex);
	on = !!on;
	if (on == sc2210->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __sc2210_start_stream(sc2210);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__sc2210_stop_stream(sc2210);
		pm_runtime_put(&client->dev);
	}

	sc2210->streaming = on;

unlock_and_return:
	mutex_unlock(&sc2210->mutex);

	return ret;
}

static int sc2210_s_power(struct v4l2_subdev *sd, int on)
{
	struct sc2210 *sc2210 = to_sc2210(sd);
	struct i2c_client *client = sc2210->client;
	int ret = 0;

	mutex_lock(&sc2210->mutex);

	/* If the power state is not modified - no work to do. */
	if (sc2210->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret |= sc2210_write_reg(sc2210->client,
			SC2210_SOFTWARE_RESET_REG,
			SC2210_REG_VALUE_08BIT,
			0x01);
		usleep_range(100, 200);
		ret |= sc2210_write_reg(sc2210->client,
			0x303f,
			SC2210_REG_VALUE_08BIT,
			0x01);

		sc2210->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		sc2210->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&sc2210->mutex);

	return ret;
}

static int __sc2210_power_on(struct sc2210 *sc2210)
{
	int ret;
	struct device *dev = &sc2210->client->dev;

	if (!IS_ERR_OR_NULL(sc2210->pins_default)) {
		ret = pinctrl_select_state(sc2210->pinctrl,
					   sc2210->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(sc2210->xvclk, SC2210_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (27MHz)\n");
	if (clk_get_rate(sc2210->xvclk) != SC2210_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 27MHz\n");
	ret = clk_prepare_enable(sc2210->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}
	if (!IS_ERR(sc2210->reset_gpio))
		gpiod_set_value_cansleep(sc2210->reset_gpio, 1);

	ret = regulator_bulk_enable(SC2210_NUM_SUPPLIES, sc2210->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(sc2210->reset_gpio))
		gpiod_set_value_cansleep(sc2210->reset_gpio, 0);

	usleep_range(500, 1000);
	if (!IS_ERR(sc2210->pwdn_gpio))
		gpiod_set_value_cansleep(sc2210->pwdn_gpio, 0);
	usleep_range(2000, 4000);

	return 0;

disable_clk:
	clk_disable_unprepare(sc2210->xvclk);

	return ret;
}

static void __sc2210_power_off(struct sc2210 *sc2210)
{
	int ret;
	struct device *dev = &sc2210->client->dev;

	if (!IS_ERR(sc2210->pwdn_gpio))
		gpiod_set_value_cansleep(sc2210->pwdn_gpio, 1);
	clk_disable_unprepare(sc2210->xvclk);
	if (!IS_ERR(sc2210->reset_gpio))
		gpiod_set_value_cansleep(sc2210->reset_gpio, 1);
	if (!IS_ERR_OR_NULL(sc2210->pins_sleep)) {
		ret = pinctrl_select_state(sc2210->pinctrl,
					   sc2210->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(SC2210_NUM_SUPPLIES, sc2210->supplies);
}

static int sc2210_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc2210 *sc2210 = to_sc2210(sd);

	return __sc2210_power_on(sc2210);
}

static int sc2210_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc2210 *sc2210 = to_sc2210(sd);

	__sc2210_power_off(sc2210);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int sc2210_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct sc2210 *sc2210 = to_sc2210(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct sc2210_mode *def_mode = &supported_modes[0];

	mutex_lock(&sc2210->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&sc2210->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int sc2210_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	struct sc2210 *sc2210 = to_sc2210(sd);

	if (fie->index >= sc2210->cfg_num)
		return -EINVAL;

	fie->code = supported_modes[fie->index].bus_fmt;
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	fie->reserved[0] = supported_modes[fie->index].hdr_mode;
	return 0;
}

static const struct dev_pm_ops sc2210_pm_ops = {
	SET_RUNTIME_PM_OPS(sc2210_runtime_suspend,
			   sc2210_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops sc2210_internal_ops = {
	.open = sc2210_open,
};
#endif

static const struct v4l2_subdev_core_ops sc2210_core_ops = {
	.s_power = sc2210_s_power,
	.ioctl = sc2210_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = sc2210_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops sc2210_video_ops = {
	.s_stream = sc2210_s_stream,
	.g_frame_interval = sc2210_g_frame_interval,
	.g_mbus_config = sc2210_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops sc2210_pad_ops = {
	.enum_mbus_code = sc2210_enum_mbus_code,
	.enum_frame_size = sc2210_enum_frame_sizes,
	.enum_frame_interval = sc2210_enum_frame_interval,
	.get_fmt = sc2210_get_fmt,
	.set_fmt = sc2210_set_fmt,
};

static const struct v4l2_subdev_ops sc2210_subdev_ops = {
	.core	= &sc2210_core_ops,   /* v4l2_subdev_core_ops sc2210_core_ops */
	.video	= &sc2210_video_ops,  /* */
	.pad	= &sc2210_pad_ops,    /* */
};

static int sc2210_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sc2210 *sc2210 = container_of(ctrl->handler,
					     struct sc2210, ctrl_handler);
	struct i2c_client *client = sc2210->client;
	s64 max;
	u32 again, again_fine, dgain, dgain_fine;
	int ret = 0;
	u32 val;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = sc2210->cur_mode->height + ctrl->val - 3;
		__v4l2_ctrl_modify_range(sc2210->exposure,
					 sc2210->exposure->minimum, max,
					 sc2210->exposure->step,
					 sc2210->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		if (sc2210->cur_mode->hdr_mode != NO_HDR)
			goto out_ctrl;
		val = ctrl->val << 1;
		ret = sc2210_write_reg(sc2210->client,
					SC2210_REG_EXP_LONG_L,
					SC2210_REG_VALUE_08BIT,
					(val << 4 & 0XF0));
		ret |= sc2210_write_reg(sc2210->client,
					SC2210_REG_EXP_LONG_M,
					SC2210_REG_VALUE_08BIT,
					(val >> 4 & 0XFF));
		ret |= sc2210_write_reg(sc2210->client,
					SC2210_REG_EXP_LONG_H,
					SC2210_REG_VALUE_08BIT,
					(val >> 12 & 0X0F));
		dev_dbg(&client->dev, "set exposure 0x%x\n", val);
		break;

	case V4L2_CID_ANALOGUE_GAIN:
		if (sc2210->cur_mode->hdr_mode != NO_HDR)
			goto out_ctrl;
		sc2210_get_gain_reg(ctrl->val, &again, &again_fine, &dgain, &dgain_fine);
		dev_dbg(&client->dev, "recv:%d set again 0x%x, again_fine 0x%x, set dgain 0x%x, dgain_fine 0x%x\n",
			ctrl->val, again, again_fine, dgain, dgain_fine);

		ret |= sc2210_write_reg(sc2210->client,
					SC2210_REG_AGAIN,
					SC2210_REG_VALUE_08BIT,
					again);
		ret |= sc2210_write_reg(sc2210->client,
					SC2210_REG_AGAIN_FINE,
					SC2210_REG_VALUE_08BIT,
					again_fine);
		ret |= sc2210_write_reg(sc2210->client,
					SC2210_REG_DGAIN,
					SC2210_REG_VALUE_08BIT,
					dgain);
		ret |= sc2210_write_reg(sc2210->client,
					SC2210_REG_DGAIN_FINE,
					SC2210_REG_VALUE_08BIT,
					dgain_fine);
		break;
	case V4L2_CID_VBLANK:
		ret = sc2210_write_reg(sc2210->client, SC2210_REG_VTS,
					SC2210_REG_VALUE_16BIT,
					ctrl->val + sc2210->cur_mode->height);
		dev_dbg(&client->dev, "set vblank 0x%x\n",
			ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = sc2210_enable_test_pattern(sc2210, ctrl->val);
		break;
	case V4L2_CID_HFLIP:
		ret = sc2210_read_reg(sc2210->client, SC2210_FLIP_REG,
				      SC2210_REG_VALUE_08BIT, &val);
		if (ret)
			break;
		if (ctrl->val)
			val |= SC2210_MIRROR_MASK;
		else
			val &= ~SC2210_MIRROR_MASK;
		ret |= sc2210_write_reg(sc2210->client, SC2210_FLIP_REG,
					SC2210_REG_VALUE_08BIT, val);
		break;
	case V4L2_CID_VFLIP:
		ret = sc2210_read_reg(sc2210->client, SC2210_FLIP_REG,
				      SC2210_REG_VALUE_08BIT, &val);
		if (ret)
			break;
		if (ctrl->val)
			val |= SC2210_FLIP_MASK;
		else
			val &= ~SC2210_FLIP_MASK;
		ret |= sc2210_write_reg(sc2210->client, SC2210_FLIP_REG,
					SC2210_REG_VALUE_08BIT, val);
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

static const struct v4l2_ctrl_ops sc2210_ctrl_ops = {
	.s_ctrl = sc2210_set_ctrl,
};

static int sc2210_initialize_controls(struct sc2210 *sc2210)
{
	const struct sc2210_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;
	u64 pixel_rate = 0;

	handler = &sc2210->ctrl_handler;
	mode = sc2210->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &sc2210->mutex;

	sc2210->link_freq = v4l2_ctrl_new_int_menu(handler, NULL,
			V4L2_CID_LINK_FREQ,
			ARRAY_SIZE(link_freq_items) - 1, 0,
			link_freq_items);
	__v4l2_ctrl_s_ctrl(sc2210->link_freq, mode->mipi_freq_idx);

	/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
	pixel_rate = (u32)link_freq_items[mode->mipi_freq_idx] / mode->bpp * 2 * SC2210_LANES;
	sc2210->pixel_rate = v4l2_ctrl_new_std(handler, NULL,
		V4L2_CID_PIXEL_RATE, 0, SC2210_MAX_PIXEL_RATE,
		1, pixel_rate);

	h_blank = mode->hts_def - mode->width;
	sc2210->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (sc2210->hblank)
		sc2210->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	sc2210->vblank = v4l2_ctrl_new_std(handler, &sc2210_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				SC2210_VTS_MAX - mode->height,
				1, vblank_def);

	exposure_max = mode->vts_def - 3;
	sc2210->exposure = v4l2_ctrl_new_std(handler, &sc2210_ctrl_ops,
				V4L2_CID_EXPOSURE, SC2210_EXPOSURE_MIN,
				exposure_max, SC2210_EXPOSURE_STEP,
				mode->exp_def);

	sc2210->anal_gain = v4l2_ctrl_new_std(handler, &sc2210_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, SC2210_GAIN_MIN,
				SC2210_GAIN_MAX, SC2210_GAIN_STEP,
				SC2210_GAIN_DEFAULT);

	sc2210->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				&sc2210_ctrl_ops, V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(sc2210_test_pattern_menu) - 1,
				0, 0, sc2210_test_pattern_menu);

	v4l2_ctrl_new_std(handler, &sc2210_ctrl_ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(handler, &sc2210_ctrl_ops, V4L2_CID_VFLIP, 0, 1, 1, 0);

	if (handler->error) {
		ret = handler->error;
		dev_err(&sc2210->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	sc2210->subdev.ctrl_handler = handler;
	sc2210->has_init_exp = false;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int sc2210_check_sensor_id(struct sc2210 *sc2210,
				  struct i2c_client *client)
{
	struct device *dev = &sc2210->client->dev;
	u32 id = 0;
	int ret;

	ret = sc2210_read_reg(client, SC2210_REG_CHIP_ID,
		SC2210_REG_VALUE_16BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%04x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected SC%04x sensor\n", CHIP_ID);

	return 0;
}

static int sc2210_configure_regulators(struct sc2210 *sc2210)
{
	unsigned int i;

	for (i = 0; i < SC2210_NUM_SUPPLIES; i++)
		sc2210->supplies[i].supply = sc2210_supply_names[i];

	return devm_regulator_bulk_get(&sc2210->client->dev,
				       SC2210_NUM_SUPPLIES,
				       sc2210->supplies);
}

#ifdef HAS_RKLASER_SYSFS
static int sc2210_get_temperature(struct sc2210 *sc2210,
				  struct i2c_client *client)
{
	int ret;
	int v[2] = {0, 0};

	mutex_lock(&sc2210->mutex);
	ret = sc2210_read_reg(client, 0x4C10,
		SC2210_REG_VALUE_08BIT, &v[0]);
	ret = sc2210_read_reg(client, 0x4C11,
		SC2210_REG_VALUE_08BIT, &v[1]);
	mutex_unlock(&sc2210->mutex);
	return (v[0] << 3) | (v[1] & 0x7);
}

static ssize_t sysfs_sensor_temp_raw_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc2210 *sc2210 = to_sc2210(sd);
	return sprintf(buf, "%d\n", sc2210_get_temperature(sc2210, client));
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
#endif

static int sc2210_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct sc2210 *sc2210;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	sc2210 = devm_kzalloc(dev, sizeof(*sc2210), GFP_KERNEL);
	if (!sc2210)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &sc2210->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &sc2210->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &sc2210->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &sc2210->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	sc2210->cfg_num = ARRAY_SIZE(supported_modes);
	sc2210->cur_mode = &supported_modes[SC2210_DEF_MODE_ID];
	sc2210->client = client;

	sc2210->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(sc2210->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	sc2210->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(sc2210->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	sc2210->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(sc2210->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	sc2210->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(sc2210->pinctrl)) {
		sc2210->pins_default =
			pinctrl_lookup_state(sc2210->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(sc2210->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		sc2210->pins_sleep =
			pinctrl_lookup_state(sc2210->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(sc2210->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = sc2210_configure_regulators(sc2210);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&sc2210->mutex);

	sd = &sc2210->subdev;
	v4l2_i2c_subdev_init(sd, client, &sc2210_subdev_ops);
	ret = sc2210_initialize_controls(sc2210);
	if (ret)
		goto err_destroy_mutex;

	ret = __sc2210_power_on(sc2210);
	if (ret)
		goto err_free_handler;

	ret = sc2210_check_sensor_id(sc2210, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &sc2210_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	sc2210->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &sc2210->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(sc2210->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 sc2210->module_index, facing,
		 SC2210_NAME, dev_name(sd->dev));
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
	__sc2210_power_off(sc2210);
err_free_handler:
	v4l2_ctrl_handler_free(&sc2210->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&sc2210->mutex);

	return ret;
}

static int sc2210_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc2210 *sc2210 = to_sc2210(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&sc2210->ctrl_handler);
	mutex_destroy(&sc2210->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__sc2210_power_off(sc2210);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id sc2210_of_match[] = {
	{ .compatible = "smartsens,sc2210" },
	{ },
};
MODULE_DEVICE_TABLE(of, sc2210_of_match);
#endif

static const struct i2c_device_id sc2210_match_id[] = {
	{ "smartsens,sc2210", 0 },
	{ },
};

static struct i2c_driver sc2210_i2c_driver = {
	.driver = {
		.name = SC2210_NAME,
		.pm = &sc2210_pm_ops,
		.of_match_table = of_match_ptr(sc2210_of_match),
	},
	.probe		= &sc2210_probe,
	.remove		= &sc2210_remove,
	.id_table	= sc2210_match_id,
};

#ifdef CONFIG_ROCKCHIP_THUNDER_BOOT
module_i2c_driver(sc2210_i2c_driver);
#else
static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&sc2210_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&sc2210_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);
#endif

MODULE_DESCRIPTION("Smartsens sc2210 sensor driver");
MODULE_LICENSE("GPL v2");
