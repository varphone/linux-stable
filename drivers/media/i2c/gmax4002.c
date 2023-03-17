// SPDX-License-Identifier: GPL-2.0
/*
 * gmax4002 driver
 *
 * Copyright (C) 2020 Fuzhou Rockchip Electronics Co., Ltd.
 * Copyright (C) 2023 Fuzhou Full-V Smart Photoelectric Co., Ltd.
 *
 * V0.0X01.0X00 first version,adjust gmax4002.
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
#define V4L2_CID_SLEEP			(V4L2_CID_IMAGE_SOURCE_CLASS_BASE + 0x1006)
#define V4L2_CID_LOCK_GAIN		(V4L2_CID_IMAGE_SOURCE_CLASS_BASE + 0x1007)

#define MIPI_FREQ_300M			(u64)300000000 // 600.00Mbps/lane
#define MIPI_FREQ_600M			(u64)600000000 // 1200.00Mbps/lane
#define MIPI_FREQ_1200M			(u64)1200000000 // 2400.00Mbps/lane

#define BITS_PER_SAMPLE			10
#define GMAX4002_LANES			4
#define GMAX4002_MAX_PIXEL_RATE		(u64)(MIPI_FREQ_600M * 2 / BITS_PER_SAMPLE * GMAX4002_LANES)
#define OF_CAMERA_HDR_MODE		"rockchip,camera-hdr-mode"

#define GMAX4002_XVCLK_FREQ		39600000

#define GMAX4002_CHIP_ID		0x0FA2
#define GMAX4002_REG_OTP_ADDR		0x3401
#define GMAX4002_REG_OTP_RVAL		0x3402

#define GMAX4002_REG_CTRL_MODE		0x2E00
#define GMAX4002_BIT_SW_STANDBY		0x0
#define GMAX4002_BIT_CLK_STABLE_EN	BIT(0)
#define GMAX4002_BIT_STREAM_EN		BIT(1)

#define GMAX4002_REG_INT_EXP_EN		0x2e03

#define GMAX4002_EXPOSURE_MIN		0
#define GMAX4002_EXPOSURE_STEP		1 // Normal=1,HDR2=2,HDR3=3
#define GMAX4002_VTS_MAX		0x7fff

#define GMAX4002_REG_DIG_GAIN_C		0x3043
#define GMAX4002_REG_DIG_GAIN_F		0x3042
#define GMAX4002_REG_DIG_GAIN_SELN	0x303d
#define GMAX4002_REG_PGA_GAIN		0x2ec9

#define GMAX4002_BIT_DIG_GAIN_C_MASK	0x03
#define GMAX4002_BIT_DIG_GAIN_F_MASK	0x0f
#define GMAX4002_BIT_DIG_GAIN_SELN_MASK	0x0f
#define GMAX4002_BIT_PGA_GAIN_MASK	0x0f

#define GMAX4002_GAIN_MIN		0
#define GMAX4002_GAIN_MAX		1023
#define GMAX4002_GAIN_STEP		1
#define GMAX4002_GAIN_DEFAULT		0

#define GMAX4002_REG_PWR_UP_EN		0x3301
#define GMAX4002_REG_PWR_DOWN_EN	0x3302

#define GMAX4002_REG_TEST_IMG_EN	0x3001
#define GMAX4002_BIT_TEST_IMG_EN	BIT(0)

#define GMAX4002_REG_FLIP_H		0x3002
#define GMAX4002_REG_FLIP_V		0x2e05
#define GMAX4002_BIT_FLIP_H_EN		BIT(0)
#define GMAX4002_BIT_FLIP_V_NE		BIT(0)


#define GMAX4002_REG_HOLD		0x2E01

#define REG_DELAY			0xFFFE
#define REG_NULL			0xFFFF

#define GMAX4002_REG_VALUE_08BIT	1
#define GMAX4002_REG_VALUE_16BIT	2
#define GMAX4002_REG_VALUE_24BIT	3

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define GMAX4002_NAME			"fvig02"

#define GMAX4002_DEF_MODE_GROUP		0
#define GMAX4002_DEF_MODE_ID		1

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug Level (0-2)");

static int default_group = GMAX4002_DEF_MODE_GROUP;
module_param_named(group, default_group, int, 0644);
MODULE_PARM_DESC(group, "Default Mode Group (0-1)");

static int default_mode = GMAX4002_DEF_MODE_ID;
module_param_named(mode, default_mode, int, 0644);
MODULE_PARM_DESC(mode, "Default Mode Id (0-3)");

static const char * const gmax4002_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define GMAX4002_NUM_SUPPLIES ARRAY_SIZE(gmax4002_supply_names)

enum gmax4002_max_pad {
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

struct gmax4002_mode {
	u32 bus_fmt;
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list_c;
	const struct regval *reg_list_s;
	const struct regval *reg_list_f;
	u32 hdr_mode;
	u32 mipi_freq_idx;
	u32 bpp;
	u32 vc[PAD_MAX];
};

struct gmax4002_mode_group {
	int num;
	const struct gmax4002_mode *modes;
};

struct gmax4002 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct regulator_bulk_data supplies[GMAX4002_NUM_SUPPLIES];
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
	struct v4l2_ctrl	*sleep;
	struct v4l2_ctrl	*lock_gain;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct gmax4002_mode *cur_mode;
	u32			cfg_num;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	bool			has_init_exp;
	u32			cur_vts;
	struct preisp_hdrae_exp_s init_hdrae_exp;
};

#define to_gmax4002(sd) container_of(sd, struct gmax4002, subdev)

/*
 * Xclk 40Mhz linear 10bit mipi 1200Mbps/lane
 */
static const struct regval gmax4002_linear_10_mipi_common_regs[] = {
	{0x2E00, 0x00},
	{0x2E01, 0x00},
	{0x2E02, 0x00},
	{0x2E03, 0x06},
	{0x2E04, 0x00},
	{0x2E05, 0x18},
	{0x2E06, 0x02},
	{0x2E07, 0x00},
	{0x2E08, 0x00},
	{0x2E09, 0x00},
	{0x2E0A, 0x00},
	{0x2E0B, 0x01},
	{0x2E0C, 0x00},
	{0x2E0D, 0xBE},
	{0x2E0E, 0x04},
	{0x2E0F, 0x01},
	{0x2E10, 0x00},
	{0x2E11, 0x12},
	{0x2E12, 0x00},
	{0x2E13, 0x00},
	{0x2E14, 0xBA},
	{0x2E15, 0x04},
	{0x2E16, 0x00},
	{0x2E17, 0x00},
	{0x2E18, 0x08},
	{0x2E19, 0x00},
	{0x2E1A, 0xB0},
	{0x2E1B, 0x04},
	{0x2E1C, 0x00},
	{0x2E1D, 0x00},
	{0x2E1E, 0x00},
	{0x2E1F, 0x00},
	{0x2E20, 0x00},
	{0x2E21, 0x00},
	{0x2E22, 0x00},
	{0x2E23, 0x00},
	{0x2E24, 0x00},
	{0x2E25, 0x00},
	{0x2E26, 0x00},
	{0x2E27, 0x00},
	{0x2E28, 0x00},
	{0x2E29, 0x00},
	{0x2E2A, 0x00},
	{0x2E2B, 0x00},
	{0x2E2C, 0x00},
	{0x2E2D, 0x00},
	{0x2E2E, 0x00},
	{0x2E2F, 0x00},
	{0x2E30, 0x00},
	{0x2E31, 0x00},
	{0x2E32, 0x00},
	{0x2E33, 0x00},
	{0x2E34, 0x00},
	{0x2E35, 0x00},
	{0x2E36, 0x00},
	{0x2E37, 0x00},
	{0x2E38, 0x00},
	{0x2E39, 0x00},
	{0x2E3A, 0x00},
	{0x2E3B, 0x00},
	{0x2E3C, 0x00},
	{0x2E3D, 0x00},
	{0x2E3E, 0x00},
	{0x2E3F, 0x00},
	{0x2E40, 0x00},
	{0x2E41, 0x00},
	{0x2E42, 0x00},
	{0x2E43, 0x00},
	{0x2E44, 0x00},
	{0x2E45, 0x00},
	{0x2E46, 0x00},
	{0x2E47, 0x00},
	{0x2E48, 0x00},
	{0x2E49, 0x00},
	{0x2E4A, 0x00},
	{0x2E4B, 0x00},
	{0x2E4C, 0x00},
	{0x2E4D, 0x00},
	{0x2E4E, 0x00},
	{0x2E4F, 0x00},
	{0x2E50, 0x00},
	{0x2E51, 0x00},
	{0x2E52, 0x00},
	{0x2E53, 0x00},
	{0x2E54, 0x00},
	{0x2E55, 0x00},
	{0x2E56, 0x00},
	{0x2E57, 0x00},
	{0x2E58, 0x01},
	{0x2E59, 0x14},
	{0x2E5A, 0x00},
	{0x2E5B, 0x00},
	{0x2E5C, 0x04},
	{0x2E5D, 0x22},
	{0x2E5E, 0x01},
	{0x2E5F, 0x02},
	{0x2E60, 0x02},
	{0x2E61, 0x02},
	{0x2E62, 0x28},
	{0x2E63, 0x28},
	{0x2E64, 0x01},
	{0x2E65, 0x00},
	{0x2E66, 0x02},
	{0x2E67, 0x00},
	{0x2E68, 0x24},
	{0x2E69, 0x05},
	{0x2E6A, 0x62},
	{0x2E6B, 0x03},
	{0x2E6C, 0x46},
	{0x2E6D, 0x78},
	{0x2E6E, 0x02},
	{0x2E6F, 0x2A},
	{0x2E70, 0xFF},
	{0x2E71, 0xFF},
	{0x2E72, 0xFF},
	{0x2E73, 0xFF},
	{0x2E74, 0x32},
	{0x2E75, 0x64},
	{0x2E76, 0x14},
	{0x2E77, 0xFF},
	{0x2E78, 0xFF},
	{0x2E79, 0xFF},
	{0x2E7A, 0x01},
	{0x2E7B, 0x87},
	{0x2E7C, 0xFF},
	{0x2E7D, 0xFF},
	{0x2E7E, 0x01},
	{0x2E7F, 0x88},
	{0x2E80, 0x05},
	{0x2E81, 0x24},
	{0x2E82, 0xFF},
	{0x2E83, 0xFF},
	{0x2E84, 0x05},
	{0x2E85, 0x31},
	{0x2E86, 0x5C},
	{0x2E87, 0x84},
	{0x2E88, 0x03},
	{0x2E89, 0x0C},
	{0x2E8A, 0x13},
	{0x2E8B, 0x34},
	{0x2E8C, 0xFF},
	{0x2E8D, 0xFF},
	{0x2E8E, 0xFF},
	{0x2E8F, 0xFF},
	{0x2E90, 0xFF},
	{0x2E91, 0xFF},
	{0x2E92, 0xFF},
	{0x2E93, 0xFF},
	{0x2E94, 0x04},
	{0x2E95, 0x0C},
	{0x2E96, 0x14},
	{0x2E97, 0x34},
	{0x2E98, 0x01},
	{0x2E99, 0x02},
	{0x2E9A, 0x11},
	{0x2E9B, 0x12},
	{0x2E9C, 0x04},
	{0x2E9D, 0x0C},
	{0x2E9E, 0x14},
	{0x2E9F, 0x34},
	{0x2EA0, 0x0B},
	{0x2EA1, 0x0C},
	{0x2EA2, 0x33},
	{0x2EA3, 0x34},
	{0x2EA4, 0x0C},
	{0x2EA5, 0x14},
	{0x2EA6, 0x10},
	{0x2EA7, 0xFF},
	{0x2EA8, 0x01},
	{0x2EA9, 0x02},
	{0x2EAA, 0x11},
	{0x2EAB, 0x12},
	{0x2EAC, 0x01},
	{0x2EAD, 0x02},
	{0x2EAE, 0xFF},
	{0x2EAF, 0xFF},
	{0x2EB0, 0xFF},
	{0x2EB1, 0xFF},
	{0x2EB2, 0xFF},
	{0x2EB3, 0xFF},
	{0x2EB4, 0xFF},
	{0x2EB5, 0xFF},
	{0x2EB6, 0x2E},
	{0x2EB7, 0x1C},
	{0x2EB8, 0x1E},
	{0x2EB9, 0x0C},
	{0x2EBA, 0x03},
	{0x2EBB, 0x00},
	{0x2EBC, 0x01},
	{0x2EBD, 0x00},
	{0x2EBE, 0x01},
	{0x2EBF, 0x03},
	{0x2EC0, 0x01},
	{0x2EC1, 0x01},
	{0x2EC2, 0x00},
	{0x2EC3, 0x01},
	{0x2EC4, 0x1E},
	{0x2EC5, 0x0C},
	{0x2EC6, 0x00},
	{0x2EC7, 0x00},
	{0x2EC8, 0x01},
	{0x2EC9, 0x01},
	{0x2ECA, 0x03},
	{0x2ECB, 0x01},
	{0x3000, 0x01},
	{0x3001, 0x02},
	{0x3002, 0x00},
	{0x3003, 0x00},
	{0x3004, 0x03},
	{0x3005, 0x00},
	{0x3006, 0x08},
	{0x3007, 0x10},
	{0x3008, 0x00},
	{0x3009, 0x04},
	{0x300A, 0x01},
	{0x300B, 0x00},
	{0x300C, 0x01},
	{0x300D, 0x0F},
	{0x300E, 0x00},
	{0x300F, 0x01},
	{0x3010, 0x01},
	{0x3011, 0x01},
	{0x3012, 0x00},
	{0x3013, 0x00},
	{0x3014, 0x8E},
	{0x3015, 0x09},
	{0x3016, 0x04},
	{0x3017, 0x00},
	{0x3018, 0x08},
	{0x3019, 0x07},
	{0x301A, 0x10},
	{0x301B, 0x07},
	{0x301C, 0x27},
	{0x301D, 0x00},
	{0x301E, 0x0B},
	{0x301F, 0x09},
	{0x3020, 0x05},
	{0x3021, 0x06},
	{0x3022, 0x96},
	{0x3023, 0xFB},
	{0x3024, 0x34},
	{0x3025, 0x00},
	{0x3026, 0x00},
	{0x3027, 0x00},
	{0x3028, 0x00},
	{0x3029, 0x00},
	{0x302A, 0x00},
	{0x302B, 0x00},
	{0x302C, 0x00},
	{0x302D, 0x00},
	{0x302E, 0x00},
	{0x302F, 0x00},
	{0x3030, 0x00},
	{0x3031, 0x00},
	{0x3032, 0x0D},
	{0x3033, 0x2B},
	{0x3034, 0x32},
	{0x3035, 0x0A},
	{0x3036, 0x09},
	{0x3037, 0x00},
	{0x3038, 0x08},
	{0x3039, 0x00},
	{0x303A, 0x00},
	{0x303B, 0x00},
	{0x303C, 0x00},
	{0x303D, 0x01},
	{0x303E, 0x00},
	{0x303F, 0x00},
	{0x3040, 0x10},
	{0x3041, 0x00},
	{0x3042, 0x10},
	{0x3043, 0x00},
	{0x3044, 0x19},
	{0x3045, 0x10},
	{0x3046, 0x00},
	{0x3047, 0x00},
	{0x3048, 0x00},
	{0x3049, 0x00},
	{0x304A, 0x00},
	{0x304B, 0x00},
	{0x304C, 0x00},
	{0x304D, 0x00},
	{0x304E, 0x00},
	{0x304F, 0x02},
	{0x3050, 0x0A},
	{0x3051, 0x02},
	{0x3052, 0x00},
	{0x3053, 0x10},
	{0x3054, 0x00},
	{0x3055, 0x00},
	{0x3056, 0x5E},
	{0x3057, 0x01},
	{0x3058, 0x00},
	{0x3059, 0x01},
	{0x305A, 0x00},
	{0x305B, 0x10},
	{0x305C, 0x00},
	{0x305D, 0x04},
	{0x305E, 0x00},
	{0x305F, 0x00},
	{0x3060, 0x00},
	{0x3200, 0x20},
	{0x3201, 0x00},
	{0x3202, 0x04},
	{0x3203, 0x03},
	{0x3204, 0x20},
	{0x3205, 0x03},
	{0x3206, 0x03},
	{0x3207, 0x03},
	{0x3208, 0x16},
	{0x3209, 0x04},
	{0x320A, 0x00},
	{0x320B, 0x16},
	{0x320C, 0x04},
	{0x320D, 0x1A},
	{0x320E, 0x0F},
	{0x320F, 0x00},
	{0x3210, 0x11},
	{0x3211, 0x07},
	{0x3212, 0x00},
	{0x3213, 0x0E},
	{0x3214, 0x1B},
	{0x3215, 0x03},
	{0x3216, 0x28},
	{0x3217, 0x04},
	{0x3218, 0x00},
	{0x3219, 0x00},
	{0x321A, 0x28},
	{0x321B, 0x00},
	{0x321C, 0x00},
	{0x321D, 0x04},
	{0x321E, 0x28},
	{0x321F, 0x04},
	{0x3220, 0x00},
	{0x3221, 0x00},
	{0x3222, 0x18},
	{0x3223, 0x03},
	{0x3224, 0x00},
	{0x3225, 0x00},
	{0x3226, 0x28},
	{0x3227, 0x03},
	{0x3228, 0x00},
	{0x3229, 0x00},
	{0x322A, 0x11},
	{0x322B, 0x03},
	{0x322C, 0x1B},
	{0x322D, 0x00},
	{0x322E, 0x07},
	{0x322F, 0x0F},
	{0x3230, 0x0E},
	{0x3231, 0x41},
	{0x3232, 0x53},
	{0x3233, 0x4E},
	{0x3234, 0x47},
	{0x3235, 0x47},
	{0x3236, 0x47},
	{0x3237, 0x47},
	{0x3238, 0x50},
	{0x3239, 0x47},
	{0x323A, 0x53},
	{0x323B, 0x53},
	{0x323C, 0x4A},
	{0x323D, 0x4A},
	{0x323E, 0x68},
	{0x323F, 0x0A},
	{0x3240, 0x00},
	{0x3241, 0x1A},
	{0x3242, 0x20},
	{0x3243, 0x04},
	{0x3244, 0x13},
	{0x3245, 0x13},
	{0x3246, 0x30},
	{0x3247, 0x30},
	{0x3248, 0x7D},
	{0x3249, 0x1C},
	{0x324A, 0x1E},
	{0x324B, 0x00},
	{0x324C, 0x00},
	{0x324D, 0x00},
	{0x324E, 0x00},
	{0x3300, 0x00},
	{0x3303, 0x00},
	{0x3304, 0x00},
	{0x3305, 0x00},
	{0x3306, 0x00},
	{0x3307, 0x01},
	{0x3308, 0x00},
	{0x3309, 0x01},
	{0x330A, 0x00},
	{0x330B, 0x01},
	{0x330C, 0x00},
	{0x330D, 0x01},
	{0x330E, 0x00},
	{0x330F, 0x01},
	{0x3310, 0x00},
	{0x3311, 0xD0},
	{0x3312, 0x07},
	{0x3313, 0xB8},
	{0x3314, 0x0B},
	{0x3315, 0xF4},
	{0x3316, 0x01},
	{0x3317, 0xE8},
	{0x3318, 0x03},
	{0x3319, 0xE8},
	{0x331A, 0x03},
	{0x331B, 0xE8},
	{0x331C, 0x03},
	{0x331D, 0xA0},
	{0x331E, 0x0F},
	{0x331F, 0xD0},
	{0x3320, 0x07},
	{0x3321, 0x01},
	{0x3322, 0x00},
	{0x3323, 0x01},
	{0x3324, 0x00},
	{0x3325, 0x01},
	{0x3326, 0x00},
	{0x3327, 0x01},
	{0x3328, 0x00},
	{0x3329, 0x01},
	{0x332A, 0x00},
	{0x332B, 0x01},
	{0x332C, 0x00},
	{0x332D, 0xA0},
	{0x332E, 0x0F},
	{0x332F, 0xE8},
	{0x3330, 0x03},
	{0x3331, 0xD0},
	{0x3332, 0x07},
	{0x3333, 0x03},
	{0x3334, 0x00},
	{0x3335, 0xA0},
	{0x3336, 0x0F},
	{0x3337, 0xD0},
	{0x3338, 0x07},
	{0x3339, 0xF4},
	{0x333A, 0x01},
	{0x333B, 0x3C},
	{0x333C, 0x00},
	{0x333D, 0xB8},
	{0x333E, 0x0B},
	{0x333F, 0xE8},
	{0x3340, 0x03},
	{0x3341, 0xE8},
	{0x3342, 0x03},
	{0x3343, 0xE8},
	{0x3344, 0x03},
	{0x3345, 0x55},
	{0x3346, 0x55},
	{0x3347, 0x55},
	{0x3348, 0x55},
	{0x3349, 0x55},
	{0x334A, 0x55},
	{0x334B, 0x55},
	{0x334C, 0x55},
	{0x334D, 0x10},
	{0x334E, 0x32},
	{0x334F, 0x54},
	{0x3350, 0x9A},
	{0x3351, 0xCD},
	{0x3352, 0x7B},
	{0x3353, 0xE8},
	{0x3354, 0x6F},
	{0x3355, 0x10},
	{0x3356, 0x42},
	{0x3357, 0x95},
	{0x3358, 0xEA},
	{0x3359, 0xCD},
	{0x335A, 0x3B},
	{0x335B, 0x87},
	{0x335C, 0x6E},
	{0x335D, 0x00},
	{0x335E, 0x00},
	{0x335F, 0x00},
	{0x3360, 0x00},
	{0x3361, 0x01},
	{0x3362, 0x00},
	{REG_NULL, 0x00},
};

/*
 * Xclk 40Mhz linear 10bit mipi 1200Mbps/lane
 */
static const struct regval gmax4002_linear_10_mipi_final_regs[] = {
	{0x2E00, 0x01},
	{0x3301, 0x01},
	{0x3302, 0x00},
	{0x2E00, 0x03},
	{REG_NULL, 0x00},
};

/*
 * Xclk 40Mhz linear 10bit 2048x1200 166fps 1200Mbps/lane
 */
static const struct regval gmax4002_linear_10_2048x1200_166fps_regs[] = {
	{REG_NULL, 0x00},
};

/*
 * Xclk 40Mhz linear 10bit 1920x1080 184fps 1200Mbps/lane
 */
static const struct regval gmax4002_linear_10_1920x1080_180fps_regs[] = {
	// 1920x1080@184 FPS
	{0x2E1A, 0x38}, // WIN1_L
	{0x2E1B, 0x04},
	{0x2E18, 0x44}, // WIN1_S
	{0x2E19, 0x00},
	{0x3005, 0x80}, // WIN_X_L
	{0x3006, 0x07},
	{0x3007, 0x50}, // WIN_X_S
	{0x3008, 0x00},
	{REG_NULL, 0x00},
};

/*
 * Xclk 40Mhz linear 10bit 1792x800 240fps 1200Mbps/lane
 */
static const struct regval gmax4002_linear_10_1792x800_240fps_regs[] = {
	// 1792x800@240 FPS
	{0x2E1A, 0x20}, // WIN1_L
	{0x2E1B, 0x03},
	{0x2E18, 0xD0}, // WIN1_S
	{0x2E19, 0x00},
	{0x3005, 0x00}, // WIN_X_L
	{0x3006, 0x07},
	{0x3007, 0x90}, // WIN_X_S
	{0x3008, 0x00},
	{REG_NULL, 0x00},
};

/*
 * Xclk 40Mhz linear 10bit 1792x768 248fps 1200Mbps/lane
 */
static const struct regval gmax4002_linear_10_1792x768_248fps_regs[] = {
	// 1792x768@248 FPS
	{0x2E1A, 0x00}, // WIN1_L
	{0x2E1B, 0x03},
	{0x2E18, 0xE0}, // WIN1_S
	{0x2E19, 0x00},
	{0x3005, 0x00}, // WIN_X_L
	{0x3006, 0x07},
	{0x3007, 0x90}, // WIN_X_S
	{0x3008, 0x00},
	{REG_NULL, 0x00},
};

/*
 * Xclk 40Mhz linear 10bit 1792x576 320fps 1200Mbps/lane
 */
static const struct regval gmax4002_linear_10_1792x576_320fps_regs[] = {
	// 1792x576@320 FPS
	{0x2E1A, 0x40}, // WIN1_L
	{0x2E1B, 0x02},
	{0x2E18, 0x40}, // WIN1_S
	{0x2E19, 0x01},
	{0x3005, 0x00}, // WIN_X_L
	{0x3006, 0x07},
	{0x3007, 0x90}, // WIN_X_S
	{0x3008, 0x00},
	{REG_NULL, 0x00},
};

/*
 * Xclk 40Mhz linear 10bit 1792x544 340fps 1200Mbps/lane
 */
static const struct regval gmax4002_linear_10_1792x544_340fps_regs[] = {
	// 1792x544@340 FPS
	{0x2E1A, 0x20}, // WIN1_L
	{0x2E1B, 0x02},
	{0x2E18, 0x50}, // WIN1_S
	{0x2E19, 0x01},
	{0x3005, 0x00}, // WIN_X_L
	{0x3006, 0x07},
	{0x3007, 0x90}, // WIN_X_S
	{0x3008, 0x00},
	{REG_NULL, 0x00},
};

/*
 * Xclk 40Mhz linear 10bit 640x480 394fps 1200Mbps/lane
 */
static const struct regval gmax4002_linear_10_640x480_394fps_regs[] = {
	// 640x480@394 FPS
	{0x2E1A, 0xE0}, // WIN1_L
	{0x2E1B, 0x01},
	{0x2E18, 0x70}, // WIN1_S
	{0x2E19, 0x01},
	{0x3005, 0x80}, // WIN_X_L
	{0x3006, 0x02},
	{0x3007, 0xD0}, // WIN_X_S
	{0x3008, 0x02},
	{REG_NULL, 0x00},
};

/*
 * Xclk 40Mhz linear 10bit 256x1024 190fps 1200Mbps/lane
 */
static const struct regval gmax4002_linear_10_256x1024_190fps_regs[] = {
	// 256x1024@190 FPS
	{0x2E1A, 0x00}, // WIN1_L
	{0x2E1B, 0x04},
	{0x2E18, 0x60}, // WIN1_S
	{0x2E19, 0x00},
	{0x3005, 0x00}, // WIN_X_L
	{0x3006, 0x01},
	{0x3007, 0x90}, // WIN_X_S
	{0x3008, 0x03},
	{REG_NULL, 0x00},
};

static const struct gmax4002_mode modes_10bit_bg10_mipi[] = {
	{
		/* linear modes */
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.width = 2048,
		.height = 1200 + 18,
		.max_fps = {
			.numerator = 10000,
			.denominator = 1660000,
		},
		.exp_def = 0x200 / 2,
		.hts_def = 0x410 * 2, // REG{0x2e5d,0x2e5e}
		.vts_def = 0x261 * 2, // REG{0x2e5d,0x2e5e}
		.reg_list_c = gmax4002_linear_10_mipi_common_regs,
		.reg_list_s = gmax4002_linear_10_2048x1200_166fps_regs,
		.reg_list_f = gmax4002_linear_10_mipi_final_regs,
		.hdr_mode = NO_HDR,
		.mipi_freq_idx = 1, // 1200.00 Mbps
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
	{
		/* linear modes */
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.width = 1920,
		.height = 1080 + 18,
		.max_fps = {
			.numerator = 10000,
			.denominator = 1800000,
		},
		.exp_def = 0x200 / 2,
		.hts_def = 0x410 * 2, // REG{0x2e5d,0x2e5e}
		.vts_def = 0x225 * 2, // REG{0x2e5d,0x2e5e}
		.reg_list_c = gmax4002_linear_10_mipi_common_regs,
		.reg_list_s = gmax4002_linear_10_1920x1080_180fps_regs,
		.reg_list_f = gmax4002_linear_10_mipi_final_regs,
		.hdr_mode = NO_HDR,
		.mipi_freq_idx = 1, // 1200.00 Mbps
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
	{
		/* linear modes */
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.width = 1792,
		.height = 800 + 18,
		.max_fps = {
			.numerator = 10000,
			.denominator = 2400000,
		},
		.exp_def = 0x200 / 2,
		.hts_def = 0x410 * 2, // REG{0x2e5d,0x2e5e}
		.vts_def = 0x199 * 2, // REG{0x2e5d,0x2e5e}
		.reg_list_c = gmax4002_linear_10_mipi_common_regs,
		.reg_list_s = gmax4002_linear_10_1792x800_240fps_regs,
		.reg_list_f = gmax4002_linear_10_mipi_final_regs,
		.hdr_mode = NO_HDR,
		.mipi_freq_idx = 1, // 1200.00 Mbps
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
	{
		/* linear modes */
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.width = 1792,
		.height = 768 + 18,
		.max_fps = {
			.numerator = 10000,
			.denominator = 2480000,
		},
		.exp_def = 0x200 / 2,
		.hts_def = 0x410 * 2, // REG{0x2e5d,0x2e5e}
		.vts_def = 0x189 * 2, // REG{0x2e5d,0x2e5e}
		.reg_list_c = gmax4002_linear_10_mipi_common_regs,
		.reg_list_s = gmax4002_linear_10_1792x768_248fps_regs,
		.reg_list_f = gmax4002_linear_10_mipi_final_regs,
		.hdr_mode = NO_HDR,
		.mipi_freq_idx = 1, // 1200.00 Mbps
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
	{
		/* linear modes */
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.width = 1792,
		.height = 576 + 18, /* */
		.max_fps = {
			.numerator = 10000,
			.denominator = 3200000,
		},
		.exp_def = 0x200 / 2,
		.hts_def = 0x410 * 2, // REG{0x2e5d,0x2e5e}
		.vts_def = 0x110 * 2, // REG{0x2e5d,0x2e5e}
		.reg_list_c = gmax4002_linear_10_mipi_common_regs,
		.reg_list_s = gmax4002_linear_10_1792x576_320fps_regs,
		.reg_list_f = gmax4002_linear_10_mipi_final_regs,
		.hdr_mode = NO_HDR,
		.mipi_freq_idx = 1, // 1200.00 Mbps
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
	{
		/* linear modes */
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.width = 1792,
		.height = 544 + 18, /* */
		.max_fps = {
			.numerator = 10000,
			.denominator = 3400000,
		},
		.exp_def = 0x200 / 2,
		.hts_def = 0x410 * 2, // REG{0x2e5d,0x2e5e}
		.vts_def = 0x110 * 2, // REG{0x2e5d,0x2e5e}
		.reg_list_c = gmax4002_linear_10_mipi_common_regs,
		.reg_list_s = gmax4002_linear_10_1792x544_340fps_regs,
		.reg_list_f = gmax4002_linear_10_mipi_final_regs,
		.hdr_mode = NO_HDR,
		.mipi_freq_idx = 1, // 1200.00 Mbps
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
	{
		/* linear modes */
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.width = 640,
		.height = 480 + 18,
		.max_fps = {
			.numerator = 10000,
			.denominator = 3940000,
		},
		.exp_def = 0x200 / 2,
		.hts_def = 0x410 * 2, // REG{0x2e5d,0x2e5e}
		.vts_def = 0x0F9 * 2, // REG{0x2e5d,0x2e5e}
		.reg_list_c = gmax4002_linear_10_mipi_common_regs,
		.reg_list_s = gmax4002_linear_10_640x480_394fps_regs,
		.reg_list_f = gmax4002_linear_10_mipi_final_regs,
		.hdr_mode = NO_HDR,
		.mipi_freq_idx = 1, // 1200.00 Mbps
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
	{
		/* linear modes */
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.width = 256,
		.height = 1024 + 18,
		.max_fps = {
			.numerator = 10000,
			.denominator = 1900000,
		},
		.exp_def = 0x200 / 2,
		.hts_def = 0x410 * 2, // REG{0x2e5d,0x2e5e}
		.vts_def = 0x209 * 2, // REG{0x2e5d,0x2e5e}
		.reg_list_c = gmax4002_linear_10_mipi_common_regs,
		.reg_list_s = gmax4002_linear_10_256x1024_190fps_regs,
		.reg_list_f = gmax4002_linear_10_mipi_final_regs,
		.hdr_mode = NO_HDR,
		.mipi_freq_idx = 1, // 1200.00 Mbps
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
};

static const struct gmax4002_mode_group supported_mode_groups[] = {
	{ ARRAY_SIZE(modes_10bit_bg10_mipi), modes_10bit_bg10_mipi },
};

static const s64 link_freq_items[] = {
	MIPI_FREQ_300M,
	MIPI_FREQ_600M,
	MIPI_FREQ_1200M,
};

static const char * const gmax4002_test_pattern_menu[] = {
	"Disabled",
	"Gray gradient"
};

/* Write registers up to 4 at a time */
static int gmax4002_write_reg(struct i2c_client *client, u16 reg, u32 len,
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
				 "gmax4002_write_reg(0x%04hx, %u, %u) = %d\n",
				 reg, len, val, ret);
			return -EIO;
		}
	}

	return 0;
}

static int gmax4002_write_array(struct i2c_client *client,
			        const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
		ret |= gmax4002_write_reg(client, regs[i].addr,
					  GMAX4002_REG_VALUE_08BIT, regs[i].val);
		ret = ret == 0x0103 ? 0 : ret;
		if (ret != 0) {
			dev_info(&client->dev,
				 "gmax4002_write_reg(0x%04hx, 0x%02hhx) @ %d\n",
				 regs[i].addr, regs[i].val, i);
		}
	}
	return ret;
}

/* Read registers up to 4 at a time */
static int gmax4002_read_reg(struct i2c_client *client, u16 reg,
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

static int gmax4002_get_reso_dist(const struct gmax4002_mode *mode,
				  struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct gmax4002_mode *
gmax4002_find_best_fit(struct gmax4002 *gmax4002, struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	const struct gmax4002_mode_group *mg =
		&supported_mode_groups[default_group];
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < gmax4002->cfg_num; i++) {
		dist = gmax4002_get_reso_dist(&mg->modes[i], framefmt);
		if ((cur_best_fit_dist == -1 || dist <= cur_best_fit_dist) &&
		    (mg->modes[i].bus_fmt == framefmt->code)) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &mg->modes[cur_best_fit];
}

static void gmax4002_change_mode(struct gmax4002 *gmax4002,
				 const struct gmax4002_mode *mode)
{
	gmax4002->cur_mode = mode;
	gmax4002->cur_vts = gmax4002->cur_mode->vts_def;
	dev_info(&gmax4002->client->dev,
		 "set fmt: cur_mode: %dx%d@%d/%d, hdr: %d\n", mode->width,
		 mode->height, mode->max_fps.numerator,
		 mode->max_fps.denominator, mode->hdr_mode);
}

static int gmax4002_set_gain(struct gmax4002 *gmax4002, int val)
{
	struct i2c_client *client = gmax4002->client;
	int cur_gain_c = 0;
	int cur_gain_f = 0;
	int cur_gain_seln = 0;
	int cur_pga_gain = 0;
	int gain_c = 0;
	int gain_f = 0;
	int gain_seln = 1;
	int pga_gain = 0;
	int ret = 0;

	if (val < 0 || val > GMAX4002_GAIN_MAX)
		return -EINVAL;

	// Fetch current values
	ret |= gmax4002_read_reg(client, GMAX4002_REG_DIG_GAIN_C,
				 GMAX4002_REG_VALUE_08BIT, &cur_gain_c);
	ret |= gmax4002_read_reg(client, GMAX4002_REG_DIG_GAIN_F,
				 GMAX4002_REG_VALUE_08BIT, &cur_gain_f);
	ret |= gmax4002_read_reg(client, GMAX4002_REG_DIG_GAIN_SELN,
				 GMAX4002_REG_VALUE_08BIT, &cur_gain_seln);
	ret |= gmax4002_read_reg(client, GMAX4002_REG_PGA_GAIN,
				 GMAX4002_REG_VALUE_08BIT, &cur_pga_gain);

	if (val < 128) { /* 0.0 ~ 7.9 */
		gain_c = 0;
		gain_f = 0;
		gain_seln = 1;
		if (val > 63)
			pga_gain = 15;
		else
			pga_gain = val / 16;
	} else if (val < 256) { /* 8.0 ~ 15.9 */
		gain_c = 1;
		gain_f = ((val - 128) / 8) & GMAX4002_BIT_DIG_GAIN_F_MASK;
		gain_seln = 0;
		pga_gain = 15;
	} else if (val < 512) { /* 16.0 ~ 31.9 */
		gain_c = 2;
		gain_f = ((val - 256) / 16) & GMAX4002_BIT_DIG_GAIN_F_MASK;
		gain_seln = 0;
		pga_gain = 15;
	} else if (val < 1024) { /* 32.0 ~ 63.9 */
		gain_c = 2;
		gain_f = ((val - 512) / 32) & GMAX4002_BIT_DIG_GAIN_F_MASK;
		gain_seln = 0;
		pga_gain = 15;
	}

	gain_c |= (cur_gain_c & ~GMAX4002_BIT_DIG_GAIN_C_MASK);
	gain_f |= (cur_gain_f & ~GMAX4002_BIT_DIG_GAIN_F_MASK);
	gain_seln |= (cur_gain_seln & ~GMAX4002_BIT_DIG_GAIN_SELN_MASK);
	pga_gain |= (cur_pga_gain & ~GMAX4002_BIT_PGA_GAIN_MASK);

	ret |= gmax4002_write_reg(client, GMAX4002_REG_HOLD,
				  GMAX4002_REG_VALUE_08BIT, 1);
	ret |= gmax4002_write_reg(client, GMAX4002_REG_DIG_GAIN_C,
				  GMAX4002_REG_VALUE_08BIT, gain_c);
	ret |= gmax4002_write_reg(client, GMAX4002_REG_DIG_GAIN_F,
				  GMAX4002_REG_VALUE_08BIT, gain_f);
	ret |= gmax4002_write_reg(client, GMAX4002_REG_DIG_GAIN_SELN,
				  GMAX4002_REG_VALUE_08BIT, gain_seln);
	ret |= gmax4002_write_reg(client, GMAX4002_REG_PGA_GAIN,
				  GMAX4002_REG_VALUE_08BIT, pga_gain);
	ret |= gmax4002_write_reg(client, GMAX4002_REG_HOLD,
				  GMAX4002_REG_VALUE_08BIT, 0);

	dev_dbg(&client->dev, "gain_c=%d,gain_f=%d,gain_seln=%d,pga_gain=%d\n",
		gain_c, gain_f, gain_seln, pga_gain);

	return ret;
}

static int gmax4002_set_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_pad_config *cfg,
			    struct v4l2_subdev_format *fmt)
{
	struct gmax4002 *gmax4002 = to_gmax4002(sd);
	const struct gmax4002_mode *mode;
	s64 h_blank, vblank_def;
	u64 pixel_rate = 0;

	mutex_lock(&gmax4002->mutex);

	mode = gmax4002_find_best_fit(gmax4002, fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&gmax4002->mutex);
		return -ENOTTY;
#endif
	} else {
		gmax4002_change_mode(gmax4002, mode);
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(gmax4002->hblank, h_blank, h_blank, 1,
					 h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(gmax4002->vblank, vblank_def,
					 GMAX4002_VTS_MAX - mode->height, 1,
					 vblank_def);
		__v4l2_ctrl_s_ctrl(gmax4002->link_freq, mode->mipi_freq_idx);
		pixel_rate = (u32)link_freq_items[mode->mipi_freq_idx] /
			     mode->bpp * 2 * GMAX4002_LANES;
		__v4l2_ctrl_s_ctrl_int64(gmax4002->pixel_rate, pixel_rate);
	}

	mutex_unlock(&gmax4002->mutex);

	return 0;
}

static int gmax4002_get_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_pad_config *cfg,
			    struct v4l2_subdev_format *fmt)
{
	struct gmax4002 *gmax4002 = to_gmax4002(sd);
	const struct gmax4002_mode *mode = gmax4002->cur_mode;

	mutex_lock(&gmax4002->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&gmax4002->mutex);
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
	mutex_unlock(&gmax4002->mutex);

	return 0;
}

static int gmax4002_enum_mbus_code(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_mbus_code_enum *code)
{
	struct gmax4002 *gmax4002 = to_gmax4002(sd);

	if (code->index != 0)
		return -EINVAL;
	code->code = gmax4002->cur_mode->bus_fmt;

	return 0;
}

static int gmax4002_enum_frame_sizes(struct v4l2_subdev *sd,
				     struct v4l2_subdev_pad_config *cfg,
				     struct v4l2_subdev_frame_size_enum *fse)
{
	struct gmax4002 *gmax4002 = to_gmax4002(sd);
	const struct gmax4002_mode_group *mg =
		&supported_mode_groups[default_group];

	v4l2_dbg(1, debug, sd, "fse->index=%d\n", fse->index);

	if (fse->index >= gmax4002->cfg_num)
		return -EINVAL;

	if (fse->code != mg->modes[fse->index].bus_fmt)
		return -EINVAL;

	fse->min_width = mg->modes[fse->index].width;
	fse->max_width = mg->modes[fse->index].width;
	fse->max_height = mg->modes[fse->index].height;
	fse->min_height = mg->modes[fse->index].height;

	return 0;
}

static int gmax4002_enable_test_pattern(struct gmax4002 *gmax4002, u32 pattern)
{
	u32 val = 0;
	int ret = 0;

	ret = gmax4002_read_reg(gmax4002->client, GMAX4002_REG_TEST_IMG_EN,
				GMAX4002_REG_VALUE_08BIT, &val);
	if (pattern)
		val |= GMAX4002_REG_TEST_IMG_EN;
	else
		val &= ~GMAX4002_REG_TEST_IMG_EN;
	ret |= gmax4002_write_reg(gmax4002->client, GMAX4002_REG_TEST_IMG_EN,
				  GMAX4002_REG_VALUE_08BIT, val);
	return ret;
}

static int gmax4002_g_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct gmax4002 *gmax4002 = to_gmax4002(sd);
	const struct gmax4002_mode *mode = gmax4002->cur_mode;

	mutex_lock(&gmax4002->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&gmax4002->mutex);

	return 0;
}

static int gmax4002_get_selection(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_selection *sel)
{
	struct gmax4002 *gmax4002 = to_gmax4002(sd);

	if (sel->target == V4L2_SEL_TGT_CROP_BOUNDS) {
		sel->r.left = 0;
		sel->r.top = 18;
		sel->r.width = gmax4002->cur_mode->width;
		sel->r.height = gmax4002->cur_mode->height - 18;
		return 0;
	}

	return -EINVAL;
}

static int gmax4002_get_mbus_config(struct v4l2_subdev *sd,
				    struct v4l2_mbus_config *config)
{
	struct gmax4002 *gmax4002 = to_gmax4002(sd);
	const struct gmax4002_mode *mode = gmax4002->cur_mode;
	u32 val = 0;

	if (mode->hdr_mode == NO_HDR)
		val = 1 << (GMAX4002_LANES - 1) | V4L2_MBUS_CSI2_CHANNEL_0 |
		      V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	if (mode->hdr_mode == HDR_X2)
		val = 1 << (GMAX4002_LANES - 1) | V4L2_MBUS_CSI2_CHANNEL_0 |
		      V4L2_MBUS_CSI2_CONTINUOUS_CLOCK |
		      V4L2_MBUS_CSI2_CHANNEL_1;

	config->type = V4L2_MBUS_CSI2;
	config->flags = val;

	return 0;
}

static void gmax4002_get_module_inf(struct gmax4002 *gmax4002,
				    struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, GMAX4002_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, gmax4002->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, gmax4002->len_name, sizeof(inf->base.lens));
}

static int gmax4002_set_hdrae(struct gmax4002 *gmax4002,
			      struct preisp_hdrae_exp_s *ae)
{
	// FIXME:
	return 0;
}

static long gmax4002_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct gmax4002 *gmax4002 = to_gmax4002(sd);
	struct rkmodule_hdr_cfg *hdr_cfg;
	const struct gmax4002_mode *mode;
	const struct gmax4002_mode_group *mg =
		&supported_mode_groups[default_group];
	long ret = 0;
	u64 pixel_rate = 0;
	u32 i, h, w, stream, val;

	switch (cmd) {
	case PREISP_CMD_SET_HDRAE_EXP:
		if (gmax4002->cur_mode->hdr_mode == HDR_X2)
			ret = gmax4002_set_hdrae(gmax4002, arg);
		break;
	case RKMODULE_GET_MODULE_INFO:
		gmax4002_get_module_inf(gmax4002, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		hdr_cfg->esp.mode = HDR_NORMAL_VC;
		hdr_cfg->hdr_mode = gmax4002->cur_mode->hdr_mode;
		v4l2_info(sd, "RKMODULE_GET_HDR_CFG()\n");
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr_cfg = (struct rkmodule_hdr_cfg *)arg;
		w = gmax4002->cur_mode->width;
		h = gmax4002->cur_mode->height;
		for (i = 0; i < gmax4002->cfg_num; i++) {
			if (w == mg->modes[i].width &&
			    h == mg->modes[i].height &&
			    mg->modes[i].hdr_mode == hdr_cfg->hdr_mode) {
				gmax4002_change_mode(gmax4002, &mg->modes[i]);
				break;
			}
		}
		if (i == gmax4002->cfg_num) {
			dev_err(&gmax4002->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr_cfg->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			mode = gmax4002->cur_mode;
			if (gmax4002->streaming) {
				ret |= gmax4002_write_array(gmax4002->client,
							    mode->reg_list_c);
				ret |= gmax4002_write_array(gmax4002->client,
							    mode->reg_list_s);
				ret |= gmax4002_write_array(gmax4002->client,
							    mode->reg_list_f);
				if (ret)
					return ret;
			}
			w = mode->hts_def - mode->width;
			h = mode->vts_def - mode->height;
			__v4l2_ctrl_modify_range(gmax4002->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(gmax4002->vblank, h,
						 GMAX4002_VTS_MAX - mode->height,
						 1, h);
			__v4l2_ctrl_s_ctrl(gmax4002->link_freq,
					   mode->mipi_freq_idx);
			pixel_rate = (u64)link_freq_items[mode->mipi_freq_idx] /
				     mode->bpp * 2 * GMAX4002_LANES;
			__v4l2_ctrl_s_ctrl_int64(gmax4002->pixel_rate,
						 pixel_rate);
			dev_info(&gmax4002->client->dev, "sensor mode: %d\n",
				 mode->hdr_mode);
		}
		v4l2_info(sd, "RKMODULE_SET_HDR_CFG()\n");
		break;
	case RKMODULE_SET_QUICK_STREAM:
		stream = *((u32 *)arg);
		ret = gmax4002_read_reg(gmax4002->client,
					GMAX4002_REG_CTRL_MODE,
					GMAX4002_REG_VALUE_08BIT, &val);
		if (stream)
			val |= GMAX4002_BIT_STREAM_EN;
		else
			val &= ~GMAX4002_BIT_STREAM_EN;
		ret |= gmax4002_write_reg(gmax4002->client,
					  GMAX4002_REG_CTRL_MODE,
					  GMAX4002_REG_VALUE_08BIT, val);
		v4l2_info(sd, "RKMODULE_SET_QUICK_STREAM()\n");
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long gmax4002_compat_ioctl32(struct v4l2_subdev *sd, unsigned int cmd,
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

		ret = gmax4002_ioctl(sd, cmd, inf);
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
		ret = gmax4002_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = gmax4002_ioctl(sd, cmd, hdr);
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
		ret = gmax4002_ioctl(sd, cmd, hdr);
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
		ret = gmax4002_ioctl(sd, cmd, hdrae);
		kfree(hdrae);
		break;
	case RKMODULE_SET_CONVERSION_GAIN:
		if (copy_from_user(&cg, up, sizeof(cg)))
			return -EFAULT;
		ret = gmax4002_ioctl(sd, cmd, &cg);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		if (copy_from_user(&stream, up, sizeof(u32)))
			return -EFAULT;
		ret = gmax4002_ioctl(sd, cmd, &stream);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int __gmax4002_mipi_phy_cal(struct gmax4002 *gmax4002)
{
	int ret = 0;
	int val = 0;
	struct i2c_client *client = gmax4002->client;

	ret |= gmax4002_read_reg(client, 0x3024, GMAX4002_REG_VALUE_08BIT, &val);
	val &= ~BIT(5);
	ret |= gmax4002_write_reg(client, 0x3024, GMAX4002_REG_VALUE_08BIT, val);
	msleep(1);

	ret |= gmax4002_read_reg(client, 0x3023, GMAX4002_REG_VALUE_08BIT, &val);
	val |= 0xf0;
	ret |= gmax4002_write_reg(client, 0x3023, GMAX4002_REG_VALUE_08BIT, val);
	msleep(1);

	ret |= gmax4002_read_reg(client, 0x3023, GMAX4002_REG_VALUE_08BIT, &val);
	val |= BIT(0);
	ret |= gmax4002_write_reg(client, 0x3023, GMAX4002_REG_VALUE_08BIT, val);
	msleep(1);

	ret |= gmax4002_read_reg(client, 0x3023, GMAX4002_REG_VALUE_08BIT, &val);
	val |= 0x06;
	ret |= gmax4002_write_reg(client, 0x3023, GMAX4002_REG_VALUE_08BIT, val);
	msleep(1);

	ret |= gmax4002_read_reg(client, 0x3023, GMAX4002_REG_VALUE_08BIT, &val);
	val |= 0x20;
	ret |= gmax4002_write_reg(client, 0x3023, GMAX4002_REG_VALUE_08BIT, val);
	msleep(1);

	ret |= gmax4002_read_reg(client, 0x3024, GMAX4002_REG_VALUE_08BIT, &val);
	val |= BIT(5);
	ret |= gmax4002_write_reg(client, 0x3024, GMAX4002_REG_VALUE_08BIT, val);
	msleep(1);

	ret |= gmax4002_read_reg(client, 0x3023, GMAX4002_REG_VALUE_08BIT, &val);
	val &= ~BIT(2);
	ret |= gmax4002_write_reg(client, 0x3023, GMAX4002_REG_VALUE_08BIT, val);
	msleep(1);

	return ret;
}

static int __gmax4002_start_stream(struct gmax4002 *gmax4002)
{
	int ret = 0;
	int val = 0;

	ret |= gmax4002_write_reg(gmax4002->client,
				  GMAX4002_REG_PWR_UP_EN,
				  GMAX4002_REG_VALUE_08BIT, 0);
	ret |= gmax4002_write_reg(gmax4002->client,
				  GMAX4002_REG_PWR_DOWN_EN,
				  GMAX4002_REG_VALUE_08BIT, 1);

	msleep(5);

	ret |= gmax4002_write_array(gmax4002->client, gmax4002->cur_mode->reg_list_c);
	ret |= gmax4002_write_array(gmax4002->client, gmax4002->cur_mode->reg_list_s);
	ret |= gmax4002_write_array(gmax4002->client, gmax4002->cur_mode->reg_list_f);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_handler_setup(&gmax4002->ctrl_handler);
	if (ret)
		return ret;

	if (gmax4002->sleep->cur.val != 0)
		return 0;

	msleep(5);

	ret = __gmax4002_mipi_phy_cal(gmax4002);

	ret |= gmax4002_read_reg(gmax4002->client,
				 GMAX4002_REG_CTRL_MODE,
				 GMAX4002_REG_VALUE_08BIT, &val);
	val |= GMAX4002_BIT_STREAM_EN;
	ret |= gmax4002_write_reg(gmax4002->client, GMAX4002_REG_CTRL_MODE,
				  GMAX4002_REG_VALUE_08BIT, val);

	return ret;
}

static int __gmax4002_stop_stream(struct gmax4002 *gmax4002)
{
	int ret, val;

	gmax4002->has_init_exp = false;
	ret = gmax4002_read_reg(gmax4002->client,
				GMAX4002_REG_CTRL_MODE,
				GMAX4002_REG_VALUE_08BIT, &val);
	val &= ~GMAX4002_BIT_STREAM_EN;
	return gmax4002_write_reg(gmax4002->client, GMAX4002_REG_CTRL_MODE,
				  GMAX4002_REG_VALUE_08BIT, val);
}

static int gmax4002_s_stream(struct v4l2_subdev *sd, int on)
{
	struct gmax4002 *gmax4002 = to_gmax4002(sd);
	struct i2c_client *client = gmax4002->client;
	int ret = 0;

	v4l2_dbg(1, debug, sd, "s_stream(%d) ...\n", on);

	mutex_lock(&gmax4002->mutex);
	on = !!on;
	if (on == gmax4002->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __gmax4002_start_stream(gmax4002);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__gmax4002_stop_stream(gmax4002);
		pm_runtime_put(&client->dev);
	}

	gmax4002->streaming = on;

unlock_and_return:
	mutex_unlock(&gmax4002->mutex);
	v4l2_dbg(1, debug, sd, "s_stream(%d) done!\n", on);
	return ret;
}

static int gmax4002_s_power(struct v4l2_subdev *sd, int on)
{
	struct gmax4002 *gmax4002 = to_gmax4002(sd);
	struct i2c_client *client = gmax4002->client;
	int ret = 0;

	v4l2_dbg(1, debug, sd, "s_power(%d) ...\n", on);

	mutex_lock(&gmax4002->mutex);

	/* If the power state is not modified - no work to do. */
	if (gmax4002->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}
		ret |= gmax4002_write_reg(gmax4002->client,
					  GMAX4002_REG_PWR_DOWN_EN,
					  GMAX4002_REG_VALUE_08BIT,
					  0);
		ret |= gmax4002_write_reg(gmax4002->client,
					  GMAX4002_REG_CTRL_MODE,
					  GMAX4002_REG_VALUE_08BIT,
					  GMAX4002_BIT_CLK_STABLE_EN);
		msleep(5);
		ret |= gmax4002_write_reg(gmax4002->client,
					  GMAX4002_REG_PWR_UP_EN,
					  GMAX4002_REG_VALUE_08BIT,
					  1);
		ret |= gmax4002_write_reg(gmax4002->client,
					  GMAX4002_REG_CTRL_MODE,
					  GMAX4002_REG_VALUE_08BIT,
					  GMAX4002_BIT_STREAM_EN);
		gmax4002->power_on = true;
	} else {
		ret |= gmax4002_write_reg(gmax4002->client,
					  GMAX4002_REG_PWR_UP_EN,
					  GMAX4002_REG_VALUE_08BIT,
					  0);
		ret |= gmax4002_write_reg(gmax4002->client,
					  GMAX4002_REG_PWR_DOWN_EN,
					  GMAX4002_REG_VALUE_08BIT,
					  1);
		pm_runtime_put(&client->dev);
		gmax4002->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&gmax4002->mutex);

	v4l2_dbg(1, debug, sd, "s_power(%d) done!\n", on);

	return ret;
}

static int __gmax4002_power_on(struct gmax4002 *gmax4002)
{
	int ret;
	struct device *dev = &gmax4002->client->dev;
	struct v4l2_subdev *sd = i2c_get_clientdata(gmax4002->client);

	v4l2_dbg(1, debug, sd, "power_on() ...\n");

	if (!IS_ERR_OR_NULL(gmax4002->pins_default)) {
		ret = pinctrl_select_state(gmax4002->pinctrl,
					   gmax4002->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}
	ret = clk_set_rate(gmax4002->xvclk, GMAX4002_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (40MHz)\n");
	ret = clk_get_rate(gmax4002->xvclk);
	if (ret != GMAX4002_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched (%d), modes are based on 40MHz\n", ret);
	ret = clk_prepare_enable(gmax4002->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		return ret;
	}

	ret = regulator_bulk_enable(GMAX4002_NUM_SUPPLIES, gmax4002->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(gmax4002->pwdn_gpio))
		gpiod_set_value_cansleep(gmax4002->pwdn_gpio, 0);

	if (!IS_ERR(gmax4002->reset_gpio))
		gpiod_set_value_cansleep(gmax4002->reset_gpio, 1);
	usleep_range(5000, 10000);
	if (!IS_ERR(gmax4002->reset_gpio))
		gpiod_set_value_cansleep(gmax4002->reset_gpio, 0);
	usleep_range(2000, 4000);

	return 0;

disable_clk:
	clk_disable_unprepare(gmax4002->xvclk);

	v4l2_dbg(1, debug, sd, "power_on() done!\n");

	return ret;
}

static void __gmax4002_power_off(struct gmax4002 *gmax4002)
{
	int ret;
	struct device *dev = &gmax4002->client->dev;
	struct v4l2_subdev *sd = i2c_get_clientdata(gmax4002->client);

	v4l2_dbg(1, debug, sd, "power_off() ...\n");

	if (!IS_ERR(gmax4002->pwdn_gpio))
		gpiod_set_value_cansleep(gmax4002->pwdn_gpio, 1);
	if (!IS_ERR(gmax4002->reset_gpio))
		gpiod_set_value_cansleep(gmax4002->reset_gpio, 1);
	if (!IS_ERR_OR_NULL(gmax4002->pins_sleep)) {
		ret = pinctrl_select_state(gmax4002->pinctrl,
					   gmax4002->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	clk_disable_unprepare(gmax4002->xvclk);
	regulator_bulk_disable(GMAX4002_NUM_SUPPLIES, gmax4002->supplies);

	v4l2_dbg(1, debug, sd, "power_off() done!\n");
}

static int gmax4002_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gmax4002 *gmax4002 = to_gmax4002(sd);

	return __gmax4002_power_on(gmax4002);
}

static int gmax4002_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gmax4002 *gmax4002 = to_gmax4002(sd);

	__gmax4002_power_off(gmax4002);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int gmax4002_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct gmax4002 *gmax4002 = to_gmax4002(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct gmax4002_mode_group *mg =
		&supported_mode_groups[default_group];
	const struct gmax4002_mode *def_mode = &mg->modes[default_mode];

	mutex_lock(&gmax4002->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&gmax4002->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int
gmax4002_enum_frame_interval(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_frame_interval_enum *fie)
{
	struct gmax4002 *gmax4002 = to_gmax4002(sd);
	const struct gmax4002_mode_group *mg = &supported_mode_groups[default_group];

	if (fie->index >= gmax4002->cfg_num)
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

static const struct dev_pm_ops gmax4002_pm_ops = {
	SET_RUNTIME_PM_OPS(gmax4002_runtime_suspend,
			   gmax4002_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops gmax4002_internal_ops = {
	.open = gmax4002_open,
};
#endif

static const struct v4l2_subdev_core_ops gmax4002_core_ops = {
	.s_power = gmax4002_s_power,
	.ioctl = gmax4002_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = gmax4002_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops gmax4002_video_ops = {
	.s_stream = gmax4002_s_stream,
	.g_frame_interval = gmax4002_g_frame_interval,
	.g_mbus_config = gmax4002_get_mbus_config,
};

static const struct v4l2_subdev_pad_ops gmax4002_pad_ops = {
	.enum_mbus_code = gmax4002_enum_mbus_code,
	.enum_frame_size = gmax4002_enum_frame_sizes,
	.enum_frame_interval = gmax4002_enum_frame_interval,
	.get_fmt = gmax4002_get_fmt,
	.set_fmt = gmax4002_set_fmt,
	.get_selection = gmax4002_get_selection,
};

static const struct v4l2_subdev_ops gmax4002_subdev_ops = {
	.core	= &gmax4002_core_ops,   /* v4l2_subdev_core_ops gmax4002_core_ops */
	.video	= &gmax4002_video_ops,  /* */
	.pad	= &gmax4002_pad_ops,    /* */
};

static int gmax4002_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gmax4002 *gmax4002 =
		container_of(ctrl->handler, struct gmax4002, ctrl_handler);
	struct i2c_client *client = gmax4002->client;
	s64 max;
	int ret = 0;
	u32 val;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = gmax4002->cur_mode->height + ctrl->val - 3;
		__v4l2_ctrl_modify_range(gmax4002->exposure,
					 gmax4002->exposure->minimum, max,
					 gmax4002->exposure->step,
					 gmax4002->exposure->default_value);
		break;
	}

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		dev_dbg(&client->dev, "set exposure 0x%x\n", val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		if (gmax4002->cur_mode->hdr_mode != NO_HDR || gmax4002->lock_gain->val)
			goto out_ctrl;
		ret = gmax4002_set_gain(gmax4002, ctrl->val);
		dev_dbg(&client->dev, "set analogue gain 0x%x\n", ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		dev_dbg(&client->dev, "set vblank 0x%x\n", ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = gmax4002_enable_test_pattern(gmax4002, ctrl->val);
		dev_dbg(&client->dev, "set test pattern: 0x%x, ret: %d\n", ctrl->val, ret);
		break;
	case V4L2_CID_HFLIP:
		ret = gmax4002_read_reg(gmax4002->client, GMAX4002_REG_FLIP_H,
				        GMAX4002_REG_VALUE_08BIT, &val);
		if (ret)
			break;
		if (ctrl->val)
			val |= GMAX4002_BIT_FLIP_H_EN;
		else
			val &= ~GMAX4002_BIT_FLIP_H_EN;
		ret |= gmax4002_write_reg(gmax4002->client, GMAX4002_REG_HOLD,
					  GMAX4002_REG_VALUE_08BIT, 1);
		ret |= gmax4002_write_reg(gmax4002->client, GMAX4002_REG_FLIP_H,
					  GMAX4002_REG_VALUE_08BIT, val);
		ret |= gmax4002_write_reg(gmax4002->client, GMAX4002_REG_HOLD,
					  GMAX4002_REG_VALUE_08BIT, 0);
		dev_dbg(&client->dev, "set hflip: 0x%x, ret: %d\n", val, ret);
		break;
	case V4L2_CID_VFLIP:
		ret = gmax4002_read_reg(gmax4002->client, GMAX4002_REG_FLIP_V,
				        GMAX4002_REG_VALUE_08BIT, &val);
		if (ret)
			break;
		if (ctrl->val)
			val &= ~GMAX4002_BIT_FLIP_V_NE;
		else
			val |= GMAX4002_BIT_FLIP_V_NE;
		ret |= gmax4002_write_reg(gmax4002->client, GMAX4002_REG_FLIP_V,
					  GMAX4002_REG_VALUE_08BIT, val);
		dev_dbg(&client->dev, "set vflip: 0x%x, ret: %d\n", val, ret);
		break;
	case V4L2_CID_EXT_TRIGGER:
		ret = gmax4002_read_reg(gmax4002->client, GMAX4002_REG_INT_EXP_EN,
				        GMAX4002_REG_VALUE_08BIT, &val);
		if (ctrl->val)
			val &= ~0x01;
		else
			val |= 0x01;
		ret |= gmax4002_write_reg(gmax4002->client, GMAX4002_REG_INT_EXP_EN,
					  GMAX4002_REG_VALUE_08BIT, val);
		dev_dbg(&client->dev, "set extenal trigger: 0x%x, ret: %d\n", ctrl->val, ret);
		break;
	case V4L2_CID_SLEEP:
		ret |= gmax4002_read_reg(gmax4002->client, GMAX4002_REG_CTRL_MODE,
					  GMAX4002_REG_VALUE_08BIT, &val);
		if (ctrl->val)
			val &= GMAX4002_BIT_STREAM_EN;
		else
			val |= GMAX4002_BIT_STREAM_EN;
		ret |= gmax4002_write_reg(gmax4002->client, GMAX4002_REG_CTRL_MODE,
					  GMAX4002_REG_VALUE_08BIT, val);
		dev_dbg(&client->dev, "set sleep: 0x%x, ret: %d\n", ctrl->val, ret);
		break;
	case V4L2_CID_LOCK_GAIN:
		dev_dbg(&client->dev, "set lock gain: 0x%x, ret: %d\n", ctrl->val, ret);
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

static const struct v4l2_ctrl_ops gmax4002_ctrl_ops = {
	.s_ctrl = gmax4002_set_ctrl,
};


static const struct v4l2_ctrl_config gmax4002_ctrl_ext_trigger = {
	.ops = &gmax4002_ctrl_ops,
	.id = V4L2_CID_EXT_TRIGGER,
	.name = "Ext Trigger",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = false,
	.max = true,
	.step = 1,
	.def = true,
};

static const struct v4l2_ctrl_config gmax4002_ctrl_sleep = {
	.ops = &gmax4002_ctrl_ops,
	.id = V4L2_CID_SLEEP,
	.name = "Sleep",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = false,
	.max = true,
	.step = 1,
	.def = false,
};

static const struct v4l2_ctrl_config gmax4002_ctrl_lock_gain = {
	.ops = &gmax4002_ctrl_ops,
	.id = V4L2_CID_LOCK_GAIN,
	.name = "Lock Gain",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = false,
	.max = true,
	.step = 1,
	.def = false,
};

static int gmax4002_initialize_controls(struct gmax4002 *gmax4002)
{
	const struct gmax4002_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;
	u64 pixel_rate = 0;

	handler = &gmax4002->ctrl_handler;
	mode = gmax4002->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 9);
	if (ret)
		return ret;
	handler->lock = &gmax4002->mutex;

	gmax4002->link_freq =
		v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				       ARRAY_SIZE(link_freq_items) - 1, 0,
				       link_freq_items);
	__v4l2_ctrl_s_ctrl(gmax4002->link_freq, mode->mipi_freq_idx);

	/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
	pixel_rate = (u32)link_freq_items[mode->mipi_freq_idx] / mode->bpp * 2 *
		     GMAX4002_LANES;
	gmax4002->pixel_rate =
		v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE, 0,
				  GMAX4002_MAX_PIXEL_RATE, 1, pixel_rate);

	if (handler->error) {
		ret = handler->error;
		dev_err(&gmax4002->client->dev, "Failed to init controls(%d)\n",
			ret);
		goto err_free_handler;
	}

	h_blank = mode->hts_def - mode->width;
	gmax4002->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
					    h_blank, h_blank, 1, h_blank);
	if (gmax4002->hblank)
		gmax4002->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	gmax4002->vblank =
		v4l2_ctrl_new_std(handler, &gmax4002_ctrl_ops, V4L2_CID_VBLANK,
				  vblank_def, GMAX4002_VTS_MAX - mode->height, 1,
				  vblank_def);

	exposure_max = mode->vts_def - 3;
	gmax4002->exposure =
		v4l2_ctrl_new_std(handler, &gmax4002_ctrl_ops, V4L2_CID_EXPOSURE,
				  GMAX4002_EXPOSURE_MIN, exposure_max,
				  GMAX4002_EXPOSURE_STEP, mode->exp_def);

	gmax4002->anal_gain =
		v4l2_ctrl_new_std(handler, &gmax4002_ctrl_ops,
				  V4L2_CID_ANALOGUE_GAIN, GMAX4002_GAIN_MIN,
				  GMAX4002_GAIN_MAX, GMAX4002_GAIN_STEP,
				  GMAX4002_GAIN_DEFAULT);

	gmax4002->test_pattern = v4l2_ctrl_new_std_menu_items(
		handler, &gmax4002_ctrl_ops, V4L2_CID_TEST_PATTERN,
		ARRAY_SIZE(gmax4002_test_pattern_menu) - 1, 0, 0,
		gmax4002_test_pattern_menu);

	v4l2_ctrl_new_std(handler, &gmax4002_ctrl_ops, V4L2_CID_HFLIP, 0, 1, 1,
			  0);
	v4l2_ctrl_new_std(handler, &gmax4002_ctrl_ops, V4L2_CID_VFLIP, 0, 1, 1,
			  0);
	gmax4002->ext_trigger =
		v4l2_ctrl_new_custom(handler, &gmax4002_ctrl_ext_trigger, NULL);
	gmax4002->sleep = v4l2_ctrl_new_custom(handler, &gmax4002_ctrl_sleep, NULL);
	gmax4002->lock_gain = v4l2_ctrl_new_custom(handler, &gmax4002_ctrl_lock_gain, NULL);

	if (handler->error) {
		ret = handler->error;
		dev_err(&gmax4002->client->dev, "Failed to init controls(%d)\n",
			ret);
		goto err_free_handler;
	}

	gmax4002->subdev.ctrl_handler = handler;
	gmax4002->has_init_exp = false;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int gmax4002_read_chip_id(struct i2c_client *client)
{
	int i = 0;
	int ret = 0;
	int ids[4] = {25, 26, 27, 28};
	int vals[4] = {0, 0, 0, 0};

	for (i = 0; i < 4; i++) {
		ret |= gmax4002_write_reg(client, GMAX4002_REG_OTP_ADDR,
					  GMAX4002_REG_VALUE_08BIT, ids[i]);
		ret |= gmax4002_read_reg(client, GMAX4002_REG_OTP_RVAL,
					 GMAX4002_REG_VALUE_08BIT, &vals[i]);
	}
	if (ret == 0) {
		ret = vals[2] << 16 | vals[1] << 8 | vals[0];
		ret = ret >> 6;
	}
	return ret;
}

static int gmax4002_check_sensor_id(struct gmax4002 *gmax4002,
				    struct i2c_client *client)
{
	struct device *dev = &gmax4002->client->dev;
	u32 id = 0;
	int ret;

	id = gmax4002_read_chip_id(client);
	if (id != GMAX4002_CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%04d), ret(%04d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected FV-IG02 sensor\n");

	return 0;
}

static int gmax4002_configure_regulators(struct gmax4002 *gmax4002)
{
	unsigned int i;

	for (i = 0; i < GMAX4002_NUM_SUPPLIES; i++)
		gmax4002->supplies[i].supply = gmax4002_supply_names[i];

	return devm_regulator_bulk_get(&gmax4002->client->dev,
				       GMAX4002_NUM_SUPPLIES,
				       gmax4002->supplies);
}

#ifdef HAS_RKLASER_SYSFS
static int gmax4002_get_temperature(struct gmax4002 *gmax4002,
				    struct i2c_client *client)
{
	int ret = 0;
	int v[2] = {0, 0};

	mutex_lock(&gmax4002->mutex);
	ret |= gmax4002_read_reg(client, 0x324f,
				 GMAX4002_REG_VALUE_08BIT, &v[0]);
	ret |= gmax4002_read_reg(client, 0x3250,
				 GMAX4002_REG_VALUE_08BIT, &v[1]);
	mutex_unlock(&gmax4002->mutex);
	return ((v[1] << 8) | v[0]);
}

static ssize_t sysfs_sensor_temp_dc_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gmax4002 *gmax4002 = to_gmax4002(sd);
	s64 temp = gmax4002_get_temperature(gmax4002, client);
	/* y = x * 0.4232 - 334.83 */
	temp *= 423;
	temp -= 334830;
	return sprintf(buf, "%lld\n", temp);
}

static ssize_t sysfs_sensor_temp_raw_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gmax4002 *gmax4002 = to_gmax4002(sd);
	return sprintf(buf, "%d\n", gmax4002_get_temperature(gmax4002, client));
}

static struct device_attribute attributes[] = {
	__ATTR(sensor_temp_dc, S_IRUSR, sysfs_sensor_temp_dc_show, NULL),
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

static int gmax4002_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct gmax4002 *gmax4002;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	gmax4002 = devm_kzalloc(dev, sizeof(*gmax4002), GFP_KERNEL);
	if (!gmax4002)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &gmax4002->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &gmax4002->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &gmax4002->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &gmax4002->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	gmax4002->cfg_num = supported_mode_groups[default_group].num;
	gmax4002->cur_mode = &supported_mode_groups[default_group].modes[default_mode];
	gmax4002->client = client;

	gmax4002->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(gmax4002->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}

	gmax4002->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(gmax4002->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	gmax4002->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(gmax4002->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	gmax4002->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(gmax4002->pinctrl)) {
		gmax4002->pins_default =
			pinctrl_lookup_state(gmax4002->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(gmax4002->pins_default))
			dev_err(dev, "could not get default pinstate\n");

		gmax4002->pins_sleep =
			pinctrl_lookup_state(gmax4002->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(gmax4002->pins_sleep))
			dev_err(dev, "could not get sleep pinstate\n");
	} else {
		dev_err(dev, "no pinctrl\n");
	}

	ret = gmax4002_configure_regulators(gmax4002);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	mutex_init(&gmax4002->mutex);

	sd = &gmax4002->subdev;
	v4l2_i2c_subdev_init(sd, client, &gmax4002_subdev_ops);
	ret = gmax4002_initialize_controls(gmax4002);
	if (ret)
		goto err_destroy_mutex;

	ret = __gmax4002_power_on(gmax4002);
	if (ret)
		goto err_free_handler;

	ret = gmax4002_check_sensor_id(gmax4002, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &gmax4002_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	gmax4002->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &gmax4002->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(gmax4002->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 gmax4002->module_index, facing,
		 GMAX4002_NAME, dev_name(sd->dev));
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
	__gmax4002_power_off(gmax4002);
err_free_handler:
	v4l2_ctrl_handler_free(&gmax4002->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&gmax4002->mutex);

	return ret;
}

static int gmax4002_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct gmax4002 *gmax4002 = to_gmax4002(sd);

#ifdef HAS_RKLASER_SYSFS
	remove_sysfs_interfaces(&client->dev);
#endif

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&gmax4002->ctrl_handler);
	mutex_destroy(&gmax4002->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__gmax4002_power_off(gmax4002);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id gmax4002_of_match[] = {
	{ .compatible = "fullv,fvig02" },
	{ .compatible = "gpixel,gmax4002" },
	{ },
};
MODULE_DEVICE_TABLE(of, gmax4002_of_match);
#endif

static const struct i2c_device_id gmax4002_match_id[] = {
	{ "fullv,fvig02", 0 },
	{ "gpixel,gmax4002", 0 },
	{ },
};

static struct i2c_driver gmax4002_i2c_driver = {
	.driver = {
		.name = GMAX4002_NAME,
		.pm = &gmax4002_pm_ops,
		.of_match_table = of_match_ptr(gmax4002_of_match),
	},
	.probe		= &gmax4002_probe,
	.remove		= &gmax4002_remove,
	.id_table	= gmax4002_match_id,
};

#ifdef CONFIG_ROCKCHIP_THUNDER_BOOT
module_i2c_driver(gmax4002_i2c_driver);
#else
static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&gmax4002_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&gmax4002_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);
#endif

MODULE_DESCRIPTION("Gpixel gmax4002 sensor driver");
MODULE_LICENSE("GPL v2");
