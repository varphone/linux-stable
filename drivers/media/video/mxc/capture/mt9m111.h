/*
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @defgroup Camera Sensor Drivers
 */

/*!
 * @file mt9m111.h
 *
 * @brief MT9M111 Camera Header file
 *
 * This header file contains defines and structures for the Micron mt9m111 camera.
 *
 * @ingroup Camera
 */

#ifndef MT9M111_H_
#define MT9M111_H_

#include <media/v4l2-common.h>
#include <linux/v4l2-mediabus.h>

/*!
 * Basic camera values
 */
#define MT9M111_FRAME_RATE        15
#define MT9M111_MCLK              54000000	/* Desired clock rate */
#define MT9M111_CLK_MIN           0	
#define MT9M111_CLK_MAX           54000000
#define MT9M111_MAX_WIDTH         1280		/* Max width for this camera */
#define MT9M111_MAX_HEIGHT        1024		/* Max height for this camera */

/*
 * MT9M111, MT9M112 and MT9M131:
 * i2c address is 0x48 or 0x5d (depending on SADDR pin)
 * The platform has to define i2c_board_info and call i2c_register_board_info()
 */

/*
 * Sensor core register addresses (0x000..0x0ff)
 */
#define MT9M111_CHIP_VERSION		0x000
#define MT9M111_ROW_START		0x001
#define MT9M111_COLUMN_START		0x002
#define MT9M111_WINDOW_HEIGHT		0x003
#define MT9M111_WINDOW_WIDTH		0x004
#define MT9M111_HORIZONTAL_BLANKING_B	0x005
#define MT9M111_VERTICAL_BLANKING_B	0x006
#define MT9M111_HORIZONTAL_BLANKING_A	0x007
#define MT9M111_VERTICAL_BLANKING_A	0x008
#define MT9M111_SHUTTER_WIDTH		0x009
#define MT9M111_ROW_SPEED		0x00a
#define MT9M111_EXTRA_DELAY		0x00b
#define MT9M111_SHUTTER_DELAY		0x00c
#define MT9M111_RESET			0x00d
#define MT9M111_READ_MODE_B		0x020
#define MT9M111_READ_MODE_A		0x021
#define MT9M111_FLASH_CONTROL		0x023
#define MT9M111_GREEN1_GAIN		0x02b
#define MT9M111_BLUE_GAIN		0x02c
#define MT9M111_RED_GAIN		0x02d
#define MT9M111_GREEN2_GAIN		0x02e
#define MT9M111_GLOBAL_GAIN		0x02f
#define MT9M111_CONTEXT_CONTROL		0x0c8
#define MT9M111_PAGE_MAP		0x0f0
#define MT9M111_BYTE_WISE_ADDR		0x0f1

#define MT9M111_RESET_SYNC_CHANGES	(1 << 15)
#define MT9M111_RESET_RESTART_BAD_FRAME	(1 << 9)
#define MT9M111_RESET_SHOW_BAD_FRAMES	(1 << 8)
#define MT9M111_RESET_RESET_SOC		(1 << 5)
#define MT9M111_RESET_OUTPUT_DISABLE	(1 << 4)
#define MT9M111_RESET_CHIP_ENABLE	(1 << 3)
#define MT9M111_RESET_ANALOG_STANDBY	(1 << 2)
#define MT9M111_RESET_RESTART_FRAME	(1 << 1)
#define MT9M111_RESET_RESET_MODE	(1 << 0)

#define MT9M111_RMB_MIRROR_COLS		(1 << 1)
#define MT9M111_RMB_MIRROR_ROWS		(1 << 0)
#define MT9M111_CTXT_CTRL_RESTART	(1 << 15)
#define MT9M111_CTXT_CTRL_DEFECTCOR_B	(1 << 12)
#define MT9M111_CTXT_CTRL_RESIZE_B	(1 << 10)
#define MT9M111_CTXT_CTRL_CTRL2_B	(1 << 9)
#define MT9M111_CTXT_CTRL_GAMMA_B	(1 << 8)
#define MT9M111_CTXT_CTRL_XENON_EN	(1 << 7)
#define MT9M111_CTXT_CTRL_READ_MODE_B	(1 << 3)
#define MT9M111_CTXT_CTRL_LED_FLASH_EN	(1 << 2)
#define MT9M111_CTXT_CTRL_VBLANK_SEL_B	(1 << 1)
#define MT9M111_CTXT_CTRL_HBLANK_SEL_B	(1 << 0)

/*
 * Colorpipe register addresses (0x100..0x1ff)
 */
#define MT9M111_OPER_MODE_CTRL		0x106
#define MT9M111_OUTPUT_FORMAT_CTRL	0x108
#define MT9M111_REDUCER_XZOOM_B		0x1a0
#define MT9M111_REDUCER_XSIZE_B		0x1a1
#define MT9M111_REDUCER_YZOOM_B		0x1a3
#define MT9M111_REDUCER_YSIZE_B		0x1a4
#define MT9M111_REDUCER_XZOOM_A		0x1a6
#define MT9M111_REDUCER_XSIZE_A		0x1a7
#define MT9M111_REDUCER_YZOOM_A		0x1a9
#define MT9M111_REDUCER_YSIZE_A		0x1aa

#define MT9M111_OUTPUT_FORMAT_CTRL2_A	0x13a
#define MT9M111_OUTPUT_FORMAT_CTRL2_B	0x19b

#define MT9M111_OPMODE_AUTOEXPO_EN	(1 << 14)
#define MT9M111_OPMODE_AUTOWHITEBAL_EN	(1 << 1)

#define MT9M111_OUTFMT_PROCESSED_BAYER	(1 << 14)
#define MT9M111_OUTFMT_BYPASS_IFP	(1 << 10)
#define MT9M111_OUTFMT_INV_PIX_CLOCK	(1 << 9)
#define MT9M111_OUTFMT_RGB		(1 << 8)
#define MT9M111_OUTFMT_RGB565		(0 << 6)
#define MT9M111_OUTFMT_RGB555		(1 << 6)
#define MT9M111_OUTFMT_RGB444x		(2 << 6)
#define MT9M111_OUTFMT_RGBx444		(3 << 6)
#define MT9M111_OUTFMT_TST_RAMP_OFF	(0 << 4)
#define MT9M111_OUTFMT_TST_RAMP_COL	(1 << 4)
#define MT9M111_OUTFMT_TST_RAMP_ROW	(2 << 4)
#define MT9M111_OUTFMT_TST_RAMP_FRAME	(3 << 4)
#define MT9M111_OUTFMT_SHIFT_3_UP	(1 << 3)
#define MT9M111_OUTFMT_AVG_CHROMA	(1 << 2)
#define MT9M111_OUTFMT_SWAP_YCbCr_C_Y	(1 << 1)
#define MT9M111_OUTFMT_SWAP_RGB_EVEN	(1 << 1)
#define MT9M111_OUTFMT_SWAP_YCbCr_Cb_Cr	(1 << 0)

/*! I2C Slave Address */
#define MT9M111_I2C_ADDRESS	0x48

/*!
 * The image resolution enum for the mt9m111 sensor
 */
typedef enum {
	MT9M111_OutputResolution_VGA = 0,	/*!< VGA size */
	MT9M111_OutputResolution_QVGA,	/*!< QVGA size */
	MT9M111_OutputResolution_CIF,	/*!< CIF size */
	MT9M111_OutputResolution_QCIF,	/*!< QCIF size */
	MT9M111_OutputResolution_QQVGA,	/*!< QQVGA size */
	MT9M111_OutputResolution_SXGA	/*!< SXGA size */
} MT9M111_OutputResolution;

enum {
	MT9M111_WINWIDTH = 0x287,
	MT9M111_WINWIDTH_DEFAULT = 0x287,
	MT9M111_WINWIDTH_MIN = 0x9,

	MT9M111_WINHEIGHT = 0x1E7,
	MT9M111_WINHEIGHT_DEFAULT = 0x1E7,

	MT9M111_HORZBLANK_DEFAULT = 0x26,
	MT9M111_HORZBLANK_MIN = 0x9,
	MT9M111_HORZBLANK_MAX = 0x3FF,

	MT9M111_VERTBLANK_DEFAULT = 0x4,
	MT9M111_VERTBLANK_MIN = 0x3,
	MT9M111_VERTBLANK_MAX = 0xFFF,
};

enum mt9m111_context {
	HIGHPOWER = 0,
	LOWPOWER,
};

typedef struct {
	u8 index;
	u16 width;
	u16 height;
} mt9m111_image_format;


#endif				/* MT9M111_H_  */
