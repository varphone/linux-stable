#ifndef MT9V002_H_
#define MT9V002_H_

#include <media/v4l2-common.h>
#include <linux/v4l2-mediabus.h>

#define is_mt9v024()   (mt9v022->model == V4L2_IDENT_MT9V024 ? 1 : 0)

/* mt9v022 selected register addresses */
#define MT9V022_CHIP_VERSION		0x00
#define MT9V022_COLUMN_START		0x01
#define MT9V022_ROW_START		0x02
#define MT9V022_WINDOW_HEIGHT		0x03
#define MT9V022_WINDOW_WIDTH		0x04
#define MT9V022_HORIZONTAL_BLANKING	0x05
#define MT9V022_VERTICAL_BLANKING	0x06
#define MT9V022_CHIP_CONTROL		0x07
#define MT9V022_SHUTTER_WIDTH1		0x08
#define MT9V022_SHUTTER_WIDTH2		0x09
#define MT9V022_SHUTTER_WIDTH_CTRL	0x0a
#define MT9V022_TOTAL_SHUTTER_WIDTH	0x0b
#define MT9V022_RESET			0x0c
#define MT9V022_READ_MODE		0x0d
#define MT9V022_MONITOR_MODE		0x0e
#define MT9V022_PIXEL_OPERATION_MODE	0x0f
#define MT9V022_LED_OUT_CONTROL		0x1b
#define MT9V022_ADC_MODE_CONTROL	0x1c
#define MT9V022_ANALOG_GAIN		0x35
#define MT9V022_BLACK_LEVEL_CALIB_CTRL	0x47
#define MT9V022_PIXCLK_FV_LV		(is_mt9v024() ? 0x72 : 0x74)
#define MT9V022_DIGITAL_TEST_PATTERN	0x7f
#define MT9V022_AEC_AGC_ENABLE		0xAF
#define MT9V022_MAX_TOTAL_SHUTTER_WIDTH	(is_mt9v024() ? 0xAD : 0xBD)

/* Progressive scan, master, defaults */
#define MT9V022_CHIP_CONTROL_DEFAULT	0x188

/*!
 * Basic camera values
 */
#define MT9V022_FRAME_RATE        30
#define MT9V022_MCLK              27000000	/* Desired clock rate */
#define MT9V022_CLK_MIN           0		/* This clock rate yields 15 fps */
#define MT9V022_CLK_MAX           48000000


#define MT9V022_MAX_WIDTH		752
#define MT9V022_MAX_HEIGHT		480
#define MT9V022_MIN_WIDTH		48
#define MT9V022_MIN_HEIGHT		32
#define MT9V022_COLUMN_SKIP		1
#define MT9V022_ROW_SKIP		4


#define ENABLE_LVDS_OUT
#define LVDS_OUT_8BIT


#endif
