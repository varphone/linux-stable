/*!
 * @defgroup Camera Sensor Drivers
 */

/*!
 * @file mt9m001.h
 *
 * @brief MT9M001 Camera Header file
 *
 * This header file contains defines and structures for the Micron mt9m001 camera.
 *
 * @ingroup Camera
 */

#ifndef MT9M001_H_
#define MT9M001_H_

#include <media/v4l2-common.h>
#include <linux/v4l2-mediabus.h>

/*!
 * Basic camera values
 */
#define MT9M001_FRAME_RATE        30
#define MT9M001_MCLK              22000000	/* Desired clock rate */
#define MT9M001_CLK_MIN           0		/* This clock rate yields 15 fps */
#define MT9M001_CLK_MAX           48000000
#define MT9M001_MAX_WIDTH         1280		/* Max width for this camera */
#define MT9M001_MAX_HEIGHT        1024		/* Max height for this camera */

/*
 * MT9M001, MT9M112 and MT9M131:
 * i2c address is 0x48 or 0x5d (depending on SADDR pin)
 * The platform has to define i2c_board_info and call i2c_register_board_info()
 */

#define MT9M001_CHIP_VERSION		0x00
#define MT9M001_ROW_START		0x01
#define MT9M001_COLUMN_START		0x02
#define MT9M001_WINDOW_HEIGHT		0x03
#define MT9M001_WINDOW_WIDTH		0x04
#define MT9M001_HORIZONTAL_BLANKING	0x05
#define MT9M001_VERTICAL_BLANKING	0x06
#define MT9M001_OUTPUT_CONTROL		0x07
#define MT9M001_SHUTTER_WIDTH		0x09
#define MT9M001_FRAME_RESTART		0x0b
#define MT9M001_SHUTTER_DELAY		0x0c
#define MT9M001_RESET			0x0d
#define MT9M001_READ_OPTIONS1		0x1e
#define MT9M001_READ_OPTIONS2		0x20
#define MT9M001_GLOBAL_GAIN		0x35
#define MT9M001_CHIP_ENABLE		0xF1

#define MT9M001_MAX_WIDTH		1280
#define MT9M001_MAX_HEIGHT		1024
#define MT9M001_MIN_WIDTH		48
#define MT9M001_MIN_HEIGHT		32
#define MT9M001_COLUMN_SKIP		20
#define MT9M001_ROW_SKIP		12

/*! I2C Slave Address */
#define MT9M001_I2C_ADDRESS	0x5d

/*!
 * The image resolution enum for the mt9m001 sensor
 */
typedef enum {
	MT9M001_OutputResolution_VGA = 0,	/*!< VGA size */
	MT9M001_OutputResolution_QVGA,	/*!< QVGA size */
	MT9M001_OutputResolution_CIF,	/*!< CIF size */
	MT9M001_OutputResolution_QCIF,	/*!< QCIF size */
	MT9M001_OutputResolution_QQVGA,	/*!< QQVGA size */
	MT9M001_OutputResolution_SXGA	/*!< SXGA size */
} MT9M001_OutputResolution;

enum {
	MT9M001_WINWIDTH = 0x287,
	MT9M001_WINWIDTH_DEFAULT = 0x287,
	MT9M001_WINWIDTH_MIN = 0x9,

	MT9M001_WINHEIGHT = 0x1E7,
	MT9M001_WINHEIGHT_DEFAULT = 0x1E7,

	MT9M001_HORZBLANK_DEFAULT = 0x26,
	MT9M001_HORZBLANK_MIN = 0x9,
	MT9M001_HORZBLANK_MAX = 0x3FF,

	MT9M001_VERTBLANK_DEFAULT = 0x4,
	MT9M001_VERTBLANK_MIN = 0x3,
	MT9M001_VERTBLANK_MAX = 0xFFF,
};

typedef struct {
	u8 index;
	u16 width;
	u16 height;
} mt9m001_image_format;


#endif				/* MT9M001_H_  */
