#ifndef _MXC_CAMERA_H_
#define _MXC_CAMERA_H_

#include <linux/device.h>

#define MXC_CAMERA_CLK_SRC	1
#define MXC_CAMERA_EXT_VSYNC	2
#define MXC_CAMERA_DP		4
#define MXC_CAMERA_PCP		8
#define MXC_CAMERA_HSP		0x10
#define MXC_CAMERA_VSP		0x20
#define MXC_CAMERA_DATAWIDTH_4	0x40
#define MXC_CAMERA_DATAWIDTH_8	0x80
#define MXC_CAMERA_DATAWIDTH_9	0x100
#define MXC_CAMERA_DATAWIDTH_10	0x200
#define MXC_CAMERA_DATAWIDTH_11	0x400
#define MXC_CAMERA_DATAWIDTH_12	0x800
#define MXC_CAMERA_DATAWIDTH_13	0x1000
#define MXC_CAMERA_DATAWIDTH_14	0x2000
#define MXC_CAMERA_DATAWIDTH_15	0x4000
#define MXC_CAMERA_DATAWIDTH_16	0x8000

#define MXC_CAMERA_DATAWIDTH_MASK (MXC_CAMERA_DATAWIDTH_4 | MXC_CAMERA_DATAWIDTH_8 | \
				   MXC_CAMERA_DATAWIDTH_9 | MXC_CAMERA_DATAWIDTH_10 | \
				   MXC_CAMERA_DATAWIDTH_11 | MXC_CAMERA_DATAWIDTH_12| \
				   MXC_CAMERA_DATAWIDTH_13 | MXC_CAMERA_DATAWIDTH_14| \
				   MXC_CAMERA_DATAWIDTH_15 | MXC_CAMERA_DATAWIDTH_16)

/**
 * struct mxc_camera_pdata - i.MXCx camera platform data
 * @flags:	MXC_CAMERA_* flags
 * @mclk_default:	master clock frequency in 10kHz units
 * @dma_dev:	IPU DMA device to match against in channel allocation
 */
struct mxc_camera_pdata {
	unsigned long flags;
	unsigned long mclk_default;
	int ipu;
	int csi;
};

#endif
