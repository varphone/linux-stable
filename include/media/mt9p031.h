#ifndef _MEDIA_MT9P031_H
#define _MEDIA_MT9P031_H

struct v4l2_subdev;

enum {
	MT9P031_COLOR_VERSION,
	MT9P031_MONOCHROME_VERSION,
};

struct mt9p031_platform_data {
	int (*set_clock)(struct v4l2_subdev *subdev, unsigned int rate);
	int (*reset)(struct v4l2_subdev *subdev, int active);
	u32 ext_freq; /* input frequency to the mt9p031 for PLL dividers */
	u32 target_freq; /* frequency target for the PLL */
	int version; /* MT9P031_COLOR_VERSION or MT9P031_MONOCHROME_VERSION */
};

#endif
