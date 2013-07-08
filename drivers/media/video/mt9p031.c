#include <linux/videodev2.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/log2.h>

#include <media/mt9p031.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-chip-ident.h>
#include <media/soc_camera.h>
#include "mxc_camera.h"

#define MT9P031_PIXEL_ARRAY_WIDTH			2752
#define MT9P031_PIXEL_ARRAY_HEIGHT			2004

#define MT9P031_CHIP_VERSION				0x00
#define		MT9P031_CHIP_VERSION_VALUE		0x1801
#define MT9P031_ROW_START				0x01
#define		MT9P031_ROW_START_MIN			0
#define		MT9P031_ROW_START_MAX			2004
#define		MT9P031_ROW_START_DEF			54
#define MT9P031_COLUMN_START				0x02
#define		MT9P031_COLUMN_START_MIN		0
#define		MT9P031_COLUMN_START_MAX		2750
#define		MT9P031_COLUMN_START_DEF		16
#define MT9P031_WINDOW_HEIGHT				0x03
#define		MT9P031_WINDOW_HEIGHT_MIN		2
#define		MT9P031_WINDOW_HEIGHT_MAX		2006
#define		MT9P031_WINDOW_HEIGHT_DEF		1944
#define MT9P031_WINDOW_WIDTH				0x04
#define		MT9P031_WINDOW_WIDTH_MIN		2
#define		MT9P031_WINDOW_WIDTH_MAX		2752
#define		MT9P031_WINDOW_WIDTH_DEF		2592
#define MT9P031_HORIZONTAL_BLANK			0x05
#define		MT9P031_HORIZONTAL_BLANK_MIN		0
#define		MT9P031_HORIZONTAL_BLANK_MAX		4095
#define MT9P031_VERTICAL_BLANK				0x06
#define		MT9P031_VERTICAL_BLANK_MIN		0
#define		MT9P031_VERTICAL_BLANK_MAX		4095
#define		MT9P031_VERTICAL_BLANK_DEF		25
#define MT9P031_OUTPUT_CONTROL				0x07
#define		MT9P031_OUTPUT_CONTROL_CEN		2
#define		MT9P031_OUTPUT_CONTROL_SYN		1
#define		MT9P031_OUTPUT_CONTROL_DEF		0x1f82
#define MT9P031_SHUTTER_WIDTH_UPPER			0x08
#define MT9P031_SHUTTER_WIDTH_LOWER			0x09
#define		MT9P031_SHUTTER_WIDTH_MIN		1
#define		MT9P031_SHUTTER_WIDTH_MAX		1048575
#define		MT9P031_SHUTTER_WIDTH_DEF		1943
#define	MT9P031_PLL_CONTROL				0x10
#define		MT9P031_PLL_CONTROL_DEF			0x0050
#define		MT9P031_PLL_CONTROL_PWROFF		0x0050
#define		MT9P031_PLL_CONTROL_PWRON		0x0051
#define		MT9P031_PLL_CONTROL_USEPLL		0x0052
#define	MT9P031_PLL_CONFIG_1				0x11
#define	MT9P031_PLL_CONFIG_2				0x12
#define MT9P031_PIXEL_CLOCK_CONTROL			0x0a
#define 	MT9P031_PIXEL_CLOCK_INVERT 		(1u << 15)
#define MT9P031_FRAME_RESTART				0x0b
#define		MT9P031_FRAME_RESTART_SET		(1u << 0)
#define MT9P031_SHUTTER_DELAY				0x0c
#define MT9P031_RST					0x0d
#define		MT9P031_RST_ENABLE			1
#define		MT9P031_RST_DISABLE			0
#define MT9P031_READ_MODE_1				0x1e
#define		MT9P031_READ_MODE_1_DEF				0x4006
#define MT9P031_READ_MODE_2				0x20
#define		MT9P031_READ_MODE_2_ROW_MIR		(1 << 15)
#define		MT9P031_READ_MODE_2_COL_MIR		(1 << 14)
#define		MT9P031_READ_MODE_2_ROW_BLC		(1 << 6)
#define MT9P031_ROW_ADDRESS_MODE			0x22
#define MT9P031_COLUMN_ADDRESS_MODE			0x23
#define MT9P031_GLOBAL_GAIN				0x35
#define		MT9P031_GLOBAL_GAIN_MIN			8
#define		MT9P031_GLOBAL_GAIN_MAX			1024
#define		MT9P031_GLOBAL_GAIN_DEF			8
#define		MT9P031_GLOBAL_GAIN_MULT		(1 << 6)
#define MT9P031_ROW_BLACK_DEF_OFFSET			0x4b
#define MT9P031_TEST_PATTERN				0xa0
#define		MT9P031_TEST_PATTERN_SHIFT		3
#define		MT9P031_TEST_PATTERN_ENABLE		(1 << 0)
#define		MT9P031_TEST_PATTERN_DISABLE		(0 << 0)
#define MT9P031_TEST_PATTERN_GREEN			0xa1
#define MT9P031_TEST_PATTERN_RED			0xa2
#define MT9P031_TEST_PATTERN_BLUE			0xa3

static bool		g_is_mono;
module_param_named(mono, g_is_mono, bool, 0444);

/* MT9P031 has only one fixed colorspace per pixelcode */
struct mt9p031_datafmt {
	enum v4l2_mbus_pixelcode	code;
	enum v4l2_colorspace		colorspace;
};

/* Find a data format by a pixel code in an array */
static const struct mt9p031_datafmt *mt9p031_find_datafmt(
	enum v4l2_mbus_pixelcode code, const struct mt9p031_datafmt *fmt,
	int n)
{
	int i;
	for (i = 0; i < n; i++)
		if (fmt[i].code == code)
			return fmt + i;

	return NULL;
}

//TODO: 12bit data has to be supported
static const struct mt9p031_datafmt mt9p031_colour_fmts[] = {
	/*
	 * Order important: first natively supported,
	 * second supported with a GPIO extender
	 */
	{V4L2_MBUS_FMT_SBGGR10_1X10, V4L2_COLORSPACE_SRGB},
//	{V4L2_MBUS_FMT_SGRBG12_1X12, V4L2_COLORSPACE_SRGB},
	{V4L2_MBUS_FMT_SBGGR8_1X8, V4L2_COLORSPACE_SRGB},
};

static const struct mt9p031_datafmt mt9p031_monochrome_fmts[] = {
	/* Order important - see above */
	{V4L2_MBUS_FMT_Y10_1X10, V4L2_COLORSPACE_JPEG},
//	{V4L2_MBUS_FMT_Y12_1X12, V4L2_COLORSPACE_JPEG},
	{V4L2_MBUS_FMT_Y8_1X8, V4L2_COLORSPACE_JPEG},
};

struct mt9p031_pll_divs {
	u32 ext_freq;
	u32 target_freq;
	u8 m;
	u8 n;
	u8 p1;
};

struct mt9p031 {
	struct v4l2_subdev subdev;
	struct v4l2_rect rect;	/* Sensor window */
	int model;	/* V4L2_IDENT_MT9V02x codes from v4l2-chip-ident.h */
	u16 xskip;
	u16 yskip;

	const struct mt9p031_datafmt *fmt;
	const struct mt9p031_datafmt *fmts;
	int num_fmts;

	bool use_pll;
	const struct mt9p031_pll_divs *pll;

	/* Register cache */
	u16 output_control;
	u16 mode2;
};

static struct mt9p031 *to_mt9p031(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct mt9p031, subdev);
}

static int mt9p031_read(struct i2c_client *client, const u8 reg)
{
	s32 data = i2c_smbus_read_word_data(client, reg);
	return data < 0 ? data : be16_to_cpu(data);
}

static int mt9p031_write(struct i2c_client *client, const u8 reg,
		     const u16 data)
{
	return i2c_smbus_write_word_data(client, reg, cpu_to_be16(data));
}

static int mt9p031_set_output_control(struct mt9p031 *mt9p031, u16 clear,
				      u16 set)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mt9p031->subdev);
	u16 value = (mt9p031->output_control & ~clear) | set;
	int ret;

	ret = mt9p031_write(client, MT9P031_OUTPUT_CONTROL, value);
	if (ret < 0)
		return ret;

	mt9p031->output_control = value;
	return 0;
}

static int mt9p031_set_mode2(struct mt9p031 *mt9p031, u16 clear, u16 set)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mt9p031->subdev);
	u16 value = (mt9p031->mode2 & ~clear) | set;
	int ret;

	ret = mt9p031_write(client, MT9P031_READ_MODE_2, value);
	if (ret < 0)
		return ret;

	mt9p031->mode2 = value;
	return 0;
}

static void mt9p031_video_remove(struct soc_camera_device *icd)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);

	dev_dbg(&icd->dev, "Video removed: %p, %p\n",
		icd->dev.parent, icd->vdev);
	if (icl->free_bus)
		icl->free_bus(icl);
}

/*TODO: add new dividers */
static const struct mt9p031_pll_divs mt9p031_divs[] = {
	/* ext_freq	target_freq	m	n	p1 */
        {27000000,      60000000,       40,     6,      3},
	{26000000,	48000000,	37,	5,	4},
	{26700000,      53300000,       40,     5,      4},
	{26700000,	96000000,	108,	10,	3},
	{26700000,      34600000,	52,	5,	8},
	{26700000,      62000000,       93,     10,     4},
        {24000000,      96000000,       24,     3,      2}
//	{26700000,      96000000,       72,     10,     2}
};

static int mt9p031_pll_get_divs(struct mt9p031 *mt9p031)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mt9p031->subdev);
	struct soc_camera_device *icd = client->dev.platform_data;
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mxc_camera_dev *mxc_cam = ici->priv;
	int i;

	if ((mxc_cam->pdata->mclk_default_rate ==
		mxc_cam->pdata->mclk_target_rate) ||
			(!mxc_cam->pdata->use_pll)) {
		mt9p031->use_pll = false;
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(mt9p031_divs); i++) {
		if (mt9p031_divs[i].ext_freq ==
				mxc_cam->pdata->mclk_default_rate &&
				mt9p031_divs[i].target_freq ==
				mxc_cam->pdata->mclk_target_rate) {
			mt9p031->pll = &mt9p031_divs[i];
			mt9p031->use_pll = true;
			return 0;
		}
	}

	dev_err(&client->dev, "Couldn't find PLL dividers for ext_freq = %ld\n", mxc_cam->pdata->mclk_default_rate);
	return -EINVAL;
}

static int mt9p031_pll_enable(struct mt9p031 *mt9p031)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mt9p031->subdev);
	int ret;

	ret = mt9p031_write(client, MT9P031_PLL_CONTROL,
			    MT9P031_PLL_CONTROL_PWRON);
	if (ret < 0)
		return ret;

	ret = mt9p031_write(client, MT9P031_PLL_CONFIG_1,
			    (mt9p031->pll->m << 8) | (mt9p031->pll->n - 1));
	if (ret < 0)
		return ret;

	ret = mt9p031_write(client, MT9P031_PLL_CONFIG_2, mt9p031->pll->p1 - 1);
	if (ret < 0)
		return ret;

	usleep_range(1000, 2000);
	ret = mt9p031_write(client, MT9P031_PLL_CONTROL,
			    MT9P031_PLL_CONTROL_PWRON |
			    MT9P031_PLL_CONTROL_USEPLL);

	return ret;
}

static inline int mt9p031_pll_disable(struct mt9p031 *mt9p031)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mt9p031->subdev);

	return mt9p031_write(client, MT9P031_PLL_CONTROL,
			     MT9P031_PLL_CONTROL_PWROFF);
}

/* -----------------------------------------------------------------------------
 * Soc camera operations
 */

static int mt9p031_set_bus_param(struct soc_camera_device *icd,
				 unsigned long flags)
{
	struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	unsigned int width_flag = flags & SOCAM_DATAWIDTH_MASK;
	int ret;

	if (!is_power_of_2(width_flag))
		return -EINVAL;

	if (icl->set_bus_param) {
		ret = icl->set_bus_param(icl, width_flag);
		if (ret)
			return ret;
	} else {
		if (width_flag != SOCAM_DATAWIDTH_10)
			return -EINVAL;
	}

	flags = soc_camera_apply_sensor_flags(icl, flags);

	if (flags & SOCAM_SENSOR_INVERT_PCLK)
		ret = mt9p031_write(client, MT9P031_PIXEL_CLOCK_CONTROL, MT9P031_PIXEL_CLOCK_INVERT);
	if (ret < 0)
		return ret;
	
	return 0;
}

static unsigned long mt9p031_query_bus_param(struct soc_camera_device *icd)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	unsigned int flags = SOCAM_MASTER | SOCAM_SLAVE |
		SOCAM_PCLK_SAMPLE_RISING | SOCAM_PCLK_SAMPLE_FALLING |
		SOCAM_HSYNC_ACTIVE_HIGH | SOCAM_HSYNC_ACTIVE_LOW |
		SOCAM_VSYNC_ACTIVE_HIGH | SOCAM_VSYNC_ACTIVE_LOW |
		SOCAM_DATA_ACTIVE_HIGH;

	if (icl->query_bus_param)
		flags |= icl->query_bus_param(icl) & SOCAM_DATAWIDTH_MASK;
	else 
		flags |= SOCAM_DATAWIDTH_10;

	return soc_camera_apply_sensor_flags(icl, flags);
}

static struct soc_camera_ops mt9p031_ops = {
	.set_bus_param		= mt9p031_set_bus_param,
	.query_bus_param	= mt9p031_query_bus_param,
};

/* -----------------------------------------------------------------------------
 * V4L2 subdev video operations
 */

static int mt9p031_set_params(struct mt9p031 *mt9p031)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mt9p031->subdev);
	const struct v4l2_rect *crop = &mt9p031->rect;
	unsigned int hblank;
	unsigned int vblank;
	unsigned int xskip;
	unsigned int yskip;
	unsigned int xbin;
	unsigned int ybin;
	unsigned int width;
	unsigned int height;
	unsigned int crop_top = crop->top;
	unsigned int crop_left = crop->left;
	unsigned int crop_width = crop->width;
	unsigned int crop_height = crop->height;
	int shutter_width_upper;
	int shutter_width_lower;
	int ret;

	/* Row and column binning and skipping. Use the maximum binning value
	 * compatible with the skipping settings.
	 */
	width = clamp_t(unsigned int, ALIGN(mt9p031->rect.width, 2),
			max(crop->width / 7, MT9P031_WINDOW_WIDTH_MIN),
			crop->width);
	height = clamp_t(unsigned int, ALIGN(mt9p031->rect.height, 2),
			max(crop->height / 8, MT9P031_WINDOW_HEIGHT_MIN),
			 crop->height);

	switch (width) {
	case (2048):
		crop_left = crop->left + ((2592 - 2048) / 2);
		crop_top = crop->top + ((1944 - 1536) / 2);
		break;
	case (1920):
		crop_left = crop->left + ((2592 - 1920) / 2);
		crop_top = crop->top + ((1944 - 1080) / 2);
		break;
	case (1600):
		crop_left = crop->left + ((2592 - 1600) / 2);
		crop_top = crop->top + ((1944 - 1200) / 2);
		break;
	case (1280):
		if (height == 1024) {
			crop_left = crop->left + ((2592 - width) / 2);
			crop_top = crop->top + ((1944 - height) / 2);
		} else if (height == 720) {
			crop_width = 2560;
			crop_left = crop->left + ((2592 - crop_width) / 2);
			crop_height = 1440;
			crop_top = crop->top + ((1944 - crop_height) / 2);
		}
		break;
	case (1024):
		crop_width = 2048;
		crop_left = crop->left + ((2592 - crop_width) / 2);
		crop_height = 1536;
		crop_top = crop->top + ((1944 - crop_height) / 2);
		break;
	case (800):
		crop_width = 1600;
		crop_left = crop->left + ((2592 - crop_width) / 2);
		crop_height = 1200;
		crop_top = crop->top + ((1944 - crop_height) / 2);
		break;
	case (640):
		crop_width = 2560;
		crop_left = crop->left + ((2592 - crop_width) / 2);
		crop_height = 1920;
		crop_top = crop->top + ((1944 - crop_height) / 2);
		break;
	default:
		break;
	}

	ret = mt9p031_write(client, MT9P031_COLUMN_START, crop_left);
	if (ret < 0)
		return ret;
	ret = mt9p031_write(client, MT9P031_ROW_START, crop_top);
	if (ret < 0)
		return ret;
	ret = mt9p031_write(client, MT9P031_WINDOW_WIDTH, crop_width - 1);
	if (ret < 0)
		return ret;
	ret = mt9p031_write(client, MT9P031_WINDOW_HEIGHT, crop_height - 1);
	if (ret < 0)
		return ret;

	xskip = DIV_ROUND_CLOSEST(crop_width, width);
	yskip = DIV_ROUND_CLOSEST(crop_height, height);
	xbin = 1 << (ffs(xskip) - 1);
	ybin = 1 << (ffs(yskip) - 1);

	ret = mt9p031_write(client, MT9P031_COLUMN_ADDRESS_MODE,
			    ((xbin - 1) << 4) | (xskip - 1));
	if (ret < 0)
		return ret;
	ret = mt9p031_write(client, MT9P031_ROW_ADDRESS_MODE,
			    ((ybin - 1) << 4) | (yskip - 1));
	if (ret < 0)
		return ret;

	/* 
	 * Blanking - calculate values according to the MT9P031's datasheet
	 */
	hblank = 346 * ybin + 64 + (80 >> min_t(unsigned int, xbin, 3));

	shutter_width_upper = mt9p031_read(client, MT9P031_SHUTTER_WIDTH_UPPER);
	shutter_width_lower = mt9p031_read(client, MT9P031_SHUTTER_WIDTH_LOWER);

	vblank = max(8, (max(1, (2 * 16 * shutter_width_upper) +
			shutter_width_lower) - crop->height)) + 1;

	ret = mt9p031_write(client, MT9P031_HORIZONTAL_BLANK, hblank - 1);
	if (ret < 0)
		return ret;
	ret = mt9p031_write(client, MT9P031_VERTICAL_BLANK, vblank - 1);
	if (ret < 0)
		return ret;

	return ret;
}

static int mt9p031_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9p031 *mt9p031 = to_mt9p031(client);
	int ret;

	ret = mt9p031_write(client,
			    MT9P031_FRAME_RESTART, MT9P031_FRAME_RESTART_SET);
	if (ret < 0)
		return ret;


	if (!enable) {
		ret = mt9p031_set_output_control(mt9p031, MT9P031_OUTPUT_CONTROL_CEN, 0);

		if (ret < 0)
			return ret;
		if (mt9p031->use_pll)
			return mt9p031_pll_disable(mt9p031);
		else
			return 0;
	}

	ret = mt9p031_set_params(mt9p031);
	if (ret < 0)
		return ret;

	/* Switch to master "normal" mode */
	ret = mt9p031_set_output_control(mt9p031, 0, MT9P031_OUTPUT_CONTROL_CEN);
	if (ret < 0)
		return ret;
	if (mt9p031->use_pll)
		return mt9p031_pll_enable(mt9p031);

	return 0;
}

static int mt9p031_s_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9p031 *mt9p031 = to_mt9p031(client);
	struct v4l2_rect rect = a->c;

	rect.left = clamp(ALIGN(a->c.left, 2), MT9P031_COLUMN_START_MIN,
			MT9P031_COLUMN_START_MAX);
	rect.top = clamp(ALIGN(a->c.top, 2), MT9P031_ROW_START_MIN,
			MT9P031_COLUMN_START_MAX);
	rect.width = clamp(ALIGN(a->c.width, 2),
			MT9P031_WINDOW_WIDTH_MIN,
			MT9P031_WINDOW_WIDTH_MAX);
	rect.height = clamp(ALIGN(a->c.height, 2),
			MT9P031_WINDOW_HEIGHT_MIN,
			MT9P031_WINDOW_HEIGHT_MAX);

	rect.width = min(rect.width, MT9P031_PIXEL_ARRAY_WIDTH - rect.left);
	rect.height = min(rect.height, MT9P031_PIXEL_ARRAY_HEIGHT - rect.top);

	soc_camera_limit_side(&rect.left, &rect.width,
		     MT9P031_COLUMN_START_DEF, MT9P031_WINDOW_WIDTH_MIN, MT9P031_WINDOW_WIDTH_MAX);

	soc_camera_limit_side(&rect.top, &rect.height,
		     MT9P031_ROW_START_DEF, MT9P031_WINDOW_HEIGHT_MIN, MT9P031_WINDOW_HEIGHT_MAX);

	dev_dbg(&client->dev, "Frame %dx%d pixel\n", rect.width, rect.height);

	mt9p031->rect = rect;

	return 0;
}

static int mt9p031_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9p031 *mt9p031 = to_mt9p031(client);

	a->c	= mt9p031->rect;
	a->type	= V4L2_BUF_TYPE_VIDEO_CAPTURE;

	return 0;
}

static int mt9p031_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	a->bounds.left			= MT9P031_COLUMN_START_DEF;
	a->bounds.top			= MT9P031_ROW_START_DEF;
	a->bounds.width			= MT9P031_WINDOW_WIDTH_DEF;
	a->bounds.height		= MT9P031_WINDOW_HEIGHT_DEF;
	a->defrect			= a->bounds;
	a->type				= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator	= 1;
	a->pixelaspect.denominator	= 1;

	return 0;
}

static int mt9p031_s_fmt(struct v4l2_subdev *sd,
			 struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9p031 *mt9p031 = to_mt9p031(client);
	struct v4l2_crop a = {
		.c = {
			.left	= mt9p031->rect.left,
			.top	= mt9p031->rect.top,
			.width	= mf->width,
			.height	= mf->height,
		},
	};
	int ret;

	ret = mt9p031_s_crop(sd, &a);
	if (!ret) {
		mf->width	= mt9p031->rect.width;
		mf->height	= mt9p031->rect.height;
		mt9p031->fmt	= mt9p031_find_datafmt(mf->code,
					mt9p031->fmts, mt9p031->num_fmts);
		mf->colorspace	= mt9p031->fmt->colorspace;
	}

	return ret;
}

static int mt9p031_try_fmt(struct v4l2_subdev *sd,
			   struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9p031 *mt9p031 = to_mt9p031(client);
	const struct mt9p031_datafmt *fmt;
	int align = mf->code == V4L2_MBUS_FMT_SBGGR8_1X8 ||
		mf->code == V4L2_MBUS_FMT_SBGGR10_1X10;

	v4l_bound_align_image(&mf->width, MT9P031_WINDOW_WIDTH_MIN,
		MT9P031_WINDOW_WIDTH_MAX, align,
		&mf->height, MT9P031_WINDOW_HEIGHT_MIN,
		MT9P031_WINDOW_HEIGHT_MAX, align, 0);

	fmt = mt9p031_find_datafmt(mf->code, mt9p031->fmts,
				   mt9p031->num_fmts);
	if (!fmt) {
		fmt = mt9p031->fmt;
		mf->code = fmt->code;
	}

	mf->colorspace	= fmt->colorspace;

	return 0;
}

static int mt9p031_g_fmt(struct v4l2_subdev *sd,
			 struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9p031 *mt9p031 = to_mt9p031(client);

	mf->width	= mt9p031->rect.width;
	mf->height	= mt9p031->rect.height;
	mf->code	= mt9p031->fmt->code;
	mf->colorspace	= mt9p031->fmt->colorspace;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int mt9p031_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			    enum v4l2_mbus_pixelcode *code)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9p031 *mt9p031 = to_mt9p031(client);

	if (index >= mt9p031->num_fmts) {
		return -EINVAL;
	}

	*code = mt9p031->fmts[index].code;
	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev core operations
 */

#define V4L2_CID_TEST_PATTERN		(V4L2_CID_USER_BASE | 0x1001)

static int mt9p031_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9p031 *mt9p031 = to_mt9p031(client);
	u16 data;
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = mt9p031_write(client, MT9P031_SHUTTER_WIDTH_UPPER,
				    (ctrl->value >> 16) & 0xffff);
		if (ret < 0)
			return ret;

		return mt9p031_write(client, MT9P031_SHUTTER_WIDTH_LOWER,
				     ctrl->value & 0xffff);

	case V4L2_CID_GAIN:
		/* Gain is controlled by 2 analog stages and a digital stage.
		 * Valid values for the 3 stages are
		 *
		 * Stage                Min     Max     Step
		 * ------------------------------------------
		 * First analog stage   x1      x2      1
		 * Second analog stage  x1      x4      0.125
		 * Digital stage        x1      x16     0.125
		 *
		 * To minimize noise, the gain stages should be used in the
		 * second analog stage, first analog stage, digital stage order.
		 * Gain from a previous stage should be pushed to its maximum
		 * value before the next stage is used.
		 */
		if (ctrl->value <= 32) {
			data = ctrl->value;
		} else if (ctrl->value <= 64) {
			ctrl->value &= ~1;
			data = (1 << 6) | (ctrl->value >> 1);
		} else {
			ctrl->value &= ~7;
			data = ((ctrl->value - 64) << 5) | (1 << 6) | 32;
		}

		return mt9p031_write(client, MT9P031_GLOBAL_GAIN, data);

	case V4L2_CID_HFLIP:
		if (ctrl->value)
			return mt9p031_set_mode2(mt9p031,
					0, MT9P031_READ_MODE_2_COL_MIR);
		else
			return mt9p031_set_mode2(mt9p031,
					MT9P031_READ_MODE_2_COL_MIR, 0);

	case V4L2_CID_VFLIP:
		if (ctrl->value)
			return mt9p031_set_mode2(mt9p031,
					0, MT9P031_READ_MODE_2_ROW_MIR);
		else
			return mt9p031_set_mode2(mt9p031,
					MT9P031_READ_MODE_2_ROW_MIR, 0);

	case V4L2_CID_TEST_PATTERN:
		if (!ctrl->value) {
			ret = mt9p031_set_mode2(mt9p031,
					0, MT9P031_READ_MODE_2_ROW_BLC);
			if (ret < 0)
				return ret;

			return mt9p031_write(client, MT9P031_TEST_PATTERN,
					     MT9P031_TEST_PATTERN_DISABLE);
		}

		ret = mt9p031_write(client, MT9P031_TEST_PATTERN_GREEN, 0x05a0);
		if (ret < 0)
			return ret;
		ret = mt9p031_write(client, MT9P031_TEST_PATTERN_RED, 0x0a50);
		if (ret < 0)
			return ret;
		ret = mt9p031_write(client, MT9P031_TEST_PATTERN_BLUE, 0x0aa0);
		if (ret < 0)
			return ret;

		ret = mt9p031_set_mode2(mt9p031, MT9P031_READ_MODE_2_ROW_BLC,
					0);
		if (ret < 0)
			return ret;
		ret = mt9p031_write(client, MT9P031_ROW_BLACK_DEF_OFFSET, 0);
		if (ret < 0)
			return ret;

		return mt9p031_write(client, MT9P031_TEST_PATTERN,
				((ctrl->value - 1) << MT9P031_TEST_PATTERN_SHIFT)
				| MT9P031_TEST_PATTERN_ENABLE);
	}
	return 0;
}

static int mt9p031_g_chip_ident(struct v4l2_subdev *sd,
				struct v4l2_dbg_chip_ident *id)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9p031 *mt9p031 = to_mt9p031(client);

	if (id->match.type != V4L2_CHIP_MATCH_I2C_ADDR);
		return -EINVAL;

	if (id->match.addr != client->addr)
		return -EINVAL;

	id->ident = mt9p031->model;
	id->revision = 0;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int mt9p031_g_register(struct v4l2_subdev *sd,
                              struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (reg->match.type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xff)
		return -EINVAL;

	if (reg->match.addr != client->addr)
		return -ENODEV;

	reg->size = 2;
	reg->val = mt9p031_read(client, reg->reg);

	if (reg->val > 0xffff)
		return -EIO;

	return 0;
}

static int mt9p031_s_register(struct v4l2_subdev *sd,
			      struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

        if (reg->match.type != V4L2_CHIP_MATCH_I2C_ADDR || reg->reg > 0xff)
		return -EINVAL;

	if (reg->match.addr != client->addr)
		return -ENODEV;

	if (mt9p031_write(client, reg->reg, reg->val) < 0)
		return -EIO;

	return 0;
}
#endif

static struct v4l2_subdev_core_ops mt9p031_subdev_core_ops = {
	.s_ctrl		= mt9p031_s_ctrl,
	.g_chip_ident	= mt9p031_g_chip_ident,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register     = mt9p031_g_register,
	.s_register     = mt9p031_s_register,
#endif
};

static struct v4l2_subdev_video_ops mt9p031_subdev_video_ops = {
	.s_stream	= mt9p031_s_stream,
	.s_mbus_fmt	= mt9p031_s_fmt,
	.try_mbus_fmt	= mt9p031_try_fmt,
	.s_crop		= mt9p031_s_crop,
	.g_crop		= mt9p031_g_crop,
	.cropcap	= mt9p031_cropcap,
	.g_mbus_fmt	= mt9p031_g_fmt,
	.enum_mbus_fmt	= mt9p031_enum_fmt,
};

static struct v4l2_subdev_ops mt9p031_subdev_ops = {
	.core	= &mt9p031_subdev_core_ops,
	.video	= &mt9p031_subdev_video_ops,
};

/* -----------------------------------------------------------------------------
 * Driver initialization and probing
 */

static int mt9p031_init(struct i2c_client *client)
{
	int ret;

	ret = mt9p031_write(client, MT9P031_WINDOW_WIDTH, MT9P031_WINDOW_WIDTH_DEF);
	if (!ret)
		mt9p031_write(client, MT9P031_WINDOW_HEIGHT, MT9P031_WINDOW_HEIGHT_DEF);
	if (!ret)
		mt9p031_write(client, MT9P031_SHUTTER_WIDTH_LOWER, MT9P031_SHUTTER_WIDTH_DEF);

	return ret;
}

static int mt9p031_video_probe(struct soc_camera_device *icd,
					struct i2c_client *client)
{
	struct mt9p031 *mt9p031 = to_mt9p031(client);
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	s32 data;
	int ret;
	unsigned long flags;

	if (!icd->dev.parent ||
	    to_soc_camera_host(icd->dev.parent)->nr != icd->iface) {
		return -ENODEV;
	}

	data = mt9p031_read(client, MT9P031_CHIP_VERSION);
	switch (data) {
		case MT9P031_CHIP_VERSION_VALUE:
			printk("%s: MT9P031 camera found, chip ID: 0x%02x\n", __func__, data);
			mt9p031->model = V4L2_IDENT_MT9P031;
			break;
		default:
			printk("%s: No MT9P031 camera found\n", __func__);
			return -EINVAL;
	}

	if (g_is_mono)
		mt9p031->fmts = mt9p031_monochrome_fmts;
	else
		mt9p031->fmts = mt9p031_colour_fmts;

	mt9p031->num_fmts = 0;

	/*
	 * This is a 12bit sensor, but by default we only allow 10bit.
	 * The platform may support different bus widths due to
	 * different routing of the data lines.
	 */
	if (icl->query_bus_param) {
		flags = icl->query_bus_param(icl);
	} else {
		flags = SOCAM_DATAWIDTH_10;
	}
	if (flags & SOCAM_DATAWIDTH_10)
		mt9p031->num_fmts++;
	else {
		mt9p031->fmts++;
	}

	mt9p031->fmt = &mt9p031->fmts[0];

	ret = mt9p031_init(client);
	if (ret < 0)
		dev_err(&client->dev, "Failed to initialise the camera\n");

	return ret;
}

static int mt9p031_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	struct mt9p031 *mt9p031;
	struct soc_camera_device *icd = client->dev.platform_data;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct soc_camera_link *icl;
	int ret = 0;

	if (!icd) {
		dev_err(&client->dev, "MT9P031: missing soc-camera data!\n");
		return -EINVAL;
	}

	icl = to_soc_camera_link(icd);
	if (!icl) {
		dev_err(&client->dev, "MT9P031 driver needs platform data\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_warn(&adapter->dev,
			 "I2C-Adapter doesn't support I2C_FUNC_SMBUS_WORD\n");
		return -EIO;
	}

	mt9p031 = kzalloc(sizeof(struct mt9p031), GFP_KERNEL);
	if (!mt9p031)
		return -ENOMEM;

	mt9p031->output_control	= MT9P031_OUTPUT_CONTROL_DEF;
	mt9p031->mode2 = MT9P031_READ_MODE_2_ROW_BLC;

	v4l2_i2c_subdev_init(&mt9p031->subdev, client, &mt9p031_subdev_ops);

	icd->ops = &mt9p031_ops;

	mt9p031->rect.width = MT9P031_WINDOW_WIDTH_DEF;
	mt9p031->rect.height = MT9P031_WINDOW_HEIGHT_DEF;
	mt9p031->rect.left = MT9P031_COLUMN_START_DEF;
	mt9p031->rect.top = MT9P031_ROW_START_DEF;

	ret = mt9p031_pll_get_divs(mt9p031);

	ret = mt9p031_video_probe(icd, client);
	if (ret) {
		icd->ops = NULL;
		kfree(mt9p031);
	}

	return ret;
}

static int mt9p031_remove(struct i2c_client *client)
{
	struct mt9p031 *mt9p031 = to_mt9p031(client);
	struct soc_camera_device *icd = client->dev.platform_data;

	icd->ops = NULL;
	mt9p031_video_remove(icd);
	kfree(mt9p031);

	return 0;
}

static const struct i2c_device_id mt9p031_id[] = {
	{ "mt9p031", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mt9p031_id);

static struct i2c_driver mt9p031_i2c_driver = {
	.driver = {
		.name = "mt9p031",
	},
	.probe		= mt9p031_probe,
	.remove		= mt9p031_remove,
	.id_table	= mt9p031_id,
};

static int __init mt9p031_mod_init(void)
{
	return i2c_add_driver(&mt9p031_i2c_driver);
}

static void __exit mt9p031_mod_exit(void)
{
	i2c_del_driver(&mt9p031_i2c_driver);
}

module_init(mt9p031_mod_init);
module_exit(mt9p031_mod_exit);

MODULE_DESCRIPTION("Micron MT9P031 Camera driver");
MODULE_AUTHOR("Raman Tunik <r.tunik@sam-solutions.net>");
MODULE_LICENSE("GPL");
