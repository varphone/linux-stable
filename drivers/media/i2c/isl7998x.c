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

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/gpio/consumer.h>
#include <linux/regmap.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "isl7998x_regs.h"

#define ISL7998X_XCLK_MIN 24000000
#define ISL7998X_XCLK_MAX 27000000

struct isl7998x_ctrls {
	struct v4l2_ctrl_handler handler;
	struct {
		struct v4l2_ctrl *auto_gain;
		struct v4l2_ctrl *gain;
	};
	struct v4l2_ctrl *brightness;
	struct v4l2_ctrl *saturation;
	struct v4l2_ctrl *contrast;
	struct v4l2_ctrl *hue;
	struct v4l2_ctrl *test_pattern;
};

struct isl7998x_dev {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */
	struct clk *xclk; /* system clock to ISL7998X */
	u32 xclk_freq;

	struct gpio_desc *pwdn_gpio;
	struct gpio_desc *reset_gpio;
	struct regmap *regmap;

	/* lock to protect all members below */
	struct mutex lock;

	int power_count;

	struct v4l2_mbus_framefmt fmt;

	const struct isl7998x_mode_info *current_mode;
	enum isl7998x_frame_rate current_fr;
	struct v4l2_fract frame_interval;

	struct isl7998x_ctrls ctrls;

	u32 prev_sysclk, prev_hts;
	u32 ae_low, ae_high, ae_target;

	bool pending_mode_change;
	bool streaming;
};

static const struct regmap_config isl7998x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.use_single_rw = true,
};

static inline struct isl7998x_dev *to_isl7998x_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct isl7998x_dev, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct isl7998x_dev,
			     ctrls.handler)->sd;
}

/* Download isl7998x settings to sensor through i2c */
static int isl7998x_load_regs(struct isl7998x_dev *sensor,
			      const struct isl7998x_mode_info *mode)
{
	return regmap_multi_reg_write(sensor->regmap, mode->regs, mode->num_regs);
}

static inline int isl7998x_read_reg(struct isl7998x_dev* sensor, u8 addr, u8* val)
{
    int err = 0;
    u32 reg_val = 0;

    err = regmap_read(sensor->regmap, addr, &reg_val);
    *val = reg_val & 0xFF;

    return err;
}

static inline int isl7998x_write_reg(struct isl7998x_dev* sensor, u8 addr, u8 val)
{
    int err;

    err = regmap_write(sensor->regmap, addr, val);
    if (err)
        pr_err("%s:i2c write failed, 0x%x = %x\n", __func__, addr, val);

    return err;
}

static int isl7998x_get_gain(struct isl7998x_dev *sensor)
{
	return 0;
}

static int isl7998x_set_stream(struct isl7998x_dev *sensor, bool on)
{
	return 0;
}

static const struct isl7998x_mode_info *
isl7998x_find_mode(struct isl7998x_dev *sensor, int fps, int width, int height)
{
	int i = 0;
	const struct isl7998x_mode_info *mode = NULL;

	for (i = 0; i < ISL7998X_NUM_MODES; i++) {
		mode = &isl7998x_mode_tables[i];

		if (mode->fps == fps &&
		    mode->width == width &&
		    mode->height == height)
			return mode;
	}

	return NULL;
}

static int isl7998x_set_mode(struct isl7998x_dev *sensor,
			     const struct isl7998x_mode_info *mode)
{
	int ret = 0;

	ret = isl7998x_load_regs(sensor, mode);

	sensor->current_mode = mode;

	return 0;
}

/* restore the last set video mode after chip power-on */
static int isl7998x_restore_mode(struct isl7998x_dev *sensor)
{
	/* now restore the last capture mode */
	return isl7998x_set_mode(sensor, sensor->current_mode);
}

static void isl7998x_power(struct isl7998x_dev *sensor, bool enable)
{
	gpiod_set_value(sensor->pwdn_gpio, enable ? 0 : 1);
}

static void isl7998x_reset(struct isl7998x_dev *sensor)
{
	if (!sensor->reset_gpio)
		return;

	gpiod_set_value(sensor->reset_gpio, 0);

	/* camera power cycle */
	isl7998x_power(sensor, false);
	usleep_range(5000, 10000);
	isl7998x_power(sensor, true);
	usleep_range(5000, 10000);

	gpiod_set_value(sensor->reset_gpio, 1);
	usleep_range(1000, 2000);

	gpiod_set_value(sensor->reset_gpio, 0);
	usleep_range(5000, 10000);
}

static int isl7998x_set_power(struct isl7998x_dev *sensor, bool on)
{
	int ret = 0;

	if (on) {
		clk_prepare_enable(sensor->xclk);

		isl7998x_reset(sensor);
		isl7998x_power(sensor, true);

		ret = isl7998x_restore_mode(sensor);
		if (ret)
			goto power_off;

		/*
		 * start streaming briefly followed by stream off in
		 * order to coax the clock lane into LP-11 state.
		 */
		ret = isl7998x_set_stream(sensor, true);
		if (ret)
			goto power_off;
		usleep_range(1000, 2000);
		ret = isl7998x_set_stream(sensor, false);
		if (ret)
			goto power_off;

		return 0;
	}

power_off:
	isl7998x_power(sensor, false);
xclk_off:
	clk_disable_unprepare(sensor->xclk);
	return ret;
}

/* --------------- Subdev Operations --------------- */

static int isl7998x_s_power(struct v4l2_subdev *sd, int on)
{
	int ret = 0;
	struct isl7998x_dev *sensor = to_isl7998x_dev(sd);

	mutex_lock(&sensor->lock);

	/*
	 * If the power count is modified from 0 to != 0 or from != 0 to 0,
	 * update the power state.
	 */
	if (sensor->power_count == !on) {
		ret = isl7998x_set_power(sensor, !!on);
		if (ret)
			goto out;
	}

	/* Update the power count. */
	sensor->power_count += on ? 1 : -1;
	WARN_ON(sensor->power_count < 0);
out:
	mutex_unlock(&sensor->lock);

	if (on && !ret && sensor->power_count == 1) {
		/* restore controls */
		ret = v4l2_ctrl_handler_setup(&sensor->ctrls.handler);
	}

	return ret;
}

static int isl7998x_try_frame_interval(
	struct isl7998x_dev *sensor,
	struct v4l2_fract *fi,
	u32 width,
	u32 height)
{
	u32 fps = 25;
	const struct isl7998x_mode_info *mode = NULL;

	if (fi->numerator == 0) {
		fi->denominator = 25;
		fi->numerator = 1;
		return 0;
	}

	fps = DIV_ROUND_CLOSEST(fi->denominator, fi->numerator);

	fi->denominator = fps;
	fi->numerator = 1;

	mode = isl7998x_find_mode(sensor, fps, width, height);

	return mode ? 0 : -EINVAL;
}

static int isl7998x_get_fmt(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_format *format)
{
	struct isl7998x_dev *sensor = to_isl7998x_dev(sd);
	struct v4l2_mbus_framefmt *fmt;

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt = v4l2_subdev_get_try_format(&sensor->sd, cfg, format->pad);
	else
		fmt = &sensor->fmt;

	format->format = *fmt;

	mutex_unlock(&sensor->lock);

	return 0;
}

static int isl7998x_try_fmt_internal(
	struct v4l2_subdev *sd,
	struct v4l2_mbus_framefmt *fmt, int fps,
	const struct isl7998x_mode_info **new_mode)
{
	struct isl7998x_dev *sensor = to_isl7998x_dev(sd);
	const struct isl7998x_mode_info *mode = NULL;

	mode = isl7998x_find_mode(sensor, fps, fmt->width, fmt->height);
	if (!mode)
		return -EINVAL;

	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = sensor->fmt.code;

	if (new_mode)
		*new_mode = mode;

	return 0;
}

static int isl7998x_set_fmt(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_format *format)
{
	int ret = 0;
	struct isl7998x_dev *sensor = to_isl7998x_dev(sd);
	const struct isl7998x_mode_info *new_mode = NULL;

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (sensor->streaming) {
		ret = -EBUSY;
		goto out;
	}

	ret = isl7998x_try_fmt_internal(sd,
					&format->format,
					sensor->frame_interval.denominator,
					&new_mode);
	if (ret)
		goto out;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *fmt =
				v4l2_subdev_get_try_format(sd, cfg, 0);

		*fmt = format->format;
		goto out;
	}

	sensor->current_mode = new_mode;
	sensor->fmt = format->format;
	sensor->pending_mode_change = true;
out:
	mutex_unlock(&sensor->lock);
	return ret;
}


/*
 * Sensor Controls.
 */

static int isl7998x_set_ctrl_hue(
	struct isl7998x_dev *sensor,
	int value)
{
	int ret = 0;

	return ret;
}

static int isl7998x_set_ctrl_contrast(
	struct isl7998x_dev *sensor,
	int value)
{
	int ret = 0;

	return ret;
}

static int isl7998x_set_ctrl_saturation(
	struct isl7998x_dev *sensor,
	int value)
{
	int ret = 0;

	return ret;
}

static int isl7998x_set_ctrl_white_balance(
	struct isl7998x_dev *sensor,
	int awb)
{
	int ret = 0;

	return ret;
}

static int isl7998x_set_ctrl_gain(
	struct isl7998x_dev *sensor,
	int auto_gain)
{
	int ret = 0;
	struct isl7998x_ctrls *ctrls = &sensor->ctrls;

	return ret;
}

static int isl7998x_set_ctrl_test_pattern(
	struct isl7998x_dev *sensor,
	int value)
{
	int ret = 0;

	return 0;
}

static int isl7998x_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	int val = 0;
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct isl7998x_dev *sensor = to_isl7998x_dev(sd);

	/* v4l2_ctrl_lock() locks our own mutex */

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		if (!ctrl->val)
			return 0;
		val = isl7998x_get_gain(sensor);
		if (val < 0)
			return val;
		sensor->ctrls.gain->val = val;
		break;
	}

	return 0;
}

static int isl7998x_s_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret = 0;
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct isl7998x_dev *sensor = to_isl7998x_dev(sd);

	/* v4l2_ctrl_lock() locks our own mutex */

	/*
	 * If the device is not powered up by the host driver do
	 * not apply any controls to H/W at this time. Instead
	 * the controls will be restored right after power-up.
	 */
	if (sensor->power_count == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		ret = isl7998x_set_ctrl_gain(sensor, ctrl->val);
		break;
	case V4L2_CID_HUE:
		ret = isl7998x_set_ctrl_hue(sensor, ctrl->val);
		break;
	case V4L2_CID_CONTRAST:
		ret = isl7998x_set_ctrl_contrast(sensor, ctrl->val);
		break;
	case V4L2_CID_SATURATION:
		ret = isl7998x_set_ctrl_saturation(sensor, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = isl7998x_set_ctrl_test_pattern(sensor, ctrl->val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops isl7998x_ctrl_ops = {
	.g_volatile_ctrl = isl7998x_g_volatile_ctrl,
	.s_ctrl = isl7998x_s_ctrl,
};

static const char * const test_pattern_menu[] = {
	"Disabled",
	"BBBBWB",
	"BBBWWB",
	"BBWBWB",
	"BBWWWB",
};

static int isl7998x_init_controls(struct isl7998x_dev *sensor)
{
	int ret = 0;
	const struct v4l2_ctrl_ops *ops = &isl7998x_ctrl_ops;
	struct isl7998x_ctrls *ctrls = &sensor->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;

	v4l2_ctrl_handler_init(hdl, 32);

	/* we can use our own mutex for the ctrl lock */
	hdl->lock = &sensor->lock;

	/* Auto/manual gain */
	ctrls->auto_gain = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_AUTOGAIN,
					     0, 1, 1, 1);
	ctrls->gain = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_GAIN,
					0, 1023, 1, 0);

	ctrls->saturation = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_SATURATION,
					      0, 255, 1, 64);
	ctrls->hue = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HUE,
				       0, 359, 1, 0);
	ctrls->contrast = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_CONTRAST,
					    0, 255, 1, 0);
	ctrls->test_pattern =
		v4l2_ctrl_new_std_menu_items(hdl, ops, V4L2_CID_TEST_PATTERN,
					     ARRAY_SIZE(test_pattern_menu) - 1,
					     0, 0, test_pattern_menu);

	if (hdl->error) {
		ret = hdl->error;
		goto free_ctrls;
	}

	ctrls->gain->flags |= V4L2_CTRL_FLAG_VOLATILE;

	v4l2_ctrl_auto_cluster(2, &ctrls->auto_gain, 0, true);

	sensor->sd.ctrl_handler = hdl;
	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static int isl7998x_enum_frame_size(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->pad != 0)
		return -EINVAL;
	if (fse->index > ISL7998X_MODE_PAL_720_576)
		return -EINVAL;

	fse->min_width = fse->max_width =
		isl7998x_mode_tables[fse->index].width;
	fse->min_height = fse->max_height =
		isl7998x_mode_tables[fse->index].height;

	return 0;
}

static int isl7998x_enum_frame_interval(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_frame_interval_enum *fie)
{
	int ret = 0;
	struct isl7998x_dev *sensor = to_isl7998x_dev(sd);
	struct v4l2_fract tpf;

	if (fie->pad != 0)
		return -EINVAL;
	if (fie->index >= ISL7998X_NUM_FRAMERATES)
		return -EINVAL;

	tpf.numerator = 1;
	tpf.denominator = isl7998x_framerates[fie->index];

	ret = isl7998x_try_frame_interval(sensor, &tpf,
					  fie->width, fie->height);
	if (ret < 0)
		return -EINVAL;

	fie->interval = tpf;

	return 0;
}

static int isl7998x_g_frame_interval(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_frame_interval *fi)
{
	struct isl7998x_dev *sensor = to_isl7998x_dev(sd);

	mutex_lock(&sensor->lock);
	fi->interval = sensor->frame_interval;
	mutex_unlock(&sensor->lock);

	return 0;
}

static int isl7998x_s_frame_interval(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_frame_interval *fi)
{
	struct isl7998x_dev *sensor = to_isl7998x_dev(sd);
	const struct isl7998x_mode_info *mode;
	int frame_rate, ret = 0;

	if (fi->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (sensor->streaming) {
		ret = -EBUSY;
		goto out;
	}

	mode = sensor->current_mode;

	ret = isl7998x_try_frame_interval(sensor, &fi->interval,
					  mode->width, mode->height);
	if (ret < 0)
		goto out;

	sensor->frame_interval = fi->interval;
	sensor->pending_mode_change = true;
out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static int isl7998x_enum_mbus_code(
	struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_mbus_code_enum *code)
{
	struct isl7998x_dev *sensor = to_isl7998x_dev(sd);

	if (code->pad != 0)
		return -EINVAL;
	if (code->index != 0)
		return -EINVAL;

	code->code = sensor->fmt.code;

	return 0;
}

static int isl7998x_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret = 0;
	struct isl7998x_dev *sensor = to_isl7998x_dev(sd);

	mutex_lock(&sensor->lock);

	if (sensor->streaming == !enable) {
		if (enable && sensor->pending_mode_change) {
			ret = isl7998x_set_mode(sensor, sensor->current_mode);
			if (ret)
				goto out;
		}

		ret = isl7998x_set_stream(sensor, enable);
		if (!ret)
			sensor->streaming = enable;
	}
out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static const struct v4l2_subdev_core_ops isl7998x_core_ops = {
	.s_power = isl7998x_s_power,
};

static const struct v4l2_subdev_video_ops isl7998x_video_ops = {
	.g_frame_interval = isl7998x_g_frame_interval,
	.s_frame_interval = isl7998x_s_frame_interval,
	.s_stream = isl7998x_s_stream,
};

static const struct v4l2_subdev_pad_ops isl7998x_pad_ops = {
	.enum_mbus_code = isl7998x_enum_mbus_code,
	.get_fmt = isl7998x_get_fmt,
	.set_fmt = isl7998x_set_fmt,
	.enum_frame_size = isl7998x_enum_frame_size,
	.enum_frame_interval = isl7998x_enum_frame_interval,
};

static const struct v4l2_subdev_ops isl7998x_subdev_ops = {
	.core = &isl7998x_core_ops,
	.video = &isl7998x_video_ops,
	.pad = &isl7998x_pad_ops,
};

static int isl7998x_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	int ret;
	u32 xclk_freq;
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct isl7998x_dev *sensor;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->regmap = devm_regmap_init_i2c(client, &isl7998x_regmap_config);
	if (IS_ERR(sensor->regmap)) {
		dev_err(&client->dev, "Regmap init failed: %ld\n",
			PTR_ERR(sensor->regmap));
		return -ENODEV;
	}

	sensor->i2c_client = client;
	sensor->fmt.code = MEDIA_BUS_FMT_UYVY8_2X8;
	sensor->fmt.width = 720;
	sensor->fmt.height = 576;
	sensor->fmt.field = V4L2_FIELD_NONE;
	sensor->frame_interval.numerator = 1;
	sensor->frame_interval.denominator = 25;
	sensor->current_mode = &isl7998x_mode_tables[ISL7998X_MODE_PAL_720_576];
	sensor->pending_mode_change = true;

	sensor->ae_target = 52;

	endpoint = fwnode_graph_get_next_endpoint(
		of_fwnode_handle(client->dev.of_node), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(endpoint, &sensor->ep);
	fwnode_handle_put(endpoint);
	if (ret) {
		dev_err(dev, "Could not parse endpoint\n");
		return ret;
	}

	if (sensor->ep.bus_type != V4L2_MBUS_CSI2) {
		dev_err(dev, "invalid bus type, must be MIPI CSI2\n");
		return -EINVAL;
	}

	/* get system clock (xclk) */
	sensor->xclk = devm_clk_get(dev, "xclk");
	if (IS_ERR(sensor->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(sensor->xclk);
	}

	ret = of_property_read_u32(dev_of_node(dev), "clock-frequency", &xclk_freq);
	if (ret < 0) {
		dev_err(dev, "xclk frequency must be set\n");
		return ret;
	}

	if (xclk_freq < ISL7998X_XCLK_MIN ||
	    xclk_freq > ISL7998X_XCLK_MAX) {
		dev_err(dev, "xclk frequency out of range: %d Hz\n",
			xclk_freq);
		return -EINVAL;
	}

	sensor->xclk_freq = clk_get_rate(sensor->xclk);
	if (xclk_freq != sensor->xclk_freq) {
		ret = clk_set_rate(sensor->xclk, xclk_freq);
		if (ret < 0) {
			dev_err(dev, "failed to set xclk to %d Hz\n", xclk_freq);
			return ret;
		}
	}

	/* request optional power down pin */
	sensor->pwdn_gpio = devm_gpiod_get_optional(dev, "pwdn",
						    GPIOD_OUT_HIGH);
	/* request optional reset pin */
	sensor->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);

	v4l2_i2c_subdev_init(&sensor->sd, client, &isl7998x_subdev_ops);

	sensor->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		return ret;

	mutex_init(&sensor->lock);

	ret = isl7998x_init_controls(sensor);
	if (ret)
		goto entity_cleanup;

	ret = v4l2_async_register_subdev(&sensor->sd);
	if (ret)
		goto free_ctrls;

	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
entity_cleanup:
	mutex_destroy(&sensor->lock);
	media_entity_cleanup(&sensor->sd.entity);
	return ret;
}

static int isl7998x_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct isl7998x_dev *sensor = to_isl7998x_dev(sd);

	v4l2_async_unregister_subdev(&sensor->sd);
	mutex_destroy(&sensor->lock);
	media_entity_cleanup(&sensor->sd.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);

	return 0;
}

static const struct i2c_device_id isl7998x_id[] = {
	{"isl7998x", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, isl7998x_id);

static const struct of_device_id isl7998x_dt_ids[] = {
	{ .compatible = "intersil,isl7998x" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, isl7998x_dt_ids);

static struct i2c_driver isl7998x_i2c_driver = {
	.driver = {
		.name  = "isl7998x",
		.of_match_table	= isl7998x_dt_ids,
	},
	.id_table = isl7998x_id,
	.probe    = isl7998x_probe,
	.remove   = isl7998x_remove,
};

module_i2c_driver(isl7998x_i2c_driver);

MODULE_DESCRIPTION("ISL79985/6/7 MIPI Camera Subdev Driver");
MODULE_LICENSE("GPL");

