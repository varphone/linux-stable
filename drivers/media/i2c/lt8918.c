/*
 * LT8918 - SDI to MIPI/CSI-2 bridge
 *
 * Copyright (C) 2018 NanJing No.55 Research Institute
 *                    Technology Development CO.,LTD
 *
 * Copyright (C) 2018 Varphone Wong <varphone@qq.com>
 *
 * All rights reserved.
 *
 */

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/camera_common.h>
#include <media/tegra-v4l2-camera.h>
#include <media/tegra_v4l2_camera.h>

#include "lt8918_mode_tbls.h"

/* clang-format off */
#define LT8918_DEFAULT_MODE         LT8918_MODE_640X480_60FPS
#define LT8918_DEFAULT_DATAFMT      MEDIA_BUS_FMT_UYVY8_1X16
#define LT8918_DEFAULT_WIDTH        640
#define LT8918_DEFAULT_HEIGHT       480
#define LT8918_DEFAULT_CLK_FREQ     24000000

#define LT8918_DEFAULT_SMC_I2C_ADDR 0x38

#define LT8918_CHIP_ID              0x00B10316

static int debug = 0;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-3)");

static int test_mode = 0;
module_param(test_mode, int, 0644);
MODULE_PARM_DESC(test_mode, "Force output test pattern if true");

struct lt8918
{
    struct camera_common_power_rail power;
    int numctrls;
    struct v4l2_ctrl_handler ctrl_handler;
    struct i2c_client* i2c_main;
    struct v4l2_subdev* subdev;
    struct v4l2_dv_timings* timing;
    struct media_pad pad;
    u32 frame_length;
    s32 group_hold_prev;
    bool group_hold_en;
    s64 last_wdr_et_val;
    struct regmap* regmap;
    struct camera_common_data* s_data;
    struct camera_common_pdata* pdata;
    /* STM32 */
    struct i2c_client* i2c_smc;
    struct regmap* regmap_smc;
    /* V4L2 Controls */
    struct v4l2_ctrl* ctrls[];
};

/* clang-format on */

static void devm_i2c_release_dummy(struct device* dev, void* res)
{
    struct i2c_adapter* adap;
    struct i2c_client* client;

    if (res)
        client = *(struct i2c_client**)res;

    if (client) {
        adap = client->adapter;
        i2c_unregister_device(client);
        i2c_put_adapter(adap);
    }
}

struct i2c_client* devm_i2c_new_dummy_with_bus(struct device* dev, int bus,
                                               u16 address)
{
    struct i2c_adapter* adap;
    struct i2c_client *ret, **dr;

    adap = i2c_get_adapter(bus);

    if (!adap)
        return ERR_PTR(-ENODEV);

    dr = devres_alloc(devm_i2c_release_dummy, sizeof(*dr), GFP_KERNEL);
    if (!dr) {
        i2c_put_adapter(adap);
        return ERR_PTR(-ENOMEM);
    }

    ret = i2c_new_dummy(adap, address);
    if (!ret) {
        i2c_put_adapter(adap);
        devres_free(dr);
    }
    else {
        *dr = ret;
        devres_add(dev, dr);
    }

    return ret;
}

static const struct regmap_config sensor_regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,
    .cache_type = REGCACHE_RBTREE,
    .use_single_rw = true,
};

static const struct regmap_config smc_regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,
    .cache_type = REGCACHE_RBTREE,
    .use_single_rw = true,
};

static int lt8918_g_volatile_ctrl(struct v4l2_ctrl* ctrl);
static int lt8918_s_ctrl(struct v4l2_ctrl* ctrl);

static const struct v4l2_ctrl_ops lt8918_ctrl_ops = {
    .g_volatile_ctrl = lt8918_g_volatile_ctrl,
    .s_ctrl = lt8918_s_ctrl,
};

static struct v4l2_ctrl_config ctrl_config_list[] = {
    /* Do not change the name field for the controls! */
    {
        .ops = &lt8918_ctrl_ops,
        .id = TEGRA_CAMERA_CID_GAIN,
        .name = "Gain",
        .type = V4L2_CTRL_TYPE_INTEGER64,
        .flags = V4L2_CTRL_FLAG_SLIDER,
        .min = 0 * FIXED_POINT_SCALING_FACTOR,
        .max = 48 * FIXED_POINT_SCALING_FACTOR,
        .def = 0 * FIXED_POINT_SCALING_FACTOR,
        .step = 3 * FIXED_POINT_SCALING_FACTOR / 10, /* 0.3 db */
    },
    {
        .ops = &lt8918_ctrl_ops,
        .id = TEGRA_CAMERA_CID_EXPOSURE,
        .name = "Exposure",
        .type = V4L2_CTRL_TYPE_INTEGER64,
        .flags = V4L2_CTRL_FLAG_SLIDER,
        .min = 30 * FIXED_POINT_SCALING_FACTOR / 1000000,
        .max = 1000000LL * FIXED_POINT_SCALING_FACTOR / 1000000,
        .def = 30 * FIXED_POINT_SCALING_FACTOR / 1000000,
        .step = 1 * FIXED_POINT_SCALING_FACTOR / 1000000,
    },
    {
        .ops = &lt8918_ctrl_ops,
        .id = TEGRA_CAMERA_CID_FRAME_RATE,
        .name = "Frame Rate",
        .type = V4L2_CTRL_TYPE_INTEGER64,
        .flags = V4L2_CTRL_FLAG_SLIDER,
        .min = 1 * FIXED_POINT_SCALING_FACTOR,
        .max = 60 * FIXED_POINT_SCALING_FACTOR,
        .def = 60 * FIXED_POINT_SCALING_FACTOR,
        .step = 1 * FIXED_POINT_SCALING_FACTOR,
    },
    {
        .ops = &lt8918_ctrl_ops,
        .id = TEGRA_CAMERA_CID_GROUP_HOLD,
        .name = "Group Hold",
        .type = V4L2_CTRL_TYPE_INTEGER_MENU,
        .min = 0,
        .max = ARRAY_SIZE(switch_ctrl_qmenu) - 1,
        .menu_skip_mask = 0,
        .def = 0,
        .qmenu_int = switch_ctrl_qmenu,
    },
    {
        .ops = &lt8918_ctrl_ops,
        .id = TEGRA_CAMERA_CID_HDR_EN,
        .name = "HDR enable",
        .type = V4L2_CTRL_TYPE_INTEGER_MENU,
        .min = 0,
        .max = ARRAY_SIZE(switch_ctrl_qmenu) - 1,
        .menu_skip_mask = 0,
        .def = 0,
        .qmenu_int = switch_ctrl_qmenu,
    },
    {
        .ops = &lt8918_ctrl_ops,
        .id = TEGRA_CAMERA_CID_OTP_DATA,
        .name = "OTP Data",
        .type = V4L2_CTRL_TYPE_STRING,
        .flags = V4L2_CTRL_FLAG_READ_ONLY,
        .min = 0,
        .max = 0,
        .step = 2,
    },
    {
        .ops = &lt8918_ctrl_ops,
        .id = TEGRA_CAMERA_CID_FUSE_ID,
        .name = "Fuse ID",
        .type = V4L2_CTRL_TYPE_STRING,
        .flags = V4L2_CTRL_FLAG_READ_ONLY,
        .min = 0,
        .max = 0,
        .step = 2,
    },
    {
        .ops = &lt8918_ctrl_ops,
        .id = TEGRA_CAMERA_CID_SENSOR_MODE_ID,
        .name = "Sensor Mode",
        .type = V4L2_CTRL_TYPE_INTEGER64,
        .flags = V4L2_CTRL_FLAG_SLIDER,
        .min = 0,
        .max = 0xFF,
        .def = 0xFE,
        .step = 1,
    },
};

static int smc_init_regmap(struct lt8918* priv)
{
    priv->i2c_smc = devm_i2c_new_dummy_with_bus(&priv->i2c_main->dev, 0,
                                                LT8918_DEFAULT_SMC_I2C_ADDR);

    if (IS_ERR(priv->i2c_smc)) {
        dev_err(&priv->i2c_main->dev, "Cannot allocate I2C client for SMC!\n");
        return PTR_ERR(priv->i2c_smc);
    }

    priv->regmap_smc = devm_regmap_init_i2c(priv->i2c_smc, &smc_regmap_config);
    if (IS_ERR(priv->regmap_smc)) {
        dev_err(&priv->i2c_main->dev, "Cannot map I2C registers for SMC!\n");
        return PTR_ERR(priv->regmap_smc);
    }

    i2c_set_clientdata(priv->i2c_smc, priv);

    return 0;
}

static void smc_reset_camera(struct regmap* regmap_smc)
{
    // RSTN_H
    regmap_write(regmap_smc, 0x00, 0x55);
    msleep_range(10);
    // RSTN_L (reset)
    regmap_write(regmap_smc, 0x00, 0xAA);
    msleep_range(100);
    // RSTN_H
    regmap_write(regmap_smc, 0x00, 0x55);
    msleep_range(10);
}

static void smc_shutdown_camera(struct regmap* regmap_smc)
{
    // RSTN_L (reset)
    regmap_write(regmap_smc, 0x00, 0xAA);
    msleep_range(10);
}

static void regmap_sequence_dump(struct reg_sequence* regs, int num_regs)
{
    int i = 0;

    printk(KERN_DEBUG "Register sequence @ %p, %d:\n", regs, num_regs);
    for (; i < num_regs; i++)
        printk(KERN_DEBUG "{%04X, %04X, %d}\n", regs[i].reg, regs[i].def,
               regs[i].delay_us);
    printk(KERN_DEBUG "\n");
}

static inline int lt8918_read_reg(struct camera_common_data* s_data, u16 addr,
                                  u8* val)
{
    struct lt8918* priv = (struct lt8918*)s_data->priv;
    int err = 0;
    u32 reg_val = 0;

    err = regmap_read(priv->regmap, addr, &reg_val);
    *val = reg_val & 0xFF;

    return err;
}

static int lt8918_write_reg(struct camera_common_data* s_data, u16 addr, u8 val)
{
    int err;
    struct lt8918* priv = (struct lt8918*)s_data->priv;

    err = regmap_write(priv->regmap, addr, val);
    if (err)
        pr_err("%s:i2c write failed, 0x%x = %x\n", __func__, addr, val);

    return err;
}

static int lt8918_write_table(struct lt8918* priv,
                              const struct lt8918_reg_table* table)
{
    regmap_sequence_dump(table->regs, table->num_regs);
    return regmap_multi_reg_write(priv->regmap, table->regs, table->num_regs);
}

static inline void lt8918_write_bank(struct lt8918* priv, u8 bank)
{
    regmap_write(priv->regmap, 0xff, bank);
}

static int lt8918_hardware_init(struct regmap* regmap)
{
    regmap_write(regmap, 0xff, 0x60);
    regmap_write(regmap, 0xee, 0x01);

    msleep_range(100);

    return 0;
}

static unsigned int lt8918_get_chip_id(struct regmap* regmap)
{
    u32 ids[4];

    regmap_write(regmap, 0xff, 0x60);
    regmap_read(regmap, 0x00, &ids[0]);
    regmap_read(regmap, 0x01, &ids[1]);
    regmap_read(regmap, 0x02, &ids[2]);
    ids[3] = 0;

    return (ids[3] << 24) | (ids[2] << 16) | (ids[1] << 8) | (ids[0] & 0xff);
}

static void lt8918_log_status(struct lt8918* priv)
{
    struct device* dev = &priv->i2c_main->dev;
    struct regmap* regmap = priv->regmap;
    u8 vals[64] __maybe_unused;
    u16 sval;
    u32 ival;
    u32 ivals[4];

    // Switch to bank 0x80
    regmap_write(regmap, 0xFF, 0x80);

    regmap_read(regmap, 0x52, &ivals[0]);
    regmap_read(regmap, 0x53, &ivals[1]);
    sval = (ivals[0] << 8) | ivals[1];
    dev_dbg(dev, "HACT = %u (0x%02x%02x)\n", sval, ivals[0], ivals[1]);

    regmap_read(regmap, 0x4A, &ivals[0]);
    regmap_read(regmap, 0x4B, &ivals[1]);
    sval = ((ivals[0] & 0x0F) << 8) | ivals[1];
    dev_dbg(dev, "HFP = %u (0x%02x%02x)\n", sval, ivals[0], ivals[1]);

    regmap_read(regmap, 0x44, &ivals[0]);
    regmap_read(regmap, 0x45, &ivals[1]);
    sval = ((ivals[0] & 0x0F) << 8) | ivals[1];
    dev_dbg(dev, "HSA = %u (0x%02x%02x)\n", sval, ivals[0], ivals[1]);

    regmap_read(regmap, 0x48, &ivals[0]);
    regmap_read(regmap, 0x49, &ivals[1]);
    sval = ((ivals[0] & 0x0F) << 8) | ivals[1];
    dev_dbg(dev, "HBP = %u (0x%02x%02x)\n", sval, ivals[0], ivals[1]);

    regmap_read(regmap, 0x4E, &ivals[0]);
    regmap_read(regmap, 0x4F, &ivals[1]);
    sval = (ivals[0] << 8) | ivals[1];
    dev_dbg(dev, "HTATOL = %u (0x%02x%02x)\n", sval, ivals[0], ivals[1]);

    regmap_read(regmap, 0x50, &ivals[0]);
    regmap_read(regmap, 0x51, &ivals[1]);
    sval = ((ivals[0] & 0x0F) << 8) | ivals[1];
    dev_dbg(dev, "VACT = %u (0x%02x%02x)\n", sval, ivals[0], ivals[1]);

    regmap_read(regmap, 0x47, &ivals[0]);
    dev_dbg(dev, "VFP = %u\n", ivals[0]);

    regmap_read(regmap, 0x43, &ivals[0]);
    dev_dbg(dev, "VSA = %u\n", ivals[0]);

    regmap_read(regmap, 0x46, &ivals[0]);
    dev_dbg(dev, "VBP = %u\n", ivals[0]);

    regmap_read(regmap, 0x4C, &ivals[0]);
    regmap_read(regmap, 0x4D, &ivals[1]);
    sval = ((ivals[0] & 0x0F) << 8) | ivals[1];
    dev_dbg(dev, "VTATOL = %u (0x%02x%02x)\n", sval, ivals[0], ivals[1]);

    // Switch to bank 0x80
    regmap_write(regmap, 0xFF, 0x83);

    // Select byteclk to read
    regmap_write(regmap, 0x33, 0x08);
    usleep_range(48000, 52000);
    regmap_read(regmap, 0x30, &ivals[0]);
    regmap_read(regmap, 0x31, &ivals[1]);
    regmap_read(regmap, 0x32, &ivals[2]);
    ival = ((ivals[0] & 0x0F) << 16) | (ivals[1] << 8) | ivals[2];
    dev_dbg(dev, "BYTECLK = %ukHz\n", ival);

    // Select pixclk to read
    regmap_write(regmap, 0x33, 0x0C);
    usleep_range(48000, 52000);
    regmap_read(regmap, 0x30, &ivals[0]);
    regmap_read(regmap, 0x31, &ivals[1]);
    regmap_read(regmap, 0x32, &ivals[2]);
    ival = ((ivals[0] & 0x0F) << 16) | (ivals[1] << 8) | ivals[2];
    dev_dbg(dev, "PIXCLK = %ukHz\n", ival);
}

static int lt8918_power_on(struct camera_common_data* s_data)
{
    int err = 0;
    struct lt8918* priv = (struct lt8918*)s_data->priv;
    struct camera_common_power_rail* pw = &priv->power;

    dev_info(&priv->i2c_main->dev, "Power On\n");

    if (priv->pdata && priv->pdata->power_on) {
        err = priv->pdata->power_on(pw);
        if (err)
            dev_err(&priv->i2c_main->dev,
                    "Platform power_on() failed, error: 0x%08X\n", err);
        else
            pw->state = SWITCH_ON;
        return err;
    }

    smc_reset_camera(priv->regmap_smc);
    lt8918_hardware_init(priv->regmap);

    pw->state = SWITCH_ON;

    return 0;
}

static int lt8918_power_off(struct camera_common_data* s_data)
{
    int err = 0;
    struct lt8918* priv = (struct lt8918*)s_data->priv;
    struct camera_common_power_rail* pw = &priv->power;

    dev_info(&priv->i2c_main->dev, "Power Off\n");

    if (priv->pdata && priv->pdata->power_off) {
        err = priv->pdata->power_off(pw);
        if (!err)
            goto power_off_done;
        else
            dev_err(&priv->i2c_main->dev,
                    "Platform power_off() failed, error: 0x%08X\n", err);
        return err;
    }

    smc_shutdown_camera(priv->regmap_smc);

power_off_done:
    pw->state = SWITCH_OFF;

    return 0;
}

static int lt8918_power_get(struct lt8918* priv)
{
    struct camera_common_power_rail* pw = &priv->power;
    struct camera_common_pdata* pdata = priv->pdata;
    const char* mclk_name;
    struct clk* parent;
    int err = 0;

    mclk_name = priv->pdata->mclk_name ? priv->pdata->mclk_name : "extperiph1";
    pw->mclk = devm_clk_get(&priv->i2c_main->dev, mclk_name);
    if (IS_ERR(pw->mclk)) {
        dev_err(&priv->i2c_main->dev, "unable to get clock %s\n", mclk_name);
        return PTR_ERR(pw->mclk);
    }

    parent = devm_clk_get(&priv->i2c_main->dev, "pllp_grtba");
    if (IS_ERR(parent))
        dev_err(&priv->i2c_main->dev, "devm_clk_get failed for pllp_grtba");
    else
        clk_set_parent(pw->mclk, parent);

    pw->reset_gpio = pdata->reset_gpio;

    pw->state = SWITCH_OFF;

    return err;
}

static int lt8918_set_frame_rate(struct lt8918* priv, s64 val);

static int lt8918_s_stream(struct v4l2_subdev* sd, int enable)
{
    struct i2c_client* client = v4l2_get_subdevdata(sd);
    struct camera_common_data* s_data = to_camera_common_data(&client->dev);
    struct lt8918* priv = (struct lt8918*)s_data->priv;
    struct v4l2_ext_controls ctrls;
    struct v4l2_ext_control control[3];
    int err;

    if (!enable) {
        err = lt8918_write_table(priv,
                                 &lt8918_mode_tables[LT8918_MODE_STOP_STREAM]);

        if (err)
            return err;

        dev_info(sd->dev, "Stream Stopped!\n");

        return 0;
    }

    dev_info(sd->dev, "Start Stream: mode=%u\n", s_data->mode);

    err = lt8918_write_table(priv, &lt8918_mode_tables[s_data->mode]);
    if (err)
        goto exit;

    if (s_data->override_enable) {
        /* write list of override regs for the asking gain, */
        /* frame rate and exposure time    */
        memset(&ctrls, 0, sizeof(ctrls));
        ctrls.ctrl_class = V4L2_CTRL_ID2CLASS(TEGRA_CAMERA_CID_FRAME_RATE);
        ctrls.count = 1;
        ctrls.controls = control;

        control[0].id = TEGRA_CAMERA_CID_FRAME_RATE;

        err = v4l2_g_ext_ctrls(&priv->ctrl_handler, &ctrls);
        if (err == 0) {
            err |= lt8918_set_frame_rate(priv, control[0].value64);
            if (err)
                dev_err(&client->dev, "%s: error frame rate override\n",
                        __func__);
        }
        else {
            dev_err(&client->dev, "%s: faile to get overrides\n", __func__);
        }
    }

    lt8918_log_status(priv);

    return 0;

exit:
    dev_err(sd->dev, "%s: error setting stream\n", __func__);
    return err;
}

static int lt8918_g_input_status(struct v4l2_subdev* sd, u32* status)
{
    struct i2c_client* client = v4l2_get_subdevdata(sd);
    struct camera_common_data* s_data = to_camera_common_data(&client->dev);
    struct lt8918* priv = (struct lt8918*)s_data->priv;
    struct camera_common_power_rail* pw = &priv->power;

    *status = pw->state == SWITCH_ON;

    return 0;
}

static struct v4l2_subdev_video_ops lt8918_subdev_video_ops = {
    .s_stream = lt8918_s_stream,
    .g_mbus_config = camera_common_g_mbus_config,
    .g_input_status = lt8918_g_input_status,
};

static struct v4l2_subdev_core_ops lt8918_subdev_core_ops = {
    .s_power = camera_common_s_power,
};

static int lt8918_get_fmt(struct v4l2_subdev* sd,
                          struct v4l2_subdev_pad_config* cfg,
                          struct v4l2_subdev_format* format)
{
    return camera_common_g_fmt(sd, &format->format);
}

static int lt8918_set_fmt(struct v4l2_subdev* sd,
                          struct v4l2_subdev_pad_config* cfg,
                          struct v4l2_subdev_format* format)
{
    int ret;
    struct v4l2_mbus_framefmt* mfmt = &format->format;

    switch (mfmt->code) {
    case MEDIA_BUS_FMT_UYVY8_1X16:
        break;
    default:
        dev_err(sd->dev, "Unsupported Media Bus Format: 0x%08X\n", mfmt->code);
        return -EINVAL;
    }

    if (format->which == V4L2_SUBDEV_FORMAT_TRY)
        ret = camera_common_try_fmt(sd, &format->format);
    else
        ret = camera_common_s_fmt(sd, &format->format);

    return ret;
}

static struct v4l2_subdev_pad_ops lt8918_subdev_pad_ops = {
    .set_fmt = lt8918_set_fmt,
    .get_fmt = lt8918_get_fmt,
    .enum_mbus_code = camera_common_enum_mbus_code,
    .enum_frame_size = camera_common_enum_framesizes,
    .enum_frame_interval = camera_common_enum_frameintervals,
};

static struct v4l2_subdev_ops lt8918_subdev_ops = {
    .core = &lt8918_subdev_core_ops,
    .video = &lt8918_subdev_video_ops,
    .pad = &lt8918_subdev_pad_ops,
};

const struct of_device_id lt8918_of_match[] = {
    {
        .compatible = "lontium,lt8918",
    },
    {},
};

MODULE_DEVICE_TABLE(of, lt8918_of_match);

static struct camera_common_sensor_ops lt8918_common_ops = {
    .power_on = lt8918_power_on,
    .power_off = lt8918_power_off,
    .write_reg = lt8918_write_reg,
    .read_reg = lt8918_read_reg,
};

static int lt8918_set_frame_rate(struct lt8918* priv, s64 val)
{
    struct camera_common_data* s_data = priv->s_data;
    const struct sensor_mode_properties* mode =
        &s_data->sensor_props.sensor_modes[s_data->mode];

    (void)mode;

    return 0;
}

static int lt8918_g_volatile_ctrl(struct v4l2_ctrl* ctrl)
{
    struct lt8918* priv =
        container_of(ctrl->handler, struct lt8918, ctrl_handler);
    int err = 0;

    if (priv->power.state == SWITCH_OFF)
        return 0;

    switch (ctrl->id) {

    default:
        pr_err("%s: unknown ctrl id: %d.\n", __func__, ctrl->id);
        return -EINVAL;
    }

    return err;
}

static int lt8918_s_ctrl(struct v4l2_ctrl* ctrl)
{
    struct lt8918* priv =
        container_of(ctrl->handler, struct lt8918, ctrl_handler);
    struct camera_common_data* s_data = priv->s_data;
    int err = 0;

    if (priv->power.state == SWITCH_OFF)
        return 0;

    switch (ctrl->id) {
    case TEGRA_CAMERA_CID_FRAME_RATE:
        err = lt8918_set_frame_rate(priv, *ctrl->p_new.p_s64);
        break;
    case TEGRA_CAMERA_CID_SENSOR_MODE_ID:
        s_data->sensor_mode_id = (int)(*ctrl->p_new.p_s64);
        break;
    default:
        pr_err("%s: unknown ctrl id: %d.\n", __func__, ctrl->id);
        return -EINVAL;
    }

    return err;
}

static int lt8918_ctrls_init(struct lt8918* priv)
{
    struct i2c_client* client = priv->i2c_main;
    struct v4l2_ctrl* ctrl;
    int num_ctrls;
    int err;
    int i;

    dev_dbg(&client->dev, "%s++\n", __func__);

    num_ctrls = ARRAY_SIZE(ctrl_config_list);
    v4l2_ctrl_handler_init(&priv->ctrl_handler, num_ctrls);

    for (i = 0; i < num_ctrls; i++) {
        ctrl = v4l2_ctrl_new_custom(&priv->ctrl_handler, &ctrl_config_list[i],
                                    NULL);
        if (ctrl == NULL) {
            dev_err(&client->dev, "Failed to init %s ctrl\n",
                    ctrl_config_list[i].name);
            continue;
        }

        if (ctrl_config_list[i].type == V4L2_CTRL_TYPE_STRING &&
            ctrl_config_list[i].flags & V4L2_CTRL_FLAG_READ_ONLY) {
            ctrl->p_new.p_char = devm_kzalloc(
                &client->dev, ctrl_config_list[i].max + 1, GFP_KERNEL);
        }
        priv->ctrls[i] = ctrl;
    }

    priv->numctrls = num_ctrls;
    priv->subdev->ctrl_handler = &priv->ctrl_handler;
    if (priv->ctrl_handler.error) {
        dev_err(&client->dev, "Error %d adding controls\n",
                priv->ctrl_handler.error);
        err = priv->ctrl_handler.error;
        goto error;
    }

    err = v4l2_ctrl_handler_setup(&priv->ctrl_handler);
    if (err) {
        dev_err(&client->dev, "Error %d setting default controls\n", err);
        goto error;
    }

    return 0;

error:
    v4l2_ctrl_handler_free(&priv->ctrl_handler);
    return err;
}

static struct camera_common_pdata*
lt8918_parse_dt(struct i2c_client* client, struct camera_common_data* s_data)
{
    struct device_node* np = client->dev.of_node;
    struct camera_common_pdata* board_priv_pdata;
    const struct of_device_id* match;
    int err;
    const char* str;

    if (!np)
        return NULL;

    match = of_match_device(lt8918_of_match, &client->dev);
    if (!match) {
        dev_err(&client->dev, "Failed to find matching dt id\n");
        return NULL;
    }

    err = of_property_read_string(np, "use_sensor_mode_id", &str);
    if (!err) {
        if (!strcmp(str, "true"))
            s_data->use_sensor_mode_id = true;
        else
            s_data->use_sensor_mode_id = false;
    }
    board_priv_pdata =
        devm_kzalloc(&client->dev, sizeof(*board_priv_pdata), GFP_KERNEL);

    err = of_property_read_string(np, "mclk", &board_priv_pdata->mclk_name);
    if (err)
        dev_err(&client->dev, "mclk not in DT\n");

    board_priv_pdata->reset_gpio = of_get_named_gpio(np, "reset-gpios", 0);
    if (err) {
        dev_err(&client->dev, "reset-gpios not found %d\n", err);
        board_priv_pdata->reset_gpio = 0;
    }

    return board_priv_pdata;
}

static int lt8918_open(struct v4l2_subdev* sd, struct v4l2_subdev_fh* fh)
{
    struct i2c_client* client = v4l2_get_subdevdata(sd);

    dev_dbg(&client->dev, "%s:\n", __func__);

    return 0;
}

static const struct v4l2_subdev_internal_ops lt8918_subdev_internal_ops = {
    .open = lt8918_open,
};

static const struct media_entity_operations lt8918_media_ops = {
    .link_validate = v4l2_subdev_link_validate,
};

static int lt8918_probe(struct i2c_client* client,
                        const struct i2c_device_id* id)
{
    struct camera_common_data* common_data;
    struct lt8918* priv;
    u32 priv_size;
    int ret;

    dev_info(&client->dev, "[LT8918]: probing v4l2 sensor at addr 0x%0x.\n",
             client->addr);

    if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
        return -EINVAL;

    common_data = devm_kzalloc(&client->dev, sizeof(struct camera_common_data),
                               GFP_KERNEL);

    priv_size = sizeof(struct lt8918) +
                sizeof(struct v4l2_ctrl*) * ARRAY_SIZE(ctrl_config_list);
    priv = devm_kzalloc(&client->dev, priv_size, GFP_KERNEL);
    if (!priv) {
        dev_err(&client->dev, "Unable to allocate memory!\n");
        return -ENOMEM;
    }

    priv->i2c_main = client;

    ret = smc_init_regmap(priv);
    if (ret) {
        dev_err(&client->dev, "Unable to allocate i2c client for SMC!\n");
        return ret;
    }

    priv->regmap = devm_regmap_init_i2c(client, &sensor_regmap_config);
    if (IS_ERR(priv->regmap)) {
        dev_err(&client->dev, "Regmap init failed: %ld\n",
                PTR_ERR(priv->regmap));
        return -ENODEV;
    }

    smc_reset_camera(priv->regmap_smc);
    lt8918_hardware_init(priv->regmap);

    ret = lt8918_get_chip_id(priv->regmap);
    if (ret != LT8918_CHIP_ID) {
        dev_err(&client->dev, "Chip Id not match (0x%08X ?= 0x%08X)\n", ret,
                LT8918_CHIP_ID);
        return -ENODEV;
    }

    if (client->dev.of_node)
        priv->pdata = lt8918_parse_dt(client, common_data);
    if (!priv->pdata) {
        dev_err(&client->dev, "Unable to get platform data\n");
        return -EFAULT;
    }

    common_data->ops = &lt8918_common_ops;
    common_data->ctrl_handler = &priv->ctrl_handler;
    common_data->dev = &client->dev;
    common_data->frmfmt = &lt8918_frmfmt[0];
    common_data->colorfmt = camera_common_find_datafmt(LT8918_DEFAULT_DATAFMT);
    common_data->power = &priv->power;
    common_data->ctrls = priv->ctrls;
    common_data->priv = (void*)priv;
    common_data->numctrls = ARRAY_SIZE(ctrl_config_list);
    common_data->numfmts = ARRAY_SIZE(lt8918_frmfmt);
    common_data->def_mode = LT8918_DEFAULT_MODE;
    common_data->def_width = LT8918_DEFAULT_WIDTH;
    common_data->def_height = LT8918_DEFAULT_HEIGHT;
    common_data->fmt_width = common_data->def_width;
    common_data->fmt_height = common_data->def_height;
    common_data->def_clk_freq = LT8918_DEFAULT_CLK_FREQ;

    priv->s_data = common_data;
    priv->subdev = &common_data->subdev;
    priv->subdev->dev = &client->dev;
    priv->s_data->dev = &client->dev;

    ret = lt8918_power_get(priv);
    if (ret)
        return ret;

    ret = camera_common_initialize(common_data, "lt8918");
    if (ret) {
        dev_err(&client->dev, "Failed to initialize lt8918.\n");
        return ret;
    }

    v4l2_i2c_subdev_init(priv->subdev, client, &lt8918_subdev_ops);

    ret = lt8918_ctrls_init(priv);
    if (ret)
        return ret;

    priv->subdev->internal_ops = &lt8918_subdev_internal_ops;
    priv->subdev->flags |=
        V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;

#if defined(CONFIG_MEDIA_CONTROLLER)
    priv->pad.flags = MEDIA_PAD_FL_SOURCE;
    priv->subdev->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
    priv->subdev->entity.ops = &lt8918_media_ops;
    ret = media_entity_init(&priv->subdev->entity, 1, &priv->pad, 0);
    if (ret < 0) {
        dev_err(&client->dev, "Unable to init media entity\n");
        return ret;
    }
#endif

    ret = v4l2_async_register_subdev(priv->subdev);
    if (ret)
        return ret;

    dev_info(&client->dev, "Detected LT8918 sensor\n");

    return 0;
}

static int lt8918_remove(struct i2c_client* client)
{
    struct camera_common_data* s_data = to_camera_common_data(&client->dev);
    struct lt8918* priv = (struct lt8918*)s_data->priv;

    v4l2_async_unregister_subdev(priv->subdev);

#if defined(CONFIG_MEDIA_CONTROLLER)
    media_entity_cleanup(&priv->subdev->entity);
#endif

    v4l2_ctrl_handler_free(&priv->ctrl_handler);
    camera_common_cleanup(s_data);

    return 0;
}

/* clang-format off */
static struct i2c_device_id lt8918_id[] = {
    {"lt8918", 0},
    {}
};

MODULE_DEVICE_TABLE(i2c, lt8918_id);

static struct i2c_driver lt8918_driver = {
    .driver = {
        .name = "lt8918",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(lt8918_of_match),
    },
    .probe = lt8918_probe,
    .remove = lt8918_remove,
    .id_table = lt8918_id,
};

module_i2c_driver(lt8918_driver);
/* clang-format on */

MODULE_DESCRIPTION("Driver for LT8918 SDI to CSI-2 Bridge");
MODULE_AUTHOR("Varphone Wong <varphone@qq.com>");
MODULE_LICENSE("GPL");
