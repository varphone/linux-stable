/********************************************************************************
 *
 *  Copyright (C) 2017 	NEXTCHIP Inc. All rights reserved.
 *  Module		: Jaguar1 Device Driver
 *  Description	: common.h
 *  Author		:
 *  Date         :
 *  Version		: Version 1.0
 *
 ********************************************************************************
 *  History      :
 *
 *
 ********************************************************************************/
#ifndef __NVP6324_H__
#define __NVP6324_H__

#include <linux/i2c.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <linux/v4l2-mediabus.h>

#define NVP6324_MAX_SENSOR_NUM 4

enum mipi_csi2_vc_pad {
	MIPI_CSI2_SENS_VC0_PAD_SOURCE = 0,
	MIPI_CSI2_SENS_VC1_PAD_SOURCE,
	MIPI_CSI2_SENS_VC2_PAD_SOURCE,
	MIPI_CSI2_SENS_VC3_PAD_SOURCE,
	MIPI_CSI2_SENS_VCX_PADS_NUM
};

struct reg_base {
	u8 bank;
	u8 addr;
	u8 mask;
	u8 offset;
};

struct reg_pack {
	struct reg_base *pbase;
	u8 *pval;
	u32 size;
};

enum nvp6324_mode {
	nvp6324_mode_min = 0,
	nvp6324_mode_1080p = 0,
	nvp6324_mode_720p,
	nvp6324_mode_sd,
	nvp6324_mode_max = nvp6324_mode_sd,
};

enum nvp6324_fps {
	nvp6324_fps_min = 0,
	nvp6324_fps_25 = 0,
	nvp6324_fps_30,
	nvp6324_fps_50,
	nvp6324_fps_60,
	nvp6324_fps_max = nvp6324_fps_60
};

enum nvp6324_phy_mclks {
	nvp6324_mclk_min = 0,
	nvp6324_378mhz = 0,
	nvp6424_594mhz,
	nvp6324_756mhz,
	nvp6324_1242mhz,
	nvp6324_mclk_max = nvp6324_1242mhz
};

enum nvp6324_analog_input {
	nvp6324_input_min = 0,
	nvp6324_input_single = 0,
	nvp6324_input_differ,
	nvp6324_input_max = nvp6324_input_differ
};

struct nvp6324 {
	struct mutex lock;
	struct i2c_client *i2c_client;
	struct v4l2_device v4l2_dev;
	struct v4l2_subdev v4l2_sd;
	struct v4l2_mbus_framefmt format;
	struct v4l2_captureparm streamcap;
#ifdef CONFIG_MEDIA_CONTROLLER
	struct media_device mdev;
	struct media_pad pads[MIPI_CSI2_SENS_VCX_PADS_NUM];
#endif
	bool pending_change;
	u32 running;
	bool is_mipi;
	int v_channel;
	u32 sensor_num;
	int reset_gpio;
	u8 current_bank;
	enum nvp6324_phy_mclks phy_mclks;
	enum nvp6324_analog_input analog_input;
	enum nvp6324_fps current_fps;
	const struct nvp6324_mode_info *current_mode;
};

static inline struct nvp6324 *subdev_to_sensor_data(struct v4l2_subdev *sd)
{
	return container_of(sd, struct nvp6324, v4l2_sd);
}

static inline int nvp6324_read_reg(struct nvp6324 *self, u8 reg)
{
	int val;
	val = i2c_smbus_read_byte_data(self->i2c_client, reg);
	if (val < 0) {
		dev_err(&self->i2c_client->dev,
			 "%s:read reg error: reg=%2x\n", __func__, reg);
		return -1;
	}
	return val;
}

static inline int nvp6324_write_reg(struct nvp6324 *self,
				    u8 reg, u8 val)
{
	int ret;
	ret = i2c_smbus_write_byte_data(self->i2c_client, reg, val);
	if (ret < 0) {
		dev_err(&self->i2c_client->dev,
			 "%s:write reg error:reg=%2x,val=%2x\n", __func__,
			 reg, val);
		return -1;
	}
	return 0;
}

static inline int nvp6324_set_reg_bank(struct nvp6324 *self, u8 bank)
{
	int ret;
	if (self->current_bank == bank)
		return 0;
	ret = nvp6324_write_reg(self, 0xff, bank);
	if (ret < 0)
		return ret;
	self->current_bank = bank;
	return 0;
}

int nvp6324_transfer_regs(struct nvp6324 *self, const struct reg_pack *reg_pack,
			  u32 channel, u32 counts);

/* video */
int nvp6324_video_init(struct nvp6324 *self);
void nvp6324_video_exit(struct nvp6324 *self);
enum nvp6324_mode nvp6324_video_current_mode(struct nvp6324 *self);

/* video eq */
void nvp6324_video_eq_set(struct nvp6324 *self);

/* mipi csi */
void nvp6324_mipi_init(struct nvp6324 *self);
void nvp6324_mipi_fmt_set(struct nvp6324 *self);
void nvp6324_arb_init(struct nvp6324 *self);

#endif
