#include <asm-generic/errno-base.h>
#include <linux/media-bus-format.h>
#include <media/v4l2-subdev.h>

#include "nvp6324.h"

/* common */
static struct reg_base nvp6324_regs_base_common[] = {
	{0x05, 0x32, 0xff, 0x0}, // NOVIDEO_DET_A
	{0x05, 0xb9, 0xff, 0x0}, // HAFC_LPF_SEL
	{0x09, 0x44, 0xff, 0x0}, // FSC_EXT_EN
	{0x05, 0x00, 0xff, 0x0}, // A_CMP_PW_MODE
	{0x05, 0x02, 0xff, 0x0}, // A_CMP_TIMEUNIT
	{0x05, 0x1e, 0xff, 0x0}, // VAFEMD
	{0x05, 0x58, 0xff, 0x0}, // VAFE1_EQ_BAND_SEL
	{0x05, 0x59, 0xff, 0x0}, // LPF_BYPASS
	{0x05, 0x5a, 0xff, 0x0}, // VAFE_IMP_CNT
	{0x05, 0x5b, 0xff, 0x0}, // VAFE_DUTY
	{0x05, 0x5c, 0xff, 0x0}, // VAFE_B_LPF_SEL
	{0x05, 0x94, 0xff, 0x0}, // PWM_DELAY_H
	{0x05, 0x95, 0xff, 0x0}, // PWM_DELAY_L
	{0x05, 0x65, 0xff, 0x0}, // VAFE_CML_SPEED

	{0x09, 0x44, 0xff, 0x0}, // FSC_EXT_EN
	{0x09, 0x50, 0xff, 0x0}, // FSC_EXT_VAL_7_0
	{0x09, 0x51, 0xff, 0x0}, // FSC_EXT_VAL_15_8
	{0x09, 0x52, 0xff, 0x0}, // FSC_EXT_VAL_23_16
	{0x09, 0x53, 0xff, 0x0}, // FSC_EXT_VAL_31_24
};

static u8 val_common[ARRAY_SIZE(nvp6324_regs_base_common)] = {
	0x10, // NOVIDEO_DET_A
	0xb2, // HAFC_LPF_SEL
	0x00, // FSC_EXT_EN
	0xd0, // A_CMP_PW_MODE
	0x0c, // A_CMP_TIMEUNIT
	0x00, // VAFEMD
	0x00, // VAFE1_EQ_BAND_SEL
	0x00, // LPF_BYPASS
	0x00, // VAFE_IMP_CNT
	0x41, // VAFE_DUTY
	0x78, // VAFE_B_LPF_SEL
	0x00, // PWM_DELAY_H
	0x00, // PWM_DELAY_L
	0x80, // VAFE_CML_SPEED

	0x00, // FSC_EXT_EN
	0x30, // FSC_EXT_VAL_7_0
	0x6f, // FSC_EXT_VAL_15_8
	0x67, // FSC_EXT_VAL_23_16
	0x48, // FSC_EXT_VAL_31_24
};

/* video mode */
static struct reg_base nvp6324_regs_base_mode[] = {
	// decoder_color_fmtdef
	{0x00, 0x20, 0xff, 0x0}, // brightnees
	{0x00, 0x24, 0xff, 0x0}, // contrast
	{0x00, 0x28, 0xff, 0x0}, // black_level
	{0x00, 0x40, 0xff, 0x0}, // hue
	{0x00, 0x44, 0xff, 0x0}, // u_gain
	{0x00, 0x48, 0xff, 0x0}, // v_gain
	{0x00, 0x4c, 0xff, 0x0}, // u_offset
	{0x00, 0x50, 0xff, 0x0}, // v_offset
	{0x00, 0x58, 0xff, 0x0}, // saturation_a
	{0x05, 0x2b, 0xff, 0x0}, // saturation_b
	{0x05, 0x24, 0xff, 0x0}, // burst_dec_a
	{0x05, 0x5f, 0xff, 0x0}, // burst_dec_b
	{0x05, 0xd1, 0xff, 0x0}, // burst_dec_c
	{0x05, 0x26, 0xff, 0x0}, // FSC_LOCK_SENSE
	{0x05, 0xb8, 0xff, 0x0}, // HPLL_MASK_END
	{0x09, 0x40, 0xff, 0x0}, // FSC_DET_MODE
	{0x05, 0xb5, 0xff, 0x0}, // HPLL Locking

	// decoder_basic_vfmt_fmtdef
	{0x00, 0x10, 0xff, 0x0}, // video_format
	{0x00, 0x04, 0xff, 0x0}, // sd_mode
	{0x00, 0x08, 0xff, 0x0}, // ahd_mode
	{0x00, 0x0c, 0xff, 0x0}, // spl_mode
	{0x05, 0x69, 0x03, 0x0}, // sd_freq_sel

	// decoder_basic_chroma_fmtdef
	{0x00, 0x5c, 0xff, 0x0}, // pal_cm_off
	{0x05, 0x28, 0xff, 0x0}, // s_point
	{0x05, 0x25, 0xff, 0x0}, // fsc_lock_mode
	{0x05, 0x90, 0xff, 0x0}, // comb_mode

	// decoder_basic_timing_fmtdef
	{0x05, 0x47, 0xff, 0x0}, // sync_rs
	{0x00, 0x68, 0xff, 0x0}, // h_delay_lsb
	{0x00, 0x6c, 0xff, 0x0}, // h_dly_msb
	{0x05, 0x38, 0x01, 0x4}, // h_mask_on
	{0x05, 0x38, 0x0f, 0x0}, // h_mask_sel
	{0x05, 0x64, 0xff, 0x0}, // mem_rdp
	{0x00, 0x64, 0xff, 0x0}, // v_blk_end_b
	{0x00, 0x60, 0xff, 0x0}, // y_delay
	{0x00, 0x14, 0x01, 0x4}, // fld_inv
	{0x00, 0x78, 0xff, 0x0}, // v_blk_end_a

	// decoder_basic_hscaler_fmtdef
	{0x09, 0x96, 0xff, 0x0}, // h_down_scaler
	{0x09, 0x97, 0xff, 0x0}, // h_scaler_mode
	{0x09, 0x98, 0xff, 0x0}, // ref_base_lsb
	{0x09, 0x99, 0xff, 0x0}, // ref_base_msb
	{0x05, 0x53, 0x03, 0x2}, // line_mem_mode
	{0x09, 0x93, 0xff, 0x0}, // h_scaler_active

	// decoder_basic_hpll_fmtdef
	{0x05, 0x50, 0xff, 0x0}, // hpll_mask_on
	{0x05, 0xbb, 0xff, 0x0}, // hafc_byp_th_e
	{0x05, 0xb7, 0xff, 0x0}, // hafc_byp_th_s
	{0x05, 0xb8, 0xff, 0x0}, // hafc_op_md

	// clock
	{0x01, 0x84, 0xff, 0x0}, // clk_adc
	{0x01, 0x88, 0xff, 0x0}, // clk_pre
	{0x01, 0x8c, 0xff, 0x0}, // clk_post

	// Reserved
	{0x05, 0x6e, 0xff, 0x0}, // vblk_end_sel
	{0x05, 0x6f, 0xff, 0x0}, // vblk_end_ext
	{0x05, 0x01, 0xff, 0x0}, // cml_mode
	{0x05, 0x05, 0xff, 0x0}, // agc_op
	{0x05, 0x1d, 0xff, 0x0}, // g_sel
	{0x05, 0x62, 0xff, 0x0}, // sync_sel
};

static u8 val_mode_1080p25[ARRAY_SIZE(nvp6324_regs_base_mode)] = {
	// decoder_color_fmtdef
	0x00, // brightnees
	0x86, // contrast
	0x80, // black_level
	0x00, // hue
	0x00, // u_gain
	0x00, // v_gain
	0xf8, // u_offset
	0xf8, // v_offset
	0x80, // saturation_a
	0xa8, // saturation_b
	0x2a, // burst_dec_a
	0x00, // burst_dec_b
	0x30, // burst_dec_c
	0x40, // FSC_LOCK_SENSE
	0x39, // HPLL_MASK_END
	0x00, // FSC_DET_MODE
	0x80, // HPLL Locking

	0x20, // video_format
	0x00, // sd_mode
	0x03, // ahd_mode
	0x00, // spl_mode
	0x00, // sd_freq_sel

	0x82, // pal_cm_off
	0x90, // s_point
	0xdc, // fsc_lock_mode
	0x01, // comb_mode

	0xee, // sync_rs
	0x48, // h_delay_lsb
	0x00, // h_dly_msb
	0x01, // h_mask_on
	0x03, // h_mask_sel
	0x00, // mem_rdp
	0x00, // v_blk_end_b
	0x10, // y_delay
	0x00, // fld_inv
	0x80, // v_blk_end_a

	0x00, // h_down_scaler
	0x00, // h_scaler_mode
	0x00, // ref_base_lsb
	0x00, // ref_base_msb
	0x00, // line_mem_mode
	0x00, // h_scaler_active

	0xc6, // hpll_mask_on
	0x0f, // hafc_byp_th_e
	0xfc, // hafc_byp_th_s
	0x39, // hafc_op_md

	0x44, // clk_adc
	0x01, // clk_pre
	0x02, // clk_post

	0x00, // vblk_end_sel
	0x00, // vblk_end_ext
	0x2c, // cml_mode
	0x24, // agc_op
	0x0c, // g_sel
	0x20, // sync_sel
};

static u8 val_mode_720p25[ARRAY_SIZE(nvp6324_regs_base_mode)] = {
	// decoder_color_fmtdef
	0x00, // brightnees
	0x86, // contrast
	0x80, // black_level
	0x00, // hue
	0x00, // u_gain
	0x00, // v_gain
	0xf8, // u_offset
	0xf8, // v_offset
	0x80, // saturation_a
	0xa8, // saturation_b
	0x2a, // burst_dec_a
	0x00, // burst_dec_b
	0x30, // burst_dec_c
	0x40, // FSC_LOCK_SENSE
	0x39, // HPLL_MASK_END
	0x00, // FSC_DET_MODE
	0x80, // HPLL Locking

	0x20, // video_format
	0x00, // sd_mode
	0x0d, // ahd_mode
	0x00, // spl_mode
	0x00, // sd_freq_sel

	0x82, // pal_cm_off
	0x90, // s_point
	0xdc, // fsc_lock_mode
	0x01, // comb_mode

	0xee, // sync_rs
	0x80, // h_delay_lsb
	0x00, // h_dly_msb
	0x01, // h_mask_on
	0x05, // h_mask_sel
	0x00, // mem_rdp
	0x00, // v_blk_end_b
	0x10, // y_delay
	0x00, // fld_inv
	0x80, // v_blk_end_a

	0x00, // h_down_scaler
	0x00, // h_scaler_mode
	0x00, // ref_base_lsb
	0x00, // ref_base_msb
	0x00, // line_mem_mode
	0x00, // h_scaler_active

	0xc6, // hpll_mask_on
	0x0f, // hafc_byp_th_e
	0xfc, // hafc_byp_th_s
	0x39, // hafc_op_md

	0x44, // clk_adc
	0x01, // clk_pre
	0x02, // clk_post

	0x00, // vblk_end_sel
	0x00, // vblk_end_ext
	0x2c, // cml_mode
	0x24, // agc_op
	0x0c, // g_sel
	0x20, // sync_sel
};

static u8 val_mode_sdp25[ARRAY_SIZE(nvp6324_regs_base_mode)] = {
	// decoder_color_fmtdef
	0x00, // brightnees
	0x86, // contrast
	0x80, // black_level
	0x00, // hue
	0x00, // u_gain
	0x00, // v_gain
	0xf8, // u_offset
	0xf8, // v_offset
	0x80, // saturation_a
	0xa8, // saturation_b
	0x2a, // burst_dec_a
	0x00, // burst_dec_b
	0x00, // burst_dec_c
	0x40, // FSC_LOCK_SENSE
	0xb8, // HPLL_MASK_END
	0x00, // FSC_DET_MODE
	0x00, // HPLL Locking

	0xdd, // video_format
	0x0f, // sd_mode
	0x00, // ahd_mode
	0x00, // spl_mode
	0x00, // sd_freq_sel

	0x0b, // pal_cm_off
	0xd0, // s_point
	0xcc, // fsc_lock_mode
	0x0d, // comb_mode

	0xee, // sync_rs
	0x60, // h_delay_lsb
	0x00, // h_dly_msb
	0x00, // h_mask_on
	0x00, // h_mask_sel
	0x01, // mem_rdp
	0x00, // v_blk_end_b
	0x10, // y_delay
	0x00, // fld_inv
	0x00, // v_blk_end_a

	0x10, // h_down_scaler
	0x10, // h_scaler_mode
	0x00, // ref_base_lsb
	0x00, // ref_base_msb
	0x01, // line_mem_mode
	0x00, // h_scaler_active

	0xc6, // hpll_mask_on
	0x0f, // hafc_byp_th_e
	0xfc, // hafc_byp_th_s
	0xb9, // hafc_op_md

	0x44, // clk_adc
	0x01, // clk_pre
	0x02, // clk_post

	0x00, // vblk_end_sel
	0x00, // vblk_end_ext
	0x2c, // cml_mode
	0x20, // agc_op
	0x0c, // g_sel
	0x20, // sync_sel
};

static u8 val_mode_1080p30[ARRAY_SIZE(nvp6324_regs_base_mode)] = {
	// decoder_color_fmtdef
	0x00, // brightnees
	0x86, // contrast
	0x80, // black_level
	0x00, // hue
	0x00, // u_gain
	0x00, // v_gain
	0xf8, // u_offset
	0xf8, // v_offset
	0x80, // saturation_a
	0xa8, // saturation_b
	0x2a, // burst_dec_a
	0x00, // burst_dec_b
	0x30, // burst_dec_c
	0x40, // FSC_LOCK_SENSE
	0x39, // HPLL_MASK_END
	0x00, // FSC_DET_MODE
	0x80, // HPLL Locking

	0x20, // video_format
	0x00, // sd_mode
	0x02, // ahd_mode
	0x00, // spl_mode
	0x00, // sd_freq_sel

	0x82, // pal_cm_off
	0x90, // s_point
	0xdc, // fsc_lock_mode
	0x01, // comb_mode

	0xee, // sync_rs
	0x48, // h_delay_lsb
	0x00, // h_dly_msb
	0x01, // h_mask_on
	0x04, // h_mask_sel
	0x00, // mem_rdp
	0x00, // v_blk_end_b
	0x10, // y_delay
	0x00, // fld_inv
	0x80, // v_blk_end_a

	0x00, // h_down_scaler
	0x00, // h_scaler_mode
	0x00, // ref_base_lsb
	0x00, // ref_base_msb
	0x00, // line_mem_mode
	0x00, // h_scaler_active

	0xc6, // hpll_mask_on
	0x0f, // hafc_byp_th_e
	0xfc, // hafc_byp_th_s
	0x39, // hafc_op_md

	0x44, // clk_adc
	0x01, // clk_pre
	0x02, // clk_post

	0x00, // vblk_end_sel
	0x00, // vblk_end_ext
	0x2c, // cml_mode
	0x24, // agc_op
	0x0c, // g_sel
	0x20, // sync_sel
};

static u8 val_mode_720p30[ARRAY_SIZE(nvp6324_regs_base_mode)] = {
	// decoder_color_fmtdef
	0x00, // brightnees
	0x86, // contrast
	0x80, // black_level
	0x00, // hue
	0x00, // u_gain
	0x00, // v_gain
	0xf8, // u_offset
	0xf8, // v_offset
	0x80, // saturation_a
	0xa8, // saturation_b
	0x2a, // burst_dec_a
	0x00, // burst_dec_b
	0x30, // burst_dec_c
	0x40, // FSC_LOCK_SENSE
	0x39, // HPLL_MASK_END
	0x00, // FSC_DET_MODE
	0x80, // HPLL Locking

	0x20, // video_format
	0x00, // sd_mode
	0x0c, // ahd_mode
	0x00, // spl_mode
	0x00, // sd_freq_sel

	0x82, // pal_cm_off
	0x90, // s_point
	0xdc, // fsc_lock_mode
	0x01, // comb_mode

	0xee, // sync_rs
	0x80, // h_delay_lsb
	0x00, // h_dly_msb
	0x01, // h_mask_on
	0x05, // h_mask_sel
	0x00, // mem_rdp
	0x00, // v_blk_end_b
	0x10, // y_delay
	0x00, // fld_inv
	0x80, // v_blk_end_a

	0x00, // h_down_scaler
	0x00, // h_scaler_mode
	0x00, // ref_base_lsb
	0x00, // ref_base_msb
	0x00, // line_mem_mode
	0x00, // h_scaler_active

	0xc6, // hpll_mask_on
	0x0f, // hafc_byp_th_e
	0xfc, // hafc_byp_th_s
	0x39, // hafc_op_md

	0x44, // clk_adc
	0x01, // clk_pre
	0x02, // clk_post

	0x10, // vblk_end_sel
	0x1c, // vblk_end_ext
	0x2c, // cml_mode
	0x24, // agc_op
	0x0c, // g_sel
	0x20, // sync_sel
};

static u8 val_mode_sdp30[ARRAY_SIZE(nvp6324_regs_base_mode)] = {
	// decoder_color_fmtdef
	0x00, // brightnees
	0x8c, // contrast
	0x80, // black_level
	0x00, // hue
	0x10, // u_gain
	0x10, // v_gain
	0xf8, // u_offset
	0xf8, // v_offset
	0x80, // saturation_a
	0xa8, // saturation_b
	0x2a, // burst_dec_a
	0x00, // burst_dec_b
	0x00, // burst_dec_c
	0x40, // FSC_LOCK_SENSE
	0xb8, // HPLL_MASK_END
	0x00, // FSC_DET_MODE
	0x00, // HPLL Locking

	0xa0, // video_format
	0x0e, // sd_mode
	0x00, // ahd_mode
	0x00, // spl_mode
	0x00, // sd_freq_sel

	0x8b, // pal_cm_off
	0xd0, // s_point
	0xdc, // fsc_lock_mode
	0x01, // comb_mode

	0x04, // sync_rs
	0x80, // h_delay_lsb
	0x00, // h_dly_msb
	0x00, // h_mask_on
	0x00, // h_mask_sel
	0x01, // mem_rdp
	0x00, // v_blk_end_b
	0x10, // y_delay
	0x01, // fld_inv
	0x80, // v_blk_end_a

	0x10, // h_down_scaler
	0x10, // h_scaler_mode
	0x00, // ref_base_lsb
	0x00, // ref_base_msb
	0x01, // line_mem_mode
	0x00, // h_scaler_active

	0x84, // hpll_mask_on
	0x0f, // hafc_byp_th_e
	0xfc, // hafc_byp_th_s
	0xb9, // hafc_op_md

	0x44, // clk_adc
	0x01, // clk_pre
	0x02, // clk_post

	0x00, // vblk_end_sel
	0x00, // vblk_end_ext
	0x2c, // cml_mode
	0x20, // agc_op
	0x0c, // g_sel
	0x20, // sync_sel
};

struct nvp6324_mode_info {
	enum nvp6324_mode mode;
	u32 width;
	u32 height;
	struct reg_pack reg_pack;
};

static struct nvp6324_mode_info nvp6324_mode_info_data[nvp6324_fps_max + 1][nvp6324_mode_max + 1] = {
	/* 25 pal fps */
	{
		{ nvp6324_mode_1080p, 1920, 1080,
			{ nvp6324_regs_base_mode,
			  val_mode_1080p25,
			  ARRAY_SIZE(nvp6324_regs_base_mode)
			},
		},
		{ nvp6324_mode_720p, 1280, 720,
			{ nvp6324_regs_base_mode,
			  val_mode_720p25,
			  ARRAY_SIZE(nvp6324_regs_base_mode)
			},
		},
		{ nvp6324_mode_sd, 720, 480,
			{ nvp6324_regs_base_mode,
			  val_mode_sdp25,
			  ARRAY_SIZE(nvp6324_regs_base_mode)
			},
		}
	},

	/* 30 ntsc fps */
	{
		{ nvp6324_mode_1080p, 1920, 1080,
			{ nvp6324_regs_base_mode,
			  val_mode_1080p30,
			  ARRAY_SIZE(nvp6324_regs_base_mode)
			},
		},
		{ nvp6324_mode_720p, 1280, 720,
			{ nvp6324_regs_base_mode,
			  val_mode_720p30,
			  ARRAY_SIZE(nvp6324_regs_base_mode)
			},
		},
		{ nvp6324_mode_sd, 720, 480,
			{ nvp6324_regs_base_mode,
			  val_mode_sdp30,
			  ARRAY_SIZE(nvp6324_regs_base_mode)
			},
		}
	},

	/* 50 pal fps */
	{
	},

	/* 60 ntsc fps */
	{
	}
};

static int nvp6324_framerates[] = {
	[nvp6324_fps_25] = 25,
	[nvp6324_fps_30] = 30,
	[nvp6324_fps_50] = 50,
	[nvp6324_fps_60] = 60,
};

struct nvp6324_input_mode {
	enum nvp6324_analog_input mode;
	struct reg_pack reg_pack;
};

/* analog single and differential */
static struct reg_base nvp6324_regs_base_analog_in[] = {
	{0x05, 0x00, 0xff, 0x0},
	{0x05, 0x01, 0xff, 0x0},
	{0x05, 0x1d, 0xff, 0x0},
	{0x05, 0x92, 0xff, 0x0},
};

static u8 val_analog_in_diff[ARRAY_SIZE(nvp6324_regs_base_analog_in)] = {
	0xd0,
	0x2c,
	0x8c,
	0x00,
};

static u8 val_analog_in_single[ARRAY_SIZE(nvp6324_regs_base_analog_in)] = {
	0xd0,
	0xa2,
	0x0c,
	0x00,
};

static struct nvp6324_input_mode nvp6324_input_mode_data[nvp6324_input_max + 1] = {
	{
		nvp6324_input_single,
		{ nvp6324_regs_base_analog_in,
		  val_analog_in_single,
		  ARRAY_SIZE(nvp6324_regs_base_analog_in),
		},
	},

	{
		nvp6324_input_differ,
		{ nvp6324_regs_base_analog_in,
		  val_analog_in_diff,
		  ARRAY_SIZE(nvp6324_regs_base_analog_in),
		},
	}
};

static int nvp6324_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct nvp6324_dev *nvp6324 = subdev_to_sensor_data(sd);

	code->code = nvp6324->format.code;
	return 0;
}

static int nvp6324_enum_framesizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index > nvp6324_mode_max)
		return -EINVAL;

	fse->max_width = fse->min_width =
		nvp6324_mode_info_data[0][fse->index].width;
	fse->max_height = fse->min_height =
		nvp6324_mode_info_data[0][fse->index].height;

	return 0;
}

static int nvp6324_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	int i, j, count;

	if (fie->index < 0 || fie->index > nvp6324_fps_max)
		return -EINVAL;

	fie->interval.numerator = 1;

	count = 0;
	for (i = 0; i < ARRAY_SIZE(nvp6324_framerates); i++) {
		for (j = 0; j <= nvp6324_mode_max; j ++) {
			if (fie->width == nvp6324_mode_info_data[i][j].width &&
			    fie->height == nvp6324_mode_info_data[i][j].height)
				count++;

			if (fie->index == (count - 1)) {
				fie->interval.denominator = nvp6324_framerates[i];
				return 0;
			}
		}
	}

	return -EINVAL;
}

static int nvp6324_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct nvp6324_dev *nvp6324 = subdev_to_sensor_data(sd);
	struct v4l2_mbus_framefmt *mf;

	if (fmt->pad)
		return -EINVAL;

	mutex_lock(&nvp6324->lock);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		mf = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
	else
		mf = &nvp6324->format;

	fmt->format = *mf;

	mutex_unlock(&nvp6324->lock);

	return 0;
}

static const struct nvp6324_mode_info *
nvp6324_find_mode(struct nvp6324_dev *nvp6324, enum nvp6324_fps fps,
		  int width, int height, bool nearest)
{
	const struct nvp6324_mode_info *mode = NULL;
	int i;

	for (i = 0; i < nvp6324_mode_max + 1; i++) {
		mode = &nvp6324_mode_info_data[fps][i];

		if ((nearest && mode->width <= width &&
		     mode->height <= height) ||
		    (!nearest && mode->width == width &&
		     mode->height == height))
			break;
	}

	if (i > nvp6324_mode_max)
		mode = &nvp6324_mode_info_data[fps][nvp6324_mode_720p];

	return mode;
}

static int nvp6324_try_fmt_internal(struct nvp6324_dev *nvp6324,
				    struct v4l2_mbus_framefmt *fmt,
				    enum nvp6324_fps fps,
				    const struct nvp6324_mode_info **new_mode)
{
	const struct nvp6324_mode_info *mode;

	mode = nvp6324_find_mode(nvp6324, fps, fmt->width, fmt->height, true);

	if (!mode)
		return -EINVAL;

	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = nvp6324->format.code;
	fmt->colorspace = nvp6324->format.colorspace;
	fmt->field = nvp6324->format.field;

	if (new_mode)
		*new_mode = mode;

	return 0;
}

static int nvp6324_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct nvp6324_dev *nvp6324 = subdev_to_sensor_data(sd);
	const struct nvp6324_mode_info *new_mode;
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	int ret = 0;

	if (fmt->pad)
		return -EINVAL;

	mutex_lock(&nvp6324->lock);

	/* if (nvp6324->running) { */
		/* ret = -EBUSY; */
		/* goto out; */
	/* } */

	ret = nvp6324_try_fmt_internal(nvp6324, mf, nvp6324->current_fps,
				       &new_mode);

	if (ret || new_mode == nvp6324->current_mode)
		goto out;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *mfmt =
			v4l2_subdev_get_try_format(sd, cfg, 0);
		*mfmt = fmt->format;
		goto out;
	}

	nvp6324->current_mode = new_mode;
	nvp6324->format = *mf;
	nvp6324->pending_change = true;

out:
	mutex_unlock(&nvp6324->lock);
	return ret;
}

static int nvp6324_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_frame_desc *fd)
{
	return 0;
}

static int nvp6324_set_frame_desc(struct v4l2_subdev *sd,
				  unsigned int pad,
				  struct v4l2_mbus_frame_desc *fd)
{
	return 0;
}

static void nvp6324_hardware_init(struct nvp6324_dev *nvp6324);
static int nvp6324_set_power(struct v4l2_subdev *sd, int on)
{
	struct nvp6324_dev *nvp6324 = subdev_to_sensor_data(sd);
	if (on) {
		v4l2_info(sd, "Enable MIPI TX\n");
		nvp6324_hardware_init(nvp6324);
	}
	else {
		v4l2_info(sd, "Disable MIPI TX\n");
		nvp6324_mipi_disable(nvp6324);
	}
	return 0;
}

static int nvp6324_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct nvp6324_dev *nvp6324 = subdev_to_sensor_data(sd);
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	enum nvp6324_fps fps;
	enum nvp6324_mode mode = nvp6324->current_mode->mode;
	u32 tgt_fps;
	int ret = 0;

	mutex_lock(&nvp6324->lock);

	/* if (nvp6324->running) { */
		/* ret = -EBUSY; */
		/* goto out; */
	/* } */

	switch (a->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:

		if ((timeperframe->numerator == 0) ||
		    (timeperframe->denominator == 0)) {
			timeperframe->denominator = nvp6324_framerates[nvp6324_fps_25];
			timeperframe->numerator = 1;
		}

		tgt_fps = timeperframe->denominator / timeperframe->numerator;

		if (tgt_fps == nvp6324_framerates[nvp6324->current_fps])
			goto out;

		if (tgt_fps > nvp6324_framerates[nvp6324_fps_max]) {
			timeperframe->denominator = nvp6324_framerates[nvp6324_fps_max];
			timeperframe->numerator = 1;
		} else if (tgt_fps < nvp6324_framerates[nvp6324_fps_min]) {
			timeperframe->denominator = nvp6324_framerates[nvp6324_fps_min];
			timeperframe->numerator = 1;
		}

		tgt_fps = timeperframe->denominator / timeperframe->numerator;

		if (tgt_fps == 25)
			fps = nvp6324_fps_25;
		else if (tgt_fps == 30)
			fps = nvp6324_fps_30;
		else {
			ret = -EINVAL;
			v4l2_err(sd, " The camera %d frame rate is not supported!\n", tgt_fps);
			goto out;
		}
		nvp6324->streamcap.timeperframe = *timeperframe;
		nvp6324->current_fps = fps;
		nvp6324->current_mode = &nvp6324_mode_info_data[fps][mode];
		nvp6324->pending_change = true;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		v4l2_info(sd, "   type is not "\
				 "V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
			a->type);
		ret = -EINVAL;
		break;

	default:
		v4l2_info(sd, "   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

out:
	mutex_unlock(&nvp6324->lock);
	return ret;
}

static int nvp6324_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct v4l2_captureparm *cparm = &a->parm.capture;
	struct nvp6324_dev *nvp6324 = subdev_to_sensor_data(sd);
	int ret = 0;

	mutex_lock(&nvp6324->lock);

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = nvp6324->streamcap.capability;
		cparm->timeperframe = nvp6324->streamcap.timeperframe;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;
	default:
		v4l2_info(sd, "   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&nvp6324->lock);

	return ret;
}

static int nvp6324_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct nvp6324_dev *nvp6324 = subdev_to_sensor_data(sd);

	mutex_lock(&nvp6324->lock);

	if (enable) {
		if (!nvp6324->running) {
			// Enable CSI output
		}
		nvp6324->running++;
	} else {
		if (nvp6324->running > 0) {
			// Disable CSI ouput
		}
		nvp6324->running--;
	}

	mutex_unlock(&nvp6324->lock);

	return 0;
}

static int nvp6324_link_setup(struct media_entity *entity,
			      const struct media_pad *local,
			      const struct media_pad *remote, u32 flags)
{
	return 0;
}

static int nvp6324_link_validate(struct media_link *link)
{
	return 0;
}

static const struct v4l2_subdev_pad_ops nvp6324_pad_ops = {
	.enum_mbus_code         = nvp6324_enum_mbus_code,
	.enum_frame_size        = nvp6324_enum_framesizes,
	.enum_frame_interval    = nvp6324_enum_frame_interval,
	.get_fmt                = nvp6324_get_fmt,
	.set_fmt                = nvp6324_set_fmt,
	.get_frame_desc         = nvp6324_get_frame_desc,
	.set_frame_desc         = nvp6324_set_frame_desc,

};

static const struct v4l2_subdev_core_ops nvp6324_core_ops = {
	.s_power	= nvp6324_set_power,
};

static const struct v4l2_subdev_video_ops nvp6324_video_ops = {
	.s_parm		= nvp6324_s_parm,
	.g_parm		= nvp6324_g_parm,
	.s_stream	= nvp6324_s_stream,
};

static const struct v4l2_subdev_ops nvp6324_subdev_ops = {
	.core	= &nvp6324_core_ops,
	.pad	= &nvp6324_pad_ops,
	.video	= &nvp6324_video_ops,
};

static const struct media_entity_operations nvp6324_sd_media_ops = {
	.link_setup	= nvp6324_link_setup,
	.link_validate	= nvp6324_link_validate,
};

static struct reg_base nvp6324_regs_base_pattern[] = {
	{0x00, 0x18, 0xff, 0x0},
	{0x00, 0x1c, 0xff, 0x0},
	{0x05, 0x6a, 0xff, 0x0},
};

static u8 val_pattern[ARRAY_SIZE(nvp6324_regs_base_pattern)] = {
	0x13,
	0x1a,
	0x80,
};

static inline void nvp6324_pattern_enable(struct nvp6324_dev *nvp6324)
{
	struct reg_pack reg_pack;

	reg_pack.pbase = nvp6324_regs_base_pattern;
	reg_pack.pval = val_pattern;
	reg_pack.size = ARRAY_SIZE(nvp6324_regs_base_pattern);

	nvp6324_transfer_regs(nvp6324, &reg_pack, 0, 4);
}

static inline void nvp6324_manual_set(struct nvp6324_dev *nvp6324)
{
	int i;
	u8 reg;

	nvp6324_set_reg_bank(nvp6324, 0x01);
	nvp6324_write_reg(nvp6324, 0x7c, 0x0);

	/* detect */
	nvp6324_set_reg_bank(nvp6324, 0x13);
	for (i = 0; i < 4; i++) {
		reg = nvp6324_read_reg(nvp6324, 0x30);
		reg &= (~(1 << (i + 4)) & (~(1 << i)));
		nvp6324_write_reg(nvp6324, 0x30, reg);

		reg = nvp6324_read_reg(nvp6324, 0x31);
		reg &= (~(1 << (i + 4)) & (~(1 << i)));
		nvp6324_write_reg(nvp6324, 0x31, reg);

		reg = nvp6324_read_reg(nvp6324, 0x32);
		reg &= (~(1 << i));
		nvp6324_write_reg(nvp6324, 0x32, reg);
	}
}

static void nvp6324_hardware_init(struct nvp6324_dev *nvp6324)
{
	struct reg_pack reg_pack;
	enum nvp6324_analog_input input = nvp6324->analog_input;

	nvp6324_mipi_init(nvp6324);

	/* configure differ or single input */
	memcpy(&reg_pack, &nvp6324_input_mode_data[input].reg_pack, sizeof(reg_pack));
	nvp6324_transfer_regs(nvp6324, &reg_pack, 0, 4);

	nvp6324_manual_set(nvp6324);

	/* vafe fsc common set */
	reg_pack.pbase = nvp6324_regs_base_common;
	reg_pack.pval = val_common;
	reg_pack.size = ARRAY_SIZE(nvp6324_regs_base_common);
	nvp6324_transfer_regs(nvp6324, &reg_pack, 0, 4);

	memcpy(&reg_pack, &nvp6324->current_mode->reg_pack, sizeof(reg_pack));
	nvp6324_transfer_regs(nvp6324, &reg_pack, 0, 4);

	nvp6324_video_eq_set(nvp6324);

	nvp6324_mipi_fmt_set(nvp6324);
	nvp6324_arb_init(nvp6324);
	nvp6324_pattern_enable(nvp6324);
}

static void nvp6324_hardware_preinit(struct nvp6324_dev *nvp6324)
{
	int i = 0;

	//Pad Control Setting
	nvp6324_set_reg_bank(nvp6324, 0x04);
	for (i = 0; i < 36; i++) {
		nvp6324_write_reg(nvp6324, 0xa0 + i, 0x24);
	}

	// Clock Delay Setting
	nvp6324_set_reg_bank(nvp6324, 0x01);
	for (i = 0; i < 4; i++) {
		nvp6324_write_reg(nvp6324, 0xcc + i, 0x64);
	}

	// MIPI_V_REG_OFF
	nvp6324_set_reg_bank(nvp6324, 0x21);
	nvp6324_write_reg(nvp6324, 0x07, 0x80);
	nvp6324_write_reg(nvp6324, 0x07, 0x00);

	// AGC_OFF  08.31
	nvp6324_set_reg_bank(nvp6324, 0x0a);
	nvp6324_write_reg(nvp6324, 0x77, 0x8F);
	nvp6324_write_reg(nvp6324, 0xF7, 0x8F);
	nvp6324_write_reg(nvp6324, 0xff, 0x0B);
	nvp6324_write_reg(nvp6324, 0x77, 0x8F);
	nvp6324_write_reg(nvp6324, 0xF7, 0x8F);
}

inline enum nvp6324_mode nvp6324_video_current_mode(struct nvp6324_dev *nvp6324)
{
	return nvp6324->current_mode->mode;
}

int nvp6324_video_init(struct nvp6324_dev *nvp6324)
{
	struct v4l2_subdev *sd;
	int ret = 0;

	nvp6324->analog_input = nvp6324_input_single;
	nvp6324->phy_mclks = nvp6324_756mhz;

	/*****************************************
	 * Pass mipi phy clock rate Mbps
	 * fcsi2 = PCLk * WIDTH * CHANNELS / LANES
	 * fsci2 = 72MPCLK * 8 bit * 4 channels / 4 lanes
	 ****************************************/
	nvp6324->streamcap.capability = V4L2_MODE_HIGHQUALITY |
					     V4L2_CAP_TIMEPERFRAME;

	nvp6324->streamcap.timeperframe.numerator = 1;
	nvp6324->streamcap.timeperframe.denominator =
		nvp6324_framerates[nvp6324_fps_25];

	nvp6324->current_fps = nvp6324_fps_25;
	nvp6324->current_mode =
		&nvp6324_mode_info_data[nvp6324_fps_25][nvp6324_mode_720p];

	nvp6324->format.code = MEDIA_BUS_FMT_UYVY8_2X8;
	nvp6324->format.width = nvp6324->current_mode->width;
	nvp6324->format.height = nvp6324->current_mode->height;
	nvp6324->format.colorspace = V4L2_COLORSPACE_DEFAULT;
	nvp6324->format.reserved[0] = 756;
	nvp6324->format.field = V4L2_FIELD_NONE;

	nvp6324->is_mipi = true;
	nvp6324->v_channel = 0;

	nvp6324_hardware_preinit(nvp6324);

	nvp6324->pending_change = false;

	sd = &nvp6324->subdev;
	v4l2_i2c_subdev_init(sd, nvp6324->i2c_client, &nvp6324_subdev_ops);

	sd->flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	nvp6324->pads[MIPI_CSI2_SENS_VC0_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	nvp6324->pads[MIPI_CSI2_SENS_VC1_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	nvp6324->pads[MIPI_CSI2_SENS_VC2_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	nvp6324->pads[MIPI_CSI2_SENS_VC3_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, MIPI_CSI2_SENS_VCX_PADS_NUM,
				     nvp6324->pads);
	if (ret < 0)
		return ret;

	sd->entity.ops = &nvp6324_sd_media_ops;
	ret = v4l2_async_register_subdev(sd);
	if (ret < 0) {
		v4l2_err(sd, "%s--Async register failed, ret=%d\n",
			__func__, ret);
		media_entity_cleanup(&sd->entity);
	}

	nvp6324_hardware_init(nvp6324);

	return ret;
}

