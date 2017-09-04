/*
 * TVP5158 - Texas Instruments TVP5158A/AM1 video decoder registers
 *
 * Copyright (c) 2005,2006 Mauro Carvalho Chehab (mchehab@infradead.org)
 * This code is placed under the terms of the GNU General Public License v2
 */

//add by xym
#ifndef __TVP5158_HEADER__
#define __TVP5158_HEADER__

#define TVP5158_STATUS_REG_1        0x00 /* Status register #1 */
#define TVP5158_STATUS_REG_2        0x01 /* Status register #2 */
#define TVP5158_COLOR_SUBCAR_PHASE_STATUS        0x02 /* Status register #2 */
/* Reserved 03h */
#define TVP5158_ROM_VER       		0x04 /* ROM version */
#define TVP5158_RAM_VER_MSB       	0x05 /* RAM version MSB */
#define TVP5158_RAM_VER_LSB       	0x06 /* RAM version LSB */
/* Reserved 07h */
#define TVP5158_CHIP_ID_MSB       	0x08 /* chip ID MSB */
#define TVP5158_CHIP_ID_LSB       	0x09 /* chip ID LSB */
/* Reserved 0Ah~0Bh */
#define TVP5158_VIDEO_STD_STATUS   	0x0C /* Video Standard Status */
#define TVP5158_VIDEO_STD_SEL  		0x0D /* Video Standard Select */
#define TVP5158_AUTOSW_MSK          0x0E /* CVBS Autoswitch mask */ 
#define TVP5158_AUTO_CONTRAST_MODE  0x0F /* Auto Contrast Mode */ 
#define TVP5158_LUMA_BRIGHT  		0x10 /* Luminance Brightness */ 
#define TVP5158_LUMA_CONTRAST  		0x11 /* Luminance Contrast */ 
#define TVP5158_BRIGHT_CONTRAST_RANGE_EXT  		0x12 /* Brightness and Contrast Range Extender*/ 
#define TVP5158_CHROMA_SATURATION 	0x13 /* Chrominance Saturation*/ 
#define TVP5158_CHROMA_HUE  		0x14 /* Chrominance Hue*/ 
/* Reserved 15h */
#define TVP5158_COLOR_KILLER  		0x16 /* Color Killer*/ 
/* Reserved 17h */
#define TVP5158_LUMA_PROC_CTL_1     0x18 /* Luminance processing control #1 */
#define TVP5158_LUMA_PROC_CTL_2     0x19 /* Luminance processing control #2 */
#define TVP5158_PWR_CTL      		0x1A /* Power Control */
#define TVP5158_CHROMA_PROC_CTL_1   0x1B /* Chrominance processing control #1 */
#define TVP5158_CHROMA_PROC_CTL_2   0x1C /* Chrominance processing control #2 */
/* Reserved 1Dh~1Fh */
#define TVP5158_AGC_GAIN_STATUS_LSB   0x20 /* AGC Gain Status 1 */
#define TVP5158_AGC_GAIN_STATUS_MSB   0x21 /* AGC Gain Status 2 */
/* Reserved 22h */
#define TVP5158_BACK_END_AGC_STATUS 0x23 /* Back-End AGC Status */
#define TVP5158_STATUS_REQUEST 		0x24 /* Status Request */
#define TVP5158_AFE_GAIN_CONTROL 	0x25 /* AFE Gain Control */
#define TVP5158_LUMA_ALC_FREEZE_UPPER_THSH 	0x26 /* Luma ALC Freeze Upper Threshold */
#define TVP5158_CHROMA_ALC_FREEZE_UPPER_THSH 	0x27 /* Chroma ALC Freeze Upper Threshold */
/* Reserved 28h */
#define TVP5158_AGC_INC_SPD 		0x29 /* AGC Increment Speed */
#define TVP5158_AGC_INC_DELAY 		0x2A /* AGC Increment Delay */
#define TVP5158_AGC_DEC_SPD 		0x2B /* AGC Decrement Speed */
#define TVP5158_AGC_DEC_DELAY 		0x2C /* AGC Decrement Delay */
#define TVP5158_AGC_WHITE_PEAK_PROC	0x2D /* AGC White Peak Processing */
#define TVP5158_BACK_END_AGC_CTL    0x2E /* Back-End AGC Control */
/* Reserved 2Fh~33h */
#define TVP5158_AFE_FINE_GAIN_LSB  	0x34 /* AFE Fine Gain LSB */
#define TVP5158_AFE_FINE_GAIN_MSB  	0x35 /* AFE Fine Gain MSB */
/* Reserved 36h~47h */
#define TVP5158_AVID_START_PIX_LSB 	0x48 /* AVID Start Pixel LSB */
#define TVP5158_AVID_START_PIX_MSB 	0x49 /* AVID Start Pixel MSB */
#define TVP5158_AVID_PIX_WIDTH_LSB 	0x4A /* AVID Pixel Width LSB */
#define TVP5158_AVID_PIX_WIDTH_MSB 	0x4B /* AVID Pixel Width MSB */
/* Reserved 4Ch~5Bh */
#define TVP5158_NR_MAX_NOISE 		0x5C /* NR Max Noise */
#define TVP5158_NR_CTL 				0x5D /* NR Control */
#define TVP5158_NOISE_FILTER_LSB 	0x5E /* NR noise filter LSB */
#define TVP5158_NOISE_FILTER_MSB	0x5F /* NR noise filter MSB */
#define TVP5158_OP_MODE_CTL			0x60 /* Operation Mode Control */
#define TVP5158_COLOR_PLL_SPD_CTL	0x61 /* Color PLL Speed Control */
/* Reserved 62h~7Bh */
#define TVP5158_SYNC_HEIGHT_LOW_THSD	0x7C /* Sync Height Low Threshold */
#define TVP5158_SYNC_HEIGHT_HIGH_THSD	0x7D /* Sync Height Low Threshold */
/* Reserved 7Eh~80h */
#define TVP5158_CLEAR_LOCK_DETECT	0x81 /* Clear Lost Lock Detect */
/* Reserved 82h~84h */
#define TVP5158_V_SYNC_FILTER_SHIFT	0x85 /* V-Sync Filter Shift */
/* Reserved 86h */
#define TVP5158_656_VER_F_BIT_CTL	0x87 /* 656 Version/F Bit Control */
#define TVP5158_F_V_BIT_DECODE_CTL	0x88 /* F- and V-Bit Decode Control */
#define TVP5158_F_V_BIT_CTL			0x89 /* F- and V-Bit Control */
/* Reserved 8Ah~8Bh */
#define TVP5158_OUT_TIMING_DELAY	0x8C /* Output Timing Delay */
/* Reserved 8Dh~8Fh */
#define TVP5158_AUTO_CONTRAST_USER_TAB_INDEX	0x8F /* Auto Contrast User Table Index */
#define TVP5158_BLUE_SCREEN_Y_CTL	0x90 /* Blue Screen Y Control */
#define TVP5158_BLUE_SCREEN_CB_CTL	0x91 /* Blue Screen Cb Control */
#define TVP5158_BLUE_SCREEN_CR_CTL	0x92 /* Blue Screen Cr Control */
#define TVP5158_BLUE_SCREEN_LSB_CTL	0x93 /* Blue Screen LSB Control */
#define TVP5158_NOISE_MEASUREMENT_LSB	0x94 /* Noise Measurement LSB */
#define TVP5158_NOISE_MEASUREMENT_MSB	0x95 /* Noise Measurement MSB */
#define TVP5158_WEAK_SIG_HIGH_THSD	0x96 /* Weak Signal High Threshold */
#define TVP5158_WEAK_SIG_LOW_THSD	0x97 /* Weak Signal Low Threshold */
/* Reserved 98h~9Dh */
#define TVP5158_NR_Y_T0				0x9E /* NR_Y_T0 */
#define TVP5158_NR_U_T0				0x9F /* NR_U_T0 */
#define TVP5158_NR_V_T0				0xA0 /* NR_V_T0 */
/* Reserved A1h */
#define TVP5158_VERT_LN_COUNT_MSB   0xA2 /* Vertical line count MSB */
#define TVP5158_VERT_LN_COUNT_LSB   0xA3 /* Vertical line count LSB */
/* Reserved A4h~A7h */
#define TVP5158_OUT_FMT_CTL_1   	0xA8 /* Output Formatter Control 1 */
#define TVP5158_OUT_FMT_CTL_2   	0xA9 /* Output Formatter Control 2 */
/* Reserved AAh~ACh */
#define TVP5158_INT_CTL   			0xAD /* Interrupt Control */
#define TVP5158_EMBED_SYNC_OFFSET_CTL_1	0xAE /* Embedded Sync Offset Control 1 */
#define TVP5158_EMBED_SYNC_OFFSET_CTL_2	0xAF /* Embedded Sync Offset Control 2 */
#define TVP5158_AVD_OUT_CTL_1	0xB0 /* AVD Output Control 1 */
#define TVP5158_AVD_OUT_CTL_2	0xB1 /* AVD Output Control 2 */
#define TVP5158_OFM_MODE_CTL	0xB2 /* OFM Mode Control */
#define TVP5158_OFM_CHL_SEL_1	0xB3 /* OFM Channel Select 1 */
#define TVP5158_OFM_CHL_SEL_2	0xB4 /* OFM Channel Select 2 */
#define TVP5158_OFM_CHL_SEL_3	0xB5 /* OFM Channel Select 3 */
#define TVP5158_OFM_SUPER_FRAME_SIZE_LSB	0xB6 /* OFM Super Frame Size LSB */
#define TVP5158_OFM_SUPER_FRAME_SIZE_MSB	0xB7 /* OFM Super Frame Size MSB */
#define TVP5158_OFM_H_BLANK_DURATION_LSB	0xB8 /* OFM H-Blank Duration LSB */
#define TVP5158_OFM_H_BLANK_DURATION_MSB	0xB9 /* OFM H-Blank Duration MSB */
#define TVP5158_OFM_MISC_OFM_CTL			0xBA /* Misc Ofm Control */
/* Reserved BBh~BFh */
#define TVP5158_AUDIO_SAMPLE_RATE_CTL		0xC0 /* Audio Sample Rate Control */
#define TVP5158_AUDIO_GAIN_CTL_1			0xC1 /* Analog Audio Gain Control 1 */
#define TVP5158_AUDIO_GAIN_CTL_2			0xC2 /* Analog Audio Gain Control 2 */
#define TVP5158_AUDIO_MODE_CTL				0xC3 /* Audio Mode Control */
#define TVP5158_AUDIO_MIXER_SEL				0xC4 /* Audio Mixer Select */
#define TVP5158_AUDIO_MUTE_CTL				0xC5 /* Audio Mute control */
#define TVP5158_AUDIO_MIXING_RATIO_CTL_1	0xC6 /* Audio Mixing Ratio Control 1 */
#define TVP5158_AUDIO_MIXING_RATIO_CTL_2	0xC7 /* Audio Mixing Ratio Control 2 */
#define TVP5158_AUDIO_CASCADE_MODE_CTL		0xC8 /* Audio Cascade Mode Control */
/* Reserved C9h~CFh */
#define TVP5158_SUPER_FRAME_EAV2SAV_DURATION_LSB		0xD0 /* Super-frame EAV2SAV duration status LSBs */
#define TVP5158_SUPER_FRAME_EAV2SAV_DURATION_MSB		0xD1 /* Super-frame EAV2SAV duration status MSBs */
#define TVP5158_SUPER_FRAME_SAV2EAV_DURATION_LSB		0xD2 /* Super-frame SAV2EAV duration status LSBs */
#define TVP5158_SUPER_FRAME_SAV2EAV_DURATION_MSB		0xD3 /* Super-frame SAV2EAV duration status MSBs */
/* Reserved D4h~DFh */
#define TVP5158_VBUS_DATA_ACCESS_NO_INC		0xE0 /* VBUS Data Access With No VBUS Address Increment */
#define TVP5158_VBUS_DATA_ACCESS_INC		0xE1 /* VBUS Data Access With VBUS Address Increment */
/* Reserved E2h~E7h */
#define TVP5158_VBUS_ADDRESS_ACCESS_1			0xE8 /* VBUS Address Access 1 */
#define TVP5158_VBUS_ADDRESS_ACCESS_2			0xE9 /* VBUS Address Access 2 */
#define TVP5158_VBUS_ADDRESS_ACCESS_3			0xEA /* VBUS Address Access 3 */
/* Reserved EBh~F1h */
#define TVP5158_INT_STATUS			        0xF2 /* Interrupt Status */
/* Reserved F3h */
#define TVP5158_INT_MASK			        0xF4 /* Interrupt Mask */
/* Reserved F5h */
#define TVP5158_INT_CLEAR			        0xF6 /* Interrupt Clear */
#define TVP5158_DECODER_WRITE_ENABLE		        0xFE /* Decoder Write Enable */
#define TVP5158_DECODER_READ_ENABLE			        0xFF /* Decoder Read Enable */

#endif
//add by xym

