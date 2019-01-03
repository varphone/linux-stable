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

#ifndef __LT8918_MODE_TBLS_H__
#define __LT8918_MODE_TBLS_H__

#include <media/camera_common.h>
#include <linux/miscdevice.h>

/* clang-format off */

struct lt8918_reg_table {
    struct reg_sequence* regs;
    unsigned int num_regs;
};

enum
{
    LT8918_MODE_640X480_60FPS,
    LT8918_MODE_720X480_30FPS,
    LT8918_MODE_720X576_25FPS,
    LT8918_MODE_1280X720_60FPS,
    LT8918_MODE_1920X1080_25FPS,
    LT8918_MODE_1920X1080_30FPS,
    LT8918_MODE_1920X1080_60FPS,
    LT8918_MODE_START_STREAM,
    LT8918_MODE_STOP_STREAM,
    LT8918_MODE_TEST_PATTERN
};

/** Registers:
 * 0x600D:
 * - RGD_MIPI_PIX_SW_RST_N[7] MIPI TX packet logic pix_clk domain soft reset.
 * - RGD_MIPI_PKT_BYTE_SW_RST_N[6] MIPI TX packet logic byte_clk domain soft reset.
 * - RGD_MIPI_DCS_SW_RST_N[5] MIPI TX dcs logic soft reset.
 * - RGD_MIPI_DPHY_SW_RST_N[4] MIPI TX DPHY logic soft reset.
 * - RGD_MIPI_DPHY_AFIFO_SW_RST_N[3] MIPI TX DPHY fifo soft reset.
 * - RGD_MIPI_RX_SW_RST_N[2] Repeater mode MIPI RX logic soft reset.
 * - [2:0] RESERVED
 * 0x7018:
 * - RG_RXPLL_PD[7] 0=Normal work, 1=Power down
 * - RG_RXPLL_CP_SEL[6] 0=Second order passive LPF PLL, 1=Adaptive BW tracking PLL
 * - RG_RXPLL_CPCUR_SEL[5] Adaptive mode: 0=Low BE, 1=High BW
 * - RG_RXPLL_LOCKEN[4] This bi controls the operation of PLL locking block
 * - RG_RXPLL_REF_SEL[3] 0=The d clock as reference, 1=LVDS byte clock as reference
 * - RG_RXPLL_DUAL_MODE_EN[2] 0=1x lvds clock as pix clock, 1=2x lvds clock as pix clock
 * - RG_RXPLL_HALF_MODE_EN[1] 0=2x lvds clock as pix clock, 1=4x lvds clock as pix clock
 * - RG_RXPLL_DDR_MODE_EN[0] 0=1x d clock as pix clock, 1=2x d clock as pix clock
 * 0x701C:
 * - [7] RESERVED
 * - RG_RXPLL_DCLK_SEL[6]
 *   - 0=The pixel clock is from PLL
 *   - 1=The pixel clock s from IDCLK
 * - RG_RXPLL_TEST_SEL[5]:
 *   - b00=Power as test signal
 *   - b01=Reference clock as test signal
 *   - b10=Feed back clock as test signal
 *   - b11=Reset signal as test signal
 * - RG_RXPLL_PIXCLK_MUXSEL[3:2]:
 *   - b00=From d clock
 *   - b01=From clock divider by 5(PLL)
 *   - b10=From clock divider by 7(PLL)
 *   - b11=From clock divider by 3(PLL)
 * - RG_RXPLL_PIXCLK_DIVSEL[1:0]
 *   - b00=/1
 *   - b01=/2
 *   - b10=/4
 *   - b11=/8
 * 0x7023
 * - RG_MLTX_SW_RST_N[7]: 0=Reset, 1=Normal work
 * - RG_MLTX_PD[6]: 0=Normal work, 1=Power down
 * - RG_MLTX_EN_SEL[5]: 0=By da_mipitx_hstx_en, 1=By rg_mipitx_hstx_en
 * - RG_MLTX_HSTXCK_EN[4]: 0=Disable, 1=Enable
 * - RG_MLTX_HSTX_EN[3:0]: bXXXX=Lane[3/2/1/0]
 * 0x7024
 * - RG_MLTX_MIPI_EN[6]: 0=Unused mode, 1=Mipi mode
 * - RG_MLTX_P2S_SEL[5]: 0=SDR, 1=DDR
 * - RG_MLTX_VREF_MODE_SEL[2]: 0=Current mirror mode, 1=Feedback mode
 * - RG_MLTX_DIFF_SWAP[1]: 0=No swap, 1=Dp/n swap
 * - RG_MLTX_DCSWAP_EN[0]: 0=No swap, 1=Swap
 * 0x7030
 * - RG_TXPLL_PD[6]: 0=Normal Work, 1=Power Down
 * - RG_TXPLL_CP_SEL[5]: 0=Second order passive, 1=Adaptive BW tracking
 * - RG_TXPLL_LPF_SEL[4]: @see RG_TXPLL_CP_SEL
 * - RG_TXPLL_VCO_SEL[3]: 0=1.8GHz/V, 1=3.5GHz/V
 * - RG_TXPLL_CP_CUR_SEL[2]: 0=Low BW, 1=High BW
 * - RG_TXPLL_LOCKEN[1]: 0=Disable, 1=Enable
 * - RG_TXPLL_FULL_RATE_EN[0] 0=Disable, 1=Enable
 * 0x7031
 * - RG_TXPLL_CPCUR_SET[7:4]: 1=6.25uA, 2=12.5uA, 4=25uA, 8=50uA
 * - RG_TXPLL_REF_SEL[3:2]: 1=Aa_pix_clk, 2=Aa_d_clk, 3=Aa_xtal_clk
 * - RG_TXPLL_PREDIV_SEL[1:0]: 0=Div1, 1=Div2, 2=Div4, 3=Div8
 * 0x7033
 * - RG_TXPLL_LOOPDIV_RATIO[6:0]: Div=N+2(N>=2)
 * 0x7034
 * - RG_TXPLL_SERICK_DIVSEL[6:4]: b1xx=Div1, b000=Div2, b001=Div4, b010=Div8, b011=Div16
 * - RG_TXPLL_POSTDIV_SEL[3:2]: Div=1<<N
 * - RG_TXPLL_ESCDIV2X_SET[1:0]: Div=1<<N
 * 0x7038 --
 * 0x8002:
 * - RGD_TTL_INV_EN[7] Input date inverse enable.
 * - RGD_TTL_RGB_SWAP[6:4]:
 *   - b000 = D[23:0] = BGR;
 *   - b011 = D[23:0] = BRG;
 *   - b100 = D[23:0] = GBR;
 *   - b101 = D[23:0] = GRB;
 *   - b110 = D[23:0] = RBG;
 *   - b111 = D[23:0] = RGB;
 * - RGD_TTL_24B_HL_SWAP_EN[3] Input 24 bit data high/low bit swap enable.
 * - RGD_TTL_8B_HL_SWAP_EN[2:0] Pre-channel(ttl process output r/g/b) high/low bit swap enable.
 * 0x8003:
 * - RGD_TTL_VS_POL_ADJ[7] TTL process output vsync polarity adjust enable:
 *   - 1=TTL process output vsync is equal to input vsync invert;
 *   - 0=TTL process output vsync is equal to input vsync;
 * - RGD_TTL_HS_POL_ADJ[6] TTL process output hsync polarity adjust enable:
 *   - 1=TTL process output hsync is equal to input hsync invert;
 *   - 0=TTL process output hsync is equal to input hsync;
 * - RGD_TTL_8B_MODE[5] TTL input 8 bit active data mode enable.
 * - RGD_DE_GEN[4] TTL output data enable from internal generated signal enable.
 * - RGD_BT_SYNC_GEN_EN[3] Embedded field change at active edge of embedded vsync.
 * - RGD_IP_SEL[2] Input format select:
 *   - 0=Progressive mode input;
 *   - 1=Interlace mode input;
 * - RGD_BT_GEN_EN[1] Embedded sync mode input enable.
 * - RGD_BT_EAV_GEN_EN[1] Embedded field change at embedded EAV flag.
 * 0x80A1 --
 * 0x8004 RGD_BT_H_ACT[15:8]
 * 0x8005 RGD_BT_H_ACT[7:0]
 * 0x8006 RGD_BT_H_FP[15:8]
 * 0x8007 RGD_BT_H_FP[7:0]
 * 0x8008 RGD_BT_H_HW[15:8]
 * 0x8009 RGD_BT_H_HW[7:0]
 * 0x800A RGD_BT_V_FP[15:8]
 * 0x800B RGD_BT_V_FP[7:0]
 * 0x800C RGD_BT_V_HW[15:8]
 * 0x800D RGD_BT_V_HW[7:0]
 * 0x800E RGD_BT_2ND_V_HSEN[15:8]
 * 0x800F RGD_BT_2ND_V_HSEN[7:0]
 * 0x8030{RO}:
 * - [7:4] RESERVED
 * - [3:0] RGOD_FREQ_IN_KHZ[19:16]
 * 0x8031{RO} RGOD_FREQ_IN_KHZ[15:8]
 * 0x8032{RO} RGOD_FREQ_IN_KHZ[7:0] Detect clock frequency indicator (unit kHz).
 * 0x8033:
 * - [7:6] RESERVED;
 * - [5] TTL input lock inverse enable;
 * - [4:0] RGD_FREQ_DET_CLK_SEL(Frequencymeter clock source select):
 *   - h00 = Ad_sys_clk;
 *   - h07 = Ad_mipitx_lprx_escclk;
 *   - h08 = Ad_mipitx_read_clk;
 *   - h09 = Ad_mipitx_write_clk;
 *   - h0A = Ad_rxpll_ref_clk;
 *   - h0B = Ad_rxpll_fb_clk;
 *   - h0C = Ad_pix_clk;
 *   - h0D = Ad_d_clk;
 *   - h0E = Ad_dessc_pll_ref_clk;
 *   - h0F = Ad_dessc_pll_fb_clk;
 *   - h10 = Ad_dessc_pix_clk;
 *   - h11 = Ad_dessc_pll_div_clk;
 *   - h12 = Ad_txpll_ref_clk;
 *   - h13 = Ad_txpll_fb_clk;
 *   - h14 = Ad_esc_clk;
 *   - hXX = Ad_sys_clk;
 * 0x8043{RO} RGOD_CHK_VID_VS_WID[7:0] The vsync width line number fo pix_clk.
 * 0x8044{RO}:
 * - [7:4] RESERVED;
 * - [3:0] RGOD_CHK_VID_HS_WID[11:8]
 * 0x8045{RO} RGOD_CHK_VID_HS_WID[7:0] The hsync width line number fo pix_clk.
 * 0x8046{RO} RGOD_CHK_VID_VS_BP[7:0] The vertical back porch line number fo pix_clk.
 * 0x8047{RO} RGOD_CHK_VID_VS_FP[7:0] The vertical front porch line number fo pix_clk.
 * 0x8048{RO}:
 * - [7:4] RESERVED;
 * - [3:0] RGOD_CHK_VID_HS_BP[11:8];
 * 0x8049{RO} RGOD_CHK_VID_HS_BP[7:0] The horizontal back porch line number fo pix_clk.
 * 0x804A{RO}:
 * - [7:4] RESERVED;
 * - [3:0] RGOD_CHK_VID_HS_FP[11:8];
* 0x804B{RO} RGOD_CHK_VID_HS_FP[7:0] The horizontal front porch line number fo pix_clk.
 * 0x804C{RO}:
 * - [7:4] RESERVED;
 * - [3:0] RGOD_CHK_VID_VS_TOTAL[11:8];
* 0x804D{RO} RGOD_CHK_VID_VS_TOTAL[7:0] The vertical total line number fo pix_clk.
* 0x804E{RO} RGOD_CHK_VID_HS_TOTAL[15:8]
* 0x804F{RO} RGOD_CHK_VID_HS_TOTAL[7:0] The horizontal total pixel number fo pix_clk.
* 0x8050{RO}:
 * - [7:4] RESERVED;
 * - [3:0] RGOD_CHK_VID_VS_ACT[11:8];
* 0x8051{RO} RGOD_CHK_VID_VS_ACTL[7:0] The vertical active line number for pix_clk.
* 0x8052{RO}:
 * - [7:4] RESERVED;
 * - [3:0] RGOD_CHK_VID_HS_ACT[15:8];
 * 0x8053{RO} RGOD_CHK_VID_HS_ACT[7:0] The horizontal active pixel number for pix_clk.
 * 0x80A5 FREQ_SET[7:0]
 * 0x80A9 M[7:0]
 * 0x80AA K1[7:0]
 * 0x80AB K2[7:0]
 * 0x80AC K3[7:0]
 * 0x80AD --
 * 0x80BE:
 * - RGD_MIPI_PTN_EN[7]
 * - RGD_PTN_GEM_MODE_SEL[6]
 * - RGD_PTN_SEQ_SEL[2:0]
 * 0x80BF:
 * - RGD_PTN_MODE_SEL[5:3]
 * - RGD_PTN_CHNL_SEL[2:0]
 * 0x80C0 RGD_PTN_DATA_VALUE[7:0]
 * 0x80C1 RGD_PTN_GCM_DE_DLY[15:7]
 * 0x80C2 RGD_PTN_GCM_DE_DLY[7:0]
 * 0x80C3 RGD_PTN_GCM_DE_TOP[7:0]
 * 0x80C4 RGD_PTN_GCM_DE_CNT[15:8]
 * 0x80C5 RGD_PTN_GCM_DE_CNT[7:0]
 * 0x80C6 RGD_PTN_GCM_DE_LIN[15:8]
 * 0x80C7 RGD_PTN_GCM_DE_LIN[7:0]
 * 0x80C8 RGD_PTN_GCM_H_TOTAL[15:8]
 * 0x80C9 RGD_PTN_GCM_H_TOTAL[7:0]
 * 0x80CA RGD_PTN_GCM_V_TOTAL[15:8]
 * 0x80CB RGD_PTN_GCM_V_TOTAL[7:0]
 * 0x80CC RGD_PTN_GCM_HWIDTH[15:8]
 * 0x80CD RGD_PTN_GCM_HWIDTH[7:0]
 * 0x80CE RGD_PTN_GCM_VWIDTH[7:0]
 * 0x813A RG_H_FP[7:0]
 * 0x813B RG_H_SYNC_WID[7:0]
 * 0x813C:
 * - [7:4] RG_H_SYNC_WID[11:8]
 * - [3:0] RG_H_FP[11:8]
 * 0x813D RG_H_TOTAL[7:0]
 * 0x813E:
 * - RG_HV_SYNC_SEL[7]:
 *   - 0=Select sync from source decode;
 *   - 1=Select sync from sync mode gen module;
 * - [5:4] RESERVED
 * - [3:0] RG_H_TOTAL[11:8]
 * 0x813F:
 * - [7:4] RG_V_SYNC_WID[3:0]
 * - [3:0] RG_V_FP[3:0]
 * 0x8340:
 * - RGD_DPHY_TX_LANE_SWAP[7:3]
 *   - d00 = 3210;
 *   - d01 = 2310;
 *   - d02 = 3120;
 *   - d03 = 2130;
 *   - d04 = 1230;
 *   - d05 = 1320;
 *   - d06 = 3201;
 *   - d07 = 2301;
 *   - d08 = 3021;
 *   - d09 = 2031;
 *   - d10 = 0231;
 *   - d11 = 0321;
 *   - d12 = 3012;
 *   - d13 = 0312;
 *   - d14 = 3102;
 *   - d15 = 0132;
 *   - d16 = 1032;
 *   - d17 = 1302;
 *   - d18 = 2013;
 *   - d19 = 0213;
 *   - d20 = 2103;
 *   - d21 = 0123;
 *   - d22 = 1023;
 *   - d23 = 1203;
 * - RGD_DPHY_TX_HS_DATA_POL[2] 0=Normal, 1=Opposite MSB/LSB
 * - RGD_CS_WRRD_MODE[1] Cmd send/receive mode sel:
 *   - 0=Send cmd mode;
 *   - 1=Cmd mode;
 * - [0] RESERVED
 * 0x8310:
 * - RGD_PKT_CSI_MODE[7] Packet format mode:
 *   - 0=DSI mode; (Note: Must be 0 in MIPI/CSI2 usecase)
 *   - 1=CSI mode;
 * - RGD_CSI_EN[6] Video mode:
 *   - 0=DSI mode;
 *   - 1=CSI mode;
 * - RGD_PKT_RGB_MODE[5] Packet RGB mode seleect:
 *   - 0=LSB;
 *   - 1=MSB;
 * - RGD_BLLP_MODE[4] BLLP mode:
 *   - 0=Non-burst mode;
 *   - 1=Burst mode(When RGD_VIDEO_MODE=3);
 * - RGD_AUTO_CLK_EN[3] Auto power down clk lane:
 *   - 0=Disable;
 *   - 1=Enable(When burst mode);
 * - [2:0] RESERVED
 * 0x8311:
 * - [7:6] RESERVED
 * - RGD_LANE_MODE[5:4] Lane number select:
 *   - d1 = 1 Lane;
 *   - d2 = 2 Lane;
 *   - d3 = 3 Lane;
 *   - d0 = 4 Lane;
 * - RGD_LANE_MODE[3:2] Video mode:
 *   - d0 = None
 *   - d1 = Non-burst pulse mode;
 *   - d2 = Non-burst event mode;
 *   - d3 = Burst mode(When RGD_BLLP_MODE=1);
 * - RGD_PKT_VI[1:0] Virutal channel indentifieri.
 * 0x8312 RGD_PKT_RD_DLY_CNT[15:8] Async FIFO read delay counter
 * 0x8313 RGD_PKT_RD_DLY_CNT[7:0]
 * 0x8314 RGD_PKT_VSA[7:0] Vsync width(DSI=VSA,CSI=VSA+VBP-1)
 * 0x8315 RGD_PKT_VBP[7:0] VBP(DSI=VBP,CSI=0x01)
 * 0x8316 RGD_PKT_VACT[15:8] Vsync active
 * 0x8317 RGD_PKT_VACT[7:0]
 * 0x8318 RGD_PKT_VFP[7:0] VFP(DSI=VFP,CSI=0x01)
 * 0x8319 RGD_PKT_HSA[7:0] Hsync width(DSI=0x1E,CSI=0x01)
 * 0x831A RGD_PKT_HBP[7:0] HBP(DSI=0x1E,CSI=0x01)
 * 0x831B RGD_PKT_HFP[7:0] HFP(DSI=0x1E,CSI=0x01)
 * 0x831C RGD_PKT_PIX_NUM[15:8] Hsync active
 * 0x831D RGD_PKT_PIX_NUM[7:0]
 * 0x831E RGD_PKT_MAX_FRAME[7:0] Max frame number(Used in CSI mode)
 * 0x831F:
 * - RGD_PKT_BLANK_SEL[7] Blank packet data select:
 *   - 0=RGD_PKT_BLANK_DATA;
 *   - 1=0;
 * - RGD_PKT_CRC_ENABLE[6] CRC function:
 *   - 1=Disable;
 *   - 0=Enable;
 * - RGD_PKT_EOTP_EN[5] EOTP packet:
 *   - 1=Disable;
 *   - 0=Enable;
 * - RGD_PKT_EMBED_EN[4] Embed packet:
 *   - 1=Disable;
 *   - 0=Enable;
 * - RGD_PKT_FRAME_NUM_EN[3] Frame bymber increase cycled:
 *   - 1=Disable;
 *   - 0=Enable;
 * - RGD_PKT_DATA_TYPE[2:0]: Packet data type:
 *   - d0=RGB888
 *   - d1=RGB6L
 *   - d2=RGB666
 *   - d3=RGB565
 *   - d4=YUV16
 *   - d5=YUV24
 *   - d6=RAW8
 *   - d7=RAW10
 * 0x8341 RGD_DPHY_CK_POST[7:0]
 * 0x8342 RGD_DPHY_CK_TRAIL[7:0]
 * 0x8343 RGD_DPHY_CK_PRPR[7:0]
 * 0x8344 RGD_DPHY_CK_ZERO[7:0]
 * 0x8346 RGD_DPHY_LPX[7:0]
 * 0x8347 RGD_DPHY_HS_PRPR[7:0]
 * 0x8348 RGD_DPHY_HS_TRAIL[7:0]
 * 0x834A RGD_DPHY_HS_RQST_PRE_CNT[7:0]
**/

/* Master Logic Receiver Initialize */
#define LT8918_DEFINE_MLRX_INIT()                                               \
    {0xFF, 0x70, 0},        \
    {0x01, 0x80, 0},        \
    {0x03, 0x48, 0},        \
    {0x04, 0xA2, 0},        \
    {0x0C, 0x80, 0},        \
    {0x13, 0x80, 0},        \
    {0x18, 0x50, 0},        \
    {0x38, 0xB0, 0}

/* BT1120 Input Configurations */
#define LT8918_DEFINE_BT1120_INPUT(HACT, VACT, HFP, HSA, HBP, VFP, VSA, VBP)    \
    {0xFF, 0x80, 0},            \
    {0x02, 0x00, 0},            \
    {0x03, 0x02, 0},            \
    {0x04, (HACT >> 8), 0},     \
    {0x05, (HACT & 0xFF), 0},   \
    {0x06, (HFP >> 8), 0},      \
    {0x07, (HFP & 0xFF), 0},    \
    {0x08, (HSA >> 8), 0},      \
    {0x09, (HSA & 0xFF), 0},    \
    {0x0A, (VFP >> 8), 0},      \
    {0x0B, (VFP & 0xFF), 0},    \
    {0x0C, (VSA >> 8), 0},      \
    {0x0D, (VSA & 0xFF), 0},    \
    {0x0E, 0x00, 0},            \
    {0x0F, 0x00, 0},            \
    {0x6A, 0x00, 0},            \
    {0x6B, 0x00, 0},            \
    {0x33, 0x00, 0},            \
    {0xBE, 0x40, 0}

/* Test Pattern Pixel Clocks */
#define LT8918_DEFINE_PTN_PIXCLK(F, M, K1, K2, K3)                              \
    {0xFF, 0x70, 0},    \
    {0x38, 0x00, 0},    \
    {0xFF, 0x80, 0},    \
    {0xA1, 0x00, 0},    \
    {0xA5, F, 0},       \
    {0xA9, M, 0},       \
    {0xAA, K1, 0},      \
    {0xAB, K2, 0},      \
    {0xAC, K3, 0},      \
    {0xAD, 0x00, 0},    \
    {0xAD, 0x02, 0}

/* Test Pattern Timings */
#define LT8918_DEFINE_PTN_TIMING(HACT, VACT, HFP, HSA, HBP, VFP, VSA, VBP)      \
    {0xFF, 0x80, 0},                        \
    {0xBE, 0xC0, 0},                        \
    {0xBF, 0x3F, 0},                        \
    {0xC0, 0xFF, 0},                        \
    {0xC1, ((HBP+HSA) >> 8), 0},            \
    {0xC2, ((HBP+HSA) & 0xFF), 0},          \
    {0xC3, (VBP+VSA), 0},                   \
    {0xC4, (HACT >> 8), 0},                 \
    {0xC5, (HACT & 0xFF), 0},               \
    {0xC6, (VACT >> 8), 0},                 \
    {0xC7, (VACT & 0xFF), 0},               \
    {0xC8, ((HACT+HFP+HBP+HSA) >> 8), 0},   \
    {0xC9, ((HACT+HFP+HBP+HSA) & 0xFF), 0}, \
    {0xCA, ((VACT+VFP+VBP+VSA) >> 8), 0},   \
    {0xCB, ((VACT+VFP+VBP+VSA) & 0xFF), 0}, \
    {0xCC, (HSA >> 8), 0},                  \
    {0xCD, (HSA & 0xFF), 0},                \
    {0xCE, (VSA), 0}

/* Transfer PLL Configurations */
#define LT8918_DEFINE_TX_PLL(R7031, R7033, R7034, R7024)                        \
    {0xFF, 0x70, 0},        \
    {0x30, 0x02, 0},        \
    {0x31, R7031, 0},       \
    {0x33, R7033, 0},       \
    {0x34, R7034, 0},       \
    {0x24, R7024, 0},       \
    {0x23, 0x13, 0},        \
    {0x23, 0x93, 0}

/* Transfer DPHY Configurations */
#define LT8918_DEFINE_TX_DPHY(RESET_DELAY_MS)                                   \
    {0xFF, 0x83, 0},                        \
    {0x40, 0x00, 0},                        \
    {0x41, 0x05, 0},                        \
    {0x42, 0x05, 0},                        \
    {0x43, 0x05, 0},                        \
    {0x44, 0x0F, 0},                        \
    {0x46, 0x08, 0},                        \
    {0x47, 0x08, 0},                        \
    {0x48, 0x0A, 0},                        \
    {0x4A, 0x20, 0},                        \
    {0xFF, 0x60, 0},                        \
    {0x0D, 0xEF, (RESET_DELAY_MS*1000)},    \
    {0x0D, 0xFF, 0}

/* Transfer Protocol Configurations */
#define LT8918_DEFINE_TX_PROTOCOL(HACT, VACT, HFP, HSA, HBP, VFP, VSA, VBP, DLY)    \
    {0xFF, 0x80, 0},                \
    {0x14, (VSA+VBP-1), 0},         \
    {0xFF, 0x83, 0},                \
    {0x12, (DLY >> 8), 0},          \
    {0x13, (DLY & 0xFF), 0},        \
    {0x14, (VSA+VBP-1), 0},         \
    {0x15, 0x01, 0},                \
    {0x16, (VACT >> 8), 0},         \
    {0x17, (VACT & 0xFF), 0},       \
    {0x18, 0x01, 0},                \
    {0x19, 0x01, 0},                \
    {0x1A, 0x01, 0},                \
    {0x1B, 0x01, 0},                \
    {0x1C, (HACT >> 8), 0},         \
    {0x1D, (HACT & 0xFF), 0},       \
    {0x1E, 0x00, 0},                \
    {0x10, 0x58, 0},                \
    {0x1F, 0x04, 0},                \
    {0x11, 0x2C, 0}

/* LT8918_MODE_640X480_60FPS
 * Pixelclk=25.2MHz, Lanes=2
 */
__maybe_unused
static struct reg_sequence lt8918_640x480_60fps[] = {
    LT8918_DEFINE_PTN_PIXCLK(0xEA, 0x90, 0x20, 0xC4, 0xCB),
    LT8918_DEFINE_TX_PLL(0x2C, 0x14, 0x02, 0x44),
    LT8918_DEFINE_PTN_TIMING(640, 480, 16, 96, 48, 10, 2, 33),
    LT8918_DEFINE_TX_DPHY(50),
    LT8918_DEFINE_TX_PROTOCOL(640, 480, 16, 96, 48, 10, 2, 33, 100),
};

/* LT8918_MODE_720X480_30FPS
 * Pixelclk=13.1736MHz, Lanes=2
 */
__maybe_unused
static struct reg_sequence lt8918_720x480_30fps[] = {
    LT8918_DEFINE_PTN_PIXCLK(0x8A, 0x80, 0x00, 0x00, 0x00),
    LT8918_DEFINE_TX_PLL(0x2C, 0x0A, 0x02, 0x44),
    LT8918_DEFINE_PTN_TIMING(720, 480, 48, 32, 80, 3, 10, 6),
    LT8918_DEFINE_TX_DPHY(50),
    LT8918_DEFINE_TX_PROTOCOL(720, 480, 48, 32, 80, 3, 10, 6, 100),
};

/* LT8918_MODE_720X576_25FPS
 * Pixelclk=74.25MHz, Lanes=2
 */
__maybe_unused
static struct reg_sequence lt8918_720x576_25fps[] = {
    LT8918_DEFINE_PTN_PIXCLK(0x8A, 0x80, 0x00, 0x00, 0x00),
    LT8918_DEFINE_TX_PLL(0x2C, 0x0A, 0x02, 0x44),
    LT8918_DEFINE_PTN_TIMING(720, 576, 48, 32, 80, 3, 10, 6),
    LT8918_DEFINE_TX_DPHY(50),
    LT8918_DEFINE_TX_PROTOCOL(720, 576, 48, 32, 80, 3, 10, 6, 100),
};

/* LT8918_MODE_1280X720_60FPS
 * Pixelclk=64.0224MHz(CVT2), Lanes=2
 */
__maybe_unused
static struct reg_sequence lt8918_1280x720_60fps[] = {
    LT8918_DEFINE_PTN_PIXCLK(0xCA, 0x94, 0x7C, 0xB7, 0x0A),
    LT8918_DEFINE_TX_PLL(0x2C, 0x28, 0x02, 0x44),
    LT8918_DEFINE_PTN_TIMING(1280, 720, 48, 32, 80, 3, 5, 8),
    LT8918_DEFINE_TX_DPHY(50),
    LT8918_DEFINE_TX_PROTOCOL(1280, 720, 48, 32, 80, 3, 5, 8, 100),
};

/* LT8918_MODE_1920X1080_25FPS
 * Pixelclk=66.5MHz, Lanes=2
 */
__maybe_unused
static struct reg_sequence lt8918_1920x1080_25fps[] = {
    LT8918_DEFINE_MLRX_INIT(),
    LT8918_DEFINE_BT1120_INPUT(1920, 1080, 88, 44, 148, 35, 3, 7),
    LT8918_DEFINE_TX_PLL(0x2C, 0x28, 0x02, 0x44),
    LT8918_DEFINE_TX_DPHY(50),
    LT8918_DEFINE_TX_PROTOCOL(1920, 1080, 88, 44, 148, 35, 3, 7, 800),
};

/* LT8918_MODE_1920X1080_30FPS
 * Pixelclk=74.25MHz, Lanes=2
 */
__maybe_unused
static struct reg_sequence lt8918_1920x1080_30fps[] = {
    LT8918_DEFINE_PTN_PIXCLK(0xCA, 0x97, 0xC2, 0x8F, 0x5C),
    LT8918_DEFINE_TX_PLL(0x2C, 0x31, 0x02, 0x44),
    LT8918_DEFINE_PTN_TIMING(1920, 1080, 88, 44, 148, 32, 5, 8),
    LT8918_DEFINE_TX_DPHY(50),
    LT8918_DEFINE_TX_PROTOCOL(1920, 1080, 88, 44, 148, 32, 5, 8, 800),
};

/* LT8918_MODE_1920X1080_60FPS
 * Pixelclk=148.5MHz, Lanes=2
 */
__maybe_unused
static struct reg_sequence lt8918_1920x1080_60fps[] = {
    LT8918_DEFINE_PTN_PIXCLK(0xAA, 0x97, 0xC2, 0x8F, 0x5C),
    LT8918_DEFINE_TX_PLL(0x2C, 0x61, 0x02, 0x44),
    LT8918_DEFINE_PTN_TIMING(1920, 1080, 88, 44, 148, 4, 5, 36),
    LT8918_DEFINE_TX_DPHY(50),
    LT8918_DEFINE_TX_PROTOCOL(1920, 1080, 88, 44, 148, 4, 5, 36, 100),
};

/* LT8918_MODE_START_STREAM */
__maybe_unused
static struct reg_sequence lt8918_start_stream[] = {
    {0xFF, 0x60, 0},
    {0x0D, 0x03, 50000},
    {0x0D, 0xFF, 0},
};

/* LT8918_MODE_STOP_STREAM */
__maybe_unused
static struct reg_sequence lt8918_stop_stream[] = {
    {0xFF, 0x70, 0},
    {0x23, 0xE0, 0}, // MIPI：POWER DOWN TX, DISABLE CLK AND DATA
    {0x29, 0x81, 0},
    {0x38, 0x00, 0},
    {0x34, 0x49, 0},
    {0x35, 0x80, 0},
    {0xFF, 0x83, 0},
    {0x11, 0x00, 0}, // TURN OFF VIDEO MODE AND LANES
};

/* LT8918_MODE_TEST_PATTERN */
__maybe_unused
static struct reg_sequence lt8918_test_pattern[] = {
    {0xFF, 0x80, 0},
};

__maybe_unused
static struct lt8918_reg_table lt8918_mode_tables[] = {
    { // LT8918_MODE_640X480_60FPS
        lt8918_640x480_60fps, ARRAY_SIZE(lt8918_640x480_60fps)
    },
    { // LT8918_MODE_720X480_30FPS
        lt8918_720x480_30fps, ARRAY_SIZE(lt8918_720x480_30fps)
    },
    { // LT8918_MODE_720X576_25FPS
        lt8918_720x576_25fps, ARRAY_SIZE(lt8918_720x576_25fps)
    },
    { // LT8918_MODE_1280X720_60FPS
        lt8918_1280x720_60fps, ARRAY_SIZE(lt8918_1280x720_60fps)
    },
    { // LT8918_MODE_1920X1080_25FPS
        lt8918_1920x1080_25fps, ARRAY_SIZE(lt8918_1920x1080_25fps)
    },
    { // LT8918_MODE_1920X1080_30FPS
        lt8918_1920x1080_30fps, ARRAY_SIZE(lt8918_1920x1080_30fps)
    },
    { // LT8918_MODE_1920X1080_60FPS
        lt8918_1920x1080_60fps, ARRAY_SIZE(lt8918_1920x1080_60fps)
    },
    { // LT8918_MODE_START_STREAM
        lt8918_start_stream, ARRAY_SIZE(lt8918_start_stream)
    },
    { // LT8918_MODE_STOP_STREAM
        lt8918_stop_stream, ARRAY_SIZE(lt8918_stop_stream)
    },
    { // LT8918_MODE_TEST_PATTERN
        lt8918_test_pattern, ARRAY_SIZE(lt8918_test_pattern)
    },
};

__maybe_unused
static const int lt8918_25fps[] = {
    25,
};

__maybe_unused
static const int lt8918_30fps[] = {
    30,
};

__maybe_unused
static const int lt8918_50fps[] = {
    50,
};

__maybe_unused
static const int lt8918_60fps[] = {
    60,
};

__maybe_unused
static const struct camera_common_frmfmt lt8918_frmfmt[] = {
    {{ 640,  480}, lt8918_60fps, 1, 0, LT8918_MODE_640X480_60FPS},
    {{ 720,  480}, lt8918_30fps, 1, 0, LT8918_MODE_720X480_30FPS},
    {{ 720,  576}, lt8918_25fps, 1, 0, LT8918_MODE_720X576_25FPS},
    {{1280,  720}, lt8918_60fps, 1, 0, LT8918_MODE_1280X720_60FPS},
    {{1920, 1080}, lt8918_25fps, 1, 0, LT8918_MODE_1920X1080_25FPS},
    {{1920, 1080}, lt8918_30fps, 1, 0, LT8918_MODE_1920X1080_30FPS},
    {{1920, 1080}, lt8918_60fps, 1, 0, LT8918_MODE_1920X1080_60FPS},
    /* Add modes with no device tree support after below */
};

/* clang-format on */

#endif /* __LT8918_MODE_TBLS_H__ */