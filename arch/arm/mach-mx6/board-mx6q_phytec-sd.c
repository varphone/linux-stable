/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/clk.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/iomux-mx6q.h>

#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-mx6q_phytec-sd.h"

static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(3, 50);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(3, 100);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(3, 200);

static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING_SHORT(2, 50);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING_SHORT(2, 100);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING_SHORT(2, 200);

enum sd_pad_mode {
	SD_PAD_MODE_LOW_SPEED,
	SD_PAD_MODE_MED_SPEED,
	SD_PAD_MODE_HIGH_SPEED,
};

#if defined(CONFIG_MACH_MX6Q_PHYFLEX)
static int plt_sd2_pad_change(unsigned int index, int clock)
{
	static enum sd_pad_mode pad_mode = SD_PAD_MODE_LOW_SPEED;

	iomux_v3_cfg_t *sd2_pads_200mhz = NULL;
	iomux_v3_cfg_t *sd2_pads_100mhz = NULL;
	iomux_v3_cfg_t *sd2_pads_50mhz = NULL;

	u32 sd2_pads_200mhz_cnt;
	u32 sd2_pads_100mhz_cnt;
	u32 sd2_pads_50mhz_cnt;

	if (cpu_is_mx6q()) {
		sd2_pads_200mhz = mx6q_sd2_200mhz;
		sd2_pads_100mhz = mx6q_sd2_100mhz;
		sd2_pads_50mhz = mx6q_sd2_50mhz;

		sd2_pads_200mhz_cnt = ARRAY_SIZE(mx6q_sd2_200mhz);
		sd2_pads_100mhz_cnt = ARRAY_SIZE(mx6q_sd2_100mhz);
		sd2_pads_50mhz_cnt = ARRAY_SIZE(mx6q_sd2_50mhz);
	}

	if (clock > 100000000) {
		if (pad_mode == SD_PAD_MODE_HIGH_SPEED)
			return 0;

		pad_mode = SD_PAD_MODE_HIGH_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd2_pads_200mhz,
							sd2_pads_200mhz_cnt);
	} else if (clock > 52000000) {
		if (pad_mode == SD_PAD_MODE_MED_SPEED)
			return 0;

		pad_mode = SD_PAD_MODE_MED_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd2_pads_100mhz,
							sd2_pads_100mhz_cnt);
	} else {
		if (pad_mode == SD_PAD_MODE_LOW_SPEED)
			return 0;

		pad_mode = SD_PAD_MODE_LOW_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd2_pads_50mhz,
							sd2_pads_50mhz_cnt);
	}
}
#endif

static int plt_sd3_pad_change(unsigned int index, int clock)
{
	static enum sd_pad_mode pad_mode = SD_PAD_MODE_LOW_SPEED;

	iomux_v3_cfg_t *sd3_pads_200mhz = NULL;
	iomux_v3_cfg_t *sd3_pads_100mhz = NULL;
	iomux_v3_cfg_t *sd3_pads_50mhz = NULL;

	u32 sd3_pads_200mhz_cnt;
	u32 sd3_pads_100mhz_cnt;
	u32 sd3_pads_50mhz_cnt;

	if (cpu_is_mx6q()) {
		sd3_pads_200mhz = mx6q_sd3_200mhz;
		sd3_pads_100mhz = mx6q_sd3_100mhz;
		sd3_pads_50mhz = mx6q_sd3_50mhz;

		sd3_pads_200mhz_cnt = ARRAY_SIZE(mx6q_sd3_200mhz);
		sd3_pads_100mhz_cnt = ARRAY_SIZE(mx6q_sd3_100mhz);
		sd3_pads_50mhz_cnt = ARRAY_SIZE(mx6q_sd3_50mhz);
	}

	if (clock > 100000000) {
		if (pad_mode == SD_PAD_MODE_HIGH_SPEED)
			return 0;
		BUG_ON(!sd3_pads_200mhz);
		pad_mode = SD_PAD_MODE_HIGH_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd3_pads_200mhz,
							sd3_pads_200mhz_cnt);
	} else if (clock > 52000000) {
		if (pad_mode == SD_PAD_MODE_MED_SPEED)
			return 0;
		BUG_ON(!sd3_pads_100mhz);
		pad_mode = SD_PAD_MODE_MED_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd3_pads_100mhz,
							sd3_pads_100mhz_cnt);
	} else {
		if (pad_mode == SD_PAD_MODE_LOW_SPEED)
			return 0;
		BUG_ON(!sd3_pads_50mhz);
		pad_mode = SD_PAD_MODE_LOW_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd3_pads_50mhz,
							sd3_pads_50mhz_cnt);
	}
}

#if defined(CONFIG_MACH_MX6Q_PHYFLEX)
static struct esdhc_platform_data mx6_phytec_sd2_data __initconst = {
//	.cd_gpio		= 0,
//	.wp_gpio		= 0,
	.support_18v		= 1,
	.support_8bit		= 1,
	.keep_power_at_suspend	= 1,
	.delay_line		= 0,
	.platform_pad_change	= plt_sd2_pad_change,
};
#endif

static struct esdhc_platform_data mx6_phytec_sd3_data __initconst = {
//	.cd_gpio		= 0,
//	.wp_gpio		= 0,
	.support_18v		= 1,
	.support_8bit		= 1,
	.keep_power_at_suspend	= 1,
	.delay_line		= 0,
	.platform_pad_change	= plt_sd3_pad_change,
};

void __init board_esdhc_init(char id, int cd_gpio, int wp_gpio)
{
#if defined(CONFIG_MACH_MX6Q_PHYFLEX)
	if (1 == id) {
		mx6_phytec_sd2_data.cd_gpio = cd_gpio;
		mx6_phytec_sd2_data.wp_gpio = wp_gpio;
		imx6q_add_sdhci_usdhc_imx(1, &mx6_phytec_sd2_data);
	} else
#endif
	if (2 == id) {
		mx6_phytec_sd3_data.cd_gpio = cd_gpio;
		mx6_phytec_sd3_data.wp_gpio = wp_gpio;
		imx6q_add_sdhci_usdhc_imx(2, &mx6_phytec_sd3_data);
	}
}
