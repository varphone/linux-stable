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

#if defined(CONFIG_WL12XX)
	#include <linux/delay.h>
	#include <linux/wl12xx.h>
	#include <linux/gpio.h>

	#include <linux/regulator/anatop-regulator.h>
	#include <linux/regulator/consumer.h>
	#include <linux/regulator/machine.h>
	#include <linux/regulator/fixed.h>

	#define WLAN_IRQ	IMX_GPIO_NR(6, 17)
	#define WLAN_ENABLE	IMX_GPIO_NR(6, 18)
#endif


static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(3, 50);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(3, 100);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(3, 200);

static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING_SHORT(2, 50);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING_SHORT(2, 100);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING_SHORT(2, 200);

static iomux_v3_cfg_t MX6DL_USDHC_PAD_SETTING_SHORT(3, 50);
static iomux_v3_cfg_t MX6DL_USDHC_PAD_SETTING_SHORT(3, 100);
static iomux_v3_cfg_t MX6DL_USDHC_PAD_SETTING_SHORT(3, 200);

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

	if (index != 2) {
		printk(KERN_ERR"no such SD host controller index %d\n", index);
		return -EINVAL;
	}

	if (cpu_is_mx6q()) {
		sd3_pads_200mhz = mx6q_sd3_200mhz;
		sd3_pads_100mhz = mx6q_sd3_100mhz;
		sd3_pads_50mhz = mx6q_sd3_50mhz;

		sd3_pads_200mhz_cnt = ARRAY_SIZE(mx6q_sd3_200mhz);
		sd3_pads_100mhz_cnt = ARRAY_SIZE(mx6q_sd3_100mhz);
		sd3_pads_50mhz_cnt = ARRAY_SIZE(mx6q_sd3_50mhz);
	} else if (cpu_is_mx6dl()) {
		sd3_pads_200mhz = mx6dl_sd3_200mhz;
		sd3_pads_100mhz = mx6dl_sd3_100mhz;
		sd3_pads_50mhz = mx6dl_sd3_50mhz;

		sd3_pads_200mhz_cnt = ARRAY_SIZE(mx6dl_sd3_200mhz);
		sd3_pads_100mhz_cnt = ARRAY_SIZE(mx6dl_sd3_100mhz);
		sd3_pads_50mhz_cnt = ARRAY_SIZE(mx6dl_sd3_50mhz);
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
	.support_18v		= 0,
	.support_8bit		= 0,
	.keep_power_at_suspend	= 1,
	.delay_line		= 0,
	.platform_pad_change	= plt_sd2_pad_change,
};
#endif

#if !defined(CONFIG_WL12XX)
static struct esdhc_platform_data mx6_phytec_sd3_data __initconst = {
	.support_18v		= 0,
	.support_8bit		= 0,
	.keep_power_at_suspend	= 1,
	.delay_line		= 0,
	.platform_pad_change	= plt_sd3_pad_change,
};

#else
static struct esdhc_platform_data mx6_phytec_sd3_data __initconst = {
	.always_present		= 1,
	.cd_gpio		= -1,
	.wp_gpio		= -1,
	.support_18v		= 0,
	.support_8bit		= 0,
	.keep_power_at_suspend	= 0,
	.caps			= MMC_CAP_POWER_OFF_CARD,
	.platform_pad_change	= plt_sd3_pad_change,
};

static void wl1271_set_power(bool enable)
{
	if (0 == enable) {
		gpio_set_value(WLAN_ENABLE, 0);     /* momentarily disable */
		mdelay(2);
		gpio_set_value(WLAN_ENABLE, 1);
	}
}

struct wl12xx_platform_data n6q_wlan_data __initdata = {
	.irq		= gpio_to_irq(WLAN_IRQ),
	.board_ref_clock = WL12XX_REFCLOCK_38, /* 38.4 MHz */
	.set_power	= wl1271_set_power,
};

static struct regulator_consumer_supply n6q_vwl1271_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
};

static struct regulator_init_data n6q_vwl1271_init = {
	.constraints = {
		.name		= "VDD_1.8V",
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(n6q_vwl1271_consumers),
	.consumer_supplies	= n6q_vwl1271_consumers,
};

static struct fixed_voltage_config n6q_vwl1271_reg_config = {
	.supply_name	= "vwl1271",
	.microvolts	= 1800000, /* 1.80V */
	.gpio		= -1,
	.startup_delay	= 70000, /* 70ms */
	.enable_high	= 1,
	.enabled_at_boot = 0,
	.init_data	= &n6q_vwl1271_init,
};

static struct platform_device n6q_vwl1271_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 4,
	.dev	= {
		.platform_data = &n6q_vwl1271_reg_config,
	},
};

#endif

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
#if !defined(CONFIG_WL12XX)
		mx6_phytec_sd3_data.cd_gpio = cd_gpio;
		mx6_phytec_sd3_data.wp_gpio = wp_gpio;
		imx6q_add_sdhci_usdhc_imx(2, &mx6_phytec_sd3_data);
#else
		gpio_request(WLAN_ENABLE, "WLAN Enable");
		gpio_direction_output(WLAN_ENABLE, 1);
		wl1271_set_power(0);
		imx6q_add_sdhci_usdhc_imx(2, &mx6_phytec_sd3_data);
		/* WL12xx WLAN Init */
		if (wl12xx_set_platform_data(&n6q_wlan_data))
		      pr_err("error setting wl12xx data\n");
		platform_device_register(&n6q_vwl1271_reg_devices);
#endif
	}
}
