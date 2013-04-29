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
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/i2c/at24.h>
#include <linux/i2c/pca953x.h>
#include <linux/leds-pca9532.h>
#include <linux/memblock.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/stmpe.h>
#include <linux/spi/max7301.h> 
#include <sound/tlv320aic3x.h>
#include <media/tw9910.h>
#include <linux/w1-gpio.h>

#include <mach/mxc_camera.h>

#include <asm/irq.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"

#include "board-mx6q_phycard.h"
#include "board-mx6q_phytec-common.h"
#include "board-mx6q_phytec-sd.h"
#include "board-mx6q_phytec-nand.h"

#define MX6_PHYCARD_CAM0_LVDS_PWRDN    IMX_GPIO_NR(5, 27)
#define MX6_PHYCARD_CAM0_OE            IMX_GPIO_NR(5, 20)

#define MX6_PHYCARD_SD3_CD		IMX_GPIO_NR(5, 22)

/* GPIO PIN, sort by PORT/BIT */
#define MX6_PHYCARD_ECSPI3_CS0		IMX_GPIO_NR(4, 24)
#define MX6_PHYCARD_ECSPI3_CS1		IMX_GPIO_NR(4, 25)

#define MX6_PHYCARD_USB_OTG_PWR		IMX_GPIO_NR(4, 15)

#define MX6_PHYCARD_USB_HOST1_OC	IMX_GPIO_NR(1, 3)

#define MX6_PHYCARD_CAP_TCH_INT0	IMX_GPIO_NR(4, 29)

#define MX6_PHYCARD_CSI0_RST		IMX_GPIO_NR(4, 5)
#define MX6_PHYCARD_CSI0_RST_TVIN	IMX_GPIO_NR(5, 25)
#define MX6_PHYCARD_MAX7310_1_BASE_ADDR	IMX_GPIO_NR(8, 0)
#define MX6_PHYCARD_PCA9538_BASE_ADDR	IMX_GPIO_NR(8, 8)

#define MX6_PHYCARD_PEB1_INT		IMX_GPIO_NR(1, 6)
#define MX6_PHYCARD_SSI_RESET		IMX_GPIO_NR(7, 12)
#define	MX6_PHYCARD_AC97_INT		IMX_GPIO_NR(5, 14)

static char *csi0;
module_param(csi0, charp, S_IRUGO);

void __init early_console_setup(unsigned long base, struct clk *clk);

extern struct regulator *(*get_cpu_regulator)(void);
extern void (*put_cpu_regulator)(void);
extern char *gp_reg_id;
extern void mx6_cpu_regulator_init(void);


static const struct anatop_thermal_platform_data
	mx6_phycard_anatop_thermal_data __initconst = {
	.name = "anatop_thermal",
};

static inline void mx6_phycard_init_uart(void)
{
	imx6q_add_imx_uart(2, NULL);
	imx6q_add_imx_uart(3, NULL);
}

static int mx6_phycard_fec_phy_init(struct phy_device *phydev)
{
	printk("FEC Manufacturer ID: 0x%X, chip ID: 0x%X\n", phy_read(phydev, 0x02), phy_read(phydev, 0x03));
	return 0;
}

static int mx6_phycard_fec_power_hibernate(struct phy_device *phydev)
{
	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init			= mx6_phycard_fec_phy_init,
	.power_hibernate	= mx6_phycard_fec_power_hibernate,
	.phy			= PHY_INTERFACE_MODE_MII,
};

static int mx6_phycard_spi_cs[] = {
	MX6_PHYCARD_ECSPI3_CS0,
	MX6_PHYCARD_ECSPI3_CS1,
};

static const struct spi_imx_master mx6_phycard_spi_data __initconst = {
	.chipselect     = mx6_phycard_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6_phycard_spi_cs),
};

static struct imxi2c_platform_data mx6_phycard_i2c0_data = {
	.bitrate = 100000,
};

static struct imxi2c_platform_data mx6_phycard_i2c1_data = {
	.bitrate = 100000,
};

static struct stmpe_ts_platform_data stmpe811_ts_data0 = {
	.sample_time	= 4, // ADC converstion time in number of clock.
	.mod_12b	= 0, // ADC Bit mode (0 -> 10bit ADC, 1 -> 12bit ADC)
	.ref_sel	= 0, // ADC reference source (0 -> internal reference, 1 -> external reference)
	.adc_freq	= 3, // ADC Clock speed (0 -> 1.625 MHz, 1 -> 3.25 MHz, 2 || 3 -> 6.5 MHz).
	.ave_ctrl	= 2, // Previous be 1
	.touch_det_delay= 4, // Recomended 3
	.settling	= 3, // Recomended >= 1ms if TS >= 6" (3=1ms, 4=5ms)
	.fraction_z	= 7, // Recomended value 7
	.i_drive	= 0, // Recomended value 0
};

static struct stmpe_platform_data stmpe811_data0 = {
        .id             = 0,
        .blocks         = STMPE_BLOCK_TOUCHSCREEN,
        .ts 		= &stmpe811_ts_data0,
};

static struct at24_platform_data at24c32 = {
	.byte_len	= SZ_32K / 8,
	.page_size	= 8,
	.flags		= AT24_FLAG_ADDR16,
};

static struct pca9532_platform_data lvds_backlight = {
	.leds = {
			{
				.name = "lvds:backlight",
				.state = PCA9532_PWM0,
				.type = PCA9532_TYPE_LED,
			},
		},
	.psc = { 0, 0 },
	.pwm = { 0, 0 },
};

static struct max7301_platform_data max7301_i2c_data = {
	.base		= -1,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("24c32", 0x50),
		.platform_data = &at24c32,
	},
};

static struct aic3x_pdata tlv320_phycard_pdata = {
	.gpio_reset = MX6_PHYCARD_SSI_RESET,
};

static int pca9538_setup(struct i2c_client *client,
				 unsigned gpio_base, unsigned ngpio,
				 void *context)
{
	int n;

	for (n = 0; n < ngpio; ++n) {
		gpio_request(gpio_base + n, "PCA9538 GPIO Expander");
		gpio_direction_output(gpio_base + n, 1);
	}

	return 0;
}

static struct pca953x_platform_data pca9538_platdata = {
	.gpio_base	= MX6_PHYCARD_PCA9538_BASE_ADDR,
	.invert		= 0,
	.setup		= pca9538_setup,
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("max7300", 0x40),
		.platform_data = &max7301_i2c_data,
	}, {
		I2C_BOARD_INFO("max1037", 0x64),
	}, {
		I2C_BOARD_INFO("rtc8564", 0x51),
	}, {
		I2C_BOARD_INFO("pca9530", 0x60),
		.platform_data = &lvds_backlight,
	}, {
		I2C_BOARD_INFO("pca9538", 0x70),
		.platform_data = &pca9538_platdata,
	},
};

static struct i2c_board_info mxc_i2c1_board_info_hda[] __initdata = {
	{
		I2C_BOARD_INFO("tlv320aic3007", 0x18),
		.platform_data = &tlv320_phycard_pdata,
	}, {
		I2C_BOARD_INFO("stmpe811", 0x44),
		.irq = gpio_to_irq(MX6_PHYCARD_CAP_TCH_INT0),
		.platform_data = (void *)&stmpe811_data0,
	},
};

static void imx6_phycard_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(MX6_PHYCARD_USB_OTG_PWR, 1);
	else
		gpio_set_value(MX6_PHYCARD_USB_OTG_PWR, 0);
}

static void __init mx6_phycard_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);

	/* disable external charger detect,
	 * or it will affect signal quality at dp.
	 */

	ret = gpio_request(MX6_PHYCARD_USB_OTG_PWR, "usb-pwr");
	if (ret) {
		pr_err("failed to get GPIO MX6_PHYCARD_USB_OTG_PWR:%d\n", ret);
		return;
	}
	gpio_direction_output(MX6_PHYCARD_USB_OTG_PWR, 0);

	mxc_iomux_set_gpr_register(1, 13, 1, 1);

	/* TODO: uncoment mx6_set_otghost_vbus_func when usb pwr hardware
	* fixes are available
	*/
	// mx6_set_otghost_vbus_func(imx6_phyflex_usbotg_vbus);

	/* TODO: remove next line when usb pwr are fixed.
	* Turn on VBUS power for USB.
	*/
	imx6_phycard_usbotg_vbus(true);

#ifdef CONFIG_USB_FSL_ARC_OTG
	mx6_usb_dr_init();
#endif /* CONFIG_USB_FSL_ARC_OTG */
}

static struct viv_gpu_platform_data imx6_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M,
};

static struct ipuv3_fb_platform_data phycard_fb_data[] = {
    {
        .disp_dev		= "ldb",
        .interface_pix_fmt	= IPU_PIX_FMT_RGB666,
        .mode_str		= "LDB-VGA",
        .default_bpp		= 16,
        .int_clk		= false,
    },
};

static struct fb_videomode ldb_modedb[] = {
        {
		/* 5.0 inch display */
		.name         = "Primeview-PD050VL1",
		.refresh      = 60,
		.xres         = 640,
		.yres         = 480,
		.pixclock     = 40000, /* in ps (25 MHz) */
		.hsync_len    = 32,
		.left_margin  = 112,
		.right_margin = 36,
		.vsync_len    = 2,
		.upper_margin = 33,
		.lower_margin = 33,
		.sync         = 0,
		.vmode        = FB_VMODE_NONINTERLACED,
		.flag         = 0,
	}, {
		/* 10.4 inch display */
		.name         = "Primeview-PD104SLF",
		.refresh      = 60,
		.xres         = 800,
		.yres         = 600,
		.pixclock     = 25000, /* in ps (40,0 MHz) */
		.hsync_len    = 128,
		.left_margin  = 42,
		.right_margin = 42,
		.vsync_len    = 4,
		.upper_margin = 23,
		.lower_margin = 1,
		.sync         = 0,
		.vmode        = FB_VMODE_NONINTERLACED,
		.flag         = 0,
	}, {
		/* 7.0 inch display */
		.name         = "Primeview-PM070WL4",
		.refresh      = 60,
		.xres         = 800,
		.yres         = 480,
		.pixclock     = 50505, /* in ps (19,8 MHz) */
		.hsync_len    = 128,
		.left_margin  = 42,
		.right_margin = 42,
		.vsync_len    = 2,
		.upper_margin = 33,
		.lower_margin = 33,
		.sync         = 0,
		.vmode        = FB_VMODE_NONINTERLACED,
		.flag         = 0,
	}, {
		/* 7.0 inch display */
		.name         = "ETM0700G0DH6",
		.refresh      = 60,
		.xres         = 800,
		.yres         = 480,
		.pixclock     = 30066, /* in ps (33,26 MHz) */
		.hsync_len    = 128,
		.left_margin  = 40,
		.right_margin = 216,
		.vsync_len    = 2,
		.upper_margin = 10,
		.lower_margin = 35,
		.sync         = 0,
		.vmode        = FB_VMODE_NONINTERLACED,
		.flag         = 0,
	}
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id		= 0,
	.disp_id	= 0,
	.ext_ref	= 1,
	//.mode		= LDB_SIN0,
	.mode		= LDB_SEP0,
	.sec_ipu_id	= 1,
	.sec_disp_id	= 1,
	.modes		= ldb_modedb,
	.num_modes	= ARRAY_SIZE(ldb_modedb),
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
	.rev		= 4,
	.csi_clk[0]	= "clko_clk",
	},
};

static const struct flexcan_platform_data mx6_phyflex_flexcan0_pdata __initconst = {
	/* PhyFlex board don't switch transivers */
	.transceiver_switch = NULL,
};

static void phycard_suspend_enter(void)
{
	/* suspend preparation */
}

static void phycard_suspend_exit(void)
{
	/* resmue resore */
}

static const struct pm_platform_data mx6_phycard_pm_data __initconst = {
	.name		= "imx_pm",
	.suspend_enter	= phycard_suspend_enter,
	.suspend_exit	= phycard_suspend_exit,
};


#ifdef CONFIG_SND_SOC_IMX_TLV320AIC3007

static struct regulator_consumer_supply tlv320aic3007_phycard_consumer_iovdd = {
	.supply		= "IOVDD",
	.dev_name	= "1-0018",
};
static struct regulator_consumer_supply tlv320aic3007_phycard_consumer_dvdd = {
	.supply		= "DVDD",
	.dev_name	= "1-0018",
};
static struct regulator_consumer_supply tlv320aic3007_phycard_consumer_avdd = {
	.supply		= "AVDD",
	.dev_name	= "1-0018",
};
static struct regulator_consumer_supply tlv320aic3007_phycard_consumer_drvdd = {
	.supply		= "DRVDD",
	.dev_name	= "1-0018",
};

static struct regulator_init_data tlv320aic3007_phycard_iovdd_reg_initdata = {
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &tlv320aic3007_phycard_consumer_iovdd,
};
static struct regulator_init_data tlv320aic3007_phycard_dvdd_reg_initdata = {
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &tlv320aic3007_phycard_consumer_dvdd,
};
static struct regulator_init_data tlv320aic3007_phycard_avdd_reg_initdata = {
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &tlv320aic3007_phycard_consumer_avdd,
};
static struct regulator_init_data tlv320aic3007_phycard_drvdd_reg_initdata = {
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &tlv320aic3007_phycard_consumer_drvdd,
};

static struct fixed_voltage_config tlv320aic3007_phycard_iovdd_reg_config = {
	.supply_name	= "IOVDD",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &tlv320aic3007_phycard_iovdd_reg_initdata,
};
static struct fixed_voltage_config tlv320aic3007_phycard_dvdd_reg_config = {
	.supply_name	= "DVDD",
	.microvolts		= 1800000,
	.gpio			= -1,
	.init_data		= &tlv320aic3007_phycard_dvdd_reg_initdata,
};
static struct fixed_voltage_config tlv320aic3007_phycard_avdd_reg_config = {
	.supply_name	= "AVDD",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &tlv320aic3007_phycard_avdd_reg_initdata,
};
static struct fixed_voltage_config tlv320aic3007_phycard_drvdd_reg_config = {
	.supply_name	= "DRVDD",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &tlv320aic3007_phycard_drvdd_reg_initdata,
};

static struct platform_device tlv320aic3007_phycard_iovdd_reg_devices = {
	.name = "reg-fixed-voltage",
	.id	  = 8,
	.dev  = {
		.platform_data = &tlv320aic3007_phycard_iovdd_reg_config,
	},
};
static struct platform_device tlv320aic3007_phycard_dvdd_reg_devices = {
	.name = "reg-fixed-voltage",
	.id   = 9,
	.dev  = {
		.platform_data = &tlv320aic3007_phycard_dvdd_reg_config,
	},
};
static struct platform_device tlv320aic3007_phycard_avdd_reg_devices = {
	.name = "reg-fixed-voltage",
	.id	  = 10,
	.dev  = {
		.platform_data = &tlv320aic3007_phycard_avdd_reg_config,
	},
};
static struct platform_device tlv320aic3007_phycard_drvdd_reg_devices = {
	.name = "reg-fixed-voltage",
	.id	  = 11,
	.dev  = {
		.platform_data = &tlv320aic3007_phycard_drvdd_reg_config,
	},
};

#endif /* CONFIG_SND_SOC_TLV320AIC3007 */

static struct regulator_consumer_supply phycard_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data phycard_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(phycard_vmmc_consumers),
	.consumer_supplies = phycard_vmmc_consumers,
};

static struct fixed_voltage_config phycard_vmmc_reg_config = {
	.supply_name	= "vmmc",
	.microvolts	= 3300000,
	.gpio		= -1,
	.init_data	= &phycard_vmmc_init,
};

static struct platform_device phycard_vmmc_reg_devices = {
	.name		= "reg-fixed-voltage",
	.id		= 0,
	.dev		= {
		.platform_data = &phycard_vmmc_reg_config,
	},
};

static struct mxc_audio_platform_data mx6_phycard_audio_data;

static struct imx_ssi_platform_data mx6_phycard_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct mxc_audio_platform_data mx6_phycard_audio_data = {
	.ssi_num = 2,
	.src_port = 2,
	.ext_port = 5,
	.hp_gpio = -1,
};

static struct platform_device mx6_phycard_audio_device = {
	.name = "tlv320aic3007",
};

static int __init mx6_phycard_init_audio(void)
{
	/* SSI audio init part */
	mxc_register_device(&mx6_phycard_audio_device,
						&mx6_phycard_audio_data);
	imx6q_add_imx_ssi(1, &mx6_phycard_ssi_pdata);

#ifdef CONFIG_SND_SOC_IMX_TLV320AIC3007
	platform_device_register(&tlv320aic3007_phycard_iovdd_reg_devices);
	platform_device_register(&tlv320aic3007_phycard_dvdd_reg_devices);
	platform_device_register(&tlv320aic3007_phycard_avdd_reg_devices);
	platform_device_register(&tlv320aic3007_phycard_drvdd_reg_devices);
#endif

	return 0;
}

#ifdef	CONFIG_SND_SOC_IMX_WM9712
/* fo phyCARD */
static iomux_v3_cfg_t mx6q_ac97_ssi_pads[] = {
	MX6Q_PAD_DISP0_DAT18__AUDMUX_AUD5_TXFS,
	MX6Q_PAD_DISP0_DAT16__AUDMUX_AUD5_TXC,
};
static iomux_v3_cfg_t mx6q_ac97_gpio_pads[] = {
	MX6Q_PAD_DISP0_DAT18__GPIO_5_12,
	MX6Q_PAD_DISP0_DAT16__GPIO_5_10,
};

//	imx_imx_ssi_data imx6_imx_ssi_data[]
static void mx6_ac97_warm_reset(struct snd_ac97 *ac97)
{
	mxc_iomux_v3_setup_multiple_pads(mx6q_ac97_gpio_pads, ARRAY_SIZE(mx6q_ac97_gpio_pads));
	gpio_set_value(IMX_GPIO_NR(5, 10), 0);
	gpio_set_value(IMX_GPIO_NR(5, 12), 0);
 	udelay(3);
 	gpio_set_value(IMX_GPIO_NR(5, 12), 1);
 	udelay(3);
 	gpio_set_value(IMX_GPIO_NR(5, 12), 0);
 	udelay(2);
	mxc_iomux_v3_setup_multiple_pads(mx6q_ac97_ssi_pads, ARRAY_SIZE(mx6q_ac97_ssi_pads));
	udelay(3);
}

static void mx6_ac97_cold_reset(struct snd_ac97 *ac97)
{
	mxc_iomux_v3_setup_multiple_pads(mx6q_ac97_gpio_pads, ARRAY_SIZE(mx6q_ac97_gpio_pads));
	gpio_set_value(IMX_GPIO_NR(5, 12), 0);
	gpio_set_value(IMX_GPIO_NR(5, 10), 0);
	udelay(3);
	gpio_set_value(MX6_PHYCARD_SSI_RESET, 0);
	udelay(5);
	gpio_set_value(MX6_PHYCARD_SSI_RESET, 1);
	udelay(2);
	mxc_iomux_v3_setup_multiple_pads(mx6q_ac97_ssi_pads, ARRAY_SIZE(mx6q_ac97_ssi_pads));
	mdelay(350); 
}

static struct imx_ssi_platform_data mx6_phycard_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN | IMX_SSI_USE_AC97 ,
	.ac97_reset = mx6_ac97_cold_reset ,
	.ac97_warm_reset = mx6_ac97_warm_reset,
};

static struct platform_device mx6_phycard_audio_device = {
	.name = "PhyCARD-ac97-audio",
};

static int imx_ac97_audmux_config(int slave, int master) {
	
	slave = slave - 1;
	master = master - 1; 

	// slave port audmux config
	mxc_audmux_v2_configure_port(slave, MXC_AUDMUX_V2_PTCR_SYN |
		MXC_AUDMUX_V2_PTCR_TCLKDIR | MXC_AUDMUX_V2_PTCR_TCSEL(master),
		MXC_AUDMUX_V2_PDCR_RXDSEL(master));

	// master port audmux config
	mxc_audmux_v2_configure_port(master, MXC_AUDMUX_V2_PTCR_SYN |
		MXC_AUDMUX_V2_PTCR_TFSDIR | MXC_AUDMUX_V2_PTCR_TFSEL(slave),
		MXC_AUDMUX_V2_PDCR_RXDSEL(slave));

	return 0;
}

static int __init mx6_ac97_init_audio(void) {

	gpio_request_one(MX6_PHYCARD_SSI_RESET, GPIOF_OUT_INIT_HIGH, "AC97");
	gpio_request_one(IMX_GPIO_NR(5, 12), GPIOF_OUT_INIT_HIGH, "AC97");
	gpio_request_one(IMX_GPIO_NR(5, 10), GPIOF_OUT_INIT_HIGH, "AC97");

#define	INTERNAL_PORT	2
#define	EXTERNAL_PORT	5
	imx_ac97_audmux_config(INTERNAL_PORT,EXTERNAL_PORT);

	platform_device_register(&mx6_phycard_audio_device);
	imx6q_add_imx_ssi(1, &mx6_phycard_ssi_pdata);

	return 0;
}
#else
static int __init mx6_ac97_init_audio(void) {
	return 0;
}
#endif

static struct mxc_dvfs_platform_data phycard_dvfscore_data = {
	.reg_id			= "cpu_vddgp",
	.clk1_id		= "cpu_clk",
	.clk2_id		= "gpc_dvfs_clk",
	.gpc_cntr_offset	= MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset	= MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset	= MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset	= MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask		= 0x1F800,
	.prediv_offset		= 11,
	.prediv_val		= 3,
	.div3ck_mask		= 0xE0000000,
	.div3ck_offset		= 29,
	.div3ck_val		= 2,
	.emac_val		= 0x08,
	.upthr_val		= 25,
	.dnthr_val		= 9,
	.pncthr_val		= 33,
	.upcnt_val		= 10,
	.dncnt_val		= 10,
	.delay_time		= 80,
};

static void __init mx6_phycard_fixup(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
}

/*
 * Dallas 1-Wire GPIO interface
 */
static struct w1_gpio_platform_data w1_gpio_pdata = {
	.pin			= IMX_GPIO_NR(5, 26),
	.is_open_drain		= 0,
	.enable_external_pullup	= NULL,
};

static struct platform_device w1_device = {
	.name			= "w1-gpio",
	.id			= -1,
	.dev.platform_data	= &w1_gpio_pdata,
};

static struct gpio_led gpio_leds[] = {
	{
		.name                   = "green",
		.default_trigger        = "mmc0",
		.gpio                   = IMX_GPIO_NR(1, 7),
	}, {
		.name                   = "red",
		.default_trigger        = "heartbeat",
		.gpio                   = IMX_GPIO_NR(3, 20),
	},
};
static struct gpio_led_platform_data gpio_led_info = {
	.leds           = gpio_leds,
	.num_leds       = ARRAY_SIZE(gpio_leds),
};
static struct platform_device leds_gpio = {
	.name   = "leds-gpio",
	.id     = -1,
	.dev    = {
		.platform_data  = &gpio_led_info,
	},
};

static struct mxc_camera_pdata mxc_ipu_csi_pdata[] = {
	{
		.flags = MXC_CAMERA_DATAWIDTH_10,
		.ipu = 0,
		.csi = 0,
		.mclk_default_rate = 27000000,
		.mclk_target_rate = 60000000,   /* only for mt9p031 */
		.use_pll = 0,                   /* only for mt9p031 */
	},
};

static u64 mxc_cam_dmamask = DMA_BIT_MASK(32);

static struct platform_device mxc_ipu_csi_devices[] = {
	{
		.name   = "mxc-camera",
		.id     = 0,
		.dev    = {
			.platform_data = &mxc_ipu_csi_pdata[0],
			.dma_mask = &mxc_cam_dmamask,
			.coherent_dma_mask = DMA_BIT_MASK(32),
		},
	},
};

int tw9910_switch_input(int input)
{
	int i;

	for (i = 0; i < 4; i++) {
		if (i == input) {
			gpio_direction_output(MX6_PHYCARD_PCA9538_BASE_ADDR
						+ i * 2, 0);
			gpio_direction_output(MX6_PHYCARD_PCA9538_BASE_ADDR
						+ i * 2 + 1, 0);
		} else {
			gpio_direction_output(MX6_PHYCARD_PCA9538_BASE_ADDR
						+ i * 2, 1);
			gpio_direction_output(MX6_PHYCARD_PCA9538_BASE_ADDR
						+ i * 2 + 1, 1);
		}
	}

	return 0;
}

struct tw9910_video_info tw9910_info = {
	.switch_input = tw9910_switch_input,
	.buswidth = SOCAM_DATAWIDTH_8,
	.mpout = TW9910_MPO_RTCO,
};

static void __init mx6_cameras_init(void)
{
	char *cam_parameter_separator = ",";
	char *csi0_cam_type = NULL;
	char *csi0_cam_address = NULL;
	long csi0_cam_address_hex;
	int i;

	if (!cpu_is_mx6q() && !cpu_is_mx6dl())
		return;

	/***************************************************
	Camera section:
	The bootarg csi0 will be interpreted. It has the following structure:
	Interface=<Camera-Type>,<I2C-Address>
	For example: csi0=VM-010,0x48

	If only the Interface-Type is specified, the default settings will
	be loaded.  If at csi0 the Camera-Type and I2C-Address are
	specified, only this camera work.
	***************************************************/

	csi0_cam_type = strsep(&csi0, cam_parameter_separator);
	csi0_cam_address = strsep(&csi0, cam_parameter_separator);

	if (csi0_cam_type != NULL) {
		if (!strcmp("VM-006", csi0_cam_type))
			csi0_cam_type = "mt9m001";
		if (!strcmp("VM-008", csi0_cam_type))
			csi0_cam_type = "tw9910";
		if (!strcmp("VM-009", csi0_cam_type))
			csi0_cam_type = "mt9m111";
		if (!strcmp("VM-010", csi0_cam_type))
			csi0_cam_type = "mt9v022";
		if (!strcmp("VM-011", csi0_cam_type))
			csi0_cam_type = "mt9p031";
	}

#define SOC_CAM_LINK(bus, bi, i2c_adapter) \
.bus_id = bus, .board_info = bi, .i2c_adapter_id = i2c_adapter
#define SOC_CAM_PDRV(dev_id, iclinks) \
.name = "soc-camera-pdrv", .id = dev_id,	\
.dev = { .platform_data = &iclinks[dev_id] }

	if (!csi0_cam_type) {
		/* Default autodetecting configuration */
		static struct i2c_board_info phycard_cameras[] = {
			{I2C_BOARD_INFO("mt9m001", 0x5d)},
			{I2C_BOARD_INFO("tw9910", 0x45)},
			{I2C_BOARD_INFO("mt9m111", 0x48)},
			{I2C_BOARD_INFO("mt9v022", 0x48)},
			{I2C_BOARD_INFO("mt9p031", 0x48)}
		};
		static struct soc_camera_link phycard_iclinks[] = {
			{SOC_CAM_LINK(0, &phycard_cameras[0], 1)},
			{SOC_CAM_LINK(0, &phycard_cameras[1], 1),
			 .priv = &tw9910_info},
			{SOC_CAM_LINK(0, &phycard_cameras[2], 1)},
			{SOC_CAM_LINK(0, &phycard_cameras[3], 1)},
			{SOC_CAM_LINK(0, &phycard_cameras[4], 1),
			 .flags = SOCAM_SENSOR_INVERT_PCLK}
		};
		static struct platform_device mxc_ipu_cameras[] = {
			{SOC_CAM_PDRV(0, phycard_iclinks)},
			{SOC_CAM_PDRV(1, phycard_iclinks)},
			{SOC_CAM_PDRV(2, phycard_iclinks)},
			{SOC_CAM_PDRV(3, phycard_iclinks)},
			{SOC_CAM_PDRV(4, phycard_iclinks)}
		};

		for (i = 0; i < ARRAY_SIZE(mxc_ipu_cameras); i++)
			platform_device_register(&mxc_ipu_cameras[i]);
	} else {
		static struct i2c_board_info phycard_cameras[] = {
			[0] = {I2C_BOARD_INFO("mt9v022", 0x48)}
		};
		static struct soc_camera_link phycard_iclinks[] = {
			{SOC_CAM_LINK(0, &phycard_cameras[0], 1)}
		};
		static struct platform_device mxc_ipu_cameras[] = {
			{SOC_CAM_PDRV(0, phycard_iclinks)}
		};

		csi0_cam_address_hex = 0x48;
		if (!strcmp("mt9m001", csi0_cam_type)) {
			mxc_ipu_csi_pdata[0].mclk_default_rate = 36000000;
			csi0_cam_address_hex = 0x5d;
		} else if (!strcmp("tw9910", csi0_cam_type)) {
			mxc_ipu_csi_pdata[0].mclk_default_rate = 27000000;
			phycard_iclinks[0].priv = &tw9910_info;
			csi0_cam_address_hex = 0x45;
		} else if (!strcmp("mt9m111", csi0_cam_type)) {
			mxc_ipu_csi_pdata[0].mclk_default_rate = 36000000;
		} else if (!strcmp("mt9v022", csi0_cam_type)) {
			mxc_ipu_csi_pdata[0].mclk_default_rate = 27000000;
		} else if (!strcmp("mt9p031", csi0_cam_type)) {
			mxc_ipu_csi_pdata[0].mclk_default_rate = 54000000;
			phycard_iclinks[0].flags = SOCAM_SENSOR_INVERT_PCLK;
		}

		if (csi0_cam_address) {
			int ret;
			long i2c_addr;

			ret = kstrtol(csi0_cam_address, 16, &i2c_addr);
			if (!ret)
				phycard_cameras[0].addr = i2c_addr;
		} else
			phycard_cameras[0].addr = csi0_cam_address_hex;

		strcpy(phycard_cameras[0].type, csi0_cam_type);

		platform_device_register(&mxc_ipu_cameras[0]);
	}

	platform_device_register(&mxc_ipu_csi_devices[0]);

	gpio_request(MX6_PHYCARD_CAM0_LVDS_PWRDN, "CSI0<->LVDS bridge #PWDN");
	gpio_request(MX6_PHYCARD_CAM0_OE, "IPU1/CSI0 camera #OE");
	gpio_direction_output(MX6_PHYCARD_CAM0_LVDS_PWRDN, 1);
	gpio_direction_output(MX6_PHYCARD_CAM0_OE, 1);
}


/*!
 * Board specific initialization.
 */
static void __init mx6_phycard_init(void)
{
	int i;

	/* imx6q SoC revision and CPU uniq ID setup */
	mx6_setup_cpuinfo();

	/* TODO: common_pads for both PhyFlex and PhyCard.
	 *All differents pads must initialised separated
	 */
	mxc_iomux_v3_setup_multiple_pads(mx6q_phytec_common_pads,
		ARRAY_SIZE(mx6q_phytec_common_pads));

	/* Enable both CSI in IOMUX mode instead of MIPI */
	mxc_iomux_set_gpr_register(1, 19, 2, 3);

	/* Init GPIO Led's */
	platform_device_register(&leds_gpio);

	gp_reg_id = phycard_dvfscore_data.reg_id;
	mx6_phycard_init_uart();

	imx6q_add_ipuv3(0, &ipu_data[0]);
	for (i = 0; i < ARRAY_SIZE(phycard_fb_data); i++)
		imx6q_add_ipuv3fb(i, &phycard_fb_data[i]);

	imx6q_add_ldb(&ldb_data);

	imx6q_add_imx_snvs_rtc();
	imx6q_add_imx_i2c(0, &mx6_phycard_i2c0_data);
	imx6q_add_imx_i2c(1, &mx6_phycard_i2c1_data);
	i2c_register_board_info(0, mxc_i2c0_board_info, ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info, ARRAY_SIZE(mxc_i2c1_board_info));

	/* SPI */
	imx6q_add_ecspi(2, &mx6_phycard_spi_data);

	imx6_init_fec(fec_data);

	imx6q_add_anatop_thermal_imx(1, &mx6_phycard_anatop_thermal_data);

	imx6q_add_pm_imx(0, &mx6_phycard_pm_data);
	board_esdhc_init(2, MX6_PHYCARD_SD3_CD, -EINVAL);

	imx_add_viv_gpu(&imx6_gpu_data, &imx6_gpu_pdata);
	imx6q_add_vpu();
	mx6_phycard_init_usb();

	/* Detecting HDA audio and STMPE811 touchscreen
	 * or WM9712 AC97 audio and touchscreen */
	gpio_request_one(MX6_PHYCARD_AC97_INT, GPIOF_IN, "Audio Detect");
	if (gpio_get_value(MX6_PHYCARD_AC97_INT)) {
		mx6_phycard_init_audio();
		i2c_register_board_info(1, mxc_i2c1_board_info_hda, ARRAY_SIZE(mxc_i2c1_board_info_hda));
	} else {
		mx6_ac97_init_audio();
	}

	mx6_cameras_init();
	platform_device_register(&phycard_vmmc_reg_devices);
	mx6_cpu_regulator_init();

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();
	board_nand_init();

	imx6q_add_dvfs_core(&phycard_dvfscore_data);

	platform_device_register(&w1_device);

}


extern void __iomem *twd_base;
static void __init mx6_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);

	early_console_setup(UART3_BASE_ADDR, uart_clk);
}

static struct sys_timer mxc_timer = {
	.init   = mx6_timer_init,
};

static void __init mx6_phycard_reserve(void)
{
	phys_addr_t phys;

	if (imx6_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(
			imx6_gpu_pdata.reserved_mem_size, SZ_4K, SZ_2G);
		memblock_free(phys, imx6_gpu_pdata.reserved_mem_size);
		memblock_remove(phys, imx6_gpu_pdata.reserved_mem_size);
		imx6_gpu_pdata.reserved_mem_base = phys;
	}
}

MACHINE_START(MX6Q_PHYCARD, "Phytec i.MX 6Quad phyCARD Board")
	.boot_params	= MX6_PHYS_OFFSET + 0x100,
	.fixup		= mx6_phycard_fixup,
	.map_io		= mx6_map_io,
	.init_irq	= mx6_init_irq,
	.init_machine	= mx6_phycard_init,
	.timer		= &mxc_timer,
	.reserve	= mx6_phycard_reserve,
MACHINE_END
