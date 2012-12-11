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

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/smsc911x.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/i2c/at24.h>
#include <linux/i2c/pca953x.h>
#include <linux/leds-pca9532.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/etherdevice.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/stmpe.h>
#include <linux/spi/max7301.h> 
#include <linux/can/platform/mcp251x.h> 
#include <sound/tlv320aic3x.h>

#include <linux/i2c-gpio.h>
#include <linux/w1-gpio.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>
#include <mach/mipi_dsi.h>
#include <mach/mipi_csi2.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

//#include <media/soc_camera_platform.h>
#include <media/soc_camera.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-mx6q_phycard.h"

#include "board-mx6q_phytec-sd.h"
#include "board-mx6q_phytec-nand.h"

#define MX6_PHYFLEX_SD3_CD		IMX_GPIO_NR(5, 22)
#define MX6_PHYFLEX_SD3_WP		IMX_GPIO_NR(5, 23)

/* GPIO PIN, sort by PORT/BIT */
#define MX6_PHYFLEX_ECSPI3_CS0		IMX_GPIO_NR(4, 24)
#define MX6_PHYFLEX_ECSPI3_CS1		IMX_GPIO_NR(4, 25)
#define MX6_PHYFLEX_ECSPI3_CS2		IMX_GPIO_NR(4, 26)
#define MX6_PHYFLEX_ECSPI3_CS3		IMX_GPIO_NR(4, 27)

#define MX6_PHYFLEX_USB_OTG_PWR		IMX_GPIO_NR(4, 15)

#define MX6_PHYFLEX_USB_HOST1_OC	IMX_GPIO_NR(1, 3)

#define MX6_PHYCARD_CAP_TCH_INT0	IMX_GPIO_NR(4, 29)
#define MX6_PHYFLEX_CAP_TCH_INT0	IMX_GPIO_NR(1, 5)
#define MX6_PHYFLEX_CAP_TCH_INT1	IMX_GPIO_NR(2, 24)

#define MX6_PHYFLEX_CSI0_RST		IMX_GPIO_NR(4, 5)
#define MX6_PHYFLEX_CSI0_RST_TVIN	IMX_GPIO_NR(5, 25)
#define MX6_PHYFLEX_MAX7310_1_BASE_ADDR	IMX_GPIO_NR(8, 0)
#define MX6_PHYFLEX_MAX7310_2_BASE_ADDR	IMX_GPIO_NR(8, 8)

#define MX6_PHYCARD_PEB1_INT		IMX_GPIO_NR(1, 6)
#define MX6_PHYCARD_SSI_RESET		IMX_GPIO_NR(7, 12)
#define	MX6_PHYCARD_AC97_INT		IMX_GPIO_NR(5, 14)

#define ENABLE_PHY
#define DDR_2GB    // for board versions with 2GB instead of 1GB

/* Kernel cmdline param to select TS */
static bool second_ts = false;
module_param(second_ts, bool, 0644);

void __init early_console_setup(unsigned long base, struct clk *clk);
static struct clk *sata_clk;

extern struct regulator *(*get_cpu_regulator)(void);
extern void (*put_cpu_regulator)(void);
extern char *gp_reg_id;
extern void mx6_cpu_regulator_init(void);

static bool 	HDA_ON;	// HDA audio ON/OFF detect


static const struct anatop_thermal_platform_data
	mx6_phyflex_anatop_thermal_data __initconst = {
	.name = "anatop_thermal",
};

static inline void mx6_phyflex_init_uart(void)
{
	imx6q_add_imx_uart(2, NULL);
	imx6q_add_imx_uart(3, NULL);
}

#ifdef ENABLE_PHY
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
#endif /* ENABLE_PHY */

static int mx6_phyflex_spi_cs[] = {
	MX6_PHYFLEX_ECSPI3_CS0,
	MX6_PHYFLEX_ECSPI3_CS1,
};

static const struct spi_imx_master mx6_phyflex_spi_data __initconst = {
	.chipselect     = mx6_phyflex_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6_phyflex_spi_cs),
};

static struct imxi2c_platform_data mx6_phyflex_i2c0_data = {
	.bitrate = 100000,
};

static struct imxi2c_platform_data mx6_phyflex_i2c1_data = {
	.bitrate = 100000,
};

static struct imxi2c_platform_data mx6_phyflex_i2c2_data = {
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

static struct at24_platform_data at24c04 = {
	.byte_len	= SZ_4K / 8,
	.page_size	= 16,
	.flags		= 0,
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

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("max1037", 0x64),
	}, {
		I2C_BOARD_INFO("24c04", 0x54),
		.platform_data = &at24c04,
	}, {
		I2C_BOARD_INFO("rtc8564", 0x51),
	}, {
		I2C_BOARD_INFO("pca9530", 0x60),
		.platform_data = &lvds_backlight,
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

static struct i2c_board_info __initdata hdmi_i2c_data[] = {
// ToDo: Uncoment next lines to make HDMI work
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
};

static struct i2c_gpio_platform_data i2c_gpio_data = {
	.sda_pin		= IMX_GPIO_NR(4, 13),
	.sda_is_open_drain      = 0,
	.scl_pin		= IMX_GPIO_NR(4, 12),
        .scl_is_open_drain      = 0,
	.timeout		= 0,    /* 0 default to 100 ms */
	.udelay			= 0,    /* 0 default to 100 kHz */
};

static struct platform_device i2c_gpio_device = {
	.name= "i2c-gpio",
	.id= 3,
	.dev= {
		.platform_data= &i2c_gpio_data,
	},
};

static void imx6_phyflex_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(MX6_PHYFLEX_USB_OTG_PWR, 1);
	else
		gpio_set_value(MX6_PHYFLEX_USB_OTG_PWR, 0);
}

static void __init mx6_phyflex_init_usb(void)
{
	int ret = 0;

	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);

	/* disable external charger detect,
	 * or it will affect signal quality at dp.
	 */

	ret = gpio_request(MX6_PHYFLEX_USB_OTG_PWR, "usb-pwr");
	if (ret) {
		pr_err("failed to get GPIO MX6_PHYFLEX_USB_OTG_PWR:%d\n", ret);
		return;
	}
	gpio_direction_output(MX6_PHYFLEX_USB_OTG_PWR, 0);

	mxc_iomux_set_gpr_register(1, 13, 1, 1);

	// ToDo: uncoment mx6_set_otghost_vbus_func when usb pwr hardware fixes are available
	// mx6_set_otghost_vbus_func(imx6_phyflex_usbotg_vbus);

	// ToDo: remove next line when usb pwr are fixed. Turn on VBUS power for USB.
	imx6_phyflex_usbotg_vbus(true);

	mx6_usb_dr_init();

	// gpio_direction_output(IMX_GPIO_NR(1, 0), 0); // Config PWR USB pin
	// gpio_set_value(IMX_GPIO_NR(1, 0), 0);	// Set USB power On
#ifdef CONFIG_USB_EHCI_ARC_HSIC
	mx6_usb_h2_init();
	mx6_usb_h3_init();
#endif
}

static struct viv_gpu_platform_data imx6_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M,
};


/* HW Initialization, if return 0, initialization is successful. */
static int mx6_phyflex_sata_init(struct device *dev, void __iomem *addr)
{
	u32 tmpdata;
	int ret = 0;
	struct clk *clk;

	/* Enable SATA PWR CTRL_0 of MAX7310 */
	gpio_request(MX6_PHYFLEX_MAX7310_1_BASE_ADDR, "SATA_PWR_EN");
	gpio_direction_output(MX6_PHYFLEX_MAX7310_1_BASE_ADDR, 1);

	sata_clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(sata_clk)) {
		dev_err(dev, "no sata clock.\n");
		return PTR_ERR(sata_clk);
	}
	ret = clk_enable(sata_clk);
	if (ret) {
		dev_err(dev, "can't enable sata clock.\n");
		goto put_sata_clk;
	}

	/* Set PHY Paremeters, two steps to configure the GPR13,
	 * one write for rest of parameters, mask of first write is 0x07FFFFFD,
	 * and the other one write for setting the mpll_clk_off_b
	 *.rx_eq_val_0(iomuxc_gpr13[26:24]),
	 *.los_lvl(iomuxc_gpr13[23:19]),
	 *.rx_dpll_mode_0(iomuxc_gpr13[18:16]),
	 *.sata_speed(iomuxc_gpr13[15]),
	 *.mpll_ss_en(iomuxc_gpr13[14]),
	 *.tx_atten_0(iomuxc_gpr13[13:11]),
	 *.tx_boost_0(iomuxc_gpr13[10:7]),
	 *.tx_lvl(iomuxc_gpr13[6:2]),
	 *.mpll_ck_off(iomuxc_gpr13[1]),
	 *.tx_edgerate_0(iomuxc_gpr13[0]),
	 */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x07FFFFFD) | 0x0593A044), IOMUXC_GPR13);

	/* enable SATA_PHY PLL */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x2) | 0x2), IOMUXC_GPR13);

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb");
	if (IS_ERR(clk)) {
		dev_err(dev, "no ahb clock.\n");
		ret = PTR_ERR(clk);
		goto release_sata_clk;
	}
	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

	ret = sata_init(addr, tmpdata);
	if (ret == 0)
		return ret;

release_sata_clk:
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);
	/* Disable SATA PWR CTRL_0 of MAX7310 */
	gpio_request(MX6_PHYFLEX_MAX7310_1_BASE_ADDR, "SATA_PWR_EN");
	gpio_direction_output(MX6_PHYFLEX_MAX7310_1_BASE_ADDR, 0);

	return ret;
}

static void mx6_phyflex_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);

	/* Disable SATA PWR CTRL_0 of MAX7310 */
	gpio_request(MX6_PHYFLEX_MAX7310_1_BASE_ADDR, "SATA_PWR_EN");
	gpio_direction_output(MX6_PHYFLEX_MAX7310_1_BASE_ADDR, 0);

}

static struct ahci_platform_data mx6_phyflex_sata_data = {
	.init	= mx6_phyflex_sata_init,
	.exit	= mx6_phyflex_sata_exit,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits	= 4,
	.clk_map_ver	= 2,
};

static struct ipuv3_fb_platform_data phyflex_fb_data[] = {
    {
        .disp_dev		= "ldb",
        .interface_pix_fmt	= IPU_PIX_FMT_RGB666,
        .mode_str		= "LDB-VGA",
        .default_bpp		= 16,
        .int_clk		= false,
    },
    {
        .disp_dev		= "ldb",
        .interface_pix_fmt	= IPU_PIX_FMT_RGB666,
        .mode_str		= "LDB-VGA",
        .default_bpp		= 16,
        .int_clk		= false,
    },
#ifdef ENABLE_HDMI
    {
        .disp_dev = "hdmi",
        .interface_pix_fmt = IPU_PIX_FMT_RGB24,
        .mode_str = "1280x1024M@60",
        .default_bpp = 16,
        .int_clk = false,
    },
#endif
};

#ifdef ENABLE_HDMI
static void hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;
	int max_ipu_id = cpu_is_mx6q() ? 1 : 0;

	if ((ipu_id > max_ipu_id) || (ipu_id < 0)) {
		pr_err("Invalid IPU select for HDMI: %d. Set to 0\n", ipu_id);
		ipu_id = 0;
	}

	if ((disp_id > 1) || (disp_id < 0)) {
		pr_err("Invalid DI select for HDMI: %d. Set to 0\n", disp_id);
		disp_id = 0;
	}

	/* Configure the connection between IPU1/2 and HDMI */
	hdmi_mux_setting = 2 * ipu_id + disp_id;

	/* GPR3, bits 2-3 = HDMI_MUX_CTL */
	mxc_iomux_set_gpr_register(3, 2, 2, hdmi_mux_setting);
}

static struct fsl_mxc_hdmi_platform_data hdmi_data = {
	.init		= hdmi_init,
};

static struct fsl_mxc_hdmi_core_platform_data hdmi_core_data = {
	.ipu_id		= 1,
	.disp_id	= 1,
};
#endif

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id		= 0,
	.disp_id	= 0,
	.ext_ref	= 1,
	//.mode		= LDB_SIN0,
#ifdef ENABLE_HDMI
	.mode		= LDB_DUL_DI0,
#else
	.mode		= LDB_SEP0,
#endif
	.sec_ipu_id	= 1,
	.sec_disp_id	= 1,
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
	.rev		= 4,
	.csi_clk[0]	= "clko_clk",
	}, {
	.rev		= 4,
	.csi_clk[0]	= "clko_clk",
	},
};

static struct platform_pwm_backlight_data mx6_phyflex_pwm_backlight_data0 = {
	.pwm_id		= 0,
	.max_brightness	= 255,
	.dft_brightness	= 128,
	.pwm_period_ns	= 50000,
};
static struct platform_pwm_backlight_data mx6_phyflex_pwm_backlight_data1 = {
	.pwm_id		= 1,
	.max_brightness	= 255,
	.dft_brightness	= 128,
	.pwm_period_ns	= 50000,
};


static const struct flexcan_platform_data mx6_phyflex_flexcan0_pdata __initconst = {
	/* PhyFlex board don't switch transivers */
	.transceiver_switch = NULL,
};

static void phyflex_suspend_enter(void)
{
	/* suspend preparation */
}

static void phyflex_suspend_exit(void)
{
	/* resmue resore */
}

static const struct pm_platform_data mx6_phyflex_pm_data __initconst = {
	.name		= "imx_pm",
	.suspend_enter	= phyflex_suspend_enter,
	.suspend_exit	= phyflex_suspend_exit,
};


#ifdef CONFIG_SND_SOC_IMX_TLV320AIC3007

static struct regulator_consumer_supply tlv320aic3007_phyflex_consumer_iovdd = {
	.supply		= "IOVDD",
	.dev_name	= "1-0018",
};
static struct regulator_consumer_supply tlv320aic3007_phyflex_consumer_dvdd = {
	.supply		= "DVDD",
	.dev_name	= "1-0018",
};
static struct regulator_consumer_supply tlv320aic3007_phyflex_consumer_avdd = {
	.supply		= "AVDD",
	.dev_name	= "1-0018",
};
static struct regulator_consumer_supply tlv320aic3007_phyflex_consumer_drvdd = {
	.supply		= "DRVDD",
	.dev_name	= "1-0018",
};

static struct regulator_init_data tlv320aic3007_phyflex_iovdd_reg_initdata = {
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &tlv320aic3007_phyflex_consumer_iovdd,
};
static struct regulator_init_data tlv320aic3007_phyflex_dvdd_reg_initdata = {
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &tlv320aic3007_phyflex_consumer_dvdd,
};
static struct regulator_init_data tlv320aic3007_phyflex_avdd_reg_initdata = {
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &tlv320aic3007_phyflex_consumer_avdd,
};
static struct regulator_init_data tlv320aic3007_phyflex_drvdd_reg_initdata = {
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &tlv320aic3007_phyflex_consumer_drvdd,
};

static struct fixed_voltage_config tlv320aic3007_phyflex_iovdd_reg_config = {
	.supply_name	= "IOVDD",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &tlv320aic3007_phyflex_iovdd_reg_initdata,
};
static struct fixed_voltage_config tlv320aic3007_phyflex_dvdd_reg_config = {
	.supply_name	= "DVDD",
	.microvolts		= 1800000,
	.gpio			= -1,
	.init_data		= &tlv320aic3007_phyflex_dvdd_reg_initdata,
};
static struct fixed_voltage_config tlv320aic3007_phyflex_avdd_reg_config = {
	.supply_name	= "AVDD",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &tlv320aic3007_phyflex_avdd_reg_initdata,
};
static struct fixed_voltage_config tlv320aic3007_phyflex_drvdd_reg_config = {
	.supply_name	= "DRVDD",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &tlv320aic3007_phyflex_drvdd_reg_initdata,
};

static struct platform_device tlv320aic3007_phyflex_iovdd_reg_devices = {
	.name = "reg-fixed-voltage",
	.id	  = 8,
	.dev  = {
		.platform_data = &tlv320aic3007_phyflex_iovdd_reg_config,
	},
};
static struct platform_device tlv320aic3007_phyflex_dvdd_reg_devices = {
	.name = "reg-fixed-voltage",
	.id   = 9,
	.dev  = {
		.platform_data = &tlv320aic3007_phyflex_dvdd_reg_config,
	},
};
static struct platform_device tlv320aic3007_phyflex_avdd_reg_devices = {
	.name = "reg-fixed-voltage",
	.id	  = 10,
	.dev  = {
		.platform_data = &tlv320aic3007_phyflex_avdd_reg_config,
	},
};
static struct platform_device tlv320aic3007_phyflex_drvdd_reg_devices = {
	.name = "reg-fixed-voltage",
	.id	  = 11,
	.dev  = {
		.platform_data = &tlv320aic3007_phyflex_drvdd_reg_config,
	},
};

#endif /* CONFIG_SND_SOC_TLV320AIC3007 */

static struct regulator_consumer_supply phyflex_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data phyflex_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(phyflex_vmmc_consumers),
	.consumer_supplies = phyflex_vmmc_consumers,
};

static struct fixed_voltage_config phyflex_vmmc_reg_config = {
	.supply_name	= "vmmc",
	.microvolts	= 3300000,
	.gpio		= -1,
	.init_data	= &phyflex_vmmc_init,
};

static struct platform_device phyflex_vmmc_reg_devices = {
	.name		= "reg-fixed-voltage",
	.id		= 0,
	.dev		= {
		.platform_data = &phyflex_vmmc_reg_config,
	},
};

static struct mxc_audio_platform_data mx6_phyflex_audio_data;

static struct imx_ssi_platform_data mx6_phyflex_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct mxc_audio_platform_data mx6_phyflex_audio_data = {
	.ssi_num = 2,
	.src_port = 2,
	.ext_port = 5,
	.hp_gpio = -1,
};

static struct platform_device mx6_phyflex_audio_device = {
	.name = "tlv320aic3007",
};

static int __init mx6_phyflex_init_audio(void)
{
	if (HDA_ON) {
		/* SSI audio init part */
		mxc_register_device(&mx6_phyflex_audio_device,
                        &mx6_phyflex_audio_data);
		imx6q_add_imx_ssi(1, &mx6_phyflex_ssi_pdata);

#ifdef CONFIG_SND_SOC_IMX_TLV320AIC3007
		platform_device_register(&tlv320aic3007_phyflex_iovdd_reg_devices);
		platform_device_register(&tlv320aic3007_phyflex_dvdd_reg_devices);
		platform_device_register(&tlv320aic3007_phyflex_avdd_reg_devices);
		platform_device_register(&tlv320aic3007_phyflex_drvdd_reg_devices);
#endif
	} else {
		// TODO: insert code for wm9712 audio
	}

	return 0;
}

#if 0
static int __init early_use_esai_record(char *p)
{
	esai_record = 1;
	return 0;
}

early_param("esai_record", early_use_esai_record);

static struct mxc_mlb_platform_data mx6_phyflex_mlb150_data = {
	.reg_nvcc		= NULL,
	.mlb_clk		= "mlb150_clk",
	.mlb_pll_clk		= "pll6",
};
#endif


static struct mxc_dvfs_platform_data phyflex_dvfscore_data = {
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

static void __init mx6_phyflex_fixup(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	// Specify manually addressing for first memory bank
	mi->nr_banks=1;
	mi->bank[0].start = 0x10000000;
#ifdef DDR_2GB
	mi->bank[0].size = SZ_2G;
#else
	mi->bank[0].size = SZ_1G;
#endif
	mi->bank[0].highmem = 0;
}

#if 0
static int __init early_enable_sgtl5000(char *p)
{
	sgtl5000_en = 1;
	return 0;
}

early_param("sgtl5000", early_enable_sgtl5000);

static int __init early_enable_spdif(char *p)
{
	spdif_en = 1;
	return 0;
}

early_param("spdif", early_enable_spdif);

static int spdif_clk_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long rate_actual;
	rate_actual = clk_round_rate(clk, rate);
	clk_set_rate(clk, rate_actual);
	return 0;
}

static struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx		= 1,		/* enable tx */
	.spdif_rx		= 1,		/* enable rx */
	/*
	 * spdif0_clk will be 454.7MHz divided by ccm dividers.
	 *
	 * 44.1KHz: 454.7MHz / 7 (ccm) / 23 (spdif) = 44,128 Hz ~ 0.06% error
	 * 48KHz:   454.7MHz / 4 (ccm) / 37 (spdif) = 48,004 Hz ~ 0.01% error
	 * 32KHz:   454.7MHz / 6 (ccm) / 37 (spdif) = 32,003 Hz ~ 0.01% error
	 */
	.spdif_clk_44100	= 1,    /* tx clk from spdif0_clk_root */
	.spdif_clk_48000	= 1,    /* tx clk from spdif0_clk_root */
	.spdif_div_44100	= 23,
	.spdif_div_48000	= 37,
	.spdif_div_32000	= 37,
	.spdif_rx_clk		= 0,    /* rx clk from spdif stream */
	.spdif_clk_set_rate	= spdif_clk_set_rate,
	.spdif_clk		= NULL, /* spdif bus clk */
};
#endif

static const struct imx_pcie_platform_data mx6_phyflex_pcie_data  __initconst = {
//	.pcie_pwr_en    = MX6_ARM2_PCIE_PWR_EN,
//        .pcie_rst       = MX6_ARM2_PCIE_RESET,
        .pcie_wake_up   = -EINVAL,
        .pcie_dis       = -EINVAL,
};


static struct mcp251x_platform_data mcp251x_info = {
	.oscillator_frequency = 24*1000*1000,
};

static struct spi_board_info mcp251x_board_info[] = {
	{
		.modalias	= "mcp2515",
		.platform_data	= &mcp251x_info,
		.max_speed_hz	= 2*1000*1000,
		.bus_num	= 2,
		.mode		= SPI_MODE_0,
		.chip_select	= 1,
		.irq		= gpio_to_irq(MX6_PHYCARD_PEB1_INT),
	},
};


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




static struct fsl_mxc_capture_platform_data capture_data[] = {
	{
		.csi = 0,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 0,
	}, 
};
/* Camera CSI 0 */
static struct i2c_board_info mt9_i2c = {
	I2C_BOARD_INFO("mt9m111", 0x48),
};
static struct soc_camera_link mt9_iclink = {
	.bus_id                 = 0, /* Must match with the camera ID */
	.board_info             = &mt9_i2c,
	.i2c_adapter_id         = 2,
	.module_name		= "mt9m111",
};
static struct platform_device mt9_camera = {
	.name   = "soc-camera-pdrv",
	.id     = 0,
	.dev    = {
		.platform_data = &mt9_iclink,
	},
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
/*
	Uncoment next lines, when second TS interupt fixed
	}, {
		.name                   = "usr2",
		.default_trigger        = "default-off",
		.gpio                   = IMX_GPIO_NR(2, 24),
*/
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

/*!
 * Board specific initialization.
 */
static void __init mx6_phyflex_init(void)
{
	int i;

	/* Setup system revision to imx6 cpu revision */
	system_rev = mx6q_revision();

	/* ToDo: common_pads for both PhyFlex and PhyCard.
	 *All differents pads must initialised separated
	 */
	mxc_iomux_v3_setup_multiple_pads(mx6q_phytec_common_pads, ARRAY_SIZE(mx6q_phytec_common_pads));

	/* Init GPIO Led's */
	platform_device_register(&leds_gpio);

	/* Detecting HDA audio and STMPE811 touchscreen
	 * or WM9712 AC97 audio and touchscreen */
	gpio_request_one(MX6_PHYCARD_AC97_INT, GPIOF_IN, "Audio Detect");
	if (gpio_get_value(MX6_PHYCARD_AC97_INT)) {
		HDA_ON = true;
	} else {
		HDA_ON = false;
	}

	gp_reg_id = phyflex_dvfscore_data.reg_id;
	mx6_phyflex_init_uart();

	imx6q_add_ipuv3(0, &ipu_data[0]);
	for (i = 0; i < ARRAY_SIZE(phyflex_fb_data); i++)
		imx6q_add_ipuv3fb(i, &phyflex_fb_data[i]);

	imx6q_add_ldb(&ldb_data);

	imx6q_add_imx_snvs_rtc();
	imx6q_add_imx_i2c(0, &mx6_phyflex_i2c0_data);
	imx6q_add_imx_i2c(1, &mx6_phyflex_i2c1_data);
	i2c_register_board_info(0, mxc_i2c0_board_info, ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info, ARRAY_SIZE(mxc_i2c1_board_info));
	if (HDA_ON) {
		i2c_register_board_info(1, mxc_i2c1_board_info_hda, ARRAY_SIZE(mxc_i2c1_board_info_hda));
	}

	/* SPI */
	imx6q_add_ecspi(2, &mx6_phyflex_spi_data);

#ifdef ENABLE_PHY
	imx6_init_fec(fec_data);
#endif /* ENABLE_PHY */

	imx6q_add_anatop_thermal_imx(1, &mx6_phyflex_anatop_thermal_data);

	imx6q_add_pm_imx(0, &mx6_phyflex_pm_data);
	board_esdhc_init(2, MX6_PHYFLEX_SD3_CD, MX6_PHYFLEX_SD3_WP);

	imx_add_viv_gpu(&imx6_gpu_data, &imx6_gpu_pdata);
	imx6q_add_vpu();
	mx6_phyflex_init_usb();
	mx6_phyflex_init_audio();

	platform_device_register(&phyflex_vmmc_reg_devices);
	mx6_cpu_regulator_init();

#if 0
	/* DISP0 Reset - Assert for i2c disabled mode */
	gpio_request(MX6_PHYFLEX_DISP0_RESET, "disp0-reset");
	gpio_direction_output(MX6_PHYFLEX_DISP0_RESET, 0);

	/* DISP0 I2C enable */
	gpio_request(MX6_PHYFLEX_DISP0_I2C_EN, "disp0-i2c");
	gpio_direction_output(MX6_PHYFLEX_DISP0_I2C_EN, 0);

	gpio_request(MX6_PHYFLEX_DISP0_PWR, "disp0-pwr");
	gpio_direction_output(MX6_PHYFLEX_DISP0_PWR, 1);
#endif

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();
	board_nand_init();

	imx6q_add_dvfs_core(&phyflex_dvfscore_data);

#if defined(CONFIG_CAN_MCP251X)
	spi_register_board_info(mcp251x_board_info, ARRAY_SIZE(mcp251x_board_info));
#endif

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

static void __init mx6_phyflex_reserve(void)
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
	.fixup		= mx6_phyflex_fixup,
	.map_io		= mx6_map_io,
	.init_irq	= mx6_init_irq,
	.init_machine	= mx6_phyflex_init,
	.timer		= &mxc_timer,
	.reserve	= mx6_phyflex_reserve,
MACHINE_END
