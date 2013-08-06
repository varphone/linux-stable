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
#include <linux/i2c/at24.h>
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
#include <linux/input/edt-ft5x06.h>
#include <linux/spi/max7301.h> 
#include <linux/can/platform/mcp251x.h> 
#include <sound/tlv320aic3x.h>
#include <media/tw9910.h>

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
#include <mach/mxc_camera.h>

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
#include "board-mx6q_phyflex.h"

#include "board-mx6q_phytec-common.h"
#include "board-mx6q_phytec-sd.h"
#include "board-mx6q_phytec-nand.h"
#include "board-mx6q_phytec-pmic.h"

#define MX6_PHYFLEX_SD3_CD		IMX_GPIO_NR(1, 27)
#define MX6_PHYFLEX_SD3_WP		IMX_GPIO_NR(1, 29)
#define MX6_PHYFLEX_SD2_CD		IMX_GPIO_NR(1, 4)
#define MX6_PHYFLEX_SD2_WP		IMX_GPIO_NR(1, 2)

/* GPIO PIN, sort by PORT/BIT */
#define MX6_PHYFLEX_CAM0_LVDS_PWRDN	IMX_GPIO_NR(1, 24)
#define MX6_PHYFLEX_CAM0_OE		IMX_GPIO_NR(5, 20)
#define MX6_PHYFLEX_CAM1_LVDS_PWRDN	IMX_GPIO_NR(2, 28)
#define MX6_PHYFLEX_CAM1_OE		IMX_GPIO_NR(3, 10)

#define MX6_PHYFLEX_LDB0_BACKLIGHT      IMX_GPIO_NR(1, 8)
#define MX6_PHYFLEX_LDB1_BACKLIGHT	IMX_GPIO_NR(2, 25) // MX6Q_PAD_EIM_OE__GPIO_2_25

#define MX6_PHYFLEX_ECSPI3_CS0		IMX_GPIO_NR(4, 24)
#define MX6_PHYFLEX_ECSPI3_CS1		IMX_GPIO_NR(4, 25)
#define MX6_PHYFLEX_ECSPI3_CS2		IMX_GPIO_NR(4, 26)
#define MX6_PHYFLEX_ECSPI3_CS3		IMX_GPIO_NR(4, 27)
#define MX6_PHYFLEX_ECSPI3_WP		IMX_GPIO_NR(3, 29)

#define MX6_PHYFLEX_LED_GREEN		IMX_GPIO_NR(1, 30)
#define MX6_PHYFLEX_LED_RED		IMX_GPIO_NR(2, 31)

#define MX6_PHYFLEX_USB_OTG_PWR		IMX_GPIO_NR(4, 15)
//#define MX6_PHYFLEX_DISP0_PWR		IMX_GPIO_NR(3, 24)
#define MX6_PHYFLEX_DISP0_I2C_EN	IMX_GPIO_NR(3, 28)

#define MX6_PHYFLEX_USB_HOST1_PWR	IMX_GPIO_NR(1, 0)
#define MX6_PHYFLEX_USB_HOST1_OC	IMX_GPIO_NR(1, 3)

#define MX6_PHYCARD_CAP_TCH_INT0	IMX_GPIO_NR(4, 29)

#define MX6_PHYFLEX_CAP_TCH_INT0	IMX_GPIO_NR(5, 8)
#define MX6_PHYFLEX_CAP_TCH_INT1	IMX_GPIO_NR(5, 7)
#define MX6_PHYFLEX_KAPA_TOUCH_INT0	IMX_GPIO_NR(7, 12)
 
#define MX6_PHYFLEX_DISP0_DET_INT	IMX_GPIO_NR(3, 31)
#define MX6_PHYFLEX_CSI0_RST		IMX_GPIO_NR(4, 5)
// #define MX6_PHYFLEX_CSI0_PWN		IMX_GPIO_NR(5, 23)
// #define MX6_PHYFLEX_CAN2_EN		IMX_GPIO_NR(5, 24)
#define MX6_PHYFLEX_CSI0_RST_TVIN	IMX_GPIO_NR(5, 25)
#define MX6_PHYFLEX_MAX7310_1_BASE_ADDR	IMX_GPIO_NR(8, 0)
#define MX6_PHYFLEX_PCA9538_BASE_ADDR	IMX_GPIO_NR(8, 8)

// #define MX6_PHYFLEX_IO_EXP_GPIO1(x)	(MX6_PHYFLEX_MAX7310_1_BASE_ADDR + (x))
// #define MX6_PHYFLEX_IO_EXP_GPIO2(x)	(MX6_PHYFLEX_MAX7310_2_BASE_ADDR + (x))
// #define MX6_PHYFLEX_CAN2_STBY		MX6_PHYFLEX_IO_EXP_GPIO2(1)
// #define BMCR_PDOWN			0x0800 /* PHY Powerdown */

#define MX6_PHYFLEX_HW_INTRO		IMX_GPIO_NR(3, 20)

#define ENABLE_HDMI
#define ENABLE_PHY

static char* csi0 = "phyCAM-P";
module_param(csi0, charp, S_IRUGO);

static char* csi1 = "phyCAM-P";
module_param(csi1, charp, S_IRUGO);

void __init early_console_setup(unsigned long base, struct clk *clk);
static struct clk *sata_clk;

extern struct regulator *(*get_cpu_regulator)(void);
extern void (*put_cpu_regulator)(void);
extern void mx6_cpu_regulator_init(void);

extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;

extern int module_rev;

static struct anatop_thermal_platform_data
	mx6_phyflex_anatop_thermal_data __initconst = {
	.name = "anatop_thermal",
};

static inline void mx6_phyflex_init_uart(void)
{
	imx6q_add_imx_uart(2, NULL);
	imx6q_add_imx_uart(3, NULL);
}

#ifdef ENABLE_PHY
static int mx6_phyflex_fec_phy_init(struct phy_device *phydev)
{
	/* from current linux-imx6, arch/arm/mach-imx/mach-imx6q.c, ksz9021rn_phy_fixup(): */
	/* min rx data delay */

	printk("FEC ID: 0x%X, 0x%X\n", phy_read(phydev, 0x02), phy_read(phydev, 0x03));


	/* enable all interrupts */
	phy_write(phydev, 0x1b, 0xff00);

	return 0;
}

#define PHY_POWERDOWN 	(1 << 11)
static int mx6_phyflex_fec_power_hibernate(struct phy_device *phydev)
{
	unsigned short val;

	/*set ksz9021rn reg 0x0 bit 11 to hibernate power*/
	val = phy_read(phydev, 0x0);

	val |= PHY_POWERDOWN;
	phy_write(phydev, 0x0, val);

	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init			= mx6_phyflex_fec_phy_init,
	.power_hibernate	= mx6_phyflex_fec_power_hibernate,
	.phy			= PHY_INTERFACE_MODE_RGMII,
};
#endif /* ENABLE_PHY */

static int mx6_phyflex_spi_cs[] = {
	MX6_PHYFLEX_ECSPI3_CS0,
	MX6_PHYFLEX_ECSPI3_CS1,
	MX6_PHYFLEX_ECSPI3_CS2,
	MX6_PHYFLEX_ECSPI3_CS3,
};

static struct spi_imx_master mx6_phyflex_spi_data __initconst = {
	.chipselect     = mx6_phyflex_spi_cs,
	.num_chipselect = ARRAY_SIZE(mx6_phyflex_spi_cs),
};


#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
static struct flash_platform_data n25q128_spi_flash_data = {
	.name		= "n25q128",
	.type		= "n25q128",
};
#endif


static struct spi_board_info n25q128_spi2_board_info[] __initdata = {
#if defined(CONFIG_MTD_M25P80)
	{
	/* The modalias must be the same as spi device driver name */
	.modalias	= "m25p80",
	.max_speed_hz	= 20000000,
	.bus_num	= 2,
	.chip_select	= 0,
	.platform_data	= &n25q128_spi_flash_data,
	},
#endif
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

static struct stmpe_ts_platform_data stmpe811_ts_data1 = {
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

static struct stmpe_platform_data stmpe811_data1 = {
        .id             = 1,
        .blocks         = STMPE_BLOCK_TOUCHSCREEN,
        .ts 		= &stmpe811_ts_data1,
};

static struct at24_platform_data at24c32 = {
	.byte_len	= SZ_32K / 8,
	.page_size	= 8,
	.flags		= AT24_FLAG_ADDR16,
};

static struct at24_platform_data at24c04 = {
	.byte_len	= SZ_4K / 8,
	.page_size	= 16,
	.flags		= 0,
};

static struct max7301_platform_data max7301_i2c_data = {
	.base		= -1,
};

static struct pca9532_platform_data user_leds_data = {
	.leds = {
			{
				.name = "user_led_0",
				.state = PCA9532_OFF,
				.type = PCA9532_TYPE_LED,
			}, {
				.name = "user_led_1",
				.state = PCA9532_ON,
				.type = PCA9532_TYPE_LED,
			}, {
				.name = "user_led_2",
				.state = PCA9532_ON,
				.type = PCA9532_TYPE_LED,
			}, {
				.name = "user_led_3",
				.state = PCA9532_OFF,
				.type = PCA9532_TYPE_LED,
			},
		},
	.psc = { 0, 0 },
	.pwm = { 0, 0 },
};

static struct edt_ft5x06_platform_data mx6_phyflex_ft5x06_data = {
	.reset_pin	= -1,   /* static high */
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
	.gpio_base	= MX6_PHYFLEX_PCA9538_BASE_ADDR,
	.invert		= 0,
	.setup		= pca9538_setup,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("max7300", 0x40),
		.platform_data = &max7301_i2c_data,
	},{
		I2C_BOARD_INFO("24c32", 0x50),
		.platform_data = &at24c32,
	},
};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("max1037", 0x64),
	}, {
		I2C_BOARD_INFO("tlv320aic3007", 0x18),
	}, {
		I2C_BOARD_INFO("rtc8564", 0x51),
	}, {
		I2C_BOARD_INFO("pca9533", 0x62),
		.platform_data = &user_leds_data,
	}, {
		I2C_BOARD_INFO("stmpe811", 0x41),
		.irq = gpio_to_irq(MX6_PHYFLEX_CAP_TCH_INT0),
		.platform_data = (void *)&stmpe811_data0,
	}, {
		I2C_BOARD_INFO("edt-ft5x06", 0x38),
		.irq = gpio_to_irq(MX6_PHYFLEX_KAPA_TOUCH_INT0),
		.platform_data = (void *)&mx6_phyflex_ft5x06_data,
	},
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
	{
/*		I2C_BOARD_INFO("max1037", 0x64),
	} , {*/
		I2C_BOARD_INFO("stmpe811", 0x41),
		.irq = gpio_to_irq(MX6_PHYFLEX_CAP_TCH_INT1),
		.platform_data = (void *)&stmpe811_data1,
	}, {
		I2C_BOARD_INFO("pca9538", 0x70),
		.platform_data = &pca9538_platdata,
	}, {
		I2C_BOARD_INFO("24c04", 0x52),
		.platform_data = &at24c04,
	},
};


static struct i2c_board_info __initdata hdmi_i2c_data[] = {
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

	ret = gpio_request_one(MX6_PHYFLEX_USB_OTG_PWR, GPIOF_OUT_INIT_LOW, "usb-pwr");
	gpio_request(MX6_PHYFLEX_USB_OTG_PWR, "usb-pwr");
	if (ret) {
		pr_err("failed to get GPIO MX6_PHYFLEX_USB_OTG_PWR:%d\n", ret);
		return;
	}

	mxc_iomux_set_gpr_register(1, 13, 1, 1);

	/* TODO: uncoment mx6_set_otghost_vbus_func
	* when usb pwr hardware fixes are available
	*/
	// mx6_set_otghost_vbus_func(imx6_phyflex_usbotg_vbus);

	/* TODO: remove next line when usb pwr are fixed.
	* Turn on VBUS power for USB.
	*/
	imx6_phyflex_usbotg_vbus(true);

/*	mx6_usb_dr_init(); */

	/* Enable USB hub power */
	ret = gpio_request(MX6_PHYFLEX_USB_HOST1_PWR, "usb-host1-pwr");
	if (ret) {
		pr_err("failed to get GPIO MX6_PHYFLEX_USB_OTG_PWR:%d\n", ret);
		return;
	}
	gpio_direction_output(MX6_PHYFLEX_USB_HOST1_PWR, 1); // Config PWR USB pin
	
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
	gpio_request_one(MX6_PHYFLEX_MAX7310_1_BASE_ADDR, GPIOF_OUT_INIT_HIGH, "SATA_PWR_EN");

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

#ifdef CONFIG_SATA_AHCI_PLATFORM
	ret = sata_init(addr, tmpdata);
	if (ret == 0)
		return ret;
#else
	usleep_range(1000, 2000);
	/* AHCI PHY enter into PDDQ mode if the AHCI module is not enabled */
	tmpdata = readl(addr + PORT_PHY_CTL);
	writel(tmpdata | PORT_PHY_CTL_PDDQ_LOC, addr + PORT_PHY_CTL);
	pr_info("No AHCI save PWR: PDDQ %s\n", ((readl(addr + PORT_PHY_CTL)
					>> 20) & 1) ? "enabled" : "disabled");
#endif

release_sata_clk:
	/* disable SATA_PHY PLL */
	writel((readl(IOMUXC_GPR13) & ~0x2), IOMUXC_GPR13);
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);
	/* Disable SATA PWR CTRL_0 of MAX7310 */
	gpio_request_one(MX6_PHYFLEX_MAX7310_1_BASE_ADDR, GPIOF_OUT_INIT_LOW, "SATA_PWR_EN");

	return ret;
}

#ifdef CONFIG_SATA_AHCI_PLATFORM
static void mx6_phyflex_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);

	/* Disable SATA PWR CTRL_0 of MAX7310 */
	gpio_request_one(MX6_PHYFLEX_MAX7310_1_BASE_ADDR, GPIOF_OUT_INIT_LOW, "SATA_PWR_EN");
}

static struct ahci_platform_data mx6_phyflex_sata_data = {
	.init	= mx6_phyflex_sata_init,
	.exit	= mx6_phyflex_sata_exit,
};
#endif

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
#ifndef ENABLE_HDMI
    {
        .disp_dev		= "ldb",
        .interface_pix_fmt	= IPU_PIX_FMT_RGB666,
        .mode_str		= "LDB-VGA",
        .default_bpp		= 16,
        .int_clk		= false,
    },
#else
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
		/* 3.5 inch display */
		.name         = "Primeview-PD035VL1",
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
#ifdef ENABLE_HDMI
	.mode		= LDB_DUL_DI0,
#else
	.mode		= LDB_SEP0,
#endif
	.sec_ipu_id	= 1,
	.sec_disp_id	= 1,
	.modes		= ldb_modedb,
	.num_modes	= ARRAY_SIZE(ldb_modedb),
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
	.rev		= 4,
	.csi_clk[0]	= "clko_clk",
	}, {
	.rev		= 4,
	.csi_clk[1]	= "clko2_clk",
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


static struct flexcan_platform_data mx6_phyflex_flexcan0_pdata __initconst = {
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

static struct pm_platform_data mx6_phyflex_pm_data __initconst = {
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
	return 0;
}

static struct mxc_dvfs_platform_data phyflex_dvfscore_data = {
	.reg_id                 = "VDDCORE",    // DA9063 regulator for vddcpu
	.soc_id                 = "VDDSOC",     // DA9063 regulator for vddsoc
	.clk1_id                = "cpu_clk",
	.clk2_id                = "gpc_dvfs_clk",
	.gpc_cntr_offset        = MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset        = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset       = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset      = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask            = 0x1F800,
	.prediv_offset          = 11,
	.prediv_val             = 3,
	.div3ck_mask            = 0xE0000000,
	.div3ck_offset          = 29,
	.div3ck_val             = 2,
	.emac_val               = 0x08,
	.upthr_val              = 25,
	.dnthr_val              = 9,
	.pncthr_val             = 33,
	.upcnt_val              = 10,
	.dncnt_val              = 10,
	.delay_time             = 80,
};

static void __init mx6_phyflex_fixup(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
}


static struct imx_pcie_platform_data mx6_phyflex_pcie_data  __initconst = {
	.pcie_rst       = IMX_GPIO_NR(2, 23),
	.pcie_pwr_en	= -EINVAL,
	.pcie_wake_up   = IMX_GPIO_NR(1, 7),
        .pcie_dis       = -EINVAL,
};

/*
 * Dallas 1-Wire GPIO interface
 */
static struct w1_gpio_platform_data w1_gpio_pdata = {
	.pin			= MX6_PHYFLEX_HW_INTRO,
	.is_open_drain		= 0,
	.enable_external_pullup	= NULL,
};

static struct platform_device w1_device = {
	.name			= "w1-gpio",
	.id			= -1,
	.dev.platform_data	= &w1_gpio_pdata,
};

#ifdef CONFIG_SOC_CAMERA
static struct mxc_camera_pdata mxc_ipu_csi_pdata[] = {
	{
		.flags = MXC_CAMERA_DATAWIDTH_10,
		.ipu = 0,
		.csi = 0,
		.mclk_default_rate = 27000000,
		.mclk_target_rate = 60000000,	//only for mt9p031
		.use_pll = 0,					//only for mt9p031
	}, {
		.flags = MXC_CAMERA_DATAWIDTH_10,
		.ipu = 1,
		.csi = 1,
		.mclk_default_rate = 27000000,
		.mclk_target_rate = 60000000,	//only for mt9p031
		.use_pll = 0,					//only for mt9p031
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
	}, {
		.name   = "mxc-camera",
		.id     = 1,
		.dev    = {
			.platform_data = &mxc_ipu_csi_pdata[1],
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
			gpio_direction_output(MX6_PHYFLEX_PCA9538_BASE_ADDR
						+ i * 2, 0);
			gpio_direction_output(MX6_PHYFLEX_PCA9538_BASE_ADDR
						+ i * 2 + 1, 0);
		} else {
			gpio_direction_output(MX6_PHYFLEX_PCA9538_BASE_ADDR
						+ i * 2, 1);
			gpio_direction_output(MX6_PHYFLEX_PCA9538_BASE_ADDR
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

#else
static void mx6_csi0_io_init(void)
{
	printk("::: mx6_csi0_io_init\n");
	mxc_iomux_set_gpr_register(1, 19, 2, 1);
}

static struct fsl_mxc_capture_platform_data capture_data[] = {
	[0] = {
		.csi = 0,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 0,
	},
	[1] = {
		.csi = 1,
		.ipu = 1,
		.mclk_source = 0,
		.is_mipi = 0,
	},
};

static struct fsl_mxc_camera_platform_data camera_data = {
	.mclk		= 26000000,
	.mclk_source	= 0,
	.csi		= 0,
	.io_init	= mx6_csi0_io_init,
};

/* Camera CSI 0 */
static struct i2c_board_info camera_i2c[] = {
	{
		I2C_BOARD_INFO("mt9m111", 0x48),
		.platform_data = (void *)&camera_data,
	}, {
		I2C_BOARD_INFO("mt9m001", 0x5d),
		.platform_data = (void *)&camera_data,
	},
};
#endif

static struct gpio_led gpio_leds[] = {
	{
		.name                   = "green",
		.default_trigger        = "mmc0",
		.gpio                   = MX6_PHYFLEX_LED_GREEN,
	}, {
		.name                   = "red",
		.default_trigger        = "nand-disk",
		.gpio                   = MX6_PHYFLEX_LED_RED,
/*
	TODO: Uncoment next lines, when second phyFlex TS interupt fixed
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

#define SNVS_LPCR 0x38
static void mx6_snvs_poweroff(void)
{
        u32 value;
        void __iomem *mx6_snvs_base = MX6_IO_ADDRESS(MX6Q_SNVS_BASE_ADDR);

	printk(KERN_INFO "Goodbye phyFLEX-i.MX6!\n");

        value = readl(mx6_snvs_base + SNVS_LPCR);
        /* set TOP and DP_EN bit */
        writel(value | 0x60, mx6_snvs_base + SNVS_LPCR);
}

/*
 * Board specific initialization.
 */
static void __init mx6_phyflex_init(void)
{
	int i;
	
	char* cam_parameter_separator = ",";

    char* csi0_interface_type;
    char* csi0_cam_type;
    char* csi0_cam_address;

    char* csi1_interface_type;
    char* csi1_cam_type;
    char* csi1_cam_address;

	long csi0_cam_address_hex;
	long csi1_cam_address_hex;
	
	/* imx6q SoC revision and CPU uniq ID setup */
	mx6_setup_cpuinfo();

	mxc_iomux_v3_setup_multiple_pads(mx6q_phytec_common_pads, ARRAY_SIZE(mx6q_phytec_common_pads));
	
	if (module_rev == PHYFLEX_MODULE_REV_1) {
		mxc_iomux_v3_setup_multiple_pads(mx6q_phytec_rev1_pads,
						ARRAY_SIZE(mx6q_phytec_rev1_pads));
	} else {
		mxc_iomux_v3_setup_multiple_pads(mx6q_phytec_rev2_pads,
						ARRAY_SIZE(mx6q_phytec_rev2_pads));
		mx6_phyflex_pcie_data.pcie_rst = IMX_GPIO_NR(4, 17);
	}

	pm_power_off = mx6_snvs_poweroff;

	/* Init GPIO Led's */
	platform_device_register(&leds_gpio);

	/* Main Voltage regulators */
	gp_reg_id = phyflex_dvfscore_data.reg_id;
	soc_reg_id = phyflex_dvfscore_data.soc_id;
	pu_reg_id = phyflex_dvfscore_data.pu_id;

	/* Init PMIC */
	mx6_phyflex_init_da9063();

	/* UART initialization*/
	mx6_phyflex_init_uart();

	/* Video devices initialization */
	imx6q_add_ipuv3(0, &ipu_data[0]);
	imx6q_add_ipuv3(1, &ipu_data[1]);
	for (i = 0; i < ARRAY_SIZE(phyflex_fb_data); i++)
		imx6q_add_ipuv3fb(i, &phyflex_fb_data[i]);

	imx6q_add_ldb(&ldb_data);

	imx6q_add_v4l2_output(0);
	
	/***************************************************************************
	Camera section:
	The bootargs csi0 and csi1 will be interpreted. 
	The bootarg csi0 and csi1 have the following structur:
	Interface=<Interface-Type>,<Camera-Type>,<I2C-Address>
	For example: csi0=phyCAM-P,VM-010,0x48	 

	If only the Interface-Type is specified, the default settings will be loaded.
	If at csi0 or csi1 the Camera-Type and I2C-Address is specified, only this camera work.
	****************************************************************************/
        csi0_interface_type = strsep(&csi0, cam_parameter_separator);
        csi0_cam_type = strsep(&csi0, cam_parameter_separator);
        csi0_cam_address = strsep(&csi0, cam_parameter_separator);

	csi1_interface_type = strsep(&csi1, cam_parameter_separator);
        csi1_cam_type = strsep(&csi1, cam_parameter_separator);
        csi1_cam_address = strsep(&csi1, cam_parameter_separator);

	if(csi0_cam_type!=NULL){
		if(strcmp("VM-006",csi0_cam_type)==0){csi0_cam_type="mt9m001";}
  		if(strcmp("VM-008",csi0_cam_type)==0){csi0_cam_type="tw9910";}
		if(strcmp("VM-009",csi0_cam_type)==0){csi0_cam_type="mt9m111";}
		if(strcmp("VM-010",csi0_cam_type)==0){csi0_cam_type="mt9v022";}
		if(strcmp("VM-011",csi0_cam_type)==0){csi0_cam_type="mt9p031";}
	}
        if(csi1_cam_type!=NULL){
            	if(strcmp("VM-006",csi1_cam_type)==0){csi1_cam_type="mt9m001";}
		if(strcmp("VM-008",csi1_cam_type)==0){csi1_cam_type="tw9910";}
        	if(strcmp("VM-009",csi1_cam_type)==0){csi1_cam_type="mt9m111";}
            	if(strcmp("VM-010",csi1_cam_type)==0){csi1_cam_type="mt9v022";}
            	if(strcmp("VM-011",csi1_cam_type)==0){csi1_cam_type="mt9p031";}
        }

#ifdef CONFIG_SOC_CAMERA
#define SOC_CAM_LINK(bus, bi, i2c_adapter) \
.bus_id = bus, .board_info = bi, .i2c_adapter_id = i2c_adapter
#define SOC_CAM_PDRV(dev_id, iclinks) \
.name = "soc-camera-pdrv", .id = dev_id, .dev = { .platform_data = &iclinks[dev_id] }

	mxc_iomux_set_gpr_register(1, 19, 2, 3);
	
	if ((csi0_cam_type == NULL) && (csi1_cam_type == NULL)){	//For default setting.
		static struct i2c_board_info phyflex_cameras[] = {
			[0] = {I2C_BOARD_INFO("mt9m001", 0x5d),},	//DRIVER-NAME and I2C-ADDRESS
            		[1] = {I2C_BOARD_INFO("tw9910", 0x45),},
            		[2] = {I2C_BOARD_INFO("mt9m111", 0x48),},
            		[3] = {I2C_BOARD_INFO("mt9v022", 0x48),},
            		[4] = {I2C_BOARD_INFO("mt9v022", 0x4c),},
			[5] = {I2C_BOARD_INFO("mt9p031", 0x48),},
			[6] = {I2C_BOARD_INFO("mt9p031", 0x5d),},
		};
		static struct soc_camera_link phyflex_iclinks[] = {
	        	{SOC_CAM_LINK(0, &phyflex_cameras[0], 2)}, 	//CSI-PORT,phyflex_cameras,I2C-INTERFACE
			{SOC_CAM_LINK(0, &phyflex_cameras[1], 2),
			.priv=&tw9910_info}, 
			{SOC_CAM_LINK(0, &phyflex_cameras[2], 2)}, 
			{SOC_CAM_LINK(0, &phyflex_cameras[3], 2)}, 
			{SOC_CAM_LINK(1, &phyflex_cameras[4], 2)},
			{SOC_CAM_LINK(0, &phyflex_cameras[5], 2),
			.flags=SOCAM_SENSOR_INVERT_PCLK},
                       {SOC_CAM_LINK(1, &phyflex_cameras[6], 2),
			.flags=SOCAM_SENSOR_INVERT_PCLK},
		};
		static struct platform_device mxc_ipu_cameras[] = {
	        	{SOC_CAM_PDRV(0, phyflex_iclinks),}, 		//DEVICE_ID,phyflex_iclinks
			{SOC_CAM_PDRV(1, phyflex_iclinks),}, 
			{SOC_CAM_PDRV(2, phyflex_iclinks),}, 
			{SOC_CAM_PDRV(3, phyflex_iclinks),}, 
			{SOC_CAM_PDRV(4, phyflex_iclinks),},
			{SOC_CAM_PDRV(5, phyflex_iclinks),},
                        {SOC_CAM_PDRV(6, phyflex_iclinks),},
		};

		if(strcmp(csi1_interface_type,"phyCAM-P")==0){
   			mxc_ipu_csi_pdata[0].flags=MXC_CAMERA_DATAWIDTH_10 | MXC_CAMERA_PCP;
			mxc_ipu_csi_pdata[1].flags=MXC_CAMERA_DATAWIDTH_10 | MXC_CAMERA_PCP;
		}

                for (i = 0; i < ARRAY_SIZE(mxc_ipu_cameras); i++){
                        platform_device_register(&mxc_ipu_cameras[i]);};

                platform_device_register(&mxc_ipu_csi_devices[0]);
                platform_device_register(&mxc_ipu_csi_devices[1]);
	}
	else{		//Only Camera-Type and I2C-Address is set in csi0 or csi1
	static struct i2c_board_info phyflex_cameras[] = {
                        [0] = {I2C_BOARD_INFO("mt9v022", 0x48),},
                        [1] = {I2C_BOARD_INFO("mt9v022", 0x4c),},
                };
                static struct soc_camera_link phyflex_iclinks[] = {
                        {SOC_CAM_LINK(0, &phyflex_cameras[0], 2)},
                        {SOC_CAM_LINK(1, &phyflex_cameras[1], 2)},
                };
                static struct platform_device mxc_ipu_cameras[] = {
                        {SOC_CAM_PDRV(0, phyflex_iclinks),},
                        {SOC_CAM_PDRV(1, phyflex_iclinks),},
                };

		if(csi0_cam_type!=NULL){/* set the max MCLK for the Camera-Type and Interface-Type */
 			if(strcmp(csi0_interface_type,"phyCAM-P")==0) {
        	        	if(strcmp("mt9m001",csi0_cam_type)==0) {
					mxc_ipu_csi_pdata[0].mclk_default_rate = 43200000;
					mxc_ipu_csi_pdata[0].flags = MXC_CAMERA_DATAWIDTH_10 | MXC_CAMERA_PCP;
				}
	                	if(strcmp("tw9910",csi0_cam_type)==0) {
					mxc_ipu_csi_pdata[0].mclk_default_rate = 27000000;
					phyflex_iclinks[0].priv=&tw9910_info;
				}
				if(strcmp("mt9m111",csi0_cam_type)==0) {
					mxc_ipu_csi_pdata[0].mclk_default_rate = 54000000;
				}
                		if(strcmp("mt9v022",csi0_cam_type)==0) {
					mxc_ipu_csi_pdata[0].mclk_default_rate = 27000000;
				}
                		if(strcmp("mt9p031",csi0_cam_type)==0) {
					mxc_ipu_csi_pdata[0].mclk_default_rate = 54000000;
					mxc_ipu_csi_pdata[0].use_pll = 0;
					phyflex_iclinks[0].flags=SOCAM_SENSOR_INVERT_PCLK;
				} /* The PLL in the mt9p031 generated 96 MHZ */
        		} else if(strcmp(csi0_interface_type,"phyCAM-S+")==0) {
        	                if(strcmp("mt9m001",csi0_cam_type)==0) {
					mxc_ipu_csi_pdata[0].mclk_default_rate = 36000000;
				}
	                        if(strcmp("tw9910",csi0_cam_type)==0) {
					mxc_ipu_csi_pdata[0].mclk_default_rate = 27000000;
					phyflex_iclinks[0].priv=&tw9910_info;
				}
				if(strcmp("mt9m111",csi0_cam_type)==0) {
					mxc_ipu_csi_pdata[0].mclk_default_rate = 36000000;
				}
                        	if(strcmp("mt9v022",csi0_cam_type)==0) {
					mxc_ipu_csi_pdata[0].mclk_default_rate = 27000000;
				}
                       		if(strcmp("mt9p031",csi0_cam_type)==0) {
					mxc_ipu_csi_pdata[0].mclk_default_rate = 54000000;
					phyflex_iclinks[0].flags=SOCAM_SENSOR_INVERT_PCLK;
				}
			}
		}

		if(csi1_cam_type!=NULL) {/* set the max MCLK for the Camera-Type and Interface-Type */
                	if(strcmp(csi1_interface_type,"phyCAM-P")==0) {
                        	if(strcmp("mt9m001",csi1_cam_type)==0) {
					mxc_ipu_csi_pdata[1].mclk_default_rate = 43200000;
					mxc_ipu_csi_pdata[1].flags=MXC_CAMERA_DATAWIDTH_10 | MXC_CAMERA_PCP;
				}
                        	if(strcmp("tw9910",csi1_cam_type)==0) {
					mxc_ipu_csi_pdata[1].mclk_default_rate = 27000000;
					phyflex_iclinks[1].priv=&tw9910_info;
				}
				if(strcmp("mt9m111",csi1_cam_type)==0) {
					mxc_ipu_csi_pdata[1].mclk_default_rate = 54000000;
				}
                        	if(strcmp("mt9v022",csi1_cam_type)==0) {
					mxc_ipu_csi_pdata[1].mclk_default_rate = 27000000;
				}
                        	if(strcmp("mt9p031",csi1_cam_type)==0) {
					mxc_ipu_csi_pdata[1].mclk_default_rate = 54000000;
					mxc_ipu_csi_pdata[1].use_pll = 0;
					phyflex_iclinks[1].flags=SOCAM_SENSOR_INVERT_PCLK;
				} /*The PLL in the mt9p031 generated 96 MHZ */
                	}
                	else if(strcmp(csi1_interface_type,"phyCAM-S+")==0) {
                        	if(strcmp("mt9m001",csi1_cam_type)==0) {
					mxc_ipu_csi_pdata[1].mclk_default_rate = 36000000;
				}
                       	 	if(strcmp("tw9910",csi1_cam_type)==0) {
					mxc_ipu_csi_pdata[1].mclk_default_rate = 27000000;
					phyflex_iclinks[1].priv=&tw9910_info;
				}
				if(strcmp("mt9m111",csi1_cam_type)==0) {
					mxc_ipu_csi_pdata[1].mclk_default_rate = 36000000;
				}
                       	 	if(strcmp("mt9v022",csi1_cam_type)==0) {
					mxc_ipu_csi_pdata[1].mclk_default_rate = 27000000;
				}
                       	 	if(strcmp("mt9p031",csi1_cam_type)==0) {
					mxc_ipu_csi_pdata[1].mclk_default_rate = 54000000;
					phyflex_iclinks[1].flags=SOCAM_SENSOR_INVERT_PCLK;
				}
                	}
		}
		if(csi0_cam_type!=NULL){
	        	csi0_cam_address_hex = simple_strtol(csi0_cam_address,NULL,16);
                        strcpy(phyflex_cameras[0].type,csi0_cam_type);
                        phyflex_cameras[0].addr=csi0_cam_address_hex;
			platform_device_register(&mxc_ipu_cameras[0]);
		}
		if(csi1_cam_type!=NULL){
			csi1_cam_address_hex = simple_strtol(csi1_cam_address,NULL,16);
                        strcpy(phyflex_cameras[1].type,csi1_cam_type);
                        phyflex_cameras[1].addr=csi1_cam_address_hex;
			platform_device_register(&mxc_ipu_cameras[1]);
		}
       	 	if(csi0_cam_type!=NULL){
			platform_device_register(&mxc_ipu_csi_devices[0]);
		}	
		if(csi1_cam_type!=NULL){	
        		platform_device_register(&mxc_ipu_csi_devices[1]);
		}
	}
#else
	imx6q_add_v4l2_capture(0, &capture_data[0]);
	imx6q_add_v4l2_capture(1, &capture_data[1]);

	/* Registering cameras */
	i2c_register_board_info(2, camera_i2c, ARRAY_SIZE(camera_i2c));
#endif

	gpio_request(MX6_PHYFLEX_CAM0_LVDS_PWRDN, "CSI0<->LVDS bridge #PWDN");
	gpio_request(MX6_PHYFLEX_CAM0_OE, "IPU1/CSI0 camera #OE");

	gpio_request(MX6_PHYFLEX_CAM1_LVDS_PWRDN, "CSI1<->LVDS bridge #PWDN");
	gpio_request(MX6_PHYFLEX_CAM1_OE, "IPU2/CSI1 camera #OE");

	if (!strcmp("phyCAM-S+", csi0_interface_type)) {
		gpio_direction_output(MX6_PHYFLEX_CAM0_LVDS_PWRDN, 1);
		gpio_direction_output(MX6_PHYFLEX_CAM0_OE, 1);
	} else if (!strcmp("none", csi0_interface_type)) {
		gpio_direction_output(MX6_PHYFLEX_CAM0_LVDS_PWRDN, 0);
		gpio_direction_output(MX6_PHYFLEX_CAM0_OE, 1);
	} else {
		gpio_direction_output(MX6_PHYFLEX_CAM0_LVDS_PWRDN, 0);
		gpio_direction_output(MX6_PHYFLEX_CAM0_OE, 0);
	}
	
	if (!strcmp("phyCAM-S+", csi1_interface_type)) {
		gpio_direction_output(MX6_PHYFLEX_CAM1_LVDS_PWRDN, 1);
		gpio_direction_output(MX6_PHYFLEX_CAM1_OE, 1);
	} else if (!strcmp("none", csi1_interface_type)) {
		gpio_direction_output(MX6_PHYFLEX_CAM1_LVDS_PWRDN, 0);
		gpio_direction_output(MX6_PHYFLEX_CAM1_OE, 1);
	} else {
		gpio_direction_output(MX6_PHYFLEX_CAM1_LVDS_PWRDN, 0);
		gpio_direction_output(MX6_PHYFLEX_CAM1_OE, 0);
	}


	imx_add_viv_gpu(&imx6_gpu_data, &imx6_gpu_pdata);
	imx6q_add_vpu();

	/* Initialize SoC RTC */
	imx6q_add_imx_snvs_rtc();

	/* Initialise i2c bus and devices */
	imx6q_add_imx_i2c(0, &mx6_phyflex_i2c0_data);
	imx6q_add_imx_i2c(1, &mx6_phyflex_i2c1_data);
	i2c_register_board_info(0, mxc_i2c0_board_info, ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info, ARRAY_SIZE(mxc_i2c1_board_info));

	imx6q_add_imx_i2c(2, &mx6_phyflex_i2c2_data);
	i2c_register_board_info(2, mxc_i2c2_board_info, ARRAY_SIZE(mxc_i2c2_board_info));

	/* Init onboard can bus */
	imx6q_add_flexcan0(&mx6_phyflex_flexcan0_pdata);

#ifdef ENABLE_HDMI
	/* GPIO i2c initialisation, needed for HDMI EDID */
	platform_device_register(&i2c_gpio_device);
	i2c_register_board_info(3, hdmi_i2c_data, ARRAY_SIZE(hdmi_i2c_data));
	imx6q_add_mxc_hdmi_core(&hdmi_core_data);
	imx6q_add_mxc_hdmi(&hdmi_data);
	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();
#endif

	/* SPI */
	imx6q_add_ecspi(2, &mx6_phyflex_spi_data);
	spi_register_board_info(n25q128_spi2_board_info, ARRAY_SIZE(n25q128_spi2_board_info));

#ifdef ENABLE_PHY
	imx6_init_fec(fec_data);
#endif /* ENABLE_PHY */

	/* Initialize thermal and power managements */
	imx6q_add_anatop_thermal_imx(1, &mx6_phyflex_anatop_thermal_data);
	imx6q_add_pm_imx(0, &mx6_phyflex_pm_data);

	platform_device_register(&phyflex_vmmc_reg_devices);

	/* SD cards initialization */
	board_esdhc_init(2, MX6_PHYFLEX_SD3_CD, MX6_PHYFLEX_SD3_WP);
	board_esdhc_init(1, MX6_PHYFLEX_SD2_CD, MX6_PHYFLEX_SD2_WP);

	mx6_phyflex_init_usb();
	mx6_phyflex_init_audio();

	if (cpu_is_mx6q()) {
#ifdef CONFIG_SATA_AHCI_PLATFORM
		imx6q_add_ahci(0, &mx6_phyflex_sata_data);
#else
		mx6_phyflex_sata_init(NULL,
			(void __iomem *)ioremap(MX6Q_SATA_BASE_ADDR, SZ_4K));
#endif
	}

	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();

	board_nand_init();

	/* DVFS initialization */
	imx6q_add_dvfs_core(&phyflex_dvfscore_data);

	/* Add PWM devices */
	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm(1);
	imx6q_add_mxc_pwm(2);
	imx6q_add_mxc_pwm(3);

	/* Initialize LDB PWM backlight */
	gpio_request_one(MX6_PHYFLEX_LDB0_BACKLIGHT, GPIOF_OUT_INIT_LOW, "ldb-backlight.0");
	gpio_request_one(MX6_PHYFLEX_LDB1_BACKLIGHT, GPIOF_OUT_INIT_LOW, "ldb-backlight.1");
	imx6q_add_mxc_pwm_backlight(0, &mx6_phyflex_pwm_backlight_data0);
	imx6q_add_mxc_pwm_backlight(1, &mx6_phyflex_pwm_backlight_data1);

	/* PCI Express initialization */
	imx6q_add_pcie(&mx6_phyflex_pcie_data);

	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);
//	imx6q_add_mlb150(&mx6_phyflex_mlb150_data);


	/* 1-wire devices registration */
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

	early_console_setup(UART4_BASE_ADDR, uart_clk);
}

static struct sys_timer mxc_timer = {
	.init   = mx6_timer_init,
};

static void __init mx6_phyflex_reserve(void)
{
	phys_addr_t phys;

	if (imx6_gpu_pdata.reserved_mem_size) {
#ifdef DDR_2GB
		phys = memblock_alloc_base(
			imx6_gpu_pdata.reserved_mem_size, SZ_4K, SZ_2G);
#else
		phys = memblock_alloc_base(
			imx6_gpu_pdata.reserved_mem_size, SZ_4K, SZ_1G);
#endif
		memblock_free(phys, imx6_gpu_pdata.reserved_mem_size);
		memblock_remove(phys, imx6_gpu_pdata.reserved_mem_size);
		imx6_gpu_pdata.reserved_mem_base = phys;
	}
}

MACHINE_START(MX6Q_PHYFLEX, "Phytec i.MX 6Quad phyFLEX Board")
	.boot_params	= MX6_PHYS_OFFSET + 0x100,
	.fixup		= mx6_phyflex_fixup,
	.map_io		= mx6_map_io,
	.init_irq	= mx6_init_irq,
	.init_machine	= mx6_phyflex_init,
	.timer		= &mxc_timer,
	.reserve	= mx6_phyflex_reserve,
MACHINE_END
