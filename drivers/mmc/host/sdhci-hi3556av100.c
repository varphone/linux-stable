/*
 * Copyright (c) 2017-2018 HiSilicon Technologies Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#define REG_EMMC_DRV_DLL_CTRL		0x1b0
#define REG_SDIO0_DRV_DLL_CTRL		0x1d4
#define REG_SDIO1_DRV_DLL_CTRL		0x1fc
#define REG_SDIO2_DRV_DLL_CTRL		/*no sdio2*/
#define SDIO_DRV_PHASE_SEL_MASK		(0x1f << 24)
#define SDIO_DRV_SEL(phase)		((phase) << 24)

#define REG_EMMC_DRV_DLL_STATUS		0x1c4
#define REG_SDIO0_DRV_DLL_STATUS	0x1e8
#define REG_SDIO1_DRV_DLL_STATUS	0x210
#define REG_SDIO2_DRV_DLL_STATUS	/*no sdio2*/
#define SDIO_DRV_DLL_LOCK		BIT(15)

#define REG_EMMC_SAMPL_DLL_STATUS	0x1bc
#define REG_SDIO0_SAMPL_DLL_STATUS	0x1e0
#define REG_SDIO1_SAMPL_DLL_STATUS	0x208
#define REG_SDIO2_SAMPL_DLL_STATUS	/*no sdio2*/
#define SDIO_SAMPL_DLL_SLAVE_READY	BIT(14)

#define REG_EMMC_SAMPL_DLL_CTRL		0x1a8
#define REG_SDIO0_SAMPL_DLL_CTRL	0x1ec
#define REG_SDIO1_SAMPL_DLL_CTRL	0x214
#define REG_SDIO2_SAMPL_DLL_CTRL	/*no sdio2*/
#define SDIO_SAMPL_DLL_SLAVE_EN		BIT(16)

#define REG_EMMC_SAMPLB_DLL_CTRL	0x1ac
#define REG_SDIO0_SAMPLB_DLL_CTRL	0x1d0
#define REG_SDIO1_SAMPLB_DLL_CTRL	0x1f8
#define REG_SDIO2_SAMPLB_DLL_CTRL	/*no sdio2*/
#define SDIO_SAMPLB_DLL_CLK_MASK	(0x1f << 24)
#define SDIO_SAMPLB_SEL(phase)		((phase) << 24)

#define REG_EMMC_DS_DLL_CTRL		0x1b4
#define EMMC_DS_DLL_MODE_SSEL		BIT(13)
#define EMMC_DS_DLL_SSEL_MASK		(0x1fff)
#define REG_EMMC_DS180_DLL_CTRL		0x1b8
#define EMMC_DS180_DLL_BYPASS		BIT(15)
#define REG_EMMC_DS_DLL_STATUS		0x1c8
#define EMMC_DS_DLL_LOCK		BIT(15)
#define EMMC_DS_DLL_MDLY_TAP_MASK	(0x1fff)

#define REG_MISC_CTRL1          0x4
#define SDIO1_PD_MUX_BYPASS     BIT(9)
#define SDIO0_PD_MUX_BYPASS     BIT(8)

#define REG_MISC_CTRL18			0x48
#define SDIO0_PWRSW_SEL_1V8		BIT(5)
#define SDIO0_PWR_EN			BIT(4)
#define SDIO0_IO_MODE_SEL_1V8	BIT(1)
#define SDIO0_PWR_CTRL_BY_MISC	BIT(0)

#define REG_IOCTL_RONSEL_1_0		0x264
#define REG_IOCTL_OD_RONSEL_2		0x268

#define REG_CTRL_SDIO0_CLK		0x006c
#define REG_CTRL_SDIO0_CMD		0x0070
#define REG_CTRL_SDIO0_DATA0	0x0074
#define REG_CTRL_SDIO0_DATA1	0x0078
#define REG_CTRL_SDIO0_DATA2	0x007c
#define REG_CTRL_SDIO0_DATA3	0x0080
#define REG_CTRL_SDIO1_CLK		0x0084
#define REG_CTRL_SDIO1_CMD		0x0088
#define REG_CTRL_SDIO1_DATA0	0x008c
#define REG_CTRL_SDIO1_DATA1	0x0090
#define REG_CTRL_SDIO1_DATA2	0x0094
#define REG_CTRL_SDIO1_DATA3	0x0098

static unsigned int sdr104_drv[] = {0x60, 0x20, 0x20, 0x20, 0x20, 0x20};
static unsigned int sdr50_drv[] = {0x60, 0x20, 0x20, 0x20, 0x20, 0x20};
static unsigned int sdr25_hs_drv[] = {0x60, 0x20, 0x20, 0x20, 0x20, 0x20};
static unsigned int other_drv[] = {0x60, 0x20, 0x20, 0x20, 0x20, 0x20};

static void hisi_set_sd_iocfg(struct sdhci_host *host)
{
	struct sdhci_hisi_priv *hisi_priv = sdhci_get_pltfm_priv(host);
	unsigned int devid = hisi_priv->devid;
	void* iocfg_regmap = hisi_priv->iocfg_regmap;
	unsigned int reg_addr, start, end;
	unsigned int *pin_drv_cap;

	if (host->timing == MMC_TIMING_UHS_SDR104)
		pin_drv_cap = sdr104_drv;
	else if (host->timing == MMC_TIMING_UHS_SDR50)
		pin_drv_cap = sdr50_drv;
	else if (host->timing == MMC_TIMING_UHS_SDR25 ||
			host->timing == MMC_TIMING_SD_HS)
		pin_drv_cap = sdr25_hs_drv;
	else
		pin_drv_cap = other_drv;

	start = devid == 1 ? REG_CTRL_SDIO0_CLK : REG_CTRL_SDIO1_CLK;
	end = devid == 1 ? REG_CTRL_SDIO0_DATA3 : REG_CTRL_SDIO1_DATA3;
	for (reg_addr = start; reg_addr <= end; reg_addr += 4) {
		regmap_write_bits(iocfg_regmap, reg_addr, 0xf0, *pin_drv_cap);
		pin_drv_cap++;
	}
}

static void hisi_set_pd_pin_status(struct regmap *misc)
{
	unsigned int ctrl;

	if (!misc)
		return;

	regmap_read(misc, REG_MISC_CTRL1, &ctrl);
	ctrl &= ~(SDIO1_PD_MUX_BYPASS | SDIO0_PD_MUX_BYPASS);
	regmap_write(misc, REG_MISC_CTRL1, ctrl);
}

static int hisi_set_signal_voltage_3v3(struct sdhci_host *host)
{
	struct sdhci_hisi_priv *hisi_priv = sdhci_get_pltfm_priv(host);
	struct regmap *misc = hisi_priv->misc_regmap;
	unsigned int ctrl;

	pr_debug("%s: set voltage to 330\n", mmc_hostname(host->mmc));

	if (hisi_priv->devid == 1) {
		regmap_read(misc, REG_MISC_CTRL18, &ctrl);
		ctrl |= SDIO0_PWR_CTRL_BY_MISC | SDIO0_PWR_EN;
		ctrl &= ~SDIO0_IO_MODE_SEL_1V8;
		regmap_write(misc, REG_MISC_CTRL18, ctrl);

		usleep_range(1000, 2000);
		ctrl &= ~SDIO0_PWRSW_SEL_1V8;
		regmap_write(misc, REG_MISC_CTRL18, ctrl);

		regmap_read(misc, REG_MISC_CTRL18, &ctrl);
		if ((ctrl & SDIO0_PWR_CTRL_BY_MISC)
				&& (ctrl & SDIO0_PWR_EN)
				&& !(ctrl & SDIO0_IO_MODE_SEL_1V8) &&
				!(ctrl & SDIO0_PWRSW_SEL_1V8))
			return 0;
	}

	pr_warn("%s: 3.3V output did not became stable\n",
			mmc_hostname(host->mmc));

	return -EAGAIN;
}

static int hisi_set_signal_voltage_1v8(struct sdhci_host *host)
{
	struct sdhci_hisi_priv *hisi_priv = sdhci_get_pltfm_priv(host);
	struct regmap *misc = hisi_priv->misc_regmap;
	unsigned int ctrl;

	pr_debug("%s: set voltage to 180\n", mmc_hostname(host->mmc));

	if (hisi_priv->devid == 0)
		return 0;

	if (hisi_priv->devid == 1) {
		regmap_read(misc, REG_MISC_CTRL18, &ctrl);
		ctrl |= SDIO0_PWRSW_SEL_1V8;
		regmap_write(misc, REG_MISC_CTRL18, ctrl);

		usleep_range(1000, 2000);

		ctrl |= SDIO0_IO_MODE_SEL_1V8;
		regmap_write(misc, REG_MISC_CTRL18, ctrl);

		regmap_read(misc, REG_MISC_CTRL18, &ctrl);
		if ((ctrl & SDIO0_PWRSW_SEL_1V8) && (ctrl & SDIO0_IO_MODE_SEL_1V8))
			return 0;
	}

	if (hisi_priv->devid == 2)
		return 0;

	pr_warn("%s: 1.8V output did not became stable\n",
			mmc_hostname(host->mmc));

	return -EAGAIN;
}

static void hisi_get_phase(struct sdhci_host *host)
{
	struct sdhci_hisi_priv *hisi_priv = sdhci_get_pltfm_priv(host);

	if (host->mmc->ios.timing == MMC_TIMING_MMC_DDR52
			|| host->mmc->ios.timing == MMC_TIMING_UHS_DDR50)
		hisi_priv->drv_phase = 8;       /*90 degree*/
	else if (host->mmc->ios.timing == MMC_TIMING_MMC_HS200
		|| host->mmc->ios.timing == MMC_TIMING_UHS_SDR104)
		hisi_priv->drv_phase = 20;      /*225 degree*/
	else if (host->mmc->ios.timing == MMC_TIMING_MMC_HS400)
		hisi_priv->drv_phase = 9;       /*101.25 degree*/
	else
		hisi_priv->drv_phase = 16;      /*180 degree */

	if (host->mmc->ios.timing == MMC_TIMING_MMC_HS400)
		hisi_priv->sampl_phase = hisi_priv->tuning_phase;
	else if (host->mmc->ios.timing == MMC_TIMING_MMC_DDR52)
		hisi_priv->sampl_phase = 20;
	else if (host->mmc->ios.timing == MMC_TIMING_UHS_DDR50
			|| host->mmc->ios.timing == MMC_TIMING_SD_HS
			|| host->mmc->ios.timing == MMC_TIMING_UHS_SDR25
			|| host->mmc->ios.timing == MMC_TIMING_MMC_HS)
		hisi_priv->sampl_phase = 4;
	else
		hisi_priv->sampl_phase = 0;
}
