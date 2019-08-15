/*
 * Copyright (c) 2016-2017 HiSilicon Technologies Co., Ltd.
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

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/reset.h>
#include <linux/mmc/host.h>
#include "sdhci-pltfm.h"
#include "mci_proc.h"

#define PHASE_SCALE	32
#define NOT_FOUND	-1
#define MAX_TUNING_NUM	1
#define MAX_FREQ	200000000

struct sdhci_hisi_priv {
	struct reset_control *crg_rst;
	struct reset_control *dll_rst;
	struct reset_control *sampl_rst;
	struct regmap *crg_regmap;
	struct regmap *misc_regmap;
	struct regmap *iocfg_regmap;
	void __iomem *phy_addr;
	unsigned int f_max;
	unsigned int devid;
	unsigned int drv_phase;
	unsigned int sampl_phase;
	unsigned int tuning_phase;
};

static inline void *sdhci_get_pltfm_priv(struct sdhci_host *host)
{
	return sdhci_pltfm_priv(sdhci_priv(host));
}


#ifdef CONFIG_ARCH_HI3559AV100
#include "sdhci-hi3559av100.c"
#endif

#ifdef CONFIG_ARCH_HI3519AV100
#include "sdhci-hi3519av100.c"
#endif

#ifdef CONFIG_ARCH_HI3556AV100
#include "sdhci-hi3556av100.c"
#endif

static unsigned int sdhci_hisi_get_max_clk(struct sdhci_host *host)
{
	struct sdhci_hisi_priv *hisi_priv = sdhci_get_pltfm_priv(host);

	return hisi_priv->f_max;
}

static int sdhci_hisi_parse_dt(struct sdhci_host *host)
{
	struct sdhci_hisi_priv *hisi_priv = sdhci_get_pltfm_priv(host);
	struct device_node *np = host->mmc->parent->of_node;
	int ret, len;

	ret = mmc_of_parse(host->mmc);
	if (ret)
		return ret;

	if (of_property_read_u32(np, "max-frequency", &hisi_priv->f_max))
		hisi_priv->f_max = MAX_FREQ;

	if (of_find_property(np, "mmc-cmd-queue", &len))
		host->mmc->caps2 |= MMC_CAP2_CMD_QUEUE;

	if (of_find_property(np, "mmc-broken-cmd23", &len) ||
		(host->mmc->caps2 & MMC_CAP2_CMD_QUEUE))
		host->quirks2 |= SDHCI_QUIRK2_HOST_NO_CMD23;

	return 0;
}

static void hisi_mmc_crg_init(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_hisi_priv *hisi_priv = sdhci_pltfm_priv(pltfm_host);

	clk_prepare_enable(pltfm_host->clk);
	reset_control_assert(hisi_priv->crg_rst);
	reset_control_assert(hisi_priv->dll_rst);
	reset_control_assert(hisi_priv->sampl_rst);
	udelay(25);
	reset_control_deassert(hisi_priv->crg_rst);
	udelay(10);
}

static void sdhci_hisi_hs400_enhanced_strobe(struct mmc_host *mmc,
		struct mmc_ios *ios)
{
	u16 ctrl;
	struct sdhci_host *host = mmc_priv(mmc);

	ctrl = sdhci_readw(host, SDHCI_EMMC_CTRL);
	if (ios->enhanced_strobe)
		ctrl |= SDHCI_ENH_STROBE_EN;
	else
		ctrl &= ~SDHCI_ENH_STROBE_EN;

	sdhci_writew(host, ctrl, SDHCI_EMMC_CTRL);
}

static int sdhci_hisi_pltfm_init(struct platform_device *pdev,
		struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_hisi_priv *hisi_priv = sdhci_pltfm_priv(pltfm_host);
	struct device_node *np = pdev->dev.of_node;
	struct clk *clk;
	int ret;

	hisi_priv->crg_rst = devm_reset_control_get(&pdev->dev, "crg_reset");
	if (IS_ERR_OR_NULL(hisi_priv->crg_rst)) {
		dev_err(&pdev->dev, "get crg_rst failed.\n");
		return PTR_ERR(hisi_priv->crg_rst);;
	}

	hisi_priv->dll_rst = devm_reset_control_get(&pdev->dev, "dll_reset");
	if (IS_ERR_OR_NULL(hisi_priv->dll_rst)) {
		dev_err(&pdev->dev, "get dll_rst failed.\n");
		return PTR_ERR(hisi_priv->dll_rst);;
	}

	hisi_priv->sampl_rst = devm_reset_control_get(&pdev->dev, "sampl_reset");
	if (IS_ERR_OR_NULL(hisi_priv->sampl_rst)) {
		dev_err(&pdev->dev, "get sampl_rst failed.\n");
		return PTR_ERR(hisi_priv->sampl_rst);;
	}

	hisi_priv->crg_regmap = syscon_regmap_lookup_by_phandle(np, "crg_regmap");
	if (IS_ERR(hisi_priv->crg_regmap))
		return PTR_ERR(hisi_priv->crg_regmap);

	if (of_property_read_u32(np, "devid", &hisi_priv->devid))
		return -EINVAL;

	if (hisi_priv->devid == 0) {
		struct resource *res;

		res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		if (!res)
			return -ENOMEM;

		hisi_priv->phy_addr = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(hisi_priv->phy_addr))
			return PTR_ERR(hisi_priv->phy_addr);
	} else {
		hisi_priv->misc_regmap = syscon_regmap_lookup_by_phandle(np,
				"misc_regmap");
		if (IS_ERR(hisi_priv->misc_regmap))
			return PTR_ERR(hisi_priv->misc_regmap);

		hisi_priv->iocfg_regmap = syscon_regmap_lookup_by_phandle(np,
				"iocfg_regmap");
		if (IS_ERR(hisi_priv->iocfg_regmap))
			return PTR_ERR(hisi_priv->iocfg_regmap);
	
		hisi_set_pd_pin_status(hisi_priv->misc_regmap);
	}

	clk = devm_clk_get(mmc_dev(host->mmc), "mmc_clk");
	if (IS_ERR_OR_NULL(clk)) {
		dev_err(mmc_dev(host->mmc), "get clk err\n");
		return -EINVAL;
	}

	pltfm_host->clk = clk;

	hisi_mmc_crg_init(host);
	ret = sdhci_hisi_parse_dt(host);
	if (ret)
		return ret;

	/* only eMMC has a hw reset, and now eMMC signaling
	 * is fixed to 180*/
	if (host->mmc->caps & MMC_CAP_HW_RESET) {
		host->flags &= ~SDHCI_SIGNALING_330;
		host->flags |= SDHCI_SIGNALING_180;
	}

	/* we parse the support timings from dts, so we read the
	 * host capabilities early and clear the timing capabilities,
	 * SDHCI_QUIRK_MISSING_CAPS is set so that sdhci driver would
	 * not read it again */
	host->caps = sdhci_readl(host, SDHCI_CAPABILITIES);
	host->caps &= ~(SDHCI_CAN_DO_HISPD);
	host->caps1 = sdhci_readl(host, SDHCI_CAPABILITIES_1);
	host->caps1 &= ~(SDHCI_SUPPORT_SDR50 | SDHCI_SUPPORT_SDR104 |
				SDHCI_SUPPORT_DDR50 | SDHCI_CAN_DO_ADMA3);
	host->caps1 |= SDHCI_USE_SDR50_TUNING;
	host->quirks |= SDHCI_QUIRK_MISSING_CAPS;
	host->quirks |= SDHCI_QUIRK_NO_ENDATTR_IN_NOPDESC;

	host->mmc_host_ops.hs400_enhanced_strobe = sdhci_hisi_hs400_enhanced_strobe;

	mci_host[slot_index++] = host->mmc;

	return 0;
}

static void hisi_set_drv_phase(struct sdhci_host *host, unsigned int phase)
{
	struct sdhci_hisi_priv *hisi_priv = sdhci_get_pltfm_priv(host);
	unsigned int devid = hisi_priv->devid;
	unsigned int offset[4] = {REG_EMMC_DRV_DLL_CTRL,
							REG_SDIO0_DRV_DLL_CTRL,
							REG_SDIO1_DRV_DLL_CTRL,
							REG_SDIO2_DRV_DLL_CTRL};

	regmap_write_bits(hisi_priv->crg_regmap, offset[devid],
			SDIO_DRV_PHASE_SEL_MASK, SDIO_DRV_SEL(phase));
}

static void hisi_set_sampl_phase(struct sdhci_host *host,
		unsigned int phase)
{
	unsigned int reg;

	reg = sdhci_readl(host, SDHCI_AT_STAT);
	reg &= ~SDHCI_PHASE_SEL_MASK;
	reg |= phase;
	sdhci_writel(host, reg, SDHCI_AT_STAT);
}

static void hisi_disable_card_clk(struct sdhci_host *host)
{
	u16 clk;

	clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
	clk &= ~SDHCI_CLOCK_CARD_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);
}

static void hisi_enable_card_clk(struct sdhci_host *host)
{
	u16 clk;

	clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
	clk |= SDHCI_CLOCK_CARD_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);
}

static void hisi_disable_inter_clk(struct sdhci_host *host)
{
	u16 clk;

	clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
	clk &= ~SDHCI_CLOCK_INT_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);
}

static void hisi_enable_sampl_dll_slave(struct sdhci_host *host)
{
	struct sdhci_hisi_priv *hisi_priv = sdhci_get_pltfm_priv(host);
	unsigned int devid = hisi_priv->devid;
	unsigned int offset[4] = {REG_EMMC_SAMPL_DLL_CTRL,
							REG_SDIO0_SAMPL_DLL_CTRL,
							REG_SDIO1_SAMPL_DLL_CTRL,
							REG_SDIO2_SAMPL_DLL_CTRL};

	regmap_write_bits(hisi_priv->crg_regmap, offset[devid],
			SDIO_SAMPL_DLL_SLAVE_EN, SDIO_SAMPL_DLL_SLAVE_EN);
}

static void hisi_wait_drv_dll_lock(struct sdhci_host *host)
{
	struct sdhci_hisi_priv *hisi_priv = sdhci_get_pltfm_priv(host);
	unsigned int devid = hisi_priv->devid;
	unsigned int reg, timeout = 20;
	unsigned int offset[4] = {REG_EMMC_DRV_DLL_STATUS,
							REG_SDIO0_DRV_DLL_STATUS,
							REG_SDIO1_DRV_DLL_STATUS,
							REG_SDIO2_DRV_DLL_STATUS};

	do {
		reg = 0;
		regmap_read(hisi_priv->crg_regmap, offset[devid], &reg);
		if (reg & SDIO_DRV_DLL_LOCK)
			return;

		mdelay(1);
		timeout--;
	} while (timeout > 0);

	pr_err("%s: DRV DLL master not locked.\n", mmc_hostname(host->mmc));
}

static void hisi_wait_sampl_dll_slave_ready(struct sdhci_host *host)
{
	struct sdhci_hisi_priv *hisi_priv = sdhci_get_pltfm_priv(host);
	unsigned int devid = hisi_priv->devid;
	unsigned int reg, timeout = 20;
	unsigned int offset[4] = {REG_EMMC_SAMPL_DLL_STATUS,
							REG_SDIO0_SAMPL_DLL_STATUS,
							REG_SDIO1_SAMPL_DLL_STATUS,
							REG_SDIO2_SAMPL_DLL_STATUS};

	do {
		reg = 0;
		regmap_read(hisi_priv->crg_regmap, offset[devid], &reg);
		if (reg & SDIO_SAMPL_DLL_SLAVE_READY)
			return;

		mdelay(1);
		timeout--;
	} while (timeout > 0);

	pr_err("%s: SAMPL DLL slave not ready.\n", mmc_hostname(host->mmc));
}

static void hisi_wait_ds_dll_lock(struct sdhci_host *host)
{
	struct sdhci_hisi_priv *hisi_priv = sdhci_get_pltfm_priv(host);
	unsigned int reg, timeout = 20;

	do {
		reg = 0;
		regmap_read(hisi_priv->crg_regmap, REG_EMMC_DS_DLL_STATUS, &reg);
		if (reg & EMMC_DS_DLL_LOCK)
			return;

		mdelay(1);
		timeout--;
	} while (timeout > 0);

	pr_err("%s: DS DLL master not locked.\n", mmc_hostname(host->mmc));
}

#if !defined(CONFIG_ARCH_HI3519AV100) && !defined(CONFIG_ARCH_HI3556AV100)
static void hisi_set_ds_dll_delay(struct sdhci_host *host)
{
	struct sdhci_hisi_priv *hisi_priv = sdhci_get_pltfm_priv(host);
	unsigned int reg, mdly_tap;

	regmap_read(hisi_priv->crg_regmap, REG_EMMC_DS_DLL_STATUS, &reg);
	mdly_tap = reg & EMMC_DS_DLL_MDLY_TAP_MASK;

	regmap_write_bits(hisi_priv->crg_regmap, REG_EMMC_DS_DLL_CTRL,
			(EMMC_DS_DLL_SSEL_MASK | EMMC_DS_DLL_MODE_SSEL),
			((mdly_tap / 4 + 12) | EMMC_DS_DLL_MODE_SSEL));

	regmap_write_bits(hisi_priv->crg_regmap, REG_EMMC_DS180_DLL_CTRL,
			EMMC_DS180_DLL_BYPASS, EMMC_DS180_DLL_BYPASS);
}
#endif


static void hisi_enable_sample(struct sdhci_host *host)
{
	unsigned int reg;

	reg = sdhci_readl(host, SDHCI_AT_CTRL);
	reg |= SDHCI_SAMPLE_EN;
	sdhci_writel(host, reg, SDHCI_AT_CTRL);
}

static void sdhci_hisi_set_clock(struct sdhci_host *host, unsigned int clock)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_hisi_priv *hisi_priv = sdhci_pltfm_priv(pltfm_host);
	unsigned long timeout;
	u16 clk = 0;

	host->mmc->actual_clock = 0;
	hisi_disable_card_clk(host);
	udelay(25);
	hisi_disable_inter_clk(host);
	if (clock == 0)
		return;

	reset_control_assert(hisi_priv->dll_rst);
	reset_control_assert(hisi_priv->sampl_rst);
	udelay(25);

	clk_set_rate(pltfm_host->clk, clock);
	host->mmc->actual_clock = clk_get_rate(pltfm_host->clk);

	hisi_get_phase(host);
	hisi_set_drv_phase(host, hisi_priv->drv_phase);
	hisi_enable_sample(host);
	hisi_set_sampl_phase(host, hisi_priv->sampl_phase);
	udelay(25);

	if (host->mmc->actual_clock > MMC_HIGH_52_MAX_DTR) {
		hisi_enable_sampl_dll_slave(host);
		reset_control_deassert(hisi_priv->dll_rst);
		reset_control_deassert(hisi_priv->sampl_rst);
	}

	clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
	clk |= SDHCI_CLOCK_INT_EN | SDHCI_CLOCK_PLL_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);
	timeout = 20;
	while (!((clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL))
				& SDHCI_CLOCK_INT_STABLE)) {
		if (timeout == 0) {
			pr_err("%s: Internal clock never "
					"stabilised.\n", mmc_hostname(host->mmc));
			return;
		}
		timeout--;
		mdelay(1);
	}

	if (host->mmc->actual_clock > MMC_HIGH_52_MAX_DTR) {
		hisi_wait_drv_dll_lock(host);
		hisi_wait_sampl_dll_slave_ready(host);
	}

	clk |= SDHCI_CLOCK_CARD_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);
	udelay(100);

	if (host->mmc->ios.timing == MMC_TIMING_MMC_HS400) {
		hisi_wait_ds_dll_lock(host);
#if !defined(CONFIG_ARCH_HI3519AV100) && !defined(CONFIG_ARCH_HI3556AV100)
		hisi_set_ds_dll_delay(host);
#endif
	}
}

static void hisi_select_sampl_phase(struct sdhci_host *host,
		unsigned int phase)
{
	hisi_disable_card_clk(host);
	hisi_set_sampl_phase(host, phase);
	hisi_wait_sampl_dll_slave_ready(host);
	hisi_enable_card_clk(host);
	udelay(1);
}

static int hisi_send_tuning(struct sdhci_host *host, u32 opcode)
{
	int count, err;

	count = 0;
	do {
		err = mmc_send_tuning(host->mmc, opcode, NULL);
		if (err)
			break;

		count++;
	} while (count < MAX_TUNING_NUM);

	return err;
}

static void hisi_pre_tuning(struct sdhci_host *host)
{
	sdhci_writel(host, host->ier | SDHCI_INT_DATA_AVAIL, SDHCI_INT_ENABLE);
	sdhci_writel(host, host->ier | SDHCI_INT_DATA_AVAIL, SDHCI_SIGNAL_ENABLE);

	hisi_wait_drv_dll_lock(host);
	hisi_enable_sampl_dll_slave(host);
	hisi_enable_sample(host);
	host->is_tuning = 1;
}

static void hisi_post_tuning(struct sdhci_host *host)
{
	unsigned short ctrl;

	ctrl = sdhci_readw(host, SDHCI_HOST_CONTROL2);
	ctrl |= SDHCI_CTRL_TUNED_CLK;
	sdhci_writew(host, ctrl, SDHCI_HOST_CONTROL2);

	sdhci_writel(host, host->ier, SDHCI_INT_ENABLE);
	sdhci_writel(host, host->ier, SDHCI_SIGNAL_ENABLE);
	host->is_tuning = 0;
}

#ifndef SDHCI_HISI_EDGE_TUNING
static int hisi_get_best_sampl(u32 candidates)
{
	int rise = NOT_FOUND, fall = NOT_FOUND;
	int win_max_r = NOT_FOUND, win_max_f = NOT_FOUND;
	int end_fall = NOT_FOUND, found = NOT_FOUND;
	int i, win, win_max = 0;

	for (i = 0; i < 32; i++) {
		if ((candidates & 0x3) == 0x2)
			rise = (i + 1) % 32;

		if ((candidates & 0x3) == 0x1) {
			fall = i;
			if (rise != NOT_FOUND) {
				win = fall - rise + 1;
				if (win > win_max) {
					win_max = win;
					found = (fall + rise) / 2;
					win_max_r = rise;
					win_max_f = fall;
					rise = NOT_FOUND;
					fall = NOT_FOUND;
				}
			} else
				end_fall = fall;
		}
		candidates = ror32(candidates, 1);
	}

	if (end_fall != NOT_FOUND && rise != NOT_FOUND) {
		fall = end_fall;
		if (end_fall < rise)
			end_fall += 32;

		win = end_fall - rise + 1;
		if (win > win_max) {
			found = (rise + (win / 2)) % 32;
			win_max_r = rise;
			win_max_f = fall;
		}
	}

	if (found != NOT_FOUND)
		printk("valid phase shift [%d, %d] Final Phase:%d\n",
				win_max_r, win_max_f, found);

	return found;
}

static int sdhci_hisi_exec_tuning(struct sdhci_host *host, u32 opcode)
{
	struct sdhci_hisi_priv *hisi_priv = sdhci_get_pltfm_priv(host);
	unsigned int sampl;
	unsigned int candidates = 0;
	int phase, err;

	hisi_pre_tuning(host);

	for (sampl = 0; sampl < PHASE_SCALE; sampl++) {
		hisi_select_sampl_phase(host, sampl);

		err = hisi_send_tuning(host, opcode);
		if (err)
			pr_debug("send tuning CMD%u fail! phase:%d err:%d\n",
					opcode, sampl, err);
		else
			candidates |= (0x1 << sampl);
	}

	pr_info("%s: tuning done! candidates 0x%X: ",
			mmc_hostname(host->mmc), candidates);

	phase = hisi_get_best_sampl(candidates);
	if (phase == NOT_FOUND) {
		phase = hisi_priv->sampl_phase;
		printk("no valid phase shift! use default %d\n", phase);
	}

	hisi_priv->tuning_phase = phase;
	hisi_select_sampl_phase(host, phase);
	hisi_post_tuning(host);

	return 0;
}

#else
static void hisi_enable_edge_tuning(struct sdhci_host *host)
{
	struct sdhci_hisi_priv *hisi_priv = sdhci_get_pltfm_priv(host);
	unsigned int devid = hisi_priv->devid;
	unsigned int samplb_offset[4] = {REG_EMMC_SAMPLB_DLL_CTRL,
							REG_SDIO0_SAMPLB_DLL_CTRL,
							REG_SDIO1_SAMPLB_DLL_CTRL,
							REG_SDIO2_SAMPLB_DLL_CTRL};
	unsigned int reg;

	regmap_write_bits(hisi_priv->crg_regmap, samplb_offset[devid],
			SDIO_SAMPLB_DLL_CLK_MASK, SDIO_SAMPLB_SEL(8));

	reg = sdhci_readl(host, SDHCI_MULTI_CYCLE);
	reg |= SDHCI_EDGE_DETECT_EN;
	sdhci_writel(host, reg, SDHCI_MULTI_CYCLE);
}

static void hisi_disable_edge_tuning(struct sdhci_host *host)
{
	unsigned int reg;

	reg = sdhci_readl(host, SDHCI_MULTI_CYCLE);
	reg &= ~SDHCI_EDGE_DETECT_EN;
	sdhci_writel(host, reg, SDHCI_MULTI_CYCLE);
}

static int sdhci_hisi_exec_edge_tuning(struct sdhci_host *host, u32 opcode)
{
	struct sdhci_hisi_priv *hisi_priv = sdhci_get_pltfm_priv(host);
	unsigned int index, val;
	unsigned int found = 0, prev_found = 0;
	unsigned int edge_p2f, edge_f2p, start, end;
	unsigned int phase, fall = NOT_FOUND, rise = NOT_FOUND;
	unsigned int fall_updat_flag = 0;
	int err, prev_err = 0;

	hisi_pre_tuning(host);
	hisi_enable_edge_tuning(host);

	start = 0;
	end = PHASE_SCALE / 4;

	edge_p2f = start;
	edge_f2p = end;
	for (index = 0; index <= end; index++) {
		hisi_select_sampl_phase(host, index * 4);

		err = mmc_send_tuning(host->mmc, opcode, NULL);
		if (!err) {
			val = sdhci_readl(host, SDHCI_MULTI_CYCLE);
			found = val & SDHCI_FOUND_EDGE;
		} else
			found = 1;

		if (prev_found && !found)
			edge_f2p = index;
		else if (!prev_found && found)
			edge_p2f = index;

		if ((edge_p2f != start) && (edge_f2p != end))
			break;

		prev_found = found;
		found = 0;
	}

	if ((edge_p2f == start) && (edge_f2p == end)) {
		pr_err("%s: tuning failed! can not found edge!\n",
				mmc_hostname(host->mmc));
		return -1;
	}

	hisi_disable_edge_tuning(host);

	start = edge_p2f * 4;
	end = edge_f2p * 4;
	if (end <= start)
		end += PHASE_SCALE;

	fall = start;
	rise = end;
	fall_updat_flag = 0;
	for (index = start; index <= end; index++) {
		hisi_select_sampl_phase(host, index % PHASE_SCALE);

		err = hisi_send_tuning(host, opcode);
		if (err)
			pr_debug("send tuning CMD%u fail! phase:%d err:%d\n",
					opcode, index, err);

		if (err && index == start) {
			if (!fall_updat_flag) {
				fall_updat_flag = 1;
				fall = start;
			}
		} else {
			if (!prev_err && err) {
				if (!fall_updat_flag) {
					fall_updat_flag = 1;
					fall = index;
				}
			}
		}


		if (prev_err && !err) {
			rise = index;
		}

		if (err && index == end)
			rise = end;


		prev_err = err;
	}

	phase = ((fall + rise) / 2 + 16) % PHASE_SCALE;

	pr_info("%s: tuning done! valid phase shift [%d, %d] Final Phase:%d\n",
			mmc_hostname(host->mmc), rise % PHASE_SCALE,
			fall % PHASE_SCALE, phase);

	hisi_priv->tuning_phase = phase;
	hisi_select_sampl_phase(host, phase);
	hisi_post_tuning(host);

	return 0;
}
#endif


static int sdhci_hisi_start_signal_voltage_switch(struct sdhci_host *host,
		struct mmc_ios *ios)
{
	switch (ios->signal_voltage) {
		case MMC_SIGNAL_VOLTAGE_330:
			if (!(host->flags & SDHCI_SIGNALING_330))
				return -EINVAL;
			return hisi_set_signal_voltage_3v3(host);
		case MMC_SIGNAL_VOLTAGE_180:
			if (!(host->flags & SDHCI_SIGNALING_180))
				return -EINVAL;
			return hisi_set_signal_voltage_1v8(host);
		default:
			/* No signal voltage switch required */
			return 0;
	}
}

static void hisi_set_io_config(struct sdhci_host *host)
{
	struct sdhci_hisi_priv *hisi_priv = sdhci_get_pltfm_priv(host);
	unsigned int devid = hisi_priv->devid;
	void* phy_addr = hisi_priv->phy_addr;
	unsigned short reg;

	if (devid == 0) {
		if (host->timing == MMC_TIMING_MMC_HS200
			|| host->timing == MMC_TIMING_MMC_HS400
			|| host->timing == MMC_TIMING_MMC_HS) {
			reg = sdhci_readw(host, SDHCI_EMMC_CTRL);
			reg |= SDHCI_CARD_IS_EMMC;
			sdhci_writew(host, reg, SDHCI_EMMC_CTRL);

			/* set drv strength to 50ohm */
#if defined(CONFIG_ARCH_HI3519AV100) || defined(CONFIG_ARCH_HI3556AV100)
			writel(0x6ff, phy_addr + REG_IOCTL_RONSEL_1_0);
#else
			writel(0x0, phy_addr + REG_IOCTL_RONSEL_1_0);
#endif
			writel(0x6ff, phy_addr + REG_IOCTL_OD_RONSEL_2);
		}
	} else {
		hisi_set_sd_iocfg(host);
	}
}

static void sdhci_hisi_set_uhs_signaling(struct sdhci_host *host, unsigned timing)
{
	sdhci_set_uhs_signaling(host, timing);
	host->timing = timing;

	hisi_set_io_config(host);
}

static void sdhci_hisi_hw_reset(struct sdhci_host *host)
{
	sdhci_writel(host, 0x0, SDHCI_EMMC_HW_RESET);
	udelay(10);
	sdhci_writel(host, 0x1, SDHCI_EMMC_HW_RESET);
	udelay(200);
}

static void hisi_host_extra_init(struct sdhci_host *host)
{
	unsigned short ctrl;
	unsigned int mbiiu_ctrl, val;

	ctrl = sdhci_readw(host, SDHCI_MSHC_CTRL);
	ctrl &= ~SDHCI_CMD_CONFLIT_CHECK;
	sdhci_writew(host, ctrl, SDHCI_MSHC_CTRL);

	mbiiu_ctrl = sdhci_readl(host, SDHCI_AXI_MBIIU_CTRL);
	mbiiu_ctrl &= ~(SDHCI_GM_WR_OSRC_LMT_MASK
			| SDHCI_GM_RD_OSRC_LMT_MASK);
	mbiiu_ctrl |= (SDHCI_GM_WR_OSRC_LMT_SEL(7)
			| SDHCI_GM_RD_OSRC_LMT_SEL(7));
	sdhci_writel(host, mbiiu_ctrl, SDHCI_AXI_MBIIU_CTRL);

	val = sdhci_readl(host, SDHCI_MULTI_CYCLE);
#if defined(CONFIG_ARCH_HI3519AV100) || defined(CONFIG_ARCH_HI3556AV100)
	val |= SDHCI_EDGE_DETECT_EN | SDHCI_DOUT_EN_F_EDGE;
#else
	val &= ~(SDHCI_DATA_DLY_EN | SDHCI_CMD_DLY_EN);
#endif

	sdhci_writel(host, val, SDHCI_MULTI_CYCLE);
	host->error_count = 0;
}

/* This api is for wifi driver rescan the sdio device, 
 * ugly but it is needed */
int hisi_sdio_rescan(int slot)
{
	struct mmc_host *mmc = mci_host[slot];

	if (!mmc) {
		pr_err("invalid mmc, please check the argument\n");
		return -EINVAL;
	}

	mmc_detect_change(mmc, 0);
	return 0;
}
EXPORT_SYMBOL_GPL(hisi_sdio_rescan);

static const struct sdhci_ops sdhci_hisi_ops = {
	.get_max_clock  = sdhci_hisi_get_max_clk,
	.set_clock = sdhci_hisi_set_clock,
	.set_bus_width = sdhci_set_bus_width,
	.reset = sdhci_reset,
	.set_uhs_signaling = sdhci_hisi_set_uhs_signaling,
	.hw_reset = sdhci_hisi_hw_reset,
	.start_signal_voltage_switch =
		sdhci_hisi_start_signal_voltage_switch,
#ifdef SDHCI_HISI_EDGE_TUNING
	.platform_execute_tuning = sdhci_hisi_exec_edge_tuning,
#else
	.platform_execute_tuning = sdhci_hisi_exec_tuning,
#endif
	.pre_init = hisi_mmc_crg_init,
	.extra_init = hisi_host_extra_init,
};

static const struct sdhci_pltfm_data sdhci_hisi_pdata = {
	.ops = &sdhci_hisi_ops,
	.quirks = SDHCI_QUIRK_DATA_TIMEOUT_USES_SDCLK |
		  SDHCI_QUIRK_INVERTED_WRITE_PROTECT |
		  SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN |
		  SDHCI_QUIRK_BROKEN_TIMEOUT_VAL,
	.quirks2 = SDHCI_QUIRK2_PRESET_VALUE_BROKEN,
};
static int sdhci_hisi_probe(struct platform_device *pdev)
{
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	int ret;

	host = sdhci_pltfm_init(pdev, &sdhci_hisi_pdata,
			sizeof(struct sdhci_hisi_priv));
	if (IS_ERR(host))
		return PTR_ERR(host);

	ret = sdhci_hisi_pltfm_init(pdev, host);
	if (ret)
		goto err_sdhci_add;

	ret = sdhci_add_host(host);
	if (ret)
		goto err_sdhci_add;

	return 0;

err_sdhci_add:
	pltfm_host = sdhci_priv(host);
	clk_disable_unprepare(pltfm_host->clk);
	sdhci_pltfm_free(pdev);
	return ret;
}

static int sdhci_hisi_remove(struct platform_device *pdev)
{
	return sdhci_pltfm_unregister(pdev);
}

static const struct of_device_id sdhci_hisi_match[] = {
	    { .compatible = "hisi-sdhci" },
		{ }
};
MODULE_DEVICE_TABLE(of, sdhci_hisi_match);

static struct platform_driver sdhci_hisi_driver = {
	.driver		= {
		.name	= "sdhci-hisi",
		.of_match_table = sdhci_hisi_match,
		.pm	= &sdhci_pltfm_pmops,
	},
	.probe		= sdhci_hisi_probe,
	.remove		= sdhci_hisi_remove,
};

static int __init sdhci_hisi_init(void)
{
	int ret;

	ret = platform_driver_register(&sdhci_hisi_driver);
	if (ret)
		return ret;

	ret = mci_proc_init();
	if (ret)
		platform_driver_unregister(&sdhci_hisi_driver);

	return ret;
}

static void __exit sdhci_hisi_exit(void)
{
	mci_proc_shutdown();

	platform_driver_unregister(&sdhci_hisi_driver);
}

module_init(sdhci_hisi_init);
module_exit(sdhci_hisi_exit);

MODULE_DESCRIPTION("SDHCI driver for hisi");
MODULE_AUTHOR("HiSilicon Technologies Co., Ltd..");
MODULE_LICENSE("GPL v2");
