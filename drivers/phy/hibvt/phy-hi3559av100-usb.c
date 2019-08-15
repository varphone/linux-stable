/*
* Copyright (c) 2017 HiSilicon Technologies Co., Ltd.
*
* This program is free software; you can redistribute  it and/or modify it
* under  the terms of  the GNU General Public License as published by the
* Free Software Foundation;  either version 2 of the  License, or (at your
* option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*/
#include <linux/delay.h>
#include <linux/phy/phy.h>
#include <linux/of_address.h>
#include <linux/usb/ch9.h>
#include "phy-hisi-usb.h"

#define USB2_PHY                0x184
#define USB2_PHY_CKEN          (0x1 << 5)
#define USB2_PHY_PORT_TREQ     (0x1 << 3)
#define USB2_PHY_REQ           (0x1 << 1)

#define USB3_COMBPHY            0x188
#define COMBPHY_REF_CKEN      (0x1 << 8)
#define COMBPHY_SRST_REQ      (0x1 << 0)

#define USB3_CTRL               0x190
#define USB3_VCC_SRST_REQ     (0x1 << 16)
#define USB3_UTMI_CKSEL       (0x1 << 29)
#define USB3_PCLK_OCC_SEL     (0x1 << 30)

#define GTXTHRCFG               0xc108
#define GRXTHRCFG               0xc10c
#define REG_GCTL                0xc110

#define USB_TXPKT_CNT_SEL       (0x1 << 29)
#define USB_TXPKT_CNT           (0x11 << 24)
#define USB_MAXTX_BURST_SIZE    (0x1 << 20)
#define CLEAN_USB3_GTXTHRCFG    0x0

#define REG_GUSB3PIPECTL0   0xc2c0
#define PCS_SSP_SOFT_RESET  (0x1 << 31)

#define USB3_COMB_PHY			0x14
#define P0_TX_SWING_COMP_CFG	0xd13
#define P0_TX_SWING_COMP_RCFG	0xd53
#define P0_TX_SWING_COMP_VAL	0xd13

#define P0_TX_TERMINATION_TRIM_CFG	0x14
#define P0_TX_TERMINATION_TRIM_RCFG	0x54
#define P0_TX_TERMINATION_TRIM_VAL	0x14

#define P1_TX_SWING_COMP_CFG	0xd33
#define P1_TX_SWING_COMP_RCFG	0xd73
#define P1_TX_SWING_COMP_VAL	0xd33

#define P1_TX_TERMINATION_TRIM_CFG	0x34
#define P1_TX_TERMINATION_TRIM_RCFG	0x74
#define P1_TX_TERMINATION_TRIM_VAL	0x34

#define USB2_PHY0_CTRL	0x24
#define USB2_PHY1_CTRL	0x30
#define USB2_PHY_VREF	(0x5 << 4)
#define USB2_PHY_PRE	(0x3 << 12)

#define USB_PORT0   0x38
#define P0_U3_PORT_DISABLE  (0x1<<3)
#define USB_PORT1   0x3c
#define P1_U3_PORT_DISABLE  (0x1<<3)

#define DOUBLE_PCIE_MODE    0x0
#define P0_PCIE_ADD_P1_USB3 0x1 << 12
#define DOUBLE_USB3         0x2 << 12
#define COMPARE_COMBPHY     0x3 << 12
#define GET_COMBPHY_MODE	0x8c

#define USB3_PORT0_CLK		0x1 << 30
#define USB3_PORT1_CLK		0x1 << 14

void hisi_switch_func(int otg)
{

}
EXPORT_SYMBOL(hisi_switch_func);

void hisi_usb3_crg_config(struct phy *phy, int u2_offset, int u3_offset)
{
	int reg;
	struct hisi_priv *priv = phy_get_drvdata(phy);

	reg = readl(priv->peri_ctrl + USB3_CTRL);
	reg |= (USB3_VCC_SRST_REQ >> u3_offset);
	writel(reg, priv->peri_ctrl + USB3_CTRL);
	udelay(500);
	reg = readl(priv->peri_ctrl + USB3_COMBPHY);
	reg |= (COMBPHY_SRST_REQ << u3_offset);
	writel(reg, priv->peri_ctrl + USB3_COMBPHY);
	udelay(100);

	/*release TPOR default release*/
	reg = readl(priv->peri_ctrl + USB2_PHY);
	reg &= ~(USB2_PHY_PORT_TREQ >> u2_offset);
	writel(reg, priv->peri_ctrl + USB2_PHY);
	udelay(200);

	/*utmi clock sel*/
	reg = readl(priv->peri_ctrl + USB3_CTRL);
	reg &= ~(USB3_UTMI_CKSEL >> u3_offset);
	writel(reg, priv->peri_ctrl + USB3_CTRL);
	udelay(200);

	/*open phy ref clk default open*/
	reg = readl(priv->peri_ctrl + USB2_PHY);
	reg |= (USB2_PHY_CKEN << u2_offset);
	writel(reg, priv->peri_ctrl + USB2_PHY);
	udelay(200);

	/*U2 phy reset release*/
	reg = readl(priv->peri_ctrl + USB2_PHY);
	reg &= ~(USB2_PHY_REQ >> u2_offset);
	writel(reg, priv->peri_ctrl + USB2_PHY);
	udelay(200);

	/*usb3 occ pclk sel*/
	reg = readl(priv->peri_ctrl + USB3_CTRL);
	reg &= ~(USB3_PCLK_OCC_SEL >> u3_offset);
	writel(reg, priv->peri_ctrl + USB3_CTRL);

	/*open ref CKEN*/
	reg = readl(priv->peri_ctrl + USB3_COMBPHY);
	reg |= (COMBPHY_REF_CKEN << u3_offset);
	writel(reg, priv->peri_ctrl + USB3_COMBPHY);
	udelay(100);

	/*U3 PHY reset release*/
	reg = readl(priv->peri_ctrl + USB3_COMBPHY);
	reg &= ~(COMBPHY_SRST_REQ << u3_offset);
	writel(reg, priv->peri_ctrl + USB3_COMBPHY);
	udelay(100);

	/*config U3 Controller USB3_0 PHY OUTPUT*/
	reg = readl(priv->peri_ctrl + USB3_CTRL);
	reg &= ~(USB3_VCC_SRST_REQ >> u3_offset);
	writel(reg, priv->peri_ctrl + USB3_CTRL);
	udelay(500);
}

void hisi_usb3_ctrl_config(struct phy *phy)
{
	int reg;
	struct hisi_priv *priv = phy_get_drvdata(phy);

	reg = readl(priv->dwc3_ctrl + REG_GUSB3PIPECTL0);
	reg |= PCS_SSP_SOFT_RESET;
	writel(reg, priv->dwc3_ctrl + REG_GUSB3PIPECTL0);
	udelay(200);
	reg = readl(priv->dwc3_ctrl + REG_GCTL);
	reg &= ~(0x3<<12);
	reg |= (0x1<<12); /*[13:12] 01: Host; 10: Device; 11: OTG*/
	writel(reg, priv->dwc3_ctrl + REG_GCTL);
	udelay(20);

	reg = readl(priv->dwc3_ctrl + REG_GUSB3PIPECTL0);
	reg &= ~PCS_SSP_SOFT_RESET;
	reg &= ~(1<<17);       //disable suspend
	writel(reg, priv->dwc3_ctrl + REG_GUSB3PIPECTL0);
	udelay(20);

	reg &= CLEAN_USB3_GTXTHRCFG;
	reg |= USB_TXPKT_CNT_SEL;
	reg |= USB_TXPKT_CNT;
	reg |= USB_MAXTX_BURST_SIZE;
	writel(reg, priv->dwc3_ctrl + GTXTHRCFG);
	udelay(20);
	writel(reg, priv->dwc3_ctrl + GRXTHRCFG);
	udelay(20);

}

void hisi_usb3_eye_config(struct phy *phy)
{
	int reg;
	struct hisi_priv *priv = phy_get_drvdata(phy);

	/* Port0 usb2 phy0 misc ctrl */
	reg = readl(priv->misc_ctrl + USB2_PHY0_CTRL);
	reg &= ~(0xf << 4);
	reg |= USB2_PHY_VREF;	/* [7:4] -> (eye vref = 4%) */
	reg |= USB2_PHY_PRE;	/* [13:12] -> (pre electric = 3x) */
	writel(reg, priv->misc_ctrl + USB2_PHY0_CTRL);
	udelay(50);

	/* Port1 usb2 phy1 misc ctrl */
	reg = readl(priv->misc_ctrl + USB2_PHY1_CTRL);
	reg &= ~(0xf << 4);
	reg |= USB2_PHY_VREF;	/* [7:4] -> (eye vref = 4%) */
	reg |= USB2_PHY_PRE;	/* [13:12] -> (pre electric = 3x) */
	writel(reg, priv->misc_ctrl + USB2_PHY1_CTRL);
	udelay(50);

	/* Port0 TX swing comp => 0x1101 */
	writel(P0_TX_SWING_COMP_CFG, priv->misc_ctrl + USB3_COMB_PHY);
	writel(P0_TX_SWING_COMP_RCFG, priv->misc_ctrl + USB3_COMB_PHY);
	writel(P0_TX_SWING_COMP_VAL, priv->misc_ctrl + USB3_COMB_PHY);
	udelay(50);

	/* Port0 termination trim => 0x0000 */
	writel(P0_TX_TERMINATION_TRIM_CFG, priv->misc_ctrl + USB3_COMB_PHY);
	writel(P0_TX_TERMINATION_TRIM_RCFG, priv->misc_ctrl + USB3_COMB_PHY);
	writel(P0_TX_TERMINATION_TRIM_VAL, priv->misc_ctrl + USB3_COMB_PHY);
	udelay(50);

	/* Port1 TX swing comp => 0x1101 */
	writel(P1_TX_SWING_COMP_CFG, priv->misc_ctrl + USB3_COMB_PHY);
	writel(P1_TX_SWING_COMP_RCFG, priv->misc_ctrl + USB3_COMB_PHY);
	writel(P1_TX_SWING_COMP_VAL, priv->misc_ctrl + USB3_COMB_PHY);
	udelay(50);

	/* Port1 termination trim => 0x0000 */
	writel(P1_TX_TERMINATION_TRIM_CFG, priv->misc_ctrl + USB3_COMB_PHY);
	writel(P1_TX_TERMINATION_TRIM_RCFG, priv->misc_ctrl + USB3_COMB_PHY);
	writel(P1_TX_TERMINATION_TRIM_VAL, priv->misc_ctrl + USB3_COMB_PHY);
	udelay(50);
}

void hisi_usb3_phy_on(struct phy *phy)
{
	int u2 = 0, u3 = 0;
	struct hisi_priv *priv = phy_get_drvdata(phy);

	if (1 == priv->phyid) {
		u2 = 1;
		u3 = 16;
	}

	hisi_usb3_crg_config(phy, u2, u3);

	hisi_usb3_ctrl_config(phy);

	hisi_usb3_eye_config(phy);

	multi_cfg_f_pcie(phy);
}
EXPORT_SYMBOL(hisi_usb3_phy_on);

void hisi_usb3_phy_off(struct phy *phy)
{
	int reg, usb2_offset, usb3_offset;
	struct hisi_priv *priv = phy_get_drvdata(phy);

	if (1 == priv->phyid) {
		usb2_offset = 1;
		usb3_offset = 16;
	}
	if (0 == priv->phyid) {
		usb2_offset = 0;
		usb3_offset = 0;
	}

	reg = readl(priv->peri_ctrl + USB3_CTRL);
	reg |= (USB3_VCC_SRST_REQ >> usb3_offset);
	writel(reg, priv->peri_ctrl + USB3_CTRL);
	udelay(500);

	reg = readl(priv->peri_ctrl + USB3_COMBPHY);
	reg |= (COMBPHY_SRST_REQ << usb3_offset);
	writel(reg, priv->peri_ctrl + USB3_COMBPHY);
	udelay(100);
}
EXPORT_SYMBOL(hisi_usb3_phy_off);

void multi_cfg_f_pcie(struct phy *phy)
{
	int reg;
	struct hisi_priv	*priv = phy_get_drvdata(phy);

	reg = readl(priv->combphy_base + GET_COMBPHY_MODE);
	reg &= COMPARE_COMBPHY;

	if (reg == P0_PCIE_ADD_P1_USB3){
		reg = readl(priv->peri_ctrl + USB3_CTRL);
		reg |= USB3_PORT0_CLK;
		writel(reg, priv->peri_ctrl + USB3_CTRL);
		udelay(100);
	}

	if (reg == DOUBLE_PCIE_MODE){
		reg = readl(priv->peri_ctrl + USB3_CTRL);
		reg |= USB3_PORT1_CLK;
		reg |= USB3_PORT0_CLK;
		writel(reg, priv->peri_ctrl + USB3_CTRL);
		udelay(100);
	}
}
EXPORT_SYMBOL(multi_cfg_f_pcie);

int hisi_usb3_init_para(struct phy *phy, struct device_node *np)
{
	struct hisi_priv *priv = phy_get_drvdata(phy);

	priv->combphy_base = of_iomap(np, 2);
	if (IS_ERR(priv->combphy_base))
		priv->combphy_base = NULL;

	priv->dwc3_ctrl = of_iomap(np, 3);
	if (IS_ERR(priv->dwc3_ctrl))
		priv->dwc3_ctrl = NULL;

	if (of_property_read_u32(np, "phyid", &priv->phyid))
		return -EINVAL;
	
	return 0;
}
EXPORT_SYMBOL(hisi_usb3_init_para);
