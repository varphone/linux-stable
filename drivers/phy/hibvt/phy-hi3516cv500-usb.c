#include <linux/delay.h>
#include <linux/phy/phy.h>
#include <linux/of_address.h>
#include <linux/usb/ch9.h>
#include "phy-hisi-usb.h"

#define CRG_BASE_REG			0x140
#define USB2_UTMI_PCTRL			(0x1 << 15)
#define USB2_PHY_TEST_SRST_REQ	(0x1 << 14)
#define USB2_UTMI_CKSEL			(0x1 << 13)
#define USB2_UTMI_CKEN			(0x1 << 12)
#define USB2_REF_CKEN			(0x1 << 9)
#define USB2_BUS_CKEN			(0x1 << 8)
#define USB2_VCC_SRST_REQ		(0x1 << 3)
#define USB2_PHY_CKEN			(0x1 << 2)
#define USB2_PHY_PORT_TREQ		(0x1 << 1)
#define USB2_PHY_REQ			(0x1 << 0)

#define CTRL_BASE_REG			0x100e0000

#define REG_GUSB3PIPECTL0		0xc2c0
#define PCS_SSP_SOFT_RESET		(0x1 << 31)
#define PORT_DISABLE_SUSPEND	(0x1 << 17)

#define REG_GCTL				0xc110
#define PORT_CAP_DIR			(0x3 << 12)
#define PORT_SET_HOST			(0x1 << 12)

#define GTXTHRCFG				0xc108
#define	USB2_G_TXTHRCFG			0x23100000

#define GRXTHRCFG				0xc10c
#define	USB2_G_RXTHRCFG			0x23100000

#define USB2_INNO_PHY_BASE_REG	0x10110000
#define	USB2_PHY_CLK_OUTPUT_REG	0x18
#define	USB2_PHY_CLK_OUTPUT_VAL	0x0c

#define	USB2_VBUS_IO_BASE_REG	0x10ff0000
#define	USB2_VBUS_IO_OFFSET		0x40
#define	USB2_VBUS_IO_VAL		0x431

#define	HS_HIGH_HEIGHT_TUNING_OFFSET	0x8
#define HS_HIGH_HEIGHT_TUNING_MASK	(0x7 << 4)
#define HS_HIGH_HEIGHT_TUNING_VAL	0x5 << 4

#define PRE_EMPHASIS_TUNING_OFFSET	0x0
#define PRE_EMPHASIS_TUNING_MASK	(0x7 << 0)
#define	PRE_EMPHASIS_TUNING_VAL		0x7 << 0

#define PRE_EMPHASIS_STRENGTH_OFFSET	0x14
#define PRE_EMPHASIS_STRENGTH_MASK	(0x7 << 2)
#define PRE_EMPHASIS_STRENGTH_VAL	0x3 << 2

#define HS_SLEW_RATE_TUNING_OFFSET	0x74
#define HS_SLEW_RATE_TUNING_MASK	(0x7 << 1)
#define HS_SLEW_RATE_TUNING_VAL		0x7 << 1

#define DISCONNECT_TRIGGER_OFFSET	0x10
#define DISCONNECT_TRIGGER_MASK		(0xf << 4)
#define DISCONNECT_TRIGGER_VAL		0xd << 4

void hisi_switch_func(int otg)
{

}
EXPORT_SYMBOL(hisi_switch_func);

static void usb_vbus_multi_gpio(void)
{
	void __iomem *vbus = ioremap_nocache(USB2_VBUS_IO_BASE_REG, 0x100);
	if (!vbus)
		return ;

	writel(USB2_VBUS_IO_VAL, vbus + USB2_VBUS_IO_OFFSET);
	udelay(20);

	iounmap(vbus);
}

static void usb_crg_c(struct phy *phy)
{
	int reg;
	struct hisi_priv *priv = phy_get_drvdata(phy);

	void __iomem *inno_clk_output = ioremap_nocache(USB2_INNO_PHY_BASE_REG, 0x100);
	if (!inno_clk_output)
		return ;

	/* usb phy reset */
	reg = readl(priv->peri_ctrl + CRG_BASE_REG);
	reg |= USB2_PHY_TEST_SRST_REQ;
	writel(reg, priv->peri_ctrl + CRG_BASE_REG);
	udelay(100);

	/* cancel usb phy srst */
	reg = readl(priv->peri_ctrl + CRG_BASE_REG);
	reg &= ~USB2_PHY_TEST_SRST_REQ;
	writel(reg, priv->peri_ctrl + CRG_BASE_REG);
	udelay(20);

	/* usb2 vcc reset */
	reg = readl(priv->peri_ctrl + CRG_BASE_REG);
	reg |= USB2_VCC_SRST_REQ;
	writel(reg, priv->peri_ctrl + CRG_BASE_REG);
	udelay(200);

	/* set inno phy output clock */
	writel(USB2_PHY_CLK_OUTPUT_VAL, inno_clk_output +
			USB2_PHY_CLK_OUTPUT_REG);
	udelay(10);

	/* open phy ref cken */
	reg = readl(priv->peri_ctrl + CRG_BASE_REG);
	reg |= USB2_PHY_CKEN;
	writel(reg, priv->peri_ctrl + CRG_BASE_REG);
	udelay(10);

	/* open utmi pctrl */
	reg = readl(priv->peri_ctrl + CRG_BASE_REG);
	reg &= ~USB2_UTMI_PCTRL;
	writel(reg, priv->peri_ctrl + CRG_BASE_REG);
	udelay(10);

	/* open utmi cksel */
	reg = readl(priv->peri_ctrl + CRG_BASE_REG);
	reg &= ~USB2_UTMI_CKSEL;
	writel(reg, priv->peri_ctrl + CRG_BASE_REG);
	udelay(10);

	/* open utmi cken */
	reg = readl(priv->peri_ctrl + CRG_BASE_REG);
	reg |= USB2_UTMI_CKEN;
	writel(reg, priv->peri_ctrl + CRG_BASE_REG);
	udelay(10);

	/* open controller ref cken */
	reg = readl(priv->peri_ctrl + CRG_BASE_REG);
	reg |= USB2_REF_CKEN;
	writel(reg, priv->peri_ctrl + CRG_BASE_REG);
	udelay(10);

	/* open bus cken */
	reg = readl(priv->peri_ctrl + CRG_BASE_REG);
	reg |= USB2_BUS_CKEN;
	writel(reg, priv->peri_ctrl + CRG_BASE_REG);
	udelay(200);

	/* cancel POR reset */
	reg = readl(priv->peri_ctrl + CRG_BASE_REG);
	reg &= ~USB2_PHY_REQ;
	writel(reg, priv->peri_ctrl + CRG_BASE_REG);
	udelay(200);

	/* cancel TPOR reset */
	reg = readl(priv->peri_ctrl + CRG_BASE_REG);
	reg &= ~USB2_PHY_PORT_TREQ;
	writel(reg, priv->peri_ctrl + CRG_BASE_REG);
	udelay(200);

	/* cancel vcc reset */
	reg = readl(priv->peri_ctrl + CRG_BASE_REG);
	reg &= ~USB2_VCC_SRST_REQ;
	writel(reg, priv->peri_ctrl + CRG_BASE_REG);
	udelay(200);

	iounmap(inno_clk_output);
}

static void usb_ctrl_c(struct phy *phy)
{
	int reg;
	struct hisi_priv *priv = phy_get_drvdata(phy);

	priv->ctrl_base = ioremap_nocache(CTRL_BASE_REG, 0x10000);
	if (!priv->ctrl_base)
		return ;

	reg = readl(priv->ctrl_base + REG_GUSB3PIPECTL0);
	reg |= PCS_SSP_SOFT_RESET;
	writel(reg, priv->ctrl_base + REG_GUSB3PIPECTL0);
	udelay(20);

	reg = readl(priv->ctrl_base + REG_GCTL);
	reg &= ~PORT_CAP_DIR;
	reg |= PORT_SET_HOST; /*[13:12] 01: Host; 10: Device; 11: OTG*/
	writel(reg, priv->ctrl_base + REG_GCTL);
	udelay(20);

	reg = readl(priv->ctrl_base + REG_GUSB3PIPECTL0);
	reg &= ~PCS_SSP_SOFT_RESET;
	reg &= ~PORT_DISABLE_SUSPEND;       //disable suspend
	writel(reg, priv->ctrl_base + REG_GUSB3PIPECTL0);
	udelay(20);

	writel(USB2_G_TXTHRCFG, priv->ctrl_base + GTXTHRCFG);
	writel(USB2_G_RXTHRCFG, priv->ctrl_base + GRXTHRCFG);
	udelay(20);

	iounmap(priv->ctrl_base);
}

static void usb_eye_c(struct phy *phy)
{
	int reg;

	void __iomem *inno_base = ioremap_nocache(USB2_INNO_PHY_BASE_REG, 0x100);
	if (!inno_base)
		return ;

	/* HS eye height tuning */
	reg = readl(inno_base + HS_HIGH_HEIGHT_TUNING_OFFSET);
	reg &= ~HS_HIGH_HEIGHT_TUNING_MASK;
	reg |= HS_HIGH_HEIGHT_TUNING_VAL;
	writel(reg, inno_base + HS_HIGH_HEIGHT_TUNING_OFFSET);

	/* Pre-emphasis tuning */
	reg = readl(inno_base + PRE_EMPHASIS_TUNING_OFFSET);
	reg &= ~PRE_EMPHASIS_TUNING_MASK;
	reg |= PRE_EMPHASIS_TUNING_VAL;
	writel(reg, inno_base + PRE_EMPHASIS_TUNING_OFFSET);

	/* Pre-emphasis strength */
	reg = readl(inno_base + PRE_EMPHASIS_STRENGTH_OFFSET);
	reg &= ~PRE_EMPHASIS_STRENGTH_MASK;
	reg |= PRE_EMPHASIS_STRENGTH_VAL;
	writel(reg, inno_base + PRE_EMPHASIS_STRENGTH_OFFSET);

	/* HS driver slew rate tunning */
	reg = readl(inno_base + HS_SLEW_RATE_TUNING_OFFSET);
	reg &= ~HS_SLEW_RATE_TUNING_MASK;
	reg |= HS_SLEW_RATE_TUNING_VAL;
	writel(reg, inno_base + HS_SLEW_RATE_TUNING_OFFSET);

	/* HOST disconnects detection trigger point */
	reg = readl(inno_base + DISCONNECT_TRIGGER_OFFSET);
	reg &= ~DISCONNECT_TRIGGER_MASK;
	reg |= DISCONNECT_TRIGGER_VAL;
	writel(reg, inno_base + DISCONNECT_TRIGGER_OFFSET);
}

void hisi_usb_phy_on(struct phy *phy)
{
	usb_crg_c(phy);

	usb_vbus_multi_gpio();

	usb_eye_c(phy);

	usb_ctrl_c(phy);
}
EXPORT_SYMBOL(hisi_usb_phy_on);

void hisi_usb_phy_off(struct phy *phy)
{
	int reg;
	struct hisi_priv *priv = phy_get_drvdata(phy);

	/* usb2 vcc reset */
	reg = readl(priv->peri_ctrl + CRG_BASE_REG);
	reg |= USB2_VCC_SRST_REQ;
	writel(reg, priv->peri_ctrl + CRG_BASE_REG);
	udelay(200);
}
EXPORT_SYMBOL(hisi_usb_phy_off);
