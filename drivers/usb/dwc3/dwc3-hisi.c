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
#include <linux/module.h>
#include <linux/phy/phy.h>
#include <linux/of_address.h>
#include <linux/usb/ch9.h>
#include <linux/slab.h>

#include "dwc3-hisi.h"

#define USB3_CTRL           0x190
#define REG_SYS_STAT		0x8c
#define PCIE_USB3_MODE_MASK	0x3 << 12
#define USB3_PCLK_OCC_SEL	0x1 << 30

/* hi3559a */
#if defined(CONFIG_ARCH_HI3559AV100)
#define PERI_CRG			0x12010000
#define SYS_CTRL			0x12020000
#endif
#define DOUBLE_PCIE_MODE    0x0
#define P0_PCIE_ADD_P1_USB3 0x1 << 12
#define DOUBLE_USB3         0x2 << 12

/* hi3556a */
#if defined(CONFIG_ARCH_HI3556AV100) || defined(CONFIG_ARCH_HI3519AV100)
#define PERI_CRG			0x04510000
#define SYS_CTRL			0x04520000
#endif
#define PCIE_X1_MODE		0x0
#define USB3_MODE			0x1

/* hi3516cv500/hi3516dv300/hi3556v200/hi3559v200 */
#if defined(CONFIG_ARCH_HI3516CV500) || defined(CONFIG_ARCH_HI3516DV300)
#define PERI_CRG			0x12010000
#define SYS_CTRL			0x12020000
#endif
#if	defined(CONFIG_ARCH_HI3556V200) || defined(CONFIG_ARCH_HI3559V200)
#define PERI_CRG			0x12010000
#define SYS_CTRL			0x12020000
#endif

static struct hi_priv	*priv;

/* hi3559av100:if in pcie mode,switch work speed to HS.*/
int hi3559a_set_speed_with_pcie(struct device *dev)
{
	int reg, ret;
	struct device_node *np = dev->of_node;

	/* allocate memory for hi3559a secret data and initialize it */
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->sys_ctrl = ioremap_nocache(SYS_CTRL, 0x1000);
	if (IS_ERR(priv->sys_ctrl)) {
		ret = PTR_ERR(priv->sys_ctrl);
		goto err;
	}

	priv->speed_id = -1;
	reg = readl(priv->sys_ctrl + REG_SYS_STAT);
	reg &= PCIE_USB3_MODE_MASK;

	switch(reg){
		case DOUBLE_PCIE_MODE:
			ret = USB_SPEED_HIGH;
		case P0_PCIE_ADD_P1_USB3:
			if (of_property_read_u32(np, "port_speed", &priv->speed_id))
				ret = USB_SPEED_UNKNOWN;
			if (priv->speed_id == 0)
				ret = USB_SPEED_HIGH;
			else if (priv->speed_id == 1)
				ret = USB_SPEED_SUPER;
			else
				ret = USB_SPEED_UNKNOWN;
		case DOUBLE_USB3:
			ret =  USB_SPEED_SUPER;
		default:
			ret = USB_SPEED_UNKNOWN;
	}
	iounmap(priv->sys_ctrl);

	return ret;
err:
	kfree(priv);
	return ret;
}

/* hi3556av100:if in pcie mode,switch work speed to HS.*/
int hi3556a_set_speed_with_pcie(struct device *dev)
{
	int reg, ret;

	/* allocate memory for hi3556a secret data and initialize it */
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->peri_crg = ioremap_nocache(PERI_CRG, 0x1000);
	if (IS_ERR(priv->peri_crg)) {
		ret = PTR_ERR(priv->peri_crg);
		goto err1;
	}

	priv->sys_ctrl = ioremap_nocache(SYS_CTRL, 0x1000);
	if (IS_ERR(priv->sys_ctrl)) {
		ret = PTR_ERR(priv->sys_ctrl);
		goto err2;
	}

	reg = readl(priv->sys_ctrl + REG_SYS_STAT);
	reg &= PCIE_USB3_MODE_MASK;

	if (reg == PCIE_X1_MODE) {
		reg = readl(priv->peri_crg + USB3_CTRL);
		reg |= USB3_PCLK_OCC_SEL;
		writel(reg, priv->peri_crg + USB3_CTRL);
		udelay(100);

		ret = USB_SPEED_HIGH;
	} else
		ret = usb_get_maximum_speed(dev);
	iounmap(priv->peri_crg);
	iounmap(priv->sys_ctrl);

	return ret;

err2:
	iounmap(priv->peri_crg);
err1:
	kfree(priv);
	return ret;
}

int usb_get_max_speed(struct device	*dev)
{
#if defined(CONFIG_ARCH_HI3559AV100)
	return hi3559a_set_speed_with_pcie(dev);
#endif
#if defined(CONFIG_ARCH_HI3556AV100) || defined(CONFIG_ARCH_HI3519AV100)
	return hi3556a_set_speed_with_pcie(dev);
#endif
	return usb_get_maximum_speed(dev);
}
EXPORT_SYMBOL(usb_get_max_speed);

void hisi_dwc3_exited(void)
{
	kfree(priv);
}
EXPORT_SYMBOL(hisi_dwc3_exited);
MODULE_LICENSE("GPL v2");
