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
#include "board-mx6q_phytec-nand.h"

/* The GPMI is conflicted with SD3, so init this in the driver. */
static iomux_v3_cfg_t mx6q_gpmi_nand[] __initdata = {
	MX6Q_PAD_NANDF_CLE__RAWNAND_CLE,
	MX6Q_PAD_NANDF_ALE__RAWNAND_ALE,
	MX6Q_PAD_NANDF_CS0__RAWNAND_CE0N,
	MX6Q_PAD_NANDF_CS1__RAWNAND_CE1N,
	MX6Q_PAD_NANDF_CS2__RAWNAND_CE2N,
	MX6Q_PAD_NANDF_CS3__RAWNAND_CE3N,
	MX6Q_PAD_NANDF_RB0__RAWNAND_READY0,
	MX6Q_PAD_NANDF_D0__RAWNAND_D0,
	MX6Q_PAD_NANDF_D1__RAWNAND_D1,
	MX6Q_PAD_NANDF_D2__RAWNAND_D2,
	MX6Q_PAD_NANDF_D3__RAWNAND_D3,
	MX6Q_PAD_NANDF_D4__RAWNAND_D4,
	MX6Q_PAD_NANDF_D5__RAWNAND_D5,
	MX6Q_PAD_NANDF_D6__RAWNAND_D6,
	MX6Q_PAD_NANDF_D7__RAWNAND_D7,
	MX6Q_PAD_SD4_CMD__RAWNAND_RDN,
	MX6Q_PAD_SD4_CLK__RAWNAND_WRN,
	MX6Q_PAD_NANDF_WP_B__RAWNAND_RESETN,
	MX6Q_PAD_SD4_DAT0__RAWNAND_D8,
	MX6Q_PAD_SD4_DAT1__RAWNAND_D9,
	MX6Q_PAD_SD4_DAT2__RAWNAND_D10,
	MX6Q_PAD_SD4_DAT3__RAWNAND_D11,
	MX6Q_PAD_SD4_DAT4__RAWNAND_D12,
	MX6Q_PAD_SD4_DAT5__RAWNAND_D13,
	MX6Q_PAD_SD4_DAT6__RAWNAND_D14,
	MX6Q_PAD_SD4_DAT7__RAWNAND_D15,
};

static int __init gpmi_nand_platform_init(void)
{
	return mxc_iomux_v3_setup_multiple_pads(mx6q_gpmi_nand, ARRAY_SIZE(mx6q_gpmi_nand));
}

/*  To be shure with sizes on bootloader, please check definitions in
    barebox-phytec/arch/arm/boards/phyflex-imx6/board.c
	NAND_PART_BAREBOX_SIZE
	NAND_PART_BAREBOXENV_SIZE
	NAND_PART_KERNEL_SIZE
	NAND_PART_ROOT_SIZE
*/
static struct mtd_partition phyflex_nand_partitions[] = {
	{
		.name	= "bootloader",
		.offset	= 0,
		.size	= SZ_2M,
	}, {
		.name	= "env",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= SZ_512K,
	}, {
		.name	= "kernel",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= SZ_8M,
	}, {
		.name	= "filesystem",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= MTDPART_SIZ_FULL,
	},
};

static const struct gpmi_nand_platform_data mx6_gpmi_nand_platform_data __initconst = {
	.platform_init           = gpmi_nand_platform_init,
	.min_prop_delay_in_ns    = 5,
	.max_prop_delay_in_ns    = 9,
	.max_chip_count          = 1,
	.partitions              = phyflex_nand_partitions,
	.partition_count         = ARRAY_SIZE(phyflex_nand_partitions),
	.enable_bbt              = 1,
};

void __init board_nand_init(void)
{
	imx6q_add_gpmi(&mx6_gpmi_nand_platform_data);
};
