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
#include "board-mx6q_phytec-common.h"

/* Setup imx6 revision and uniq ID */
#define HW_OCOTP_DEVIDn(n)        (0x00000410 + (n) * 0x10)
void __init mx6_setup_cpuinfo(void)
{
	system_rev = mx6q_revision();
	system_serial_high = readl(MX6_IO_ADDRESS(OCOTP_BASE_ADDR) + HW_OCOTP_DEVIDn(0));
	system_serial_low  = readl(MX6_IO_ADDRESS(OCOTP_BASE_ADDR) + HW_OCOTP_DEVIDn(1));
}
