/*
 * linux/regulator/ricoh568-regulator.h
 *
 * Regulator driver for RICOH568 power management chip.
 *
 * Copyright (C) 2012-2016 RICOH COMPANY,LTD
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
 
#ifndef __LINUX_REGULATOR_RICOH568_H
#define __LINUX_REGULATOR_RICOH568_H

#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>

#define ricoh568_rails(_name) "RICOH568_"#_name

/* RICHOH Regulator IDs */
enum regulator_id {
	RICOH568_ID_DC1,
	RICOH568_ID_DC2,	
	RICOH568_ID_DC3,
	RICOH568_ID_DC4,
	RICOH568_ID_LDO1,
	RICOH568_ID_LDO2,
	RICOH568_ID_LDO3,
	RICOH568_ID_LDO4,
	RICOH568_ID_LDO5,
};

struct ricoh568_regulator_platform_data {
		struct regulator_init_data regulator;
		int init_uV;
		unsigned init_enable:1;
		unsigned init_apply:1;
		int sleep_uV;
		int sleep_slots;
		unsigned long ext_pwr_req;
		unsigned long flags;
};

#endif
