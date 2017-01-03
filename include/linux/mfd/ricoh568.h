/* 
 * include/linux/mfd/ricoh568.h
 *
 * Core driver interface to access RICOH RN5T568 power management chip.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#ifndef __LINUX_MFD_RICOH568_H
#define __LINUX_MFD_RICOH568_H

#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/i2c.h>

/* Maximum number of main interrupts */
#define MAX_INTERRUPT_MASKS	4
#define MAX_MAIN_INTERRUPT	7
#define MAX_GPEDGE_REG		2

/* Power control register */
#define RICOH568_PWR_WD			0x0B
#define RICOH568_PWR_WD_COUNT		0x0C
#define RICOH568_PWR_FUNC		0x0D
#define RICOH568_PWR_SLP_CNT		0x0E
#define RICOH568_PWR_REP_CNT		0x0F
#define RICOH568_PWR_ON_TIMSET		0x10
#define RICOH568_PWR_NOE_TIMSET		0x11
#define RICOH568_PWR_IRSEL		0x15

#define RICOH568_INT_ILIMTHL            0x78

/* Interrupt enable register */
#define RICOH568_INT_EN_SYS		0x12
#define RICOH568_INT_EN_DCDC		0x40
#define RICOH568_INT_EN_GPIO		0x94

/* Interrupt select register */
#define RICOH568_PWR_IRSEL			0x15

/* interrupt status registers (monitor regs)*/
#define RICOH568_INTC_INTPOL		0x9C
#define RICOH568_INTC_INTEN		0x9D
#define RICOH568_INTC_INTMON		0x9E

#define RICOH568_INT_MON_SYS		0x14
#define RICOH568_INT_MON_DCDC		0x42

/* interrupt clearing registers */
#define RICOH568_INT_IR_SYS		0x13
#define RICOH568_INT_IR_DCDC		0x41
#define RICOH568_INT_IR_GPIOR		0x95
#define RICOH568_INT_IR_GPIOF		0x96

/* GPIO register base address */
#define RICOH568_GPIO_IOSEL		0x90
#define RICOH568_GPIO_IOOUT		0x91
#define RICOH568_GPIO_GPEDGE1		0x92
#define RICOH568_GPIO_GPEDGE2		0x93
//#define RICOH568_GPIO_EN_GPIR		0x94
//#define RICOH568_GPIO_IR_GPR		0x95
//#define RICOH568_GPIO_IR_GPF		0x96
#define RICOH568_GPIO_MON_IOIN		0x97
#define RICOH568_GPIO_LED_FUNC		0x98

#define	RICOH568_PSWR			0x07

#define RICOH_DC1_SLOT 0x16
#define RICOH_DC2_SLOT 0x17
#define RICOH_DC3_SLOT 0x18
#define RICOH_DC4_SLOT 0x19

#define RICOH_LDO1_SLOT 0x1b
#define RICOH_LDO2_SLOT 0x1c
#define RICOH_LDO3_SLOT 0x1d
#define RICOH_LDO4_SLOT 0x1e
#define RICOH_LDO5_SLOT 0x1f

#define 	RICOH568_NUM_REGULATOR 9

/* RICOH568 IRQ definitions */
enum {
	RICOH568_IRQ_POWER_ON,
	RICOH568_IRQ_EXTIN,
	RICOH568_IRQ_PRE_VINDT,
	RICOH568_IRQ_PREOT,
	RICOH568_IRQ_POWER_OFF,
	RICOH568_IRQ_NOE_OFF,
	RICOH568_IRQ_WD,

	RICOH568_IRQ_DC1LIM,
	RICOH568_IRQ_DC2LIM,
	RICOH568_IRQ_DC3LIM,
	RICOH568_IRQ_DC4LIM,
	
	RICOH568_IRQ_GPIO0,
	RICOH568_IRQ_GPIO1,
	RICOH568_IRQ_GPIO2,
	RICOH568_IRQ_GPIO3,

	/* Should be last entry */
	RICOH568_NR_IRQS,
};

/* Ricoh568 gpio definitions */
enum {
	RICOH568_GPIO0,
	RICOH568_GPIO1,
	RICOH568_GPIO2,
	RICOH568_GPIO3,
};

enum ricoh568_sleep_control_id {
	RICOH568_DS_DC1,
	RICOH568_DS_DC2,
	RICOH568_DS_DC3,
	RICOH568_DS_DC4,
	RICOH568_DS_LDO1,
	RICOH568_DS_LDO2,
	RICOH568_DS_LDO3,
	RICOH568_DS_LDO4,
	RICOH568_DS_LDO5,
};

struct ricoh568_subdev_info {
	int		id;
	const char	*name;
	void		*platform_data;
};

struct ricoh568_gpio_init_data {
	unsigned output_mode_en:1; 	/* Enable output mode during init */
	unsigned output_val:1;  	/* Output value if it is in output mode */
	unsigned init_apply:1;  	/* Apply init data on configuring gpios*/
	unsigned led_mode:1;  		/* Select LED mode during init */
	unsigned led_func:1;  		/* Set LED function if LED mode is 1 */
};

struct ricoh568 {
	struct device		*dev;
	struct i2c_client	*client;
	struct mutex		io_lock;
	int			gpio_base;
	struct gpio_chip	gpio_chip;
	int			irq_base;
//	struct irq_chip		irq_chip;
	int			chip_irq;
	struct mutex		irq_lock;
	unsigned long		group_irq_en[MAX_MAIN_INTERRUPT];

	/* For main interrupt bits in INTC */
	u8			intc_inten_cache;
	u8			intc_inten_reg;

	/* For group interrupt bits and address */
	u8			irq_en_cache[MAX_INTERRUPT_MASKS];
	u8			irq_en_reg[MAX_INTERRUPT_MASKS];

	/* For gpio edge */
	u8			gpedge_cache[MAX_GPEDGE_REG];
	u8			gpedge_reg[MAX_GPEDGE_REG];

	int			bank_num;
	struct irq_domain *irq_domain;
};

struct ricoh568_platform_data {
	int		num_subdevs;
	struct	ricoh568_subdev_info *subdevs;
	int (*init_port)(int irq_num); // Init GPIO for IRQ pin
	int		gpio_base;
	int		irq_base;
	struct ricoh568_gpio_init_data *gpio_init_data;
	int num_gpioinit_data;
	bool enable_shutdown_pin;
	bool pm_off;
	struct regulator_init_data *reg_init_data[RICOH568_NUM_REGULATOR];
	int irq_gpio;
	int pmic_sleep_gpio; /* */
	bool pmic_sleep;
};

/* ==================================== */
/* RICOH568 Power_Key device data	*/
/* ==================================== */
struct ricoh568_pwrkey_platform_data {
	int irq;
	unsigned  delay_ms;
};
extern int ricoh568_pwrkey_wakeup;
extern struct ricoh568 *g_ricoh568;

extern int ricoh568_read(struct device *dev, uint8_t reg, uint8_t *val);
extern int ricoh568_bulk_reads(struct device *dev, u8 reg, u8 count,
								uint8_t *val);
extern int ricoh568_write(struct device *dev, u8 reg, uint8_t val);
extern int ricoh568_bulk_writes(struct device *dev, u8 reg, u8 count,
								uint8_t *val);
extern int ricoh568_set_bits(struct device *dev, u8 reg, uint8_t bit_mask);
extern int ricoh568_clr_bits(struct device *dev, u8 reg, uint8_t bit_mask);
extern int ricoh568_update(struct device *dev, u8 reg, uint8_t val,
								uint8_t mask);
extern int ricoh568_irq_init(struct ricoh568 *ricoh568, int irq, struct ricoh568_platform_data *pdata);
extern int ricoh568_irq_exit(struct ricoh568 *ricoh568);


#endif
