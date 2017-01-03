/*
 * driver/mfd/ricoh568-irq.c
 *
 * Interrupt driver for RICOH RN5T568 power management chip.
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
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/mfd/ricoh568.h>
#include <linux/irqdomain.h>

static DEFINE_MUTEX(int_flag_mutex);

enum int_type {
	SYS_INT  = 0x1,
	DCDC_INT = 0x2,
	GPIO_INT = 0x10,
};

static int gpedge_add[] = {
	RICOH568_GPIO_GPEDGE1,
	RICOH568_GPIO_GPEDGE2
};

static int irq_en_add[] = {
	RICOH568_INT_EN_SYS,
	RICOH568_INT_EN_DCDC,
	RICOH568_INT_EN_GPIO,
	RICOH568_INT_EN_GPIO,
};

static int irq_mon_add[] = {
	RICOH568_INT_IR_SYS, //RICOH568_INT_MON_SYS,
	RICOH568_INT_IR_DCDC, //RICOH568_INT_MON_DCDC,
	RICOH568_INT_IR_GPIOR,
	RICOH568_INT_IR_GPIOF,
};

static int irq_clr_add[] = {
	RICOH568_INT_IR_SYS,
	RICOH568_INT_IR_DCDC,
	RICOH568_INT_IR_GPIOR,
	RICOH568_INT_IR_GPIOF,
};

static int main_int_type[] = {
	SYS_INT,
	DCDC_INT,
	GPIO_INT,
};

struct ricoh568_irq_data {
	u8	int_type;
	u8	master_bit;
	u8	int_en_bit;
	u8	mask_reg_index;
	int	grp_index;
};

#define RICOH568_IRQ(_int_type, _master_bit, _grp_index, _int_bit, _mask_ind) \
	{						\
		.int_type	= _int_type,		\
		.master_bit	= _master_bit,		\
		.grp_index	= _grp_index,		\
		.int_en_bit	= _int_bit,		\
		.mask_reg_index	= _mask_ind,		\
	}

static const struct ricoh568_irq_data ricoh568_irqs[RICOH568_NR_IRQS] = {
	[RICOH568_IRQ_POWER_ON]		= RICOH568_IRQ(SYS_INT,  0, 0, 0, 0),
	[RICOH568_IRQ_EXTIN]		= RICOH568_IRQ(SYS_INT,  0, 1, 1, 0),
	[RICOH568_IRQ_PRE_VINDT]	= RICOH568_IRQ(SYS_INT,  0, 2, 2, 0),
	[RICOH568_IRQ_PREOT]		= RICOH568_IRQ(SYS_INT,  0, 3, 3, 0),
	[RICOH568_IRQ_POWER_OFF]	= RICOH568_IRQ(SYS_INT,  0, 4, 4, 0),
	[RICOH568_IRQ_NOE_OFF]		= RICOH568_IRQ(SYS_INT,  0, 5, 5, 0),
	[RICOH568_IRQ_WD]		= RICOH568_IRQ(SYS_INT,  0, 6, 6, 0),

	[RICOH568_IRQ_DC1LIM]		= RICOH568_IRQ(DCDC_INT, 1, 0, 0, 1),
	[RICOH568_IRQ_DC2LIM]		= RICOH568_IRQ(DCDC_INT, 1, 1, 1, 1),
	[RICOH568_IRQ_DC3LIM]		= RICOH568_IRQ(DCDC_INT, 1, 2, 2, 1),
	[RICOH568_IRQ_DC4LIM]		= RICOH568_IRQ(DCDC_INT, 1, 3, 3, 1),

	[RICOH568_IRQ_GPIO0]		= RICOH568_IRQ(GPIO_INT, 4, 0, 0, 6),
	[RICOH568_IRQ_GPIO1]		= RICOH568_IRQ(GPIO_INT, 4, 1, 1, 6),
	[RICOH568_IRQ_GPIO2]		= RICOH568_IRQ(GPIO_INT, 4, 2, 2, 6),
	[RICOH568_IRQ_GPIO3]		= RICOH568_IRQ(GPIO_INT, 4, 3, 3, 6),
};
static const inline struct ricoh568_irq_data * irq_to_ricoh568_irq(struct ricoh568 *ricoh568, int irq)
{
	struct irq_data *data = irq_get_irq_data(irq);
	return &ricoh568_irqs[data->hwirq];
}
static void ricoh568_irq_lock(struct irq_data *irq_data)
{
	struct ricoh568 *ricoh568 = irq_data_get_irq_chip_data(irq_data);

	mutex_lock(&ricoh568->irq_lock);
}

static void ricoh568_irq_unmask(struct irq_data *irq_data)
{
	struct ricoh568 *ricoh568 = irq_data_get_irq_chip_data(irq_data);
	const struct ricoh568_irq_data *data= irq_to_ricoh568_irq(ricoh568,irq_data->irq);
    	mutex_lock(&int_flag_mutex);

	ricoh568->group_irq_en[data->master_bit] |= (1 << data->grp_index);
	if (ricoh568->group_irq_en[data->master_bit])
		ricoh568->intc_inten_reg |= 1 << data->master_bit;

	if (data->master_bit == 6)	/* if Charger */
		ricoh568->irq_en_reg[data->mask_reg_index]
						&= ~(1 << data->int_en_bit);
	else
		ricoh568->irq_en_reg[data->mask_reg_index]
						|= 1 << data->int_en_bit;
   	 mutex_unlock(&int_flag_mutex);
}

static void ricoh568_irq_mask(struct irq_data *irq_data)
{
	struct ricoh568 *ricoh568 = irq_data_get_irq_chip_data(irq_data);
	const struct ricoh568_irq_data *data= irq_to_ricoh568_irq(ricoh568,irq_data->irq);
        mutex_lock(&int_flag_mutex);

	ricoh568->group_irq_en[data->master_bit] &= ~(1 << data->grp_index);
	if (!ricoh568->group_irq_en[data->master_bit])
		ricoh568->intc_inten_reg &= ~(1 << data->master_bit);

	if (data->master_bit == 6)	/* if Charger */
		ricoh568->irq_en_reg[data->mask_reg_index]
						|= 1 << data->int_en_bit;
	else
		ricoh568->irq_en_reg[data->mask_reg_index]
						&= ~(1 << data->int_en_bit);
        mutex_unlock(&int_flag_mutex);
}

static void ricoh568_irq_sync_unlock(struct irq_data *irq_data)
{
	struct ricoh568 *ricoh568 = irq_data_get_irq_chip_data(irq_data);
	int i;

	for (i = 0; i < ARRAY_SIZE(ricoh568->gpedge_reg); i++) {
		if (ricoh568->gpedge_reg[i] != ricoh568->gpedge_cache[i]) {
			if (!WARN_ON(ricoh568_write(ricoh568->dev, gpedge_add[i],ricoh568->gpedge_reg[i])))
				ricoh568->gpedge_cache[i] =ricoh568->gpedge_reg[i];
		}
	}

	for (i = 0; i < ARRAY_SIZE(ricoh568->irq_en_reg); i++) {
		if (ricoh568->irq_en_reg[i] != ricoh568->irq_en_cache[i]) {
			if (!WARN_ON(ricoh568_write(ricoh568->dev, irq_en_add[i],ricoh568->irq_en_reg[i])))
				ricoh568->irq_en_cache[i] =ricoh568->irq_en_reg[i];
		}
	}

	if (ricoh568->intc_inten_reg != ricoh568->intc_inten_cache) {
		if (!WARN_ON(ricoh568_write(ricoh568->dev,RICOH568_INTC_INTEN, ricoh568->intc_inten_reg)))
			ricoh568->intc_inten_cache = ricoh568->intc_inten_reg;
	}

	mutex_unlock(&ricoh568->irq_lock);
}

static int ricoh568_irq_set_type(struct irq_data *irq_data, unsigned int type)
{
	struct ricoh568 *ricoh568 = irq_data_get_irq_chip_data(irq_data);
	const struct ricoh568_irq_data *data= irq_to_ricoh568_irq(ricoh568,irq_data->irq);
	int val = 0;
	int gpedge_index;
	int gpedge_bit_pos;

	if (data->int_type & GPIO_INT) {
		gpedge_index = data->int_en_bit / 4;
		gpedge_bit_pos = data->int_en_bit % 4;

		if (type & IRQ_TYPE_EDGE_FALLING)
			val |= 0x2;

		if (type & IRQ_TYPE_EDGE_RISING)
			val |= 0x1;

		ricoh568->gpedge_reg[gpedge_index] &= ~(3 << gpedge_bit_pos);
		ricoh568->gpedge_reg[gpedge_index] |= (val << gpedge_bit_pos);
		ricoh568_irq_unmask(irq_data);
	}
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int ricoh568_irq_set_wake(struct irq_data *irq_data, unsigned int on)
{
	struct ricoh568 *ricoh568 = irq_data_get_irq_chip_data(irq_data);
	return irq_set_irq_wake(ricoh568->chip_irq, on);	//i2c->irq
}
#else
#define ricoh568_irq_set_wake NULL
#endif
u8 ricoh568_pwr_key_reg;
static irqreturn_t ricoh568_irq(int irq, void *data)
{
	struct ricoh568 *ricoh568 = data;
	u8 int_sts[MAX_INTERRUPT_MASKS];
	u8 master_int;
	int i;
	int ret;
	unsigned int rtc_int_sts = 0;
	int cur_irq = 0;

	ret = ricoh568_read(ricoh568->dev, RICOH568_INT_IR_SYS, &ricoh568_pwr_key_reg);

	/* Clear the status */
	for (i = 0; i < MAX_INTERRUPT_MASKS; i++)
		int_sts[i] = 0;

	ret = ricoh568_read(ricoh568->dev, RICOH568_INTC_INTMON,
						&master_int);
//	printk("PMU1: %s: master_int=0x%x\n", __func__, master_int);
	if (ret < 0) {
		dev_err(ricoh568->dev, "Error in reading reg 0x%02x "
			"error: %d\n", RICOH568_INTC_INTMON, ret);
		return IRQ_HANDLED;
	}

	for (i = 0; i < MAX_INTERRUPT_MASKS; ++i) {
		/* Even if INTC_INTMON register = 1, INT signal might not output
	  	 because INTC_INTMON register indicates only interrupt facter level.
	  	 So remove the following procedure */
		if (!(master_int & main_int_type[i]))
			continue;

		ret = ricoh568_read(ricoh568->dev,
				irq_mon_add[i], &int_sts[i]);
//		printk("PMU2: %s: int_sts[%d]=0x%x\n", __func__,i, int_sts[i]);
		if (ret < 0) {
			dev_err(ricoh568->dev, "Error in reading reg 0x%02x "
				"error: %d\n", irq_mon_add[i], ret);
			int_sts[i] = 0;
			continue;
		}
		if (!int_sts[i])
			continue;

		ret = ricoh568_write(ricoh568->dev,
			irq_clr_add[i], ~int_sts[i]);
		if (ret < 0)
			dev_err(ricoh568->dev, "Error in reading reg 0x%02x "
				"error: %d\n", irq_clr_add[i], ret);

	}

	/* Merge gpio interrupts  for rising and falling case*/
	int_sts[6] |= int_sts[7];

	/* Call interrupt handler if enabled */
        mutex_lock(&int_flag_mutex);
	for (i = 0; i <RICOH568_NR_IRQS; ++i) {
		const struct ricoh568_irq_data *data = &ricoh568_irqs[i];
		if ((int_sts[data->mask_reg_index] & (1 << data->int_en_bit)) &&(ricoh568->group_irq_en[data->master_bit] & (1 << data->grp_index))){
			cur_irq = irq_find_mapping(ricoh568->irq_domain, i);
			if (cur_irq)
				handle_nested_irq(cur_irq);
		}
	}
        mutex_unlock(&int_flag_mutex);

//	printk(KERN_INFO "PMU: %s: out\n", __func__);
	return IRQ_HANDLED;
}

static struct irq_chip ricoh568_irq_chip = {
	.name = "ricoh568",
	//.irq_mask = ricoh568_irq_mask,
	//.irq_unmask = ricoh568_irq_unmask,
	.irq_enable = ricoh568_irq_unmask,
	.irq_disable = ricoh568_irq_mask,
	.irq_bus_lock = ricoh568_irq_lock,
	.irq_bus_sync_unlock = ricoh568_irq_sync_unlock,
	.irq_set_type = ricoh568_irq_set_type,
	.irq_set_wake = ricoh568_irq_set_wake,
};

static int ricoh568_irq_domain_map(struct irq_domain *d, unsigned int irq,
					irq_hw_number_t hw)
{
	struct ricoh568 *ricoh568 = d->host_data;

	irq_set_chip_data(irq, ricoh568);
	irq_set_chip_and_handler(irq, &ricoh568_irq_chip, handle_edge_irq);
	irq_set_nested_thread(irq, 1);
#ifdef CONFIG_ARM
	set_irq_flags(irq, IRQF_VALID);
#else
	irq_set_noprobe(irq);
#endif
	return 0;
}

static struct irq_domain_ops ricoh568_irq_domain_ops = {
	.map = ricoh568_irq_domain_map,
};

int ricoh568_irq_init(struct ricoh568 *ricoh568, int irq,
				struct ricoh568_platform_data *pdata)
{
	int i, ret, val, irq_type, flags;
	u8 reg_data = 0;
	struct irq_domain *domain;
	printk("%s, line=%d, %d\n", __func__,__LINE__, irq);

	if (!irq) {
		dev_warn(ricoh568->dev, "No interrupt support, no core IRQ\n");
		return 0;
	}

	mutex_init(&ricoh568->irq_lock);

	/* Initialize all locals to 0 */
	for (i = 0; i < 2; i++) {
		ricoh568->irq_en_cache[i] = 0;
		ricoh568->irq_en_reg[i] = 0;
	}

	/* Initialize rtc */
	ricoh568->irq_en_cache[2] = 0x20;
	ricoh568->irq_en_reg[2] = 0x20;

	/* Initialize all locals to 0 */
	for (i = 3; i < 8; i++) {
		ricoh568->irq_en_cache[i] = 0;
		ricoh568->irq_en_reg[i] = 0;
	}

	// Charger Mask register must be set to 1 for masking Int output.
	for (i = 8; i < MAX_INTERRUPT_MASKS; i++) {
		ricoh568->irq_en_cache[i] = 0xff;
		ricoh568->irq_en_reg[i] = 0xff;
	}

	ricoh568->intc_inten_cache = 0;
	ricoh568->intc_inten_reg = 0;
	for (i = 0; i < MAX_GPEDGE_REG; i++) {
		ricoh568->gpedge_cache[i] = 0;
		ricoh568->gpedge_reg[i] = 0;
	}
	printk("%s,line=%d\n", __func__,__LINE__);

	/* Initailize all int register to 0 */
	for (i = 0; i < MAX_INTERRUPT_MASKS; i++)  {
		ret = ricoh568_write(ricoh568->dev,
				irq_en_add[i],
				ricoh568->irq_en_reg[i]);
		if (ret < 0)
			dev_err(ricoh568->dev, "Error in writing reg 0x%02x "
				"error: %d\n", irq_en_add[i], ret);
	}

	/* 568 Not use
	for (i = 0; i < MAX_GPEDGE_REG; i++)  {
		ret = ricoh568_write(ricoh568->dev,
				gpedge_add[i],
				ricoh568->gpedge_reg[i]);
		if (ret < 0)
			dev_err(ricoh568->dev, "Error in writing reg 0x%02x "
				"error: %d\n", gpedge_add[i], ret);
	}
	*/

	ret = ricoh568_write(ricoh568->dev, RICOH568_INTC_INTEN, 0x0);
	if (ret < 0)
		dev_err(ricoh568->dev, "Error in writing reg 0x%02x "
				"error: %d\n", RICOH568_INTC_INTEN, ret);

	/* Clear all interrupts in case they woke up active. */
	for (i = 0; i < MAX_INTERRUPT_MASKS; i++)  {
		ret = ricoh568_write(ricoh568->dev,
				irq_clr_add[i], 0);
		if (ret < 0)
			dev_err(ricoh568->dev, "Error in writing reg 0x%02x "
				"error: %d\n", irq_clr_add[i], ret);
	}

#if 0
	if (pdata->irq_gpio && !ricoh568->chip_irq) {
		// ricoh568->chip_irq = gpio_to_irq(pdata->irq_gpio);
		// printk("%s,line=%d: %d\n", __func__,__LINE__, &ricoh568->chip_irq);

		if (pdata->irq_gpio) {
			ret = gpio_request(pdata->irq_gpio, "ricoh568_pmic_irq");
			if (ret < 0) {
				dev_err(ricoh568->dev,
					"Failed to request gpio %d with ret:"
					"%d\n",	pdata->irq_gpio, ret);
				return IRQ_NONE;
			}
			gpio_direction_input(pdata->irq_gpio);
			val = gpio_get_value(pdata->irq_gpio);
			if (val){
				irq_type = IRQ_TYPE_LEVEL_LOW;
				flags = IRQF_TRIGGER_LOW;
			}
			else{
				irq_type = IRQ_TYPE_LEVEL_HIGH;
				flags = IRQF_TRIGGER_HIGH;
			}
			gpio_free(pdata->irq_gpio);
			pr_info("%s: ricoh568_pmic_irq=%x\n", __func__, val);
		}
	}
#endif
	domain = irq_domain_add_linear(NULL, RICOH568_NR_IRQS,
					&ricoh568_irq_domain_ops, ricoh568);
	if (!domain) {
		dev_err(ricoh568->dev, "could not create irq domain\n");
		return -ENODEV;
	}
	ricoh568->irq_domain = domain;

//	ret = devm_request_threaded_irq(ricoh568->dev,ricoh568->chip_irq, NULL, ricoh568_irq, IRQF_TRIGGER_FALLING | IRQF_ONESHOT , "ricoh568", ricoh568);
#if 0

if (gpio_is_valid(pdata->irq_gpio)) {
	ret = devm_gpio_request_one(&ricoh568->client->dev, pdata->irq_gpio,
				GPIOF_IN, "ricoh568 irq");
	if (ret) {
		dev_err(ricoh568->dev,
			"Failed to request GPIO %d, error %d\n",
			pdata->irq_gpio, ret);
		return ret;
	}
}
#endif
ret = devm_request_threaded_irq(ricoh568->dev, ricoh568->chip_irq, NULL,
				ricoh568_irq,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"ricoh568_mfd_int", ricoh568);
if (ret) {
	dev_err(ricoh568->dev, "Unable to request touchscreen IRQ.\n");
	return ret;
}

irq_set_irq_type(ricoh568->chip_irq, irq_type);
enable_irq_wake(ricoh568->chip_irq);

	return ret;
	//return 0;
}

int ricoh568_irq_exit(struct ricoh568 *ricoh568)
{
	return 0;
}
