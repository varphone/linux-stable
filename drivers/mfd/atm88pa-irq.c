/*
 * drivers/mfd/atm88pa-irq.c
 *
 * IRQ handling for ATMEGA88PA MFD driver
 *
 * Copyright (C) 2017 NanJing No.55 Research Institute Technology Development CO.,LTD.
 *
 * Author: Varphone Wong <varphone@qq.com>
 *
 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mfd/atm88pa-private.h>

static irqreturn_t atm88pa_irq_handler(int irq, void *data)
{
	struct atm88pa *atm = (struct atm88pa *)data;

	atm88pa_keypad_update(atm);

	return IRQ_HANDLED;
}

int atm88pa_irq_init(struct atm88pa *atm)
{
	int ret;

	if (!atm->i2c)
		return -EINVAL;

	ret = request_threaded_irq(atm->i2c->irq,
				   NULL, atm88pa_irq_handler,
				   IRQF_ONESHOT, "atm88pa-int", atm);
	if (ret) {
		dev_err(atm->dev, "request irq: %d failed, err: %d\n",
			atm->i2c->irq, ret);
		return ret;
	}
	
	dev_info(atm->dev, "irq %d installed.\n", atm->i2c->irq);

	return 0;
}

int atm88pa_irq_exit(struct atm88pa *atm)
{
	free_irq(atm->i2c->irq, atm);

	return 0;
}

int atm88pa_irq_resume(struct atm88pa *atm)
{
#if 0
	enable_irq(atm->i2c->irq);
#endif
	return 0;
}

int atm88pa_irq_suspend(struct atm88pa *atm)
{
#if 0
	disable_irq(atm->i2c->irq);
#endif
	return 0;
}
