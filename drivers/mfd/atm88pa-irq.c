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
#include <linux/mfd/atm88pa.h>
#include <linux/mfd/atm88pa-private.h>

static int g_poll_delay = 50;
module_param_named(poll_delay, g_poll_delay, int, S_IRUSR | S_IWUSR);

static int g_use_irq = 0;
module_param_named(use_irq, g_use_irq, int, S_IRUSR | S_IWUSR);

static irqreturn_t atm88pa_irq_handler(int irq, void *data)
{
	struct atm88pa *atm = (struct atm88pa *)data;
	int ret;

	ret = atm88pa_read(atm, ATM88PA_REG_INT_CTRL);
	if (ret >= 0) {
		if (ret & 0x10)
			atm88pa_keypad_update(atm);
		if (ret & 0x20)
			atm88pa_update_status(atm);
	}

	return IRQ_HANDLED;
}

static enum hrtimer_restart atm88pa_timer_handler(struct hrtimer *timer)
{
	struct atm88pa *atm = container_of(timer, struct atm88pa, timer);
	/* Execute in workqueue */
	queue_work(atm->timer_wq, &atm->timer_work);

	/* Set next time */
	hrtimer_forward_now(&atm->timer, ms_to_ktime(g_poll_delay));

	/* Restart the timer */
	return HRTIMER_RESTART;
}

static void atm88pa_timer_work_handler(struct work_struct *work)
{
	struct atm88pa *atm = container_of(work, struct atm88pa, timer_work);

	atm88pa_keypad_update(atm);
	atm88pa_update_status(atm);
}

int atm88pa_irq_init(struct atm88pa *atm)
{
	int ret;

	if (!atm->i2c)
		return -EINVAL;

	/* Flush current status */
	atm88pa_update_status(atm);

	/* Flush current keys */
	atm88pa_keypad_update(atm);

	/* Enable status and keys interrupts */
	atm88pa_write(atm, ATM88PA_REG_INT_CTRL, 0x03);

	if (g_use_irq) {
		ret = request_threaded_irq(atm->i2c->irq,
					   NULL, atm88pa_irq_handler,
					   IRQF_ONESHOT, "atm88pa-int", atm);
		if (ret) {
			dev_err(atm->dev, "request irq: %d failed, err: %d\n",
				atm->i2c->irq, ret);
			return ret;
		}

		dev_info(atm->dev, "irq %d installed.\n", atm->i2c->irq);
	}
	else {
		/* Setup workqueue */
		atm->timer_wq = create_singlethread_workqueue("atm88pa-timer");
		if (atm->timer_wq == NULL) {
			dev_err(atm->dev, "failed to create timer workqueue!");
			return -ENOMEM;
		}
		INIT_WORK(&atm->timer_work, atm88pa_timer_work_handler);

		/* Setup timer */
		hrtimer_init(&atm->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		atm->timer.function = atm88pa_timer_handler;
		hrtimer_start(&atm->timer, ms_to_ktime(g_poll_delay), HRTIMER_MODE_REL);

		dev_info(atm->dev, "polled keys and status.");
	}

	return 0;
}

int atm88pa_irq_exit(struct atm88pa *atm)
{
	if (atm->timer_wq) {
		destroy_workqueue(atm->timer_wq);
		atm->timer_wq = NULL;
	}

	if (g_use_irq)
		free_irq(atm->i2c->irq, atm);
	else
		hrtimer_cancel(&atm->timer);

	return 0;
}

int atm88pa_irq_resume(struct atm88pa *atm)
{
	if (g_use_irq)
		enable_irq(atm->i2c->irq);

	return 0;
}

int atm88pa_irq_suspend(struct atm88pa *atm)
{
	if (g_use_irq)
		disable_irq(atm->i2c->irq);

	return 0;
}
