/*
* driver/input/misc/ricoh568-pwrkey.c
*
* Power Key driver for RICOH RN5T568 power management chip.
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
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
*/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mfd/ricoh568.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/delay.h>

#include <linux/mfd/ricoh568.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>

#define RICOH568_ONKEY_TRIGGER_LEVEL	0
#define RICOH568_ONKEY_OFF_IRQ		0

struct ricoh568_pwrkey {
	struct device *dev;
	struct input_dev *pwr;
	#if RICOH568_ONKEY_TRIGGER_LEVEL
		struct timer_list timer;
	#endif
	struct workqueue_struct *workqueue;
	struct work_struct work;
	unsigned long delay;
	int key_irq;
	bool pressed_first;
	struct ricoh568_pwrkey_platform_data *pdata;
	spinlock_t lock;
};

struct ricoh568_pwrkey *g_pwrkey;

#if RICOH568_ONKEY_TRIGGER_LEVEL
void ricoh568_pwrkey_timer(unsigned long t)
{
	queue_work(g_pwrkey->workqueue, &g_pwrkey->work);
}
#endif
//extern void rk_send_wakeup_key(void);
extern u8 ricoh568_pwr_key_reg;
static void ricoh568_irq_work(struct work_struct *work)
{
	/* unsigned long flags; */
	uint8_t val;
	int i=0;

//	printk("PMU: %s: \n",__func__);
	//spin_lock_irqsave(&g_pwrkey->lock, flags);
	if((ricoh568_pwr_key_reg & 0x01) && ricoh568_pwrkey_wakeup){
		printk("PMU: %s: pwrkey_wakeup\n",__func__);
		ricoh568_pwrkey_wakeup = 0;
		input_event(g_pwrkey->pwr, EV_KEY, KEY_POWER, 1);
		input_event(g_pwrkey->pwr, EV_SYN, 0, 0);
		input_event(g_pwrkey->pwr, EV_KEY, KEY_POWER, 0);
		input_event(g_pwrkey->pwr, EV_SYN, 0, 0);
		do{
			ricoh568_read(g_pwrkey->dev->parent, RICOH568_INT_MON_SYS, &val);
       		val &= 0x01;
			i += 1;
			msleep(100);
		}while(val && (i < 15));
		return;
	}
	ricoh568_read(g_pwrkey->dev->parent, RICOH568_INT_MON_SYS, &val);
	dev_dbg(g_pwrkey->dev, "pwrkey is pressed?(0x%x): 0x%x\n",
						RICOH568_INT_MON_SYS, val);
//	printk(KERN_INFO "PMU: %s: val=0x%x\n", __func__, val);
	val &= 0x1;
	if(val){
		#if (RICOH568_ONKEY_TRIGGER_LEVEL)
		g_pwrkey->timer.expires = jiffies + g_pwrkey->delay;
		add_timer(&g_pwrkey->timer);
		#endif
		if (!g_pwrkey->pressed_first){
			g_pwrkey->pressed_first = true;
//			printk("PMU1: %s: Power Key!!!\n",__func__);
			//input_report_key(g_pwrkey->pwr, KEY_POWER, 1);
			//input_sync(g_pwrkey->pwr);
			input_event(g_pwrkey->pwr, EV_KEY, KEY_POWER, 1);
			input_event(g_pwrkey->pwr, EV_SYN, 0, 0);
		}
	} else {
		if (g_pwrkey->pressed_first) {
//			printk(KERN_INFO "PMU2: %s: Power Key!!!\n", __func__);
			/* input_report_key(g_pwrkey->pwr, KEY_POWER, 0); */
			/* input_sync(g_pwrkey->pwr); */
			input_event(g_pwrkey->pwr, EV_KEY, KEY_POWER, 0);
			input_event(g_pwrkey->pwr, EV_SYN, 0, 0);
		}
		g_pwrkey->pressed_first = false;
	}

	/* spin_unlock_irqrestore(&g_pwrkey->lock, flags); */
}

static irqreturn_t pwrkey_irq(int irq, void *_pwrkey)
{
//	printk(KERN_INFO "PMU: %s:\n", __func__);
//	rk_send_wakeup_key();
	#if (RICOH568_ONKEY_TRIGGER_LEVEL)
	g_pwrkey->timer.expires = jiffies + g_pwrkey->delay;
	add_timer(&g_pwrkey->timer);
	#else
	queue_work(g_pwrkey->workqueue, &g_pwrkey->work);
	#endif
	return IRQ_HANDLED;
}

#if RICOH568_ONKEY_OFF_IRQ
static irqreturn_t pwrkey_irq_off(int irq, void *_pwrkey)
{
	dev_warn(g_pwrkey->dev, "ONKEY is pressed long time!\n");
	return IRQ_HANDLED;
}
#endif

#ifdef CONFIG_OF
static struct ricoh568_pwrkey_platform_data *
ricoh568_pwrkey_dt_init(struct platform_device *pdev)
{
	struct device_node *nproot = pdev->dev.parent->of_node;
	struct device_node *np;
	struct ricoh568_pwrkey_platform_data *pdata;

	if (!nproot)
		return pdev->dev.platform_data;

	np = of_find_node_by_name(nproot, "pwrkey");
	if (!np) {
		dev_err(&pdev->dev, "failed to find pwrkey node\n");
		return NULL;
	}

	pdata = devm_kzalloc(&pdev->dev,
			sizeof(struct ricoh568_pwrkey_platform_data),
			GFP_KERNEL);

	of_property_read_u32(np, "ricoh,pwrkey-delay-ms", &pdata->delay_ms);
	of_node_put(np);

	return pdata;
}
#else
static struct ricoh568_pwrkey_platform_data *
ricoh568_pwrkey_dt_init(struct platform_device *pdev)
{
	return pdev->dev.platform_data;
}
#endif

static int ricoh568_pwrkey_probe(struct platform_device *pdev)
{
	struct input_dev *pwr;
	int key_irq;
	int err;
	struct ricoh568_pwrkey *pwrkey;
	struct ricoh568_pwrkey_platform_data *pdata;
	struct ricoh568 *ricoh568 = dev_get_drvdata(pdev->dev.parent);
	uint8_t val;

//	printk("PMU: %s: \n",__func__);

	pdata = ricoh568_pwrkey_dt_init(pdev);
	if (!pdata) {
		dev_err(&pdev->dev, "platform data isn't assigned to "
			"power key\n");
		return -EINVAL;
	}
	key_irq  = irq_create_mapping(ricoh568->irq_domain, RICOH568_IRQ_POWER_ON);
	printk(KERN_INFO "PMU1: %s: key_irq=%d\n", __func__, key_irq);
	pwrkey = kzalloc(sizeof(*pwrkey), GFP_KERNEL);
	if (!pwrkey)
		return -ENOMEM;

	pwrkey->dev = &pdev->dev;
	pwrkey->pdata = pdata;
	pwrkey->pressed_first = false;
	pwrkey->delay = HZ / 1000 * pdata->delay_ms;
	g_pwrkey = pwrkey;
	pwr = input_allocate_device();
	if (!pwr) {
		dev_dbg(&pdev->dev, "Can't allocate power button\n");
		err = -ENOMEM;
		goto free_pwrkey;
	}
	input_set_capability(pwr, EV_KEY, KEY_POWER);
	pwr->name = "ricoh568_pwrkey";
	pwr->phys = "ricoh568_pwrkey/input0";
	pwr->dev.parent = &pdev->dev;

	#if RICOH568_ONKEY_TRIGGER_LEVEL
	init_timer(&pwrkey->timer);
	pwrkey->timer.function = ricoh568_pwrkey_timer;
	#endif

	spin_lock_init(&pwrkey->lock);
	err = input_register_device(pwr);
	if (err) {
		dev_dbg(&pdev->dev, "Can't register power key: %d\n", err);
		goto free_input_dev;
	}
	pwrkey->key_irq = key_irq;
	pwrkey->pwr = pwr;
	platform_set_drvdata(pdev, pwrkey);

	/* Check if power-key is pressed at boot up */
	err = ricoh568_read(pwrkey->dev->parent, RICOH568_INT_MON_SYS, &val);
	if (err < 0) {
		dev_err(&pdev->dev, "Key-press status at boot failed rc=%d\n",
									 err);
		goto unreg_input_dev;
	}
	val &= 0x1;
	if (val) {
		input_report_key(pwrkey->pwr, KEY_POWER, 1);
//		printk(KERN_INFO "******KEY_POWER:1\n");
		input_sync(pwrkey->pwr);
		pwrkey->pressed_first = true;
	}

	#if !(RICOH568_ONKEY_TRIGGER_LEVEL)
		/* trigger both edge */
		ricoh568_set_bits(pwrkey->dev->parent, RICOH568_PWR_IRSEL, 0x1);
	#endif
	err = request_threaded_irq(key_irq, NULL, pwrkey_irq,IRQF_ONESHOT, "ricoh568_pwrkey", pwrkey);
	if (err < 0) {
		dev_err(&pdev->dev, "Can't get %d IRQ for pwrkey: %d\n",
								key_irq, err);
		goto unreg_input_dev;
	}
	/*
	#if RICOH568_ONKEY_OFF_IRQ
	err = request_threaded_irq( key_irq +RICOH568_ONKEY_OFF_IRQ, NULL,pwrkey_irq_off, IRQF_ONESHOT,
						"ricoh568_pwrkey_off", pwrkey);
	if (err < 0) {
		dev_err(&pdev->dev, "Can't get %d IRQ for ricoh568_pwrkey_off: %d\n",
			key_irq + RICOH568_ONKEY_OFF_IRQ, err);
		free_irq(key_irq, pwrkey);
		goto unreg_input_dev;
	}
	#endif
*/
	pwrkey->workqueue = create_singlethread_workqueue("ricoh568_pwrkey");
	INIT_WORK(&pwrkey->work, ricoh568_irq_work);

	/* Enable power key IRQ */
	/* trigger both edge */
	ricoh568_set_bits(pwrkey->dev->parent, RICOH568_PWR_IRSEL, 0x1);
	/* Enable system interrupt */
	ricoh568_set_bits(pwrkey->dev->parent, RICOH568_INTC_INTEN, 0x1);
	/* Enable power-on interrupt */
	ricoh568_set_bits(pwrkey->dev->parent, RICOH568_INT_EN_SYS, 0x1);
//	printk(KERN_INFO "PMU: %s is OK!\n", __func__);
	return 0;

unreg_input_dev:
	input_unregister_device(pwr);
	pwr = NULL;

free_input_dev:
	input_free_device(pwr);
	free_pwrkey:
	kfree(pwrkey);

	return err;
}

static int ricoh568_pwrkey_remove(struct platform_device *pdev)
{
	struct ricoh568_pwrkey *pwrkey = platform_get_drvdata(pdev);

	flush_workqueue(pwrkey->workqueue);
	destroy_workqueue(pwrkey->workqueue);
	free_irq(pwrkey->key_irq, pwrkey);
	input_unregister_device(pwrkey->pwr);
	kfree(pwrkey);

	return 0;
}

#ifdef CONFIG_PM
static int ricoh568_pwrkey_suspend(struct device *dev)
{
	struct ricoh568_pwrkey *info = dev_get_drvdata(dev);

//	printk(KERN_INFO "PMU: %s\n", __func__);

//	if (info->key_irq)
//		disable_irq(info->key_irq);
	cancel_work_sync(&info->work);
	flush_workqueue(info->workqueue);

	return 0;
}

static int ricoh568_pwrkey_resume(struct device *dev)
{
	struct ricoh568_pwrkey *info = dev_get_drvdata(dev);

//	printk(KERN_INFO "PMU: %s\n", __func__);
	queue_work(info->workqueue, &info->work);
//	if (info->key_irq)
//		enable_irq(info->key_irq);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id ricoh568_pwrkey_dt_match[] = {
	{ .compatible = "ricoh,ricoh568-pwrkey", },
	{},
};
MODULE_DEVICE_TABLE(of, ricoh568_pwrkey_dt_match);
#endif

static const struct dev_pm_ops ricoh568_pwrkey_pm_ops = {
	.suspend	= ricoh568_pwrkey_suspend,
	.resume		= ricoh568_pwrkey_resume,
};
#endif

static struct platform_driver ricoh568_pwrkey_driver = {
	.probe = ricoh568_pwrkey_probe,
	.remove = ricoh568_pwrkey_remove,
	.driver = {
		.name = "ricoh568-pwrkey",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ricoh568_pwrkey_dt_match),
#ifdef CONFIG_PM
		.pm	= &ricoh568_pwrkey_pm_ops,
#endif
	},
};

static int __init ricoh568_pwrkey_init(void)
{
	return platform_driver_register(&ricoh568_pwrkey_driver);
}
subsys_initcall_sync(ricoh568_pwrkey_init);

static void __exit ricoh568_pwrkey_exit(void)
{
	platform_driver_unregister(&ricoh568_pwrkey_driver);
}
module_exit(ricoh568_pwrkey_exit);


MODULE_ALIAS("platform:ricoh568-pwrkey");
MODULE_AUTHOR("leozhang <Leozhang@reds.ricoh.com>");
MODULE_DESCRIPTION("ricoh568 Power Key");
MODULE_LICENSE("GPL v2");
