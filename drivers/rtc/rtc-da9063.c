
/* rtc-da9063.c - Real time clock device driver for DA9063
 * Copyright (C) 2012  Dialog Semiconductor Ltd.
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mfd/da9063/registers.h>
#include <linux/mfd/da9063/core.h>

#define YEARS_TO_DA9063(year)		((year) - 100)
#define MONTHS_TO_DA9063(month)		((month) + 1)
#define YEARS_FROM_DA9063(year)		((year) + 100)
#define MONTHS_FROM_DA9063(month)	((month) - 1)

#define CLOCK_DATA_LEN	(DA9063_REG_COUNT_Y - DA9063_REG_COUNT_S + 1)
#define ALARM_DATA_LEN	(DA9063_REG_ALARM_Y - DA9063_REG_ALARM_MI + 1)
enum {
	DATA_SEC = 0,
	DATA_MIN,
	DATA_HOUR,
	DATA_DAY,
	DATA_MONTH,
	DATA_YEAR,
};

struct da9063_rtc {
	struct rtc_device	*rtc_dev;
	struct da9063		*hw;
	int			irq_alarm;
	int			irq_tick;

	/* Config flag */
	int			tick_wake;

	/* Used to expand alarm precision from minutes up to seconds
	   using hardware ticks */
	unsigned int		alarmSecs;
	unsigned int		alarmTicks;
};

static void da9063_data_to_tm(u8 *data, struct rtc_time *tm)
{
	tm->tm_sec = data[DATA_SEC] & DA9063_COUNT_SEC_MASK;
	tm->tm_min = data[DATA_MIN] & DA9063_COUNT_MIN_MASK;
	tm->tm_hour = data[DATA_HOUR] & DA9063_COUNT_HOUR_MASK;
	tm->tm_mday = data[DATA_DAY] & DA9063_COUNT_DAY_MASK;
	tm->tm_mon = MONTHS_FROM_DA9063(data[DATA_MONTH] &
					 DA9063_COUNT_MONTH_MASK);
	tm->tm_year = YEARS_FROM_DA9063(data[DATA_YEAR] &
					 DA9063_COUNT_YEAR_MASK);
}

static void da9063_tm_to_data(struct rtc_time *tm, u8 *data)
{
	data[DATA_SEC] &= ~DA9063_COUNT_SEC_MASK;
	data[DATA_SEC] |= tm->tm_sec & DA9063_COUNT_SEC_MASK;
	data[DATA_MIN] &= ~DA9063_COUNT_MIN_MASK;
	data[DATA_MIN] |= tm->tm_min & DA9063_COUNT_MIN_MASK;
	data[DATA_HOUR] &= ~DA9063_COUNT_HOUR_MASK;
	data[DATA_HOUR] |= tm->tm_hour & DA9063_COUNT_HOUR_MASK;
	data[DATA_DAY] &= ~DA9063_COUNT_DAY_MASK;
	data[DATA_DAY] |= tm->tm_mday & DA9063_COUNT_DAY_MASK;
	data[DATA_MONTH] &= ~DA9063_COUNT_MONTH_MASK;
	data[DATA_MONTH] |= MONTHS_TO_DA9063(tm->tm_mon) &
			    DA9063_COUNT_MONTH_MASK;
	data[DATA_YEAR] &= ~DA9063_COUNT_YEAR_MASK;
	data[DATA_YEAR] |= YEARS_TO_DA9063(tm->tm_year) &
			   DA9063_COUNT_YEAR_MASK;
}

#define DA9063_ALARM_DELAY	INT_MAX
static int da9063_rtc_test_delay(struct rtc_time *alarm, struct rtc_time *cur)
{
	unsigned long a_time, c_time;

	rtc_tm_to_time(alarm, &a_time);
	rtc_tm_to_time(cur, &c_time);

	/* Alarm time has already passed */
	if (a_time < c_time)
		return -1;

	/* If alarm is set for current minute, return ticks to count down.
	   If alarm is set for following minutes, return DA9063_ALARM_DELAY
	   to set alarm first.
	   But when it is less than 2 seconds for the former to become true,
	   return ticks, because alarm needs some time to synchronise. */
	if (a_time - c_time < alarm->tm_sec + 2)
		return a_time - c_time;
	else
		return DA9063_ALARM_DELAY;
}

static int da9063_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct da9063_rtc *rtc = dev_get_drvdata(dev);
	u8 data[CLOCK_DATA_LEN];
	int ret;

	ret = da9063_block_read(rtc->hw,
				DA9063_REG_COUNT_S, CLOCK_DATA_LEN, data);
	if (ret < 0)
		return ret;

	/* Check, if RTC logic is initialised */
	if (!(data[DATA_SEC] & DA9063_RTC_READ))
		return -EBUSY;

	da9063_data_to_tm(data, tm);

	return rtc_valid_tm(tm);
}

static int da9063_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct da9063_rtc *rtc = dev_get_drvdata(dev);
	u8 data[CLOCK_DATA_LEN] = { [0 ... (CLOCK_DATA_LEN - 1)] = 0 };
	int ret;

	da9063_tm_to_data(tm, data);

	ret = da9063_block_write(rtc->hw,
				 DA9063_REG_COUNT_S, CLOCK_DATA_LEN, data);

	return ret;
}

static int da9063_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct da9063_rtc *rtc = dev_get_drvdata(dev);
	u8 data[CLOCK_DATA_LEN];
	int ret;

	ret = da9063_block_read(rtc->hw, DA9063_REG_ALARM_MI, ALARM_DATA_LEN,
				&data[DATA_MIN]);
	if (ret < 0)
		return ret;

	da9063_data_to_tm(data, &alrm->time);
	alrm->time.tm_sec = rtc->alarmSecs;
	alrm->enabled = !!(data[DATA_YEAR] & DA9063_ALARM_ON);

	/* If there is no ticks left to count down and RTC event is
	   not processed yet, indicate pending */
	if (rtc->alarmTicks == 0) {
		ret = da9063_reg_read(rtc->hw, DA9063_REG_EVENT_A);
		if (ret < 0)
			return ret;
		if (ret & (DA9063_E_ALARM | DA9063_E_TICK))
			alrm->pending = 1;
	} else {
		alrm->pending = 0;
	}

	return 0;
}

static int da9063_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct da9063_rtc *rtc = dev_get_drvdata(dev);
	u8 data[CLOCK_DATA_LEN] = { [0 ... (CLOCK_DATA_LEN - 1)] = 0 };
	struct rtc_time cur_tm;
	int cmp_val;
	int ret;

	data[DATA_MIN] = DA9063_ALARM_STATUS_ALARM;
	data[DATA_MONTH] = DA9063_TICK_TYPE_SEC;
	if (rtc->tick_wake)
		data[DATA_MONTH] |= DA9063_TICK_WAKE;

	ret = da9063_rtc_read_time(dev, &cur_tm);
	if (ret < 0)
		return ret;

	if (alrm->enabled) {
		cmp_val = da9063_rtc_test_delay(&alrm->time, &cur_tm);
		if (cmp_val == DA9063_ALARM_DELAY) {
			/* Set alarm for longer delay */
			data[DATA_YEAR] |= DA9063_ALARM_ON;
		} else if (cmp_val > 0) {
			/* Count ticks for shorter delay */
			rtc->alarmTicks = cmp_val - 1;
			data[DATA_YEAR] |= DA9063_TICK_ON;
		} else if (cmp_val == 0) {
			/* Just about time - report event */
			rtc_update_irq(rtc->rtc_dev, 1, RTC_IRQF | RTC_AF);
		}
	}

	da9063_tm_to_data(&alrm->time, data);
	rtc->alarmSecs = alrm->time.tm_sec;

	return da9063_block_write(rtc->hw, DA9063_REG_ALARM_MI, ALARM_DATA_LEN,
				 &data[DATA_MIN]);
}

static int da9063_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct da9063_rtc *rtc = dev_get_drvdata(dev);
	struct rtc_wkalrm alrm;
	int ret;

	ret = da9063_reg_read(rtc->hw, DATA_YEAR);
	if (ret < 0)
		return ret;

	if (enabled) {
		/* Enable alarm, if it is not enabled already */
		if (!(ret & (DA9063_ALARM_ON | DA9063_TICK_ON))) {
			ret = da9063_rtc_read_alarm(dev, &alrm);
			if (ret < 0)
				return ret;

			alrm.enabled = 1;
			ret = da9063_rtc_set_alarm(dev, &alrm);
		}
	} else {
		ret = da9063_reg_clear_bits(rtc->hw, DA9063_REG_ALARM_Y,
					    DA9063_ALARM_ON);
	}

	return ret;
}

/* On alarm interrupt, start to count ticks to enable seconds precision
   (if alarm seconds != 0). */
static irqreturn_t da9063_alarm_event(int irq, void *data)
{
	struct da9063_rtc *rtc = data;

	if (rtc->alarmSecs) {
		rtc->alarmTicks = rtc->alarmSecs - 1;
		da9063_reg_update(rtc->hw, DA9063_REG_ALARM_Y,
				  DA9063_ALARM_ON | DA9063_TICK_ON,
				  DA9063_TICK_ON);
	} else {
		da9063_reg_clear_bits(rtc->hw, DA9063_REG_ALARM_Y,
				      DA9063_ALARM_ON);
		rtc_update_irq(rtc->rtc_dev, 1, RTC_IRQF | RTC_AF);
	}

	return IRQ_HANDLED;
}

/* On tick interrupt, count down seconds left to timeout */
static irqreturn_t da9063_tick_event(int irq, void *data)
{
	struct da9063_rtc *rtc = data;

	if (rtc->alarmTicks-- == 0) {
		da9063_reg_clear_bits(rtc->hw,
				      DA9063_REG_ALARM_Y, DA9063_TICK_ON);
		rtc_update_irq(rtc->rtc_dev, 1, RTC_IRQF | RTC_UF);
	}

	return IRQ_HANDLED;
}

static const struct rtc_class_ops da9063_rtc_ops = {
	.read_time = da9063_rtc_read_time,
	.set_time = da9063_rtc_set_time,
	.read_alarm = da9063_rtc_read_alarm,
	.set_alarm = da9063_rtc_set_alarm,
	.alarm_irq_enable = da9063_rtc_alarm_irq_enable,
};

static __devinit int da9063_rtc_probe(struct platform_device *pdev)
{
	struct da9063 *da9063 = dev_get_drvdata(pdev->dev.parent);
	struct da9063_rtc *rtc;
	int ret;
	int alarm_mo;

	/* Enable RTC hardware */
	ret = da9063_reg_set_bits(da9063, DA9063_REG_CONTROL_E, DA9063_RTC_EN);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to enable RTC.\n");
		return ret;
	}

	ret = da9063_reg_set_bits(da9063, DA9063_REG_EN_32K, DA9063_CRYSTAL);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to run 32 KHz OSC.\n");
		return ret;
	}

	ret = da9063_reg_read(da9063, DA9063_REG_ALARM_MO);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to read RTC register.\n");
		return ret;
	}
	alarm_mo = ret;

	/* Make sure that ticks are disabled. */
	ret = da9063_reg_clear_bits(da9063, DA9063_REG_ALARM_Y,
				    DA9063_TICK_ON);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to access RTC alarm register.\n");
		return ret;
	}

	/* Register RTC device */
	rtc = devm_kzalloc(&pdev->dev, sizeof *rtc, GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;

	platform_set_drvdata(pdev, rtc);

	rtc->hw = da9063;
	rtc->rtc_dev = rtc_device_register(DA9063_DRVNAME_RTC, &pdev->dev,
					   &da9063_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc->rtc_dev)) {
		dev_err(&pdev->dev, "Failed to register RTC device: %ld\n",
			PTR_ERR(rtc->rtc_dev));
		return PTR_ERR(rtc->rtc_dev);
	}

	if (alarm_mo & DA9063_TICK_WAKE)
		rtc->tick_wake = 1;

	/* Register interrupts. Complain on errors but let device
	   to be registered at least for date/time. */
	rtc->irq_alarm = platform_get_irq_byname(pdev, "ALARM");
	ret = request_threaded_irq(rtc->irq_alarm, NULL, da9063_alarm_event,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT, "ALARM", rtc);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request ALARM IRQ.\n");
		rtc->irq_alarm = -ENXIO;
		return 0;
	}

	rtc->irq_tick = platform_get_irq_byname(pdev, "TICK");
	ret = request_threaded_irq(rtc->irq_tick, NULL, da9063_tick_event,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT, "TICK", rtc);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request TICK IRQ.\n");
		rtc->irq_tick = -ENXIO;
	}

	return 0;
}

static int __devexit da9063_rtc_remove(struct platform_device *pdev)
{
	struct da9063_rtc *rtc = platform_get_drvdata(pdev);

	if (rtc->irq_alarm >= 0)
		free_irq(rtc->irq_alarm, rtc);

	if (rtc->irq_tick >= 0)
		free_irq(rtc->irq_tick, rtc);

	rtc_device_unregister(rtc->rtc_dev);
	return 0;
}

static struct platform_driver da9063_rtc_driver = {
	.probe		= da9063_rtc_probe,
	.remove		= __devexit_p(da9063_rtc_remove),
	.driver		= {
		.name	= DA9063_DRVNAME_RTC,
		.owner	= THIS_MODULE,
	},
};

static int __init da9063_rtc_init(void)
{
	return platform_driver_register(&da9063_rtc_driver);
}
module_init(da9063_rtc_init);

static void __exit da9063_rtc_exit(void)
{
	platform_driver_unregister(&da9063_rtc_driver);
}
module_exit(da9063_rtc_exit);

/* Module information */
MODULE_AUTHOR("S Twiss <stwiss.opensource@diasemi.com>");
MODULE_DESCRIPTION("Real time clock device driver for Dialog DA9063");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DA9063_DRVNAME_RTC);
