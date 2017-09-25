/*
 * apds9306.c - IIO driver for Avago APDS9306 ambient light sensor
 *
 * Copyright 2013 Oleksandr Kravchenko <o.v.kravchenko@globallogic.com>
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>
#include <linux/delay.h>

#define APDS9306_DRV_NAME "apds9306"
#define APDS9306_IRQ_NAME "apds9306_event" 


/* Register set */
#define APDS9306_MAIN_CTRL			0x00 	/* Control of basic functions */
#define APDS9306_ALS_MEAS_RATE  	0x04 	//ALS measurement rate and resolution in Active mode
#define APDS9306_ALS_GAIN			0x05 	//ALS analog gain range
#define APDS9306_Part_ID			0x06	//Part number ID and revision ID
#define APDS9306_MAIN_STATUS		0x07	//Power-on status, interrupt status, data status
#define APDS9306_CLEAR_DATA_0		0x0A	//Clear ADC measurement data - LSB
#define APDS9306_CLEAR_DATA_1		0x0B	//Clear ADC measurement data
#define APDS9306_CLEAR_DATA_2		0x0C	//Clear ADC measurement data - MSB
#define APDS9306_ALS_DATA_0			0x0D	//ALS ADC measurement data - LSB
#define APDS9306_ALS_DATA_1			0x0E	//ALS ADC measurement data
#define APDS9306_ALS_DATA_2			0x0F	//ALS ADC measurement data - MSB
#define APDS9306_INT_CFG			0x19	//Interrupt configuration
#define APDS9306_INT_PERSISTENCE	0x1A	//Interrupt persist setting
#define APDS9306_ALS_THRES_UP_0		0x21	//ALS interrupt upper threshold, LSB
#define APDS9306_ALS_THRES_UP_1		0x22	//ALS interrupt upper threshold
#define APDS9306_ALS_THRES_UP_2		0x23	//ALS interrupt upper threshold, MSB
#define APDS9306_ALS_THRES_LOW_0	0x24	//ALS interrupt lower threshold, LSB
#define APDS9306_ALS_THRES_LOW_1	0x25	//ALS interrupt lower threshold
#define APDS9306_ALS_THRES_LOW_2	0x26	//ALS interrupt lower threshold, MSB
#define APDS9306_ALS_THRES_VAR		0x27	//ALS interrupt variance threshold

#define APDS9306_POWER_ON_STATUS	0x20	
/* Power on/off value for APDS9306_MAIN_CTRL register */
#define APDS9306_POWER_ON	0x02
#define APDS9306_POWER_OFF	0x10
/* Interrupts */

#define APDS9306_INTR_SEL		0x10
#define APDS9306_INTR_MODE		0x08
#define APDS9306_INTR_ENABLE	0x04



#define APDS9306_THRESH_MAX	0xfffff /* Max threshold value */

static const int clear_als_it_value[] = {0,0,0,0,0,0};

extern struct iio_dev *devm_iio_device_alloc(struct device *dev, int sizeof_priv);
extern void iio_device_unregister(struct iio_dev *indio_dev);
extern int iio_device_register(struct iio_dev *indio_dev);

struct apds9306_data {								
	struct i2c_client *client;
	struct mutex mutex;
	int power_state;
	int thresh_low;
	int thresh_hi;
	int intr_en;
};


static int apds9306_get_adc_val(struct apds9306_data *data, int adc_number)		
{
	int ret,als_data=0;
	int flags,i;

	if (!data->power_state)
		return -EBUSY;

	/* Select ALS_DATA_X or CLEAR_DATA_X data register */
	flags = adc_number ? APDS9306_ALS_DATA_2 : APDS9306_CLEAR_DATA_2;	
	for(i = 0;i <3;i++)
	{	ret = i2c_smbus_read_byte_data(data->client, (flags-i));	
		if (ret < 0)
			goto err;
				
		als_data=als_data<<8;
		als_data|=ret;		
	}
	return als_data; 
err:	
	if(adc_number)		
		dev_err(&data->client->dev,
			"failed to read ALS_DATA_%d value\n", (2-i));
	else 
		dev_err(&data->client->dev,
			"failed to read CLEAR_DATA_%d value\n", (2-i));

	return ret;
}

static int apds9306_set_thresh_low(struct apds9306_data *data, int value)			
{
	int ret,i;
	int regval;

	if (!data->power_state)
		return -EBUSY;

	if (value > APDS9306_THRESH_MAX)
		return -EINVAL;
	for(i=0;i<3;i++)
	{	regval = value >>(i*8);
		regval &= 0xff;
			
		ret = i2c_smbus_write_word_data(data->client, APDS9306_ALS_THRES_LOW_0+i , regval);
		if (ret) {
			dev_err(&data->client->dev, "failed to set als_thresh_low_%d \n",i);
			return ret;
		}
	}
	data->thresh_low = value;

	return 0;
}

static int apds9306_set_thresh_hi(struct apds9306_data *data, int value)		
{
	int ret,i;
	int regval;
	if (!data->power_state)
		return -EBUSY;

	if (value > APDS9306_THRESH_MAX)
		return -EINVAL;
	for(i=0;i<3;i++)
	{	regval	= value >>(i*8);
		regval &= 0xff;
			
		ret = i2c_smbus_write_byte_data(data->client, APDS9306_ALS_THRES_UP_0+i, regval);
		if (ret) {
			dev_err(&data->client->dev, "failed to set als_thresh_up_%d \n",i);
			return ret;
		}
	}
	data->thresh_hi = value;

	return 0;
}

static int apds9306_set_intr_state(struct apds9306_data *data, int state)		
{
	int ret;
	u8 cmd;

	if (!data->power_state)
		return -EBUSY;

	ret = i2c_smbus_read_byte_data(data->client,APDS9306_INT_CFG);
	if (ret < 0) {
		dev_err(&data->client->dev,
			"failed to read als_int_cfg %d \n",ret);
		return ret;
	}	
	
	cmd = state ? APDS9306_INTR_ENABLE : 0x00;
	cmd |= ret;
	ret = i2c_smbus_write_byte_data(data->client,
			APDS9306_INT_CFG , cmd);
			
	if (ret) {
		dev_err(&data->client->dev,
			"failed to set interrupt state %d\n", state);
		return ret;
	}
	data->intr_en = state;

	return 0;
}

static int apds9306_set_power_state(struct apds9306_data *data, int state)			
{
	int ret;
	u8 cmd;

	cmd = state ? APDS9306_POWER_ON : APDS9306_POWER_OFF;
	ret = i2c_smbus_write_byte_data(data->client,APDS9306_MAIN_CTRL, cmd);
	if (ret) {
		dev_err(&data->client->dev,
			"failed to set power state %d , ret = 0x%x\n", state,ret);
		return ret;
	}
	data->power_state = state;

	return 0;
}

static void apds9306_clear_intr(struct apds9306_data *data)		
{
	int ret;

	ret = i2c_smbus_read_byte_data(data->client, APDS9306_MAIN_STATUS);
	if (ret < 0)
		dev_err(&data->client->dev, "failed to clear interrupt\n");
}

static int apds9306_chip_init(struct apds9306_data *data)		
{
	int ret;
	
	/* Need to set power off to ensure that the chip is off */
	ret = apds9306_set_power_state(data, 0);
	if (ret < 0)
		goto err;
	mdelay(1);
	/*
	 * Probe the chip. To do so we try to power up the device and then to
	 * read back the 0x02 code
	 */
	ret = apds9306_set_power_state(data, 1);
	if (ret < 0)
		goto err;
	ret = i2c_smbus_read_byte_data(data->client,APDS9306_MAIN_CTRL);
	if ((ret & APDS9306_POWER_ON)!= APDS9306_POWER_ON) {
		ret = -ENODEV;
		goto err;
	}
	/*
	 * Disable interrupt to ensure thai it is doesn't enable
	 * i.e. after device soft reset
	 */
	ret = apds9306_set_intr_state(data, 0);
	if (ret < 0)
		goto err;

	return 0;

err:
	dev_err(&data->client->dev, "failed to init the chip\n");
	return ret;
}

static int apds9306_read_raw(struct iio_dev *indio_dev,						
		struct iio_chan_spec const *chan, int *val, int *val2,
		long mask)
{
	int ret = -EINVAL;
	struct apds9306_data *data = iio_priv(indio_dev);

	mutex_lock(&data->mutex);
	switch (chan->type) {
	case IIO_LIGHT:
		ret = apds9306_get_adc_val(data,1);
		if (ret < 0) {
			break;
		}
		*val = ret;
		ret = IIO_VAL_INT;
		break;
	case IIO_INTENSITY:
		ret = apds9306_get_adc_val(data,0);
		if (ret < 0)
			break;
		*val = ret;
		ret = IIO_VAL_INT;
		break;
	default:
		break;
	}
	mutex_unlock(&data->mutex);

	return ret;
}

static int apds9306_read_thresh(struct iio_dev *indio_dev,						
		const struct iio_chan_spec *chan, enum iio_event_type type,
		enum iio_event_direction dir, enum iio_event_info info,
		int *val, int *val2)
{
	struct apds9306_data *data = iio_priv(indio_dev);

	switch (dir) {
	case IIO_EV_DIR_RISING:
		*val = data->thresh_hi;
		break;
	case IIO_EV_DIR_FALLING:
		*val = data->thresh_low;
		break;
	default:
		return -EINVAL;
	}

	return IIO_VAL_INT;
}

static int apds9306_write_thresh(struct iio_dev *indio_dev,						
		const struct iio_chan_spec *chan, enum iio_event_type type,
		enum iio_event_direction dir, enum iio_event_info info, int val,
		int val2)
{
	struct apds9306_data *data = iio_priv(indio_dev);
	int ret;

	mutex_lock(&data->mutex);
	if (dir == IIO_EV_DIR_RISING)
		ret = apds9306_set_thresh_hi(data, val);
	else
		ret = apds9306_set_thresh_low(data, val);
	mutex_unlock(&data->mutex);

	return ret;
}

static int apds9306_read_interrupt_config(struct iio_dev *indio_dev,			
		const struct iio_chan_spec *chan,
		enum iio_event_type type,
		enum iio_event_direction dir)
{
	struct apds9306_data *data = iio_priv(indio_dev);

	return data->intr_en;
}

static int apds9306_write_interrupt_config(struct iio_dev *indio_dev,			
		const struct iio_chan_spec *chan, enum iio_event_type type,
		enum iio_event_direction dir, int state)
{
	struct apds9306_data *data = iio_priv(indio_dev);
	int ret;

	mutex_lock(&data->mutex);
	ret = apds9306_set_intr_state(data, state);
	mutex_unlock(&data->mutex);

	return ret;
}

static ssize_t apds9306_get_it_available(struct device *dev,
	struct device_attribute *attr, char *buf)
{	int i, n, len;
	n = ARRAY_SIZE(clear_als_it_value);
	for(i = 0,len = 0; i < n; i++)
		len +=sprintf(buf + len,"0x%d ",clear_als_it_value[i]);
	return len + sprintf(buf + len, "\n");
}

static IIO_DEVICE_ATTR(in_illuminance_integration_time_available,
	S_IRUGO, apds9306_get_it_available, NULL, 0);

static struct attribute *apds9306_attributes[] = {
	&iio_dev_attr_in_illuminance_integration_time_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group apds9306_attribute_group = {
	.attrs		= apds9306_attributes
};

static const struct iio_info apds9306_info_no_irq = {			
	.driver_module	= THIS_MODULE,
	.read_raw	= apds9306_read_raw,
	.attrs		= &apds9306_attribute_group,
};

static const struct iio_info apds9306_info = {					
	.driver_module		= THIS_MODULE,
	.read_raw		= apds9306_read_raw,
	.read_event_value	= apds9306_read_thresh,
	.write_event_value	= apds9306_write_thresh,
	.attrs			= &apds9306_attribute_group,
	.read_event_config	= apds9306_read_interrupt_config,
	.write_event_config	= apds9306_write_interrupt_config,
};

static const struct iio_event_spec apds9306_event_spec[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
			BIT(IIO_EV_INFO_ENABLE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
			BIT(IIO_EV_INFO_ENABLE),
	},
};

static const struct iio_chan_spec apds9306_channels[] = {
	{
		.type = IIO_LIGHT,
		.channel = 0,
		.indexed = true,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
	}, {
		.type = IIO_INTENSITY,
		.channel = 0,
		.channel2 = IIO_MOD_LIGHT_BOTH,
		.indexed = true,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.event_spec = apds9306_event_spec,
		.num_event_specs = ARRAY_SIZE(apds9306_event_spec),
	}, {
		.type = IIO_INTENSITY,
		.channel = 1,
		.channel2 = IIO_MOD_LIGHT_IR,
		.indexed = true,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
};

static irqreturn_t apds9306_interrupt_handler(int irq, void *private)		
{
	struct iio_dev *dev_info = private;
	struct apds9306_data *data = iio_priv(dev_info);

	iio_push_event(dev_info,
		       IIO_UNMOD_EVENT_CODE(IIO_INTENSITY, 0,
					    IIO_EV_TYPE_THRESH,
					    IIO_EV_DIR_EITHER),
		       iio_get_time_ns());

	apds9306_clear_intr(data);

	return IRQ_HANDLED;
}

static int apds9306_probe(struct i2c_client *client,			
		const struct i2c_device_id *id)
{
	struct apds9306_data *data;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;
	
	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;
	
	ret = apds9306_chip_init(data);
	if (ret < 0)
		goto err;

	mutex_init(&data->mutex);

	indio_dev->dev.parent = &client->dev;
	indio_dev->channels = apds9306_channels;
	indio_dev->num_channels = ARRAY_SIZE(apds9306_channels);
	indio_dev->name = APDS9306_DRV_NAME;
	indio_dev->modes = INDIO_DIRECT_MODE;

	if (client->irq)
		indio_dev->info = &apds9306_info;
	else
		indio_dev->info = &apds9306_info_no_irq;

	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq,
				NULL, apds9306_interrupt_handler,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				APDS9306_IRQ_NAME, indio_dev);
		if (ret) {
			dev_err(&client->dev, "irq request error %d\n", -ret);
			goto err;
		}		
		pr_err("apds9306_irq=0x%x \n",ret);
	}

	ret = iio_device_register(indio_dev);
	if (ret < 0)
		goto err;
	return 0;

err:
	/* Ensure that power off in case of error */
	apds9306_set_power_state(data, 0);	
	pr_err("Out  apds9306_probe \n");
	return ret;
}

static int apds9306_remove(struct i2c_client *client)			
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct apds9306_data *data = iio_priv(indio_dev);
	
	iio_device_unregister(indio_dev);

	/* Ensure that power off and interrupts are disabled */
	apds9306_set_intr_state(data, 0);
	apds9306_set_power_state(data, 0);
	
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int apds9306_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct apds9306_data *data = iio_priv(indio_dev);
	int ret;

	mutex_lock(&data->mutex);
	ret = apds9306_set_power_state(data, 0);
	mutex_unlock(&data->mutex);

	return ret;
}

static int apds9306_resume(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct apds9306_data *data = iio_priv(indio_dev);
	int ret;

	mutex_lock(&data->mutex);
	ret = apds9306_set_power_state(data, 1);
	mutex_unlock(&data->mutex);

	return ret;
}

static SIMPLE_DEV_PM_OPS(apds9306_pm_ops, apds9306_suspend, apds9306_resume);
#define APDS9306_PM_OPS (&apds9306_pm_ops)
#else
#define APDS9306_PM_OPS NULL
#endif

static struct i2c_device_id apds9306_id[] = {
	{ APDS9306_DRV_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, apds9306_id);


#ifdef CONFIG_APDS9306
static const struct of_device_id apds9603[] = {
	{ .compatible = "tdc,apds9306", },
	{ }
};
MODULE_DEVICE_TABLE(of, mty065x_of_match);
#endif

static struct i2c_driver apds9306_driver = {
	.driver = {
		.name	= APDS9306_DRV_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_APDS9306
		.of_match_table	= of_match_ptr(apds9603),
#endif
		.pm	= APDS9306_PM_OPS,
	},
	.probe		= apds9306_probe,
	.remove		= apds9306_remove,
	.id_table	= apds9306_id,
};

module_i2c_driver(apds9306_driver);

MODULE_AUTHOR("Kravchenko Oleksandr <o.v.kravchenko@globallogic.com>");
MODULE_AUTHOR("GlobalLogic inc.");
MODULE_DESCRIPTION("APDS9306 ambient light photo sensor driver");
MODULE_LICENSE("GPL");
