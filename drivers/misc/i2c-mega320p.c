/*
 * drivers/misc/i2c-mega320p.c
 *
 * LCD control subsystem of MEGA320P MISC driver
 *
 * Copyright (C) 2021 NanJing No.55 Research Institute Technology Development CO.,LTD.
 *
 * Author: zhangbaolin <zhangbaolin@qq.com>
 *
 * Copyright (C) 2021 Intel Corp
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/pm_runtime.h>

#define I2C_MEGA320P_REG_SW_VER			0x00
#define I2C_MEGA320P_REG_LCD_ENABLED	0x01
#define I2C_MEGA320P_REG_LCD_PWM		0x02
#define I2C_MEGA320P_REG_LEFT_LAMP		0x03
#define I2C_MEGA320P_REG_RIGHT_LAMP		0x04

#define DRIVER_NAME "i2c_mega320p"

struct i2c_mega320p_data {
	struct device		*dev;
	struct i2c_client	*i2c;
	struct mutex mutex;
};

int i2c_mega320p_read(struct device *dev, u8 reg)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret;
	int retries = 3;
	while (retries-- > 0) {
		ret = i2c_smbus_read_byte_data(client, reg);
		if (ret >= 0)
			break;
	}
	return ret;
}

int i2c_mega320p_write(struct device *dev, u8 reg, u8 val)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret;
	int retries = 3;
	while (retries-- > 0) {
		ret = i2c_smbus_write_byte_data(client, reg, val);
		if (ret >= 0)
			break;
	}
	return ret;
}

int i2c_mega320p_get_sw_ver(struct device *dev)
{
	return i2c_mega320p_read(dev, I2C_MEGA320P_REG_SW_VER);
}

static ssize_t right_lamp_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int ret_val;
	int on_off;
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_mega320p_data *data = i2c_get_clientdata(client);
	sscanf(buf, "%d", &on_off);

	mutex_lock(&data->mutex);
	if (on_off == 0)
		ret_val = 0;
	else
		ret_val = 1;
	ret_val = i2c_smbus_write_byte_data(client, I2C_MEGA320P_REG_RIGHT_LAMP, ret_val);
	mutex_unlock(&data->mutex);
	return ret_val;
}

static ssize_t right_lamp_show(struct device *dev,
			struct device_attribute *attr,  char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int  val;

	val = i2c_smbus_read_byte_data(client, I2C_MEGA320P_REG_RIGHT_LAMP);
	if (val < 0)
		return val;
	return sprintf(buf, "%d\n", val);
}

static ssize_t left_lamp_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int ret_val;
	int on_off;
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_mega320p_data *data = i2c_get_clientdata(client);
	sscanf(buf, "%d", &on_off);

	mutex_lock(&data->mutex);
	if (on_off == 0)
		ret_val = 0;
	else
		ret_val = 1;
	ret_val = i2c_smbus_write_byte_data(client, I2C_MEGA320P_REG_LEFT_LAMP, ret_val);
	mutex_unlock(&data->mutex);
	return ret_val;
}

static ssize_t left_lamp_show(struct device *dev,
			struct device_attribute *attr,  char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int  val;

	val = i2c_smbus_read_byte_data(client, I2C_MEGA320P_REG_LEFT_LAMP);
	if (val < 0)
		return val;
	return sprintf(buf, "%d\n", val);
}

static ssize_t lcd_pwm_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int ret_val;
	int pwm;
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_mega320p_data *data = i2c_get_clientdata(client);
	sscanf(buf, "%d", &pwm);

	mutex_lock(&data->mutex);
	pwm &= 0xff;
	ret_val = i2c_smbus_write_byte_data(client, I2C_MEGA320P_REG_LCD_PWM, pwm);
	mutex_unlock(&data->mutex);
	return ret_val;
}

static ssize_t lcd_pwm_show(struct device *dev,
			struct device_attribute *attr,  char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int  val;

	val = i2c_smbus_read_byte_data(client, I2C_MEGA320P_REG_LCD_PWM);
	if (val < 0)
		return val;
	return sprintf(buf, "%d\n", val);
}

static ssize_t lcd_enabled_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int ret_val,on_off;
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_mega320p_data *data = i2c_get_clientdata(client);
	sscanf(buf, "%d", &on_off);
	mutex_lock(&data->mutex);
	if (on_off == 0)
		ret_val = 0;
	else
		ret_val = 1;
	ret_val = i2c_smbus_write_byte_data(client, I2C_MEGA320P_REG_LCD_ENABLED, ret_val);
	mutex_unlock(&data->mutex);
	return ret_val;
}

static ssize_t lcd_enabled_show(struct device *dev,
			struct device_attribute *attr,  char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int  val;

	val = i2c_smbus_read_byte_data(client, I2C_MEGA320P_REG_LCD_ENABLED);
	if (val < 0)
		return val;
	return sprintf(buf, "%d\n", val);
}

static ssize_t sw_ver_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_mega320p_data *data = i2c_get_clientdata(client);
	int ret_val;

	/* Protect against parallel reads */
	pm_runtime_get_sync(dev);
	mutex_lock(&data->mutex);

	ret_val = i2c_smbus_read_byte_data(client, I2C_MEGA320P_REG_SW_VER);
	if (ret_val < 0) {
		goto failed;
	}

	mutex_unlock(&data->mutex);
	pm_runtime_put_sync(dev);

	return sprintf(buf, "%d\n", ret_val);
failed:
	mutex_unlock(&data->mutex);
	pm_runtime_put_sync(dev);
	return ret_val;
}

static DEVICE_ATTR(sw_ver, S_IRUGO, sw_ver_show, NULL);
static DEVICE_ATTR(lcd_enabled, S_IRUGO | S_IWUSR,lcd_enabled_show, lcd_enabled_store);
static DEVICE_ATTR(lcd_pwm, S_IRUGO | S_IWUSR,lcd_pwm_show, lcd_pwm_store);
static DEVICE_ATTR(left_lamp, S_IRUGO | S_IWUSR,left_lamp_show, left_lamp_store);
static DEVICE_ATTR(right_lamp, S_IRUGO | S_IWUSR,right_lamp_show, right_lamp_store);
// static DEVICE_ATTR(lux0_input, S_IRUGO, als_lux0_input_data_show, NULL);

static struct attribute *mid_att_mpeg[] = {
	&dev_attr_sw_ver.attr,
	&dev_attr_lcd_enabled.attr,
	&dev_attr_lcd_pwm.attr,
	&dev_attr_left_lamp.attr,
	&dev_attr_right_lamp.attr,
	NULL
};

static const struct attribute_group m_als_gr = {
	.name = "i2c_mega320p",
	.attrs = mid_att_mpeg
};

static int i2c_mega320p_set_default_config(struct i2c_client *client)
{
	int ret_val;
	/* Write the command and then turn off */
	ret_val = i2c_smbus_write_byte_data(client, I2C_MEGA320P_REG_LCD_ENABLED, 0x01);
	if (ret_val < 0) {
		dev_err(&client->dev, "failed default turn off write\n");
		return ret_val;
	}
	/* Turn off the backlight and do not display the original logo */
	ret_val = i2c_smbus_write_byte_data(client, I2C_MEGA320P_REG_LCD_PWM, 0x00);
	if (ret_val < 0)
		dev_err(&client->dev, "failed default turn off the backlight write\n");
	dev_info(&client->dev, "default turn on and 0x00\n");
	return ret_val;
}

static int i2c_mega320p_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	int res;
	struct i2c_mega320p_data *data;
	int chip_ver;
	data = devm_kzalloc(&client->dev, sizeof(struct i2c_mega320p_data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(&client->dev, "Memory allocation failed\n");
		return -ENOMEM;
	}
	i2c_set_clientdata(client, data);
	data->dev = &client->dev;
	data->i2c = client;
	chip_ver = i2c_smbus_read_byte_data(client, I2C_MEGA320P_REG_SW_VER);
	if (chip_ver != 0x01) {
		dev_err(&client->dev, "I2C_MEGA320P chip not found\n");
		return -ENOMEM;
	}
	res = sysfs_create_group(&client->dev.kobj, &m_als_gr);
	if (res) {
		dev_err(&client->dev, "device create file failed\n");
		goto i2c_mega320p_error1;
	}
	i2c_mega320p_set_default_config(client);
	dev_info(&client->dev, "I2C_MEGA320P chip found\n");
	mutex_init(&data->mutex);

	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);

	return res;
i2c_mega320p_error1:
	kfree(data);
	return res;
}

static int i2c_mega320p_remove(struct i2c_client *client)
{
	struct i2c_mega320p_data *data = i2c_get_clientdata(client);

	pm_runtime_get_sync(&client->dev);

	
	sysfs_remove_group(&client->dev.kobj, &m_als_gr);

	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
	pm_runtime_put_noidle(&client->dev);

	kfree(data);
	return 0;
}

#ifdef CONFIG_PM

static int i2c_mega320p_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);


	return 0;
}

static int i2c_mega320p_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);


	return 0;
}

static UNIVERSAL_DEV_PM_OPS(i2c_mega320p_pm_ops, i2c_mega320p_suspend,
	i2c_mega320p_resume, NULL);

#define I2C_MEGA320P_PM_OPS (&i2c_mega320p_pm_ops)

#else	/* CONFIG_PM */
#define I2C_MEGA320P_PM_OPS NULL
#endif	/* CONFIG_PM */

static const struct i2c_device_id i2c_mega320p_id[] = {
	{ DRIVER_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, i2c_mega320p_id);

static struct i2c_driver i2c_mega320p_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.pm = I2C_MEGA320P_PM_OPS,
	},
	.probe = i2c_mega320p_probe,
	.remove = i2c_mega320p_remove,
	.id_table = i2c_mega320p_id,
};

module_i2c_driver(i2c_mega320p_driver);

MODULE_AUTHOR("Zhangbaolin <Zhangbaolin@qq.com");
MODULE_DESCRIPTION("i2c-mega320p Driver");
MODULE_LICENSE("GPL v1");
