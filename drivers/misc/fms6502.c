/*
 * drivers/misc/fms6502.c
 *
 * 8-input,6-output video switch matrix of FMS6502 MISC driver
 *
 * Copyright (C) 2021 NanJing No.55 Research Institute Technology Development CO.,LTD.
 *
 * Author: zhangbaolin <zhangbaolin@qq.com>
 *
 * Copyright (C) 2021 Intel Corp
 *
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>


#define FMS6502_REG_OUTPUT_1_2		0x00
#define FMS6502_REG_OUTPUT_3_4		0x01
#define FMS6502_REG_OUTPUT_5_6		0x02
#define FMS6502_REG_CLAMP		    0x03
#define FMS6502_REG_GAIN    		0x04

static const struct i2c_device_id fms6502_id[] = {
	{ "fms6502", 0 },
	{ }
};

struct fms6502_data {
	struct device		*dev;
	struct i2c_client	*i2c;
};

struct i2c_client *fms6502_client = NULL;

void fms6502_write_data(u8 cmd, u8 val)
{
	i2c_smbus_write_byte_data(fms6502_client, (u8)cmd, (u8)val);
}
int fms6502_read_data(u8 cmd)
{
	return i2c_smbus_read_byte_data(fms6502_client, cmd);
}

int limit_inputval(int val, int min, int max)
{
	if(val < min) return min;
	else if(val > max) return max;
	else return val;
}

static ssize_t show_out1(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	int ret_val = fms6502_read_data(FMS6502_REG_OUTPUT_1_2) & 0x0F;
	
	return sprintf(buf, "%d\n", ret_val);
}

static ssize_t store_out1(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int val;
    int ret;

	sscanf(buf, "%d", &val);
    val = limit_inputval(val,0,8);
	ret = fms6502_read_data(FMS6502_REG_OUTPUT_1_2);
    if (ret < 0)
        return ret;
    ret &= 0xF0;
    val |= ret;
	fms6502_write_data(FMS6502_REG_OUTPUT_1_2, val);
	return count;
}

static ssize_t show_out2(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	int ret_val = fms6502_read_data(FMS6502_REG_OUTPUT_1_2) & 0xF0;
	ret_val = ret_val >> 4;

	return sprintf(buf, "%d\n", ret_val);
}

static ssize_t store_out2(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int val;
    int ret;

	sscanf(buf, "%d", &val);
    val = limit_inputval(val,0,8);
	ret = fms6502_read_data(FMS6502_REG_OUTPUT_1_2);
    if (ret < 0)
        return ret;
    ret &= 0x0F;
    val = (val << 4) | ret;
	fms6502_write_data(FMS6502_REG_OUTPUT_1_2, val);
	return count;
}

static ssize_t show_out3(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	int ret_val = fms6502_read_data(FMS6502_REG_OUTPUT_3_4) & 0x0F;
	
	return sprintf(buf, "%d\n", ret_val);
}

static ssize_t store_out3(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int val;
    int ret;

	sscanf(buf, "%d", &val);
    val = limit_inputval(val,0,8);
	ret = fms6502_read_data(FMS6502_REG_OUTPUT_3_4);
    if (ret < 0)
        return ret;
    ret &= 0xF0;
    val |= ret;
	fms6502_write_data(FMS6502_REG_OUTPUT_3_4, val);
	return count;
}

static ssize_t show_out4(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	int ret_val = fms6502_read_data(FMS6502_REG_OUTPUT_3_4) & 0xF0;
	ret_val = ret_val >> 4;

	return sprintf(buf, "%d\n", ret_val);
}

static ssize_t store_out4(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int val;
    int ret;

	sscanf(buf, "%d", &val);
    val = limit_inputval(val,0,8);
	ret = fms6502_read_data(FMS6502_REG_OUTPUT_3_4);
    if (ret < 0)
        return ret;
    ret &= 0x0F;
    val = (val << 4) | ret;
	fms6502_write_data(FMS6502_REG_OUTPUT_3_4, val);
	return count;
}

static ssize_t show_out5(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	int ret_val = fms6502_read_data(FMS6502_REG_OUTPUT_5_6) & 0x0F;
	
	return sprintf(buf, "%d\n", ret_val);
}

static ssize_t store_out5(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int val;
    int ret;

	sscanf(buf, "%d", &val);
    val = limit_inputval(val,0,8);
	ret = fms6502_read_data(FMS6502_REG_OUTPUT_5_6);
    if (ret < 0)
        return ret;
    ret &= 0xF0;
    val |= ret;
	fms6502_write_data(FMS6502_REG_OUTPUT_5_6, val);
	return count;
}

static ssize_t show_out6(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	int ret_val = fms6502_read_data(FMS6502_REG_OUTPUT_5_6) & 0xF0;
	ret_val = ret_val >> 4;

	return sprintf(buf, "%d\n", ret_val);
}

static ssize_t store_out6(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int val;
    int ret;

	sscanf(buf, "%d", &val);
    val = limit_inputval(val,0,8);
	ret = fms6502_read_data(FMS6502_REG_OUTPUT_5_6);
    if (ret < 0)
        return ret;
    ret &= 0x0F;
    val = (val << 4) | ret;
	fms6502_write_data(FMS6502_REG_OUTPUT_5_6, val);
	return count;
}

static ssize_t show_Clamp(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	int ret_val = fms6502_read_data(FMS6502_REG_CLAMP);

	return sprintf(buf, "%d\n", ret_val);
}

static ssize_t store_Clamp(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int val;

	sscanf(buf, "%d", &val);
    
	fms6502_write_data(FMS6502_REG_CLAMP, val);
	return count;
}

static ssize_t show_Gain(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	int ret_val = fms6502_read_data(FMS6502_REG_GAIN);

	return sprintf(buf, "%d\n", ret_val);
}

static ssize_t store_Gain(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int val;

	sscanf(buf, "%d", &val);

	fms6502_write_data(FMS6502_REG_GAIN, val);
	return count;
}

static DEVICE_ATTR(out1, S_IWUSR | S_IRUGO, show_out1, store_out1);
static DEVICE_ATTR(out2, S_IWUSR | S_IRUGO, show_out2, store_out2);
static DEVICE_ATTR(out3, S_IWUSR | S_IRUGO, show_out3, store_out3);
static DEVICE_ATTR(out4, S_IWUSR | S_IRUGO, show_out4, store_out4);
static DEVICE_ATTR(out5, S_IWUSR | S_IRUGO, show_out5, store_out5);
static DEVICE_ATTR(out6, S_IWUSR | S_IRUGO, show_out6, store_out6);
static DEVICE_ATTR(Clamp, S_IWUSR | S_IRUGO, show_Clamp, store_Clamp);
static DEVICE_ATTR(Gain, S_IWUSR | S_IRUGO, show_Gain, store_Gain);

static struct attribute *fms6502_attributes[] = {
	&dev_attr_out1.attr,
	&dev_attr_out2.attr,
	&dev_attr_out3.attr,
	&dev_attr_out4.attr,
	&dev_attr_out5.attr,
	&dev_attr_out6.attr,
	&dev_attr_Clamp.attr,
	&dev_attr_Gain.attr,
	NULL,
};


static const struct attribute_group fms6502_attr_group = {
	.attrs = fms6502_attributes,
};

static const struct attribute_group *fms6502_attr_groups[] = {
	&fms6502_attr_group,
	NULL,
};

static int fms6502_misc_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int fms6502_misc_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long fms6502_misc_ioctl(struct file *file, unsigned int cmd,
			        unsigned long arg)
{
	return 0;
}

static struct file_operations fms6502_misc_fops = {
	.owner		= THIS_MODULE,
	.open		= fms6502_misc_open,
	.release	= fms6502_misc_release,
	.unlocked_ioctl	= fms6502_misc_ioctl,
};

static struct miscdevice fms6502_misc = {
	.name	= "fms6502",
	.minor	= MISC_DYNAMIC_MINOR,
	.fops	= &fms6502_misc_fops,
	.groups	= fms6502_attr_groups
};

static int fms6502_misc_register(struct i2c_client *client)
{
	int ret;
	fms6502_misc.parent = &client->dev;

	ret = misc_register(&fms6502_misc);
	if (ret) {
		dev_err(&client->dev, "misc: %s register failed, err: %d\n",
			fms6502_misc.name, ret);
		return ret;
	}

	dev_info(&client->dev, "misc: %s registered.\n", fms6502_misc.name);

	return 0;
}

static int fms6502_misc_unregister(struct i2c_client *client)
{
	int ret;

	ret = misc_deregister(&fms6502_misc);
	if (ret) {
		dev_err(&client->dev, "misc: %s unregister failed, err: %d\n",
			fms6502_misc.name, ret);
		return ret;
	}

	dev_info(&client->dev, "misc: %s unregistered.\n", fms6502_misc.name);

	return 0;
}

static int fms6502_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = client->adapter;
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;
	struct fms6502_data *data;
	data = devm_kzalloc(&client->dev, sizeof(struct fms6502_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	i2c_set_clientdata(client, data);
	data->dev = &client->dev;
	data->i2c = client;
	
	//initialize variable
	fms6502_client = client;	
	fms6502_misc_register(client);
    
    dev_info(&client->dev, "8-Input,6-Output video switch matrix of FMS6502 driver loaded successfully\n");
	// printk("8-Input,6-Output video switch matrix of FMS6502 driver loaded successfully\n");
	return 0;
}

static int fms6502_remove(struct i2c_client *client)
{
	fms6502_misc_unregister(client);
	return 0;
}
/*
static int fms6502_suspend(struct i2c_client *client)
{
	return 0;
}
static int fms6502_resume(struct i2c_client *client)
{
	fms6502_enable(gEnable);
	return 0;
}*/
static struct i2c_driver fms6502_driver = {
	.driver = {
		.name   = "fms6502",
		.owner  = THIS_MODULE,
	},
	.probe          = fms6502_probe,
	.remove         = fms6502_remove,
	.id_table       = fms6502_id,
};

static int __init fms6502_init(void)
{
	u8 err;
	err = i2c_add_driver(&fms6502_driver);
	if(err != 0)
		pr_err("%s:driver registration failed, error=%d\n",
				__func__, err);
	else
		pr_err("fms6502 init driver registration ok\n");
	return err;
}

static void __exit fms6502_exit(void)
{
	i2c_del_driver(&fms6502_driver);
}

MODULE_AUTHOR("zhangbaolin <zhangbaolin@qq.com>");
MODULE_DESCRIPTION("8-Input,6-Output Video Switch Matrix");
MODULE_LICENSE("GPL");

module_init(fms6502_init);
module_exit(fms6502_exit);