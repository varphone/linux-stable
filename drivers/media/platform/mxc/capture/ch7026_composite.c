/*
    This driver supports Chrontel CH7025/CH7026 TV/VGA Encoder.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; version 2 of the License.
*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>
#include "ch7026_composite.h"

#define CH7026_DEVICE_ID_CMD		0x00
#define CH7026_DEVICE_REVISION_ID_CMD	0x01
#define CH7026_OUTPUT_FORMAT		0x0D
#define CH7025_DID 0x55
#define CH7026_DID 0x54

static const struct i2c_device_id ch7026_id[] = {
	{ "ch7026", 0 },
	{ }
};

struct ch7026_data {
	struct device		*dev;
	struct i2c_client	*i2c;
	struct backlight_device	*bl;
	u16			chip_id;
};


struct i2c_client *ch7026_client = NULL;
static struct input_stream_info gInputInfo;
static int gEnable;

unsigned char REG_MAP_720_576[ ][2] =
{
	{ 0x02, 0x01 },{ 0x02, 0x03 },{ 0x03, 0x00 },{ 0x04, 0x39 },{ 0x07, 0x18 },{ 0x0A, 0x10 },{ 0x0D, 0x83 }, { 0x0C, 0x50 },
	{ 0x0E, 0xE4 },{ 0x0F, 0x1A },{ 0x10, 0xD0 },{ 0x11, 0x60 },{ 0x12, 0x40 },{ 0x13, 0x0C },
	{ 0x14, 0x40 },{ 0x15, 0x12 },{ 0x16, 0x40 },{ 0x17, 0x71 },{ 0x19, 0x05 },{ 0x1A, 0x05 },
	{ 0x1D, 0xC0 },{ 0x21, 0x12 },{ 0x22, 0x40 },{ 0x23, 0x71 },{ 0x41, 0xE2 },{ 0x4D, 0x04 },
	{ 0x51, 0x54 },{ 0x52, 0x1B },{ 0x53, 0x1A },{ 0x55, 0xE5 },{ 0x5C, 0x10 },{ 0x5E, 0x80 },
	{ 0x77, 0x43 },{ 0x7D, 0x62 },{ 0x04, 0x38 },{ 0x06, 0x71 },
	{ 0x03, 0x00 },{ 0x03, 0x00 },{ 0x03, 0x00 },{ 0x03, 0x00 },{ 0x03, 0x00 },
	{ 0x06, 0x70 },{ 0x02, 0x02 },{ 0x02, 0x03 },{ 0x04, 0x00 },
};

#define REG_MAP_720_576_LENGTH ( sizeof(REG_MAP_720_576) / (2*sizeof(unsigned char)) )

void ch7026_write_data(u8 cmd, u8 val)
{
	i2c_smbus_write_byte_data(ch7026_client, (u8)cmd, (u8)val);
}
int ch7026_read_data(u8 cmd)
{
	return i2c_smbus_read_byte_data(ch7026_client, cmd);
}

void ch7026_enable(bool enable)
{
	int i;
	int regmap_length, val;
	unsigned char *reg_map;

	if(ch7026_client==NULL){
		printk("can't find ch7026 chip\n");
		return;
	}
	val = ch7026_read_data(0x00);
	if ( (val != CH7025_DID ) && (val != CH7026_DID))
	{
		printk("ch7026 vendor ID error!");
		return;
	}
	
	// init sequence. //
	ch7026_write_data(0x02, 0x01);
	ch7026_write_data(0x02, 0x03);
	ch7026_write_data(0x03, 0x00);	//Page selection Register is set page1
	ch7026_write_data(0x04, 0x39);	//CH7025/CH7026 is in power-down state

	if(!enable) { 
		gEnable = 0;
		return;
	}
	gEnable = 1;
	reg_map = (unsigned char *) REG_MAP_720_576;
	regmap_length = REG_MAP_720_576_LENGTH;	
	for (i = 0 ; i < regmap_length ; i++ )
		ch7026_write_data(reg_map[2*i], reg_map[2*i+1]);
}

void ch7026_set_input_stream(struct input_stream_info *info)
{
	gInputInfo.xres = info->xres;
	gInputInfo.yres = info->yres;
	gInputInfo.iformat = info->iformat;
	gInputInfo.oformat = info->oformat;
	gInputInfo.swap = info->swap;
}

int Limit(int val, int min, int max)
{
	if(val < min) return min;
	else if(val > max) return max;
	else return val;
}

static ssize_t show_AllParam(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	int len = 0;
	int i;
	s32 rd;
	if ((rd = (int)ch7026_read_data(CH7026_DEVICE_ID_CMD)) < 0) {
		return -EIO;
     	}
	len += sprintf(buf+len, "DEVICE ID (0x00) = 0x%02x\n", rd);
	
	len += sprintf(buf+len, "\nDump CH7026 registers(HEX):\n");
	for(i=0; i<0x80; i++){
		if ((rd = (int)ch7026_read_data((u8)i)) < 0) {
			return -EIO;
		}
		len  += sprintf(buf+len, "%02x ", rd);
		if(i%16 == 15)
			len  += sprintf(buf+len, "\n");
	}
	return len;
}
static ssize_t show_Resolution(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	//horizontal
	int it, ha, va;
	it = ch7026_read_data(0x0F) & 0x07;
	ha = (it << 8) | ch7026_read_data(0x10);
	//vertical
	it = ch7026_read_data(0x15) & 0x07;
	va = (it << 8) | ch7026_read_data(0x16);
	return sprintf(buf, "horizontal = %d, vertical=%d\n", ha, va);
	
}
static ssize_t show_Saturation(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	int sat = ch7026_read_data(0x2F) & 0x7F ;
	return sprintf(buf, "Saturation = %d\n", sat);
}
static ssize_t store_Saturation(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int sat;
	u8 old;

	sscanf(buf, "%d", &sat);
	old = ch7026_read_data(0x2F);
	old = (old & 0x80) | (Limit(sat,0,127) & 0x7F);
	ch7026_write_data(0x2F,old);
	
	return count;
}
static ssize_t show_Hue(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	int hue = ch7026_read_data(0x2E) & 0x7F;
	
	return sprintf(buf, "Hue = %d\n", hue);
}
static ssize_t store_Hue(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int hue;
	u8 old;

	sscanf(buf, "%d", &hue);
	old = ch7026_read_data(0x2E);
	old = (old & 0x80) | (Limit(hue,0,127) & 0x7F);
	ch7026_write_data(0x2E,old);
		
	return count;
}
static ssize_t show_Contrast(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	int contrast = ch7026_read_data(0x30) & 0x7F;
	
	return sprintf(buf, "contrast = %d\n", contrast);
}
static ssize_t store_Contrast(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int cta;
	u8 old;

	sscanf(buf, "%d", &cta);
	old = ch7026_read_data(0x30);
	old = (old & 0x80) | (Limit(cta,0,127) & 0x7F);
	ch7026_write_data(0x30,old);
	
	return count;
}

static ssize_t show_Brightness(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	int brightness = ch7026_read_data(0x31) & 0xFF;
	
	return sprintf(buf, "brightness = %d\n", brightness);
}

static ssize_t store_Brightness(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int bri;

	sscanf(buf, "%d", &bri);
	ch7026_write_data(0x31, (Limit(bri,0,255) & 0xFF));
	
	return count;
}

static ssize_t store_SetInputSignal(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	sscanf(buf, "%d %d %d %d %d", &(gInputInfo.xres), &(gInputInfo.yres), 
			&(gInputInfo.iformat), &(gInputInfo.oformat), &(gInputInfo.swap));
	printk("store_SetInputSignal xres = %d yres = %d iformat = %d oformat = %d swap = %d\n",
			gInputInfo.xres, gInputInfo.yres, gInputInfo.iformat, gInputInfo.oformat, gInputInfo.swap);
	return count;
}
static ssize_t store_ResetChip(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int enable;
	sscanf(buf, "%d", &enable);
	ch7026_enable((bool)enable);
	
	return count;
}

static ssize_t store_TestPatternSelection(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int mode;

	sscanf(buf, "%d", &mode);
	ch7026_write_data(0x03, 0x01);
	ch7026_write_data(0x04, (Limit(mode,0,0x3f) & 0x3f));	
	ch7026_write_data(0x03, 0x00);
	
	return count;
}

static ssize_t store_QuitTestPattern(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int mode;

	sscanf(buf, "%d", &mode);
	if(mode == 1)
	{
		ch7026_write_data(0x03, 0x01);
		ch7026_write_data(0x04, 0x00);
		ch7026_write_data(0x03, 0x00);
	}

	return count;
}

static DEVICE_ATTR(AllParam, S_IRUGO, show_AllParam, NULL);
static DEVICE_ATTR(Resolution, S_IRUGO, show_Resolution, NULL);
static DEVICE_ATTR(Saturation, S_IWUSR | S_IRUGO, show_Saturation, store_Saturation);
static DEVICE_ATTR(Hue, S_IWUSR | S_IRUGO, show_Hue, store_Hue);
static DEVICE_ATTR(Contrast, S_IWUSR | S_IRUGO, show_Contrast, store_Contrast);
static DEVICE_ATTR(Brightness, S_IWUSR | S_IRUGO, show_Brightness, store_Brightness);
static DEVICE_ATTR(SetInputSignal, S_IWUSR , NULL, store_SetInputSignal);
static DEVICE_ATTR(ResetChip, S_IWUSR , NULL, store_ResetChip);
static DEVICE_ATTR(TestPatternSelection, S_IWUSR , NULL, store_TestPatternSelection);
static DEVICE_ATTR(QuitTestPattern, S_IWUSR , NULL, store_QuitTestPattern);

static struct attribute *ch7026_attributes[] = {
	&dev_attr_AllParam.attr,
	&dev_attr_Resolution.attr,
	&dev_attr_Saturation.attr,
	&dev_attr_Hue.attr,
	&dev_attr_Contrast.attr,
	&dev_attr_Brightness.attr,
	&dev_attr_SetInputSignal.attr,
	&dev_attr_ResetChip.attr,
	&dev_attr_TestPatternSelection.attr,
	&dev_attr_QuitTestPattern.attr,
	NULL,
};


static const struct attribute_group ch7026_attr_group = {
	.attrs = ch7026_attributes,
};

static const struct attribute_group *ch7026_attr_groups[] = {
	&ch7026_attr_group,
	NULL,
};

static int ch7026_misc_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int ch7026_misc_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long ch7026_misc_ioctl(struct file *file, unsigned int cmd,
			        unsigned long arg)
{
	return 0;
}

static struct file_operations ch7026_misc_fops = {
	.owner		= THIS_MODULE,
	.open		= ch7026_misc_open,
	.release	= ch7026_misc_release,
	.unlocked_ioctl	= ch7026_misc_ioctl,
};

static struct miscdevice ch7026_misc = {
	.name	= "ch7026",
	.minor	= MISC_DYNAMIC_MINOR,
	.fops	= &ch7026_misc_fops,
	.groups	= ch7026_attr_groups
};

static int ch7026_misc_register(struct i2c_client *client)
{
	int ret;
	ch7026_misc.parent = &client->dev;

	ret = misc_register(&ch7026_misc);
	if (ret) {
		dev_err(&client->dev, "misc: %s register failed, err: %d\n",
			ch7026_misc.name, ret);
		return ret;
	}

	dev_info(&client->dev, "misc: %s registered.\n", ch7026_misc.name);

	return 0;
}

static int ch7026_misc_unregister(struct i2c_client *client)
{
	int ret;

	ret = misc_deregister(&ch7026_misc);
	if (ret) {
		dev_err(&client->dev, "misc: %s unregister failed, err: %d\n",
			ch7026_misc.name, ret);
		return ret;
	}

	dev_info(&client->dev, "misc: %s unregistered.\n", ch7026_misc.name);

	return 0;
}

static int ch7026_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	u8 val;
	int err;
	struct i2c_adapter *adapter = client->adapter;
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;
	
	struct ch7026_data *data;
	data = devm_kzalloc(&client->dev, sizeof(struct ch7026_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	i2c_set_clientdata(client, data);
	data->dev = &client->dev;
	data->i2c = client;

	//Make sure CH7025/26B in the system:
	val = i2c_smbus_read_byte_data(client, 0x00);
	if ( (val != CH7025_DID ) && (val != CH7026_DID))
	{
		dev_err(&client->dev, "Failed to initialize Chrontel CH7025/CH7026 TV/VGA Encoder\n");
		return -EIO; // CH7025/26B was not found
	}
	
	//initialize variable
	gInputInfo.xres = 720;
	gInputInfo.yres = 576;
	gInputInfo.iformat = CH7026_RGB666;
	gInputInfo.oformat = CH7026_PAL_B;
	gInputInfo.swap = CH7026_RGB_ORDER;
	gEnable = 0;
	ch7026_client = client;

	printk("Chrontel CH7025/CH7026 TV/VGA Encoder driver loaded successfully\n");
	
	ch7026_misc_register(client);
	//ch7026_enable(true);
	return 0;
}
static int ch7026_remove(struct i2c_client *client)
{
	ch7026_misc_unregister(client);
	return 0;
}
/*
static int ch7026_suspend(struct i2c_client *client)
{
	return 0;
}
static int ch7026_resume(struct i2c_client *client)
{
	ch7026_enable(gEnable);
	return 0;
}*/
static struct i2c_driver ch7026_driver = {
	.driver = {
		.name   = "ch7026",
		.owner  = THIS_MODULE,
	},
	.probe          = ch7026_probe,
	.remove         = ch7026_remove,
	.id_table       = ch7026_id,
};

static int __init ch7026_init(void)
{
	u8 err;
	err = i2c_add_driver(&ch7026_driver);
	if(err != 0)
		pr_err("%s:driver registration failed, error=%d\n",
				__func__, err);
	else
		pr_err("ch7026 init driver registration ok\n");
	return err;
}

static void __exit ch7026_exit(void)
{
	i2c_del_driver(&ch7026_driver);
}

MODULE_AUTHOR("zhangbaolin <nj_zbl@163.com>");
MODULE_DESCRIPTION("Chrontel CH7025/CH7026 TV/VGA Encoder");
MODULE_LICENSE("GPL");

module_init(ch7026_init);
module_exit(ch7026_exit);
