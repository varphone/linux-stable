/*
 * MYZR Technology Co.,Ltd
 * http://www.myzr.com.cn
 * Tang Bin <tangb@myzr.com.cn>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>

#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/sched.h>

#define ZLG7290_NAME		"zlg7290"
#define ZLG7290_LED_NAME	"zlg7290_led"

#define REG_SYSTEM		0x00
#define REG_KEY_VAL		0x01
#define REG_REPEAT_CNT	0x02
#define REG_FUNC_KEY	0x03

#define REG_CMD_BUF0	0x07
#define REG_CMD_BUF1	0x08
#define REG_FLASH_ONOFF	0x0C
#define REG_SCAN_NUM	0x0D
#define REG_DP_RAM0		0x10
#define REG_DP_RAM1		0x11
#define REG_DP_RAM2		0x12
#define REG_DP_RAM3		0x13
#define REG_DP_RAM4		0x14
#define REG_DP_RAM5		0x15
#define REG_DP_RAM6		0x16
#define REG_DP_RAM7		0x17

#define ZLG7290_LED_MAJOR	800
#define ZLG7290_LED_MINOR	0
#define ZLG7290_LED_DEVICES	1

#define WRITE_DPRAM _IO('Z', 0)

struct zlg7290
{
	struct i2c_client *client;
	
	struct input_dev *input;
	struct delayed_work work;
	unsigned long delay;
	
	struct cdev cdev;
};

struct zlg7290 *ptr_zlg7290;

unsigned int key_value[65] = {
	0,
	1,  2,  3,  4,  5,  6,  7,  8,
	9,  10, 11, 12, 13, 14, 15, 16,
	17, 18, 19, 20, 21, 22, 23, 24,
	25, 26, 27, 28, 29, 30, 31, 32,
	33, 34, 35, 36, 37, 38, 39, 40,
	41, 42, 43, 44, 45, 46, 47, 48,
	49, 50, 51, 52, 53, 54, 55, 56,
	57, 58, 59, 60, 61, 62, 63, 64,
}; 

static int zlg7290_hw_write(struct zlg7290 *zlg7290,  int len, size_t *retlen, char *buf)
{
	struct i2c_client *client = zlg7290->client;
	int ret;

	struct i2c_msg msg[] = {
		{ client->addr, 0, len, buf},
	};

	ret =i2c_transfer(client->adapter, msg, 1);
	if (ret < 0) 
	{
		dev_err(&client->dev, "i2c write error!\n");
		return -EIO;
	}

	*retlen = len;
	return 0;
}

static int zlg7290_hw_read(struct zlg7290 *zlg7290, int len, size_t *retlen, char *buf)
{
	struct i2c_client *client = zlg7290->client;
	int ret;

	struct i2c_msg msg[] = { 
		{ client->addr, 0, len, buf},
		{ client->addr, I2C_M_RD, len, buf },
	};

	ret =i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) 
	{
		dev_err(&client->dev, "i2c read error!\n");
		return -EIO;
	}

	*retlen = len;
	return 0;
}

static int zlg7290_led_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int zlg7290_led_release(struct inode *inode, struct file *file)
{
	return 0;
}

static void zlg7290_kpad_work(struct work_struct *work) 
{
	struct zlg7290 *zlg7290 = container_of(work, struct zlg7290, work.work);
	unsigned char val = 0;
	size_t len;

	val = REG_SYSTEM;
	zlg7290_hw_read(zlg7290, 1, &len, &val);
	if(val & 0x1) {
		val = REG_KEY_VAL;
		zlg7290_hw_read(zlg7290, 1, &len, &val);

		if (val == 0) {
			val = REG_FUNC_KEY;
			zlg7290_hw_read(zlg7290, 1, &len, &val);
			if (val == 0 || val == 0xFF)
				goto rework;
		}

		if (val > 56) {
			switch (val) {
				case 0xFE: val = 57; break;
				case 0xFD: val = 58; break;
				case 0xFB: val = 59; break;
				case 0xF7: val = 60; break;
				case 0xEF: val = 61; break;
				case 0xDF: val = 62; break;
				case 0xBF: val = 63; break;
				case 0x7F: val = 64; break;
			}
		}

		input_report_key(zlg7290->input, key_value[val], 1);
		input_report_key(zlg7290->input, key_value[val], 0);
		input_sync(zlg7290->input);
	} 
	return;
	
rework:
	schedule_delayed_work(&zlg7290->work, zlg7290->delay);
}

static long 
zlg7290_led_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned char data_buf[8] = {0};
	unsigned char write_val[2] = {0};
	ssize_t len = 0;
	int i = 0;
	
	switch(cmd){
		case WRITE_DPRAM:
			if(copy_from_user(data_buf, (void *)arg, 8))
				return -EFAULT;
			
			for(i = 0; i < 8; i++)
			{
				write_val[0] = REG_DP_RAM0 + i;
				write_val[1] = data_buf[i];
				zlg7290_hw_write(ptr_zlg7290, 2, &len, write_val);
				msleep(1);
			}
			break;
		default:
			dev_err(&ptr_zlg7290->client->dev, "unsupported command!\n");
			break;
	}

	return 0;
}

static struct file_operations zlg7290_led_fops = {
	.owner = THIS_MODULE,
	.open = zlg7290_led_open,
	.release = zlg7290_led_release,
	.unlocked_ioctl = zlg7290_led_ioctl,
};

static int register_zlg7290_led(struct zlg7290 *zlg7290) 
{
	struct cdev *zlg7290_cdev;
	int ret;
	dev_t devid;

	devid = MKDEV(ZLG7290_LED_MAJOR, ZLG7290_LED_MINOR);
	ret = register_chrdev_region(devid, ZLG7290_LED_DEVICES, ZLG7290_LED_NAME);
	if (ret < 0) {
		dev_err(&zlg7290->client->dev, "register chrdev fail!\n");
		return ret;
	}

	zlg7290_cdev = &zlg7290->cdev;
	cdev_init(zlg7290_cdev, &zlg7290_led_fops);
	zlg7290_cdev->owner = THIS_MODULE;
	ret = cdev_add(zlg7290_cdev, devid, 1);
	if (ret < 0) {
		dev_err(&zlg7290->client->dev, "cdev add fail!\n");
		goto err_unreg_chrdev;
	}

	return 0;

err_unreg_chrdev:
	unregister_chrdev_region(devid, ZLG7290_LED_DEVICES);
	return ret;
}

static int unregister_zlg7290_led(struct zlg7290 *zlg7290) 
{
	cdev_del(&zlg7290->cdev);
	
	unregister_chrdev_region(MKDEV(ZLG7290_LED_MAJOR, ZLG7290_LED_MINOR), ZLG7290_LED_DEVICES);
	
	return 0;
}

irqreturn_t zlg7290_kpad_irq(int irq, void *handle)
{
	struct zlg7290 *zlg7290 = handle;
	
	schedule_delayed_work(&zlg7290->work, zlg7290->delay);
	
	return IRQ_HANDLED;
}

static int 
zlg7290_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct input_dev *input_dev;
	int ret = 0;
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE)) {
		dev_err(&client->dev, "%s adapter not supported\n",
			dev_driver_string(&client->adapter->dev));
		return -ENODEV;
	}
	
	ptr_zlg7290 = kzalloc(sizeof(struct zlg7290), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ptr_zlg7290 || !input_dev) {
		ret = -ENOMEM;
		goto err_free_mem;
	}
	
	input_dev->name = client->name;
	input_dev->phys = "zlg7290-keys/input0";
	input_dev->dev.parent = &client->dev;
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0001;
	input_dev->evbit[0] = BIT_MASK(EV_KEY);
	
	for(ret = 1; ret <= 64; ret++) {
		input_dev->keybit[BIT_WORD(key_value[ret])] |= BIT_MASK(key_value[ret]);
	}

	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(&client->dev, "unable to register input device\n");
		goto err_unreg_dev;
	}

	ptr_zlg7290->client = client;
	ptr_zlg7290->input = input_dev;
	i2c_set_clientdata(client, ptr_zlg7290);
	
	ret = request_threaded_irq(client->irq, NULL, zlg7290_kpad_irq, 
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					input_dev->name, ptr_zlg7290);
	if (ret) {
		dev_err(&client->dev, "irq %d busy?\n", client->irq);
		goto err_free_irq;
	}
	
	INIT_DELAYED_WORK(&ptr_zlg7290->work, zlg7290_kpad_work);
	ptr_zlg7290->delay = msecs_to_jiffies(30);
	device_init_wakeup(&client->dev, 1);

	ret = register_zlg7290_led(ptr_zlg7290);
	if (ret < 0)
		goto err_free_irq;

	return 0;

err_free_irq:
	free_irq(client->irq, ptr_zlg7290);
err_unreg_dev:
	input_unregister_device(input_dev);
	input_dev = NULL;
err_free_mem:
	input_free_device(input_dev);
	kfree(ptr_zlg7290);

	return ret;
}

static int zlg7290_remove(struct i2c_client *client) 
{
	struct zlg7290 *zlg7290 = i2c_get_clientdata(client);
	
	unregister_zlg7290_led(zlg7290);
	
	free_irq(client->irq, NULL);
	i2c_set_clientdata(client, NULL);
	
	input_unregister_device(zlg7290->input);
	input_free_device(zlg7290->input);
	
	kfree(zlg7290);
	return 0;
}

static const struct i2c_device_id zlg7290_id[] = {
	{ZLG7290_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, zlg7290_id);

#ifdef CONFIG_OF
static const struct of_device_id zlg7290_dt_ids[] = {
	{ .compatible = "myzr,zlg7290", },
	{ }
};
MODULE_DEVICE_TABLE(of, zlg7290_dt_ids);
#endif

static struct i2c_driver zlg7290_driver= {
	.driver	= {
		.name	= ZLG7290_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(zlg7290_dt_ids),
	},
	.probe		= zlg7290_probe,
	.id_table	= zlg7290_id,
	.remove		= zlg7290_remove,
};

module_i2c_driver(zlg7290_driver);

MODULE_AUTHOR("Tang Bin <tangb@myzr.com.cn>");
MODULE_DESCRIPTION("Keypad & Leds driver for ZLG7290");
MODULE_LICENSE("GPL");
