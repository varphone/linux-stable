/*
 * drivers/input/keyboard/Atmega_i2c_keypad.c
 *
 * not a real keypad, the Atmega send keypad data via I2C, the host
 * decode the data and know which button was down or released
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/leds.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/ioctl.h>
#include <asm/uaccess.h>
#include "atmega_kp.h"

#define ATMEGA_FPGA_RST  		0x01 /* reset FPGA when write register 0x1 to 0x1*/
#define ATMEGA_BACKLIGHT  		0x03 /* Write: set the backlight value, varies from 1 to 10 */
#define ATMEGA_CMD_READ_VERSION  	0x65 /* Read: chip version. */
#define ATMEGA_CMD_WRITE_BACKLIGHT_EN  	0x05 /* Write: enable or disable backlight*/


/* The possible addresses corresponding to CONFIG1 and CONFIG2 pin wirings. todo */
/* ATMEGA ADDR: 0x7A */
#define ATMEGA_I2C_VERSION  		0x00

static struct atmega_chip  * atmega;
struct miscdevice atmega_ctrl_dev;
static struct i2c_driver atmega_i2c_driver;

struct atmega_chip {
	/* device lock */
	struct mutex		lock;
	struct i2c_client	*client;
	struct miscdevice	*mdev;
	bool			kp_enabled;
};

#define client_to_atmega(c)	container_of(c, struct atmega_chip, client)
#define dev_to_atmega(d)	container_of(d, struct atmega_chip, client->dev)
#define work_to_atmega(w)	container_of(w, struct atmega_chip, work)

#define ATMEGA_MAX_DATA  	8

/*
 * To write, we just access the chip's address in write mode, and dump the
 * command and data out on the bus.  The command byte and data are taken as
 * sequential u8s out of varargs, to a maximum of ATMEGA_MAX_DATA.
 */
static int atmega_write(struct atmega_chip *atm, int len, ...)
{
	int ret, i;
	va_list ap;
	u8 data[ATMEGA_MAX_DATA];

	va_start(ap, len);

	if (unlikely(len > ATMEGA_MAX_DATA)) {
		dev_err(&atm->client->dev, "tried to send %d bytes\n", len);
		va_end(ap);
		return 0;
	}

	for (i = 0; i < len; i++)
		data[i] = va_arg(ap, int);

	va_end(ap);

	/*
	 * If the host is asleep while we send the data, we can get a NACK
	 * back while it wakes up, so try again, once.
	 */
	ret = i2c_master_send(atm->client, data, len);
	if (unlikely(ret == -EREMOTEIO))
		ret = i2c_master_send(atm->client, data, len);
	if (unlikely(ret != len))
		dev_err(&atm->client->dev, "sent %d bytes of %d total\n",
			len, ret);

	return ret;
}

/*
 * To read, we first send the command byte to the chip and end the transaction,
 * then access the chip in read mode, at which point it will send the data.
 */
static int atmega_read(struct atmega_chip *atm, u8 addr, u8 *buf, int len)
{
	int ret;

	/*
	 * If the host is asleep while we send the byte, we can get a NACK
	 * back while it wakes up, so try again, once.
	 */
	ret = i2c_master_send(atm->client, &addr, 1);
	if (unlikely(ret == -EREMOTEIO))
		ret = i2c_master_send(atm->client, &addr, 1);
	if (unlikely(ret != 1)) {
		dev_err(&(atm->client->dev), "sending read cmd 0x%02x failed\n",addr);
		return 0;
	}

	ret = i2c_master_recv(atm->client, buf, len);
	if (unlikely(ret != len))
		dev_err(&atm->client->dev, "wanted %d bytes, got %d\n",
			len, ret);

	return ret;
}

/*
 * Read the chip Version.
 */
static int atmega_read_version(struct atmega_chip *atm, u8 *buf)
{
	int bytes;

	bytes = atmega_read(atm, ATMEGA_CMD_READ_VERSION, buf, 1);
	if (unlikely(bytes != 1)) {
		return -EIO;
	}
	return bytes;
}

/*
 * Read the chip Version.
 */
static int atmega_read_backlight(struct atmega_chip *atm, u8 *buf)
{
	int bytes;

	bytes = atmega_read(atm, ATMEGA_BACKLIGHT, buf, 1);
	if (unlikely(bytes != 1)) {
		return -EIO;
	}
	return bytes;
}


static ssize_t atmega_show_disable(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct atmega_chip *atm = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", !atm->kp_enabled);
}

static ssize_t atmega_set_disable(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct atmega_chip *atm = dev_get_drvdata(dev);
	int ret;
	unsigned long i;

	ret = strict_strtoul(buf, 10, &i);

	mutex_lock(&atm->lock);
	atm->kp_enabled = !i;
	mutex_unlock(&atm->lock);

	return count;
}
static DEVICE_ATTR(disable_kp, 0644, atmega_show_disable, atmega_set_disable);

static int __devinit atmega_kp_probe(struct i2c_client *client,
				     const struct i2c_device_id *id)
{
	struct atmega_chip *atm;
	int err;
	u8 data[2];

	atm = kzalloc(sizeof *atm, GFP_KERNEL);
	if (!atm) {
		err = -ENOMEM;
		goto fail1;
	}
	atm->client = client;
	mutex_init(&atm->lock);

	if (atmega_read_backlight(atm, data) !=1) {
		dev_err(&client->dev, "device not found OR dismatched version: 0x%02x\n", *data);
		err = -ENODEV;
		goto fail1;
	}

	err = device_create_file(&client->dev, &dev_attr_disable_kp);
	if (err < 0)
		goto fail1;

	err = misc_register(&atmega_ctrl_dev);
	if(err) {
		dev_err(&client->dev, "could not register atmega_ctrl as misc\n");
		goto fail2;
	}
	atm->mdev = &atmega_ctrl_dev;

	i2c_set_clientdata(client, atm);

	atmega = atm;
	return 0;

fail2:
	device_remove_file(&client->dev, &dev_attr_disable_kp);
fail1:
	kfree(atm);
	return err;
}

static int __devexit atmega_remove(struct i2c_client *client)
{
	struct atmega_chip *atm = i2c_get_clientdata(client);

	misc_deregister(&atmega_ctrl_dev);
	device_remove_file(&atm->client->dev, &dev_attr_disable_kp);
	kfree(atm);

	return 0;
}

static const struct i2c_device_id atmega_id[] = {
	{ "atmega-keypad", 0 },
	{ }
};

static struct i2c_driver atmega_i2c_driver = {
	.driver = {
		.name	= "atmega-keypad",
		/* .pm	= &atmega_pm_ops, */
	},
	.probe		= atmega_kp_probe,
	.remove		= __devexit_p(atmega_remove),
	.id_table	= atmega_id,
};

static int atmega_ctrl_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int atmega_ctrl_release(struct inode *inode, struct file * filp)
{
	return 0;
}

static int atmega_ctrl_read(struct file *filp, char *buf, size_t count, loff_t *fpos)
{
	return 0;
}

static ssize_t atmega_ctrl_write(struct file *filp, const char *buf, size_t count, loff_t *fpos)
{
	return 0;
}

static int atmega_ctrl_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret;
	int bytes = 0;
	u8 on = 0;
	u8 backlight = 0;

	switch(cmd) {
	case ATMEGA_S_BACKLIGHT_ON:
		on = (u8)arg;
		bytes = atmega_write(atmega, 2, ATMEGA_CMD_WRITE_BACKLIGHT_EN, !!on);
		if(likely(2 == bytes)) {
			return 0;
		}
		else {
			return -EIO;
		}

	case ATMEGA_S_BACKLIGHT_VALUE:
		backlight = (u8)arg;
		if (backlight > 100)
			return -EINVAL;

		bytes = atmega_write(atmega, 2, ATMEGA_BACKLIGHT, backlight);
		if (likely(2 == bytes)) {
			return 0;
		}
		else {
			return -EIO;
		}

        case ATMEGA_G_BACKLIGHT_VALUE:
		bytes = atmega_read(atmega, ATMEGA_BACKLIGHT, &backlight, 1);
		ret = copy_to_user((void *__user)arg, &backlight, 1);
		if (bytes != 1) {
			return -EIO;
		}
		else {
			return 0;
		}
        case ATMEGA_RESET_FPGA:
		bytes = atmega_write(atmega, 2, ATMEGA_FPGA_RST, 0x1);
		if(likely(2 == bytes)) {
			return 0;
		}
		else {
			return -EIO;
		}
	default:
		dev_err(&atmega->client->dev, "unknown ioctl %d\n", cmd);
		break;
	}
	return 0;
}

long atmega_ctrl_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret;

	mutex_lock(&atmega->lock);
	ret = atmega_ctrl_ioctl(filp, cmd, arg);
	mutex_unlock(&atmega->lock);

	return ret;

}

struct file_operations atmega_ctrl_fops =
{
	.owner = THIS_MODULE,
	.open = atmega_ctrl_open,
	.release = atmega_ctrl_release,
	.write = atmega_ctrl_write,
	.read = atmega_ctrl_read,
	.unlocked_ioctl = atmega_ctrl_unlocked_ioctl
};

struct miscdevice atmega_ctrl_dev =
{
	.minor = MISC_DYNAMIC_MINOR,
	.fops = &atmega_ctrl_fops,
	.name = "atmega_ctrl",
	.nodename = "atmega_ctrl_node"
};

MODULE_DEVICE_TABLE(i2c, atmega_id);

static int __init atmega_kp_init(void)
{
	return i2c_add_driver(&atmega_i2c_driver);
}
module_init(atmega_kp_init);

static void __exit atmega_kp_exit(void)
{
	i2c_del_driver(&atmega_i2c_driver);
}
module_exit(atmega_kp_exit);

MODULE_AUTHOR("Timo O. Karjalainen <timo.o.karjalainen@nokia.com>");
MODULE_AUTHOR("Daniel Stone");
MODULE_AUTHOR("Felipe Balbi <felipe.balbi@nokia.com>");
MODULE_DESCRIPTION("atmega keypad driver");
MODULE_LICENSE("GPL");
