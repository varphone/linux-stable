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
#include <linux/i2c/atmega_kp.h>
#include <linux/slab.h>
#include <linux/gpio.h>

#include <linux/miscdevice.h>
#include <linux/ioctl.h>
#include <asm/uaccess.h>

/* Commands to send to the chip. todo */
// read cmd
#define ATMEGA_CMD_READ_KEY				0x00 /* Read: key state */
#define ATMEGA_CMD_READ_TEMPERATURE		0x01 /* Read: temperature */
#define ATMEGA_CMD_READ_STATE			0x03 /* Read: state(8bit)=(x-x-x-x-cam-heat-func-lm75) */
#define ATMEGA_CMD_READ_VERSION			0x04 /* Read: chip version. */
// write cmd
#define ATMEGA_CMD_WRITE_BACKLIGHT_EN   0x05 /* Write: enable or disable backlight*/
#define ATMEGA_CMD_WRITE_FRONTCAM_EN     0x06 /* Write: enable or disable front camera*/
#define ATMEGA_CMD_WRITE_REARCAM_EN     0x07 /* Write: enable or disable back camera*/
#define ATMEGA_CMD_WRITE_CAM_LED      0x08 /* Write: set cam led to indicate front or back */
#define ATMEGA_CMD_WRITE_ALARM_LED      0x09 /* Write: turn alarm led on or off */

//obsoleted
//#define ATMEGA_CMD_WRITE_TEMPERATURE_MODE      0x0A /* Write: set the temperature mode, 1 for normal temperature, 0 for low temperature */
//#define ATMEGA_CMD_READ_TEMPERATURE_MODE      0x0A /* Read: get the temperature mode, 1 for normal temperature, 0 for low temperature */

//
#define ATMEGA_CMD_WRITE_BACKLIGHT      0x0A /* Write: set the backlight value, varies from 1 to 10 */

/* The possible addresses corresponding to CONFIG1 and CONFIG2 pin wirings. todo */
#define ATMEGA_I2C_ADDR		(0x7A)	
#define ATMEGA_I2C_VERSION	(0x6E)	

static struct atmega_chip  * atmega;
struct miscdevice atmega_ctrl_dev;
static struct i2c_driver atmega_i2c_driver;


struct atmega_chip {
	/* device lock */
	struct mutex		lock;
	struct i2c_client	*client;
	struct work_struct	work;
	struct input_dev	*idev;
	struct miscdevice	*mdev;
	bool			kp_enabled;
	bool			pm_suspend;
	u8 				old_keys;
	char			phys[32];
	unsigned short		keymap[ATMEGA_KEYMAP_SIZE];
	int			debounce_time;
};

#define client_to_atmega(c)	container_of(c, struct atmega_chip, client)
#define dev_to_atmega(d)	container_of(d, struct atmega_chip, client->dev)
#define work_to_atmega(w)	container_of(w, struct atmega_chip, work)

#define ATMEGA_MAX_DATA 8

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
static int atmega_read(struct atmega_chip *atm, u8 cmd, u8 *buf, int len)
{
	int ret;

	/*
	 * If the host is asleep while we send the byte, we can get a NACK
	 * back while it wakes up, so try again, once.
	 */
	ret = i2c_master_send(atm->client, &cmd, 1);
	if (unlikely(ret == -EREMOTEIO))
		ret = i2c_master_send(atm->client, &cmd, 1);
	if (unlikely(ret != 1)) {
		dev_err(&(atm->client->dev), "sending read cmd 0x%02x failed\n",
			cmd);
		return 0;
	}

	ret = i2c_master_recv(atm->client, buf, len);
	if (unlikely(ret != len))
		dev_err(&atm->client->dev, "wanted %d bytes, got %d\n",
			len, ret);
	//add by xym start
	else {
		dev_dbg(&(atm->client->dev), "atmega-keypad: buf = %d\n", *buf);

	}
	//add by xym end

	return ret;
}


static void process_keys(struct atmega_chip *atm, u8 new_keys)
{
	u8 diff_keys = atm->old_keys ^ new_keys;
	int i;

	if(diff_keys != 0) {
		atm->old_keys = new_keys;
	}
	else {
		return;
	}

	for(i = 0; i < ATMEGA_KEY_NUM; i++) {
        if(((diff_keys >> i) & 0x1) != 0) {
			u8 isdown = (new_keys >> i) & 0x1;
			unsigned short keycode = atm->keymap[i];

			dev_dbg(&atm->client->dev, "key 0x%02x %s\n",
			 	keycode, isdown ? "down" : "up");

			if (atm->kp_enabled) {
				input_report_key(atm->idev, keycode, isdown);
				input_sync(atm->idev);
			}
		}
	}
}


static int atmega_init(struct atmega_chip *atm)
{
	int bytes;
	u8 keys;

	/* The docs say we must pass 0xAA as the data byte. */
	bytes = atmega_read(atm, ATMEGA_CMD_READ_KEY, &keys, 1);
	if (unlikely(bytes != 1)) {
		return -EIO;
	}
	else {
		atm->old_keys = keys;
		return 0;
	}

}

/*
 * Bottom half: handle the interrupt by posting key events, or dealing with
 * errors appropriately.
 */
static void atmega_work(struct work_struct *work)
{
	struct atmega_chip *atm = work_to_atmega(work);
	u8 keys;
	int i = 3;

	mutex_lock(&atm->lock);

	while ((atmega_read(atm, ATMEGA_CMD_READ_KEY, &keys, 1) != 1)) {
		if(i-- <= 0) {
			return;
		}
	}

	process_keys(atm, keys);
	//while(atmega_write(atm, 1, ATMEGA_CMD_READ_CONFIRM) != 1) {}

	mutex_unlock(&atm->lock);
}

/*
 * We cannot use I2C in interrupt context, so we just schedule work.
 */
static irqreturn_t atmega_irq(int irq, void *data)
{
	struct atmega_chip *atm = data;

	if(NULL != atm) {
		schedule_work(&atm->work);
	}
	else {
		printk("atm = null\n");
	}

	return IRQ_HANDLED;
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
	struct atmega_kp_platform_data *pdata = client->dev.platform_data;
	struct input_dev *idev;
	struct atmega_chip *atm;
	//int pwm;
	int i, err, gpio;
	//unsigned long tmo;
	u8 data[2];

	if (!pdata) {
		dev_err(&client->dev, "missing platform_data\n");
		return -EINVAL;
	}


	atm = kzalloc(sizeof *atm, GFP_KERNEL);
	idev = input_allocate_device();
	if (!atm || !idev) {
		err = -ENOMEM;
		goto fail1;
	}

	atm->client = client;
	atm->idev = idev;
	mutex_init(&atm->lock);
	INIT_WORK(&atm->work, atmega_work);

	atm->debounce_time = pdata->debounce_time;

	/* Nothing's set up to service the IRQ yet, so just spin for max.
	 * 100ms until we can configure. */
	/*
	tmo = jiffies + msecs_to_jiffies(100);
	while (lm8323_read(lm, LM8323_CMD_READ_INT, data, 1) == 1) {
		if (data[0] & INT_NOINIT)
			break;

		if (time_after(jiffies, tmo)) {
			dev_err(&client->dev,
				"timeout waiting for initialisation\n");
			break;
		}

		msleep(1);
	}

	*/

	/* If a true probe check the device */
	//2015-05-28
	if (atmega_read_version(atm, data) != 1 || (*data) != ATMEGA_I2C_VERSION) {
		dev_err(&client->dev, "device not found OR dismatched version\n");
		err = -ENODEV;
		goto fail1;
	}

	if(0 != atmega_init(atm)) {
		dev_err(&client->dev, "atmega-keypad init fail\n");
		err = -EIO;
		goto fail1;
	}
	//2015-05-28

	atm->kp_enabled = true;
	err = device_create_file(&client->dev, &dev_attr_disable_kp);
	if (err < 0)
		goto fail1;

	idev->name = pdata->name ? : "atmega-keypad";
	snprintf(atm->phys, sizeof(atm->phys),
		 "%s/input-kp", dev_name(&client->dev));
	idev->phys = atm->phys;

	idev->evbit[0] = BIT(EV_KEY);
	for (i = 0; i < ATMEGA_KEYMAP_SIZE; i++) {
		__set_bit(pdata->keymap[i], idev->keybit);
		atm->keymap[i] = pdata->keymap[i];
	}
	__clear_bit(KEY_RESERVED, idev->keybit);

	if (pdata->repeat)
		__set_bit(EV_REP, idev->evbit);

	err = input_register_device(idev);
	if (err) {
		dev_dbg(&client->dev, "error registering input device\n");
		goto fail2;
	}


	gpio = irq_to_gpio(client->irq);
	err = gpio_request(gpio, "atmega-int-gpio");
	if(err < 0) {
		dev_err(&client->dev, "could not get GPIO %d, error %d\n", gpio, err);
		goto fail3;
	}
	err = gpio_direction_input(gpio);
	if (err < 0) {
		dev_err(&client->dev, "failed to configure"
			" direction for GPIO %d, error %d\n", gpio, err);
		goto fail4;
	}

	err = request_irq(client->irq, atmega_irq,
			  IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			  "atmega", atm);
	if (err) {
		dev_err(&client->dev, "could not get IRQ %d\n", client->irq);
		goto fail4;
	}

	device_init_wakeup(&client->dev, 1);
	enable_irq_wake(client->irq);


	err = misc_register(&atmega_ctrl_dev);
	if(err) {
		dev_err(&client->dev, "could not register atmega_ctrl as misc\n");
		goto fail5;
	}
	atm->mdev = &atmega_ctrl_dev;

	i2c_set_clientdata(client, atm);

	atmega = atm;
	return 0;

fail5:
	free_irq(client->irq, atm);
fail4:
	gpio_free(gpio);
fail3:
	input_unregister_device(idev);
	idev = NULL;
fail2:
	device_remove_file(&client->dev, &dev_attr_disable_kp);
fail1:
	input_free_device(idev);
	kfree(atm);
	return err;
}

static int __devexit atmega_remove(struct i2c_client *client)
{
	struct atmega_chip *atm = i2c_get_clientdata(client);

	misc_deregister(&atmega_ctrl_dev);
	disable_irq_wake(client->irq);
	free_irq(client->irq, atm);
	cancel_work_sync(&atm->work);

	input_unregister_device(atm->idev);

	device_remove_file(&atm->client->dev, &dev_attr_disable_kp);


	kfree(atm);

	return 0;
}

#ifdef CONFIG_PM
/*
 * We don't need to explicitly suspend the chip, as it already switches off
 * when there's no activity.
 */
static int atmega_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct atmega_chip *atm = i2c_get_clientdata(client);
	//int i;

	irq_set_irq_wake(client->irq, 0);
	disable_irq(client->irq);

	mutex_lock(&atm->lock);
	atm->pm_suspend = true;
	mutex_unlock(&atm->lock);

	return 0;
}

static int atmega_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct atmega_chip *atm = i2c_get_clientdata(client);

	mutex_lock(&atm->lock);
	atm->pm_suspend = false;
	mutex_unlock(&atm->lock);

	enable_irq(client->irq);
	irq_set_irq_wake(client->irq, 1);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(atmega_pm_ops, atmega_suspend, atmega_resume);

static const struct i2c_device_id atmega_id[] = {
	{ "atmega-keypad", 0 },
	{ }
};

static struct i2c_driver atmega_i2c_driver = {
	.driver = {
		.name	= "atmega-keypad",
		.pm	= &atmega_pm_ops,
	},
	.probe		= atmega_kp_probe,
	.remove		= __devexit_p(atmega_remove),
	.id_table	= atmega_id,
};

int atmega_ctrl_open(struct inode *inode, struct file *filp)
{
	return 0;
}

int atmega_ctrl_release(struct inode *inode, struct file * filp) 
{
	return 0;
}

int atmega_ctrl_read(struct file *filp, char *buf, size_t count, loff_t *fpos)
{
	return 0;
}

ssize_t atmega_ctrl_write(struct file *filp, const char *buf, size_t count, loff_t *fpos) {
	return 0;
}


int atmega_ctrl_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int bytes = 0;
	u8 temp[2] = {0,};
	s16 temperature = 0;
	u8 state = 0;
	u8 on = 0;
	u8 front = 0;
	u8 backlight = 0;

	switch(cmd) {

		case ATMEGA_G_TEMPERATURE:

			bytes = atmega_read(atmega, ATMEGA_CMD_READ_TEMPERATURE, temp, 2);
			if(likely(2 == bytes)) {
				temperature = (s16)(temp[1] << 8) | (temp[0]);
				put_user(temperature, (s16 *) arg);
				return 0;
			}
			else {
				return -EIO;
			}

		case ATMEGA_G_STATE: //?????

			bytes = atmega_read(atmega, ATMEGA_CMD_READ_STATE, &state, 1);
			if(likely(1 == bytes)) {
				put_user(state, (u8 *) arg);
				return 0;
			}
			else {
				return -EIO;
			}

//obsoleted
/*
		case ATMEGA_G_TEMPERATURE_MODE:

			//0 for low temperature, 1 for normal temperature
			bytes = atmega_read(atmega, ATMEGA_CMD_READ_TEMPERATURE_MODE, &state, 1);
			if(likely(1 == bytes)) {
				put_user(state, (u8 *) arg);
				return 0;
			}
			else {
				return -EIO;
			}
*/


		case ATMEGA_S_BACKLIGHT_ON:

//			get_user(on, (u8 *) arg);
			on = (u8)arg;

			bytes = atmega_write(atmega, 2, ATMEGA_CMD_WRITE_BACKLIGHT_EN, !!on);
			if(likely(2 == bytes)) {
				return 0;
			}
			else {
				return -EIO;
			}

		case ATMEGA_S_FRONTCAM_ON:

//			get_user(on, (u8 *) arg);
			on = (u8)arg;

			bytes = atmega_write(atmega, 2, ATMEGA_CMD_WRITE_FRONTCAM_EN, !!on);
			if(likely(2 == bytes)) {
				return 0;
			}
			else {
				return -EIO;
			}

		case ATMEGA_S_REARCAM_ON:

//			get_user(on, (u8 *) arg);
			on = (u8) arg;

			bytes = atmega_write(atmega, 2, ATMEGA_CMD_WRITE_REARCAM_EN, !!on);
			if(likely(2 == bytes)) {
				return 0;
			}
			else {
				return -EIO;
			}

		case ATMEGA_S_CAM_LED:

//			get_user(front, (u8 *) arg);
			front = (u8) arg;

			bytes = atmega_write(atmega, 2, ATMEGA_CMD_WRITE_CAM_LED, !!front);
			if(likely(2 == bytes)) {
				return 0;
			}
			else {
				return -EIO;
			}

		case ATMEGA_S_ALARM_LED:

//			get_user(on, (u8 *) arg);
			on = (u8) arg;

			bytes = atmega_write(atmega, 2, ATMEGA_CMD_WRITE_ALARM_LED, !!on);
			if(likely(2 == bytes)) {
				return 0;
			}
			else {
				return -EIO;
			}

//obsoleted
/*
		case ATMEGA_S_TEMPERATURE_MODE:

//			get_user(on, (u8 *) arg);
			mode = (u8) arg;

			//0 for low temperature, 1 for normal temperature
			bytes = atmega_write(atmega, 2, ATMEGA_CMD_WRITE_TEMPERATURE_MODE, !!mode);
			if(likely(2 == bytes)) {
				return 0;
			}
			else {
				return -EIO;
			}
*/
		case ATMEGA_S_BACKLIGHT_VALUE:


//			get_user(on, (u8 *) arg);
			backlight = (u8) arg;

			//backlight varies from 1 to 10
			bytes = atmega_write(atmega, 2, ATMEGA_CMD_WRITE_BACKLIGHT, backlight);
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

long atmega_ctrl_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
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

