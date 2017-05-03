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
#include <linux/gpio.h>
#include <linux/ioctl.h>
#include <asm/uaccess.h>
#include <linux/input/atmega_keypad.h>

/* Commands to send to the chip. todo */
/* read cmd */
#define ATMEGA_CMD_READ_KEY             0x00 /* Read: key state */
#define ATMEGA_CMD_READ_TEMPERATURE     0x01 /* Read: temperature */
#define ATMEGA_CMD_READ_STATE           0x03 /* Read: state(8bit)=(x-x-x-x-cam-heat-func-lm75) */
#define ATMEGA_CMD_READ_VERSION         0x04 /* Read: chip version. */

/* write cmd */
#define ATMEGA_CMD_WRITE_BACKLIGHT_EN   0x05 /* Write: enable or disable backlight*/
#define ATMEGA_CMD_WRITE_FRONTCAM_EN    0x06 /* Write: enable or disable front camera*/
#define ATMEGA_CMD_WRITE_REARCAM_EN     0x07 /* Write: enable or disable back camera*/
#define ATMEGA_CMD_WRITE_CAM_LED        0x08 /* Write: set cam led to indicate front or back */
#define ATMEGA_CMD_WRITE_ALARM_LED      0x09 /* Write: turn alarm led on or off */

#define ATMEGA_CMD_WRITE_BACKLIGHT      0x0A /* Write: set the backlight value, varies from 1 to 10 */

/* The possible addresses corresponding to CONFIG1 and CONFIG2 pin wirings. todo */
#define ATMEGA_I2C_ADDR                 (0x7A)
#define ATMEGA_I2C_VERSION              (0x6E)

struct atmega_keypad {
	struct i2c_client*      client;
	struct input_dev*       idev;
	struct mutex            lock;
	struct work_struct      work;
	u8                      old_keys;
	char                    phys[32];
	int                     debounce_time;
	int                     repeat;
	unsigned short          keymap[ATMEGA_KEYMAP_SIZE];
};

#define client_to_atmega(c)     container_of(c, struct atmega_keypad, client)
#define dev_to_atmega(d)        container_of(d, struct atmega_keypad, client->dev)
#define work_to_atmega(w)       container_of(w, struct atmega_keypad, work)

#define ATMEGA_MAX_DATA		8

/*
 * To write, we just access the chip's address in write mode, and dump the
 * command and data out on the bus.  The command byte and data are taken as
 * sequential u8s out of varargs, to a maximum of ATMEGA_MAX_DATA.
 */
static int atmega_keypad_write(struct atmega_keypad* kp, int len, ...)
{
	int ret, i;
	va_list ap;
	u8 data[ATMEGA_MAX_DATA];
	va_start(ap, len);
	if (unlikely(len > ATMEGA_MAX_DATA)) {
		dev_err(&kp->client->dev, "tried to send %d bytes\n", len);
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
	ret = i2c_master_send(kp->client, data, len);
	if (unlikely(ret == -EREMOTEIO))
		ret = i2c_master_send(kp->client, data, len);
	if (unlikely(ret != len))
		dev_err(&kp->client->dev, "sent %d bytes of %d total\n",
		        len, ret);
	return ret;
}

/*
 * To read, we first send the command byte to the chip and end the transaction,
 * then access the chip in read mode, at which point it will send the data.
 */
static int atmega_keypad_read(struct atmega_keypad* kp, u8 cmd, u8* buf,
                              int len)
{
	int ret;
	/*
	 * If the host is asleep while we send the byte, we can get a NACK
	 * back while it wakes up, so try again, once.
	 */
	ret = i2c_master_send(kp->client, &cmd, 1);
	if (unlikely(ret == -EREMOTEIO))
		ret = i2c_master_send(kp->client, &cmd, 1);
	if (unlikely(ret != 1)) {
		dev_err(&(kp->client->dev), "sending read cmd 0x%02x failed\n",
		        cmd);
		return 0;
	}
	ret = i2c_master_recv(kp->client, buf, len);
	if (unlikely(ret != len))
		dev_err(&kp->client->dev, "wanted %d bytes, got %d\n",
		        len, ret);
	else {
		dev_dbg(&(kp->client->dev), "atmega-keypad: buf = %d\n", *buf);
	}
	return ret;
}


static void atmega_keypad_process_keys(struct atmega_keypad* kp, u8 new_keys)
{
	u8 diff_keys = kp->old_keys ^ new_keys;
	int i;
	if (diff_keys != 0) {
		kp->old_keys = new_keys;
	}
	else {
		return;
	}
	for (i = 0; i < ATMEGA_KEY_NUM; i++) {
		if (((diff_keys >> i) & 0x1) != 0) {
			u8 isdown = (new_keys >> i) & 0x1;
			unsigned short keycode = kp->keymap[i];
			dev_dbg(&kp->client->dev, "key 0x%02x %s\n",
			        keycode, isdown ? "down" : "up");
			input_report_key(kp->idev, keycode, isdown);
			input_sync(kp->idev);
		}
	}
}


static int atmega_keypad_init(struct atmega_keypad* kp)
{
	int bytes;
	u8 keys;
	/* The docs say we must pass 0xAA as the data byte. */
	bytes = atmega_keypad_read(kp, ATMEGA_CMD_READ_KEY, &keys, 1);
	if (unlikely(bytes != 1)) {
		return -EIO;
	}
	else {
		kp->old_keys = keys;
		return 0;
	}
}

/*
 * Bottom half: handle the interrupt by posting key events, or dealing with
 * errors appropriately.
 */
static void atmega_keypad_work(struct work_struct* work)
{
	struct atmega_keypad* kp = work_to_atmega(work);
	u8 keys;
	int i = 3;
	mutex_lock(&kp->lock);
	while ((atmega_keypad_read(kp, ATMEGA_CMD_READ_KEY, &keys, 1) != 1)) {
		if (i-- <= 0) {
			mutex_unlock(&kp->lock);
			return;
		}
	}
	atmega_keypad_process_keys(kp, keys);
	mutex_unlock(&kp->lock);
}

/*
 * We cannot use I2C in interrupt context, so we just schedule work.
 */
static irqreturn_t atmega_keypad_irq(int irq, void* data)
{
	struct atmega_keypad* kp = data;
	if (NULL != kp) {
		schedule_work(&kp->work);
	}
	else {
	}
	return IRQ_HANDLED;
}

/*
 * Read the chip Version.
 */
static int atmega_keypad_read_version(struct atmega_keypad* kp, u8* buf)
{
	int bytes;
	bytes = atmega_keypad_read(kp, ATMEGA_CMD_READ_VERSION, buf, 1);
	if (unlikely(bytes != 1)) {
		return -EIO;
	}
	return bytes;
}

#ifdef CONFIG_OF
static int atmega_keypad_probe_dt(struct device* dev,
                                  struct atmega_keypad* kp)
{
	int ret;
	struct device_node* np = dev->of_node;
	ret = of_property_read_u32(np, "debounce_time", &kp->debounce_time);
	if (ret) {
		dev_err(dev, "debounce_time missing or invalid\n");
		return ret;
	}
	ret = of_property_read_u32(np, "repeat", &kp->repeat);
	if (ret) {
		dev_err(dev, "repeat missing or invalid\n");
		return ret;
	}
	ret = of_property_read_u16_array(np, "keymap", kp->keymap,
	                                 ARRAY_SIZE(kp->keymap));
	if (ret) {
		dev_err(dev, "keymap missing or invalid\n");
		return ret;
	}
	return 0;
}
#else
static inline int atmega_keypad_probe_dt(struct device* dev,
                                         struct atmega_keypad* kp)
{
	return -ENODEV;
}
#endif

static int atmega_keypad_probe(struct i2c_client* client,
                               const struct i2c_device_id* id)
{
	struct atmega_keypad* kp;
	struct atmega_keypad_platform_data* pdata = dev_get_platdata(&client->dev);
	struct input_dev* idev;
	int i, err, gpio;
	u8 data[2];
	/* Allocate resource */
	kp = kzalloc(sizeof * kp, GFP_KERNEL);
	if (!kp)
		return -ENOMEM;
	kp->client      = client;
	mutex_init(&kp->lock);
	INIT_WORK(&kp->work, atmega_keypad_work);
	/* Check chip version */
	if (atmega_keypad_read_version(kp, data) != 1 ||
	    (*data) != ATMEGA_I2C_VERSION) {
		dev_err(&client->dev, "device not found OR dismatched version\n");
		return -ENODEV;
	}
	if (0 != atmega_keypad_init(kp)) {
		dev_err(&client->dev, "atmega-keypad init fail\n");
		return -EIO;
	}
	if (!pdata) {
		err = atmega_keypad_probe_dt(&client->dev, kp);
		if (err) {
			dev_err(&client->dev,
			        "DT probe failed and no platform data present\n");
			kfree(kp);
			return err;
		}
	}
	else {
		kp->debounce_time = pdata->debounce_time;
		kp->repeat = pdata->repeat;
		memcpy(kp->keymap, pdata->keymap, sizeof(kp->keymap));
	}
	/* Allocate input device */
	idev = input_allocate_device();
	if (!idev) {
		err = -ENOMEM;
		goto alloc_idev_error;
	}
	idev->name = "atmega-keypad";
	snprintf(kp->phys, sizeof(kp->phys),
	         "%s/input-kp", dev_name(&client->dev));
	idev->phys = kp->phys;
	idev->evbit[0] = BIT(EV_KEY);
	for (i = 0; i < ATMEGA_KEYMAP_SIZE; i++) {
		__set_bit(kp->keymap[i], idev->keybit);
	}
	__clear_bit(KEY_RESERVED, idev->keybit);
	if (kp->repeat)
		__set_bit(EV_REP, idev->evbit);
	err = input_register_device(idev);
	if (err) {
		dev_dbg(&client->dev, "error registering input device\n");
		goto register_idev_error;
	}
	kp->idev = idev;
/*
	gpio = irq_to_gpio(client->irq);
	err = gpio_request(gpio, "atmega-keypad-int-gpio");
	if (err < 0) {
		dev_err(&client->dev, "could not get GPIO %d, error %d\n", gpio, err);
		goto request_gpio_error;
	}
	err = gpio_direction_input(gpio);
	if (err < 0) {
		dev_err(&client->dev, "failed to configure"
		        " direction for GPIO %d, error %d\n", gpio, err);
		goto config_gpio_error;
	}
*/

	err = request_irq(client->irq, atmega_keypad_irq,
	                  IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
	                  "atmega-keypad", kp);
	if (err) {
		dev_err(&client->dev, "request IRQ: %d error: %d\n", client->irq, err);
		goto request_irq_error;
	}
	device_init_wakeup(&client->dev, 1);
	enable_irq_wake(client->irq);
	i2c_set_clientdata(client, kp);
	return 0;
request_irq_error:
/*
config_gpio_error:
	gpio_free(gpio);
request_gpio_error:
*/
	input_unregister_device(idev);
register_idev_error:
	input_free_device(idev);
alloc_idev_error:
	kfree(kp);
	return err;
}

static int atmega_keypad_remove(struct i2c_client* client)
{
	struct atmega_keypad* kp = i2c_get_clientdata(client);
	disable_irq_wake(client->irq);
	free_irq(client->irq, kp);
	cancel_work_sync(&kp->work);
	input_unregister_device(kp->idev);
	kfree(kp);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
/*
 * We don't need to explicitly suspend the chip, as it already switches off
 * when there's no activity.
 */
static int atmega_keypad_suspend(struct device* dev)
{
	struct i2c_client* client = to_i2c_client(dev);
	struct atmega_keypad* kp = i2c_get_clientdata(client);
	irq_set_irq_wake(client->irq, 0);
	disable_irq(client->irq);
	return 0;
}

static int atmega_keypad_resume(struct device* dev)
{
	struct i2c_client* client = to_i2c_client(dev);
	struct atmega_keypad* kp = i2c_get_clientdata(client);
	enable_irq(client->irq);
	irq_set_irq_wake(client->irq, 1);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(atmega_keypad_pm_ops,
                         atmega_keypad_suspend,
                         atmega_keypad_resume);

static const struct i2c_device_id atmega_keypad_id[] = {
	{ "atmega-keypad", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, atmega_keypad_id);

#ifdef CONFIG_OF
static const struct of_device_id atmega_keypad_of_match[] = {
	{ .compatible = "atmega,atmega-keypad", },
	{ }
};
MODULE_DEVICE_TABLE(of, atmega_keypad_of_match);
#endif

static struct i2c_driver atmega_keypad_driver = {
	.driver = {
		.owner          = THIS_MODULE,
		.name           = "atmega-keypad",
		.of_match_table = of_match_ptr(atmega_keypad_of_match),
		.pm             = &atmega_keypad_pm_ops,
	},
	.id_table       = atmega_keypad_id,
	.probe          = atmega_keypad_probe,
	.remove         = atmega_keypad_remove,
};

module_i2c_driver(atmega_keypad_driver);

MODULE_AUTHOR("Timo O. Karjalainen <timo.o.karjalainen@nokia.com>");
MODULE_AUTHOR("Daniel Stone");
MODULE_AUTHOR("Felipe Balbi <felipe.balbi@nokia.com>");
MODULE_DESCRIPTION("atmega keypad driver");
MODULE_LICENSE("GPL");

