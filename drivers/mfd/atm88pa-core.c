/*
 * drivers/mfd/atm88pa-core.c
 *
 * Core of ATMEGA88PA MFD driver
 *
 * Copyright (C) 2017 NanJing No.55 Research Institute Technology Development CO.,LTD.
 *
 * Author: Varphone Wong <varphone@qq.com>
 *
 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/leds.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/mfd/atm88pa.h>
#include <linux/mfd/atm88pa-private.h>

int atm88pa_read(struct atm88pa *atm, u8 reg)
{
	return i2c_smbus_read_byte_data(atm->i2c, reg);
}

int atm88pa_read_word(struct atm88pa *atm, u8 reg)
{
	return i2c_smbus_read_word_data(atm->i2c, reg);
}

int atm88pa_write(struct atm88pa *atm, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(atm->i2c, reg, val);
}

int atm88pa_write_word(struct atm88pa *atm, u8 reg, u16 val)
{
	return i2c_smbus_write_word_data(atm->i2c, reg, val);
}

int atm88pa_get_sw_ver(struct atm88pa *atm)
{
	return atm88pa_read_word(atm, ATM88PA_REG_SW_VER);
}

void atm88pa_update_status(struct atm88pa *atm)
{
	int ret;
	u8 new_status, diff_status;

	ret = atm88pa_read(atm, ATM88PA_REG_STATUS);
	if (ret < 0) {
		dev_warn(atm->dev, "read status failed, err: %d\n", ret);
	} else {
		new_status = ret;
		diff_status = atm->old_status ^ new_status;
		atm->old_status = new_status;

		dev_dbg(atm->dev, "status 0x%02x, diff 0x%02x\n", new_status, diff_status);

		/* Check power flag */
		if (diff_status & 0x04) {
			if (new_status & 0x04) {
				/* Simulate KEY_BATTERY released */
				atm88pa_keypad_simulate_key(atm, KEY_BATTERY, 0);
			} else {
				/* Simulate KEY_BATTERY pressed */
				atm88pa_keypad_simulate_key(atm, KEY_BATTERY, 1);
			}
		}

		/* Check hall flags */
		if (diff_status & 0x18) {
			/* If cover closed, send KEY_SLEEP event */
			if ((new_status & 0x18) == 0x18) {
				/* Simulate KEY_SLEEP pressed and released */
				atm88pa_keypad_simulate_key(atm, KEY_SLEEP, 1);
				atm88pa_keypad_simulate_key(atm, KEY_SLEEP, 0);
			} else if ((new_status & 0x18) == 0) {
				/* Simulate KEY_WAKEUP pressed and released */
				atm88pa_keypad_simulate_key(atm, KEY_WAKEUP, 1);
				atm88pa_keypad_simulate_key(atm, KEY_WAKEUP, 0);
			}
		}

		if (diff_status) {
			/* Simulate KEY_SYSRQ pressed and released */
			atm88pa_keypad_simulate_key(atm, KEY_SYSRQ, 1);
			atm88pa_keypad_simulate_key(atm, KEY_SYSRQ, 0);
		}
	}
}

#ifdef CONFIG_OF
static int atm88pa_parse_dt(struct atm88pa *atm)
{
	int i, ret;
	struct device_node *np = atm->dev->of_node;
	struct device_node *keypad = of_get_child_by_name(np, "keypad");
	struct device_node *leds = of_get_child_by_name(np, "leds");
	struct device_node *power = of_get_child_by_name(np, "power");
	struct device_node *child = NULL;
	struct led_classdev *led;
	enum of_gpio_flags flags;

	/* Properties of keypad/... */
	ret = of_property_read_u32(keypad, "debounce_time", &atm->keypad.debounce_time);
	if (ret) {
		dev_err(atm->dev, "debounce_time missing or invalid\n");
		return ret;
	}
	ret = of_property_read_u32(keypad, "repeat", &atm->keypad.repeat);
	if (ret) {
		dev_err(atm->dev, "repeat missing or invalid\n");
		return ret;
	}
	ret = of_property_read_u16_array(keypad, "keymap", atm->keypad.keymap,
	                                 ARRAY_SIZE(atm->keypad.keymap));
	if (ret) {
		dev_err(atm->dev, "keymap missing or invalid\n");
		return ret;
	}

	/* Properties of leds/... */
	for_each_child_of_node(leds, child) {
		ret = of_property_read_u32(child, "id", &i);
		if (ret || i < 0 || i > ARRAY_SIZE(atm->leds)) {
			dev_err(atm->dev, "wrong LED id number\n");
			return -EINVAL;
		}

		led = &atm->leds[i];

		ret = of_property_read_string(child, "label", &led->name);
		if (ret) {
			if (child->name == NULL) {
				dev_err(atm->dev, "label missing or invalid\n");
				return ret;
			}
			led->name = child->name;
		}
		ret = of_property_read_u32(child, "brightness", &led->brightness);
		if (ret)
			atm->leds[i].brightness = LED_OFF;

		ret = of_property_read_u32(child, "max-brightness", &led->max_brightness);
		if (ret)
			atm->leds[i].max_brightness = LED_FULL;

		of_property_read_string(child, "default-trigger", &led->default_trigger);
	}

	/* Properties of power/... */
	atm->power.gpio = of_get_gpio_flags(power, 0, &flags);
	atm->power.active_low = flags & OF_GPIO_ACTIVE_LOW;

	return 0;
}
#else
static inline int atm88pa_parse_dt(struct device *dev, struct atm88pa *atm)
{
	return -ENODEV;
}
#endif

static ssize_t atm88pa_get_amb_light_attr(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct atm88pa *atm = miscdev_to_atm88pa(dev);
	int light;

	/* The v1.00 does not supportted */
	if (atm->chip_ver == 100)
		return -ENXIO;

	light = atm88pa_read_word(atm, ATM88PA_REG_AMB_LIGHT);

	return scnprintf(buf, PAGE_SIZE, "%u\n", light);
}

static ssize_t atm88pa_get_cam_switch_attr(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct atm88pa *atm = miscdev_to_atm88pa(dev);
	int cam;

	cam = atm88pa_read(atm, ATM88PA_REG_CAM_SWITCH);

	return scnprintf(buf, PAGE_SIZE, "%u\n", cam);
}

static ssize_t atm88pa_set_cam_switch_attr(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct atm88pa *atm = miscdev_to_atm88pa(dev);
	int cam;

	if (sscanf(buf, "%u", &cam) == 1) {
		atm88pa_write(atm, ATM88PA_REG_CAM_SWITCH, (u8)cam);
		return count;
	}
	return -EINVAL;
}

static ssize_t atm88pa_get_cam_b_pwr_attr(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct atm88pa *atm = miscdev_to_atm88pa(dev);
	int ctrl;

	ctrl = atm88pa_read(atm, ATM88PA_REG_BC_PWR_CTRL);

	return scnprintf(buf, PAGE_SIZE, "%u\n", ctrl);
}

static ssize_t atm88pa_set_cam_b_pwr_attr(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct atm88pa *atm = miscdev_to_atm88pa(dev);
	int ctrl;

	if (sscanf(buf, "%u", &ctrl) == 1) {
		atm88pa_write(atm, ATM88PA_REG_BC_PWR_CTRL, (u8)ctrl);
		return count;
	}
	return -EINVAL;
}

static ssize_t atm88pa_get_cam_f_pwr_attr(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct atm88pa *atm = miscdev_to_atm88pa(dev);
	int ctrl;

	ctrl = atm88pa_read(atm, ATM88PA_REG_FC_PWR_CTRL);

	return scnprintf(buf, PAGE_SIZE, "%u\n", ctrl);
}

static ssize_t atm88pa_set_cam_f_pwr_attr(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct atm88pa *atm = miscdev_to_atm88pa(dev);
	int ctrl;

	if (sscanf(buf, "%u", &ctrl) == 1) {
		atm88pa_write(atm, ATM88PA_REG_FC_PWR_CTRL, (u8)ctrl);
		return count;
	}
	return -EINVAL;
}

static ssize_t atm88pa_get_cam_l_pwr_attr(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct atm88pa *atm = miscdev_to_atm88pa(dev);
	int ctrl;

	ctrl = atm88pa_read(atm, ATM88PA_REG_LC_PWR_CTRL);

	return scnprintf(buf, PAGE_SIZE, "%u\n", ctrl);
}

static ssize_t atm88pa_set_cam_l_pwr_attr(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct atm88pa *atm = miscdev_to_atm88pa(dev);
	int ctrl;

	if (sscanf(buf, "%u", &ctrl) == 1) {
		atm88pa_write(atm, ATM88PA_REG_LC_PWR_CTRL, (u8)ctrl);
		return count;
	}
	return -EINVAL;
}

static ssize_t atm88pa_get_cam_r_pwr_attr(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct atm88pa *atm = miscdev_to_atm88pa(dev);
	int ctrl;

	ctrl = atm88pa_read(atm, ATM88PA_REG_RC_PWR_CTRL);

	return scnprintf(buf, PAGE_SIZE, "%u\n", ctrl);
}

static ssize_t atm88pa_set_cam_r_pwr_attr(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct atm88pa *atm = miscdev_to_atm88pa(dev);
	int ctrl;

	if (sscanf(buf, "%u", &ctrl) == 1) {
		atm88pa_write(atm, ATM88PA_REG_RC_PWR_CTRL, (u8)ctrl);
		return count;
	}
	return -EINVAL;
}

static ssize_t atm88pa_get_keypad_light_attr(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct atm88pa *atm = miscdev_to_atm88pa(dev);
	int val;

	/* Only for v1.00 */
	if (atm->chip_ver != 100)
		return -ENXIO;

	val = atm88pa_read(atm, ATM88PA_REG_KEYPAD_LIGHT);

	return scnprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t atm88pa_set_keypad_light_attr(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct atm88pa *atm = miscdev_to_atm88pa(dev);
	int val;

	/* Only for v1.00 */
	if (atm->chip_ver != 100)
		return -ENXIO;

	if (sscanf(buf, "%u", &val) == 1) {
		atm88pa_write(atm, ATM88PA_REG_KEYPAD_LIGHT, (u8)val);
		return count;
	}

	return -EINVAL;
}

static ssize_t atm88pa_get_lcd_pwr_attr(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct atm88pa *atm = miscdev_to_atm88pa(dev);
	int ctrl;

	ctrl = atm88pa_read(atm, ATM88PA_REG_LCD_PWR_CTRL);

	return scnprintf(buf, PAGE_SIZE, "%u\n", ctrl);
}

static ssize_t atm88pa_set_lcd_pwr_attr(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct atm88pa *atm = miscdev_to_atm88pa(dev);
	int ctrl;

	if (sscanf(buf, "%u", &ctrl) == 1) {
		atm88pa_write(atm, ATM88PA_REG_LCD_PWR_CTRL, (u8)ctrl);
		return count;
	}
	return -EINVAL;
}


static ssize_t atm88pa_get_lcd_light_attr(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct atm88pa *atm = miscdev_to_atm88pa(dev);
	int cam;

	cam = atm88pa_read(atm, ATM88PA_REG_LCD_LIGHT);

	return scnprintf(buf, PAGE_SIZE, "%u\n", cam);
}

static ssize_t atm88pa_set_lcd_light_attr(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct atm88pa *atm = miscdev_to_atm88pa(dev);
	int cam;

	if (sscanf(buf, "%u", &cam) == 1) {
		atm88pa_write(atm, ATM88PA_REG_LCD_LIGHT, (u8)cam);
		return count;
	}
	return -EINVAL;
}

static ssize_t atm88pa_get_status_attr(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct atm88pa *atm = miscdev_to_atm88pa(dev);
	int status;

	status = atm88pa_read(atm, ATM88PA_REG_STATUS);

	return scnprintf(buf, PAGE_SIZE, "0x%02x\n", status);
}

static ssize_t atm88pa_get_sucap_volt_attr(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct atm88pa *atm = miscdev_to_atm88pa(dev);
	int volt;

	volt = atm88pa_read_word(atm, ATM88PA_REG_SUCAP_VOLT);

	return scnprintf(buf, PAGE_SIZE, "%u\n", volt);
}

static ssize_t atm88pa_get_sup_light_attr(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct atm88pa *atm = miscdev_to_atm88pa(dev);
	int val;

	val = atm88pa_read(atm, ATM88PA_REG_SUP_LIGHT);

	return scnprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t atm88pa_set_sup_light_attr(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct atm88pa *atm = miscdev_to_atm88pa(dev);
	int val;

	if (sscanf(buf, "%u", &val) == 1) {
		atm88pa_write(atm, ATM88PA_REG_SUP_LIGHT, (u8)val);
		return count;
	}
	return -EINVAL;
}

static ssize_t atm88pa_get_temperature_attr(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct atm88pa *atm = miscdev_to_atm88pa(dev);
	int temperature;

	temperature = atm88pa_read_word(atm, ATM88PA_REG_TEMPERATURE);

	return scnprintf(buf, PAGE_SIZE, "%d\n", temperature);
}

static ssize_t atm88pa_get_uart1_switch_attr(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
	struct atm88pa *atm = miscdev_to_atm88pa(dev);
	int val;

	/* Only for v1.00 */
	if (atm->chip_ver != 100)
		return -ENXIO;

	val = atm88pa_read(atm, ATM88PA_REG_UART1_SWITCH);

	return scnprintf(buf, PAGE_SIZE, "%u\n", val);
}

static ssize_t atm88pa_set_uart1_switch_attr(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct atm88pa *atm = miscdev_to_atm88pa(dev);
	int val;

	/* Only for v1.00 */
	if (atm->chip_ver != 100)
		return -ENXIO;

	if (sscanf(buf, "%u", &val) == 1) {
		atm88pa_write(atm, ATM88PA_REG_UART1_SWITCH, (u8)val);
		return count;
	}

	return -EINVAL;
}

static ssize_t atm88pa_get_version_attr(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct atm88pa *atm = miscdev_to_atm88pa(dev);
	int version;

	version = atm88pa_get_sw_ver(atm);

	return scnprintf(buf, PAGE_SIZE, "%d\n", version);
}


DEVICE_ATTR(amb_light, S_IRUGO, atm88pa_get_amb_light_attr, NULL);
DEVICE_ATTR(cam_switch, S_IRUGO | S_IWUSR, atm88pa_get_cam_switch_attr, atm88pa_set_cam_switch_attr);
DEVICE_ATTR(cam_b_pwr, S_IRUGO | S_IWUSR, atm88pa_get_cam_b_pwr_attr, atm88pa_set_cam_b_pwr_attr);
DEVICE_ATTR(cam_f_pwr, S_IRUGO | S_IWUSR, atm88pa_get_cam_f_pwr_attr, atm88pa_set_cam_f_pwr_attr);
DEVICE_ATTR(cam_l_pwr, S_IRUGO | S_IWUSR, atm88pa_get_cam_l_pwr_attr, atm88pa_set_cam_l_pwr_attr);
DEVICE_ATTR(cam_r_pwr, S_IRUGO | S_IWUSR, atm88pa_get_cam_r_pwr_attr, atm88pa_set_cam_r_pwr_attr);
DEVICE_ATTR(keypad_light, S_IRUGO | S_IWUSR, atm88pa_get_keypad_light_attr, atm88pa_set_keypad_light_attr);
DEVICE_ATTR(lcd_pwr, S_IRUGO | S_IWUSR, atm88pa_get_lcd_pwr_attr, atm88pa_set_lcd_pwr_attr);
DEVICE_ATTR(lcd_light, S_IRUGO | S_IWUSR, atm88pa_get_lcd_light_attr, atm88pa_set_lcd_light_attr);
DEVICE_ATTR(status, S_IRUGO, atm88pa_get_status_attr, NULL);
DEVICE_ATTR(sucap_volt, S_IRUGO, atm88pa_get_sucap_volt_attr, NULL);
DEVICE_ATTR(sup_light, S_IRUGO | S_IWUSR, atm88pa_get_sup_light_attr, atm88pa_set_sup_light_attr);
DEVICE_ATTR(temperature, S_IRUGO, atm88pa_get_temperature_attr, NULL);
DEVICE_ATTR(uart1_switch, S_IRUGO | S_IWUSR, atm88pa_get_uart1_switch_attr, atm88pa_set_uart1_switch_attr);
DEVICE_ATTR(version, S_IRUGO, atm88pa_get_version_attr, NULL);

static struct attribute *atm88pa_attrs[] = {
	&dev_attr_amb_light.attr,
	&dev_attr_cam_switch.attr,
	&dev_attr_cam_b_pwr.attr,
	&dev_attr_cam_f_pwr.attr,
	&dev_attr_cam_l_pwr.attr,
	&dev_attr_cam_r_pwr.attr,
	&dev_attr_keypad_light.attr,
	&dev_attr_lcd_pwr.attr,
	&dev_attr_lcd_light.attr,
	&dev_attr_status.attr,
	&dev_attr_sucap_volt.attr,
	&dev_attr_sup_light.attr,
	&dev_attr_temperature.attr,
	&dev_attr_uart1_switch.attr,
	&dev_attr_version.attr,
	NULL
};

static const struct attribute_group atm88pa_attr_group = {
	.attrs = atm88pa_attrs,
};

static const struct attribute_group *atm88pa_attr_groups[] = {
	&atm88pa_attr_group,
	NULL,
};

static int atm88pa_misc_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int atm88pa_misc_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long atm88pa_misc_ioctl(struct file *file, unsigned int cmd,
			       unsigned long arg)
{
	return 0;
}

static struct file_operations atm88pa_misc_fops = {
	.owner		= THIS_MODULE,
	.open		= atm88pa_misc_open,
	.release	= atm88pa_misc_release,
	.unlocked_ioctl	= atm88pa_misc_ioctl,
};

/*
static struct miscdevice atm88pa_misc = {
	.name	= "atm88pa",
	.minor	= MISC_DYNAMIC_MINOR,
	.fops	= &atm88pa_misc_fops,
};
*/

static int atm88pa_misc_register(struct atm88pa *atm)
{
	int ret;
	struct miscdevice *misc = &atm->misc;

	misc->name	= "atm88pa";
	misc->minor	= MISC_DYNAMIC_MINOR;
	misc->fops	= &atm88pa_misc_fops;
	misc->groups	= atm88pa_attr_groups;
	misc->parent	= atm->dev;

	ret = misc_register(misc);
	if (ret) {
		dev_err(atm->dev, "misc: %s register failed, err: %d\n",
			misc->name, ret);
		return ret;
	}

	dev_info(atm->dev, "misc: %s registered.\n", misc->name);

	return 0;
}

static int atm88pa_misc_unregister(struct atm88pa *atm)
{
	int ret;
	struct miscdevice *misc = &atm->misc;
	
	ret = misc_deregister(misc);
	if (ret) {
		dev_err(atm->dev, "misc: %s unregister failed, err: %d\n",
			misc->name, ret);
		return ret;
	}

	dev_info(atm->dev, "misc: %s unregistered.\n", misc->name);

	return 0;
}

static int atm88pa_probe(struct i2c_client* client,
			 const struct i2c_device_id* id)
{
	int err;
	struct atm88pa *atm;

	/* Allocate resource */
	atm = devm_kzalloc(&client->dev, sizeof(*atm), GFP_KERNEL);
	if (!atm)
		return -ENOMEM;

	i2c_set_clientdata(client, atm);

	atm->dev = &client->dev;
	atm->i2c = client;
	atm->chip_ver = atm88pa_get_sw_ver(atm);

	/*
	 * Check software version, current:
         *   5.01 for CVR-MIL-V2
	 *   6.00 for CVR-MIL-V2-A
	 */
	if (atm->chip_ver != 100 &&
	    atm->chip_ver != 501 &&
	    atm->chip_ver != 600) {
		dev_err(atm->dev, "ATMEGA88PA not found.\n");
		return -ENODEV;
	}

	err = atm88pa_parse_dt(atm);
	if (err) {
		dev_err(atm->dev, "DT not found\n");
		return err;
	}

	/* Clear interrupts */
	atm88pa_read(atm, ATM88PA_REG_INT_CTRL);

	/* Save current status */
	atm->old_status = atm88pa_read(atm, ATM88PA_REG_STATUS);

	/* Save current keys */
	atm->keypad.old_keys = atm88pa_read(atm, ATM88PA_REG_KEYS);

	err = atm88pa_misc_register(atm);
	if (err) {
		goto misc_error;
	}

	err = atm88pa_keypad_register(atm);
	if (err) {
		dev_err(atm->dev, "register keypad subsystem failed, err: %d\n", err);
		goto keypad_error;
	}

	err = atm88pa_leds_register(atm);
	if (err) {
		dev_err(atm->dev, "register leds subsystem failed, err: %d\n", err);
		goto leds_error;
	}

	err = atm88pa_power_register(atm);
	if (err) {
		dev_err(atm->dev, "register power subsystem failed, err: %d\n", err);
		goto power_error;
	}

	err = atm88pa_irq_init(atm);
	if (err) {
		dev_err(atm->dev, "initialize irq failed, err: %d\n", err);
		goto irq_error;
	}

	dev_info(atm->dev, "probed.\n");

	return 0;

irq_error:
	atm88pa_power_unregister(atm);
power_error:
	atm88pa_leds_unregister(atm);
leds_error:
	atm88pa_keypad_unregister(atm);
keypad_error:
	atm88pa_misc_unregister(atm);
misc_error:
	return err;
}

static int atm88pa_remove(struct i2c_client *client)
{
	struct atm88pa *atm = i2c_get_clientdata(client);

	atm88pa_irq_exit(atm);
	atm88pa_misc_unregister(atm);
	atm88pa_keypad_unregister(atm);
	atm88pa_leds_unregister(atm);
	atm88pa_power_unregister(atm);

	dev_info(&client->dev, "removed.\n");

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int atm88pa_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct atm88pa *atm = i2c_get_clientdata(client);

	return atm88pa_irq_suspend(atm);
}

static int atm88pa_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct atm88pa *atm = i2c_get_clientdata(client);

	return atm88pa_irq_resume(atm);
}
#endif

static SIMPLE_DEV_PM_OPS(atm88pa_pm_ops,
                         atm88pa_suspend,
                         atm88pa_resume);

static const struct i2c_device_id atm88pa_id[] = {
	{ "atm88pa", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, atm88pa_id);

#ifdef CONFIG_OF
static const struct of_device_id atm88pa_of_match[] = {
	{ .compatible = "tdc,atm88pa", },
	{ }
};
MODULE_DEVICE_TABLE(of, atm88pa_of_match);
#endif

static struct i2c_driver atm88pa_driver = {
	.driver = {
		.name           = "atm88pa",
		.owner          = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(atm88pa_of_match),
#endif
		.pm             = &atm88pa_pm_ops,
	},
	.id_table       = atm88pa_id,
	.probe          = atm88pa_probe,
	.remove         = atm88pa_remove,
};

module_i2c_driver(atm88pa_driver);

MODULE_AUTHOR("Varphone Wong <varphone@qq.com>");
MODULE_DESCRIPTION("ATMEGA88PA MFD driver");
MODULE_LICENSE("GPL");

