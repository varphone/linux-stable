/*
 * driver/mfd/ricoh568.c
 *
 * Core driver implementation to access RICOH RC5T568 power management chip.
 *
 * Copyright (C) 2012-2016 RICOH COMPANY,LTD
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
/*#define DEBUG			1*/
/*#define VERBOSE_DEBUG		1*/
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/mfd/ricoh568.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regmap.h>
#include <linux/delay.h>

struct ricoh568 *g_ricoh568;
struct sleep_control_data {
	u8 reg_add;
};
static struct mfd_cell ricoh568_dev[] = {
	{
		.name = "ricoh568-regulator",
	},
	{
		.name = "ricoh568-pwrkey",
	},
};


#define SLEEP_INIT(_id, _reg)		\
	[RICOH568_DS_##_id] = {.reg_add = _reg}
/*
static struct sleep_control_data sleep_data[] = {
	SLEEP_INIT(DC1, 0x16),
	SLEEP_INIT(DC2, 0x17),
	SLEEP_INIT(DC3, 0x18),
	SLEEP_INIT(DC4, 0x19),
	SLEEP_INIT(LDO1, 0x1B),
	SLEEP_INIT(LDO2, 0x1C),
	SLEEP_INIT(LDO3, 0x1D),
	SLEEP_INIT(LDO4, 0x1E),
	SLEEP_INIT(LDO5, 0x1F),
};
*/
static inline int __ricoh568_read(struct i2c_client *client,
				  u8 reg, uint8_t *val)
{
	int ret =0;
	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "failed reading at 0x%02x %d\n", reg,ret);
		return ret;
	}

	*val = (uint8_t)ret;
	dev_dbg(&client->dev, "ricoh568: reg read  reg=%x, val=%x\n",
				reg, *val);
	return 0;
}

static inline int __ricoh568_bulk_reads(struct i2c_client *client, u8 reg,
				int len, uint8_t *val)
{
	int ret;
	int i;

	ret = i2c_smbus_read_i2c_block_data(client, reg, len, val);
	if (ret < 0) {
		dev_err(&client->dev, "failed reading from 0x%02x %dn", reg,ret);
		return ret;
	}
	for (i = 0; i < len; ++i) {
		dev_dbg(&client->dev, "ricoh568: reg read  reg=%x, val=%x\n",
				reg + i, *(val + i));
	}
	return 0;
}

static inline int __ricoh568_write(struct i2c_client *client, u8 reg, uint8_t val)
{
	int ret=0;

	dev_dbg(&client->dev, "ricoh568: reg write  reg=%x, val=%x\n", reg, val);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		dev_err(&client->dev, "failed writing 0x%02x to 0x%02x\n", val, reg);
		return ret;
	}
	return 0;
}

static inline int __ricoh568_bulk_writes(struct i2c_client *client, u8 reg,
				  int len, uint8_t *val)
{
	int ret=0;
	int i;

	for (i = 0; i < len; ++i) {
		dev_dbg(&client->dev, "ricoh568: reg write  reg=%x, val=%x\n",
				reg + i, *(val + i));
	}

	ret = i2c_smbus_write_i2c_block_data(client, reg, len, val);
	if (ret < 0) {
		dev_err(&client->dev, "failed writings to 0x%02x\n", reg);
		return ret;
	}
	return 0;
}

int ricoh568_write(struct device *dev, u8 reg, uint8_t val)
{
	struct ricoh568 *ricoh568 = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&ricoh568->io_lock);
	if( !ret )
		ret = __ricoh568_write(to_i2c_client(dev), reg, val);
	mutex_unlock(&ricoh568->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(ricoh568_write);

int ricoh568_bulk_writes(struct device *dev, u8 reg, u8 len, uint8_t *val)
{
	struct ricoh568 *ricoh568 = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&ricoh568->io_lock);
	if( !ret )
		ret = __ricoh568_bulk_writes(to_i2c_client(dev), reg, len, val);
	mutex_unlock(&ricoh568->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(ricoh568_bulk_writes);

int ricoh568_read(struct device *dev, u8 reg, uint8_t *val)
{
	struct ricoh568 *ricoh568 = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&ricoh568->io_lock);
	if( !ret )
		ret = __ricoh568_read(to_i2c_client(dev), reg, val);
	mutex_unlock(&ricoh568->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(ricoh568_read);

int ricoh568_bulk_reads(struct device *dev, u8 reg, u8 len, uint8_t *val)
{
	struct ricoh568 *ricoh568 = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&ricoh568->io_lock);
	if( !ret )
		ret = __ricoh568_bulk_reads(to_i2c_client(dev), reg, len, val);
	mutex_unlock(&ricoh568->io_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(ricoh568_bulk_reads);

int ricoh568_set_bits(struct device *dev, u8 reg, uint8_t bit_mask)
{
	struct ricoh568 *ricoh568 = dev_get_drvdata(dev);
	uint8_t reg_val;
	int ret = 0;

	mutex_lock(&ricoh568->io_lock);
	if (!ret) {
		ret = __ricoh568_read(to_i2c_client(dev), reg, &reg_val);
		if (ret<0)
			goto out;

		if ((reg_val & bit_mask) != bit_mask) {
			reg_val |= bit_mask;
			ret = __ricoh568_write(to_i2c_client(dev), reg,
								 reg_val);
		}
	}
out:
	mutex_unlock(&ricoh568->io_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(ricoh568_set_bits);

int ricoh568_clr_bits(struct device *dev, u8 reg, uint8_t bit_mask)
{
	struct ricoh568 *ricoh568 = dev_get_drvdata(dev);
	uint8_t reg_val;
	int ret = 0;

	mutex_lock(&ricoh568->io_lock);
	if( !ret ){
		ret = __ricoh568_read(to_i2c_client(dev), reg, &reg_val);
		if (ret<0)
			goto out;

		if (reg_val & bit_mask) {
			reg_val &= ~bit_mask;
			ret = __ricoh568_write(to_i2c_client(dev), reg,
								 reg_val);
		}
	}
out:
	mutex_unlock(&ricoh568->io_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(ricoh568_clr_bits);

int ricoh568_update(struct device *dev, u8 reg, uint8_t val, uint8_t mask)
{
	struct ricoh568 *ricoh568 = dev_get_drvdata(dev);
	uint8_t reg_val;
	int ret = 0;

	mutex_lock(&ricoh568->io_lock);
	if( !ret ){
		ret = __ricoh568_read(ricoh568->client, reg, &reg_val);
		if (ret<0)
			goto out;

		if ((reg_val & mask) != val) {
			reg_val = (reg_val & ~mask) | (val & mask);
			ret = __ricoh568_write(ricoh568->client, reg, reg_val);
		}
	}
out:
	mutex_unlock(&ricoh568->io_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(ricoh568_update);


static struct i2c_client *ricoh568_i2c_client;
static void ricoh568_device_shutdown(struct i2c_client *client)
{
	int ret;

	struct ricoh568 *ricoh568 = g_ricoh568;
	printk("%s,line=%d\n", __func__,__LINE__);

    ret = ricoh568_write(ricoh568->dev, RICOH568_INTC_INTEN, 0);
	ret = ricoh568_clr_bits(ricoh568->dev,RICOH568_PWR_REP_CNT,(0x1<<0));//Not repeat power ON after power off
	mutex_lock(&ricoh568->io_lock);
	mdelay(100);
}
EXPORT_SYMBOL_GPL(ricoh568_device_shutdown);

static void ricoh568_power_off(void)
{
	int ret;
	uint8_t val = 0;

	struct i2c_client *client = ricoh568_i2c_client;

	ret = __ricoh568_read(client, RICOH568_PWR_SLP_CNT, &val);//Power OFF
	if(ret < 0){
		printk("ricoh568 power off error!\n");
	}else{
		val = val&0xFE;
	}

	ret = __ricoh568_write(client, RICOH568_PWR_SLP_CNT,val);//Power OFF
	if (ret < 0) {
		printk("ricoh568 power off error!\n");
	}

	while(1)wfi();
}
EXPORT_SYMBOL_GPL(ricoh568_power_off);

#if 0
static int ricoh568_gpio_get(struct gpio_chip *gc, unsigned offset)
{
	struct ricoh568 *ricoh568 = container_of(gc, struct ricoh568, gpio_chip);
	uint8_t val;
	int ret;

	ret = ricoh568_read(ricoh568->dev, RICOH568_GPIO_MON_IOIN, &val);
	if (ret < 0)
		return ret;

	return ((val & (0x1 << offset)) != 0);
}

static void ricoh568_gpio_set(struct gpio_chip *gc, unsigned offset,
			int value)
{
	struct ricoh568 *ricoh568 = container_of(gc, struct ricoh568, gpio_chip);
	if (value)
		ricoh568_set_bits(ricoh568->dev, RICOH568_GPIO_IOOUT,
						1 << offset);
	else
		ricoh568_clr_bits(ricoh568->dev, RICOH568_GPIO_IOOUT,
						1 << offset);
}

static int ricoh568_gpio_input(struct gpio_chip *gc, unsigned offset)
{
	struct ricoh568 *ricoh568 = container_of(gc, struct ricoh568, gpio_chip);

	return ricoh568_clr_bits(ricoh568->dev, RICOH568_GPIO_IOSEL,
						1 << offset);
}

static int ricoh568_gpio_output(struct gpio_chip *gc, unsigned offset,
				int value)
{
	struct ricoh568 *ricoh568 = container_of(gc, struct ricoh568, gpio_chip);

	ricoh568_gpio_set(gc, offset, value);
	return ricoh568_set_bits(ricoh568->dev, RICOH568_GPIO_IOSEL,
						1 << offset);
}

static int ricoh568_gpio_to_irq(struct gpio_chip *gc, unsigned off)
{
	struct ricoh568 *ricoh568 = container_of(gc, struct ricoh568, gpio_chip);

	if ((off >= 0) && (off < 8))
		return ricoh568->irq_base + RICOH568_IRQ_GPIO0 + off;

	return -EIO;
}

static void ricoh568_gpio_init(struct ricoh568 *ricoh568,
	struct ricoh568_platform_data *pdata)
{
	int ret;
	int i;
	struct ricoh568_gpio_init_data *ginit;

	if (pdata->gpio_base  <= 0)
		return;

	for (i = 0; i < pdata->num_gpioinit_data; ++i) {
		ginit = &pdata->gpio_init_data[i];

		if (!ginit->init_apply)
			continue;

		if (ginit->output_mode_en) {
			/* GPIO output mode */
			if (ginit->output_val)
				/* output H */
				ret = ricoh568_set_bits(ricoh568->dev,
					RICOH568_GPIO_IOOUT, 1 << i);
			else
				/* output L */
				ret = ricoh568_clr_bits(ricoh568->dev,
					RICOH568_GPIO_IOOUT, 1 << i);
			if (!ret)
				ret = ricoh568_set_bits(ricoh568->dev,
					RICOH568_GPIO_IOSEL, 1 << i);
		} else
			/* GPIO input mode */
			ret = ricoh568_clr_bits(ricoh568->dev,
					RICOH568_GPIO_IOSEL, 1 << i);

		/* if LED function enabled in OTP */
		if (ginit->led_mode) {
			/* LED Mode 1 */
			if (i == 0)	/* GP0 */
				ret = ricoh568_set_bits(ricoh568->dev,
					 RICOH568_GPIO_LED_FUNC,
					 0x04 | (ginit->led_func & 0x03));
			if (i == 1)	/* GP1 */
				ret = ricoh568_set_bits(ricoh568->dev,
					 RICOH568_GPIO_LED_FUNC,
					 0x40 | (ginit->led_func & 0x03) << 4);

		}


		if (ret < 0)
			dev_err(ricoh568->dev, "Gpio %d init "
				"dir configuration failed: %d\n", i, ret);

	}

	ricoh568->gpio_chip.owner		= THIS_MODULE;
	ricoh568->gpio_chip.label		= ricoh568->client->name;
	ricoh568->gpio_chip.dev			= ricoh568->dev;
	ricoh568->gpio_chip.base		= pdata->gpio_base;
	ricoh568->gpio_chip.ngpio		= RICOH568_NR_GPIO;
	ricoh568->gpio_chip.can_sleep	= 1;

	ricoh568->gpio_chip.direction_input	= ricoh568_gpio_input;
	ricoh568->gpio_chip.direction_output	= ricoh568_gpio_output;
	ricoh568->gpio_chip.set			= ricoh568_gpio_set;
	ricoh568->gpio_chip.get			= ricoh568_gpio_get;
	ricoh568->gpio_chip.to_irq	  	= ricoh568_gpio_to_irq;

	ret = gpiochip_add(&ricoh568->gpio_chip);
	if (ret)
		dev_warn(ricoh568->dev, "GPIO registration failed: %d\n", ret);
}
#endif
static int ricoh568_remove_subdev(struct device *dev, void *unused)
{
	platform_device_unregister(to_platform_device(dev));
	return 0;
}

static int ricoh568_remove_subdevs(struct ricoh568 *ricoh568)
{
	return device_for_each_child(ricoh568->dev, NULL,
				     ricoh568_remove_subdev);
}
#if 0
static int ricoh568_add_subdevs(struct ricoh568 *ricoh568,
				struct ricoh568_platform_data *pdata)
{
	struct ricoh568_subdev_info *subdev;
	struct platform_device *pdev;
	int i, ret = 0;

	for (i = 0; i < pdata->num_subdevs; i++) {
		subdev = &pdata->subdevs[i];

		pdev = platform_device_alloc(subdev->name, subdev->id);

		pdev->dev.parent = ricoh568->dev;
		pdev->dev.platform_data = subdev->platform_data;

		ret = platform_device_add(pdev);
		if (ret)
			goto failed;
	}
	return 0;

failed:
	ricoh568_remove_subdevs(ricoh568);
	return ret;
}
#endif
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>
static void print_regs(const char *header, struct seq_file *s,
		struct i2c_client *client, int start_offset,
		int end_offset)
{
	uint8_t reg_val;
	int i;
	int ret;

	seq_printf(s, "%s\n", header);
	for (i = start_offset; i <= end_offset; ++i) {
		ret = __ricoh568_read(client, i, &reg_val);
		if (ret >= 0)
			seq_printf(s, "Reg 0x%02x Value 0x%02x\n", i, reg_val);
	}
	seq_printf(s, "------------------\n");
}

static int dbg_ricoh_show(struct seq_file *s, void *unused)
{
	struct ricoh568 *ricoh = s->private;
	struct i2c_client *client = ricoh->client;

	seq_printf(s, "RICOH568 Registers\n");
	seq_printf(s, "------------------\n");

	print_regs("System Regs",		s, client, 0x0, 0x05);
	print_regs("Power Control Regs",s, client, 0x07, 0x2B);
	print_regs("DCDC  Regs",		s, client, 0x2C, 0x43);
	print_regs("LDO   Regs",		s, client, 0x44, 0x61);
	print_regs("GPIO  Regs",		s, client, 0x90, 0x98);
	print_regs("INTC  Regs",		s, client, 0x9C, 0x9E);
	return 0;
}

static int dbg_ricoh_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_ricoh_show, inode->i_private);
}

static const struct file_operations debug_fops = {
	.open		= dbg_ricoh_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
static void __init ricoh568_debuginit(struct ricoh568 *ricoh)
{
	(void)debugfs_create_file("ricoh568", S_IRUGO, NULL,
			ricoh, &debug_fops);
}
#else
static void print_regs(const char *header, struct i2c_client *client,
		int start_offset, int end_offset)
{
	uint8_t reg_val;
	int i;
	int ret;

	printk(KERN_INFO "%s\n", header);
	for (i = start_offset; i <= end_offset; ++i) {
		ret = __ricoh568_read(client, i, &reg_val);
		if (ret >= 0)
			printk(KERN_INFO "Reg 0x%02x Value 0x%02x\n",
							 i, reg_val);
	}
	printk(KERN_INFO "------------------\n");
}
static void __init ricoh568_debuginit(struct ricoh568 *ricoh)
{
	struct i2c_client *client = ricoh->client;

	printk(KERN_INFO "RICOH568 Registers\n");
	printk(KERN_INFO "------------------\n");

	print_regs("System Regs",		client, 0x0, 0x05);
	print_regs("Power Control Regs",	client, 0x07, 0x2B);
	print_regs("DCDC  Regs",		client, 0x2C, 0x43);
	print_regs("LDO   Regs",		client, 0x44, 0x5C);
	print_regs("GPIO  Regs",		client, 0x90, 0x9B);
	print_regs("INTC  Regs",		client, 0x9C, 0x9E);

	return 0;
}
#endif

#ifdef CONFIG_OF
static struct ricoh568_platform_data *ricoh568_parse_dt(struct ricoh568 *ricoh568)
{
	struct ricoh568_platform_data *pdata;
	struct device_node *ricoh568_pmic_np;

	ricoh568_pmic_np = of_node_get(ricoh568->dev->of_node);
	if (!ricoh568_pmic_np) {
		printk("could not find pmic sub-node\n");
		return NULL;
	}
	pdata = devm_kzalloc(ricoh568->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	pdata->irq_gpio = of_get_named_gpio(ricoh568_pmic_np,"gpios",0);
	printk("%s, %d, %d, %d\n", __func__, __LINE__, of_get_named_gpio(ricoh568_pmic_np,"gpios",0), pdata->irq_gpio);
		if (!gpio_is_valid(pdata->irq_gpio)) {
			printk("invalid gpio: %d\n",  pdata->irq_gpio);
			return NULL;
		}

#if 1
	pdata->pmic_sleep_gpio = of_get_named_gpio(ricoh568_pmic_np,"gpios",1);
			if (!gpio_is_valid(pdata->pmic_sleep_gpio)) {
				printk("invalid gpio: %d\n",  pdata->pmic_sleep_gpio);
		}

	pdata->pmic_sleep = true;

	pdata->pm_off = of_property_read_bool(ricoh568_pmic_np,"ricoh568,system-power-controller");
#endif
	return pdata;
}

#else
static struct ricoh568_platform_data *ricoh568_parse_dt(struct ricoh568 *ricoh568)
{
	return NULL;
}
#endif

static int ricoh568_i2c_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct ricoh568 *ricoh568;
	struct ricoh568_platform_data *pdata = dev_get_platdata(&client->dev);;
	int ret=0;
	printk("%s,line=%d\n", __func__,__LINE__);

	ricoh568 = devm_kzalloc(&client->dev,sizeof(struct ricoh568), GFP_KERNEL);
	if (ricoh568 == NULL)
		return -ENOMEM;

	ricoh568->client = client;
	ricoh568->dev = &client->dev;
	i2c_set_clientdata(client, ricoh568);
	mutex_init(&ricoh568->io_lock);

	ricoh568->bank_num = 0;

	if (ricoh568->dev->of_node){
		printk("%s,line=%d\n", __func__,__LINE__);
		pdata = ricoh568_parse_dt(ricoh568);
		printk("%s,line=%d, %d\n", __func__,__LINE__, pdata->irq_gpio);
	}
	else{
		goto err;
	}

	ret = ricoh568_irq_init(ricoh568, pdata->irq_gpio, pdata);
	printk("%s,line=%d, %d, %d\n", __func__,__LINE__, pdata->irq_gpio, &pdata->irq_gpio);
	/******************************set sleep vol & dcdc mode******************/
	#if 1
	//#ifdef CONFIG_OF
	if (pdata->pmic_sleep_gpio) {
			ret = gpio_request(pdata->pmic_sleep_gpio, "ricoh568_pmic_sleep");
			if (ret < 0) {
				dev_err(ricoh568->dev,"Failed to request gpio %d with ret:""%d\n",	pdata->pmic_sleep_gpio, ret);
				return IRQ_NONE;
			}
			gpio_direction_output(pdata->pmic_sleep_gpio,0);
			ret = gpio_get_value(pdata->pmic_sleep_gpio);
			gpio_free(pdata->pmic_sleep_gpio);
			pr_info("%s: ricoh568_pmic_sleep=%x\n", __func__, ret);
	}
	#endif
	/**********************************************************/

#if 1
	ret = mfd_add_devices(ricoh568->dev, -1,
			     ricoh568_dev, ARRAY_SIZE(ricoh568_dev),
			      NULL, 0,NULL);
	g_ricoh568 = ricoh568;
	if (pdata->pm_off && !pm_power_off) {
		pm_power_off = ricoh568_power_off;
	}
	ricoh568_debuginit(ricoh568);
#endif
	ricoh568_i2c_client = client;
	return 0;
err:
	mfd_remove_devices(ricoh568->dev);
	return ret;
}

static int ricoh568_i2c_remove(struct i2c_client *client)
{
	struct ricoh568 *ricoh568 = i2c_get_clientdata(client);
	ricoh568_remove_subdevs(ricoh568);
	return 0;
}

#ifdef CONFIG_PM
extern u8 ricoh568_pwr_key_reg;
int ricoh568_pwrkey_wakeup = 0;
static int ricoh568_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
//	printk("PMU: %s: \n",__func__);

	if (g_ricoh568->chip_irq)
		disable_irq(g_ricoh568->chip_irq);
	ricoh568_pwrkey_wakeup = 1;
	__ricoh568_write(client, RICOH568_INT_IR_SYS, 0x0); //Clear PWR_KEY IRQ
	 __ricoh568_read(client, RICOH568_INT_IR_SYS, &ricoh568_pwr_key_reg);
	return 0;
}
static int ricoh568_i2c_resume(struct i2c_client *client)
{
	/*
	uint8_t reg_val;
	int ret;
	ret = __ricoh568_read(client, RICOH568_INT_IR_SYS, &reg_val);
	if(ricoh568_pwr_key_reg & 0x01) { //If PWR_KEY wakeup
		//printk("PMU: %s: PWR_KEY Wakeup %08x\n",__func__,ricoh568_pwr_key_reg);
		rcoh568_pwrkey_wakeup = 1;
		__ricoh568_write(client, RICOH568_INT_IR_SYS, 0x0); //Clear PWR_KEY IRQ
	}
	*/

	if (g_ricoh568->chip_irq)
		enable_irq(g_ricoh568->chip_irq);
	return 0;
}

static int  ricoh568_i2c_late_suspend(struct device *dev)
{
	struct i2c_client *client = i2c_verify_client(dev);

        ricoh568_i2c_suspend(client,PMSG_SUSPEND);
	return 0;
}

static int rockchip_i2c_late_resume(struct device *dev)
{
	struct i2c_client *client = i2c_verify_client(dev);

        ricoh568_i2c_resume(client);
	return 0;
}

static const struct dev_pm_ops ricoh568_i2c_dev_pm= {
	.suspend_late = ricoh568_i2c_late_suspend,
	.resume_early = rockchip_i2c_late_resume,
};

#endif

static const struct i2c_device_id ricoh568_i2c_id[] = {
	{"ricoh568", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, ricoh568_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id ricoh568_dt_match[] = {
	{ .compatible = "ricoh, ricoh568", },
	{},
};
MODULE_DEVICE_TABLE(of, ricoh568_dt_match);
#endif

static struct i2c_driver ricoh568_i2c_driver = {
	.driver = {
		   .name = "ricoh568",
		   .owner = THIS_MODULE,
                  #ifdef CONFIG_PM
		    .pm	= (&ricoh568_i2c_dev_pm),
                  #endif
		   .of_match_table = of_match_ptr(ricoh568_dt_match),
		   },
	.probe = ricoh568_i2c_probe,
	.remove = ricoh568_i2c_remove,
	.shutdown = ricoh568_device_shutdown,

	.id_table = ricoh568_i2c_id,
};


static int __init ricoh568_i2c_init(void)
{
	int ret = -ENODEV;

	pr_err("ricoh568_i2c_init\n");
	ret = i2c_add_driver(&ricoh568_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register I2C driver: %d\n", ret);

	return ret;
}

subsys_initcall_sync(ricoh568_i2c_init);

static void __exit ricoh568_i2c_exit(void)
{
	i2c_del_driver(&ricoh568_i2c_driver);
}

module_exit(ricoh568_i2c_exit);

MODULE_DESCRIPTION("RICOH RC5T568 PMU multi-function core driver");
MODULE_AUTHOR("Leozhang <leozhang@reds.ricoh.com>");
MODULE_LICENSE("GPL");
