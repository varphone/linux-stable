#include <asm-generic/errno-base.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/v4l2-mediabus.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-subdev.h>

#include "nvp6324.h"

int nvp6324_transfer_regs(struct nvp6324_dev *nvp6324, const struct reg_pack *reg_pack,
			  u32 channel, u32 counts)
{
	u32 i, j;
	u32 ch = channel;
	u8 bank, addr, val, mask, offset;
	const struct reg_base *pbase = reg_pack->pbase;
	u8 *pval = reg_pack->pval;
	u32 size = reg_pack->size;

	if (counts == 0 || counts > 4 || (ch + counts) > 4 || size == 0)
		return -EINVAL;

	for (i = 0; i < counts; i++, ch++) {
		for (j = 0; j < size; j++) {
			bank = pbase[j].bank;
			addr = pbase[j].addr;
			mask = pbase[j].mask;
			offset = pbase[j].offset;
			val = pval[j];

			switch(bank) {
			case 0x5:
				bank += ch;
				break;
			case 0x9:
				if (addr >= 0x96 && addr <=0x9e)
					addr += ch << 5;
				else if (addr >= 0x50 && addr <= 0x53)
					addr += ch << 2;
				else
					addr += ch;
				break;
			case 0xa:
				if ((addr >= 0x30 && addr <=0x3c) ||
				    (addr >= 0x25 && addr <=0x27)) {
					bank += ch / 2;
					addr += ch % 2 * 0x80;
				} else {
					addr += ch;
				}
				break;
			default:
				addr += ch;
				break;
 			}

			val = (val & mask) << offset;
			nvp6324_set_reg_bank(nvp6324, bank);
			nvp6324_write_reg(nvp6324, addr, val);
		}
	}

	return 0;
}

static void nvp6324_hw_reset(struct nvp6324_dev *nvp6324)
{
	gpio_set_value(nvp6324->pwn_gpio, 0);
	udelay(200);
	gpio_set_value(nvp6324->pwn_gpio, 1);
	msleep(1);
}

static int nvp6324_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct nvp6324_dev *nvp6324;
	int ret;
	u8 rev;

	nvp6324 = devm_kzalloc(dev, sizeof(*nvp6324), GFP_KERNEL);
	if (!nvp6324)
		return -ENOMEM;

	nvp6324->sensor_clk = devm_clk_get(dev, "capture_mclk");
	if (IS_ERR(nvp6324->sensor_clk)) {
		nvp6324->sensor_clk = NULL;
		dev_err(dev, "clock-frequency missing or invalid\n");
		return PTR_ERR(nvp6324->sensor_clk);
	}

	ret = of_property_read_u32(dev->of_node, "mclk",
				   &(nvp6324->mclk));
	if (ret) {
		dev_err(dev, "mclk missing or invalid\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "mclk_source",
				   (u32*)&(nvp6324->mclk_source));
	if (ret) {
		dev_err(dev, "mclk_source missing or invalid\n");
		return ret;
	}

	nvp6324->pwn_gpio = of_get_named_gpio(dev->of_node, "pwn-gpios", 0);
	if (!gpio_is_valid(nvp6324->pwn_gpio)) {
		dev_err(dev, "no sensor pwdn pin available\n");
		return -ENODEV;
	}
	ret = devm_gpio_request_one(dev, nvp6324->pwn_gpio, GPIOF_OUT_INIT_HIGH,
					"nvp6324_pwd");
	if (ret < 0)
		return ret;

	nvp6324_hw_reset(nvp6324);

	clk_prepare_enable(nvp6324->sensor_clk);

	mutex_init(&nvp6324->lock);

	nvp6324->i2c_client = client;

	nvp6324->current_bank = 0xff;
	nvp6324_set_reg_bank(nvp6324, 0);
	ret = nvp6324_read_reg(nvp6324, 0xf4);
	if (ret != 0xb0) {
		dev_warn(dev, "nvp6324 not found, chip id reg 0xf4=0x%x.\n", ret);
		clk_disable_unprepare(nvp6324->sensor_clk);
		return -ENODEV;
	}

	rev = nvp6324_read_reg(nvp6324, 0xf5);

	ret = nvp6324_video_init(nvp6324);
	if (ret < 0) {
		dev_err(dev, "failed to init video: %d\n", ret);
		return ret;
	}

	dev_info(dev, "nvp6324 found, address: 0x%02x, chip id: 0xb0, "
		      "rev: 0x%02x\n", client->addr, rev);

	return 0;
}

static int nvp6324_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct nvp6324_dev *nvp6324 = subdev_to_sensor_data(sd);

	mutex_destroy(&nvp6324->lock);
	clk_disable_unprepare(nvp6324->sensor_clk);
	media_entity_cleanup(&sd->entity);
	v4l2_async_unregister_subdev(sd);

	return 0;
}

static const struct i2c_device_id nvp6324_id[] = {
	{},
};

MODULE_DEVICE_TABLE(i2c, nvp6324_id);

static const struct of_device_id nvp6324_of_match[] = {
	{ .compatible = "nextchip,nvp6324_mipi" },
	{ /* sentinel */ }
};

static struct i2c_driver nvp6324_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "nvp6324_mipi",
		.of_match_table = of_match_ptr(nvp6324_of_match),
	},
	.probe = nvp6324_probe,
	.remove = nvp6324_remove,
	.id_table = nvp6324_id,
};

module_i2c_driver(nvp6324_i2c_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("NVP6324 Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
