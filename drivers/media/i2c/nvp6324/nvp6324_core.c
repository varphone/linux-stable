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

int nvp6324_transfer_regs(struct nvp6324 *self, const struct reg_pack *reg_pack,
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
			nvp6324_set_reg_bank(self, bank);
			nvp6324_write_reg(self, addr, val);
		}
	}
	return 0;
}

static void nvp6324_hw_reset(struct nvp6324 *self)
{
	gpio_set_value(self->reset_gpio, 0);
	udelay(200);
	gpio_set_value(self->reset_gpio, 1);
	msleep(1);
}

static int nvp6324_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct nvp6324 *self;
	int ret;
	u8 rev;

	self = devm_kzalloc(dev, sizeof(*self), GFP_KERNEL);
	if (!self)
		return -ENOMEM;

	/* Setup the reset gpio */
	self->reset_gpio = of_get_named_gpio(dev->of_node, "reset-gpios", 0);
	if (!gpio_is_valid(self->reset_gpio)) {
		dev_err(dev, "no sensor reset pin available\n");
		return -ENODEV;
	}

	ret = devm_gpio_request_one(dev, self->reset_gpio, GPIOF_OUT_INIT_HIGH,
				    "nvp6324_rst");
	if (ret < 0) {
		dev_err(dev, "failed to request sensor reset gpio: %d\n", ret);
		return ret;
	}

	/* Setup required states */
	self->i2c_client = client;
	self->current_bank = 0xff;

	/* Reset the chip */
	nvp6324_hw_reset(self);

	/* Verify the chip id */
	nvp6324_set_reg_bank(self, 0);
	ret = nvp6324_read_reg(self, 0xf4);
	if (ret != 0xb0) {
		dev_warn(dev, "Chip Not Found!\n");
		return -ENODEV;
	}
	rev = nvp6324_read_reg(self, 0xf5);
	dev_info(dev, "Found, Chip Id: 0xb0, Rev: 0x%02x\n", rev);

	/* Initialize the lock */
	mutex_init(&self->lock);

	ret = nvp6324_video_init(self);
	if (ret < 0) {
		dev_err(dev, "failed to init video: %d\n", ret);
		goto err_nvp6324_video_init;
	}

	return 0;

err_nvp6324_video_init:
	mutex_destroy(&self->lock);
	return ret;
}

static int nvp6324_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct nvp6324 *self = subdev_to_sensor_data(sd);

	nvp6324_video_exit(self);
	mutex_destroy(&self->lock);

	return 0;
}

static const struct i2c_device_id nvp6324_id[] = {
	{},
};

MODULE_DEVICE_TABLE(i2c, nvp6324_id);

static const struct of_device_id nvp6324_of_match[] = {
	{ .compatible = "nextchip,nvp6324" },
	{ /* sentinel */ }
};

static struct i2c_driver nvp6324_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "nvp6324",
		.of_match_table = of_match_ptr(nvp6324_of_match),
	},
	.probe = nvp6324_probe,
	.remove = nvp6324_remove,
	.id_table = nvp6324_id,
};

module_i2c_driver(nvp6324_i2c_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_AUTHOR("CETC55, Technology Development CO.,LTD.");
MODULE_AUTHOR("Varphone Wong <varphone@qq.com>");
MODULE_DESCRIPTION("Nextchip NVP6324 AHD Video Decoder Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
