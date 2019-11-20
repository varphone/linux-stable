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
	struct media_device* mdev;
	struct v4l2_device* vdev;
	struct nvp6324 *nvp6324;
	int ret;
	u8 rev;

	nvp6324 = devm_kzalloc(dev, sizeof(*nvp6324), GFP_KERNEL);
	if (!nvp6324)
		return -ENOMEM;

	/* Setup the reset gpio */
	nvp6324->reset_gpio = of_get_named_gpio(dev->of_node, "reset-gpios", 0);
	if (!gpio_is_valid(nvp6324->reset_gpio)) {
		dev_err(dev, "no sensor reset pin available\n");
		return -ENODEV;
	}

	ret = devm_gpio_request_one(dev, nvp6324->reset_gpio, GPIOF_OUT_INIT_HIGH,
					"nvp6324_rst");
	if (ret < 0) {
		dev_err(dev, "failed to request sensor reset gpio: %d\n", ret);
		return ret;
	}

	/* Setup required states */
	nvp6324->i2c_client = client;
	nvp6324->current_bank = 0xff;

	/* Reset the chip */
	nvp6324_hw_reset(nvp6324);

	/* Verify the chip id */
	nvp6324_set_reg_bank(nvp6324, 0);
	ret = nvp6324_read_reg(nvp6324, 0xf4);
	if (ret != 0xb0) {
		dev_warn(dev, "nvp6324 not found, chip id reg 0xf4=0x%x.\n", ret);
		return -ENODEV;
	}
	rev = nvp6324_read_reg(nvp6324, 0xf5);
	dev_info(dev, "found, address: 0x%02x, chip id: 0xb0, rev: 0x%02x\n",
		 client->addr, rev);

	/* Initialize the lock */
	mutex_init(&nvp6324->lock);

	/* Initialize the media controller */
	mdev = &nvp6324->mdev;
	memset(mdev, 0, sizeof(*mdev));
	snprintf(mdev->model, sizeof(mdev->model), "NVP6324");
	strlcpy(mdev->bus_info, "i2c", sizeof(mdev->bus_info));
	mdev->hw_revision = KERNEL_VERSION(1, 0, 0);
	mdev->driver_version = KERNEL_VERSION(1, 0, 0);
	mdev->dev = dev;
	media_device_init(mdev);

	/* Register the v4l2 device */
	vdev = &nvp6324->v4l2_dev;
	memset(vdev, 0, sizeof(*vdev));
	vdev->mdev = mdev;
	ret = v4l2_device_register(dev, vdev);
	if (ret < 0) {
		dev_err(dev, "failed to register v4l2 device: %d\n", ret);
		goto err_v4l2_device_register;
	}

	ret = media_device_register(mdev);
	if (ret < 0) {
		dev_err(dev, "failed to register media controller: %d\n", ret);
		goto err_media_device_register;
	}

	ret = nvp6324_video_init(nvp6324);
	if (ret < 0) {
		dev_err(dev, "failed to init video: %d\n", ret);
		goto err_nvp6324_video_init;
	}

	return 0;

err_nvp6324_video_init:
err_media_device_register:
	v4l2_device_unregister(vdev);
err_v4l2_device_register:
	media_device_cleanup(mdev);
	mutex_destroy(&nvp6324->lock);
	return ret;
}

static int nvp6324_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct nvp6324 *nvp6324 = subdev_to_sensor_data(sd);

	nvp6324_video_exit(nvp6324);
	v4l2_device_unregister(&nvp6324->v4l2_dev);
	media_device_unregister(&nvp6324->mdev);
	media_device_cleanup(&nvp6324->mdev);
	mutex_destroy(&nvp6324->lock);

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

MODULE_AUTHOR("Freescale Semiconductor, Inc. and Varphone Wong <varphone@qq.com>");
MODULE_DESCRIPTION("NVP6324 Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
