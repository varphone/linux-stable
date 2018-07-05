/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/init.h>
#include <linux/ipu.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mxcfb.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include "mxc_dispdrv.h"

#define ADV739X_MODE_NTSC	0
#define ADV739X_MODE_PAL	1

struct adv739x_platform_data {
	u32 default_ifmt;
	u32 ipu_id;
	u32 disp_id;
};

struct adv739x_data {
	struct platform_device *pdev;
	struct i2c_client *client;
	struct mxc_dispdrv_handle *disp_adv739x;
	struct fb_info *fbi;

	int ipu_id;
	int disp_id;
	int default_ifmt;

	int cur_mode;
	int enabled;
	struct notifier_block nb;
};

/*
 * left_margin: used for field0 vStart width in lines
 *
 * right_margin: used for field0 vEnd width in lines
 *
 * up_margin: used for field1 vStart width in lines
 *
 * down_margin: used for field1 vEnd width in lines
 *
 * hsync_len: EAV Code + Blanking Video + SAV Code (in pixel clock count)
 *         For BT656 NTSC, it is 4 + 67*4 + 4 = 276.
 *         For BT1120 NTSC, it is 4 + 67*2 + 4 = 142.
 *         For BT656 PAL, it is 4 + 70*4 + 4 = 288.
 *         For BT1120 PAL, it is 4 + 70*2 + 4 = 148.
 *
 * vsync_len: not used, set to 1
 */
static struct fb_videomode adv739x_modedb[] = {
	{
		/* NTSC Interlaced output */
		"BT656-NTSC", 60, 720, 480, 37037,
		19, 3,
		20, 3,
		276, 1,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_INTERLACED,
		FB_MODE_IS_DETAILED,
	},
	{
		/* PAL Interlaced output */
		"BT656-PAL", 50, 720, 576, 37037,
		22, 2,
		23, 2,
		288, 1,
		FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
		FB_VMODE_INTERLACED,
		FB_MODE_IS_DETAILED,
	},
};

static int adv739x_modedb_sz = ARRAY_SIZE(adv739x_modedb);

static int adv739x_write(struct i2c_client *client, u8 reg, u8 data)
{
	int ret = 0;
	ret = i2c_smbus_write_byte_data(client, reg, data);

	return ret;
}

__maybe_unused
static int adv739x_read(struct i2c_client *client, u8 reg)
{
	int data = 0;
	data = i2c_smbus_read_byte_data(client, reg);

	return data;
}

static void adv739x_setmode(struct adv739x_data *adv739x, int mode)
{
	struct i2c_client *client = adv739x->client;

	if (adv739x->enabled == 0)
		return;

	dev_dbg(&adv739x->client->dev, "adv739x_setmode: mode = %d.\n", mode);
	switch (mode) {
	case ADV739X_MODE_NTSC:
		// Reg 0x17: reset
		adv739x_write(client, 0x17, 0x02);

		mdelay(20);

		// Reg 0x00: DAC1~3 power on
		adv739x_write(client, 0x00, 0x1C);

		// Reg 0x01: SD input
		adv739x_write(client, 0x01, 0x00);

		//NTSC
		// Reg 0x80: SD, NTSC
		adv739x_write(client, 0x80, 0x10);

		// Reg 0x82: SD, CVBS
		adv739x_write(client, 0x82, 0xCB);
		break;

	case ADV739X_MODE_PAL:
		// Reg 0x17: reset
		adv739x_write(client, 0x17, 0x02);

		mdelay(20);

		// Reg 0x00: DAC1~3 power on
		adv739x_write(client, 0x00, 0x1C);

		// Reg 0x01: SD input
		adv739x_write(client, 0x01, 0x00);

		// Reg 0x80: SD, PAL
		adv739x_write(client, 0x80, 0x11);

		// Reg 0x82: SD, CVBS
		adv739x_write(client, 0x82, 0xC3);
		adv739x_write(client, 0x8C, 0xCB);
		adv739x_write(client, 0x8D, 0x8A);
		adv739x_write(client, 0x8E, 0x09);
		adv739x_write(client, 0x8F, 0x2A);
		break;

	default:
		dev_err(&adv739x->client->dev, "unsupported mode.\n");
		break;
	}
}

static void adv739x_poweroff(struct adv739x_data *adv739x)
{
	if (adv739x->enabled != 0) {
		dev_dbg(&adv739x->client->dev, "adv739x_poweroff.\n");

		/* power off the adv739x */
		adv739x_write(adv739x->client, 0x00, 0x1F);

		adv739x->enabled = 0;
	}
}

static void adv739x_poweron(struct adv739x_data *adv739x)
{
	if (adv739x->enabled == 0) {
		dev_dbg(&adv739x->client->dev, "adv739x_poweron.\n");

		adv739x->enabled = 1;
		adv739x_setmode(adv739x, adv739x->cur_mode);
	}
}

int adv739x_fb_event(struct notifier_block *nb, unsigned long val, void *v)
{
	struct fb_event *event = v;
	struct fb_info *fbi = event->info;
	struct adv739x_data *adv739x = container_of(nb, struct adv739x_data, nb);

	if (strcmp(event->info->fix.id, adv739x->fbi->fix.id))
		return 0;

	switch (val) {
	case FB_EVENT_MODE_CHANGE:
		if (strcmp(fbi->mode->name, "BT656-NTSC") == 0)
			adv739x->cur_mode = ADV739X_MODE_NTSC;
		else if (strcmp(fbi->mode->name, "BT656-PAL") == 0)
			adv739x->cur_mode = ADV739X_MODE_PAL;
		adv739x_setmode(adv739x, adv739x->cur_mode);
		break;
	case FB_EVENT_BLANK:
		if (*((int *)event->data) == FB_BLANK_UNBLANK)
			adv739x_poweron(adv739x);
		else
			adv739x_poweroff(adv739x);
		break;
	}
	return 0;
}

static int adv739x_disp_init(struct mxc_dispdrv_handle *disp,
                             struct mxc_dispdrv_setting *setting)
{
	int ret = 0, i;
	struct adv739x_data *adv739x = mxc_dispdrv_getdata(disp);
	struct adv739x_platform_data *plat = adv739x->client->dev.platform_data;
	struct fb_videomode *modedb = adv739x_modedb;
	int modedb_sz = adv739x_modedb_sz;
	static bool inited = false;

	if (inited)
		return -EBUSY;

	inited = true;

	/* use platform defined ipu/di */
	ret = ipu_di_to_crtc(&adv739x->pdev->dev, plat->ipu_id,
	                     plat->disp_id, &setting->crtc);
	if (ret < 0)
		return ret;

	ret = fb_find_mode(&setting->fbi->var, setting->fbi,
	                   setting->dft_mode_str, modedb, modedb_sz, NULL,
	                   setting->default_bpp);
	if (!ret) {
		fb_videomode_to_var(&setting->fbi->var, &modedb[0]);
		setting->if_fmt = plat->default_ifmt;
	}

	INIT_LIST_HEAD(&setting->fbi->modelist);
	for (i = 0; i < modedb_sz; i++) {
		fb_add_videomode(&modedb[i], &setting->fbi->modelist);
	}

	adv739x->fbi = setting->fbi;
	adv739x->enabled = 0;
	adv739x->cur_mode = ADV739X_MODE_PAL;	//default mode

	adv739x->pdev =
	    platform_device_register_simple("mxcfb_adv739x", 0, NULL, 0);
	if (IS_ERR(adv739x->pdev)) {
		dev_err(&adv739x->client->dev,
		        "Unable to register mxcfb_adv739x as a platform device\n");
		ret = PTR_ERR(adv739x->pdev);
		goto register_pltdev_failed;
	}

	adv739x->nb.notifier_call = adv739x_fb_event;
	ret = fb_register_client(&adv739x->nb);
	if (ret < 0)
		goto reg_fbclient_failed;

	return ret;

reg_fbclient_failed:
	platform_device_unregister(adv739x->pdev);
register_pltdev_failed:
	return ret;
}

static void adv739x_disp_deinit(struct mxc_dispdrv_handle *disp)
{
	struct adv739x_data *adv739x = mxc_dispdrv_getdata(disp);

	fb_unregister_client(&adv739x->nb);

	adv739x_poweroff(adv739x);

	platform_device_unregister(adv739x->pdev);
}

static int adv739x_get_of_property(struct i2c_client *client,
                                   struct adv739x_platform_data *plat)
{
	int err;
	u32 ipu_id, disp_id;
	const char *default_ifmt;
	struct device_node *np = client->dev.of_node;

	err = of_property_read_string(np, "default_ifmt", &default_ifmt);
	if (err) {
		dev_dbg(&client->dev, "get of property default_ifmt fail\n");
		return err;
	}
	err = of_property_read_u32(np, "ipu_id", &ipu_id);
	if (err) {
		dev_dbg(&client->dev, "get of property ipu_id fail\n");
		return err;
	}
	err = of_property_read_u32(np, "disp_id", &disp_id);
	if (err) {
		dev_dbg(&client->dev, "get of property disp_id fail\n");
		return err;
	}

	plat->ipu_id = ipu_id;
	plat->disp_id = disp_id;
	if (!strncmp(default_ifmt, "BT656", 5))
		plat->default_ifmt = IPU_PIX_FMT_BT656;
	else if (!strncmp(default_ifmt, "BT1120", 6))
		plat->default_ifmt = IPU_PIX_FMT_BT1120;
	else {
		dev_err(&client->dev, "err default_ifmt!\n");
		return -ENOENT;
	}

	return err;
}

static struct mxc_dispdrv_driver adv739x_disp_driver = {
	.name = "adv739x",
	.init = adv739x_disp_init,
	.deinit = adv739x_disp_deinit,
};

static int __init adv739x_tvout_probe(struct i2c_client *client,
                                      const struct i2c_device_id *id)
{
	int ret = 0;
	struct adv739x_data *adv739x;
	struct adv739x_platform_data *plat;

	if (!i2c_check_functionality(client->adapter,
	                             I2C_FUNC_SMBUS_BYTE |
	                             I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	adv739x = devm_kzalloc(&client->dev, sizeof(struct adv739x_data),
	                       GFP_KERNEL);
	if (!adv739x)
		return -ENOMEM;

	plat = devm_kzalloc(&client->dev, sizeof(struct adv739x_platform_data),
	                    GFP_KERNEL);
	if (!plat)
		return -ENOMEM;

	ret = adv739x_get_of_property(client, plat);
	if (ret < 0)
		return ret;

	adv739x->client = client;
	adv739x->client->dev.platform_data = plat;

	adv739x->disp_adv739x = mxc_dispdrv_register(&adv739x_disp_driver);
	mxc_dispdrv_setdata(adv739x->disp_adv739x, adv739x);

	i2c_set_clientdata(client, adv739x);

	return ret;
}

static int __exit adv739x_tvout_remove(struct i2c_client *client)
{
	struct adv739x_data *adv739x = i2c_get_clientdata(client);

	mxc_dispdrv_puthandle(adv739x->disp_adv739x);
	mxc_dispdrv_unregister(adv739x->disp_adv739x);

	return 0;
}

static const struct i2c_device_id adv739x_tvout_id[] = {
	{ "adv739x_tvout", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, adv739x_tvout_id);

static const struct of_device_id adv739x_tvout_dt_ids[] = {
	{ .compatible = "fsl,adv739x_tvout", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, adv739x_tvout_dt_ids);

static struct i2c_driver adv739x_tvout_i2c_driver = {
	.driver = {
		.name = "adv739x_tvout",
		.owner = THIS_MODULE,
		.of_match_table = adv739x_tvout_dt_ids,
	},
	.probe = adv739x_tvout_probe,
	.remove = adv739x_tvout_remove,
	.id_table = adv739x_tvout_id,
};

module_i2c_driver(adv739x_tvout_i2c_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("ADV739x TV encoder driver");
MODULE_LICENSE("GPL");
