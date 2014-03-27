/*
 * imx-wm9712.c  --  SoC audio for imx phycard in AC97 mode
 *
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/fsl_devices.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/ac97_codec.h>
#include <asm/mach-types.h>
#include <mach/audmux.h>

#include "imx-ssi.h"

#define		WORD_PER_FRAME_AC97		13
#define		BIT_PER_WORD_AC97		16

static struct snd_soc_card imx_phycard;

static int imx_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int channels = params_channels(params);
	int ret;

	ret = snd_soc_dai_set_sysclk(cpu_dai, IMX_SSP_SYS_CLK, 0, SND_SOC_CLOCK_IN);

	if (ret < 0)
		return ret;

	snd_soc_dai_set_tdm_slot(cpu_dai, 0xfffc, 0xfffc, WORD_PER_FRAME_AC97 * channels, BIT_PER_WORD_AC97);

	if (ret < 0)
		return ret;

    ret = snd_soc_dai_set_sysclk(cpu_dai, IMX_SSP_SYS_CLK, 0,  SND_SOC_CLOCK_OUT);

	return 0;
}

static struct snd_soc_ops imx_phycard_hifi_ops = {
	.hw_params = imx_hw_params,
};

static struct snd_soc_dai_link imx_phycard_dai_ac97[] = {
	{
		.name		= "HiFi",
		.stream_name	= "HiFi",
		.codec_dai_name		= "wm9712-hifi",
		.codec_name	= "wm9712-codec",
		.cpu_dai_name	= "imx-ssi.1",
		.platform_name	= "imx-pcm-audio.1",
		.ops = 	&imx_phycard_hifi_ops,
	},
};

static struct snd_soc_card imx_phycard = {
	.name		= "wm9712-audio",
	.dai_link	= imx_phycard_dai_ac97,
	.num_links	= ARRAY_SIZE(imx_phycard_dai_ac97),
};

static int imx_audmux_config(int slave, int master)
{
	unsigned int ptcr, pdcr;
	slave = slave - 1;
	master = master - 1;

	ptcr = MXC_AUDMUX_V2_PTCR_SYN |
		MXC_AUDMUX_V2_PTCR_TCLKDIR |
		MXC_AUDMUX_V2_PTCR_TCSEL(master);
	pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(master);
	mxc_audmux_v2_configure_port(slave, ptcr, pdcr);

	ptcr = MXC_AUDMUX_V2_PTCR_SYN |
		MXC_AUDMUX_V2_PTCR_TFSDIR |
		MXC_AUDMUX_V2_PTCR_TFSEL(slave);
	pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(slave);
	mxc_audmux_v2_configure_port(master, ptcr, pdcr);

	return 0;
}

static int __devinit imx_wm9712_probe(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	int ret = 0;

	imx_audmux_config(plat->src_port, plat->ext_port);

	if (plat->init && plat->init())
		ret = -EINVAL;

	return ret;
}

static int imx_wm9712_remove(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;

	if (plat->finit)
		plat->finit();

	return 0;
}

static struct platform_driver imx_wm9712_driver = {
	.probe = imx_wm9712_probe,
	.remove = imx_wm9712_remove,
	.driver = {
		.name = "imx-wm9712",
	},
};

static struct platform_device *imx_phycard_snd_ac97_device;
static struct platform_device *imx_phycard_snd_device;

static int __init imx_phycard_init(void)
{
	int ret;

	ret = platform_driver_register(&imx_wm9712_driver);
	if (ret)
		return -ENOMEM;

	imx_phycard_snd_ac97_device = platform_device_alloc("soc-audio", -1);
	if (!imx_phycard_snd_ac97_device)
		return -ENOMEM;
	platform_set_drvdata(imx_phycard_snd_ac97_device, &imx_phycard);
	ret = platform_device_add(imx_phycard_snd_ac97_device);
	if (ret)
		goto fail1;

	imx_phycard_snd_device = platform_device_alloc("wm9712-codec", -1);
	if (!imx_phycard_snd_device) {
		ret = -ENOMEM;
		goto fail2;
	}
	ret = platform_device_add(imx_phycard_snd_device);

	if (ret) {
		printk(KERN_ERR "ASoC: Platform device allocation failed\n");
		goto fail3;
	}

	return 0;

fail3:
	platform_device_put(imx_phycard_snd_device);
fail2:
	platform_device_del(imx_phycard_snd_ac97_device);
fail1:
	platform_device_put(imx_phycard_snd_ac97_device);
	platform_driver_unregister(&imx_wm9712_driver);
	return ret;
}

static void __exit imx_phycard_exit(void)
{
	platform_device_unregister(imx_phycard_snd_ac97_device);
	platform_device_unregister(imx_phycard_snd_device);
	platform_driver_unregister(&imx_wm9712_driver);
}

late_initcall(imx_phycard_init);
module_exit(imx_phycard_exit);

MODULE_AUTHOR("Anatoly Palto");
MODULE_DESCRIPTION("WM9712 PHYCard ASoC driver");
MODULE_LICENSE("GPL");
