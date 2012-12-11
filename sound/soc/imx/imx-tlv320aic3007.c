/*
 * ASoC driver for i.MX6 boards with tlv320aic3007 codec
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/fsl_devices.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <asm/mach-types.h>
#include <mach/audmux.h>

#include "../codecs/tlv320aic3x.h"
#include "imx-ssi.h"

#define CODEC_CLOCK 19200000

static int imx_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;
	unsigned int channels = params_channels(params);

	// set codec DAI configuration
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S   |
                              SND_SOC_DAIFMT_NB_NF |
                              SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
                              SND_SOC_DAIFMT_NB_NF |
                              SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(codec_dai, 0, CODEC_CLOCK, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

    snd_soc_dai_set_tdm_slot(cpu_dai, 0xfffffffc, 0xfffffffc, 2, 0);

    ret = snd_soc_dai_set_sysclk(cpu_dai, IMX_SSP_SYS_CLK, 0,
                                 SND_SOC_CLOCK_IN);

	return 0;
}

static struct snd_soc_ops imx_ops = {
	.hw_params = imx_hw_params,
};

/* davinci-evm machine dapm widgets */
static const struct snd_soc_dapm_widget aic3x_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_LINE("Line Out", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_LINE("Line In", NULL),
};

/* davinci-evm machine audio_mapnections to the codec pins */
static const struct snd_soc_dapm_route audio_map[] = {
	/* Headphone connected to HPLOUT, HPROUT */
	{"Headphone Jack", NULL, "HPLOUT"},
	{"Headphone Jack", NULL, "HPROUT"},

	/* Line Out connected to LLOUT, RLOUT */
	{"Line Out", NULL, "LLOUT"},
	{"Line Out", NULL, "RLOUT"},

	/* Mic connected to (MIC3L | MIC3R) */
	{"MIC3L", NULL, "Mic Bias 2V"},
	{"MIC3R", NULL, "Mic Bias 2V"},
	{"Mic Bias 2V", NULL, "Mic Jack"},

	/* Line In connected to (LINE1L | LINE2L), (LINE1R | LINE2R) */
	{"LINE1L", NULL, "Line In"},
	{"LINE2L", NULL, "Line In"},
	{"LINE1R", NULL, "Line In"},
	{"LINE2R", NULL, "Line In"},
};


/* Logic for a aic3x as connected on a imx */
static int imx_aic3x_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	/* Add davinci-evm specific widgets */
	snd_soc_dapm_new_controls(dapm, aic3x_dapm_widgets,
				  ARRAY_SIZE(aic3x_dapm_widgets));

	/* Set up davinci-evm specific audio path audio_map */
	snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));

	/* not connected */
	snd_soc_dapm_disable_pin(dapm, "MONO_LOUT");
	snd_soc_dapm_disable_pin(dapm, "HPLCOM");
	snd_soc_dapm_disable_pin(dapm, "HPRCOM");

	/* always connected */
	snd_soc_dapm_enable_pin(dapm, "Headphone Jack");
	snd_soc_dapm_enable_pin(dapm, "Line Out");
	snd_soc_dapm_enable_pin(dapm, "Mic Jack");
	snd_soc_dapm_enable_pin(dapm, "Line In");

	snd_soc_dapm_sync(dapm);

	return 0;
}

static struct snd_soc_dai_link imx_tlv320aic3007_dai = {
	.name = "TLV320AIC3X",
	.stream_name = "AIC3X",
	.cpu_dai_name= "imx-ssi.1",
	.codec_dai_name = "tlv320aic3x-hifi",
	.codec_name = "tlv320aic3x-codec.1-0018",
	.platform_name = "imx-pcm-audio.1",
	.init = imx_aic3x_init,
	.ops = &imx_ops,
};


static struct snd_soc_card imx_tlv320aic3007_card = {
	.name = "imx_tlv320aic3007-audio",
	.dai_link = &imx_tlv320aic3007_dai,
	.num_links = 1,
};


static int imx_audmux_config(int slave, int master)
{
	unsigned int ptcr, pdcr;
    unsigned int ptcr_read, pdcr_read;
	slave = slave - 1;
	master = master - 1;

	/* SSI0 mastered by port 5 */
	ptcr = MXC_AUDMUX_V2_PTCR_TFSDIR |
		MXC_AUDMUX_V2_PTCR_TFSEL(master) |
		MXC_AUDMUX_V2_PTCR_TCLKDIR |
		MXC_AUDMUX_V2_PTCR_TCSEL(master);
	pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(master);
	mxc_audmux_v2_configure_port(slave, ptcr, pdcr);

	ptcr = 0;
	pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(slave);
	mxc_audmux_v2_configure_port(master, ptcr, pdcr);

	return 0;
}


static int __devinit imx_tlv320aic3007_probe(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
    
	imx_audmux_config(plat->src_port, plat->ext_port);
    
	int ret = 0;

	ret = -EINVAL;
	if (plat->init && plat->init())
		return ret;

	return 0;
}

static int imx_tlv320aic3007_remove(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;

	if (plat->finit)
		plat->finit();

	return 0;
}


static struct platform_driver tlv320aic3007_audio_driver = {
	.probe = imx_tlv320aic3007_probe,
	.remove = imx_tlv320aic3007_remove,
	.driver = {
        .name = "tlv320aic3007",
    },
};



static struct platform_device *imx_tlv320aic3007_snd_device;

static int __init imx_tlv320aic3007_init(void)
{
	int ret;

	ret = platform_driver_register(&tlv320aic3007_audio_driver);
	if (ret)
		return -ENOMEM;

    imx_tlv320aic3007_snd_device = platform_device_alloc("soc-audio", -1);
	if (!imx_tlv320aic3007_snd_device)
		return -ENOMEM;

	platform_set_drvdata(imx_tlv320aic3007_snd_device, &imx_tlv320aic3007_card);
	ret = platform_device_add(imx_tlv320aic3007_snd_device);
	if (ret)
    {
		platform_device_put(imx_tlv320aic3007_snd_device);
    }

    return ret;
}

static void __exit imx_tlv320aic3007_exit(void)
{
	platform_device_unregister(imx_tlv320aic3007_snd_device);
}

module_init(imx_tlv320aic3007_init);
module_exit(imx_tlv320aic3007_exit);

MODULE_AUTHOR("Lavnikevich Dmitry");
MODULE_DESCRIPTION("TLV320AIC3007 PHYFlex ASoC driver");
MODULE_LICENSE("GPL");
