/*
 * sn-soc-hammerhead.c  --  SoC audio for Hammerhead board
 *
 * Author:	Andre Renaud <andre@bluewatersys.com>
 *		Bluewater Systems Ltd
 * Created:	25 Nov 2008
 *
 * Based on corgi.c by:
 *
 * Copyright 2005 Wolfson Microelectronics PLC.
 * Copyright 2005 Openedhand Ltd.
 *
 * Authors: Liam Girdwood <liam.girdwood <at> wolfsonmicro.com>
 *          Richard Purdie <richard <at> openedhand.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <mach/hardware.h>
#include <mach/cpu.h>

#include "../codecs/tlv320aic26.h"
#include "atmel-pcm.h"
#include "atmel_ssc_dai.h"

/*
 * The codec clock is provided externally either from a dedicated
 * crystal or from the FPGA
 */
#define CODEC_CLK_RATE	24000000
#define MCK_RATE	12000000

static int hammerhead_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	int ret;

	ret = snd_soc_dai_set_sysclk(codec_dai, 0, CODEC_CLK_RATE,
				     SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	pr_debug("hammerhead_audio: started\n");
	return 0;
}

static void hammerhead_shutdown(struct snd_pcm_substream *substream)
{
	pr_debug("hammerhead_audio: stopped\n");
}

static int hammerhead_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	int ret;
	struct clk *mclk;
	unsigned long mclk_rate;
	unsigned int rate;
	int cmr_div, period;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, (SND_SOC_DAIFMT_I2S |
					      SND_SOC_DAIFMT_CBS_CFS));
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, (SND_SOC_DAIFMT_I2S |
					    SND_SOC_DAIFMT_CBS_CFS));
	if (ret < 0)
		return ret;

	/* Get the cpu (mclk) clock speed */
	mclk = clk_get(NULL, "mck");
	if (IS_ERR(mclk))
		return PTR_ERR(mclk);
	mclk_rate = clk_get_rate(mclk);
	clk_put(mclk);

	rate = params_rate(params);
	cmr_div = (mclk_rate + (rate * 32)) / (2 * rate * 32);
	period = 15;

	pr_debug("hammerhead_audio: mclk = %ld, period = %d, cmr_div = %d\n",
		 mclk_rate, period, cmr_div);

	/* Check that the rate is valid */
	if (cmr_div < 1 || cmr_div >= 8190)
		return -EINVAL;

	/* set the MCK divider for BCLK */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, ATMEL_SSC_CMR_DIV, cmr_div);
	if (ret < 0)
		return ret;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* set the BCLK divider for DACLRC */
		ret = snd_soc_dai_set_clkdiv(cpu_dai, ATMEL_SSC_TCMR_PERIOD,
					     period);
	} else {
		/* set the BCLK divider for ADCLRC */
		ret = snd_soc_dai_set_clkdiv(cpu_dai, ATMEL_SSC_RCMR_PERIOD,
					     period);
	}
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops hammerhead_ops = {
	.startup	= hammerhead_startup,
	.hw_params	= hammerhead_hw_params,
	.shutdown	= hammerhead_shutdown,
};

static const struct snd_soc_dapm_widget hammerhead_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
};

static const struct snd_soc_dapm_route intercon[] = {
	{"Headphone Jack", NULL, "LHPOUT"},
	{"Headphone Jack", NULL, "RHPOUT"},
	{"MICIN",          NULL, "Mic Jack"},
};

static int hammerhead_tlv320_init(struct snd_soc_codec *codec)
{
	int i;

	/* Add specific widgets */
	for (i = 0; i < ARRAY_SIZE(hammerhead_dapm_widgets); i++)
		snd_soc_dapm_new_control(codec, &hammerhead_dapm_widgets[i]);

	/* Set up specific audio path interconnects */
	snd_soc_dapm_add_routes(codec, intercon, ARRAY_SIZE(intercon));

	snd_soc_dapm_enable_pin(codec, "Headphone Jack");
	snd_soc_dapm_enable_pin(codec, "Mic Jack");

	snd_soc_dapm_sync(codec);

	return 0;
}

static struct snd_soc_dai_link hammerhead_dai = {
	.name		= "tlv320aic26",
	.stream_name	= "TLV320aic26 PCM",
	.cpu_dai	= &atmel_ssc_dai[0],
	.codec_dai	= &aic26_dai,
	.init		= hammerhead_tlv320_init,
	.ops		= &hammerhead_ops,
};

static struct snd_soc_card snd_soc_hammerhead = {
	.name		= "Hammerhead",
	.platform	= &atmel_soc_platform,
	.dai_link	= &hammerhead_dai,
	.num_links	= 1,
};

static struct snd_soc_device hammerhead_snd_devdata = {
	.card		= &snd_soc_hammerhead,
	.codec_dev	= &aic26_soc_codec_dev,
};

static struct platform_device *hammerhead_snd_device;

static int __init hammerhead_init(void)
{
	struct atmel_ssc_info *ssc_p = hammerhead_dai.cpu_dai->private_data;
	struct ssc_device *ssc = NULL;
	int ret;

	ssc = ssc_request(0);
	if (IS_ERR(ssc)) {
		ret = PTR_ERR(ssc);
		ssc = NULL;
		goto fail;
	}
	ssc_p->ssc = ssc;

	hammerhead_snd_device = platform_device_alloc("soc-audio", -1);
	if (!hammerhead_snd_device) {
		ret = -ENOMEM;
		goto fail;
	}

	platform_set_drvdata(hammerhead_snd_device, &hammerhead_snd_devdata);
	hammerhead_snd_devdata.dev = &hammerhead_snd_device->dev;
	ret = platform_device_add(hammerhead_snd_device);
	if (ret) {
		goto fail_free;
	}

	pr_info("Hammerhead audio intitialised\n");
	return 0;

fail_free:
	platform_device_del(hammerhead_snd_device);
	platform_device_put(hammerhead_snd_device);
fail:
	return ret;
}

static void __exit hammerhead_exit(void)
{
	struct atmel_ssc_info *ssc_p = hammerhead_dai.cpu_dai->private_data;
	struct ssc_device *ssc;

	if (ssc_p) {
		ssc = ssc_p->ssc;
		if (ssc)
			ssc_free(ssc);
		ssc_p->ssc = NULL;
	}

	platform_device_unregister(hammerhead_snd_device);
	hammerhead_snd_device = NULL;
}

module_init(hammerhead_init);
module_exit(hammerhead_exit);

/* Module information */
MODULE_AUTHOR("Andre Renaud <andre@bluewatersys.com");
MODULE_DESCRIPTION("ALSA SoC Hammerhead");
MODULE_LICENSE("GPL");
