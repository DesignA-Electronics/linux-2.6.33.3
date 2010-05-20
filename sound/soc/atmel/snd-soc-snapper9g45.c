/*
 * snapper9260.c  --  SoC audio for Snapper9260 board
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

#include "../codecs/tlv320aic23.h"
#include "atmel-pcm.h"
#include "atmel_ssc_dai.h"

/*
 * This is the target rate for PCK0, we get the actual rate of the clock
 * after we try and set it, and pass this to the set_sysclk functions
 */
#define PCK0_RATE 12000000

static struct clk *pck0_clk = NULL, *pll_clk = NULL;
static int pck0_rate;

static int sn9260_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	int ret;

	/* codec system clock is supplied by PCK0, set to 12MHz */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0,
				     pck0_rate, SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	clk_enable(pll_clk);
	clk_enable(pck0_clk);

	pr_debug("snapper9260_audio: started\n");
	return 0;
}

static void sn9260_shutdown(struct snd_pcm_substream *substream)
{
	clk_disable(pck0_clk);
	clk_disable(pll_clk);

	pr_debug("snapper9260_audio: stopped\n");
}

static int sn9260_hw_params(struct snd_pcm_substream *substream,
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

	pr_debug("snapper9260_audio: mclk = %ld, period = %d, cmr_div = %d\n",
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

static struct snd_soc_ops sn9260_ops = {
	.startup	= sn9260_startup,
	.hw_params	= sn9260_hw_params,
	.shutdown	= sn9260_shutdown,
};

static const struct snd_soc_dapm_widget sn9260_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_LINE("Line Out", NULL),
	SND_SOC_DAPM_LINE("Line In", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
};

static const struct snd_soc_dapm_route intercon[] = {
	{"Headphone Jack", NULL, "LHPOUT"},
        {"Headphone Jack", NULL, "RHPOUT"},

	{"Line Out", NULL, "LOUT"},
	{"Line Out", NULL, "ROUT"},

        {"LLINEIN", NULL, "Line In"},
        {"RLINEIN", NULL, "Line In"},

        {"MICIN", NULL,	"Mic Jack"},
};

static int sn9260_tlv320_init(struct snd_soc_codec *codec)
{
	int i;

	/* Add specific widgets */
	for (i = 0; i < ARRAY_SIZE(sn9260_dapm_widgets); i++)
		snd_soc_dapm_new_control(codec, &sn9260_dapm_widgets[i]);

	/* Set up specific audio path interconnects */
	snd_soc_dapm_add_routes(codec, intercon, ARRAY_SIZE(intercon));

	snd_soc_dapm_enable_pin(codec, "Headphone Jack");
	snd_soc_dapm_enable_pin(codec, "Line Out");
	snd_soc_dapm_enable_pin(codec, "Line In");
	snd_soc_dapm_enable_pin(codec, "Mic Jack");
        printk("Socdev: %p\n", codec->socdev);

	snd_soc_dapm_sync(codec);

	return 0;
}

static struct snd_soc_dai_link sn9260_dai = {
	.name		= "tlv320aic23",
	.stream_name	= "TLV320aic23b PCM",
	.cpu_dai	= &atmel_ssc_dai[0],
	.codec_dai	= &tlv320aic23_dai,
	.init		= sn9260_tlv320_init,
	.ops		= &sn9260_ops,
};

static struct snd_soc_card snd_soc_snapper9260 = {
	.name		= "Snapper 9260",
	.platform	= &atmel_soc_platform,
	.dai_link	= &sn9260_dai,
	.num_links	= 1,
};

static struct snd_soc_device sn9260_snd_devdata = {
	.card		= &snd_soc_snapper9260,
	.codec_dev	= &soc_codec_dev_tlv320aic23,
};

static struct platform_device *sn9260_snd_device;

static int __init sn9260_init(void)
{
	struct atmel_ssc_info *ssc_p = sn9260_dai.cpu_dai->private_data;
	struct ssc_device *ssc = NULL;
	int ret;

	ssc = ssc_request(0);
	if (IS_ERR(ssc)) {
		ret = PTR_ERR(ssc);
		ssc = NULL;
		goto fail;
	}
	ssc_p->ssc = ssc;

	sn9260_snd_device = platform_device_alloc("soc-audio", -1);
	if (!sn9260_snd_device) {
		ret = -ENOMEM;
		goto fail;
	}

	platform_set_drvdata(sn9260_snd_device, &sn9260_snd_devdata);
	sn9260_snd_devdata.dev = &sn9260_snd_device->dev;

	ret = platform_device_add(sn9260_snd_device);
	if (ret) {
		platform_device_put(sn9260_snd_device);
		goto fail;
	}

	/*
	 * Set PCK0 rate to 12 Mhz. On Snapper 9260, we can get 12mhz by
	 * dividing from plla. On Snapper 9G20, we need to divide from pllb
	 * instead.
	 */
	if (cpu_is_at91sam9g20())
		pll_clk = clk_get(NULL, "pllb");
        else if (cpu_is_at91sam9g45())
                pll_clk = clk_get(NULL, "main");
	else
		pll_clk = clk_get(NULL, "plla");
	if (IS_ERR(pll_clk)) {
		ret = PTR_ERR(pll_clk);
		goto fail;
	}

	pck0_clk = clk_get(NULL, "pck0");
	if (IS_ERR(pck0_clk)) {
		ret = PTR_ERR(pck0_clk);
		goto fail_pck0;
	}

	ret = clk_set_parent(pck0_clk, pll_clk);
        if (ret < 0)
                goto fail_clk_set_rate;

	pck0_rate = clk_set_rate(pck0_clk, PCK0_RATE);
	if (pck0_rate < 0)
		goto fail_clk_set_rate;

	/* Put pck0 pin in alternative function mode */
        at91_set_B_periph(AT91_PIN_PC1, 0);

	pr_info("snapper9260_audio: clock = %dhz\n", pck0_rate);
	return 0;

fail_clk_set_rate:
	clk_put(pck0_clk);
fail_pck0:
	clk_put(pll_clk);
fail:
	return ret;
}

static void __exit sn9260_exit(void)
{
	struct atmel_ssc_info *ssc_p = sn9260_dai.cpu_dai->private_data;
	struct ssc_device *ssc;

	if (ssc_p) {
		ssc = ssc_p->ssc;
		if (ssc)
			ssc_free(ssc);
		ssc_p->ssc = NULL;
	}

	clk_put(pck0_clk);
	clk_put(pll_clk);

	platform_device_unregister(sn9260_snd_device);
	sn9260_snd_device = NULL;
}

module_init(sn9260_init);
module_exit(sn9260_exit);

/* Module information */
MODULE_AUTHOR("Andre Renaud <andre@bluewatersys.com");
MODULE_DESCRIPTION("ALSA SoC Snapper 9260");
MODULE_LICENSE("GPL");
