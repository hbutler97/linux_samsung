/*
 *  at3_dummy.c
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>

#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>

#include "i2s.h"
#include "i2s-regs.h"

static struct snd_soc_card at3;

#define TDM_MAX_SLOT(a, b) ((a)>(b))?(a):(b)
#define TDM_SLOT_WIDTH		16

static int at3_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;
//	int tdm_slot_num = TDM_MAX_SLOT(
//		CONFIG_SND_SOC_I2S_TXSLOT_NUMBER,CONFIG_SND_SOC_I2S_RXSLOT_NUMBER);
//	int i, txmask = 1, rxmask = 1;

	dev_info(card->dev, "%s-%d %dch, %dHz, %dbytes\n",
			rtd->dai_link->name, substream->stream,
			params_channels(params), params_rate(params),
			params_buffer_bytes(params));

	/* Set CPU DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_DSP_A
					 | SND_SOC_DAIFMT_NB_NF
					 | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		dev_err(card->dev, "Failed to set aif1 cpu fmt: %d\n", ret);
		return ret;
	}

#if 0
	for (i=CONFIG_SND_SOC_I2S_TXSLOT_NUMBER;--i;)
		txmask = (txmask<<1)|1;
	for (i=CONFIG_SND_SOC_I2S_RXSLOT_NUMBER;--i;)
		rxmask = (rxmask<<1)|1;
#endif

	ret = snd_soc_dai_set_tdm_slot(cpu_dai, TDM_TX_SLOTS_MASK,
			TDM_RX_SLOTS_MASK, params_channels(params),
			runtime->sample_bits);
	if (ret < 0) {
		dev_err(card->dev, "Failed to set tdm slot: %d\n", ret);
 		return ret;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, SAMSUNG_I2S_CDCLK,
					0, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(card->dev, "Failed to set SAMSUNG_I2S_CDCL: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, SAMSUNG_I2S_OPCLK,
					0, MOD_OPCLK_PCLK);
	if (ret < 0) {
		dev_err(card->dev, "Failed to set SAMSUNG_I2S_OPCL: %d\n", ret);
		return ret;
	}

	return ret;
}

static struct snd_soc_ops at3_ops = {
	.hw_params = at3_hw_params,
};

static struct snd_soc_dai_link at3_dai[] = {
	{ /* Primary DAI i/f */
		.name = "DUMMY PRI",
		.stream_name = "i2s0-pri",
		.codec_dai_name = "dummy-aif1",
		.ops = &at3_ops,
	}
};

static int at3_suspend_post(struct snd_soc_card *card)
{
	return 0;
}

static int at3_resume_pre(struct snd_soc_card *card)
{
	return 0;
}

static struct snd_soc_card at3 = {
	.name = "AT3-I2S",
	.owner = THIS_MODULE,
	.suspend_post = at3_suspend_post,
	.resume_pre = at3_resume_pre,
	.dai_link = at3_dai,
	.num_links = ARRAY_SIZE(at3_dai),
};

static int at3_audio_probe(struct platform_device *pdev)
{
	int n, ret;
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_card *card = &at3;

	card->dev = &pdev->dev;

	for (n = 0; np && n < ARRAY_SIZE(at3_dai); n++) {
		if (!at3_dai[n].cpu_dai_name) {
			at3_dai[n].cpu_of_node = of_parse_phandle(np,
					"samsung,audio-cpu", n);

			if (!at3_dai[n].cpu_of_node) {
				dev_err(&pdev->dev, "Property "
				"'samsung,audio-cpu' missing or invalid\n");
				ret = -EINVAL;
			}
		}

		if (!at3_dai[n].platform_name)
			at3_dai[n].platform_of_node = at3_dai[n].cpu_of_node;

		at3_dai[n].codec_name = NULL;
		at3_dai[n].codec_of_node = of_parse_phandle(np,
				"samsung,audio-codec", n);
		if (!at3_dai[0].codec_of_node) {
			dev_err(&pdev->dev,
			"Property 'samsung,audio-codec' missing or invalid\n");
			ret = -EINVAL;
		}
	}

	ret = snd_soc_register_card(card);

	if (ret)
		dev_err(&pdev->dev, "snd_soc_register_card() failed:%d\n", ret);

	dev_info(&pdev->dev, "%s : register is called\n", __func__);
	dev_info(&pdev->dev, "%s : register is called\n", __func__);
	dev_info(&pdev->dev, "%s : register is called\n", __func__);
	dev_info(&pdev->dev, "%s : register is called\n", __func__);
	dev_info(&pdev->dev, "%s : register is called\n", __func__);
	dev_info(&pdev->dev, "%s : register is called\n", __func__);
	dev_info(&pdev->dev, "%s : register is called\n", __func__);
	dev_info(&pdev->dev, "%s : register is called\n", __func__);

	return ret;
}

static int at3_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id samsung_dummy_of_match[] = {
	{ .compatible = "samsung,at3-dummy", },
	{},
};
MODULE_DEVICE_TABLE(of, samsung_dummy_of_match);
#endif /* CONFIG_OF */

static struct platform_driver at3_audio_driver = {
	.driver		= {
		.name	= "at3-audio",
		.owner	= THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = of_match_ptr(samsung_dummy_of_match),
	},
	.probe		= at3_audio_probe,
	.remove		= at3_audio_remove,
};

module_platform_driver(at3_audio_driver);

MODULE_DESCRIPTION("ALSA SoC AT3 DUMMY");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:AT3-audio");
