/*
 * wl1271bt.c  --  ALSA SoC WL1271 Bluetooth codec driver for omap3evm board
 *
 * Author: Sinoj M. Issac, <sinoj at mistralsolutions.com>
 *         Vishveshwar Bhat, <vishveshwar dot bhat at ti.com>
 *
 * Based on sound/soc/codecs/twl4030.c by Steve Sakoman
 *
 * This file provides stub codec that can be used on OMAP3530 evm to
 * send/receive voice samples to/from WL1271 Bluetooth chip over PCM interface.
 * The Bluetoothchip codec interface is configured by HCI commands. ALSA is
 * configured and aligned to the codec interface.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <sound/soc.h>
#include <sound/pcm.h>

/*
 * Since WL1271 PCM interface is intended for Voice,
 * Support sampling rate 8K only
 */
#define STUB_RATES  SNDRV_PCM_RATE_8000_96000

#define WL1271BT_RATES		SNDRV_PCM_RATE_8000
#define WL1271BT_FORMATS	SNDRV_PCM_FMTBIT_S16_LE

struct snd_soc_dai_driver wl1271bt_dai[] = {
{
	.name = "wl1271bt",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = STUB_RATES,
		.formats = WL1271BT_FORMATS,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = WL1271BT_RATES,
		.formats = WL1271BT_FORMATS,
	},
},

};

static struct snd_soc_codec_driver soc_codec_dev_wl1271bt;

static __devinit int wl1271bt_dev_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev, &soc_codec_dev_wl1271bt,
			wl1271bt_dai, ARRAY_SIZE(wl1271bt_dai));
}

static int __devexit wl1271bt_dev_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

MODULE_ALIAS("platform:wl1271bt");
static struct platform_driver wl1271bt_driver = {
	.probe		= wl1271bt_dev_probe,
	.remove		= __devexit_p(wl1271bt_dev_remove),
	.driver		= {
		.name	= "wl1271bt-dummy-codec",
		.owner	= THIS_MODULE,
	},
};

static int __init wl1271bt_modinit(void)
{
	return platform_driver_register(&wl1271bt_driver);
}
module_init(wl1271bt_modinit);

static void __exit wl1271bt_modexit(void)
{
	platform_driver_unregister(&wl1271bt_driver);
}
module_exit(wl1271bt_modexit);

MODULE_AUTHOR("Sinoj M. Issac");
MODULE_DESCRIPTION("WL1271 Bluetooth codec driver");
MODULE_LICENSE("GPL");

