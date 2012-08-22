/*
 * TI ADC MFD driver
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include "../iio.h"
#include <linux/mfd/ti_tscadc.h>
#include <linux/platform_data/ti_adc.h>

struct adc_device {
	struct ti_tscadc_dev	*mfd_tscadc;
	struct iio_dev	*idev;
	int channels;
};

static unsigned int adc_readl(struct adc_device *adc, unsigned int reg)
{
	return readl(adc->mfd_tscadc->tscadc_base + reg);
}

static void adc_writel(struct adc_device *adc, unsigned int reg,
					unsigned int val)
{
	writel(val, adc->mfd_tscadc->tscadc_base + reg);
}

static void adc_step_config(struct adc_device *adc_dev)
{
	unsigned int    stepconfig;
	int i, channels = 0, steps;

	/*
	 * There are 16 configurable steps and 8 analog input
	 * lines available which are shared between Touchscreen and ADC.
	 *
	 * Steps backwards i.e. from 16 towards 0 are used by ADC
	 * depending on number of input lines needed.
	 * Channel would represent which analog input
	 * needs to be given to ADC to digitalize data.
	 */

	steps = TOTAL_STEPS - adc_dev->channels;
	channels = TOTAL_CHANNELS - adc_dev->channels;

	stepconfig = TSCADC_STEPCONFIG_AVG_16 | TSCADC_STEPCONFIG_FIFO1;

	for (i = (steps + 1); i <= TOTAL_STEPS; i++) {
		adc_writel(adc_dev, TSCADC_REG_STEPCONFIG(i),
				stepconfig | TSCADC_STEPCONFIG_INP(channels));
		adc_writel(adc_dev, TSCADC_REG_STEPDELAY(i),
				TSCADC_STEPCONFIG_OPENDLY);
		channels++;
	}
	adc_writel(adc_dev, TSCADC_REG_SE, TSCADC_STPENB_STEPENB);
}

static int tiadc_channel_init(struct iio_dev *idev, struct adc_device *adc_dev)
{
	struct iio_chan_spec *chan_array;
	int i;

	idev->num_channels = adc_dev->channels;
	chan_array = kcalloc(idev->num_channels, sizeof(struct iio_chan_spec),
					GFP_KERNEL);

	if (chan_array == NULL)
		return -ENOMEM;

	for (i = 0; i < (idev->num_channels); i++) {
		struct iio_chan_spec *chan = chan_array + i;
		chan->type = IIO_VOLTAGE;
		chan->indexed = 1;
		chan->channel = i;
		chan->scan_type.sign = 'u';
		chan->scan_type.realbits = 12;
		chan->scan_type.storagebits = 32;
		chan->scan_type.shift = 0;
	}

	idev->channels = chan_array;
	return idev->num_channels;
}

static void tiadc_channel_remove(struct iio_dev *idev)
{
	kfree(idev->channels);
}

static int tiadc_read_raw(struct iio_dev *idev,
		struct iio_chan_spec const *chan,
		int *val, int *val2, long mask)
{
	struct adc_device *adc_dev = iio_priv(idev);
	int i;
	unsigned int fifo1count, readx1;

	fifo1count = adc_readl(adc_dev, TSCADC_REG_FIFO1CNT);
	for (i = 0; i < fifo1count; i++) {
		readx1 = adc_readl(adc_dev, TSCADC_REG_FIFO1);
		if (i == chan->channel) {
			readx1 = readx1 & 0xfff;
			*val = readx1;
		}
	}
	adc_writel(adc_dev, TSCADC_REG_SE, TSCADC_STPENB_STEPENB);
	return IIO_VAL_INT;
}

static const struct iio_info tiadc_info = {
	.read_raw = &tiadc_read_raw,
};

static int __devinit tiadc_probe(struct platform_device *pdev)
{
	struct iio_dev		*idev;
	struct adc_device	*adc_dev = NULL;
	struct ti_tscadc_dev	*tscadc_dev = pdev->dev.platform_data;
	struct mfd_tscadc_board	*pdata;
	int			err;

	pdata = (struct mfd_tscadc_board *)tscadc_dev->dev->platform_data;
	if (!pdata || !pdata->adc_init)  {
		dev_err(tscadc_dev->dev, "Could not find platform data\n");
		return -EINVAL;
	}

	idev = iio_allocate_device(sizeof(struct adc_device));
	if (idev == NULL) {
		dev_err(&pdev->dev, "failed to allocate iio device.\n");
		err = -ENOMEM;
		goto err_allocate;
	}
	adc_dev = iio_priv(idev);

	tscadc_dev->adc = adc_dev;
	adc_dev->mfd_tscadc = tscadc_dev;
	adc_dev->idev = idev;
	adc_dev->channels = pdata->adc_init->adc_channels;

	idev->dev.parent = &pdev->dev;
	idev->name = dev_name(&pdev->dev);
	idev->modes = INDIO_DIRECT_MODE;
	idev->info = &tiadc_info;

	adc_step_config(adc_dev);

	err = tiadc_channel_init(idev, adc_dev);
	if (err < 0)
		goto err_cleanup_channels;

	err = iio_device_register(idev);
	if (err)
		goto err_unregister;

	dev_info(&pdev->dev, "attached adc driver\n");
	platform_set_drvdata(pdev, idev);

	return 0;

err_unregister:
	tiadc_channel_remove(idev);
err_cleanup_channels:
	iio_device_unregister(idev);
	iio_free_device(idev);
err_allocate:
	return err;
}

static int __devexit tiadc_remove(struct platform_device *pdev)
{
	struct ti_tscadc_dev	*tscadc_dev = pdev->dev.platform_data;
	struct adc_device	*adc_dev = tscadc_dev->adc;
	struct iio_dev		*idev = adc_dev->idev;

	iio_device_unregister(idev);
	tiadc_channel_remove(idev);

	tscadc_dev->adc = NULL;
	iio_free_device(idev);

	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver tiadc_driver = {
	.driver = {
		.name   = "tiadc",
		.owner = THIS_MODULE,
	},
	.probe          = tiadc_probe,
	.remove         = __devexit_p(tiadc_remove),
};

module_platform_driver(tiadc_driver);

MODULE_DESCRIPTION("TI ADC controller driver");
MODULE_AUTHOR("Rachna Patil <rachna@ti.com>");
MODULE_LICENSE("GPL");
