/*
 * TI Touch Screen driver
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
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
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/input/ti_tsc.h>
#include <linux/delay.h>
#include <linux/mfd/ti_tscadc.h>

#define MAX_12BIT                       ((1 << 12) - 1)

int pen = 1;
unsigned int bckup_x = 0, bckup_y = 0;

struct tscadc {
	struct input_dev	*input;
	struct ti_tscadc_dev	*mfd_tscadc;
	int			wires;
	struct tsc_axis		x;
	struct tsc_axis		y;
	int			x_plate_resistance;
	int			irq;
	int			steps_to_config;
};

static unsigned int tscadc_readl(struct tscadc *ts, unsigned int reg)
{
	return readl(ts->mfd_tscadc->tscadc_base + reg);
}

static void tscadc_writel(struct tscadc *tsc, unsigned int reg,
					unsigned int val)
{
	writel(val, tsc->mfd_tscadc->tscadc_base + reg);
}

static void tsc_step_config(struct tscadc *ts_dev)
{
	unsigned int	stepconfigx = 0, stepconfigy = 0;
	unsigned int	delay, chargeconfig = 0;
	unsigned int	stepconfigz1 = 0, stepconfigz2 = 0;
	int		i, total_steps;

	/* Configure the Step registers */

	delay = TSCADC_STEPCONFIG_SAMPLEDLY | TSCADC_STEPCONFIG_OPENDLY;

	total_steps = 2 * ts_dev->steps_to_config;
	stepconfigx = TSCADC_STEPCONFIG_MODE_HWSYNC |
			TSCADC_STEPCONFIG_AVG_16 | TSCADC_STEPCONFIG_XPP;

	switch (ts_dev->wires) {
	case 4:
		stepconfigx |= TSCADC_STEPCONFIG_INP_AN2 |
				TSCADC_STEPCONFIG_XNN;
		break;
	case 5:
		stepconfigx |= TSCADC_STEPCONFIG_YNN |
				TSCADC_STEPCONFIG_INP_AN4 |
				TSCADC_STEPCONFIG_XNN |
				TSCADC_STEPCONFIG_YPP;
		break;
	case 8:
		stepconfigx |= TSCADC_STEPCONFIG_INP_AN2 |
				TSCADC_STEPCONFIG_XNN;
		break;
	}

	for (i = 1; i <= ts_dev->steps_to_config; i++) {
		tscadc_writel(ts_dev, TSCADC_REG_STEPCONFIG(i), stepconfigx);
		tscadc_writel(ts_dev, TSCADC_REG_STEPDELAY(i), delay);
	}

	stepconfigy = TSCADC_STEPCONFIG_MODE_HWSYNC |
			TSCADC_STEPCONFIG_AVG_16 | TSCADC_STEPCONFIG_YNN |
			TSCADC_STEPCONFIG_INM_ADCREFM | TSCADC_STEPCONFIG_FIFO1;
	switch (ts_dev->wires) {
	case 4:
		stepconfigy |= TSCADC_STEPCONFIG_YPP;
		break;
	case 5:
		stepconfigy |= TSCADC_STEPCONFIG_XPP |
			TSCADC_STEPCONFIG_INP_AN4 | TSCADC_STEPCONFIG_XNP |
			TSCADC_STEPCONFIG_YPN;
		break;
	case 8:
		stepconfigy |= TSCADC_STEPCONFIG_YPP;
		break;
	}

	for (i = (ts_dev->steps_to_config + 1); i <= total_steps; i++) {
		tscadc_writel(ts_dev, TSCADC_REG_STEPCONFIG(i), stepconfigy);
		tscadc_writel(ts_dev, TSCADC_REG_STEPDELAY(i), delay);
	}

	chargeconfig = TSCADC_STEPCONFIG_XPP | TSCADC_STEPCONFIG_YNN |
			TSCADC_STEPCHARGE_RFP_XPUL |
			TSCADC_STEPCHARGE_RFM_XNUR |
			TSCADC_STEPCHARGE_INM_AN1 | TSCADC_STEPCHARGE_INP_AN1;
	tscadc_writel(ts_dev, TSCADC_REG_CHARGECONFIG, chargeconfig);
	tscadc_writel(ts_dev, TSCADC_REG_CHARGEDELAY, TSCADC_CHARGEDLY_OPENDLY);

	 /* Configure to calculate pressure */
	stepconfigz1 = TSCADC_STEPCONFIG_MODE_HWSYNC |
			TSCADC_STEPCONFIG_AVG_16 | TSCADC_STEPCONFIG_XNP |
			TSCADC_STEPCONFIG_YPN | TSCADC_STEPCONFIG_INM_ADCREFM;
	stepconfigz2 = stepconfigz1 | TSCADC_STEPCONFIG_INP_AN3 |
				TSCADC_STEPCONFIG_FIFO1;
	tscadc_writel(ts_dev, TSCADC_REG_STEPCONFIG(total_steps + 1),
						stepconfigz1);
	tscadc_writel(ts_dev, TSCADC_REG_STEPDELAY(total_steps + 1), delay);
	tscadc_writel(ts_dev, TSCADC_REG_STEPCONFIG(total_steps + 2),
						stepconfigz2);
	tscadc_writel(ts_dev, TSCADC_REG_STEPDELAY(total_steps + 2), delay);

	tscadc_writel(ts_dev, TSCADC_REG_SE, TSCADC_STPENB_STEPENB_TC);
}

static irqreturn_t tscadc_interrupt(int irq, void *dev)
{
	struct tscadc		*ts_dev = (struct tscadc *)dev;
	struct input_dev	*input_dev = ts_dev->input;
	unsigned int		status, irqclr = 0;
	int			i;
	int			fsm = 0, fifo0count = 0, fifo1count = 0;
	unsigned int		readx1 = 0, ready1 = 0;
	unsigned int		prev_val_x = ~0, prev_val_y = ~0;
	unsigned int		prev_diff_x = ~0, prev_diff_y = ~0;
	unsigned int		cur_diff_x = 0, cur_diff_y = 0;
	unsigned int		val_x = 0, val_y = 0, diffx = 0, diffy = 0;
	unsigned int		z1 = 0, z2 = 0, z = 0;
	unsigned int		channel;

	status = tscadc_readl(ts_dev, TSCADC_REG_IRQSTATUS);

	if (status & TSCADC_IRQENB_FIFO0THRES) {
		fifo0count = tscadc_readl(ts_dev, TSCADC_REG_FIFO0CNT);
		fifo1count = tscadc_readl(ts_dev, TSCADC_REG_FIFO1CNT);
		for (i = 0; i < (fifo0count-1); i++) {
			readx1 = tscadc_readl(ts_dev, TSCADC_REG_FIFO0);
			channel = readx1 & 0xf0000;
			channel = channel >> 0x10;
			if ((channel >= 0) &&
				(channel < ts_dev->steps_to_config)) {
				readx1 = readx1 & 0xfff;

				if (ts_dev->x.inverted)
					readx1 = ts_dev->x.max - readx1 + ts_dev->x.min;

				if (readx1 > prev_val_x)
					cur_diff_x = readx1 - prev_val_x;
				else
					cur_diff_x = prev_val_x - readx1;

				if (cur_diff_x < prev_diff_x) {
					prev_diff_x = cur_diff_x;
					val_x = readx1;
				}
				prev_val_x = readx1;
			}

			ready1 = tscadc_readl(ts_dev, TSCADC_REG_FIFO1);
			channel = ready1 & 0xf0000;
			channel = channel >> 0x10;
			if ((channel >= ts_dev->steps_to_config) &&
				(channel < (2 * ts_dev->steps_to_config - 1))) {
				ready1 &= 0xfff;

				if (ts_dev->y.inverted)
					ready1 = ts_dev->y.max - ready1 + ts_dev->y.min;

				if (ready1 > prev_val_y)
					cur_diff_y = ready1 - prev_val_y;
				else
					cur_diff_y = prev_val_y - ready1;

				if (cur_diff_y < prev_diff_y) {
					prev_diff_y = cur_diff_y;
					val_y = ready1;
				}
				prev_val_y = ready1;
			}
		}

		if (val_x > bckup_x) {
			diffx = val_x - bckup_x;
			diffy = val_y - bckup_y;
		} else {
			diffx = bckup_x - val_x;
			diffy = bckup_y - val_y;
		}
		bckup_x = val_x;
		bckup_y = val_y;

		z1 = ((tscadc_readl(ts_dev, TSCADC_REG_FIFO0)) & 0xfff);
		z2 = ((tscadc_readl(ts_dev, TSCADC_REG_FIFO1)) & 0xfff);

		fifo1count = tscadc_readl(ts_dev, TSCADC_REG_FIFO1CNT);
		for (i = 0; i < fifo1count; i++)
			tscadc_readl(ts_dev, TSCADC_REG_FIFO1);

		fifo0count = tscadc_readl(ts_dev, TSCADC_REG_FIFO0CNT);
		for (i = 0; i < fifo0count; i++)
			tscadc_readl(ts_dev, TSCADC_REG_FIFO0);

		if ((z1 != 0) && (z2 != 0)) {
			/*
			 * cal pressure using formula
			 * Resistance(touch) = x plate resistance *
			 * x postion/4096 * ((z2 / z1) - 1)
			 */
			z = z2 - z1;
			z *= val_x;
			z *= ts_dev->x_plate_resistance;
			z /= z1;
			z = (z + 2047) >> 12;

			/*
			 * Sample found inconsistent by debouncing
			 * or pressure is beyond the maximum.
			 * Don't report it to user space.
			 */
			if (pen == 0) {
				if ((diffx < 15) && (diffy < 15)
						&& (z <= MAX_12BIT)) {
					input_report_abs(input_dev, ABS_X,
							val_x);
					input_report_abs(input_dev, ABS_Y,
							val_y);
					input_report_abs(input_dev, ABS_PRESSURE,
							z);
					input_report_key(input_dev, BTN_TOUCH,
							1);
					input_sync(input_dev);
				}
			}
		}
		irqclr |= TSCADC_IRQENB_FIFO0THRES;
	}

	udelay(315);

	status = tscadc_readl(ts_dev, TSCADC_REG_RAWIRQSTATUS);
	if (status & TSCADC_IRQENB_PENUP) {
		/* Pen up event */
		fsm = tscadc_readl(ts_dev, TSCADC_REG_ADCFSM);
		if (fsm == 0x10) {
			pen = 1;
			bckup_x = 0;
			bckup_y = 0;
			input_report_key(input_dev, BTN_TOUCH, 0);
			input_report_abs(input_dev, ABS_PRESSURE, 0);
			input_sync(input_dev);
		} else {
			pen = 0;
		}
		irqclr |= TSCADC_IRQENB_PENUP;
	}
	irqclr |= TSCADC_IRQENB_HW_PEN;

	tscadc_writel(ts_dev, TSCADC_REG_IRQSTATUS, irqclr);

	tscadc_writel(ts_dev, TSCADC_REG_SE, TSCADC_STPENB_STEPENB_TC);
	return IRQ_HANDLED;
}

/*
* The functions for inserting/removing driver as a module.
*/

static	int __devinit tscadc_probe(struct platform_device *pdev)
{
	struct tscadc			*ts_dev;
	struct input_dev		*input_dev;
	int				err;
	int				irqenable;
	struct ti_tscadc_dev		*tscadc_dev = pdev->dev.platform_data;
	struct mfd_tscadc_board		*pdata;

	pdata = (struct mfd_tscadc_board *)tscadc_dev->dev->platform_data;
	if (!pdata) {
		dev_err(tscadc_dev->dev, "Could not find platform data\n");
		return -EINVAL;
	}

	/* Allocate memory for device */
	ts_dev = kzalloc(sizeof(struct tscadc), GFP_KERNEL);
	if (!ts_dev) {
		dev_err(&pdev->dev, "failed to allocate memory.\n");
		return -ENOMEM;
	}

	tscadc_dev->tsc = ts_dev;
	ts_dev->mfd_tscadc = tscadc_dev;

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&pdev->dev, "failed to allocate input device.\n");
		err = -ENOMEM;
		goto err_free_mem;
	}
	ts_dev->input = input_dev;

	ts_dev->irq = tscadc_dev->irq;
	err = request_irq(ts_dev->irq, tscadc_interrupt, IRQF_DISABLED,
				pdev->dev.driver->name, ts_dev);
	if (err) {
		dev_err(&pdev->dev, "failed to allocate irq.\n");
		goto err_fail;
	}

	ts_dev->wires = pdata->tsc_init->wires;
	ts_dev->x_plate_resistance = pdata->tsc_init->x_plate_resistance;
	ts_dev->steps_to_config = pdata->tsc_init->steps_to_configure;
	ts_dev->x.min = pdata->tsc_init->x.min;
	ts_dev->x.max = pdata->tsc_init->x.max;
	ts_dev->x.inverted = pdata->tsc_init->x.inverted;
	ts_dev->y.min = pdata->tsc_init->y.min;
	ts_dev->y.max = pdata->tsc_init->y.max;
	ts_dev->y.inverted = pdata->tsc_init->y.inverted;

	/* IRQ Enable */
	irqenable = TSCADC_IRQENB_FIFO0THRES;
	tscadc_writel(ts_dev, TSCADC_REG_IRQENABLE, irqenable);

	tsc_step_config(ts_dev);

	tscadc_writel(ts_dev, TSCADC_REG_FIFO0THR, ts_dev->steps_to_config);

	input_dev->name = "ti-tsc";
	input_dev->dev.parent = &pdev->dev;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	input_set_abs_params(input_dev, ABS_X,
			ts_dev->x.min ? : 0,
			ts_dev->x.max ? : MAX_12BIT,
			0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			ts_dev->y.min ? : 0,
			ts_dev->y.max ? : MAX_12BIT,
			0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, MAX_12BIT, 0, 0);

	/* register to the input system */
	err = input_register_device(input_dev);
	if (err)
		goto err_free_irq;

	platform_set_drvdata(pdev, ts_dev);
	return 0;

err_free_irq:
	free_irq(ts_dev->irq, ts_dev);
err_fail:
	input_free_device(ts_dev->input);
err_free_mem:
	platform_set_drvdata(pdev, NULL);
	kfree(ts_dev);
	return err;
}

static int __devexit tscadc_remove(struct platform_device *pdev)
{
	struct ti_tscadc_dev	*tscadc_dev = pdev->dev.platform_data;
	struct tscadc		*ts_dev = tscadc_dev->tsc;

	free_irq(ts_dev->irq, ts_dev);

	input_unregister_device(ts_dev->input);
	kfree(ts_dev);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver ti_tsc_driver = {
	.probe	  = tscadc_probe,
	.remove	 = __devexit_p(tscadc_remove),
	.driver	 = {
		.name   = "tsc",
		.owner  = THIS_MODULE,
	},
};

static int __init ti_tsc_init(void)
{
	return platform_driver_register(&ti_tsc_driver);
}
module_init(ti_tsc_init);

static void __exit ti_tsc_exit(void)
{
	platform_driver_unregister(&ti_tsc_driver);
}
module_exit(ti_tsc_exit);

MODULE_DESCRIPTION("TI touchscreen controller driver");
MODULE_AUTHOR("Rachna Patil <rachna@ti.com>");
MODULE_LICENSE("GPL");
