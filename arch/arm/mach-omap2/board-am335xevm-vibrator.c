/*
 * Code for Vibrator (Haptics) on AM335X EVM.
 *
 * Copyright (C) 2012 Texas Instruments, Inc. - http://www.ti.com/
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
#include <linux/hrtimer.h>
#include <linux/wakelock.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include "mux.h"

#include <../../../drivers/staging/android/timed_output.h>

/*Vibrator GPIO Number */
#define VIBRATOR_GPIO	52	/* GPIO1_20, GPMC_A4 */

#define MAX_TIMEOUT	10000	/* 10s */

static struct work_struct vibrator_work;

static int vibe_state;

static struct vibrator {
    struct wake_lock wklock;
    struct hrtimer timer;
    struct mutex lock;
    unsigned gpio_en;
} vibdata;

static void update_vibrator(struct work_struct *work)
{
    while (1) {
        if (vibe_state) {
            gpio_set_value(vibdata.gpio_en, 1);
            msleep_interruptible(1);

            gpio_set_value(vibdata.gpio_en, 0);
            msleep_interruptible(1);
        }
        else {
            gpio_set_value(vibdata.gpio_en, 0);

            break;
        }
    }
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
    if (hrtimer_active(&vibdata.timer)) {
        ktime_t r = hrtimer_get_remaining(&vibdata.timer);
        return ktime_to_ms(r);
    }

    return 0;
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
    mutex_lock(&vibdata.lock);

    hrtimer_cancel(&vibdata.timer);

    if (value) {
        wake_lock(&vibdata.wklock);

        vibe_state = 1;

        if (value > MAX_TIMEOUT)
            value = MAX_TIMEOUT;

        hrtimer_start(&vibdata.timer, ns_to_ktime((u64)value * NSEC_PER_MSEC),
            HRTIMER_MODE_REL);
    }
    else {
        wake_unlock(&vibdata.wklock);

        vibe_state = 0;
    }

    mutex_unlock(&vibdata.lock);

    schedule_work(&vibrator_work);
}

static struct timed_output_dev to_dev = {
    .name	= "vibrator",
    .get_time	= vibrator_get_time,
    .enable	= vibrator_enable,
};

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
    wake_unlock(&vibdata.wklock);

    vibe_state = 0;

    schedule_work(&vibrator_work);

    return HRTIMER_NORESTART;
}

static int __init vibrator_init(void)
{
    int ret;

    hrtimer_init(&vibdata.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    vibdata.timer.function = vibrator_timer_func;

    wake_lock_init(&vibdata.wklock, WAKE_LOCK_SUSPEND, "vibrator");
    mutex_init(&vibdata.lock);

    ret = timed_output_dev_register(&to_dev);
    if (ret < 0) {
        pr_err("Unable to register Vibrator as timed_output driver\n");

        mutex_destroy(&vibdata.lock);
        wake_lock_destroy(&vibdata.wklock);

        return -1;
    }

    return 0;
}

int __init am335xevm_vibrator_init(void)
{
    int ret;

    INIT_WORK(&vibrator_work, update_vibrator);

    vibe_state = 0;

    vibdata.gpio_en = VIBRATOR_GPIO;

    omap_mux_init_signal("gpmc_a4.gpio1_20", OMAP_MUX_MODE7 | OMAP_PIN_OUTPUT);

    ret = gpio_request(vibdata.gpio_en, "vibrator-en");
    if (ret) {
        pr_err("Unable to request GPIO for Vibrator driver\n");

        return ret;
    }

    gpio_direction_output(vibdata.gpio_en, 0);

    ret = vibrator_init();

    return ret;
}
EXPORT_SYMBOL(am335xevm_vibrator_init);
