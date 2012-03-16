/*
 *  tsl2550.c - Linux kernel modules for ambient light sensor
 *
 *  Copyright (C) 2007 Rodolfo Giometti <giometti@linux.it>
 *  Copyright (C) 2007 Eurotech S.p.A. <info@eurotech.it>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/wakelock.h>

#define TSL2550_DRV_NAME	"tsl2550"
#define DRIVER_VERSION		"1.2"

/*
 * Defines
 */

#define TSL2550_POWER_DOWN		0x00
#define TSL2550_POWER_UP		0x03
#define TSL2550_STANDARD_RANGE		0x18
#define TSL2550_EXTENDED_RANGE		0x1d
#define TSL2550_READ_ADC0		0x43
#define TSL2550_READ_ADC1		0x83

/* start time delay for light sensor in nano seconds */
#define LIGHT_SENSOR_START_TIME_DELAY 50000000

#define BUFFER_NUM	6

#define tsl2550_dbgmsg(str, args...) pr_debug("%s: " str, __func__, ##args)

/*
 * Structs
 */

struct tsl2550_data {
	struct i2c_client *client;
	struct input_dev *light_input_dev;
	struct mutex update_lock;
	struct workqueue_struct *wq;
	struct wake_lock prx_wake_lock;
	struct work_struct work_light;
	struct hrtimer timer;
	ktime_t light_poll_delay;
	int lux_value_buf[BUFFER_NUM];
	int index_count;
	bool buf_initialized;

	unsigned int power_state:1;
	unsigned int operating_mode:1;
	unsigned int enable:1;
};

/*
 * Global data
 */

static const u8 TSL2550_MODE_RANGE[2] = {
	TSL2550_STANDARD_RANGE, TSL2550_EXTENDED_RANGE,
};

/*
 * Management functions
 */

static int tsl2550_set_operating_mode(struct i2c_client *client, int mode)
{
	struct tsl2550_data *data = i2c_get_clientdata(client);

	int ret = i2c_smbus_write_byte(client, TSL2550_MODE_RANGE[mode]);

	data->operating_mode = mode;

	return ret;
}

static int tsl2550_set_power_state(struct i2c_client *client, int state)
{
	struct tsl2550_data *data = i2c_get_clientdata(client);
	int ret;

	if (state == 0)
		ret = i2c_smbus_write_byte(client, TSL2550_POWER_DOWN);
	else {
		ret = i2c_smbus_write_byte(client, TSL2550_POWER_UP);

		/* On power up we should reset operating mode also... */
		tsl2550_set_operating_mode(client, data->operating_mode);
	}

	data->power_state = state;

	return ret;
}

static int tsl2550_get_adc_value(struct i2c_client *client, u8 cmd)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, cmd);
	if (ret < 0)
		return ret;
	if (!(ret & 0x80))
		return -EAGAIN;
	return ret & 0x7f;	/* remove the "valid" bit */
}

/*
 * LUX calculation
 */

#define	TSL2550_MAX_LUX		1846

static const u8 ratio_lut[] = {
	100, 100, 100, 100, 100, 100, 100, 100,
	100, 100, 100, 100, 100, 100, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 98, 98, 98, 98, 98,
	98, 98, 97, 97, 97, 97, 97, 96,
	96, 96, 96, 95, 95, 95, 94, 94,
	93, 93, 93, 92, 92, 91, 91, 90,
	89, 89, 88, 87, 87, 86, 85, 84,
	83, 82, 81, 80, 79, 78, 77, 75,
	74, 73, 71, 69, 68, 66, 64, 62,
	60, 58, 56, 54, 52, 49, 47, 44,
	42, 41, 40, 40, 39, 39, 38, 38,
	37, 37, 37, 36, 36, 36, 35, 35,
	35, 35, 34, 34, 34, 34, 33, 33,
	33, 33, 32, 32, 32, 32, 32, 31,
	31, 31, 31, 31, 30, 30, 30, 30,
	30,
};

static const u16 count_lut[] = {
	0, 1, 2, 3, 4, 5, 6, 7,
	8, 9, 10, 11, 12, 13, 14, 15,
	16, 18, 20, 22, 24, 26, 28, 30,
	32, 34, 36, 38, 40, 42, 44, 46,
	49, 53, 57, 61, 65, 69, 73, 77,
	81, 85, 89, 93, 97, 101, 105, 109,
	115, 123, 131, 139, 147, 155, 163, 171,
	179, 187, 195, 203, 211, 219, 227, 235,
	247, 263, 279, 295, 311, 327, 343, 359,
	375, 391, 407, 423, 439, 455, 471, 487,
	511, 543, 575, 607, 639, 671, 703, 735,
	767, 799, 831, 863, 895, 927, 959, 991,
	1039, 1103, 1167, 1231, 1295, 1359, 1423, 1487,
	1551, 1615, 1679, 1743, 1807, 1871, 1935, 1999,
	2095, 2223, 2351, 2479, 2607, 2735, 2863, 2991,
	3119, 3247, 3375, 3503, 3631, 3759, 3887, 4015,
};

/*
 * This function is described into Taos TSL2550 Designer's Notebook
 * pages 2, 3.
 */
static int tsl2550_calculate_lux(u8 ch0, u8 ch1)
{
	unsigned int lux;

	/* Look up count from channel values */
	u16 c0 = count_lut[ch0];
	u16 c1 = count_lut[ch1];

	/*
	 * Calculate ratio.
	 * Note: the "128" is a scaling factor
	 */
	u8 r = 128;

	/* Avoid division by 0 and count 1 cannot be greater than count 0 */
	if (c1 <= c0)
		if (c0) {
			r = c1 * 128 / c0;

			/* Calculate LUX */
			lux = ((c0 - c1) * ratio_lut[r]) / 256;
		} else
			lux = 0;
	else
		return -EAGAIN;

	/* LUX range check */
	return lux > TSL2550_MAX_LUX ? TSL2550_MAX_LUX : lux;
}

static void tsl2550_light_enable(struct tsl2550_data *data)
{
	tsl2550_dbgmsg("starting poll timer, delay %lldns\n",
		    ktime_to_ns(data->light_poll_delay));
	/* push -1 to input subsystem to enable real value to go through next */
	input_report_abs(data->light_input_dev, ABS_MISC, -1);
	hrtimer_start(&data->timer, ktime_set(0, LIGHT_SENSOR_START_TIME_DELAY),
					HRTIMER_MODE_REL);
}

static void tsl2550_light_disable(struct tsl2550_data *data)
{
	tsl2550_dbgmsg("cancelling poll timer\n");
	hrtimer_cancel(&data->timer);
	cancel_work_sync(&data->work_light);
	/* mark the adc buff as not initialized
	 * so that it will be filled again on next light sensor start
	 */
	data->buf_initialized = false;
}

/*
 * SysFS support
 */

static ssize_t tsl2550_show_power_state(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tsl2550_data *data = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "%u\n", data->power_state);
}

static ssize_t tsl2550_store_power_state(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct tsl2550_data *data = dev_get_drvdata(dev);
	int ret;
	bool new_value;

	if (sysfs_streq(buf, "1"))
		new_value = true;
	else if (sysfs_streq(buf, "0"))
		new_value = false;
	else {
		pr_err("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	mutex_lock(&data->update_lock);
	ret = tsl2550_set_power_state(data->client, new_value);
	/* Save power state for suspend/resume */
	data->enable = new_value;
	mutex_unlock(&data->update_lock);

	if (ret < 0)
		return ret;

	if (new_value)
		tsl2550_light_enable(data);
	else
		tsl2550_light_disable(data);

	return count;
}

static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO | S_IWGRP,
		   tsl2550_show_power_state, tsl2550_store_power_state);

static ssize_t poll_delay_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct tsl2550_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%lld\n", ktime_to_ns(data->light_poll_delay));
}


static ssize_t poll_delay_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tsl2550_data *data = dev_get_drvdata(dev);
	int64_t new_delay;
	int err;

	err = strict_strtoll(buf, 10, &new_delay);
	if (err < 0)
		return err;

	tsl2550_dbgmsg("new delay = %lldns, old delay = %lldns\n",
		    new_delay, ktime_to_ns(data->light_poll_delay));
	mutex_lock(&data->update_lock);
	if (new_delay != ktime_to_ns(data->light_poll_delay)) {
		data->light_poll_delay = ns_to_ktime(new_delay);
		if (data->power_state) {
			tsl2550_light_disable(data);
			tsl2550_light_enable(data);
		}
	}
	mutex_unlock(&data->update_lock);

	return size;
}

static DEVICE_ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
		   poll_delay_show, poll_delay_store);

static ssize_t tsl2550_show_operating_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tsl2550_data *data = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "%u\n", data->operating_mode);
}

static ssize_t tsl2550_store_operating_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tsl2550_data *data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	if (val < 0 || val > 1)
		return -EINVAL;

	if (data->power_state == 0)
		return -EBUSY;

	mutex_lock(&data->update_lock);
	ret = tsl2550_set_operating_mode(client, val);
	mutex_unlock(&data->update_lock);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(operating_mode, S_IWUSR | S_IRUGO,
		   tsl2550_show_operating_mode, tsl2550_store_operating_mode);

static int __tsl2550_show_lux(struct i2c_client *client)
{
	struct tsl2550_data *data = i2c_get_clientdata(client);
	u8 ch0, ch1;
	int ret;

	ret = tsl2550_get_adc_value(client, TSL2550_READ_ADC0);

	if (ret < 0)
		return ret;
	ch0 = ret;

	ret = tsl2550_get_adc_value(client, TSL2550_READ_ADC1);

	if (ret < 0)
		return ret;
	ch1 = ret;

	/* Do the job */
	ret = tsl2550_calculate_lux(ch0, ch1);
	if (ret < 0)
		return ret;
	if (data->operating_mode == 1)
		ret *= 5;

	return ret;
}

static struct attribute *tsl2550_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_poll_delay.attr,
	&dev_attr_operating_mode.attr,
	NULL
};

static const struct attribute_group tsl2550_attr_group = {
	.attrs = tsl2550_attributes,
};

static int lightsensor_get_luxvalue(struct tsl2550_data *data)
{
	int i = 0;
	int j = 0;
	unsigned int lux_total = 0;
	int lux_avr_value;
	unsigned int index = 0;
	unsigned int lux_max = 0;
	unsigned int lux_min = 0;
	int value = 0;

	/* get lux value */
	mutex_lock(&data->update_lock);
	value = __tsl2550_show_lux(data->client);
	mutex_unlock(&data->update_lock);

	if (value < 0) {
		pr_err("lightsensor returned error %d\n", value);
		return value;
	}
	tsl2550_dbgmsg("light value %d\n", value);

	index = (data->index_count++) % BUFFER_NUM;

	/* buffer initialize (light sensor off ---> light sensor on) */
	if (!data->buf_initialized) {
		data->buf_initialized = true;
		for (j = 0; j < BUFFER_NUM; j++)
			data->lux_value_buf[j] = value;
	} else
		data->lux_value_buf[index] = value;

	lux_max = data->lux_value_buf[0];
	lux_min = data->lux_value_buf[0];

	for (i = 0; i < BUFFER_NUM; i++) {
		lux_total += data->lux_value_buf[i];

		if (lux_max < data->lux_value_buf[i])
			lux_max = data->lux_value_buf[i];

		if (lux_min > data->lux_value_buf[i])
			lux_min = data->lux_value_buf[i];
	}
	lux_avr_value = (lux_total-(lux_max+lux_min))/(BUFFER_NUM-2);

	if (data->index_count == BUFFER_NUM)
		data->index_count = 0;

	tsl2550_dbgmsg("average light value %d\n", lux_avr_value);
	return lux_avr_value;
}

static void tsl2550_work_func_light(struct work_struct *work)
{
	struct tsl2550_data *data = container_of(work, struct tsl2550_data,
					      work_light);

	int adc = lightsensor_get_luxvalue(data);
	if (adc >= 0) {
		input_report_abs(data->light_input_dev, ABS_MISC, adc);
		input_sync(data->light_input_dev);
	}
}

/* This function is for light sensor.  It operates every a few seconds.
 * It asks for work to be done on a thread because i2c needs a thread
 * context (slow and blocking) and then reschedules the timer to run again.
 */
static enum hrtimer_restart tsl2550_timer_func(struct hrtimer *timer)
{
	struct tsl2550_data *data = container_of(timer, struct tsl2550_data, timer);
	queue_work(data->wq, &data->work_light);
	hrtimer_forward_now(&data->timer, data->light_poll_delay);
	return HRTIMER_RESTART;
}



/*
 * Initialization function
 */

static int tsl2550_init_client(struct i2c_client *client)
{
	struct tsl2550_data *data = i2c_get_clientdata(client);
	int err;

	/*
	 * Probe the chip. To do so we try to power up the device and then to
	 * read back the 0x03 code
	 */
	err = i2c_smbus_read_byte_data(client, TSL2550_POWER_UP);
	if (err < 0)
		return err;
	if (err != TSL2550_POWER_UP)
		return -ENODEV;
	data->power_state = 1;

	/* Set the default operating mode */
	err = i2c_smbus_write_byte(client,
				   TSL2550_MODE_RANGE[data->operating_mode]);
	if (err < 0)
		return err;

	return 0;
}

/*
 * I2C init/probing/exit functions
 */

static struct i2c_driver tsl2550_driver;
static int __devinit tsl2550_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct tsl2550_data *data;
	struct input_dev *input_dev;
	int *opmode, err = -ENODEV;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WRITE_BYTE
					    | I2C_FUNC_SMBUS_READ_BYTE_DATA)) {
		err = -EIO;
		goto exit;
	}

	data = kzalloc(sizeof(struct tsl2550_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		pr_err("%s: failed to alloc memory for module data\n",
		       __func__);
		return -ENOMEM;
	}
	data->client = client;
	i2c_set_clientdata(client, data);

	/* Check platform data */
	opmode = client->dev.platform_data;
	if (opmode) {
		if (*opmode < 0 || *opmode > 1) {
			dev_err(&client->dev, "invalid operating_mode (%d)\n",
					*opmode);
			err = -EINVAL;
			goto exit_kfree;
		}
		data->operating_mode = *opmode;
	} else
		data->operating_mode = 0;	/* default mode is standard */
	dev_info(&client->dev, "%s operating mode\n",
			data->operating_mode ? "extended" : "standard");

	/* Initialize the TSL2550 chip */
	err = tsl2550_init_client(client);
	if (err)
		goto exit_kfree;

	wake_lock_init(&data->prx_wake_lock, WAKE_LOCK_SUSPEND,
		"prx_wake_lock");
	mutex_init(&data->update_lock);

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &tsl2550_attr_group);
	if (err)
		goto exit_kfree;

	/* hrtimer settings.  we poll for light values using a timer. */
	hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->light_poll_delay = ns_to_ktime(200 * NSEC_PER_MSEC);
	data->timer.function = tsl2550_timer_func;

	/* the timer just fires off a work queue request.  we need a thread
	 * to read the i2c (can be slow and blocking)
	 */
	data->wq = create_singlethread_workqueue("tsl2550_wq");
	if (!data->wq) {
		err = -ENOMEM;
		pr_err("%s: could not create workqueue\n", __func__);
		goto err_create_workqueue;
	}
	/* this is the thread function we run on the work queue */
	INIT_WORK(&data->work_light, tsl2550_work_func_light);

	/* allocate lightsensor-level input_device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("%s: could not allocate input device\n", __func__);
		err = -ENOMEM;
		goto err_input_allocate_device_light;
	}
	input_set_drvdata(input_dev, data);
	input_dev->name = "lightsensor-level";
	input_set_capability(input_dev, EV_ABS, ABS_MISC);
	input_set_abs_params(input_dev, ABS_MISC, 0, 1, 0, 0);

	dev_info(&client->dev, "registering lightsensor-level input device\n");
	err = input_register_device(input_dev);
	if (err < 0) {
		pr_err("%s: could not register input device\n", __func__);
		input_free_device(input_dev);
		goto err_input_register_device_light;
	}
	data->light_input_dev = input_dev;
	err = sysfs_create_group(&input_dev->dev.kobj,
				 &tsl2550_attr_group);
	if (err) {
		pr_err("%s: could not create sysfs group\n", __func__);
		goto err_sysfs_create_group_light;
	}

	dev_info(&client->dev, "support ver. %s enabled\n", DRIVER_VERSION);

	return 0;
	/* error, unwind it all */
err_sysfs_create_group_light:
	input_unregister_device(data->light_input_dev);
err_input_register_device_light:
err_input_allocate_device_light:
	destroy_workqueue(data->wq);
err_create_workqueue:
	mutex_destroy(&data->update_lock);
	wake_lock_destroy(&data->prx_wake_lock);
exit_kfree:
	kfree(data);
exit:
	return err;
}

static int __devexit tsl2550_remove(struct i2c_client *client)
{
	struct tsl2550_data *data = i2c_get_clientdata(client);
	sysfs_remove_group(&data->light_input_dev->dev.kobj,
			   &tsl2550_attr_group);

	destroy_workqueue(data->wq);
	input_unregister_device(data->light_input_dev);

	/* Power down the device */
	tsl2550_set_power_state(client, 0);

	mutex_destroy(&data->update_lock);
	wake_lock_destroy(&data->prx_wake_lock);

	kfree(i2c_get_clientdata(client));

	return 0;
}

#ifdef CONFIG_PM

static int tsl2550_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct tsl2550_data *data = i2c_get_clientdata(client);

	if (data->enable)
		tsl2550_light_disable(data);

	return tsl2550_set_power_state(client, 0);
}

static int tsl2550_resume(struct i2c_client *client)
{
	int ret;
	struct tsl2550_data *data = i2c_get_clientdata(client);

	ret = tsl2550_set_power_state(client, 1);
	if (ret) {
		/* re-enable input events if required */
		if(data->enable)
			tsl2550_light_enable(data);
	}

	return ret;
}

#else

#define tsl2550_suspend		NULL
#define tsl2550_resume		NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id tsl2550_id[] = {
	{ "tsl2550", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tsl2550_id);

static struct i2c_driver tsl2550_driver = {
	.driver = {
		.name	= TSL2550_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.suspend = tsl2550_suspend,
	.resume	= tsl2550_resume,
	.probe	= tsl2550_probe,
	.remove	= __devexit_p(tsl2550_remove),
	.id_table = tsl2550_id,
};

static int __init tsl2550_init(void)
{
	return i2c_add_driver(&tsl2550_driver);
}

static void __exit tsl2550_exit(void)
{
	i2c_del_driver(&tsl2550_driver);
}

MODULE_AUTHOR("Rodolfo Giometti <giometti@linux.it>");
MODULE_DESCRIPTION("TSL2550 ambient light sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(tsl2550_init);
module_exit(tsl2550_exit);
