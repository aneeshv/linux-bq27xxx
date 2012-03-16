/*
 * lm75.c - Part of lm_sensors, Linux kernel modules for hardware
 *	 monitoring
 * Copyright (c) 1998, 1999  Frodo Looijaard <frodol@dds.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include "lm75.h"

#define LM75_DEBUG  0

static struct input_dev *lm75_input_dev;

/* Used globally in current driver */
static int lm75_timeout;
static bool lm75_enabled;

/* Workqueue related definitions */
static struct workqueue_struct *lm75_work_q;
static struct delayed_work lm75_delayed_work;

/* */
struct device *lm75_i2c_dev;

/*
 * This driver handles the LM75 and compatible digital temperature sensors.
 */

enum lm75_type {		/* keep sorted in alphabetical order */
	adt75,
	ds1775,
	ds75,
	lm75,
	lm75a,
	max6625,
	max6626,
	mcp980x,
	stds75,
	tcn75,
	tmp100,
	tmp101,
	tmp105,
	tmp175,
	tmp275,
	tmp75,
};

/* Addresses scanned */
static const unsigned short normal_i2c[] = { 0x48, 0x49, 0x4a, 0x4b, 0x4c,
					0x4d, 0x4e, 0x4f, I2C_CLIENT_END };


/* The LM75 registers */
#define LM75_REG_CONF		0x01
static const u8 LM75_REG_TEMP[3] = {
	0x00,		/* input */
	0x03,		/* max */
	0x02,		/* hyst */
};

/* Each client has this additional data */
struct lm75_data {
	struct device		*hwmon_dev;
	struct mutex		update_lock;
	u8			orig_conf;
	char			valid;		/* !=0 if registers are valid */
	unsigned long		last_updated;	/* In jiffies */
	u16			temp[3];	/* Register values,
						   0 = input
						   1 = max
						   2 = hyst */
};

static int lm75_read_value(struct i2c_client *client, u8 reg);
static int lm75_write_value(struct i2c_client *client, u8 reg, u16 value);
static struct lm75_data *lm75_update_device(struct device *dev);



static int lm75_work(void)
{
	int temperature = 0;

	struct lm75_data *data = lm75_update_device(lm75_i2c_dev);

	temperature = LM75_TEMP_FROM_REG(data->temp[0]);

#if LM75_DEBUG
	printk(KERN_DEBUG "lm75_work called!! temp=%d\n", temperature);
#endif

	input_report_abs(lm75_input_dev, ABS_MISC, temperature);
	input_sync(lm75_input_dev);

	if (lm75_enabled) {

#if LM75_DEBUG
		printk(KERN_DEBUG "lm75 queuing at delay= %u\n", lm75_timeout);
#endif
		queue_delayed_work(lm75_work_q,
				   &lm75_delayed_work,
				   lm75_timeout);
	}

	return 0;
}

/*-----------------------------------------------------------------------*/
/* sysfs attributes for input-device */

static ssize_t lm75_poll_delay_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	long int delay_to_report;

	/* millisecs to nanosecs */
	delay_to_report = jiffies_to_msecs(lm75_timeout*1000000);

#if LM75_DEBUG
	printk(KERN_DEBUG "delay_to_report=%ld\n", delay_to_report);
#endif
	return snprintf(buf, PAGE_SIZE, "%ld\n", delay_to_report);
}


static ssize_t lm75_poll_delay_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf,
				     size_t size)
{
	int err;
	long int new_delay;

#if LM75_DEBUG
	printk(KERN_DEBUG "buf=%s\n", buf);
	printk(KERN_DEBUG "size=%u\n", size);
#endif

	err = kstrtol(buf, 10, &new_delay);
	if (err < 0)
		return err;

	/* nanosecs to millisecs */
	lm75_timeout = msecs_to_jiffies(new_delay/1000000);


#ifdef LM75_DEBUG
	printk(KERN_DEBUG "poll_delay_store new_delay=%ld\n", new_delay);
	printk(KERN_DEBUG "poll_delay_store lm75_timeout=%d\n", lm75_timeout);
#endif
	return size;
}


static ssize_t lm75_enable_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
#ifdef LM75_DEBUG
	printk(KERN_DEBUG "enabled=%d\n", lm75_enabled);
#endif
	return snprintf(buf, PAGE_SIZE, "%d\n", lm75_enabled);
}


static ssize_t lm75_enable_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf,
				 size_t size)
{
	bool new_value;

	if (sysfs_streq(buf, "1"))
		new_value = true;
	else if (sysfs_streq(buf, "0"))
		new_value = false;
	else {
		printk(KERN_DEBUG "%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

#if LM75_DEBUG
	printk(KERN_DEBUG "new=%d,old=%d\n", new_value, lm75_enabled);
#endif

	if (new_value && !lm75_enabled) {
		lm75_enabled = true;
#if LM75_DEBUG
		printk(KERN_DEBUG "ENABLING TEMP SENSOR\n\n");
#endif
		queue_delayed_work(lm75_work_q,
				   &lm75_delayed_work,
				   lm75_timeout);

	} else if (!new_value && lm75_enabled) {
		lm75_enabled = false;
#if LM75_DEBUG
		printk(KERN_DEBUG "DISABLING TEMP SENSOR\n\n");
#endif
		flush_workqueue(lm75_work_q);
		cancel_delayed_work(&lm75_delayed_work);
	}

	return size;
}


static struct device_attribute dev_attr_lm75_poll_delay =
	__ATTR(poll_delay, S_IRUGO | S_IWUGO | S_IXUGO,
	       lm75_poll_delay_show, lm75_poll_delay_store);

static struct device_attribute dev_attr_lm75_enable =
	__ATTR(enable, S_IRUGO | S_IWUGO | S_IXUGO,
	       lm75_enable_show, lm75_enable_store);


static struct attribute *lm75_sysfs_attrs[] = {
	&dev_attr_lm75_enable.attr,
	&dev_attr_lm75_poll_delay.attr,
	NULL
};

static struct attribute_group lm75_attribute_group = {
	.attrs = lm75_sysfs_attrs,
};


/*-----------------------------------------------------------------------*/

/* sysfs attributes for hwmon */

static ssize_t show_temp(struct device *dev, struct device_attribute *da,
			 char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	struct lm75_data *data = lm75_update_device(dev);
	return sprintf(buf, "%d\n",
		       LM75_TEMP_FROM_REG(data->temp[attr->index]));
}

static ssize_t set_temp(struct device *dev, struct device_attribute *da,
			const char *buf, size_t count)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	struct i2c_client *client = to_i2c_client(dev);
	struct lm75_data *data = i2c_get_clientdata(client);
	int nr = attr->index;
	long temp;
	int error;

	error = strict_strtol(buf, 10, &temp);
	if (error)
		return error;

	mutex_lock(&data->update_lock);
	data->temp[nr] = LM75_TEMP_TO_REG(temp);
	lm75_write_value(client, LM75_REG_TEMP[nr], data->temp[nr]);
	mutex_unlock(&data->update_lock);
	return count;
}

static SENSOR_DEVICE_ATTR(temp1_max, S_IWUSR | S_IRUGO,
			show_temp, set_temp, 1);
static SENSOR_DEVICE_ATTR(temp1_max_hyst, S_IWUSR | S_IRUGO,
			show_temp, set_temp, 2);
static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, show_temp, NULL, 0);

static struct attribute *lm75_attributes[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp1_max.dev_attr.attr,
	&sensor_dev_attr_temp1_max_hyst.dev_attr.attr,

	NULL
};

static const struct attribute_group lm75_group = {
	.attrs = lm75_attributes,
};

/*-----------------------------------------------------------------------*/

/* device probe and removal */

static int
lm75_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct lm75_data *data;
	int status;
	u8 set_mask, clr_mask;
	int new;

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA))
		return -EIO;

	data = kzalloc(sizeof(struct lm75_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	i2c_set_clientdata(client, data);
	mutex_init(&data->update_lock);

	/* Set to LM75 resolution (9 bits, 1/2 degree C) and range.
	 * Then tweak to be more precise when appropriate.
	 */
	set_mask = 0;
	clr_mask = (1 << 0)			/* continuous conversions */
		| (1 << 6) | (1 << 5);		/* 9-bit mode */

	/* configure as specified */
	status = lm75_read_value(client, LM75_REG_CONF);
	if (status < 0) {
		dev_dbg(&client->dev, "Can't read config? %d\n", status);
		goto exit_free;
	}
	data->orig_conf = status;
	new = status & ~clr_mask;
	new |= set_mask;
	if (status != new)
		lm75_write_value(client, LM75_REG_CONF, new);
	dev_dbg(&client->dev, "Config %02x\n", new);

	/* Register sysfs hooks */
	status = sysfs_create_group(&client->dev.kobj, &lm75_group);
	if (status)
		goto exit_free;

	/* allocate lightsensor-level input_device */
	lm75_input_dev = input_allocate_device();
	if (!lm75_input_dev) {
		pr_err("%s: could not allocate input device\n", __func__);
		status = -ENOMEM;
		goto exit_input_allocate_device_temp;
	}
	set_bit(EV_ABS, lm75_input_dev->evbit);
	set_bit(ABS_MISC, lm75_input_dev->absbit);

	lm75_input_dev->name = "lm75-temp";
	input_set_abs_params(lm75_input_dev, ABS_MISC, -55000, 125000, 0, 0);

	status = input_register_device(lm75_input_dev);
	if (status) {

		printk(KERN_DEBUG "lm75 registration failed: %d\n", status);
		goto exit_release_input_dev;
	}

	status = sysfs_create_group(&lm75_input_dev->dev.kobj,
				    &lm75_attribute_group);
	if (status) {
		printk(KERN_DEBUG "%s: could not create sysfs\n", __func__);
		goto exit_release_input_dev;
	}

	/* default poll-rate is 200ms */
	lm75_timeout = msecs_to_jiffies(200);

	/* Workqueue Initialisation */
	lm75_work_q = create_singlethread_workqueue("lm75_work_queue");
	INIT_DELAYED_WORK((struct delayed_work *)&lm75_delayed_work, lm75_work);


	data->hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(data->hwmon_dev)) {
		status = PTR_ERR(data->hwmon_dev);
		goto exit_remove;
	}

	lm75_i2c_dev = &client->dev;

	dev_info(&client->dev, "%s: sensor '%s'\n",
		 dev_name(data->hwmon_dev), client->name);

	return 0;

exit_release_input_dev:
	input_free_device(lm75_input_dev);
exit_input_allocate_device_temp:

exit_remove:
	sysfs_remove_group(&client->dev.kobj, &lm75_group);
exit_free:
	kfree(data);
	return status;
}

static int lm75_remove(struct i2c_client *client)
{
	struct lm75_data *data = i2c_get_clientdata(client);

	hwmon_device_unregister(data->hwmon_dev);
	sysfs_remove_group(&client->dev.kobj, &lm75_group);
	lm75_write_value(client, LM75_REG_CONF, data->orig_conf);
	kfree(data);
	return 0;
}

static const struct i2c_device_id lm75_ids[] = {
	{ "adt75", adt75, },
	{ "ds1775", ds1775, },
	{ "ds75", ds75, },
	{ "lm75", lm75, },
	{ "lm75a", lm75a, },
	{ "max6625", max6625, },
	{ "max6626", max6626, },
	{ "mcp980x", mcp980x, },
	{ "stds75", stds75, },
	{ "tcn75", tcn75, },
	{ "tmp100", tmp100, },
	{ "tmp101", tmp101, },
	{ "tmp105", tmp105, },
	{ "tmp175", tmp175, },
	{ "tmp275", tmp275, },
	{ "tmp75", tmp75, },
	{ /* LIST END */ }
};
MODULE_DEVICE_TABLE(i2c, lm75_ids);

#define LM75A_ID 0xA1

/* Return 0 if detection is successful, -ENODEV otherwise */
static int lm75_detect(struct i2c_client *new_client,
		       struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = new_client->adapter;
	int i;
	int conf, hyst, os;
	bool is_lm75a = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_WORD_DATA))
		return -ENODEV;

	/*
	 * Now, we do the remaining detection. There is no identification-
	 * dedicated register so we have to rely on several tricks:
	 * unused bits, registers cycling over 8-address boundaries,
	 * addresses 0x04-0x07 returning the last read value.
	 * The cycling+unused addresses combination is not tested,
	 * since it would significantly slow the detection down and would
	 * hardly add any value.
	 *
	 * The National Semiconductor LM75A is different than earlier
	 * LM75s.  It has an ID byte of 0xaX (where X is the chip
	 * revision, with 1 being the only revision in existence) in
	 * register 7, and unused registers return 0xff rather than the
	 * last read value.
	 *
	 * Note that this function only detects the original National
	 * Semiconductor LM75 and the LM75A. Clones from other vendors
	 * aren't detected, on purpose, because they are typically never
	 * found on PC hardware. They are found on embedded designs where
	 * they can be instantiated explicitly so detection is not needed.
	 * The absence of identification registers on all these clones
	 * would make their exhaustive detection very difficult and weak,
	 * and odds are that the driver would bind to unsupported devices.
	 */

	/* Unused bits */
	conf = i2c_smbus_read_byte_data(new_client, 1);
	if (conf & 0xe0)
		return -ENODEV;

	/* First check for LM75A */
	if (i2c_smbus_read_byte_data(new_client, 7) == LM75A_ID) {
		/* LM75A returns 0xff on unused registers so
		   just to be sure we check for that too. */
		if (i2c_smbus_read_byte_data(new_client, 4) != 0xff
		 || i2c_smbus_read_byte_data(new_client, 5) != 0xff
		 || i2c_smbus_read_byte_data(new_client, 6) != 0xff)
			return -ENODEV;
		is_lm75a = 1;
		hyst = i2c_smbus_read_byte_data(new_client, 2);
		os = i2c_smbus_read_byte_data(new_client, 3);
	} else { /* Traditional style LM75 detection */
		/* Unused addresses */
		hyst = i2c_smbus_read_byte_data(new_client, 2);
		if (i2c_smbus_read_byte_data(new_client, 4) != hyst
		 || i2c_smbus_read_byte_data(new_client, 5) != hyst
		 || i2c_smbus_read_byte_data(new_client, 6) != hyst
		 || i2c_smbus_read_byte_data(new_client, 7) != hyst)
			return -ENODEV;
		os = i2c_smbus_read_byte_data(new_client, 3);
		if (i2c_smbus_read_byte_data(new_client, 4) != os
		 || i2c_smbus_read_byte_data(new_client, 5) != os
		 || i2c_smbus_read_byte_data(new_client, 6) != os
		 || i2c_smbus_read_byte_data(new_client, 7) != os)
			return -ENODEV;
	}

	/* Addresses cycling */
	for (i = 8; i <= 248; i += 40) {
		if (i2c_smbus_read_byte_data(new_client, i + 1) != conf
		 || i2c_smbus_read_byte_data(new_client, i + 2) != hyst
		 || i2c_smbus_read_byte_data(new_client, i + 3) != os)
			return -ENODEV;
		if (is_lm75a && i2c_smbus_read_byte_data(new_client, i + 7)
				!= LM75A_ID)
			return -ENODEV;
	}

	strlcpy(info->type, is_lm75a ? "lm75a" : "lm75", I2C_NAME_SIZE);

	return 0;
}

#ifdef CONFIG_PM
static int lm75_suspend(struct device *dev)
{
	int status;
	struct i2c_client *client = to_i2c_client(dev);

	if (lm75_enabled) {
		flush_workqueue(lm75_work_q);
		cancel_delayed_work(&lm75_delayed_work);
	}

	status = lm75_read_value(client, LM75_REG_CONF);
	if (status < 0) {
		dev_dbg(&client->dev, "Can't read config? %d\n", status);
		return status;
	}
	status = status | LM75_SHUTDOWN;
	lm75_write_value(client, LM75_REG_CONF, status);
	return 0;
}

static int lm75_resume(struct device *dev)
{
	int status;
	struct i2c_client *client = to_i2c_client(dev);
	status = lm75_read_value(client, LM75_REG_CONF);
	if (status < 0) {
		dev_dbg(&client->dev, "Can't read config? %d\n", status);
		return status;
	}
	status = status & ~LM75_SHUTDOWN;
	lm75_write_value(client, LM75_REG_CONF, status);

	if (lm75_enabled)
		queue_delayed_work(lm75_work_q,
				   &lm75_delayed_work,
				   lm75_timeout);
	return 0;
}

static const struct dev_pm_ops lm75_dev_pm_ops = {
	.suspend	= lm75_suspend,
	.resume		= lm75_resume,
};
#define LM75_DEV_PM_OPS (&lm75_dev_pm_ops)
#else
#define LM75_DEV_PM_OPS NULL
#endif /* CONFIG_PM */

static struct i2c_driver lm75_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= "lm75",
		.pm	= LM75_DEV_PM_OPS,
	},
	.probe		= lm75_probe,
	.remove		= lm75_remove,
	.id_table	= lm75_ids,
	.detect		= lm75_detect,
	.address_list	= normal_i2c,
};

/*-----------------------------------------------------------------------*/

/* register access */

/*
 * All registers are word-sized, except for the configuration register.
 * LM75 uses a high-byte first convention, which is exactly opposite to
 * the SMBus standard.
 */
static int lm75_read_value(struct i2c_client *client, u8 reg)
{
	if (reg == LM75_REG_CONF)
		return i2c_smbus_read_byte_data(client, reg);
	else
		return i2c_smbus_read_word_swapped(client, reg);
}

static int lm75_write_value(struct i2c_client *client, u8 reg, u16 value)
{
	if (reg == LM75_REG_CONF)
		return i2c_smbus_write_byte_data(client, reg, value);
	else
		return i2c_smbus_write_word_swapped(client, reg, value);
}

static struct lm75_data *lm75_update_device(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lm75_data *data = i2c_get_clientdata(client);

	mutex_lock(&data->update_lock);

	if (time_after(jiffies, data->last_updated + HZ + HZ / 2)
	    || !data->valid) {
		int i;
		dev_dbg(&client->dev, "Starting lm75 update\n");

		for (i = 0; i < ARRAY_SIZE(data->temp); i++) {
			int status;

			status = lm75_read_value(client, LM75_REG_TEMP[i]);
			if (status < 0)
				dev_dbg(&client->dev, "reg %d, err %d\n",
						LM75_REG_TEMP[i], status);
			else
				data->temp[i] = status;
		}
		data->last_updated = jiffies;
		data->valid = 1;
	}

	mutex_unlock(&data->update_lock);

	return data;
}

/*-----------------------------------------------------------------------*/

/* module glue */

static int __init sensors_lm75_init(void)
{
	return i2c_add_driver(&lm75_driver);
}

static void __exit sensors_lm75_exit(void)
{
	i2c_del_driver(&lm75_driver);
}

MODULE_AUTHOR("Frodo Looijaard <frodol@dds.nl>");
MODULE_DESCRIPTION("LM75 driver");
MODULE_LICENSE("GPL");

module_init(sensors_lm75_init);
module_exit(sensors_lm75_exit);
