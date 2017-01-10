/*
 * STTS751 sensor driver
 *
 * Copyright (C) 2016-2017 Istituto Italiano di Tecnologia - RBCS - EDL
 * Robotics, Brain and Cognitive Sciences department
 * Electronic Design Laboratory
 *
 * Written by Andrea Merello <andrea.merello@gmail.com>
 *
 * Based on  LM95241 driver and LM90 driver
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
 */

#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/util_macros.h>

#define DEVNAME "stts751"

static const unsigned short normal_i2c[] = {
	0x48, 0x49, 0x38, 0x39,  /* STTS751-0 */
	0x4A, 0x4B, 0x3A, 0x3B,  /* STTS751-1 */
	I2C_CLIENT_END };

#define STTS751_REG_TEMP_H	0x00
#define STTS751_REG_STATUS	0x01
#define STTS751_STATUS_TRIPT	BIT(0)
#define STTS751_STATUS_TRIPL	BIT(5)
#define STTS751_STATUS_TRIPH	BIT(6)
#define STTS751_REG_TEMP_L	0x02
#define STTS751_REG_CONF	0x03
#define STTS751_CONF_RES_MASK	0x0C
#define STTS751_CONF_RES_SHIFT  2
#define STTS751_CONF_EVENT_DIS  BIT(7)
#define STTS751_CONF_STOP	BIT(6)
#define STTS751_REG_RATE	0x04
#define STTS751_REG_HLIM_H	0x05
#define STTS751_REG_HLIM_L	0x06
#define STTS751_REG_LLIM_H	0x07
#define STTS751_REG_LLIM_L	0x08
#define STTS751_REG_TLIM	0x20
#define STTS751_REG_HYST	0x21
#define STTS751_REG_SMBUS_TO	0x22

#define STTS751_REG_PROD_ID	0xFD
#define STTS751_REG_MAN_ID	0xFE
#define STTS751_REG_REV_ID	0xFF

#define STTS751_0_PROD_ID	0x00
#define STTS751_1_PROD_ID	0x01
#define ST_MAN_ID		0x53

/*
 * Possible update intervals are (in mS):
 * 16000, 8000, 4000, 2000, 1000, 500, 250, 125, 62.5, 31.25
 * However we are not going to complicate things too much and we stick to the
 * approx value in mS.
 */
static const int stts751_intervals[] = {
	16000, 8000, 4000, 2000, 1000, 500, 250, 125, 63, 31
};

static const struct i2c_device_id stts751_id[] = {
	{ "stts751", 0 },
	{ }
};

struct stts751_priv {
	struct device *dev;
	struct i2c_client *client;
	struct mutex access_lock;
	u8 interval;
	int res;
	int event_max, event_min;
	int therm;
	int hyst;
	bool smbus_timeout;
	int temp;
	unsigned long last_update, last_alert_update;
	u8 config;
	bool min_alert, max_alert, therm_trip;
	bool data_valid;
};

/*
 * These functions converts temperature from HW format to integer format and
 * vice-vers. They are (mostly) taken from lm90 driver. Unit is in mC.
 */
static int stts751_to_deg(s32 hw_val)
{
	return hw_val * 125 / 32;
}

static s32 stts751_to_hw(int val)
{
	s32 hw_val;

	if (val < 0)
		hw_val = (val - 62) / 125 * 32;
	else
		hw_val = (val + 62) / 125 * 32;

	return hw_val;
}

static int stts751_adjust_resolution(struct stts751_priv *priv)
{
	u8 res;

	switch (priv->interval) {
	case 9:
		/* 10 bits */
		res = 0;
		break;
	case 8:
		/* 11 bits */
		res = 1;
		break;
	default:
		/* 12 bits */
		res = 3;
		break;
	}

	if (priv->res == res)
		return 0;

	priv->config &= ~STTS751_CONF_RES_MASK;
	priv->config |= res << STTS751_CONF_RES_SHIFT;

	return i2c_smbus_write_byte_data(priv->client,
				STTS751_REG_CONF, priv->config);
}

static int stts751_update_temp(struct stts751_priv *priv)
{
	s32 integer1, integer2, frac;
	int ret = 0;

	mutex_lock(&priv->access_lock);

	/*
	 * There is a trick here, like in the lm90 driver. We have to read two
	 * registers to get the sensor temperature, but we have to beware a
	 * conversion could occur between the readings. We could use the
	 * one-shot conversion register, but we don't want to do this (disables
	 * hardware monitoring). So the solution used here is to read the high
	 * byte once, then the low byte, then the high byte again. If the new
	 * high byte matches the old one, then we have a valid reading. Else we
	 * have to read the low byte again, and now we believe we have a correct
	 * reading.
	 */
	integer1 = i2c_smbus_read_byte_data(priv->client, STTS751_REG_TEMP_H);
	if (integer1 < 0) {
		dev_dbg(&priv->client->dev,
			"I2C read failed (temp H). ret: %x\n", ret);
		ret = integer1;
		goto exit;
	}

	frac = i2c_smbus_read_byte_data(priv->client, STTS751_REG_TEMP_L);
	if (frac < 0) {
		dev_dbg(&priv->client->dev,
			"I2C read failed (temp L). ret: %x\n", ret);
		ret = frac;
		goto exit;
	}

	integer2 = i2c_smbus_read_byte_data(priv->client, STTS751_REG_TEMP_H);
	if (integer2 < 0) {
		dev_dbg(&priv->client->dev,
			"I2C 2nd read failed (temp H). ret: %x\n", ret);
		ret = integer2;
		goto exit;
	}

	if (integer1 != integer2) {
		frac = i2c_smbus_read_byte_data(priv->client,
						STTS751_REG_TEMP_L);
		if (frac < 0) {
			dev_dbg(&priv->client->dev,
				"I2C 2nd read failed (temp L). ret: %x\n", ret);
			ret = frac;
			goto exit;
		}
	}

exit:
	mutex_unlock(&priv->access_lock);
	if (ret)
		return ret;

	priv->temp = stts751_to_deg((integer1 << 8) | frac);
	return ret;
}

static int stts751_set_temp_reg16(struct stts751_priv *priv, int temp,
				u8 hreg, u8 lreg)
{
	s32 hwval;
	int ret;

	hwval = stts751_to_hw(temp);

	mutex_lock(&priv->access_lock);
	ret = i2c_smbus_write_byte_data(priv->client, hreg, hwval >> 8);
	if (!ret)
		ret = i2c_smbus_write_byte_data(priv->client, lreg, hwval & 0xff);
	mutex_unlock(&priv->access_lock);

	return ret;
}

static int stts751_set_temp_reg8(struct stts751_priv *priv, int temp, u8 reg)
{
	s32 hwval;
	int ret;

	hwval = stts751_to_hw(temp);

	mutex_lock(&priv->access_lock);
	ret = i2c_smbus_write_byte_data(priv->client, reg, hwval >> 8);
	mutex_unlock(&priv->access_lock);

	return ret;
}

static int stts751_read_reg16(struct stts751_priv *priv, int *temp,
				u8 hreg, u8 lreg)
{
	int integer, frac;

	integer = i2c_smbus_read_byte_data(priv->client, hreg);
	if (integer < 0)
		return integer;

	frac = i2c_smbus_read_byte_data(priv->client, lreg);
	if (frac < 0)
		return frac;

	*temp = stts751_to_deg((integer << 8) | frac);

	return 0;
}

static int stts751_read_reg8(struct stts751_priv *priv, int *temp, u8 reg)
{
	int integer;

	integer = i2c_smbus_read_byte_data(priv->client, reg);
	if (integer < 0)
		return integer;

	*temp = stts751_to_deg(integer << 8);

	return 0;
}

static int stts751_update_alert(struct stts751_priv *priv)
{
	int ret;
	const int cache_time =
		DIV_ROUND_UP(stts751_intervals[priv->interval] * HZ, 1000);

	ret = i2c_smbus_read_byte_data(priv->client, STTS751_REG_STATUS);
	if (ret < 0)
		return ret;

	if (time_after(jiffies,	priv->last_alert_update + cache_time)) {
		priv->max_alert = false;
		priv->min_alert = false;
		priv->therm_trip = false;
		priv->last_alert_update = jiffies;
	}

	priv->max_alert |= !!(ret & STTS751_STATUS_TRIPH);
	priv->min_alert |= !!(ret & STTS751_STATUS_TRIPL);
	priv->therm_trip |= !!(ret & STTS751_STATUS_TRIPT);
	return 0;
}

static void stts751_alert(struct i2c_client *client,
			enum i2c_alert_protocol type, unsigned int data)
{
	int ret;
	struct stts751_priv *priv = i2c_get_clientdata(client);

	if (type != I2C_PROTOCOL_SMBUS_ALERT)
		return;

	dev_dbg(&client->dev, "alert!");

	mutex_lock(&priv->access_lock);
	ret = stts751_update_alert(priv);
	if (ret < 0) {
		/* default to worst case */
		priv->max_alert = true;
		priv->min_alert = true;

		dev_warn(&priv->client->dev,
			"Alert received, but can't communicate to the device. Something bad happening? Triggering all alarms!");
	}

	if (priv->max_alert) {
		dev_notice(&client->dev, "got alert for HIGH temperature");

		/* unblock alert poll */
		sysfs_notify(&priv->dev->kobj, NULL, "temp1_event_max_alert");
		kobject_uevent(&priv->dev->kobj, KOBJ_CHANGE);
	}

	if (priv->min_alert) {
		dev_notice(&client->dev, "got alert for LOW temperature");

		/* unblock alert poll */
		sysfs_notify(&priv->dev->kobj, NULL, "temp1_event_min_alert");
		kobject_uevent(&priv->dev->kobj, KOBJ_CHANGE);
	}
	mutex_unlock(&priv->access_lock);
}

static ssize_t show_max_alert(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	int ret;
	struct stts751_priv *priv = dev_get_drvdata(dev);

	mutex_lock(&priv->access_lock);
	ret = stts751_update_alert(priv);
	mutex_unlock(&priv->access_lock);
	if (ret < 0)
		return ret;

	return snprintf(buf, PAGE_SIZE - 1, "%d\n", priv->max_alert);
}

static ssize_t show_min_alert(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	int ret;
	struct stts751_priv *priv = dev_get_drvdata(dev);

	mutex_lock(&priv->access_lock);
	ret = stts751_update_alert(priv);
	mutex_unlock(&priv->access_lock);
	if (ret < 0)
		return ret;

	return snprintf(buf, PAGE_SIZE - 1, "%d\n", priv->min_alert);
}

static ssize_t show_input(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	int ret;
	struct stts751_priv *priv = dev_get_drvdata(dev);

	/*
	 * Adjust the cache time wrt the sample rate. We do 4X in order to get
	 * a new measure in no more than 1/4 of the sample time (that seemed
	 * reasonable to me).
	 */
	int cache_time = stts751_intervals[priv->interval] / 4 * HZ / 1000;

	if (time_after(jiffies,	priv->last_update + cache_time) ||
		!priv->data_valid) {
		ret = stts751_update_temp(priv);
		if (ret)
			return ret;
		priv->last_update = jiffies;
		priv->data_valid = true;
	}

	return snprintf(buf, PAGE_SIZE - 1, "%d\n", priv->temp);
}

static ssize_t show_therm(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct stts751_priv *priv = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE - 1, "%d\n", priv->therm);
}

static ssize_t set_therm(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	int ret;
	long temp;
	struct stts751_priv *priv = dev_get_drvdata(dev);

	if (kstrtol(buf, 10, &temp) < 0)
		return -EINVAL;
	/* HW works in range -64C to +127.937C */
	temp = clamp_val(temp, -64000, 127937);
	ret = stts751_set_temp_reg8(priv, temp, STTS751_REG_TLIM);
	if (ret)
		return ret;

	dev_dbg(dev, "setting therm %ld", temp);

	priv->therm = temp;
	return count;
}

static ssize_t show_hyst(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct stts751_priv *priv = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE - 1, "%d\n", priv->therm - priv->hyst);
}

static ssize_t set_hyst(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	int ret;
	long temp;

	struct stts751_priv *priv = dev_get_drvdata(dev);

	if (kstrtol(buf, 10, &temp) < 0)
		return -EINVAL;
	/* HW works in range -64C to +127.937C */
	temp = clamp_val(temp, -64000, priv->therm);
	temp = priv->therm - temp;
	ret = stts751_set_temp_reg8(priv, temp, STTS751_REG_HYST);
	if (ret)
		return ret;

	dev_dbg(dev, "setting hyst %ld", temp);

	priv->hyst = temp;
	return count;
}

static ssize_t show_max(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct stts751_priv *priv = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE - 1, "%d\n", priv->event_max);
}

static ssize_t set_max(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	int ret;
	long temp;
	struct stts751_priv *priv = dev_get_drvdata(dev);

	if (kstrtol(buf, 10, &temp) < 0)
		return -EINVAL;
	/* HW works in range -64C to +127.937C */
	temp = clamp_val(temp, priv->event_min, 127937);
	ret = stts751_set_temp_reg16(priv, temp,
				STTS751_REG_HLIM_H, STTS751_REG_HLIM_L);
	if (ret)
		return ret;

	dev_dbg(dev, "setting event max %ld", temp);
	priv->event_max = temp;
	return count;
}

static ssize_t show_min(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct stts751_priv *priv = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE - 1, "%d\n", priv->event_min);
}

static ssize_t set_min(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	int ret;
	long temp;
	struct stts751_priv *priv = dev_get_drvdata(dev);

	if (kstrtol(buf, 10, &temp) < 0)
		return -EINVAL;
	/* HW works in range -64C to +127.937C */
	temp = clamp_val(temp, -64000, priv->event_max);
	ret = stts751_set_temp_reg16(priv, temp,
				STTS751_REG_LLIM_H, STTS751_REG_LLIM_L);
	if (ret)
		return ret;

	dev_dbg(dev, "setting event min %ld", temp);

	priv->event_min = temp;
	return count;
}

static ssize_t show_interval(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct stts751_priv *priv = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE - 1, "%d\n",
			stts751_intervals[priv->interval]);
}

static ssize_t set_interval(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	unsigned long val;
	int idx;
	int ret = 0;
	struct stts751_priv *priv = dev_get_drvdata(dev);

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	idx = find_closest_descending(val, stts751_intervals,
				ARRAY_SIZE(stts751_intervals));

	dev_dbg(dev, "setting interval. req:%lu, idx: %d, val: %d", val, idx,
		stts751_intervals[idx]);

	if (priv->interval == idx)
		return count;

	mutex_lock(&priv->access_lock);

	/*
	 * In early development stages I've become suspicious about the chip
	 * starting to misbehave if I ever set, even briefly, an invalid
	 * configuration. While I'm not sure this is really needed, be
	 * conservative and set rate/resolution in such an order that avoids
	 * passing through an invalid configuration.
	 */

	/* speed up: lower the resolution, then modify convrate */
	if (priv->interval < idx) {
		priv->interval = idx;
		ret = stts751_adjust_resolution(priv);
		if (ret)
			goto exit;
	}

	ret = i2c_smbus_write_byte_data(priv->client, STTS751_REG_RATE, idx);
	if (ret)
		goto exit;
	/* slow down: modify convrate, then raise resolution */
	if (priv->interval != idx) {
		priv->interval = idx;
		ret = stts751_adjust_resolution(priv);
		if (ret)
			goto exit;
	}
exit:
	mutex_unlock(&priv->access_lock);

	return count;
}

static int stts751_detect(struct i2c_client *new_client,
			  struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = new_client->adapter;
	const char *name;
	int mfg_id, prod_id, rev_id;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	mfg_id = i2c_smbus_read_byte_data(new_client, ST_MAN_ID);
	if (mfg_id != ST_MAN_ID)
		return -ENODEV;

	prod_id = i2c_smbus_read_byte_data(new_client, STTS751_REG_PROD_ID);

	switch (prod_id) {
	case STTS751_0_PROD_ID:
		name = "STTS751-0";
		break;
	case STTS751_1_PROD_ID:
		name = "STTS751-1";
		break;
	default:
		return -ENODEV;
	}
	dev_dbg(&new_client->dev, "Chip %s detected!", name);

	rev_id = i2c_smbus_read_byte_data(new_client, STTS751_REG_REV_ID);
	if (rev_id < 0)
		return -ENODEV;
	if (rev_id != 0x1) {
		dev_notice(&new_client->dev,
			"Chip revision 0x%x is untested\nPlease report whether it works to andrea.merello@gmail.com",
			rev_id);
	}

	strlcpy(info->type, stts751_id[0].name, I2C_NAME_SIZE);
	return 0;
}

static int stts751_read_chip_config(struct stts751_priv *priv)
{
	int ret;

	ret = i2c_smbus_read_byte_data(priv->client, STTS751_REG_CONF);
	if (ret < 0)
		return ret;
	priv->config = ret;
	priv->res = (ret & STTS751_CONF_RES_MASK) >> STTS751_CONF_RES_SHIFT;

	ret = i2c_smbus_read_byte_data(priv->client, STTS751_REG_RATE);
	if (ret < 0)
		return ret;
	priv->interval = ret;

	ret = stts751_read_reg16(priv, &priv->event_max,
				STTS751_REG_HLIM_H, STTS751_REG_HLIM_L);
	if (ret)
		return ret;

	ret = stts751_read_reg16(priv, &priv->event_min,
				STTS751_REG_LLIM_H, STTS751_REG_LLIM_L);
	if (ret)
		return ret;

	ret = stts751_read_reg8(priv, &priv->therm, STTS751_REG_TLIM);
	if (ret)
		return ret;

	ret = stts751_read_reg8(priv, &priv->hyst, STTS751_REG_HYST);
	if (ret)
		return ret;

	return ret;
}

static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, show_input, NULL, 0);
static SENSOR_DEVICE_ATTR(temp1_event_min, S_IWUSR | S_IRUGO,
			show_min, set_min, 0);
static SENSOR_DEVICE_ATTR(temp1_event_max, S_IWUSR | S_IRUGO,
			show_max, set_max, 0);
static SENSOR_DEVICE_ATTR(temp1_event_min_alert, S_IRUGO,
			show_min_alert, NULL, 0);
static SENSOR_DEVICE_ATTR(temp1_event_max_alert, S_IRUGO,
			show_max_alert, NULL, 0);
static SENSOR_DEVICE_ATTR(temp1_therm, S_IWUSR | S_IRUGO, show_therm,
			set_therm, 0);
static SENSOR_DEVICE_ATTR(temp1_therm_hyst, S_IWUSR | S_IRUGO, show_hyst,
			set_hyst, 0);
static SENSOR_DEVICE_ATTR(update_interval, S_IWUSR | S_IRUGO,
			show_interval, set_interval, 0);

static struct attribute *stts751_attrs[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_update_interval.dev_attr.attr,
	&sensor_dev_attr_temp1_event_min.dev_attr.attr,
	&sensor_dev_attr_temp1_event_max.dev_attr.attr,
	&sensor_dev_attr_temp1_event_min_alert.dev_attr.attr,
	&sensor_dev_attr_temp1_event_max_alert.dev_attr.attr,
	&sensor_dev_attr_temp1_therm.dev_attr.attr,
	&sensor_dev_attr_temp1_therm_hyst.dev_attr.attr,
	NULL
};
ATTRIBUTE_GROUPS(stts751);

static int stts751_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct stts751_priv *priv;
	int ret;
	bool smbus_to;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client = client;
	i2c_set_clientdata(client, priv);
	mutex_init(&priv->access_lock);

	if (device_property_present(&client->dev,
					"smbus-timeout-disable")) {
		smbus_to = !device_property_read_bool(&client->dev,
						"smbus-timeout-disable");

		ret = i2c_smbus_write_byte_data(priv->client,
						STTS751_REG_SMBUS_TO,
						smbus_to ? 0x80 : 0);
		if (ret)
			return ret;
	}

	ret = stts751_read_chip_config(priv);
	if (ret)
		return ret;

	priv->config &= ~(STTS751_CONF_STOP | STTS751_CONF_EVENT_DIS);
	ret = i2c_smbus_write_byte_data(priv->client,
					STTS751_REG_CONF, priv->config);
	if (ret)
		return ret;

	priv->dev = devm_hwmon_device_register_with_groups(&client->dev,
							client->name, priv,
							stts751_groups);
	return PTR_ERR_OR_ZERO(priv->dev);
}

MODULE_DEVICE_TABLE(i2c, stts751_id);

static struct i2c_driver stts751_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= DEVNAME,
	},
	.probe		= stts751_probe,
	.id_table	= stts751_id,
	.detect		= stts751_detect,
	.alert		= stts751_alert,
	.address_list	= normal_i2c,
};

module_i2c_driver(stts751_driver);

MODULE_AUTHOR("Andrea Merello <andrea.merello@gmail.com>");
MODULE_DESCRIPTION("STTS751 sensor driver");
MODULE_LICENSE("GPL");
