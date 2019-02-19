/* Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/i2c/drv8830.h>

#define PULSE_DURATION_MS 400

struct drv8830_data {
	struct i2c_client *client;
	struct work_struct pulse_work;
	s32 fault_gpio;
	u8 vset;
	u8 open_target;
	u8 open_state;
};

static int drv8830_read_reg(struct i2c_client *client, u32 reg)
{
	int rc;

	rc = i2c_smbus_read_byte_data(client, reg);
	if (rc < 0)
		dev_err(&client->dev, "i2c reg read for 0x%x failed\n", reg);
	return rc;
}

static int drv8830_write_reg(struct i2c_client *client, u32 reg, u8 val)
{
	int rc;

	rc = i2c_smbus_write_byte_data(client, reg, val);
	if (rc < 0)
		dev_err(&client->dev, "i2c reg write for 0x%xfailed\n", reg);

	return rc;
}

static int drv8830_pulse_motor(struct drv8830_data *data) {
	int rc;
	struct i2c_client *client = data->client;

	if (data->open_target) {
		drv8830_write_reg(client, DRV8830_CONTROL_REG,
			(data->vset << DRV8830_VSET_OFFSET) | DRV8830_IN1);
	} else {
		drv8830_write_reg(client, DRV8830_CONTROL_REG,
			(data->vset << DRV8830_VSET_OFFSET) | DRV8830_IN0);
	}

	msleep(PULSE_DURATION_MS);

	/* Brake the motor so the cut filter doesn't move */
	rc = drv8830_write_reg(client, DRV8830_CONTROL_REG, DRV8830_BRAKE);
	data->open_state = data->open_target;

	return rc;
}

static void drv8830_worker(struct work_struct *work) {
	struct drv8830_data *data = container_of(work, struct drv8830_data, pulse_work);

	drv8830_pulse_motor(data);
}

static ssize_t drv8830_show_open(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        struct drv8830_data *data = i2c_get_clientdata(to_i2c_client(dev));

        return sprintf(buf, "%u\n", data->open_state);
}

static ssize_t drv8830_store_open(struct device *dev,
                        struct device_attribute *attr, const char *buf, size_t count)
{
        struct i2c_client *client = to_i2c_client(dev);
        struct drv8830_data *data = i2c_get_clientdata(client);
        int ret;

	ret = kstrtou8(buf, 8, &data->open_target);

	if (ret < 0)
		return ret;

	cancel_work_sync(&data->pulse_work);
	schedule_work(&data->pulse_work);

        return count;
}

static DEVICE_ATTR(open, S_IWUSR | S_IRUGO, drv8830_show_open, drv8830_store_open);

static struct attribute *drv8830_attributes[] = {
        &dev_attr_open.attr,
        NULL
};

static const struct attribute_group drv8830_attr_group = {
        .attrs = drv8830_attributes,
};

static int drv8830_parse_dt(struct device *dev, struct drv8830_pdata *pdata)
{
	u32 vset;
	int rc = 0;

	rc = of_property_read_u32(dev->of_node, "ti,vset_setting", &vset);

	if (rc < 0) {
		pdata->vset = 0;
		return rc;
	} else {
		/* vset must be 6 bits or less */
		pdata->vset = (u8) (vset & GENMASK(5,0));
	}

	pdata->fault_gpio = of_get_named_gpio(dev->of_node, "ti,fault_gpio", 0);
	if (!gpio_is_valid(pdata->fault_gpio))
	{
		dev_err(dev, "%s - Error: Could not get named gpio! Error code: %d\n", __func__, pdata->fault_gpio);
		return pdata->fault_gpio;
	}

	return 0;
}

static irqreturn_t drv8830_irq(int irq, void *data) {
	struct drv8830_data *drv_data = data;
	struct i2c_client *client = drv_data->client;

	u8 fault_reg;
	fault_reg = drv8830_read_reg(drv_data->client, DRV8830_FAULT_REG);

	if ((fault_reg & DRV8830_FAULT) == 0x0)
		return IRQ_NONE;

	if (fault_reg & DRV8830_ILIMIT) {
		dev_crit(&client->dev, "FAULT: extended current limit event\n");
	} else if (fault_reg & DRV8830_OTS) {
		dev_crit(&client->dev, "FAULT: overtemperature (OTS) condition\n");
	} else if (fault_reg & DRV8830_UVLO) {
		dev_crit(&client->dev, "FAULT: undervoltage lockout\n");
	} else if (fault_reg & DRV8830_OCP) {
		dev_crit(&client->dev, "FAULT: overcurrent (OCP) event\n");
	} else {
		dev_crit(&client->dev, "Fault bit set but no specific fault\n");
	}
	drv8830_write_reg(client, DRV8830_FAULT_REG, fault_reg | DRV8830_CLEAR);

	return IRQ_HANDLED;
}

static int drv8830_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct drv8830_data *data;
	struct drv8830_pdata *pdata;
	int rc;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c is not supported\n");
		return -EIO;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct drv8830_pdata), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "unable to allocate data\n");
			return -ENOMEM;
		}
		/* parse DT */
		rc = drv8830_parse_dt(&client->dev, pdata);
		if (rc) {
			dev_err(&client->dev, "DT parsing failed\n");
			return rc;
		}
	} else {
		pdata = client->dev.platform_data;
		if (!pdata) {
			dev_err(&client->dev, "invalid data\n");
			return -EINVAL;
		}
	}

	data = devm_kzalloc(&client->dev, sizeof(struct drv8830_data),
					GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "unable to allocate memory\n");
		return -ENOMEM;
	}

	data->vset = pdata->vset;
	data->fault_gpio = pdata->fault_gpio;

	/* Initially pulse closed */
	data->open_target = 0;
	data->open_state = 0;

	i2c_set_clientdata(client, data);

	data->client = client;

	rc = devm_request_threaded_irq(&client->dev, gpio_to_irq(data->fault_gpio), NULL, drv8830_irq,
			IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
			"drv8830", data);

	rc = sysfs_create_group(&client->dev.kobj, &drv8830_attr_group);
	if (rc < 0) {
		dev_err(&client->dev, "can't create sysfs group \n");
		return rc;
	}

	INIT_WORK(&data->pulse_work, drv8830_worker);
	schedule_work(&data->pulse_work);

	dev_info(&client->dev, "probe completed");

	return rc;
}

static int drv8830_remove(struct i2c_client *client)
{
	struct drv8830_data *data = i2c_get_clientdata(client);

	cancel_work_sync(&data->pulse_work);

	return 0;
}

static const struct i2c_device_id drv8830_id_table[] = {
	{"drv8830", 0},
	{ },
};
MODULE_DEVICE_TABLE(i2c, drv8830_id_table);

#ifdef CONFIG_OF
static const struct of_device_id drv8830_of_id_table[] = {
	{.compatible = "ti, drv8830"},
	{ },
};
#else
#define drv8830_of_id_table NULL
#endif

static struct i2c_driver drv8830_i2c_driver = {
	.driver = {
		.name = "drv8830",
		.owner = THIS_MODULE,
		.of_match_table = drv8830_of_id_table,
	},
	.probe = drv8830_probe,
	.remove = drv8830_remove,
	.id_table = drv8830_id_table,
};

module_i2c_driver(drv8830_i2c_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TI DRV8830 chip driver");
