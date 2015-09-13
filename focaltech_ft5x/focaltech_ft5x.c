/*
 * Driver for FocalTech ft5x i2c touchscreen controller
 *
 * Copyright (c) 2015 Red Hat Inc.
 * Copyright (c) 2015 Bertrik Sikken <bertrik@sikken.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Red Hat authors:
 * Hans de Goede <hdegoede@redhat.com>
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/module.h>
#include <linux/of.h>

#define FT5X_REG_TOUCHDATA      0
#define FT5X_MAX_TOUCHES        5

#define FT5X_EVENT_PRESSED      0
#define FT5X_EVENT_RELEASED     4
#define FT5X_EVENT_KEPT         8

/* FT5X_REG_PMODE */
#define FT5X_REG_PMODE		0xA5
#define PMODE_ACTIVE        	0x00
#define PMODE_MONITOR       	0x01
#define PMODE_STANDBY       	0x02
#define PMODE_HIBERNATE     	0x03

#define TS_WAKEUP_LOW_PERIOD	20
#define TS_WAKEUP_HIGH_PERIOD	20

struct ft5x_touch {
	__u8 data[4];
	__u16 padding;
} __packed;

struct ft5x_touch_data {
	__u16 padding;
	__u8 touch_count;
	struct ft5x_touch touches[FT5X_MAX_TOUCHES];
} __packed;

struct ft5x_data {
	struct i2c_client *client;
	struct input_dev *input;
	struct gpio_desc *wake_gpio;
	u32 max_x;
	u32 max_y;
	bool invert_x;
	bool invert_y;
	bool swap_x_y;
};

static int ft5x_read_touch_data(struct i2c_client *client,
				struct ft5x_touch_data *touch_data)
{
	u8 reg = FT5X_REG_TOUCHDATA;
	struct i2c_msg msg[2] = {
		{
		 .addr = client->addr,
		 .len = 1,
		 .buf = &reg},
		{
		 .addr = client->addr,
		 .flags = I2C_M_RD,
		 .len = sizeof(struct ft5x_touch_data),
		 .buf = (u8 *) touch_data}
	};

	return i2c_transfer(client->adapter, msg, 2);
}

static int ft5x_write_register(struct i2c_client *client, u8 reg, u8 data)
{
	u8 buf[2] = { reg, data };
	struct i2c_msg msg = {
		.addr = client->addr,
		.len = 2,
		.buf = buf
	};

	return i2c_transfer(client->adapter, &msg, 1);
}

static inline bool ft5x_touch_active(u8 event)
{
	return (event == FT5X_EVENT_KEPT);
}

static irqreturn_t ft5x_irq(int irq, void *dev_id)
{
	struct ft5x_data *data = dev_id;
	struct device *dev = &data->client->dev;
	struct ft5x_touch_data touch_data;
	int i, ret, x, y, slot;

	ret = ft5x_read_touch_data(data->client, &touch_data);
	if (ret < 0) {
		dev_err(dev, "Error reading touch data: %d\n", ret);
		return IRQ_HANDLED;
	}

	if (touch_data.touch_count > FT5X_MAX_TOUCHES) {
		dev_warn(dev, "Too many touches %d > %d\n",
			 touch_data.touch_count, FT5X_MAX_TOUCHES);
		touch_data.touch_count = FT5X_MAX_TOUCHES;
	}

	for (i = 0; i < touch_data.touch_count; i++) {
		struct ft5x_touch *touch = &touch_data.touches[i];

		u8 event = (touch->data[0] >> 4) & 0xF;
		bool act = ft5x_touch_active(event);
		slot = (touch->data[2] >> 4) & 0xF;

		input_mt_slot(data->input, slot);
		input_mt_report_slot_state(data->input, MT_TOOL_FINGER, act);

		if (!act)
			continue;

		x = ((touch->data[0] & 0xF) << 8) | touch->data[1];
		y = ((touch->data[2] & 0xF) << 8) | touch->data[3];

		if (data->invert_x)
			x = data->max_x - x;

		if (data->invert_y)
			y = data->max_y - y;

		if (!data->swap_x_y) {
			input_event(data->input, EV_ABS, ABS_MT_POSITION_X, x);
			input_event(data->input, EV_ABS, ABS_MT_POSITION_Y, y);
		} else {
			input_event(data->input, EV_ABS, ABS_MT_POSITION_X, y);
			input_event(data->input, EV_ABS, ABS_MT_POSITION_Y, x);
		}
	}

	input_mt_sync_frame(data->input);
	input_sync(data->input);

	return IRQ_HANDLED;
}

static int ft5x_start(struct input_dev *dev)
{
	struct ft5x_data *data = input_get_drvdata(dev);

	gpiod_set_value_cansleep(data->wake_gpio, 0);
	msleep(TS_WAKEUP_LOW_PERIOD);
	gpiod_set_value_cansleep(data->wake_gpio, 1);
	msleep(TS_WAKEUP_HIGH_PERIOD);

	enable_irq(data->client->irq);

	return 0;
}

static void ft5x_stop(struct input_dev *dev)
{
	struct ft5x_data *data = input_get_drvdata(dev);

	disable_irq(data->client->irq);

	ft5x_write_register(data->client, FT5X_REG_PMODE, PMODE_HIBERNATE);
}

#ifdef CONFIG_PM_SLEEP
static int ft5x_suspend(struct device *dev)
{
	struct ft5x_data *data = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&data->input->mutex);
	if (data->input->users)
		ft5x_stop(data->input);
	mutex_unlock(&data->input->mutex);

	return 0;
}

static int ft5x_resume(struct device *dev)
{
	struct ft5x_data *data = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&data->input->mutex);
	if (data->input->users)
		ft5x_start(data->input);
	mutex_unlock(&data->input->mutex);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(ft5x_pm_ops, ft5x_suspend, ft5x_resume);

static int ft5x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;
	struct ft5x_data *data;
	struct input_dev *input;
	u32 fuzz_x = 0, fuzz_y = 0;
	int error;

	if (!client->irq) {
		dev_err(dev, "Error no irq specified\n");
		return -EINVAL;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->wake_gpio = devm_gpiod_get(dev, "wake", GPIOD_OUT_LOW);
	if (IS_ERR(data->wake_gpio)) {
		error = PTR_ERR(data->wake_gpio);
		if (error != -EPROBE_DEFER)
			dev_err(dev, "Error getting wake gpio: %d\n", error);
		return error;
	}

	if (of_property_read_u32(np, "touchscreen-size-x", &data->max_x) ||
	    of_property_read_u32(np, "touchscreen-size-y", &data->max_y)) {
		dev_err(dev, "Error touchscreen-size-x and/or -y missing\n");
		return -EINVAL;
	}

	/* Optional */
	of_property_read_u32(np, "touchscreen-fuzz-x", &fuzz_x);
	of_property_read_u32(np, "touchscreen-fuzz-y", &fuzz_y);
	data->invert_x = of_property_read_bool(np, "touchscreen-inverted-x");
	data->invert_y = of_property_read_bool(np, "touchscreen-inverted-y");
	data->swap_x_y = of_property_read_bool(np, "touchscreen-swapped-x-y");

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	input->name = client->name;
	input->id.bustype = BUS_I2C;
	input->open = ft5x_start;
	input->close = ft5x_stop;
	input->dev.parent = dev;

	if (!data->swap_x_y) {
		input_set_abs_params(input, ABS_MT_POSITION_X, 0,
				     data->max_x, fuzz_x, 0);
		input_set_abs_params(input, ABS_MT_POSITION_Y, 0,
				     data->max_y, fuzz_y, 0);
	} else {
		input_set_abs_params(input, ABS_MT_POSITION_X, 0,
				     data->max_y, fuzz_y, 0);
		input_set_abs_params(input, ABS_MT_POSITION_Y, 0,
				     data->max_x, fuzz_x, 0);
	}

	error = input_mt_init_slots(input, FT5X_MAX_TOUCHES,
				    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (error)
		return error;

	data->client = client;
	data->input = input;
	input_set_drvdata(input, data);

	error = devm_request_threaded_irq(dev, client->irq, NULL, ft5x_irq,
					  IRQF_ONESHOT, client->name, data);
	if (error) {
		dev_err(dev, "Error requesting irq: %d\n", error);
		return error;
	}

	/* Stop device till opened */
	ft5x_stop(data->input);

	error = input_register_device(input);
	if (error)
		return error;

	i2c_set_clientdata(client, data);

	return 0;
}

static const struct of_device_id ft5x_of_match[] = {
	{.compatible = "focaltech,ft5x"},
	{}
};

MODULE_DEVICE_TABLE(of, ft5x_of_match);

/* This is useless for OF-enabled devices, but it is needed by I2C subsystem */
static const struct i2c_device_id ft5x_i2c_id[] = {
	{},
};

MODULE_DEVICE_TABLE(i2c, ft5x_i2c_id);

static struct i2c_driver ft5x_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "focaltech_ft5x",
		   .pm = &ft5x_pm_ops,
		   .of_match_table = ft5x_of_match,
		   },
	.probe = ft5x_probe,
	.id_table = ft5x_i2c_id,
};

module_i2c_driver(ft5x_driver);

MODULE_DESCRIPTION("FocalTech FT5x I2C Touchscreen Driver");
MODULE_AUTHOR("Hans de Goede <hdegoede@redhat.com>");
MODULE_LICENSE("GPL");
