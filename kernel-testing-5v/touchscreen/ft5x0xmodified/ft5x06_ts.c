/* drivers/input/touchscreen/ft5x06_ts.c
 *
 * FocalTech ft5x0x TouchScreen driver - ported for kernel 5.10
 * Original: Copyright (c) 2010 Focal tech Ltd.
 * Ported for modern kernels: removed earlysuspend, updated GPIO/ACPI APIs
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2.
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/gpio/consumer.h>
#include <linux/acpi.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include "ft5x06_ts.h"

struct ts_event {
	u16 au16_x[CFG_MAX_TOUCH_POINTS];
	u16 au16_y[CFG_MAX_TOUCH_POINTS];
	u8  au8_touch_event[CFG_MAX_TOUCH_POINTS];
	u8  au8_finger_id[CFG_MAX_TOUCH_POINTS];
	u16 pressure;
	u8  touch_point;
};

struct ft5x0x_ts_data {
	unsigned int irq;
	unsigned int x_max;
	unsigned int y_max;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct ts_event event;
	struct gpio_desc *gpiod_reset;
};

int ft5x0x_i2c_Read(struct i2c_client *client, char *writebuf,
		    int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				.addr  = client->addr,
				.flags = 0,
				.len   = writelen,
				.buf   = writebuf,
			},
			{
				.addr  = client->addr,
				.flags = I2C_M_RD,
				.len   = readlen,
				.buf   = readbuf,
			},
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "%s: i2c read error=%d\n",
				__func__, ret);
	} else {
		struct i2c_msg msgs[] = {
			{
				.addr  = client->addr,
				.flags = I2C_M_RD,
				.len   = readlen,
				.buf   = readbuf,
			},
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s: i2c read error=%d\n",
				__func__, ret);
	}
	return ret;
}

int ft5x0x_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;
	struct i2c_msg msg[] = {
		{
			.addr  = client->addr,
			.flags = 0,
			.len   = writelen,
			.buf   = writebuf,
		},
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s: i2c write error\n", __func__);
	return ret;
}

static void ft5x0x_ts_release(struct ft5x0x_ts_data *data)
{
	int i;

	for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++) {
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev,
					   MT_TOOL_FINGER, false);
	}
	input_report_key(data->input_dev, BTN_TOUCH, 0);
	input_sync(data->input_dev);
}

static int ft5x0x_read_touchdata(struct ft5x0x_ts_data *data)
{
	struct ts_event *event = &data->event;
	u8 buf[POINT_READ_BUF] = { 0 };
	int ret;
	int i;
	u8 pointid = FT_MAX_ID;

	ret = ft5x0x_i2c_Read(data->client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0) {
		dev_err(&data->client->dev, "%s: read touchdata failed\n",
			__func__);
		return ret;
	}

	memset(event, 0, sizeof(struct ts_event));
	event->touch_point = 0;

	for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++) {
		pointid = (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
		if (pointid >= FT_MAX_ID)
			break;
		event->touch_point++;
		event->au16_x[i] =
			(s16)(buf[FT_TOUCH_X_H_POS + FT_TOUCH_STEP * i] & 0x0F) << 8 |
			(s16)buf[FT_TOUCH_X_L_POS + FT_TOUCH_STEP * i];
		event->au16_y[i] =
			(s16)(buf[FT_TOUCH_Y_H_POS + FT_TOUCH_STEP * i] & 0x0F) << 8 |
			(s16)buf[FT_TOUCH_Y_L_POS + FT_TOUCH_STEP * i];
		event->au8_touch_event[i] =
			buf[FT_TOUCH_EVENT_POS + FT_TOUCH_STEP * i] >> 6;
		event->au8_finger_id[i] =
			(buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
	}

	event->pressure = FT_PRESS;
	return 0;
}

static void ft5x0x_report_value(struct ft5x0x_ts_data *data)
{
	struct ts_event *event = &data->event;
	int i;
	int uppoint = 0;

	for (i = 0; i < event->touch_point; i++) {
		input_mt_slot(data->input_dev, event->au8_finger_id[i]);

		if (event->au8_touch_event[i] == 0 ||
		    event->au8_touch_event[i] == 2) {
			input_mt_report_slot_state(data->input_dev,
						   MT_TOOL_FINGER, true);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR,
					 event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X,
					 event->au16_x[i]);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
					 event->au16_y[i]);
		} else {
			uppoint++;
			input_mt_slot(data->input_dev, event->au8_finger_id[i]);
			input_mt_report_slot_state(data->input_dev,
						   MT_TOOL_FINGER, false);
		}
	}

	if (event->touch_point == uppoint)
		input_report_key(data->input_dev, BTN_TOUCH, 0);
	else
		input_report_key(data->input_dev, BTN_TOUCH,
				 event->touch_point > 0);
	input_sync(data->input_dev);
}

static irqreturn_t ft5x0x_ts_interrupt(int irq, void *dev_id)
{
	struct ft5x0x_ts_data *ft5x0x_ts = dev_id;
	int ret;

	ret = ft5x0x_read_touchdata(ft5x0x_ts);
	if (ret == 0)
		ft5x0x_report_value(ft5x0x_ts);

	return IRQ_HANDLED;
}

static int ft5x0x_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	struct input_dev *input_dev;
	struct gpio_desc *gpiod_irq;
	int err = 0;
	unsigned char uc_reg_value;
	unsigned char uc_reg_addr;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C not supported\n");
		return -ENODEV;
	}

	/* Verify this is really a FocalTech chip */
	uc_reg_addr = 0xa8;
	ft5x0x_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	if ((uc_reg_value != 0xa0) && (uc_reg_value != 0x59)) {
		dev_err(&client->dev, "wrong touch chip ID: 0x%x\n",
			uc_reg_value);
		return -ENODEV;
	}

	ft5x0x_ts = devm_kzalloc(&client->dev, sizeof(*ft5x0x_ts), GFP_KERNEL);
	if (!ft5x0x_ts)
		return -ENOMEM;

	ft5x0x_ts->client = client;
	ft5x0x_ts->x_max = FT5X0X_MAX_X;
	ft5x0x_ts->y_max = FT5X0X_MAX_Y;
	i2c_set_clientdata(client, ft5x0x_ts);

	/* Get reset GPIO via ACPI - index 0 */
	ft5x0x_ts->gpiod_reset = devm_gpiod_get_index(&client->dev,
						       NULL, 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ft5x0x_ts->gpiod_reset)) {
		dev_warn(&client->dev, "no reset GPIO, continuing anyway\n");
		ft5x0x_ts->gpiod_reset = NULL;
	}

	/* Get IRQ GPIO via ACPI - index 1 */
	gpiod_irq = devm_gpiod_get_index(&client->dev, NULL, 1, GPIOD_IN);
	if (IS_ERR(gpiod_irq)) {
		dev_err(&client->dev, "failed to get IRQ GPIO\n");
		return PTR_ERR(gpiod_irq);
	}
	client->irq = gpiod_to_irq(gpiod_irq);
	ft5x0x_ts->irq = client->irq;

	/* Reset the chip */
	if (ft5x0x_ts->gpiod_reset) {
		gpiod_set_value_cansleep(ft5x0x_ts->gpiod_reset, 0);
		msleep(20);
		gpiod_set_value_cansleep(ft5x0x_ts->gpiod_reset, 1);
		msleep(150);
	}

	/* Allocate input device */
	input_dev = devm_input_allocate_device(&client->dev);
	if (!input_dev) {
		dev_err(&client->dev, "failed to allocate input device\n");
		return -ENOMEM;
	}
	ft5x0x_ts->input_dev = input_dev;

	input_dev->name = FT5X0X_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_mt_init_slots(input_dev, CFG_MAX_TOUCH_POINTS,
			    INPUT_MT_DIRECT);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0,
			     ft5x0x_ts->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0,
			     ft5x0x_ts->y_max, 0, 0);

	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev, "failed to register input device\n");
		return err;
	}

	/* Read firmware info */
	uc_reg_addr = FT5x0x_REG_FW_VER;
	ft5x0x_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	dev_info(&client->dev, "FT5x0x firmware version: 0x%x\n", uc_reg_value);

	uc_reg_addr = FT5x0x_REG_POINT_RATE;
	ft5x0x_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	dev_info(&client->dev, "FT5x0x report rate: %dHz\n", uc_reg_value * 10);

	err = request_threaded_irq(client->irq, NULL, ft5x0x_ts_interrupt,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				   client->dev.driver->name, ft5x0x_ts);
	if (err < 0) {
		dev_err(&client->dev, "failed to request IRQ %d\n", client->irq);
		return err;
	}

	dev_info(&client->dev,
		 "FT5x0x touchscreen probe OK, IRQ=%d, res=%dx%d\n",
		 client->irq, ft5x0x_ts->x_max + 1, ft5x0x_ts->y_max + 1);

	return 0;
}

static int ft5x0x_ts_remove(struct i2c_client *client)
{
	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(client);

	free_irq(client->irq, ft5x0x_ts);
	ft5x0x_ts_release(ft5x0x_ts);
	return 0;
}

static int __maybe_unused ft5x0x_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ft5x0x_ts_data *ts = i2c_get_clientdata(client);
	u8 buf[2] = { 0xa5, 0x03 };

	disable_irq(ts->irq);
	ft5x0x_i2c_Write(client, buf, 2);
	return 0;
}

static int __maybe_unused ft5x0x_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ft5x0x_ts_data *ts = i2c_get_clientdata(client);

	if (ts->gpiod_reset) {
		gpiod_set_value_cansleep(ts->gpiod_reset, 0);
		msleep(20);
		gpiod_set_value_cansleep(ts->gpiod_reset, 1);
		msleep(150);
	}
	enable_irq(ts->irq);
	ft5x0x_ts_release(ts);
	return 0;
}

static SIMPLE_DEV_PM_OPS(ft5x0x_pm_ops,
			  ft5x0x_ts_suspend, ft5x0x_ts_resume);

static const struct i2c_device_id ft5x0x_ts_id[] = {
	{ FT5X0X_NAME, 0 },
	{ "FTTH5406:00", 0 },
	{ "FTTH5406", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);

static const struct acpi_device_id ft5x0x_acpi_match[] = {
	{ "ft5x0x_ts", 0 },
	{ "FTTH5406", 0 },
	{ "FTTH5406:00", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, ft5x0x_acpi_match);

static struct i2c_driver ft5x0x_ts_driver = {
	.probe   = ft5x0x_ts_probe,
	.remove  = ft5x0x_ts_remove,
	.id_table = ft5x0x_ts_id,
	.driver  = {
		.name            = FT5X0X_NAME,
		.owner           = THIS_MODULE,
		.pm              = &ft5x0x_pm_ops,
		.acpi_match_table = ACPI_PTR(ft5x0x_acpi_match),
	},
};

module_i2c_driver(ft5x0x_ts_driver);

MODULE_AUTHOR("FocalTech / ported for kernel 5.10");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");
