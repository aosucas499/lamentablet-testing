/*
 * FocalTech ft5x06 touch screen driver
 * Based on Boundary Devices driver, adapted for BayTrail tablets
 * without ACPI/DTS - GPIOs hardcoded for kernel 5.10+
 *
 * GPIO mapping kernel 5.10 (BayTrail):
 *   SCORE base=410 (102 pins)
 *   NCORE base=382 (28 pins)
 *   SUS   base=338 (44 pins)
 *
 * Wakeup/IRQ GPIO: SUS pin 3 = 338+3 = 341
 * Reset GPIO:      SUS pin 26 = 338+... (try 128 first)
 *
 * Original: Copyright (c) Boundary Devices <info@boundarydevices.com>
 * Adapted for BayTrail tablets without ACPI
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/input/mt.h>
#include <linux/input.h>
#include <linux/timer.h>

/* BayTrail GPIO bases in kernel 5.10 */
#define VLV_GPIO_SUS_BASE	338

/* GPIO pins for this tablet */
#define FT5X06_GPIO_WAKEUP	(VLV_GPIO_SUS_BASE + 3)   /* 341 - IRQ pin */
#define FT5X06_GPIO_RESET	128                        /* adjust if needed */

/* Screen resolution */
#define FT5X06_MAX_X		1279
#define FT5X06_MAX_Y		799

/* Max touch points */
#define MAX_TOUCHES		5

/* Module parameters to override defaults */
static int gpio_wakeup = FT5X06_GPIO_WAKEUP;
static int gpio_reset = FT5X06_GPIO_RESET;
static int max_x = FT5X06_MAX_X;
static int max_y = FT5X06_MAX_Y;
module_param(gpio_wakeup, int, 0644);
module_param(gpio_reset, int, 0644);
module_param(max_x, int, 0644);
module_param(max_y, int, 0644);
MODULE_PARM_DESC(gpio_wakeup, "Wakeup/IRQ GPIO number (default: 341)");
MODULE_PARM_DESC(gpio_reset,  "Reset GPIO number (default: 128)");
MODULE_PARM_DESC(max_x, "Screen max X (default: 1279)");
MODULE_PARM_DESC(max_y, "Screen max Y (default: 799)");

#define WORK_MODE	0

struct point {
	int x;
	int y;
	int id;
};

struct ft5x06_ts {
	struct i2c_client	*client;
	struct input_dev	*idev;
	int			use_count;
	int			irq;
	struct gpio_desc	*wakeup_gpio;
	struct gpio_desc	*reset_gpio;
	struct timer_list	release_timer;
	unsigned		down_mask;
	unsigned		firmware_bug_hit;
	struct point		points[MAX_TOUCHES];
	unsigned char		buf[4 + (6 * MAX_TOUCHES)];
};

static void write_reg(struct ft5x06_ts *ts, int regnum, int value)
{
	u8 buf[] = { regnum, value };
	struct i2c_msg pkt = {
		ts->client->addr, 0, sizeof(buf), buf
	};
	i2c_transfer(ts->client->adapter, &pkt, 1);
}

static void set_mode(struct ft5x06_ts *ts, int mode)
{
	write_reg(ts, 0, (mode & 7) << 4);
}

static void release_slots(struct ft5x06_ts *ts, unsigned mask)
{
	while (mask) {
		int slot = __ffs(mask);
		mask &= ~(1 << slot);
		input_mt_slot(ts->idev, slot);
		input_mt_report_slot_state(ts->idev, MT_TOOL_FINGER, 0);
	}
}

static void ts_evt_add(struct ft5x06_ts *ts, unsigned buttons, struct point *p)
{
	struct input_dev *idev = ts->idev;
	unsigned down_mask = 0;
	unsigned tmp;
	int i;

	if (!buttons) {
		tmp = ts->down_mask;
		ts->down_mask = 0;
		release_slots(ts, tmp);
		input_report_key(idev, BTN_TOUCH, 0);
	} else {
		for (i = 0; i < buttons; i++) {
			input_mt_slot(idev, p[i].id);
			input_mt_report_slot_state(idev, MT_TOOL_FINGER, 1);
			down_mask |= 1 << p[i].id;
			input_report_abs(idev, ABS_MT_POSITION_X, p[i].x);
			input_report_abs(idev, ABS_MT_POSITION_Y, p[i].y);
		}
		tmp = ts->down_mask & ~down_mask;
		ts->down_mask = down_mask;
		release_slots(ts, tmp);
		input_report_key(idev, BTN_TOUCH, 1);
	}
	input_sync(idev);
}

static void ts_release_timer(struct timer_list *t)
{
	struct ft5x06_ts *ts = from_timer(ts, t, release_timer);

	if (ts->down_mask) {
		ts_evt_add(ts, 0, NULL);
		ts->firmware_bug_hit++;
	}
}

static void ts_reset(struct ft5x06_ts *ts)
{
	if (ts->reset_gpio) {
		gpiod_set_value(ts->reset_gpio, 1);
		msleep(5);
		gpiod_set_value(ts->reset_gpio, 0);
		msleep(5);
	}
	set_mode(ts, WORK_MODE);
}

static irqreturn_t ts_interrupt(int irq, void *id)
{
	struct ft5x06_ts *ts = id;
	int ret;
	unsigned char startch[1] = { 0 };
	struct i2c_msg readpkt[2] = {
		{ ts->client->addr, 0, 1, startch },
		{ ts->client->addr, I2C_M_RD, 3 + (6 * MAX_TOUCHES), ts->buf }
	};
	int buttons = 0;
	int i;
	unsigned char *p;
	int fails = 0;

	del_timer_sync(&ts->release_timer);

	/* Read while wakeup GPIO is active (touch present) */
	do {
        ret = i2c_transfer(ts->client->adapter, readpkt,
                           ARRAY_SIZE(readpkt));
        
        if (ret != ARRAY_SIZE(readpkt)) {
            dev_err(&ts->client->dev, "i2c_transfer failed(%d)\n", ret);
            if (fails > 2) {
                ts_reset(ts);
                fails = 0;
            } else {
                msleep(100);
                fails++;
            }
            continue; // Si falla, reintenta (vuelve al inicio del do)
        }
        
        // --- PROCESAMIENTO EXITOSO ---
        fails = 0;
        p = ts->buf + 3;
        buttons = ts->buf[2];
        
        if (buttons > MAX_TOUCHES) {
            if (!gpiod_get_value(ts->wakeup_gpio))
                break; // Si no hay toque, sale del bucle
            buttons = MAX_TOUCHES;
        }

        for (i = 0; i < buttons; i++) {
            ts->points[i].x = (((p[0] & 0x0f) << 8) | p[1]) & 0x7ff;
            ts->points[i].id = (p[2] >> 4);
            ts->points[i].y = (((p[2] & 0x0f) << 8) | p[3]) & 0x7ff;
            /* Clamp to screen bounds */
            if (ts->points[i].x > max_x)
                ts->points[i].x = max_x;
            if (ts->points[i].y > max_y)
                ts->points[i].y = max_y;
            p += 6;
        }
        
        ts_evt_add(ts, buttons, ts->points);
        
        // --- EL BREAK AQUÍ ES CRUCIAL ---
        // Indica que ya leímos y procesamos los datos. 
        // Salimos del bucle para terminar la interrupción.
        break; 

    } while (0); 

    if (ts->down_mask)
        mod_timer(&ts->release_timer, jiffies + msecs_to_jiffies(100));

    return IRQ_HANDLED;
}

static int ts_startup(struct ft5x06_ts *ts)
{
	int ret = 0;

	if (ts->use_count++ != 0)
		return 0;

	ret = request_threaded_irq(ts->irq, NULL, ts_interrupt,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_SHARED,
				   "ft5x06-ts", ts);
	if (ret) {
		dev_err(&ts->client->dev, "error requesting irq %d: %d\n",
			ts->irq, ret);
		ts->use_count--;
		return ret;
	}

	set_mode(ts, WORK_MODE);
	return 0;
}

static void ts_shutdown(struct ft5x06_ts *ts)
{
	if (ts && --ts->use_count == 0) {
		free_irq(ts->irq, ts);
		del_timer_sync(&ts->release_timer);
	}
}

static int ts_open(struct input_dev *idev)
{
	struct ft5x06_ts *ts = input_get_drvdata(idev);
	return ts_startup(ts);
}

static void ts_close(struct input_dev *idev)
{
	struct ft5x06_ts *ts = input_get_drvdata(idev);
	ts_shutdown(ts);
}

static int ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5x06_ts *ts;
	struct input_dev *idev;
	struct gpio_desc *gp;
	int err;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;
	i2c_set_clientdata(client, ts);

	/* Reset GPIO - hardcoded */
	if (gpio_request(gpio_reset, "ft5x06-reset") == 0) {
		gpio_direction_output(gpio_reset, 0);
		ts->reset_gpio = gpio_to_desc(gpio_reset);
		dev_info(&client->dev, "reset GPIO %d acquired\n", gpio_reset);
	} else {
		dev_warn(&client->dev, "could not get reset GPIO %d\n", gpio_reset);
	}

	/* Reset sequence */
	ts_reset(ts);
	msleep(200);

	/* Wakeup/IRQ GPIO - hardcoded */
	if (gpio_request(gpio_wakeup, "ft5x06-wakeup") == 0) {
		gpio_direction_input(gpio_wakeup);
		ts->wakeup_gpio = gpio_to_desc(gpio_wakeup);
		ts->irq = gpio_to_irq(gpio_wakeup);
		dev_info(&client->dev, "wakeup GPIO %d -> IRQ %d\n",
			 gpio_wakeup, ts->irq);
	} else {
		dev_warn(&client->dev, "could not get wakeup GPIO %d\n",
			 gpio_wakeup);
		/* fallback: use IRQ 0 from ACPI like edt-ft5x06 */
		ts->irq = 0;  /* IRQ del ACPI - mismo que edt-ft5x06 */
ts->wakeup_gpio = NULL; /* sin GPIO, leer siempre */
		dev_info(&client->dev, "using client IRQ %d\n", ts->irq);
	}

	/* Allocate input device */
	idev = input_allocate_device();
	if (!idev) {
		err = -ENOMEM;
		goto exit_free;
	}

	ts->idev = idev;
	idev->name = "ft5x06-ts";
	idev->id.bustype = BUS_I2C;
	idev->id.product = client->addr;
	idev->open  = ts_open;
	idev->close = ts_close;

	__set_bit(EV_ABS, idev->evbit);
	__set_bit(EV_KEY, idev->evbit);
	__set_bit(EV_SYN, idev->evbit);
	__set_bit(BTN_TOUCH, idev->keybit);
	__set_bit(INPUT_PROP_DIRECT, idev->propbit);

	input_mt_init_slots(idev, MAX_TOUCHES, INPUT_MT_DIRECT);
	input_set_abs_params(idev, ABS_MT_POSITION_X, 0, max_x, 0, 0);
	input_set_abs_params(idev, ABS_MT_POSITION_Y, 0, max_y, 0, 0);
	input_set_abs_params(idev, ABS_MT_TRACKING_ID, 0, MAX_TOUCHES, 0, 0);

	input_set_drvdata(idev, ts);

	err = input_register_device(idev);
	if (err) {
		dev_err(&client->dev, "failed to register input device\n");
		goto exit_free_input;
	}

	timer_setup(&ts->release_timer, ts_release_timer, 0);

	dev_info(&client->dev, "ft5x06 probe OK IRQ=%d GPIO_IRQ=%d GPIO_RST=%d res=%dx%d\n",
		 ts->irq, gpio_wakeup, gpio_reset, max_x + 1, max_y + 1);
	return 0;

exit_free_input:
	input_free_device(idev);
exit_free:
	if (ts->reset_gpio)
		gpio_free(gpio_reset);
	kfree(ts);
	return err;
}

static int ts_remove(struct i2c_client *client)
{
	struct ft5x06_ts *ts = i2c_get_clientdata(client);

	del_timer_sync(&ts->release_timer);
	input_unregister_device(ts->idev);
	input_free_device(ts->idev);
	if (ts->reset_gpio)
		gpio_free(gpio_reset);
	if (ts->wakeup_gpio)
		gpio_free(gpio_wakeup);
	kfree(ts);
	return 0;
}

static const struct i2c_device_id ft5x06_id[] = {
	{ "ft5x06-ts", 0 },
	{ "ft5x0x_ts", 0 },
	{ "FTTH5406",  0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ft5x06_id);

static const struct acpi_device_id ft5x06_acpi_match[] = {
	{ "FTTH5406", 0 },
	{ "ft5x0x_ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, ft5x06_acpi_match);

static struct i2c_driver ft5x06_driver = {
	.driver = {
		.name             = "ft5x06-ts",
		.acpi_match_table = ACPI_PTR(ft5x06_acpi_match),
	},
	.id_table = ft5x06_id,
	.probe    = ts_probe,
	.remove   = ts_remove,
};

module_i2c_driver(ft5x06_driver);

MODULE_AUTHOR("Boundary Devices / adapted for BayTrail");
MODULE_DESCRIPTION("FocalTech ft5x06 - BayTrail hardcoded GPIO, no ACPI needed");
MODULE_LICENSE("GPL");
