// SPDX-License-Identifier: GPL-2.0
// Samsung S6SY761 Touchscreen device driver
//
// Copyright (c) 2017 Samsung Electronics Co., Ltd.
// Copyright (c) 2017 Andi Shyti <andi@etezian.org>
// Copyright (c) 2026 Lukas Timmermann <foss@timmermann.space>

#include <linux/unaligned.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#include "s6sy761_core.h"

/* commands */
#define S6SY761_SENSE_ON		0x10
#define S6SY761_SENSE_OFF		0x11
#define S6SY761_TOUCH_FUNCTION		0x30 /* R/W for get/set */
#define S6SY761_FIRMWARE_INTEGRITY	0x21
#define S6SY761_PANEL_INFO		0x23
#define S6SY761_DEVICE_ID		0x52
#define S6SY761_BOOT_STATUS		0x55
#define S6SY761_READ_ONE_EVENT		0x60
#define S6SY761_READ_ALL_EVENT		0x61
#define S6SY761_CLEAR_EVENT_STACK	0x62
#define S6SY761_APPLICATION_MODE	0xe4

/* events */
#define S6SY761_EVENT_INFO		0x02
#define S6SY761_EVENT_VENDOR_INFO	0x07

/* info */
#define S6SY761_INFO_BOOT_COMPLETE	0x00

/* firmware status */
#define S6SY761_FW_OK			0x80

/*
 * the functionalities are put as a reference
 * as in the device I am using none of them
 * works therefore not used in this driver yet.
 */
/* touchscreen functionalities */
#define S6SY761_MASK_TOUCH		BIT(0)
#define S6SY761_MASK_HOVER		BIT(1)
#define S6SY761_MASK_COVER		BIT(2)
#define S6SY761_MASK_GLOVE		BIT(3)
#define S6SY761_MASK_STYLUS		BIT(4)
#define S6SY761_MASK_PALM		BIT(5)
#define S6SY761_MASK_WET		BIT(6)
#define S6SY761_MASK_PROXIMITY		BIT(7)

/* boot status (BS) */
#define S6SY761_BS_BOOT_LOADER		0x10
#define S6SY761_BS_APPLICATION		0x20

/* event id */
#define S6SY761_EVENT_ID_COORDINATE	0x00
#define S6SY761_EVENT_ID_STATUS		0x01

/* event register masks */
#define S6SY761_MASK_TOUCH_STATE	0xc0 /* byte 0 */
#define S6SY761_MASK_TID		0x3c
#define S6SY761_MASK_EID		0x03
#define S6SY761_MASK_X			0xf0 /* byte 3 */
#define S6SY761_MASK_Y			0x0f
#define S6SY761_MASK_Z			0x3f /* byte 6 */
#define S6SY761_MASK_LEFT_EVENTS	0x3f /* byte 7 */
#define S6SY761_MASK_TOUCH_TYPE		0xc0 /* MSB in byte 6, LSB in byte 7 */

/* event touch state values */
#define S6SY761_TS_NONE			0x00
#define S6SY761_TS_PRESS		0x01
#define S6SY761_TS_MOVE			0x02
#define S6SY761_TS_RELEASE		0x03

/* application modes */
#define S6SY761_APP_NORMAL		0x0
#define S6SY761_APP_LOW_POWER		0x1
#define S6SY761_APP_TEST		0x2
#define S6SY761_APP_FLASH		0x3
#define S6SY761_APP_SLEEP		0x4

#define S6SY761_DEVID_SIZE		3
#define S6SY761_PANEL_ID_SIZE		11
#define S6SY761_TS_STATUS_SIZE		5
#define S6SY761_MAX_FINGERS		10

struct s6sy761_data {
	struct device *dev;
	const struct s6sy761_bus_ops *bus_ops;
	u8 *xfer_buf;
	/* serializes bus access and protects xfer_buf */
	struct mutex xfer_lock;
	int irq;
	struct regulator_bulk_data regulators[2];
	struct gpio_desc *reset_gpio;
	struct input_dev *input;
	struct touchscreen_properties prop;

	u8 data[S6SY761_EVENT_SIZE * S6SY761_EVENT_COUNT];

	u16 devid;
	u8 tx_channel;
};

enum s6sy761_regulators {
	S6SY761_REGULATOR_VDD,
	S6SY761_REGULATOR_AVDD,
};

static int s6sy761_read(struct s6sy761_data *sdata, u8 cmd, size_t len, void *buf)
{
	guard(mutex)(&sdata->xfer_lock);
	return sdata->bus_ops->read(sdata->dev, sdata->xfer_buf, cmd, len, buf);
}

static int s6sy761_write(struct s6sy761_data *sdata, u8 cmd, size_t len, const void *buf)
{
	guard(mutex)(&sdata->xfer_lock);
	return sdata->bus_ops->write(sdata->dev, sdata->xfer_buf, cmd, len, buf);
}

static void s6sy761_report_coordinates(struct s6sy761_data *sdata,
				       u8 *event, u8 tid)
{
	u8 major = event[4];
	u8 minor = event[5];
	u8 z = event[6] & S6SY761_MASK_Z;
	u16 x = (event[1] << 4) | ((event[3] & S6SY761_MASK_X) >> 4);
	u16 y = (event[2] << 4) | (event[3] & S6SY761_MASK_Y);

	input_mt_slot(sdata->input, tid);

	input_mt_report_slot_state(sdata->input, MT_TOOL_FINGER, true);
	input_report_abs(sdata->input, ABS_MT_POSITION_X, x);
	input_report_abs(sdata->input, ABS_MT_POSITION_Y, y);
	input_report_abs(sdata->input, ABS_MT_TOUCH_MAJOR, major);
	input_report_abs(sdata->input, ABS_MT_TOUCH_MINOR, minor);
	input_report_abs(sdata->input, ABS_MT_PRESSURE, z);

	input_sync(sdata->input);
}

static void s6sy761_report_release(struct s6sy761_data *sdata,
				   u8 *event, u8 tid)
{
	input_mt_slot(sdata->input, tid);
	input_mt_report_slot_state(sdata->input, MT_TOOL_FINGER, false);

	input_sync(sdata->input);
}

static void s6sy761_handle_coordinates(struct s6sy761_data *sdata, u8 *event)
{
	u8 tid;
	u8 touch_state;

	if (unlikely(!(event[0] & S6SY761_MASK_TID)))
		return;

	tid = ((event[0] & S6SY761_MASK_TID) >> 2) - 1;
	touch_state = (event[0] & S6SY761_MASK_TOUCH_STATE) >> 6;

	switch (touch_state) {

	case S6SY761_TS_NONE:
		break;
	case S6SY761_TS_RELEASE:
		s6sy761_report_release(sdata, event, tid);
		break;
	case S6SY761_TS_PRESS:
	case S6SY761_TS_MOVE:
		s6sy761_report_coordinates(sdata, event, tid);
		break;
	}
}

static void s6sy761_handle_events(struct s6sy761_data *sdata, u8 n_events)
{
	int i;

	for (i = 0; i < n_events; i++) {
		u8 *event = &sdata->data[i * S6SY761_EVENT_SIZE];
		u8 event_id = event[0] & S6SY761_MASK_EID;

		if (!event[0])
			return;

		switch (event_id) {

		case S6SY761_EVENT_ID_COORDINATE:
			s6sy761_handle_coordinates(sdata, event);
			break;

		case S6SY761_EVENT_ID_STATUS:
			break;

		default:
			break;
		}
	}
}

static irqreturn_t s6sy761_irq_handler(int irq, void *dev)
{
	struct s6sy761_data *sdata = dev;
	int ret;
	u8 n_events;

	ret = s6sy761_read(sdata, S6SY761_READ_ONE_EVENT,
			   S6SY761_EVENT_SIZE, sdata->data);
	if (ret < 0) {
		dev_err(sdata->dev, "failed to read events\n");
		return IRQ_HANDLED;
	}

	if (!sdata->data[0])
		return IRQ_HANDLED;

	n_events = sdata->data[7] & S6SY761_MASK_LEFT_EVENTS;
	if (unlikely(n_events > S6SY761_EVENT_COUNT - 1))
		return IRQ_HANDLED;

	if (n_events) {
		ret = s6sy761_read(sdata, S6SY761_READ_ALL_EVENT,
				   n_events * S6SY761_EVENT_SIZE,
				   sdata->data + S6SY761_EVENT_SIZE);
		if (ret < 0) {
			dev_err(sdata->dev, "failed to read events\n");
			return IRQ_HANDLED;
		}
	}

	s6sy761_handle_events(sdata, n_events +  1);

	return IRQ_HANDLED;
}

static int s6sy761_input_open(struct input_dev *dev)
{
	struct s6sy761_data *sdata = input_get_drvdata(dev);

	return s6sy761_write(sdata, S6SY761_SENSE_ON, 0, NULL);
}

static void s6sy761_input_close(struct input_dev *dev)
{
	struct s6sy761_data *sdata = input_get_drvdata(dev);
	int ret;

	ret = s6sy761_write(sdata, S6SY761_SENSE_OFF, 0, NULL);
	if (ret)
		dev_err(sdata->dev, "failed to turn off sensing\n");
}

static ssize_t s6sy761_sysfs_devid(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct s6sy761_data *sdata = dev_get_drvdata(dev);

	return sprintf(buf, "%#x\n", sdata->devid);
}

static DEVICE_ATTR(devid, 0444, s6sy761_sysfs_devid, NULL);

static struct attribute *s6sy761_sysfs_attrs[] = {
	&dev_attr_devid.attr,
	NULL
};

static const struct attribute_group s6sy761_sysfs_group = {
	.attrs = s6sy761_sysfs_attrs,
};

const struct attribute_group *s6sy761_sysfs_groups[] = {
	&s6sy761_sysfs_group,
	NULL,
};
EXPORT_SYMBOL_NS_GPL(s6sy761_sysfs_groups, "S6SY761");

static int s6sy761_power_on(struct s6sy761_data *sdata)
{
	__le16 touch_func;
	u8 buffer[S6SY761_EVENT_SIZE];
	u8 event;
	u8 status;
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(sdata->regulators),
				    sdata->regulators);
	if (ret)
		return ret;

	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(sdata->reset_gpio, 0);

	msleep(140);

	/* double check whether the touch is functional */
	ret = s6sy761_read(sdata, S6SY761_READ_ONE_EVENT, S6SY761_EVENT_SIZE, buffer);
	if (ret < 0)
		return ret;

	event = (buffer[0] >> 2) & 0xf;

	if ((event != S6SY761_EVENT_INFO &&
	     event != S6SY761_EVENT_VENDOR_INFO) ||
	    buffer[1] != S6SY761_INFO_BOOT_COMPLETE) {
		return -ENODEV;
	}

	ret = s6sy761_read(sdata, S6SY761_BOOT_STATUS, 1, &status);
	if (ret < 0)
		return ret;

	/* for some reasons the device might be stuck in the bootloader */
	if (status != S6SY761_BS_APPLICATION)
		return -ENODEV;

	/* enable touch functionality */
	touch_func = cpu_to_le16(S6SY761_MASK_TOUCH);
	ret = s6sy761_write(sdata, S6SY761_TOUCH_FUNCTION, sizeof(touch_func), &touch_func);
	if (ret)
		return ret;

	return 0;
}

static int s6sy761_hw_init(struct s6sy761_data *sdata,
			   unsigned int *max_x, unsigned int *max_y)
{
	u8 buffer[S6SY761_PANEL_ID_SIZE]; /* larger read size */
	u8 integrity;
	int ret;

	ret = s6sy761_power_on(sdata);
	if (ret)
		return ret;

	ret = s6sy761_read(sdata, S6SY761_DEVICE_ID, S6SY761_DEVID_SIZE, buffer);
	if (ret < 0)
		return ret;

	sdata->devid = get_unaligned_be16(buffer + 1);

	ret = s6sy761_read(sdata, S6SY761_PANEL_INFO, S6SY761_PANEL_ID_SIZE, buffer);
	if (ret < 0)
		return ret;

	*max_x = get_unaligned_be16(buffer);
	*max_y = get_unaligned_be16(buffer + 2);

	/* if no tx channels defined, at least keep one */
	sdata->tx_channel = max_t(u8, buffer[8], 1);

	ret = s6sy761_read(sdata, S6SY761_FIRMWARE_INTEGRITY, 1, &integrity);
	if (ret < 0)
		return ret;
	else if (integrity != S6SY761_FW_OK)
		return -ENODEV;

	return 0;
}

static void s6sy761_power_off(void *data)
{
	struct s6sy761_data *sdata = data;

	disable_irq(sdata->irq);
	gpiod_set_value_cansleep(sdata->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(sdata->regulators),
						sdata->regulators);
}

int s6sy761_probe(struct device *dev, const struct s6sy761_bus_ops *bus_ops, int irq)
{
	struct s6sy761_data *sdata;
	unsigned int max_x, max_y;
	int err;

	sdata = devm_kzalloc(dev, sizeof(*sdata), GFP_KERNEL);
	if (!sdata)
		return -ENOMEM;

	sdata->xfer_buf = devm_kzalloc(dev, S6SY761_XFER_BUFFER_SIZE, GFP_KERNEL);
	if (!sdata->xfer_buf)
		return -ENOMEM;

	dev_set_drvdata(dev, sdata);

	sdata->bus_ops = bus_ops;
	sdata->dev = dev;
	sdata->irq = irq;

	err = devm_mutex_init(dev, &sdata->xfer_lock);
	if (err)
		return err;

	sdata->regulators[S6SY761_REGULATOR_VDD].supply = "vdd";
	sdata->regulators[S6SY761_REGULATOR_AVDD].supply = "avdd";
	err = devm_regulator_bulk_get(dev,
				      ARRAY_SIZE(sdata->regulators),
				      sdata->regulators);
	if (err)
		return err;

	sdata->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						    GPIOD_OUT_HIGH);
	if (IS_ERR(sdata->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(sdata->reset_gpio),
				     "failed to get reset GPIO\n");

	err = devm_add_action_or_reset(dev, s6sy761_power_off, sdata);
	if (err)
		return err;

	err = s6sy761_hw_init(sdata, &max_x, &max_y);
	if (err)
		return err;

	sdata->input = devm_input_allocate_device(dev);
	if (!sdata->input)
		return -ENOMEM;

	sdata->input->name = S6SY761_DEV_NAME;
	sdata->input->id.bustype = bus_ops->bustype;
	sdata->input->open = s6sy761_input_open;
	sdata->input->close = s6sy761_input_close;

	input_set_abs_params(sdata->input, ABS_MT_POSITION_X, 0, max_x, 0, 0);
	input_set_abs_params(sdata->input, ABS_MT_POSITION_Y, 0, max_y, 0, 0);
	input_set_abs_params(sdata->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(sdata->input, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
	input_set_abs_params(sdata->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(sdata->input, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
	input_set_abs_params(sdata->input, ABS_MT_PRESSURE, 0, 255, 0, 0);

	touchscreen_parse_properties(sdata->input, true, &sdata->prop);

	if (!input_abs_get_max(sdata->input, ABS_X) ||
	    !input_abs_get_max(sdata->input, ABS_Y)) {
		dev_warn(dev, "the axis have not been set\n");
	}

	err = input_mt_init_slots(sdata->input, sdata->tx_channel,
				  INPUT_MT_DIRECT);
	if (err)
		return err;

	input_set_drvdata(sdata->input, sdata);

	err = input_register_device(sdata->input);
	if (err)
		return err;

	err = devm_request_threaded_irq(dev, sdata->irq, NULL,
					s6sy761_irq_handler,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					"s6sy761_irq", sdata);
	if (err)
		return err;

	pm_runtime_enable(dev);

	return 0;
}
EXPORT_SYMBOL_NS_GPL(s6sy761_probe, "S6SY761");

void s6sy761_remove(struct device *dev)
{
	pm_runtime_disable(dev);
}
EXPORT_SYMBOL_NS_GPL(s6sy761_remove, "S6SY761");

static int s6sy761_runtime_suspend(struct device *dev)
{
	struct s6sy761_data *sdata = dev_get_drvdata(dev);
	u8 mode = S6SY761_APP_SLEEP;

	return s6sy761_write(sdata, S6SY761_APPLICATION_MODE, 1, &mode);
}

static int s6sy761_runtime_resume(struct device *dev)
{
	struct s6sy761_data *sdata = dev_get_drvdata(dev);
	u8 mode = S6SY761_APP_NORMAL;

	return s6sy761_write(sdata, S6SY761_APPLICATION_MODE, 1, &mode);
}

static int s6sy761_suspend(struct device *dev)
{
	struct s6sy761_data *sdata = dev_get_drvdata(dev);

	s6sy761_power_off(sdata);

	return 0;
}

static int s6sy761_resume(struct device *dev)
{
	struct s6sy761_data *sdata = dev_get_drvdata(dev);
	int err;

	err = s6sy761_power_on(sdata);
	if (err)
		return err;

	enable_irq(sdata->irq);

	guard(mutex)(&sdata->input->mutex);

	if (!input_device_enabled(sdata->input))
		return 0;

	err = s6sy761_write(sdata, S6SY761_SENSE_ON, 0, NULL);

	return err;
}

EXPORT_NS_GPL_DEV_PM_OPS(s6sy761_pm_ops, S6SY761) = {
	SYSTEM_SLEEP_PM_OPS(s6sy761_suspend, s6sy761_resume)
	RUNTIME_PM_OPS(s6sy761_runtime_suspend, s6sy761_runtime_resume, NULL)
};

MODULE_AUTHOR("Andi Shyti <andi.shyti@samsung.com>");
MODULE_DESCRIPTION("Samsung S6SY761 Touch Screen core");
MODULE_LICENSE("GPL");
