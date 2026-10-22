// SPDX-License-Identifier: GPL-2.0-only
/*
 * FocalTech FT3658 SPI touchscreen controller driver
 *
 * Based on the FocalTech touch driver for Google Pixel devices,
 * Copyright (c) 2012-2020, FocalTech Systems, Ltd.
 */

#include <linux/bitfield.h>
#include <linux/crc-ccitt.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>

#define FT3658_REG_TOUCH_DATA		0x01
#define FT3658_REG_CHIP_ID		0xa3
#define FT3658_REG_FW_VER		0xa6
#define FT3658_REG_CHIP_ID2		0x9f

#define FT3658_CHIP_ID			0x56
#define FT3658_CHIP_ID2			0x52

/* Read command header, then dummy bytes before the data phase */
#define FT3658_CMD_READ			0x80
#define FT3658_CMD_CRC			0x20
#define FT3658_HDR_LEN			4
#define FT3658_DUMMY_LEN		3
#define FT3658_DATA_OFS			(FT3658_HDR_LEN + FT3658_DUMMY_LEN)
#define FT3658_CRC_LEN			2
#define FT3658_STATUS_ERR		0xa0
#define FT3658_XFER_RETRIES		3
#define FT3658_CS_HIGH_US		150

#define FT3658_MAX_TOUCHES		10
#define FT3658_TOUCH_LEN		6
#define FT3658_TOUCH_BASE		2
/* Status bytes, touch records and the trailing firmware state */
#define FT3658_TOUCH_DATA_LEN		90

#define FT3658_EVENT_UP			1
#define FT3658_TOUCH_EVENT		GENMASK(7, 6)
#define FT3658_TOUCH_ID			GENMASK(7, 4)
#define FT3658_TOUCH_HI			GENMASK(3, 0)
#define FT3658_TOUCH_SIZE		GENMASK(7, 1)
/* Contact size unit in pixels */
#define FT3658_MM2PX			8

#define FT3658_BUF_LEN			128

struct ft3658 {
	struct spi_device *spi;
	struct input_dev *input;
	struct touchscreen_properties prop;
	struct gpio_desc *reset_gpio;
	struct mutex lock;
	u8 tx[FT3658_BUF_LEN] ____cacheline_aligned;
	u8 rx[FT3658_BUF_LEN];
	u8 data[FT3658_TOUCH_DATA_LEN];
};

/* CRC-16/MCRF4XX, as calculated by the controller over the read data */
static u16 ft3658_crc(const u8 *data, size_t len)
{
	return crc_ccitt(0xffff, data, len);
}

static int ft3658_xfer(struct ft3658 *ts, size_t len)
{
	struct spi_transfer xfer = {
		.tx_buf = ts->tx,
		.rx_buf = ts->rx,
		.len = len,
	};
	int ret;

	ret = spi_sync_transfer(ts->spi, &xfer, 1);
	usleep_range(FT3658_CS_HIGH_US, FT3658_CS_HIGH_US + 50);
	if (ret)
		return ret;

	return ts->rx[3] & FT3658_STATUS_ERR ? -EIO : 0;
}

static int ft3658_read(struct ft3658 *ts, u8 reg, u8 *buf, size_t len)
{
	size_t xfer_len, data_len;
	int i, ret = -EIO;
	u16 crc;

	data_len = len;
	xfer_len = FT3658_DATA_OFS + len + FT3658_CRC_LEN;
	/* The controller expects DMA sized transfers to be word aligned */
	if (xfer_len > 64) {
		data_len += ALIGN(xfer_len, 4) - xfer_len;
		xfer_len = ALIGN(xfer_len, 4);
	}
	if (xfer_len > FT3658_BUF_LEN)
		return -EINVAL;

	guard(mutex)(&ts->lock);

	for (i = 0; i < FT3658_XFER_RETRIES; i++) {
		memset(ts->tx, 0, xfer_len);
		ts->tx[0] = reg;
		ts->tx[1] = FT3658_CMD_READ | FT3658_CMD_CRC;
		ts->tx[2] = data_len >> 8;
		ts->tx[3] = data_len & 0xff;

		ret = ft3658_xfer(ts, xfer_len);
		if (ret)
			continue;

		crc = get_unaligned_le16(&ts->rx[xfer_len - FT3658_CRC_LEN]);
		if (ft3658_crc(&ts->rx[FT3658_DATA_OFS],
			       xfer_len - FT3658_DATA_OFS - FT3658_CRC_LEN) != crc) {
			ret = -EBADMSG;
			continue;
		}

		memcpy(buf, &ts->rx[FT3658_DATA_OFS], len);
		return 0;
	}

	return ret;
}

static int ft3658_read_reg(struct ft3658 *ts, u8 reg, u8 *val)
{
	return ft3658_read(ts, reg, val, 1);
}

static void ft3658_report(struct ft3658 *ts)
{
	struct input_dev *input = ts->input;
	const u8 *t;
	unsigned int x, y, major, minor;
	int i, id;

	for (i = 0; i < FT3658_MAX_TOUCHES; i++) {
		t = &ts->data[FT3658_TOUCH_BASE + i * FT3658_TOUCH_LEN];

		id = FIELD_GET(FT3658_TOUCH_ID, t[2]);
		if (id >= FT3658_MAX_TOUCHES)
			break;

		input_mt_slot(input, id);
		if (FIELD_GET(FT3658_TOUCH_EVENT, t[0]) == FT3658_EVENT_UP) {
			input_mt_report_slot_inactive(input);
			continue;
		}

		x = FIELD_GET(FT3658_TOUCH_HI, t[0]) << 8 | t[1];
		y = FIELD_GET(FT3658_TOUCH_HI, t[2]) << 8 | t[3];
		minor = FIELD_GET(FT3658_TOUCH_SIZE, t[4]) * FT3658_MM2PX;
		major = FIELD_GET(FT3658_TOUCH_SIZE, t[5]) * FT3658_MM2PX;

		input_mt_report_slot_state(input, MT_TOOL_FINGER, true);
		touchscreen_report_pos(input, &ts->prop, x, y, true);
		input_report_abs(input, ABS_MT_TOUCH_MAJOR, max(major, minor));
		input_report_abs(input, ABS_MT_TOUCH_MINOR, min(major, minor));
	}

	input_mt_sync_frame(input);
	input_sync(input);
}

static irqreturn_t ft3658_irq(int irq, void *dev_id)
{
	struct ft3658 *ts = dev_id;
	int ret;

	ret = ft3658_read(ts, FT3658_REG_TOUCH_DATA, ts->data, sizeof(ts->data));
	if (ret) {
		dev_err_ratelimited(&ts->spi->dev,
				    "failed to read touch data: %d\n", ret);
		return IRQ_HANDLED;
	}

	/* The controller returns all ones while it recovers from a reset */
	if (ts->data[1] == 0xff && ts->data[2] == 0xff)
		return IRQ_HANDLED;

	ft3658_report(ts);

	return IRQ_HANDLED;
}

static void ft3658_reset(struct ft3658 *ts)
{
	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ts->reset_gpio, 0);
	msleep(200);
}

static int ft3658_identify(struct ft3658 *ts)
{
	struct device *dev = &ts->spi->dev;
	u8 id = 0, id2 = 0, fw = 0;
	int i, ret;

	for (i = 0; i < 5; i++) {
		ret = ft3658_read_reg(ts, FT3658_REG_CHIP_ID, &id);
		if (!ret)
			ret = ft3658_read_reg(ts, FT3658_REG_CHIP_ID2, &id2);
		if (!ret && id == FT3658_CHIP_ID && id2 == FT3658_CHIP_ID2)
			break;
		msleep(200);
	}

	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to read the chip ID, response %*ph\n",
				     12, ts->rx);
	if (id != FT3658_CHIP_ID || id2 != FT3658_CHIP_ID2)
		return dev_err_probe(dev, -ENODEV,
				     "unexpected chip ID %02x%02x\n", id, id2);

	ft3658_read_reg(ts, FT3658_REG_FW_VER, &fw);
	dev_info(dev, "FT3658 chip ID %02x%02x, firmware %02x\n", id, id2, fw);

	return 0;
}

static int ft3658_probe(struct spi_device *spi)
{
	static const char * const supplies[] = { "avdd", "vdd" };
	struct device *dev = &spi->dev;
	struct input_dev *input;
	struct ft3658 *ts;
	int ret;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->spi = spi;
	ret = devm_mutex_init(dev, &ts->lock);
	if (ret)
		return ret;

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return ret;

	ret = devm_regulator_bulk_get_enable(dev, ARRAY_SIZE(supplies),
					     supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable supplies\n");

	ts->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->reset_gpio),
				     "failed to get the reset GPIO\n");

	ft3658_reset(ts);

	ret = ft3658_identify(ts);
	if (ret)
		return ret;

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	ts->input = input;
	input->name = "FocalTech FT3658 Touchscreen";
	input->id.bustype = BUS_SPI;

	input_set_capability(input, EV_ABS, ABS_MT_POSITION_X);
	input_set_capability(input, EV_ABS, ABS_MT_POSITION_Y);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);

	touchscreen_parse_properties(input, true, &ts->prop);
	if (!ts->prop.max_x || !ts->prop.max_y)
		return dev_err_probe(dev, -EINVAL,
				     "touchscreen-size-x/y are required\n");

	ret = input_mt_init_slots(input, FT3658_MAX_TOUCHES,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return ret;

	ret = input_register_device(input);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, spi->irq, NULL, ft3658_irq,
					IRQF_ONESHOT, "ft3658", ts);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request the IRQ\n");

	return 0;
}

static const struct of_device_id ft3658_of_match[] = {
	{ .compatible = "focaltech,ft3658" },
	{ }
};
MODULE_DEVICE_TABLE(of, ft3658_of_match);

static const struct spi_device_id ft3658_spi_ids[] = {
	{ "ft3658" },
	{ }
};
MODULE_DEVICE_TABLE(spi, ft3658_spi_ids);

static struct spi_driver ft3658_driver = {
	.driver = {
		.name = "ft3658",
		.of_match_table = ft3658_of_match,
	},
	.id_table = ft3658_spi_ids,
	.probe = ft3658_probe,
};
module_spi_driver(ft3658_driver);

MODULE_DESCRIPTION("FocalTech FT3658 SPI touchscreen driver");
MODULE_LICENSE("GPL");
