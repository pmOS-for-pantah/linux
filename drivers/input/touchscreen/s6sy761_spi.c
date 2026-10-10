// SPDX-License-Identifier: GPL-2.0
// Samsung S6SY761 SPI Touchscreen device driver
//
// Copyright (c) 2026 Lukas Timmermann <foss@timmermann.space>

#include <linux/delay.h>
#include <linux/input.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>

#include "s6sy761_core.h"

#define S6SY761_SPI_SYNC_CODE		0xaa
#define S6SY761_SPI_HEADER_SIZE		5 /* sync, 2 len, 2 reserved */
#define S6SY761_SPI_CHECKSUM_SIZE	1
#define S6SY761_SPI_READ_HEADER_SIZE	7 /* sync, 4 len, opcode, status */

/* opcodes that need a longer wait before the reply is ready */
#define S6SY761_SPI_CMD_FW_INTEGRITY	0x21
#define S6SY761_SPI_CMD_READ_ALL_EVENT	0x61

static u8 s6sy761_spi_checksum(const u8 *buf, size_t len)
{
	u8 sum = 0;
	size_t i;

	for (i = 0; i < len; i++)
		sum += buf[i];

	return sum;
}

static int s6sy761_spi_write(struct device *dev, u8 *xfer_buf,
			     u8 cmd, size_t length, const void *values)
{
	struct spi_device *spi = to_spi_device(dev);

	/* header + opcode + payload + checksum */
	size_t frame_len = S6SY761_SPI_HEADER_SIZE + 1 + length +
				S6SY761_SPI_CHECKSUM_SIZE;
	size_t xfer_len = ALIGN(frame_len, 4);

	if (xfer_len > S6SY761_XFER_BUFFER_SIZE)
		return -EINVAL;

	xfer_buf[0] = S6SY761_SPI_SYNC_CODE;
	put_unaligned_be16(length + 1, xfer_buf + 1);
	xfer_buf[3] = 0;
	xfer_buf[4] = 0;
	xfer_buf[5] = cmd;

	if (length)
		memcpy(xfer_buf + 6, values, length);

	xfer_buf[frame_len - 1] = s6sy761_spi_checksum(xfer_buf, frame_len - 1);

	/* the chip expects transfers in multiples of 4 bytes */
	memset(xfer_buf + frame_len, 0, xfer_len - frame_len);

	return spi_write(spi, xfer_buf, xfer_len);
}

static unsigned int s6sy761_spi_read_delay_us(u8 cmd)
{
	switch (cmd) {
	case S6SY761_SPI_CMD_FW_INTEGRITY:
		return 20000;
	case S6SY761_SPI_CMD_READ_ALL_EVENT:
		return 550;
	default:
		return 100;
	}
}

static int s6sy761_spi_read(struct device *dev, u8 *xfer_buf,
			    u8 cmd, size_t length, void *values)
{
	struct spi_device *spi = to_spi_device(dev);
	struct spi_transfer reply = { };
	/* request: header + opcode + checksum, no payload */
	size_t req_len = S6SY761_SPI_HEADER_SIZE + 1 +
			 S6SY761_SPI_CHECKSUM_SIZE;
	/* reply: header + data + checksum */
	size_t resp_len = S6SY761_SPI_READ_HEADER_SIZE + length +
			  S6SY761_SPI_CHECKSUM_SIZE;
	int ret;

	if (ALIGN(resp_len, 4) > S6SY761_XFER_BUFFER_SIZE)
		return -EINVAL;

	/* tell the chip what to read and how much */
	xfer_buf[0] = S6SY761_SPI_SYNC_CODE;
	put_unaligned_be16(1, xfer_buf + 1);		/* write length: opcode */
	put_unaligned_be16(length, xfer_buf + 3);	/* read length */
	xfer_buf[5] = cmd;
	xfer_buf[req_len - 1] = s6sy761_spi_checksum(xfer_buf, req_len - 1);
	memset(xfer_buf + req_len, 0, ALIGN(req_len, 4) - req_len);

	ret = spi_write(spi, xfer_buf, ALIGN(req_len, 4));
	if (ret)
		return ret;

	/* give the chip time to prepare the reply */
	fsleep(s6sy761_spi_read_delay_us(cmd));

	memset(xfer_buf, 0, ALIGN(resp_len, 4));
	reply.tx_buf = xfer_buf;
	reply.rx_buf = xfer_buf;
	reply.len = ALIGN(resp_len, 4);
	ret = spi_sync_transfer(spi, &reply, 1);
	if (ret)
		return ret;

	if (xfer_buf[0] != S6SY761_SPI_SYNC_CODE ||
	    xfer_buf[5] != cmd ||
	    xfer_buf[resp_len - 1] !=
			s6sy761_spi_checksum(xfer_buf, resp_len - 1))
		return -EIO;

	memcpy(values, xfer_buf + S6SY761_SPI_READ_HEADER_SIZE, length);

	return 0;
}

static const struct s6sy761_bus_ops s6sy761_spi_bus_ops = {
	.bustype = BUS_SPI,
	.read = s6sy761_spi_read,
	.write = s6sy761_spi_write,
};

static int s6sy761_spi_probe(struct spi_device *spi)
{
	return s6sy761_probe(&spi->dev, &s6sy761_spi_bus_ops, spi->irq);
}

static void s6sy761_spi_remove(struct spi_device *spi)
{
	s6sy761_remove(&spi->dev);
}

static const struct of_device_id s6sy761_of_match[] = {
	{ .compatible = "samsung,s6sy791", },
	{ },
};
MODULE_DEVICE_TABLE(of, s6sy761_of_match);

static const struct spi_device_id s6sy761_id[] = {
	{ "s6sy791" },
	{ }
};
MODULE_DEVICE_TABLE(spi, s6sy761_id);

static struct spi_driver s6sy761_driver = {
	.driver = {
		.name = S6SY761_DEV_NAME,
		.dev_groups = s6sy761_sysfs_groups,
		.of_match_table = s6sy761_of_match,
		.pm = pm_ptr(&s6sy761_pm_ops),
	},
	.probe = s6sy761_spi_probe,
	.remove = s6sy761_spi_remove,
	.id_table = s6sy761_id,
};

module_spi_driver(s6sy761_driver);

MODULE_IMPORT_NS("S6SY761");
MODULE_AUTHOR("Lukas Timmermann <foss@timmermann.space>");
MODULE_DESCRIPTION("Samsung S6SY791 Touch Screen");
MODULE_LICENSE("GPL");
