// SPDX-License-Identifier: GPL-2.0
// Samsung S6SY761 I2C Touchscreen device driver
//
// Copyright (c) 2017 Samsung Electronics Co., Ltd.
// Copyright (c) 2017 Andi Shyti <andi@etezian.org>
// Copyright (c) 2026 Lukas Timmermann <foss@timmermann.space>

#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>

#include "s6sy761_core.h"

static int s6sy761_i2c_write(struct device *dev, u8 *xfer_buf,
			     u8 cmd, size_t length, const void *values)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret;

	if (length + 1 > S6SY761_XFER_BUFFER_SIZE)
		return -EINVAL;

	xfer_buf[0] = cmd;

	if (length)
		memcpy(xfer_buf + 1, values, length);

	ret = i2c_master_send(client, xfer_buf, length + 1);
	if (ret < 0)
		return ret;
	if (ret != length + 1)
		return -EIO;

	return 0;
}

static int s6sy761_i2c_read(struct device *dev, u8 *xfer_buf,
			    u8 cmd, size_t length, void *values)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_msg msgs[2] = {
		{
			.addr	= client->addr,
			.len	= 1,
			.buf	= &cmd,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= values,
		},
	};
	int ret;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0)
		return ret;

	return ret == ARRAY_SIZE(msgs) ? 0 : -EIO;
}

static const struct s6sy761_bus_ops s6sy761_i2c_bus_ops = {
	.bustype = BUS_I2C,
	.read = s6sy761_i2c_read,
	.write = s6sy761_i2c_write,
};

static int s6sy761_i2c_probe(struct i2c_client *client)
{
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	return s6sy761_probe(&client->dev, &s6sy761_i2c_bus_ops, client->irq);
}

static void s6sy761_i2c_remove(struct i2c_client *client)
{
	s6sy761_remove(&client->dev);
}

static const struct of_device_id s6sy761_of_match[] = {
	{ .compatible = "samsung,s6sy761", },
	{ },
};
MODULE_DEVICE_TABLE(of, s6sy761_of_match);

static const struct i2c_device_id s6sy761_id[] = {
	{ "s6sy761" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, s6sy761_id);

static struct i2c_driver s6sy761_driver = {
	.driver = {
		.name = S6SY761_DEV_NAME,
		.dev_groups = s6sy761_sysfs_groups,
		.of_match_table = s6sy761_of_match,
		.pm = pm_ptr(&s6sy761_pm_ops),
	},
	.probe = s6sy761_i2c_probe,
	.remove = s6sy761_i2c_remove,
	.id_table = s6sy761_id,
};

module_i2c_driver(s6sy761_driver);

MODULE_IMPORT_NS("S6SY761");
MODULE_AUTHOR("Andi Shyti <andi.shyti@samsung.com>");
MODULE_DESCRIPTION("Samsung S6SY761 Touch Screen");
MODULE_LICENSE("GPL");
