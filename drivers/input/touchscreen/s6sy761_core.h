/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _S6SY761_H
#define _S6SY761_H

#include <linux/align.h>
#include <linux/types.h>

#define S6SY761_EVENT_SIZE			8
#define S6SY761_EVENT_COUNT			32
/* largest SPI read: reply header + all events + checksum */
#define S6SY761_XFER_BUFFER_SIZE	ALIGN(7 + S6SY761_EVENT_SIZE * S6SY761_EVENT_COUNT + 1, 4)

#define S6SY761_DEV_NAME	"s6sy761"

struct device;
struct dev_pm_ops;
struct attribute_group;

struct s6sy761_bus_ops {
	u16 bustype;
	int (*write)(struct device *dev, u8 *xfer_buf,
		     u8 cmd, size_t length, const void *values);
	int (*read)(struct device *dev, u8 *xfer_buf,
		    u8 cmd, size_t length, void *values);
};

extern const struct dev_pm_ops s6sy761_pm_ops;

extern const struct attribute_group *s6sy761_sysfs_groups[];

int s6sy761_probe(struct device *dev, const struct s6sy761_bus_ops *bus_ops, int irq);
void s6sy761_remove(struct device *dev);

#endif
