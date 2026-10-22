// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google gs201 DSIM MIPI DSI host
 *
 * Copyright (c) 2026 sidharthify <wednisegit@gmail.com>
 *
 * The DSIM turns the DECON output into DSI packets and also carries the
 * panel commands, through a packet header and a payload FIFO.
 *
 * TODO: The bootloader leaves the D-PHY PLL, the lanes, the timings and
 * the DSC slice setup running for the panel, and this driver keeps that
 * link. Bringing the link up from reset is still missing.
 */

#include <linux/component.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <video/mipi_display.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_encoder.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>

#include "exynos_drm_crtc.h"
#include "exynos_drm_drv.h"

#define DSIM_ESCMODE			0x002c
#define DSIM_ESCMODE_CMD_LPDT		BIT(7)
#define DSIM_INTSRC			0x0050
#define DSIM_INTSRC_RX_DATA_DONE	BIT(18)
#define DSIM_PKTHDR			0x0058
#define DSIM_PKTHDR_BTA			BIT(24)
#define DSIM_PAYLOAD			0x005c
#define DSIM_RXFIFO			0x0060
#define DSIM_FIFOCTRL			0x0068
#define DSIM_FIFOCTRL_EMPTY_RX		BIT(12)
#define DSIM_FIFOCTRL_FULL_PH_SFR	BIT(11)
#define DSIM_FIFOCTRL_EMPTY_PH_SFR	BIT(10)
#define DSIM_FIFOCTRL_FULL_PL_SFR	BIT(9)
#define DSIM_FIFOCTRL_EMPTY_PL_SFR	BIT(8)

#define DSIM_TX_TIMEOUT_US		100000
#define DSIM_RX_TIMEOUT_US		100000

struct gs201_dsim {
	struct device *dev;
	void __iomem *regs;
	struct mipi_dsi_host host;
	struct drm_encoder encoder;
	struct drm_bridge *panel_bridge;
	unsigned long mode_flags;
	/* Serialises use of the command FIFOs */
	struct mutex lock;
};

static inline struct gs201_dsim *host_to_dsim(struct mipi_dsi_host *host)
{
	return container_of(host, struct gs201_dsim, host);
}

static int gs201_dsim_wait_tx(struct gs201_dsim *dsim)
{
	u32 empty = DSIM_FIFOCTRL_EMPTY_PH_SFR | DSIM_FIFOCTRL_EMPTY_PL_SFR;
	u32 val;

	return readl_poll_timeout(dsim->regs + DSIM_FIFOCTRL, val,
				  (val & empty) == empty, 10,
				  DSIM_TX_TIMEOUT_US);
}

static void gs201_dsim_write_payload(struct gs201_dsim *dsim, const u8 *buf,
				     size_t len)
{
	while (len) {
		size_t n = min_t(size_t, len, 4);
		u32 word = 0;
		size_t i;

		for (i = 0; i < n; i++)
			word |= buf[i] << (8 * i);

		writel(word, dsim->regs + DSIM_PAYLOAD);
		buf += n;
		len -= n;
	}
}

static ssize_t gs201_dsim_read_response(struct gs201_dsim *dsim,
					const struct mipi_dsi_msg *msg)
{
	u8 *rx = msg->rx_buf;
	size_t len = 0, count;
	u32 val, word;
	int ret;

	ret = readl_poll_timeout(dsim->regs + DSIM_FIFOCTRL, val,
				 !(val & DSIM_FIFOCTRL_EMPTY_RX), 10,
				 DSIM_RX_TIMEOUT_US);
	if (ret)
		return ret;

	word = readl(dsim->regs + DSIM_RXFIFO);

	switch (word & 0xff) {
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
		count = 1;
		break;
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
		count = 2;
		break;
	case MIPI_DSI_RX_GENERIC_LONG_READ_RESPONSE:
	case MIPI_DSI_RX_DCS_LONG_READ_RESPONSE:
		count = (word >> 8) & 0xffff;
		while (len < count) {
			word = readl(dsim->regs + DSIM_RXFIFO);
			for (int i = 0; i < 4 && len < count; i++, len++)
				if (len < msg->rx_len)
					rx[len] = word >> (8 * i);
		}
		return min(count, msg->rx_len);
	case MIPI_DSI_RX_ACKNOWLEDGE_AND_ERROR_REPORT:
		dev_err(dsim->dev, "Panel reported error 0x%04x\n",
			(word >> 8) & 0xffff);
		return -EIO;
	default:
		dev_err(dsim->dev, "Unexpected response 0x%08x\n", word);
		return -EIO;
	}

	for (len = 0; len < count && len < msg->rx_len; len++)
		rx[len] = word >> (8 * (len + 1));

	return len;
}

static ssize_t gs201_dsim_transfer(struct mipi_dsi_host *host,
				   const struct mipi_dsi_msg *msg)
{
	struct gs201_dsim *dsim = host_to_dsim(host);
	struct mipi_dsi_packet packet;
	bool read = msg->rx_len && msg->rx_buf;
	u32 val;
	int ret;

	ret = mipi_dsi_create_packet(&packet, msg);
	if (ret)
		return ret;

	mutex_lock(&dsim->lock);

	val = readl(dsim->regs + DSIM_ESCMODE);
	if (msg->flags & MIPI_DSI_MSG_USE_LPM)
		val |= DSIM_ESCMODE_CMD_LPDT;
	else
		val &= ~DSIM_ESCMODE_CMD_LPDT;
	writel(val, dsim->regs + DSIM_ESCMODE);

	if (read) {
		/* Drop anything left over from an earlier read */
		while (!(readl(dsim->regs + DSIM_FIFOCTRL) & DSIM_FIFOCTRL_EMPTY_RX))
			readl(dsim->regs + DSIM_RXFIFO);
		writel(DSIM_INTSRC_RX_DATA_DONE, dsim->regs + DSIM_INTSRC);

		/* Let the panel answer with up to rx_len bytes */
		writel(MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE |
		       (msg->rx_len & 0xff) << 8 | (msg->rx_len >> 8) << 16,
		       dsim->regs + DSIM_PKTHDR);
		ret = gs201_dsim_wait_tx(dsim);
		if (ret)
			goto out;
	}

	gs201_dsim_write_payload(dsim, packet.payload, packet.payload_length);
	writel((read ? DSIM_PKTHDR_BTA : 0) | packet.header[0] |
	       packet.header[1] << 8 | packet.header[2] << 16,
	       dsim->regs + DSIM_PKTHDR);

	ret = gs201_dsim_wait_tx(dsim);
	if (ret) {
		dev_err(dsim->dev, "Timed out sending command 0x%02x\n",
			packet.header[0]);
		goto out;
	}

	ret = read ? gs201_dsim_read_response(dsim, msg) : packet.size;

out:
	mutex_unlock(&dsim->lock);
	return ret;
}

static int gs201_dsim_bind(struct device *dev, struct device *master, void *data)
{
	struct gs201_dsim *dsim = dev_get_drvdata(dev);
	struct drm_encoder *encoder = &dsim->encoder;
	struct drm_device *drm = data;
	struct drm_connector *connector;
	struct exynos_drm_crtc *crtc;
	int ret;

	ret = drm_simple_encoder_init(drm, encoder, DRM_MODE_ENCODER_DSI);
	if (ret)
		return ret;

	ret = exynos_drm_set_possible_crtcs(encoder, EXYNOS_DISPLAY_TYPE_LCD);
	if (ret < 0)
		return ret;

	crtc = exynos_drm_crtc_get_by_type(drm, EXYNOS_DISPLAY_TYPE_LCD);
	if (crtc)
		crtc->i80_mode = !(dsim->mode_flags & MIPI_DSI_MODE_VIDEO);

	ret = drm_bridge_attach(encoder, dsim->panel_bridge, NULL,
				DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (ret)
		return ret;

	connector = drm_bridge_connector_init(drm, encoder);
	if (IS_ERR(connector))
		return PTR_ERR(connector);

	return drm_connector_attach_encoder(connector, encoder);
}

static void gs201_dsim_unbind(struct device *dev, struct device *master,
			      void *data)
{
}

static const struct component_ops gs201_dsim_component_ops = {
	.bind = gs201_dsim_bind,
	.unbind = gs201_dsim_unbind,
};

/*
 * Join the DRM device only once the panel is there, so that the device
 * never comes up without its connector.
 */
static int gs201_dsim_host_attach(struct mipi_dsi_host *host,
				  struct mipi_dsi_device *device)
{
	struct gs201_dsim *dsim = host_to_dsim(host);
	struct drm_panel *panel;

	panel = of_drm_find_panel(device->dev.of_node);
	if (IS_ERR(panel))
		return PTR_ERR(panel);

	dsim->panel_bridge = devm_drm_panel_bridge_add_typed(dsim->dev, panel,
							     DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(dsim->panel_bridge))
		return PTR_ERR(dsim->panel_bridge);

	dsim->mode_flags = device->mode_flags;

	return component_add(dsim->dev, &gs201_dsim_component_ops);
}

static int gs201_dsim_host_detach(struct mipi_dsi_host *host,
				  struct mipi_dsi_device *device)
{
	struct gs201_dsim *dsim = host_to_dsim(host);

	component_del(dsim->dev, &gs201_dsim_component_ops);

	return 0;
}

static const struct mipi_dsi_host_ops gs201_dsim_host_ops = {
	.attach = gs201_dsim_host_attach,
	.detach = gs201_dsim_host_detach,
	.transfer = gs201_dsim_transfer,
};

static int gs201_dsim_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gs201_dsim *dsim;
	int ret;

	dsim = devm_kzalloc(dev, sizeof(*dsim), GFP_KERNEL);
	if (!dsim)
		return -ENOMEM;

	dsim->dev = dev;

	ret = devm_mutex_init(dev, &dsim->lock);
	if (ret)
		return ret;

	dsim->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dsim->regs))
		return PTR_ERR(dsim->regs);

	dsim->host.dev = dev;
	dsim->host.ops = &gs201_dsim_host_ops;

	platform_set_drvdata(pdev, dsim);

	return mipi_dsi_host_register(&dsim->host);
}

static void gs201_dsim_remove(struct platform_device *pdev)
{
	struct gs201_dsim *dsim = platform_get_drvdata(pdev);

	mipi_dsi_host_unregister(&dsim->host);
}

static const struct of_device_id gs201_dsim_of_match[] = {
	{ .compatible = "google,gs201-dsim" },
	{ }
};
MODULE_DEVICE_TABLE(of, gs201_dsim_of_match);

struct platform_driver gs201_dsim_driver = {
	.probe = gs201_dsim_probe,
	.remove = gs201_dsim_remove,
	.driver = {
		.name = "gs201-dsim",
		.of_match_table = gs201_dsim_of_match,
	},
};
