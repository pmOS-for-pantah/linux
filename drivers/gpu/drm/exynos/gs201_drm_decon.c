// SPDX-License-Identifier: GPL-2.0-only
/*
 * Google gs201 DECON display controller
 *
 * Copyright (c) 2026 sidharthify <wednisegit@gmail.com>
 *
 * The DECON blends windows fed by DPP channels and sends the result to
 * the DSIM. This driver drives one window from DMA channel 0 and runs the
 * DECON in command mode, triggered by the panel TE signal.
 *
 * TODO: The bootloader leaves the DECON, its clocks and power domain,
 * the DSC encoder and the DSIM link configured for the panel, and this
 * driver keeps that setup. Programming the pipeline from reset is still
 * missing, as are the clocks and the power domain.
 */

#include <linux/bitfield.h>
#include <linux/component.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_print.h>
#include <drm/drm_vblank.h>

#include "exynos_drm_crtc.h"
#include "exynos_drm_drv.h"
#include "exynos_drm_fb.h"
#include "exynos_drm_plane.h"

/* DECON registers */
#define DECON_FRAME_COUNT		0x0004
#define DECON_SHD_REG_UP_REQ		0x0050
#define SHD_REG_UP_REQ_ALL_WIN		GENMASK(5, 0)
#define DECON_INT_EN			0x0060
#define INT_EN_FRAME_DONE		BIT(13)
#define INT_EN_FRAME_START		BIT(12)
#define INT_EN_EXTRA			BIT(4)
#define INT_EN				BIT(0)
#define DECON_INT_PEND			0x0070
#define INT_PEND_FRAME_DONE		BIT(13)
#define INT_PEND_FRAME_START		BIT(12)

/* DPP read DMA registers, one 4 KiB block per channel */
#define RDMA_IN_CTRL_0			0x0008
#define RDMA_IMG_FORMAT_MASK		GENMASK(13, 8)
/*
 * The format codes name the bytes from the highest address down, the
 * reverse of DRM fourcc. DRM_FORMAT_XRGB8888 is "BGRX8888" here.
 */
#define RDMA_IMG_FORMAT_XRGB8888	4
#define RDMA_SRC_SIZE			0x0010
#define RDMA_SRC_OFFSET			0x0014
#define RDMA_IMG_SIZE			0x0018
#define RDMA_BASEADDR_Y8		0x0040
#define RDMA_SIZE(w, h)			(((h) << 16) | (w))

#define GS201_DECON_WIN			0
#define GS201_DECON_CHANNEL		0

struct gs201_decon {
	struct device *dev;
	struct drm_device *drm;
	void *dma_priv;
	void __iomem *regs;
	void __iomem *dma;
	struct exynos_drm_crtc *crtc;
	struct exynos_drm_plane plane;
	struct exynos_drm_plane_config plane_config;
	int irq;
};

/* Formats with alpha need blending, which is not set up yet */
static const u32 gs201_decon_formats[] = {
	DRM_FORMAT_XRGB8888,
};

static void __iomem *gs201_decon_rdma(struct gs201_decon *ctx)
{
	return ctx->dma + GS201_DECON_CHANNEL * SZ_4K;
}

static int gs201_decon_enable_vblank(struct exynos_drm_crtc *crtc)
{
	struct gs201_decon *ctx = crtc->ctx;

	writel(INT_PEND_FRAME_DONE | INT_PEND_FRAME_START,
	       ctx->regs + DECON_INT_PEND);
	writel(INT_EN_FRAME_DONE | INT_EN, ctx->regs + DECON_INT_EN);

	return 0;
}

static void gs201_decon_disable_vblank(struct exynos_drm_crtc *crtc)
{
	struct gs201_decon *ctx = crtc->ctx;

	writel(0, ctx->regs + DECON_INT_EN);
}

static enum drm_mode_status gs201_decon_mode_valid(struct exynos_drm_crtc *crtc,
						   const struct drm_display_mode *mode)
{
	/* Only command mode panels behind the DSIM for now */
	return crtc->i80_mode ? MODE_OK : MODE_BAD;
}

static void gs201_decon_update_plane(struct exynos_drm_crtc *crtc,
				     struct exynos_drm_plane *plane)
{
	struct exynos_drm_plane_state *state =
		to_exynos_plane_state(plane->base.state);
	struct gs201_decon *ctx = crtc->ctx;
	struct drm_framebuffer *fb = state->base.fb;
	void __iomem *rdma = gs201_decon_rdma(ctx);
	unsigned int cpp = fb->format->cpp[0];

	writel((readl(rdma + RDMA_IN_CTRL_0) & ~RDMA_IMG_FORMAT_MASK) |
	       FIELD_PREP(RDMA_IMG_FORMAT_MASK, RDMA_IMG_FORMAT_XRGB8888),
	       rdma + RDMA_IN_CTRL_0);

	/* The source width is the full line, set by the pitch */
	writel(RDMA_SIZE(fb->pitches[0] / cpp, fb->height), rdma + RDMA_SRC_SIZE);
	writel(RDMA_SIZE(state->src.x, state->src.y), rdma + RDMA_SRC_OFFSET);
	writel(RDMA_SIZE(state->src.w, state->src.h), rdma + RDMA_IMG_SIZE);
	writel(exynos_drm_fb_dma_addr(fb, 0), rdma + RDMA_BASEADDR_Y8);
}

static void gs201_decon_atomic_flush(struct exynos_drm_crtc *crtc)
{
	struct gs201_decon *ctx = crtc->ctx;

	/*
	 * Latch the new window and DMA setup at the next frame start. The
	 * window the bootloader routed channel 0 to is not known, so ask
	 * for all of them.
	 */
	writel(SHD_REG_UP_REQ_ALL_WIN, ctx->regs + DECON_SHD_REG_UP_REQ);

	exynos_crtc_handle_event(crtc);
}

static const struct exynos_drm_crtc_ops gs201_decon_crtc_ops = {
	.enable_vblank = gs201_decon_enable_vblank,
	.disable_vblank = gs201_decon_disable_vblank,
	.mode_valid = gs201_decon_mode_valid,
	.update_plane = gs201_decon_update_plane,
	.atomic_flush = gs201_decon_atomic_flush,
};

static irqreturn_t gs201_decon_irq(int irq, void *data)
{
	struct gs201_decon *ctx = data;
	u32 pend;

	pend = readl(ctx->regs + DECON_INT_PEND);
	writel(pend, ctx->regs + DECON_INT_PEND);

	if ((pend & INT_PEND_FRAME_DONE) && ctx->crtc)
		drm_crtc_handle_vblank(&ctx->crtc->base);

	return IRQ_HANDLED;
}

static int gs201_decon_bind(struct device *dev, struct device *master, void *data)
{
	struct gs201_decon *ctx = dev_get_drvdata(dev);
	struct drm_device *drm = data;
	int ret;

	ctx->drm = drm;

	ctx->plane_config.pixel_formats = gs201_decon_formats;
	ctx->plane_config.num_pixel_formats = ARRAY_SIZE(gs201_decon_formats);
	ctx->plane_config.type = DRM_PLANE_TYPE_PRIMARY;

	ret = exynos_plane_init(drm, &ctx->plane, GS201_DECON_WIN,
				&ctx->plane_config);
	if (ret)
		return ret;

	ctx->crtc = exynos_drm_crtc_create(drm, &ctx->plane.base,
					   EXYNOS_DISPLAY_TYPE_LCD,
					   &gs201_decon_crtc_ops, ctx);
	if (IS_ERR(ctx->crtc))
		return PTR_ERR(ctx->crtc);

	return exynos_drm_register_dma(drm, dev, &ctx->dma_priv);
}

static void gs201_decon_unbind(struct device *dev, struct device *master,
			       void *data)
{
	struct gs201_decon *ctx = dev_get_drvdata(dev);

	writel(0, ctx->regs + DECON_INT_EN);
	exynos_drm_unregister_dma(ctx->drm, dev, &ctx->dma_priv);
	ctx->crtc = NULL;
}

static const struct component_ops gs201_decon_component_ops = {
	.bind = gs201_decon_bind,
	.unbind = gs201_decon_unbind,
};

static int gs201_decon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gs201_decon *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = dev;

	ctx->regs = devm_platform_ioremap_resource_byname(pdev, "decon");
	if (IS_ERR(ctx->regs))
		return PTR_ERR(ctx->regs);

	ctx->dma = devm_platform_ioremap_resource_byname(pdev, "dma");
	if (IS_ERR(ctx->dma))
		return PTR_ERR(ctx->dma);

	ctx->irq = platform_get_irq_byname(pdev, "frame-done");
	if (ctx->irq < 0)
		return ctx->irq;

	writel(0, ctx->regs + DECON_INT_EN);

	ret = devm_request_irq(dev, ctx->irq, gs201_decon_irq, 0,
			       dev_name(dev), ctx);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request IRQ\n");

	platform_set_drvdata(pdev, ctx);

	return component_add(dev, &gs201_decon_component_ops);
}

static void gs201_decon_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &gs201_decon_component_ops);
}

static const struct of_device_id gs201_decon_of_match[] = {
	{ .compatible = "google,gs201-decon" },
	{ }
};
MODULE_DEVICE_TABLE(of, gs201_decon_of_match);

struct platform_driver gs201_decon_driver = {
	.probe = gs201_decon_probe,
	.remove = gs201_decon_remove,
	.driver = {
		.name = "gs201-decon",
		.of_match_table = gs201_decon_of_match,
	},
};
