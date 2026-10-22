// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos ACPM protocol based clock driver.
 *
 * Copyright 2025 Linaro Ltd.
 */

#include <linux/array_size.h>
#include <linux/clk-provider.h>
#include <linux/container_of.h>
#include <linux/device/devres.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/units.h>

struct acpm_clk {
	u32 id;
	struct clk_hw hw;
	unsigned int mbox_chan_id;
	struct acpm_handle *handle;
};

struct acpm_clk_variant {
	const char *name;
};

/**
 * struct acpm_clk_pair - an operating point of a clock spanning two domains
 * @rate:	rate of the primary domain, in kHz
 * @rate2:	rate of the secondary domain at this point, in kHz
 */
struct acpm_clk_pair {
	u32 rate;
	u32 rate2;
};

/**
 * struct acpm_clk_paired_data - a clock that scales two DVFS domains together
 * @name:	clock name
 * @id:		DVFS domain that sets the rate of the clock
 * @id2:	DVFS domain that follows it, never slower than @id
 * @pairs:	operating points, in ascending order of @rate
 * @nr_pairs:	number of operating points
 */
struct acpm_clk_paired_data {
	const char *name;
	unsigned int id;
	unsigned int id2;
	const struct acpm_clk_pair *pairs;
	unsigned int nr_pairs;
};

struct acpm_clk_driver_data {
	const struct acpm_clk_variant *clks;
	unsigned int nr_clks;
	const struct acpm_clk_paired_data *paired;
	unsigned int nr_paired;
	unsigned int mbox_chan_id;
};

struct acpm_clk_paired {
	struct clk_hw hw;
	const struct acpm_clk_paired_data *data;
	unsigned int mbox_chan_id;
	struct acpm_handle *handle;
};

#define to_acpm_clk(clk) container_of(clk, struct acpm_clk, hw)
#define to_acpm_clk_paired(clk) container_of(clk, struct acpm_clk_paired, hw)

#define ACPM_CLK(cname)					\
	{						\
		.name		= cname,		\
	}

static const struct acpm_clk_variant gs101_acpm_clks[] = {
	ACPM_CLK("mif"),
	ACPM_CLK("int"),
	ACPM_CLK("cpucl0"),
	ACPM_CLK("cpucl1"),
	ACPM_CLK("cpucl2"),
	ACPM_CLK("g3d"),
	ACPM_CLK("g3dl2"),
	ACPM_CLK("tpu"),
	ACPM_CLK("intcam"),
	ACPM_CLK("tnr"),
	ACPM_CLK("cam"),
	ACPM_CLK("mfc"),
	ACPM_CLK("disp"),
	ACPM_CLK("bo"),
};

static const struct acpm_clk_driver_data acpm_clk_gs101 = {
	.clks = gs101_acpm_clks,
	.nr_clks = ARRAY_SIZE(gs101_acpm_clks),
	.mbox_chan_id = 0,
};

/*
 * GPU operating points of gs201 as shader (G3D) and L2 (G3DL2) rates, from
 * the GPU DVFS table of the gs201 Pixel kernels, plus the boot level of
 * both domains.
 */
static const struct acpm_clk_pair gs201_gpu_pairs[] = {
	{ 151000, 151000 },
	{ 202000, 302000 },
	{ 251000, 302000 },
	{ 302000, 470000 },
	{ 351000, 470000 },
	{ 400000, 603000 },
	{ 471000, 603000 },
	{ 510000, 750000 },
	{ 572000, 750000 },
	{ 701000, 885000 },
	{ 762000, 996000 },
	{ 848000, 996000 },
};

static const struct acpm_clk_paired_data gs201_acpm_paired[] = {
	{
		.name = "gpu",
		.id = 5,	/* G3D */
		.id2 = 6,	/* G3DL2 */
		.pairs = gs201_gpu_pairs,
		.nr_pairs = ARRAY_SIZE(gs201_gpu_pairs),
	},
};

static const struct acpm_clk_driver_data acpm_clk_gs201 = {
	.clks = gs101_acpm_clks,
	.nr_clks = ARRAY_SIZE(gs101_acpm_clks),
	.paired = gs201_acpm_paired,
	.nr_paired = ARRAY_SIZE(gs201_acpm_paired),
	.mbox_chan_id = 0,
};

static unsigned long acpm_clk_recalc_rate(struct clk_hw *hw,
					  unsigned long parent_rate)
{
	struct acpm_clk *clk = to_acpm_clk(hw);

	return clk->handle->ops->dvfs.get_rate(clk->handle, clk->mbox_chan_id,
					       clk->id);
}

static int acpm_clk_set_rate(struct clk_hw *hw, unsigned long rate,
			     unsigned long parent_rate)
{
	struct acpm_clk *clk = to_acpm_clk(hw);

	return clk->handle->ops->dvfs.set_rate(clk->handle, clk->mbox_chan_id,
					       clk->id, rate);
}

static const struct clk_ops acpm_clk_ops = {
	.recalc_rate = acpm_clk_recalc_rate,
	.determine_rate = clk_determine_rate_noop,
	.set_rate = acpm_clk_set_rate,
};

static const struct acpm_clk_pair *
acpm_clk_paired_find(const struct acpm_clk_paired_data *data, unsigned long rate)
{
	unsigned int i;

	/* The lowest operating point at or above the rate, else the highest */
	for (i = 0; i < data->nr_pairs; i++)
		if ((unsigned long)data->pairs[i].rate * HZ_PER_KHZ >= rate)
			return &data->pairs[i];

	return &data->pairs[data->nr_pairs - 1];
}

static unsigned long acpm_clk_paired_recalc_rate(struct clk_hw *hw,
						 unsigned long parent_rate)
{
	struct acpm_clk_paired *clk = to_acpm_clk_paired(hw);

	return clk->handle->ops->dvfs.get_rate(clk->handle, clk->mbox_chan_id,
					       clk->data->id);
}

static int acpm_clk_paired_determine_rate(struct clk_hw *hw,
					  struct clk_rate_request *req)
{
	struct acpm_clk_paired *clk = to_acpm_clk_paired(hw);

	req->rate = (unsigned long)acpm_clk_paired_find(clk->data, req->rate)->rate *
		    HZ_PER_KHZ;

	return 0;
}

static int acpm_clk_paired_set_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long parent_rate)
{
	struct acpm_clk_paired *clk = to_acpm_clk_paired(hw);
	const struct acpm_dvfs_ops *ops = &clk->handle->ops->dvfs;
	const struct acpm_clk_paired_data *data = clk->data;
	const struct acpm_clk_pair *pair = acpm_clk_paired_find(data, rate);
	unsigned long cur;
	int ret;

	cur = ops->get_rate(clk->handle, clk->mbox_chan_id, data->id);

	/* Keep the secondary domain at or above the primary one throughout */
	if ((unsigned long)pair->rate * HZ_PER_KHZ > cur) {
		ret = ops->set_rate(clk->handle, clk->mbox_chan_id, data->id2,
				    (unsigned long)pair->rate2 * HZ_PER_KHZ);
		if (ret)
			return ret;

		return ops->set_rate(clk->handle, clk->mbox_chan_id, data->id,
				     (unsigned long)pair->rate * HZ_PER_KHZ);
	}

	ret = ops->set_rate(clk->handle, clk->mbox_chan_id, data->id,
			    (unsigned long)pair->rate * HZ_PER_KHZ);
	if (ret)
		return ret;

	return ops->set_rate(clk->handle, clk->mbox_chan_id, data->id2,
			     (unsigned long)pair->rate2 * HZ_PER_KHZ);
}

static const struct clk_ops acpm_clk_paired_ops = {
	.recalc_rate = acpm_clk_paired_recalc_rate,
	.determine_rate = acpm_clk_paired_determine_rate,
	.set_rate = acpm_clk_paired_set_rate,
};

static int acpm_clk_register(struct device *dev, struct acpm_clk *aclk,
			     const char *name)
{
	struct clk_init_data init = {};

	init.name = name;
	init.ops = &acpm_clk_ops;
	aclk->hw.init = &init;

	return devm_clk_hw_register(dev, &aclk->hw);
}

static int acpm_clk_probe(struct platform_device *pdev)
{
	struct acpm_handle *acpm_handle;
	struct clk_hw_onecell_data *clk_data;
	struct clk_hw **hws;
	struct device *dev = &pdev->dev;
	const struct acpm_clk_driver_data *data;
	struct acpm_clk_paired *paired;
	struct acpm_clk *aclks;
	unsigned int mbox_chan_id;
	int i, err, count;

	acpm_handle = devm_acpm_get_by_node(dev, dev->parent->of_node);
	if (IS_ERR(acpm_handle))
		return dev_err_probe(dev, PTR_ERR(acpm_handle),
				     "Failed to get acpm handle\n");

	data = (const struct acpm_clk_driver_data *)
		platform_get_device_id(pdev)->driver_data;
	count = data->nr_clks + data->nr_paired;
	mbox_chan_id = data->mbox_chan_id;

	clk_data = devm_kzalloc(dev, struct_size(clk_data, hws, count),
				GFP_KERNEL);
	if (!clk_data)
		return -ENOMEM;

	clk_data->num = count;
	hws = clk_data->hws;

	aclks = devm_kcalloc(dev, count, sizeof(*aclks), GFP_KERNEL);
	if (!aclks)
		return -ENOMEM;

	for (i = 0; i < data->nr_clks; i++) {
		struct acpm_clk *aclk = &aclks[i];

		/*
		 * The code assumes the clock IDs start from zero,
		 * are sequential and do not have gaps.
		 */
		aclk->id = i;
		aclk->handle = acpm_handle;
		aclk->mbox_chan_id = mbox_chan_id;

		hws[i] = &aclk->hw;

		err = acpm_clk_register(dev, aclk, data->clks[i].name);
		if (err)
			return dev_err_probe(dev, err,
					     "Failed to register clock\n");
	}

	paired = devm_kcalloc(dev, data->nr_paired, sizeof(*paired),
			      GFP_KERNEL);
	if (data->nr_paired && !paired)
		return -ENOMEM;

	/* Clocks spanning two domains follow the per-domain ones */
	for (i = 0; i < data->nr_paired; i++) {
		struct clk_init_data init = {
			.name = data->paired[i].name,
			.ops = &acpm_clk_paired_ops,
			.flags = CLK_GET_RATE_NOCACHE,
		};

		paired[i].data = &data->paired[i];
		paired[i].handle = acpm_handle;
		paired[i].mbox_chan_id = mbox_chan_id;
		paired[i].hw.init = &init;
		hws[data->nr_clks + i] = &paired[i].hw;

		err = devm_clk_hw_register(dev, &paired[i].hw);
		if (err)
			return dev_err_probe(dev, err,
					     "Failed to register clock\n");
	}

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get,
					   clk_data);
}

static const struct platform_device_id acpm_clk_id[] = {
	{ .name = "gs101-acpm-clk", .driver_data = (kernel_ulong_t)&acpm_clk_gs101 },
	{ .name = "gs201-acpm-clk", .driver_data = (kernel_ulong_t)&acpm_clk_gs201 },
	{ }
};
MODULE_DEVICE_TABLE(platform, acpm_clk_id);

static struct platform_driver acpm_clk_driver = {
	.driver	= {
		.name = "acpm-clocks",
	},
	.probe = acpm_clk_probe,
	.id_table = acpm_clk_id,
};
module_platform_driver(acpm_clk_driver);

MODULE_AUTHOR("Tudor Ambarus <tudor.ambarus@linaro.org>");
MODULE_DESCRIPTION("Samsung Exynos ACPM clock driver");
MODULE_LICENSE("GPL");
