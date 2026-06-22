// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2024-2025 Tomeu Vizoso <tomeu@tomeuvizoso.net> */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dev_printk.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/iommu.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>

#include "rocket_core.h"
#include "rocket_job.h"
#include <linux/regulator/consumer.h>

int rocket_core_init(struct rocket_core *core)
{
	struct device *dev = core->dev;
	struct platform_device *pdev = to_platform_device(dev);
	int err = 0;

	core->resets[0].id = "srst_a";
	core->resets[1].id = "srst_h";
	err = devm_reset_control_bulk_get_exclusive(&pdev->dev, ARRAY_SIZE(core->resets),
						    core->resets);
	if (err)
		return dev_err_probe(dev, err, "failed to get resets for core %d\n", core->index);

	core->num_clks = devm_clk_bulk_get_all(dev, &core->clks);
	if (core->num_clks < 0)
		return dev_err_probe(dev, core->num_clks,
				     "failed to get clocks for core %d\n",
				     core->index);

	core->npu_supply = devm_regulator_get_optional(dev, "npu");
	if (IS_ERR(core->npu_supply)) {
		if (PTR_ERR(core->npu_supply) == -ENODEV)
			core->npu_supply = NULL;
		else
			return dev_err_probe(dev, PTR_ERR(core->npu_supply),
					     "failed to get npu regulator\n");
	}

	core->sram_supply = devm_regulator_get_optional(dev, "sram");
	if (IS_ERR(core->sram_supply)) {
		if (PTR_ERR(core->sram_supply) == -ENODEV)
			core->sram_supply = NULL;
		else
			return dev_err_probe(dev, PTR_ERR(core->sram_supply),
					     "failed to get sram regulator\n");
	}

	core->pc_iomem = devm_platform_ioremap_resource_byname(pdev, "pc");
	if (IS_ERR(core->pc_iomem)) {
		dev_err(dev, "couldn't find PC registers %ld\n", PTR_ERR(core->pc_iomem));
		return PTR_ERR(core->pc_iomem);
	}

	core->cna_iomem = devm_platform_ioremap_resource_byname(pdev, "cna");
	if (IS_ERR(core->cna_iomem)) {
		dev_err(dev, "couldn't find CNA registers %ld\n", PTR_ERR(core->cna_iomem));
		return PTR_ERR(core->cna_iomem);
	}

	core->core_iomem = devm_platform_ioremap_resource_byname(pdev, "core");
	if (IS_ERR(core->core_iomem)) {
		dev_err(dev, "couldn't find CORE registers %ld\n", PTR_ERR(core->core_iomem));
		return PTR_ERR(core->core_iomem);
	}

	dma_set_max_seg_size(dev, UINT_MAX);

	err = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(40));
	if (err)
		return err;

	core->iommu_group = iommu_group_get(dev);

	err = rocket_job_init(core);
	if (err) {
		iommu_group_put(core->iommu_group);
		core->iommu_group = NULL;
		return err;
	}

	pm_runtime_use_autosuspend(dev);

	/*
	 * As this NPU will be most often used as part of a media pipeline that
	 * ends presenting in a display, choose 50 ms (~3 frames at 60Hz) as an
	 * autosuspend delay as that will keep the device powered up while the
	 * pipeline is running.
	 */
	pm_runtime_set_autosuspend_delay(dev, 50);

	pm_runtime_enable(dev);

	/*
	 * Do not force a runtime resume during probe.
	 *
	 * On RK3588 with the vendor 6.1 PREEMPT_RT kernel this early
	 * resume path can stall in the Rockchip IOMMU/clock runtime-PM
	 * chain before userspace starts.  Keep runtime PM enabled and let
	 * the first real job submission perform pm_runtime_get_sync().
	 */
	dev_info(dev, "Rockchip NPU core %d: deferring probe-time hardware resume\n",
		 core->index);

	return 0;
}

void rocket_core_fini(struct rocket_core *core)
{
	pm_runtime_dont_use_autosuspend(core->dev);
	pm_runtime_disable(core->dev);
	iommu_group_put(core->iommu_group);
	core->iommu_group = NULL;
	rocket_job_fini(core);
}

void rocket_core_reset(struct rocket_core *core)
{
	reset_control_bulk_assert(ARRAY_SIZE(core->resets), core->resets);

	udelay(10);

	reset_control_bulk_deassert(ARRAY_SIZE(core->resets), core->resets);
}
