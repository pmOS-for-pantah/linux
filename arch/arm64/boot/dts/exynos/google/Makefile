# SPDX-License-Identifier: GPL-2.0

dtb-$(CONFIG_ARCH_EXYNOS) += \
	gs101-oriole.dtb \
	gs101-raven.dtb \
	gs101-bluejay.dtb \
	gs201-panther.dtb

# The bootloader applies overlays that reference labels in the base DTB
DTC_FLAGS_gs201-panther += -@
