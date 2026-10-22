/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Clock IDs of the ACPM firmware on the Google gs201 SoC.
 */

#ifndef _DT_BINDINGS_CLOCK_GOOGLE_GS201_ACPM_H
#define _DT_BINDINGS_CLOCK_GOOGLE_GS201_ACPM_H

#define GS201_CLK_ACPM_DVFS_MIF				0
#define GS201_CLK_ACPM_DVFS_INT				1
#define GS201_CLK_ACPM_DVFS_CPUCL0			2
#define GS201_CLK_ACPM_DVFS_CPUCL1			3
#define GS201_CLK_ACPM_DVFS_CPUCL2			4
#define GS201_CLK_ACPM_DVFS_G3D				5
#define GS201_CLK_ACPM_DVFS_G3DL2			6
#define GS201_CLK_ACPM_DVFS_TPU				7
#define GS201_CLK_ACPM_DVFS_INTCAM			8
#define GS201_CLK_ACPM_DVFS_TNR				9
#define GS201_CLK_ACPM_DVFS_CAM				10
#define GS201_CLK_ACPM_DVFS_MFC				11
#define GS201_CLK_ACPM_DVFS_DISP			12
#define GS201_CLK_ACPM_DVFS_BO				13

/* Shader and L2 domains of the GPU, scaled together */
#define GS201_CLK_ACPM_GPU				14

#endif /* _DT_BINDINGS_CLOCK_GOOGLE_GS201_ACPM_H */
