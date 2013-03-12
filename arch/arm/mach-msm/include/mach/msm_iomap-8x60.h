/*
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 * The MSM peripherals are spread all over across 768MB of physical
 * space, which makes just having a simple IO_ADDRESS macro to slide
 * them into the right virtual location rough.  Instead, we will
 * provide a master phys->virt mapping for peripherals here.
 *
 */

#ifndef __ASM_ARCH_MSM_IOMAP_8X60_H
#define __ASM_ARCH_MSM_IOMAP_8X60_H

/* Physical base address and size of peripherals.
 * Ordered by the virtual base addresses they will be mapped at.
 *
 * MSM_VIC_BASE must be an value that can be loaded via a "mov"
 * instruction, otherwise entry-macro.S will not compile.
 *
 * If you add or remove entries here, you'll want to edit the
 * msm_io_desc array in arch/arm/mach-msm/io.c to reflect your
 * changes.
 *
 */

#define MSM_QGIC_DIST_BASE	IOMEM(0xFA000000)
#define MSM_QGIC_DIST_PHYS	0x02080000
#define MSM_QGIC_DIST_SIZE	SZ_4K

#define MSM_QGIC_CPU_BASE	IOMEM(0xFA001000)
#define MSM_QGIC_CPU_PHYS	0x02081000
#define MSM_QGIC_CPU_SIZE	SZ_4K

#define MSM_ACC_BASE		IOMEM(0xFA002000)
#define MSM_ACC_PHYS		0x02001000
#define MSM_ACC_SIZE		SZ_4K

#define MSM_GCC_BASE		IOMEM(0xFA003000)
#define MSM_GCC_PHYS		0x02082000
#define MSM_GCC_SIZE		SZ_4K

#define MSM_TLMM_BASE		IOMEM(0xFA004000)
#define MSM_TLMM_PHYS		0x00800000
#define MSM_TLMM_SIZE		SZ_16K

#define MSM_CLK_CTL_BASE	IOMEM(0xFA010000)
#define MSM_CLK_CTL_PHYS	0x00900000
#define MSM_CLK_CTL_SIZE	SZ_16K

#define MSM_MMSS_CLK_CTL_BASE	IOMEM(0xFA014000)
#define MSM_MMSS_CLK_CTL_PHYS	0x04000000
#define MSM_MMSS_CLK_CTL_SIZE	SZ_4K

#define MSM_LPASS_CLK_CTL_BASE	IOMEM(0xFA015000)
#define MSM_LPASS_CLK_CTL_PHYS	0x28000000
#define MSM_LPASS_CLK_CTL_SIZE	SZ_4K

#define MSM_SCPLL_BASE          IOMEM(0xFA016000)
#define MSM_SCPLL_PHYS          0x00903000
#define MSM_SCPLL_SIZE          SZ_1K

#define MSM_TMR_BASE		IOMEM(0xFA100000)
#define MSM_TMR_PHYS		0x02000000
#define MSM_TMR_SIZE		(SZ_1M)

#define MSM_SHARED_RAM_BASE	IOMEM(0xFA200000)
#define MSM_SHARED_RAM_SIZE	SZ_1M

#define MSM_ACC0_BASE           IOMEM(0xFA300000)
#define MSM_ACC0_PHYS           0x02041000
#define MSM_ACC0_SIZE           SZ_4K

#define MSM_ACC1_BASE           IOMEM(0xFA301000)
#define MSM_ACC1_PHYS           0x02051000
#define MSM_ACC1_SIZE           SZ_4K

#define MSM_DMOV_ADM0_BASE	IOMEM(0xFA400000)
#define MSM_DMOV_ADM0_PHYS	0x18300000
#define MSM_DMOV_ADM0_SIZE	SZ_1M

#define MSM_DMOV_ADM1_BASE	IOMEM(0xFA500000)
#define MSM_DMOV_ADM1_PHYS	0x18400000
#define MSM_DMOV_ADM1_SIZE	SZ_1M

#define MSM_DMOV_BASE		MSM_DMOV_ADM0_BASE

#define MSM_SMMU_JPEGD_PHYS 0x07300000
#define MSM_SMMU_JPEGD_SIZE SZ_1M

#define MSM_SMMU_VPE_PHYS 0x07400000
#define MSM_SMMU_VPE_SIZE SZ_1M

#define MSM_SMMU_MDP0_PHYS 0x07500000
#define MSM_SMMU_MDP0_SIZE SZ_1M

#define MSM_SMMU_MDP1_PHYS 0x07600000
#define MSM_SMMU_MDP1_SIZE SZ_1M

#define MSM_SMMU_ROT_PHYS 0x07700000
#define MSM_SMMU_ROT_SIZE SZ_1M

#define MSM_SMMU_IJPEG_PHYS 0x07800000
#define MSM_SMMU_IJPEG_SIZE SZ_1M

#define MSM_SMMU_VFE_PHYS 0x07900000
#define MSM_SMMU_VFE_SIZE SZ_1M

#define MSM_SMMU_VCODEC_A_PHYS 0x07A00000
#define MSM_SMMU_VCODEC_A_SIZE SZ_1M

#define MSM_SMMU_VCODEC_B_PHYS 0x07B00000
#define MSM_SMMU_VCODEC_B_SIZE SZ_1M

#define MSM_SMMU_GFX3D_PHYS 0x07C00000
#define MSM_SMMU_GFX3D_SIZE SZ_1M

#define MSM_SMMU_GFX2D0_PHYS 0x07D00000
#define MSM_SMMU_GFX2D0_SIZE SZ_1M

#endif
