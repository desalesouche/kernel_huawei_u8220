/* linux/arch/arm/mach-msm/board-halibut.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
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
 */

/* from board-halibut.c of 2.6.29 pulse */

#include <linux/bootmem.h>
#include <linux/android_pmem.h>


#ifdef CONFIG_MSM_STACKED_MEMORY
#define MSM_SMI_BASE		0x100000
#define MSM_SMI_SIZE		0x800000

#define MSM_PMEM_GPU0_BASE	MSM_SMI_BASE
#ifdef CONFIG_MSM_HW3D
#define MSM_PMEM_GPU0_SIZE	0x700000
#else
#define MSM_PMEM_GPU0_SIZE	0x800000
#endif
#endif

#ifndef CONFIG_HUAWEI_SMI_64M 
#define MSM_PMEM_MDP_SIZE	0x800000
#define MSM_PMEM_CAMERA_SIZE	0xa00000
#define MSM_PMEM_ADSP_SIZE	0x800000
#define MSM_PMEM_GPU1_SIZE	0x800000
#define MSM_FB_SIZE		0x200000

#define PMEM_KERNEL_EBI1_SIZE	0x200000
#else //CONFIG_HUAWEI_SMI_64M

//              ______________________
//             ^         |	FB 2M		|0x400,0000
//             |		|_______________|0x3E0,0000
//             |          |	ADSP 12M	|
//            32M	|_______________|0x320,0000
//             |          |				|
//             |		|				|
//             |          |				|
//             |          |	MODEM 19M	|
//             V_____ |_______________|0x200,0000


/*GPU1 can't migrate to SMI  */

#define MSM_PMEM_GPU1_SIZE	0x800000

#define  MSM_FB_BASE      0x3E00000
#define  MSM_FB_SIZE	0x200000	// 2 M

//#define MSM_PMEM_CAMERA_BASE 0x3400000
#define MSM_PMEM_CAMERA_SIZE	0xa00000 //10M

#define MSM_PMEM_ADSP_BASE    0x3200000
#define MSM_PMEM_ADSP_SIZE    0xc00000 //12M

//#define  MSM_PMEM_MDP_BASE   0x2300000
#define MSM_PMEM_MDP_SIZE	0x800000
#define PMEM_KERNEL_EBI1_SIZE	0x200000
#endif

#ifdef CONFIG_MSM_HW3D
#define MSM_PMEM_GPU1_BASE  0x17800000//0x10000000+121*1204*1024,
static struct msm_hw3d_meminfo hw3d_gpu_setting = {
	.pmem_gpu0_start = MSM_PMEM_GPU0_BASE,
	.pmem_gpu0_size =  MSM_PMEM_GPU0_SIZE,
	.pmem_gpu1_start = MSM_PMEM_GPU1_BASE,	//no matter, any value is ok
	.pmem_gpu1_size = MSM_PMEM_GPU1_SIZE,	//no matter, any value is ok
};
#endif


#ifdef CONFIG_HUAWEI_EBI_DEVICE
static struct android_pmem_platform_data android_pmem_kernel_ebi1_pdata = {
	.name = PMEM_KERNEL_EBI1_DATA_NAME,
	/* if no allocator_type, defaults to PMEM_ALLOCATORTYPE_BITMAP,
	 * the only valid choice at this time. The board structure is
	 * set to all zeros by the C runtime initialization and that is now
	 * the enum value of PMEM_ALLOCATORTYPE_BITMAP, now forced to 0 in
	 * include/linux/android_pmem.h.
	 */
    .allocator_type = PMEM_ALLOCATORTYPE_BUDDYBESTFIT,
	.cached = 0,
};
#endif
#if 0//CONFIG_MSM_HW3D 
// we modify pmem.c,so remove these pmem devices
static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_BUDDYBESTFIT,
	.cached = 1,
};

static struct android_pmem_platform_data android_pmem_camera_pdata = {
	.name = "pmem_camera",
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BUDDYBESTFIT,
	.cached = 0,
};

#ifdef CONFIG_MSM_STACKED_MEMORY
static struct android_pmem_platform_data android_pmem_gpu0_pdata = {
	.name = "pmem_gpu0",
	.start = MSM_PMEM_GPU0_BASE,
	.size = MSM_PMEM_GPU0_SIZE,
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 0,
};
#endif

static struct android_pmem_platform_data android_pmem_gpu1_pdata = {
	.name = "pmem_gpu1",
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 0,
};
#else //CONFIG_MSM_HW3D 
static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.size = MSM_PMEM_MDP_SIZE,
	.allocator_type = PMEM_ALLOCATORTYPE_BUDDYBESTFIT,
	.cached = 1,
};
#if 0
static struct android_pmem_platform_data android_pmem_camera_pdata = {
	.name = "pmem_camera",
	//.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.no_allocator = 1,
	.cached = 1,
};
#endif
static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};
#endif//CONFIG_MSM_HW3D 

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

#if 0//CONFIG_MSM_HW3D
#ifdef CONFIG_MSM_STACKED_MEMORY
static struct platform_device android_pmem_gpu0_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_gpu0_pdata },
};
#endif

static struct platform_device android_pmem_gpu1_device = {
	.name = "android_pmem",
	.id = 3,
	.dev = { .platform_data = &android_pmem_gpu1_pdata },
};
static struct platform_device android_pmem_camera_device = {
	.name = "android_pmem",
	.id = 4,
	.dev = { .platform_data = &android_pmem_camera_pdata },
};
#else//CONFIG_MSM_HW3D
#if 0
static struct platform_device android_pmem_camera_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_camera_pdata },
};
#endif
#endif//CONFIG_MSM_HW3D

#ifdef CONFIG_HUAWEI_EBI_DEVICE
static struct platform_device android_pmem_kernel_ebi1_device = {
	.name = "android_pmem",
	.id = 5,
	.dev = { .platform_data = &android_pmem_kernel_ebi1_pdata },
};
#endif


#ifdef CONFIG_HUAWEI_EBI_DEVICE
static unsigned pmem_kernel_ebi1_size = PMEM_KERNEL_EBI1_SIZE;
static void __init pmem_kernel_ebi1_size_setup(char **p)
{
	pmem_kernel_ebi1_size = memparse(*p, p);
}
__early_param("pmem_kernel_ebi1_size=", pmem_kernel_ebi1_size_setup);
#endif

static unsigned pmem_mdp_size = MSM_PMEM_MDP_SIZE;
static void __init pmem_mdp_size_setup(char **p)
{
	pmem_mdp_size = memparse(*p, p);
}
__early_param("pmem_mdp_size=", pmem_mdp_size_setup);

static unsigned pmem_camera_size = MSM_PMEM_CAMERA_SIZE;
static void __init pmem_camera_size_setup(char **p)
{
	pmem_camera_size = memparse(*p, p);
}
__early_param("pmem_camera_size=", pmem_camera_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static void __init pmem_adsp_size_setup(char **p)
{
	pmem_adsp_size = memparse(*p, p);
}
__early_param("pmem_adsp_size=", pmem_adsp_size_setup);

static unsigned pmem_gpu1_size = MSM_PMEM_GPU1_SIZE;
static void __init pmem_gpu1_size_setup(char **p)
{
	pmem_gpu1_size = memparse(*p, p);
}
__early_param("pmem_gpu1_size=", pmem_gpu1_size_setup);

static unsigned fb_size = MSM_FB_SIZE;
static void __init fb_size_setup(char **p)
{
	fb_size = memparse(*p, p);
}
__early_param("fb_size=", fb_size_setup);

static void __init msm_halibut_allocate_memory_regions(void)
{
	void *addr=NULL;
	unsigned long size;
	struct resource *res=NULL;

#ifndef CONFIG_HUAWEI_SMI_64M
	#ifdef CONFIG_HUAWEI_EBI_DEVICE
	size = pmem_kernel_ebi1_size;
	if (size) {
		addr = alloc_bootmem_aligned(size, 0x100000);
		android_pmem_kernel_ebi1_pdata.start = __pa(addr);
		android_pmem_kernel_ebi1_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for kernel"
			" ebi1 pmem arena\n", size, addr, __pa(addr));
	}
	#endif

	size = pmem_mdp_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_pdata.start = __pa(addr);
		android_pmem_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for mdp "
			"pmem arena\n", size, addr, __pa(addr));
	}

#if 0
	size = pmem_camera_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_camera_pdata.start = __pa(addr);
		android_pmem_camera_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for camera"
			" pmem arena\n", size, addr, __pa(addr));
	}
#endif
	

	size = pmem_adsp_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_adsp_pdata.start = __pa(addr);
		android_pmem_adsp_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for adsp "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = pmem_gpu1_size;
	if (size) {
		addr = alloc_bootmem_aligned(size, 0x100000);
		android_pmem_gpu1_pdata.start = __pa(addr);
		android_pmem_gpu1_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for gpu1 "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = fb_size ? : MSM_FB_SIZE;
	addr = alloc_bootmem(size);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
		size, addr, __pa(addr));
	
#else 	/*CONFIG_HUAWEI_SMI_64M */

	size = pmem_mdp_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_pdata.start = __pa(addr);
		android_pmem_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for mdp "
			"pmem arena\n", size, addr, __pa(addr));
	}

#if 0
	size = pmem_camera_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_camera_pdata.start = __pa(addr);
		android_pmem_camera_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for camera"
			" pmem arena\n", size, addr, __pa(addr));
	}
#endif


	size = pmem_adsp_size;
	if (size) {
		//addr = alloc_bootmem(size);
		android_pmem_adsp_pdata.start =  MSM_PMEM_ADSP_BASE;
		android_pmem_adsp_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for adsp "
			"pmem arena\n", size, addr, __pa(addr));
	}

#ifdef CONFIG_MSM_HW3D
	size = pmem_gpu1_size;
	res = platform_get_resource_byname(&hw3d_device, IORESOURCE_MEM,"ebi");
	if (size) {
		addr = alloc_bootmem_aligned(size, 0x100000);
		res->start = __pa(addr);
		res->end = res->start+size-1;
		pr_info("allocating %lu bytes at %p (%lx physical) for gpu1 "
			"pmem arena\n", size, addr, __pa(addr));
	}
#endif

	size = fb_size ? : MSM_FB_SIZE;
	//addr =alloc_bootmem(size);
	msm_fb_resources[0].start = MSM_FB_BASE;
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
		size, addr, __pa(addr));
	
#endif /*CONFIG_HUAWEI_SMI_64M*/
}


//static struct platform_device *devices[] __initdata = {
/*
#ifdef CONFIG_HUAWEI_EBI_DEVICE
	&android_pmem_kernel_ebi1_device,
#endif	
	&android_pmem_device,
	&android_pmem_adsp_device,
#if 0	
	&android_pmem_camera_device,	
#endif
#if 0
#ifdef CONFIG_MSM_STACKED_MEMORY
	&android_pmem_gpu0_device,
#endif
	&android_pmem_gpu1_device,
#endif
*/


//static void __init halibut_map_io(void)
//	msm_halibut_allocate_memory_regions();

