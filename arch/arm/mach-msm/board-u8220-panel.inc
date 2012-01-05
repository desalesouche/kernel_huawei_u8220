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

#include "smd_private.h"



#if defined(CONFIG_FB_MSM_MDDI_TC358721XBG_VGA_QCIF) || \
	defined(CONFIG_FB_MSM_MDDI_TC358723XBG_VGA_QCIF)	
static void msm_fb_mddi_power_save(int on)
{           
    
} 
#else
static char *msm_fb_vreg[] = {
	"gp5"
};

#define MSM_FB_VREG_OP(name, op) \
do { \
	vreg = vreg_get(0, name); \
	if (vreg_##op(vreg)) \
		printk(KERN_ERR "%s: %s vreg operation failed \n", \
			(vreg_##op == vreg_enable) ? "vreg_enable" \
				: "vreg_disable", name); \
} while (0)

static int mddi_power_save_on;
static void msm_fb_mddi_power_save(int on)
{
	struct vreg *vreg;
	int i;
	int flag_on = !!on;

	if (mddi_power_save_on == flag_on)
		return;

	mddi_power_save_on = flag_on;

	if (machine_is_msm7201a_ffa())
		gpio_direction_output(88, flag_on);

	for (i = 0; i < ARRAY_SIZE(msm_fb_vreg); i++) {
		if (flag_on)
			MSM_FB_VREG_OP(msm_fb_vreg[i], enable);
		else
			MSM_FB_VREG_OP(msm_fb_vreg[i], disable);
	}
}
#endif



#define PM_VID_EN_CONFIG_PROC          24
#define PM_VID_EN_API_PROG             0x30000061
#define PM_VID_EN_API_VERS             0x00010001

static struct msm_rpc_endpoint *pm_vid_en_ep;

static int msm_fb_pm_vid_en(int on)
{
	int rc = 0;
	struct msm_fb_pm_vid_en_req {
		struct rpc_request_hdr hdr;
		uint32_t on;
	} req;

	pm_vid_en_ep = msm_rpc_connect(PM_VID_EN_API_PROG,
					PM_VID_EN_API_VERS, 0);
	if (IS_ERR(pm_vid_en_ep)) {
		printk(KERN_ERR "%s: msm_rpc_connect failed! rc = %ld\n",
			__func__, PTR_ERR(pm_vid_en_ep));
		return -EINVAL;
	}

	req.on = cpu_to_be32(on);
	rc = msm_rpc_call(pm_vid_en_ep,
			PM_VID_EN_CONFIG_PROC,
			&req, sizeof(req),
			5 * HZ);
	if (rc)
		printk(KERN_ERR
			"%s: msm_rpc_call failed! rc = %d\n", __func__, rc);

	msm_rpc_close(pm_vid_en_ep);
	return rc;
}

static int mddi_get_panel_num(void)
{
	if (machine_is_msm7201a_surf())
		return 2;
	else
		return 1;
}

static int mddi_toshiba_backlight_level(int level, int max, int min)
{
	int out_val;

	if (!max)
		return 0;

	if (machine_is_msm7201a_ffa()) {
		out_val = 2200 + (((max - level) * (4000 - 2200)) / max);
	} else {
		out_val = (level * 4999) / max;
	}

	return out_val;
}

static int mddi_sharp_backlight_level(int level, int max, int min)
{
	if (machine_is_msm7201a_ffa())
		return level;
	else
		return -1;
}
static uint32_t mddi_get_panel_type(void)
{
    uint32_t *lcd_type_p = (uint32_t*)smem_alloc(SMEM_LCD_CUR_PANEL, sizeof(uint32_t));
	if(lcd_type_p)
	{
        return *lcd_type_p;
	}
    else
    {
        return 0;
    }

}
static struct tvenc_platform_data tvenc_pdata = {
	.pm_vid_en = msm_fb_pm_vid_en,
};

static struct mddi_platform_data mddi_pdata = {
	.mddi_power_save = msm_fb_mddi_power_save,
};

static struct msm_panel_common_pdata mddi_toshiba_pdata = {
	.backlight_level = mddi_toshiba_backlight_level,
	.panel_num = mddi_get_panel_num,
};


#ifdef CONFIG_FB_MSM_MDDI_TC358721XBG_VGA_QCIF
static struct msm_panel_common_pdata mddi_tc358721xbg_pdata = {
	.backlight_level = mddi_tc358721xbg_backlight_level,
};
#endif // CONFIG_FB_MSM_MDDI_TC358721XBG_VGA_QCIF

#ifdef CONFIG_FB_MSM_MDDI_TC358723XBG_VGA_QCIF
static int mddi_tc358723xbg_backlight_level(int level ,int max,int min)
{
	//0~15
	return level;
}

static struct msm_panel_common_pdata mddi_tc358723xbg_pdata = {
	.backlight_level = mddi_tc358723xbg_backlight_level,
	.get_panel_type = mddi_get_panel_type,  
};
#endif // CONFIG_FB_MSM_MDDI_TC358723XBG_VGA_QCIF

static struct msm_panel_common_pdata mddi_sharp_pdata = {
	.backlight_level = mddi_sharp_backlight_level,
};

static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_fb_resources),
	.resource       = msm_fb_resources,
};

static struct platform_device mddi_toshiba_device = {
	.name   = "mddi_toshiba",
	.id     = 0,
	.dev    = {
		.platform_data = &mddi_toshiba_pdata,
	}
};

#ifdef CONFIG_FB_MSM_MDDI_TC358721XBG_VGA_QCIF
static struct platform_device mddi_tc358721xbg_device = {
	.name   = "mddi_tc21xbg_vga",
	.id     = 0,
	.dev    = {
		.platform_data = &mddi_tc358721xbg_pdata,
	}
};
#endif //CONFIG_FB_MSM_MDDI_TC358721XBG_VGA_QCIF

#ifdef CONFIG_FB_MSM_MDDI_TC358723XBG_VGA_QCIF
static struct platform_device mddi_tc358723xbg_device = {
	.name   = "mddi_tc23xbg_vga",
	.id     = 0,
	.dev    = {
		.platform_data = &mddi_tc358723xbg_pdata,
	}
};
#endif //CONFIG_FB_MSM_MDDI_TC358723XBG_VGA_QCIF

static struct platform_device mddi_sharp_device = {
	.name   = "mddi_sharp_qvga",
	.id     = 0,
	.dev    = {
		.platform_data = &mddi_sharp_pdata,
	}
};




static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = 97,
};

static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
	//msm_fb_register_device("ebi2", 0); 
	msm_fb_register_device("pmdh", &mddi_pdata);
	msm_fb_register_device("emdh", 0);
	msm_fb_register_device("tvenc", &tvenc_pdata);
}


//static struct platform_device *devices[] __initdata
/*
	&msm_fb_device,
#ifdef CONFIG_FB_MSM_MDDI_TC358723XBG_VGA_QCIF
	&mddi_tc358723xbg_device,
#endif //CONFIG_FB_MSM_MDDI_TC358723XBG_VGA_QCIF
	&mddi_toshiba_device,
	&mddi_sharp_device,
*/


//static void __init halibut_init(void)
//	msm_fb_add_devices();

