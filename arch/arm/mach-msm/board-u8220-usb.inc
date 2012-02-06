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
 */

/* from board-msm7x25.c */

/*
CONFIG_USB_GADGET=y
CONFIG_USB_ANDROID=y
CONFIG_USB_AUTO_INSTALL=y
#### CONFIG_USB_FUNCTION
CONFIG_USB_ANDROID_DIAG=y
CONFIG_USB_ANDROID_CDC_ECM=y
CONFIG_USB_ANDROID_RNDIS=y
# CONFIG_USB_ANDROID_RMNET is not set
CONFIG_USB_F_SERIAL=y

CONFIG_USB_MSM_OTG_72K=y
#### CONFIG_USB_EHCI_MSM


*/


#include <linux/usb/mass_storage_function.h>

#include <mach/msm_hsusb.h>
#include <mach/rpc_hsusb.h>

#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android.h>
#endif

#ifdef CONFIG_USB_AUTO_INSTALL
#include "../../../drivers/usb/gadget/usb_switch_huawei.h"
#include "../../../arch/arm/mach-msm/proc_comm.h"
#include "smd_private.h"

#define USB_SERIAL_LEN 20
/* keep the parameters transmitted from SMEM */
smem_huawei_vender usb_para_data;

/* keep the boot mode transfered from APPSBL */
unsigned int usb_boot_mode = 0;

/* keep usb parameters transfered from modem */
app_usb_para usb_para_info;

/* all the pid used by mobile */
/* add new pid config for google */
/* new requirement: usb tethering */
usb_pid_stru usb_pid_array[]={
    {PID_ONLY_CDROM,     PID_NORMAL,     PID_UDISK, PID_AUTH,     PID_GOOGLE, PID_WLAN}, /* for COMMON products */
    {PID_ONLY_CDROM_TMO, PID_NORMAL_TMO, PID_UDISK, PID_AUTH_TMO, PID_GOOGLE, PID_WLAN}, /* for TMO products */
};

/* pointer to the member of usb_pid_array[], according to the current product */
usb_pid_stru *curr_usb_pid_ptr = &usb_pid_array[0];
#endif  /* #ifdef CONFIG_USB_AUTO_INSTALL */


#ifdef CONFIG_USB_ANDROID
/* dynamic composition */
static struct usb_composition usb_func_composition[] = {
	{
		/* MSC */
		.product_id         = 0xF000,
		.functions	    = 0x02,
		.adb_product_id     = 0x9015,
		.adb_functions	    = 0x12
	},
#ifdef CONFIG_USB_F_SERIAL
	{
		/* MODEM */
		.product_id         = 0xF00B,
		.functions	    = 0x06,
		.adb_product_id     = 0x901E,
		.adb_functions	    = 0x16,
	},
#endif
#ifdef CONFIG_USB_ANDROID_DIAG
	{
		/* DIAG */
		.product_id         = 0x900E,
		.functions	    = 0x04,
		.adb_product_id     = 0x901D,
		.adb_functions	    = 0x14,
	},
#endif
#if defined(CONFIG_USB_ANDROID_DIAG) && defined(CONFIG_USB_F_SERIAL)
	{
		/* DIAG + MODEM */
		.product_id         = 0x9004,
		.functions	    = 0x64,
		.adb_product_id     = 0x901F,
		.adb_functions	    = 0x0614,
	},
	{
		/* DIAG + MODEM + NMEA*/
		.product_id         = 0x9016,
		.functions	    = 0x764,
		.adb_product_id     = 0x9020,
		.adb_functions	    = 0x7614,
	},
	{
		/* DIAG + MODEM + NMEA + MSC */
		.product_id         = 0x9017,
		.functions	    = 0x2764,
		.adb_product_id     = 0x9018,
		.adb_functions	    = 0x27614,
	},
#endif

#ifdef CONFIG_USB_AUTO_INSTALL
    /* new requirement: usb tethering */
    {
        /* MSC, WLAN */
        .product_id         = PID_WLAN,
        .functions      = 0xA,
        .adb_product_id     = PID_WLAN,
        .adb_functions      = 0xA,
    },

/* add new pid config for google */
/* < google_usb_drv */	
	{
		/* MSC, MSC+ADB */
		.product_id         = PID_GOOGLE_MS,
		.functions	    = 0x2,
		.adb_product_id     = PID_GOOGLE,
		.adb_functions	    = 0x12,
	},
/* google_usb_drv > */	
	{
		/* CDROM */
		.product_id         = PID_ONLY_CDROM,
		.functions	    = 0x2,
		.adb_product_id     = PID_ONLY_CDROM,
		.adb_functions	    = 0x2,
	},
	{
		/* CDROM */
		.product_id         = PID_ONLY_CDROM_TMO,
		.functions	    = 0x2,
		.adb_product_id     = PID_ONLY_CDROM_TMO,
		.adb_functions	    = 0x2,
	},
    {
		/* GENERIC_MODEM + ACM_MODEM + MSC + ADB + DIAG */
		.product_id         = PID_NORMAL_TMO,
		.functions	    = 0x276,
		.adb_product_id     = PID_NORMAL_TMO,
		.adb_functions	    = 0x41276,
	},
	{
		/* GENERIC_MODEM + ACM_MODEM + MSC + ADB + DIAG */
		.product_id         = PID_AUTH_TMO,
		.functions	    = 0x41276,
		.adb_product_id     = PID_AUTH_TMO,
		.adb_functions	    = 0x41276,
	},
	{
		/* GENERIC_MODEM + ACM_MODEM + MSC + ADB + DIAG */
		.product_id         = PID_NORMAL,
		.functions	    = 0x276,
		.adb_product_id     = PID_NORMAL,
		.adb_functions	    = 0x41276,
	},
	{
		/* UDISK */
		.product_id         = PID_UDISK,
		.functions	    = 0x2,
		.adb_product_id     = PID_UDISK,
		.adb_functions	    = 0x2,
	},
	{
		/* GENERIC_MODEM + ACM_MODEM + MSC + ADB + DIAG */
		.product_id         = PID_AUTH,
		.functions	    = 0x41276,
		.adb_product_id     = PID_AUTH,
		.adb_functions	    = 0x41276,
	},

#endif  /* CONFIG_USB_AUTO_INSTALL */

#ifdef CONFIG_USB_ANDROID_CDC_ECM
	{
		/* MSC + CDC-ECM */
		.product_id         = 0x9014,
		.functions	    = 0x82,
		.adb_product_id     = 0x9023,
		.adb_functions	    = 0x812,
	},
#endif
#ifdef CONFIG_USB_ANDROID_RMNET
	{
		/* DIAG + RMNET */
		.product_id         = 0x9021,
		.functions	    = 0x94,
		.adb_product_id     = 0x9022,
		.adb_functions	    = 0x914,
	},
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
	{
		/* RNDIS */
		.product_id         = 0xF00E,
		.functions	    = 0xA,
		.adb_product_id     = 0x9024,
		.adb_functions	    = 0x1A,
	},
#endif
};
static struct android_usb_platform_data android_usb_pdata = {
#ifdef CONFIG_USB_AUTO_INSTALL
	.vendor_id	= HUAWEI_VID,
/* < google_usb_drv */	
//	.vendor_id	= 0x18D1, /* 0x05C6, */
/* google_usb_drv > */	
#else	
	.vendor_id	= 0x05C6,
#endif  /* CONFIG_USB_AUTO_INSTALL */
	.version	= 0x0100,
	.compositions   = usb_func_composition,
	.num_compositions = ARRAY_SIZE(usb_func_composition),
/* add new pid config for google */
#ifdef CONFIG_USB_AUTO_INSTALL
	.product_name	= "Ideos",
	.manufacturer_name = "Huawei Incorporated",
#else	
	.product_name	= "Qualcomm HSUSB Device",
	.manufacturer_name = "Qualcomm Incorporated",
#endif  /* CONFIG_USB_AUTO_INSTALL */
	.nluns = 1,
};
static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static int hsusb_rpc_connect(int connect)
{
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}
#endif

#if defined(CONFIG_USB_MSM_OTG_72K) || defined(CONFIG_USB_EHCI_MSM)
static int msm_hsusb_rpc_phy_reset(void __iomem *addr)
{
	return msm_hsusb_phy_reset();
}
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static struct msm_otg_platform_data msm_otg_pdata = {
	.rpc_connect	= hsusb_rpc_connect,
	.phy_reset	= msm_hsusb_rpc_phy_reset,
/* defined in rpc_pmapp.h which was not used on original pulse board.
	.pmic_notif_init         = msm_pm_app_rpc_init,
	.pmic_notif_deinit       = msm_pm_app_rpc_deinit,
	.pmic_register_vbus_sn   = msm_pm_app_register_vbus_sn,
	.pmic_unregister_vbus_sn = msm_pm_app_unregister_vbus_sn,
	.pmic_enable_ldo         = msm_pm_app_enable_usb_ldo,
*/
};

#ifdef CONFIG_USB_GADGET
static struct msm_hsusb_gadget_platform_data msm_gadget_pdata;
#endif
#endif


/*
static struct platform_device *devices[] __initdata = {

#ifdef CONFIG_USB_MSM_OTG_72K
	&msm_device_otg,
#ifdef CONFIG_USB_GADGET
	&msm_device_gadget_peripheral,
#endif
#endif

#ifdef CONFIG_USB_ANDROID
	&android_usb_device,
#endif
};
*/



#ifdef CONFIG_USB_EHCI_MSM
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
	if (on)
		msm_hsusb_vbus_powerup();
	else
		msm_hsusb_vbus_shutdown();
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
	.phy_info       = (USB_PHY_INTEGRATED | USB_PHY_MODEL_65NM),
	.phy_reset = msm_hsusb_rpc_phy_reset,
	.vbus_power = msm_hsusb_vbus_power,
};
static void __init msm7x2x_init_host(void)
{
	if (machine_is_msm7x25_ffa() || machine_is_msm7x27_ffa())
		return;

	msm_add_host(0, &msm_usb_host_pdata);
}
#endif



#ifdef CONFIG_USB_AUTO_INSTALL
/* provide a method to map pid_index to usb_pid, 
 * pid_index is kept in NV(4526). 
 * At power up, pid_index is read in modem and transfer to app in share memory.
 * pid_index can be modified through write file fixusb(msm_hsusb_store_fixusb).
*/
u16 pid_index_to_pid(u32 pid_index)
{
    u16 usb_pid = 0xFFFF;
    
    switch(pid_index)
    {
        case CDROM_INDEX:
            usb_pid = curr_usb_pid_ptr->cdrom_pid;
            break;
        case NORM_INDEX:
            usb_pid = curr_usb_pid_ptr->norm_pid;
            break;
        case AUTH_INDEX:
            usb_pid = curr_usb_pid_ptr->auth_pid;
            break;
        /* add new pid config for google */
        case GOOGLE_INDEX:
            usb_pid = curr_usb_pid_ptr->google_pid;
            break;
        /* new requirement: usb tethering */
        case GOOGLE_WLAN_INDEX:
            usb_pid = curr_usb_pid_ptr->wlan_pid;
        /* set the USB pid to multiport when the index is 0
           This is happened when the NV is not set or set 
           to zero 
        */
        case ORI_INDEX:
        default:
            usb_pid = curr_usb_pid_ptr->norm_pid;
            break;
    }

    USB_PR("%s, pid_index=%d, usb_pid=0x%x\n", __func__, pid_index, usb_pid);
    
    return usb_pid;
}

/* add new pid config for google */
void set_usb_pid_sn(u32 pid_index)
{
    switch(pid_index)
    {
        /* new requirement: usb tethering */
        case GOOGLE_WLAN_INDEX:
            USB_PR("set pid=0x%x, sn=%s\n", GOOGLE_WLAN_INDEX, NULL);
            android_set_product_id(PID_WLAN);
            set_usb_sn(NULL);
            break;
        case GOOGLE_INDEX:
            USB_PR("set pid=0x%x, sn=%s\n", PID_GOOGLE_MS, usb_para_data.usb_para.usb_serial);
            android_set_product_id(PID_GOOGLE_MS);
            set_usb_sn(usb_para_data.usb_para.usb_serial);
            break;
            
        case NORM_INDEX:
            USB_PR("set pid=0x%x, sn=%s\n", curr_usb_pid_ptr->norm_pid, USB_SN_STRING);
            android_set_product_id(curr_usb_pid_ptr->norm_pid);
            set_usb_sn(USB_SN_STRING);
            break;
            
        case CDROM_INDEX:
            USB_PR("set pid=0x%x, sn=%s\n", curr_usb_pid_ptr->cdrom_pid, "");
            android_set_product_id(curr_usb_pid_ptr->cdrom_pid);
            set_usb_sn(NULL);
            break;
            
        case ORI_INDEX:
            USB_PR("set pid=0x%x, sn=%s\n", curr_usb_pid_ptr->norm_pid, "");
            android_set_product_id(curr_usb_pid_ptr->norm_pid);
            set_usb_sn(NULL);
            break;
            
        case AUTH_INDEX:
            USB_PR("set pid=0x%x, sn=%s\n", curr_usb_pid_ptr->auth_pid, "");
            android_set_product_id(curr_usb_pid_ptr->auth_pid);
            set_usb_sn(NULL);
            break;
            
        default:
            USB_PR("set pid=0x%x, sn=%s\n", curr_usb_pid_ptr->norm_pid, "");
            android_set_product_id(curr_usb_pid_ptr->norm_pid);
            set_usb_sn(NULL);
            break;
    }

}

/*  
 * Get usb parameter from share memory and set usb serial number accordingly.
 */
static void proc_usb_para(void)
{
    smem_huawei_vender *usb_para_ptr;
    //u16 pid;
    char *vender_name="t-mobile";

    USB_PR("< %s\n", __func__);

    /* initialize */
    usb_para_info.usb_pid_index = 0;
    usb_para_info.usb_pid = PID_NORMAL;
    
    /* now the smem_id_vendor0 smem id is a new struct */
    usb_para_ptr = (smem_huawei_vender*)smem_alloc(SMEM_ID_VENDOR0, sizeof(smem_huawei_vender));
    if (!usb_para_ptr)
    {
    	USB_PR("%s: Can't find usb parameter\n", __func__);
        return;
    }

    USB_PR("vendor:%s,country:%s\n", usb_para_ptr->vender_para.vender_name, usb_para_ptr->vender_para.country_name);

    memcpy(&usb_para_data, usb_para_ptr, sizeof(smem_huawei_vender));
    
    /* decide usb pid array according to the vender name */
    if(!memcmp(usb_para_ptr->vender_para.vender_name, vender_name, strlen(vender_name)))
    {
        curr_usb_pid_ptr = &usb_pid_array[1];
        USB_PR("USB setting is TMO\n");
    }
    else
    {
        curr_usb_pid_ptr = &usb_pid_array[0];
        USB_PR("USB setting is NORMAL\n");
    }

    USB_PR("smem usb_serial=%s, usb_pid_index=%d\n", usb_para_ptr->usb_para.usb_serial, usb_para_ptr->usb_para.usb_pid_index);

    /* when manufacture, we need to use the diag. so if the usb_serial is null
       and the nv value is google index, we set the ports to normal.
    */
    if (0 == usb_para_data.usb_para.usb_serial[0] 
      && GOOGLE_INDEX == usb_para_ptr->usb_para.usb_pid_index)
    {
      USB_PR("%s usb serial number is null in google mode. so switch to original mode\n", __func__);
      usb_para_ptr->usb_para.usb_pid_index = ORI_INDEX;
    }

    usb_para_info.usb_pid_index = usb_para_ptr->usb_para.usb_pid_index;

    /* for debug */
    //usb_para_info.usb_pid_index = 22;
    
    usb_para_info.usb_pid = pid_index_to_pid(usb_para_ptr->usb_para.usb_pid_index);

    /* add new pid config for google */
    set_usb_pid_sn(usb_para_info.usb_pid_index);

    /* set the usb device product name */
    android_usb_pdata.product_name = get_product_name();
    
    USB_PR("curr_usb_pid_ptr: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n", 
        curr_usb_pid_ptr->cdrom_pid, 
        curr_usb_pid_ptr->norm_pid, 
        curr_usb_pid_ptr->udisk_pid,
        curr_usb_pid_ptr->auth_pid,
        curr_usb_pid_ptr->google_pid);
    USB_PR("usb_para_info: usb_pid_index=%d, usb_pid = 0x%x>\n", 
        usb_para_info.usb_pid_index, 
        usb_para_info.usb_pid);

}

char* get_product_name(void)
{
  char *vender_tmo_name="t-mobile";

  /* only u8150 tmo product name is Comet, others is Ideos */
  if( machine_is_msm7x25_u8150()
    && (0 == memcmp(usb_para_data.vender_para.vender_name, vender_tmo_name, strlen(vender_tmo_name)))
    )
  {
    return "Comet";
  }
  else
  {
    return "Ideos";
  }
}

#endif  /* CONFIG_USB_AUTO_INSTALL */





//static void __init msm7x2x_init(void)
/*
static void msm7x2x_usb_init(void)
{
#ifdef CONFIG_USB_AUTO_INSTALL
    proc_usb_para();
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
	msm_otg_pdata.pemp_level =
			PRE_EMPHASIS_WITH_20_PERCENT;
	msm_otg_pdata.drv_ampl = HS_DRV_AMPLITUDE_5_PERCENT;
	msm_otg_pdata.cdr_autoreset = CDR_AUTO_RESET_ENABLE;
	msm_otg_pdata.phy_reset_sig_inverted = 1;

#ifdef CONFIG_USB_GADGET
	msm_gadget_pdata.swfi_latency =
		msm7x27_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
#endif
#endif

#ifdef CONFIG_USB_EHCI_MSM
	msm7x2x_init_host();
#endif
}
*/





/*
static void __init msm7x2x_fixup(struct machine_desc *desc,
                                 struct tag *tags,
                                 char **cmdline,
                                 struct meminfo *mi)
{
#ifdef CONFIG_USB_AUTO_INSTALL
    // get the boot mode transfered from APPSBL //
    usb_boot_mode = parse_tag_boot_mode_id((const struct tag *)tags);
    USB_PR("%s,usb_boot_mode=0x%x\n", __func__, usb_boot_mode);
#endif
}
*/
