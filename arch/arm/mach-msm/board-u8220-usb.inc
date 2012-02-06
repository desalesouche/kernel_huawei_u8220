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
	.vendor_id	= 0x05C6,
	.version	= 0x0100,
	.compositions   = usb_func_composition,
	.num_compositions = ARRAY_SIZE(usb_func_composition),
/* add new pid config for google */
	.product_name	= "Qualcomm HSUSB Device",
	.manufacturer_name = "Qualcomm Incorporated",
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



//static void __init msm7x2x_init(void)
/*
static void msm7x2x_usb_init(void)
{
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

