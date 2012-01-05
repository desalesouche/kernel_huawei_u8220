/* linux/arch/arm/mach-msm/board-eve.c
 *
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

/* from board-eve.c and board-msm7x25.c of 2.6.32 kernels */

//
#include <mach/msm_hsusb.h>
//
#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android.h>
#include <mach/rpc_hsusb.h>
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


#if defined(CONFIG_USB_MSM_OTG_72K)
static struct msm_gpio usb_gpio_lpm_config[] = {
	{ GPIO_CFG(111, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), "ulpi_data_0" },
	{ GPIO_CFG(112, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), "ulpi_data_1" },
	{ GPIO_CFG(113, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), "ulpi_data_2" },
	{ GPIO_CFG(114, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), "ulpi_data_3" },
	{ GPIO_CFG(115, 2, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), "ulpi_data_4" },
	{ GPIO_CFG(116, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), "ulpi_data_5" },
	{ GPIO_CFG(117, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), "ulpi_data_6" },
	{ GPIO_CFG(118, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), "ulpi_data_7" },
	{ GPIO_CFG(119, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), "ulpi_dir" },
	{ GPIO_CFG(120, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), "ulpi_next" },
	{ GPIO_CFG(121, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), "ulpi_stop" },
};
static struct msm_gpio usb_gpio_lpm_unconfig[] = {
	{ GPIO_CFG(111, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
		"ulpi_data_0" },
	{ GPIO_CFG(112, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
		"ulpi_data_1" },
	{ GPIO_CFG(113, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
		"ulpi_data_2" },
	{ GPIO_CFG(114, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
		"ulpi_data_3" },
	{ GPIO_CFG(115, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
		"ulpi_data_4" },
	{ GPIO_CFG(116, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
		"ulpi_data_5" },
	{ GPIO_CFG(117, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
		"ulpi_data_6" },
	{ GPIO_CFG(118, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),
		"ulpi_data_7" },
	{ GPIO_CFG(119, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
		"ulpi_dir" },
	{ GPIO_CFG(120, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
		"ulpi_next" },
	{ GPIO_CFG(121, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),
		"ulpi_stop" },
};
#endif


#if defined(CONFIG_USB_MSM_OTG_72K)
static int hsusb_phy_reset(void __iomem *addr)
{
	msm_hsusb_phy_reset();
	return 0;
}
#endif

#ifdef CONFIG_USB_EHCI_MSM
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
	if (on)
		msm_hsusb_vbus_powerup();
	else
		msm_hsusb_vbus_shutdown();
}
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

#ifdef CONFIG_USB_MSM_OTG_72K
static struct msm_otg_platform_data msm_otg_pdata = {
	.rpc_connect	= hsusb_rpc_connect,
	.phy_reset      = hsusb_phy_reset,
#ifdef CONFIG_USB_EHCI_MSM
	.vbus_power	= msm_hsusb_vbus_power,
#endif
/*
	.chg_vbus_draw	= hsusb_chg_vbus_draw,
	.chg_connected	= hsusb_chg_connected,
	.chg_init	= hsusb_chg_init,
*/
};

#ifdef CONFIG_USB_GADGET
static struct msm_hsusb_gadget_platform_data msm_gadget_pdata;
#endif
#endif


//static struct platform_device *devices[] __initdata = {
/*
#ifdef CONFIG_USB_MSM_OTG_72K
	&msm_device_otg,
#ifdef CONFIG_USB_GADGET
	&msm_device_gadget_peripheral,
#endif
#endif
	&msm_device_hsusb_otg,
	&msm_device_hsusb_host,
#if defined(CONFIG_USB_FUNCTION) || defined(CONFIG_USB_ANDROID)
	&msm_device_hsusb_peripheral,
#endif
#ifdef CONFIG_USB_FUNCTION
	&mass_storage_device,
#endif
#ifdef CONFIG_USB_ANDROID
#ifdef CONFIG_USB_ANDROID_RNDIS
	//&rndis_device,
#endif
	&android_usb_device,
#endif
*/




static struct msm_hsusb_platform_data msm_hsusb_pdata = {
#ifdef CONFIG_USB_ANDROID
	.phy_reset	= hsusb_phy_reset,
#endif
};
/*
static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.version	= 0x0100,
	.phy_info	= (USB_PHY_INTEGRATED | USB_PHY_MODEL_65NM),
	.vendor_id          = 0x5c6,
	.product_name       = "Qualcomm HSUSB Device",
	.serial_number      = "1234567890ABCDEF",
	.manufacturer_name  = "Qualcomm Incorporated",
	.compositions	= usb_func_composition,
	.num_compositions = ARRAY_SIZE(usb_func_composition),
	.function_map   = usb_functions_map,
	.num_functions	= ARRAY_SIZE(usb_functions_map),
	.config_gpio    = NULL,
};
*/


//static void __init halibut_init(void)
/*
	msm_hsusb_pdata.soc_version = socinfo_get_version();
#ifdef CONFIG_USB_MSM_OTG_72K
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
#ifdef CONFIG_USB_GADGET

//	msm_otg_pdata.swfi_latency =
//		msm_pm_data
//		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;

	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
#endif
#endif
	msm_device_hsusb_peripheral.dev.platform_data = &msm_hsusb_pdata;
	msm_device_hsusb_host.dev.platform_data = &msm_hsusb_pdata;

#ifdef CONFIG_USB_FUNCTION
        //hsusb_gpio_init();
#endif

#ifdef CONFIG_USB_ANDROID
        //msm_hsusb_rpc_connect();
        //msm_hsusb_set_vbus_state(1) ;
#endif
*/

