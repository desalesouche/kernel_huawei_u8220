/* linux/arch/arm/mach-msm/board-halibut.c
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/power_supply.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/setup.h>

#include <mach/vreg.h>
#include <mach/mpp.h>
#include <mach/gpio.h>
#include <mach/irqs.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <mach/msm_serial_hs.h>
#include <mach/vreg.h>
#include <mach/msm_rpcrouter.h>
#include <mach/memory.h>
#include <mach/msm_battery.h>
#include <mach/rpc_server_handset.h>

#include "devices.h"
#include "socinfo.h"
#include "clock.h"
#include "msm-keypad-devices.h"
#include "linux/hardware_self_adapt.h"
#include "pm.h"

#ifdef CONFIG_HUAWEI_U8220_KEYBOARD
#include "keypad_linux_u8220.h"
#endif

#ifdef CONFIG_HUAWEI_MSM_VIBRATOR
#include "msm_vibrator.h"
#endif

#ifdef CONFIG_HUAWEI_JOGBALL
#include "jogball_device.h"
#endif

#ifdef CONFIG_TOUCHSCREEN_MELFAS
#include <linux/melfas_i2c_ts.h>
#endif

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI_TM1319
#include <linux/synaptics_i2c_rmi.h>
#endif


#include "board-u8220-mmc.inc"
#include "board-u8220-usb.inc"
#include "board-u8220-panel.inc"
#include "board-u8220-mem.inc"
#include "board-u8220-bt.inc"
#include "board-u8220-camera.inc"


static struct resource smc91x_resources[] = {
	[0] = {
		.start	= 0x9C004300,
		.end	= 0x9C0043ff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= MSM_GPIO_TO_INT(49),
		.end	= MSM_GPIO_TO_INT(49),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct msm_handset_platform_data hs_platform_data = {
	.hs_name = "7k_handset",
};

static struct platform_device hs_device = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = &hs_platform_data,
	},
};

static struct platform_device smc91x_device = {
	.name		= "smc91x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(smc91x_resources),
	.resource	= smc91x_resources,
};

static struct i2c_board_info i2c_devices[] = {
#ifndef CONFIG_HUAWEI_CAMERA
#ifdef CONFIG_MT9D112
	{
		I2C_BOARD_INFO("mt9d112", 0x78 >> 1),
	},
#endif
#ifdef CONFIG_S5K3E2FX
	{
		I2C_BOARD_INFO("s5k3e2fx", 0x20 >> 1),
	},
#endif
#ifdef CONFIG_MT9P012
	{
		I2C_BOARD_INFO("mt9p012", 0x6C >> 1),
	},
#endif
#if defined(CONFIG_MT9T013) || defined(CONFIG_SENSORS_MT9T013)
	{
		I2C_BOARD_INFO("mt9t013", 0x6C),
	},
#endif
#else //CONFIG_HUAWEI_CAMERA//
	{
		I2C_BOARD_INFO("mt9t013_liteon", 0x6C >> 1),
	},
	{
		I2C_BOARD_INFO("mt9t013_byd", 0x6C),
	},
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_OV3647
	{
		I2C_BOARD_INFO("ov3647", 0x90 >> 1),
	},
#endif //CONFIG_HUAWEI_CAMERA_SENSOR_OV3647

#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_OV7690
	{
		I2C_BOARD_INFO("ov7690", 0x42 >> 1),
	},
#endif //CONFIG_HUAWEI_CAMERA_SENSOR_OV7690

#endif //CONFIG_HUAWEI_CAMERA

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI_TM1319
	{
		I2C_BOARD_INFO(SYNAPTICS_I2C_RMI_NAME, 0x20),
		.irq = MSM_GPIO_TO_INT(57)  //gpio 20 is interupt for touchscreen.//
	},
#endif

#ifdef CONFIG_TOUCHSCREEN_MELFAS
	{
		I2C_BOARD_INFO(MELFAS_I2C_NAME, 0x22),
		.irq = MSM_GPIO_TO_INT(57)  /*gpio 57 is interupt for touchscreen.*/
	},
#endif

#ifdef CONFIG_ACCELEROMETER_ADXL345
	{
		I2C_BOARD_INFO("GS", 0xA6 >> 1),
		.irq = MSM_GPIO_TO_INT(31)
	},
#endif
#ifdef CONFIG_ACCELEROMETER_ST_L1S35DE
	{
		I2C_BOARD_INFO("gs_st", 0x3a >> 1),
		.irq = MSM_GPIO_TO_INT(31)
	},
#endif
#ifdef CONFIG_ACCELEROMETER_MMA7455L
	{
		I2C_BOARD_INFO("freescale", 0x1c),
		.irq = MSM_GPIO_TO_INT(31)
	},
#endif
#ifdef CONFIG_SENSORS_AKM8973
	{
		I2C_BOARD_INFO("akm8973", 0x3c>>1), //7 bit addr, no write bit
		.irq = MSM_GPIO_TO_INT(88)
	}
#endif

};


#define SND(desc, num) { .name = #desc, .id = num }
static struct snd_endpoint snd_endpoints_list[] = {
	SND(HANDSET, 0),
	SND(MONO_HEADSET, 2),
	SND(HEADSET, 3),
	SND(SPEAKER, 6),
	SND(TTY_HEADSET, 8),
	SND(TTY_VCO, 9),
	SND(TTY_HCO, 10),
	SND(BT, 12),
	SND(IN_S_SADC_OUT_HANDSET, 16),
	SND(IN_S_SADC_OUT_SPEAKER_PHONE, 25),
	SND(HEADSET_AND_SPEAKER,26), 
	SND(BT_EC_OFF,27),
	SND(CURRENT, 29),
};
#undef SND

static struct msm_snd_endpoints halibut_snd_endpoints = {
	.endpoints = snd_endpoints_list,
	.num = sizeof(snd_endpoints_list) / sizeof(struct snd_endpoint)
};

static struct platform_device halibut_snd = {
	.name = "msm_snd",
	.id = -1,
	.dev    = {
		.platform_data = &halibut_snd_endpoints
	},
};

#define DEC0_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRNB)| \
	(1<<MSM_ADSP_CODEC_WAV)|(1<<MSM_ADSP_CODEC_ADPCM)| \
	(1<<MSM_ADSP_CODEC_YADPCM)|(1<<MSM_ADSP_CODEC_EVRC)| \
	(1<<MSM_ADSP_CODEC_QCELP))
#define DEC1_FORMAT ((1<<MSM_ADSP_CODEC_WAV)|(1<<MSM_ADSP_CODEC_ADPCM)| \
	(1<<MSM_ADSP_CODEC_YADPCM)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC2_FORMAT ((1<<MSM_ADSP_CODEC_WAV)|(1<<MSM_ADSP_CODEC_ADPCM)| \
	(1<<MSM_ADSP_CODEC_YADPCM)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC3_FORMAT ((1<<MSM_ADSP_CODEC_WAV)|(1<<MSM_ADSP_CODEC_ADPCM)| \
	(1<<MSM_ADSP_CODEC_YADPCM)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC4_FORMAT (1<<MSM_ADSP_CODEC_MIDI)

static unsigned int dec_concurrency_table[] = {
	/* Audio LP */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DMA)), 0,
	0, 0, 0,

	/* Concurrency 1 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	 /* Concurrency 2 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 3 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 4 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 5 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 6 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),
};

#define DEC_INFO(name, queueid, decid, nr_codec) { .module_name = name, \
	.module_queueid = queueid, .module_decid = decid, \
	.nr_codec_support = nr_codec}

static struct msm_adspdec_info dec_info_list[] = {
	DEC_INFO("AUDPLAY0TASK", 13, 0, 10), /* AudPlay0BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY1TASK", 14, 1, 4),  /* AudPlay1BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY2TASK", 15, 2, 4),  /* AudPlay2BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY3TASK", 16, 3, 4),  /* AudPlay3BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY4TASK", 17, 4, 1),  /* AudPlay4BitStreamCtrlQueue */
};

static struct msm_adspdec_database msm_device_adspdec_database = {
	.num_dec = ARRAY_SIZE(dec_info_list),
	.num_concurrency_support = (ARRAY_SIZE(dec_concurrency_table) / \
					ARRAY_SIZE(dec_info_list)),
	.dec_concurrency_table = dec_concurrency_table,
	.dec_info_list = dec_info_list,
};

static struct platform_device msm_device_adspdec = {
	.name = "msm_adspdec",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_adspdec_database
	},
};

static u32 msm_calculate_batt_capacity(u32 current_voltage);

static struct msm_psy_batt_pdata msm_psy_batt_data = {
	.voltage_min_design 	= 3200,
	.voltage_max_design	= 4200,
	.avail_chg_sources   	= AC_CHG | USB_CHG ,
	.batt_technology        = POWER_SUPPLY_TECHNOLOGY_LION,
	.calculate_capacity	= &msm_calculate_batt_capacity,
};

static u32 msm_calculate_batt_capacity(u32 current_voltage)
{
	u32 low_voltage   = msm_psy_batt_data.voltage_min_design;
	u32 high_voltage  = msm_psy_batt_data.voltage_max_design;

	return (current_voltage - low_voltage) * 100
		/ (high_voltage - low_voltage);
}

#ifndef CONFIG_HUAWEI_BATTERY
static struct platform_device msm_batt_device = {
	.name 		    = "msm-battery",
	.id		    = -1,
	.dev.platform_data  = &msm_psy_batt_data,
};
#else
static struct platform_device huawei_battery_device = {
	.name = "huawei_battery",
	.id		= -1,
};
#endif

#ifdef CONFIG_LEDS_MSM_PMIC
static struct platform_device msm_device_pmic_leds = {
	.name = "pmic-leds",
	.id		= -1,
};
#endif

#ifdef CONFIG_HUAWEI_WIFI_SDCC
static struct platform_device msm_wlan_ar6000 = {
	.name		= "wlan_ar6000",
	.id		= 1,
	.num_resources	= 0,
	.resource	= NULL,
};
#endif

static struct platform_device *devices[] __initdata = {
#ifdef CONFIG_HUAWEI_WIFI_SDCC 
	&msm_wlan_ar6000,
#endif
#if !defined(CONFIG_MSM_SERIAL_DEBUGGER)
	&msm_device_uart3,
#endif
	&msm_device_uart_dm1,
	&msm_device_smd,
	&msm_device_dmov,
	&msm_device_nand,
	&msm_device_i2c,
	&smc91x_device,
	&msm_device_tssc,

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

//usb
#ifdef CONFIG_USB_MSM_OTG_72K
	&msm_device_otg,
#ifdef CONFIG_USB_GADGET
	&msm_device_gadget_peripheral,
#endif
#endif
// remains?
	&msm_device_hsusb_otg,
	&msm_device_hsusb_host,
#if defined(CONFIG_USB_FUNCTION) || defined(CONFIG_USB_ANDROID)
	&msm_device_hsusb_peripheral,
#endif
#ifdef CONFIG_USB_FUNCTION
	&mass_storage_device,
#endif
//

#ifdef CONFIG_USB_ANDROID
#ifdef CONFIG_USB_ANDROID_RNDIS
	//&rndis_device,
#endif
	&android_usb_device,
#endif

#ifdef CONFIG_BT
	&msm_bt_power_device,
#endif


#ifndef CONFIG_HUAWEI_CAMERA
#ifdef CONFIG_MT9T013
	&msm_camera_sensor_mt9t013,
#endif
#ifdef CONFIG_MT9D112
	&msm_camera_sensor_mt9d112,
#endif
#ifdef CONFIG_S5K3E2FX
	&msm_camera_sensor_s5k3e2fx,
#endif
#ifdef CONFIG_MT9P012
	&msm_camera_sensor_mt9p012,
#endif
#else //CONFIG_HUAWEI_CAMERA//
#ifdef CONFIG_MT9T013
	&msm_camera_sensor_mt9t013_byd,
#endif
#ifdef CONFIG_MT9T013
	&msm_camera_sensor_mt9t013_liteon,
#endif
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_OV3647
	&msm_camera_sensor_ov3647,
#endif
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_OV7690
	&msm_camera_sensor_ov7690,
#endif
#endif // #ifndef CONFIG_HUAWEI_CAMERA//


	&halibut_snd,
	&msm_device_adspdec,
	&msm_bluesleep_device,
	&msm_fb_device,
#ifdef CONFIG_FB_MSM_MDDI_TC358723XBG_VGA_QCIF
	&mddi_tc358723xbg_device,
#endif //CONFIG_FB_MSM_MDDI_TC358723XBG_VGA_QCIF
	&mddi_toshiba_device,
	&mddi_sharp_device,
#ifndef CONFIG_HUAWEI_BATTERY
	&msm_batt_device,
#else
	&huawei_battery_device,
#endif

	&hs_device,
#ifdef CONFIG_LEDS_MSM_PMIC
	&msm_device_pmic_leds,
#endif
};

extern struct sys_timer msm_timer;

static void __init halibut_init_irq(void)
{
	msm_init_irq();
}

static struct msm_acpu_clock_platform_data halibut_clock_data = {
	.acpu_switch_time_us = 50,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.max_axi_khz = 128000,
};

void msm_serial_debug_init(unsigned int base, int irq,
				struct device *clk_device, int signal_irq);

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 16000,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency = 12000,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 2000,
};

static void
msm_i2c_gpio_config(int iface, int config_type)
{
	int gpio_scl;
	int gpio_sda;
	if (iface) {
		gpio_scl = 95;
		gpio_sda = 96;
	} else {
		gpio_scl = 60;
		gpio_sda = 61;
	}
	if (config_type) {
		gpio_tlmm_config(GPIO_CFG(gpio_scl, 1, GPIO_INPUT,
					GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(gpio_sda, 1, GPIO_INPUT,
					GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);
	} else {
		gpio_tlmm_config(GPIO_CFG(gpio_scl, 0, GPIO_OUTPUT,
					GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(gpio_sda, 0, GPIO_OUTPUT,
					GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);
	}
}

static struct msm_i2c_platform_data msm_i2c_pdata = {
	.clk_freq = 100000,
	.rmutex  = 0,
	.pri_clk = 60,
	.pri_dat = 61,
	.aux_clk = 95,
	.aux_dat = 96,
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_init(void)
{
	if (gpio_request(60, "i2c_pri_clk"))
		pr_err("failed to request gpio i2c_pri_clk\n");
	if (gpio_request(61, "i2c_pri_dat"))
		pr_err("failed to request gpio i2c_pri_dat\n");
	if (gpio_request(95, "i2c_sec_clk"))
		pr_err("failed to request gpio i2c_sec_clk\n");
	if (gpio_request(96, "i2c_sec_dat"))
		pr_err("failed to request gpio i2c_sec_dat\n");

	msm_i2c_pdata.pm_lat =
		msm_pm_data[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN]
		.latency;
	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static void __init halibut_init(void)
{
#ifdef CONFIG_TOUCHSCREEN_MELFAS
    int ret;
    unsigned mpp_ts_reset = 13;

    ret = mpp_config_digital_out(mpp_ts_reset,
          MPP_CFG(MPP_DLOGIC_LVL_MSMP,MPP_DLOGIC_OUT_CTRL_LOW));
    if (ret)
    {
        printk(KERN_ERR "%s: Failed to configure mpp (%d)\n",__func__, ret);
    }
#endif

#ifdef CONFIG_HUAWEI_CAMERA
    sensor_vreg_disable(sensor_vreg_array,ARRAY_SIZE(sensor_vreg_array));
#endif

	if (socinfo_init() < 0)
		BUG();

	msm_clock_init(msm_clocks_7x01a, msm_num_clocks_7x01a);

	if (machine_is_msm7201a_ffa()) {
		smc91x_resources[0].start = 0x98000300;
		smc91x_resources[0].end = 0x980003ff;
		smc91x_resources[1].start = MSM_GPIO_TO_INT(85);
		smc91x_resources[1].end = MSM_GPIO_TO_INT(85);
	}

	/* All 7x01 2.0 based boards are expected to have RAM chips capable
	 * of 160 MHz. */
	if (cpu_is_msm7x01()
	    && SOCINFO_VERSION_MAJOR(socinfo_get_version()) == 2)
		halibut_clock_data.max_axi_khz = 160000;

	msm_acpu_clock_init(&halibut_clock_data);

#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
	msm_serial_debug_init(MSM_UART3_PHYS, INT_UART3,
			      &msm_device_uart3.dev, 1);
#endif
// remained
	msm_hsusb_pdata.soc_version = socinfo_get_version();
#ifdef CONFIG_USB_MSM_OTG_72K
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
/* //from 7x25
	msm_otg_pdata.pemp_level =
			PRE_EMPHASIS_WITH_20_PERCENT;
	msm_otg_pdata.drv_ampl = HS_DRV_AMPLITUDE_5_PERCENT;
	msm_otg_pdata.cdr_autoreset = CDR_AUTO_RESET_ENABLE;
	msm_otg_pdata.phy_reset_sig_inverted = 1;
*/
#ifdef CONFIG_USB_GADGET
/*
	msm_otg_pdata.swfi_latency =
		msm_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
*/
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
#endif
#endif

#ifdef CONFIG_MSM_CAMERA
	config_camera_off_gpios(); /* might not be necessary */
#endif
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));
// remained
	msm_device_hsusb_peripheral.dev.platform_data = &msm_hsusb_pdata,
	msm_device_hsusb_host.dev.platform_data = &msm_hsusb_pdata,
//

	platform_add_devices(devices, ARRAY_SIZE(devices));
	msm_device_i2c_init();

#ifndef CONFIG_HUAWEI_U8220_KEYBOARD
	#ifdef CONFIG_SURF_FFA_GPIO_KEYPAD
	if (machine_is_msm7201a_ffa())
		platform_device_register(&keypad_device_7k_ffa);
	else
		platform_device_register(&keypad_device_surf);
	#endif
#else //CONFIG_HUAWEI_U8220_KEYBOARD
	init_u8220_keypad(1);
#endif //CONFIG_HUAWEI_U8220_KEYBOARD

#ifdef CONFIG_HUAWEI_JOGBALL
	platform_device_register(&jogball_device);
#endif

	halibut_init_mmc();
/* remained
#ifdef CONFIG_USB_FUNCTION
	hsusb_gpio_init();
#endif
*/
	msm_fb_add_devices();
	bt_power_init();
/* remained
#ifdef CONFIG_USB_ANDROID
	msm_hsusb_rpc_connect();
	msm_hsusb_set_vbus_state(1) ;
#endif
*/
#ifdef CONFIG_HUAWEI_MSM_VIBRATOR
	init_vibrator_device();
#endif
	msm_pm_set_platform_data(msm_pm_data, ARRAY_SIZE(msm_pm_data));
#ifdef CONFIG_MSM_HW3D
	msm_add_gpu_devices(&hw3d_gpu_setting);
#endif
}


static void __init halibut_map_io(void)
{
	msm_shared_ram_phys = 0x01F00000;

	msm_map_common_io();
	msm_halibut_allocate_memory_regions();
}

/*sub board id interface*/
hw_ver_sub_type get_hw_sub_board_id(void)
{
	return HW_VER_SUB_SURF;
}

lcd_panel_type lcd_panel_probe(void)
{
	/* any panel to satisfy callback */
	return LCD_S6D74A0_SAMSUNG_HVGA;
}

char *get_lcd_panel_name(void)
{
	char *pname = "UNKNOWN LCD";
	return pname;
}

static bool camera_i2c_state = false;
bool camera_is_supported(void)
{
    return camera_i2c_state;
}

void set_camera_support(bool status)
{
    camera_i2c_state = status;
}
static bool st303_gs_state = false;

bool st303_gs_is_supported(void)
{
    return st303_gs_state;
}

void set_st303_gs_support(bool status)
{
    st303_gs_state = status;
}

/*
 *  return: 0 ----not support RGB LED driver
 *          1 ----support RGB LED driver
 */
bool rgb_led_is_supported(void)
{
	return false;
}

/*
 *  return: 0 ----not support bcm wifi
 *          1 ----support bcm wifi
 *          *p_gpio  return MSM WAKEUP WIFI gpio value
 */
unsigned int board_support_bcm_wifi(unsigned *p_gpio)
{
	unsigned gpio_msm_wake_wifi = 124;
	if(p_gpio != NULL)
	{
		*p_gpio = gpio_msm_wake_wifi;
	}
	return 0;
}

EXPORT_SYMBOL(board_support_bcm_wifi);

MACHINE_START(HALIBUT, "Halibut Board (QCT SURF7200A)")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= halibut_map_io,
	.init_irq	= halibut_init_irq,
	.init_machine	= halibut_init,
	.timer		= &msm_timer,
MACHINE_END

MACHINE_START(MSM7201A_FFA, "QCT FFA7201A Board")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= halibut_map_io,
	.init_irq	= halibut_init_irq,
	.init_machine	= halibut_init,
	.timer		= &msm_timer,
MACHINE_END

MACHINE_START(MSM7201A_SURF, "QCT SURF7201A Board")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= halibut_map_io,
	.init_irq	= halibut_init_irq,
	.init_machine	= halibut_init,
	.timer		= &msm_timer,
MACHINE_END
