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

#include <mach/camera.h>

/*
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
*/

#ifdef CONFIG_MSM_CAMERA
static uint32_t camera_off_gpio_table[] = {
	/* parallel CAMERA interfaces */
	GPIO_CFG(0,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT0 */
	GPIO_CFG(1,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT1 */
	GPIO_CFG(2,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
	GPIO_CFG(3,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
	GPIO_CFG(4,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
	GPIO_CFG(5,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
	GPIO_CFG(6,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
	GPIO_CFG(7,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
	GPIO_CFG(8,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
	GPIO_CFG(9,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
	GPIO_CFG(10, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
	GPIO_CFG(11, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
	GPIO_CFG(12, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* PCLK */
	GPIO_CFG(13, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* MCLK */
};

static uint32_t camera_on_gpio_table[] = {
	/* parallel CAMERA interfaces */
#ifndef CONFIG_HUAWEI_CAMERA
	GPIO_CFG(0,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT0 */
	GPIO_CFG(1,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT1 */
	GPIO_CFG(2,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
	GPIO_CFG(3,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
	GPIO_CFG(4,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
	GPIO_CFG(5,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
	GPIO_CFG(6,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
	GPIO_CFG(7,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
	GPIO_CFG(8,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
	GPIO_CFG(9,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
	GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
	GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
	GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_16MA), /* PCLK */
	GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA), /* MCLK */
#else //#CONFIG_HUAWEI_CAMERA
  GPIO_CFG(0,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT0 */
   GPIO_CFG(1,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT1 */
   GPIO_CFG(2,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
   GPIO_CFG(3,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
   GPIO_CFG(4,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
   GPIO_CFG(5,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
   GPIO_CFG(6,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
   GPIO_CFG(7,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
   GPIO_CFG(8,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
   GPIO_CFG(9,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
   GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
   GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
   GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_16MA), /* PCLK */
   GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
   GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
   GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA), /* MCLK */
   GPIO_CFG(21, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* CAMIF_SHDN_INS */
   GPIO_CFG(29, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* CAMIF_SHDN_OUTS */
   GPIO_CFG(89, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* reset */
   GPIO_CFG(109, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), /* vcm */
   GPIO_CFG(30, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* module */
#endif //#CONFIG_HUAWEI_CAMERA
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_ENABLE);
		if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static void config_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

static void config_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}

#ifdef CONFIG_HUAWEI_CAMERA
static int32_t sensor_vreg_enable
(
    struct msm_camera_sensor_vreg *sensor_vreg,
    uint8_t vreg_num
)
{
    struct vreg *vreg_handle;
    uint8_t temp_vreg_sum;
    int32_t rc;
    
    if(sensor_vreg == NULL)
    {
        return 0;
    }
    
    for(temp_vreg_sum = 0; temp_vreg_sum < vreg_num;temp_vreg_sum++)
    {
        vreg_handle = vreg_get(0, sensor_vreg[temp_vreg_sum].vreg_name);
    	if (!vreg_handle) {
    		printk(KERN_ERR "vreg_handle get failed\n");
    		return -EIO;
    	}
    	rc = vreg_set_level(vreg_handle, sensor_vreg[temp_vreg_sum].mv);
    	if (rc) {
    		printk(KERN_ERR "vreg_handle set level failed\n");
    		return -EIO;
    	}
    	rc = vreg_enable(vreg_handle);
    	if (rc) {
    		printk(KERN_ERR "vreg_handle enable failed\n");
    		return -EIO;
    	}
    }
    return 0;
}

static int32_t sensor_vreg_disable
(
    struct msm_camera_sensor_vreg *sensor_vreg,
    uint8_t vreg_num
)
{
    struct vreg *vreg_handle;
    uint8_t temp_vreg_sum;
    int32_t rc;
    
    if(sensor_vreg == NULL)
    {
        return 0;
    }

    for(temp_vreg_sum = 0; temp_vreg_sum < vreg_num;temp_vreg_sum++)
    {
        vreg_handle = vreg_get(0, sensor_vreg[temp_vreg_sum].vreg_name);
    	if (!vreg_handle) {
    		printk(KERN_ERR "vreg_handle get failed\n");
    		return -EIO;
    	}
    	rc = vreg_disable(vreg_handle);
    	if (rc) {
    		printk(KERN_ERR "vreg_handle disable failed\n");
    		return -EIO;
    	}
    }
    return 0;
}
 struct msm_camera_sensor_vreg sensor_vreg_array[] = {
    {
		.vreg_name   = "gp3",
		.mv	  = 2600,
	}, 
    {
		.vreg_name   = "gp1",
		.mv	  = 2800,
	},    
    {
		.vreg_name   = "gp2",
		.mv	  = 1800,
	},
   
};
#endif //CONFIG_HUAWEI_CAMERA

static struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

#ifndef CONFIG_HUAWEI_CAMERA

#ifdef CONFIG_MT9D112
static struct msm_camera_sensor_info msm_camera_sensor_mt9d112_data = {
	.sensor_name	= "mt9d112",
	.sensor_reset	= 89,
	.sensor_pwd	= 85,
	.vcm_pwd	= 0,
	.pdata		= &msm_camera_device_data,
	.flash_type     = MSM_CAMERA_FLASH_LED
};

static struct platform_device msm_camera_sensor_mt9d112 = {
	.name	   = "msm_camera_mt9d112",
	.dev	    = {
		.platform_data = &msm_camera_sensor_mt9d112_data,
	},
};
#endif

#ifdef CONFIG_S5K3E2FX
static struct msm_camera_sensor_info msm_camera_sensor_s5k3e2fx_data = {
	.sensor_name	= "s5k3e2fx",
	.sensor_reset	= 89,
	.sensor_pwd	= 85,
	.vcm_pwd	= 0,
	.pdata		= &msm_camera_device_data,
	.flash_type     = MSM_CAMERA_FLASH_LED
};

static struct platform_device msm_camera_sensor_s5k3e2fx = {
	.name	   = "msm_camera_s5k3e2fx",
	.dev	    = {
		.platform_data = &msm_camera_sensor_s5k3e2fx_data,
	},
};
#endif

#ifdef CONFIG_MT9P012
static struct msm_camera_sensor_info msm_camera_sensor_mt9p012_data = {
	.sensor_name	= "mt9p012",
	.sensor_reset	= 89,
	.sensor_pwd	= 85,
	.vcm_pwd	= 88,
	.pdata		= &msm_camera_device_data,
	.flash_type     = MSM_CAMERA_FLASH_LED
};

static struct platform_device msm_camera_sensor_mt9p012 = {
	.name	   = "msm_camera_mt9p012",
	.dev	    = {
		.platform_data = &msm_camera_sensor_mt9p012_data,
	},
};
#endif

#ifdef CONFIG_MT9T013
static struct msm_camera_sensor_info msm_camera_sensor_mt9t013_data = {
	.sensor_name	= "mt9t013",
	.sensor_reset	= 89,
	.sensor_pwd	= 85,
	.vcm_pwd	= 0,
	.pdata		= &msm_camera_device_data,
	.flash_type     = MSM_CAMERA_FLASH_LED
};

static struct platform_device msm_camera_sensor_mt9t013 = {
	.name	   = "msm_camera_mt9t013",
	.dev	    = {
		.platform_data = &msm_camera_sensor_mt9t013_data,
	},
};
#endif
#else //CONFIG_HUAWEI_CAMERA

#ifdef CONFIG_MT9T013
static struct msm_camera_sensor_info msm_camera_sensor_mt9t013_byd_data = {
	.sensor_name	= "mt9t013_byd",
	.sensor_reset	= 89,
	.sensor_pwd	= /*17*/29,
	.vcm_pwd	= /*0*/109,
	.pdata		= &msm_camera_device_data,
    .flash_type		= MSM_CAMERA_FLASH_NONE,
#ifdef CONFIG_HUAWEI_CAMERA
        .sensor_module_id  = 30,
        .sensor_module_value  = 1,
        .sensor_vreg  = sensor_vreg_array,
        .vreg_num     = ARRAY_SIZE(sensor_vreg_array),
        .vreg_enable_func = sensor_vreg_enable,
        .vreg_disable_func = sensor_vreg_disable,
        .slave_sensor = 0,
//        .master_init_control_slave = sensor_master_init_control_slave,
#endif        
};


static struct platform_device msm_camera_sensor_mt9t013_byd = {
	.name	   = "msm_camera_byd3m",
	.id        = -1,
	.dev	    = {
		.platform_data = &msm_camera_sensor_mt9t013_byd_data,
	},
};
#endif //CONFIG_MT9T013

#ifdef CONFIG_MT9T013
static struct msm_camera_sensor_info msm_camera_sensor_mt9t013_liteon_data = {
	.sensor_name	= "mt9t013_liteon",
	.sensor_reset	= 89,
	.sensor_pwd	= /*17*/29,
	.vcm_pwd	= 109,
	.pdata		= &msm_camera_device_data,
    .flash_type		= MSM_CAMERA_FLASH_NONE,
#ifdef CONFIG_HUAWEI_CAMERA
        .sensor_module_id  = 30,
        .sensor_module_value  = 0,
        .sensor_vreg  = sensor_vreg_array,
        .vreg_num     = ARRAY_SIZE(sensor_vreg_array),
        .vreg_enable_func = sensor_vreg_enable,
        .vreg_disable_func = sensor_vreg_disable,
        .slave_sensor = 0,
//        .master_init_control_slave = sensor_master_init_control_slave,
#endif        
};

static struct platform_device msm_camera_sensor_mt9t013_liteon = {
	.name	   = "msm_camera_liteon3m",
	.id        = -1,
	.dev	    = {
		.platform_data = &msm_camera_sensor_mt9t013_liteon_data,
	},
};
#endif //CONFIG_MT9T013

#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_OV3647
static struct msm_camera_sensor_info msm_camera_sensor_ov3647_data = {
	.sensor_name	= "ov3647",
	.sensor_reset	= 89,
	.sensor_pwd	= 29,
	.vcm_pwd	= 109,
	.pdata		= &msm_camera_device_data,
    .flash_type		= MSM_CAMERA_FLASH_NONE,
#ifdef CONFIG_HUAWEI_CAMERA
        .sensor_module_id  = 30,
        .sensor_module_value  = 0,
        .sensor_vreg  = sensor_vreg_array,
        .vreg_num     = ARRAY_SIZE(sensor_vreg_array),
        .vreg_enable_func = sensor_vreg_enable,
        .vreg_disable_func = sensor_vreg_disable,
        .slave_sensor = 0,
//        .master_init_control_slave = sensor_master_init_control_slave,
#endif        
};

static struct platform_device msm_camera_sensor_ov3647 = {
	.name	   = "msm_camera_ov3647",
	.id        = -1,
	.dev	    = {
		.platform_data = &msm_camera_sensor_ov3647_data,
	},
};
#endif

#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_OV7690	
static struct msm_camera_sensor_info msm_camera_sensor_ov7690_data = {
	.sensor_name	= "ov7690",
	.sensor_reset	= 89,
	.sensor_pwd	= 21,
	.vcm_pwd	= 0,
	.pdata		= &msm_camera_device_data,
    .flash_type		= MSM_CAMERA_FLASH_NONE,
#ifdef CONFIG_HUAWEI_CAMERA
        .sensor_module_id  = 30,
        .sensor_module_value  = 0,
        .sensor_vreg  = sensor_vreg_array,
        .vreg_num     = ARRAY_SIZE(sensor_vreg_array),
        .vreg_enable_func = sensor_vreg_enable,
        .vreg_disable_func = sensor_vreg_disable,
        .slave_sensor = 1,
//        .master_init_control_slave = sensor_master_init_control_slave,
#endif        
};

static struct platform_device msm_camera_sensor_ov7690 = {
	.name	   = "msm_camera_ov7690",
	.id        = -1,
	.dev	    = {
		.platform_data = &msm_camera_sensor_ov7690_data,
	},
};
#endif
#endif/* CONFIG_HUAWEI_CAMERA*/

#endif /*CONFIG_MSM_CAMERA*/


//static struct platform_device *devices[] __initdata = {
/*
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
*/

//static void __init halibut_init(void)
/*
#ifdef CONFIG_HUAWEI_CAMERA
    sensor_vreg_disable(sensor_vreg_array,ARRAY_SIZE(sensor_vreg_array));
#endif

#ifdef CONFIG_MSM_CAMERA
	config_camera_off_gpios(); // might not be necessary //
#endif
*/
