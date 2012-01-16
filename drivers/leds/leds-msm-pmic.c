/*
 * leds-msm-pmic.c - MSM PMIC LEDs driver.
 *
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>

#include <mach/pmic.h>
#include <asm/mach-types.h>

#ifdef CONFIG_HUAWEI_APPS
#define MAX_KEYPAD_BL_LEVEL	64
#else
#define MAX_KEYPAD_BL_LEVEL	16
#endif
extern atomic_t kp_suspend_flag;
atomic_t button_flag = ATOMIC_INIT(0);			/* "0" means turn 0ff */
static void msm_keypad_bl_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
    int ret = 0;

    /* U8110 has no keypad backlight */
    if( machine_is_msm7x25_u8110() )
    {
        /* there is nothing to do */
    }
    /* U8100 series uses LCD_DRV to control keypad backlight */
    else if( machine_is_msm7x25_u8100() || machine_is_msm7x25_u8105() \
             || machine_is_msm7x25_u8107() || machine_is_msm7x25_u8109() )
    {
        /*set keypad backlight current to 10mA*/
        ret = pmic_set_led_intensity(LED_LCD, value / LED_FULL);
    }
    else if ( machine_is_msm7x25_u8150() || machine_is_msm7x25_c8150() || machine_is_msm7x25_u8159() )
	{
		ret = pmic_set_led_intensity(LED_KEYPAD, 0);		/* never turn on */
		if ( !atomic_read(&kp_suspend_flag) ) 
		{
 			ret = pmic_set_led_intensity(LED_LCD, value / LED_FULL);
		}
		if (LED_FULL == value) {
			atomic_set(&button_flag, 1);
		} else {
			atomic_set(&button_flag, 0);
		}
	}
    /* other series uses KEYPAD_DRV to control keypad backlight */
    else
    {
        /*set keypad backlight current to 10mA*/
        ret = pmic_set_led_intensity(LED_KEYPAD, value / LED_FULL);
    }     
	
    if (ret)
        dev_err(led_cdev->dev, "can't set keypad backlight\n");
}

static struct led_classdev msm_kp_bl_led = {
	.name			= "keyboard-backlight",
	.brightness_set		= msm_keypad_bl_led_set,
	.brightness		= LED_OFF,
};

static int msm_pmic_led_probe(struct platform_device *pdev)
{
	int rc;

	rc = led_classdev_register(&pdev->dev, &msm_kp_bl_led);
	if (rc) {
		dev_err(&pdev->dev, "unable to register led class driver\n");
		return rc;
	}
	msm_keypad_bl_led_set(&msm_kp_bl_led, LED_OFF);
	return rc;
}

static int __devexit msm_pmic_led_remove(struct platform_device *pdev)
{
	led_classdev_unregister(&msm_kp_bl_led);

	return 0;
}

#ifdef CONFIG_PM
static int msm_pmic_led_suspend(struct platform_device *dev,
		pm_message_t state)
{
	led_classdev_suspend(&msm_kp_bl_led);

	return 0;
}

static int msm_pmic_led_resume(struct platform_device *dev)
{
	led_classdev_resume(&msm_kp_bl_led);

	return 0;
}
#else
#define msm_pmic_led_suspend NULL
#define msm_pmic_led_resume NULL
#endif

static struct platform_driver msm_pmic_led_driver = {
	.probe		= msm_pmic_led_probe,
	.remove		= __devexit_p(msm_pmic_led_remove),
	.suspend	= msm_pmic_led_suspend,
	.resume		= msm_pmic_led_resume,
	.driver		= {
		.name	= "pmic-leds",
		.owner	= THIS_MODULE,
	},
};

static int __init msm_pmic_led_init(void)
{
	return platform_driver_register(&msm_pmic_led_driver);
}
module_init(msm_pmic_led_init);

static void __exit msm_pmic_led_exit(void)
{
	platform_driver_unregister(&msm_pmic_led_driver);
}
module_exit(msm_pmic_led_exit);

MODULE_DESCRIPTION("MSM PMIC LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:pmic-leds");
