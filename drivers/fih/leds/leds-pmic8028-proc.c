/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <fih/dynloader.h>
#include <fih/leds-pmic8028-proc.h>

#define PM8028_MAX_LEDS		        (PMIC8028_ID_LED_MAX + 1)

struct pmic8028_led_data
{
    struct led_classdev     cdev;
    int                     id;
    enum led_brightness     brightness;
    int                     blink;
    struct work_struct      work;
    struct mutex            lock;
    spinlock_t              value_lock;

    struct work_struct      blink_work;
    struct mutex            blink_lock;
    spinlock_t              blink_value_lock;
};

struct pmic8028_led_drvdata
{
    uint32_t                ontime;
    uint32_t                offtime;
    int                     blink_once;

    struct work_struct      ontime_work;
    struct mutex            ontime_lock;
    spinlock_t              ontime_value_lock;

    struct work_struct      offtime_work;
    struct mutex            offtime_lock;
    spinlock_t              offtime_value_lock;

    struct work_struct      blink_once_work;
    struct mutex            blink_once_lock;
    spinlock_t              blink_once_value_lock;
};

static struct pmic8028_led_data     led_data[PM8028_MAX_LEDS];
static struct pmic8028_led_drvdata  led_drvdata;
extern int proc_comm_set_led(unsigned id, unsigned level);

static void led_lc_set(struct pmic8028_led_data *led, enum led_brightness value)
{
    unsigned brightness = 0;

    //printk( KERN_INFO "[pmic8028 led]: %s: led->id = %d, brightness = %d\n", __func__, led->id, value);

    /*Value Current Units
        0   0       mA
        1   -5     mA
        2   -10     mA
        3   -15     mA
        4   -20     mA
        5   -25     mA
        6   -30     mA
        7   -35     mA
        8   -40     mA
        9   -45     mA
    */

    switch (value)
    {
    case LED_OFF:
        brightness = 0; /*keypad leds level*/
        break; 
    case LED_HALF:
        brightness = 1;
        break; 
    case LED_FULL:
        brightness = 1; /*keypad leds max value 20ma * 2 */
        break; 
    default :
        brightness = 0;
        break; 
    }   

    proc_comm_set_led(led->id, brightness);
}

static void pmic8028_led_set(struct led_classdev *led_cdev,
                             enum led_brightness value)
{
    struct pmic8028_led_data *led = container_of(led_cdev, struct pmic8028_led_data, cdev);
    unsigned long flags;

    printk( KERN_INFO "[pmic8028 led]: %s\n", __func__ );

    spin_lock_irqsave(&led->value_lock, flags);
    led->brightness = value;
    schedule_work(&led->work);
    spin_unlock_irqrestore(&led->value_lock, flags);
}

static void pmic8028_led_work(struct work_struct *work)
{
    struct pmic8028_led_data *led = container_of(work,
                                                 struct pmic8028_led_data, work);

    printk( KERN_INFO "[pmic8028 led]: %s\n", __func__ );

    if (led)
    {
        mutex_lock(&led->lock);

        if ((led->id < 0) || 
            (led->id > PMIC8028_ID_LED_MAX))
        {
            pr_err("%s: invalid LED ID (%d) specified\n", __func__, led->id);
        }
        else
        {
            led_lc_set(led, led->brightness);
        }
        mutex_unlock(&led->lock);
    }
}

static void pmic8028_ontime_work(struct work_struct *work)
{
    printk( KERN_INFO "[pmic8028 led]: %s\n", __func__ );
    mutex_lock(&led_drvdata.ontime_lock);
    proc_comm_set_led(SMEM_LED_ONMS,  led_drvdata.ontime);
    mutex_unlock(&led_drvdata.ontime_lock);
}

static void pmic8028_offtime_work(struct work_struct *work)
{
    printk( KERN_INFO "[pmic8028 led]: %s\n", __func__ );
    mutex_lock(&led_drvdata.offtime_lock);
    proc_comm_set_led(SMEM_LED_ONMS,  led_drvdata.offtime);
    mutex_unlock(&led_drvdata.offtime_lock);
}

static void pmic8028_blink_once_work(struct work_struct *work)
{
    printk( KERN_INFO "[pmic8028 led]: %s\n", __func__ );
    mutex_lock(&led_drvdata.blink_once_lock);
    proc_comm_set_led(SMEM_LED_ONMS,  led_drvdata.blink_once);
    mutex_unlock(&led_drvdata.blink_once_lock);
}

static void pmic8028_led_blink_work(struct work_struct *work)
{
    struct pmic8028_led_data *led = container_of(work,
                                                 struct pmic8028_led_data, blink_work);

    printk( KERN_INFO "[pmic8028 led]: %s\n", __func__ );

    if (led)
    {
        mutex_lock(&led->blink_lock);
        switch (led->id)
        {
        case PMIC8028_ID_LED_RED:
            proc_comm_set_led(SMEM_RED_BLINK, led->blink);
            break;
        case PMIC8028_ID_LED_GREEN:
            proc_comm_set_led(SMEM_GREEN_BLINK, led->blink);
            break;
        case PMIC8028_ID_LED_BLUE:
            proc_comm_set_led(SMEM_BLUE_BLINK, led->blink);
            break;
        default :
            pr_err("%s: invalid LED ID (%d) for blink\n", __func__, led->id);
            break;
        }
        mutex_unlock(&led->blink_lock);
    }
}

static ssize_t pmic8028_led_blink_solid_store(struct device *dev,
                                              struct device_attribute *attr,
                                              const char *buf, size_t size)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    struct pmic8028_led_data *led = NULL;
    unsigned long flags;

    if (led_cdev)
    {
        led = container_of(led_cdev, struct pmic8028_led_data, cdev);
        if (led)
        {
            sscanf(buf, "%d", &led->blink);

            spin_lock_irqsave(&led->blink_value_lock, flags);
            schedule_work(&led->blink_work);
            spin_unlock_irqrestore(&led->blink_value_lock, flags);
        }
    }

    return size;
}
static DEVICE_ATTR(blink, 0644, NULL, pmic8028_led_blink_solid_store);

static ssize_t pmic8028_led_ontime_ms_store(struct device *dev,
                                            struct device_attribute *attr,
                                            const char *buf, size_t size)
{
    unsigned long flags;

    sscanf(buf, "%d", &led_drvdata.ontime);

    printk("[pmic8028 led]: %s, ontime [ %d ms]\n", __func__, led_drvdata.ontime);

    spin_lock_irqsave(&led_drvdata.ontime_value_lock, flags);
    schedule_work(&led_drvdata.ontime_work);
    spin_unlock_irqrestore(&led_drvdata.ontime_value_lock, flags);

    return size;
}
static DEVICE_ATTR(ledon, 0644, NULL, pmic8028_led_ontime_ms_store);

static ssize_t pmic8028_led_offtime_ms_store(struct device *dev,
                                             struct device_attribute *attr,
                                             const char *buf, size_t size)
{
    unsigned long flags;

    sscanf(buf, "%d", &led_drvdata.offtime);

    printk("[pmic8028 led]: %s, offtime [ %d ms]\n", __func__, led_drvdata.offtime);

    spin_lock_irqsave(&led_drvdata.offtime_value_lock, flags);
    schedule_work(&led_drvdata.offtime_work);
    spin_unlock_irqrestore(&led_drvdata.offtime_value_lock, flags);

    return size;    
}

static DEVICE_ATTR(ledoff, 0644, NULL, pmic8028_led_offtime_ms_store);

static ssize_t pmic8028_led_blink_once_store(struct device *dev,
                                             struct device_attribute *attr,
                                             const char *buf, size_t size)
{
    unsigned long flags;

    sscanf(buf, "%d", &led_drvdata.blink_once);

    printk("[pmic8028 led]: %s, blink_once [ %d ]\n", __func__, led_drvdata.blink_once);

    spin_lock_irqsave(&led_drvdata.blink_once_value_lock, flags);
    schedule_work(&led_drvdata.blink_once_work);
    spin_unlock_irqrestore(&led_drvdata.blink_once_value_lock, flags);

    return size;
}
static DEVICE_ATTR(blink_once, 0644, NULL, pmic8028_led_blink_once_store);

static int pmic8028_led_probe(struct platform_device *pdev)
{
    struct pmic8028_leds_platform_data  *pdata = pdev->dev.platform_data;
    struct pmic8028_led_data            *led_dat;
    struct pmic8028_led                 *curr_led;
    int                                 rc, i = 0;

    printk( KERN_INFO "[pmic8028 led]: %s\n", __func__ );

    for (i = 0; i < pdata->num_leds; i++)
    {
        curr_led    = &pdata->leds[i];

        if ((curr_led->id < 0) || 
            (curr_led->id > PMIC8028_ID_LED_MAX))
        {
            pr_err("%s: invalid LED ID (%d) specified\n", __func__, curr_led->id);
        }
        else
        {
            led_dat     = &led_data[curr_led->id];

            led_dat->cdev.name              = curr_led->name;
            led_dat->cdev.default_trigger   = curr_led->default_trigger;
            led_dat->cdev.brightness_set    = pmic8028_led_set;
            led_dat->cdev.brightness        = LED_OFF;
            led_dat->cdev.max_brightness    = curr_led->max_brightness;
            led_dat->cdev.flags             = LED_CORE_SUSPENDRESUME;
            led_dat->id                     = curr_led->id;
            led_dat->brightness             = LED_OFF; /*registered*/
            led_dat->blink                  = 0; /*registered*/

            /* Close all Leds */
            proc_comm_set_led(curr_led->id, 0);

            mutex_init(&led_dat->lock);
            spin_lock_init(&led_dat->value_lock);
            INIT_WORK(&led_dat->work, pmic8028_led_work);

            rc = led_classdev_register(&pdev->dev, &led_dat->cdev);
            if (rc)
            {
                dev_err(&pdev->dev, "unable to register led %d\n",
                        led_dat->id);
                goto fail_id_check;
            }

            mutex_init(&led_dat->blink_lock);
            spin_lock_init(&led_dat->blink_value_lock);
            INIT_WORK(&led_dat->blink_work, pmic8028_led_blink_work);

            rc = device_create_file(led_dat->cdev.dev, &dev_attr_blink);
            if (rc)
            {
                dev_err(led_dat->cdev.dev,
                        "%s: device_create_file failed\n", __func__);
                goto fail_id_check;
            }
        }
    }

    led_drvdata.ontime      = 0;
    led_drvdata.offtime     = 0;
    led_drvdata.blink_once  = 0;

    mutex_init(&led_drvdata.ontime_lock);
    spin_lock_init(&led_drvdata.ontime_value_lock);
    INIT_WORK(&led_drvdata.ontime_work, pmic8028_ontime_work);

    rc = device_create_file(&pdev->dev, &dev_attr_ledon);
    if (rc)
    {
        dev_err(&pdev->dev,
                "%s: create dev_attr_ledon failed\n", __func__);
        goto fail_create_file;
    }

    mutex_init(&led_drvdata.offtime_lock);
    spin_lock_init(&led_drvdata.offtime_value_lock);
    INIT_WORK(&led_drvdata.offtime_work, pmic8028_offtime_work);

    rc = device_create_file(&pdev->dev, &dev_attr_ledoff);
    if (rc)
    {
        dev_err(&pdev->dev,
                "%s: create dev_attr_ledoff failed\n", __func__);
        goto fail_create_file;
    }

    mutex_init(&led_drvdata.blink_once_lock);
    spin_lock_init(&led_drvdata.blink_once_value_lock);
    INIT_WORK(&led_drvdata.blink_once_work, pmic8028_blink_once_work);

    rc = device_create_file(&pdev->dev, &dev_attr_blink_once);
    if (rc)
    {
        dev_err(&pdev->dev,
                "%s: create dev_attr_blink_once failed\n", __func__);
        goto fail_create_file;
    }

    platform_set_drvdata(pdev, led_data);

    return 0;

fail_create_file:   
    device_remove_file(&pdev->dev, &dev_attr_ledon);
    device_remove_file(&pdev->dev, &dev_attr_ledoff);
    device_remove_file(&pdev->dev, &dev_attr_blink_once);
fail_id_check:
    for (i = 0; i < pdata->num_leds; i++)
    {
        device_remove_file(led_data[i].cdev.dev, &dev_attr_blink);
        led_classdev_unregister(&led_data[i].cdev);
    }

    return rc;
}

static int __devexit pmic8028_led_remove(struct platform_device *pdev)
{
    int i;
    struct pmic8028_leds_platform_data *pdata = pdev->dev.platform_data;
    struct pmic8028_led_data *led = platform_get_drvdata(pdev);

    printk( KERN_INFO "[pmic8028 led]: %s\n", __func__ );

    cancel_work_sync(&led_drvdata.ontime_work);
    cancel_work_sync(&led_drvdata.offtime_work);
    cancel_work_sync(&led_drvdata.blink_once_work);

    for (i = 0; i < pdata->num_leds; i++)
    {
        led_classdev_unregister(&led[led->id].cdev);
        cancel_work_sync(&led[led->id].work);
    }

    return 0;
}

static struct platform_driver pmic8028_led_driver = {
    .probe      = pmic8028_led_probe,
    .remove     = __devexit_p(pmic8028_led_remove),
                  .driver     = {
        .name   = "pm8028-led",
        .owner  = THIS_MODULE,
    },
};

static int __init pmic8028_led_init(void* hdl)
{
    printk( KERN_INFO "[pmic8028 led]: %s\n", __func__ );
    return platform_driver_register(&pmic8028_led_driver);
}
//module_init(pmic8028_led_init);
DECLARE_DYN_LOADER(DRIVER_TYPE_LED, 
			PM8028_LED_V1, 
			pmic8028_led_init, LEVEL6);

static void __exit pmic8028_led_exit(void)
{
    printk( KERN_INFO "[pmic8028 led]: %s\n", __func__ );
    platform_driver_unregister(&pmic8028_led_driver);
}
module_exit(pmic8028_led_exit);

MODULE_DESCRIPTION("PMIC8028 LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:pmic8028-led");
