/*
 * Copyright (c) 2010-2011 Yamaha Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <fih/dynloader.h>
#include "../SenseTek_proximity/stk_i2c_ps31xx.h"

#if SENSOR_TYPE == 1
#define SENSOR_NAME "accelerometer"
#elif SENSOR_TYPE == 2
#define SENSOR_NAME "geomagnetic"
#elif SENSOR_TYPE == 3
#define SENSOR_NAME "orientation"
#elif SENSOR_TYPE == 4
#define SENSOR_NAME "gyroscope"
#elif SENSOR_TYPE == 5
#define SENSOR_NAME "light"
#elif SENSOR_TYPE == 6
#define SENSOR_NAME "pressure"
#elif SENSOR_TYPE == 7
#define SENSOR_NAME "temperature"
#elif SENSOR_TYPE == 8
#define SENSOR_NAME "proximity"
#endif

#define SENSOR_DEFAULT_DELAY            (200)   /* 200 ms */
#define SENSOR_MAX_DELAY                (2000)  /* 2000 ms */
#define ABS_STATUS                      (ABS_BRAKE)
#define ABS_WAKE                        (ABS_MISC)
#define ABS_CONTROL_REPORT              (ABS_THROTTLE)

static int suspend(void);
static int resume(void);

struct sensor_data {
    struct mutex mutex;
    int enabled;
    int delay;
#if DEBUG
    int suspend;
#endif
};

static struct platform_device *sensor_pdev = NULL;
static struct input_dev *this_data = NULL;

#ifdef CONFIG_SENSORS_YAS_CM3623
extern int yas_als_suspend(void);
extern int yas_als_resume(void);
extern int yas_als_setdelay(int delay);
/*jordan*/
extern int yas_als_read_alsdata(void);
/*---------------*/
#endif

//extern int Get_is_cm3623_exist(void);

static int
suspend(void)
{
    /* implement suspend of the sensor */
    YLOGD(("%s: suspend\n", SENSOR_NAME));

#ifdef CONFIG_SENSORS_YAS_CM3623
    yas_als_suspend();
#else
    if (strcmp(SENSOR_NAME, "gyroscope") == 0) {
        /* suspend gyroscope */
    }
    else if (strcmp(SENSOR_NAME, "light") == 0) {
        /* suspend light */
    }
    else if (strcmp(SENSOR_NAME, "pressure") == 0) {
        /* suspend pressure */
    }
    else if (strcmp(SENSOR_NAME, "temperature") == 0) {
        /* suspend temperature */
    }
    else if (strcmp(SENSOR_NAME, "proximity") == 0) {
        /* suspend proximity */
    }
#endif
    return 0;
}

static int
resume(void)
{
    /* implement resume of the sensor */
    YLOGD(("%s: resume\n", SENSOR_NAME));

#ifdef CONFIG_SENSORS_YAS_CM3623
    yas_als_resume();
#else
    if (strcmp(SENSOR_NAME, "gyroscope") == 0) {
        /* resume gyroscope */
    }
    else if (strcmp(SENSOR_NAME, "light") == 0) {
        /* resume light */
    }
    else if (strcmp(SENSOR_NAME, "pressure") == 0) {
        /* resume pressure */
    }
    else if (strcmp(SENSOR_NAME, "temperature") == 0) {
        /* resume temperature */
    }
    else if (strcmp(SENSOR_NAME, "proximity") == 0) {
        /* resume proximity */
    }
#endif

#if DEBUG
    {
        struct sensor_data *data = input_get_drvdata(this_data);
        data->suspend = 0;
    }
#endif /* DEBUG */

    return 0;
}

void yas_light_sensor_notify(int x, int y, int z)
{
    if(this_data){
#if TRACE_OUTPUT
        printk(KERN_INFO "%s: %s x = %d, y = %d, z = %d\n", SENSOR_NAME, __func__, x, y, z);
#endif
        input_report_abs(this_data, ABS_X, x * 1000);
        input_report_abs(this_data, ABS_Y, y * 1000);
        input_report_abs(this_data, ABS_Z, z * 1000);
        input_sync(this_data);
    }
}
EXPORT_SYMBOL(yas_light_sensor_notify);

/* Sysfs interface */
static ssize_t
sensor_delay_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);
    int delay;

    mutex_lock(&data->mutex);

    delay = data->delay;

    mutex_unlock(&data->mutex);

    return sprintf(buf, "%d\n", delay);
}

static ssize_t
sensor_delay_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);
    int value = simple_strtol(buf, NULL, 10);

    if (value < 0) {
        return count;
    }

    if (SENSOR_MAX_DELAY < value) {
        value = SENSOR_MAX_DELAY;
    }

    mutex_lock(&data->mutex);

    data->delay = value;
#ifdef CONFIG_SENSORS_YAS_CM3623
    yas_als_setdelay(data->delay);
#else
    input_report_abs(input_data, ABS_CONTROL_REPORT, (data->enabled<<16) | value);
#endif

    mutex_unlock(&data->mutex);

    return count;
}

static ssize_t
sensor_enable_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);
    int enabled;

    mutex_lock(&data->mutex);

    enabled = data->enabled;

    mutex_unlock(&data->mutex);

    return sprintf(buf, "%d\n", enabled);
}

static ssize_t
sensor_enable_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    struct input_dev *input_data = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input_data);
    int value = simple_strtol(buf, NULL, 10);

    value = !!value;

    mutex_lock(&data->mutex);

    #ifdef CONFIG_SENSORS_YAS_CM3623
    /*Nothing*/
    #else
    input_report_abs(input_data, ABS_CONTROL_REPORT, (value<<16) | data->delay);
    #endif

    if (data->enabled && !value) {
        suspend();
    }
    if (!data->enabled && value) {
        resume();
    }
    data->enabled = value;

    mutex_unlock(&data->mutex);

    return count;
}

static ssize_t
sensor_wake_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    struct input_dev *input_data = to_input_dev(dev);
    static int cnt = 1;

    input_report_abs(input_data, ABS_WAKE, cnt++);

    return count;
}

#if DEBUG

static int sensor_suspend(struct platform_device *pdev, pm_message_t state);
static int sensor_resume(struct platform_device *pdev);

static ssize_t
sensor_debug_suspend_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct sensor_data *data = input_get_drvdata(input);

    return sprintf(buf, "%d\n", data->suspend);
}

static ssize_t
sensor_debug_suspend_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    unsigned long suspend = simple_strtol(buf, NULL, 10);

    if (suspend) {
        pm_message_t msg;
        memset(&msg, 0, sizeof(msg));
        sensor_suspend(sensor_pdev, msg);
    } else {
        sensor_resume(sensor_pdev);
    }

    return count;
}

#endif /* DEBUG */

/*Jordan*/
static ssize_t
sensor_data_ftm_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
	printk("sensor_data_ftm_show\n");
    //return sprintf(buf, "%d", hwdep_driver.yas_als_read_alsdata());
	return sprintf(buf, "%d", yas_als_read_alsdata() );
}
/*------------------------*/

static ssize_t
sensor_data_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    unsigned long flags;
#if SENSOR_TYPE <= 4
    int x = 0, y = 0, z = 0;
#else
    int x = 0;
#endif

    spin_lock_irqsave(&input_data->event_lock, flags);

    if(input_data->absinfo){
        x = input_data->absinfo[ABS_X].value;
    #if SENSOR_TYPE <= 4
        y = input_data->absinfo[ABS_Y].value;
        z = input_data->absinfo[ABS_Z].value;
    #endif
    }

    spin_unlock_irqrestore(&input_data->event_lock, flags);

#if SENSOR_TYPE <= 4
    return sprintf(buf, "%d %d %d\n", x, y, z);
#else
    return sprintf(buf, "%d\n", x);
#endif
}

static ssize_t
sensor_status_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    struct input_dev *input_data = to_input_dev(dev);
    unsigned long flags;
    int status = 0;

    spin_lock_irqsave(&input_data->event_lock, flags);

    if(input_data->absinfo)
        status = input_data->absinfo[ABS_STATUS].value;

    spin_unlock_irqrestore(&input_data->event_lock, flags);

    return sprintf(buf, "%d\n", status);
}

static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
        sensor_delay_show, sensor_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
        sensor_enable_show, sensor_enable_store);
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP,
        NULL, sensor_wake_store);
static DEVICE_ATTR(data, S_IRUGO, sensor_data_show, NULL);
/*Jordan*/
static DEVICE_ATTR(data_ftm, S_IRUGO, sensor_data_ftm_show, NULL);
/*-----------*/
static DEVICE_ATTR(status, S_IRUGO, sensor_status_show, NULL);

#if DEBUG
static DEVICE_ATTR(debug_suspend, S_IRUGO|S_IWUSR,
                   sensor_debug_suspend_show, sensor_debug_suspend_store);
#endif /* DEBUG */

static struct attribute *sensor_attributes[] = {
    &dev_attr_delay.attr,
    &dev_attr_enable.attr,
    &dev_attr_wake.attr,
	/*Jordan*/
	&dev_attr_data_ftm.attr,
	/*-----------*/
    &dev_attr_data.attr,
    &dev_attr_status.attr,
#if DEBUG
    &dev_attr_debug_suspend.attr,
#endif /* DEBUG */
    NULL
};

static struct attribute_group sensor_attribute_group = {
    .attrs = sensor_attributes
};

static int
sensor_suspend(struct platform_device *pdev, pm_message_t state)
{
    struct sensor_data *data = input_get_drvdata(this_data);
    int rt = 0;

    mutex_lock(&data->mutex);

    if (data->enabled) {
        #ifdef CONFIG_SENSORS_YAS_CM3623
        /*Nothing*/
        #else
        input_report_abs(this_data, ABS_CONTROL_REPORT, (0<<16) | data->delay);
        #endif
        rt = suspend();
    }

    mutex_unlock(&data->mutex);

    return rt;
}

static int
sensor_resume(struct platform_device *pdev)
{
    struct sensor_data *data = input_get_drvdata(this_data);
    int rt = 0;

    mutex_lock(&data->mutex);

    if (data->enabled) {
        rt = resume();
        #ifdef CONFIG_SENSORS_YAS_CM3623
        /*Nothing*/
        #else
        input_report_abs(this_data, ABS_CONTROL_REPORT, (1<<16) | data->delay);
        #endif
    }

    mutex_unlock(&data->mutex);

    return rt;
}

static int
sensor_probe(struct platform_device *pdev)
{
    struct sensor_data *data = NULL;
    struct input_dev *input_data = NULL;
    int input_registered = 0, sysfs_created = 0;
    int rt;

	printk(KERN_INFO "%s: %s \n", SENSOR_NAME, __func__);

    data = kzalloc(sizeof(struct sensor_data), GFP_KERNEL);
    if (!data) {
        rt = -ENOMEM;
        goto err;
    }
    data->enabled = 0;
    data->delay = SENSOR_DEFAULT_DELAY;

    input_data = input_allocate_device();
    if (!input_data) {
        rt = -ENOMEM;
        YLOGE(("sensor_probe: Failed to allocate input_data device\n"));
        goto err;
    }

    set_bit(EV_ABS, input_data->evbit);
    input_set_capability(input_data, EV_ABS, ABS_X);
#if SENSOR_TYPE <= 4
    input_set_capability(input_data, EV_ABS, ABS_Y);
    input_set_capability(input_data, EV_ABS, ABS_Z);
#endif
    input_set_capability(input_data, EV_ABS, ABS_STATUS); /* status */
    input_set_capability(input_data, EV_ABS, ABS_WAKE); /* wake */
    input_set_capability(input_data, EV_ABS, ABS_CONTROL_REPORT); /* enabled/delay */
    input_set_abs_params(input_data, ABS_WAKE, 0, 0xffff, 0, 0);
    input_data->name = SENSOR_NAME;

    rt = input_register_device(input_data);
    if (rt) {
        YLOGE(("sensor_probe: Unable to register input_data device: %s\n",
               input_data->name));
        goto err;
    }
    input_set_drvdata(input_data, data);
    input_registered = 1;

    rt = sysfs_create_group(&input_data->dev.kobj,
            &sensor_attribute_group);
    if (rt) {
        YLOGE(("sensor_probe: sysfs_create_group failed[%s]\n",
               input_data->name));
        goto err;
    }

    sysfs_created = 1;

    rt = sysfs_create_link(input_data->dev.kobj.parent, 
                            &input_data->dev.kobj, 
                            input_data->name);
    if (rt) {        
        YLOGE(("sensor_probe: sysfs_create_link failed[%s]\n",
               input_data->name));
        goto err;
    }

    mutex_init(&data->mutex);
    this_data = input_data;

    return 0;

err:
    if (data != NULL) {
        if (input_data != NULL) {
            if (sysfs_created) {
                sysfs_remove_group(&input_data->dev.kobj,
                        &sensor_attribute_group);
            }
            if (input_registered) {
                input_unregister_device(input_data);
            }
            else {
                input_free_device(input_data);
            }
            input_data = NULL;
        }
        kfree(data);
    }

    printk(KERN_ERR "%s: %s err = %d\n", SENSOR_NAME, __func__, rt);

    return rt;
}

static int
sensor_remove(struct platform_device *pdev)
{
    struct sensor_data *data;

    if (this_data != NULL) {
        data = input_get_drvdata(this_data);        
        sysfs_remove_link(this_data->dev.kobj.parent, 
                          this_data->name);
        sysfs_remove_group(&this_data->dev.kobj,
                &sensor_attribute_group);
        input_unregister_device(this_data);
        if (data != NULL) {
            kfree(data);
        }
    }

    return 0;
}

/*
 * Module init and exit
 */
static struct platform_driver sensor_driver = {
    .probe      = sensor_probe,
    .remove     = sensor_remove,
    .suspend    = sensor_suspend,
    .resume     = sensor_resume,
    .driver = {
        .name   = SENSOR_NAME, 
        .owner  = THIS_MODULE,
    },
};

static int __init sensor_init(void *hdl)
{
	int get_cm3623;
    (void)hdl;
	
	get_cm3623 = Get_is_cm3623_exist();
	if(get_cm3623 == -1)
	{
		printk(KERN_ERR "[cm3623]%s: Get_is_cm3623_exist fail\n", __func__);		
	}
	else if(get_cm3623 == 0)
	{
		printk(KERN_INFO "[cm3623]%s: No capella cm3623\n", __func__);		
		return -EINVAL;
	}	
	else if(get_cm3623 == 1)
	{
		printk(KERN_INFO "[cm3623]%s: capella cm3623 detected!\n", __func__);		
	}	
	else
		printk(KERN_ERR "[cm3623]%s:Get_is_cm3623_exist error!\n", __func__);				
	
	
    sensor_pdev = platform_device_register_simple(SENSOR_NAME, 0, NULL, 0);
    if (IS_ERR(sensor_pdev)) {
        return -1;
    }
    return platform_driver_register(&sensor_driver);
}
DECLARE_DYN_LOADER(DRIVER_TYPE_PS, PSALS_CM3623_V1, sensor_init, LEVEL6);

static int __init sensor_init_2nd_source(void *hdl)
{
	return sensor_init(hdl);	
}

static int __init sensor_init_2nd_source_V1(void *hdl)
{
	return sensor_init(hdl);	
}

static int __init sensor_init_2nd_source_V2(void *hdl)
{
	return sensor_init(hdl);	
}

DECLARE_DYN_LOADER(DRIVER_TYPE_PS, STK3101, sensor_init_2nd_source, LEVEL6);
DECLARE_DYN_LOADER(DRIVER_TYPE_PS, STK3101_V1, sensor_init_2nd_source_V1, LEVEL6);
DECLARE_DYN_LOADER(DRIVER_TYPE_PS, STK3101_V2, sensor_init_2nd_source_V2, LEVEL6);


static int __init sensor_init_v2(void *hdl)
{
	int get_cm3623;
    (void)hdl;
	
	get_cm3623 = Get_is_cm3623_exist();
	if(get_cm3623 == -1)
	{
		printk(KERN_ERR "[cm3623]%s: Get_is_cm3623_exist fail\n", __func__);		
	}
	else if(get_cm3623 == 0)
	{
		printk(KERN_INFO "[cm3623]%s: No capella cm3623\n", __func__);		
		return -EINVAL;
	}	
	else if(get_cm3623 == 1)
	{
		printk(KERN_INFO "[cm3623]%s: capella cm3623 detected!\n", __func__);		
	}	
	else
		printk(KERN_ERR "[cm3623]%s:Get_is_cm3623_exist error!\n", __func__);				
	
    sensor_pdev = platform_device_register_simple(SENSOR_NAME, 0, NULL, 0);
    if (IS_ERR(sensor_pdev)) {
        return -1;
    }
    return platform_driver_register(&sensor_driver);
}
DECLARE_DYN_LOADER(DRIVER_TYPE_PS, PSALS_CM3623_V2, sensor_init_v2, LEVEL6);

static void __exit sensor_exit(void)
{
    platform_driver_unregister(&sensor_driver);
    platform_device_unregister(sensor_pdev);
}
module_exit(sensor_exit);

MODULE_AUTHOR("Yamaha Corporation");
MODULE_LICENSE( "GPL" );
MODULE_VERSION("3.0.402");
