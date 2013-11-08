/*
 * Copyright (c) 2010 Yamaha Corporation
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
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <fih/dynloader.h>

#define __LINUX_KERNEL_DRIVER__
#include "../include/yas.h"


#define YAS_ACC_KERNEL_VERSION                  "3.0.402"
#define YAS_ACC_KERNEL_NAME                     "accelerometer"
#define YAS_ACC_KERNEL_RAW_NAME                 "accelerometer_raw"
#define YAS_ACC_KERNEL_STEP_NAME                "accelerometer_step"

#define ABS_RAW_DISTORTION                  (ABS_THROTTLE)
#define ABS_RAW_THRESHOLD                   (ABS_RUDDER)
#define ABS_RAW_SHAPE                       (ABS_WHEEL)
#define ABS_RAW_REPORT                      (ABS_GAS)

/* ABS axes parameter range [um/s^2] (for input event) */
#define GRAVITY_EARTH                                                                 9806550
#define ABSMAX_2G                                                         (GRAVITY_EARTH * 2)
#define ABSMIN_2G                                                        (-GRAVITY_EARTH * 2)

#define delay_to_jiffies(d)                                       ((d)?msecs_to_jiffies(d):1)
#define actual_delay(d)                               (jiffies_to_msecs(delay_to_jiffies(d)))

#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef max
#define max(_a, _b)     (((_a) > (_b)) ? (_a) : (_b))
#endif

/* ---------------------------------------------------------------------------------------- *
   Function prototype declaration
 * ---------------------------------------------------------------------------------------- */
static struct yas_acc_private_data *yas_acc_get_data(void);
static void yas_acc_set_data(struct yas_acc_private_data *);

static int yas_acc_ischg_enable(struct yas_acc_driver *, int);

static int yas_acc_lock(void);
static int yas_acc_unlock(void);
static int yas_acc_i2c_open(void);
static int yas_acc_i2c_close(void);
static int yas_acc_i2c_write(uint8_t, uint8_t, const uint8_t *, int);
static int yas_acc_i2c_read(uint8_t, uint8_t, uint8_t *, int);
static void yas_acc_msleep(int);

static int yas_acc_core_driver_init(struct yas_acc_private_data *);
static void yas_acc_core_driver_fini(struct yas_acc_private_data *);
static int yas_acc_get_enable(struct yas_acc_driver *);
static int yas_acc_set_enable(struct yas_acc_driver *, int);
static int yas_acc_get_delay(struct yas_acc_driver *);
static int yas_acc_set_delay(struct yas_acc_driver *, int);
static int yas_acc_get_position(struct yas_acc_driver *);
static int yas_acc_set_position(struct yas_acc_driver *, int);
static int yas_acc_get_threshold(struct yas_acc_driver *);
static int yas_acc_set_threshold(struct yas_acc_driver *, int);
static int yas_acc_get_filter_enable(struct yas_acc_driver *);
static int yas_acc_set_filter_enable(struct yas_acc_driver *, int);
static int yas_acc_measure(struct yas_acc_driver *, struct yas_acc_data *);
static int yas_acc_input_init(struct yas_acc_private_data *);
static void yas_acc_input_fini(struct yas_acc_private_data *);

static ssize_t yas_acc_enable_show(struct device *, struct device_attribute *, char *);
static ssize_t yas_acc_enable_store(struct device *,struct device_attribute *, const char *, size_t);
static ssize_t yas_acc_delay_show(struct device *, struct device_attribute *, char *);
static ssize_t yas_acc_delay_store(struct device *, struct device_attribute *, const char *, size_t);
static ssize_t yas_acc_position_show(struct device *, struct device_attribute *, char *);
static ssize_t yas_acc_position_store(struct device *, struct device_attribute *, const char *, size_t);
static ssize_t yas_acc_threshold_show(struct device *, struct device_attribute *, char *);
static ssize_t yas_acc_threshold_store(struct device *, struct device_attribute *, const char *, size_t);
static ssize_t yas_acc_filter_enable_show(struct device *, struct device_attribute *, char *);
static ssize_t yas_acc_filter_enable_store(struct device *, struct device_attribute *, const char *, size_t);
static ssize_t yas_acc_wake_store(struct device *, struct device_attribute *, const char *, size_t);
static ssize_t yas_acc_private_data_show(struct device *, struct device_attribute *, char *);
#if DEBUG
static ssize_t yas_acc_debug_reg_show(struct device *, struct device_attribute *, char *);
static int yas_acc_suspend(struct i2c_client *, pm_message_t);
static int yas_acc_resume(struct i2c_client *);
#endif

static void yas_acc_work_func(struct work_struct *);
static int yas_acc_probe(struct i2c_client *, const struct i2c_device_id *);
static int yas_acc_remove(struct i2c_client *);
#ifndef CONFIG_PM
static int yas_acc_suspend(struct i2c_client *, pm_message_t);
static int yas_acc_resume(struct i2c_client *);
#endif
/* ---------------------------------------------------------------------------------------- *
   Driver private data
 * ---------------------------------------------------------------------------------------- */
struct yas_acc_private_data {
    struct mutex driver_mutex;
    struct mutex data_mutex;
    struct i2c_client *client;
    struct input_dev *input;
    struct input_dev *input_raw;

#ifdef CONFIG_INPUT_YAS_STEP
    struct input_dev    *input_step;
    int                 step;
    int                 stepEnable;
    int                 stepDelay;
    int                 delayCount;
    int                 delayCountTemp;
    int                 delayCountType;   /*0:no count down, 1: acc count donw, 2 : step count down*/
#endif
    int                 accEnable;    
    int                 accDelay;

    struct yas_acc_driver *driver;
    struct delayed_work work;
    struct yas_acc_data last;
    int suspend;
    int suspend_enable;

    struct FIH_gsensor *fih_acc_data;
};

static struct yas_acc_private_data *yas_acc_private_data = NULL;
static struct yas_acc_private_data *yas_acc_get_data(void) {return yas_acc_private_data;}
static void yas_acc_set_data(struct yas_acc_private_data *data) {yas_acc_private_data = data;}

/* ---------------------------------------------------------------------------------------- *
   Local function
 * ---------------------------------------------------------------------------------------- */
static int yas_acc_ischg_enable(struct yas_acc_driver *driver, int enable)
{
    if (driver->get_enable() == enable) {
        return 0;
    }

    return 1;
}

/* ---------------------------------------------------------------------------------------- *
   Accelerlomete core driver callback function
 * ---------------------------------------------------------------------------------------- */
static int yas_acc_lock(void)
{
    struct yas_acc_private_data *data = yas_acc_get_data();

    mutex_lock(&data->driver_mutex);

    return 0;
}

static int yas_acc_unlock(void)
{
    struct yas_acc_private_data *data = yas_acc_get_data();

    mutex_unlock(&data->driver_mutex);

    return 0;
}

static int yas_acc_i2c_open(void)
{
    return 0;
}

static int yas_acc_i2c_close(void)
{
    return 0;
}

static int yas_acc_i2c_write(uint8_t slave, uint8_t adr, const uint8_t *buf, int len)
{
    struct yas_acc_private_data *data = yas_acc_get_data();
    struct i2c_msg msg[2];
    char buffer[16];
    uint8_t reg;
    int err;
    int i;

    if (len > 15) {
        return -1;
    }

    reg = adr;
    buffer[0] = reg;
    for (i = 0; i < len; i++) {
        buffer[i+1] = buf[i];
    }

    msg[0].addr = slave;
    msg[0].flags = 0;
    msg[0].len = len + 1;
    msg[0].buf = buffer;
    err = i2c_transfer(data->client->adapter, msg, 1);
    if (err != 1) {
        dev_err(&data->client->dev,
                "i2c_transfer() write error: slave_addr=%02x, reg_addr=%02x, err=%d\n", slave, adr, err);
        return err;
    }

    return 0;
}

static int yas_acc_i2c_read(uint8_t slave, uint8_t adr, uint8_t *buf, int len)
{
    struct yas_acc_private_data *data = yas_acc_get_data();
    struct i2c_msg msg[2];
    uint8_t reg;
    int err;

    reg = adr;
    msg[0].addr = slave;
    msg[0].flags = 0;
    msg[0].len = 1;
    msg[0].buf = &reg;
    msg[1].addr = slave;
    msg[1].flags = I2C_M_RD;
    msg[1].len = len;
    msg[1].buf = buf;

    err = i2c_transfer(data->client->adapter, msg, 2);
    if (err != 2) {
        dev_err(&data->client->dev,
                "i2c_transfer() read error: slave_addr=%02x, reg_addr=%02x, err=%d\n", slave, adr, err);
        return err;
    }

    return 0;
}

static void yas_acc_msleep(int msec)
{
    msleep(msec);
}



/* ---------------------------------------------------------------------------------------- *
   Accelerometer core driver access function
 * ---------------------------------------------------------------------------------------- */
static int yas_acc_core_driver_init(struct yas_acc_private_data *data)
{
    struct yas_acc_driver_callback *cbk;
    struct yas_acc_driver *driver;
    int err;

    data->driver = driver = kzalloc(sizeof(struct yas_acc_driver), GFP_KERNEL);
    if (!driver) {
        err = -ENOMEM;
        return err;
    }

    cbk = &driver->callback;
    cbk->lock = yas_acc_lock;
    cbk->unlock = yas_acc_unlock;
    cbk->i2c_open = yas_acc_i2c_open;
    cbk->i2c_close = yas_acc_i2c_close;
    cbk->i2c_write = yas_acc_i2c_write;
    cbk->i2c_read = yas_acc_i2c_read;
    cbk->msleep = yas_acc_msleep;

    if(data->fih_acc_data != NULL) {
        err = data->fih_acc_data->driver_init(driver, data->fih_acc_data);
    }
    else {
        err = YAS_ERROR_NOT_INITIALIZED;
    }
    if (err != YAS_NO_ERROR) {
        kfree(driver);
        return err;
    }

    err = driver->init();
    if (err != YAS_NO_ERROR) {
        kfree(driver);
        return err;
    }

    err = driver->set_position(data->fih_acc_data->placement);
    if (err != YAS_NO_ERROR) {
        kfree(driver);
        return err;
    }

    return 0;
}

static void yas_acc_core_driver_fini(struct yas_acc_private_data *data)
{
    struct yas_acc_driver *driver = data->driver;

    driver->term();
    kfree(driver);
}

static int yas_acc_get_enable(struct yas_acc_driver *driver)
{
    return driver->get_enable();
}

static int yas_acc_set_enable(struct yas_acc_driver *driver, int enable)
{
    struct yas_acc_private_data *data = yas_acc_get_data();
    int delay = driver->get_delay();

    if (yas_acc_ischg_enable(driver, enable)) {
        if (enable) {
            driver->set_enable(enable);
            schedule_delayed_work(&data->work, delay_to_jiffies(delay) + 1);
        } else {
            cancel_delayed_work_sync(&data->work);
            driver->set_enable(enable);
        }
    }

    return 0;
}

static int yas_acc_set_suspend_power(struct yas_acc_driver *driver, int enable)
{
    if (yas_acc_ischg_enable(driver, enable)) {
        printk(KERN_INFO "%s (%d)\n", __func__, enable);
        driver->set_enable(enable);
    }

    return 0;
}

static int yas_acc_get_delay(struct yas_acc_driver *driver)
{
    return driver->get_delay();
}

static int yas_acc_set_delay(struct yas_acc_driver *driver, int delay)
{
    struct yas_acc_private_data *data = yas_acc_get_data();

    if (driver->get_enable()) {
        cancel_delayed_work_sync(&data->work);
        driver->set_delay(actual_delay(delay));
        schedule_delayed_work(&data->work, delay_to_jiffies(delay) + 1);
    } else {
        driver->set_delay(actual_delay(delay));
    }

    return 0;
}

static int yas_acc_get_offset(struct yas_acc_driver *driver, struct yas_vector *offset)
{
    return driver->get_offset(offset);
}

static int yas_acc_set_offset(struct yas_acc_driver *driver, struct yas_vector *offset)
{
    return driver->set_offset(offset);
}

static int yas_acc_get_position(struct yas_acc_driver *driver)
{
    return driver->get_position();
}

static int yas_acc_set_position(struct yas_acc_driver *driver, int position)
{
    return driver->set_position(position);
}

static int yas_acc_get_threshold(struct yas_acc_driver *driver)
{
    struct yas_acc_filter filter;

    driver->get_filter(&filter);

    return filter.threshold;
}

static int yas_acc_set_threshold(struct yas_acc_driver *driver, int threshold)
{
    struct yas_acc_filter filter;

    filter.threshold = threshold;

    return driver->set_filter(&filter);
}

static int yas_acc_get_filter_enable(struct yas_acc_driver *driver)
{
    return driver->get_filter_enable();
}

static int yas_acc_set_filter_enable(struct yas_acc_driver *driver, int enable)
{
    return driver->set_filter_enable(enable);
}

static int yas_acc_measure(struct yas_acc_driver *driver, struct yas_acc_data *accel)
{
    int err;

    err = driver->measure(accel);
    if (err != YAS_NO_ERROR) {
        return err;
    }

#if TRACE_OUTPUT
    printk("data(%10d %10d %10d) raw(%5d %5d %5d)\n",
           accel->xyz.v[0], 
           accel->xyz.v[1], 
           accel->xyz.v[2], 
           accel->raw.v[0], 
           accel->raw.v[1], 
           accel->raw.v[2]);
#endif

    return err;
}

/* ---------------------------------------------------------------------------------------- *
   Input device interface
 * ---------------------------------------------------------------------------------------- */
static int yas_acc_input_init(struct yas_acc_private_data *data)
{
    struct input_dev *input = NULL, *input_raw = NULL;
#ifdef CONFIG_INPUT_YAS_STEP
    struct input_dev *input_step = NULL;
#endif
    int err;

    input = input_allocate_device();
    if (!input) {
        return -ENOMEM;
    }
    input->name = "accelerometer";
    input->id.bustype = BUS_I2C;

    input_set_capability(input, EV_ABS, ABS_MISC);
    input_set_abs_params(input, ABS_X, ABSMIN_2G, ABSMAX_2G, 0, 0);
    input_set_abs_params(input, ABS_Y, ABSMIN_2G, ABSMAX_2G, 0, 0);
    input_set_abs_params(input, ABS_Z, ABSMIN_2G, ABSMAX_2G, 0, 0);
    input_set_drvdata(input, data);

    err = input_register_device(input);
    if (err < 0) {
        input_free_device(input);
        return err;
    }
    data->input = input;

    /* input_raw */
    input_raw = input_allocate_device();
    if (input_raw == NULL) {
        input_unregister_device(data->input);
        input_free_device(data->input);
        data->input = NULL;
        err = -ENOMEM;
        return err;
    }

    input_raw->name = YAS_ACC_KERNEL_RAW_NAME;
    set_bit(EV_ABS, input_raw->evbit);
    input_set_capability(input_raw, EV_ABS, ABS_MISC);
    input_set_abs_params(input_raw, ABS_X, ABSMIN_2G, ABSMAX_2G, 0, 0);
    input_set_abs_params(input_raw, ABS_Y, ABSMIN_2G, ABSMAX_2G, 0, 0);
    input_set_abs_params(input_raw, ABS_Z, ABSMIN_2G, ABSMAX_2G, 0, 0);
    input_set_capability(input_raw, EV_ABS, ABS_RAW_DISTORTION);
    input_set_capability(input_raw, EV_ABS, ABS_RAW_THRESHOLD);
    input_set_capability(input_raw, EV_ABS, ABS_RAW_SHAPE);
    input_set_capability(input_raw, EV_ABS, ABS_RAW_REPORT);
    input_set_drvdata(input_raw, data);

    err = input_register_device(input_raw);
    if (err) {
        input_unregister_device(data->input);
        input_free_device(data->input);
        input_free_device(input_raw);
        data->input     = NULL;
        data->input_raw = NULL;
        err = -ENOMEM;
        return err;

    }

    data->input_raw = input_raw;

#ifdef CONFIG_INPUT_YAS_STEP
    /* input_step */
    input_step = input_allocate_device();
    if (input_step == NULL) {
        input_unregister_device(data->input_raw);
        input_unregister_device(data->input);
        input_free_device(data->input);
        input_free_device(data->input_raw);
        data->input     = NULL;
        data->input_raw = NULL;
        err = -ENOMEM;
        return err;
    }

    input_step->name = YAS_ACC_KERNEL_STEP_NAME;
    set_bit(EV_ABS, input_step->evbit);

    input_set_capability(input_step, EV_ABS, ABS_MISC);
    input_set_abs_params(input_step, ABS_X, 0, 0xffff, 0, 0);
    input_set_abs_params(input_step, ABS_Y, 0, 0xffff, 0, 0);
    input_set_abs_params(input_step, ABS_Z, 0, 0xffff, 0, 0);
    input_set_capability(input_step, EV_ABS, ABS_RAW_DISTORTION);
    input_set_capability(input_step, EV_ABS, ABS_RAW_THRESHOLD);
    input_set_capability(input_step, EV_ABS, ABS_RAW_SHAPE);
    input_set_capability(input_step, EV_ABS, ABS_RAW_REPORT);
    input_set_drvdata(input_step, data);

    err = input_register_device(input_step);
    if (err) {
        input_unregister_device(data->input_raw);
        input_unregister_device(data->input);
        input_free_device(data->input);
        input_free_device(data->input_raw);
        input_free_device(input_step);
        data->input         = NULL;
        data->input_raw     = NULL;
        data->input_step    = NULL;
        err = -ENOMEM;
    }

    data->input_step = input_step;
#endif

    return 0;
}

static void yas_acc_input_fini(struct yas_acc_private_data *data)
{
    if(data->input_raw){
        input_unregister_device(data->input_raw);
        input_free_device(data->input_raw);
        data->input_raw = NULL;
    }
    if(data->input){
        input_unregister_device(data->input);
        input_free_device(data->input);
        data->input = NULL;
    }

#ifdef CONFIG_INPUT_YAS_STEP
    if(data->input_step){
        input_unregister_device(data->input_step);
        input_free_device(data->input_step);
        data->input_step = NULL;
    }
#endif
}

static ssize_t yas_acc_enable_show(struct device *dev,
                                   struct device_attribute *attr,
                                   char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);

    return sprintf(buf, "%d\n", (data->accEnable ? yas_acc_get_enable(data->driver) : data->accEnable));
}

static ssize_t yas_acc_enable_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf,
                                    size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);
    unsigned long enable = simple_strtoul(buf, NULL, 10);

    printk(KERN_INFO "yas_acc_enable_store +++\n");

    mutex_lock(&data->data_mutex);
    data->accEnable = enable;
#ifdef CONFIG_INPUT_YAS_STEP
    enable = (data->accEnable || data->stepEnable);
#endif 
    mutex_unlock(&data->data_mutex);

    yas_acc_set_enable(data->driver, enable);

    printk(KERN_INFO "yas_acc_enable_store ---\n");
	
    return count;
}

static ssize_t yas_acc_delay_show(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);

    return sprintf(buf, "%d\n", (data->accDelay ? yas_acc_get_delay(data->driver) : data->accDelay));
}

static ssize_t yas_acc_delay_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);
    unsigned long delay = simple_strtoul(buf, NULL, 10);    

    printk(KERN_INFO "yas_acc_delay_store+++\n");

    mutex_lock(&data->data_mutex);
    data->accDelay = delay;

#ifdef CONFIG_INPUT_YAS_STEP
    if(data->accDelay > data->stepDelay)
    {
        if(0 == data->stepDelay)
        {
            delay                   = data->accDelay;
            data->delayCount        = 0;
            data->delayCountTemp    = 0;
            data->delayCountType    = 0;
        }else
        {
            delay                   = data->stepDelay;            
            data->delayCount        = (data->accDelay / data->stepDelay);
            data->delayCountTemp    = data->delayCount;
            data->delayCountType    = 1;
        }       
    }
    else
    {
        if(0 == data->accDelay)
        {
            delay                   = data->stepDelay;
            data->delayCount        = 0;
            data->delayCountTemp    = 0;
            data->delayCountType    = 0;
        }else
        {
            delay                   = data->accDelay;            
            data->delayCount        = (data->stepDelay / data->accDelay);
            data->delayCountTemp    = data->delayCount;
            data->delayCountType    = 2;
        }
    }
#endif 
    mutex_unlock(&data->data_mutex);

    yas_acc_set_delay(data->driver, delay);
    printk(KERN_INFO "yas_acc_delay_store---\n");

    return count;
}

static ssize_t yas_acc_offset_show(struct device *dev,
                                   struct device_attribute *attr,
                                   char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);
    struct yas_vector offset;

    yas_acc_get_offset(data->driver, &offset);

    return sprintf(buf, "%d %d %d\n", offset.v[0], offset.v[1], offset.v[2]);
}

static ssize_t yas_acc_offset_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf,
                                    size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);
    struct yas_vector offset;
    static int raw_report = 0;

    sscanf(buf, "%d %d %d", &offset.v[0], &offset.v[1], &offset.v[2]);

#if TRACE_OUTPUT
    printk(KERN_INFO "%s: %s %d %d %d\n", 
           YAS_ACC_KERNEL_NAME, 
           __func__,
           offset.v[0], offset.v[1], offset.v[2]);
#endif

    yas_acc_set_offset(data->driver, &offset);

    /*save offset to calibration config*/
    if(data->input_raw){
        raw_report |= YAS_REPORT_CALIB_OFFSET_CHANGED;		
        input_report_abs(data->input_raw, ABS_RAW_REPORT, raw_report);
        raw_report += 1;
        raw_report &= 1;
    }

    return count;
}

static ssize_t yas_acc_calibration_show(struct device *dev,
                                   struct device_attribute *attr,
                                   char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);
    struct yas_vector offset;

    yas_acc_get_offset(data->driver, &offset);

    return sprintf(buf, "%d %d %d\n", offset.v[0], offset.v[1], offset.v[2]);
}

static ssize_t yas_acc_calibration_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf,
                                    size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);
    struct yas_vector offset;

    sscanf(buf, "%d %d %d", &offset.v[0], &offset.v[1], &offset.v[2]);

#if TRACE_OUTPUT
	printk(KERN_INFO "%s: %s %d %d %d\n", 
           YAS_ACC_KERNEL_NAME, 
           __func__,
           offset.v[0], offset.v[1], offset.v[2]);
#endif

    yas_acc_set_offset(data->driver, &offset);

    return count;
}

static ssize_t yas_acc_position_show(struct device *dev,
                                     struct device_attribute *attr,
                                     char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);

    return sprintf(buf, "%d\n", yas_acc_get_position(data->driver));
}

static ssize_t yas_acc_position_store(struct device *dev,
                                      struct device_attribute *attr,
                                      const char *buf,
                                      size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);
    unsigned long position = simple_strtoul(buf, NULL,10);

    yas_acc_set_position(data->driver, position);

    return count;
}

static ssize_t yas_acc_threshold_show(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);

    return sprintf(buf, "%d\n", yas_acc_get_threshold(data->driver));
}

static ssize_t yas_acc_threshold_store(struct device *dev,
                                       struct device_attribute *attr,
                                       const char *buf,
                                       size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);
    unsigned long threshold = simple_strtoul(buf, NULL,10);

    yas_acc_set_threshold(data->driver, threshold);

    return count;
}

static ssize_t yas_acc_filter_enable_show(struct device *dev,
                                          struct device_attribute *attr,
                                          char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);

    return sprintf(buf, "%d\n", yas_acc_get_filter_enable(data->driver));
}

static ssize_t yas_acc_filter_enable_store(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf,
                                           size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);
    unsigned long enable = simple_strtoul(buf, NULL,10);;

    yas_acc_set_filter_enable(data->driver, enable);

    return count;
}

static ssize_t yas_acc_wake_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf,
                                  size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    static atomic_t serial = ATOMIC_INIT(0);

    input_report_abs(input, ABS_MISC, atomic_inc_return(&serial));

    return count;
}

static ssize_t yas_acc_private_data_show(struct device *dev,
                                         struct device_attribute *attr,
                                         char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);
    struct yas_acc_data accel;

    mutex_lock(&data->data_mutex);
    accel = data->last;
    mutex_unlock(&data->data_mutex);

    return sprintf(buf, "%d %d %d\n", accel.xyz.v[0], accel.xyz.v[1], accel.xyz.v[2]);
}

#if DEBUG
#if YAS_ACC_DRIVER == YAS_ACC_DRIVER_BMA150
#define ADR_MAX (0x16)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_BMA222 || \
      YAS_ACC_DRIVER == YAS_ACC_DRIVER_BMA250
#define ADR_MAX (0x40)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_KXSD9
#define ADR_MAX (0x0f)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_KXTE9
#define ADR_MAX (0x5c)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_KXTF9
#define ADR_MAX (0x60)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_LIS331DL  || \
      YAS_ACC_DRIVER == YAS_ACC_DRIVER_LIS331DLH || \
      YAS_ACC_DRIVER == YAS_ACC_DRIVER_LIS331DLM
#define ADR_MAX (0x40)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_LIS3DH
#define ADR_MAX (0x3e)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_ADXL345
#define ADR_MAX (0x3a)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_ADXL346
#define ADR_MAX (0x3d)
#elif YAS_ACC_DRIVER == YAS_ACC_DRIVER_MMA8452Q || \
      YAS_ACC_DRIVER == YAS_ACC_DRIVER_MMA8453Q
#define ADR_MAX (0x32)
#else
#define ADR_MAX (0x16)
#endif

#ifdef ADR_MAX
#undef ADR_MAX
#define ADR_MAX (0x60)	// Use max value (FIH)
#endif
static uint8_t reg[ADR_MAX];
static int FIH_ADR_MAX = 0x16;



static ssize_t yas_acc_debug_reg_show(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);
    struct i2c_client *client = data->client;
    ssize_t count = 0;
    int ret;
    int i;

    memset(reg, -1, FIH_ADR_MAX);
    for (i = 0; i < FIH_ADR_MAX; i++) {
        ret = data->driver->get_register(i, &reg[i]);
        if(ret != 0) {
            dev_err(&client->dev, "get_register() erorr %d (%d)\n", ret, i);
        } else {
            count += sprintf(&buf[count], "%02x: %d\n", i, reg[i]);
        }
    }

    return count;
}

static ssize_t yas_acc_debug_suspend_show(struct device *dev,
                                          struct device_attribute *attr,
                                          char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);

    return sprintf(buf, "%d\n", data->suspend);
}

static ssize_t yas_acc_debug_suspend_store(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf, size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);
    struct i2c_client *client = data->client;
    unsigned long suspend = simple_strtoul(buf, NULL, 10);

    if (suspend) {
        pm_message_t msg;
        yas_acc_suspend(client, msg);
    } else {
        yas_acc_resume(client);
    }

    return count;
}
#endif /* DEBUG */

#ifdef CONFIG_INPUT_YAS_STEP
static ssize_t yas_acc_step_enable_show(struct device *dev,
                                   struct device_attribute *attr,
                                   char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);

    return sprintf(buf, "%d\n", (data->stepEnable ? yas_acc_get_enable(data->driver): data->stepEnable));
}

static ssize_t yas_acc_step_enable_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf,
                                    size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);
    unsigned long enable = simple_strtoul(buf, NULL, 10);

    mutex_lock(&data->data_mutex);
    data->stepEnable    = enable;
    data->step          = 0;
    enable              = (data->accEnable || data->stepEnable);
    mutex_unlock(&data->data_mutex);

    yas_acc_set_enable(data->driver, enable);

    return count;
}

static ssize_t yas_acc_step_delay_show(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);

    return sprintf(buf, "%d\n", (data->stepDelay ? yas_acc_get_delay(data->driver) : data->stepDelay));
}

static ssize_t yas_acc_step_delay_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);
    unsigned long delay = simple_strtoul(buf, NULL, 10);

    mutex_lock(&data->data_mutex);
    data->stepDelay = delay; 

    if(data->accDelay > data->stepDelay)
    {
        if(0 == data->stepDelay)
        {
            delay                   = data->accDelay;
            data->delayCount        = 0;
            data->delayCountTemp    = 0;
            data->delayCountType    = 0;
        }else
        {
            delay                   = data->stepDelay;            
            data->delayCount        = (data->accDelay / data->stepDelay);
            data->delayCountTemp    = data->delayCount;
            data->delayCountType    = 1;
        }       
    }
    else
    {
        if(0 == data->accDelay)
        {
            delay                   = data->stepDelay;
            data->delayCount        = 0;
            data->delayCountTemp    = 0;
            data->delayCountType    = 0;
        }else
        {
            delay                   = data->accDelay;            
            data->delayCount        = (data->stepDelay / data->accDelay);
            data->delayCountTemp    = data->delayCount;
            data->delayCountType    = 2;
        }
    }

    mutex_unlock(&data->data_mutex);

    yas_acc_set_delay(data->driver, delay);

    return count;
}

static ssize_t yas_acc_step_wake_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf,
                                  size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    static atomic_t serial = ATOMIC_INIT(0);

    input_report_abs(input, ABS_MISC, atomic_inc_return(&serial));

    return count;
}

static ssize_t yas_acc_step_private_data_show(struct device *dev,
                                         struct device_attribute *attr,
                                         char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct yas_acc_private_data *data = input_get_drvdata(input);
    struct yas_acc_data accel;

    mutex_lock(&data->data_mutex);
    accel = data->last;
    mutex_unlock(&data->data_mutex);

    return sprintf(buf, "%d %d %d\n", accel.xyz.v[0], accel.xyz.v[1], accel.xyz.v[2]);
}

#endif /*CONFIG_INPUT_YAS_STEP*/


static ssize_t yas_acc_chipinfo_show(struct device *dev,
                                   struct device_attribute *attr,
                                   char *buf)
{
	struct yas_acc_private_data *data = yas_acc_get_data();

	if(data->fih_acc_data == NULL) {
		return sprintf(buf, "bma150\n");
	}
	
	return sprintf(buf, "%s\n", data->fih_acc_data->chipinfo);
}

static ssize_t yas_acc_distortion_show(struct device *dev,
                                   struct device_attribute *attr,
                                   char *buf)
{
	struct yas_acc_private_data *data = yas_acc_get_data();

	if(data->fih_acc_data == NULL) {
		return sprintf(buf, "4000\n");
	}
	
	return sprintf(buf, "%d\n", data->fih_acc_data->distortion);
}

static DEVICE_ATTR(chipinfo,
                   S_IRUGO|S_IWUSR,
                   yas_acc_chipinfo_show,
                   NULL
                   );

static DEVICE_ATTR(distortion,
                   S_IRUGO|S_IWUSR,
                   yas_acc_distortion_show,
                   NULL
                   );

static DEVICE_ATTR(enable,
                   S_IRUGO|S_IWUSR|S_IWGRP,
                   yas_acc_enable_show,
                   yas_acc_enable_store
                   );
static DEVICE_ATTR(delay,
                   S_IRUGO|S_IWUSR|S_IWGRP,
                   yas_acc_delay_show,
                   yas_acc_delay_store
                   );
static DEVICE_ATTR(offset,
                   S_IRUGO|S_IWUSR,
                   yas_acc_offset_show,
                   yas_acc_offset_store
                   );
static DEVICE_ATTR(position,
                   S_IRUGO|S_IWUSR,
                   yas_acc_position_show,
                   yas_acc_position_store
                   );
static DEVICE_ATTR(threshold,
                   S_IRUGO|S_IWUSR,
                   yas_acc_threshold_show,
                   yas_acc_threshold_store
                   );
static DEVICE_ATTR(filter_enable,
                   S_IRUGO|S_IWUSR,
                   yas_acc_filter_enable_show,
                   yas_acc_filter_enable_store
                   );
static DEVICE_ATTR(wake,
                   S_IWUSR|S_IWGRP,
                   NULL,
                   yas_acc_wake_store);
static DEVICE_ATTR(data,
                   S_IRUGO,
                   yas_acc_private_data_show,
                   NULL);
#if DEBUG
static DEVICE_ATTR(debug_reg,
                   S_IRUGO,
                   yas_acc_debug_reg_show,
                   NULL
                   );
static DEVICE_ATTR(debug_suspend,
                   S_IRUGO|S_IWUSR,
                   yas_acc_debug_suspend_show,
                   yas_acc_debug_suspend_store
                   );
#endif /* DEBUG */

static DEVICE_ATTR(calibration,
                   S_IRUGO|S_IWUSR,
                   yas_acc_calibration_show,
                   yas_acc_calibration_store
                   );

/*----------------------------------------------------------------------------------*/
#ifdef CONFIG_INPUT_YAS_STEP
#define DEVICE_ATTR_EX(_devname, _attrname, _mode, _show, _store) \
struct device_attribute dev_attr_##_devname = __ATTR(_attrname, _mode, _show, _store)

static DEVICE_ATTR_EX(step_enable,
                      enable,
                      S_IRUGO|S_IWUSR|S_IWGRP,
                      yas_acc_step_enable_show,
                      yas_acc_step_enable_store
                     );
static DEVICE_ATTR_EX(step_delay,
                      delay,
                      S_IRUGO|S_IWUSR|S_IWGRP,
                      yas_acc_step_delay_show,
                      yas_acc_step_delay_store
                     );
static DEVICE_ATTR_EX(step_wake,
                      wake,
                      S_IWUSR|S_IWGRP,
                      NULL,
                      yas_acc_step_wake_store);
static DEVICE_ATTR_EX(step_data,
                      data,
                      S_IRUGO,
                      yas_acc_step_private_data_show,
                      NULL);

static struct attribute *yas_acc_step_attributes[] = {
    &dev_attr_step_enable.attr,
    &dev_attr_step_delay.attr,
    &dev_attr_step_wake.attr,
    &dev_attr_step_data.attr,
    NULL
};

static struct attribute_group yas_acc_step_attribute_group = {
    .attrs = yas_acc_step_attributes
};
#endif /*CONFIG_INPUT_YAS_STEP*/
/*----------------------------------------------------------------------------------*/

static struct attribute *yas_acc_attributes[] = {
    &dev_attr_enable.attr,
    &dev_attr_delay.attr,
    &dev_attr_offset.attr,
    &dev_attr_position.attr,
    &dev_attr_threshold.attr,
    &dev_attr_filter_enable.attr,
    &dev_attr_wake.attr,
    &dev_attr_data.attr,
#if DEBUG
    &dev_attr_debug_reg.attr,
    &dev_attr_debug_suspend.attr,
#endif /* DEBUG */
    &dev_attr_distortion.attr,
    &dev_attr_chipinfo.attr,    
    NULL
};

static struct attribute *yas_acc_raw_attributes[] = {
    &dev_attr_calibration.attr,
    NULL
};

static struct attribute_group yas_acc_attribute_group = {
    .attrs = yas_acc_attributes
};

static struct attribute_group yas_acc_raw_attribute_group = {
    .attrs = yas_acc_raw_attributes
};

static void yas_acc_work_func(struct work_struct *work)
{
    struct yas_acc_private_data *data = container_of((struct delayed_work *)work,
                                              struct yas_acc_private_data, work);
    struct yas_acc_data accel;
    unsigned long delay = delay_to_jiffies(yas_acc_get_delay(data->driver));
    int err;
    int enable = yas_acc_get_enable(data->driver);

    accel.xyz.v[0] = accel.xyz.v[1] = accel.xyz.v[2] = 0;
    if(enable == 0 ||
		(err = yas_acc_measure(data->driver, &accel)) != YAS_NO_ERROR) {
        printk(KERN_INFO "%s: %d\n", __func__, enable);		
        goto ERR;
    }

#if TRACE_OUTPUT
	printk(KERN_INFO "%s: %s \n", YAS_ACC_KERNEL_NAME, __func__);
#endif

#ifdef CONFIG_INPUT_YAS_STEP   
    if(data->accEnable)
    {
        int report = 0;

        switch(data->delayCountType)
        {
        case 2:            
        case 0:
            report = 1;
            break;
        case 1:
            if(0 == data->delayCountTemp)
            {      
                data->delayCountTemp = data->delayCount;      
                report = 1;
            }else
            {
                data->delayCountTemp--;
                report = 0;
            }
            break;
        }

        if(report)
        {
#if TRACE_OUTPUT
            printk("yas_acc_work_func : data(%10d %10d %10d) raw(%5d %5d %5d)\n",
                   accel.xyz.v[0], 
                   accel.xyz.v[1], 
                   accel.xyz.v[2], 
                   accel.raw.v[0], 
                   accel.raw.v[1], 
                   accel.raw.v[2]);
#endif
            input_report_abs(data->input, ABS_X, accel.xyz.v[0]);
            input_report_abs(data->input, ABS_Y, accel.xyz.v[1]);
            input_report_abs(data->input, ABS_Z, accel.xyz.v[2]);
            input_sync(data->input);
        }
    }

    if(data->stepEnable)
    {        
        int report = 0;

        switch(data->delayCountType)
        {
        case 1:            
        case 0:
            report = 1;
            break;
        case 2:
            if(0 == data->delayCountTemp)
            {      
                data->delayCountTemp = data->delayCount;      
                report = 1;
            }else
            {
                data->delayCountTemp--;
                report = 0;
            }
            break;
        }

        if(report)
        {
#if TRACE_OUTPUT
            printk("yas_acc_work_func : step(%10d)\n", data->step);
#endif
            input_report_abs(data->input_step, ABS_X, 1000*data->step++);
            input_report_abs(data->input_step, ABS_Y, 0);
            input_report_abs(data->input_step, ABS_Z, 0);
            input_sync(data->input_step);
        }
    }
#else
    input_report_abs(data->input, ABS_X, accel.xyz.v[0]);
    input_report_abs(data->input, ABS_Y, accel.xyz.v[1]);
    input_report_abs(data->input, ABS_Z, accel.xyz.v[2]);
    input_sync(data->input);
#endif

    mutex_lock(&data->data_mutex);
    data->last = accel;
    mutex_unlock(&data->data_mutex);

ERR:
    schedule_delayed_work(&data->work, delay);
}

static int yas_acc_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct yas_acc_private_data *data;
    int err;

	printk(KERN_INFO "%s: %s \n", YAS_ACC_KERNEL_NAME, __func__);

    /* Setup private data */
    data = kzalloc(sizeof(struct yas_acc_private_data), GFP_KERNEL);
    if (!data) {
        err = -ENOMEM;
        goto ERR1;
    }
    yas_acc_set_data(data);

    mutex_init(&data->driver_mutex);
    mutex_init(&data->data_mutex);

    /* Setup i2c client */
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        err = -ENODEV;
        goto ERR2;
    }
    data->fih_acc_data = (struct FIH_gsensor *)client->dev.platform_data;
    if(data->fih_acc_data != NULL) {
#if DEBUG		
        FIH_ADR_MAX = data->fih_acc_data->reg_max;
#endif
        ;
    }	
    
    i2c_set_clientdata(client, data);
    data->client = client;

    /* Setup accelerometer core driver */
    err = yas_acc_core_driver_init(data);
    if (err < 0) {
        goto ERR2;
    }

    /* Setup driver interface */
    INIT_DELAYED_WORK(&data->work, yas_acc_work_func);

    /* Setup input device interface */
    err = yas_acc_input_init(data);
    if (err < 0) {
        goto ERR3;
    }

    /* Setup sysfs */
    err = sysfs_create_group(&data->input->dev.kobj, &yas_acc_attribute_group);
    if (err < 0) {
        goto ERR4;
    }

    err = sysfs_create_link(data->input->dev.kobj.parent, 
                            &data->input->dev.kobj, 
                            data->input->name);
    if (err < 0) {
        goto ERR5;
    }

    err = sysfs_create_group(&data->input_raw->dev.kobj, &yas_acc_raw_attribute_group);
    if (err) {
        goto ERR6;
    } 

    err = sysfs_create_link(data->input_raw->dev.kobj.parent, 
                            &data->input_raw->dev.kobj, 
                            data->input_raw->name);
    if (err < 0) {
        goto ERR7;
    }

#ifdef CONFIG_INPUT_YAS_STEP
    err = sysfs_create_group(&data->input_step->dev.kobj, &yas_acc_step_attribute_group);
    if (err) {
        goto ERR8;
    }    

    err = sysfs_create_link(data->input_step->dev.kobj.parent, 
                            &data->input_step->dev.kobj, 
                            data->input_step->name);
    if (err < 0) {
        goto ERR9;
    }
#endif

    return 0;

#ifdef CONFIG_INPUT_YAS_STEP
ERR9:
    sysfs_remove_group(&data->input_step->dev.kobj, &yas_acc_step_attribute_group);
ERR8:
    sysfs_remove_link(data->input_raw->dev.kobj.parent, data->input_raw->name);
#endif

ERR7:
    sysfs_remove_group(&data->input_raw->dev.kobj, &yas_acc_raw_attribute_group);
ERR6:
    sysfs_remove_link(data->input->dev.kobj.parent, data->input->name);
ERR5:
    sysfs_remove_group(&data->input->dev.kobj, &yas_acc_attribute_group);
ERR4:
    yas_acc_input_fini(data);
ERR3:
    yas_acc_core_driver_fini(data);
ERR2:
    kfree(data);
ERR1:

    printk(KERN_ERR "%s: %s err = %d\n", YAS_ACC_KERNEL_NAME, __func__, err);
    return err;
}

static int yas_acc_remove(struct i2c_client *client)
{
    struct yas_acc_private_data *data = i2c_get_clientdata(client);
    struct yas_acc_driver *driver = data->driver;

    yas_acc_set_enable(driver, 0);

#ifdef CONFIG_INPUT_YAS_STEP
    sysfs_remove_link(data->input_step->dev.kobj.parent, data->input_step->name);
    sysfs_remove_group(&data->input_step->dev.kobj, &yas_acc_step_attribute_group);
#endif

    sysfs_remove_link(data->input_raw->dev.kobj.parent, data->input_raw->name);
    sysfs_remove_link(data->input->dev.kobj.parent, data->input->name);

    sysfs_remove_group(&data->input_raw->dev.kobj, &yas_acc_raw_attribute_group);
    sysfs_remove_group(&data->input->dev.kobj, &yas_acc_attribute_group);

    yas_acc_input_fini(data);
    yas_acc_core_driver_fini(data);
    kfree(data);

    return 0;
}

#ifndef CONFIG_PM
static int yas_acc_suspend(struct i2c_client *client, pm_message_t mesg)
{
    struct yas_acc_private_data *data = i2c_get_clientdata(client);
    struct yas_acc_driver *driver = data->driver;

    mutex_lock(&data->data_mutex);

    if (data->suspend == 0) {
        data->suspend_enable = yas_acc_get_enable(driver);
        if (data->suspend_enable) {
            cancel_delayed_work_sync(&data->work);
            yas_acc_set_enable(driver, 0);
        }
    }
    data->suspend = 1;

    mutex_unlock(&data->data_mutex);

    return 0;
}

static int yas_acc_resume(struct i2c_client *client)
{
    struct yas_acc_private_data *data = i2c_get_clientdata(client);
    struct yas_acc_driver *driver = data->driver;
    int delay;

    mutex_lock(&data->data_mutex);

    if (data->suspend == 1) {
        if (data->suspend_enable) {
            delay = yas_acc_get_delay(driver);
            schedule_delayed_work(&data->work, delay_to_jiffies(delay) + 1);
            yas_acc_set_enable(driver, 1);
        }
    }
    data->suspend = 0;

    mutex_unlock(&data->data_mutex);

    return 0;
}
#endif

#ifdef CONFIG_PM
static int yas_acc_pm_suspend(struct device *dev) 
{
    struct yas_acc_private_data *data = dev_get_drvdata(dev);
    struct yas_acc_driver *driver = data->driver;

	printk(KERN_INFO "%s\n", __func__);

    mutex_lock(&data->data_mutex);

    if (data->suspend == 0) {
        data->suspend_enable = yas_acc_get_enable(driver);
        if (data->suspend_enable) {
            //cancel_delayed_work_sync(&data->work);
            //yas_acc_set_enable(driver, 0);
            yas_acc_set_suspend_power(driver, 0);
        }
    }
    data->suspend = 1;

    mutex_unlock(&data->data_mutex);

    return 0;
}

static int yas_acc_pm_resume(struct device *dev) {

    struct yas_acc_private_data *data = dev_get_drvdata(dev);
    struct yas_acc_driver *driver = data->driver;
    int delay;

	printk(KERN_INFO "%s\n", __func__);

    mutex_lock(&data->data_mutex);

    if (data->suspend == 1) {
        if (data->suspend_enable) {
            //delay = yas_acc_get_delay(driver);
            //schedule_delayed_work(&data->work, delay_to_jiffies(delay) + 1);
            //yas_acc_set_enable(driver, 1);
            (void)delay;
            yas_acc_set_suspend_power(driver, 1);
        }
    }
    data->suspend = 0;

    mutex_unlock(&data->data_mutex);

    return 0;
}

static struct dev_pm_ops yas_acc_pm_ops = {
    .suspend        = yas_acc_pm_suspend,
    .resume         = yas_acc_pm_resume,
};
#endif

static const struct i2c_device_id yas_acc_id[] = {
    {YAS_ACC_KERNEL_NAME, 0},
    {},
};

MODULE_DEVICE_TABLE(i2c, yas_acc_id);

struct i2c_driver yas_acc_driver = {
    .driver = {
        .name = "accelerometer",
        .owner = THIS_MODULE,
#ifdef CONFIG_PM
        .pm = &yas_acc_pm_ops,
#endif
    },
    .probe = yas_acc_probe,
    .remove = yas_acc_remove,
#ifndef CONFIG_PM
    .suspend = yas_acc_suspend,
    .resume = yas_acc_resume,
#endif
    .id_table = yas_acc_id,
};

/*E: for ftm mode */
static int yas_acc_open(struct inode *inode, struct file *file)
{
    printk(KERN_INFO "[yas_acc]%s\n", __func__);
	return 0;
}

static ssize_t yas_acc_read(struct file *file, char __user *buffer, size_t size, loff_t *f_ops)
{	
    printk(KERN_INFO "[yas_acc]%s\n", __func__);
	return 0; 
}

/*GSENSOR*/ 
#define SENSOR_GSENSOR_ON	    0x0C00
#define SENSOR_GSENSOR_OFF		0x0C01
#define SENSOR_GSENSOR_READID	0x0C02
#define SENSOR_GSENSOR_READXYZ	0x0C03

static long yas_acc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
    struct yas_acc_private_data *data = yas_acc_get_data();
    struct yas_acc_driver       *driver;// = data->driver; 
                                        
    printk(KERN_INFO "[yas_acc] %s : cmd = 0x%x, arg = 0x%x\n", __func__, cmd, (unsigned int)arg);

    if(!data || !data->driver)
    {
        printk(KERN_INFO "[yas_acc] %s :error\n", __func__);
        return -EFAULT;
    }

    driver = data->driver;               

	switch (cmd)
	{/*GSENSOR*/ 
    case SENSOR_GSENSOR_ON:
        driver->set_enable(1);
        break;
    case SENSOR_GSENSOR_OFF:
        driver->set_enable(0);
        break;
    case SENSOR_GSENSOR_READID:
        {
            int id = 00;

            if (copy_to_user(argp, &id, sizeof(int)))
            {
                return -EFAULT;
            }
        }
        break;
    case SENSOR_GSENSOR_READXYZ:
        {
            struct yas_acc_data accdata;
            int rt;

            if (data->driver->measure == NULL) 
            {
                return -EFAULT;
            }

            rt = data->driver->measure(&accdata);
            if (rt < 0) {
                YLOGE(("measure failed[%d]\n", rt));
            }
            YLOGD(("raw [%d][%d][%d] xyz[%d][%d][%d]\n",
                    accdata.raw.v[0], accdata.raw.v[1], accdata.raw.v[2],
                    accdata.xyz.v[0], accdata.xyz.v[1], accdata.xyz.v[2]));

            if (copy_to_user(argp, &accdata, sizeof(struct yas_acc_data)))
            {
                return -EFAULT;
            }
        }
        break;
    default:
        return -1;
    }
	return 0;
}

static int yas_acc_release(struct inode *inode, struct file *filp)
{
    printk(KERN_INFO "[yas_acc]%s\n", __func__);
	return 0;
}

static const struct file_operations yas_acc_dev_fops = {
	.owner = THIS_MODULE,
	.open = yas_acc_open,
	.read = yas_acc_read,
	.unlocked_ioctl = yas_acc_ioctl,
	.release = yas_acc_release,
};

static struct miscdevice yas_acc_dev =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "bma020",             //reserve the file name for FTM can control
    .fops = &yas_acc_dev_fops,
};
/*E: for ftm mode */



/*FIHTDC-DerrickDRLiu-[IRM] FTM test function usage*/
#ifdef CONFIG_FIH_7X27A_PROJS
#define BMA150_NAME "bma150"
#define BMA250_NAME "bma250"

#define BMA150_CHIP_ID_REG		0x00
#define BMA250_CHIP_ID_REG		0x00
#define BMA150_CHIP_ID			0x02
#define BMA250_CHIP_ID          0x03
#define BMA150_DETECT_CHIP (1)
#define BMA250_DETECT_CHIP 0x562

static int bma150_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	int id;

	id = i2c_smbus_read_byte_data(client, BMA150_CHIP_ID_REG);
	if (id != BMA150_CHIP_ID)
	{
		printk(KERN_ERR "Bosch Gsensor not found" \
			"i2c may error %d\n", id);
		return -ENODEV;
	}else{
		printk(KERN_INFO "Bosch Gsensor Device detected!\n" \
				"BMA150 [%d] registered I2C driver!\n", id);
	}

	return 0;
}

//for TNQ/TBP, NJ-YXH ->
static int bma250_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	int id;

    id = i2c_smbus_read_byte_data(client, BMA250_CHIP_ID_REG);
    printk(KERN_ERR "###YXH, bma250_detect, get id %d\n", id);

    if(id != BMA250_CHIP_ID) 
    {
        return -ENODEV;
    }

    return 0;
}
//for TNQ/TBP, NJ-YXH <-
static long bma150_dev_ioctl(struct file *fp, unsigned int cmd, unsigned long args)
{
    struct yas_acc_private_data *data = yas_acc_get_data();
    int rt = 0;
    int result = 0;
    
    switch (cmd) {
	case BMA150_DETECT_CHIP:
	    rt = bma150_detect(data->client, NULL);
	    if(rt < 0)
	    {
	    	result = -EFAULT;
	    }
	    break;
	default:
            result = -EFAULT;
    }
    
    return result;
}

//for TNQ/TBP, NJ-YXH ->
static long bma250_dev_ioctl(struct file *fp, unsigned int cmd, unsigned long args)
{
    struct yas_acc_private_data *data = yas_acc_get_data();
    int rt = 0;
    int result = 0;
    
    printk(KERN_ERR "###YXH, bma250_dev_ioctl\n");
    switch (cmd) {
    case BMA250_DETECT_CHIP:
        printk(KERN_ERR "###YXH, bma250_dev_ioctl, BMA250_DETECT_CHIP\n");
	    rt = bma250_detect(data->client, NULL);
	    if(rt < 0)
	    {
	    	result = -EFAULT;
	    }
        break;
	default:
            result = -EFAULT;
    }
    
    return result;
}
//for TNQ/TBP, NJ-YXH <-
ssize_t bmax50_dev_read(struct file * file, char __user * buffer, 
							size_t size, loff_t * f_pos )
{
	void __user *argp = (void __user *)buffer;
	struct yas_acc_private_data *data = yas_acc_get_data();
	struct yas_acc_driver       *driver;// = data->driver; 
	struct yas_acc_data accdata;
	int rt;
                                        

	if(!data || !data->driver)
	{
		printk(KERN_INFO "[yas_acc] %s :error\n", __func__);
		return -EFAULT;
	}

	driver = data->driver;               


	if (data->driver->measure == NULL) 
	{
   		return -EFAULT;
	}

	rt = data->driver->measure(&accdata);
	if (rt < 0) {
		YLOGE(("measure failed[%d]\n", rt));
	}
	YLOGD(("raw [%d][%d][%d] xyz[%d][%d][%d]\n",
		accdata.raw.v[0], accdata.raw.v[1], accdata.raw.v[2],
		accdata.xyz.v[0], accdata.xyz.v[1], accdata.xyz.v[2]));

	if (copy_to_user(argp, &(accdata.xyz.v[0]), sizeof(struct yas_vector)))
	{
		return -EFAULT;
	}

	return 0;
}


ssize_t bmax50_dev_write(struct file * file, const char __user * buffer, size_t size, loff_t * f_pos )
{
	struct yas_acc_private_data *data = yas_acc_get_data();
	int enable = 2;

	if(copy_from_user(&enable, buffer, sizeof(int)) != 0)
	{
		printk(KERN_ERR "bmax50_dev_write: copy_from_user error!! enable %d\n", enable);
		return -EFAULT;
	}
	printk(KERN_INFO "[bmax50] copy_from_user ok, enable %d\n", enable);
	
	if ((enable == 0) || (enable == 1)) {
		mutex_lock(&data->data_mutex);
		data->accEnable = enable;
#ifdef CONFIG_INPUT_YAS_STEP
		enable = (data->accEnable || data->stepEnable);
#endif 
		mutex_unlock(&data->data_mutex);		

		yas_acc_set_enable(data->driver, enable);
	}else{
		printk(KERN_ERR "bmax50_dev_write: wrong parameter, enable %d\n", enable);
	}
	
	return 0;
}

static struct file_operations gsensor_fileops = {
    .owner      = THIS_MODULE,
    .unlocked_ioctl      = bma150_dev_ioctl,
    .read 	= bmax50_dev_read,	
    .write  = bmax50_dev_write,
};

//for TNQ/TBP, NJ-YXH ->
static struct file_operations gsensor_fileops_bma250 = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = bma250_dev_ioctl,
    .read 	= bmax50_dev_read,	
    .write  = bmax50_dev_write,
};
//for TNQ/TBP, NJ-YXH <-
static struct miscdevice gsensor_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = BMA150_NAME,
    .fops  = &gsensor_fileops,
};

//for TNQ/TBP, NJ-YXH ->
static struct miscdevice gsensor_device_bma250 = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = BMA250_NAME,
    .fops  = &gsensor_fileops_bma250,
};
//for TNQ/TBP, NJ-YXH <-
#endif
/*FIHTDC-DerrickDRLiu-[IRM] FTM test function usage*/




/* ---------------------------------------------------------------------------------------- *
   Module init and exit
 * ---------------------------------------------------------------------------------------- */
static int __init yas_acc_init(void *hdl)
{
    int ret;

    if((ret=ADD_DYN_I2C_DRIVER(hdl, &yas_acc_driver)))
    {
        printk(KERN_ERR "[yas_acc] %s: yas_acc_driver driver init fail\n", __func__);
        return ret;
    }

    if((ret=misc_register(&yas_acc_dev)))
	{
        i2c_del_driver(&yas_acc_driver);
		printk(KERN_ERR"[yas_acc] %s: yas_acc_dev register failed.\n", __func__);
	}

	/* FIHTDC-DerrickDRLiu-[IRM] FTM test function usage { */
#ifdef CONFIG_FIH_7X27A_PROJS
	ret = misc_register(&gsensor_device);
	if (ret < 0) {
		printk(KERN_ERR "[%s] gsensor_device failed[%d]\n", __FUNCTION__, ret);
	}
	else {
		printk(KERN_ERR "[%s] gsensor_device ok\n", __FUNCTION__);
	}	
#endif
	/* } FIHTDC-DerrickDRLiu-[IRM] FTM test function usage */

    return ret;
}
DECLARE_DYN_LOADER(DRIVER_TYPE_GSENSOR, GSENSOR_BMA150_V1, yas_acc_init, LEVEL6);

static int __init yas_acc_init_v2(void *hdl)
{
    int ret;

    if((ret=ADD_DYN_I2C_DRIVER(hdl, &yas_acc_driver)))
    {
        printk(KERN_ERR "[yas_acc] %s: yas_acc_driver driver init fail\n", __func__);
        return ret;
    }

    if((ret=misc_register(&yas_acc_dev)))
	{
        i2c_del_driver(&yas_acc_driver);
		printk(KERN_ERR"[yas_acc] %s: yas_acc_dev register failed.\n", __func__);
	}

	/* FIHTDC-DerrickDRLiu-[IRM] FTM test function usage { */
#ifdef CONFIG_FIH_7X27A_PROJS
	//ret = misc_register(&gsensor_device);
	ret = misc_register(&gsensor_device_bma250); //for TNQ&TBP, NJ-YXH
	if (ret < 0) {
		printk(KERN_ERR "[%s] gsensor_device failed[%d]\n", __FUNCTION__, ret);
	}
	else {
		printk(KERN_ERR "[%s] gsensor_device ok\n", __FUNCTION__);
	}	
#endif
	/* } FIHTDC-DerrickDRLiu-[IRM] FTM test function usage */

    return ret;
}
DECLARE_DYN_LOADER(DRIVER_TYPE_GSENSOR, GSENSOR_BMA250_V1, yas_acc_init_v2, LEVEL6);

static int __init yas_acc_init_v3(void *hdl)
{
    int ret;

    if((ret=ADD_DYN_I2C_DRIVER(hdl, &yas_acc_driver)))
    {
        printk(KERN_ERR "[yas_acc] %s: yas_acc_driver driver init fail\n", __func__);
        return ret;
    }

    if((ret=misc_register(&yas_acc_dev)))
	{
        i2c_del_driver(&yas_acc_driver);
		printk(KERN_ERR"[yas_acc] %s: yas_acc_dev register failed.\n", __func__);
	}

	/* FIHTDC-DerrickDRLiu-[IRM] FTM test function usage { */
#ifdef CONFIG_FIH_7X27A_PROJS
	//ret = misc_register(&gsensor_device);
	ret = misc_register(&gsensor_device_bma250); //for TNQ&TBP, NJ-YXH
	if (ret < 0) {
		printk(KERN_ERR "[%s] gsensor_device failed[%d]\n", __FUNCTION__, ret);
	}
	else {
		printk(KERN_ERR "[%s] gsensor_device ok\n", __FUNCTION__);
	}	
#endif
	/* } FIHTDC-DerrickDRLiu-[IRM] FTM test function usage */

    return ret;
}
DECLARE_DYN_LOADER(DRIVER_TYPE_GSENSOR, GSENSOR_BMA250_V2, yas_acc_init_v3, LEVEL6);

static void __exit yas_acc_exit(void)
{
    i2c_del_driver(&yas_acc_driver);
	misc_deregister(&yas_acc_dev);
}
module_exit(yas_acc_exit);

MODULE_DESCRIPTION("accelerometer kernel driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(YAS_ACC_KERNEL_VERSION);
