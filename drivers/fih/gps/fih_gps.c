/*
 *     gps.c - For user space to enable/disable EXT_GPS_LNA_EN (81).
 *
 *     Copyright (C) 2009 Albert Fang <albertycfang@fih-foxconn.com>
 *     Copyright (C) 2009 FIH CO., Inc.
 *
 *     This program is free software; you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation; version 2 of the License.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <mach/gpio.h>
#include <asm/uaccess.h>

#define EXT_GPS_LNA_EN          81
#define GPS_ON_CMD              0x00
#define GPS_OFF_CMD             0x01
#define HIGH                    1
#define LOW                     0

#define DEBUG 

#ifdef DEBUG
#define DBG_MSG(msg, ...)       printk("[GPS_MSG] %s : " msg "\n", __FUNCTION__, ## __VA_ARGS__)
#else
#define DBG_MSG(msg, ...)
#endif

#define INF_MSG(msg, ...)       printk("[GPS_INF] %s : " msg "\n", __FUNCTION__, ## __VA_ARGS__)
#define ERR_MSG(msg, ...)       printk("[GPS_ERR] %s : " msg "\n", __FUNCTION__, ## __VA_ARGS__)

static int gps_dev_open(struct inode *inodep, struct file *filep)
{
    DBG_MSG("EXT GPS LNA open!");
    return 0;
}

static int gps_dev_release(struct inode *inodep, struct file *filep)
{
    DBG_MSG("EXT GPS LNA release!");
    return 0;
}

static long gps_dev_ioctl(/*struct inode *inodep, */struct file *filep, unsigned int cmd, unsigned long arg)
{
    DBG_MSG("ioctl command = %d", cmd);

    switch (cmd)
    {
        case GPS_ON_CMD:
            DBG_MSG("GPS_ON_CMD.");
            gpio_set_value(EXT_GPS_LNA_EN, 1);
            break;
        case GPS_OFF_CMD:
            DBG_MSG("GPS_OFF_CMD.");
            gpio_set_value(EXT_GPS_LNA_EN, 0);
            break;
        default:
            ERR_MSG("ioctl error!");
            break;
    }   
    return 0;
}

static struct file_operations gps_dev_fops = 
{
    .open           = gps_dev_open,
    .unlocked_ioctl = gps_dev_ioctl,
    .release        = gps_dev_release,
};

static struct miscdevice gps_cdev = 
{
    MISC_DYNAMIC_MINOR,
    "gps",
    &gps_dev_fops
};

static int __init gps_init(void)
{
    int ret = 0;

    gpio_tlmm_config(GPIO_CFG(EXT_GPS_LNA_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);    
    ret = gpio_request(EXT_GPS_LNA_EN, "GPS_LNA");
    if (ret) INF_MSG("GPS_LNA failed!");
    return misc_register(&gps_cdev);
}

static void __exit gps_exit(void)
{
    misc_deregister(&gps_cdev);
}

module_init(gps_init);
module_exit(gps_exit);

MODULE_AUTHOR("Albert Fang <albertycfang@fih-foxconn.com>");
MODULE_DESCRIPTION("For GPS LNA enable/disable");
MODULE_LICENSE("GPL");
