/*
 * Asahi Kasei 3-axis Electronic Compass Sennsor Chip Driver 
 *
 * Copyright (C) 2008, Chi Mei Communication Systems, INC. (neillai@tp.cmcs.com.tw)
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 */


#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/sysctl.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/poll.h>

#include <linux/sched.h>
#include <linux/freezer.h>
#include <asm/ioctl.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>

#include <linux/gpio.h>

#include "LPSCM3602.h"

#include <linux/input.h>
#include <linux/miscdevice.h>
#include <mach/msm_smd.h>
#include <linux/delay.h>
#include <mach/vreg.h>
#include <linux/moduleparam.h>

#define __LINUX_KERNEL_DRIVER__
#include "../include/yas.h"

#include <fih/dynloader.h>
#include "../SenseTek_proximity/stk_i2c_ps31xx.h"

int raw_data = -1;
static int cm3623_hw_ok = 1;
static int capella_detect = 0;
/**********************************************************************/
/*B: For FTM test */
/*PROXIMITY*/ 
#define SENSOR_PROXIMITY_ON		0x0B00
#define SENSOR_PROXIMITY_OFF	0x0B01
#define SENSOR_PROXIMITY_READ	0x0B02 //read PS_OUT (1:far state/ 0:near state)

/*LIGHT*/ 
#define SENSOR_LIGHT_ON			0x0B03
#define SENSOR_LIGHT_OFF		0x0B04
#define SENSOR_LIGHT_READ		0x0B05 //read LS raw data                                    
/*E: For FTM test */

#define YAS_CM3623_PS_GPIO_OUT  17
#define CM3623_VREG             "L12"

#define delay_to_jiffies(d) ((d)?msecs_to_jiffies(d):1)
#define actual_delay(d)     (jiffies_to_msecs(delay_to_jiffies(d)))

/**********************************************************************/

static struct i2c_client *ALS1_client = NULL;
static struct i2c_client *ALS2_client = NULL;
static struct i2c_client *PS1_client = NULL;
static struct i2c_client *PS2_client = NULL;
static struct i2c_client *ARA_client = NULL;

/**********************************************************************/

//static const u8 alsps_mode = CM3623_INITIAL_INT_DATA;//interrupt mode
static const u8 alsps_mode = CM3623_INITIAL_POL_DATA;//simple PS Logic Hight/Low mode

/********************************************/
static u8 als_reg_set = 
                    1 << CM3623_ALS_GAIN1_OFFSET /* ALS threshold window setting: +/- 1024 STEP */
                  | 1 << CM3623_ALS_GAIN0_OFFSET
                  | 0 << CM3623_ALS_THD1_OFFSET
                  | 1 << CM3623_ALS_THD0_OFFSET  
                  | 0 << CM3623_ALS_IT1_OFFSET     /* ALS intergration time setting: 100 ms 	*/
                  | 0 << CM3623_ALS_IT0_OFFSET   
                //| 0 << CM3623_ALS_WDM_OFFSET		/* ALS data mode setting: Byte mode 	*/
                  | 1 << CM3623_ALS_WDM_OFFSET     /* ALS data mode setting: Word mode 	*/
                  | 1 << CM3623_ALS_SD_OFFSET;     /* ALS shut down enable */

/********************************************/

static const u8 ps_reg_set = 
                   0 << CM3623_PS_DR1_OFFSET        /* IR LED ON/Off duty ratio setting: 1/160 	*/
                 | 0 << CM3623_PS_DR0_OFFSET
                 | 1 << CM3623_PS_IT1_OFFSET       /* PS intergration time setting: 1.875 T 	*/
                 | 1 << CM3623_PS_IT0_OFFSET
                 | 0 << CM3623_PS_INTALS_OFFSET    /* ALS INT disable	*/
                 //| 1 << CM3623_PS_INTALS_OFFSET    /* ALS INT enable	*/
                 | 0 << CM3623_PS_INTPS_OFFSET     /* PS INT disable 	*/
                 //| 1 << CM3623_PS_INTPS_OFFSET     /* PS INT enable 	*/
                 | 0 << CM3623_PS_RES_OFFSET
                 | 1 << CM3623_PS_SD_OFFSET;       /* PS shut down enable */

/********************************************/

//static const u8 ps_thd  = CM3623_PS_THD_MIN_VAL;  /* PS threshold value for PS interruption activity*/

/********************************************/

/*Promixity*/
static int              yas_ps_irq_flag = 0;
static int              yas_ps_irq      = -1;
static char             yas_ps_enable   = 0;
static unsigned long    yas_ps_delay    = 500; /*default 500ms*/
static char             yas_ps_level    = -1;

/*Light*/
static unsigned long    yas_als_delay   = 500; /*default 500ms*/
static char             yas_als_enable  = 0;
static u8               yas_als_msb     = 0;
static u8               yas_als_lsb     = 0;
static u16              yas_als_data_last = 0;
static struct delayed_work yas_als_work;
static struct delayed_work yas_ps_work;

/*Fast path*/
#ifdef CONFIG_FASTPATH
static int          fih_in_call         = 0;
#endif /*CONFIG_FIH_FASTPATH*/

/*Power control*/
static int          cm3623_power        = 0;

#ifdef CONFIG_FASTPATH
extern int notify_from_proximity(bool bFlag);
#endif /*CONFIG_FIH_FASTPATH*/

#ifdef CONFIG_INPUT_YAS_PROXIMITY
extern void yas_prox_sensor_notify(int x, int y, int z);
#endif

#ifdef CONFIG_INPUT_YAS_LIGHT
extern void yas_light_sensor_notify(int x, int y, int z);
#endif

static int cm3623_initchip(void);

//static int is_cm3623_exist = 1;
//extern int Get_is_cm3623_exist(void);
/**********************************************************************/

static int cm3623_power_control(unsigned int cmd) {

    //printk(KERN_INFO "[cm3623]%s (cmd = %d)\n", __func__, cmd);
    cm3623_power = cmd;
    return 0;
}

#ifdef CONFIG_FASTPATH
static int set_fih_in_call(const char *val, struct kernel_param *kp) {
    char *endp;
    int  l;
    int  rv = 0;

    if (!val)
        return -EINVAL;
    l = simple_strtoul(val, &endp, 0);
    if (endp == val)
        return -EINVAL;

    *((int *)kp->arg) = l;

    if (l == 0) {
        notify_from_proximity(0);
    }

    return rv;
}
module_param_call(fih_in_call, set_fih_in_call, param_get_int, &fih_in_call, 0644);
#endif /*CONFIG_FIH_FASTPATH*/

static int cm3623_write_byte(struct i2c_client *client, unsigned char data) {
    int ret=-1;
    unsigned char command;
    command = data;

    ret = i2c_master_send( client, &command, 1);
    if (ret !=1) {
        printk(KERN_ERR "[cm3623]%s: write %s fail(ret:%d)\n", __func__, client->name, ret);
        return -EIO;
    }
    return 0;   
}

static int cm3623_onoff_ALS(unsigned int cmd) {

    //printk(KERN_INFO "[cm3623]%s (cmd = %d)\n", __func__, cmd);

    if(ALS1_client){
        if (POWER_OFF==cmd) {
            yas_als_enable = 0;  
            /*Write shut down command*/
            cm3623_write_byte(ALS1_client, als_reg_set| 0x01);
           // cancel_delayed_work_sync(&yas_als_work);
            return 1;
        }
        else if (POWER_ON==cmd) {
			printk("TEST_ALS: enable ALS IN\n");
            yas_als_enable = 1;
            printk("TEST_ALS: yas_als_enable=%d\n",yas_als_enable);
            /*Write initial command*/
            cm3623_write_byte(ALS1_client, als_reg_set & 0xfe);
            schedule_delayed_work(&yas_als_work, delay_to_jiffies(yas_als_delay) + 1);
			printk("TEST_ALS: enable ALS OUT\n");
            return 1;
        }
        return -EINVAL;
    }
    else
        return -EINVAL;
}

static int cm3623_onoff_PS(unsigned int cmd) {

   // printk(KERN_INFO "[cm3623]%s (cmd = %d)\n", __func__, cmd);

    if(PS1_client){
        if (POWER_OFF==cmd) {
            if (yas_ps_enable) {
              //  cancel_delayed_work_sync(&yas_ps_work);
                yas_ps_level = -1;   
                yas_ps_enable = 0;   
                /*Write shut down command*/
                cm3623_write_byte(PS1_client, ps_reg_set|0x01);           
		printk("%s: off PS\n", __FUNCTION__);
                if (yas_ps_irq_flag==1) {
                    disable_irq(yas_ps_irq);
                    yas_ps_irq_flag=0;
                }
            }
            return 1;
        }
        else if (POWER_ON==cmd) {
            if (!yas_ps_enable) {
	#if 1
		printk(KERN_ERR "schedule work to read PS raw data");
		schedule_delayed_work(&yas_ps_work, delay_to_jiffies(yas_ps_delay));
	#endif
                yas_ps_enable = 1;
    
                cm3623_write_byte(PS1_client, ps_reg_set&0xfe);
		printk("%s: on PS\n", __FUNCTION__);
                if (yas_ps_irq_flag==0) {
                    enable_irq(yas_ps_irq); 
                    yas_ps_irq_flag=1;              
                }
            }
            return 1;
        }
        return -EINVAL;
    }
    else
        return -EINVAL;
}

/**********************************************************************/
/* PS */
int yas_ps_suspend(void){

    //printk(KERN_INFO "[cm3623]%s\n", __func__);
    if(!cm3623_hw_ok) {
    	printk("cm3623_hw_ok = 0\n");
    	return 0;	
    }

    cm3623_onoff_PS(POWER_OFF);

    /*ALS and PS off, turn off the cm3602 ic*/
    if(!yas_als_enable && !yas_ps_enable){
        cm3623_power_control(POWER_OFF);
    }
    return 0;
}

int yas_ps_resume(void){


	// We use system power, can't contorl it, so don't send init command too
#if 0
    int ret = -1;
    if (!cm3623_power)
    {
        cm3623_power_control(POWER_ON);
        ret = cm3623_initchip();
        if (ret < 0)
            return -EIO;
    }
#endif

    if(!cm3623_hw_ok) {
    	printk("cm3623_hw_ok = 0\n");
    	return 0;	
    }
    
    cm3623_onoff_PS(POWER_ON);
    return 0;
}

int yas_ps_setdelay(int delay){

    //printk(KERN_INFO "[cm3623]%s delay = %d\n", __func__, delay);
    if(!cm3623_hw_ok) {
    	printk("cm3623_hw_ok = 0\n");
    	return 0;	
    }

    yas_ps_delay = actual_delay(delay);

    if (yas_ps_enable) {
        cm3623_onoff_PS(POWER_ON);
    }
    return 0;
}

EXPORT_SYMBOL(yas_ps_suspend);
EXPORT_SYMBOL(yas_ps_resume);
EXPORT_SYMBOL(yas_ps_setdelay);

/**********************************************************************/
/* ALS */
int yas_als_suspend(void){

    //printk(KERN_INFO "[cm3623]%s\n", __func__);
    if(!cm3623_hw_ok) {
    	printk("cm3623_hw_ok = 0\n");
    	return 0;	
    }

    cm3623_onoff_ALS(POWER_OFF);

    /*ALS and PS off, turn off the cm3602 ic*/
    if(!yas_als_enable && !yas_ps_enable){
        cm3623_power_control(POWER_OFF);
    }
    return 0;
}
int yas_als_resume(void){

	// We use system power, can't contorl it, so don't send init command too
#if 0
    int ret = -1;
    if (!cm3623_power)
    {
        cm3623_power_control(POWER_ON);
        ret = cm3623_initchip();
        if (ret < 0)
            return -EIO;
    }
#endif

    if(!cm3623_hw_ok) {
    	printk("cm3623_hw_ok = 0\n");
    	return 0;	
    }
    
    cm3623_onoff_ALS(POWER_ON);
    return 0;
}

int yas_als_setdelay(int delay){

    //printk(KERN_INFO "[cm3623]%s delay = %d\n", __func__, delay);
    if(!cm3623_hw_ok) {
    	printk("cm3623_hw_ok = 0\n");
    	return 0;	
    }

    yas_als_delay = actual_delay(delay);

    if (yas_als_enable) {
        cm3623_onoff_ALS(POWER_ON);
      //  cancel_delayed_work_sync(&yas_als_work);
        schedule_delayed_work(&yas_als_work, delay_to_jiffies(delay) + 1);
    }

    return 0;
}
EXPORT_SYMBOL(yas_als_suspend);
EXPORT_SYMBOL(yas_als_resume);
EXPORT_SYMBOL(yas_als_setdelay);
/**********************************************************************/

#define ALS_MODE_MASK	(0x2)
#define ALS_GAIN_MASK	(0x3<<6)

static unsigned int light_translation(unsigned int lux, unsigned int setting)
{
	if((setting&ALS_MODE_MASK) == 0) // byte mode adjustment
	{
		switch(setting&ALS_GAIN_MASK>>6)
		{
		case 0:
			lux = (lux>>8)&0xff; 
			break;
		case 1:
			lux = (lux>>7)&0xff;
			break;
		case 2:
			lux = (lux>>6)&0xff;
			break;
		case 3:
			lux = (lux>>5)&0xff;
			break;
		}
	}

	return lux;
}

static unsigned int light_translation_irm_pr3(unsigned int lux, unsigned int setting)
{
	if((setting&ALS_MODE_MASK) == 0) // byte mode adjustment
	{
		switch(setting&ALS_GAIN_MASK>>6)
		{
		case 0:
			lux = (lux>>8)&0xff; 
			break;
		case 1:
			lux = (lux>>7)&0xff;
			break;
		case 2:
			lux = (lux>>6)&0xff;
			break;
		case 3:
			lux = (lux>>5)&0xff;
			break;
		}
	}

	if(lux == 1)
		lux = 0;

	return lux;
}

unsigned int (*translate_lux)(unsigned int lux, unsigned int setting) = light_translation;


int yas_als_read_alsdata(void)
{
    unsigned char   data1;
    int             ret = -1;
    int             als_data = 0;

	if(!cm3623_hw_ok) {
		return -1;
	}

    //read ALS data	
    if (yas_als_enable) {
        if(ALS1_client){
            ret = i2c_master_recv(ALS1_client, &data1, 1);
            if (ret ==1) {
                yas_als_msb = data1;            
            }
			else {
				return -2;
			}
        }
		else {
			return -3;
        }
        if(ALS2_client){
            ret = i2c_master_recv(ALS2_client, &data1, 1);
            if (ret ==1) {
                yas_als_lsb = data1;            
            }
			else {
				return -4;
			}
        }
		else {
			return -5;
        }
        als_data = yas_als_msb << 8;
        als_data = als_data | yas_als_lsb;

        als_data = translate_lux(als_data, als_reg_set);
    }
	else {
		return -6;
	}

   // printk(KERN_ERR "[cm3623]%s: als_data = %d\n", __func__, als_data);

    return als_data;
}

static void yas_als_work_func(struct work_struct *work)
{
    int     als_data = 0;

	
    als_data  = yas_als_read_alsdata();
	

#ifdef CONFIG_INPUT_YAS_LIGHT
    if(yas_als_data_last != als_data && als_data >= 0){
        yas_als_data_last = als_data;
        yas_light_sensor_notify(als_data, 0, 0);
    } 
#endif

   if(yas_als_enable)
   	{
        if (yas_als_delay > 0 ) {
            schedule_delayed_work(&yas_als_work, delay_to_jiffies(yas_als_delay));
        }
        else {
            schedule_delayed_work(&yas_als_work, 0);
        }
   	}
}


static void ps_work_func(struct work_struct *work)
{
	u8 rawdata[1];
	int             ret = -1;

	ret = i2c_master_recv(PS1_client, rawdata, 1);
	
	printk(KERN_ERR "PS Raw Data: 0x%x\n", (int)rawdata[0]);
	if(yas_ps_enable)
	{
	      if (yas_ps_delay > 0 ) {
            schedule_delayed_work(&yas_ps_work, delay_to_jiffies(yas_ps_delay));
        }
        else {
            schedule_delayed_work(&yas_ps_work, 0);
        }
	}
}

/*Jordan - add ps raw_data*/
int ps_raw_data(void)
{
	u8 rawdata[1];
	int             ret = -1;

	ret = i2c_master_recv(PS1_client, rawdata, 1);
    if(ret < 0)
        raw_data = ret;
    else {
	    raw_data = (int)rawdata[0];
    }
	printk(KERN_ERR "PS Raw Data: 0x%x\n", (int)rawdata[0]);
	return raw_data;
}
/*-------------------------*/

EXPORT_SYMBOL(ps_raw_data);


static irqreturn_t yas_ps_isr( int irq, void * dev_id) {

    int level;

    gpio_direction_input(YAS_CM3623_PS_GPIO_OUT);
    level = gpio_get_value(YAS_CM3623_PS_GPIO_OUT);

    yas_ps_level = level;
	printk("%s: ps level %d\n", __FUNCTION__, level);

#ifdef CONFIG_FASTPATH
    if (fih_in_call) {
        if (yas_ps_level == 1) {
            notify_from_proximity(0);
        }
        else if (yas_ps_level == 0) {
            notify_from_proximity(1);
        }
    }
#endif /*CONFIG_FIH_FASTPATH*/

#ifdef CONFIG_INPUT_YAS_PROXIMITY
    yas_prox_sensor_notify(yas_ps_level, 0, 0);
#endif

    printk(KERN_ERR "[cm3623]%s: yas_ps_level = %d\n", __func__, yas_ps_level);
   
    return IRQ_HANDLED;
}

/* PS threshold value for PS interruption activity*/
static int cm3623_get_hw_psthd(void)
{
    int hwid = 0; /*Todo : add other hwid support*/

    switch(hwid)
    {
    case 0:
        return CM3623_PS_THD_MIN_VAL; /*tinboost+*/      
    default :
        return CM3623_PS_THD_MIN_VAL; 
    }
}

/* 
initial cm3623 chip, and check cm3623 is available
*/
static int cm3623_initchip(void) {
    int count = 0;
    unsigned char data1;        
    int ret =-1;

     printk(KERN_INFO "[cm3623]%s\n", __func__);

    while (!gpio_get_value(YAS_CM3623_PS_GPIO_OUT)) {
        if(ARA_client){
            ret = i2c_master_recv(ARA_client, &data1, 1);
        }
        if (ret !=1) {
            printk(KERN_ERR "[cm3623]%s: read ARA fail(ret:%d)\n", __func__,ret);
				if (++count >= 5)
				return -EIO;
        }
    }

    //initial   
    if(ALS2_client)
	{
        if(cm3623_write_byte(ALS2_client, alsps_mode) < 0)
			return -EIO;
	
	}
    //init ALS
    if(ALS1_client)
	{
        if(cm3623_write_byte(ALS1_client, als_reg_set) < 0)
			return -EIO;
	}
    //Set the threshold window for PS interruption activity as minimum value  	
    if(PS2_client)
	{
        if(cm3623_write_byte(PS2_client, cm3623_get_hw_psthd()) < 0)
			return -EIO;
	}
    //init ps  	
    if(PS1_client)
	{
        if(cm3623_write_byte(PS1_client, ps_reg_set|0x01) < 0)
			return -EIO;
	}
    return 0;
}

static int ALS1_remove(struct i2c_client *client) {
    //printk(KERN_INFO "[cm3623]%s\n", __func__);
    return 0;
}

#if 0
static int ALS1_suspend(struct i2c_client *client, pm_message_t state) {
    //printk(KERN_INFO "[cm3623]%s\n", __func__);
    return 0;
}
 
static int ALS1_resume(struct i2c_client *client) {
    //printk(KERN_INFO "[cm3623]%s\n", __func__); 
    return 0;
}
#endif

static int ALS1_probe(struct i2c_client *client, const struct i2c_device_id *id) {
#if 1
	struct FIH_lightsensor *data = (struct FIH_lightsensor *)client->dev.platform_data;

	if(data != NULL) {
		als_reg_set = (data->setting)&0xFF;
		printk("initial als setting: %c\n", als_reg_set);
	}
#endif

    printk(KERN_INFO "[cm3623]%s: (name:%s, addr:0x%x)\n", __func__, client->name,client->addr);
    ALS1_client = client;     
    return 0;
}


static const struct i2c_device_id ALS1_id[] = {
    { "cm3623_als1", 0},
    {}
};

static struct i2c_driver ALS1_driver = {
    .probe      = ALS1_probe,
    .remove     = ALS1_remove,
    .id_table   = ALS1_id,
    //.suspend    = ALS1_suspend,
    //.resume     = ALS1_resume,
    .driver = {
        .name   = "cm3623_als1",
    },
};

static int ALS2_probe(struct i2c_client *client, const struct i2c_device_id *id) {
    printk(KERN_INFO "[cm3623]%s: (name:%s, addr:0x%x)\n", __func__, client->name,client->addr);
    ALS2_client = client;   
    return 0;
}

static int ALS2_remove(struct i2c_client *client) {
    //printk(KERN_INFO "[cm3623]%s\n", __func__);
    return 0;
}

#if 0
static int ALS2_suspend(struct i2c_client *client, pm_message_t mesg) {
    //printk(KERN_INFO "[cm3623]%s\n", __func__);
    return 0;
}

static int ALS2_resume(struct i2c_client *client) {
    //printk(KERN_INFO "[cm3623]%s\n", __func__);
    return 0;
}
#endif

static const struct i2c_device_id ALS2_id[] = {
    { "cm3623_als2", 0},
    {}
};

static struct i2c_driver ALS2_driver = {
    .probe      = ALS2_probe,
    .remove     = ALS2_remove,
    //.suspend    = ALS2_suspend,
    //.resume     = ALS2_resume,
    .id_table   = ALS2_id,
    .driver = {
        .name = "cm3623_als2",
    },
};

static int PS1_probe(struct i2c_client *client, const struct i2c_device_id *id) {
    printk(KERN_INFO "[cm3623]%s: (name:%s, addr:0x%x)\n", __func__, client->name,client->addr);
    PS1_client = client;    
    return 0;
}

static int PS1_remove(struct i2c_client *client) {
    //printk(KERN_INFO "[cm3623]%s\n", __func__);
    return 0;
}

#if 0
static int PS1_suspend(struct i2c_client *client, pm_message_t mesg) {
    //printk(KERN_INFO "[cm3623]%s\n", __func__);
    return 0;
}

static int PS1_resume(struct i2c_client *client) {
    //printk(KERN_INFO "[cm3623]%s\n", __func__);
    return 0;
}
#endif

static const struct i2c_device_id PS1_id[] = {
    { "cm3623_ps1", 0},
    {}
};

static struct i2c_driver PS1_driver = {
    .probe      = PS1_probe,
    .remove     = PS1_remove,
    //.suspend    = PS1_suspend,
    //.resume     = PS1_resume,
    .id_table   = PS1_id,
    .driver = {
        .name = "cm3623_ps1",
    },
};

static int PS2_remove(struct i2c_client *client) {
    //printk(KERN_INFO "[cm3623]%s\n", __func__);
#if 0  
    /*do not poweron the chip when boot*/
    cm3623_power_control(POWER_OFF);
#endif

    free_irq(yas_ps_irq, NULL);
	gpio_free(YAS_CM3623_PS_GPIO_OUT);

    return 0;
}

#ifdef CONFIG_PM
static int PS2_suspend(struct device *dev) {
	//printk(KERN_INFO "[cm3623]cm3623: %s \n", __func__);
	enable_irq_wake(MSM_GPIO_TO_INT(YAS_CM3623_PS_GPIO_OUT));
    return 0;
}

static int PS2_resume(struct device *dev) {
	//printk(KERN_INFO "[cm3623]cm3623: %s \n", __func__);
    disable_irq_wake(MSM_GPIO_TO_INT(YAS_CM3623_PS_GPIO_OUT));
    return 0;
}
#endif

static int PS2_probe(struct i2c_client *client,const struct i2c_device_id *id) {

    int level;
    int ret = -EINVAL;

    printk(KERN_INFO "[cm3623]%s: (name:%s, addr:0x%x)\n", __func__, client->name,client->addr);

    PS2_client = client;

    ret = gpio_tlmm_config(GPIO_CFG(YAS_CM3623_PS_GPIO_OUT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 
                           GPIO_CFG_ENABLE);
    if (ret) {
        printk(KERN_ERR "[cm3623]%s: YAS_CM3623_PS_GPIO_OUT config failed : gpio_tlmm_config=%d\n", 
               __func__, ret);     
        printk("cm3623_hw_ok = 0\n");   
        cm3623_hw_ok = 0;           
        return -EIO;
    }

    /* poweron the chip when boot*/
    cm3623_power_control(POWER_ON);

    ret = cm3623_initchip();

    if (ret < 0)
	{
		gpio_free(YAS_CM3623_PS_GPIO_OUT);
	//	is_cm3623_exist = 0;
        printk("cm3623_hw_ok = 0\n");
        cm3623_hw_ok = 0;
        return -EIO;
	}
    /* close alsps sensors after poweron and poweroff the chip */
    yas_ps_suspend();
    yas_als_suspend();

	level = gpio_get_value(YAS_CM3623_PS_GPIO_OUT);
     yas_prox_sensor_notify(level, 0, 0);
    /*For irq read*/
    gpio_request(YAS_CM3623_PS_GPIO_OUT, "YAS PS ISR");

    yas_ps_irq = MSM_GPIO_TO_INT(YAS_CM3623_PS_GPIO_OUT);

    INIT_DELAYED_WORK(&yas_ps_work, ps_work_func);

    ret = request_irq(yas_ps_irq, 
                      yas_ps_isr, 
                      IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, 
                      "cm3623", NULL); 
    if (ret) {
        printk(KERN_ERR "[cm3623]%s: Can't allocate irq %d\n", __func__, ret);
        return ret;
    }

    disable_irq(yas_ps_irq);
    yas_ps_irq_flag=0;
    /* Setup driver interface */
    INIT_DELAYED_WORK(&yas_als_work, yas_als_work_func);


    return 0;
}

static const struct i2c_device_id PS2_id[] = {
    { "cm3623_ps2", 0},
    {}
};

#ifdef CONFIG_PM
static struct dev_pm_ops cm3623_alsps_pm_ops = {
    .suspend        = PS2_suspend,
    .resume         = PS2_resume,
};
#endif

static struct i2c_driver PS2_driver = {
    .probe      = PS2_probe,
    .remove     = PS2_remove,
    .id_table   = PS2_id,
    .driver     = {
        .name = "cm3623_ps2",
#ifdef CONFIG_PM
        .pm = &cm3623_alsps_pm_ops,
#endif
    },
};

static int ARA_probe(struct i2c_client *client, const struct i2c_device_id *id) {
    printk(KERN_INFO "[cm3623]%s: (name:%s, addr:0x%x)\n", __func__, client->name,client->addr);
    ARA_client = client;    
    return 0;
}

static int ARA_remove(struct i2c_client *client) {
    //printk(KERN_INFO "[cm3623]%s\n", __func__);
    return 0;
}

#if 0
static int ARA_suspend(struct i2c_client *client, pm_message_t mesg) {
    //printk(KERN_INFO "[cm3623]%s\n", __func__);
    return 0;
}

static int ARA_resume(struct i2c_client *client) {
    //printk(KERN_INFO "[cm3623]%s\n", __func__);
    return 0;
}
#endif

static const struct i2c_device_id ARA_id[] = {
    { "cm3623_ara", 0},
    {}
};

static struct i2c_driver ARA_driver = {
    .probe      = ARA_probe,
    .remove     = ARA_remove,
    //.suspend    = ARA_suspend,
    //.resume     = ARA_resume,
    .id_table   = ARA_id,
    .driver = {
        .name = "cm3623_ara",
    },
};

/*B: for ftm mode */
static int cm3623_open(struct inode *inode, struct file *file)
{
    printk(KERN_INFO "[cm3623]%s\n", __func__);
	return 0;
}

static ssize_t cm3623_read(struct file *file, char __user *buffer, size_t size, loff_t *f_ops)
{	
    printk(KERN_INFO "[cm3623]%s\n", __func__);
	return 0; 
}

static long cm3623_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    printk(KERN_INFO "[cm3623]%s : cmd=0x%x, arg=0x%x\n", __func__, cmd, (unsigned int)arg);

	switch (cmd)
	{
    case SENSOR_PROXIMITY_OFF:
        yas_ps_suspend();
        break;
    case SENSOR_LIGHT_OFF:
        yas_als_suspend();
        break;
    case SENSOR_PROXIMITY_ON:
        yas_ps_resume();
        break;
    case SENSOR_LIGHT_ON:
        yas_als_resume();
        break;
    case SENSOR_LIGHT_READ://read LS raw data
        return (int)yas_als_read_alsdata();
    case SENSOR_PROXIMITY_READ://read PS_OUT (1:far state/ 0:near state)
        //read from yas_ps_isr();
        return (int)yas_ps_level;
    default:
        break;
    }
	return 0;
}

static int cm3623_release(struct inode *inode, struct file *filp)
{
    printk(KERN_INFO "[cm3623]%s\n", __func__);
	return 0;
}

static const struct file_operations cm3623_dev_fops = {
	.owner = THIS_MODULE,
	.open = cm3623_open,
	.read = cm3623_read,
	.unlocked_ioctl = cm3623_ioctl,
	.release = cm3623_release,
};

static struct miscdevice cm3623_dev =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "cm3602_alsps",             //reserve the file name for FTM can control
    .fops = &cm3623_dev_fops,
};
/*E: for ftm mode */

static int __init cm3623_init(void *hdl) {
    int ret;
	int get_cm3623;
	
	translate_lux = light_translation_irm_pr3;
	
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
		capella_detect = 1;
		printk(KERN_INFO "[cm3623]%s: capella cm3623 detected!\n", __func__);		
	}	
	else
		printk(KERN_ERR "[cm3623]%s:Get_is_cm3623_exist error!\n", __func__);			
	
    if ((ret=ADD_DYN_I2C_DRIVER(hdl, &ALS1_driver))) {
        printk(KERN_ERR "[cm3623]%s: ALS1 driver init fail\n", __func__);
        cm3623_hw_ok = 0;	
		return -EINVAL;
    }
    else if ((ret=ADD_DYN_I2C_DRIVER(hdl, &ALS2_driver))) {
        i2c_del_driver(&ALS2_driver);       
        printk(KERN_ERR "[cm3623]%s: ALS2 driver init fail\n", __func__);
        cm3623_hw_ok = 0;	
		return -EINVAL;
    }
    else if ((ret=ADD_DYN_I2C_DRIVER(hdl, &PS1_driver))) {
        i2c_del_driver(&ALS1_driver);
        i2c_del_driver(&ALS2_driver);   
        printk(KERN_ERR "[cm3623]%s: PS1 driver init fail\n", __func__);
        cm3623_hw_ok = 0;	
		return -EINVAL;
    }
    else if ((ret=ADD_DYN_I2C_DRIVER(hdl, &ARA_driver))) {
        i2c_del_driver(&ALS1_driver);
        i2c_del_driver(&ALS2_driver);
        i2c_del_driver(&PS1_driver);        
        printk(KERN_ERR "[cm3623]%s: ARA driver init fail\n", __func__);
        cm3623_hw_ok = 0;	
		return -EINVAL;
    }
    else if ((ret=ADD_DYN_I2C_DRIVER(hdl, &PS2_driver))) {
        i2c_del_driver(&ALS1_driver);
        i2c_del_driver(&ALS2_driver);
        i2c_del_driver(&PS1_driver);        
        i2c_del_driver(&ARA_driver);        
        printk(KERN_ERR "[cm3623]%s: PS2 driver init fail\n", __func__);
        cm3623_hw_ok = 0;	
		return -EINVAL;
    }
	/*
	if(is_cm3623_exist == 0)
	{
		i2c_del_driver(&ALS1_driver);   
		i2c_del_driver(&ALS2_driver);   
		i2c_del_driver(&PS1_driver);    
		i2c_del_driver(&PS2_driver);    
		i2c_del_driver(&ARA_driver);
		printk(KERN_INFO "[cm3623]%s: cm3623_dev register failed, delete i2c driver\n", __func__);    
		return -EINVAL;
	}
	  */  
    if(0 == ret)
    {
        if (0 != misc_register(&cm3623_dev))
        {
            i2c_del_driver(&ALS1_driver);   
            i2c_del_driver(&ALS2_driver);   
            i2c_del_driver(&PS1_driver);    
            i2c_del_driver(&PS2_driver);    
            i2c_del_driver(&ARA_driver);
            printk(KERN_INFO "[cm3623]%s: cm3623_dev register failed\n", __func__);    
        }
    }
    else
    { 
    	cm3623_hw_ok = 0;	
    	printk("cm3623_hw_ok = 0\n");
    }

    printk(KERN_INFO "[cm3623]%s: cm3623 add I2C driver ok\n", __func__);

    return ret;
}
DECLARE_DYN_LOADER(DRIVER_TYPE_PS, PSALS_CM3623_V1, cm3623_init, LEVEL6);

static int __init cm3623_init_second_source(void *hdl) {
	return cm3623_init(hdl);
}

static int __init cm3623_init_second_source_V1(void *hdl) {
	return cm3623_init(hdl);
}

static int __init cm3623_init_second_source_V2(void *hdl) {
	return cm3623_init(hdl);
}

DECLARE_DYN_LOADER(DRIVER_TYPE_PS, STK3101, cm3623_init_second_source, LEVEL6);
DECLARE_DYN_LOADER(DRIVER_TYPE_PS, STK3101_V1, cm3623_init_second_source_V1, LEVEL6);
DECLARE_DYN_LOADER(DRIVER_TYPE_PS, STK3101_V2, cm3623_init_second_source_V2, LEVEL6);


static int __init cm3623_init_v2(void *hdl) {
    int ret;
	int get_cm3623;
	
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
		capella_detect = 1;
		printk(KERN_INFO "[cm3623]%s: capella cm3623 detected!\n", __func__);		
	}	
	else
		printk(KERN_ERR "[cm3623]%s:Get_is_cm3623_exist error!\n", __func__);	
		
    if ((ret=ADD_DYN_I2C_DRIVER(hdl, &ALS1_driver))) {
        printk(KERN_ERR "[cm3623]%s: ALS1 driver init fail\n", __func__);
    }
    else if ((ret=ADD_DYN_I2C_DRIVER(hdl, &ALS2_driver))) {
        i2c_del_driver(&ALS2_driver);       
        printk(KERN_ERR "[cm3623]%s: ALS2 driver init fail\n", __func__);
    }
    else if ((ret=ADD_DYN_I2C_DRIVER(hdl, &PS1_driver))) {
        i2c_del_driver(&ALS1_driver);
        i2c_del_driver(&ALS2_driver);   
        printk(KERN_ERR "[cm3623]%s: PS1 driver init fail\n", __func__);
    }
    else if ((ret=ADD_DYN_I2C_DRIVER(hdl, &ARA_driver))) {
        i2c_del_driver(&ALS1_driver);
        i2c_del_driver(&ALS2_driver);
        i2c_del_driver(&PS1_driver);        
        printk(KERN_ERR "[cm3623]%s: ARA driver init fail\n", __func__);
    }
    else if ((ret=ADD_DYN_I2C_DRIVER(hdl, &PS2_driver))) {
        i2c_del_driver(&ALS1_driver);
        i2c_del_driver(&ALS2_driver);
        i2c_del_driver(&PS1_driver);        
        i2c_del_driver(&ARA_driver);        
        printk(KERN_ERR "[cm3623]%s: PS2 driver init fail\n", __func__);
    }
    else {
        printk(KERN_INFO "[cm3623]%s: cm3623 add I2C driver ok\n", __func__);
    }
	/*
	if(is_cm3623_exist == 0)
	{
		i2c_del_driver(&ALS1_driver);   
		i2c_del_driver(&ALS2_driver);   
		i2c_del_driver(&PS1_driver);    
		i2c_del_driver(&PS2_driver);    
		i2c_del_driver(&ARA_driver);
		printk(KERN_INFO "[cm3623]%s: cm3623_dev register failed, delete i2c driver\n", __func__);    
		return -EINVAL;
	}	
	*/
    if(0 == ret)
    {
        if (0 != misc_register(&cm3623_dev))
        {
            i2c_del_driver(&ALS1_driver);   
            i2c_del_driver(&ALS2_driver);   
            i2c_del_driver(&PS1_driver);    
            i2c_del_driver(&PS2_driver);    
            i2c_del_driver(&ARA_driver);
            printk(KERN_INFO "[cm3623]%s: cm3623_dev register failed.\n", __func__);    
        }
    }
    else
    { 
    	cm3623_hw_ok = 0;	
    	printk("cm3623_hw_ok = 0\n");
    }

    return ret;
}
DECLARE_DYN_LOADER(DRIVER_TYPE_PS, PSALS_CM3623_V2, cm3623_init_v2, LEVEL6);

static void __exit cm3623_exit(void) {
    printk(KERN_INFO "[cm3623]%s\n", __func__);
    i2c_del_driver(&ALS1_driver);   
    i2c_del_driver(&ALS2_driver);   
    i2c_del_driver(&PS1_driver);    
    i2c_del_driver(&PS2_driver);    
    i2c_del_driver(&ARA_driver);
	misc_deregister(&cm3623_dev);
}


module_exit(cm3623_exit);







static int my_set_byte_value(const char *val, struct kernel_param *kp) 
{
 char *endp;
 int  l;
 int  rv = 0;
 
 if (!val)
  return -EINVAL;
 
 l = simple_strtoul(val, &endp, 0);
 if (endp == val)
  return -EINVAL;
 
 *((u8 *)kp->arg) = l;

 if(yas_als_enable) {
  cm3623_write_byte(ALS1_client, als_reg_set & 0xfe);
 }
  
 return rv;
}

static int my_get_byte_value(char *buffer, struct kernel_param *kp)
{
 char *newPos;
 newPos = buffer;

 newPos += sprintf(newPos, "ALS setting: %d", (int)(*((u8 *)kp->arg)));  
 
 return (newPos - buffer);
}

module_param(cm3623_hw_ok, int, 0644);
module_param(capella_detect, int, 0644);
module_param_call(als_reg_set, my_set_byte_value, my_get_byte_value, &als_reg_set, 0644);








MODULE_DESCRIPTION("cm3623 Sennsor driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dk.HZ.Wu");
