#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <fih/sn3193.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <fih/leds-pmic8028-proc.h>
#include <fih/dynloader.h>


static enum led_brightness     keyboard_brightness = 0;
static struct work_struct      keyboard_work;
static struct mutex            keyboard_lock;
static spinlock_t              keyboard_value_lock;

static enum led_brightness     headsetloop_brightness = 0; 
static struct work_struct      headsetloop_work;
static struct mutex            headsetloop_lock;
static spinlock_t              headsetloop_value_lock;

static int sn3193_b1 = 1;   //green
static int sn3193_b2 = 2;   //blue
static int sn3193_b3 = 3;   //red

static char old_bank1=0; /*led on/off*/
static char old_bank2=0;
static char old_bank3=0;

static char old_blink_bank1=0; /*led long on/off*/
static char old_blink_bank2=0;
static char old_blink_bank3=0;
 /*add for control brightness(current)*/
static unsigned int g_brightness;
static unsigned int r_brightness;
static unsigned int b_brightness;
struct sn3193_driver_data {
	struct i2c_client *client;
    struct led_classdev led_dev[PMIC8028_ID_LED_MAX];
    int target_time;
    int start_time;
    int blink_once;
    u8 bl_enable;
	u8 en_pin;
    u8 chg_led_ctrl_pin;   //just pullup for IRMPrime 11/30

}*sn3193_dd;

extern int proc_comm_set_led(unsigned id, unsigned level);

static void keyboard_set_work(struct work_struct *work)
{
    mutex_lock(&keyboard_lock);
    proc_comm_set_led(PMIC8028_ID_LED_KEYBOARD, keyboard_brightness);
    mutex_unlock(&keyboard_lock);
}

static void headsetloop_set_work(struct work_struct *work)
{
    mutex_lock(&headsetloop_lock);
    proc_comm_set_led(PMIC8028_ID_HEADSET_LOOP, headsetloop_brightness);
    mutex_unlock(&headsetloop_lock);
}

static int sn3193_write(struct sn3193_driver_data* dd, u8 *cmd,
							int length)
{
    int ret;
    
    struct i2c_msg msgs[] = {
        [0] = {
            .addr   = dd->client->addr,
            .flags  = 0,
            .buf    = (void *)cmd,
            .len    = length
        },
    };
    ret = i2c_transfer(dd->client->adapter, msgs, 1);
    return (ret < 0) ? -1 : 0;    
}

static int sn3193_pins_config(struct sn3193_driver_data* dd)
{
	int rc = 0;
    u8 cmd[2] = {0, 0};

    if (dd->chg_led_ctrl_pin != 255){
        rc = gpio_request(dd->chg_led_ctrl_pin, "CHG LED CTRL");
        if (rc < 0) {
            dev_err(&dd->client->dev, "failed to request %d GPIO\n",
                        dd->chg_led_ctrl_pin);
            return rc;
        }
        gpio_direction_output(dd->chg_led_ctrl_pin, 1); //just pullup for IRMPrime 11/30
    
    }
	rc = gpio_request(dd->en_pin, "sn3193 EN");
	if (rc < 0) {
		dev_err(&dd->client->dev, "failed to request %d GPIO\n", 
					dd->en_pin);
		return rc;
	}
	//gpio_direction_output(dd->en_pin, 1);
	gpio_direction_output(dd->en_pin, 0);
     cmd[0] = 0x2f;                       //ADD by huanjingjing-BSP-11/16 for reset registers
     cmd[1] = 0x20;                                    
     rc = sn3193_write(sn3193_dd, cmd, 2);
	return 0;	
}

static void sn3193_pins_release(struct sn3193_driver_data* dd)
{
	gpio_free(dd->en_pin);
    if (dd->chg_led_ctrl_pin != 255){
        gpio_free(dd->chg_led_ctrl_pin);   //just free in IRMPrime   11/30
    }
}

static void sn3193_chip_enable(struct sn3193_driver_data* dd, bool enable)
{
	gpio_set_value_cansleep(dd->en_pin, enable);
	udelay(950); /* necessary delay time before I2C communication */
}

/*add for control GPIO,reduce power consumption   BSP-huanjingjing-11/3*/
static void sn3193_gpio_enable(struct sn3193_driver_data* dd, bool enable)
{
    gpio_direction_output(dd->en_pin, enable);
}

static int sn3193_chip_config(struct sn3193_driver_data* dd)
{
	u8 cmd[2] = {0x00, 0x00};
	int rc = 0;

	/* Pull EN pin to HI */
	sn3193_chip_enable(dd, true);

	printk("[SN3193 led]:start!!! %s,\n", __func__);
	cmd[0] = 0x00;
	cmd[1] = 0x20;
	rc = sn3193_write(dd, cmd, 2);
	cmd[0] = 0x01;
	cmd[1] = 0x00;
	rc = sn3193_write(dd, cmd, 2);
	cmd[0] = 0x03;

    cmd[1] = 0x04;  /*0x08 5mA--->0x04 10mA*/
	rc = sn3193_write(dd, cmd, 2);


	printk("[SN3193 led]: END!!!%s,\n", __func__);
	return 0;
}
static 
void sn3193_led_set_fade_current(int bank, int bl_current)
{
	u8 cmd[2] = {0, 0};
	int rc = 0;
	printk("[SN3193 led]: START!!!%s,\n", __func__);
    //sn3193_gpio_enable(sn3193_dd, true); //ADD FOR GPIO->1   BSP-huanjingjing-11/3
    if(bank == 1)
    {
        old_blink_bank1 = 0;                      //if sn3193_b1 long on
    }
    else if(bank == 2) 
    {
        old_blink_bank2= 0;                       //if sn3193_b2 long on
    }
    else if(bank == 3)
    {
        old_blink_bank3 = 0;                     //if sn3193_b3 long on
    }



    if((old_blink_bank1) || (old_blink_bank2) || (old_blink_bank3))          //if have any one blink(isn't long on)
    {
        cmd[0] = 0x01;
        cmd[1] = 0x00;                                     //set 0x01-RAM for no breath stop function
        rc = sn3193_write(sn3193_dd, cmd, 2);
    }
    else                                                                    //if all led long on
    {
        cmd[0] = 0x01;
        cmd[1] = 0x20;                                    //set 0x01-RAM for breath stop function
        rc = sn3193_write(sn3193_dd, cmd, 2);
    }
	cmd[0] = 0x02;
	cmd[1] = 0x20;                      //modify by jing-jing.huan10-20 '0x00->0x20'
	rc = sn3193_write(sn3193_dd, cmd, 2);
	cmd[0] = SN3193_BASEBANK + bank;
	cmd[1] = bl_current;
	rc = sn3193_write(sn3193_dd, cmd, 2);
	if (rc < 0)
		dev_err(&sn3193_dd->client->dev, "%s: failed to set [register 0x%02x, data 0x%02x]\n",
					__func__, cmd[0], cmd[1]);
	cmd[0] = 0x1d;
	cmd[1] = 0x07;
	rc = sn3193_write(sn3193_dd, cmd, 2);
    cmd[0] = 0x07;
    cmd[1] = 0x20;
    rc = sn3193_write(sn3193_dd, cmd, 2);

    /*add by hjj 10-28*/
    /* Set Target and Start Times */
    /*set 0x0a~0x0c T0 to 0x00*/
    if(bank == 1)
    {
        cmd[0] = 0x0a;
        cmd[1] = 0x00;
        rc = sn3193_write(sn3193_dd, cmd, 2);
    }
    if(bank == 2)
    {
        cmd[0] = 0x0b;
        cmd[1] = 0x00;
        rc = sn3193_write(sn3193_dd, cmd, 2);
    }
    if(bank == 3)
    {
        cmd[0] = 0x0c;
        cmd[1] = 0x00;
        rc = sn3193_write(sn3193_dd, cmd, 2);
    }
    /*set 0x10~0x12 -->T1=0.52s,T2=0.52s*/
    if(bank == 1)
    {
        cmd[0] = 0x10;
        cmd[1] = 0x46;        
        rc = sn3193_write(sn3193_dd, cmd, 2);
    }
    if(bank == 2)
    {
        cmd[0] = 0x11;
        cmd[1] = 0x46;        
        rc = sn3193_write(sn3193_dd, cmd, 2);
    }
    if(bank == 3)
    {
        cmd[0] = 0x12;
        cmd[1] = 0x46;        
        rc = sn3193_write(sn3193_dd, cmd, 2);
    }
    /*set 0x16~0x18 T3=0.52s,T4=1.04s*/
    if(bank == 1)
    {
        cmd[0] = 0x16;
        cmd[1] = 0x48;     
        rc = sn3193_write(sn3193_dd, cmd, 2);
    }
    if(bank == 2)
    {
        cmd[0] = 0x17;
        cmd[1] = 0x48;    
        rc = sn3193_write(sn3193_dd, cmd, 2);
    }
    if(bank == 3)
    {
        cmd[0] = 0x18;
        cmd[1] = 0x48;    
        rc = sn3193_write(sn3193_dd, cmd, 2);
    } 
    cmd[0] = 0x1c;
    cmd[1] = 0x20;
    rc = sn3193_write(sn3193_dd, cmd, 2);
    /*in order to save EQ, add the control of leds'SW OFF*/
    if(bank == 1)
    {
        old_bank1 = bl_current;                                  //if sn3193_b1 on
    }
    else if(bank == 2)
    {
        old_bank2 = bl_current;                                  //if sn3193_b2 on
    }
    else if(bank == 3)
    {
        old_bank3 = bl_current;                                  //if sn3193_b3 on
    }

    if((old_bank1 == 0) && (old_bank2 == 0) && (old_bank3 == 0))                //if all leds off
    {
        cmd[0] = 0x00;
        cmd[1] = 0x01;                            //set 0x00-RAM to shut the export,and SW cutdown mode
        rc = sn3193_write(sn3193_dd, cmd, 2);
        sn3193_gpio_enable(sn3193_dd, false);      //ADD FOR GPIO->0   BSP-huanjingjing-11/3
    }
    else
    {
        sn3193_gpio_enable(sn3193_dd, true); //ADD FOR GPIO->1   BSP-huanjingjing-11/14
        cmd[0] = 0x00;
        cmd[1] = 0x20;                           //open the export, and standard mode
        rc = sn3193_write(sn3193_dd, cmd, 2);
    }

   printk("[SN3193 led]: END!!!%s,\n", __func__);

}

static
void sn3193_led_set_breath_current(int bank, int bl_current,
										int start_time, int target_time)
{
	u8 cmd[2] = {0, 0};
	int rc = 0;
	printk("[SN3193 led]: START!!!%s,\n", __func__);
    printk("breath:-bank=%d,bl_current=%d,start_time=%d,\n", bank,bl_current,start_time);
    /*if sn3193_bn run for breath ,old_blink_bankn=1 ,which used for "shield the HWchip's Breathing setting conflict"*/
    /*add by hjj 10-28*/
    //sn3193_gpio_enable(sn3193_dd, true);     //ADD FOR GPIO->1   BSP-huanjingjing-11/3
    if(bank == 1)
    {
        if(bl_current)
            old_blink_bank1 = 1;
        else
            old_blink_bank1 = 0;
        
    }
    else if(bank == 2)
    {
        if(bl_current)
            old_blink_bank2 = 1;
        else
            old_blink_bank2 = 0;
        
    }
    else if(bank == 3)
    {
        if(bl_current)
            old_blink_bank3 = 1;
        else
            old_blink_bank3 = 0;
        
    }
    if((old_blink_bank1) || (old_blink_bank2) || (old_blink_bank3))
    {
        cmd[0] = 0x01;
        cmd[1] = 0x00;                                     //if led have any one blink, set 0x01-RAM for no breath stop function
        rc = sn3193_write(sn3193_dd, cmd, 2);
    }
    else
    {
        cmd[0] = 0x01;
        cmd[1] = 0x20;                                    //if led have none blink, set 0x01-RAM for breath stop function
        rc = sn3193_write(sn3193_dd, cmd, 2);
    }

    cmd[0] = 0x01;
    cmd[1] = 0x00;
    rc = sn3193_write(sn3193_dd, cmd, 2);
    
	cmd[0] = 0x02;
	cmd[1] = 0x20;
	rc = sn3193_write(sn3193_dd, cmd, 2);
	cmd[0] = SN3193_BASEBANK + bank;
	cmd[1] = bl_current;
	rc = sn3193_write(sn3193_dd, cmd, 2);
	if (rc < 0)
		dev_err(&sn3193_dd->client->dev, "%s: failed to set [register 0x%02x, data 0x%02x]\n",
					__func__, cmd[0], cmd[1]);
	cmd[0] = 0x1d;
	cmd[1] = 0x07;
	rc = sn3193_write(sn3193_dd, cmd, 2);
	cmd[0] = 0x07;
	cmd[1] = 0x20;
	rc = sn3193_write(sn3193_dd, cmd, 2);

	/* Set Target and Start Times */
    if(bank == 1)
    {
        cmd[0] = 0x0a;
        cmd[1] = 0x00;
        rc = sn3193_write(sn3193_dd, cmd, 2);
    }
	
    if(bank == 2)
    {
        cmd[0] = 0x0b;
        cmd[1] = 0x00;
        rc = sn3193_write(sn3193_dd, cmd, 2);
    }
    if(bank == 3)
    {
        cmd[0] = 0x0c;
        cmd[1] = 0x00;
        rc = sn3193_write(sn3193_dd, cmd, 2);
    }
    if(bank == 1)
    {
        cmd[0] = 0x10;
        cmd[1] = 0x46;        // -->T1=0.52s,T2=0.52s
        rc = sn3193_write(sn3193_dd, cmd, 2);
    }
    if(bank == 2)
    {
        cmd[0] = 0x11;
        cmd[1] = 0x46;        
        rc = sn3193_write(sn3193_dd, cmd, 2);
    }
    if(bank == 3)
    {
        cmd[0] = 0x12;
        cmd[1] = 0x46;        
        rc = sn3193_write(sn3193_dd, cmd, 2);
    }
    if(bank == 1)
    {
        cmd[0] = 0x16;
        cmd[1] = 0x48;    
        rc = sn3193_write(sn3193_dd, cmd, 2);
    }
    if(bank == 2)
    {
        cmd[0] = 0x17;
        cmd[1] = 0x48;     
        rc = sn3193_write(sn3193_dd, cmd, 2);
    }
    if(bank == 3)
    {
        cmd[0] = 0x18;
        cmd[1] = 0x48;     
        rc = sn3193_write(sn3193_dd, cmd, 2);
    }
	cmd[0] = 0x1c;
	cmd[1] = 0x20;
	rc = sn3193_write(sn3193_dd, cmd, 2);
    /*in order to save EQ, add the control of leds'SW OFF*/
    if(bank == 1)
    {
        old_bank1 = bl_current;
    }
    if(bank == 2)
    {
        old_bank2 = bl_current;
    }
    if(bank == 3)
    {
        old_bank3 = bl_current;
    }

    if((old_bank1 == 0) && (old_bank2 == 0) && (old_bank3 == 0))
    {
        cmd[0] = 0x00;
        cmd[1] = 0x01;
        rc = sn3193_write(sn3193_dd, cmd, 2);
    sn3193_gpio_enable(sn3193_dd, false);      //ADD FOR GPIO->0   BSP-huanjingjing-11/3
    }
    else
    {
        sn3193_gpio_enable(sn3193_dd, true);     //ADD FOR GPIO->1   BSP-huanjingjing-11/14
        cmd[0] = 0x00;
        cmd[1] = 0x20;
        rc = sn3193_write(sn3193_dd, cmd, 2);
    }

	printk("[SN3193 led]: END!!!%s,\n", __func__);
	
}

static void sn3193_led_brightness_set(struct led_classdev *led_cdev,
                   enum led_brightness brightness)
{
    int idx = 0;   
    unsigned long flags;

    if (!strcmp(led_cdev->name, "red"))
        idx = SN3193_LED_RED;
    else if (!strcmp(led_cdev->name, "green"))
        idx = SN3193_LED_GREEN;
    else if (!strcmp(led_cdev->name, "blue"))
        idx = SN3193_LED_BLUE;
    else if (!strcmp(led_cdev->name, "button-backlight"))
        idx = PMIC8028_ID_LED_KEYBOARD;
	else if (!strcmp(led_cdev->name, "headset-loop"))
        idx = PMIC8028_ID_HEADSET_LOOP;

    dev_info(led_cdev->dev, "%s: IDX[%d] %s\n", __func__, idx, brightness?"ON":"OFF");		
    
    switch (idx) {
	case SN3193_LED_RED:
		sn3193_led_set_fade_current(sn3193_b3,
									brightness?r_brightness:0);
		break;
	case SN3193_LED_GREEN:
		sn3193_led_set_fade_current(sn3193_b1,
									brightness?g_brightness:0);
		break;
	case SN3193_LED_BLUE:
		sn3193_led_set_fade_current(sn3193_b2,
									brightness?b_brightness:0);
		break;
    case PMIC8028_ID_LED_KEYBOARD:
        {
            spin_lock_irqsave(&keyboard_value_lock, flags);
            keyboard_brightness = brightness;
            schedule_work(&keyboard_work);
            spin_unlock_irqrestore(&keyboard_value_lock, flags);
        }
        break;
    case PMIC8028_ID_HEADSET_LOOP:
        {
            spin_lock_irqsave(&headsetloop_value_lock, flags);
            headsetloop_brightness = brightness;
            schedule_work(&headsetloop_work);
            spin_unlock_irqrestore(&headsetloop_value_lock, flags);
        }
		break;
	}	
}

static ssize_t sn3193_blink_solid_store(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t size)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    int idx = 0;
    int on = 0;
    
    if (!strcmp(led_cdev->name, "red"))
        idx = SN3193_LED_RED;
    else if (!strcmp(led_cdev->name, "green"))
        idx = SN3193_LED_GREEN;
    else if (!strcmp(led_cdev->name, "blue"))
        idx = SN3193_LED_BLUE;
   // else if (!strcmp(led_cdev->name, "button-backlight"))
    //    idx = sn3193_LED_TP;
        
    sscanf(buf, "%d", &on);
    dev_info(led_cdev->dev, "%s: IDX[%d] %s\n", __func__, idx, on?"BLINK":"STOP");
    
    switch (idx) {
	case SN3193_LED_RED:
		sn3193_led_set_breath_current(sn3193_b3,
										on?r_brightness:0,
										sn3193_dd->start_time,
										sn3193_dd->target_time
									);
		break;
	case SN3193_LED_GREEN:
		sn3193_led_set_breath_current(sn3193_b1,
										on?g_brightness:0,
										sn3193_dd->start_time,
										sn3193_dd->target_time
									);
		break;
	case SN3193_LED_BLUE:
		sn3193_led_set_breath_current(sn3193_b2,
										on?b_brightness:0,
										sn3193_dd->start_time,
										sn3193_dd->target_time
									);
		break;
	}
	
	return size;
}
static DEVICE_ATTR(blink, 0644, NULL, sn3193_blink_solid_store);

static int sn3193_get_proper_time(int ms)
{

        if (ms <= 10)
            return SN3193_TIME_0MS;
        if (ms <= 130)
			return SN3193_TIME_130MS;
		if (ms <= 260+128)
			return SN3193_TIME_260MS;
		if (ms <= 520+128)
			return SN3193_TIME_520MS;
		if (ms <= 1040+512)
			return SN3193_TIME_1040MS;
		if (ms <= 2080+512)
			return SN3193_TIME_2080MS;
		if (ms <= 4160+512)
			return SN3193_TIME_4160MS;
		if (ms <= 8320+512)
			return SN3193_TIME_8320MS;
		
		return SN3193_TIME_16440MS;
}

static ssize_t sn3193_target_time_ms_store(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t size)
{
    sscanf(buf, "%d", &sn3193_dd->target_time);
    dev_info(dev, "%s: Target Time[1] %d ms\n", __func__, sn3193_dd->target_time);
    
    sn3193_dd->target_time = sn3193_get_proper_time(sn3193_dd->target_time);
    dev_info(dev, "%s: Target Time[2] %d\n", __func__, sn3193_dd->target_time);

    return size;
}
static DEVICE_ATTR(ledon, 0644, NULL, sn3193_target_time_ms_store);

static ssize_t sn3193_start_time_ms_store(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t size)
{
    sscanf(buf, "%d", &sn3193_dd->start_time);
    dev_info(dev, "%s: Start Time[1] %d ms\n", __func__, sn3193_dd->start_time);
    
    sn3193_dd->start_time = sn3193_get_proper_time(sn3193_dd->start_time);
    dev_info(dev, "%s: Start Time[2] %d\n", __func__, sn3193_dd->start_time);    

    return size;
}
static DEVICE_ATTR(ledoff, 0644, NULL, sn3193_start_time_ms_store);

static ssize_t sn3193_blink_once_store(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t size)
{
    sscanf(buf, "%d", &sn3193_dd->blink_once);
    dev_info(dev, "%s: Blink once\n", __func__);

    return size;
}
static DEVICE_ATTR(blink_once, 0644, NULL, sn3193_blink_once_store);

static int sn3193_led_probe(struct i2c_client *client, 
							const struct i2c_device_id *id)
{
	struct sn3193_platform_data *pd = client->dev.platform_data;
	int ret = -ENODEV;
	int i = 0;

	
	printk("[SN3193 led]: %s, slave addr [ %d ]\n", __func__, client->addr);
	sn3193_dd = kzalloc(sizeof(struct sn3193_driver_data), GFP_KERNEL);
	if (!sn3193_dd) {
		ret = -ENOMEM;
		goto err;
	}
	
	sn3193_dd->start_time	= SN3193_TIME_2080MS;
	sn3193_dd->target_time	= SN3193_TIME_2080MS;
	sn3193_dd->blink_once	= 0;
	sn3193_dd->bl_enable		= 0;
	sn3193_dd->client		= client;
	sn3193_dd->en_pin		= pd->en_pin;
	//sn3193_dd->pwm_pin		= pd->pwm_pin;
	sn3193_dd->chg_led_ctrl_pin = pd->chg_led_ctrl_pin;
	printk("[SN3193 led]: %s, slave addr111111 [ %d ]\n", __func__, sn3193_dd->client->addr);
	if (sn3193_pins_config(sn3193_dd))
		goto err_conf_pins;
		
	if (sn3193_chip_config(sn3193_dd))
		goto err_conf_chip;
    /*ADD by huanjingjing 10-11,initialize the RGB lights'value*/
    sn3193_led_set_fade_current(SN3193_B1, 0);
	sn3193_led_set_fade_current(SN3193_B2, 0);
	sn3193_led_set_fade_current(SN3193_B3, 0);
	proc_comm_set_led(PMIC8028_ID_LED_KEYBOARD, 0);
	proc_comm_set_led(PMIC8028_ID_HEADSET_LOOP, 0);  
	
	for (i = 0; i < PMIC8028_ID_LED_MAX; i++) {
		switch (i) {
		case SN3193_LED_RED:
			sn3193_dd->led_dev[i].name = "red";
			break;
		case SN3193_LED_GREEN:
			sn3193_dd->led_dev[i].name = "green";
			break;
		case SN3193_LED_BLUE:
			sn3193_dd->led_dev[i].name = "blue";
			break;
		case PMIC8028_ID_LED_KEYBOARD:
			sn3193_dd->led_dev[i].name = "button-backlight";
			break;
		case PMIC8028_ID_HEADSET_LOOP:
			sn3193_dd->led_dev[i].name = "headset-loop";
			break;
        default:
            break;
		}
      if ((i != SN3193_LED_RED) && (i != SN3193_LED_GREEN) && (i != SN3193_LED_BLUE) && (i != PMIC8028_ID_LED_KEYBOARD) && (i != PMIC8028_ID_HEADSET_LOOP))
            continue;		
		sn3193_dd->led_dev[i].brightness_set = sn3193_led_brightness_set;

        ret = led_classdev_register(&sn3193_dd->client->dev, &sn3193_dd->led_dev[i]);
        if (ret) {
            dev_err(&sn3193_dd->client->dev,
                        "%s: led_classdev_register failed\n", __func__);
            goto err_register_led_dev;
        }
    
        dev_info(&sn3193_dd->client->dev,
                "%s: LED name(%s) was registered\n",
                __func__, sn3193_dd->led_dev[i].name);
	}
	
	for (i = 0; i < PMIC8028_ID_LED_MAX; i++) {
	   if ((i != SN3193_LED_RED) && (i != SN3193_LED_GREEN) && (i != SN3193_LED_BLUE) && (i != PMIC8028_ID_LED_KEYBOARD) && (i != PMIC8028_ID_HEADSET_LOOP))
            continue;
		ret = device_create_file(sn3193_dd->led_dev[i].dev,
									&dev_attr_blink);
		if (ret) {
			dev_err(&sn3193_dd->client->dev,
						"%s: device_create_file failed\n", __func__);
			goto err_register_blink_attr;
		}
    }
    
    ret = device_create_file(&sn3193_dd->client->dev, &dev_attr_ledon);
    if (ret) {
        dev_err(&sn3193_dd->client->dev,
               "%s: create dev_attr_ledon failed\n", __func__);
        goto err_register_blink_attr;
    }
    
    ret = device_create_file(&sn3193_dd->client->dev, &dev_attr_ledoff);
    if (ret) {
        dev_err(&sn3193_dd->client->dev,
               "%s: create dev_attr_ledoff failed\n", __func__);
        goto err_out_attr_ledoff;
    }
    
    ret = device_create_file(&sn3193_dd->client->dev, &dev_attr_blink_once);
    if (ret) {
        dev_err(&sn3193_dd->client->dev,
               "%s: create dev_attr_blink_once failed\n", __func__);
        goto err_out_attr_blink_once;
    }

    mutex_init(&keyboard_lock);
    spin_lock_init(&keyboard_value_lock);
    INIT_WORK(&keyboard_work, keyboard_set_work);

    mutex_init(&headsetloop_lock);
    spin_lock_init(&headsetloop_value_lock);
    INIT_WORK(&headsetloop_work, headsetloop_set_work);

	return 0;
	
err_out_attr_blink_once:
    device_remove_file(&sn3193_dd->client->dev, &dev_attr_ledoff);
err_out_attr_ledoff:
    device_remove_file(&sn3193_dd->client->dev, &dev_attr_ledon);
err_register_blink_attr:
	for (i = 0; i < SN3193_LED_MAX; i++)
		device_remove_file(sn3193_dd->led_dev[i].dev, &dev_attr_blink);
err_register_led_dev:
	for (i = 0; i < SN3193_LED_MAX; i++)
		led_classdev_unregister(&sn3193_dd->led_dev[i]);	
err_conf_chip:
	sn3193_chip_enable(sn3193_dd, false);
err_conf_pins:
	sn3193_pins_release(sn3193_dd);
	kfree(sn3193_dd);
err:
	return ret;
}

/*sn3193_led_probe_v2 is add for IRMPrime,to except register"button-backlight",huanjingjing-12-10*/
static int sn3193_led_probe_v2(struct i2c_client *client, 
							const struct i2c_device_id *id)
{
	struct sn3193_platform_data *pd = client->dev.platform_data;
	int ret = -ENODEV;
	int i = 0;

	
	printk("[SN3193 led]: %s, slave addr [ %d ]\n", __func__, client->addr);
	sn3193_dd = kzalloc(sizeof(struct sn3193_driver_data), GFP_KERNEL);
	if (!sn3193_dd) {
		ret = -ENOMEM;
		goto err;
	}
	
	sn3193_dd->start_time	= SN3193_TIME_2080MS;
	sn3193_dd->target_time	= SN3193_TIME_2080MS;
	sn3193_dd->blink_once	= 0;
	sn3193_dd->bl_enable		= 0;
	sn3193_dd->client		= client;
	sn3193_dd->en_pin		= pd->en_pin;
	//sn3193_dd->pwm_pin		= pd->pwm_pin;
	sn3193_dd->chg_led_ctrl_pin = pd->chg_led_ctrl_pin;
	printk("[SN3193 led]: %s, slave addr111111 [ %d ]\n", __func__, sn3193_dd->client->addr);
	if (sn3193_pins_config(sn3193_dd))
		goto err_conf_pins;
		
	if (sn3193_chip_config(sn3193_dd))
		goto err_conf_chip;
    /*ADD by huanjingjing 10-11,initialize the RGB lights'value*/
    sn3193_led_set_fade_current(SN3193_B1, 0);
	sn3193_led_set_fade_current(SN3193_B2, 0);
	sn3193_led_set_fade_current(SN3193_B3, 0);
	//proc_comm_set_led(PMIC8028_ID_LED_KEYBOARD, 0);
	proc_comm_set_led(PMIC8028_ID_HEADSET_LOOP, 0);  
	
	for (i = 0; i < PMIC8028_ID_LED_MAX; i++) {
		switch (i) {
		case SN3193_LED_RED:
			sn3193_dd->led_dev[i].name = "red";
			break;
		case SN3193_LED_GREEN:
			sn3193_dd->led_dev[i].name = "green";
			break;
		case SN3193_LED_BLUE:
			sn3193_dd->led_dev[i].name = "blue";
			break;
		//case PMIC8028_ID_LED_KEYBOARD:
		//	sn3193_dd->led_dev[i].name = "button-backlight";
		//	break;
		case PMIC8028_ID_HEADSET_LOOP:
			sn3193_dd->led_dev[i].name = "headset-loop";
			break;
        default:
            break;
		}
      if ((i != SN3193_LED_RED) && (i != SN3193_LED_GREEN) && (i != SN3193_LED_BLUE) && (i != PMIC8028_ID_HEADSET_LOOP))
            continue;		
		sn3193_dd->led_dev[i].brightness_set = sn3193_led_brightness_set;

        ret = led_classdev_register(&sn3193_dd->client->dev, &sn3193_dd->led_dev[i]);
        if (ret) {
            dev_err(&sn3193_dd->client->dev,
                        "%s: led_classdev_register failed\n", __func__);
            goto err_register_led_dev;
        }
    
        dev_info(&sn3193_dd->client->dev,
                "%s: LED name(%s) was registered\n",
                __func__, sn3193_dd->led_dev[i].name);
	}
	
	for (i = 0; i < PMIC8028_ID_LED_MAX; i++) {
	   if ((i != SN3193_LED_RED) && (i != SN3193_LED_GREEN) && (i != SN3193_LED_BLUE) && (i != PMIC8028_ID_HEADSET_LOOP))
            continue;
		ret = device_create_file(sn3193_dd->led_dev[i].dev,
									&dev_attr_blink);
		if (ret) {
			dev_err(&sn3193_dd->client->dev,
						"%s: device_create_file failed\n", __func__);
			goto err_register_blink_attr;
		}
    }
    
    ret = device_create_file(&sn3193_dd->client->dev, &dev_attr_ledon);
    if (ret) {
        dev_err(&sn3193_dd->client->dev,
               "%s: create dev_attr_ledon failed\n", __func__);
        goto err_register_blink_attr;
    }
    
    ret = device_create_file(&sn3193_dd->client->dev, &dev_attr_ledoff);
    if (ret) {
        dev_err(&sn3193_dd->client->dev,
               "%s: create dev_attr_ledoff failed\n", __func__);
        goto err_out_attr_ledoff;
    }
    
    ret = device_create_file(&sn3193_dd->client->dev, &dev_attr_blink_once);
    if (ret) {
        dev_err(&sn3193_dd->client->dev,
               "%s: create dev_attr_blink_once failed\n", __func__);
        goto err_out_attr_blink_once;
    }

    //mutex_init(&keyboard_lock);
    //spin_lock_init(&keyboard_value_lock);
    //INIT_WORK(&keyboard_work, keyboard_set_work);

    mutex_init(&headsetloop_lock);
    spin_lock_init(&headsetloop_value_lock);
    INIT_WORK(&headsetloop_work, headsetloop_set_work);

	return 0;
	
err_out_attr_blink_once:
    device_remove_file(&sn3193_dd->client->dev, &dev_attr_ledoff);
err_out_attr_ledoff:
    device_remove_file(&sn3193_dd->client->dev, &dev_attr_ledon);
err_register_blink_attr:
	for (i = 0; i < SN3193_LED_MAX; i++)
		device_remove_file(sn3193_dd->led_dev[i].dev, &dev_attr_blink);
err_register_led_dev:
	for (i = 0; i < SN3193_LED_MAX; i++)
		led_classdev_unregister(&sn3193_dd->led_dev[i]);	
err_conf_chip:
	sn3193_chip_enable(sn3193_dd, false);
err_conf_pins:
	sn3193_pins_release(sn3193_dd);
	kfree(sn3193_dd);
err:
	return ret;
}

static int sn3193_led_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id sn3193_idtable[] = {
	{"sn3193-led", 0},
	{}
};


static struct i2c_driver sn3193_led_driver = {
	.probe		= sn3193_led_probe,
	.remove		= sn3193_led_remove,
	.id_table	= sn3193_idtable,
	.driver		= {
		.name	= "sn3193-led",
		.owner	= THIS_MODULE,
	},
};
static struct i2c_driver sn3193_led_driver_v2 = {
	.probe		= sn3193_led_probe_v2,
	.remove		= sn3193_led_remove,
	.id_table	= sn3193_idtable,
	.driver		= {
		.name	= "sn3193-led",
		.owner	= THIS_MODULE,
	},
};
#if 0
static int __init sn3193_led_init(void)
{
	printk( KERN_INFO "[SN3193 led]: %s\n", __func__ );
	return i2c_add_driver(&sn3193_led_driver);
}
module_init(sn3193_led_init);
#else
static int __init sn3193_led_V1_init(void *hdl)
{
    g_brightness = 0x05;   //0.2mA
    r_brightness = 0x05;
    b_brightness = 0x05;
	return ADD_DYN_I2C_DRIVER(hdl, &sn3193_led_driver);
}
DECLARE_DYN_LOADER(DRIVER_TYPE_LED, SN3193_LED_V1, sn3193_led_V1_init, LEVEL6);
static int __init sn3193_led_V2_init(void *hdl)
{
    g_brightness = 0x05;   //0.2mA
    r_brightness = 0x05;
    b_brightness = 0x05;
	return ADD_DYN_I2C_DRIVER(hdl, &sn3193_led_driver);
}
DECLARE_DYN_LOADER(DRIVER_TYPE_LED, SN3193_LED_V2, sn3193_led_V2_init, LEVEL6);

static int __init sn3193_led_V3_init(void *hdl)
{
    sn3193_b1 = 2;
    sn3193_b2 = 3;
    sn3193_b3 = 1;
    g_brightness = 0x80;  //5mA    ITV    2012/2/2
    r_brightness = 0xff;  //10mA
    b_brightness = 0xff;  //10mA
	return ADD_DYN_I2C_DRIVER(hdl, &sn3193_led_driver);
}
DECLARE_DYN_LOADER(DRIVER_TYPE_LED, SN3193_LED_V3, sn3193_led_V3_init, LEVEL6);
//IRMPrime       //11/30
static int __init sn3193_led_V4_init(void *hdl)
{
    sn3193_b1 = 2;
    sn3193_b2 = 3;
    sn3193_b3 = 1;
    g_brightness = 0x20;   //0.2mA->1.25mA
    r_brightness = 0x20;
    b_brightness = 0x20;
	return ADD_DYN_I2C_DRIVER(hdl, &sn3193_led_driver_v2);
}
DECLARE_DYN_LOADER(DRIVER_TYPE_LED, SN3193_LED_V4, sn3193_led_V4_init, LEVEL6);       //11/30
#endif
static void __exit sn3193_led_exit(void)
{
	i2c_del_driver(&sn3193_led_driver);
    i2c_del_driver(&sn3193_led_driver_v2);
}
module_exit(sn3193_led_exit);

MODULE_DESCRIPTION("sn3193 LEDs driver");
