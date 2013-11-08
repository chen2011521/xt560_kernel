#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/time.h>
#include <linux/switch.h>
#include <mach/msm_iomap.h>
#include <mach/msm_smd.h>
#include <linux/wakelock.h>
#include <linux/completion.h>
#include <linux/sched.h>

#include <fih/dynloader.h>
#include <linux/leds.h>

#define EKTF4751SK_name "ektf4751sk"

//////TKP define+++++//////
#define TKP_PHYSLEN 25
#define EKTF4751SK_NORMAL_MODE 0
#define EKTF4751SK_SLEEP_MODE 1
#define EKTF4751SK_HIBERNATE_MODE 2
#define TKP_IN_MXKYEVTS 256
#define TKP_IN_KEYPRESS 1
#define TKP_IN_KEYRELEASE 0
#define NUM_KPC_DATA_BYTE_REG 5
#define TKP_BUTTONS 8
#define INT_PIN 94
#define HALL_INT 122
static u8 g_last_btn_status = 0x00;
//////TKP define----//////
//#define DEBUG
#define SOFTKEY_BACKLIGHT_ON 1
#define SOFTKEY_BACKLIGHT_OFF 0


//static int g_device_PRODUCT_ID, g_device_PHASE_ID ;
/*
 * The EKTF4751SK_record structure consolates all the data/variables
 * specific to managing the single instance of the keyboard.
 */
struct EKTF4751SK_record {
	struct	i2c_client *mykeyboard;
	struct	input_dev *i2ckbd_idev;
	int	product_info;
	bool key_first_status[TKP_BUTTONS];
	char	physinfo[TKP_PHYSLEN];
	int	irqpin;

	uint8_t kybd_exists;
	uint8_t kybd_connected;
	uint8_t kcnt;
	struct	delayed_work kb_cmdq;
	struct	work_struct qkybd_irqwork;
	struct 	led_classdev led_dev;    
	
	

	u32 (*xlf)(struct EKTF4751SK_record *kbdrec, s32 code,
		   s32 *kstate);
	spinlock_t EKTF4751SK_lock;
	struct early_suspend early_suspend;
};

struct input_dev *EKTF4751SK_kpdev		= NULL;
struct EKTF4751SK_record *EKTF4751SK_rd	= NULL;


static struct i2c_client *g_i2c_client;
#define KBDIRQNO(kbdrec)  (MSM_GPIO_TO_INT(kbdrec->irqpin))
static uint16_t FIHKeypad_set[TKP_IN_MXKYEVTS] =
{
	KEY_HOME,
	KEY_SEARCH,
	KEY_MENU,
	KEY_BACK,
	KEY_F5,
	KEY_F6,
	KEY_F7,
	KEY_F8
};



struct input_dev *EKTF4751SK_msm_keypad_get_input_dev(void)
{	
	return EKTF4751SK_kpdev;
}
EXPORT_SYMBOL(EKTF4751SK_msm_keypad_get_input_dev);



static irqreturn_t EKTF4751SK_irqhandler(int irq, void *dev_id)
{
	struct EKTF4751SK_record *kbdrec = dev_id;

	printk( "EKTF4751SK_irqhandler+++++++++++\r\n" );
	disable_irq_nosync(irq);
	if (kbdrec->kybd_connected) {
		if (KBDIRQNO(kbdrec) == irq) {
			schedule_work(&kbdrec->qkybd_irqwork);   //wake up
		} 

	}
	
	printk( "EKTF4751SK_irqhandler-----------\r\n" );
	
	return IRQ_HANDLED;
}

static int EKTF4751SK_irqsetup(struct EKTF4751SK_record *kbdrec)
{
		
	int rc ;
	gpio_tlmm_config(GPIO_CFG(INT_PIN, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);//interrupt pin
	gpio_request(INT_PIN,EKTF4751SK_name);

	rc = request_irq(KBDIRQNO(kbdrec), &EKTF4751SK_irqhandler,
			     (IRQF_TRIGGER_LOW | IRQF_DISABLED),
			     EKTF4751SK_name, kbdrec);				     
	if (rc < 0) {
		printk(KERN_ERR
		       "Could not register for  %s interrupt "
		       "(rc = %d)\n", EKTF4751SK_name, rc);
		rc = -EIO;
	}
	return rc;
}


static int EKTF4751SK_release_gpio(struct EKTF4751SK_record *kbrec)
{
	int kbd_irqpin		= kbrec->irqpin;

	gpio_free(kbd_irqpin);

	return 0;
}



/* read keyboard via i2c address + register offset, return # bytes read */
static int EKTF4751SK_i2c_read(struct i2c_client *kbd, uint8_t regaddr,
		     uint8_t *buf, uint32_t rdlen)
{
	u8 ldat = regaddr;
	struct i2c_msg msgs[] = {
		[0] = {
			.addr	= kbd->addr,
			.flags	= 0,
			.buf	= (void *)&ldat,
			.len	= 1
		},
		[1] = {
			.addr	= kbd->addr,
			.flags	= I2C_M_RD,
			.buf	= buf,
			.len	= rdlen
		}
	};
	//printk("******kbd->addr=%x, regaddr=0x%x \n",kbd->addr,regaddr);
	return (i2c_transfer(kbd->adapter, msgs, 2) < 0) ? -1 : 0;
}

/* Write the specified data to the  control reg */

static int EKTF4751SK_i2c_write(struct i2c_client *kbd, uint8_t regaddr,
		     uint8_t *buf, uint32_t dlen)
{
	s16 i;
	u8 rpd[4];
	struct i2c_msg msgs[] = {
		[0] = {
			.addr	= kbd->addr,
			.flags	= 0,
			.buf	= (void *)rpd,
			.len	= (dlen + 1)
		}
	};

	rpd[0] = regaddr;
	for (i = 0; i < dlen; i++)
		rpd[i+1] = buf[i];

	return (i2c_transfer(kbd->adapter, msgs, 1) < 0) ? -1 : 0;
}


/*
 * handler function called via work queue
 *
 * This function reads from the keyboard via I2C until the
 * keyboard's output queue of scan codes is empty.
 *
 * If running on scan set 1, we support the "RAW" keyboard mode
 * directly. The RAW mode is required if using X11. Currently,
 * RAW mode is not supported for scan set 2.
 */
static void EKTF4751SK_fetchkeys(struct work_struct *work)
{
	struct EKTF4751SK_record *kbdrec	= container_of(work, struct EKTF4751SK_record, qkybd_irqwork);
	struct i2c_client *kbdcl	= kbdrec->mykeyboard;
	struct input_dev *idev		= kbdrec->i2ckbd_idev;
	s32 kevent			= TKP_IN_KEYRELEASE;
	s16 rc				= -EIO;
	u8 rdat;
	u8 i;
	u8 change_bit;
	u8 gpio_value;

	printk( "EKTF4751SK_fetchkeys++++++++++++++\n");
	//Make sure that registering to input subsystem was finished.
	if (!kbdrec->kybd_connected) {
		printk("EKTF4751SK not connected\n");
		return;
	}

	//read button status:
	//0x00: button status register
	rc = EKTF4751SK_i2c_read(kbdcl, 0x00, &rdat, sizeof(rdat));
#if 1
	if(!rc)
		printk("====EKTF4751 reg0x00=0x%x====\n",rdat);
	else
		printk("i2c fail, rc=%d\n",rc);
#endif
	change_bit = rdat^g_last_btn_status;
	if(!change_bit)
		printk("no key is pressed/released \n");
	else
	{
		kbdrec->kcnt = 0;
		for(i=0; i<TKP_BUTTONS; ++i)
		{

			if((change_bit>>i)&0x01)
			{
				//bit i is changed
				kevent = (rdat>>i)&0x01;
				if(kevent==TKP_IN_KEYPRESS)
				{
				    if(!kbdrec->key_first_status[i])
				    {   
					printk( "button[%d] is pressed\n",i);
					//for ftm
					gpio_value=gpio_get_value(INT_PIN);
					input_report_key(idev, FIHKeypad_set[i], TKP_IN_KEYPRESS);
					kbdrec->key_first_status[i] = 1;
				    }
				}
				else
				{
				    if(kbdrec->key_first_status[i])
				    {   
					printk( "button[%d] is released\n",i);
					//for ftm
					gpio_value=gpio_get_value(INT_PIN);
					input_report_key(idev, FIHKeypad_set[i], TKP_IN_KEYRELEASE);
					kbdrec->key_first_status[i] = 0;
				    }
				}
				//input_report_key(idev, FIHKeypad_set[i], kevent);
				kbdrec->kcnt++;
				//break;
			}

		}
	}
	g_last_btn_status = rdat;
	if (0 < kbdrec->kcnt) 
	{
		input_sync(idev); //input report finished
		kbdrec->kcnt	= 0;
	} else if (0 == kbdrec->kcnt) {
		printk( "0 keys processed after interrupt\n");
	}
	enable_irq(KBDIRQNO(EKTF4751SK_rd));
}



static void EKTF4751SK_shutdown(struct EKTF4751SK_record *rd)
{
	if (rd->kybd_connected) {
		dev_info(&rd->mykeyboard->dev, "disconnecting keyboard\n");
		rd->kybd_connected = 0;
		free_irq(KBDIRQNO(rd), rd);

		flush_work(&rd->qkybd_irqwork);

	}
}

static int EKTF4751SK_eventcb(struct input_dev *dev, unsigned int type,
			    unsigned int code, int value)
{
	int rc = -EPERM;
#if 0
	struct EKTF4751SK_record *kbdrec = input_get_drvdata(dev);
	struct device *kbdev = &kbdrec->mykeyboard->dev;

	switch (type) {
	case EV_MSC:
		/* raw events are forwarded to keyboard handler */
		break;
	case EV_REP:
		break;
	case EV_LED:
		break;
	default:
		dev_warn(kbdev, "rcv'd unrecognized command (%d)\n", type);
	}
#endif

	return rc;
}

static int EKTF4751SK_opencb(struct input_dev *dev)
{
	int rc=0;
	struct EKTF4751SK_record *kbdrec	= input_get_drvdata(dev);
	//struct i2c_client *kbd		= kbdrec->mykeyboard;

	printk( "ENTRY: input_dev open callback\n");

	//rc = EKTF4751SK_getkbinfo(kbd, &kbdrec->product_info);	
	if (!rc) {
	//	dev->id.version = kbdrec->product_info & 0xFF;
	//	dev->id.product = (kbdrec->product_info & ~0xFF);
		kbdrec->kybd_connected = 1;
		enable_irq(KBDIRQNO(EKTF4751SK_rd));
	} else
		rc = -EIO;
	return rc;
}

static void EKTF4751SK_closecb(struct input_dev *idev)
{
	struct EKTF4751SK_record *kbdrec	= input_get_drvdata(idev);
	struct device *dev		= &kbdrec->mykeyboard->dev;

	dev_dbg(dev, "ENTRY: close callback\n");
	EKTF4751SK_shutdown(kbdrec);
}

static struct input_dev *create_inputdev_instance(struct EKTF4751SK_record *kbdrec)
{
	struct device *dev	= &kbdrec->mykeyboard->dev;
	struct input_dev *idev	= 0;
	s16 kidx;

	idev = input_allocate_device();
	if (idev != NULL) {
		idev->name		= "qt602240key";                        //EKTF4751SK_name
		idev->phys		= kbdrec->physinfo;
		idev->id.bustype	= BUS_I2C;
		//idev->id.vendor 	= QCVENDOR_ID;
		idev->id.product	= 1;
		idev->id.version	= 1;
		idev->open		= EKTF4751SK_opencb;
		idev->close		= EKTF4751SK_closecb;
		idev->event		= EKTF4751SK_eventcb;
		//idev->keycode		= FIHKeypad_set;
		idev->keycodesize	= sizeof(uint8_t);
		idev->keycodemax	= TKP_IN_MXKYEVTS;
		idev->evbit[0]		= BIT(EV_KEY) | BIT(EV_SW);
		




	   	idev->keycode		= FIHKeypad_set ;

	   	for (kidx = 0; kidx < TKP_IN_MXKYEVTS; kidx++)
			__set_bit(FIHKeypad_set[kidx], idev->keybit);
		input_set_abs_params(idev, ABS_X, 0, 0, 0, 0);

		input_set_drvdata(idev, kbdrec);
		
		EKTF4751SK_kpdev = idev;
	} else {
		dev_err(dev,
			"Failed to allocate input device for %s\n",
			EKTF4751SK_name);
	}
	
	return idev;
}

static void EKTF4751SK_connect2inputsys(struct work_struct *work)
{
	struct EKTF4751SK_record *kbdrec =
		container_of(work, struct EKTF4751SK_record, kb_cmdq.work);
	//// struct device *dev = &kbdrec->mykeyboard->dev;

	printk( "EKTF4751SK_connect2inputsys\r\n" );
	kbdrec->i2ckbd_idev = create_inputdev_instance(kbdrec);
	if (kbdrec->i2ckbd_idev) {
		if (input_register_device(kbdrec->i2ckbd_idev) != 0) {
			printk( "Failed to register with"
				" input system\n");
			input_free_device(kbdrec->i2ckbd_idev);
		}
	}

	input_sync(kbdrec->i2ckbd_idev);
//	enable_irq(KBDIRQNO(EKTF4751SK_rd));
}

/* utility function used by probe */
static int testfor_keybd(struct i2c_client *new_kbd)
{
	int rc = 0;
	struct EKTF4751SK_record *rd = i2c_get_clientdata(new_kbd);
	printk( "[jjj] testfor_keybd\r\n" ); 
	if (!rd->kybd_exists) {

		if (1) {		
			printk( 
				 "Detected %s, attempting to initialize "
				 "keyboard\n", EKTF4751SK_name);
			
			snprintf(rd->physinfo, TKP_PHYSLEN,
				 "%s/%s/event0",
				 dev_name(&new_kbd->adapter->dev),
				 dev_name(&new_kbd->dev));
			
			printk("rd->physinfo: %s \n",rd->physinfo) ;
			/*snprintf(rd->physinfo, TKP_PHYSLEN,
				 "%s/%s/event0",
				 new_kbd->adapter->dev.bus_id,
				 new_kbd->dev.bus_id);*/
			rd->kybd_exists = 1;
			INIT_DELAYED_WORK(&rd->kb_cmdq,
					  EKTF4751SK_connect2inputsys);
			schedule_delayed_work(&rd->kb_cmdq,
					      msecs_to_jiffies(600));
		}
	}
	
	return rc;
}

static ssize_t sk_backlight_set(char buf)
{
	int rc;
	u8 wdat;
	unsigned leds_state = buf;

	
	wdat = 0x02;
	rc = EKTF4751SK_i2c_write(g_i2c_client, 0x8B, &wdat, sizeof(wdat));
	if(!rc)
		printk("disable ektf4751 output pings!!\n");
	else
	{
		printk("disable ektf4751 output pings fail!!\n");
		return -1;
	}
	
	switch (leds_state) {

	case SOFTKEY_BACKLIGHT_OFF:

		wdat = 0x00;
		rc = EKTF4751SK_i2c_write(g_i2c_client, 0x8B, &wdat, sizeof(wdat));
		if(!rc)
			printk("softkey backlight off!!\n");
		else{
			printk("ektf4751sk control led fail!!\n");
			return -1;
		}
		break;
		
	case SOFTKEY_BACKLIGHT_ON: 
	default:
		wdat = 0x03;
		rc = EKTF4751SK_i2c_write(g_i2c_client, 0x8B, &wdat, sizeof(wdat));
		if(!rc)
			printk("softkey backlight on!!\n");
		else{
			printk("ektf4751sk control led fail!!\n");
			return -1;
		}
		break;
	}
	msleep(1);
	return 0;
}


//backlight virtual file+++++
static ssize_t sk_backlight_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client =
		container_of(dev, struct i2c_client, dev);
	int rc;
	u8 wdat;
	//u8 rdat;	
	unsigned leds_state=0;
/*
	wdat = 0x01;
	rc = EKTF4751SK_i2c_write(g_i2c_client, 0x00, &wdat, sizeof(wdat));
	msleep(1000);
*/
	sscanf(buf, "%d\n", &leds_state);
	
		wdat = 0x02;
		rc = EKTF4751SK_i2c_write(client, 0x8B, &wdat, sizeof(wdat));
		if(!rc)
			printk("disable ektf4751 output pings!!\n");
		else
		{
			printk("disable ektf4751 output pings fail!!\n");
			return -1;
		}
	
	switch (leds_state) {

	case SOFTKEY_BACKLIGHT_OFF:

		wdat = 0x00;
		rc = EKTF4751SK_i2c_write(client, 0x8B, &wdat, sizeof(wdat));
		if(!rc)
			printk("softkey backlight off!!\n");
		else
		{
			printk("ektf4751sk control led fail!!\n");
			return -1;
		}
		break;
		
	case SOFTKEY_BACKLIGHT_ON: 
	default:

		wdat = 0x03;		
		rc = EKTF4751SK_i2c_write(client, 0x8B, &wdat, sizeof(wdat));
		if(!rc)
			printk("softkey backlight on!!\n");
		else{
			printk("ektf4751sk control led fail!!\n");
			return -1;
		}
		break;

	}

	msleep(1);
	return 0;
}
DEVICE_ATTR(sk_backlight, 0664, NULL, sk_backlight_store);
//firmware version virtual file+++++
#if 1
static ssize_t sk_firmware_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	u8 rdat = 0xFF;
	int rc;
	
	printk( "EKTF4751SK: read firmware version\n");
	rc = EKTF4751SK_i2c_read(g_i2c_client, 0xFF, &rdat, sizeof(rdat));
	if(!rc)
	{
		printk("get firmware version: v.0x%x...\n",rdat);
		return sprintf(buf, "%x\r\n", rdat);
	}	
	else
	{
		printk("get firmware version fail...\n");
		return rc;
	}

}
#endif
DEVICE_ATTR(sk_firmware_version, 0664, sk_firmware_version_show, NULL);
//firmware version virtual file-----
/*
static int create_attributes(struct i2c_client *client)
{
	int rc;

	rc = device_create_file(&client->dev, &dev_attr_sk_backlight);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create softkey attribute \"skbacklight\" failed!! <%d>", __func__, rc);	
		return rc; 
	}

	
	return rc;	
}
*/
//backlight virtual file-----
//power state virtual file+++++
#define SOFTKEY_OPERATION_MODE 1
#define SOFTKEY_SLEEP_MODE 0
int ektf4751sk_power_state_switch(unsigned int enable)
{
	int rc;
	u8 wdat;

	switch (enable) {

	case SOFTKEY_OPERATION_MODE: 
		
		wdat = 0x90;
		rc = EKTF4751SK_i2c_write(g_i2c_client, 0x02, &wdat, sizeof(wdat));
		if(!rc)
			printk("ektf4751sk enter operation mode!!\n");
		else
		{
			printk("ektf4751sk enter operation mode fail!!\n");
			return -1;
		}

		break;
	case SOFTKEY_SLEEP_MODE:
	
		wdat = 0x91;
		rc = EKTF4751SK_i2c_write(g_i2c_client, 0x02, &wdat, sizeof(wdat));
		if(!rc)
			printk("ektf4751sk enter sleep mode\n");
		else
		{
			printk("ektf4751sk enter sleep mode fail!!\n");
			return -1;
		}
		//0822 patch+++
		g_last_btn_status = 0x00;
		//0822 patch---
		break;
	}
	return 0;
}
EXPORT_SYMBOL(ektf4751sk_power_state_switch);
//vince------------
static ssize_t sk_power_state_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
#if 0
	struct i2c_client *client =
		container_of(dev, struct i2c_client, dev);
#endif
	int rc;
//	u8 wdat;
	unsigned power_state=0;
	sscanf(buf, "%d\n", &power_state);
	
	rc = ektf4751sk_power_state_switch(power_state);
#if 0
	switch (power_state) {

	case SOFTKEY_OPERATION_MODE: 
		
		wdat = 0x90;
		rc = EKTF4751SK_i2c_write(client, 0x02, &wdat, sizeof(wdat));
		if(!rc)
			printk("ektf4751sk enter operation mode!!\n");
		else
		{
			printk("ektf4751sk enter operation mode fail!!\n");
			return -1;
		}

		break;
	case SOFTKEY_SLEEP_MODE:
	
		wdat = 0x91;
		rc = EKTF4751SK_i2c_write(client, 0x02, &wdat, sizeof(wdat));
		if(!rc)
			printk("ektf4751sk enter sleep mode\n");
		else
		{
			printk("ektf4751sk enter sleep mode fail!!\n");
			return -1;
		}

		break;
	}
#endif
	msleep(1);
	return 0;
}
static ssize_t sk_key_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int temp=12;
	u8 rdat[12];
	u8 addr[12]={0x00,0x02,0x26,0x27,0x28,0x29,0x20,0x21,0x66,0x67,0x94,0x95};
	int gpio_value = 123;
	int i=0;
	int rc;
	for(i=0;i<temp;i++)
	{
		rc = EKTF4751SK_i2c_read(g_i2c_client, addr[i], &rdat[i], sizeof(rdat[i]));
		if(rc)
		{
			printk("get key state fail...\n");
			return sprintf(buf, "fail to get key state,may be power down\n");
		}
		
		if(addr[i]<0x10)
		{
		
			if(rdat[i]<0x10)
				sprintf(buf+i*14,"add:0x0%x=0x0%x\n",addr[i],rdat[i]);
			else
				sprintf(buf+i*14,"add:0x0%x=0x%x\n",addr[i],rdat[i]);
		}
		else
		{
			if(rdat[i]<0x10)
				sprintf(buf+i*14,"add:0x%x=0x0%x\n",addr[i],rdat[i]);
			else
				sprintf(buf+i*14,"add:0x%x=0x%x\n",addr[i],rdat[i]);

		}
	
	}
	gpio_value=gpio_get_value(INT_PIN);
	sprintf(buf+i*14,"gpio_value:%d\n",gpio_value);
	return i*14+13;

}

static ssize_t sk_key_calibrate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u8 cmd=0;
	u8 para=0;
	u8 wdat=0;
	int rc=0;
	
	if((*(buf+0)=='0')&&(*(buf+1)=='x')&&(*(buf+5)=='0')&&(*(buf+6)=='x'))
	{
		if('0'<=*(buf+2)&&*(buf+2)<='9')
			cmd=(*(buf+2)-'0')*16;
		else
			cmd=(*(buf+2)-'a'+10)*16;
		if('0'<=*(buf+3)&&*(buf+3)<='9')
			cmd+=*(buf+3)-'0';
		else
			cmd+=*(buf+3)-'a'+10;
		if('0'<=*(buf+7)&&*(buf+7)<='9')
			para=(*(buf+7)-'0')*16;
		else
			para=(*(buf+7)-'a'+10)*16;
		if('0'<=*(buf+8)&&*(buf+8)<='9')
			para+=*(buf+8)-'0';
		else
			para+=*(buf+8)-'a'+10;
	//	printk("cmd:0x%x___para:0x%x\n",cmd,para);
		rc = EKTF4751SK_i2c_write(g_i2c_client, cmd, &para, sizeof(para));
		if(rc)
		{
			printk("fuction:%s write eeprom fail\n",__func__);
			return -1;
		}
		wdat = 0x01;
		rc = EKTF4751SK_i2c_write(g_i2c_client, 0x04, &wdat, sizeof(wdat));

		return 0;
	}
	return -1;
}
DEVICE_ATTR(sk_power_state, 0664, NULL, sk_power_state_store);
DEVICE_ATTR(sk_key_state, 0664,sk_key_state_show, NULL);
DEVICE_ATTR(sk_key_calibrate, 0644, NULL, sk_key_calibrate_store);
static int create_attributes(struct i2c_client *client)
{
	int rc;

	rc = device_create_file(&client->dev, &dev_attr_sk_backlight);
	rc = device_create_file(&client->dev, &dev_attr_sk_power_state);
	rc = device_create_file(&client->dev, &dev_attr_sk_firmware_version);
	rc = device_create_file(&client->dev, &dev_attr_sk_key_state);
	rc = device_create_file(&client->dev, &dev_attr_sk_key_calibrate);
	if (rc < 0) {
		dev_err(&client->dev, "%s: Create softkey attribute \"skbacklight\" failed!! <%d>", __func__, rc);	
		return rc; 
	}

	
	return rc;	
}

//power state virtual file-----



static int remove_attributes(struct i2c_client *client)
{
//2010.10.13 Jones	
//device_remove_file(&client->dev, &dev_attr_brightness);
//2010.10.13 Jones
//device_remove_file(&client->dev, &dev_attr_btn_brightness);
	device_remove_file(&client->dev, &dev_attr_sk_backlight);
#ifdef DEBUG
	device_remove_file(&client->dev, &dev_attr_debug_info);
#endif
//2010.10.14 Vince
//device_remove_file(&client->dev, &dev_attr_blink);


	return 0;
}

static int EKTF4751SK_remove(struct i2c_client *kbd)
{
	struct EKTF4751SK_record *rd = i2c_get_clientdata(kbd);

	dev_info(&kbd->dev, "removing keyboard driver\n");
	device_init_wakeup(&kbd->dev, 0);

	if (rd->i2ckbd_idev) {
		dev_dbg(&kbd->dev, "deregister from input system\n");
		input_unregister_device(rd->i2ckbd_idev);
		rd->i2ckbd_idev = 0;
	}
	EKTF4751SK_shutdown(rd);
	EKTF4751SK_release_gpio(rd);

	remove_attributes(kbd);

	//switch_dev_unregister(&ringer_switch_dev);
	
	unregister_early_suspend(&rd->early_suspend);

	//kill_proc_info(1, SIGKILL, thread_id);

	kfree(rd);

	return 0;
}

#ifdef CONFIG_PM
#if 1                                                                       //add by meepo
static int EKTF4751SK_suspend(struct i2c_client *kbd, pm_message_t mesg)
{
	u8 wdat;
	int rc=0;
	dev_dbg(&kbd->dev, "%s: Enter SUSPEND Mode.\n", __func__);
	disable_irq(KBDIRQNO(EKTF4751SK_rd));
	rc = cancel_work_sync(&EKTF4751SK_rd->qkybd_irqwork);
    if(rc){
        printk("EKTF4751SK_early_suspend disable_irq\n");
        enable_irq(KBDIRQNO(EKTF4751SK_rd));
    }
	
	wdat = 0x91;
	rc=EKTF4751SK_i2c_write(kbd, 0x02, &wdat, sizeof(wdat));
	if(!rc)
		printk("ektf4751sk enter sleep mode\n");
	else
	{
		printk("ektf4751sk enter sleep mode fail!!\n");
		return -1;
	}
	
	g_last_btn_status = 0x00;
	
	return 0;
}
#else
static int EKTF4751SK_suspend(struct i2c_client *kbd, pm_message_t mesg)
{
	int rc=0;
	u8 wdat;
	bool hallstat = (bool)gpio_get_value(HALL_INT);
	dev_dbg(&kbd->dev, "%s: Enter SUSPEND Mode.\n", __func__);

	//check: if upper touch is open, do sleep, otherwise do nothing
	//hallstat=1: upper touch is opened
	//hallstat=0: upper touch is closed
	if(hallstat)
	{
		wdat = 0x91;
		rc = EKTF4751SK_i2c_write(kbd, 0x02, &wdat, sizeof(wdat));
		if(!rc)
			printk("ektf4751sk enter sleep mode\n");
		else
		{
			printk("ektf4751sk enter sleep mode fail!!\n");
			return -1;
		}
		//0822 patch+++
		g_last_btn_status = 0x00;
		//0822 patch---
	}
	else
		printk("ektf4751sk doesn't sleep since upper touch is closed!!\n");
	return 0;
}
#endif

#if 1                                                                   //add by meepo
static int EKTF4751SK_resume(struct i2c_client *kbd)
{
	u8 wdat;
	int rc=0;
	dev_dbg(&kbd->dev, "%s: Leave SUSPEND Mode.\n", __func__);
	enable_irq(KBDIRQNO(EKTF4751SK_rd));
	
	wdat = 0x90;
	rc=EKTF4751SK_i2c_write(kbd, 0x02, &wdat, sizeof(wdat));
	if(!rc)
			printk("ektf4751sk enter operation mode!!\n");
	else
	{
		printk("ektf4751sk enter operation mode fail!!\n");
		return -1;
	}
	return 0;
}


#else
static int EKTF4751SK_resume(struct i2c_client *kbd)
{
	int rc=0;
	u8 wdat;

	bool hallstat = (bool)gpio_get_value(HALL_INT);
	dev_dbg(&kbd->dev, "%s: Leave SUSPEND Mode.\n", __func__);

	//check: if upper touch is open, do resume, otherwise do nothing
	//hallstat=1: upper touch is opened
	//hallstat=0: upper touch is closed
	if(hallstat)
	{
		wdat = 0x90;
		rc = EKTF4751SK_i2c_write(kbd, 0x02, &wdat, sizeof(wdat));
		if(!rc)
			printk("ektf4751sk enter operation mode!!\n");
		else
		{
			printk("ektf4751sk enter operation mode fail!!\n");
			return -1;
		}
		msleep(40);	
	}
	else
		printk("ektf4751sk doesn't resume since upper touch is closed!!\n");
	return 0;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void EKTF4751SK_early_suspend(struct early_suspend *h)
{
	struct EKTF4751SK_record *kybd;
	kybd = container_of(h, struct EKTF4751SK_record, early_suspend);
	EKTF4751SK_suspend(kybd->mykeyboard, PMSG_SUSPEND);
	dev_dbg(&kybd->mykeyboard->dev, "%s: EARLY_SUSPEND Starting\n", __func__);
}

static void EKTF4751SK_late_resume(struct early_suspend *h)
{
	struct EKTF4751SK_record *kybd;
	kybd = container_of(h, struct EKTF4751SK_record, early_suspend);
	EKTF4751SK_resume(kybd->mykeyboard);
	dev_dbg(&kybd->mykeyboard->dev, "%s: LATE RESUME Finished\n", __func__);
}
#endif

#else
#define EKTF4751SK_suspend NULL
#define EKTF4751SK_resume  NULL
#ifdef CONFIG_HAS_EARLYSUSPEND
#define EKTF4751SK_early_suspend NULL
#define EKTF4751SK_late_resume NULL
#endif
#endif

static struct i2c_device_id EKTF4751SK_idtable[] = {
       { EKTF4751SK_name, 0 },
       { }
};
MODULE_DEVICE_TABLE(i2c, EKTF4751SK_idtable);

static int EKTF4751SK_probe(struct i2c_client *client,
				    const struct i2c_device_id *id);
				    
static struct i2c_driver i2ckbd_EKTF4751SK = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = EKTF4751SK_name,
	},
	.probe	  = EKTF4751SK_probe,
	.remove	  = EKTF4751SK_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend  = EKTF4751SK_suspend,
	.resume   = EKTF4751SK_resume,
#endif
	.id_table = EKTF4751SK_idtable,
};

static void EKTF4751SK_led_brightness_set(struct led_classdev *led_cdev,
                   enum led_brightness brightness)
{
	if (!strcmp(led_cdev->name, "button-backlight"))
	{
		if(brightness)
			sk_backlight_set(1);
		else		
			sk_backlight_set(0);

	}
}


static int EKTF4751SK_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	int rc = -ENOMEM;
	u8 wdat ;
	bool hallstat;


	g_i2c_client = client;


	printk( "EKTF4751SK_probe+++++++++++++++++++\r\n" ); 
	EKTF4751SK_rd = kzalloc(sizeof(struct EKTF4751SK_record), GFP_KERNEL);
	if (!EKTF4751SK_rd) {
		dev_err(&client->dev, "EKTF4751SK_record memory allocation failed!!\n");
		return rc;
	}
	client->driver		= &i2ckbd_EKTF4751SK;

	
	i2c_set_clientdata(client, EKTF4751SK_rd);
	EKTF4751SK_rd->mykeyboard		= client;
	EKTF4751SK_rd->irqpin		= INT_PIN;
///
	spin_lock_init(&EKTF4751SK_rd->EKTF4751SK_lock);

#ifdef CONFIG_HAS_EARLYSUSPEND
//	EKTF4751SK_rd->early_suspend.level		= EARLY_SUSPEND_LEVEL_DISABLE_QWERTY_KEYPAD; // jjj
	EKTF4751SK_rd->early_suspend.suspend	= EKTF4751SK_early_suspend;
	EKTF4751SK_rd->early_suspend.resume	= EKTF4751SK_late_resume;
	register_early_suspend(&EKTF4751SK_rd->early_suspend);
#endif


	//Initialize IRQ
	INIT_WORK(&EKTF4751SK_rd->qkybd_irqwork, EKTF4751SK_fetchkeys);

	rc = EKTF4751SK_irqsetup(EKTF4751SK_rd); 
	if (rc)
		goto failexit2;

	rc = testfor_keybd(client);
	if (!rc)
		device_init_wakeup(&client->dev, 1);
	else {
		mdelay(1000);
		
		rc = testfor_keybd(client);
		if (rc) {
			printk( "%s: Initialize keyboard failed!! <%d>", __func__, rc);		
			goto failexit2;
		}
	}
	
	rc = create_attributes(client);
	if (rc < 0) {
		printk( "%s: create attributes failed!! <%d>", __func__, rc);

		return rc;
	}


	//check: if upper touch is open, do sleep, otherwise do nothing
	//hallstat=1: upper touch is opened
	//hallstat=0: upper touch is closed
   //hallstat = (bool)gpio_get_value(HALL_INT);
#if 0
   rc = EKTF4751SK_i2c_read(g_i2c_client, 0xFF, &rdat, sizeof(rdat));
	if(!rc)
	{
		printk("get firmware version: v.0x%x...\n",rdat);
		return sprintf(buf, "%x\r\n", rdat);
	}	
	else
	{
		printk("get firmware version fail...\n");
		return rc;
	}
	do{
	rc = EKTF4751SK_i2c_read(g_i2c_client, 0xFC, &rdat, sizeof(rdat));//added for test
	printk("ektf4751sk i2c_address:%x value :%x, rc:%d\n",g_i2c_client->addr,rdat,rc);
		}while(ii--);
#endif

#if 0
	if(!hallstat)
	{
		wdat = 0x90;
		rc = EKTF4751SK_i2c_write(g_i2c_client, 0x02, &wdat, sizeof(wdat));
		if(!rc)
			printk("ektf4751sk enter sleep mode\n");
		else
		{
			printk("ektf4751sk enter sleep mode fail!!\n");
		}
	}
	else
		printk("ektf4751sk doesn't sleep since upper touch is opened!!\n");
#endif	

	

	EKTF4751SK_rd->led_dev.name = "button-backlight";
	EKTF4751SK_rd->led_dev.brightness_set = EKTF4751SK_led_brightness_set;
	rc = led_classdev_register(&client->dev, &EKTF4751SK_rd->led_dev);
	if (rc) {
	    printk("%s: led_classdev_register failed\n", __func__);
	}


//set sensitivity
	wdat = 0x90;
	rc = EKTF4751SK_i2c_write(g_i2c_client, 0x02, &wdat, sizeof(wdat));
	if(!rc)
	printk("ektf4751sk enter operation mode!!\n");
	else
	{
		printk("ektf4751sk enter operation mode fail!!\n");
		return -1;
	}
	
	wdat = 0x12;
	rc = EKTF4751SK_i2c_write(g_i2c_client, 0x66, &wdat, sizeof(wdat));
	rc = EKTF4751SK_i2c_write(g_i2c_client, 0x67, &wdat, sizeof(wdat));
	wdat = 0x00;
	rc = EKTF4751SK_i2c_write(g_i2c_client, 0x20, &wdat, sizeof(wdat));
	wdat = 0x00;
	rc = EKTF4751SK_i2c_write(g_i2c_client, 0x21, &wdat, sizeof(wdat));
	wdat = 0x86;
	rc = EKTF4751SK_i2c_write(g_i2c_client, 0x94, &wdat, sizeof(wdat));
	wdat = 0x78;
	rc = EKTF4751SK_i2c_write(g_i2c_client, 0x95, &wdat, sizeof(wdat));

	wdat = 0x01;
	rc = EKTF4751SK_i2c_write(g_i2c_client, 0x04, &wdat, sizeof(wdat));
	
	
	hallstat = (bool)gpio_get_value(INT_PIN);
	printk("interrupt:%d\n",hallstat);
	printk( "EKTF4751SK_probe----------------------done\n" ); 
	return 0;

 failexit2:
//	free_irq(KBDIRQNO(EKTF4751SK_rd), EKTF4751SK_rd);

// failexit1:
//	EKTF4751SK_release_gpio(EKTF4751SK_rd);
//// There is some problem with kfree(EKTF4751SK_rd);
//// 	kfree(EKTF4751SK_rd);

	return rc;
}


static int __init EKTF4751SK_init(void *hdl )
{
    printk(KERN_INFO"[EKTF4751SK] %s\n",__func__);

    return ADD_DYN_I2C_DRIVER(hdl, &i2ckbd_EKTF4751SK);
}
DECLARE_DYN_LOADER(DRIVER_TYPE_TOUCH, TOUCH_ATMEL_QT602240_V6, EKTF4751SK_init, LEVEL6);


static void __exit EKTF4751SK_exit(void)
{
	i2c_del_driver(&i2ckbd_EKTF4751SK);
}

module_exit(EKTF4751SK_exit);


MODULE_VERSION("1.0");
MODULE_DESCRIPTION("I2C TOUCH KEYPAD driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:EKTF4751SK");

