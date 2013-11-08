/*
 *  FIH headset device detection driver.
 *
 */

/*  

    Logically, the H2W driver is always present, and H2W state (hi->state)
    indicates what is currently plugged into the headset interface.

    The headset detection work involves GPIO. A headset will still
    pull this line low.

    Headset insertion/removal causes UEvent's to be sent, and
    /sys/class/switch/headset_sensor/state to be updated.

    Button presses are interpreted as input event (KEY_MEDIA). 

*/


#include <linux/module.h>
#include <linux/sysdev.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/switch.h>
#include <linux/debugfs.h>
#include <asm/gpio.h>
#include <asm/atomic.h>
#include <mach/mpp.h>
//#include <linux/fih_hw_info.h>
#include <asm/io.h>     
#include "../../../gpio_hw.h"   
//#include "../../../../drivers/serial/msm_serial.h"   //for gasko/dorian only, uart 83
#include <mach/pmic.h>
#include <linux/earlysuspend.h>
#include <linux/slab.h>

#include <mach/rpc_server_handset.h>

#include "fih_headset.h"

//Define the GPIOs
//#define AUD_PIN_HEADSET_DET     114 
#define AUD_PIN_HOOK_BTN        86

#define ASWISH_CTL              37

#define CONFIG_DEBUG_FIH_HEADSET 

#define KEY_RINGSWITCH          184 

#define DEBUGLOG 1
#if DEBUGLOG
    #define H2W_DBG(fmt, arg...) printk(KERN_INFO "[AUD_HS] %s " fmt "\n", __FUNCTION__, ## arg)
#else
    #define H2W_DBG(fmt, arg...) do {} while (0)
#endif

static struct workqueue_struct *g_detection_work_queue;
static void detection_work(struct work_struct *work);
static DECLARE_WORK(g_detection_work, detection_work);
void report_hs_key(uint32_t key_code, uint32_t key_parm);

static int bn_irq_enable               = 0;
static bool mHeadphone                 = false;
static bool AUD_HOOK_BTN               = false;
static int aud_pin_headset_det          = 114;

//Define the switch state
// /sys/class/switch/headset_sensor/state
//Need to be syncronized with HeadsetObserver.java
enum {
    NO_DEVICE                   = 0,
    HEADSET                     = 1,
    NOMIC_HEADSET               = 2,//MM-RC-SupportNO_MIC_Hs-00+
};

// headset phone jack detect
// GPIO_114 : 1 plug out
//            0 plug in
int HS_PLUG_IN                  = 0;
int IRQF_TRIGGER_HS_INSERTED    = IRQF_TRIGGER_HIGH;

// hook button detect
// ___      ____
//    \____/
enum {
    BTN_STATE_PRESSED           = 0,
    BTN_STATE_RELEASED          = 1,
    IRQF_TRIGGER_BTN_PRESSED    = IRQF_TRIGGER_LOW,
};

struct h2w_info {
    struct switch_dev sdev;
    struct input_dev *input;
    struct input_dev *hs_input; 

    atomic_t btn_state;
    atomic_t hs_state;
    int ignore_hs;
    int ignore_btn;

    unsigned int irq;
    unsigned int irq_btn;

    struct hrtimer timer;
    ktime_t debounce_time;

    struct hrtimer btn_timer;
    ktime_t btn_debounce_time;
};
struct h2w_info *hi;



static ssize_t trout_h2w_print_name(struct switch_dev *sdev, char *buf)
{
    int state = 0;
    state = switch_get_state(&hi->sdev);
    H2W_DBG("%s, state = %d", __FUNCTION__, state);

	switch (state) 
	{
    case NO_DEVICE:
        return sprintf(buf, "No Device\n");
    case HEADSET:         
        return sprintf(buf, "Headset\n");
    }

    return -EINVAL;
}



void aud_hs_print_gpio(void)
{
    H2W_DBG("hs gpio(%d) = %d, ptt gpio(%d) = %d ", aud_pin_headset_det, gpio_get_value(aud_pin_headset_det), AUD_PIN_HOOK_BTN, gpio_get_value(AUD_PIN_HOOK_BTN));
}



void aud_hs_dump_reg(void)
{
#if 0
    H2W_DBG("");
    H2W_DBG("GPIO 94, mask = 0x0400-0000");
    //H2W_DBG("GPIO 87, mask = 0x0008-0000");
    H2W_DBG("GPIO_OUT_3 : 0x%08x", readl(GPIO_OUT_3));
    H2W_DBG("GPIO_OE_3 : 0x%08x", readl(GPIO_OE_3));    
    H2W_DBG("GPIO_IN_3 : 0x%08x", readl(GPIO_IN_3));
    H2W_DBG("GPIO_INT_EDGE_3 : 0x%08x", readl(GPIO_INT_EDGE_3));
    H2W_DBG("GPIO_INT_POS_3 : 0x%08x", readl(GPIO_INT_POS_3));
    H2W_DBG("GPIO_INT_EN_3 : 0x%08x", readl(GPIO_INT_EN_3));  
    H2W_DBG("GPIO_INT_CLEAR_3 : 0x%08x", readl(GPIO_INT_CLEAR_3));
    H2W_DBG("GPIO_INT_STATUS_3 : 0x%08x", readl(GPIO_INT_STATUS_3));
    H2W_DBG("========================");
    H2W_DBG("GPIO 21, mask = 0x0000-0020");
    H2W_DBG("GPIO_OUT_1 : 0x%08x", readl(GPIO_OUT_1));
    H2W_DBG("GPIO_OE_1 : 0x%08x", readl(GPIO_OE_1));    
    H2W_DBG("GPIO_IN_1 : 0x%08x", readl(GPIO_IN_1));
    H2W_DBG("GPIO_INT_EDGE_1 : 0x%08x", readl(GPIO_INT_EDGE_1));
    H2W_DBG("GPIO_INT_POS_1 : 0x%08x", readl(GPIO_INT_POS_1));
    H2W_DBG("GPIO_INT_EN_1 : 0x%08x", readl(GPIO_INT_EN_1));  
    H2W_DBG("GPIO_INT_CLEAR_1 : 0x%08x", readl(GPIO_INT_CLEAR_1));
    H2W_DBG("GPIO_INT_STATUS_1 : 0x%08x", readl(GPIO_INT_STATUS_1));
#endif

}


//KEY_MEDIA is defined in <Linux/include/input.h>
//You can modify the keypad layout to map this key for Android
//For ADQ project, it will be located on : Vendor/qcom/msm7225_adq/stmpe1601.kl
//check the Android.mk first.
//#ifdef AUD_HOOK_BTN

#define HS_HEADSET_SWITCH_K         0x84
#define HS_REL_K                    0xFF 
#define HS_NONE_K                   0x00

static void button_pressed(void)
{
	H2W_DBG("+");
	atomic_set(&hi->btn_state, 1); 
	/* FIH, TerryChen  , 2011/08/25 { */
	/* Report the correct value of key press/release*/	
    //input_report_key(hi->input, KEY_MEDIA, 1);
    //input_sync(hi->input);			
    report_hs_key(HS_HEADSET_SWITCH_K, HS_NONE_K);
	/*} FIH, TerryChen , 2011/08/25 */		
	H2W_DBG("-");
}

static void button_released(void)
{
	H2W_DBG("+");
    atomic_set(&hi->btn_state, 0);
    /* FIH, TerryChen  , 2011/08/25 { */
    /* Report the correct value of key press/release*/      
    //input_report_key(hi->input, KEY_MEDIA, 0); 
    //input_sync(hi->input);			
    report_hs_key(HS_HEADSET_SWITCH_K, HS_REL_K);
    /*} FIH, TerryChen , 2011/08/25 */      
    H2W_DBG("-");
}
//#endif

#ifdef CONFIG_MSM_SERIAL_DEBUGGER
extern void msm_serial_debug_enable(int);
#endif



static void insert_headset(void)
{
	unsigned long irq_flags;

    H2W_DBG("+");

    input_sync(hi->hs_input);           

#ifdef CONFIG_MSM_SERIAL_DEBUGGER
    msm_serial_debug_enable(false);
#endif
// Div2-SW5-BSP-Audio-00+{
    //Terry 2011.7.14  If PM_HSED_CONTROLLER_0 disabled, headset detect pin can't work		
		if(AUD_HOOK_BTN) //vincent modified for complient with NJ Project
		{
		      pmic_hsed_enable(PM_HSED_CONTROLLER_0, PM_HSED_ENABLE_ALWAYS);		
		}
    //H2W_DBG("aud_hs:open mic bias\n ");
    msleep(100);

    if(AUD_HOOK_BTN)
    {
        /* On some non-standard headset adapters (usually those without a
         * button) the btn line is pulled down at the same time as the detect
         * line. We can check here by sampling the button line, if it is
         * low then it is probably a bad adapter so ignore the button.
         * If the button is released then we stop ignoring the button, so that
         * the user can recover from the situation where a headset is plugged
         * in with button held down.
         */
	    hi->ignore_btn = !gpio_get_value(AUD_PIN_HOOK_BTN);   /*FIHTDC, For detecting hook key after plug headset in, MayLi, 2011.09.09*/
    }
// Support NOMIC_HEADSET
	if(gpio_get_value(aud_pin_headset_det)==HS_PLUG_IN)
	{
        H2W_DBG("aud_hs:HEADSET is plugging\n ");
        // Check the headset type, 1: With microphone, 0: No microphone
 	      if(AUD_HOOK_BTN) //vincent modified for complient with NJ Project E
        {     
		        if (gpio_get_value(AUD_PIN_HOOK_BTN)==BTN_STATE_PRESSED) {
		   	        H2W_DBG("No microphone\n");	
// Div2-SW5-BSP-Audio-00+{
// IRM.B-4772: Can not detect the hook key event after plug in the non-omtp headset.
				    // disable_irq(hi->irq_btn);
// Div2-SW5-BSP-Audio-00+}
			        bn_irq_enable=0;
					switch_set_state(&hi->sdev, NOMIC_HEADSET);
		    		H2W_DBG("aud_hs: disable mic bias\n ");
		        }
		        else {
		   	        H2W_DBG("With microphone\n");	
				    switch_set_state(&hi->sdev, HEADSET);          
				    /* Enable button irq */
			        if (bn_irq_enable==0)
			        {
		       	        H2W_DBG("Enable button irq\n");	
				        local_irq_save(irq_flags);
				        set_irq_type(hi->irq_btn, (gpio_get_value(AUD_PIN_HOOK_BTN) ?
						        IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH));
				        enable_irq(hi->irq_btn);
				        local_irq_restore(irq_flags);
				        bn_irq_enable=1;
		            }
        	}    
        	
        }  //vincent modified for complient with NJ Project M
        else
        {
            // Now,NJ project doesn't support 3/4 headset. 2011/11/12
           //switch_set_state(&hi->sdev, HEADSET);
        }  //vincent modified for complient with NJ Project M     		        
        H2W_DBG("switch_get_state= %d ",switch_get_state(&hi->sdev));   
    }
    H2W_DBG("-");
// Div2-SW5-BSP-Audio-00+}
}

static void remove_headset(void)
{
	unsigned long irq_flags;

    H2W_DBG("+");

    switch_set_state(&hi->sdev, NO_DEVICE);

    input_sync(hi->hs_input);           

    if (AUD_HOOK_BTN) 
	{
        mHeadphone=false;
        if (bn_irq_enable==1) 
		{
            /* Disable button */
            local_irq_save(irq_flags);
            disable_irq(hi->irq_btn);
            set_irq_wake(hi->irq_btn, 0); 
            local_irq_restore(irq_flags);
            bn_irq_enable=0;
        }


        if (atomic_read(&hi->btn_state))
            button_released();
    }
    //Terry 2011.7.14  If PM_HSED_CONTROLLER_0 disabled, headset detect pin can't work		
 	      if(AUD_HOOK_BTN) //vincent modified for complient with NJ Project E
        {     

    pmic_hsed_enable(PM_HSED_CONTROLLER_0, PM_HSED_ENABLE_OFF);
}
    //H2W_DBG("aud_hs:close mic bias\n ");

    H2W_DBG("-");
}

static void detection_work(struct work_struct *work) 
{
    //unsigned long irq_flags;
    int cable_in1;

    H2W_DBG("");

    //aud_hs_dump_reg();

    if (gpio_get_value(aud_pin_headset_det) != HS_PLUG_IN) 
	{
        /* Headset not plugged in */
        if ((switch_get_state(&hi->sdev) == HEADSET)||(switch_get_state(&hi->sdev) == NOMIC_HEADSET)) //MM-RC-SupportNO_MIC_Hs-00*
        {
            H2W_DBG("Headset is plugged out.\n");
            remove_headset();

        }
        return;
    }

    /* Something plugged in, lets make sure its a headset */
    cable_in1 = gpio_get_value(aud_pin_headset_det);

	if (cable_in1 == HS_PLUG_IN ) 
	{
	  	if (switch_get_state(&hi->sdev) == NO_DEVICE)
	  	{
	  		H2W_DBG("Headset is plugged in.\n");
	  		insert_headset();
	  	}
	}else 
	{
	  	H2W_DBG("WARN: aud_pin_headset_det was low, but not a headset ");
	}

}

static enum hrtimer_restart button_event_timer_func(struct hrtimer *data)
{
    H2W_DBG("+");
    //aud_hs_dump_reg(); 

    if (switch_get_state(&hi->sdev) == HEADSET) {
        if (gpio_get_value(AUD_PIN_HOOK_BTN) == BTN_STATE_RELEASED) {
            if (hi->ignore_btn)
                hi->ignore_btn = 0;
            else if (atomic_read(&hi->btn_state))
                button_released();
		} else {
            if (!hi->ignore_btn && !atomic_read(&hi->btn_state))
                button_pressed();
        }
    }

    H2W_DBG("-");
    return HRTIMER_NORESTART;
}

static enum hrtimer_restart detect_event_timer_func(struct hrtimer *data)
{
	H2W_DBG("+");

    queue_work(g_detection_work_queue, &g_detection_work);
    H2W_DBG("-");
    return HRTIMER_NORESTART;
}

static irqreturn_t detect_irq_handler(int irq, void *dev_id)
{
	int value1, value2;
	int retry_limit = 10;

    H2W_DBG("");
    aud_hs_print_gpio(); 

    //debunce
    do {
        value1 = gpio_get_value(aud_pin_headset_det);
        set_irq_type(hi->irq, value1 ?
                     IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
        value2 = gpio_get_value(aud_pin_headset_det);         
        //H2W_DBG("VALUE1 = %d, value2 = %d\n", value1, value2);
    } while (value1 != value2 && retry_limit-- > 0);

    H2W_DBG("value2 = %d (%d retries)", value2, (10-retry_limit));  

    /*
  * If the sdev is NO_DEVICE, and we detect the headset has been plugged,
  * then we can do headset_insertion check.
  */
    if ((switch_get_state(&hi->sdev) == NO_DEVICE) ^ (value2^HS_PLUG_IN)) {
        if (switch_get_state(&hi->sdev) == HEADSET)
            hi->ignore_btn = 1;
        /* Do the rest of the work in timer context */
        hrtimer_start(&hi->timer, hi->debounce_time, HRTIMER_MODE_REL);
    }

    return IRQ_HANDLED;
}

static irqreturn_t button_irq_handler(int irq, void *dev_id)
{
    int value1, value2;
    int retry_limit = 10;

    H2W_DBG("");
    aud_hs_print_gpio(); 
    do {
        value1 = gpio_get_value(AUD_PIN_HOOK_BTN);
        set_irq_type(hi->irq_btn, value1 ?
                     IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
        value2 = gpio_get_value(AUD_PIN_HOOK_BTN);
        //H2W_DBG("VALUE1 = %d, value2 = %d\n", value1, value2);
    } while (value1 != value2 && retry_limit-- > 0);

    H2W_DBG("Hook BTN :value2 = %d (%d retries)", value2, (10-retry_limit));


    hrtimer_start(&hi->btn_timer, hi->btn_debounce_time, HRTIMER_MODE_REL);

    return IRQ_HANDLED;
}

#if defined(CONFIG_DEBUG_FS)

static int __init h2w_debug_init(void)
{
	struct dentry *dent;

    dent = debugfs_create_dir("h2w", 0);
    if (IS_ERR(dent))
        return PTR_ERR(dent);

    return 0;
}

device_initcall(h2w_debug_init);
#endif


static int trout_h2w_probe(struct platform_device *pdev)
{
	int ret;

    printk(KERN_INFO "[AUD_HS]: Registering H2W (headset) driver\n");

    hi = kzalloc(sizeof(struct h2w_info), GFP_KERNEL);
    if (!hi)
        return -ENOMEM;

    atomic_set(&hi->btn_state, 0);
    hi->ignore_btn = 0;

    // Headset insertion/removal causes UEvent's to be sent, and
    // /sys/class/switch/h2w/state to be updated.

    hi->debounce_time = ktime_set(0, 500000000);  /* 500 ms */
    hi->btn_debounce_time = ktime_set(0, 80000000); /* 80 ms */   //debounce time too short will affect the behavior of headset plugin/out in phone call 
    hi->sdev.name = "h2w";
    hi->sdev.print_name = trout_h2w_print_name; 
    hi->hs_input = input_allocate_device();
    if (!hi->hs_input) {
        ret = -ENOMEM;
        goto err_request_input_dev;
    }
    atomic_set(&hi->hs_state, 0);
    hi->ignore_hs = 0;
    hi->hs_input->name = "fih_ringswitch";
    set_bit(EV_KEY, hi->hs_input->evbit);     
    set_bit(KEY_RINGSWITCH, hi->hs_input->keybit);            

    ret = input_register_device(hi->hs_input);
    if (ret < 0)
        goto err_register_hs_input_dev;


    ret = switch_dev_register(&hi->sdev);
    if (ret < 0)
        goto err_switch_dev_register;

    g_detection_work_queue = create_workqueue("detection");

    if (g_detection_work_queue == NULL) {
        ret = -ENOMEM;
        goto err_create_work_queue;
    }
	
//++FIH-NJ-BSP-SW4 Simon NPM project need config gpio to dectect headset insert irq.
    //if(aud_pin_headset_det == 39)
    //gpio_tlmm_config(GPIO_CFG(aud_pin_headset_det, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
//--FIH-NJ-BSP-SW4 Simon NPM project need config gpio to dectect headset insert irq.

    ret = gpio_request(aud_pin_headset_det, "h2w_detect");  
    if (ret < 0)
        goto err_request_detect_gpio;

    if(AUD_HOOK_BTN)
	{
        ret = gpio_request(AUD_PIN_HOOK_BTN, "h2w_button");   
        if (ret < 0)
            goto err_request_button_gpio;
    }

    ret = gpio_direction_input(aud_pin_headset_det);    
    if (ret < 0)
        goto err_set_detect_gpio;
    else
        H2W_DBG(" set aid gpio(%d) as input pin : success.\r\n", aud_pin_headset_det);

    if(AUD_HOOK_BTN)
	{
        ret = gpio_direction_input(AUD_PIN_HOOK_BTN);   
        if (ret < 0)
            goto err_set_button_gpio;
        else
            H2W_DBG(" set ptt gpio(%d) as input pin : success.\r\n", AUD_PIN_HOOK_BTN);
    }



    hi->irq = gpio_to_irq(aud_pin_headset_det);   
    if (hi->irq < 0) {
        ret = hi->irq;
        goto err_get_h2w_detect_irq_num_failed;
    }
    else
        H2W_DBG(" hs_det gpio_to_irq(%d): success.\r\n", aud_pin_headset_det);

    if(AUD_HOOK_BTN)
	{
        hi->irq_btn = gpio_to_irq(AUD_PIN_HOOK_BTN);   
        if (hi->irq_btn < 0) {
            ret = hi->irq_btn;
            goto err_get_button_irq_num_failed;
        }
        else
            H2W_DBG(" hook_btn gpio_to_irq(%d): success.\r\n", AUD_PIN_HOOK_BTN);
    }

    hrtimer_init(&hi->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    hi->timer.function = detect_event_timer_func;
    if(AUD_HOOK_BTN)
	{
        hrtimer_init(&hi->btn_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        hi->btn_timer.function = button_event_timer_func;
    }
    aud_hs_print_gpio(); 
    //aud_hs_dump_reg();
    //printk(KERN_INFO "[AUD_HS] roger: gpio 94 should not be LOW without insertion\r\n");

    //When 	headset inserted, gpio H->L, so we detect LOW level.
    ret = request_irq(hi->irq, detect_irq_handler,
                      IRQF_TRIGGER_HS_INSERTED, "h2w_detect", NULL);
    if (ret < 0)
        goto err_request_detect_irq;
    else
        H2W_DBG(" request_irq (gpio %d, IRQF_TRIGGER_LOW) success\n", aud_pin_headset_det);

    if(AUD_HOOK_BTN)
	{
        hi->input = input_allocate_device();
        if (!hi->input) {
            ret = -ENOMEM;
            goto err_request_input_dev;
        }
        input_set_drvdata(hi->input, hi);
    
        hi->input->name = "h2w";
        hi->input->evbit[0] = BIT_MASK(EV_KEY); 
        //hi->input->keybit[BIT_WORD(KEY_MEDIA)] = BIT_MASK(KEY_MEDIA);   
    
        //input_set_capability(hi->input, EV_KEY, KEY_MEDIA);
    
        ret = input_register_device(hi->input);
        if (ret < 0)
            goto err_register_input_dev;
    
    
        /* Disable button until plugged in */
        set_irq_flags(hi->irq_btn, IRQF_VALID | IRQF_NOAUTOEN);
        ret = request_irq(hi->irq_btn, button_irq_handler,
                          IRQF_TRIGGER_BTN_PRESSED, "h2w_button", NULL);         
        if (ret < 0)
            goto err_request_h2w_headset_button_irq;
        else
            H2W_DBG("request_irq (gpio %d, IRQF_TRIGGER_HIGH) success\n", AUD_PIN_HOOK_BTN);
    }

    // Set headset_detect pin as wake up pin
    ret = set_irq_wake(hi->irq, 1);
    if (ret < 0)
        goto err_request_input_dev;

    printk(KERN_INFO "aud:	gpio %d is %d, enable gpio%d wake up pin\n", aud_pin_headset_det, gpio_get_value(aud_pin_headset_det), aud_pin_headset_det);

    if (gpio_get_value(aud_pin_headset_det) == HS_PLUG_IN) 
	{
        if(AUD_HOOK_BTN)
		{
            ret = set_irq_wake(hi->irq_btn, 1);
            if (ret < 0)
                goto err_request_input_dev;
        }
    }
    else 
	{
        if(AUD_HOOK_BTN)
		{
            if (ret < 0)
                goto err_request_input_dev;
        }
    }
/* FIH, TerryChen  , 2011/08/25 { */
/* Add the headset detection when booting */
    H2W_DBG("Perform initial detection\n");
    if(AUD_HOOK_BTN)
    {
        detection_work(&g_detection_work);  
    }
/*} FIH, TerryChen , 2011/08/25 */       

    //aud_hs_dump_reg(); //seven+
    return 0;

    err_register_input_dev:
    if(AUD_HOOK_BTN)
	{
        printk(KERN_ERR "aud_hs: err_register_input_dev\n");
        input_free_device(hi->input);
    }


    err_register_hs_input_dev:
    printk(KERN_ERR "aud_hs: err_register_hs_input_dev\n");
    input_free_device(hi->hs_input);


    err_request_input_dev:
    if(AUD_HOOK_BTN)
	{
        printk(KERN_ERR "aud_hs: err_request_input_dev\n");
        free_irq(hi->irq_btn, 0);
    }
    err_request_h2w_headset_button_irq:
    printk(KERN_ERR "aud_hs: request_h2w_headset_button_irq\n");
    free_irq(hi->irq, 0);
    err_request_detect_irq:
    err_get_button_irq_num_failed:
    err_get_h2w_detect_irq_num_failed:
    err_set_button_gpio:
    err_set_detect_gpio:
    printk(KERN_ERR "aud_hs: AUD_PIN_HOOK_BTN, gpio/irq error\n");
    if(AUD_HOOK_BTN)
	{
        gpio_free(AUD_PIN_HOOK_BTN);       
    }
    err_request_button_gpio:
    printk(KERN_ERR "aud_hs: err_request_button_gpio\n");
    gpio_free(aud_pin_headset_det);  
    err_request_detect_gpio:
    printk(KERN_ERR "aud_hs: err_request_detect_gpio\n");
    destroy_workqueue(g_detection_work_queue);
    err_create_work_queue:
    printk(KERN_ERR "aud_hs: err_create_work_queue\n");    
    switch_dev_unregister(&hi->sdev);
    err_switch_dev_register:
    printk(KERN_ERR "aud_hs: Failed to register driver\n");

    return ret;
}

static int trout_h2w_remove(struct platform_device *pdev)
{
    H2W_DBG("");
    if (switch_get_state(&hi->sdev))
        remove_headset();
    if(AUD_HOOK_BTN)
	{
        input_unregister_device(hi->input);
        gpio_free(AUD_PIN_HOOK_BTN);       
    }
    gpio_free(aud_pin_headset_det);      
    if(AUD_HOOK_BTN)
	{
        free_irq(hi->irq_btn, 0);
    }
    free_irq(hi->irq, 0);
    destroy_workqueue(g_detection_work_queue);
    switch_dev_unregister(&hi->sdev);

    return 0;
}

static struct platform_device trout_h2w_device = {
    .name       = "headset_sensor",
};

static struct platform_driver trout_h2w_driver = {
    .probe      = trout_h2w_probe,
    .remove     = trout_h2w_remove,
    .driver     = {
        .name       = "headset_sensor",
        .owner      = THIS_MODULE,
    },
};
//IRM
int trout_h2w_init_v1(void* hdl)
{
    AUD_HOOK_BTN = true;

    printk(KERN_INFO"aud_hs: trout_h2w_init_v1, AUD_HOOK_BTN = %d\n", AUD_HOOK_BTN);

    return 0;
}

//TBP & TINQ
int trout_h2w_init_v2(void* hdl)
{
    AUD_HOOK_BTN = false;

    printk(KERN_INFO"aud_hs: trout_h2w_init_v2, AUD_HOOK_BTN = %d\n", AUD_HOOK_BTN);

    return 0;
}

/*Vincent added for ITV HooKKey : PMIC  Headset detect:1  B*/
int trout_h2w_init_v3(void* hdl)
{
    AUD_HOOK_BTN = false;
    HS_PLUG_IN = 1;

    printk(KERN_INFO"aud_hs: trout_h2w_init_v3, AUD_HOOK_BTN = %d\n", AUD_HOOK_BTN);

    return 0;
}
/*Vincent added for ITV HooKKey : PMIC  Headset detect:1  E*/

int trout_h2w_init_v4(void* hdl)
{
//++FIH-NJ-BSP-SW4 Simon NPM
    AUD_HOOK_BTN = false;    
//--FIH-NJ-BSP-SW4 Simon NPM 
    IRQF_TRIGGER_HS_INSERTED = IRQF_TRIGGER_LOW;
    aud_pin_headset_det = 39;


    printk(KERN_INFO"aud_hs: trout_h2w_init_v4, AUD_HOOK_BTN = %d\n", AUD_HOOK_BTN);

    return 0;
}

int __init headset_init(void)
{
    printk(KERN_INFO"[%s] entry\n", __func__);

    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_HEADSET, 
                                     HEADSET_V1), 
                   "HEADSET_V1", 
                   trout_h2w_init_v1, 
                   __FILE__, __LINE__);

    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_HEADSET, 
                                     HEADSET_V2), 
                   "HEADSET_V2", 
                   trout_h2w_init_v2, 
                   __FILE__, __LINE__);

    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_HEADSET, 
                                     HEADSET_V3), 
                   "HEADSET_V3", 
                   trout_h2w_init_v3, 
                   __FILE__, __LINE__);

   add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_HEADSET, 
                                     HEADSET_V4), 
                   "HEADSET_V4", 
                   trout_h2w_init_v4, 
                   __FILE__, __LINE__);
    return 0;
}

static int __init trout_h2w_init(void) {

    int ret;
    ret = platform_driver_register(&trout_h2w_driver);
    if (ret)
        return ret;
    return platform_device_register(&trout_h2w_device);
}

static void __exit trout_h2w_exit(void)
{
	platform_device_unregister(&trout_h2w_device);
	platform_driver_unregister(&trout_h2w_driver);
}
module_init(trout_h2w_init);
module_exit(trout_h2w_exit);

MODULE_AUTHOR("Seven Lin <sevenlin@fihspec.com>");
MODULE_DESCRIPTION("FIH headset detection driver");
MODULE_LICENSE("Proprietary");
