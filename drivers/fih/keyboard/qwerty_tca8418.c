/*
 * tca8418_keypad.c - TCA8418 Key Switch Controller Driver
 *
 * Copyright (C) 2009 Samsung Electronics
 * Kim Kyuwon <q1.kim@samsung.com>
 *
 * Based on pxa27x_keypad.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Datasheet: http://www.maxim-ic.com/quick_view2.cfm/qv_pk/5456
 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <fih/dynloader.h>

#define TCA8418_MAX_KEY_ROWS	8
#define TCA8418_MAX_KEY_COLS	8
#define TCA8418_MAX_KEY_NUM	(TCA8418_MAX_KEY_ROWS * TCA8418_MAX_KEY_COLS)
#define TCA8418_ROW_SHIFT	3


//TCA8418 reg
#define     TCA8418_KEY_INT_STAT       0x02
#define     TCA8418_KEY_LCK_EC         0x03
#define     TCA8418_KEY_EVENT_A        0x04
#define     TCA8418_KEY_LCK_TIMER      0x0E
#define     TCA8418_KEY_UNLOCK_KEY1    0x0F
#define     TCA8418_KEY_UNLOCK_KEY2    0x10
#define     TCA8418_KEY_CFG_REG        0x01
#define     TCA8418_KEY_GPIO1_REG      0x1D
#define     TCA8418_KEY_GPIO2_REG      0x1E


#define TCA8418_GPIO_KEY_IRQ    94  //for qwerty interrupt pin
#define TCA8418_GPIO_KEY_RESET  93  //for qwerty reset pin

#define DEBUG

static void tca8418_qwerty_keypad_reset(void);



struct tca8418_keypad {
	/* matrix key code map */
	unsigned short keycodes[TCA8418_MAX_KEY_NUM];

	struct input_dev *input_dev;
	struct i2c_client *client;
	struct work_struct kbd_irqwork;
};

static struct tca8418_keypad *tca8418_key;

static int tca8418_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret = i2c_smbus_write_byte_data(client, reg, val);

	if (ret < 0)
		dev_err(&client->dev, "%s: reg 0x%x, val 0x%x, err %d\n",
			__func__, reg, val, ret);
	return ret;
}

static int tca8418_read_reg(struct i2c_client *client, int reg)
{
	int ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0)
		dev_err(&client->dev, "%s: reg 0x%x, err %d\n",
			__func__, reg, ret);
	return ret;
}

static void tca8418_build_keycode(struct tca8418_keypad *keypad,
				const struct matrix_keymap_data *keymap_data)
{
	struct input_dev *input_dev = keypad->input_dev;
	int i;

	pr_err("tca8418_build_keycode1....size= %d \n",keymap_data->keymap_size);
   
	for (i = 0; i < keymap_data->keymap_size; i++) {
		unsigned int key = keymap_data->keymap[i];
		unsigned int row = KEY_ROW(key);
		unsigned int col = KEY_COL(key);
		unsigned int scancode = MATRIX_SCAN_CODE(row, col,
						TCA8418_ROW_SHIFT);
		unsigned short keycode = KEY_VAL(key);

		keypad->keycodes[scancode] = keycode;
        	pr_err("tca8418_build_keycode2....key= %x \n",key);
        	pr_err("tca8418_build_keycode3....r= %d  c= %d\n",row,col);
        	pr_err("tca8418_build_keycode4....s=%d  keycode = %d\n",scancode,keycode);

		__set_bit(keycode, input_dev->keybit);
	}
	__clear_bit(KEY_RESERVED, input_dev->keybit);
}   

static void kbd_work(struct work_struct *work)
{
	struct tca8418_keypad *keypad = tca8418_key;
    	struct input_dev *input_dev = keypad->input_dev;
	//unsigned int val,row, col, release,code;
    	unsigned int release,code;
    	unsigned int key_clk_ec = 0;
    	unsigned int key_counter = 0;
    	unsigned int read_val = 0;
    	unsigned int stat;

    	stat = tca8418_read_reg(keypad->client, TCA8418_KEY_INT_STAT);
    	pr_err("tca8418_interrupt......%x.., stat %d\n",  0, stat);

handler:
    	if ( (stat&0x01)== 0x01 )
    	{
        	key_counter = 0;
	        key_clk_ec = tca8418_read_reg(keypad->client, TCA8418_KEY_LCK_EC);
	        key_counter = key_clk_ec&0x0F;

        	pr_err("tca8418_interrupt3......ce = %x.  co = %d.\n",key_clk_ec, key_counter);

	        while ( key_counter-- )
        	{
            		read_val = tca8418_read_reg(keypad->client, TCA8418_KEY_EVENT_A);
			pr_err("tca8418_interrupt4......%x\n",read_val);

            		code = read_val & 0x7f;
            		release = read_val >> 7;

 			pr_err("tca8418_interrupt5...key[%d] = %d, %s\n", code, keypad->keycodes[code], release ? "press":"release");
//            input_event(input_dev, EV_MSC, MSC_SCAN, code);
            		input_report_key(input_dev, keypad->keycodes[code], release);
            		input_sync(input_dev);
        	}
    	}

    	tca8418_write_reg(keypad->client, TCA8418_KEY_INT_STAT, stat);
    
    	stat = tca8418_read_reg(keypad->client, TCA8418_KEY_INT_STAT);
    	pr_err("tca8418_interrupt, after write, stat %d\n", stat);
    	if(stat != 0)
		goto handler;
}

/* runs in an IRQ thread -- can (and will!) sleep */
static irqreturn_t tca8418_interrupt(int irq, void *dev_id)
{
	struct tca8418_keypad *keypad = dev_id;

	if(keypad != NULL)
		schedule_work(&keypad->kbd_irqwork);
	return IRQ_HANDLED;
}


static int tca8418_open(struct input_dev *dev)
{
//	struct tca8418_keypad *keypad = input_get_drvdata(dev);

	return 0;
}

static void tca8418_close(struct input_dev *dev)
{
	return;
}

static void tca8418_qwerty_keypad_reset(void)
{       

    	pr_err("tca8418_qwerty_keypad_reset....");
 
	gpio_set_value(TCA8418_GPIO_KEY_RESET, 1);
    	msleep(10);

    	gpio_set_value(TCA8418_GPIO_KEY_RESET, 0);
    	msleep(10);

    	gpio_set_value(TCA8418_GPIO_KEY_RESET, 1);
    	msleep(10);
}

static int tca8148_initialize(struct i2c_client *client)
{
    	unsigned char write_count = 0;
    	unsigned char config_reg = 0;
    	unsigned char kp_gpio_select1 = 0;
    	unsigned char kp_gpio_select2 = 0;
    	int dataOut = 0;
    	int ret;

    	do
    	{
        	ret = 0;
	        dataOut = 0;
        	config_reg = 0x29;
	        ret = tca8418_write_reg(client, TCA8418_KEY_CFG_REG, config_reg);//config register
        	dataOut = tca8418_read_reg(client, TCA8418_KEY_CFG_REG);

        	if (config_reg == dataOut)
          		break;
        	else
			if ( write_count == 9 )
        	        	return -1;

		msleep(10);
        	write_count++;
    	} while (write_count < 10);

    	msleep(10);

    	//KEY_GPIO1_REG
    	write_count = 0;
    	do
    	{
        	ret = 0;
        	dataOut = 0;
	        kp_gpio_select1 = 0xFF;
        
        	ret = tca8418_write_reg(client, TCA8418_KEY_GPIO1_REG, kp_gpio_select1);//config register
       		dataOut = tca8418_read_reg(client, TCA8418_KEY_GPIO1_REG);

        	if (kp_gpio_select1 == dataOut)
			break;
        	else
            		if ( write_count == 9 )
                		return -1;

        	msleep(10);
        	write_count++;
    	} while (write_count < 10);
   
    	msleep(10);

    	//KEY_GPIO2_REG
    	write_count = 0;
    	do
    	{
        	ret = 0;
        	dataOut = 0;
        	kp_gpio_select2 = 0xFF;
        
        	ret = tca8418_write_reg(client, TCA8418_KEY_GPIO2_REG, kp_gpio_select2);//config register
        	dataOut = tca8418_read_reg(client, TCA8418_KEY_GPIO2_REG);

        	if (kp_gpio_select2 == dataOut)
          		break;
        	else
            		if ( write_count == 9 )
		                return -1;

        	msleep(10);
        	write_count++;
    	} while (write_count < 10);
      	msleep(10);

    	return 0;
}


static int __devinit tca8418_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	const struct matrix_keymap_data *keymap_data = client->dev.platform_data;
	struct tca8418_keypad *keypad;
	struct input_dev *input_dev;
	//int ret;
	int error;
    	int irqrequest = 0;

	if (!client->irq) {
		dev_err(&client->dev, "The irq number should not be zero\n");
		return -EINVAL;
	}

    /* Initialize TCA8418 */
    	tca8418_qwerty_keypad_reset();
    	error = tca8148_initialize(client);
	if(error < 0)
		return -EINVAL;

	keypad = kzalloc(sizeof(struct tca8418_keypad), GFP_KERNEL);
	input_dev = input_allocate_device();

    	keypad->client = client;
    	keypad->input_dev = input_dev;
    	input_dev->name = client->name;
	input_dev->id.bustype = BUS_I2C;
	input_dev->open = tca8418_open;
	input_dev->close = tca8418_close;
	input_dev->dev.parent = &client->dev;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_REP);
//	input_dev->evbit[0] = BIT_MASK(EV_KEY);
	input_dev->keycodesize = sizeof(keypad->keycodes[0]);
	input_dev->keycodemax = ARRAY_SIZE(keypad->keycodes);
	input_dev->keycode = keypad->keycodes;

	input_set_capability(input_dev, EV_MSC, MSC_SCAN);
    	input_set_drvdata(input_dev, keypad); 
    
    	INIT_WORK(&keypad->kbd_irqwork, kbd_work);

    	tca8418_build_keycode(keypad, keymap_data);
	// Register the input device
    	error = input_register_device(input_dev);
 
    	i2c_set_clientdata(client, keypad);

    	device_init_wakeup(&client->dev, false);

    	tca8418_key = keypad;

    	error = request_threaded_irq(client->irq,  NULL, tca8418_interrupt,
				     IRQF_TRIGGER_FALLING/* | IRQF_ONESHOT*/,
				     client->name, keypad);
	if(error)
		goto failed;
    	irqrequest = 1;
    	return error;

failed:
	input_free_device(input_dev);
	kfree(keypad);
	input_unregister_device(input_dev);

    	if (client->irq && irqrequest){
        	free_irq(client->irq, keypad);
    	}

	return -EINVAL;
}

static int __devexit tca8418_remove(struct i2c_client *client)
{
	struct tca8418_keypad *keypad = i2c_get_clientdata(client);

	flush_work(&keypad->kbd_irqwork);
	free_irq(client->irq, keypad);
	input_unregister_device(keypad->input_dev);
	kfree(keypad);

	return 0;
}

#ifdef CONFIG_PM
static int tca8418_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int stat;
	int rc;
	
	printk("%s\n", __FUNCTION__);
        tca8418_write_reg(client, TCA8418_KEY_CFG_REG, 0x0);//config register
	
    	stat = tca8418_read_reg(client, TCA8418_KEY_INT_STAT);
	printk("%s, stat is %d\n", __FUNCTION__, stat);

	disable_irq(client->irq);

 	rc = cancel_work_sync(&tca8418_key->kbd_irqwork);
	

	
	if (device_may_wakeup(&client->dev))
		enable_irq_wake(client->irq);

	return 0;
}

static int tca8418_resume(struct i2c_client *client)
{
	printk("%s\n", __FUNCTION__);

        tca8418_write_reg(client, TCA8418_KEY_CFG_REG, 0x29);//config register
      
        enable_irq(client->irq);
	
	if (device_may_wakeup(&client->dev))
		disable_irq_wake(client->irq);

	return 0;
}
#else
#define tca8418_suspend	NULL
#define tca8418_resume	NULL
#endif

static ssize_t tca8418_proc_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	unsigned int val;
	
	if (tca8418_key != NULL) {
		val = tca8418_read_reg(tca8418_key->client, TCA8418_KEY_INT_STAT);
		printk("%s, read_val %d\n", __FUNCTION__, val);
		if(copy_to_user(buf, &val, sizeof(unsigned int)))
			return -EFAULT;
	}
	return 0;
}

static ssize_t tca8418_proc_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	unsigned int val;
	
        if (copy_from_user(&val, buf, sizeof(unsigned int)))
                return -EFAULT;

	if (tca8418_key != NULL) {
		tca8418_write_reg(tca8418_key->client, TCA8418_KEY_INT_STAT, val);
		printk("%s, write val %d\n", __FUNCTION__, val);
	}
	return 0;
}

static const struct i2c_device_id tca8418_ids[] = {
	{"tca8418-keypad", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, tca8418_ids);

static struct i2c_driver tca8418_i2c_driver = {
		.driver = {
		.name = "tca8418-keypad",
	},
	.probe		= tca8418_probe,
	.remove		= __devexit_p(tca8418_remove),
	.suspend	= tca8418_suspend,
	.resume		= tca8418_resume,
	.id_table	= tca8418_ids,
};

static const struct file_operations tca8418_proc_fileops = {
        .owner          = THIS_MODULE,
        .read           = tca8418_proc_read,
	.write		= tca8418_proc_write,
};


static int __init tca8418_init(void *hdl)
{
        struct proc_dir_entry *entry;
	entry = proc_create("tca8418-keypad", 0, NULL, &tca8418_proc_fileops);	
	return ADD_DYN_I2C_DRIVER(hdl, &tca8418_i2c_driver);
}
DECLARE_DYN_LOADER(DRIVER_TYPE_QWERTY, FIH_QWERTY_KEYPAD_V1, tca8418_init, LEVEL6);

static void __exit tca8418_exit(void)
{
	i2c_del_driver(&tca8418_i2c_driver);
	remove_proc_entry("tca8418-keypad", NULL);
}
module_exit(tca8418_exit);

MODULE_AUTHOR("<foxconn.com>");
MODULE_DESCRIPTION("TCA8418 Key Switch Controller Driver");
MODULE_LICENSE("GPL v2");
