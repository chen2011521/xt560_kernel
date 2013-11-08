#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <fih/sc668.h>
#include <linux/delay.h>
#include <mach/gpio.h>

#include <fih/dynloader.h>

struct sc668_driver_data {
	struct i2c_client *client;
    struct led_classdev led_dev[SC668_LED_MAX];
    int target_time;
    int start_time;
    int blink_once;
    u8 bl_enable;
	u8 en_pin;
	u8 pwm_pin;
	u8 chg_led_ctrl_pin;
}*sc668_dd;

static int sc668_read(struct sc668_driver_data* dd, u8 cmd, u8 *data, 
							int length)
{
    int ret;
    
    struct i2c_msg msgs[] = {
        [0] = {
            .addr   = dd->client->addr,
            .flags  = 0,
            .buf    = (void *)&cmd,
            .len    = 1
        },
        [1] = {
            .addr   = dd->client->addr,
            .flags  = I2C_M_RD,
            .buf    = (void *)data,
            .len    = length
        }
    };

    ret = i2c_transfer(dd->client->adapter, msgs, 2);
    return (ret < 0) ? -1 : 0;    
}

static int sc668_write(struct sc668_driver_data* dd, u8 *cmd,
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

static int sc668_pins_config(struct sc668_driver_data* dd)
{
	int rc = 0;
	
	rc = gpio_request(dd->en_pin, "SC668 EN");
	if (rc < 0) {
		dev_err(&dd->client->dev, "failed to request %d GPIO\n", 
					dd->en_pin);
		return rc;
	}
	gpio_direction_output(dd->en_pin, 0);
	
	rc = gpio_request(dd->pwm_pin, "SC668 PWN");
	if (rc < 0) {
		dev_err(&dd->client->dev, "failed to request %d GPIO\n",
					dd->pwm_pin);
		return rc;
	}
	gpio_direction_output(dd->pwm_pin, 1);
	
	rc = gpio_request(dd->chg_led_ctrl_pin, "CHG LED CTRL");
	if (rc < 0) {
		dev_err(&dd->client->dev, "failed to request %d GPIO\n",
					dd->chg_led_ctrl_pin);
		return rc;
	}
	gpio_direction_output(dd->chg_led_ctrl_pin, 1);
	
	return 0;	
}

static void sc668_pins_release(struct sc668_driver_data* dd)
{
	gpio_free(dd->en_pin);
	gpio_free(dd->pwm_pin);
	gpio_free(dd->chg_led_ctrl_pin);
}

static void sc668_chip_enable(struct sc668_driver_data* dd, bool enable)
{
	gpio_set_value_cansleep(dd->en_pin, enable);
	udelay(950); /* necessary delay time before I2C communication */
}

static int sc668_chip_config(struct sc668_driver_data* dd)
{
	u8 cmd[2] = {0x00, 0x00};
	int rc = 0;

	/* Pull EN pin to HI */
	sc668_chip_enable(dd, true);
	
	/* 
	 * Bank Enable 
	 * Bank1: Enabled
	 * Bank2: Enabled
	 * Bank3: Enabled
	 * Bank4: Enabled
	 */
	cmd[0] = SC668_BANKEN;
	cmd[1] = 0x1F;
	rc = sc668_write(dd, cmd, 2);
	if (rc < 0) {
		dev_err(&dd->client->dev, "failed to enable banks\r\n");
		return rc;
	}
	
	/*
	 * All backlight disable
	 * BL1: Red LED
	 * BL2: Green LED
	 * BL3: Blue LED
	 * BL4: TP LED
	 * BL5: TP LED
	 */
	cmd[0] = SC668_BLEN;
	cmd[1] = 0x00;
	rc = sc668_write(dd, cmd, 2);
	if (rc < 0) {
		dev_err(&dd->client->dev, "failed to enable channels\r\n");
		return rc;
	}
	rc = sc668_read(dd, cmd[0], &cmd[1], 1);
	if (rc < 0) {
		dev_err(&dd->client->dev, "failed to read SC668_BLEN(init)\r\n");
		return rc;
	}
	dd->bl_enable = cmd[1];
	
	/* 
	 * Bank and Group Assignment 
	 * Bank1: TP Backlight
	 * Bank2: Blue LED
	 * Bank3: Green LED
	 * Bank4: Red LED
	 * 
	 * Group1: Bank1, Bank2, Bank3
	 * Group2: Bank4
	 */
	cmd[0] = SC668_LIGHTING_EFFECT;
	cmd[1] = SC668_GRP_B123_B4 | SC668_BANK_BL48_BL3_BL2_BL1;
	rc = sc668_write(dd, cmd, 2);
	if (rc < 0) {
		dev_err(&dd->client->dev, "failed to set banks and groups\n");
		return rc;
	}
	
	/* 
	 * Initial Effect Rates 
	 * Group1: Breath 4ms/step, Fade 1ms/Step
	 * Group2: Breath 4ms/step, Fade 1ms/Step
	 */
	cmd[0] = SC668_EFFECT_RATES;
	cmd[1] = SC668_EFFRATES_B16F4MS | 
				(SC668_EFFRATES_B16F4MS << SC668_EFFECT_RATES_GRP2);
	rc = sc668_write(dd, cmd, 2);
	if (rc < 0) {
		dev_err(&dd->client->dev, "failed to set effect rates\n");
		return rc;
	}
				
	/* 
	 * Initial Target and Start Times
	 * Group1: Target Time: 1024ms, Start Time 1024ms
	 * Gropu2: Target Time: 1024ms, Start Time 1024ms
	 */
	cmd[0] = SC668_TARGET_START_TIMES_G1;
	cmd[1] = dd->start_time | 
				(dd->target_time << SC668_TARGET_START_TIMES_TT0);
	rc = sc668_write(dd, cmd, 2);
	if (rc < 0) {
		dev_err(&dd->client->dev, "failed to set target and start time for G1\n");
		return rc;
	}
				
	cmd[0] = SC668_TARGET_START_TIMES_G2;
	cmd[1] = dd->start_time | 
				(dd->target_time << SC668_TARGET_START_TIMES_TT0);
	rc = sc668_write(dd, cmd, 2);
	if (rc < 0) {
		dev_err(&dd->client->dev, "failed to set target and start time for G2\n");
		return rc;
	}
	
	return 0;
}

static
void sc668_backlight_enable(int bank, bool enable)
{
	u8 cmd[2] = {SC668_BLEN, 0};
	int rc = 0;
	
	rc = sc668_read(sc668_dd, cmd[0], &cmd[1], 1);
	if (rc < 0)
		dev_err(&sc668_dd->client->dev, "read SC668_BLEN failed\r\n");
	else
		dev_info(&sc668_dd->client->dev, "SC668_BLEN 0x%02x[1]\r\n", cmd[1]);
		
	switch(bank) {
	case SC668_B1:
		enable ? (cmd[1] |= 0x18) : (cmd[1] &= 0x07);
		break;
	case SC668_B2:
		enable ? (cmd[1] |= 0x04) : (cmd[1] &= 0x1B);
		break;
	case SC668_B3:
		enable ? (cmd[1] |= 0x02) : (cmd[1] &= 0x1D);
		break;
	case SC668_B4:
		enable ? (cmd[1] |= 0x01) : (cmd[1] &= 0x1E);
	}
	
	rc = sc668_write(sc668_dd, cmd, 2);
	if (rc < 0)
		dev_err(&sc668_dd->client->dev, "write SC668_BLEN failed\r\n");

	rc = sc668_read(sc668_dd, cmd[0], &cmd[1], 1);
	if (rc < 0)
		dev_err(&sc668_dd->client->dev, "read SC668_BLEN failed\r\n");
	else
		dev_info(&sc668_dd->client->dev, "SC668_BLEN 0x%02x[2]\r\n", cmd[1]);
		
	sc668_dd->bl_enable = cmd[1];
}

static 
void sc668_led_set_fade_current(int bank, int bl_current)
{
	u8 cmd[2] = {0, 0};
	int rc = 0;
			
	cmd[0] = SC668_BLINKEN_B1 + bank;
	cmd[1] = 0x00;
	rc = sc668_write(sc668_dd, cmd, 2);
	if (rc < 0)
		dev_err(&sc668_dd->client->dev, "%s: failed to set [register 0x%02x, data 0x%02x]\n",
					__func__, cmd[0], cmd[1]);
		
	cmd[0] = SC668_FADEEN_B1 + bank;
	cmd[1] = 0x20 | bl_current;
	rc = sc668_write(sc668_dd, cmd, 2);
	if (rc < 0)
		dev_err(&sc668_dd->client->dev, "%s: failed to set [register 0x%02x, data 0x%02x]\n",
					__func__, cmd[0], cmd[1]);
					
	sc668_backlight_enable(bank, bl_current?true:false);
}

static
void sc668_led_set_breath_current(int bank, int bl_current,
										int start_time, int target_time)
{
	u8 cmd[2] = {0, 0};
	int rc = 0;
	
	/* Set Target and Start Times */
	cmd[0] = (bank == SC668_B4)?SC668_TARGET_START_TIMES_G2:SC668_TARGET_START_TIMES_G1;
	cmd[1] = start_time | 
				(target_time << SC668_TARGET_START_TIMES_TT0);
	rc = sc668_write(sc668_dd, cmd, 2);
	if (rc < 0)
		dev_err(&sc668_dd->client->dev, "failed to set target and start time for bank %d\n", 
					bank + 1);
					
	cmd[0] = SC668_FADEEN_B1 + bank;
	cmd[1] = 0x20;
	rc = sc668_write(sc668_dd, cmd, 2);
	if (rc < 0)
		dev_err(&sc668_dd->client->dev, "%s: failed to set [register 0x%02x, data 0x%02x]\n",
					__func__, cmd[0], cmd[1]);
					
	cmd[0] = SC668_BLINKEN_B1 + bank;
	cmd[1] = 0x20 | bl_current;
	rc = sc668_write(sc668_dd, cmd, 2);
	if (rc < 0)
		dev_err(&sc668_dd->client->dev, "%s: failed to set [register 0x%02x, data 0x%02x]\n",
					__func__, cmd[0], cmd[1]);
					
	sc668_backlight_enable(bank, bl_current?true:false);
}

static void sc668_led_brightness_set(struct led_classdev *led_cdev,
                   enum led_brightness brightness)
{
    int idx = 0;
    
    if (!strcmp(led_cdev->name, "red"))
        idx = SC668_LED_RED;
    else if (!strcmp(led_cdev->name, "green"))
        idx = SC668_LED_GREEN;
    else if (!strcmp(led_cdev->name, "blue"))
        idx = SC668_LED_BLUE;
    else if (!strcmp(led_cdev->name, "button-backlight"))
        idx = SC668_LED_TP;

    dev_info(led_cdev->dev, "%s: IDX[%d] %s\n", __func__, idx, brightness?"ON":"OFF");		
    
    switch (idx) {
	case SC668_LED_RED:
		sc668_led_set_fade_current(SC668_B4,
									brightness?SC668_BLCURR_10MA:SC668_BLCURR_0MA);
		break;
	case SC668_LED_GREEN:
		sc668_led_set_fade_current(SC668_B3,
									brightness?SC668_BLCURR_5MA:SC668_BLCURR_0MA);
		break;
	case SC668_LED_BLUE:
		sc668_led_set_fade_current(SC668_B2,
									brightness?SC668_BLCURR_10MA:SC668_BLCURR_0MA);
		break;
	case SC668_LED_TP:
		sc668_led_set_fade_current(SC668_B1,
									brightness?SC668_BLCURR_10MA:SC668_BLCURR_0MA);
	}	
}

static ssize_t sc668_blink_solid_store(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t size)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    int idx = 0;
    int on = 0;
    
    if (!strcmp(led_cdev->name, "red"))
        idx = SC668_LED_RED;
    else if (!strcmp(led_cdev->name, "green"))
        idx = SC668_LED_GREEN;
    else if (!strcmp(led_cdev->name, "blue"))
        idx = SC668_LED_BLUE;
    else if (!strcmp(led_cdev->name, "button-backlight"))
        idx = SC668_LED_TP;
        
    sscanf(buf, "%d", &on);
    dev_info(led_cdev->dev, "%s: IDX[%d] %s\n", __func__, idx, on?"BLINK":"STOP");
    
    switch (idx) {
	case SC668_LED_RED:
		sc668_led_set_breath_current(SC668_B4,
										on?SC668_BLCURR_5MA:SC668_BLCURR_0MA,
										sc668_dd->start_time,
										sc668_dd->target_time
									);
		break;
	case SC668_LED_GREEN:
		sc668_led_set_breath_current(SC668_B3,
										on?SC668_BLCURR_5MA:SC668_BLCURR_0MA,
										sc668_dd->start_time,
										sc668_dd->target_time
									);
		break;
	case SC668_LED_BLUE:
		sc668_led_set_breath_current(SC668_B2,
										on?SC668_BLCURR_5MA:SC668_BLCURR_0MA,
										sc668_dd->start_time,
										sc668_dd->target_time
									);
		break;
	case SC668_LED_TP:
		sc668_led_set_breath_current(SC668_B1,
										on?SC668_BLCURR_5MA:SC668_BLCURR_0MA,
										sc668_dd->start_time,
										sc668_dd->target_time										
									);
	}
	
	return size;
}
static DEVICE_ATTR(blink, 0644, NULL, sc668_blink_solid_store);

static int sc668_get_proper_time(int ms)
{
		if (ms <= 32)
			return SC668_TIME_32MS;
		if (ms <= 64)
			return SC668_TIME_64MS;
		if (ms <= 256 + 128)
			return SC668_TIME_256MS;
		if (ms <= 512 + 128)
			return SC668_TIME_512MS;
		if (ms <= 1024 + 512)
			return SC668_TIME_1024MS;
		if (ms <= 2048 + 512)
			return SC668_TIME_2048MS;
		if (ms <= 3072 + 512)
			return SC668_TIME_3072MS;
		
		return SC668_TIME_4096MS;
}

static ssize_t sc668_target_time_ms_store(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t size)
{
    sscanf(buf, "%d", &sc668_dd->target_time);
    dev_info(dev, "%s: Target Time[1] %d ms\n", __func__, sc668_dd->target_time);
    
    sc668_dd->target_time = sc668_get_proper_time(sc668_dd->target_time);
    dev_info(dev, "%s: Target Time[2] %d\n", __func__, sc668_dd->target_time);

    return size;
}
static DEVICE_ATTR(ledon, 0644, NULL, sc668_target_time_ms_store);

static ssize_t sc668_start_time_ms_store(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t size)
{
    sscanf(buf, "%d", &sc668_dd->start_time);
    dev_info(dev, "%s: Start Time[1] %d ms\n", __func__, sc668_dd->start_time);
    
    sc668_dd->start_time = sc668_get_proper_time(sc668_dd->start_time);
    dev_info(dev, "%s: Start Time[2] %d\n", __func__, sc668_dd->start_time);    

    return size;
}
static DEVICE_ATTR(ledoff, 0644, NULL, sc668_start_time_ms_store);

static ssize_t sc668_blink_once_store(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t size)
{
    sscanf(buf, "%d", &sc668_dd->blink_once);
    dev_info(dev, "%s: Blink once\n", __func__);

    return size;
}
static DEVICE_ATTR(blink_once, 0644, NULL, sc668_blink_once_store);

static int sc668_led_probe(struct i2c_client *client, 
							const struct i2c_device_id *id)
{
	struct sc668_platform_data *pd = client->dev.platform_data;
	int ret = -ENODEV;
	int i = 0;
	
	sc668_dd = kzalloc(sizeof(struct sc668_driver_data), GFP_KERNEL);
	if (!sc668_dd) {
		ret = -ENOMEM;
		goto err;
	}
	
	sc668_dd->start_time	= SC668_TIME_1024MS;
	sc668_dd->target_time	= SC668_TIME_1024MS;
	sc668_dd->blink_once	= 0;
	sc668_dd->bl_enable		= 0;
	sc668_dd->client		= client;
	sc668_dd->en_pin		= pd->en_pin;
	sc668_dd->pwm_pin		= pd->pwm_pin;
	sc668_dd->chg_led_ctrl_pin = pd->chg_led_ctrl_pin;
	
	if (sc668_pins_config(sc668_dd))
		goto err_conf_pins;
		
	if (sc668_chip_config(sc668_dd))
		goto err_conf_chip;
	
	for (i = 0; i < SC668_LED_MAX; i++) {
		switch (i) {
		case SC668_LED_RED:
			sc668_dd->led_dev[i].name = "red";
			break;
		case SC668_LED_GREEN:
			sc668_dd->led_dev[i].name = "green";
			break;
		case SC668_LED_BLUE:
			sc668_dd->led_dev[i].name = "blue";
			break;
		case SC668_LED_TP:
			sc668_dd->led_dev[i].name = "button-backlight";
		}
		
		sc668_dd->led_dev[i].brightness_set = sc668_led_brightness_set;

        ret = led_classdev_register(&sc668_dd->client->dev, &sc668_dd->led_dev[i]);
        if (ret) {
            dev_err(&sc668_dd->client->dev,
                        "%s: led_classdev_register failed\n", __func__);
            goto err_register_led_dev;
        }
    
        dev_info(&sc668_dd->client->dev,
                "%s: LED name(%s) was registered\n",
                __func__, sc668_dd->led_dev[i].name);
	}
	
	for (i = 0; i < SC668_LED_MAX; i++) {
		ret = device_create_file(sc668_dd->led_dev[i].dev,
									&dev_attr_blink);
		if (ret) {
			dev_err(&sc668_dd->client->dev,
						"%s: device_create_file failed\n", __func__);
			goto err_register_blink_attr;
		}
    }
    
    ret = device_create_file(&sc668_dd->client->dev, &dev_attr_ledon);
    if (ret) {
        dev_err(&sc668_dd->client->dev,
               "%s: create dev_attr_ledon failed\n", __func__);
        goto err_register_blink_attr;
    }
    
    ret = device_create_file(&sc668_dd->client->dev, &dev_attr_ledoff);
    if (ret) {
        dev_err(&sc668_dd->client->dev,
               "%s: create dev_attr_ledoff failed\n", __func__);
        goto err_out_attr_ledoff;
    }
    
    ret = device_create_file(&sc668_dd->client->dev, &dev_attr_blink_once);
    if (ret) {
        dev_err(&sc668_dd->client->dev,
               "%s: create dev_attr_blink_once failed\n", __func__);
        goto err_out_attr_blink_once;
    }
		
	return 0;
	
err_out_attr_blink_once:
    device_remove_file(&sc668_dd->client->dev, &dev_attr_ledoff);
err_out_attr_ledoff:
    device_remove_file(&sc668_dd->client->dev, &dev_attr_ledon);
err_register_blink_attr:
	for (i = 0; i < SC668_LED_MAX; i++)
		device_remove_file(sc668_dd->led_dev[i].dev, &dev_attr_blink);
err_register_led_dev:
	for (i = 0; i < SC668_LED_MAX; i++)
		led_classdev_unregister(&sc668_dd->led_dev[i]);	
err_conf_chip:
	sc668_chip_enable(sc668_dd, false);
err_conf_pins:
	sc668_pins_release(sc668_dd);
	kfree(sc668_dd);
err:
	return ret;
}

static int __devexit sc668_led_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id sc668_idtable[] = {
	{"sc668-led", 0},
	{}
};

static int sc668_led_suspend(struct i2c_client *client, pm_message_t mesg)
{
	printk(KERN_INFO "KunFeng sc668_led_suspend");
	gpio_direction_output(sc668_dd->pwm_pin, 0);
	return 0;
}
static int sc668_led_resume(struct i2c_client *client)
{
	printk(KERN_INFO "KunFeng sc668_led_resume");
	gpio_direction_output(sc668_dd->pwm_pin, 1);
	return 0;
}

static struct i2c_driver sc668_led_driver = {
	.probe		= sc668_led_probe,
	.remove		= __devexit_p(sc668_led_remove),
	.suspend        = sc668_led_suspend,
	.resume         = sc668_led_resume,
	.id_table	= sc668_idtable,
	.driver		= {
		.name	= "sc668-led",
		.owner	= THIS_MODULE,
	},
};

#if 0
static int __init sc668_led_init(void)
{
	return i2c_add_driver(&sc668_led_driver);
}
module_init(sc668_led_init);
#else
static int __init sc668_led_init(void *hdl)
{
	return ADD_DYN_I2C_DRIVER(hdl, &sc668_led_driver);
}
DECLARE_DYN_LOADER(DRIVER_TYPE_LED, SC668_LED_V1, sc668_led_init, LEVEL6);
#endif

static void __exit sc668_led_exit(void)
{
	i2c_del_driver(&sc668_led_driver);
}
module_exit(sc668_led_exit);

MODULE_DESCRIPTION("SC668 LEDs driver");
