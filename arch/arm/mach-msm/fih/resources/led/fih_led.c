/**
* LED Driver
********************************************/
#include "fih_led.h"

// DIV2-SW2-BSP-IRM-LEDS+[
#ifdef CONFIG_LEDS_SC668
#include <fih/sc668.h>		// DIV2-SW2-BSP-IRM-LEDS

#define GPIO_SC668_EN		127
#define GPIO_SC668_PWM		9
#define GPIO_CHG_LED_CTRL	115

static struct sc668_platform_data sc668_pdata = {
	.en_pin				= GPIO_SC668_EN,
	.pwm_pin			= GPIO_SC668_PWM,
	.chg_led_ctrl_pin	= GPIO_CHG_LED_CTRL,
};

static int config_sc668_gpios(void* hdl) {
	int rc = 0;
	
	rc = gpio_tlmm_config( GPIO_CFG( GPIO_SC668_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA ), GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s not able to get gpio %d\n", __func__, GPIO_SC668_EN);

	rc = gpio_tlmm_config( GPIO_CFG( GPIO_SC668_PWM, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA ), GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s not able to get gpio %d\n", __func__, GPIO_SC668_PWM);
		
	rc = gpio_tlmm_config( GPIO_CFG( GPIO_CHG_LED_CTRL, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA ), GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s not able to get gpio %d\n", __func__, GPIO_CHG_LED_CTRL);

    return rc;
}

static struct i2c_board_info sc668_led_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("sc668-led", 0xE0 >> 1),//0x70
		.platform_data = &sc668_pdata,
	},
};
#endif	/* !CONFIG_LEDS_SC668 */

#ifdef CONFIG_LEDS_SN3193
#include <fih/sn3193.h>
#define GPIO_SN3193_EN_V1		5
#define GPIO_SN3193_EN_V2		127
#define GPIO_SN3193_EN_V3		96 //IRMPrime
#define GPIO_SN3193_CHG_LED_CTRL_V3		93  //IRMPrime  11/30 
#define GPIO_SN3193_CHG_LED_CTRL_V2		255
#define GPIO_SN3193_CHG_LED_CTRL_V1		255

static struct sn3193_platform_data sn3193_pdata_V1 = {
	.en_pin				= GPIO_SN3193_EN_V1,
    .chg_led_ctrl_pin	= GPIO_SN3193_CHG_LED_CTRL_V1,
};

static int config_sn3193_V1_gpios(void* hdl) {
	int rc = 0;
	
	rc = gpio_tlmm_config( GPIO_CFG( GPIO_SN3193_EN_V1, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA ), GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s not able to get gpio %d\n", __func__, GPIO_SN3193_EN_V1);
    return rc;
}

static struct i2c_board_info sn3193_led_V1_i2c_info[] __initdata = {
	{
         I2C_BOARD_INFO("sn3193-led", 0xd0 >> 1),
		.platform_data = &sn3193_pdata_V1,

	},
};
static struct sn3193_platform_data sn3193_pdata_V2 = {
	.en_pin				= GPIO_SN3193_EN_V2,
    .chg_led_ctrl_pin	= GPIO_SN3193_CHG_LED_CTRL_V2,
};

static int config_sn3193_V2_gpios(void* hdl) {
	int rc = 0;
	
	rc = gpio_tlmm_config( GPIO_CFG( GPIO_SN3193_EN_V2, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA ), GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s not able to get gpio %d\n", __func__, GPIO_SN3193_EN_V2);
    return rc;
}

static struct i2c_board_info sn3193_led_V2_i2c_info[] __initdata = {
	{
         I2C_BOARD_INFO("sn3193-led", 0xd0 >> 1),
		.platform_data = &sn3193_pdata_V2,

	},
};
/* IRMPrime begin ++*/
static struct sn3193_platform_data sn3193_pdata_V3 = {
	.en_pin				= GPIO_SN3193_EN_V3,
    .chg_led_ctrl_pin	= GPIO_SN3193_CHG_LED_CTRL_V3,
};

static int config_sn3193_V3_gpios(void* hdl) {
	int rc = 0;
	
	rc = gpio_tlmm_config( GPIO_CFG( GPIO_SN3193_EN_V3, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA ), GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s not able to get gpio %d\n", __func__, GPIO_SN3193_EN_V3);
    rc = gpio_tlmm_config( GPIO_CFG( GPIO_SN3193_CHG_LED_CTRL_V3, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA ), GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s not able to get gpio %d\n", __func__, GPIO_SN3193_CHG_LED_CTRL_V3);
    return rc;
}

static struct i2c_board_info sn3193_led_V3_i2c_info[] __initdata = {
	{
         I2C_BOARD_INFO("sn3193-led", 0xd0 >> 1),
		.platform_data = &sn3193_pdata_V3,

	},
};
/* IRMPrime begin --*/
#endif

#ifdef CONFIG_LEDS_PMIC8028_PROC
#include <fih/leds-pmic8028-proc.h>
static struct pmic8028_led pmic8028_leds[] = {
	[0] = {
		.name		= "red",
		.max_brightness = 255,
		.id		= PMIC8028_ID_LED_RED,
	},
	[1] = {
		.name		= "green",
		.max_brightness = 255,
		.id		= PMIC8028_ID_LED_GREEN,
	},
	[2] = {
		.name		= "blue",
		.max_brightness = 255,
		.id		= PMIC8028_ID_LED_BLUE,
	},
    [3] = {
		.name		= "button-backlight",
		.max_brightness = 255,
		.id		= PMIC8028_ID_LED_KEYBOARD,
	},
	[4] = {
		.name		= "headset-loop",
		.max_brightness = 255,
		.id		= PMIC8028_ID_HEADSET_LOOP,
	},
};

static struct pmic8028_leds_platform_data pmic8028_leds_data = {
	.num_leds = ARRAY_SIZE(pmic8028_leds),
	.leds	= pmic8028_leds,
};
static struct platform_device pmic8028_leds_device = {
	.name		= "pm8028-led",
	.id		    = -1,
	.dev		= {
		.platform_data	= &pmic8028_leds_data,
	},
};
#endif

int __init led_init(void)
{
    printk(KERN_INFO"[%s] entry\n", __func__);

#ifdef CONFIG_LEDS_SC668
    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_LED, 
                                     SC668_LED_V1), 
                   "SC668_LED_V1", 
                   config_sc668_gpios, 
                   __FILE__, __LINE__);

	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_LED, 
											SC668_LED_V1), 
						MSM_GSBI0_QUP_I2C_BUS_ID, sc668_led_i2c_info);
#endif
#ifdef CONFIG_LEDS_SN3193
    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_LED, 
                                     SN3193_LED_V1), 
                   "SN3193_LED_V1", 
                   config_sn3193_V1_gpios, 
                   __FILE__, __LINE__);
    ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_LED, 
                                         SN3193_LED_V1), 
                       MSM_GSBI1_QUP_I2C_BUS_ID, sn3193_led_V1_i2c_info);

    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_LED, 
                                     SN3193_LED_V2), 
                   "SN3193_LED_V2", 
                   config_sn3193_V2_gpios, 
                   __FILE__, __LINE__);
    ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_LED, 
                                         SN3193_LED_V2), 
                       MSM_GSBI1_QUP_I2C_BUS_ID, sn3193_led_V2_i2c_info);

    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_LED, 
                                     SN3193_LED_V3), 
                   "SN3193_LED_V3", 
                   config_sn3193_V2_gpios, 
                   __FILE__, __LINE__);
    ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_LED, 
                                         SN3193_LED_V3), 
                       MSM_GSBI1_QUP_I2C_BUS_ID, sn3193_led_V2_i2c_info);
//IRMPrime
    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_LED, 
                                     SN3193_LED_V4), 
                   "SN3193_LED_V4", 
                   config_sn3193_V3_gpios, 
                   __FILE__, __LINE__);
    ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_LED, 
                                         SN3193_LED_V4), 
                       MSM_GSBI1_QUP_I2C_BUS_ID, sn3193_led_V3_i2c_info);
#endif			

#ifdef CONFIG_LEDS_PMIC8028_PROC
    REGISTER_DYN_PLATFORM_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_LED, PM8028_LED_V1), 
                                 &pmic8028_leds_device);
#endif

	return 0;	
}
