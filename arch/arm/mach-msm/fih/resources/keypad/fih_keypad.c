/**
* Keypad Driver
****************************************************/
#include "fih_keypad.h"

// DIV2-SW2-BSP-IRM-BUTTONS +[
#ifdef CONFIG_FIH_KEYBOARD_GPIO
#include <fih/gpio_keys.h>	// DIV2-SW2-BSP-IRM-BUTTONS

#define GPIO_KEY_VOLDN		36
#define GPIO_KEY_VOLUP		37
#define GPIO_KEY_FOCUS		38
#define GPIO_KEY_CAMERA		39

#define KEY_FOCUS 			KEY_F13

#define ACTIVE_LOW_V1 	    0

static struct gpio_keys_button the_buttons_v1[] = {
	{
		.gpio				= GPIO_KEY_VOLDN,
		.code				= KEY_VOLUMEDOWN,
		.desc				= "Vol_Down",
		.active_low			= ACTIVE_LOW_V1,
		.type				= EV_KEY,
		.wakeup				= 1,
		.debounce_interval	= 2
	},
	{
		.gpio				= GPIO_KEY_VOLUP,
		.code				= KEY_VOLUMEUP,
		.desc				= "Vol_Up",
		.active_low			= ACTIVE_LOW_V1,
		.type				= EV_KEY,
		.wakeup				= 1,
		.debounce_interval	= 2
	},
	{
		.gpio				= GPIO_KEY_FOCUS,
		.code				= KEY_FOCUS,
		.desc				= "Cam_F",
		.active_low			= ACTIVE_LOW_V1,
		.type				= EV_KEY,
		.wakeup				= 0,
		.debounce_interval	= 2
	},
	{
		.gpio				= GPIO_KEY_CAMERA,
		.code				= KEY_CAMERA,
		.desc				= "Cam_T",
		.active_low			= ACTIVE_LOW_V1,
		.type				= EV_KEY,
		.wakeup				= 0,
		.debounce_interval	= 2
	},
};

static struct gpio_keys_platform_data the_button_data_v1 = {
	.buttons	= the_buttons_v1,
	.nbuttons	= ARRAY_SIZE(the_buttons_v1),
	.rep		= 0
};

static struct platform_device IRM_GPIO_key_v1 = {
	.name   = "gpio-keys",
	.id     = -1,
	.dev    = {
			.platform_data = &the_button_data_v1,
		},
};

#define ACTIVE_LOW_V2 	    1

static struct gpio_keys_button the_buttons_v2[] = {
	{
		.gpio				= GPIO_KEY_VOLDN,
		.code				= KEY_VOLUMEDOWN,
		.desc				= "Vol_Down",
		.active_low			= ACTIVE_LOW_V2,
		.type				= EV_KEY,
		.wakeup				= 0,
		.debounce_interval	= 2
	},
	{
		.gpio				= GPIO_KEY_VOLUP,
		.code				= KEY_VOLUMEUP,
		.desc				= "Vol_Up",
		.active_low			= ACTIVE_LOW_V2,
		.type				= EV_KEY,
		.wakeup				= 0,
		.debounce_interval	= 2
	},
	{
		.gpio				= GPIO_KEY_FOCUS,
		.code				= KEY_FOCUS,
		.desc				= "Cam_F",
		.active_low			= ACTIVE_LOW_V2,
		.type				= EV_KEY,
		.wakeup				= 0,
		.debounce_interval	= 2
	},
	{
		.gpio				= GPIO_KEY_CAMERA,
		.code				= KEY_CAMERA,
		.desc				= "Cam_T",
		.active_low			= ACTIVE_LOW_V2,
		.type				= EV_KEY,
		.wakeup				= 0,
		.debounce_interval	= 2
	},
};

static struct gpio_keys_platform_data the_button_data_v2 = {
	.buttons	= the_buttons_v2,
	.nbuttons	= ARRAY_SIZE(the_buttons_v2),
	.rep		= 0
};

static struct platform_device IRM_GPIO_key_v2 = {
	.name   = "gpio-keys",
	.id     = -1,
	.dev    = {
			.platform_data = &the_button_data_v2,
		},
};

#define ACTIVE_LOW_V3 	    0

static struct gpio_keys_button the_buttons_v3[] = {
	{
		.gpio				= GPIO_KEY_VOLDN,
		.code				= KEY_VOLUMEDOWN,
		.desc				= "Vol_Down",
		.active_low			= ACTIVE_LOW_V3,
		.type				= EV_KEY,
		.wakeup				= 1,  //FIH-NJ-BSP, jerry modify as wake up source. 2012/02/15
		.debounce_interval	= 2
	},
	{
		.gpio				= GPIO_KEY_VOLUP,
		.code				= KEY_VOLUMEUP,
		.desc				= "Vol_Up",
		.active_low			= ACTIVE_LOW_V3,
		.type				= EV_KEY,
		.wakeup				= 1,  //FIH-NJ-BSP, jerry modify as wake up source. 2012/02/15
		.debounce_interval	= 2
	},
};

static struct gpio_keys_platform_data the_button_data_v3 = {
	.buttons	= the_buttons_v3,
	.nbuttons	= ARRAY_SIZE(the_buttons_v3),
	.rep		= 0
};

static struct platform_device IRM_GPIO_key_v3 = {
	.name   = "gpio-keys",
	.id     = -1,
	.dev    = {
			.platform_data = &the_button_data_v3,
		},
};

/* FIHNJDC, SW4-BSP Jenny, 2011/10/12{ */
/* add for TBE keypad */
#define ACTIVE_LOW_V4 	    1

static struct gpio_keys_button the_buttons_v4[] = {
	{
		.gpio				= GPIO_KEY_VOLDN,
		.code				= KEY_VOLUMEDOWN,
		.desc				= "Vol_Down",
		.active_low			= ACTIVE_LOW_V4,
		.type				= EV_KEY,
		.wakeup				= 0,
		.debounce_interval	= 2
	},
	{
		.gpio				= GPIO_KEY_VOLUP,
		.code				= KEY_VOLUMEUP,
		.desc				= "Vol_Up",
		.active_low			= ACTIVE_LOW_V4,
		.type				= EV_KEY,
		.wakeup				= 0,
		.debounce_interval	= 2
	},
};

static struct gpio_keys_platform_data the_button_data_v4 = {
	.buttons	= the_buttons_v4,
	.nbuttons	= ARRAY_SIZE(the_buttons_v4),
	.rep		= 0
};

static struct platform_device IRM_GPIO_key_v4 = {
	.name   = "gpio-keys",
	.id     = -1,
	.dev    = {
			.platform_data = &the_button_data_v4,
		},
};
/* } FIHNJDC, SW4-BSP Jenny, 2011/10/12 */
static int config_buttons_gpios_v1(void *hdl) {
	int rc = 0;
	
	rc = gpio_tlmm_config( GPIO_CFG( GPIO_KEY_CAMERA, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA ), GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s not able to get gpio %d\n", __func__, GPIO_KEY_CAMERA);

	rc = gpio_tlmm_config( GPIO_CFG( GPIO_KEY_FOCUS, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA ), GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s not able to get gpio %d\n", __func__, GPIO_KEY_FOCUS);
		
	rc = gpio_tlmm_config( GPIO_CFG( GPIO_KEY_VOLDN, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA ), GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s not able to get gpio %d\n", __func__, GPIO_KEY_VOLDN);
		
	rc = gpio_tlmm_config( GPIO_CFG( GPIO_KEY_VOLUP, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA ), GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s not able to get gpio %d\n", __func__, GPIO_KEY_VOLUP);

	return rc;
}

static int config_buttons_gpios_v2(void *hdl) {
	int rc = 0;
	
	rc = gpio_tlmm_config( GPIO_CFG( GPIO_KEY_CAMERA, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA ), GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s not able to get gpio %d\n", __func__, GPIO_KEY_CAMERA);

	rc = gpio_tlmm_config( GPIO_CFG( GPIO_KEY_FOCUS, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA ), GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s not able to get gpio %d\n", __func__, GPIO_KEY_FOCUS);
		
	rc = gpio_tlmm_config( GPIO_CFG( GPIO_KEY_VOLDN, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA ), GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s not able to get gpio %d\n", __func__, GPIO_KEY_VOLDN);
		
	rc = gpio_tlmm_config( GPIO_CFG( GPIO_KEY_VOLUP, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA ), GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s not able to get gpio %d\n", __func__, GPIO_KEY_VOLUP);

	return rc;
}

static int config_buttons_gpios_v3(void *hdl) {
	int rc = 0;
	
	rc = gpio_tlmm_config( GPIO_CFG( GPIO_KEY_VOLDN, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA ), GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s not able to get gpio %d\n", __func__, GPIO_KEY_VOLDN);
		
	rc = gpio_tlmm_config( GPIO_CFG( GPIO_KEY_VOLUP, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA ), GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s not able to get gpio %d\n", __func__, GPIO_KEY_VOLUP);

	return rc;
}
#endif /* CONFIG_FIH_KEYBOARD_GPIO */
// DIV2-SW2-BSP-IRM-BUTTONS +]

#ifdef CONFIG_KEYBOARD_QWERTY_TCA8418
#define QWERTY_KEY_I2C_SLAVE_ID  (0x68 >> 1)
#define QWERTY_KEY_ROW 8
#define QWERTY_KEY_COL 8
#define QWERTY_KEY_NUMBER (QWERTY_KEY_ROW * QWERTY_KEY_COL)

#define GPIO_QWERTY_IRQ    94  //for qwerty interrupt pin
#define GPIO_QWERTY_RESET  93  //for qwerty reset pin

#define QKP_INDEX(row, col, val)  ((row << 24) | (col << 16) | val)


static int config_qwerty_keyapd_gpios(void* hdl)
{

    int rc = 0;
    rc = gpio_tlmm_config(GPIO_CFG(GPIO_QWERTY_RESET, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP,/*GPIO_CFG_NO_PULL,*/ GPIO_CFG_2MA),GPIO_CFG_ENABLE);
    if (rc)
        pr_err("%s not able to get gpio %d\n", __func__, GPIO_QWERTY_RESET);

        rc = gpio_tlmm_config(GPIO_CFG(GPIO_QWERTY_IRQ , 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),GPIO_CFG_ENABLE);
    if (rc)
        pr_err("%s not able to get gpio %d\n", __func__, GPIO_QWERTY_IRQ);

    return rc;
}

/*===============================================
     18               28                  38
  45               14 24  8                  48
     34                4                  54

43   55   25   36   45   16   26    6   56   44
 3   15   35    5    2   17   27    7   37   47
13   23   33   53    1   11   21   31   41   51
  12    22   32                 32    42   52
==================================================*/
static const unsigned int qwerty_keypad[QWERTY_KEY_NUMBER] = {
    QKP_INDEX(0, 0, KEY_RESERVED),
    QKP_INDEX(0, 1, KEY_V),
    QKP_INDEX(0, 2, KEY_G),
    QKP_INDEX(0, 3, KEY_A),
    QKP_INDEX(0, 4, KEY_DOWN),
    QKP_INDEX(0, 5, KEY_F),
    QKP_INDEX(0, 6, KEY_I),
    QKP_INDEX(0, 7, KEY_K),
//    QKP_INDEX(0, 8, KEY_RESERVED),
//    QKP_INDEX(0, 9, KEY_RESERVED),

    QKP_INDEX(1, 0, KEY_RIGHT),      //8
    QKP_INDEX(1, 1, KEY_RESERVED),
    QKP_INDEX(1, 2, KEY_RESERVED),
    QKP_INDEX(1, 3, KEY_B),
    QKP_INDEX(1, 4, KEY_LEFTALT),
    QKP_INDEX(1, 5, KEY_LEFTSHIFT),
    QKP_INDEX(1, 6, KEY_LEFT),
    QKP_INDEX(1, 7, KEY_S),
//    QKP_INDEX(1, 8, KEY_RESERVED),
//    QKP_INDEX(1, 9, KEY_RESERVED),

    QKP_INDEX(2, 0, KEY_Y),        //16
    QKP_INDEX(2, 1, KEY_H),
    QKP_INDEX(2, 2, KEY_MENU),
    QKP_INDEX(2, 3, KEY_RESERVED),
    QKP_INDEX(2, 4, KEY_RESERVED),
    QKP_INDEX(2, 5, KEY_N),
    QKP_INDEX(2, 6, KEY_EMAIL),
    QKP_INDEX(2, 7, KEY_Z),
//    QKP_INDEX(2, 8, KEY_RESERVED),
//    QKP_INDEX(2, 9, KEY_RESERVED),

  //  QKP_INDEX(3, 0, KEY_END),   //24
    QKP_INDEX(3, 0, KEY_REPLY),
    QKP_INDEX(3, 1, KEY_E),
    QKP_INDEX(3, 2, KEY_U),
    QKP_INDEX(3, 3, KEY_J),
    QKP_INDEX(3, 4, KEY_UP),
    QKP_INDEX(3, 5, KEY_RESERVED),
    QKP_INDEX(3, 6, KEY_RESERVED),
    QKP_INDEX(3, 7, KEY_M),
//    QKP_INDEX(3, 8, KEY_RESERVED),
//    QKP_INDEX(3, 9, KEY_RESERVED),

    // temp disable
    QKP_INDEX(4, 0, KEY_SPACE), //32
    QKP_INDEX(4, 1, KEY_X),
    QKP_INDEX(4, 2, KEY_HOME),
    QKP_INDEX(4, 3, KEY_D),
    QKP_INDEX(4, 4, KEY_R),
    QKP_INDEX(4, 5, KEY_L),
    QKP_INDEX(4, 6, KEY_BACK),
    QKP_INDEX(4, 7, KEY_RESERVED),
//    QKP_INDEX(4, 8, KEY_RESERVED),
//    QKP_INDEX(4, 9, KEY_RESERVED),

    QKP_INDEX(5, 0, KEY_RESERVED),   //40
    QKP_INDEX(5, 1, KEY_DOT),
    QKP_INDEX(5, 2, KEY_COMMA),
    QKP_INDEX(5, 3, KEY_Q),
    QKP_INDEX(5, 4, KEY_P),
    QKP_INDEX(5, 5, KEY_SEND),
    QKP_INDEX(5, 6, KEY_T),
    QKP_INDEX(5, 7, KEY_BACKSPACE),
//    QKP_INDEX(5, 8, KEY_RESERVED),
//    QKP_INDEX(5, 9, KEY_RESERVED),
    QKP_INDEX(6, 0, KEY_END),    //48
    QKP_INDEX(6, 1, KEY_RESERVED),
    QKP_INDEX(6, 2, KEY_RESERVED),
    QKP_INDEX(6, 3, KEY_ENTER),
    QKP_INDEX(6, 4, KEY_RIGHTALT),
    QKP_INDEX(6, 5, KEY_C),
    QKP_INDEX(6, 6, KEY_SEARCH),
    QKP_INDEX(6, 7, KEY_W),
//    QKP_INDEX(6, 8, KEY_RESERVED),
//    QKP_INDEX(6, 9, KEY_RESERVED),

    QKP_INDEX(7, 0, KEY_O),   //56
    QKP_INDEX(7, 1, KEY_RESERVED),
    QKP_INDEX(7, 2, KEY_RESERVED),
    QKP_INDEX(7, 3, KEY_RESERVED),
    QKP_INDEX(7, 4, KEY_RESERVED),
    QKP_INDEX(7, 5, KEY_RESERVED),
    QKP_INDEX(7, 6, KEY_RESERVED),
    QKP_INDEX(7, 7, KEY_RESERVED),
//    QKP_INDEX(7, 8, KEY_RESERVED),
//    QKP_INDEX(7, 9, KEY_RESERVED),

};

struct tca8418_qwerty_platform_data{
    const unsigned int *keypad_map;
    unsigned int        keymap_size;

};

static struct tca8418_qwerty_platform_data tca8418_qwerty_data = {
        .keypad_map     = qwerty_keypad,
        .keymap_size 	= QWERTY_KEY_NUMBER,
};

static struct i2c_board_info tca8418_qwertykey_i2c_info[] __initdata = {
    {
	I2C_BOARD_INFO("tca8418-keypad", QWERTY_KEY_I2C_SLAVE_ID),
        .irq            = MSM_GPIO_TO_INT(GPIO_QWERTY_IRQ ),
        .platform_data  = &tca8418_qwerty_data,
    },
};

#endif

int __init keypad_init(void)
{
    printk(KERN_INFO"[%s] entry\n", __func__);

#ifdef CONFIG_FIH_KEYBOARD_GPIO
    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_KEYPAD, FIH_GPIO_KEYPAD_V1),
		                   "FIH_GPIO_KEYPAD_V1", config_buttons_gpios_v1, __FILE__, __LINE__);
    REGISTER_DYN_PLATFORM_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_KEYPAD, 
                                                   FIH_GPIO_KEYPAD_V1), 
                                 &IRM_GPIO_key_v1);

    // for TBP
    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_KEYPAD, FIH_GPIO_KEYPAD_V2),
		                   "FIH_GPIO_KEYPAD_V2", config_buttons_gpios_v2, __FILE__, __LINE__);
    REGISTER_DYN_PLATFORM_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_KEYPAD, 
                                                   FIH_GPIO_KEYPAD_V2), 
                                 &IRM_GPIO_key_v2);
   
    // for TNQ
    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_KEYPAD, FIH_GPIO_KEYPAD_V3),
		                   "FIH_GPIO_KEYPAD_V3", config_buttons_gpios_v3, __FILE__, __LINE__);
    REGISTER_DYN_PLATFORM_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_KEYPAD, 
                                                   FIH_GPIO_KEYPAD_V3), 
                                 &IRM_GPIO_key_v3);
    // for TPE
    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_KEYPAD, FIH_GPIO_KEYPAD_V4),
		                   "FIH_GPIO_KEYPAD_V4", config_buttons_gpios_v3, __FILE__, __LINE__);
    REGISTER_DYN_PLATFORM_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_KEYPAD, 
                                                   FIH_GPIO_KEYPAD_V4), 
                                 &IRM_GPIO_key_v4);

#endif

#ifdef CONFIG_KEYBOARD_QWERTY_TCA8418
    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_QWERTY, FIH_QWERTY_KEYPAD_V1),
		                   "FIH_QWERTY_KEYPAD_V1", config_qwerty_keyapd_gpios, __FILE__, __LINE__);
    ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_QWERTY,
                                         FIH_QWERTY_KEYPAD_V1),
                       		MSM_GSBI1_QUP_I2C_BUS_ID, tca8418_qwertykey_i2c_info);

#endif
    return 0;
}
