/**
* Touch Screen Driver
******************************************/

#include "fih_touchscreen.h"

#ifdef CONFIG_FIH_TOUCHSCREEN_ATMEL_QT602240
static struct i2c_board_info tp_exp_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("qt602240", 0x4A),
	},
};


static struct i2c_board_info tp_exp_key_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("ektf4751sk", 0x1C),
	},	
};

#endif

int __init touchscreen_init(void)
{
#ifdef CONFIG_FIH_TOUCHSCREEN_ATMEL_QT602240
    /*IRM*/
	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_TOUCH, TOUCH_ATMEL_QT602240_V1), 
				MSM_GSBI0_QUP_I2C_BUS_ID,
				tp_exp_i2c_info);

    ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_TOUCH, TOUCH_ATMEL_QT602240_V2), 
				MSM_GSBI1_QUP_I2C_BUS_ID,
				tp_exp_i2c_info);

	/*TBP*/			
	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_TOUCH, TOUCH_ATMEL_QT602240_V3), 
				MSM_GSBI1_QUP_I2C_BUS_ID,
				tp_exp_i2c_info);
    /*TNQ*/
	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_TOUCH, TOUCH_ATMEL_QT602240_V4), 
				MSM_GSBI1_QUP_I2C_BUS_ID,
				tp_exp_i2c_info);

    /*ITV*/
	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_TOUCH, TOUCH_ATMEL_QT602240_V5), 
				MSM_GSBI1_QUP_I2C_BUS_ID,
				tp_exp_i2c_info);

    /*IRM2*/
	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_TOUCH, TOUCH_ATMEL_QT602240_V6), 
				MSM_GSBI1_QUP_I2C_BUS_ID,
				tp_exp_i2c_info);
    
        ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_TOUCH, TOUCH_ATMEL_QT602240_V6), 
                    MSM_GSBI1_QUP_I2C_BUS_ID,
                    tp_exp_key_i2c_info);

    
#endif
	return 0;
}
