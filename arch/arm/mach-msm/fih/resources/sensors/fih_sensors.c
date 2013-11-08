/** 
* Sensors Driver
****************************************************/
#include "fih_sensors.h"

/* DerrickDRLiu, for porting E compass, sensor architecture { */
#ifdef CONFIG_SENSORS_YAS_CM3623
struct FIH_lightsensor {
    unsigned int setting;
};

static struct FIH_lightsensor light_data_v1 = {
	.setting = 0xd7,
};

static struct FIH_lightsensor light_data_v2 = {
	.setting = 0xd3,
};

static struct i2c_board_info cm3623_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("cm3623_als1", 0x90>>1),//0x48, 0x20>>1, 0x10
		.platform_data = &light_data_v1,
	},
	{
		I2C_BOARD_INFO("cm3623_als1", 0x90>>1),//0x48, 0x20>>1, 0x10
		.platform_data = &light_data_v2,
	},
	{
		I2C_BOARD_INFO("cm3623_als2", 0x92>>1),//0x49, 0x22>>1, 0x11
	},
	{
		I2C_BOARD_INFO("cm3623_ps1", 0xF0>>1),//0x78,  0xB0>>1, 0x58
	},
	{
		I2C_BOARD_INFO("cm3623_ps2", 0xF2>>1),//0x79,  0xB2>>1, 0x59
	},
	{
		I2C_BOARD_INFO("cm3623_ara", 0x18>>1),//0x0c,  0x18>>1, 0x0c
	},
};
#endif

#ifdef CONFIG_STK_PS_0x90

static struct FIH_lightsensor light_data_stk_v1 = {
	.setting = 500,
};

static struct FIH_lightsensor light_data_stk_v2 = {
	.setting = 2000,
};

static struct i2c_board_info stk3101_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("stk_ps", 0x10),	// fake i2c address
		.platform_data = &light_data_stk_v1,
	},
	{
		I2C_BOARD_INFO("stk_ps", 0x10),	// fake i2c address
		.platform_data = &light_data_stk_v2,
	},
};
#endif

#ifdef CONFIG_INPUT_YAS_MAGNETOMETER
struct FIH_geomsensor {
    int placement;
};

static struct FIH_geomsensor geom_data_v1 = {
	.placement = 3,
};

static struct FIH_geomsensor geom_data_v2 = {
	.placement = 7,
};

static struct FIH_geomsensor geom_data_v3 = {
	.placement = 5,
};

static struct FIH_geomsensor geom_data_v4 = {
	.placement = 4,
};
static struct i2c_board_info msensor_i2c_info_v1[] __initdata = {
	{
            I2C_BOARD_INFO("geomagnetic", 0x2e),   //"YAS529"
            .platform_data = &geom_data_v1,
            //.irq	= MSM_GPIO_TO_INT(17),
        },
};

static struct i2c_board_info msensor_i2c_info_v2[] __initdata = {
	{
            I2C_BOARD_INFO("geomagnetic", 0x2e),   //"YAS529"
            .platform_data = &geom_data_v2,
            //.irq	= MSM_GPIO_TO_INT(17),
        },
};
static struct i2c_board_info msensor_i2c_info_v3[] __initdata = {
	{
            I2C_BOARD_INFO("geomagnetic", 0x2e),   //"YAS529"
            .platform_data = &geom_data_v3,
            //.irq	= MSM_GPIO_TO_INT(17),
        },
};

static struct i2c_board_info msensor_i2c_info_v4[] __initdata = {
	{
            I2C_BOARD_INFO("geomagnetic", 0x2e),   //"YAS529"
            .platform_data = &geom_data_v4,
         },
};

#endif

#ifdef CONFIG_INPUT_YAS_ACCELEROMETER
struct FIH_gsensor {
    int placement;
    int distortion;
    int reg_max;
    int i2c_addr;
    char *chipinfo;	
    int (*driver_init)(void *, struct FIH_gsensor *);
};

int bma150_yas_acc_driver_init(void *drv, struct FIH_gsensor *acc_data);
int bma250_yas_acc_driver_init(void *drv, struct FIH_gsensor *acc_data);

static struct FIH_gsensor bma150_data = {
	.driver_init = bma150_yas_acc_driver_init,
	.placement = 0,
	.distortion = 4000,
	.reg_max = 0x16,
	.i2c_addr = 0x38,
	.chipinfo = "bma150",
};

static struct FIH_gsensor bma250_data_v1 = {
	.driver_init = bma250_yas_acc_driver_init,
	.placement = 4,
	.distortion = 4000,
	.reg_max = 0x16,
	.i2c_addr = 0x18,
	.chipinfo = "bma250",
};

static struct FIH_gsensor bma250_data_v2 = {
	.driver_init = bma250_yas_acc_driver_init,
	.placement = 7,
	.distortion = 4000,
	.reg_max = 0x16,
	.i2c_addr = 0x18,
	.chipinfo = "bma250",
};

static struct i2c_board_info gsensor_i2c_info_v1[] __initdata = {
    {
        I2C_BOARD_INFO("accelerometer", 0x38), //"BMA150"
            .platform_data = &bma150_data,
			 //.irq	= MSM_GPIO_TO_INT(28),
    },
};

static struct i2c_board_info gsensor_i2c_info_v2[] __initdata = {
    {
        I2C_BOARD_INFO("accelerometer", 0x18), //"BMA250"
        .platform_data = &bma250_data_v1,
        //.irq	= MSM_GPIO_TO_INT(28),
    },
};

static struct i2c_board_info gsensor_i2c_info_v3[] __initdata = {
    {
        I2C_BOARD_INFO("accelerometer", 0x18), //"BMA250"
        .platform_data = &bma250_data_v2,
        //.irq	= MSM_GPIO_TO_INT(28),
    },
};
#endif

int __init sensors_init(void)
{
    /*IRM*/
	#ifdef CONFIG_SENSORS_YAS_CM3623
	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_PS, 
												PSALS_CM3623_V1), 
								MSM_GSBI0_QUP_I2C_BUS_ID, 
								&cm3623_i2c_info[0]);
	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_PS, 
												PSALS_CM3623_V1), 
								MSM_GSBI0_QUP_I2C_BUS_ID, 
								&cm3623_i2c_info[2]);
	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_PS, 
												PSALS_CM3623_V1), 
								MSM_GSBI0_QUP_I2C_BUS_ID, 
								&cm3623_i2c_info[3]);
	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_PS, 
												PSALS_CM3623_V1), 
								MSM_GSBI0_QUP_I2C_BUS_ID, 
								&cm3623_i2c_info[4]);
	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_PS, 
												PSALS_CM3623_V1), 
								MSM_GSBI0_QUP_I2C_BUS_ID, 
								&cm3623_i2c_info[5]);																									
	
	#endif

	#ifdef CONFIG_INPUT_YAS_MAGNETOMETER
	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_MSENSOR, 
												MSENSOR_YAS529_V1), 
								MSM_GSBI0_QUP_I2C_BUS_ID, 
								msensor_i2c_info_v1);
	#endif

	#ifdef CONFIG_INPUT_YAS_ACCELEROMETER
	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_GSENSOR, 
												GSENSOR_BMA150_V1), 
								MSM_GSBI0_QUP_I2C_BUS_ID, 
								gsensor_i2c_info_v1);
	#endif
	
    /* TBP */
	#ifdef CONFIG_SENSORS_YAS_CM3623
	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_PS, 
												PSALS_CM3623_V2), 
								MSM_GSBI1_QUP_I2C_BUS_ID, 
								&cm3623_i2c_info[1]);
	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_PS, 
												PSALS_CM3623_V2), 
								MSM_GSBI1_QUP_I2C_BUS_ID, 
								&cm3623_i2c_info[2]);
	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_PS, 
												PSALS_CM3623_V2), 
								MSM_GSBI1_QUP_I2C_BUS_ID, 
								&cm3623_i2c_info[3]);
	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_PS, 
												PSALS_CM3623_V2), 
								MSM_GSBI1_QUP_I2C_BUS_ID, 
								&cm3623_i2c_info[4]);
	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_PS, 
												PSALS_CM3623_V2), 
								MSM_GSBI1_QUP_I2C_BUS_ID, 
								&cm3623_i2c_info[5]);																												
	#endif

	#ifdef CONFIG_INPUT_YAS_MAGNETOMETER
	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_MSENSOR, 
												MSENSOR_YAS529_V2), 
								MSM_GSBI1_QUP_I2C_BUS_ID, 
								msensor_i2c_info_v2);
   ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_MSENSOR, 
												MSENSOR_YAS529_V3), 
								MSM_GSBI1_QUP_I2C_BUS_ID, 
								msensor_i2c_info_v3);

    ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_MSENSOR, 
												MSENSOR_YAS529_V4), 
								MSM_GSBI1_QUP_I2C_BUS_ID, 
								msensor_i2c_info_v4);
	#endif

	#ifdef CONFIG_INPUT_YAS_ACCELEROMETER
	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_GSENSOR, 
												GSENSOR_BMA250_V1), 
								MSM_GSBI1_QUP_I2C_BUS_ID, 
								gsensor_i2c_info_v2);

	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_GSENSOR, 
												GSENSOR_BMA250_V2), 
								MSM_GSBI1_QUP_I2C_BUS_ID, 
								gsensor_i2c_info_v3);
	#endif
	

	#ifdef CONFIG_STK_PS_0x90
	// TBP		
	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_PS, 
				STK3101_V1), 
				MSM_GSBI1_QUP_I2C_BUS_ID, 
				&stk3101_i2c_info[1]);	
	//IRE
	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_PS, 
				STK3101), 
				MSM_GSBI0_QUP_I2C_BUS_ID, 
				&stk3101_i2c_info[0]);		
	//IronPrime
     ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_PS, 
				STK3101_V2), 
				MSM_GSBI1_QUP_I2C_BUS_ID, 
				&stk3101_i2c_info[1]);	
	#endif

	#ifdef CONFIG_SENSORS_YAS_CM3623

	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_PS, 
								STK3101_V1), 
								MSM_GSBI1_QUP_I2C_BUS_ID, 
								&cm3623_i2c_info[0]);
	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_PS, 
								STK3101_V1), 
								MSM_GSBI1_QUP_I2C_BUS_ID, 
								&cm3623_i2c_info[2]);
	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_PS, 
								STK3101_V1), 
								MSM_GSBI1_QUP_I2C_BUS_ID, 
								&cm3623_i2c_info[3]);
	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_PS, 
								STK3101_V1), 
								MSM_GSBI1_QUP_I2C_BUS_ID, 
								&cm3623_i2c_info[4]);
	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_PS, 
								STK3101_V1), 
								MSM_GSBI1_QUP_I2C_BUS_ID, 
								&cm3623_i2c_info[5]);			


	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_PS, 
							   STK3101_V2), 
							   MSM_GSBI1_QUP_I2C_BUS_ID, 
							   &cm3623_i2c_info[0]);
   ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_PS, 
							   STK3101_V2), 
							   MSM_GSBI1_QUP_I2C_BUS_ID, 
							   &cm3623_i2c_info[2]);
   ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_PS, 
							   STK3101_V2), 
							   MSM_GSBI1_QUP_I2C_BUS_ID, 
							   &cm3623_i2c_info[3]);
   ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_PS, 
							   STK3101_V2), 
							   MSM_GSBI1_QUP_I2C_BUS_ID, 
							   &cm3623_i2c_info[4]);
   ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_PS, 
							   STK3101_V2), 
							   MSM_GSBI1_QUP_I2C_BUS_ID, 
							   &cm3623_i2c_info[5]);		   


   ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_PS, 
								STK3101), 
								MSM_GSBI0_QUP_I2C_BUS_ID, 
								&cm3623_i2c_info[0]);
	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_PS, 
								STK3101), 
								MSM_GSBI0_QUP_I2C_BUS_ID, 
								&cm3623_i2c_info[2]);
	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_PS, 
								STK3101), 
								MSM_GSBI0_QUP_I2C_BUS_ID, 
								&cm3623_i2c_info[3]);
	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_PS, 
								STK3101), 
								MSM_GSBI0_QUP_I2C_BUS_ID, 
								&cm3623_i2c_info[4]);
	ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_PS, 
								STK3101), 
								MSM_GSBI0_QUP_I2C_BUS_ID, 
								&cm3623_i2c_info[5]);																									
	#endif	
	
	return 0;
}
/* } DerrickDRLiu, for porting E compass, sensor architecture */
