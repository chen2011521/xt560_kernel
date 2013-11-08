/* FIH, NicoleWeng, 2010/03/09 { */
//#define FTM
#define CM3623_PS_GPIO_OUT 82// Domino:82


/* power state */ 
#define POWER_ON			0
#define POWER_OFF			1	

#define CM3623_INITIAL_INT_DATA    0x10  /* interrupt mode */ 
#define CM3623_INITIAL_POL_DATA    0x20  /* polling mode */


#define CM3623_ALS_GAIN1_OFFSET   7  /* Gain setting for Sensitiviry Range Selection */  
#define CM3623_ALS_GAIN0_OFFSET   6  
#define CM3623_ALS_THD1_OFFSET    5  /* Threshold window setting */
#define CM3623_ALS_THD0_OFFSET    4
#define CM3623_ALS_IT1_OFFSET     3  /* Integration Time setting */
#define CM3623_ALS_IT0_OFFSET     2
#define CM3623_ALS_WDM_OFFSET     1  /* Data mode setting, 1 for byte mode, 0 for word mode */
#define CM3623_ALS_SD_OFFSET      0  /* Shutdown mode setting, 1 for SD enable, 0 for SD disable */

#define CM3623_PS_DR1_OFFSET      7  /* IR LED on/off duty ratio setting */
#define CM3623_PS_DR0_OFFSET      6
#define CM3623_PS_IT1_OFFSET      5  /* Integration Time setting */
#define CM3623_PS_IT0_OFFSET      4
#define CM3623_PS_INTALS_OFFSET   3  /* ALS interruption setting */
#define CM3623_PS_INTPS_OFFSET    2  /* PS interruption setting */
#define CM3623_PS_RES_OFFSET      1  /* reserved, always 0 */
#define CM3623_PS_SD_OFFSET       0  /* Shutdown mode setting, 1 for SD enable, 0 for SD disable */

/* The slave address and function description*/

#define CM3623_INTERRUPT_ADDR			0x19	/* CM3623 interrupt address									*/
#define CM3623_ARA_ADDR					0x18	/* CM3623 alert response address (ARA) address				*/
#define CM3623_INITIAL_ADDR				0x92	/* CM3623 initialization usage								*/
#define CM3623_ALS_WRITE_ADDR			0x90	/* The slave address of ALS to write setting command to ALS		*/
#define CM3623_ALS_READ_MSB_ADDR		0x91	/* The slave address of ALS to read MSB 8 bits of ALS data		*/
#define CM3623_ALS_READ_LSB_ADDR		0x93	/* The slave address of ALS to read LSB 8 bits of ALS data		*/
#define CM3623_PS_WRITE_ADDR			0xF0	/* The slave address of PS to write setting command to PS		*/
#define CM3623_PS_READ_ADDR				0xF1	/* The slave address of PS to read 8 bits of ALS data				*/
#define CM3623_PS_THD_ADDR				0xF2	/* The slave address of PS to write INT threshold command to PS	*/

/* Since CM3623 contains an 8-bits command register written via the I2C bus										*/
/* Every time the transaction of data on the I2C bus will be assigned a bit to indicate the I2C activity, read"1" or write"0"		*/
/* The slave address of CM3623 must tansfer to 7-bits, as someone use the basic function of I2C bus, i2c_smbus_xfer()		*/
/* Because that function will assign the last bit automatically which is based on the activity of I2C							*/
/* The 7-bits of slave addres will be transfer as below:															*/

#define CM3623_INTERRUPT_ADDR_7BITS						((CM3623_INTERRUPT_ADDR>>1)&0x7F)		/*0x0C*/
#define CM3623_ARA_ADDR_7BITS							((CM3623_ARA_ADDR>>1)&0x7F)				/*0x0C*/
#define CM3623_INITIAL_ADDR_7BITS 			/*0x11*/	((CM3623_INITIAL_ADDR>>1)&0x7F) 		/*0x49*/
#define CM3623_ALS_WRITE_ADDR_7BITS 		/*0x10*/	((CM3623_ALS_WRITE_ADDR>>1)&0x7F)		/*0x48*/
#define CM3623_ALS_READ_MSB_ADDR_7BITS 		/*0x10*/	((CM3623_ALS_READ_MSB_ADDR>>1)&0x7F)	/*0x48*/
#define CM3623_ALS_READ_LSB_ADDR_7BITS 		/*0x11*/	((CM3623_ALS_READ_LSB_ADDR>>1)&0x7F)	/*0x49*/
#define CM3623_PS_WRITE_ADDR_7BITS			/*0x58*/	((CM3623_PS_WRITE_ADDR>>1)&0x7F)		/*0x78*/
#define CM3623_PS_READ_ADDR_7BITS         	/*0x58*/	((CM3623_PS_READ_ADDR>>1)&0x7F)			/*0x78*/
#define CM3623_PS_THD_ADDR_7BITS			/*0x59*/	((CM3623_PS_THD_ADDR>>1)&0x7F)			/*0x79*/

#define CM3623_PS_THD_MIN_VAL		0x03	/* The minimum threshold value for PS interruption activity */
#define CM3623_PS_THD_MAX_VAL		0xFF	/* The maximum threshold value for PS interruption activity */

#define CM3623_PSREG_INITIAL_VAL  	0x30	/*PS ON*/
#define CM3623_ALSREG_INITIAL_VAL 	0xD2	/*ALS ON*/

/* IOCTL */
#define INACTIVE_PS						0x0B00
#define INACTIVE_LS						0x0B01
#define ACTIVE_PS						0x0B02
#define ACTIVE_LS						0x0B03
#define INIT_CHIP  						0x0B04
#define READ_LS							0x0B05
#define READ_PS							0x0B06
#define READ_PS_DATA 					0x0B07
#define PS_STATUS	 					0x0B08
#define CM3602_FQC_Testing				0x0B09
#define INACTIVE_ALL 					0x0B0A
#define SHOW_als_reg_set 				0x0B0B
#define SHOW_ps_thd 					0x0B0C
#define SHOW_ps_reg_set 				0x0B0D
#define SHOW_init_reg_set 				0x0B0E
#define SET_als_reg 					0x0B0F
#define SET_ps_thd 						0x0B10
#define SET_ps_reg						0x0B11
#define SET_init_reg					0x0B12
#define RESET							0x0B13
/*FIH-NJDC, DK++, For poll read*/
#define READ_INT_LS						0x0B14
#define READ_INT_PS						0x0B15

/* IOCTL 
#define INACTIVE_PS						0
#define ACTIVE_PS						1
#define ACTIVE_LS						11
#define INACTIVE_LS						3
#define INIT_CHIP  						4
#define READ_LS							5
#define READ_PS							6
#define READ_PS_DATA 				7
#define PS_STATUS	 					8
#define CM3602_FQC_Testing		9
#define INACTIVE_ALL 					10
#define POWEROFF_CHIP				2
#define POWERON_CHIP				12
#define SHOW_als_reg_set 			13
#define SHOW_ps_thd 					14
#define SHOW_ps_reg_set 			15
#define SHOW_init_reg_set 			16
#define SET_als_reg 						17
#define SET_ps_thd 						18
#define SET_ps_reg						19
#define SET_init_reg		 				20
#define RESET								21*/

/*} FIH, NicoleWeng, 2010/03/09  */
/*
#define CM3602_PS_OFF			0
#define CM3602_PS_ON			1
#define CM3602_ALS_ON				2
#define CM3602_ALS_OFF				3
#define CM3602_PS_ON_SIGNAL		4
#define CM3602_ALS_READ			5
#define CM3602_PS_READ			6
#define CM3602_ACTIVATE			7
#define CM3602_Proximity_Status	8

// [FXX_CR], Add for proximity driver to turn on/off BL and TP. 
#define CM3602_FQC_Testing		9

#define CM3602_OFF		10
*/
