#ifndef __FIH_VERSION_DEFINITION_H
#define __FIH_VERSION_DEFINITION_H


typedef enum 
{
	Project_IRM = 1,	// Iron Max UMTS Single SIM
	Project_IRE = 2,	// Iron Max EVDO Single SIM
	Project_IMN = 3,	// Iron Mini	
	Project_ITV = 4,	// Ironmax TV UMTS single SIM
	Project_IT2 = 5,	// Ironmax TV UMTS W+G
	Project_TBP = 6,	// TinBoost+ UMTS Single SIM
	Project_TBE = 7,	// TinBoost+ EVDO Single SIM
	Project_TNQ = 8,	// TinQ UMTS Single SIM
	Project_TQ2 = 9,	// TinQ UMTS W+G
	Project_IRQ = 10,	// Iron Q-mini
	Project_NPM = 11,
	Project_IMP = 12,	// Iron Max Plus
	Project_IPD = 13,	// Iron Max Plus Dual SIM
	Project_TPP = 14,
	PROJECT_MAX
} fih_product_id_type;


/*Modified By FIH-SW4-BSP,JiaHao,2011-11-14*/
typedef enum 
{
  Phase_PR0 = 0,
	Phase_PR1 = 1,
	Phase_PR2 = 2,
	Phase_PR3 = 3,
	Phase_PR4 = 4,
	Phase_PR5 = 5,
	Phase_PR6 = 6,
	Phase_PR1p5 = 15,
	Phase_PR2p5 = 25,
	Phase_PR3p5 = 35,
	Phase_PCR = 100,
	Phase_MP = 110,
  	PHASE_MAX
}fih_product_phase_type;


typedef enum 
{
  GSM_900_1800_CDMA_BC0 = 0x1,
  CDMA_BC0 = 0x2,           // Div1-FW1-BSP_HWID-02
  CDMA_BC01F = 0x3,
  CDMA_BC01 = 0x4,          // Added By SW4-BSP,Jiahao,2011-10-13
  CDMA_BC1 = 0x5,			//Modified By FIH-SW4-BSP,JiaHao,2011-11-14
  WCDMA_145_band = 0x6,
  WCDMA_148_band = 0x7,
  WCDMA_245_band = 0x8,
  WCDMA_15_band = 0x9,
  WCDMA_18_band = 0xA,
  WCDMA_25_band = 0xB,      // Div1-FW1-BSP_HWID-02
  WCDMA_125_band = 0xC,    // Div1-FW1-BSP_HWID-03
  WCDMA_128_band = 0xD,    // Div1-FW1-BSP_HWID-03
  WCDMA_14_band = 0xE,      // Div1-FW1-BSP_HWID-03
  BAND_MAX
}fih_band_id_type;


enum OMTP_Type {
	HEADSET_NORMAL = 0,
	HEADSET_OMTP = 1,
};

enum HAC_Type {
	/* HEADSET_NORMAL = 0, */
	HEADSET_HAC = 1,
};

enum SIM_NUM {
	SINGLE_SIM = 0,
	DUAL_SIM = 1,
};

// DRAM_Type
#define DRAM_2G 0x52414D32
#define DRAM_4G 0x52414D34

enum DEVINFO_Type {
	DEVINFO_V1 = 1,
	DEVINFO_V2 = 2,
	DEVINFO_V3 = 3,
	DEVINFO_TBP_V1 = 4,
	DEVINFO_TBP_V2 = 5,
	DEVINFO_TNQ_V1 = 6,
	DEVINFO_TNQ_V2 = 7,
    DEVINFO_NPM_V1 = 7,
};

enum KEYPAD_Type {
	FIH_GPIO_KEYPAD_V1 = 1,
	// TBP PR1
    	FIH_GPIO_KEYPAD_V2 = 2,
	// TNQ
    	FIH_GPIO_KEYPAD_V3 = 3,
    // TPE
    	FIH_GPIO_KEYPAD_V4 = 4,
	KEYPAD_QUALCOMM_DEFAULT = 255,
};

enum QWERTY_Type {
	QWERTY_KEYPAD_NO = 0,
	FIH_QWERTY_KEYPAD_V1 =1,
	QWERTY_QUALCOMM_DEFAULT = 255,
};

enum MAIN_CAM_Type {
	MAIN_CAM_MT9E013_V1 = 1,
	// TBP
    	MAIN_CAM_OV5640_V1 = 2,
	// TNQ
    	MAIN_CAM_OV5640_V2 = 3,
    //ITV
	MAIN_CAM_MT9E013_V2 = 4,
    //NPM
    MAIN_CAM_MT9E013_V3 = 5,
    //NPM PR1.5
    MAIN_CAM_MT9E013_V4 = 6,

};

enum FRONT_CAM_Type {
	FRONT_CAM_MT9V115_V1 = 1,
    FRONT_CAM_OV7692_V1 = 2,
    //ITV
    FRONT_CAM_OV7692_V2 = 3,
	//TBP PR2.5
	FRONT_CAM_OV7692_V3 = 4,
    //NPM
    FRONT_CAM_OV7692_V4 = 5,
};

enum LCM_Type {
	LCM_NO = 0,
	LCM_NOVATEK_NT35560_V1 = 1,
	// TBP
	LCM_TOSHIBA_UPD161809_V1 = 2,
	// TNQ
	LCM_RENESAS_R61531_V1 = 3,
    // ITV
    LCM_NOVATEK_NT35560_V2 = 4,
    //NPM
    LCM_SAMSUNG_S6E63M0X_V1 = 5,
	LCM_QUALCOMM_DEFAULT = 255,
};

enum TOUCH_Type {
	TOUCH_NO = 0,
	TOUCH_ATMEL_QT602240_V1 = 1,
	TOUCH_ATMEL_QT602240_V2 = 2,
	// TBP
	TOUCH_ATMEL_QT602240_V3 = 3,
	// TNQ
	TOUCH_ATMEL_QT602240_V4 = 4,
    // ITV
	TOUCH_ATMEL_QT602240_V5 = 5,
    //NPM
	TOUCH_ATMEL_QT602240_V6 = 6,
	TOUCH_QUALCOMM_DEFAULT = 255,
};

enum MSENSOR_Type {
	MSENSOR_YAS529_V1 = 1,
	MSENSOR_YAS529_V2 = 2,
	MSENSOR_YAS530_V1 = 3,
	MSENSOR_YAS529_V3 = 4,
	MSENSOR_YAS529_V4 = 5,
};

enum GSENSOR_Type {
	GSENSOR_BMA150_V1 = 1,
    GSENSOR_BMA250_V1 = 2,
    GSENSOR_BMA250_V2 = 3,
};

enum GYRO_Type {
	GYRO_NO = 0,
	GYRO_V1 = 1,
};

enum SD_Type {
	SD_V1 = 1,
	SD_V2 = 2,
	SD_QUALCOMM_DEFAULT = 255,
};

enum WIFI_Type {
	WIFI_V1 = 1,
};

enum BT_Type {
	BT_V1 = 1,
};

enum FM_Type {
	FM_V1 = 1,
};

enum NFC_Type {
	NFC_NO = 0,
	NFC_V1 = 1,
    //NPM
    NFC_V2 = 2,
};

enum BATTERY_Type {
	BATTERY_V1 = 1,
    BATTERY_V2 = 2,
    //NPM
    BATTERY_V3 = 3,
};

enum FLASH_LIGHT_Type {
	FLASH_LIGHT_V1 = 1,/**IMax*/
	FLASH_LIGHT_V2 = 2,/**ITV*/
    FLASH_LIGHT_V3 = 3,/*NPM*/
};
/*++{add for distinguish different Voltage*/
enum VIB_Type {
	VIB_Default = 0,
	VIB_V1 = 1,
};
/*add for distinguish different Voltage}++*/

enum LED_Type {
	SC668_LED_V1 = 1,
    PM8028_LED_V1 = 2,
    SN3193_LED_V1 = 3,
    SN3193_LED_V2 = 4,
    SN3193_LED_V3 = 5,
    //NPM
    SN3193_LED_V4 = 6,
};

enum PS_Type {
	PSALS_CM3623_V1 = 1,
	PSALS_CM3623_V2 = 2,
	STK3101 = 3,
	STK3101_V1= 4,
	STK3101_V2 = 5,
};

enum ALS_Type {
	ALS_V1 = 1,
	ALS_V2 = 2,
};

enum RSVD1_Type {
	RSVD1_V1 = 1,
};

enum RSVD2_Type {
	RSVD2_V1 = 1,
};

enum RSVD3_Type {
	RSVD3_V1 = 1,
};

enum RSVD4_Type {
	RSVD4_V1 = 1,
};

enum RSVD5_Type {
	RSVD5_V1 = 1,
};

/*FIH-NJ-SD4, Alex, Add,*/
enum TSIF_TYPE{
      TSIF_V1 = 1,
};

enum TSIF_NM32X_62X_TYPE{
      TSIF_NM32X_62X_V1 = 1,
};


enum DTV_NMI_TYPE{
      DTV_NMI_V1 = 1,
};
/* add new type here */
/*FIH-NJ-SD4, DK, Add,*/
enum UART_Type {
	UART_V1 = 1,
    UART_V2 = 2,
	UART_QUALCOMM_DEFAULT = 255,
};

enum HEADSET_Type {
	HEADSET_V1 = 1,
	HEADSET_V2 = 2,
	HEADSET_V3 = 3,
    //NPM
    HEADSET_V4 = 4,
};

#endif /* __FIH_VERSION_DEFINITION_H */
