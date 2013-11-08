#ifndef DYN_LOADER_H
#define DYN_LOADER_H

#ifndef NULL
#define NULL ((void *)0)
#endif

typedef enum {
	DRIVER_TYPE_DEVINFO = 0,
	DRIVER_TYPE_KEYPAD,
	DRIVER_TYPE_MAIN_CAM,
	DRIVER_TYPE_FRONT_CAM,
	DRIVER_TYPE_LCM,
	DRIVER_TYPE_TOUCH,	// 5
	DRIVER_TYPE_MSENSOR,
	DRIVER_TYPE_GSENSOR,
	DRIVER_TYPE_GYRO,
	DRIVER_TYPE_SD,	
	DRIVER_TYPE_WIFI,	// 10
	DRIVER_TYPE_BT,
	DRIVER_TYPE_FM,
	DRIVER_TYPE_NFC,
	DRIVER_TYPE_BATTERY,

	DRIVER_TYPE_FLASH_LIGHT, // 15
	DRIVER_TYPE_LED,
	DRIVER_TYPE_PS,		// 17
	DRIVER_TYPE_ALS,
    /*FIH-NJ-SD4, DK, Add,*/
    	DRIVER_TYPE_RSVD1,
	DRIVER_TYPE_UART = DRIVER_TYPE_RSVD1,
	DRIVER_TYPE_RSVD2,
	DRIVER_TYPE_HEADSET = DRIVER_TYPE_RSVD2,
	DRIVER_TYPE_RSVD3,
	DRIVER_TYPE_QWERTY = DRIVER_TYPE_RSVD3,
	DRIVER_TYPE_RSVD4,
	DRIVER_TYPE_RSVD5,
	/* add new type here */

	/*FIH-NJ-SD4, Alex, Add,*/ 
	DRIVER_TYPE_TSIF,
	DRIVER_TYPE_TSIF_NM32X_62X,
	DRIVER_TYPE_DTV_NMI,
    DRIVER_TYPE_VIB,      //FIH-NJ-SD4, Huanjingjing, 2012/3/21 
    /* add new type here */

	PLATFORM_DRIVER_TEST,
	I2C_DRIVER_TEST,
		
	DRIVER_TYPE_LAST,
} DRIVER_TYPE_E;

#include <fih/version_definition.h>
struct st_hwid_table {
	char HWID_Reading_1;
	char HWID_Reading_2;
	char HWID_Reading_3;
	char Project_ID;
	char Phase_ID;
	char Band_ID;
	char OMTP;
	char HAC;
	char SIM;
	unsigned int DRAM;
	char DEVINFO;
	char KEYPAD;
	char MAIN_CAM;
	char FRONT_CAM;
	char LCM;
	char TOUCH;
	char MSENSOR;
	char GSENSOR;
	char GYRO;
	char SD;
	char WIFI;
	char BT;
	char FM;
	char NFC;
	char BATTERY;
	
	char FLASH_LIGHT;
	char LED;
	char PS;
	char ALS;
	char UART;
	char HEADSET;
	char QWERTY_KEYPAD;
	char RSVD4;
	char RSVD5;
	char TSIF;
	char TSIF_NM32_62X;
	char DTV_NMI;

};

// FIHTDC, HenryMCWang, 2011.10.31, for new hardware table, +++
struct st_new_hwid_table {
	char HWID_Reading_1;
	char HWID_Reading_2;
	char HWID_Reading_3;
	char Project_ID;
	char Phase_ID;
	char OMTP;
	char HAC;
	char SIM;
	unsigned long long Band_ID;
	unsigned long DRAM;
	char DEVINFO;
	char KEYPAD;
	char MAIN_CAM;
	char FRONT_CAM;
	char LCM;
	char TOUCH;
	char MSENSOR;
	char GSENSOR;
	char GYRO;
	char SD;
	char WIFI;
	char BT;
	char FM;
	char NFC;
	char BATTERY;
	char FLASH_LIGHT;
	char LED;
	char PS;
	char ALS;
	char UART;
	char HEADSET;
	char QWERTY_KEYPAD;
	char TSIF;
	char TSIF_NM32_62X;
	char DTV_NMI;
	char VIB;
};
// FIHTDC, HenryMCWang, 2011.10.31, for new hardware table, ---

typedef int DRIVER_VERSION_E;

#define TEST_PASS_VER1	1
#define TEST_DENY_VER1	0
#define TEST_PASS_VER2	((1<<1)+1)
#define TEST_DENY_VER2	((1<<1)+0)
#define TEST_PASS_VER3	((2<<1)+1)
#define TEST_DENY_VER3	((2<<1)+0)
#define TEST_PASS_VER4	((3<<1)+1)
#define TEST_DENY_VER4	((3<<1)+0)

#include <linux/platform_device.h>
#include <linux/i2c.h>


typedef int (*INIT)(void *hdl);
typedef int (*PLATFORM_PROBE)(struct platform_device *dev);
typedef int (*I2C_PROBE)(struct i2c_client *client, const struct i2c_device_id *id);

void * decl_version_ctrl(DRIVER_TYPE_E type, DRIVER_VERSION_E version, char *dbg);

int add_dyn_i2c_driver(void *hdl, struct i2c_driver *drv,
			char *path, unsigned int line);
int add_dyn_i2c_device(void *hdl, 
		int busnum, struct i2c_board_info *dev,
		char *path, unsigned int line);
int call_dyn_i2c_driver_probe(DRIVER_TYPE_E type, I2C_PROBE fn, 
				struct i2c_client *client, 
				const struct i2c_device_id *id);


int add_dyn_platform_driver(void *hdl, struct platform_driver *drv, 
				char *path, unsigned int line);
int add_dyn_platform_device(void *hdl, struct platform_device *dev,
				char *path, unsigned int line);
int register_dyn_platform_device(void *hdl, struct platform_device *dev,
				char *path, unsigned int line);				
int call_dyn_platform_driver_probe(DRIVER_TYPE_E type, PLATFORM_PROBE fn, struct platform_device *dev);
				
int add_dyn_module(void *hdl, char *name, INIT fn, char *path, unsigned int line);


#define DECL_VERSION_CTRL(TYPE, VERSION)	\
		decl_version_ctrl(TYPE, VERSION, #TYPE":"#VERSION)
		
		
		
#define ADD_DYN_I2C_DRIVER(HDL, DRV) \
		add_dyn_i2c_driver((HDL), (DRV), __FILE__, __LINE__)
		
#define ADD_DYN_I2C_DEVICE(HDL, BUSID, DEV) \
		add_dyn_i2c_device((HDL), (BUSID), (DEV), __FILE__, __LINE__)
		
#define DECL_I2C_DRIVER_PROBE(TYPE, FUNC, CLNT, ID) \
		extern int hook##FUNC(struct i2c_client *CLNT, const struct i2c_device_id *ID); \
		int FUNC(struct i2c_client *CLNT, const struct i2c_device_id *ID) { \
			return call_dyn_i2c_driver_probe(TYPE, hook##FUNC, CLNT, ID); \
		} \
		int hook##FUNC(struct i2c_client *CLNT, const struct i2c_device_id *ID)

		
		
#define ADD_DYN_PLATFORM_DRIVER(HDL, DRV) \
			add_dyn_platform_driver((HDL), (DRV), __FILE__, __LINE__)

#define ADD_DYN_PLATFORM_DEVICE(HDL, DEV) \
		add_dyn_platform_device((HDL), (DEV), __FILE__, __LINE__)

#define REGISTER_DYN_PLATFORM_DEVICE(HDL, DEV) \
		register_dyn_platform_device((HDL), (DEV), __FILE__, __LINE__)

#define DECL_PLATFORM_DRIVER_PROBE(TYPE, FUNC, DEV) \
		extern int hook##FUNC(struct platform_device *DEV); \
		int FUNC(struct platform_device *DEV) { \
			return call_dyn_platform_driver_probe(TYPE, hook##FUNC, DEV); \
		} \
		int hook##FUNC(struct platform_device *DEV)

#define LEVEL0	"0"
#define LEVEL1	"1"
#define LEVEL2	"2"
#define LEVEL3	"3"
#define LEVEL4	"4"
#define LEVEL5	"5"
#define LEVEL6	"6"
#define LEVEL7	"7"
					
/** 
* [LEVEL] determine driver loading dependency
* 	LEVEL0: the same as "pure_initcall"
* 	LEVEL1: the same as "core_initcall"
* 	LEVEL2: the same as "postcore_initcall"
* 	LEVEL3: the same as "arch_initcall"
* 	LEVEL4: the same as "subsys_initcall"
* 	LEVEL5: the same as "fs_initcall"
* 	LEVEL6: the same as "device_initcall" / "module_init"
* 	LEVEL7: the same as "late_initcall"
**************************************************************/
#define DECLARE_DYN_LOADER(TYPE, VERSION, INIT, LEVEL) \
	static int __init wrapper##INIT(void)			\
	{		\
		void *hdl = decl_version_ctrl(TYPE, VERSION, #TYPE"_"#VERSION);	\
		return add_dyn_module(hdl, #INIT, INIT, __FILE__, __LINE__);	\
	}							\
	static initcall_t __initcall_##INIT __used 	\
		__attribute__((__section__(".initcall" LEVEL ".init"))) = wrapper##INIT


		
#if 0
typedef struct _dyn_platform_device {
	DRIVER_TYPE_E type;
	DRIVER_VERSION_E version;
	struct platform_device *dev;
} dyn_platform_device;
#endif


typedef struct _init_project {
	int project_id;
	int (*init)(void);
} init_project_s;

int call_project_init(init_project_s *projects, int num);
		
#endif	/* DYN_LOADER_H */
