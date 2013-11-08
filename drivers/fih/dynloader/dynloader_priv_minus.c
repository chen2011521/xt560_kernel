#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>	// for kzalloc

#include <linux/proc_fs.h> 
#include <linux/seq_file.h> 
#include <linux/uaccess.h>

#include <fih/dynloader.h>
#include <linux/spinlock.h>

#include <mach/msm_iomap.h>
#include <asm/mach/map.h>

#include "../arch/arm/mach-msm/proc_comm.h"

// FIHTDC, HenryMCWang, 2011.10.31, for new hardware table, +++
//static struct st_hwid_table g_hwcfg;
struct st_new_hwid_table *p_new_hwid_table;
// FIHTDC, HenryMCWang, 2011.10.31, for new hardware table, ---
static char versionCtrl[DRIVER_TYPE_LAST];

void dumpMEM(void)
{
	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> HWID_Reading_1 = %d\n", p_new_hwid_table->HWID_Reading_1);
	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> HWID_Reading_2 = %d\n", p_new_hwid_table->HWID_Reading_2);
	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> HWID_Reading_3 = %d\n", p_new_hwid_table->HWID_Reading_3);
	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> Project_ID = %d\n", p_new_hwid_table->Project_ID);
	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> Phase_ID = %d\n", p_new_hwid_table->Phase_ID);
	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> OMTP = %d\n", p_new_hwid_table->OMTP);
	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> HAC = %d\n", p_new_hwid_table->HAC);

	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> SIM = %d\n", p_new_hwid_table->SIM);
	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> Band_ID = %llu\n", p_new_hwid_table->Band_ID);
	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> DRAM = %lx\n", p_new_hwid_table->DRAM);
	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> DEVINFO = %d\n", p_new_hwid_table->DEVINFO);
	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> KEYPAD = %d\n", p_new_hwid_table->KEYPAD);

	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> MAIN_CAM = %d\n", p_new_hwid_table->MAIN_CAM);
	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> FRONT_CAM = %d\n", p_new_hwid_table->FRONT_CAM);
	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> LCM = %d\n", p_new_hwid_table->LCM);
	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> TOUCH = %d\n", p_new_hwid_table->TOUCH);

	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> MSENSOR = %d\n", p_new_hwid_table->MSENSOR);
	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> GSENSOR = %d\n", p_new_hwid_table->GSENSOR);
	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> GYRO = %d\n", p_new_hwid_table->GYRO);
	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> SD = %d\n", p_new_hwid_table->SD);

	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> WIFI = %d\n", p_new_hwid_table->WIFI);
	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> BT = %d\n", p_new_hwid_table->BT);
	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> FM = %d\n", p_new_hwid_table->FM);
	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> NFC = %d\n", p_new_hwid_table->NFC);

	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> BATTERY = %d\n", p_new_hwid_table->BATTERY);
	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> FLASH_LIGHT = %d\n", p_new_hwid_table->FLASH_LIGHT);
	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> LED = %d\n", p_new_hwid_table->LED);
	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> PS = %d\n", p_new_hwid_table->PS);

	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> ALS = %d\n", p_new_hwid_table->ALS);
	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> TSIF = %d\n", p_new_hwid_table->TSIF);
	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> TSIF_NM32_62X = %d\n", p_new_hwid_table->TSIF_NM32_62X);
	printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> DTV_NMI = %d\n", p_new_hwid_table->DTV_NMI);
    printk(KERN_INFO "[read_hw_cfg] p_new_hwid_table-> VIB = %d\n", p_new_hwid_table->VIB);
}

char * read_hw_cfg(void) 
{
	static int first = 1;
	
	if(1 == first) {

	p_new_hwid_table = (struct st_new_hwid_table *)MSM_HW_TABLE_BASE;

	//dumpMEM();
	
	versionCtrl[DRIVER_TYPE_DEVINFO] = p_new_hwid_table->DEVINFO;
	versionCtrl[DRIVER_TYPE_KEYPAD] = p_new_hwid_table->KEYPAD;
	versionCtrl[DRIVER_TYPE_MAIN_CAM] = p_new_hwid_table->MAIN_CAM;
	versionCtrl[DRIVER_TYPE_FRONT_CAM] = p_new_hwid_table->FRONT_CAM;
	versionCtrl[DRIVER_TYPE_LCM] = p_new_hwid_table->LCM;
	versionCtrl[DRIVER_TYPE_TOUCH] = p_new_hwid_table->TOUCH;
	versionCtrl[DRIVER_TYPE_MSENSOR] = p_new_hwid_table->MSENSOR;
	versionCtrl[DRIVER_TYPE_GSENSOR] = p_new_hwid_table->GSENSOR;
	versionCtrl[DRIVER_TYPE_GYRO] = p_new_hwid_table->GYRO;
	versionCtrl[DRIVER_TYPE_SD] = p_new_hwid_table->SD;
	versionCtrl[DRIVER_TYPE_WIFI] = p_new_hwid_table->WIFI;
	versionCtrl[DRIVER_TYPE_BT] = p_new_hwid_table->BT;
	versionCtrl[DRIVER_TYPE_FM] = p_new_hwid_table->FM;
	versionCtrl[DRIVER_TYPE_NFC] = p_new_hwid_table->NFC;
	versionCtrl[DRIVER_TYPE_BATTERY] = p_new_hwid_table->BATTERY;
	
	versionCtrl[DRIVER_TYPE_FLASH_LIGHT] = p_new_hwid_table->FLASH_LIGHT;
	versionCtrl[DRIVER_TYPE_LED] = p_new_hwid_table->LED;
	versionCtrl[DRIVER_TYPE_PS] = p_new_hwid_table->PS;
	versionCtrl[DRIVER_TYPE_ALS] = p_new_hwid_table->ALS;
	// FIH-NJ-SD4, DK, Add,
	versionCtrl[DRIVER_TYPE_UART] = p_new_hwid_table->UART;	
	versionCtrl[DRIVER_TYPE_HEADSET] = p_new_hwid_table->HEADSET;	
	versionCtrl[DRIVER_TYPE_QWERTY] = p_new_hwid_table->QWERTY_KEYPAD;
	//temperially modified and waited the modem to add new component id.
	versionCtrl[DRIVER_TYPE_TSIF] =p_new_hwid_table->TSIF;  
	versionCtrl[DRIVER_TYPE_TSIF_NM32X_62X] = p_new_hwid_table->TSIF_NM32_62X;
	versionCtrl[DRIVER_TYPE_DTV_NMI] = p_new_hwid_table->DTV_NMI;
    versionCtrl[DRIVER_TYPE_VIB] = p_new_hwid_table->VIB;;

/*	
		proc_comm_read_hwid_table(&g_hwcfg);
		
		versionCtrl[DRIVER_TYPE_DEVINFO] = g_hwcfg.DEVINFO;
		versionCtrl[DRIVER_TYPE_KEYPAD] = g_hwcfg.KEYPAD;
		versionCtrl[DRIVER_TYPE_MAIN_CAM] = g_hwcfg.MAIN_CAM;
		versionCtrl[DRIVER_TYPE_FRONT_CAM] = g_hwcfg.FRONT_CAM;
		versionCtrl[DRIVER_TYPE_LCM] = g_hwcfg.LCM;
		versionCtrl[DRIVER_TYPE_TOUCH] = g_hwcfg.TOUCH;
		versionCtrl[DRIVER_TYPE_MSENSOR] = g_hwcfg.MSENSOR;
		versionCtrl[DRIVER_TYPE_GSENSOR] = g_hwcfg.GSENSOR;
		versionCtrl[DRIVER_TYPE_GYRO] = g_hwcfg.GYRO;
		versionCtrl[DRIVER_TYPE_SD] = g_hwcfg.SD;
		versionCtrl[DRIVER_TYPE_WIFI] = g_hwcfg.WIFI;
		versionCtrl[DRIVER_TYPE_BT] = g_hwcfg.BT;
		versionCtrl[DRIVER_TYPE_FM] = g_hwcfg.FM;
		versionCtrl[DRIVER_TYPE_NFC] = g_hwcfg.NFC;
		versionCtrl[DRIVER_TYPE_BATTERY] = g_hwcfg.BATTERY;
		
		versionCtrl[DRIVER_TYPE_FLASH_LIGHT] = g_hwcfg.FLASH_LIGHT;
		versionCtrl[DRIVER_TYPE_LED] = g_hwcfg.LED;
		versionCtrl[DRIVER_TYPE_PS] = g_hwcfg.PS;
		versionCtrl[DRIVER_TYPE_ALS] = g_hwcfg.ALS;
        	// FIH-NJ-SD4, DK, Add,
		versionCtrl[DRIVER_TYPE_UART] = g_hwcfg.UART;	
		versionCtrl[DRIVER_TYPE_HEADSET] = g_hwcfg.HEADSET;	
		versionCtrl[DRIVER_TYPE_QWERTY] = g_hwcfg.QWERTY_KEYPAD;
		versionCtrl[DRIVER_TYPE_RSVD4] = g_hwcfg.RSVD4;
		versionCtrl[DRIVER_TYPE_RSVD5] = g_hwcfg.RSVD5;
        //temperially modified and waited the modem to add new component id.
        versionCtrl[DRIVER_TYPE_TSIF] =g_hwcfg.TSIF;  
	    versionCtrl[DRIVER_TYPE_TSIF_NM32X_62X] = g_hwcfg.TSIF_NM32_62X;
	    versionCtrl[DRIVER_TYPE_DTV_NMI] = g_hwcfg.DTV_NMI;
*/

		
		first = 0;
	}
	
	return versionCtrl;
}


#define DEBUG

#if 1
#define DBG(FORMAT, ARGS...) \
		printk(KERN_INFO FORMAT, ##ARGS)
#else
#define DBG(MSG...)
#endif		

#if 1
// void spin_lock(spinlock_t *lock);
#define MYLOCK(LOCK)	\
			spin_lock(LOCK)

// void spin_unlock(spinlock_t *lock); 			
#define MYUNLOCK(LOCK)	\
			spin_unlock(LOCK)
#else
// void spin_lock(spinlock_t *lock);
#define MYLOCK(LOCK)	\
			(void)LOCK

// void spin_unlock(spinlock_t *lock); 			
#define MYUNLOCK(LOCK)	\
			(void)LOCK
#endif			

typedef enum _VERSION_CTRL_STATUS {
	STATUS_ALLOW = 0,
	STATUS_REG_OK,
	STATUS_BUNDLE,

	STATUS_NOT_ALLOW = 1000,
	STATUS_REG_FAIL,
} VERSION_CTRL_STATUS_E;


#ifdef DEBUG
#define DEBUGINFO \
		char *name; \
		char *path; \
		int line; \
		char dbg[1];
#else
#define DEBUGINFO
#endif


typedef enum _DYN_UNIT_TYPE {
	PLATFORM_DEVICE_TYPE = 0,
	PLATFORM_DRIVER_TYPE,
	I2C_DEVICE_TYPE,
	I2C_DRIVER_TYPE,
	MODULE_TYPE,
} DYN_UNIT_TYPE_E;

char * unit_name[] = {
	[PLATFORM_DEVICE_TYPE] = "Platform Device",
	[PLATFORM_DRIVER_TYPE] = "Platform Driver",
	[I2C_DEVICE_TYPE] = "I2C Device",
	[I2C_DRIVER_TYPE] = "I2C Driver",
	[MODULE_TYPE] = "Module",
};


typedef struct _dyn_driver {	
	int probe_addr;
	int wrong_devs;
} dyn_driver_s;

typedef struct _dyn_device {
	void *priv_data;
	int result;
} dyn_device_s;

typedef struct _dyn_module {
	int result;
} dyn_module_s;

typedef struct _dyn_unit {
	struct list_head list;
	
	VERSION_CTRL_STATUS_E status;
	DYN_UNIT_TYPE_E type;
	union {
		dyn_driver_s dyn_drv;
		dyn_device_s dyn_dev;
		dyn_module_s dyn_mod;
	} u;

	DEBUGINFO
} dyn_unit_s;


typedef struct _version_ctrl_hdl {
	struct list_head list;
	
	DRIVER_VERSION_E version;
	VERSION_CTRL_STATUS_E status;

	struct list_head dyn_mod_list;	
	struct list_head dyn_dev_list;
	struct list_head dyn_drv_list;
	
	char dbg[1];
} version_ctrl_hdl_s;


static spinlock_t ver_ctrl_lock = SPIN_LOCK_UNLOCKED; 


struct list_head version_ctrl[DRIVER_TYPE_LAST];
char version_ctrl_init[DRIVER_TYPE_LAST] = {0};


VERSION_CTRL_STATUS_E dyn_ctrl_allow(DRIVER_TYPE_E type, DRIVER_VERSION_E version)
{	
	char *versionCtrl;
	
	if((type == PLATFORM_DRIVER_TEST) || (type == I2C_DRIVER_TEST)) {
		if(0x1 == (version & 0x1))
			return STATUS_ALLOW;
		else 
			return STATUS_NOT_ALLOW;
	}
	else {
		versionCtrl = read_hw_cfg();
		if(*(versionCtrl+type) == version) {
			return STATUS_ALLOW;
		}
	}
	
	return STATUS_NOT_ALLOW;
}



void * decl_version_ctrl(DRIVER_TYPE_E type, DRIVER_VERSION_E version, char *dbg) 
{
	version_ctrl_hdl_s *ctrl;
	struct list_head *lh;

	MYLOCK(&ver_ctrl_lock);
	if(0 == version_ctrl_init[type]) {
		version_ctrl_init[type] = 1;
		INIT_LIST_HEAD(&version_ctrl[type]);
	}
	
	list_for_each(lh, &(version_ctrl[type])) {
		ctrl = list_entry(lh, version_ctrl_hdl_s, list);
		if(version == ctrl->version) {
			MYUNLOCK(&ver_ctrl_lock);
			return (void *)ctrl;	
		}
	}
	MYUNLOCK(&ver_ctrl_lock);
	
	// create a new one
	ctrl = kzalloc(sizeof(version_ctrl_hdl_s)+strlen(dbg), GFP_KERNEL);
	if (ctrl) {
		ctrl->version = version;
		ctrl->status = dyn_ctrl_allow(type, version);
		strcpy(ctrl->dbg, dbg);
		INIT_LIST_HEAD(&(ctrl->list));
		INIT_LIST_HEAD(&(ctrl->dyn_dev_list));
		INIT_LIST_HEAD(&(ctrl->dyn_drv_list));
		INIT_LIST_HEAD(&(ctrl->dyn_mod_list)); 
		
		MYLOCK(&ver_ctrl_lock);
		if(ctrl->status == STATUS_ALLOW) {
			// put allowed version control in the first entry
			list_add(&(ctrl->list), &(version_ctrl[type]));
		}
		else {
			list_add_tail(&(ctrl->list), &(version_ctrl[type]));
		}
		MYUNLOCK(&ver_ctrl_lock);
		
		return (void *)ctrl;
	}
	
	return NULL;
}


dyn_unit_s * create_dyn_unit(const char *name, const char *path, const unsigned int line)
{
	dyn_unit_s *unit;
	unsigned int size;

#ifdef DEBUG
	char dbg[1000];
	sprintf(dbg, "%s %s", path, name);
	size = sizeof(dyn_unit_s) + strlen(dbg);
#else
	size = sizeof(dyn_unit_s);
#endif
	
	unit = kzalloc(size, GFP_KERNEL);
	if(NULL != unit) {
		INIT_LIST_HEAD(&(unit->list));
		unit->status = STATUS_ALLOW;
		
#ifdef DEBUG	
		strcpy(unit->dbg, dbg);	
		unit->path = unit->dbg;
		*(unit->path + strlen(path)) = 0;
		unit->name = unit->dbg + strlen(path) + 1;
		unit->line = line;
#endif
	}

	return unit;
}




/** 
* Dynamic platform driver/device loading with version control 
***************************************************************************/
dyn_unit_s * dyn_ctrl_valid_platform_device(DRIVER_TYPE_E type, struct platform_device *dev) 
{
	struct list_head *lh;
	version_ctrl_hdl_s *ctrl;
	dyn_unit_s *unit;

	MYLOCK(&ver_ctrl_lock);
	if(0 == version_ctrl_init[type]) {
		MYUNLOCK(&ver_ctrl_lock);
		return NULL;
	}

#if 0	
	ctrl = NULL;
	list_for_each(lh, &(version_ctrl[type])) {
		ctrl = list_entry(lh, version_ctrl_hdl_s, list);
		if(STATUS_ALLOW == ctrl->status) {
			break;
		}
		ctrl = NULL;
	}
#else
	ctrl = list_first_entry(&(version_ctrl[type]), version_ctrl_hdl_s, list);
	if(STATUS_ALLOW != ctrl->status) {
		ctrl = NULL;
	}
#endif
	
	if(NULL == ctrl) {
		MYUNLOCK(&ver_ctrl_lock);
		return NULL;	
	}
	
	list_for_each(lh, &(ctrl->dyn_dev_list)) {
		unit = list_entry(lh, dyn_unit_s, list);
		if((PLATFORM_DEVICE_TYPE == unit->type) && (STATUS_REG_OK == unit->status)) {
			if(unit->u.dyn_dev.priv_data == dev->dev.platform_data) {
				MYUNLOCK(&ver_ctrl_lock);
				return unit;	
			}
		}
	}
	
	MYUNLOCK(&ver_ctrl_lock);
	return NULL;	
}

int add_dyn_platform_driver(void *hdl, struct platform_driver *drv,
				char *path, unsigned int line)
{
	version_ctrl_hdl_s *ctrl;
	dyn_unit_s *unit;
	int ret;

	if(NULL == hdl) {
		return -EPERM;	
	}
	
	ctrl = (version_ctrl_hdl_s *)hdl;
	
	unit = create_dyn_unit(drv->driver.name, path, line);
	if(NULL != unit) {
		DBG("add_dyn_platform_driver: add platform driver\n");
		unit->type = PLATFORM_DRIVER_TYPE;
		
		MYLOCK(&ver_ctrl_lock);
		list_add(&(unit->list), &(ctrl->dyn_drv_list));
		MYUNLOCK(&ver_ctrl_lock);
	}
	else {
		return -ENOMEM;	
	}

	if(ctrl->status == STATUS_ALLOW) {
		ret = platform_driver_register(drv);
		if(0 > ret) {
			unit->status = STATUS_REG_FAIL;
		}
		else {
			unit->status = STATUS_REG_OK;
		}
		
		return ret;
	}
	
	unit->status = STATUS_NOT_ALLOW;
	return -EPERM;
}



int add_dyn_platform_device(void *hdl, struct platform_device *dev,
				char *path, unsigned int line) 		
{
	version_ctrl_hdl_s *ctrl;
	dyn_unit_s *unit;
	int ret;

	if(NULL == hdl) {
		return -EPERM;	
	}
	
	ctrl = (version_ctrl_hdl_s *)hdl;
	
	unit = create_dyn_unit(dev->name, path, line);
	if(NULL != unit) {
		unit->type = PLATFORM_DEVICE_TYPE;
		unit->u.dyn_dev.priv_data = dev->dev.platform_data;
		
		/** I use "priv_data" address as the device id.
		* If the device doesn't contain a private data,
		* I need to give it a uniquie address. */
		if(NULL == unit->u.dyn_dev.priv_data) {
			unit->u.dyn_dev.priv_data = (void *)unit;	
		}
		
		MYLOCK(&ver_ctrl_lock);
		list_add(&(unit->list), &(ctrl->dyn_dev_list));
		MYUNLOCK(&ver_ctrl_lock);
		
		DBG("add_dyn_platform_device: add platform device\n");
	}
	else {
		return -ENOMEM;	
	}

	if(ctrl->status == STATUS_ALLOW) {
		ret = platform_device_add(dev);
		if(0 > ret) {
			unit->status = STATUS_REG_FAIL;
		}
		else {
			unit->status = STATUS_REG_OK;
		}
		unit->u.dyn_dev.result = ret;
		
		return ret;
	}
	
	unit->status = STATUS_NOT_ALLOW;
	return -EPERM;
}


int register_dyn_platform_device(void *hdl, struct platform_device *dev,
				char *path, unsigned int line) 		
{
	version_ctrl_hdl_s *ctrl;
	dyn_unit_s *unit;
	int ret;

	if(NULL == hdl) {
		return -EPERM;	
	}
	
	ctrl = (version_ctrl_hdl_s *)hdl;
	
	unit = create_dyn_unit(dev->name, path, line);
	if(NULL != unit) {
		unit->type = PLATFORM_DEVICE_TYPE;
		unit->u.dyn_dev.priv_data = dev->dev.platform_data;
		
		/** I use "priv_data" address as the device id.
		* If the device doesn't contain a private data,
		* I need to give it a uniquie address. */
		if(NULL == unit->u.dyn_dev.priv_data) {
			unit->u.dyn_dev.priv_data = (void *)unit;	
		}
		
		MYLOCK(&ver_ctrl_lock);
		list_add(&(unit->list), &(ctrl->dyn_dev_list));
		MYUNLOCK(&ver_ctrl_lock);
		
		DBG("register_dyn_platform_device: add platform device\n");
	}
	else {
		return -ENOMEM;	
	}

	if(ctrl->status == STATUS_ALLOW) {
		device_initialize(&dev->dev);
		ret = platform_device_add(dev);
		if(0 > ret) {
			unit->status = STATUS_REG_FAIL;
		}
		else {
			unit->status = STATUS_REG_OK;
		}
		unit->u.dyn_dev.result = ret;
		
		return ret;
	}
	
	unit->status = STATUS_NOT_ALLOW;
	return -EPERM;
}


int call_dyn_platform_driver_probe(DRIVER_TYPE_E type, PLATFORM_PROBE fn, struct platform_device *dev)
{
	dyn_unit_s *unit;
	
	unit = dyn_ctrl_valid_platform_device(type, dev);
	if(NULL != unit) {
		if(unit == unit->u.dyn_dev.priv_data) {
			/** In this case, it means original platform 
			* device doesn't have a private data. */
			dev->dev.platform_data = NULL;
		}
		
		unit->status = STATUS_BUNDLE;
		unit->u.dyn_dev.result = fn(dev);
		return unit->u.dyn_dev.result;
	}
	else {
		fn(dev);
	}
	
	return -EPERM;
}








/**
* Dynamic I2C driver/device loading with version control 
***************************************************************************/
dyn_unit_s * dyn_ctrl_valid_i2c_device(DRIVER_TYPE_E type, struct i2c_client *client) 
{
	struct list_head *lh;
	version_ctrl_hdl_s *ctrl;
	dyn_unit_s *unit;


	MYLOCK(&ver_ctrl_lock);
	if(0 == version_ctrl_init[type]) {
		MYUNLOCK(&ver_ctrl_lock);
		return NULL;
	}

#if 0	
	ctrl = NULL;
	list_for_each(lh, &(version_ctrl[type])) {
		ctrl = list_entry(lh, version_ctrl_hdl_s, list);
		if(STATUS_ALLOW == ctrl->status) {
			break;
		}
		ctrl = NULL;
	}
#else
	ctrl = list_first_entry(&(version_ctrl[type]), version_ctrl_hdl_s, list);
	if(STATUS_ALLOW != ctrl->status) {
		ctrl = NULL;
	}
#endif
	
	if(NULL == ctrl) {
		MYUNLOCK(&ver_ctrl_lock);
		return NULL;	
	}
	
	
	list_for_each(lh, &(ctrl->dyn_dev_list)) {
		unit = list_entry(lh, dyn_unit_s, list);
		if((I2C_DEVICE_TYPE == unit->type) && (STATUS_REG_OK == unit->status)) {
			if(unit->u.dyn_dev.priv_data == client->dev.platform_data) {
				MYUNLOCK(&ver_ctrl_lock);
				return unit;	
			}
		}
	}
	
	MYUNLOCK(&ver_ctrl_lock);
	return NULL;	
}

int add_dyn_i2c_driver(void *hdl, struct i2c_driver *drv,
			char *path, unsigned int line)
{
	version_ctrl_hdl_s *ctrl;
	dyn_unit_s *unit;
	int ret;

	if(NULL == hdl) {
		return -EPERM;	
	}
	
	ctrl = (version_ctrl_hdl_s *)hdl;
	
	unit = create_dyn_unit(drv->driver.name, path, line);
	if(NULL != unit) {
		unit->type = I2C_DRIVER_TYPE;
		
		MYLOCK(&ver_ctrl_lock);
		list_add(&(unit->list), &(ctrl->dyn_drv_list));
		MYUNLOCK(&ver_ctrl_lock);
	}
	else {
		return -ENOMEM;	
	}

	if(ctrl->status == STATUS_ALLOW) {
		ret = i2c_add_driver(drv);
		if(0 > ret) {
			unit->status = STATUS_REG_FAIL;
		}
		else {
			unit->status = STATUS_REG_OK;
		}
		
		return ret;
	}
	
	unit->status = STATUS_NOT_ALLOW;
	return -EPERM;
}



int add_dyn_i2c_device(void *hdl, 
		int busnum, struct i2c_board_info *dev,
		char *path, unsigned int line) 		
{
	version_ctrl_hdl_s *ctrl;
	dyn_unit_s *unit;
	int ret;

	if(NULL == hdl) {
		return -EPERM;	
	}
	
	ctrl = (version_ctrl_hdl_s *)hdl;
	
	unit = create_dyn_unit(dev->type, path, line);
	if(NULL != unit) {
		unit->type = I2C_DEVICE_TYPE;
		unit->u.dyn_dev.priv_data = dev->platform_data;
		
		/** I use "priv_data" address as the device id.
		* If the device doesn't contain a private data,
		* I need to give it a uniquie address. */
		if(NULL == unit->u.dyn_dev.priv_data) {
			unit->u.dyn_dev.priv_data = (void *)unit;	
		}
		
		MYLOCK(&ver_ctrl_lock);
		list_add(&(unit->list), &(ctrl->dyn_dev_list));
		MYUNLOCK(&ver_ctrl_lock);
	}
	else {
		return -ENOMEM;	
	}

	if(ctrl->status == STATUS_ALLOW) {
		ret = i2c_register_board_info(busnum, dev, 1);
		if(0 > ret) {
			unit->status = STATUS_REG_FAIL;
		}
		else {
			unit->status = STATUS_REG_OK;
		}
		unit->u.dyn_dev.result = ret;
		
		return ret;
	}
	
	unit->status = STATUS_NOT_ALLOW;
	return -EPERM;
}



int call_dyn_i2c_driver_probe(DRIVER_TYPE_E type, I2C_PROBE fn, 
				struct i2c_client *client, 
				const struct i2c_device_id *id)				
{
	dyn_unit_s *unit;
	
	unit = dyn_ctrl_valid_i2c_device(type, client);
	if(NULL != unit) {
		if(unit == unit->u.dyn_dev.priv_data) {
			/** In this case, it means original platform 
			* device doesn't have a private data. */
			client->dev.platform_data = NULL;
		}
		
		unit->status = STATUS_BUNDLE;
		unit->u.dyn_dev.result = fn(client, id);
		return unit->u.dyn_dev.result;
	}
	else {
		fn(client, id);
	}
	
	return -EPERM;
}



/** 
* Dynamic module loading with version control 
***************************************************************************/
int add_dyn_module(void *hdl, char *name, INIT fn, char *path, unsigned int line) 		
{
	version_ctrl_hdl_s *ctrl;
	dyn_unit_s *unit;
	int ret;

	DBG("add_dyn_module: %s\n", name);
	
	if(NULL == hdl) {
		return -EPERM;	
	}
	
	ctrl = (version_ctrl_hdl_s *)hdl;
	
	unit = create_dyn_unit(name, path, line);
	if(NULL != unit) {
		unit->type = MODULE_TYPE;
		
		MYLOCK(&ver_ctrl_lock);
		list_add(&(unit->list), &(ctrl->dyn_mod_list));
		MYUNLOCK(&ver_ctrl_lock);
	}
	else {
		DBG("add_dyn_module: no mem\n");
		return -ENOMEM;	
	}

	if(ctrl->status == STATUS_ALLOW) {
		unit->u.dyn_mod.result = ret = fn(hdl);
		return ret;
	}
	
	unit->status = STATUS_NOT_ALLOW;
	return -EPERM;
}





/**
* Dynamic Loader Debug Information 
*******************************************/
#ifdef DEBUG
typedef struct _DBGINFO {
	DRIVER_TYPE_E curr_driver_type;
	
	struct list_head *ctrl_head;
	struct list_head *curr_ctrl;
	
	struct list_head *driver_head;
	struct list_head *curr_driver;
	
	struct list_head *device_head;
	struct list_head *curr_device;
	
	struct list_head *module_head;
	struct list_head *curr_module;
	
	int show_version_info;
} DBGINFO_S;


/**
* seq_start() takes a position as an argument and returns an iterator which
* will start reading at that position.
*/
static void* seq_start(struct seq_file *s, loff_t *pos)
{
	DBGINFO_S *dbginfo;
	DRIVER_TYPE_E type;
	version_ctrl_hdl_s *ctrl;
	
	if(*pos >= DRIVER_TYPE_LAST) {
		DBG("seq_start: incorrect *pos\n");
		return NULL;
	}
	
	dbginfo = kzalloc(sizeof(DBGINFO_S), GFP_KERNEL);
	if (!dbginfo) {
		DBG("seq_start: no memory\n");
		return NULL;
	}
	
	DBG("seq_start: current *pos=%lld\n", *pos);
	dbginfo->curr_driver_type = type = *pos;
	MYLOCK(&ver_ctrl_lock);
	if(1 == version_ctrl_init[type]) {
		dbginfo->ctrl_head = &version_ctrl[type];
		if(dbginfo->ctrl_head->next != dbginfo->ctrl_head) {
			dbginfo->curr_ctrl = dbginfo->ctrl_head->next;
			ctrl = list_entry(dbginfo->curr_ctrl, version_ctrl_hdl_s, list);
			
			dbginfo->driver_head = &(ctrl->dyn_drv_list);
			if(dbginfo->driver_head->next != dbginfo->driver_head) {
				dbginfo->curr_driver = dbginfo->driver_head->next;
			}
			
			dbginfo->device_head = &(ctrl->dyn_dev_list);
			if(dbginfo->device_head->next != dbginfo->device_head) {
				dbginfo->curr_device = dbginfo->device_head->next;
			}
			
			dbginfo->module_head = &(ctrl->dyn_mod_list);
			if(dbginfo->module_head->next != dbginfo->module_head) {
				dbginfo->curr_module = dbginfo->module_head->next;
			}
			
			dbginfo->show_version_info = 1;
		}
	}
	MYUNLOCK(&ver_ctrl_lock);
	
	return dbginfo;
}

/**
* move the iterator forward to the next position in the sequence
*/
static void* seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	DRIVER_TYPE_E type;
	DBGINFO_S *dbginfo = (DBGINFO_S *)v;
	version_ctrl_hdl_s *ctrl;
	
	DBG("seq_next: current *pos=%lld\n", *pos);
	if(dbginfo->curr_ctrl == NULL &&
			dbginfo->curr_module == NULL &&
			dbginfo->curr_device == NULL &&
			dbginfo->curr_driver == NULL) {
		*pos = *pos + 1;
		type = (int)(*pos);
		
		DBG("seq_next: move to next type (%d)\n", type);
		
		if(*pos >= DRIVER_TYPE_LAST) {
			return NULL;
		}
		
		memset(dbginfo, 0, sizeof(DBGINFO_S));
		MYLOCK(&ver_ctrl_lock);
		if(1 == version_ctrl_init[type]) {
			dbginfo->ctrl_head = &version_ctrl[type];
			if(dbginfo->ctrl_head->next != dbginfo->ctrl_head) {
				dbginfo->curr_ctrl = dbginfo->ctrl_head->next;
				ctrl = list_entry(dbginfo->curr_ctrl, version_ctrl_hdl_s, list);
				
				dbginfo->driver_head = &(ctrl->dyn_drv_list);
				if(dbginfo->driver_head->next != dbginfo->driver_head) {
					dbginfo->curr_driver = dbginfo->driver_head->next;
					
					DBG("seq_next: set dbginfo->curr_driver\n");
				}
				
				dbginfo->device_head = &(ctrl->dyn_dev_list);
				if(dbginfo->device_head->next != dbginfo->device_head) {
					dbginfo->curr_device = dbginfo->device_head->next;
					
					DBG("seq_next: set dbginfo->curr_device\n");
				}
				
				dbginfo->module_head = &(ctrl->dyn_mod_list);
				if(dbginfo->module_head->next != dbginfo->module_head) {
					dbginfo->curr_module = dbginfo->module_head->next;
					
					DBG("seq_next: set dbginfo->curr_module\n");
				}
				
				dbginfo->show_version_info = 1;
				
				DBG("seq_next: set dbginfo->show_version_info\n");
			}
		}
		MYUNLOCK(&ver_ctrl_lock);
	}
	else if(dbginfo->curr_module == NULL &&
				dbginfo->curr_device == NULL &&
				dbginfo->curr_driver == NULL) {		
		DBG("seq_next: move to next version control\n");
		
		MYLOCK(&ver_ctrl_lock);									
		if(dbginfo->curr_ctrl->next != dbginfo->ctrl_head) {
			dbginfo->curr_ctrl = dbginfo->curr_ctrl->next;
			ctrl = list_entry(dbginfo->curr_ctrl, version_ctrl_hdl_s, list);
			
			dbginfo->driver_head = &(ctrl->dyn_drv_list);
			if(dbginfo->driver_head->next != dbginfo->driver_head) {
				dbginfo->curr_driver = dbginfo->driver_head->next;
				
				DBG("seq_next: set dbginfo->curr_driver\n");
			}
			
			dbginfo->device_head = &(ctrl->dyn_dev_list);
			if(dbginfo->device_head->next != dbginfo->device_head) {
				dbginfo->curr_device = dbginfo->device_head->next;
				
				DBG("seq_next: set dbginfo->curr_device\n");
			}
			
			dbginfo->module_head = &(ctrl->dyn_mod_list);
			if(dbginfo->module_head->next != dbginfo->module_head) {
				dbginfo->curr_module = dbginfo->module_head->next;
				
				DBG("seq_next: set dbginfo->curr_module\n");
			}
			
			dbginfo->show_version_info = 1;
			
			DBG("seq_next: set dbginfo->show_version_info\n");
		}
		else {
			dbginfo->curr_ctrl = NULL;
			dbginfo->show_version_info = 0;
		}
		MYUNLOCK(&ver_ctrl_lock);			
	}
	else if(dbginfo->show_version_info == 1) {
		dbginfo->show_version_info = 0;
		DBG("seq_next: clear dbginfo->show_version_info\n");		
	}
	else if(dbginfo->curr_module != NULL) {
		MYLOCK(&ver_ctrl_lock);
		if(dbginfo->curr_module->next != dbginfo->module_head) {
			dbginfo->curr_module = dbginfo->curr_module->next;
		}
		else {
			dbginfo->curr_module = NULL;
		}
		MYUNLOCK(&ver_ctrl_lock);
		
		DBG("seq_next: move to next module (%X)\n", (int)dbginfo->curr_module);		
	}
	else if(dbginfo->curr_driver != NULL) {
		MYLOCK(&ver_ctrl_lock);
		if(dbginfo->curr_driver->next != dbginfo->driver_head) {
			dbginfo->curr_driver = dbginfo->curr_driver->next;
		}
		else {
			dbginfo->curr_driver = NULL;
		}
		MYUNLOCK(&ver_ctrl_lock);
		
		DBG("seq_next: move to next driver (%X)\n", (int)dbginfo->curr_driver);
	}
	else if(dbginfo->curr_device != NULL) {
		MYLOCK(&ver_ctrl_lock);
		if(dbginfo->curr_device->next != dbginfo->device_head) {
			dbginfo->curr_device = dbginfo->curr_device->next;
		}
		else {
			dbginfo->curr_device = NULL;
		}
		MYUNLOCK(&ver_ctrl_lock);
		
		DBG("seq_next: move to next device (%X)\n", (int)dbginfo->curr_device);
	}
	
	return dbginfo;
}

/**
* stop() is called when iteration is complete (clean up)
*/
static void seq_stop(struct seq_file *s, void *v)
{
	DBG("seq_stop\n");
	kfree(v);
}



/**
* success return 0, otherwise return error code
*/
static int seq_show(struct seq_file *s, void *v)
{
	DBGINFO_S *dbginfo = (DBGINFO_S *)v;
	dyn_unit_s *unit;
	version_ctrl_hdl_s *ctrl;
	
	DBG("seq_show\n");
	
	if(dbginfo->show_version_info == 1) {
		if(dbginfo->curr_ctrl != NULL) {
			ctrl = list_entry(dbginfo->curr_ctrl, version_ctrl_hdl_s, list);
			seq_printf(s, "Version Control: %s, Status: %d\n", ctrl->dbg, ctrl->status);
			DBG("Version Control: %s, Status: %d\n", ctrl->dbg, ctrl->status);
		}
	}
	else if(dbginfo->curr_module != NULL) {
		unit = list_entry(dbginfo->curr_module, dyn_unit_s, list);
		seq_printf(s, "\t%s: %s, Status: %d, Result: %d, Path: %s:%d\n", 
					unit_name[unit->type],
					unit->name, unit->status, unit->u.dyn_mod.result,
					unit->path, unit->line);
		DBG("\t%s: %s, Status: %d, Result: %d, Path: %s:%d\n", 
					unit_name[unit->type],
					unit->name, unit->status, unit->u.dyn_mod.result,
					unit->path, unit->line);			
	}
	else if(dbginfo->curr_driver != NULL) {
		unit = list_entry(dbginfo->curr_driver, dyn_unit_s, list);
		seq_printf(s, "\t%s: %s, Status: %d, Mal-devices: %d, Path: %s:%d\n", 
					unit_name[unit->type],
					unit->name, unit->status, unit->u.dyn_drv.wrong_devs,
					unit->path, unit->line);
		DBG("\t%s: %s, Status: %d, Mal-devices: %d, Path: %s:%d\n", 
					unit_name[unit->type],
					unit->name, unit->status, unit->u.dyn_drv.wrong_devs,
					unit->path, unit->line);			
	}
	else if(dbginfo->curr_device != NULL) {
		unit = list_entry(dbginfo->curr_device, dyn_unit_s, list);
		seq_printf(s, "\t%s: %s, Status: %d, Result: %d, Path: %s:%d\n", 
					unit_name[unit->type],
					unit->name, unit->status, unit->u.dyn_dev.result,
					unit->path, unit->line);
		DBG("\t%s: %s, Status: %d, Result: %d, Path: %s:%d\n", 
					unit_name[unit->type],
					unit->name, unit->status, unit->u.dyn_dev.result,
					unit->path, unit->line);			
	}
	
	return 0;
}


static struct seq_operations seq_ops = {
	.start = seq_start,
	.next = seq_next,
	.stop = seq_stop,
	.show = seq_show
};


static int proc_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &seq_ops);
}


static struct file_operations proc_ops = {
	.owner = THIS_MODULE, 
	.open = proc_open,
	.read = seq_read, 
	.llseek = seq_lseek, 
	.release = seq_release 
};


static int __init init_modules(void)
{
	struct proc_dir_entry *ent;

	ent = create_proc_entry("dynloader", 0, NULL);
	if (ent) {
		ent->proc_fops = &proc_ops;
	}
	return 0;
}

module_init(init_modules);


int dynloader_get_hwcfg(char *buffer, struct kernel_param *kp)
{
	char *newPos;
	char *versionCtrl;
	DRIVER_TYPE_E type;
	
	newPos = buffer;
	versionCtrl = read_hw_cfg();
	
	newPos += sprintf(newPos, "HWCFG:");	
	/*
	newPos += sprintf(newPos, "%X,%X,%X-%X,%X,%X-%X,%X,%X,%X-", 
				g_hwcfg.HWID_Reading_1, g_hwcfg.HWID_Reading_2, g_hwcfg.HWID_Reading_3,
				g_hwcfg.Project_ID, g_hwcfg.Phase_ID, g_hwcfg.Band_ID, 
				g_hwcfg.OMTP, g_hwcfg.HAC, g_hwcfg.SIM, g_hwcfg.DRAM);
	*/
	
	newPos += sprintf(newPos, "%X,%X,%X-%X,%X,%llX-%X,%X,%X,%lX-", 
				p_new_hwid_table->HWID_Reading_1, p_new_hwid_table->HWID_Reading_2, p_new_hwid_table->HWID_Reading_3,
				p_new_hwid_table->Project_ID, p_new_hwid_table->Phase_ID, p_new_hwid_table->Band_ID, 
				p_new_hwid_table->OMTP, p_new_hwid_table->HAC, p_new_hwid_table->SIM, p_new_hwid_table->DRAM);
				
	for(type=DRIVER_TYPE_DEVINFO; type<DRIVER_TYPE_LAST; type++) {	
		newPos += sprintf(newPos, "%X,", *(versionCtrl+type));		
	}
	
	return (newPos - buffer);
}

module_param_call(dynloader_hwcfg, NULL, dynloader_get_hwcfg, NULL, 0444);

#endif



int call_project_init(init_project_s *projects, int num)
{
	int i;
	
	DBG("call_project_init\n");
	read_hw_cfg();
	for(i=0; i<num; i++) {
		//if((int)g_hwcfg.Project_ID == projects[i].project_id) {
		if((int)p_new_hwid_table->Project_ID == projects[i].project_id) {
			DBG("PROJECT ID: %d\n", p_new_hwid_table->Project_ID);
			projects[i].init();
			break;	
		}
	}	
	
	return 0;
}


#if 0
/** 
* Show HW information for user space through Procfs
***************************************************/
static int proc_calc_metrics(char *page, char **start, off_t off,
								int count, int *eof, int len)
{
	if (len <= off+count) 
		*eof = 1;
	*start = page + off;
	len -= off;
	if (len>count) 
		len = count;
	if (len<0) 
		len = 0;
	return len;
}

static int band_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len;
	int pi = g_hwcfg.Band_ID;
	char ver[40];

	switch (pi)
	{
	case GSM_900_1800_CDMA_BC0:
		strcpy( ver, "GSM_BAND_23, CDMA_BAND_0\n");
		break;
	case CDMA_BC0:
		strcpy( ver, "CDMA_BAND_0\n");
		break;
	case WCDMA_145_band:
		strcpy( ver, "GSM_BAND_1234, WCDMA_BAND_145\n");
		break;
	case WCDMA_148_band:
		strcpy( ver, "GSM_BAND_1234, WCDMA_BAND_148\n");
		break;
	case WCDMA_245_band:
		strcpy( ver, "GSM_BAND_1234, WCDMA_BAND_245\n");
		break;
	case WCDMA_15_band:
		strcpy( ver, "GSM_BAND_1234, WCDMA_BAND_15\n");
		break;
	case WCDMA_18_band:
		strcpy( ver, "GSM_BAND_1234, WCDMA_BAND_18\n");
		break;
	case WCDMA_25_band:
		strcpy( ver, "GSM_BAND_1234, WCDMA_BAND_25\n");
		break;
	case WCDMA_125_band:
		strcpy( ver, "GSM_BAND_1234, WCDMA_BAND_125\n");
		break;
	case WCDMA_128_band:
		strcpy( ver, "GSM_BAND_1234, WCDMA_BAND_128\n");
		break;
	case WCDMA_14_band:
			strcpy( ver, "GSM_BAND_1234, WCDMA_BAND_14\n");
			break;
	default:
		strcpy( ver, "Unkonwn RF band id\n");
		break;
	}
	len = snprintf(page, PAGE_SIZE, "%s\n", ver);

	return proc_calc_metrics(page, start, off, count, eof, len);
}

static int device_model_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len;
	int pi = g_hwcfg.Project_ID;
	char ver[24];

	switch (pi){
		case Project_IRM:
			strcpy(ver, "IRM");
			break;
		case Project_IRE:
			strcpy(ver, "IRE");
			break;
		case Project_IMN:
			strcpy(ver, "IMN");
			break;
		case Project_ITV:
			strcpy(ver, "ITV");
			break;
		case Project_IT2:
			strcpy(ver, "IT2");
			break;
		case Project_TBP:
			strcpy(ver, "TBP");
			break;
		case Project_TBE:
			strcpy(ver, "TBE");
			break;
		case Project_TNQ:
			strcpy(ver, "TNQ");
			break;
		case Project_TQ2:
			strcpy(ver, "TQ2");
			break;
		case Project_IRQ:
			strcpy(ver, "IRQ");
			break;
		case Project_NPM:
			strcpy(ver, "NPM");
			break;
		case Project_IMP:
			strcpy(ver, "IMP");
			break;
		case Project_IPD:
			strcpy(ver, "IPD");
			break;
		case Project_TPP:
			strcpy(ver, "TPP");
			break;
		default:
			strcpy(ver, "Unkonwn Device Model");
			break;
	}

	len = snprintf(page, PAGE_SIZE, "%s\n", ver);

	return proc_calc_metrics(page, start, off, count, eof, len);	
}

static int baseband_read_proc(char *page, char **start, off_t off,
							 int count, int *eof, void *data)
{
	int len;
	int pp = g_hwcfg.Phase_ID;
	char ver[24];

	switch (pp){
	case Phase_PR0:
		strcpy(ver, "PR0");
		break;		
	case Phase_PR1:
		strcpy(ver, "PR1");
		break;
	case Phase_PR2:
		strcpy(ver, "PR2");
		break;
	case Phase_PR3:
		strcpy(ver, "PR3");
		break;
	case Phase_PR4:
		strcpy(ver, "PR4");
		break;
	case Phase_PR5:
		strcpy(ver, "PR5");
		break;
	case Phase_PR6:
		strcpy(ver, "PR6");
		break;
	case Phase_MP:
		strcpy(ver, "MP");
		break;	 	 
	default:
		strcpy(ver, "Unkonwn Baseband version");
		break;
	}

	len = snprintf(page, PAGE_SIZE, "%s\n",ver);

	return proc_calc_metrics(page, start, off, count, eof, len);
	}

/*
static int oemrev_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len;
	char *pe;

	pe = fih_get_version();

	len = snprintf(page, PAGE_SIZE, "%s\n", pe);
		
	return proc_calc_metrics(page, start, off, count, eof, len);	
}
*/

static struct {
		char *name;
		int (*read_proc)(char*,char**,off_t,int,int*,void*);
} *p, fxx_info[] = {
	{"devmodel",	device_model_read_proc},
	{"baseband",	baseband_read_proc},
	{"bandinfo",	band_read_proc},
	//{"oemrev",	oemrev_proc},
	{NULL,},
};
#endif
