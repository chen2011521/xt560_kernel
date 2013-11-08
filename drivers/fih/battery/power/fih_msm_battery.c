/* Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 * this needs to be before <linux/kernel.h> is loaded,
 * and <linux/sched.h> loads <linux/kernel.h>
 */
#define DEBUG  0

#include <linux/slab.h>
#include <linux/earlysuspend.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
/* FIH, Debbie, 2011/08/12 { */
/* 1070 porting*/
#include <linux/proc_fs.h>
#include <linux/wakelock.h>
#include <linux/mutex.h>
/* FIH, Debbie, 2011/08/12 } */

#include <asm/atomic.h>

#include <mach/msm_rpcrouter.h>
/* FIH, Debbie, 2011/08/12 { */
/* 1070 porting*/
#include <mach/msm_hsusb.h>
/* FIH, Debbie, 2011/08/12 } */
#include <mach/msm_battery.h>

/* FIH, Debbie, 2011/08/18 { */
/* [workaround]add share memory command for batt temp */
#include "../../../../arch/arm/mach-msm/proc_comm.h"
/* FIH, Debbie, 2011/08/18 } */
#include <linux/delay.h>
#include <mach/rpc_hsusb.h>
/*FIH-NJ-BSP Jerry add. 2011/10/08 +{*/
#include <fih/dynloader.h>
#include "../../../../arch/arm/mach-msm/fih/resources/mach-msm/fih_smd_private.h"
#include "../../../../arch/arm/mach-msm/fih/resources/battery/fih_battery.h" //Jerry, 2011/12/23.
/*FIH-NJ-BSP Jerry add. 2011/10/08 }+*/

#define BATTERY_RPC_PROG	0x30000089
#define BATTERY_RPC_VER_1_1	0x00010001
#define BATTERY_RPC_VER_2_1	0x00020001
#define BATTERY_RPC_VER_4_1     0x00040001
#define BATTERY_RPC_VER_5_1     0x00050001

#define BATTERY_RPC_CB_PROG	(BATTERY_RPC_PROG | 0x01000000)

#define CHG_RPC_PROG		0x3000001a
#define CHG_RPC_VER_1_1		0x00010001
#define CHG_RPC_VER_1_3		0x00010003
#define CHG_RPC_VER_2_2		0x00020002
#define CHG_RPC_VER_3_1         0x00030001
#define CHG_RPC_VER_4_1         0x00040001

#define BATTERY_REGISTER_PROC				2
#define BATTERY_MODIFY_CLIENT_PROC			4
#define BATTERY_DEREGISTER_CLIENT_PROC			5
#define BATTERY_READ_MV_PROC				12
#define BATTERY_ENABLE_DISABLE_FILTER_PROC		14

#define VBATT_FILTER			2

#define BATTERY_CB_TYPE_PROC		1
#define BATTERY_CB_ID_ALL_ACTIV		1
#define BATTERY_CB_ID_LOW_VOL		2

#define BATTERY_LOW		3400
#define BATTERY_HIGH		4200

#define ONCRPC_CHG_GET_GENERAL_STATUS_PROC	12
#define ONCRPC_CHARGER_API_VERSIONS_PROC	0xffffffff

#define BATT_RPC_TIMEOUT    5000	/* 5 sec */

#define INVALID_BATT_HANDLE    -1

#define RPC_TYPE_REQ     0
#define RPC_TYPE_REPLY   1
#define RPC_REQ_REPLY_COMMON_HEADER_SIZE   (3 * sizeof(uint32_t))


#if DEBUG
#define DBG_LIMIT(x...) do {if (printk_ratelimit()) pr_debug(x); } while (0)
#else
#define DBG_LIMIT(x...) do {} while (0)
#endif

enum {
	BATTERY_REGISTRATION_SUCCESSFUL = 0,
	BATTERY_DEREGISTRATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
	BATTERY_MODIFICATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
	BATTERY_INTERROGATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
	BATTERY_CLIENT_TABLE_FULL = 1,
	BATTERY_REG_PARAMS_WRONG = 2,
	BATTERY_DEREGISTRATION_FAILED = 4,
	BATTERY_MODIFICATION_FAILED = 8,
	BATTERY_INTERROGATION_FAILED = 16,
	/* Client's filter could not be set because perhaps it does not exist */
	BATTERY_SET_FILTER_FAILED         = 32,
	/* Client's could not be found for enabling or disabling the individual
	 * client */
	BATTERY_ENABLE_DISABLE_INDIVIDUAL_CLIENT_FAILED  = 64,
	BATTERY_LAST_ERROR = 128,
};

enum {
	BATTERY_VOLTAGE_UP = 0,
	BATTERY_VOLTAGE_DOWN,
	BATTERY_VOLTAGE_ABOVE_THIS_LEVEL,
	BATTERY_VOLTAGE_BELOW_THIS_LEVEL,
	BATTERY_VOLTAGE_LEVEL,
	BATTERY_ALL_ACTIVITY,
	VBATT_CHG_EVENTS,
	BATTERY_VOLTAGE_UNKNOWN,
};

/*
 * This enum contains defintions of the charger hardware status
 */
enum chg_charger_status_type {
	/* The charger is good      */
	CHARGER_STATUS_GOOD,
	/* The charger is bad       */
	CHARGER_STATUS_BAD,
	/* The charger is weak      */
	CHARGER_STATUS_WEAK,
	/* Invalid charger status.  */
	CHARGER_STATUS_INVALID
};

/*
 *This enum contains defintions of the charger hardware type
 */
enum chg_charger_hardware_type {
	/* The charger is removed                 */
	CHARGER_TYPE_NONE,
	/* The charger is a regular wall charger   */
	CHARGER_TYPE_WALL,
	/* The charger is a PC USB                 */
	CHARGER_TYPE_USB_PC,
	/* The charger is a wall USB charger       */
	CHARGER_TYPE_USB_WALL,
	/* The charger is a USB carkit             */
	CHARGER_TYPE_USB_CARKIT,
	/* Invalid charger hardware status.        */
	CHARGER_TYPE_INVALID
};

/*
 *  This enum contains defintions of the battery status
 */
enum chg_battery_status_type {
	/* The battery is good        */
	BATTERY_STATUS_GOOD,
	/* The battery is cold/hot    */
	BATTERY_STATUS_BAD_TEMP,
	/* The battery is bad         */
	BATTERY_STATUS_BAD,
	/* The battery is removed     */
	BATTERY_STATUS_REMOVED,		/* on v2.2 only */
	BATTERY_STATUS_INVALID_v1 = BATTERY_STATUS_REMOVED,
	/* Invalid battery status.    */
	BATTERY_STATUS_INVALID
};

/*
 *This enum contains defintions of the battery voltage level
 */
enum chg_battery_level_type {
	/* The battery voltage is dead/very low (less than 3.2V) */
	BATTERY_LEVEL_DEAD,
	/* The battery voltage is weak/low (between 3.2V and 3.4V) */
	BATTERY_LEVEL_WEAK,
	/* The battery voltage is good/normal(between 3.4V and 4.2V) */
	BATTERY_LEVEL_GOOD,
	/* The battery voltage is up to full (close to 4.2V) */
	BATTERY_LEVEL_FULL,
	/* Invalid battery voltage level. */
	BATTERY_LEVEL_INVALID
};

#ifndef CONFIG_BATTERY_MSM_FAKE
struct rpc_reply_batt_chg_v1 {
	struct rpc_reply_hdr hdr;
	u32 	more_data;

	u32	charger_status;
	u32	charger_type;
	u32	battery_status;
	u32	battery_level;
	u32     battery_voltage;
	u32	battery_temp;
};

struct rpc_reply_batt_chg_v2 {
	struct rpc_reply_batt_chg_v1	v1;

	u32	is_charger_valid;
	u32	is_charging;
	u32	is_battery_valid;
	u32	ui_event;
};

union rpc_reply_batt_chg {
	struct rpc_reply_batt_chg_v1	v1;
	struct rpc_reply_batt_chg_v2	v2;
};

static union rpc_reply_batt_chg rep_batt_chg;
#endif

/* FIH, Debbie, 2011/08/12 { */
/* 1070 porting*/
static DEFINE_MUTEX(charger_status_lock);
/* FIH, Debbie, 2011/08/12 } */

struct msm_battery_info {
	u32 voltage_max_design;
	u32 voltage_min_design;
	u32 chg_api_version;
	u32 batt_technology;
	u32 batt_api_version;

	u32 avail_chg_sources;
	u32 current_chg_source;

	u32 batt_status;
	u32 batt_health;
	u32 charger_valid;
	u32 batt_valid;
	u32 batt_capacity; /* in percentage */

	u32 charger_status;
	u32 charger_type;
	u32 battery_status;
	u32 battery_level;
	u32 battery_voltage; /* in millie volts */
	u32 battery_temp;  /* in celsius */

	u32(*calculate_capacity) (u32 voltage);

	s32 batt_handle;

	struct power_supply *msm_psy_ac;
	struct power_supply *msm_psy_usb;
	struct power_supply *msm_psy_batt;
	struct power_supply *current_ps;

	struct msm_rpc_client *batt_client;
	struct msm_rpc_endpoint *chg_ep;

	wait_queue_head_t wait_q;

	u32 vbatt_modify_reply_avail;

	struct early_suspend early_suspend;
	/* FIH, Debbie, 2011/08/12 { */
	/* 1070 porting*/
	struct wake_lock msm_batt_wakelock;
	
	bool driver_ready;
	/* FIH, Debbie, 2011/08/12 { */
};

/* FIH, Debbie, 2011/08/29 { */
/* add for polling battery status*/
static struct timer_list polling_timer;
//static struct work_struct msm_batt_update;
/* FIH, Debbie, 2011/08/29 } */

static struct msm_battery_info msm_batt_info = {
	.batt_handle = INVALID_BATT_HANDLE,
	.charger_status = CHARGER_STATUS_BAD,
	.charger_type = CHARGER_TYPE_INVALID,
	.battery_status = BATTERY_STATUS_GOOD,
	.battery_level = BATTERY_LEVEL_FULL,
	.battery_voltage = BATTERY_HIGH,
	.batt_capacity = 100,
	.batt_status = POWER_SUPPLY_STATUS_DISCHARGING,
	.batt_health = POWER_SUPPLY_HEALTH_GOOD,
	.batt_valid  = 1,
	.battery_temp = 23,
	.vbatt_modify_reply_avail = 0,
	/* FIH, Debbie, 2011/08/12 { */
	/* 1070 porting*/
	.driver_ready = false,
	/* FIH, Debbie, 2011/08/12 } */
};

/* FIH, Debbie, 2011/08/12 { */
/* 1070 porting*/
enum {
    MSM_BATT_CBVERIFY_OTP_TEST,
    MSM_BATT_CBVERIFY_CHARGING_TEST,
    MSM_BATT_CBVERIFY_REM_OTP,
    MSM_BATT_CBVERIFY_REM_DISCHARING,
    MSM_BATT_CBVERIFY_LOW_BAT_TEST,
    MSM_BATT_CBVERIFY_CHARGING_SWITCH,
    MSM_BATT_CBVERIFY_SET_VALUE_TEMPERATURE, 
    MSM_BATT_CBVERIFY_SET_VALUE_VOLTAGE, 
    MSM_BATT_CBVERIFY_BATT_ID_VALID,
    MSM_BATT_CBVERIFY_MAX,
    MSM_BATT_CBVERIFY_TEST_DISABLE = 88,
    MSM_BATT_CBVERIFY_TEST_ENABLE  = 98,
};

/* FIH, Debbie, 2011/09/22 { */
/* update battery id status */
enum {
	BATTERY_ID_UNKNOWN,
	BATTERY_ID_NOEPROM,
	BATTERY_ID_INVALID,
	BATTERY_ID_VALID,
};
/* FIH, Debbie, 2011/09/22 } */


/* FIH, Debbie, 2011/08/30 { */
/* 1070 porting, report wrong battery id */
extern int hsusb_chg_notify_over_tempearture(bool OT_flag); //FIH, Debbie, 2011/08/25
extern bool w1_bq2024_is_battery_id_valid(void);  //FIH, Debbie, 2011/08/25
extern int check_battery_type(void);  //FIH-NJ Jerry add, 2011/12/22
/* FIH, Debbie, 2011/08/30 }*/
/* FIH, Debbie, 2011/09/22 { */
/* update battery id status */
bool battery_id_valid_flag = true;
bool is_battery_id_read = false;
extern int w1_bq2024_get_battery_id_status(void);
/* FIH, Debbie, 2011/09/22 } */

static struct proc_dir_entry *cbverify_proc_entry;
static int test_value[MSM_BATT_CBVERIFY_MAX];
static bool test_item[MSM_BATT_CBVERIFY_MAX];
static bool test_on = false;
/* FIH, Debbie, 2011/08/12 } */

/* FIH, IvanCHTang, 2011/10/21 { */
/* [IRM], Over temperature protection - [4] */
#define BATTERY_TEMP_LOW_LIMIT 0 //Low temperature limit for charging
#define BATTERY_TEMP_HIGH_LIMIT 45 //High temperature limit for charging
#define BATTERY_THERMAL_TEMP_OFFSET 1 //When charging, the temp. read from thermistor is higher then actual battery temp. 
#define BATTERY_TEMP_HIGH_LIMIT_PART 60 //High temperature limit for partial-charging

#define BATTERY_TEMP_COOL_DOWN_FROM_EMGCY 60//Battery temp. lower than 50 degree leaves cool down mode.
#define BATTERY_TEMP_SHUTDOWN_AP 65 //Battery temperature threshod to shut down AP to cool down device.
#define BATTERY_TEMP_EMGCY_CALL_ONLY 68//Battery temperature threshod for emergency call only.
/* } FIH, IvanCHTang, 2011/10/21 */

static enum power_supply_property msm_power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *msm_power_supplied_to[] = {
	"battery",
};

static int msm_power_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
			val->intval = msm_batt_info.current_chg_source & AC_CHG
			    ? 1 : 0;
		}
		if (psy->type == POWER_SUPPLY_TYPE_USB) {
			val->intval = msm_batt_info.current_chg_source & USB_CHG
			    ? 1 : 0;
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct power_supply msm_psy_ac = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.supplied_to = msm_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(msm_power_supplied_to),
	.properties = msm_power_props,
	.num_properties = ARRAY_SIZE(msm_power_props),
	.get_property = msm_power_get_property,
};

static struct power_supply msm_psy_usb = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.supplied_to = msm_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(msm_power_supplied_to),
	.properties = msm_power_props,
	.num_properties = ARRAY_SIZE(msm_power_props),
	.get_property = msm_power_get_property,
};

static enum power_supply_property msm_batt_power_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
/* FIH, Debbie, 2011/08/12 { */
/* 1070 porting*/
	POWER_SUPPLY_PROP_TEMP,
#ifdef CONFIG_FIH_FTM
	POWER_SUPPLY_PROP_VOLTAGE_AVG,	// DIV2-SW2-BSP-IRM-FTM-BACKUP-BATTERY
#endif
/* FIH, Debbie, 2011/08/12 } */
};

/* FIH, Debbie, 2011/08/12 { */
/* 1070 porting*/
#ifdef CONFIG_FIH_FTM
extern int proc_comm_config_coin_cell(int vset, int *voltage, int status);	// DIV2-SW2-BSP-IRM-FTM-BACKUP-BATTERY
#endif

/* FIH, IvanCHTang, 2011/9/21 { */
/* [IRM], Average the voltages - [1] */
#define FLAG_AVERAGE_VOLT 1
#define VOLT_SAMPLE 10
#define VOLT_IGNORE_TIME 5
#define SET_CHARGE_CURRENT_OFF 0
#define SET_CHARGE_CURRENT_100MA 100
#define SET_CHARGE_CURRENT_500MA 500
#define SET_CHARGE_CURRENT_750MA 750
#define SET_CHARGE_CURRENT_1A 1000
#define BATT_POLLING_TIME 40 //50

/* FIH-NJ-BSP, Jerry removed, defined in fih_battery.h  2011/12/23
typedef struct _VOLT_TO_PERCENT
{
    u16 dwVolt;
    u16 dwPercent;
} VOLT_TO_PERCENT;
FIH-NJ-BSP, Jerry removed, 2011/12/23 */

typedef struct _FIH_BATTERY_PARAM
{
	u16 full_volt;
	u16 chg_ac_current;
	VOLT_TO_PERCENT *p_Volt2percent;
} FIH_BATTERY_PARAM;

static FIH_BATTERY_PARAM g_battery_param;

#if 0
static VOLT_TO_PERCENT g_Volt2Percent_v1[11] =
{
  { 3400, 0},	   // empty,    Rx Table
  { 3567, 10},    // level 1
  { 3630, 20},    // level 2
  { 3665, 30},    // level 3
  { 3693, 40},    // level 4
  { 3732, 50},    // level 5
  { 3783, 60},    // level 6
  { 3849, 70},    // level 7
  { 3915, 80},    // level 8
  { 4000, 90},    // level 9
  { 4100, 100},   // full
};
#else
//use original mapping table
static VOLT_TO_PERCENT g_Volt2Percent_v1[11] =
{
  { 3400, 0},	   // empty,    Rx Table
  { 3562, 10},    // level 1
  { 3616, 20},    // level 2
  { 3651, 30},    // level 3
  { 3678, 40},    // level 4
  { 3711, 50},    // level 5
  { 3756, 60},    // level 6
  { 3818, 70},    // level 7
  { 3887, 80},    // level 8
  { 3973, 90},    // level 9
  { 4100, 100},   // full
};
#endif
static VOLT_TO_PERCENT g_Volt2Percent_v2[11] =
{
	{3400, 0},  	 // empty,    Rx Table
	{3577, 10},      // level 1	
	{3644, 20},      // level 2	
	{3682, 30},      // level 3	
	{3717, 40},      // level 4	
	{3765, 50},      // level 5	
	{3832, 60},      // level 6	
	{3921, 70},      // level 7	
	{4027, 80},      // level 8	
	{4137, 90},      // level 9	
	{4300, 100},     // full
};

bool flag_first_avg_voltage = true;
u32 batt_avg_voltage = 0;
u32 batt_pre_voltage = 0;
u32 batt_new_voltage = 0;
u32 batt_avg_capacity = 0;
u32 time_cycle = 0;
u32 pre_time_cycle = 0;
u32 batt_pre_capacity = 0;
bool flag_is_charging = false;	/* status of charing or discharging */
bool flag_is_charge_full = false;	/* status of charging full or not */
u32 system_time_second = 0;
int batt_avg_temperature = 0;

bool flag_ovt_part_chg = false; //set when enable partial charging
bool flag_ovt = false;			//set when over temp
bool flag_ovt_high_temp = false; //set when temp is extremly high

int test_temperature= 0;
int test_voltage = 0;

void batt_set_charge_current(int mA)
{
	int set_current_mA = mA;
	u32 now_voltage = 0;

	if(!test_item[MSM_BATT_CBVERIFY_SET_VALUE_VOLTAGE])
		now_voltage = batt_avg_voltage;
	else
		now_voltage = test_voltage;

	if (flag_ovt && (mA!=SET_CHARGE_CURRENT_OFF)) {
		set_current_mA = SET_CHARGE_CURRENT_OFF;
		
		if (flag_ovt_part_chg)
			set_current_mA = SET_CHARGE_CURRENT_500MA;
	}

	printk("BATT: %s - set_current_mA(%d) \n", __func__, set_current_mA);
	/* Set the charging current to fit the condition */
	switch(set_current_mA){
        case SET_CHARGE_CURRENT_1A:
        case SET_CHARGE_CURRENT_750MA:
		case SET_CHARGE_CURRENT_500MA:
		case SET_CHARGE_CURRENT_100MA:
			msm_chg_usb_i_is_available(set_current_mA);
			break;
		case SET_CHARGE_CURRENT_OFF:
		default:
			msm_chg_usb_i_is_not_available();
			break;
	}
}

static u32 msm_batt_get_vbatt_voltage(void);

static u32 batt_average_voltage(u32 voltage)
{
	static u32 volt_data[VOLT_SAMPLE] = {0};
	static u32 data_count = 0;
	//static u32 ignore_data_count = 0;
	u32 sum_voltage = 0;
	int i;

	/* Init. voltage list */
	if (flag_first_avg_voltage) {
		DBG_LIMIT("BATT: 1st get voltage data \n");
		for (i = 0; i < VOLT_SAMPLE; i++) {
			volt_data[i] = voltage;
		}
		batt_pre_voltage = batt_new_voltage = voltage;
		flag_first_avg_voltage = false;
	}

	//volt_data[data_count%VOLT_SAMPLE] = batt_pre_voltage;
	volt_data[data_count%VOLT_SAMPLE] = voltage;
	DBG_LIMIT("BATT: %s volt_data[%d] = %d \n", __func__, (data_count%VOLT_SAMPLE), (volt_data[data_count%VOLT_SAMPLE]));
	data_count++;
	
	for (i = 0; i < VOLT_SAMPLE; i++) {
		sum_voltage+=volt_data[i];
	}
	DBG_LIMIT("BATT: %s sum_voltage = %d \n", __func__, sum_voltage);

	return (sum_voltage/VOLT_SAMPLE);
}

//static u32 batt_calculate_voltage(u32 batt_voltage)
static u32 batt_calculate_voltage(void)
{
	static u32 cal_new_voltage = 0;
	static u32 cal_pre_voltage = 0;
	int iC, Vbat_threshold = 0; /* Create different thresholds in diffrerent voltage range to prevent voltage drop issue*/
	u32 sum_voltage = 0;
	static u32 chg_time_duration = 0;
	u32 chg_Vbat_threshold = 0;
	u32 cal_tmp_voltage = 0;
	u32 jC = 0;
	u32 measure_times = 0;

    //FIH-NJ-BSP Jerry add, 2011/12/22. +++
    if( check_battery_type() == BATTERY_TYPE_4V35 )
    {
        g_battery_param.full_volt = 4300;
        g_battery_param.chg_ac_current = SET_CHARGE_CURRENT_750MA;
        g_battery_param.p_Volt2percent = g_Volt2Percent_v2;

    }
    //FIH-NJ-BSP Jerry add, 2011/12/22. ---

	printk("BATT: g_battery_param.full_volt(%d), g_battery_param.chg_ac_current(%d), g_battery_param.p_Volt2percent[10].dwVolt(%d) \n", 
		g_battery_param.full_volt, g_battery_param.chg_ac_current, g_battery_param.p_Volt2percent[10].dwVolt);

	if (batt_pre_capacity!=0) { 
		measure_times = time_cycle/(BATT_POLLING_TIME-5); //40
		if (measure_times>10)
		{
			measure_times=10;
		}
		DBG_LIMIT("BATT: measure_times(%d) \n", measure_times);
	} else {
		measure_times = 1;
	}

	battery_id_valid_flag = w1_bq2024_is_battery_id_valid();
	
	/* In charging mode, voltage go up */
	if (battery_id_valid_flag && 
		flag_is_charging && !flag_is_charge_full && 
		(!flag_ovt || (flag_ovt && flag_ovt_part_chg))) 
		{

		chg_time_duration = get_seconds()-system_time_second;
		if ( (chg_time_duration<3) && (cal_pre_voltage!=0) ) {
			DBG_LIMIT("BATT: %s() - [CHARGE] chg_time_duration less than 3 seconds, return cal_pre_voltage(%d) \n",
						__func__, cal_pre_voltage);
			return cal_pre_voltage;
		}

		/* Disable charging if charger or USB cable exists */
		batt_set_charge_current(SET_CHARGE_CURRENT_OFF);
		printk("BATT: %s - [CHARGE] Disable charging current! \n", __func__);

		msleep(2000);
#if 0
		cal_new_voltage = msm_batt_get_vbatt_voltage();		
		if (cal_new_voltage<=0) {
#endif
		cal_new_voltage = proc_comm_read_battery_voltage();
		//printk("BATT: proc_comm_read_battery_voltage(): cal_new_voltage (%u) from smem \n", cal_new_voltage);
#if 0
		}	
#endif
        /*Create different thresholds in diffrerent voltage range to prevent voltage drop issue*/

        for ( iC=0;iC<=10;iC++ )
        {
            if ( cal_pre_voltage <= g_battery_param.p_Volt2percent[iC].dwVolt )
                break;
        }

        if ( iC==0 )
            chg_Vbat_threshold = g_battery_param.p_Volt2percent[1].dwVolt -g_battery_param.p_Volt2percent[0].dwVolt;
        else if ( iC== 11 ) //avoid out of index
            chg_Vbat_threshold = g_battery_param.p_Volt2percent[10].dwVolt - g_battery_param.p_Volt2percent[9].dwVolt;
        else
            chg_Vbat_threshold=(g_battery_param.p_Volt2percent[iC].dwVolt -g_battery_param.p_Volt2percent[iC-1].dwVolt);

        printk("BATT: %s - [CHARGE] cal_new_voltage(%d), cal_pre_voltage(%d), chg_Vbat_threshold(%d) \n", 
               __func__, cal_new_voltage, cal_pre_voltage, chg_Vbat_threshold);

        /* Enable charging if charger or USB cable exists */
        if ( msm_batt_info.current_chg_source & AC_CHG )
        {
            batt_set_charge_current(g_battery_param.chg_ac_current);
            if ( iC <= 2 )
                chg_Vbat_threshold = chg_Vbat_threshold*5/6;
            else if ( iC <= 7 )
                chg_Vbat_threshold = chg_Vbat_threshold*2/3;
            else if ( iC <= 9 )
                chg_Vbat_threshold = chg_Vbat_threshold*2/5;
            else
                chg_Vbat_threshold = chg_Vbat_threshold/3;
        }
        else if ( msm_batt_info.current_chg_source & USB_CHG )
        {
            batt_set_charge_current(SET_CHARGE_CURRENT_500MA);
            if ( iC <= 2 )
                chg_Vbat_threshold = chg_Vbat_threshold*2/3;
            else if ( iC <= 4 )
                chg_Vbat_threshold = chg_Vbat_threshold*1/2;
            else if ( iC <= 9 )
                chg_Vbat_threshold = chg_Vbat_threshold/3;
            else
                chg_Vbat_threshold = chg_Vbat_threshold*2/5;
        }

        printk("BATT: %s - [CHARGE] chg_Vbat_threshold(%d) \n", __func__, chg_Vbat_threshold);

	    cal_tmp_voltage = cal_new_voltage;
	    for(jC=0; jC<measure_times; jC++)	{
			if ((cal_pre_voltage >0)&&(cal_tmp_voltage>(cal_pre_voltage+chg_Vbat_threshold))) {
				cal_new_voltage = cal_pre_voltage+chg_Vbat_threshold;
			} else {
				cal_new_voltage = cal_tmp_voltage;
			}
			printk("BATT: %s - [CHARGE] 2. cal_new_voltage(%d) \n", __func__, cal_new_voltage);
			/* voltage only go up in charging mode */
			if ((cal_pre_voltage >0)&&(cal_new_voltage < cal_pre_voltage)) {
				DBG_LIMIT("BATT: %s - [CHARGE] cal_new_voltage < cal_pre_voltage, return cal_pre_voltage(%d) \n", 
						__func__, cal_pre_voltage);
				return cal_pre_voltage;
			}

			/* Average charging voltages */
			cal_pre_voltage = batt_average_voltage(cal_new_voltage);
			printk("BATT: %s - [CHARGE] average charging voltage = %d \n",	__func__, cal_pre_voltage);
		}

	}else{
		/* Discharge if AC/USB cable connected & charging full */
		if((flag_is_charging && flag_is_charge_full) || 
			(!battery_id_valid_flag) ||
			(flag_ovt && !flag_ovt_part_chg))
			batt_set_charge_current(SET_CHARGE_CURRENT_OFF);

		for(jC=0; jC<measure_times; jC++)	{
			sum_voltage = 0;
			/* Update the new voltage */
			/* sample voltage 3 times */
			for(iC=0; iC<3; iC++) {
				//sum_voltage += msm_batt_get_vbatt_voltage();
				sum_voltage += proc_comm_read_battery_voltage();
				msleep(20);
			}
			cal_new_voltage = sum_voltage/3;
			if (cal_new_voltage<=0) {
				cal_new_voltage = proc_comm_read_battery_voltage();
				printk("BATT: update cal_new_voltage (%u) from smem \n", cal_new_voltage);
			}		

			/* Init. calculate cal_pre_voltage, cal_new_voltage */
			if (flag_first_avg_voltage) {
				cal_pre_voltage = cal_new_voltage;
			}

            /*Create different thresholds in diffrerent voltage range to prevent voltage drop issue*/

            for ( iC=0;iC<=10;iC++ )
            {
                if ( cal_pre_voltage <= g_battery_param.p_Volt2percent[iC].dwVolt )
                    break;
            }

            printk("BATT: %s - [DISCHARGE] cal_new_voltage(%d), cal_pre_voltage(%d) \n", 
                   __func__, cal_new_voltage, cal_pre_voltage);

            if ( iC==11 )
            {
                iC--; //avoid out of idx
                printk("BATT: %s - out of idx, set iC to %d\r\n", __func__, iC);    
            }
            Vbat_threshold=(g_battery_param.p_Volt2percent[iC].dwVolt - g_battery_param.p_Volt2percent[iC-1].dwVolt);

            if ( iC==2 )
                Vbat_threshold=Vbat_threshold*6/5;
            else if ( iC==1 )
                Vbat_threshold=Vbat_threshold*3/2;
            else if ( iC==0 )
                Vbat_threshold=0;

            printk("BATT: %s - Vbat_threshold[%d]=%d\r\n", __func__, iC, Vbat_threshold);

			/* voltage only go down in discharging mode*/
			if (cal_new_voltage > cal_pre_voltage) {
				DBG_LIMIT("BATT: %s - batt_voltage is higher than cal_pre_voltage!!! \n", __func__);
				return cal_pre_voltage;
			} else if ((cal_pre_voltage-cal_new_voltage) > 300) {
				DBG_LIMIT("BATT: %s - cal_pre_voltage-batt_voltage = %d mV \n", __func__, (cal_pre_voltage-cal_new_voltage));
				return cal_pre_voltage;
			} else if((cal_pre_voltage-cal_new_voltage) > Vbat_threshold) {
				/* Use different thresholds in diffrerent voltage range to prevent voltage drop issue in suspend mode*/
				cal_new_voltage = cal_pre_voltage-Vbat_threshold;
				DBG_LIMIT("BATT: %s - cal_new_voltage = %d \n",	__func__, cal_new_voltage);
			}
			/* Average battery voltages */
			cal_pre_voltage = batt_average_voltage(cal_new_voltage);
			DBG_LIMIT("BATT: %s - average voltage = %d \n",	__func__, cal_pre_voltage);
		}
	}

	return cal_pre_voltage;
}
/* } FIH, IvanCHTang, 2011/9/21 */

/*
(Over heat) : T<0 or T>45
(Cool Down Mode) EG call: 68 < T  
(Alarm Mode) Shut down AP: 65 < T < 68 
(Normal Mode) Back to normal: T<60 
(partial charging) : 45 < T < 60  (when V < 3.8)
*/
static int batt_health_update(void)
{
	int batt_health = POWER_SUPPLY_HEALTH_GOOD;
	int now_temperature = 0;

	if(!test_item[MSM_BATT_CBVERIFY_SET_VALUE_TEMPERATURE])
		now_temperature = batt_avg_temperature;
	else
		now_temperature = test_temperature;
	

	if ( !flag_ovt && 
		(now_temperature< BATTERY_TEMP_LOW_LIMIT-BATTERY_THERMAL_TEMP_OFFSET
		|| BATTERY_TEMP_HIGH_LIMIT+BATTERY_THERMAL_TEMP_OFFSET < now_temperature) )
	{
		flag_ovt=true;
	}else if (flag_ovt && 
		( BATTERY_TEMP_LOW_LIMIT+BATTERY_THERMAL_TEMP_OFFSET <= now_temperature
		&&  now_temperature <= BATTERY_TEMP_HIGH_LIMIT-BATTERY_THERMAL_TEMP_OFFSET) )
	{
		flag_ovt=false;
	}


	if (flag_ovt) 
	{
		printk(KERN_INFO "%s(): in OVT state, now_temperature(%d)\n",__func__, now_temperature);
		/* Over temp. T<-1 or T>46 */
		batt_health = POWER_SUPPLY_HEALTH_OVERHEAT;

		/* for partial charging */
		if (batt_avg_voltage < 3800 &&
			 BATTERY_TEMP_HIGH_LIMIT+BATTERY_THERMAL_TEMP_OFFSET < now_temperature && 
			now_temperature < BATTERY_TEMP_HIGH_LIMIT_PART )
		{
			flag_ovt_part_chg=true;
		}else{
			flag_ovt_part_chg=false;
		}
			
		/* for customer request */
		if (now_temperature > BATTERY_TEMP_EMGCY_CALL_ONLY) {
			/* Temp. is over 68, emgcy call only */
			flag_ovt_high_temp = true;
			batt_health = POWER_SUPPLY_HEALTH_OVERHEAT_EMGCY_CALL_ONLY;
		} else if (now_temperature > BATTERY_TEMP_SHUTDOWN_AP 
				&& now_temperature <= BATTERY_TEMP_EMGCY_CALL_ONLY) {
			/* 65<T<68, shutdown AP */	
			batt_health = POWER_SUPPLY_HEALTH_OVERHEAT_SHUTDOWN_AP;
		} else if (flag_ovt_high_temp && (now_temperature<BATTERY_TEMP_COOL_DOWN_FROM_EMGCY)) {
			/* Enter cool down from emgcy mode */
			flag_ovt_high_temp = false; 
			batt_health = POWER_SUPPLY_HEALTH_OVERHEAT;
		} 
		
	}else{
		printk(KERN_INFO "%s(): in non-OVT state, now_temperature(%d)\n",__func__, now_temperature);
		/* non Over temp */
		flag_ovt = flag_ovt_high_temp = flag_ovt_part_chg = false;
		batt_health = POWER_SUPPLY_HEALTH_GOOD;
	}

	printk("%s():flag_ovt(%d), flag_ovt_part_chg(%d), flag_ovt_high_temp(%d), batt_health(%d) \n", __func__,
			flag_ovt, flag_ovt_part_chg, flag_ovt_high_temp, batt_health);

	return batt_health;
}

/* FIH, IvanCHTang, 2011/9/24 { */
/* [IRM], Fix incorrect battery status - [2] */
#define FLAG_UPDATE_CORRECT_STATUS 1
#define FULL_CHARGING_TIME 5400

static int batt_status_update(void)
{
    int stat_val = POWER_SUPPLY_STATUS_NOT_CHARGING;
    static u32 batt_full_time_start = 0;
    static u32 batt_full_time_duration = 0;

    /* Battery is valid */
    if ( battery_id_valid_flag )
    {
        /* Charging mode */
        //if ( flag_is_charging && (!flag_ovt || (flag_ovt && flag_ovt_part_chg)) )
        if ( flag_is_charging )
        {
            if (!flag_ovt || (flag_ovt && flag_ovt_part_chg))
            {
                stat_val = POWER_SUPPLY_STATUS_CHARGING;
                /* Battery is charged full or not */
    
                if ( batt_avg_voltage > g_battery_param.full_volt )
                {
                    stat_val = POWER_SUPPLY_STATUS_FULL;
                    if ( batt_full_time_start == 0 )
                    {
                        batt_full_time_start = get_seconds();
                        printk("BATT: %s - [CHARGE] batt_full_time_start(%u) \n", __func__, batt_full_time_start);
                    }
    
                    batt_full_time_duration = get_seconds() - batt_full_time_start;
                    if ( batt_full_time_duration > FULL_CHARGING_TIME )
                    {
                        flag_is_charge_full = true;
                        printk("BATT: %s - [CHARGE] Charging full, voltage(%u) \n", 
                               __func__, batt_avg_voltage);
                    }
                }
                else if ( batt_avg_voltage <= g_battery_param.full_volt )
                {
                    if ( batt_full_time_duration >= FULL_CHARGING_TIME )
                    {
                        printk("BATT: %s - [CHARGE] Re-charging, voltage(%u) \n",
                               __func__, batt_avg_voltage);
                    }
                    else
                    {
                        printk("BATT: %s - [CHARGE] charging, voltage(%u) and reset batt_full_time_start!! \n",
                               __func__, batt_avg_voltage);
                    }
                    batt_full_time_start = 0;
                    flag_is_charge_full = false;
                }
            }
            else
                stat_val = POWER_SUPPLY_STATUS_DISCHARGING;
        }
        else
        {
            stat_val = POWER_SUPPLY_STATUS_NOT_CHARGING;
            if ( batt_avg_capacity < 15 )
            {
                //stat_val = POWER_SUPPLY_STATUS_LOW_POWER;
                stat_val = POWER_SUPPLY_STATUS_NOT_CHARGING;
                printk("BATT: %s - batt_avg_capacity(%u), set low power!! \n", __func__, batt_avg_capacity);
            }
        }
    }
    else
    {
        printk("BATT: %s - Invalid battery!!! \n", __func__);
        stat_val = POWER_SUPPLY_STATUS_UNKNOWN;
    }
    return stat_val;
}
/* } FIH, IvanCHTang, 2011/9/24 */

/* FIH, Debbie, 2011/08/12 } */
static int msm_batt_power_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		/* FIH, IvanCHTang, 2011/9/24 { */
		/* [IRM], Fix incorrect battery status - [2] */
		#if FLAG_UPDATE_CORRECT_STATUS
        val->intval = batt_status_update();
		#else
		val->intval = msm_batt_info.batt_status;
		#endif
		/* } FIH, IvanCHTang, 2011/9/24 */
		printk("BATT: POWER_SUPPLY_PROP_STATUS, status(%d)\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		/* FIH, IvanCHTang, 2011/10/21 { */
		/* [IRM], Over temperature protection - [4] */
		#if 1
		val->intval = batt_health_update();
//		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		#else 
		val->intval = msm_batt_info.batt_health;
		#endif
		/* } FIH, IvanCHTang, 2011/10/21 */
		DBG_LIMIT("BATT: POWER_SUPPLY_PROP_HEALTH, health(%d)\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = msm_batt_info.batt_valid;
		printk("BATT: POWER_SUPPLY_PROP_PRESENT, valid(%d)\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = msm_batt_info.batt_technology;
		DBG_LIMIT("BATT: POWER_SUPPLY_PROP_TECHNOLOGY, tech(%d)\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = msm_batt_info.voltage_max_design;
		DBG_LIMIT("BATT: POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN, volt_max(%d)\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = msm_batt_info.voltage_min_design;
		DBG_LIMIT("BATT: POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN, volt_min(%d)\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		/* FIH, IvanCHTang, 2011/9/21 { */
		/* [IRM], Average the voltages - [1] */
		#if FLAG_AVERAGE_VOLT
		if (!test_item[MSM_BATT_CBVERIFY_SET_VALUE_VOLTAGE])
			val->intval = batt_avg_voltage * 1000;
		else
			val->intval = test_voltage*1000;
		#else //FLAG_AVERAGE_VOLT
		/* FIH, Debbie, 2011/08/12 { */
		/* 1070 porting*/
		val->intval = msm_batt_info.battery_voltage * 1000;
		/* FIH, Debbie, 2011/08/12 { */
		#endif //FLAG_AVERAGE_VOLT
		/* } FIH, IvanCHTang, 2011/9/21 */
		DBG_LIMIT("BATT: POWER_SUPPLY_PROP_VOLTAGE_NOW, volt.(%d)\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		/* FIH, IvanCHTang, 2011/9/21 { */
		/* [IRM], Average the voltages - [1] */
		#if FLAG_AVERAGE_VOLT
		/* Update capacity every 30 seconds */
		time_cycle = get_seconds() - pre_time_cycle;
		if ( (time_cycle < 30) && (batt_pre_capacity!=0) ) {
			DBG_LIMIT("BATT: Don't update capacity in 30 seconds!!!\n");
			val->intval = batt_pre_capacity;
		} else {
			batt_avg_voltage = batt_calculate_voltage();
			batt_avg_capacity = msm_batt_info.calculate_capacity(batt_avg_voltage);
			printk("BATT: batt_voltage = %u, batt_avg_voltage = %u mV [batt_avg_capacity = %d%%]\n",
						msm_batt_info.battery_voltage, batt_avg_voltage, batt_avg_capacity);
			val->intval = batt_avg_capacity;	
			batt_pre_capacity = batt_avg_capacity;
			pre_time_cycle = get_seconds();
		}
		#else //FLAG_AVERAGE_VOLT
		val->intval = msm_batt_info.batt_capacity;
		#endif //FLAG_AVERAGE_VOLT
		printk("BATT: POWER_SUPPLY_PROP_CAPACITY, capacity(%d)\n", val->intval);
		break;
/* FIH, Debbie, 2011/08/12 { */
/* 1070 porting*/
	case POWER_SUPPLY_PROP_TEMP:
		
		if (!test_item[MSM_BATT_CBVERIFY_SET_VALUE_TEMPERATURE]) {
			batt_avg_temperature = proc_comm_read_battery_thermal()/10;
		
			if(fih_get_product_id()==Project_IRM){
				if(fih_get_product_phase()<=Phase_PR2){
					printk("IRM with old hw, report fake temperature,(%d)(%d)\n", fih_get_product_id(),fih_get_product_phase());
					batt_avg_temperature=30;
				}
			}
			val->intval = batt_avg_temperature*10;
		} else
			val->intval = test_temperature*10;
		
		printk("BATT: POWER_SUPPLY_PROP_TEMP, temp.(%d),product(%d),phase(%d)\n", 
			val->intval, fih_get_product_id(), fih_get_product_phase());
		break;
// DIV2-SW2-BSP-IRM-FTM-BACKUP-BATTERY+[
#ifdef CONFIG_FIH_FTM
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		proc_comm_config_coin_cell(2, &val->intval, 1);
		DBG_LIMIT("BATT: POWER_SUPPLY_PROP_VOLTAGE_AVG, volt_avg(%d)\n", val->intval);
		break;
#endif
// DIV2-SW2-BSP-IRM-FTM-BACKUP-BATTERY+]
/* FIH, Debbie, 2011/08/12 { */
	default:
		return -EINVAL;
	}
	return 0;
}


static struct power_supply msm_psy_batt = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = msm_batt_power_props,
	.num_properties = ARRAY_SIZE(msm_batt_power_props),
	.get_property = msm_batt_power_get_property,
};


/* FIH, IvanCHTang, 2011/10/6 { */
/* [IRM], Polling function of battery info. - [3] */
struct workqueue_struct *fih_msm_batt_wq;
/* } FIH, IvanCHTang, 2011/10/6 */

#ifndef CONFIG_BATTERY_MSM_FAKE
struct msm_batt_get_volt_ret_data {
	u32 battery_voltage;
};

static int msm_batt_get_volt_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct msm_batt_get_volt_ret_data *data_ptr, *buf_ptr;

	data_ptr = (struct msm_batt_get_volt_ret_data *)data;
	buf_ptr = (struct msm_batt_get_volt_ret_data *)buf;

	data_ptr->battery_voltage = be32_to_cpu(buf_ptr->battery_voltage);

	return 0;
}

static u32 msm_batt_get_vbatt_voltage(void)
{
	int rc;

	struct msm_batt_get_volt_ret_data rep;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_READ_MV_PROC,
			NULL, NULL,
			msm_batt_get_volt_ret_func, &rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: FAIL: vbatt get volt. rc=%d\n", __func__, rc);
		return 0;
	}

	return rep.battery_voltage;
}

#define	be32_to_cpu_self(v)	(v = be32_to_cpu(v))

static int msm_batt_get_batt_chg_status(void)
{
	int rc;

	struct rpc_req_batt_chg {
		struct rpc_request_hdr hdr;
		u32 more_data;
	} req_batt_chg;
	struct rpc_reply_batt_chg_v1 *v1p;

	req_batt_chg.more_data = cpu_to_be32(1);

	memset(&rep_batt_chg, 0, sizeof(rep_batt_chg));

	v1p = &rep_batt_chg.v1;
	rc = msm_rpc_call_reply(msm_batt_info.chg_ep,
				ONCRPC_CHG_GET_GENERAL_STATUS_PROC,
				&req_batt_chg, sizeof(req_batt_chg),
				&rep_batt_chg, sizeof(rep_batt_chg),
				msecs_to_jiffies(BATT_RPC_TIMEOUT));
	if (rc < 0) {
		pr_err("%s: ERROR. msm_rpc_call_reply failed! proc=%d rc=%d\n",
		       __func__, ONCRPC_CHG_GET_GENERAL_STATUS_PROC, rc);
		return rc;
	} else if (be32_to_cpu(v1p->more_data)) {
		be32_to_cpu_self(v1p->charger_status);
		be32_to_cpu_self(v1p->charger_type);
		be32_to_cpu_self(v1p->battery_status);
		be32_to_cpu_self(v1p->battery_level);
		be32_to_cpu_self(v1p->battery_voltage);
		be32_to_cpu_self(v1p->battery_temp);
	} else {
		pr_err("%s: No battery/charger data in RPC reply\n", __func__);
		return -EIO;
	}

	return 0;
}

/* FIH, IvanCHTang, 2011/10/6 { */
/* [IRM], Polling function of battery info. - [3] */
static void polling_timer_func_batt(unsigned long unused)
{
	msm_psy_batt.changed=1;
	queue_work(fih_msm_batt_wq, &msm_psy_batt.changed_work);

	mod_timer(&polling_timer,
			jiffies + msecs_to_jiffies(BATT_POLLING_TIME*1000));
}

/* } FIH, IvanCHTang, 2011/10/6 */

/* FIH, Debbie, 2011/08/29 { */
/* add for polling battery status*/
static void msm_batt_update_psy_status(struct work_struct* work)
//static void msm_batt_update_psy_status(void)
/* FIH, Debbie, 2011/08/29 } */
{
	static u32 unnecessary_event_count;
	u32	charger_status;
	u32	charger_type;
	u32	battery_status;
	u32	battery_level;
	u32     battery_voltage;
	u32	battery_temp;
	struct	power_supply	*supp;

	printk("BATT: %s\n", __func__);

	/* FIH, Debbie, 2011/08/12 { */
	/* 1070 porting*/
	if (msm_batt_info.driver_ready)
		wake_lock(&msm_batt_info.msm_batt_wakelock);
	/* FIH, Debbie, 2011/08/12 } */

	if (msm_batt_get_batt_chg_status())
		return;

	charger_status = rep_batt_chg.v1.charger_status;
	charger_type = rep_batt_chg.v1.charger_type;
	//battery_status = rep_batt_chg.v1.battery_status;
	battery_status = BATTERY_STATUS_GOOD;
	battery_level = rep_batt_chg.v1.battery_level;
	battery_voltage = rep_batt_chg.v1.battery_voltage;
	/* FIH, Debbie, 2011/08/12 { */
	/* 1070 porting*/
	if (test_on && test_item[MSM_BATT_CBVERIFY_OTP_TEST]) 
	{
		battery_temp = test_value[MSM_BATT_CBVERIFY_OTP_TEST];
	}
	else
	{
	/* FIH, Debbie, 2011/08/12 } */
		/* FIH, Debbie, 2011/08/18 { */
		/* [workaround]add share memory command for batt temp */
		battery_temp = proc_comm_read_battery_thermal();
		battery_voltage = proc_comm_read_battery_voltage();
		//battery_temp = rep_batt_chg.v1.battery_temp;
		/* FIH, Debbie, 2011/08/18 } */
	}
	/* Make correction for battery status */
	if (battery_status == BATTERY_STATUS_INVALID_v1) {
		if (msm_batt_info.chg_api_version < CHG_RPC_VER_3_1)
			battery_status = BATTERY_STATUS_INVALID;
	}

	/* Ignore to update batter information */
	if (charger_status == msm_batt_info.charger_status &&
	    charger_type == msm_batt_info.charger_type &&
	    battery_status == msm_batt_info.battery_status &&
	    battery_level == msm_batt_info.battery_level &&
	    battery_voltage == msm_batt_info.battery_voltage &&
	    /* FIH, Debbie, 2011/08/12 { */
	    /* 1070 porting*/
	    battery_temp == msm_batt_info.battery_temp &&
	    !test_on) {
	    //battery_temp == msm_batt_info.battery_temp) {
	    /* FIH, Debbie, 2011/08/12 } */
		/* Got unnecessary event from Modem PMIC VBATT driver.
		 * Nothing changed in Battery or charger status.
		 */
		unnecessary_event_count++;
		/* FIH, Debbie, 2011/08/12 { */
		/* 1070 porting*/
		if ((unnecessary_event_count % 200) == 1)
		//if ((unnecessary_event_count % 20) == 1)
		/* FIH, Debbie, 2011/08/12 { */
			DBG_LIMIT("BATT: same event count = %u\n",
				 unnecessary_event_count);
		if(msm_batt_info.driver_ready)
			wake_unlock(&msm_batt_info.msm_batt_wakelock);
		return;
	}

	unnecessary_event_count = 0;

	DBG_LIMIT("BATT: rcvd: %d, %d, %d, %d; %d, %d\n",
		 charger_status, charger_type, battery_status,
		 battery_level, battery_voltage, battery_temp);

	if (battery_status == BATTERY_STATUS_INVALID &&
	    battery_level != BATTERY_LEVEL_INVALID) {
		DBG_LIMIT("BATT: change status(%d) to (%d) for level=%d\n",
			 battery_status, BATTERY_STATUS_GOOD, battery_level);
		battery_status = BATTERY_STATUS_GOOD;
	}

	/* FIH, Debbie, 2011/08/12 { */
	/* 1070 porting*/
	mutex_lock(&charger_status_lock);
	/* FIH, Debbie, 2011/08/12 } */

	if (msm_batt_info.charger_type != charger_type) {
	/* FIH, Debbie, 2011/08/12 { */
	/* 1070 porting*/
		if (charger_type == CHARGER_TYPE_USB_PC ||
		    charger_type == CHARGER_TYPE_USB_CARKIT) {
			supp = &msm_psy_usb;
		} else if (charger_type == CHARGER_TYPE_USB_WALL ||
				   charger_type == CHARGER_TYPE_WALL) {
			supp = &msm_psy_ac;
		} else {
			supp = &msm_psy_batt;

			/* Correct charger status */
			if (charger_status != CHARGER_STATUS_INVALID) {
				charger_status = CHARGER_STATUS_INVALID;
			}
		}
#if 0
		if (charger_type == CHARGER_TYPE_USB_WALL ||
		    charger_type == CHARGER_TYPE_USB_PC ||
		    charger_type == CHARGER_TYPE_USB_CARKIT) {
			DBG_LIMIT("BATT: USB charger plugged in\n");
			msm_batt_info.current_chg_source = USB_CHG;
			supp = &msm_psy_usb;
		} else if (charger_type == CHARGER_TYPE_WALL) {
			DBG_LIMIT("BATT: AC Wall changer plugged in\n");
			msm_batt_info.current_chg_source = AC_CHG;
			supp = &msm_psy_ac;
		} else {
			if (msm_batt_info.current_chg_source & AC_CHG)
				DBG_LIMIT("BATT: AC Wall charger removed\n");
			else if (msm_batt_info.current_chg_source & USB_CHG)
				DBG_LIMIT("BATT: USB charger removed\n");
			else
				DBG_LIMIT("BATT: No charger present\n");
			msm_batt_info.current_chg_source = 0;
			supp = &msm_psy_batt;

			/* Correct charger status */
			if (charger_status != CHARGER_STATUS_INVALID) {
				DBG_LIMIT("BATT: No charging!\n");
				charger_status = CHARGER_STATUS_INVALID;
				msm_batt_info.batt_status =
					POWER_SUPPLY_STATUS_NOT_CHARGING;
			}
		}
#endif
/* FIH, Debbie, 2011/08/12 } */
	} else
		supp = NULL;

	if (msm_batt_info.charger_status != charger_status) {
		if (charger_status == CHARGER_STATUS_GOOD ||
		    charger_status == CHARGER_STATUS_WEAK) {
			if (msm_batt_info.current_chg_source) {
				/* FIH, Debbie, 2011/08/12 { */
				/* 1070 porting*/
				#if 0
				DBG_LIMIT("BATT: Charging.\n");
				msm_batt_info.batt_status =
					POWER_SUPPLY_STATUS_CHARGING;
				#endif
				/* FIH, Debbie, 2011/08/12 } */

				/* Correct when supp==NULL */
				if (msm_batt_info.current_chg_source & AC_CHG)
					supp = &msm_psy_ac;
				else
					supp = &msm_psy_usb;
			}
		} else {
			/* FIH, Debbie, 2011/08/12 { */
			/* 1070 porting*/
			#if 0
			DBG_LIMIT("BATT: No charging.\n");
			msm_batt_info.batt_status =
				POWER_SUPPLY_STATUS_NOT_CHARGING;
			#endif
			/* FIH, Debbie, 2011/08/12 } */
			supp = &msm_psy_batt;
		}
	} else {
		/* Correct charger status */
		if (charger_type != CHARGER_TYPE_INVALID &&
		    charger_status == CHARGER_STATUS_GOOD) {
			/* FIH, Debbie, 2011/08/12 { */
			/* 1070 porting*/
			#if 0
			DBG_LIMIT("BATT: In charging\n");
			msm_batt_info.batt_status =
				POWER_SUPPLY_STATUS_CHARGING;
			#endif
			/* FIH, Debbie, 2011/08/12 } */
		}
	}

	/* Correct battery voltage and status */
	if (!battery_voltage) {
		if (charger_status == CHARGER_STATUS_INVALID) {
			DBG_LIMIT("BATT: Read VBATT\n");
			battery_voltage = msm_batt_get_vbatt_voltage();
		} else
			/* Use previous */
			battery_voltage = msm_batt_info.battery_voltage;
	}
	if (battery_status == BATTERY_STATUS_INVALID) {
		if (battery_voltage >= msm_batt_info.voltage_min_design &&
		    battery_voltage <= msm_batt_info.voltage_max_design) {
			DBG_LIMIT("BATT: Battery valid\n");
			msm_batt_info.batt_valid = 1;
			battery_status = BATTERY_STATUS_GOOD;
		}
	}

	if (msm_batt_info.battery_status != battery_status) {
		if (battery_status != BATTERY_STATUS_INVALID) {
			msm_batt_info.batt_valid = 1;

			if (battery_status == BATTERY_STATUS_BAD) {
				DBG_LIMIT("BATT: Battery bad.\n");
				msm_batt_info.batt_health =
					POWER_SUPPLY_HEALTH_DEAD;
			} else if (battery_status == BATTERY_STATUS_BAD_TEMP) {
				DBG_LIMIT("BATT: Battery overheat.\n");
				msm_batt_info.batt_health =
					POWER_SUPPLY_HEALTH_OVERHEAT;
			} else {
				DBG_LIMIT("BATT: Battery good.\n");
				msm_batt_info.batt_health =
					POWER_SUPPLY_HEALTH_GOOD;
			}
		}
/* FIH, Debbie, 2011/09/22 { */
#if 0
		else {
			printk(KERN_INFO "battery_status = %d,msm_batt_info.battery_status = %d\n", battery_status,msm_batt_info.battery_status);
			msm_batt_info.batt_valid = 0;
			DBG_LIMIT("BATT: Battery invalid.\n");
			msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_UNKNOWN;
		}
#endif
/* FIH, Debbie, 2011/09/22 } */
		/* FIH, Debbie, 2011/08/12 { */
		/* 1070 porting*/
		#if 0
		if (msm_batt_info.batt_status != POWER_SUPPLY_STATUS_CHARGING) {
			if (battery_status == BATTERY_STATUS_INVALID) {
				DBG_LIMIT("BATT: Battery -> unknown\n");
				msm_batt_info.batt_status =
					POWER_SUPPLY_STATUS_UNKNOWN;
			} else {
				DBG_LIMIT("BATT: Battery -> discharging\n");
				msm_batt_info.batt_status =
					POWER_SUPPLY_STATUS_DISCHARGING;
			}
		}
		#endif
		/* FIH, Debbie, 2011/08/12 } */

		if (!supp) {
			if (msm_batt_info.current_chg_source) {
				if (msm_batt_info.current_chg_source & AC_CHG)
					supp = &msm_psy_ac;
				else
					supp = &msm_psy_usb;
			} else
				supp = &msm_psy_batt;
		}
	}

	/* FIH, Debbie, 2011/08/30 { */
	/* report wrong battery id */
	#if 0
	if (!w1_bq2024_is_battery_id_valid() /*|| OT*/)
	{
		//DBG_LIMIT("BATT: %s Wrong Battery ID\n",__func__);
		printk(KERN_INFO "BATT: %s Wrong Battery ID\n", __func__);
		msm_batt_info.batt_status = POWER_SUPPLY_STATUS_UNKNOWN;
	}
	#endif
	/* FIH, Debbie, 2011/08/30 } */

	msm_batt_info.charger_status 	= charger_status;
	msm_batt_info.charger_type 	= charger_type;
	msm_batt_info.battery_status 	= battery_status;
	msm_batt_info.battery_level 	= battery_level;
	msm_batt_info.battery_temp 	= battery_temp;

	/* FIH, Debbie, 2011/08/12 { */
	/* 1070 porting*/
	if (test_on && test_item[MSM_BATT_CBVERIFY_LOW_BAT_TEST]) {
		msm_batt_info.batt_capacity =
			test_value[MSM_BATT_CBVERIFY_LOW_BAT_TEST];
			
		DBG_LIMIT("BATT Test: [capacity = %d%%]\n",
			 test_value[MSM_BATT_CBVERIFY_LOW_BAT_TEST]);	

		if (!supp)
			supp = msm_batt_info.current_ps;
	} else if (msm_batt_info.battery_voltage != battery_voltage) {
	//if (msm_batt_info.battery_voltage != battery_voltage) {
	/* FIH, Debbie, 2011/08/12 } */
		msm_batt_info.battery_voltage  	= battery_voltage;
		msm_batt_info.batt_capacity =
		msm_batt_info.calculate_capacity(battery_voltage);
		DBG_LIMIT("BATT: voltage = %u mV [capacity = %d%%]\n",
			 battery_voltage, msm_batt_info.batt_capacity);
		/* FIH, IvanCHTang, 2011/9/22 { */
		/* [IRM], Average the voltages - [1] */
		//batt_avg_voltage = batt_average_voltage(msm_batt_info.battery_voltage);
		/* } FIH, IvanCHTang, 2011/9/22 */
		if (!supp)
			supp = msm_batt_info.current_ps;
	}

	/* FIH, Debbie, 2011/08/25 { */
	/* report charging full*/
	if ((msm_batt_info.batt_capacity == 100)&&(msm_batt_info.batt_status == POWER_SUPPLY_STATUS_CHARGING))
	{
		msm_batt_info.batt_status = POWER_SUPPLY_STATUS_FULL;
		DBG_LIMIT("%s(): set the charging status to full\n", __func__);	
	}
	else if (msm_batt_info.batt_status == POWER_SUPPLY_STATUS_FULL) 
	{
		if (msm_batt_info.batt_capacity < 98) 
		{
			msm_batt_info.batt_status = POWER_SUPPLY_STATUS_CHARGING;
			DBG_LIMIT("%s(): set the charging status to charging\n", __func__);	
		}
	}
	else if (msm_batt_info.batt_status == POWER_SUPPLY_STATUS_NOT_CHARGING)
	{
		if (msm_batt_info.batt_capacity < 15) 
		{
			DBG_LIMIT("%s(): set the charging status to low power\n", __func__);	
			msm_batt_info.batt_status = POWER_SUPPLY_STATUS_LOW_POWER;
		}
	}
	/* FIH, Debbie, 2011/08/25 } */
	
	/* FIH, Debbie, 2011/09/22 { */	
	/* update battery id status */
	if(!battery_id_valid_flag)
	{
		//DBG_LIMIT("BATT: %s Wrong Battery ID\n",__func__);
		printk(KERN_INFO "BATT: %s Wrong Battery ID\n", __func__);
		printk(KERN_INFO "BATT: w1_bq2024_get_battery_id_status() = %d, battery_id_valid_flag = %d\n",w1_bq2024_get_battery_id_status(),battery_id_valid_flag);
		battery_status = BATTERY_STATUS_INVALID;
		charger_status = CHARGER_STATUS_INVALID;
		//msm_batt_info.batt_status = POWER_SUPPLY_STATUS_UNKNOWN;
		//msm_batt_info.charger_status = CHARGER_STATUS_INVALID;
	}
	if(battery_status == BATTERY_STATUS_INVALID)
	{
		msm_batt_info.batt_valid = 0;
		msm_batt_info.batt_status = POWER_SUPPLY_STATUS_UNKNOWN;
		msm_batt_info.charger_status = CHARGER_STATUS_INVALID;
	}
	if (!supp)
	{
		supp = msm_batt_info.current_ps;
	}
	/* FIH, Debbie, 2011/09/22 } */	

	if (supp) {
		msm_batt_info.current_ps = supp;
		DBG_LIMIT("BATT: Supply = %s\n", supp->name);
		power_supply_changed(supp);
	}

	/* FIH, Debbie, 2011/08/12 { */
	/* 1070 porting*/
	mutex_unlock(&charger_status_lock);
	
	if (msm_batt_info.driver_ready) {
		wake_unlock(&msm_batt_info.msm_batt_wakelock);
		//wake_lock_timeout(&msm_batt_info.msm_batt_wakelock, 10 * HZ);
	}
	/* FIH, Debbie, 2011/08/12 } */
}


/* FIH, Debbie, 2011/08/12 { */
/* 1070 porting*/
void msm_batt_update_charger_type(enum chg_type charger_type)
{
    if (msm_batt_info.driver_ready)
        wake_lock(&msm_batt_info.msm_batt_wakelock);

    mutex_lock(&charger_status_lock);
    switch (charger_type) {
    case USB_CHG_TYPE__SDP:
        DBG_LIMIT("BATT: USB charger plugged in\n");   
        msm_batt_info.current_chg_source = USB_CHG;
        break;
    case USB_CHG_TYPE__CARKIT:
        DBG_LIMIT("BATT: CAR KIT plugged in\n");
        break;
    case USB_CHG_TYPE__WALLCHARGER:
        DBG_LIMIT("BATT: AC Wall changer plugged in\n");
        msm_batt_info.current_chg_source = AC_CHG;
        break;
    case USB_CHG_TYPE__INVALID:
        if (msm_batt_info.current_chg_source & AC_CHG)
            DBG_LIMIT("BATT: AC Wall charger removed\n");
        else if (msm_batt_info.current_chg_source & USB_CHG)
            DBG_LIMIT("BATT: USB charger removed\n");
        else
            DBG_LIMIT("BATT: No charger present\n");
            
        msm_batt_info.current_chg_source = 0;
        break;
    }
    
    if (msm_batt_info.current_chg_source & (AC_CHG | USB_CHG)) 
	{

		battery_id_valid_flag = w1_bq2024_is_battery_id_valid();
		/* FIH, Debbie, 2011/09/22 { */	
		/* update battery id status */ 
		if(!battery_id_valid_flag)
		/* FIH, Debbie, 2011/09/22 } */	
        {
		/* FIH, Debbie, 2011/08/30 { */
		/* report wrong battery id */
		DBG_LIMIT("BATT: %s Wrong Battery ID\n",__func__);
		msm_batt_info.batt_status = POWER_SUPPLY_STATUS_UNKNOWN;
		msm_batt_info.charger_status = CHARGER_STATUS_INVALID;
		//msm_batt_info.batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
		/* FIH, Debbie, 2011/08/30 } */
        }
        /*else if (msm_batt_info.full)
        {
            msm_batt_info.batt_status = POWER_SUPPLY_STATUS_FULL;
        }
        */
        else
        {
            msm_batt_info.batt_status = POWER_SUPPLY_STATUS_CHARGING;
        }
        /* FIH, IvanCHTang, 2011/9/23 { */
        /* [IRM], Average the voltages - [1] */
        flag_is_charging = true;
        system_time_second = get_seconds();
        /* } FIH, IvanCHTang, 2011/9/23 */
    } else {
        msm_batt_info.batt_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
        /* FIH, IvanCHTang, 2011/9/23 { */
        /* [IRM], Average the voltages - [1] */
        flag_is_charging = false;
        /* } FIH, IvanCHTang, 2011/9/23 */
    }

    mutex_unlock(&charger_status_lock);

    if (msm_batt_info.driver_ready) {
        power_supply_changed(msm_batt_info.msm_psy_ac);
        wake_unlock(&msm_batt_info.msm_batt_wakelock);
        wake_lock_timeout(&msm_batt_info.msm_batt_wakelock, 10 * HZ);
    }
}
EXPORT_SYMBOL(msm_batt_update_charger_type);
/* FIH, Debbie, 2011/08/12 } */

#ifdef CONFIG_HAS_EARLYSUSPEND
struct batt_modify_client_req {

	u32 client_handle;

	/* The voltage at which callback (CB) should be called. */
	u32 desired_batt_voltage;

	/* The direction when the CB should be called. */
	u32 voltage_direction;

	/* The registered callback to be called when voltage and
	 * direction specs are met. */
	u32 batt_cb_id;

	/* The call back data */
	u32 cb_data;
};

struct batt_modify_client_rep {
	u32 result;
};

static int msm_batt_modify_client_arg_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct batt_modify_client_req *batt_modify_client_req =
		(struct batt_modify_client_req *)data;
	u32 *req = (u32 *)buf;
	int size = 0;

	*req = cpu_to_be32(batt_modify_client_req->client_handle);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->desired_batt_voltage);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->voltage_direction);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->batt_cb_id);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->cb_data);
	size += sizeof(u32);

	return size;
}

static int msm_batt_modify_client_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct  batt_modify_client_rep *data_ptr, *buf_ptr;

	data_ptr = (struct batt_modify_client_rep *)data;
	buf_ptr = (struct batt_modify_client_rep *)buf;

	data_ptr->result = be32_to_cpu(buf_ptr->result);

	return 0;
}

static int msm_batt_modify_client(u32 client_handle, u32 desired_batt_voltage,
	     u32 voltage_direction, u32 batt_cb_id, u32 cb_data)
{
	int rc;

	struct batt_modify_client_req  req;
	struct batt_modify_client_rep rep;

	req.client_handle = client_handle;
	req.desired_batt_voltage = desired_batt_voltage;
	req.voltage_direction = voltage_direction;
	req.batt_cb_id = batt_cb_id;
	req.cb_data = cb_data;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_MODIFY_CLIENT_PROC,
			msm_batt_modify_client_arg_func, &req,
			msm_batt_modify_client_ret_func, &rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: ERROR. failed to modify  Vbatt client\n",
		       __func__);
		return rc;
	}

	if (rep.result != BATTERY_MODIFICATION_SUCCESSFUL) {
		pr_err("%s: ERROR. modify client failed. result = %u\n",
		       __func__, rep.result);
		return -EIO;
	}

	return 0;
}

void msm_batt_early_suspend(struct early_suspend *h)
{
	int rc;

	pr_debug("%s: enter\n", __func__);

	if (msm_batt_info.batt_handle != INVALID_BATT_HANDLE) {
		rc = msm_batt_modify_client(msm_batt_info.batt_handle,
				BATTERY_LOW, BATTERY_VOLTAGE_BELOW_THIS_LEVEL,
				BATTERY_CB_ID_LOW_VOL, BATTERY_LOW);

		if (rc < 0) {
			pr_err("%s: msm_batt_modify_client. rc=%d\n",
			       __func__, rc);
			return;
		}
	} else {
		pr_err("%s: ERROR. invalid batt_handle\n", __func__);
		return;
	}

	pr_debug("%s: exit\n", __func__);
}

void msm_batt_late_resume(struct early_suspend *h)
{
	int rc;

	pr_debug("%s: enter\n", __func__);

	if (msm_batt_info.batt_handle != INVALID_BATT_HANDLE) {
		rc = msm_batt_modify_client(msm_batt_info.batt_handle,
				BATTERY_LOW, BATTERY_ALL_ACTIVITY,
			       BATTERY_CB_ID_ALL_ACTIV, BATTERY_ALL_ACTIVITY);
		if (rc < 0) {
			pr_err("%s: msm_batt_modify_client FAIL rc=%d\n",
			       __func__, rc);
			return;
		}
	} else {
		pr_err("%s: ERROR. invalid batt_handle\n", __func__);
		return;
	}

	/* FIH, Debbie, 2011/08/29 { */
	/* add for polling battery status*/
	msm_batt_update_psy_status(NULL);
	//msm_batt_update_psy_status();
	/* FIH, Debbie, 2011/08/29 } */
	pr_debug("%s: exit\n", __func__);
}
#endif

struct msm_batt_vbatt_filter_req {
	u32 batt_handle;
	u32 enable_filter;
	u32 vbatt_filter;
};

struct msm_batt_vbatt_filter_rep {
	u32 result;
};

static int msm_batt_filter_arg_func(struct msm_rpc_client *batt_client,

		void *buf, void *data)
{
	struct msm_batt_vbatt_filter_req *vbatt_filter_req =
		(struct msm_batt_vbatt_filter_req *)data;
	u32 *req = (u32 *)buf;
	int size = 0;

	*req = cpu_to_be32(vbatt_filter_req->batt_handle);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(vbatt_filter_req->enable_filter);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(vbatt_filter_req->vbatt_filter);
	size += sizeof(u32);
	return size;
}

static int msm_batt_filter_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{

	struct msm_batt_vbatt_filter_rep *data_ptr, *buf_ptr;

	data_ptr = (struct msm_batt_vbatt_filter_rep *)data;
	buf_ptr = (struct msm_batt_vbatt_filter_rep *)buf;

	data_ptr->result = be32_to_cpu(buf_ptr->result);
	return 0;
}

static int msm_batt_enable_filter(u32 vbatt_filter)
{
	int rc;
	struct  msm_batt_vbatt_filter_req  vbatt_filter_req;
	struct  msm_batt_vbatt_filter_rep  vbatt_filter_rep;

	vbatt_filter_req.batt_handle = msm_batt_info.batt_handle;
	vbatt_filter_req.enable_filter = 1;
	vbatt_filter_req.vbatt_filter = vbatt_filter;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_ENABLE_DISABLE_FILTER_PROC,
			msm_batt_filter_arg_func, &vbatt_filter_req,
			msm_batt_filter_ret_func, &vbatt_filter_rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: FAIL: enable vbatt filter. rc=%d\n",
		       __func__, rc);
		return rc;
	}

	if (vbatt_filter_rep.result != BATTERY_DEREGISTRATION_SUCCESSFUL) {
		pr_err("%s: FAIL: enable vbatt filter: result=%d\n",
		       __func__, vbatt_filter_rep.result);
		return -EIO;
	}

	pr_debug("%s: enable vbatt filter: OK\n", __func__);
	return rc;
}

struct batt_client_registration_req {
	/* The voltage at which callback (CB) should be called. */
	u32 desired_batt_voltage;

	/* The direction when the CB should be called. */
	u32 voltage_direction;

	/* The registered callback to be called when voltage and
	 * direction specs are met. */
	u32 batt_cb_id;

	/* The call back data */
	u32 cb_data;
	u32 more_data;
	u32 batt_error;
};

struct batt_client_registration_req_4_1 {
	/* The voltage at which callback (CB) should be called. */
	u32 desired_batt_voltage;

	/* The direction when the CB should be called. */
	u32 voltage_direction;

	/* The registered callback to be called when voltage and
	 * direction specs are met. */
	u32 batt_cb_id;

	/* The call back data */
	u32 cb_data;
	u32 batt_error;
};

struct batt_client_registration_rep {
	u32 batt_handle;
};

struct batt_client_registration_rep_4_1 {
	u32 batt_handle;
	u32 more_data;
	u32 err;
};

static int msm_batt_register_arg_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct batt_client_registration_req *batt_reg_req =
		(struct batt_client_registration_req *)data;

	u32 *req = (u32 *)buf;
	int size = 0;


	if (msm_batt_info.batt_api_version == BATTERY_RPC_VER_4_1) {
		*req = cpu_to_be32(batt_reg_req->desired_batt_voltage);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->voltage_direction);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->batt_cb_id);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->cb_data);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->batt_error);
		size += sizeof(u32);

		return size;
	} else {
		*req = cpu_to_be32(batt_reg_req->desired_batt_voltage);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->voltage_direction);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->batt_cb_id);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->cb_data);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->more_data);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->batt_error);
		size += sizeof(u32);

		return size;
	}

}

static int msm_batt_register_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct batt_client_registration_rep *data_ptr, *buf_ptr;
	struct batt_client_registration_rep_4_1 *data_ptr_4_1, *buf_ptr_4_1;

	if (msm_batt_info.batt_api_version == BATTERY_RPC_VER_4_1) {
		data_ptr_4_1 = (struct batt_client_registration_rep_4_1 *)data;
		buf_ptr_4_1 = (struct batt_client_registration_rep_4_1 *)buf;

		data_ptr_4_1->batt_handle
			= be32_to_cpu(buf_ptr_4_1->batt_handle);
		data_ptr_4_1->more_data
			= be32_to_cpu(buf_ptr_4_1->more_data);
		data_ptr_4_1->err = be32_to_cpu(buf_ptr_4_1->err);
		return 0;
	} else {
		data_ptr = (struct batt_client_registration_rep *)data;
		buf_ptr = (struct batt_client_registration_rep *)buf;

		data_ptr->batt_handle = be32_to_cpu(buf_ptr->batt_handle);
		return 0;
	}
}

static int msm_batt_register(u32 desired_batt_voltage,
			     u32 voltage_direction, u32 batt_cb_id, u32 cb_data)
{
	struct batt_client_registration_req batt_reg_req;
	struct batt_client_registration_req_4_1 batt_reg_req_4_1;
	struct batt_client_registration_rep batt_reg_rep;
	struct batt_client_registration_rep_4_1 batt_reg_rep_4_1;
	void *request;
	void *reply;
	int rc;

	if (msm_batt_info.batt_api_version == BATTERY_RPC_VER_4_1) {
		batt_reg_req_4_1.desired_batt_voltage = desired_batt_voltage;
		batt_reg_req_4_1.voltage_direction = voltage_direction;
		batt_reg_req_4_1.batt_cb_id = batt_cb_id;
		batt_reg_req_4_1.cb_data = cb_data;
		batt_reg_req_4_1.batt_error = 1;
		request = &batt_reg_req_4_1;
	} else {
		batt_reg_req.desired_batt_voltage = desired_batt_voltage;
		batt_reg_req.voltage_direction = voltage_direction;
		batt_reg_req.batt_cb_id = batt_cb_id;
		batt_reg_req.cb_data = cb_data;
		batt_reg_req.more_data = 1;
		batt_reg_req.batt_error = 0;
		request = &batt_reg_req;
	}

	if (msm_batt_info.batt_api_version == BATTERY_RPC_VER_4_1)
		reply = &batt_reg_rep_4_1;
	else
		reply = &batt_reg_rep;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_REGISTER_PROC,
			msm_batt_register_arg_func, request,
			msm_batt_register_ret_func, reply,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: FAIL: vbatt register. rc=%d\n", __func__, rc);
		return rc;
	}

	if (msm_batt_info.batt_api_version == BATTERY_RPC_VER_4_1) {
		if (batt_reg_rep_4_1.more_data != 0
			&& batt_reg_rep_4_1.err
				!= BATTERY_REGISTRATION_SUCCESSFUL) {
			pr_err("%s: vBatt Registration Failed proc_num=%d\n"
					, __func__, BATTERY_REGISTER_PROC);
			return -EIO;
		}
		msm_batt_info.batt_handle = batt_reg_rep_4_1.batt_handle;
	} else
		msm_batt_info.batt_handle = batt_reg_rep.batt_handle;

	return 0;
}

struct batt_client_deregister_req {
	u32 batt_handle;
};

struct batt_client_deregister_rep {
	u32 batt_error;
};

static int msm_batt_deregister_arg_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct batt_client_deregister_req *deregister_req =
		(struct  batt_client_deregister_req *)data;
	u32 *req = (u32 *)buf;
	int size = 0;

	*req = cpu_to_be32(deregister_req->batt_handle);
	size += sizeof(u32);

	return size;
}

static int msm_batt_deregister_ret_func(struct msm_rpc_client *batt_client,
				       void *buf, void *data)
{
	struct batt_client_deregister_rep *data_ptr, *buf_ptr;

	data_ptr = (struct batt_client_deregister_rep *)data;
	buf_ptr = (struct batt_client_deregister_rep *)buf;

	data_ptr->batt_error = be32_to_cpu(buf_ptr->batt_error);

	return 0;
}

static int msm_batt_deregister(u32 batt_handle)
{
	int rc;
	struct batt_client_deregister_req req;
	struct batt_client_deregister_rep rep;

	req.batt_handle = batt_handle;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
			BATTERY_DEREGISTER_CLIENT_PROC,
			msm_batt_deregister_arg_func, &req,
			msm_batt_deregister_ret_func, &rep,
			msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: FAIL: vbatt deregister. rc=%d\n", __func__, rc);
		return rc;
	}

	if (rep.batt_error != BATTERY_DEREGISTRATION_SUCCESSFUL) {
		pr_err("%s: vbatt deregistration FAIL. error=%d, handle=%d\n",
		       __func__, rep.batt_error, batt_handle);
		return -EIO;
	}

	return 0;
}
#endif  /* CONFIG_BATTERY_MSM_FAKE */

static int msm_batt_cleanup(void)
{
	int rc = 0;

#ifndef CONFIG_BATTERY_MSM_FAKE
	if (msm_batt_info.batt_handle != INVALID_BATT_HANDLE) {

		rc = msm_batt_deregister(msm_batt_info.batt_handle);
		if (rc < 0)
			pr_err("%s: FAIL: msm_batt_deregister. rc=%d\n",
			       __func__, rc);
	}

	msm_batt_info.batt_handle = INVALID_BATT_HANDLE;

	if (msm_batt_info.batt_client)
		msm_rpc_unregister_client(msm_batt_info.batt_client);
#endif  /* CONFIG_BATTERY_MSM_FAKE */

	if (msm_batt_info.msm_psy_ac)
		power_supply_unregister(msm_batt_info.msm_psy_ac);

	if (msm_batt_info.msm_psy_usb)
		power_supply_unregister(msm_batt_info.msm_psy_usb);
	if (msm_batt_info.msm_psy_batt)
		power_supply_unregister(msm_batt_info.msm_psy_batt);

#ifndef CONFIG_BATTERY_MSM_FAKE
	if (msm_batt_info.chg_ep) {
		rc = msm_rpc_close(msm_batt_info.chg_ep);
		if (rc < 0) {
			pr_err("%s: FAIL. msm_rpc_close(chg_ep). rc=%d\n",
			       __func__, rc);
		}
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	if (msm_batt_info.early_suspend.suspend == msm_batt_early_suspend)
		unregister_early_suspend(&msm_batt_info.early_suspend);
#endif
#endif
	return rc;
}

static u32 msm_batt_capacity(u32 current_voltage)
{
	u32 low_voltage = msm_batt_info.voltage_min_design;
	u32 high_voltage = msm_batt_info.voltage_max_design;

	if (current_voltage <= low_voltage)
		return 0;
	else if (current_voltage >= high_voltage)
		return 100;
	else
		return (current_voltage - low_voltage) * 100
			/ (high_voltage - low_voltage);
}

#ifndef CONFIG_BATTERY_MSM_FAKE
int msm_batt_get_charger_api_version(void)
{
	int rc ;
	struct rpc_reply_hdr *reply;

	struct rpc_req_chg_api_ver {
		struct rpc_request_hdr hdr;
		u32 more_data;
	} req_chg_api_ver;

	struct rpc_rep_chg_api_ver {
		struct rpc_reply_hdr hdr;
		u32 num_of_chg_api_versions;
		u32 *chg_api_versions;
	};

	u32 num_of_versions;

	struct rpc_rep_chg_api_ver *rep_chg_api_ver;


	req_chg_api_ver.more_data = cpu_to_be32(1);

	msm_rpc_setup_req(&req_chg_api_ver.hdr, CHG_RPC_PROG, CHG_RPC_VER_1_1,
			  ONCRPC_CHARGER_API_VERSIONS_PROC);

	rc = msm_rpc_write(msm_batt_info.chg_ep, &req_chg_api_ver,
			sizeof(req_chg_api_ver));
	if (rc < 0) {
		pr_err("%s: FAIL: msm_rpc_write. proc=0x%08x, rc=%d\n",
		       __func__, ONCRPC_CHARGER_API_VERSIONS_PROC, rc);
		return rc;
	}

	for (;;) {
		rc = msm_rpc_read(msm_batt_info.chg_ep, (void *) &reply, -1,
				BATT_RPC_TIMEOUT);
		if (rc < 0)
			return rc;
		if (rc < RPC_REQ_REPLY_COMMON_HEADER_SIZE) {
			pr_err("%s: LENGTH ERR: msm_rpc_read. rc=%d (<%d)\n",
			       __func__, rc, RPC_REQ_REPLY_COMMON_HEADER_SIZE);

			rc = -EIO;
			break;
		}
		/* we should not get RPC REQ or call packets -- ignore them */
		if (reply->type == RPC_TYPE_REQ) {
			pr_err("%s: TYPE ERR: type=%d (!=%d)\n",
			       __func__, reply->type, RPC_TYPE_REQ);
			kfree(reply);
			continue;
		}

		/* If an earlier call timed out, we could get the (no
		 * longer wanted) reply for it.	 Ignore replies that
		 * we don't expect
		 */
		if (reply->xid != req_chg_api_ver.hdr.xid) {
			pr_err("%s: XID ERR: xid=%d (!=%d)\n", __func__,
			       reply->xid, req_chg_api_ver.hdr.xid);
			kfree(reply);
			continue;
		}
		if (reply->reply_stat != RPCMSG_REPLYSTAT_ACCEPTED) {
			rc = -EPERM;
			break;
		}
		if (reply->data.acc_hdr.accept_stat !=
				RPC_ACCEPTSTAT_SUCCESS) {
			rc = -EINVAL;
			break;
		}

		rep_chg_api_ver = (struct rpc_rep_chg_api_ver *)reply;

		num_of_versions =
			be32_to_cpu(rep_chg_api_ver->num_of_chg_api_versions);

		rep_chg_api_ver->chg_api_versions =  (u32 *)
			((u8 *) reply + sizeof(struct rpc_reply_hdr) +
			sizeof(rep_chg_api_ver->num_of_chg_api_versions));

		rc = be32_to_cpu(
			rep_chg_api_ver->chg_api_versions[num_of_versions - 1]);

		pr_debug("%s: num_of_chg_api_versions = %u. "
			"The chg api version = 0x%08x\n", __func__,
			num_of_versions, rc);
		break;
	}
	kfree(reply);
	return rc;
}

static int msm_batt_cb_func(struct msm_rpc_client *client,
			    void *buffer, int in_size)
{
	int rc = 0;
	struct rpc_request_hdr *req;
	u32 procedure;
	u32 accept_status;

	req = (struct rpc_request_hdr *)buffer;
	procedure = be32_to_cpu(req->procedure);

	switch (procedure) {
	case BATTERY_CB_TYPE_PROC:
		accept_status = RPC_ACCEPTSTAT_SUCCESS;
		break;

	default:
		accept_status = RPC_ACCEPTSTAT_PROC_UNAVAIL;
		pr_err("%s: ERROR. procedure (%d) not supported\n",
		       __func__, procedure);
		break;
	}

	msm_rpc_start_accepted_reply(msm_batt_info.batt_client,
			be32_to_cpu(req->xid), accept_status);

	rc = msm_rpc_send_accepted_reply(msm_batt_info.batt_client, 0);
	if (rc)
		pr_err("%s: FAIL: sending reply. rc=%d\n", __func__, rc);

	if (accept_status == RPC_ACCEPTSTAT_SUCCESS)
	/* FIH, Debbie, 2011/08/29 { */
	/* add for polling battery status*/
	{
		/* FIH, IvanCHTang, 2011/10/6 { */
		/* [IRM], Polling function of battery info. - [3] */
		#if 0 // Removed by IvanCHTang++
		msm_batt_update_psy_status(NULL);
		//msm_batt_update_psy_status();
		#endif // Removed by IvanCHTang--
		/* } FIH, IvanCHTang, 2011/10/6 */
	}
	/* FIH, Debbie, 2011/08/29 } */
	return rc;
}
#endif  /* CONFIG_BATTERY_MSM_FAKE */

/* FIH, Debbie, 2011/08/12 { */
/* 1070 porting*/
static int
proc_calc_metrics(char *page, char **start, off_t off,
                 int count, int *eof, int len)
{
    if (len <= off+count) *eof = 1;
    *start = page + off;
    len -= off;
    if (len > count) len = count;
    if (len < 0) len = 0;
    return len;
}

static int
msm_batt_cbverify_proc_read(char *page, char **start, off_t off,
              int count, int *eof, void *data)
{
    char buf[256];
    int i = 0;
    int len;
    
    printk(KERN_INFO "procfile_read (/proc/cbverify) called\n");
    
    if (test_on)
        snprintf(buf, 256, "TEST ON\n");
    else
        snprintf(buf, 256, "TEST OFF\n");
            
    for (i = 0; (i < MSM_BATT_CBVERIFY_MAX); i++) {
        snprintf(buf, 256, "%s%02d ", buf, i);
        switch (i) {
        case MSM_BATT_CBVERIFY_OTP_TEST:
            snprintf(buf, 256, "%sOTP", buf);
            break;
        case MSM_BATT_CBVERIFY_CHARGING_TEST:
            snprintf(buf, 256, "%sCHG", buf);
            break;
        case MSM_BATT_CBVERIFY_REM_OTP:
            snprintf(buf, 256, "%sREMOTP", buf);
            break;
        case MSM_BATT_CBVERIFY_REM_DISCHARING:
            snprintf(buf, 256, "%sREMDIS", buf);
            break;
        case MSM_BATT_CBVERIFY_LOW_BAT_TEST:
            snprintf(buf, 256, "%sLOWBAT", buf);
            break;
        case MSM_BATT_CBVERIFY_CHARGING_SWITCH:
            snprintf(buf, 256, "%sCHGSW", buf);
			case MSM_BATT_CBVERIFY_SET_VALUE_TEMPERATURE:
				snprintf(buf, 256, "%sSET_TEMP", buf);
			case MSM_BATT_CBVERIFY_SET_VALUE_VOLTAGE:
				snprintf(buf, 256, "%sSET_VOLT", buf);
			case MSM_BATT_CBVERIFY_BATT_ID_VALID:
				snprintf(buf, 256, "%sBATT_ID", buf);
        }
        
        if (test_item[i]) {
            snprintf(buf, 256, "%s[T]", buf);
            if (i != MSM_BATT_CBVERIFY_REM_OTP &&
                i != MSM_BATT_CBVERIFY_REM_DISCHARING &&
                i != MSM_BATT_CBVERIFY_CHARGING_SWITCH)
                snprintf(buf, 256, "%s%d\n", buf, test_value[i]);
            else
                snprintf(buf, 256, "%s\n", buf);
		} 
		else if (i == MSM_BATT_CBVERIFY_BATT_ID_VALID) {
			if (w1_bq2024_is_battery_id_valid())
				snprintf(buf, 256, "%s [Valid]\n", buf);
			else
				snprintf(buf, 256, "%s [Invalid]\n", buf);
		}
		else
            snprintf(buf, 256, "%s[F]\n", buf);
    }

#if defined(CONFIG_FIH_POWER_LOG)
    pmlog("%s\n", buf);
#endif
    len = snprintf(page, PAGE_SIZE, "%s", buf);
        
    return proc_calc_metrics(page, start, off, count, eof, len);
}

static int
msm_batt_cbverify_proc_write(struct file *file, const char __user *buffer,
               unsigned long count, void *data)
{
    char cmd[256];
    int i = 0;
    int item = 0;
    int on = 0;
    int value = 0;
    
    printk(KERN_INFO "procfile_write (/proc/cbverify) called\n");
    
    if (copy_from_user(cmd, buffer, count)) {
        return -EFAULT;
    } else {
        sscanf(cmd, "%d %d %d",  &item, &on, &value);
		printk("BATT: write test - item(%d) on(%d) value(%d) \n",  item, on, value);
        
        switch (item) {
        case MSM_BATT_CBVERIFY_OTP_TEST:
        case MSM_BATT_CBVERIFY_CHARGING_TEST:
        case MSM_BATT_CBVERIFY_LOW_BAT_TEST:
            test_value[item] = value;
        case MSM_BATT_CBVERIFY_REM_OTP:
        case MSM_BATT_CBVERIFY_REM_DISCHARING:
		case MSM_BATT_CBVERIFY_CHARGING_SWITCH:
            test_item[item] = on;
            break;
			case MSM_BATT_CBVERIFY_SET_VALUE_TEMPERATURE:
				test_item[item]= on; 
				test_temperature = value;
			break;
			case MSM_BATT_CBVERIFY_SET_VALUE_VOLTAGE:
				test_item[item]= on;
				test_voltage = value;
			break;
        case MSM_BATT_CBVERIFY_TEST_ENABLE:
            test_on = true;
            if (test_item[MSM_BATT_CBVERIFY_OTP_TEST] || test_item[MSM_BATT_CBVERIFY_LOW_BAT_TEST]) 
            /* FIH, Debbie, 2011/08/29 { */
            /* add for polling battery status*/
            {
                msm_batt_update_psy_status(NULL);
                //msm_batt_update_psy_status();
            }
            /* FIH, Debbie, 2011/08/29 } */
            /*
            if (test_item[MSM_BATT_CBVERIFY_CHARGING_SWITCH])
				hsusb_chg_notify_over_tempearture(true);
            msm_batt_disable_discharging_monitor(test_item[MSM_BATT_CBVERIFY_REM_DISCHARING]);*/
            break;
        case MSM_BATT_CBVERIFY_TEST_DISABLE:
            test_on = false;
            if (test_item[MSM_BATT_CBVERIFY_OTP_TEST] || test_item[MSM_BATT_CBVERIFY_LOW_BAT_TEST]) 
            /* FIH, Debbie, 2011/08/29 { */
            /* add for polling battery status*/
            {
                msm_batt_update_psy_status(NULL);
                //msm_batt_update_psy_status();
            }
            /* FIH, Debbie, 2011/08/29 } */
            /*
            if (test_item[MSM_BATT_CBVERIFY_REM_DISCHARING])
                msm_batt_disable_discharging_monitor(false);
            if (test_item[MSM_BATT_CBVERIFY_CHARGING_SWITCH])
				hsusb_chg_notify_over_tempearture(false);*/
            for (i = 0; i < MSM_BATT_CBVERIFY_MAX; i++) {
                test_item[i] = false;
                test_value[i] = 0;
            }
        }
        return count;
    }
        
    return count;
}
/* FIH, Debbie, 2011/08/12 } */

/* FIH, Debbie, 2011/08/29 { */
/* add for polling battery status*/
#if 0 // Removed by IvanCHTang++
static void msm_battery_polling_timer_func(unsigned long unused)
{
	/* FIH, Debbie, 2011/09/22 { */	
	/* update battery id status */ 
	//printk(KERN_INFO "%s\n",__func__);
	if(!is_battery_id_read)
	{
		printk(KERN_INFO "read battery id\n");
		if((!w1_bq2024_is_battery_id_valid())||(w1_bq2024_get_battery_id_status() == BATTERY_ID_NOEPROM))
		{
			printk(KERN_INFO "INVLAID BATTERY ID\n");
			battery_id_valid_flag = false;
		}
		is_battery_id_read = true;
	}
	/* FIH, Debbie, 2011/09/22 } */	

	schedule_work(&msm_batt_update);
	mod_timer(&polling_timer,
            jiffies + msecs_to_jiffies(60000)); /* FIH, Debbie, 2011/09/22 */	
}
#endif // Removed by IvanCHTang--
/* FIH, Debbie, 2011/08/29 } */

static int msm_batt_probe(struct platform_device *pdev)
{
	/* FIH, Debbie, 2011/08/12 { */
	/* 1070 porting*/
	int rc, i;
	//int rc;
	/* FIH, Debbie, 2011/08/12 } */
	struct msm_psy_batt_pdata *pdata = pdev->dev.platform_data;

	if (pdev->id != -1) {
		dev_err(&pdev->dev,
			"%s: MSM chipsets Can only support one"
			" battery ", __func__);
		return -EINVAL;
	}

#ifndef CONFIG_BATTERY_MSM_FAKE
	if (pdata->avail_chg_sources & AC_CHG) {
#else
	{
#endif
		rc = power_supply_register(&pdev->dev, &msm_psy_ac);
		if (rc < 0) {
			dev_err(&pdev->dev,
				"%s: power_supply_register failed"
				" rc = %d\n", __func__, rc);
			msm_batt_cleanup();
			return rc;
		}
		msm_batt_info.msm_psy_ac = &msm_psy_ac;
		msm_batt_info.avail_chg_sources |= AC_CHG;
	}

	if (pdata->avail_chg_sources & USB_CHG) {
		rc = power_supply_register(&pdev->dev, &msm_psy_usb);
		if (rc < 0) {
			dev_err(&pdev->dev,
				"%s: power_supply_register failed"
				" rc = %d\n", __func__, rc);
			msm_batt_cleanup();
			return rc;
		}
		msm_batt_info.msm_psy_usb = &msm_psy_usb;
		msm_batt_info.avail_chg_sources |= USB_CHG;
	}

	if (!msm_batt_info.msm_psy_ac && !msm_batt_info.msm_psy_usb) {

		dev_err(&pdev->dev,
			"%s: No external Power supply(AC or USB)"
			"is avilable\n", __func__);
		msm_batt_cleanup();
		return -ENODEV;
	}


	msm_batt_info.voltage_max_design = pdata->voltage_max_design;
	msm_batt_info.voltage_min_design = pdata->voltage_min_design;
	msm_batt_info.batt_technology = pdata->batt_technology;
	msm_batt_info.calculate_capacity = pdata->calculate_capacity;

	if (!msm_batt_info.voltage_min_design)
		msm_batt_info.voltage_min_design = BATTERY_LOW;
	if (!msm_batt_info.voltage_max_design)
		msm_batt_info.voltage_max_design = BATTERY_HIGH;

	if (msm_batt_info.batt_technology == POWER_SUPPLY_TECHNOLOGY_UNKNOWN)
		msm_batt_info.batt_technology = POWER_SUPPLY_TECHNOLOGY_LION;

	if (!msm_batt_info.calculate_capacity)
		msm_batt_info.calculate_capacity = msm_batt_capacity;

	rc = power_supply_register(&pdev->dev, &msm_psy_batt);
	if (rc < 0) {
		dev_err(&pdev->dev, "%s: power_supply_register failed"
			" rc=%d\n", __func__, rc);
		msm_batt_cleanup();
		return rc;
	}
	msm_batt_info.msm_psy_batt = &msm_psy_batt;

	/* FIH, Debbie, 2011/08/12 { */
	/* 1070 porting*/
	test_on = false;
	for (i = 0; i < MSM_BATT_CBVERIFY_MAX; i++) {
		test_item[i] = false;
		test_value[i] = 0;
	}
	cbverify_proc_entry = create_proc_entry("cbverify", 0664, NULL);
	cbverify_proc_entry->read_proc   = msm_batt_cbverify_proc_read;
	cbverify_proc_entry->write_proc  = msm_batt_cbverify_proc_write;
	/* FIH, Debbie, 2011/08/12 } */

#ifndef CONFIG_BATTERY_MSM_FAKE
	rc = msm_batt_register(BATTERY_LOW, BATTERY_ALL_ACTIVITY,
			       BATTERY_CB_ID_ALL_ACTIV, BATTERY_ALL_ACTIVITY);
	if (rc < 0) {
		dev_err(&pdev->dev,
			"%s: msm_batt_register failed rc = %d\n", __func__, rc);
		msm_batt_cleanup();
		return rc;
	}

	rc =  msm_batt_enable_filter(VBATT_FILTER);

	if (rc < 0) {
		dev_err(&pdev->dev,
			"%s: msm_batt_enable_filter failed rc = %d\n",
			__func__, rc);
		msm_batt_cleanup();
		return rc;
	}
	/* FIH, Debbie, 2011/08/12 { */
	/* 1070 porting*/
	wake_lock_init(&msm_batt_info.msm_batt_wakelock, WAKE_LOCK_SUSPEND, "msm_batt");
	msm_batt_info.driver_ready = true;
	/* FIH, Debbie, 2011/08/12 } */

/* FIH, IvanCHTang, 2011/10/6 { */
/* [IRM], Polling function of battery info. - [3] */
#if 1
	/* Modify to create a new work queue for BT play MP3 smoothly*/
	fih_msm_batt_wq=create_singlethread_workqueue("fih_msm_battery");
	if (!fih_msm_batt_wq) {
		printk("%s(): create workque (fih_msm_batt_wq) failed \n", __func__);
		return -ENOMEM;
	}
	setup_timer(&polling_timer, polling_timer_func_batt, 0);
		mod_timer(&polling_timer, jiffies + msecs_to_jiffies(BATT_POLLING_TIME*1000));
#else
	/* FIH, Debbie, 2011/08/29 { */
	/* add for polling battery status*/
	INIT_WORK(&msm_batt_update, msm_batt_update_psy_status);
    
	setup_timer(&polling_timer,
		msm_battery_polling_timer_func, 0);
	mod_timer(&polling_timer,
		jiffies + msecs_to_jiffies(5000)); /* FIH, Debbie, 2011/09/22 */	
	/* FIH, Debbie, 2011/08/29 } */
#endif
/* } FIH, IvanCHTang, 2011/10/6 */

#ifdef CONFIG_HAS_EARLYSUSPEND
	msm_batt_info.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	msm_batt_info.early_suspend.suspend = msm_batt_early_suspend;
	msm_batt_info.early_suspend.resume = msm_batt_late_resume;
	register_early_suspend(&msm_batt_info.early_suspend);
#endif
	/* FIH, Debbie, 2011/08/29 { */
	/* add for polling battery status*/
	//msm_batt_update_psy_status();
	/* FIH, Debbie, 2011/08/29 } */

#else
	power_supply_changed(&msm_psy_ac);
#endif  /* CONFIG_BATTERY_MSM_FAKE */

	return 0;
}



static int msm_batt_remove(struct platform_device *pdev)
{
	int rc;
	rc = msm_batt_cleanup();

	if (rc < 0) {
		dev_err(&pdev->dev,
			"%s: msm_batt_cleanup  failed rc=%d\n", __func__, rc);
		return rc;
	}
	del_timer_sync(&polling_timer); /* FIH, Debbie, 2011/09/22 */	
	return 0;
}


static struct platform_driver msm_batt_driver = {
	.probe = msm_batt_probe,
	.remove = __devexit_p(msm_batt_remove),
	.driver = {
		   .name = "msm-battery",
		   .owner = THIS_MODULE,
		   },
};


static int __devinit msm_batt_init_rpc(void)
{
	int rc;

#ifdef CONFIG_BATTERY_MSM_FAKE
	pr_info("Faking MSM battery\n");
#else

	msm_batt_info.chg_ep =
		msm_rpc_connect_compatible(CHG_RPC_PROG, CHG_RPC_VER_4_1, 0);
	msm_batt_info.chg_api_version =  CHG_RPC_VER_4_1;
	if (msm_batt_info.chg_ep == NULL) {
		pr_err("%s: rpc connect CHG_RPC_PROG = NULL\n", __func__);
		return -ENODEV;
	}

	if (IS_ERR(msm_batt_info.chg_ep)) {
		msm_batt_info.chg_ep = msm_rpc_connect_compatible(
				CHG_RPC_PROG, CHG_RPC_VER_3_1, 0);
		msm_batt_info.chg_api_version =  CHG_RPC_VER_3_1;
	}
	if (IS_ERR(msm_batt_info.chg_ep)) {
		msm_batt_info.chg_ep = msm_rpc_connect_compatible(
				CHG_RPC_PROG, CHG_RPC_VER_1_1, 0);
		msm_batt_info.chg_api_version =  CHG_RPC_VER_1_1;
	}
	if (IS_ERR(msm_batt_info.chg_ep)) {
		msm_batt_info.chg_ep = msm_rpc_connect_compatible(
				CHG_RPC_PROG, CHG_RPC_VER_1_3, 0);
		msm_batt_info.chg_api_version =  CHG_RPC_VER_1_3;
	}
	if (IS_ERR(msm_batt_info.chg_ep)) {
		msm_batt_info.chg_ep = msm_rpc_connect_compatible(
				CHG_RPC_PROG, CHG_RPC_VER_2_2, 0);
		msm_batt_info.chg_api_version =  CHG_RPC_VER_2_2;
	}
	if (IS_ERR(msm_batt_info.chg_ep)) {
		rc = PTR_ERR(msm_batt_info.chg_ep);
		pr_err("%s: FAIL: rpc connect for CHG_RPC_PROG. rc=%d\n",
		       __func__, rc);
		msm_batt_info.chg_ep = NULL;
		return rc;
	}

	/* Get the real 1.x version */
	if (msm_batt_info.chg_api_version == CHG_RPC_VER_1_1)
		msm_batt_info.chg_api_version =
			msm_batt_get_charger_api_version();

	/* Fall back to 1.1 for default */
	if (msm_batt_info.chg_api_version < 0)
		msm_batt_info.chg_api_version = CHG_RPC_VER_1_1;
	msm_batt_info.batt_api_version =  BATTERY_RPC_VER_4_1;

	msm_batt_info.batt_client =
		msm_rpc_register_client("battery", BATTERY_RPC_PROG,
					BATTERY_RPC_VER_4_1,
					1, msm_batt_cb_func);

	if (msm_batt_info.batt_client == NULL) {
		pr_err("%s: FAIL: rpc_register_client. batt_client=NULL\n",
		       __func__);
		return -ENODEV;
	}
	if (IS_ERR(msm_batt_info.batt_client)) {
		msm_batt_info.batt_client =
			msm_rpc_register_client("battery", BATTERY_RPC_PROG,
						BATTERY_RPC_VER_1_1,
						1, msm_batt_cb_func);
		msm_batt_info.batt_api_version =  BATTERY_RPC_VER_1_1;
	}
	if (IS_ERR(msm_batt_info.batt_client)) {
		msm_batt_info.batt_client =
			msm_rpc_register_client("battery", BATTERY_RPC_PROG,
						BATTERY_RPC_VER_2_1,
						1, msm_batt_cb_func);
		msm_batt_info.batt_api_version =  BATTERY_RPC_VER_2_1;
	}
	if (IS_ERR(msm_batt_info.batt_client)) {
		msm_batt_info.batt_client =
			msm_rpc_register_client("battery", BATTERY_RPC_PROG,
						BATTERY_RPC_VER_5_1,
						1, msm_batt_cb_func);
		msm_batt_info.batt_api_version =  BATTERY_RPC_VER_5_1;
	}
	if (IS_ERR(msm_batt_info.batt_client)) {
		rc = PTR_ERR(msm_batt_info.batt_client);
		pr_err("%s: ERROR: rpc_register_client: rc = %d\n ",
		       __func__, rc);
		msm_batt_info.batt_client = NULL;
		return rc;
	}
#endif  /* CONFIG_BATTERY_MSM_FAKE */

    rc = platform_driver_register(&msm_batt_driver);

	if (rc < 0)
		pr_err("%s: FAIL: platform_driver_register. rc = %d\n",
		       __func__, rc);

	return rc;
}



static int __init msm_batt_init_v1(void* hdl)
{
	int rc;

	pr_debug("%s: enter\n", __func__);

    g_battery_param.full_volt = 4120;
    g_battery_param.chg_ac_current = SET_CHARGE_CURRENT_1A;
    g_battery_param.p_Volt2percent = g_Volt2Percent_v1;

	rc = msm_batt_init_rpc();

	if (rc < 0) {
		pr_err("%s: FAIL: msm_batt_init_rpc.  rc=%d\n", __func__, rc);
		msm_batt_cleanup();
		return rc;
	}

	pr_info("%s: Charger/Battery = 0x%08x/0x%08x (RPC version)\n",
		__func__, msm_batt_info.chg_api_version,
		msm_batt_info.batt_api_version);

	return 0;
}

static int __init msm_batt_init_v2(void* hdl)
{
	int rc;

	pr_debug("%s: enter\n", __func__);

    g_battery_param.full_volt = 4300;
    g_battery_param.chg_ac_current = SET_CHARGE_CURRENT_750MA;
    g_battery_param.p_Volt2percent = g_Volt2Percent_v2;

	rc = msm_batt_init_rpc();

	if (rc < 0) {
		pr_err("%s: FAIL: msm_batt_init_rpc.  rc=%d\n", __func__, rc);
		msm_batt_cleanup();
		return rc;
	}

	pr_info("%s: Charger/Battery = 0x%08x/0x%08x (RPC version)\n",
		__func__, msm_batt_info.chg_api_version,
		msm_batt_info.batt_api_version);

	return 0;
}

//IRMPrime
static int __init msm_batt_init_v3(void* hdl)
{
	int rc;

	pr_debug("%s: enter\n", __func__);

    g_battery_param.full_volt = 4120;
    g_battery_param.chg_ac_current = SET_CHARGE_CURRENT_750MA;
    g_battery_param.p_Volt2percent = g_Volt2Percent_v1;

	rc = msm_batt_init_rpc();

	if (rc < 0) {
		pr_err("%s: FAIL: msm_batt_init_rpc.  rc=%d\n", __func__, rc);
		msm_batt_cleanup();
		return rc;
	}

	pr_info("%s: Charger/Battery = 0x%08x/0x%08x (RPC version)\n",
		__func__, msm_batt_info.chg_api_version,
		msm_batt_info.batt_api_version);

	return 0;
}

DECLARE_DYN_LOADER(DRIVER_TYPE_BATTERY, 
                   BATTERY_V1, 
                   msm_batt_init_v1, 
                   LEVEL6);

DECLARE_DYN_LOADER(DRIVER_TYPE_BATTERY, 
                   BATTERY_V2, 
                   msm_batt_init_v2, 
                   LEVEL6);
//IRMPrime
DECLARE_DYN_LOADER(DRIVER_TYPE_BATTERY, 
                   BATTERY_V3, 
                   msm_batt_init_v3, 
                   LEVEL6);

static void __exit msm_batt_exit(void)
{
    platform_driver_unregister(&msm_batt_driver);

}

//module_init(msm_batt_init);
module_exit(msm_batt_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Kiran Kandi, Qualcomm Innovation Center, Inc.");
MODULE_DESCRIPTION("Battery driver for Qualcomm MSM chipsets.");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:msm_battery");
