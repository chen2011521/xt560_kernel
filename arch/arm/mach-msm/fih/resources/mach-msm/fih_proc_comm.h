/* arch/arm/mach-msm/proc_comm.h
 *
 * Copyright (c) 2007-2009,2011 Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
 
#ifndef _ARCH_ARM_MACH_MSM_MSM_FIH_PROC_COMM_H_
#define _ARCH_ARM_MACH_MSM_MSM_FIH_PROC_COMM_H_

/* FIH, JiaHao, 2011/08/16 { */
#include <linux/fih_hw_info.h>
#include <fih/dynloader.h>
/* FIH, JiaHao, 2011/08/16 } */

/*DerrickDRLiu, add this function declaration at header file*/
int msm_proc_comm_oem(unsigned cmd, unsigned *data1, unsigned *data2, unsigned *cmd_parameter);

//FIHTDC-DerrickDRLiu-DbgCfgTool-00+[
int msm_proc_comm_oem_n(unsigned cmd, unsigned *data1, unsigned *data2, unsigned *cmd_parameter, int para_size);
//#define NV_FIHDBG_I    51001
#define NV_FIHDBG_I 8047
#define NV_ERR_FATAL_OPTIONS_I    905 //FIHTDC-DerrickDRLiu-Modem_Debug_NV-00+
/*--------------------------------------------------------------------------* 
 * Size of smem_oem_cmd_data to carry both return value and FIH debug 
 * configurations.
 *
 * FIHDBG:(FIH_DEBUG_CMD_DATA_SIZE:5 int)
 *
 *   |-- 1 unsigned int return value  --|-- 4 unsigned int(128 bit) configurations --|
 *
 *   a) SMEM command's return value:
 *       true  : 1
 *       false : 0
 *   b) configurations:
 *       Magic number : 0xFFFF FFFF FFFF FFFF FFFF FFFF FFFF FFFF is default value.
 *       enable       : bit(n) = 1
 *       disable      : bit(n) = 0
 *--------------------------------------------------------------------------*/
#define FIH_DEBUG_CMD_DATA_SIZE  5
#define FIH_DEBUG_CFG_LEN  4 * sizeof(unsigned int)

/*--------------------------------------------------------------------------*
 * Function    : fih_read_fihdbg_config_nv
 *
 * Description :
 *     Read fih debug configuration settings from nv item (NV_FIHDBG_I).
 *
 * Parameters  :
 *     String of 16 bytes fih debug configuration setting array in 
 *     unsigned char.
 *
 * Return value: Integer
 *     Zero     - Successful
 *     Not zero - Fail
 *--------------------------------------------------------------------------*/
int fih_read_fihdbg_config_nv( unsigned char* );

//int fih_read_fihversion_nv( unsigned char* );     //Div2D5-LC-BSP-Porting_OTA_SDDownload-00 +

/*--------------------------------------------------------------------------*
 * Function    : fih_write_fihdbg_config_nv
 *
 * Description :
 *     Write fih debug configuration settings into nv item (NV_FIHDBG_I).
 *
 * Parameters  :
 *     String of 16 bytes fih debug configuration setting array in 
 *     unsigned char.
 *
 * Return value: Integer
 *     Zero     - Successful
 *     Not zero - Fail
 *--------------------------------------------------------------------------*/
int fih_write_fihdbg_config_nv( unsigned char* );

int fih_write_modem_debug_nv( int value ); //FIHTDC-DerrickDRLiu-Modem_Debug_NV-00+
//FIHTDC-DerrickDRLiu-DbgCfgTool-00+]
//Div2-SW2-BSP, JOE HSU ,+++
#define SMEM_OEM_CMD_BUF_SIZE  32

typedef enum
{
  SMEM_PROC_COMM_OEM_ADC_READ = 0,              /* ZEUS_CR_52 */
  SMEM_PROC_COMM_OEM_PM_SET_LED_INTENSITY,               /*1*/
  SMEM_PROC_COMM_OEM_PM_MIC_EN, /* ZEUS_CR_130 */  
  SMEM_PROC_COMM_OEM_EBOOT_SLEEP_REQ,       /* ZEUS_CR_165 */
  SMEM_PROC_COMM_OEM_RESET_PM_RTC,          /* ZEUS_CR_177 */
  SMEM_PROC_COMM_OEM_PWR_KEY_DECT,                       /*5*/
  SMEM_PROC_COMM_OEM_PRODUCT_ID_READ,           /* ZEUS_CR_558  */
  SMEM_PROC_COMM_OEM_PRODUCT_ID_WRITE,          /* ZEUS_CR_558  */
  SMEM_PROC_COMM_OEM_SERIAL_NUM_READ,           /* ZEUS_CR_558  */
  SMEM_PROC_COMM_OEM_SERIAL_NUM_WRITE,          /* ZEUS_CR_558  */
  SMEM_PROC_COMM_OEM_TEST_FLAG_READ,                     /*10*/
  SMEM_PROC_COMM_OEM_TEST_FLAG_WRITE,           /* ZEUS_CR_558  */
  SMEM_PROC_COMM_OEM_RESET_CHIP_EBOOT,           /* ZEUS_CR_1129  */
  SMEM_PROC_COMM_OEM_NV_WRITE,
  SMEM_PROC_COMM_OEM_NV_READ,
  SMEM_PROC_COMM_OEM_ADIE_ADC_READ,                      /*15*/
  SMEM_PROC_COMM_OEM_POWER_OFF,         /* FIH, Paul Huang, 2009/08/12 */
/* FIH; Tiger; 2009/12/10 { */
/* add TCP filter command */
  SMEM_PROC_COMM_OEM_UPDATE_TCP_FILTER,
  SMEM_PROC_COMM_OEM_SET_RTC_ALARM,     /*F0X_B_446: Setting the RTC alarm*/
  SMEM_PROC_COMM_OEM_GET_RTC_ALARM,     /*F0X_B_446: Getting the RTC alarm*/
  SMEM_PROC_COMM_OEM_GET_SYSTEM_TIME,                    /*20*/
/* } FIH; Tiger; 2009/12/10 */
// +++ for SD card download, paul huang
  SMEM_PROC_COMM_OEM_ALLOC_SD_DL_INFO,
// --- for SD card download, paul huang
  /* FIH, SimonSSChang 2010/05/31 { */
  /* keep AXI on 160MHz*/
  SMEM_PRPC_COMM_OEM_FIX_AXI_CLOCK = 22, 
  /* } FIH, SimonSSChang 2010/05/31 */
/* FIH 2010/12/07, SquareCHFang, NVbackup { */
  SMEM_PROC_COMM_OEM_BACKUP_UNIQUE_NV,
  SMEM_PROC_COMM_OEM_BACKUP_NV_FLAG,
/* } FIH 2010/12/07, SquareCHFang, NVbackup */
  SMEM_PROC_COMM_OEM_WRITE_FUSE,                         /*25*/
  SMEM_PROC_COMM_OEM_READ_FUSE, //FIH,NeoChen, for read&write FUSE,20100727
  SMEM_PROC_COMM_OEM_OTP_PROCESS,
  SMEM_PROC_COMM_OEM_LED_INDICATION,//Add for new led mechanism_AI1D.B-42
  SMEM_PROC_COMM_OEM_SIM_STATE, //IvanCHTan, add for domino sim state
  SMEM_PROC_COMM_OEM_PERSO_RESET,                        /*30*/
  SMEM_PROC_COMM_OEM_BLOW_FUSE,
// MTD_MM_YW_FTM_AUDIO_SMEM+[
  SMEM_PROC_COMM_OEM_AUDIO_ADIE_WRITE,
  SMEM_PROC_COMM_OEM_AUDIO_ADIE_READ, 
  SMEM_PROC_COMM_OEM_AUDIO_LOOPBACK,  
// MTD_MM_YW_FTM_AUDIO_SMEM+]   
  SMEM_PROC_COMM_OEM_VIB, // MTD-KERNEL-EL-VIB00+        /*35*/
  SMEM_PROC_COMM_OEM_CONFIG_CHG_CURRENT,	// DIV2-SW2-BSP-IRM-FTM-CHG
  SMEM_PROC_COMM_OEM_CONFIG_COIN_CELL,		// DIV2-SW2-BSP-IRM-FTM-BACKUP-BATTERY
  SMEM_PROC_COMM_OEM_BATTERY_INFORMATION,	// DIV2-SW2-BSP-IRM-BATTERY  /* FIH, Debbie, 2011/08/23 */
  SMEM_PROC_COMM_OEM_FIH_CFGDATA_HWID_READ, /* FIH, JiaHao, 2011/08/16 */
  /*FIH, CHHsieh, 2011/09/23 */
  SMEM_PROC_COMM_OEM_AUDIO_HESDKEY,             /*40*/
  SMEM_PROC_COMM_OEM_HIGH_BATTERY_DETECTION, /*FIH, CHHsieh, 2011/09/15 */
/* FIHTDC, Div2-SW2-BSP PeterKCTseng, MSM7X27a Audio, 2011/10/14 { */
//#if defined(CONFIG_FIH_PROJECT_IRM)
  SMEM_PROC_COMM_OEM_AUDIO_SET_PATH,
  SMEM_PROC_COMM_OEM_AUDIO_SET_VOLUME,
  SMEM_PROC_COMM_OEM_AUDIO_DSP_LOOPBACK,
  SMEM_PROC_COMM_OEM_AUDIO_TONES_PLAY,
  SMEM_PROC_COMM_OEM_AUDIO_TONES_STOP,
  SMEM_PROC_COMM_OEM_PRL_RESTORE,
//#endif
  SMEM_PROC_COMM_OEM_GPS_CN_TEST, //48
/* FIHTDC, Div2-SW2-BSP PeterKCTseng, MSM7X27a Audio, 2011/10/14 } */
  
  SMEM_PROC_COMM_OEM_PRIL_CHANGE_REQ,
  SMEM_PROC_COMM_OEM_PRIL_VERIFY_RESULT,


  
  SMEM_PROC_COMM_OEM_NUM_CMDS  /* always last! */
} smem_proc_comm_oem_cmd_type;

#define SMEM_PROC_COMM_OEM_UPDATE_TCP_FILTER  SMEM_PROC_COMM_OEM_NUM_CMDS
/*FIH, DerrickDRLiu,it is the wrong smem command,give the correct SMEM command for power off alarm*/
//#define SMEM_PROC_COMM_OEM_SET_RTC_ALARM      SMEM_PROC_COMM_OEM_NUM_CMDS   
//#define SMEM_PROC_COMM_OEM_GET_RTC_ALARM      SMEM_PROC_COMM_OEM_NUM_CMDS
//#define SMEM_PROC_COMM_OEM_GET_SYSTEM_TIME    SMEM_PROC_COMM_OEM_NUM_CMDS
/*FIH, DerrickDRLiu,it is the wrong smem command,give the correct SMEM command for power off alarm*/
#define SMEM_PRPC_COMM_OEM_FIX_AXI_CLOCK      SMEM_PROC_COMM_OEM_NUM_CMDS 

typedef union smem_oem_cmd_data
{
  struct t_cmd_data
  {
    unsigned int check_flag;
    unsigned int cmd_parameter[SMEM_OEM_CMD_BUF_SIZE];
  }cmd_data;

  struct t_return_data
  {
    unsigned int check_flag;
    unsigned int return_value[SMEM_OEM_CMD_BUF_SIZE];
  }return_data;
  
} smem_oem_cmd_data;

#define smem_oem_locked_flag	0x10000000
#define smem_oem_unlocked_flag	0x20000000
void proc_comm_read_PID_testFlag( unsigned int oem_cmd,unsigned* data);
void proc_comm_write_PID_testFlag( uint32_t oem_cmd,unsigned* data);
void proc_comm_ftm_wlanaddr_write(char* buf);
int proc_comm_ftm_wlanaddr_read(char * buf);
int msm_proc_comm_oem_multi(unsigned cmd, unsigned *data1, unsigned *data2, unsigned *cmd_parameter, int number);
void proc_comm_ftm_bdaddr_write(char* buf);
void proc_comm_ftm_bdaddr_read(char * buf);
int proc_comm_phone_getsimstatus(void);  // SW-PHONE-NT-FTM_SIMTest-01 +
void proc_comm_ftm_imei_read(unsigned* buf);
void proc_comm_ftm_imei_write(unsigned* buf);
// MTD_MM_YW_FTM_AUDIO_SMEM+[
int proc_audio_adie_write(int32_t, int32_t);
int proc_audio_adie_read(int32_t, int32_t *);
int proc_audio_loopback(int, int);

void proc_comm_read_hook_Flag( uint32_t oem_cmd,unsigned* data );

// MTD_MM_YW_FTM_AUDIO_SMEM+] 
int proc_vib_action(int32_t reg, int32_t val);// MTD-KERNEL-EL-VIB00+
//Div2-SW2-BSP, JOE HSU ,---
/* FIHTDC, Div2-SW2-BSP CHHsieh, NVBackup { */
int proc_comm_ftm_backup_unique_nv(void);
int proc_comm_ftm_nv_flag(void);
/* } FIHTDC, Div2-SW2-BSP CHHsieh, NVBackup */
int proc_comm_ftm_change_modem_lpm(void);
int proc_comm_read_battery_thermal(void); // DIV2-SW2-BSP-IRM-BATTERY+ /* FIH, Debbie, 2011/08/16 */
int proc_comm_read_battery_voltage(void); // DIV2-SW2-BSP-IRM-BATTERY+ /* FIH, Debbie, 2011/08/23 */
int proc_comm_hv_battery_dection(unsigned batt_type );  // FIH-NJ-BSP Jerry, 2011/09/27
int proc_comm_read_hwid_table(struct st_hwid_table *p_hwid_table); /* FIH, JiaHao, 2011/08/16 */
/* FIHTDC, CHHsieh, FD1 NV programming in factory { */
void proc_comm_ftm_customer_pid_write(unsigned* buf);
void proc_comm_ftm_customer_pid_read(unsigned* buf);
void proc_comm_ftm_customer_pid2_write(unsigned* buf);
void proc_comm_ftm_customer_pid2_read(unsigned* buf);
void proc_comm_ftm_customer_swid_write(unsigned* buf);
void proc_comm_ftm_customer_swid_read(unsigned* buf);
void proc_comm_ftm_dom_write(unsigned* buf);
void proc_comm_ftm_dom_read(unsigned* buf);
/* } FIHTDC, CHHsieh, FD1 NV programming in factory */
/* FIH, Debbie, 2011/09/05 { */
int proc_comm_config_coin_cell(int vset, int *voltage, int status);
int proc_comm_config_chg_current(bool on, int curr);
/* FIH, Debbie, 2011/09/05 } */
/* FIH, CHHsieh, 2011/08/16 { */
/* Moto CFC requirement */
unsigned int proc_comm_reset_chip(unsigned *cmd_parameter);
int msm_proc_comm_oem_rw_NV(unsigned cmd, unsigned *data1, unsigned *data2, unsigned *cmd_parameter);
int proc_comm_oem_smem_process(int *buf, int oem_data1);
/*} FIH, CHHsieh, 2011/08/16 */
/* FIH, CHHsieh, 2011/10/19 { */
/* DUT can not enter FTM mode after flash */
void proc_comm_pm_restart(void);
/* } FIH, CHHsieh, 2011/10/19 */
/* FIH, CHHsieh, 2011/10/21 { */
/* IRE ftm command W/R NV through smem command */
int proc_comm_ftm_meid_write(char* buf);
int proc_comm_ftm_meid_read(unsigned* buf);
int proc_comm_ftm_esn_read(unsigned* buf);
int proc_comm_ftm_akey1_write(unsigned* buf);
int proc_comm_ftm_akey2_write(unsigned* buf);
int proc_comm_ftm_akey1_read(char* buf);
int proc_comm_ftm_akey2_read(char* buf);
int proc_comm_ftm_spc_read(char* buf);
int proc_comm_ftm_spc_write(unsigned* buf);
/* FIH, CHHsieh, 2011/10/21 } */

/* FIH, Square, 2011/11/03 { */
int proc_comm_read_product_id(unsigned int *product_id);
/* FIH, Square, 2011/11/03 } */
int proc_comm_OEM_GPS_MODE(unsigned mode,unsigned satellite,unsigned  * CNvalue);
int proc_comm_req_RPLI(void);
int proc_comm_verify_result(int cmd);


/*Michael modify for get system time, 2010/02/23 { */
typedef struct
{
   unsigned short year;            // Year [1980..2100]
   unsigned short month;           // Month of year [1..12]
   unsigned short day;             // Day of month [1..31]
   unsigned short hour;            // Hour of day [0..23]
   unsigned short minute;          // Minute of hour [0..59]
   unsigned short second;          // Second of minute [0..59]
   unsigned short day_of_week;     // Day of the week [0..6] Monday..Sunday
   unsigned short pad;
} pm_rtc_julian_type;
/*Michael modify for get system time, 2010/02/08 } */
#endif //_ARCH_ARM_MACH_MSM_MSM_FIH_PROC_COMM_H_

