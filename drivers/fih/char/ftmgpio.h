/*
*     ftmgpio.h - GPIO access driver header for FTM use
*
*     Copyright (C) 2009 Clement Hsu <clementhsu@tp.cmcs.com.tw>
*     Copyright (C) 2009 Chi Mei Communication Systems Inc.
*
*     This program is free software; you can redistribute it and/or modify
*     it under the terms of the GNU General Public License as published by
*     the Free Software Foundation; version 2 of the License.
*/

// ioctl operation definition for GPIO device driver
//MTD-BSP-AlwaysChen-SMEM_OEM_COMMAND-02*[
#define IOCTL_GPIO_INIT					1    
#define IOCTL_GPIO_SET			2    
#define IOCTL_GPIO_GET					3
#define IOCTL_GPIO_SET_OUTPUT_HIGH		4
#define IOCTL_GPIO_SET_OUTPUT_LOW		5
#define IOCTL_GPIO_SET_OUTPUT			6

#define IOCTL_PID_GET			7
#define IOCTL_PID_SET			8
#define IOCTL_IMEI_GET			9
#define IOCTL_IMEI_SET			10

#define IOCTL_UART_BAUDRATE_460800		12

#define IOCTL_BDADDR_GET		14
#define IOCTL_BDADDR_SET		15

//MTD-MM-YW-FTM-AUDIO+[
#define IOCTL_LOOPBACK_EN       16
#define IOCTL_LOOPBACK_DIS      17
#define IOCTL_MICBIAS_CTL       18
//MTD-MM-YW-FTM-AUDIO+]

//Div6-PT2-Peripheral-CD-WIFI_FTM_TEST-02+[
#define IOCTL_WLANADDR_GET		19
#define IOCTL_WLANADDR_SET		20
//Div6-PT2-Peripheral-CD-WIFI_FTM_TEST-02+]
#define IOCTL_HV_GET			21
/* FIH, CHHsieh, 2011/08/16 { */
/* Moto CFC requirement */
#define IOCTL_RESET				22
#define IOCTL_NV_COMMAND		23
#define IOCTL_OEM_SMEM_PROCESS	30
/*} FIH, CHHsieh, 2011/08/16 */
/* FIHTDC, Div2-SW2-BSP CHHsieh, NVBackup { */
#define IOCTL_BACKUP_UNIQUE_NV	24
#define IOCTL_HW_RESET			25
#define IOCTL_BACKUP_NV_FLAG	27
/* } FIHTDC, Div2-SW2-BSP CHHsieh, NVBackup */
#define IOCTL_FUSE_BOOT_SET		28
#define IOCTL_FUSE_BOOT_GET		29
/* FIHTDC, CHHsieh, PMIC Unlock { */
#define IOCTL_PMIC_UNLOCK		32
/* } FIHTDC, CHHsieh, PMIC Unlock */
/* FIHTDC, CHHsieh, MEID Get/Set { */
#define IOCTL_MEID_GET			34
#define IOCTL_MEID_SET			35
/* } FIHTDC, CHHsieh, MEID Get/Set */
/* FIHTDC, CHHsieh, FD1 NV programming in factory { */
#define IOCTL_ESN_GET			36
#define IOCTL_AKEY1_SET			37
#define IOCTL_AKEY2_SET			38
#define IOCTL_AKEY1_GET			57
#define IOCTL_AKEY2_GET			60
#define IOCTL_SPC_GET			53
#define IOCTL_SPC_SET			39
#define IOCTL_CUSTOMERPID_GET	40
#define IOCTL_CUSTOMERPID_SET	42
#define IOCTL_CUSTOMERPID2_GET	43
#define IOCTL_CUSTOMERPID2_SET	44
#define IOCTL_CUSTOMERSWID_GET	45
#define IOCTL_CUSTOMERSWID_SET	46
#define IOCTL_DOM_GET			47
#define IOCTL_DOM_SET			48
/* } FIHTDC, CHHsieh, FD1 NV programming in factory */
/* FIHTDC, CHHsieh, FB3 CA/WCA Time Set/Get { */
#define IOCTL_CATIME_GET		49
#define IOCTL_CATIME_SET		50
#define IOCTL_WCATIME_GET		51
#define IOCTL_WCATIME_SET		52
/* } FIHTDC, CHHsieh, FB3 CA/WCA Time Set/Get */
#define IOCTL_HW_RESET_TO_FTM	54
/* FIHTDC, CHHsieh, FB0 FactoryInfo Set/Get { */
#define IOCTL_FACTORYINFO_GET	55
#define IOCTL_FACTORYINFO_SET	56
/* } FIHTDC, CHHsieh, FB0 FactoryInfo Set/Get */
#define IOCTL_CHANGE_MODEM_LPM	58
//MTD-BSP-AlwaysChen-SMEM_OEM_COMMAND-02*]

#define IOCTL_VIB_ACTION		59 // MTD-KERNEL-EL-VIB00+
#define IOCTL_HOOKKEY_GET       62
#define IOCTL_GPS_ON       63
#define IOCTL_GPS_OFF       64

#define IOCTL_PRIL_CHANGE_REQ       65
#define IOCTL_PRIL_VERIFY_RESULT           66



#define IOCTL_FLASHLED_ON       97
#define IOCTL_FLASHLED_OFF      98

// GPIO direction definition
#define GPIO_IN			0
#define GPIO_OUT		1

// GPIO Output pin High/Low definition
#define GPIO_LOW		0
#define GPIO_HIGH		1


static int ftmgpio_dev_open( struct inode * inode, struct file * file );

static ssize_t ftmgpio_dev_read( struct file * file, char __user * buffer, size_t size, loff_t * f_pos );

static int ftmgpio_dev_write( struct file * filp, const char __user * buffer, size_t length, loff_t * offset );

static long ftmgpio_dev_ioctl(struct file * filp, unsigned int cmd, unsigned long arg );

static int __init ftmgpio_init(void);

static void __exit ftmgpio_exit(void);                         



typedef enum
{
  GPIO_INPUT  = 0,
  GPIO_OUTPUT = 1
} GPIO_DirectionType;

typedef enum
{
  GPIO_2MA     = 0,
  GPIO_4MA     = 0x1,
  GPIO_6MA     = 0x2,
  GPIO_8MA     = 0x3,
  GPIO_10MA    = 0x4,
  GPIO_12MA    = 0x5,
  GPIO_14MA    = 0x6,
  GPIO_16MA    = 0x7,
} GPIO_DriveStrenthType;

typedef enum
{
  GPIO_NO_PULL    = 0,
  GPIO_PULL_DOWN  = 0x1,
  GPIO_KEEPER     = 0x2,
  GPIO_PULL_UP    = 0x3
} GPIO_PullupPulldownType;

typedef enum
{
    GPIO_ENABLE = 0,
    GPIO_DISABLE
}GPIO_CfgType;












