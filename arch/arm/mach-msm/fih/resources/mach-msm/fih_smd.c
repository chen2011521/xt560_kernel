/* arch/arm/mach-msm/smd.c
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2011, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
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

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/termios.h>
#include <linux/ctype.h>
#include <linux/remote_spinlock.h>
#include <linux/uaccess.h>
#include <mach/msm_smd.h>
#include <mach/msm_iomap.h>
#include <mach/system.h>
#include <mach/subsystem_notif.h>

#include "../../../smd_private.h"
#include "../../../proc_comm.h"
#include "../../../modem_notifier.h"

//SW2-5-1-MP-DbgCfgTool-00+[
static unsigned int msm_poweron_cause;
module_param_named(poweron_cause, msm_poweron_cause, int, S_IRUGO);
//SW2-5-1-MP-DbgCfgTool-00+]

//SW2-5-1-MP-HostOemInfo-00+[
unsigned int fih_host_log_from_uart = 0x0;
unsigned int fih_host_usb_id = 0x0;
unsigned int fih_host_boot_mode = 0x0; //SW-2-5-1-MP-DbgCfgTool-03+
//SW2-5-1-MP-HostOemInfo-00+]

void fih_get_host_oem_info(void)
{
    struct smem_host_oem_info* fih_smem_host_used;
    printk(KERN_INFO "fih_smem_alloc_for_host_used\n");
    fih_smem_host_used = smem_alloc(SMEM_ID_VENDOR2, sizeof(*fih_smem_host_used));
    if(fih_smem_host_used == NULL)    	
    {
    	printk(KERN_INFO "fih_smem_alloc_for_host_used = NULL!\n");
        fih_host_log_from_uart = 0;
	fih_host_usb_id = 0;
	fih_host_boot_mode = 0; //SW-2-5-1-MP-DbgCfgTool-03+
	return;
    }
    if(fih_smem_host_used)
    {
        fih_host_log_from_uart = (fih_smem_host_used->host_log_from_uart);
	fih_host_usb_id = (fih_smem_host_used->host_usb_id);
	fih_host_boot_mode = (fih_smem_host_used->host_used3); //SW-2-5-1-MP-DbgCfgTool-03+
    }
    printk(KERN_INFO "smd.c, fih_smem_host_used address = 0x%X, host_usb_id = 0x%X, host_log_from_uart = 0x%X\n"
    , (unsigned int)fih_smem_host_used
    , fih_smem_host_used->host_usb_id
    , fih_smem_host_used->host_log_from_uart);
}

unsigned int fih_read_uart_switch_from_smem(void)   //printk used
{
    return fih_host_log_from_uart;
}

unsigned int fih_read_usb_id_from_smem(void)   //board-msm7x27.c, and msm_hsusb.c  used
{
    return fih_host_usb_id;
}
//SW2-5-1-MP-DbgCfgTool-03+[
unsigned int fih_read_boot_mode_from_smem(void)
{
    return fih_host_boot_mode;
}
//SW2-5-1-MP-DbgCfgTool-03+]

//Div2-SW2-BSP,JOE HSU,+++
static struct smem_oem_info oem_info = {0};
static unsigned fih_hwid = 0;
static unsigned int fih_product_id = 0;
static unsigned int fih_product_phase = 0;
static unsigned int fih_band_id = 0;
char fih_oem_modem_rev[16] = {0};

/* FIH, CHHsieh, 2011/10/20 { */
/* get nand info. form share memory */
char fih_flash_name[32]  = {0};
/* FIH, CHHsieh, 2011/10/20 } */

void fih_get_oem_info(void)
{
	struct smem_oem_info *fih_smem_info = smem_alloc(SMEM_ID_VENDOR0, sizeof(oem_info));
	if (fih_smem_info==NULL) {
		return;
	}
	
	memcpy(&oem_info, fih_smem_info, sizeof(oem_info));
	
	msm_poweron_cause = oem_info.power_on_cause; //SW2-5-1-MP-DbgCfgTool-00+
	printk(KERN_INFO "FIH kernel - power_on_cause = 0x%x\r\n", msm_poweron_cause);
	
	fih_hwid = oem_info.hw_id;	
  	printk(KERN_INFO "FIH kernel - fih_hwid = 0x%x\r\n", fih_hwid);
  
	//get product id
	fih_product_id = oem_info.hw_id & 0xff;
	printk(KERN_INFO "FIH kernel - fih_product_id = 0x%x\r\n", fih_product_id);

	//get product phsae
	fih_product_phase = (oem_info.hw_id>>8) & 0xff;
	printk(KERN_INFO "FIH kernel - fih_product_phase = 0x%x\r\n", fih_product_phase);

	//get band id
	fih_band_id = (oem_info.hw_id>>16) & 0xff;
	printk(KERN_INFO "FIH kernel - fih_band_id = 0x%x\r\n", fih_band_id);

	//get modem version
	memcpy(fih_oem_modem_rev, oem_info.oem_mod_rev, 16);
	printk(KERN_INFO "FIH kernel - fih_oem_modem_rev = %s\n", fih_oem_modem_rev);

/* FIH, CHHsieh, 2011/10/20 { */
/* get nand info. form share memory */
//get ram info and device name form share memory @ /proc/nandinfo
	memcpy(fih_flash_name, fih_smem_info->flash_name, 32);
	printk(KERN_INFO "FIH kernel - fih_flash_name = %s\n", fih_flash_name);
/* FIH, CHHsieh, 2011/10/20 } */
		
} /*fih_get_oem_info*/
EXPORT_SYMBOL(fih_get_oem_info);

/* FIH, CHHsieh, 2011/10/20 { */
/* get nand info. form share memory */
char* fih_read_flash_name_from_smem(void)
{
    return fih_flash_name ;
}
/* FIH, CHHsieh, 2011/10/20 } */

unsigned int fih_get_product_id(void)
{
	return fih_product_id;
} /*fih_get_product_id*/
EXPORT_SYMBOL(fih_get_product_id);

unsigned int fih_get_product_phase(void)
{
	return fih_product_phase;
} /*fih_get_product_phase*/
EXPORT_SYMBOL(fih_get_product_phase);

unsigned int fih_get_band_id(void)
{
	return fih_band_id;
} /*fih_get_band_id*/
EXPORT_SYMBOL(fih_get_band_id);

char *fih_get_version(void)
{
	return fih_oem_modem_rev;
} /*fih_get_band_id*/
EXPORT_SYMBOL(fih_get_version);
//Div2-SW2-BSP,JOE HSU,---

//Div2-SW2-BSP, HenryMCWang, get power on cause, +++
unsigned int fih_get_poweroncause_info(void)
{
	return oem_info.power_on_cause;
}
//Div2-SW2-BSP, HenryMCWang, get power on cause, +++

/* FIH, CHHsieh, 2011/11/07 { */
/* fver cause kernel panic when factory reset */
unsigned check_reboot_mode(void)
{
	unsigned mode[2] = {0, 0};
	unsigned int mode_len = sizeof(mode);
	uint32_t *smem_status;
	
	printk("[chhsieh]%s : start\n", __func__);
	smem_status = smem_alloc(SMEM_APPS_BOOT_MODE, mode_len );
	if (*smem_status==0)
	{
		printk(KERN_ERR "ERROR: unable to read shared memory for reboot mode\n");
		return 0;
	}
	printk(KERN_INFO "check_reboot_mode[0x%x]=0x%x\n",*smem_status,*(uint32_t*)smem_status);
    return *(uint32_t*)smem_status;
}
/* FIH, CHHsieh, 2011/11/07 } */

