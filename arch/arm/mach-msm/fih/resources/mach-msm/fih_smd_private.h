/* arch/arm/mach-msm/smd_private.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007-2011, Code Aurora Forum. All rights reserved.
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

/*Div2-SW2-BSP, JOE HSU, +++ */
struct smem_oem_info
{
        unsigned int hw_id;
        unsigned int keypad_info;
        unsigned int power_on_cause;
        unsigned int network_mode; /* add for Read modem mode from smem */
        /* add for display info about download and sd ram dump*/
        unsigned int oemsbl_mode;
        char   flash_name[32];
        char   oem_mod_rev[16];
        unsigned int progress;
        unsigned int msg_counter;
        /* get ram info form share memory */
        unsigned int dram_info;
};

void fih_get_oem_info(void);
unsigned int fih_get_product_id(void);
unsigned int fih_get_product_phase(void);
unsigned int fih_get_band_id(void);
char *fih_get_version(void);
/*Div2-SW2-BSP, JOE HSU, --- */

//Div2-SW2-BSP, HenryMCWang, get power on cause, +++
unsigned int fih_get_poweroncause_info(void);
//Div2-SW2-BSP, HenryMCWang, get power on cause, +++

//Div2-SW2-BSP, HenryMCWang, fih_get_host_oem_info(void), +++
//SW2-5-1-MP-HostOemInfo-00+[
struct smem_host_oem_info
{
        unsigned int host_usb_id;
        unsigned int host_log_from_uart;
        unsigned int host_used3;
        unsigned int host_used4;
};

void fih_get_host_oem_info(void);
unsigned int fih_read_uart_switch_from_smem(void);
unsigned int fih_read_usb_id_from_smem(void);
unsigned int fih_read_boot_mode_from_smem(void); //SW-2-5-1-MP-DbgCfgTool-03+
//SW2-5-1-MP-HostOemInfo-00+]
//Div2-SW2-BSP, HenryMCWang, fih_get_host_oem_info(void), +++

// FIHTDC, HenryMCWang, 2011.11.03, for event log, +++
void smsm_event_sleep_info(uint32_t wakeup_reason);
// FIHTDC, HenryMCWang, 2011.11.03, for event log, ---

/* FIH, CHHsieh, 2011/11/07 { */
/* fver cause kernel panic when factory reset */
unsigned check_reboot_mode(void);
/* FIH, CHHsieh, 2011/11/07 } */

