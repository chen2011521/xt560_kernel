/* linux/arch/arm/mach-msm/fih_rpc_hsusb.c
 *
 * Copyright (c) 2008-2011, Code Aurora Forum. All rights reserved.
 *
 * All source code in this file is licensed under the following license except
 * where indicated.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 */


/* FIH, Debbie, 2011/08/12 { */
/* 1070 porting*/
/* Div2-SW2-BSP-IRM-CHG { */
#include <linux/mutex.h>
#include <mach/msm_hsusb.h>
#include <mach/rpc_hsusb.h>
//#include "fih_rpc_hsusb.h"

DEFINE_MUTEX(OT_flag_lock);
static bool g_OT_flag = false;
static unsigned chg_current = 0;
extern void msm_batt_update_charger_type(enum chg_type charger_type);
/* FIH, Debbie, 2011/09/22 { */	
/* update battery id status */
extern bool battery_id_valid_flag;
/* FIH, Debbie, 2011/09/22 { */	
/* } Div2-SW2-BSP-IRM-CHG */
/* FIH, Debbie, 2011/08/12 } */


#ifdef CONFIG_USB_GADGET_MSM_72K
/* charger api wrappers */

void fih_hsusb_chg_vbus_draw(unsigned mA)
{
	/* FIH, Debbie, 2011/08/12 { */
	/* 1070 porting*/
	/* Div2-SW2-BSP-IRM-CHG { */
	chg_current = mA;

	mutex_lock(&OT_flag_lock);

	/* FIH, Debbie, 2011/09/22 { */	
	/* update battery id status */
	if(g_OT_flag || (!battery_id_valid_flag)){ 
	/* FIH, Debbie, 2011/09/22 { */	
		if (g_OT_flag)
			pr_info("%s, Over Temperature \n", __func__);
		else
			pr_info("%s, Invalid Battery \n", __func__);
	msm_chg_usb_i_is_available(0);
	} else
	/* } Div2-SW2-BSP-IRM-CHG */
	/* FIH, Debbie, 2011/08/12 } */
	msm_chg_usb_i_is_available(mA);

	/* FIH, Debbie, 2011/08/12 { */
	/* 1070 porting*/
	mutex_unlock(&OT_flag_lock);	// Div2-SW2-BSP-IRM-CHG
	/* FIH, Debbie, 2011/08/12 } */
}
EXPORT_SYMBOL(fih_hsusb_chg_vbus_draw);

/* FIH, Debbie, 2011/08/12 { */
/* 1070 porting*/
/* Div2-SW2-BSP-IRM-CHG { */
void hsusb_chg_notify_over_tempearture(bool OT_flag)
{
	mutex_lock(&OT_flag_lock);
	if (g_OT_flag != OT_flag) {
		g_OT_flag = OT_flag;
		if (g_OT_flag)
		msm_chg_usb_i_is_available(0);
	else
		msm_chg_usb_i_is_available(chg_current);
	}
	mutex_unlock(&OT_flag_lock);
}
EXPORT_SYMBOL(hsusb_chg_notify_over_tempearture);

#endif
