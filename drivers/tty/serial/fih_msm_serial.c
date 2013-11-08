/*
 * drivers/serial/msm_serial.c - driver for msm7k serial device and console
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
 * Author: Robert Love <rlove@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#if defined(CONFIG_SERIAL_MSM_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
# define SUPPORT_SYSRQ
#endif

#include <linux/hrtimer.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/console.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/nmi.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <mach/msm_serial_pdata.h>
#include "msm_serial.h"

 //FIHTDC-DerrickDRLiu-DbgCfgTool-00+[
#ifdef CONFIG_FIH_REMOVE_SERIAL_DYNAMICALLY
#include <mach/msm_smd.h>  
#include <../devices.h>
#include <mach/gpio.h> //SW2-5-2-MP-DbgCfgTool-06+

extern int console_suspend_enabled; //SW2-5-2-MP-DbgCfgTool-08+
 //FIHTDC-DerrickDRLiu-DbgCfgTool-00+]
#endif

extern int ReadUartNV(unsigned int *bvalue);

/* FIH, CHHsieh, 2011/09/18 { */
/* turn on kernel uart log in FTM */
#ifndef CONFIG_FIH_FTM
//FIHTDC-DerrickDRLiu-DbgCfgTool-00+[
int uart_switch(void)
{
    struct clk *uart2_clk;
    unsigned int n_UartNV = 0;

    ReadUartNV(&n_UartNV);
    if(!n_UartNV)
    {
	printk(KERN_INFO "msm_serial: Disable UART2 DM clock\n");
	gpio_tlmm_config(GPIO_CFG(21, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_DISABLE);
	gpio_tlmm_config(GPIO_CFG(108, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_DISABLE);
	uart2_clk = clk_get(&msm_device_uart_dm2.dev, "uartdm_clk");        
	if(!IS_ERR(uart2_clk)){
		clk_enable(uart2_clk);   //If clk_enable is not called, calling clk_disable will enter ram dump.
		clk_disable(uart2_clk);        
	}
	#ifndef CONFIG_FIH_MODEM_REMOVE_UART2_CLK
	  /* Disable console suspend to guarantee there's no buffer copy delay after resume */
	  console_suspend_enabled = 0; //SW2-5-2-MP-DbgCfgTool-08+
	#endif /* CONFIG_FIH_MODEM_REMOVE_UART2_CLK */
	return 0; 
    }
    #ifndef CONFIG_FIH_MODEM_REMOVE_UART2_CLK
    /* Enable suspend console when UART clock is enable, it will buffer the kernel log, and emit it after resume
     * This can make sure DUT can enter suspend mode without interrupt */
    console_suspend_enabled = 1; //SW2-5-2-MP-DbgCfgTool-08+
    #endif /* CONFIG_FIH_MODEM_REMOVE_UART2_CLK */
    return 1;
}
//FIHTDC-DerrickDRLiu-DbgCfgTool-00+]    
EXPORT_SYMBOL(uart_switch);
#endif //CONFIG_FIH_FTM
/* } FIH, CHHsieh, 2011/09/18 */

