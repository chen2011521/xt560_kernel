/* Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
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
#include <fih/dynloader.h>

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_toshiba_fwvga.h"
#include <linux/gpio.h>
#include <mach/vreg.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>

static struct msm_panel_common_pdata *mipi_toshiba_fwvga_pdata;

static struct dsi_buf toshiba_tx_buf;
static struct dsi_buf toshiba_rx_buf;


static struct delayed_work   lcd_backlight_work;
static struct mutex         lcd_backlight_lock;
static spinlock_t           lcd_backlight_value_lock;
static __u32                lcd_backlight_level;

static boolean mipi_isr_done=FALSE;
static int bl_level_old = 31;
static struct proc_dir_entry *toshiba_lcm_file;

#define DELAYED_WORK_TIME   50
static char mcap_off[2] = {0xb2, 0x00};
static char ena_test_reg[3] = {0xEF, 0x01, 0x01};
static char two_lane[3] = {0xEF, 0x60, 0x63};
static char dmode_wvga[2] = {0xB3, 0x00};

static char hor_addr_2A_wvga[5] = {0x2A, 0x00, 0x00, 0x01, 0xdf};
static char hor_addr_2B_wvga[5] = {0x2B, 0x00, 0x00, 0x03, 0x55};

static char exit_sleep[2] = {0x11, 0x00};
static char display_on[2] = {0x29, 0x00};
static char display_off[2] = {0x28, 0x00};
static char enter_sleep[2] = {0x10, 0x00};

static struct dsi_cmd_desc toshiba_display_off_cmds[] = {
    {DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(display_off), display_off},
    {DTYPE_DCS_WRITE, 1, 0, 0, 120, sizeof(enter_sleep), enter_sleep}
};

static struct dsi_cmd_desc toshiba_display_on_cmds[] = {
    {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(mcap_off), mcap_off},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(ena_test_reg), ena_test_reg},
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(two_lane), two_lane},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(dmode_wvga), dmode_wvga},
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(hor_addr_2A_wvga),
        hor_addr_2A_wvga},
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(hor_addr_2B_wvga),
        hor_addr_2B_wvga},
    {DTYPE_DCS_WRITE, 1, 0, 0, 140, sizeof(exit_sleep), exit_sleep},
    {DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(display_on), display_on}
};

static char manufacture_id[2] = {0xa1, 0x00}; /* DTYPE_DCS_READ */
static struct dsi_cmd_desc toshiba_manufacture_id_cmd = {
    DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(manufacture_id), manufacture_id};

static uint32 toshiba_manufa_id;
static boolean id_readed = FALSE;

static uint32 mipi_toshiba_fwvga_manufacture_id(struct msm_fb_data_type *mfd)
{
    struct dsi_buf *rp, *tp;
    struct dsi_cmd_desc *cmd;
    uint32 *lp;

    tp = &toshiba_tx_buf;
    rp = &toshiba_rx_buf;
    mipi_dsi_buf_init(rp);
    mipi_dsi_buf_init(tp);

    cmd = &toshiba_manufacture_id_cmd;
    mipi_dsi_cmds_rx(mfd, tp, rp, cmd, 3);
    lp = (uint32 *)rp->data;

    pr_info("%s: manufacture_id=%x", __func__, *lp);
    printk(KERN_ERR "%s manufacture_id 0x%x\n", __func__,*lp);
    return *lp;
}

static int mipi_toshiba_fwvga_lcd_on(struct platform_device *pdev) 
{
    struct msm_fb_data_type *mfd;

    mfd = platform_get_drvdata(pdev);

    printk(KERN_ERR "[DISPLAY] Enter %s\n", __func__);
    if (!mfd)
        return -ENODEV;
    if (mfd->key != MFD_KEY)
        return -EINVAL;

	if(!id_readed) {
		   id_readed = TRUE;
		   toshiba_manufa_id = mipi_toshiba_fwvga_manufacture_id(mfd);
		   if(!toshiba_manufa_id)
		   		return 0;
	}

    MIPI_OUTP(MIPI_DSI_BASE + 0x190, 0x3180);
    mutex_lock(&mfd->dma->ov_mutex);

    mipi_set_tx_power_mode(1);

    mipi_dsi_cmds_tx(mfd, &toshiba_tx_buf, toshiba_display_on_cmds,
                     ARRAY_SIZE(toshiba_display_on_cmds));

    printk(KERN_ERR "[DISPLAY] %s toshiba_manufacture_id=0x%x \n", __func__,toshiba_manufa_id);
    mipi_set_tx_power_mode(0);
    mutex_unlock(&mfd->dma->ov_mutex);

    printk(KERN_ERR "[DISPLAY] exit %s\n", __func__);
    return 0;
}

static int mipi_toshiba_fwvga_lcd_off(struct platform_device *pdev) 
{

    struct msm_fb_data_type *mfd;

    mfd = platform_get_drvdata(pdev);

    printk(KERN_ERR "[DISPLAY] Enter %s\n", __func__);
    if (!mfd)
        return -ENODEV;
    if (mfd->key != MFD_KEY)
        return -EINVAL;

    mutex_lock(&mfd->dma->ov_mutex);
    mipi_dsi_cmds_tx(mfd, &toshiba_tx_buf, toshiba_display_off_cmds,
                     ARRAY_SIZE(toshiba_display_off_cmds));
    mipi_isr_done = FALSE;

    mutex_unlock(&mfd->dma->ov_mutex);

    return 0;
}

extern int proc_comm_set_led(unsigned id, unsigned level);

static void lcd_backlight_proc_work(struct work_struct *work)
{
    //printk(KERN_ERR "[BL] %s lcd_backlight_level=%d\n", __func__,lcd_backlight_level);
    mutex_lock(&lcd_backlight_lock);
    proc_comm_set_led(0, lcd_backlight_level);
    mutex_unlock(&lcd_backlight_lock);
}

static void mipi_toshiba_fwvga_lcd_backlight(struct msm_fb_data_type *mfd)
{
    unsigned long flags;

    //printk(KERN_ERR "[BL] Enter %s,mfd->bl_level=%d mipi_isr_done=%d\n", __func__,mfd->bl_level,mipi_isr_done);
    if(mfd->bl_level!=0){
        if (bl_level_old == mfd->bl_level)
            return;
    }

    spin_lock_irqsave(&lcd_backlight_value_lock, flags);
    lcd_backlight_level = mfd->bl_level;
    if (lcd_backlight_level == 0 ) {
        schedule_delayed_work(&lcd_backlight_work,0);
    }
    else {
        if (mipi_isr_done) {
            schedule_delayed_work(&lcd_backlight_work, msecs_to_jiffies(DELAYED_WORK_TIME));
        }
    }
    bl_level_old = mfd->bl_level;
    spin_unlock_irqrestore(&lcd_backlight_value_lock, flags);
}

void lcd_backlight_trigger_toshiba(void)
{

    //printk(KERN_ERR "[BL] Enter %s\n", __func__);
    if (mipi_isr_done)
        return;

    schedule_delayed_work(&lcd_backlight_work, msecs_to_jiffies(DELAYED_WORK_TIME));
    mipi_isr_done = TRUE;

}
EXPORT_SYMBOL(lcd_backlight_trigger_toshiba);

//static int __devinit mipi_toshiba_fwvga_lcd_probe(struct platform_device *pdev) 
DECL_PLATFORM_DRIVER_PROBE(DRIVER_TYPE_LCM, mipi_toshiba_fwvga_lcd_probe, pdev)
{
    printk(KERN_ERR "[DISPLAY] Enter %s pdev->id %d\n", __func__,pdev->id);
    if (pdev->id == 0) {
        mipi_toshiba_fwvga_pdata = pdev->dev.platform_data;
        return 0;
    }

    printk(KERN_ERR "[DISPLAY] Enter %s\n", __func__);
    msm_fb_add_device(pdev);

    return 0;
}

static struct platform_driver this_driver = {
    .probe  = mipi_toshiba_fwvga_lcd_probe,
    .driver = {
        .name   = "mipi_toshiba_fwvga",
    },
};

static struct msm_fb_panel_data toshiba_fwvga_panel_data = {
    .on     = mipi_toshiba_fwvga_lcd_on,
    .off        = mipi_toshiba_fwvga_lcd_off,
    .set_backlight = mipi_toshiba_fwvga_lcd_backlight,  
};

static int ch_used[3];

int mipi_toshiba_fwvga_device_register(struct msm_panel_info *pinfo,
                                       u32 channel, u32 panel) 
{
    struct platform_device *pdev = NULL;
    int ret;

    if ((channel >= 3) || ch_used[channel])
        return -ENODEV;

    ch_used[channel] = TRUE;

    printk(KERN_ERR "[DISPLAY] Enter %s\n", __func__);
    pdev = platform_device_alloc("mipi_toshiba_fwvga", (panel << 8)|channel);
    if (!pdev)
        return -ENOMEM;

    toshiba_fwvga_panel_data.panel_info = *pinfo;

    ret = platform_device_add_data(pdev, &toshiba_fwvga_panel_data,
                                   sizeof(toshiba_fwvga_panel_data));
    if (ret) {
        printk(KERN_ERR
               "%s: platform_device_add_data failed!\n", __func__);
        goto err_device_put;
    }

    ret = platform_device_add(pdev);
    if (ret) {
        printk(KERN_ERR
               "%s: platform_device_register failed!\n", __func__);
        goto err_device_put;
    }

    return 0;

    err_device_put:
    platform_device_put(pdev);
    return ret;
}

static int toshiba_proc_read(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{        
		printk("%s, toshiba_manufa_id 0x%x\n", __FUNCTION__, toshiba_manufa_id);
        return sprintf(page, "0x%x", toshiba_manufa_id);
}


static int __init mipi_toshiba_fwvga_lcd_init(void* hdl) 
{
    printk(KERN_ERR "[DISPLAY] Enter %s\n", __func__);
    mipi_dsi_buf_alloc(&toshiba_tx_buf, DSI_BUF_SIZE);
    mipi_dsi_buf_alloc(&toshiba_rx_buf, DSI_BUF_SIZE);

    mutex_init(&lcd_backlight_lock);
    spin_lock_init(&lcd_backlight_value_lock);
    INIT_DELAYED_WORK(&lcd_backlight_work, lcd_backlight_proc_work);

    toshiba_lcm_file=create_proc_entry("lcm_id", 0, NULL);	
    if(toshiba_lcm_file)
       toshiba_lcm_file->read_proc = toshiba_proc_read;
    return ADD_DYN_PLATFORM_DRIVER(hdl, &this_driver);
    //return platform_driver_register(&this_driver);
}

//module_init(mipi_toshiba_fwvga_lcd_init);
DECLARE_DYN_LOADER(DRIVER_TYPE_LCM, 
                   LCM_TOSHIBA_UPD161809_V1, 
                   mipi_toshiba_fwvga_lcd_init, LEVEL6);
