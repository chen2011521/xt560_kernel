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
#include "mipi_renesas_qvga.h"
#include <linux/gpio.h>
#include <mach/vreg.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>

static struct msm_panel_common_pdata *mipi_renesas_qvga_pdata;

static struct dsi_buf renesas_tx_buf;
static struct dsi_buf renesas_rx_buf;

static struct delayed_work   lcd_backlight_work;
static struct mutex         lcd_backlight_lock;
static spinlock_t           lcd_backlight_value_lock;
static __u32                lcd_backlight_level;

static boolean mipi_isr_done=FALSE;
static int bl_level_old = 31;
static struct proc_dir_entry *renesas_lcm_file;

#define DELAYED_WORK_TIME   50

/*FIH-NJDC-SW4 Jenny for PR2.5 panel performance form vendor {*/
static char command_protect[2] = {0xb0, 0x04};
static char set_address_mode[2] = {0x36, 0x00};
static char set_pixel_format[2] = {0x3a,0x07};

static char setting_panel[9] = {0xc0, 0x13, 0x3f, 0x40, 0x10, 0x00, 0x01, 0x00};
static char setting_display_timing[6] = {0xc1, 0x06, 0x2D, 0x08, 0x08, 0x00};
static char setting_source_timing[5] = {0xc4, 0x20, 0x00, 0x02, 0x01};
static char  setting_gama_a[25] = {0xc8, 0x02, 0x12, 0x1C, 0x27, 0x36, 0x4C, 0x3F, 0x34, 0x2D, 0x28, 0x23, 0x05, 0x02, 0x12, 0x1C, 0x27, 0x36, 0x4c, 0x3F, 0x34, 0x2D, 0x28, 0x23, 0x05};
static char  setting_gama_b[25] = {0xc9, 0x02, 0x12, 0x1C, 0x27, 0x36, 0x4C, 0x3F, 0x34, 0x2D, 0x28, 0x23, 0x05, 0x02, 0x12, 0x1C, 0x27, 0x36, 0x4c, 0x3F, 0x34, 0x2D, 0x28, 0x23, 0x05};
static char  setting_gama_c[25] = {0xca, 0x02, 0x12, 0x1C, 0x27, 0x36, 0x4C, 0x3F, 0x34, 0x2D, 0x28, 0x23, 0x05, 0x02, 0x12, 0x1C, 0x27, 0x36, 0x4c, 0x3F, 0x34, 0x2D, 0x28, 0x23, 0x05};
static char power_setting[17] = {0xd0, 0x31, 0x0c, 0x0E, 0x10, 0x29, 0x04, 0x01, 0x00, 0x08, 0x01, 0x00, 0x06, 0x01, 0x00, 0x00, 0x20};
static char vcom_setting[5] = {0xd1, 0x02, 0x2F, 0x2F, 0x2A};
static char set_DDB_write_control[7] = {0xE1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static char NVM_load_control[2] = {0xE2,0x80};
static char hor_addr_2A_wvga[5] = {0x2A, 0x00, 0x00, 0x01, 0xdf};
static char hor_addr_2B_wvga[5] = {0x2B, 0x00, 0x00, 0x01, 0x3f};
static char exit_sleep_mod[2] = {0x11, 0x00};
static char display_on[2] = {0x29, 0x00};
static char display_off[2] = {0x28, 0x00};
static char enter_sleep[2] = {0x10, 0x00};
/*} FIH-NJDC-SW4 Jenny for PR2.5 panel performance form vendor */

static struct dsi_cmd_desc renesas_display_off_cmds[] = {
    {DTYPE_DCS_WRITE, 1, 0, 0,  0, sizeof(display_off), display_off},
    {DTYPE_DCS_WRITE, 1, 0, 0, 10, sizeof(enter_sleep), enter_sleep}
};

static struct dsi_cmd_desc renesas_display_on_cmds[] = {
    {DTYPE_GEN_WRITE2, 1, 0, 0, 0, sizeof(command_protect), command_protect},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(setting_panel),
        setting_panel},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(setting_display_timing),
        setting_display_timing},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(setting_source_timing),
        setting_source_timing},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(setting_gama_a),
        setting_gama_a},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(setting_gama_b),
        setting_gama_b},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(setting_gama_c),
        setting_gama_c},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(power_setting),
        power_setting},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(vcom_setting),
        vcom_setting},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(set_DDB_write_control),
        set_DDB_write_control},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(NVM_load_control),
        NVM_load_control},

    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(set_pixel_format),
        set_pixel_format},
    {DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(set_address_mode),
        set_address_mode},

    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(hor_addr_2A_wvga),
        power_setting},
    {DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(hor_addr_2B_wvga),
        power_setting},
    {DTYPE_DCS_WRITE, 1, 0, 0, 130, sizeof(exit_sleep_mod), exit_sleep_mod},
    {DTYPE_DCS_WRITE, 1, 0, 0, 100, sizeof(display_on), display_on}
};

static uint32 renesas_manufa_id;
static boolean id_readed = FALSE;

static char manufacture_id[2] = {0xbf, 0x00}; /* DTYPE_DCS_READ */
static struct dsi_cmd_desc renesas_manufacture_id_cmd = {
    DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(manufacture_id), manufacture_id};

static char macp_id[2] = {0xb0, 0x00}; /* DTYPE_DCS_READ */
static struct dsi_cmd_desc renesas_macp_id_cmd = {
    DTYPE_DCS_READ, 1, 0, 1, 1, sizeof(macp_id), macp_id};

static uint32 mipi_renesas_qvga_manufacture_id(struct msm_fb_data_type *mfd)
{
    struct dsi_buf *rp, *tp;
    struct dsi_cmd_desc *cmd;
    uint32 *lp;

    tp = &renesas_tx_buf;
    rp = &renesas_rx_buf;
    mipi_dsi_buf_init(rp);
    mipi_dsi_buf_init(tp);

    cmd = &renesas_manufacture_id_cmd;
    mipi_dsi_cmds_rx(mfd, tp, rp, cmd, 3);
    lp = (uint32 *)rp->data;
    pr_info("%s: manufacture_id=%x", __func__, *lp);
    printk(KERN_ERR "%s manufacture_id 0x%x\n", __func__,*lp);
    return *lp;
}

static uint32 mipi_renesas_qvga_macp_id(struct msm_fb_data_type *mfd)
{
    struct dsi_buf *rp, *tp;
    struct dsi_cmd_desc *cmd;
    uint32 *lp;

    tp = &renesas_tx_buf;
    rp = &renesas_rx_buf;
    mipi_dsi_buf_init(rp);
    mipi_dsi_buf_init(tp);

    cmd = &renesas_macp_id_cmd;
    mipi_dsi_cmds_rx(mfd, tp, rp, cmd, 3);
    lp = (uint32 *)rp->data;
    pr_info("%s: macp_id=%x", __func__, *lp);
    printk(KERN_ERR "%s macp_id 0x%x\n", __func__,*lp);
    return *lp;
}

static int mipi_renesas_qvga_lcd_on(struct platform_device *pdev) 
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
		mipi_renesas_qvga_macp_id(mfd);
		mdelay(2);
		renesas_manufa_id = mipi_renesas_qvga_manufacture_id(mfd);
		if(!renesas_manufa_id)
		   return 0;
	}

    mutex_lock(&mfd->dma->ov_mutex);

    mipi_set_tx_power_mode(1);

    mipi_dsi_cmds_tx(mfd, &renesas_tx_buf, renesas_display_on_cmds,
                     ARRAY_SIZE(renesas_display_on_cmds));


    mipi_set_tx_power_mode(0);

    mutex_unlock(&mfd->dma->ov_mutex);

    printk(KERN_ERR "[DISPLAY] exit %s\n", __func__);
    return 0;
}

static int mipi_renesas_qvga_lcd_off(struct platform_device *pdev) 
{
    struct msm_fb_data_type *mfd;

    mfd = platform_get_drvdata(pdev);

    printk(KERN_ERR "[DISPLAY] Enter %s\n", __func__);
    if (!mfd)
        return -ENODEV;
    if (mfd->key != MFD_KEY)
        return -EINVAL;

    mutex_lock(&mfd->dma->ov_mutex);
    mipi_dsi_cmds_tx(mfd, &renesas_tx_buf, renesas_display_off_cmds,
                     ARRAY_SIZE(renesas_display_off_cmds));
    mutex_unlock(&mfd->dma->ov_mutex);
    mipi_isr_done = FALSE;
    msleep(120);

    return 0;
}

extern int proc_comm_set_led(unsigned id, unsigned level);

static void lcd_backlight_proc_work(struct work_struct *work)
{
    mutex_lock(&lcd_backlight_lock);
    proc_comm_set_led(0, lcd_backlight_level);
    mutex_unlock(&lcd_backlight_lock);
}


static void mipi_renesas_qvga_lcd_backlight(struct msm_fb_data_type *mfd)
{
    unsigned long flags;

    //printk(KERN_ERR "[BL] Enter %s,mfd->bl_level=%d mdp_isr_done=%d\n", __func__,mfd->bl_level,mdp_isr_done);

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

void lcd_backlight_trigger_renesas(void)
{

    //printk(KERN_ERR "[BL] Enter %s\n", __func__);
    if (mipi_isr_done)
        return;

    schedule_delayed_work(&lcd_backlight_work, msecs_to_jiffies(DELAYED_WORK_TIME));
    mipi_isr_done = TRUE;
}
EXPORT_SYMBOL(lcd_backlight_trigger_renesas);

DECL_PLATFORM_DRIVER_PROBE(DRIVER_TYPE_LCM, mipi_renesas_qvga_lcd_probe, pdev)
{
    printk(KERN_ERR "[DISPLAY] Enter %s pdev->id %d\n", __func__,pdev->id);
    if (pdev->id == 0) {
        mipi_renesas_qvga_pdata = pdev->dev.platform_data;
        return 0;
    }

    printk(KERN_ERR "[DISPLAY] Enter %s\n", __func__);
    msm_fb_add_device(pdev);

    return 0;
}

static struct platform_driver this_driver = {
    .probe  = mipi_renesas_qvga_lcd_probe,
    .driver = {
        .name   = "mipi_renesas_qvga",
    },
};

static struct msm_fb_panel_data renesas_qvga_panel_data = {
    .on     = mipi_renesas_qvga_lcd_on,
    .off        = mipi_renesas_qvga_lcd_off,
    .set_backlight = mipi_renesas_qvga_lcd_backlight,  
};

static int ch_used[3];

int mipi_renesas_qvga_device_register(struct msm_panel_info *pinfo,
                                      u32 channel, u32 panel) 
{
    struct platform_device *pdev = NULL;
    int ret;

    if ((channel >= 3) || ch_used[channel])
        return -ENODEV;

    ch_used[channel] = TRUE;

    printk(KERN_ERR "[DISPLAY] Enter %s\n", __func__);
    pdev = platform_device_alloc("mipi_renesas_qvga", (panel << 8)|channel);
    if (!pdev)
        return -ENOMEM;

    renesas_qvga_panel_data.panel_info = *pinfo;

    ret = platform_device_add_data(pdev, &renesas_qvga_panel_data,
                                   sizeof(renesas_qvga_panel_data));
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
static int renesas_proc_read(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{        
		printk("%s, renesas_manufa_id 0x%x\n", __FUNCTION__, renesas_manufa_id);
        return sprintf(page, "0x%x", renesas_manufa_id);
}

static int __init mipi_renesas_qvga_lcd_init(void* hdl) 
{
    printk(KERN_ERR "[DISPLAY] Enter %s\n", __func__);
    mipi_dsi_buf_alloc(&renesas_tx_buf, DSI_BUF_SIZE);
    mipi_dsi_buf_alloc(&renesas_rx_buf, DSI_BUF_SIZE);

    mutex_init(&lcd_backlight_lock);
    spin_lock_init(&lcd_backlight_value_lock);
    INIT_DELAYED_WORK(&lcd_backlight_work, lcd_backlight_proc_work);

    renesas_lcm_file=create_proc_entry("lcm_id", 0, NULL);	
    if(renesas_lcm_file)
        renesas_lcm_file->read_proc = renesas_proc_read;

    return ADD_DYN_PLATFORM_DRIVER(hdl, &this_driver);
}

DECLARE_DYN_LOADER(DRIVER_TYPE_LCM, 
                   LCM_RENESAS_R61531_V1, 
                   mipi_renesas_qvga_lcd_init, LEVEL6);


