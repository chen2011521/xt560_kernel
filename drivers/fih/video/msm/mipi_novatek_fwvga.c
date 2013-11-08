/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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
#include "mipi_novatek_fwvga.h"
#include <mach/vreg.h>
#include <linux/gpio.h>

#include "mipi_himax_hx8363a.c"

static struct work_struct	lcd_backlight_work;
static struct mutex		    lcd_backlight_lock;
static spinlock_t		    lcd_backlight_value_lock;
static __u32                lcd_backlight_level;

static struct msm_panel_common_pdata *mipi_novatek_fwvga_pdata;

static struct dsi_buf novatek_fwvga_tx_buf;
static struct dsi_buf novatek_fwvga_rx_buf;
static struct proc_dir_entry *novatek_lcm_file = NULL;
//#define GPIO_LCM_RESET 129
#define GPIO_LCM_BKLIGHT 49
#define TIME_BASE  2  //2us - 180 us
#define TIME_FRAME 2  //at least 2 us
bool itv_var = 0;

//static unsigned mipi_dsi_gpio[] = {
//	GPIO_CFG(GPIO_LCM_RESET, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
//};

static unsigned lcm_bkl_gpio[] = {
	GPIO_CFG(GPIO_LCM_BKLIGHT, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_LCM_BKLIGHT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA)
};

//static struct vreg *vreg_l12_lcm_avdd = NULL;
//static struct vreg *vreg_s3_lcm_iovdd = NULL;
static int bl_level_old = 12;
//static int gpio_lcm_reset_request = 1;
static spinlock_t lock_onewire = SPIN_LOCK_UNLOCKED;
//static int display_initialize = 0;

/* novatek fwvga panel */
//static char sw_reset[2] = {0x01, 0x00}; /* DTYPE_DCS_WRITE */
//static char enter_sleep[1] = {0x10}; /* DTYPE_DCS_WRITE */
//static char exit_sleep[1] = {0x11}; /* DTYPE_DCS_WRITE */
//static char display_off[1] = {0x28}; /* DTYPE_DCS_WRITE */
//static char display_on[1] = {0x29}; /* DTYPE_DCS_WRITE */
//static char set_tear_on[2] = {0x35, 0x00}; /* DTYPE_DCS_WRITE1 */
//static char set_tear_scanline[3] = {0x44, 0x00, 0x00}; /* DTYPE_DCS_LWRITE */
/* FIH, Ming, Rotate 180 degree { */
static char set_address_mode[2] = {0x36, 0xD4}; /* DTYPE_DCS_WRITE1 */
/* } FIH, Ming, Rotate 180 degree */
/* FIH, Ming, Deep Standby Mode { */
static char enter_deep_standby[2] = {0x4F, 0x01}; /* DTYPE_DCS_WRITE1 */
/* } FIH, Ming, Deep Standby Mode */
//static char write_memory_start[1] = {0x2C}; /* DTYPE_DCS_LWRITE */
static char periperal_on[2]  = {0x00, 0x00};
static char peripheral_off[2] = {0x00, 0x00};

static struct dsi_cmd_desc novatek_fwvga_video_on_cmds[] = {
//	{DTYPE_DCS_WRITE, 1, 0, 0, 50,
//		sizeof(sw_reset), sw_reset},
//	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
//		sizeof(exit_sleep), exit_sleep},
//	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
//		sizeof(set_tear_on), set_tear_on},
//	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
//		sizeof(set_tear_scanline), set_tear_scanline},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 10,
		sizeof(set_address_mode), set_address_mode},
//	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
//		sizeof(write_memory_start), write_memory_start},
	{DTYPE_PERIPHERAL_ON, 1, 0, 0, 0,
		sizeof(periperal_on), periperal_on},
//	{DTYPE_DCS_WRITE, 1, 0, 0, 1,
//		sizeof(display_on), display_on},
};

static struct dsi_cmd_desc novatek_fwvga_display_off_cmds[] = {
	{DTYPE_PERIPHERAL_OFF, 1, 0, 0, 120,
		sizeof(peripheral_off), peripheral_off},
//	{DTYPE_DCS_WRITE, 1, 0, 0, 1,
//		sizeof(display_off), display_off},
//	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
//		sizeof(enter_sleep), enter_sleep},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,         /* FIH, Ming, 2011/11/02, Deep Standby Mode */
		sizeof(enter_deep_standby), enter_deep_standby}
};

enum {
	DSI_CMD_ON,
	DSI_CMD_OFF,
	DSI_CMD_NUM
};

#define DSI_CMDS(x) {x, ARRAY_SIZE(x)}

static struct {
	char * name;
	struct {
		struct dsi_cmd_desc * cmds;
		int size;
	} dsi[DSI_CMD_NUM];
} g_panel_para[]= {
	{"novatek",	{DSI_CMDS(novatek_fwvga_video_on_cmds), DSI_CMDS(novatek_fwvga_display_off_cmds)}},
	{"hx8363a", {DSI_CMDS(hx8363a_video_on_cmds), DSI_CMDS(hx8363a_display_off_cmds)}},
}, * g_panel = NULL;
int g_panel_id = -1;

//static struct dsi_cmd_desc novatek_panel_off_cmds[] = {
//	{DTYPE_DCS_WRITE, 1, 0, 0, 10,
//		sizeof(display_off), display_off}
//};

void inline onewire_send_addr(u8 addr);
int inline onewire_send_data(u8 data, int ack);
extern int g_mdpon_boot;

static char manufacture_id[2] = {0x04, 0x00}; /* DTYPE_DCS_READ */
static struct dsi_cmd_desc novatek_manufacture_id_cmd = {
    DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(manufacture_id), manufacture_id};

static uint32 novatek_manufa_id;

static uint32 mipi_novatek_fwvga_manufacture_id(struct msm_fb_data_type *mfd)
{
    struct dsi_buf *rp, *tp;
    struct dsi_cmd_desc *cmd;
    uint32 *lp;

    tp = &novatek_fwvga_tx_buf;
    rp = &novatek_fwvga_rx_buf;
    mipi_dsi_buf_init(rp);
    mipi_dsi_buf_init(tp);

    cmd = &novatek_manufacture_id_cmd;
    mipi_dsi_cmds_rx(mfd, tp, rp, cmd, 3);
    lp = (uint32 *)rp->data;

    pr_info("%s: manufacture_id=%x", __func__, *lp);
    printk(KERN_ERR "%s manufacture_id 0x%x\n", __func__,*lp);
    return *lp;
}

static char panel_hw_id[4] = {0xB9, 0xFF, 0x83, 0x63}; /* DTYPE_DCS_LWRITE */

static struct dsi_cmd_desc panel_wr_hw_id_cmds[] = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,
		sizeof(panel_hw_id), panel_hw_id},
};

static struct dsi_cmd_desc panel_rd_hw_id_cmds[] = {
	{DTYPE_DCS_READ, 1, 0, 1, 5,
		1, panel_hw_id},
};

extern void cpy_panel_name(char * src);
extern char * panel_name_arary[];

static int get_irm_panel_type(struct msm_fb_data_type *mfd)
{
	struct dsi_cmd_desc *cmd;
    uint32 *lp;

	mipi_dsi_cmds_tx(mfd, &novatek_fwvga_tx_buf, panel_wr_hw_id_cmds,
		ARRAY_SIZE(panel_wr_hw_id_cmds));
	cmd = panel_rd_hw_id_cmds;
	mipi_dsi_cmds_rx(mfd, &novatek_fwvga_tx_buf, &novatek_fwvga_rx_buf, cmd, 3);
	lp = (uint32 *)novatek_fwvga_rx_buf.data;
	printk(KERN_INFO "read panel reg[0xB9] : %x %x %x\n", novatek_fwvga_rx_buf.data[0], novatek_fwvga_rx_buf.data[1], novatek_fwvga_rx_buf.data[2]);
	cpy_panel_name(panel_name_arary[(*lp == 0x6383ff) ? 1 : 0]);
	return (*lp == 0x6383ff) ? 1 : 0;
}

static int mipi_novatek_fwvga_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct mipi_panel_info *mipi;
	int rc = 0;

	printk(KERN_INFO "[DISPLAY] +%s\n", __func__);
	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	mipi = &mfd->panel_info.mipi;

	if (!g_panel)
		g_panel = &g_panel_para[(g_panel_id = get_irm_panel_type(mfd))];

#if 0
	if (display_initialize)
		return 0;

	if (vreg_l12_lcm_avdd == NULL) {
		vreg_l12_lcm_avdd = vreg_get(NULL, "gp2");
		if (IS_ERR(vreg_l12_lcm_avdd)) {
			printk(KERN_ERR "[DISPLAY] %s: vreg_get(%s) failed (%ld)\n",
				__func__, "L12(gp2)", PTR_ERR(vreg_l12_lcm_avdd));
			return 0;
		}

		rc = vreg_set_level(vreg_l12_lcm_avdd, 2850);
		if (rc) {
			printk(KERN_ERR "[DISPLAY] %s: L12(GP2) set_level failed (%d)\n",
				__func__, rc);
			return 0;
		}
	}

	if (vreg_s3_lcm_iovdd == NULL) {
		vreg_s3_lcm_iovdd = vreg_get(NULL, "msme1");
		if (IS_ERR(vreg_s3_lcm_iovdd)) {
			printk(KERN_ERR "[DISPLAY] %s: vreg_get(%s) failed (%ld)\n",
				__func__, "S3(msme1)", PTR_ERR(vreg_s3_lcm_iovdd));
			return 0;
		}

		rc = vreg_set_level(vreg_s3_lcm_iovdd, 1800);
		if (rc) {
			printk(KERN_ERR "[DISPLAY] %s: S3(MSME1) set level failed (%d)\n",
				__func__, rc);
			return 0;
		}
	}

	rc = vreg_enable(vreg_l12_lcm_avdd);
	if (rc) {
		printk(KERN_ERR "[DISPLAY] %s: L12(GP2) enable failed (%d)\n",
			__func__, rc);
		return 0;
	}
	msleep(5);
	rc = vreg_enable(vreg_s3_lcm_iovdd);
	if (rc) {
		printk(KERN_ERR "[DISPLAY] %s: S3(MSME1) enable failed (%d)\n",
			__func__, rc);
		return 0;
	}
	msleep(25);

	if (!gpio_lcm_reset_request) {
		if (!gpio_tlmm_config(mipi_dsi_gpio[0], GPIO_CFG_ENABLE)) {
			gpio_direction_output(GPIO_LCM_RESET, 0);
			msleep(5);
			gpio_direction_output(GPIO_LCM_RESET, 1);
			msleep(120);
		} else {
			printk(KERN_ERR "[DISPLAY] Failed LCD reset enable\n");
			return 0;
		}
	} else {
		printk(KERN_ERR "[DISPLAY] Failed GPIO LCD reset\n");
		return 0;
	}
#endif	

    if ((g_mdpon_boot == 1) && novatek_lcm_file) {
        novatek_manufa_id = mipi_novatek_fwvga_manufacture_id(mfd);
        if (!novatek_manufa_id)
            return 0;
    }
	mutex_lock(&mfd->dma->ov_mutex);
	mipi_dsi_cmds_tx(mfd, &novatek_fwvga_tx_buf, g_panel->dsi[DSI_CMD_ON].cmds,
			g_panel->dsi[DSI_CMD_ON].size);
	mutex_unlock(&mfd->dma->ov_mutex);

    if ((!itv_var) && (g_mdpon_boot == 0))
    {
        unsigned long flags;
        spin_lock_irqsave(&lock_onewire, flags);
        gpio_set_value(GPIO_LCM_BKLIGHT, 1);
        udelay(100);    //detection delay

        gpio_set_value(GPIO_LCM_BKLIGHT, 0);
        udelay(260);    //detection time

        gpio_set_value(GPIO_LCM_BKLIGHT, 1);
        mdelay(1);      //detection window

        onewire_send_addr(0x72);
        rc = onewire_send_data(bl_level_old, 1);
        spin_unlock_irqrestore(&lock_onewire, flags);
    }
	else rc = -999;

    printk(KERN_INFO "[DISPLAY] -%s : bl(%d):%d\n", __func__, bl_level_old, rc);
//	display_initialize = 1;

	return 0;
}

static int mipi_novatek_fwvga_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	printk(KERN_INFO "[DISPLAY] +%s: (enter_sleep & enter_deep_standby)\n", __func__);

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

//	if (!display_initialize)
//		return 0;
    if (!itv_var)
    {
        unsigned long flags;
        spin_lock_irqsave(&lock_onewire, flags);
        gpio_set_value(GPIO_LCM_BKLIGHT, 0);
        spin_unlock_irqrestore(&lock_onewire, flags);
    }

	mutex_lock(&mfd->dma->ov_mutex);
	mipi_dsi_cmds_tx(mfd, &novatek_fwvga_tx_buf, g_panel->dsi[DSI_CMD_OFF].cmds,
			g_panel->dsi[DSI_CMD_OFF].size);
	mutex_unlock(&mfd->dma->ov_mutex);

/*
	if (!gpio_lcm_reset_request) {
		if (!gpio_tlmm_config(mipi_dsi_gpio[0], GPIO_CFG_ENABLE)) {
			gpio_direction_output(GPIO_LCM_RESET, 0);  //MTD-MM-DISPLAY-NC-LCM-02
		} else {
			printk(KERN_ERR "[DISPLAY] Failed LCD reset enable\n");
			return 0;
		}
	} else {
		printk(KERN_ERR "[DISPLAY] Failed GPIO LCD reset\n");
		return 0;
	}
*/
//	display_initialize = 0;

	return 0;
}

void onewire_reset(void)
{
	/*** enable 1-wire omde ***/
	
	gpio_direction_output(GPIO_LCM_BKLIGHT, 0);
	mdelay(3);

	gpio_set_value(GPIO_LCM_BKLIGHT, 1);
	udelay(100); //detection delay
//	printk( KERN_ERR "pull %d high (%d)\n",BL_CTRL, gpio_get_value(BL_CTRL));

	gpio_set_value(GPIO_LCM_BKLIGHT, 0);
	udelay(260); //detection time
//	printk( KERN_ERR "pull %d low (%d)\n",BL_CTRL, gpio_get_value(BL_CTRL));

	gpio_set_value(GPIO_LCM_BKLIGHT, 1);
//	printk( KERN_ERR "pull %d high (%d)\n",BL_CTRL, gpio_get_value(BL_CTRL));	

	mdelay(1); //detection window
}

void inline onewire_send_bit(u32 bit)
{
	int low_time,high_time;

	if (bit) {
		low_time = TIME_BASE;
		high_time = TIME_BASE * 3;
	} else {
		low_time = TIME_BASE * 3;
		high_time = TIME_BASE;
	}

	gpio_set_value(GPIO_LCM_BKLIGHT, 0);
	udelay(low_time);
	gpio_set_value(GPIO_LCM_BKLIGHT, 1);
	udelay(high_time);
}

void inline onewire_send_addr(u8 addr)
{
	int i;
	gpio_direction_output(GPIO_LCM_BKLIGHT, 1);
	udelay(TIME_FRAME);
	for (i = 7; i >= 0; i --) {
		if ((addr >> i) & 0x01) 
			onewire_send_bit(1);
		else
			onewire_send_bit(0);
	}
	gpio_set_value(GPIO_LCM_BKLIGHT, 0);
	udelay(TIME_FRAME);
}

int inline onewire_send_data(u8 data, int ack)
{
	int i, ret = 0;
	if (ack) {
		data = data | (0x01 << 7);
	}

	gpio_set_value(GPIO_LCM_BKLIGHT, 1);
	udelay(TIME_FRAME);

	for (i = 7; i >= 0; i --) {
		if ((data >> i) & 0x01) 
			onewire_send_bit(1);
		else
			onewire_send_bit(0);
	}
	gpio_set_value(GPIO_LCM_BKLIGHT, 0);
	udelay(TIME_FRAME);

	/*** handle ACK ***/
	if (!ack) {
		gpio_set_value(GPIO_LCM_BKLIGHT, 1);
	} else {
		gpio_set_value(GPIO_LCM_BKLIGHT, 1);

		/*  switch to input */
		gpio_tlmm_config(lcm_bkl_gpio[1], GPIO_CFG_ENABLE);

		udelay(100);  //2us + 512 us
		if (gpio_get_value(GPIO_LCM_BKLIGHT)) {
			printk(KERN_ERR "[DISPLAY] %s(): No ack !!!\n", __func__);
			ret = -1;
		} else {
			//pull low mean ack
//			printk(KERN_ERR "[DISPLAY] %s(): Got ack.\n", __func__);
		}

		/*  switch back to output */
		gpio_tlmm_config(lcm_bkl_gpio[0], GPIO_CFG_ENABLE);
	}

	return ret;
}

static void mipi_novatek_fwvga_lcd_backlight_v1(struct msm_fb_data_type *mfd)
{
	int rc, retry_count = 0;
	unsigned long flags;

	printk(KERN_INFO "[BL] set level %d->%d\n", bl_level_old, mfd->bl_level);

	if (bl_level_old == mfd->bl_level)
		return;

	if (mfd->bl_level > 31)
		mfd->bl_level = 31;

	if (mfd->bl_level < 0)
		mfd->bl_level = 0;

	spin_lock_irqsave(&lock_onewire, flags);
	if (gpio_get_value(GPIO_LCM_BKLIGHT) == 0) {
		bl_level_old = mfd->bl_level;
		spin_unlock_irqrestore(&lock_onewire, flags);
		printk(KERN_ERR "[BL] %s : skip.\n", __func__);
		return;
	}
	spin_unlock_irqrestore(&lock_onewire, flags);

	do {
		spin_lock_irqsave(&lock_onewire, flags);
		if (retry_count > 3) onewire_reset();
		onewire_send_addr(0x72);
		rc = onewire_send_data(mfd->bl_level, 1);
		spin_unlock_irqrestore(&lock_onewire, flags);
	} while (rc && (retry_count++ < 5));

	if (retry_count)
		printk(KERN_ERR "[BL] %s : retry %d - ret:%d\n", __func__, retry_count, rc);

	bl_level_old = mfd->bl_level;
}

extern int proc_comm_set_led(unsigned id, unsigned level);

static void lcd_backlight_proc_work(struct work_struct *work)
{
	mutex_lock(&lcd_backlight_lock);
    proc_comm_set_led(0, lcd_backlight_level);
	mutex_unlock(&lcd_backlight_lock);
}

static void mipi_novatek_fwvga_lcd_backlight_v2(struct msm_fb_data_type *mfd)
{
	unsigned long flags;
    if(mfd->bl_level!=0)                          //add by hjj 2/20
    {
        if(bl_level_old == mfd->bl_level)
            return;
    }

    spin_lock_irqsave(&lcd_backlight_value_lock, flags);
    lcd_backlight_level = mfd->bl_level;
	schedule_work(&lcd_backlight_work);
	spin_unlock_irqrestore(&lcd_backlight_value_lock, flags);
    bl_level_old = mfd->bl_level;
}
/* FIHTDC, Div2-SW2-BSP, Penho, IRM.B-1220 { */
void set_lcd_backlight(int lv)
{
    if (itv_var)
    {
        if (lv == 0)
        {
            proc_comm_set_led(0,0);
            printk(KERN_INFO "[BL] proc_comm_set_led 0 ");
        }
        else
        {
            proc_comm_set_led(0,40);                 /*add for ITV DIS_backlight*/
            printk(KERN_INFO "[BL] proc_comm_set_led 40 ");
        }
    }
    else
    {
        onewire_reset();
        onewire_send_addr(0x72);
        onewire_send_data(lv, 0);
    }
}
EXPORT_SYMBOL(set_lcd_backlight);
/* } FIHTDC, Div2-SW2-BSP, Penho, IRM.B-1220 */

//static int __devinit mipi_novatek_fwvga_lcd_probe(struct platform_device *pdev)
DECL_PLATFORM_DRIVER_PROBE(DRIVER_TYPE_LCM, mipi_novatek_fwvga_lcd_probe, pdev)
{
//	int i;
//	unsigned long flags;
	printk(KERN_INFO "[DISPLAY] +%s\n", __func__);

	if (pdev->id == 0) {
		mipi_novatek_fwvga_pdata = pdev->dev.platform_data;
		return 0;
	}

	msm_fb_add_device(pdev);

	spin_lock_init(&lock_onewire);	

	if (gpio_tlmm_config(lcm_bkl_gpio[0], GPIO_CFG_ENABLE)) {
		printk(KERN_ERR "[DISPLAY] Failed LCM BKL enable\n");
		return -1;
	}
	
	/* FIHSPEC, AmberYang { */
	/* Patch: FixTouchCannotUseIssue */
	gpio_request(126, "camera reset");
	gpio_set_value(126, 0);
	msleep(10);
	gpio_set_value(126, 1);
	msleep(10);
	gpio_set_value(126, 0);	
	gpio_free(126);
	/* } FIHSPEC, AmberYang */

    if(!itv_var) 
        gpio_request(GPIO_LCM_BKLIGHT, "lcd_bl_ctrl");

//	gpio_lcm_reset_request = gpio_request(GPIO_LCM_RESET, "lcd_reset");
/*
	spin_lock_irqsave(&lock_onewire, flags);
	onewire_reset();
	for (i = 1; i < 32; i ++) {
		onewire_send_addr(0x72);
		onewire_send_data(i, 0);
		mdelay(30);
	}
	spin_unlock_irqrestore(&lock_onewire, flags);
*/

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_novatek_fwvga_lcd_probe,
	.driver = {
		.name   = "mipi_novatek_fwvga",
	},
};

static struct msm_fb_panel_data novatek_fwvga_panel_data = {
	.on		= mipi_novatek_fwvga_lcd_on,
	.off	= mipi_novatek_fwvga_lcd_off,
	.set_backlight = mipi_novatek_fwvga_lcd_backlight_v1,
};

static int ch_used[3];

int mipi_novatek_fwvga_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	pdev = platform_device_alloc("mipi_novatek_fwvga", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	novatek_fwvga_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &novatek_fwvga_panel_data,
		sizeof(novatek_fwvga_panel_data));
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

static int novatek_proc_read(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{        
		printk("%s, novatek_manufa_id 0x%x\n", __FUNCTION__, novatek_manufa_id);
        return sprintf(page, "0x%x", novatek_manufa_id);
}

static int __init mipi_novatek_fwvga_lcd_init_v1(void* hdl)
{
	printk(KERN_INFO "[DISPLAY] +%s\n", __func__);
	mipi_dsi_buf_alloc(&novatek_fwvga_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&novatek_fwvga_rx_buf, DSI_BUF_SIZE);

    return ADD_DYN_PLATFORM_DRIVER(hdl, &this_driver);
	//return platform_driver_register(&this_driver);
}

//module_init(mipi_novatek_fwvga_lcd_init);
DECLARE_DYN_LOADER(DRIVER_TYPE_LCM, 
			       LCM_NOVATEK_NT35560_V1, 
			       mipi_novatek_fwvga_lcd_init_v1, LEVEL6);

static int __init mipi_novatek_fwvga_lcd_init_v2(void* hdl)
{
    itv_var = 1; 
    novatek_fwvga_panel_data.set_backlight = mipi_novatek_fwvga_lcd_backlight_v2;

	printk(KERN_INFO "[DISPLAY] +%s\n", __func__);
	mipi_dsi_buf_alloc(&novatek_fwvga_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&novatek_fwvga_rx_buf, DSI_BUF_SIZE);

    mutex_init(&lcd_backlight_lock);
    spin_lock_init(&lcd_backlight_value_lock);
    INIT_WORK(&lcd_backlight_work, lcd_backlight_proc_work);
  
    novatek_lcm_file=create_proc_entry("lcm_id", 0, NULL);	
    if(novatek_lcm_file)
       novatek_lcm_file->read_proc = novatek_proc_read;
    return ADD_DYN_PLATFORM_DRIVER(hdl, &this_driver);
}

DECLARE_DYN_LOADER(DRIVER_TYPE_LCM, 
			       LCM_NOVATEK_NT35560_V2, 
			       mipi_novatek_fwvga_lcd_init_v2, LEVEL6);

