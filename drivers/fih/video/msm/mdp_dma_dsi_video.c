/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/fb.h>
#include <asm/system.h>
#include <mach/hardware.h>
#include "mdp.h"
#include "msm_fb.h"
#include "mdp4.h"
/* FIHTDC, Div2-SW2-BSP, Penho, IRM.B-1044 { */
#ifdef CONFIG_FIH_7X27A_PROJS
#include "mipi_dsi.h"
#endif	// CONFIG_FIH_7X27A_PROJS
/* } FIHTDC, Div2-SW2-BSP, Penho, IRM.B-1044 */

#define DSI_VIDEO_BASE	0xF0000
#define DMA_P_BASE      0x90000

static int first_pixel_start_x;
static int first_pixel_start_y;
/* FIHTDC, Div2-SW2-BSP, Penho, IRM.B-1220 { */
#ifdef CONFIG_FIH_7X27A_PROJS
int g_mdpon_boot = 1;
static struct timer_list lcdon_timer;

extern void set_lcd_backlight(int lv);

static void boot_lcdbl_on_func(unsigned long unused)
{
	set_lcd_backlight(12);
	g_mdpon_boot = 0;
}
#endif	// CONFIG_FIH_7X27A_PROJS
/* } FIHTDC, Div2-SW2-BSP, Penho, IRM.B-1220 */

int mdp_dsi_video_on(struct platform_device *pdev)
{
	int dsi_width;
	int dsi_height;
	int dsi_bpp;
	int dsi_border_clr;
	int dsi_underflow_clr;
	int dsi_hsync_skew;

	int hsync_period;
	int hsync_ctrl;
	int vsync_period;
	int display_hctl;
	int display_v_start;
	int display_v_end;
	int active_hctl;
	int active_h_start;
	int active_h_end;
	int active_v_start;
	int active_v_end;
	int ctrl_polarity;
	int h_back_porch;
	int h_front_porch;
	int v_back_porch;
	int v_front_porch;
	int hsync_pulse_width;
	int vsync_pulse_width;
	int hsync_polarity;
	int vsync_polarity;
	int data_en_polarity;
	int hsync_start_x;
	int hsync_end_x;
	uint8 *buf;
	uint32 dma2_cfg_reg;

	int bpp;
	struct fb_info *fbi;
	struct fb_var_screeninfo *var;
	struct msm_fb_data_type *mfd;
	int ret;

	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;

	if (mfd->key != MFD_KEY)
		return -EINVAL;

	fbi = mfd->fbi;
	var = &fbi->var;

	bpp = fbi->var.bits_per_pixel / 8;
	buf = (uint8 *) fbi->fix.smem_start;
	buf += fbi->var.xoffset * bpp +
		fbi->var.yoffset * fbi->fix.line_length;

	dma2_cfg_reg = DMA_PACK_ALIGN_LSB | DMA_OUT_SEL_DSI_VIDEO;

	if (mfd->fb_imgType == MDP_BGR_565)
		dma2_cfg_reg |= DMA_PACK_PATTERN_BGR;
	else if (mfd->fb_imgType == MDP_RGBA_8888)
		dma2_cfg_reg |= DMA_PACK_PATTERN_BGR;
	else
		dma2_cfg_reg |= DMA_PACK_PATTERN_RGB;

	if (bpp == 2)
		dma2_cfg_reg |= DMA_IBUF_FORMAT_RGB565;
	else if (bpp == 3)
		dma2_cfg_reg |= DMA_IBUF_FORMAT_RGB888;
	else
		dma2_cfg_reg |= DMA_IBUF_FORMAT_xRGB8888_OR_ARGB8888;

	switch (mfd->panel_info.bpp) {
	case 24:
		dma2_cfg_reg |= DMA_DSTC0G_8BITS |
			DMA_DSTC1B_8BITS | DMA_DSTC2R_8BITS;
		break;
	case 18:
		dma2_cfg_reg |= DMA_DSTC0G_6BITS |
			DMA_DSTC1B_6BITS | DMA_DSTC2R_6BITS;
		break;
	case 16:
		dma2_cfg_reg |= DMA_DSTC0G_6BITS |
			DMA_DSTC1B_5BITS | DMA_DSTC2R_5BITS;
		break;
	default:
		printk(KERN_ERR "mdp lcdc can't support format %d bpp!\n",
			mfd->panel_info.bpp);
		return -ENODEV;
	}
	/* MDP cmd block enable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);

/* FIHTDC, Div2-SW2-BSP, Penho, IRM.B-1044 { */
#ifdef CONFIG_FIH_7X27A_PROJS
	if ((inpdw(MDP_BASE + 0xF0000)) & 0x01) { /* enabled already */
/* FIHTDC, Div2-SW2-BSP, Penho, IRM.B-1220 { */
        if (g_mdpon_boot) {
        	set_lcd_backlight(0);
        	setup_timer(&lcdon_timer, boot_lcdbl_on_func, 0);
        	mod_timer(&lcdon_timer, jiffies + msecs_to_jiffies(650));
        }
/* } FIHTDC, Div2-SW2-BSP, Penho, IRM.B-1220 */

        //mipi_dsi_panel_power_on_first = 0;
        printk(KERN_INFO "[DISPLAY] %s: +mipi_dsi_shutdown\n", __func__);
        // mipi_dsi_shutdown
        writel(0x00000000, MDP_BASE + 0xF0000);//0xF0000 MDP_DSI_VIDEO_EN
        mdelay(60);
        writel(0x00000000, MDP_BASE + 0x20);//0x20 MDP_INTR_ENABLE
        writel(0x01010101, MIPI_DSI_BASE + 0x010C);//0x010C DSI_INT_CTRL
        writel(0x13FF3BFF, MIPI_DSI_BASE + 0x0108);//0x0108 DSI_ERR_INT_MASK0
        writel(0, MIPI_DSI_BASE + 0x0200);//0x0200 DSIPHY_PLL_CTRL_0
        writel(0, MIPI_DSI_BASE + 0x0118);//0x0118 DSI_CLK_CTRL
        writel(0, MIPI_DSI_BASE + 0x0000);//0x0000 DSI_CTRL

        /*Turning off DMA_P block*/
        mdp_pipe_ctrl(MDP_DMA2_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);		/* IRM.B-375 */
	}
	else g_mdpon_boot = 0;	// FIHTDC, Div2-SW2-BSP, Penho, IRM.B-1220
#endif	// CONFIG_FIH_7X27A_PROJS
/* } FIHTDC, Div2-SW2-BSP, Penho, IRM.B-1044 */

	/* starting address */
	MDP_OUTP(MDP_BASE + DMA_P_BASE + 0x8, (uint32) buf);

	/* active window width and height */
	MDP_OUTP(MDP_BASE + DMA_P_BASE + 0x4, ((fbi->var.yres) << 16) |
		(fbi->var.xres));

	/* buffer ystride */
	MDP_OUTP(MDP_BASE + DMA_P_BASE + 0xc, fbi->fix.line_length);

	/* x/y coordinate = always 0 for lcdc */
	MDP_OUTP(MDP_BASE + DMA_P_BASE + 0x10, 0);

	/* dma config */
	MDP_OUTP(MDP_BASE + DMA_P_BASE, dma2_cfg_reg);

	/*
	 * DSI timing setting
	 */
	h_back_porch = var->left_margin;
	h_front_porch = var->right_margin;
	v_back_porch = var->upper_margin;
	v_front_porch = var->lower_margin;
	hsync_pulse_width = var->hsync_len;
	vsync_pulse_width = var->vsync_len;
	dsi_border_clr = mfd->panel_info.lcdc.border_clr;
	dsi_underflow_clr = mfd->panel_info.lcdc.underflow_clr;
	dsi_hsync_skew = mfd->panel_info.lcdc.hsync_skew;
	dsi_width = mfd->panel_info.xres;
	dsi_height = mfd->panel_info.yres;
	dsi_bpp = mfd->panel_info.bpp;
	hsync_period = h_back_porch + dsi_width + h_front_porch + 1;
	hsync_ctrl = (hsync_period << 16) | hsync_pulse_width;
	hsync_start_x = h_back_porch;
	hsync_end_x = dsi_width + h_back_porch - 1;
	display_hctl = (hsync_end_x << 16) | hsync_start_x;

	vsync_period =
		(v_back_porch + dsi_height + v_front_porch + 1) * hsync_period;
	display_v_start = v_back_porch * hsync_period + dsi_hsync_skew;
	display_v_end = (dsi_height + v_back_porch) * hsync_period;

	active_h_start = hsync_start_x + first_pixel_start_x;
	active_h_end = active_h_start + var->xres - 1;
	active_hctl = ACTIVE_START_X_EN |
			(active_h_end << 16) | active_h_start;

	active_v_start = display_v_start +
			first_pixel_start_y * hsync_period;
	active_v_end = active_v_start +	(var->yres) * hsync_period - 1;
	active_v_start |= ACTIVE_START_Y_EN;

	dsi_underflow_clr |= 0x80000000;	/* enable recovery */
	hsync_polarity = 0;
	vsync_polarity = 0;
	data_en_polarity = 0;

	ctrl_polarity =	(data_en_polarity << 2) |
		(vsync_polarity << 1) | (hsync_polarity);

	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x4, hsync_ctrl);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x8, vsync_period);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0xc, vsync_pulse_width);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x10, display_hctl);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x14, display_v_start);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x18, display_v_end);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x1c, active_hctl);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x20, active_v_start);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x24, active_v_end);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x28, dsi_border_clr);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x2c, dsi_underflow_clr);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x30, dsi_hsync_skew);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE + 0x38, ctrl_polarity);

	ret = panel_next_on(pdev);
	if (ret == 0) {
		/* enable DSI block */
		MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE, 1);
		/*Turning on DMA_P block*/
		mdp_pipe_ctrl(MDP_DMA2_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
	}

	/* MDP cmd block disable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);

	return ret;
}

int mdp_dsi_video_off(struct platform_device *pdev)
{
	int ret = 0;
	/* MDP cmd block enable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
	MDP_OUTP(MDP_BASE + DSI_VIDEO_BASE, 0);
	/* MDP cmd block disable */
	mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);
	/*Turning off DMA_P block*/
	mdp_pipe_ctrl(MDP_DMA2_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);

	ret = panel_next_off(pdev);
	/* delay to make sure the last frame finishes */
	msleep(20);

	return ret;
}

void mdp_dsi_video_update(struct msm_fb_data_type *mfd)
{
	struct fb_info *fbi = mfd->fbi;
	uint8 *buf;
	int bpp;
	unsigned long flag;
	int irq_block = MDP_DMA2_TERM;

	if (!mfd->panel_power_on)
		return;

	down(&mfd->dma->mutex);

	bpp = fbi->var.bits_per_pixel / 8;
	buf = (uint8 *) fbi->fix.smem_start;
	buf += fbi->var.xoffset * bpp +
		fbi->var.yoffset * fbi->fix.line_length;
	/* no need to power on cmd block since it's dsi mode */
	/* starting address */
	MDP_OUTP(MDP_BASE + DMA_P_BASE + 0x8, (uint32) buf);
	/* enable  irq */
	spin_lock_irqsave(&mdp_spin_lock, flag);
	mdp_enable_irq(irq_block);
	INIT_COMPLETION(mfd->dma->comp);
	mfd->dma->waiting = TRUE;

	outp32(MDP_INTR_CLEAR, LCDC_FRAME_START);
	mdp_intr_mask |= LCDC_FRAME_START;
	outp32(MDP_INTR_ENABLE, mdp_intr_mask);

	spin_unlock_irqrestore(&mdp_spin_lock, flag);
	wait_for_completion_killable(&mfd->dma->comp);
	mdp_disable_irq(irq_block);
	up(&mfd->dma->mutex);
}
