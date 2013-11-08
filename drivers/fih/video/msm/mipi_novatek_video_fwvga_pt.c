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

static struct msm_panel_info pinfo;

static struct mipi_dsi_phy_ctrl dsi_video_mode_phy_db_v1 = {
		/* DSI Bit Clock at 400 MHz, 2 lane, RGB888, 48fps*/
		/* regulator */
		{0x03, 0x01, 0x01, 0x00},
		/* timing */
		{0x74, 0x2d, 0x0f, 0x00, 0x3c, 0x47, 0x14, 0x31,
		0x13, 0x03, 0x04},
		/* phy ctrl */
		{0x7f, 0x00, 0x00, 0x00},
		/* strength */
		{0xff, 0x02, 0x06, 0x00},
		/* pll control */
		{0x00, 0x8a, 0x31, 0xd2, 0x00, 0x40, 0x37, 0x62,
		0x01, 0x0f, 0x07,
		0x05, 0x14, 0x03, 0x0, 0x0, 0x0, 0x20, 0x0, 0x02, 0x0}
};

static struct mipi_dsi_phy_ctrl dsi_video_mode_phy_db_v2 = {
		/* DSI_BIT_CLK at 355 Mhz, 2 lane, RGB888 */
		{0x13, 0x01, 0x01, 0x00}, /* regulator */                    //0x03-->0x13 change the DC/DC mode to LDO mode  2012-3-5  BSP-HJJ
		/* timing */
		{0xA8, 0x89, 0x16, 0x0, 0x91, 0x95, 0x19,
		0x8b, 0x10, 0x03, 0x04},
		{0x7f, 0x00, 0x00, 0x00}, /* phy ctrl */
		{0xee, 0x02, 0x86, 0x00}, /* strength */
		/* pll control */
		{0x40, 0x5e, 0x31, 0xd2, 0x00, 0x50, 0x48, 0x63,
		0x01, 0x0F, 0x07,
		0x05, 0x14, 0x03, 0x0, 0x0, 0x54, 0x06, 0x10, 0x04, 0x0},
};
static int __init mipi_video_novatek_fwvga_pt_init_v1(void* hdl)
{
	int ret;

#ifdef CONFIG_FB_MSM_MIPI_PANEL_DETECT
	if (msm_fb_detect_client("mipi_video_novatek_fwvga"))
		return 0;
#endif

	printk(KERN_INFO "[DISPLAY] +%s\n", __func__);
	pinfo.xres = 480;
	pinfo.yres = 854;
	pinfo.type = MIPI_VIDEO_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 24;
	pinfo.lcdc.h_back_porch = 125;	//min:0.5us	//80;
	pinfo.lcdc.h_front_porch = 125;	//min:0.5us	//24;
	pinfo.lcdc.h_pulse_width = 8;
	pinfo.lcdc.v_back_porch = 11;	//16;
	pinfo.lcdc.v_front_porch = 4;	//8;
	pinfo.lcdc.v_pulse_width = 1;
	pinfo.clk_rate = 499000000;
	pinfo.lcdc.border_clr = 0;	/* blk */
	pinfo.lcdc.underflow_clr = 0xff;	/* blue */
	pinfo.lcdc.hsync_skew = 0;
	pinfo.bl_max = 31;
	pinfo.bl_min = 1;
	pinfo.fb_num = 2;
	/* FIH, Ming, Panel_Size { */
	pinfo.lcd.refx100 = 6000;
	pinfo.width = 50;  // 50.04mm 
	pinfo.height = 89; // 89.03mm
	/* } FIH, Ming, Panel_Size */

	pinfo.mipi.mode = DSI_VIDEO_MODE;
	pinfo.mipi.pulse_mode_hsa_he = TRUE;
	pinfo.mipi.hfp_power_stop = TRUE;
	pinfo.mipi.hbp_power_stop = TRUE;
	pinfo.mipi.hsa_power_stop = TRUE;
	pinfo.mipi.eof_bllp_power_stop = TRUE;
	pinfo.mipi.bllp_power_stop = TRUE;
	pinfo.mipi.traffic_mode = DSI_BURST_MODE;
	pinfo.mipi.dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
	pinfo.mipi.vc = 0;
	pinfo.mipi.rgb_swap = DSI_RGB_SWAP_RGB;
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.data_lane1 = TRUE;
	pinfo.mipi.tx_eot_append = TRUE;
	pinfo.mipi.t_clk_post = 0x20;
	pinfo.mipi.t_clk_pre = 0x2F;
	pinfo.mipi.stream = 0; /* dma_p */
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_NONE;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.frame_rate = 60;
	pinfo.mipi.dsi_phy_db = &dsi_video_mode_phy_db_v1;
	pinfo.mipi.dlane_swap = 0x01;

	ret = mipi_novatek_fwvga_device_register(&pinfo, MIPI_DSI_PRIM,
						MIPI_DSI_PANEL_FWVGA_PT);
	if (ret)
		pr_err("%s: failed to register device!\n", __func__);

	return ret;
}

static int __init mipi_video_novatek_fwvga_pt_init_v2(void* hdl)
{
	int ret;

#ifdef CONFIG_FB_MSM_MIPI_PANEL_DETECT
	if (msm_fb_detect_client("mipi_video_novatek_fwvga"))
		return 0;
#endif

	printk(KERN_INFO "[DISPLAY] +%s\n", __func__);
	pinfo.xres = 480;
	pinfo.yres = 854;
	pinfo.type = MIPI_VIDEO_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 24;
	pinfo.lcdc.h_back_porch = 200;   //min 0.5us //80
	pinfo.lcdc.h_front_porch = 200;  //min 0.5us //24
	pinfo.lcdc.h_pulse_width = 8;
	pinfo.lcdc.v_back_porch = 20;     //16
	pinfo.lcdc.v_front_porch = 10;     //8
	pinfo.lcdc.v_pulse_width = 1;
	pinfo.clk_rate = 499000000;
	pinfo.lcdc.border_clr = 0;	/* blk */
	pinfo.lcdc.underflow_clr = 0xff;	/* blue */
	pinfo.lcdc.hsync_skew = 0;
	pinfo.bl_max = 100;               //31-->100  huanjingjing-12/14
	pinfo.bl_min = 1;
	pinfo.fb_num = 2;
	/* FIH, Ming, Panel_Size { */
	pinfo.lcd.refx100 = 6000;
	pinfo.width = 50;  // 50.04mm 
	pinfo.height = 89; // 89.03mm
	/* } FIH, Ming, Panel_Size */

	pinfo.mipi.mode = DSI_VIDEO_MODE;
	pinfo.mipi.pulse_mode_hsa_he = TRUE;
	pinfo.mipi.hfp_power_stop = TRUE;
	pinfo.mipi.hbp_power_stop = TRUE;
	pinfo.mipi.hsa_power_stop = TRUE;
	pinfo.mipi.eof_bllp_power_stop = TRUE;
	pinfo.mipi.bllp_power_stop = TRUE;
	pinfo.mipi.traffic_mode = DSI_BURST_MODE;
	pinfo.mipi.dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
	pinfo.mipi.vc = 0;
	pinfo.mipi.rgb_swap = DSI_RGB_SWAP_RGB;
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.data_lane1 = TRUE;
	pinfo.mipi.tx_eot_append = TRUE;
	pinfo.mipi.t_clk_post = 0x20;
	pinfo.mipi.t_clk_pre = 0x2F;
	pinfo.mipi.stream = 0; /* dma_p */
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_NONE;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.frame_rate = 60;
	pinfo.mipi.dsi_phy_db = &dsi_video_mode_phy_db_v2;
	pinfo.mipi.dlane_swap = 0x01;

	ret = mipi_novatek_fwvga_device_register(&pinfo, MIPI_DSI_PRIM,
						MIPI_DSI_PANEL_FWVGA_PT);
	if (ret)
		pr_err("%s: failed to register device!\n", __func__);

	return ret;
}

DECLARE_DYN_LOADER(DRIVER_TYPE_LCM, 
			       LCM_NOVATEK_NT35560_V1, 
			       mipi_video_novatek_fwvga_pt_init_v1, LEVEL6);

DECLARE_DYN_LOADER(DRIVER_TYPE_LCM, 
			       LCM_NOVATEK_NT35560_V2, 
			       mipi_video_novatek_fwvga_pt_init_v2, LEVEL6);

