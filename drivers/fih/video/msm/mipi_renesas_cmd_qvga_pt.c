/* Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
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

static struct msm_panel_info pinfo;

static struct mipi_dsi_phy_ctrl dsi_cmd_mode_phy_db = {
/* DSI Bit Clock at 221.18 MHz, 1 lane, RGB888 */
/* regulator */
    {0x13, 0x01, 0x01, 0x00},
/* timing   */
    {0x66, 0x26, 0x0e, 0x00, 0x0f, 0x84, 0x1e, 0x88,
        0x0f, 0x03, 0x04},
/* phy ctrl */
    {0x7f, 0x00, 0x00, 0x00},
/* strength */
    {0xbb, 0x02, 0x06, 0x00},
/* pll control */
    {0x01, 0xb9, 0x31, 0xda, 0x00, 0x40, 0x37, 0x62,
     0x01, 0x0f, 0x07,
     0x05, 0x14, 0x03, 0x0, 0x0, 0x0, 0x20, 0x0, 0x02, 0x0},
};

static int __init mipi_cmd_renesas_qvga_pt_init(void* hdl)
{
    int ret;

#ifdef CONFIG_FB_MSM_MIPI_PANEL_DETECT
    if (msm_fb_detect_client("mipi_cmd_renesas_qvga"))
        return 0;
#endif

    printk(KERN_ERR "[DISPLAY] Enter %s\n", __func__);

    pinfo.xres = 480;
    pinfo.yres = 320; 
    pinfo.type = MIPI_CMD_PANEL;
    pinfo.pdest = DISPLAY_1;
    pinfo.wait_cycle = 0;
    pinfo.bpp = 24;

	pinfo.lcdc.h_back_porch =100;
	pinfo.lcdc.h_front_porch = 100;
	pinfo.lcdc.h_pulse_width = 20;
	pinfo.lcdc.v_back_porch = 20;
	pinfo.lcdc.v_front_porch = 20;
	pinfo.lcdc.v_pulse_width = 5;

	pinfo.lcdc.border_clr = 0;	/* blk */
	pinfo.lcdc.underflow_clr = 0xff;	/* blue */
	pinfo.lcdc.hsync_skew = 0;
    pinfo.bl_max = 100;
    pinfo.bl_min = 1;
    pinfo.fb_num = 2;
	pinfo.clk_rate = 221000000;
    /*has MDP vsync*/
	/*pinfo.lcd.vsync_enable = TRUE;
	pinfo.lcd.hw_vsync_mode = TRUE;*/
	pinfo.lcd.refx100 = 5000; /* adjust refx100 to prevent tearing */

    pinfo.mipi.mode = DSI_CMD_MODE;
    pinfo.mipi.dst_format = DSI_CMD_DST_FORMAT_RGB888;
    pinfo.mipi.vc = 0;
    pinfo.mipi.rgb_swap = DSI_RGB_SWAP_RGB;
    pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.t_clk_post = 0x22;
	pinfo.mipi.t_clk_pre = 0x39;
    pinfo.mipi.stream = 0;  /* dma_p */
    pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_SW;
    pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
    pinfo.mipi.te_sel = 0; /* TE from data link */
    pinfo.mipi.interleave_max = 1;
    pinfo.mipi.insert_dcs_cmd = TRUE;
    pinfo.mipi.wr_mem_continue = 0x3c;
    pinfo.mipi.wr_mem_start = 0x2c;
    pinfo.mipi.dsi_phy_db = &dsi_cmd_mode_phy_db;
    pinfo.mipi.tx_eot_append = 0x01;
    pinfo.mipi.rx_eot_ignore = 0x00;
    pinfo.mipi.dlane_swap =0x01;

    ret = mipi_renesas_qvga_device_register(&pinfo, MIPI_DSI_PRIM,
                                             MIPI_DSI_PANEL_FWVGA_PT);
    if (ret)
        printk(KERN_ERR "%s: failed to register device!\n", __func__);

    return ret;
}

DECLARE_DYN_LOADER(DRIVER_TYPE_LCM, 
			       LCM_RENESAS_R61531_V1, 
			       mipi_cmd_renesas_qvga_pt_init, LEVEL6);


