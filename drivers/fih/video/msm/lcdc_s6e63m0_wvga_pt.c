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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <mach/gpio.h>
#include <mach/pmic.h>
#include <mach/socinfo.h>
#include "msm_fb.h"
#include "lcdc_s6e63m0_gamma.h"
#include <fih/dynloader.h>

struct s6e63m0_state_type {
	boolean disp_initialized;
	boolean display_on;
	boolean disp_powered_up;
};

static int spi_cs;
static int spi_sclk;
static int spi_mosi;

static struct s6e63m0_state_type s6e63m0_state = { 0 };
static struct msm_panel_common_pdata *lcdc_s6e63m0_pdata;
static struct proc_dir_entry *s6e63m0_lcm_file;
static unsigned char s6e63m0_manufa_id;
static boolean first_poweron = TRUE;
static struct mutex         lcd_disp_on_lock;
static __u32   lcd_backlight_level;


#define SLEEPMSEC		0x1000
#define ENDDEF			0x2000
#define	DEFMASK			0xFF00
#define COMMAND_ONLY		0xFE
#define DATA_ONLY		0xFF

#define MIN_BRIGHTNESS		0
#define MAX_BRIGHTNESS		10

static const unsigned short SEQ_PANEL_CONDITION_SET[] = {
	0xF8, 0x01,
	DATA_ONLY, 0x27,
	DATA_ONLY, 0x27,
	DATA_ONLY, 0x07,
	DATA_ONLY, 0x07,
	DATA_ONLY, 0x54,
	DATA_ONLY, 0x9f,
	DATA_ONLY, 0x63,
	DATA_ONLY, 0x86,
	DATA_ONLY, 0x1a,
	DATA_ONLY, 0x33,
	DATA_ONLY, 0x0d,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,

	ENDDEF, 0x0000
};


static const unsigned short SEQ_DISPLAY_CONDITION_SET[] = {
	0xf2, 0x02,
	DATA_ONLY, 0x03,//VBP
	DATA_ONLY, 0x1c,//VFP
	DATA_ONLY, 0x10,//HBP
	DATA_ONLY, 0x10,//HFP

	0xf7, 0x00,//scan direction (GTCON[0]&&SS)
	DATA_ONLY, 0x00,
	DATA_ONLY, 0xf0,

	ENDDEF, 0x0000
};

static const unsigned short SEQ_GAMMA_SETTING[] = {
	0xfa, 0x02,
	DATA_ONLY, 0x18,
	DATA_ONLY, 0x08,
	DATA_ONLY, 0x24,
	DATA_ONLY, 0x44,
	DATA_ONLY, 0x4a,
	DATA_ONLY, 0x2a,
	DATA_ONLY, 0xb5,
	DATA_ONLY, 0xbe,
	DATA_ONLY, 0xab,
	DATA_ONLY, 0xb1,
	DATA_ONLY, 0xba,
	DATA_ONLY, 0xa6,
	DATA_ONLY, 0xc5,
	DATA_ONLY, 0xca,
	DATA_ONLY, 0xbc,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x91,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x86,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0xbe,

	0xfa, 0x03,

	ENDDEF, 0x0000
};//s6e63m0_all_180

static const unsigned short SEQ_ETC_CONDITION_SET[] = {
	0xf6, 0x00,
	DATA_ONLY, 0x8e,
	DATA_ONLY, 0x07,

	0xb3, 0x6c,

	0xb5, 0x2c,
	DATA_ONLY, 0x12,
	DATA_ONLY, 0x0c,
	DATA_ONLY, 0x0a,
	DATA_ONLY, 0x10,
	DATA_ONLY, 0x0e,
	DATA_ONLY, 0x17,
	DATA_ONLY, 0x13,
	DATA_ONLY, 0x1f,
	DATA_ONLY, 0x1a,
	DATA_ONLY, 0x2a,
	DATA_ONLY, 0x24,
	DATA_ONLY, 0x1f,
	DATA_ONLY, 0x1b,
	DATA_ONLY, 0x1a,
	DATA_ONLY, 0x17,

	DATA_ONLY, 0x2b,
	DATA_ONLY, 0x26,
	DATA_ONLY, 0x22,
	DATA_ONLY, 0x20,
	DATA_ONLY, 0x3a,
	DATA_ONLY, 0x34,
	DATA_ONLY, 0x30,
	DATA_ONLY, 0x2c,
	DATA_ONLY, 0x29,
	DATA_ONLY, 0x26,
	DATA_ONLY, 0x25,
	DATA_ONLY, 0x23,
	DATA_ONLY, 0x21,
	DATA_ONLY, 0x20,
	DATA_ONLY, 0x1e,
	DATA_ONLY, 0x1e,

	0xb6, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x11,
	DATA_ONLY, 0x22,
	DATA_ONLY, 0x33,
	DATA_ONLY, 0x44,
	DATA_ONLY, 0x44,
	DATA_ONLY, 0x44,

	DATA_ONLY, 0x55,
	DATA_ONLY, 0x55,
	DATA_ONLY, 0x66,
	DATA_ONLY, 0x66,
	DATA_ONLY, 0x66,
	DATA_ONLY, 0x66,
	DATA_ONLY, 0x66,
	DATA_ONLY, 0x66,

	0xb7, 0x2c,
	DATA_ONLY, 0x12,
	DATA_ONLY, 0x0c,
	DATA_ONLY, 0x0a,
	DATA_ONLY, 0x10,
	DATA_ONLY, 0x0e,
	DATA_ONLY, 0x17,
	DATA_ONLY, 0x13,
	DATA_ONLY, 0x1f,
	DATA_ONLY, 0x1a,
	DATA_ONLY, 0x2a,
	DATA_ONLY, 0x24,
	DATA_ONLY, 0x1f,
	DATA_ONLY, 0x1b,
	DATA_ONLY, 0x1a,
	DATA_ONLY, 0x17,

	DATA_ONLY, 0x2b,
	DATA_ONLY, 0x26,
	DATA_ONLY, 0x22,
	DATA_ONLY, 0x20,
	DATA_ONLY, 0x3a,
	DATA_ONLY, 0x34,
	DATA_ONLY, 0x30,
	DATA_ONLY, 0x2c,
	DATA_ONLY, 0x29,
	DATA_ONLY, 0x26,
	DATA_ONLY, 0x25,
	DATA_ONLY, 0x23,
	DATA_ONLY, 0x21,
	DATA_ONLY, 0x20,
	DATA_ONLY, 0x1e,
	DATA_ONLY, 0x1e,

	0xb8, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x11,
	DATA_ONLY, 0x22,
	DATA_ONLY, 0x33,
	DATA_ONLY, 0x44,
	DATA_ONLY, 0x44,
	DATA_ONLY, 0x44,

	DATA_ONLY, 0x55,
	DATA_ONLY, 0x55,
	DATA_ONLY, 0x66,
	DATA_ONLY, 0x66,
	DATA_ONLY, 0x66,
	DATA_ONLY, 0x66,
	DATA_ONLY, 0x66,
	DATA_ONLY, 0x66,

	0xb9, 0x2c,
	DATA_ONLY, 0x12,
	DATA_ONLY, 0x0c,
	DATA_ONLY, 0x0a,
	DATA_ONLY, 0x10,
	DATA_ONLY, 0x0e,
	DATA_ONLY, 0x17,
	DATA_ONLY, 0x13,
	DATA_ONLY, 0x1f,
	DATA_ONLY, 0x1a,
	DATA_ONLY, 0x2a,
	DATA_ONLY, 0x24,
	DATA_ONLY, 0x1f,
	DATA_ONLY, 0x1b,
	DATA_ONLY, 0x1a,
	DATA_ONLY, 0x17,

	DATA_ONLY, 0x2b,
	DATA_ONLY, 0x26,
	DATA_ONLY, 0x22,
	DATA_ONLY, 0x20,
	DATA_ONLY, 0x3a,
	DATA_ONLY, 0x34,
	DATA_ONLY, 0x30,
	DATA_ONLY, 0x2c,
	DATA_ONLY, 0x29,
	DATA_ONLY, 0x26,
	DATA_ONLY, 0x25,
	DATA_ONLY, 0x23,
	DATA_ONLY, 0x21,
	DATA_ONLY, 0x20,
	DATA_ONLY, 0x1e,
	DATA_ONLY, 0x1e,

	0xba, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x11,
	DATA_ONLY, 0x22,
	DATA_ONLY, 0x33,
	DATA_ONLY, 0x44,
	DATA_ONLY, 0x44,
	DATA_ONLY, 0x44,

	DATA_ONLY, 0x55,
	DATA_ONLY, 0x55,
	DATA_ONLY, 0x66,
	DATA_ONLY, 0x66,
	DATA_ONLY, 0x66,
	DATA_ONLY, 0x66,
	DATA_ONLY, 0x66,
	DATA_ONLY, 0x66,

	0xc1, 0x4d,
	DATA_ONLY, 0x96,
	DATA_ONLY, 0x1d,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x01,
	DATA_ONLY, 0xdf,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x03,
	DATA_ONLY, 0x1f,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x00,
	DATA_ONLY, 0x03,
	DATA_ONLY, 0x06,
	DATA_ONLY, 0x09,
	DATA_ONLY, 0x0d,
	DATA_ONLY, 0x0f,
	DATA_ONLY, 0x12,
	DATA_ONLY, 0x15,
	DATA_ONLY, 0x18,

	0xb2, 0x10,
	DATA_ONLY, 0x10,
	DATA_ONLY, 0x0b,
	DATA_ONLY, 0x05,

	ENDDEF, 0x0000
};

static const unsigned short SEQ_ACL_ON[] = {
	/* ACL on */
	0xc0, 0x01,

	ENDDEF, 0x0000
};

static const unsigned short SEQ_ACL_OFF[] = {
	/* ACL off */
	0xc0, 0x00,

	ENDDEF, 0x0000
};

static const unsigned short SEQ_ELVSS_ON[] = {
	/* ELVSS on */
	0xb1, 0x0b,

	ENDDEF, 0x0000
};

static const unsigned short SEQ_ELVSS_OFF[] = {
	/* ELVSS off */
	0xb1, 0x0a,

	ENDDEF, 0x0000
};

static const unsigned short SEQ_STAND_BY_OFF[] = {
	0x11, COMMAND_ONLY,
    SLEEPMSEC,120,
	ENDDEF, 0x0000
};

static const unsigned short SEQ_STAND_BY_ON[] = {
	0x10, COMMAND_ONLY,

	ENDDEF, 0x0000
};

static const unsigned short SEQ_DISPLAY_ON[] = {
	0x29, COMMAND_ONLY,

	ENDDEF, 0x0000
};

// SPI Transfer Timing format 
//         __                                                               ______
// SPI_CS    \__~__________________________________________________________/ 
//
//         _____  ____   ____   ____   ____   ____   ____   ____   ____   _______
// SPI_CLK      ~     |_|    |_|    |_|    |_|    |_|    |_|    |_|    |_|    |_|
//
//                    ___    ___    ___    ___    ___    ___    ___    ___
// SDO/SDI _____~ ___/DCX\__/D7 \__/D6 \__/D5 \__/D4 \__/D3 \__/D2 \__/D1 \__/D0 \______
//                   \___/  \___/  \___/  \___/  \___/  \___/  \___/  \___/  \___/

static void s6e63m0_spi_start(void)
{
    gpio_direction_output(spi_cs, 0);

}

static void s6e63m0_spi_stop(void)
{
    gpio_direction_output(spi_cs, 1);
}

static void s6e63m0_spi_write_bit(unsigned char byData)
{    

    gpio_set_value (spi_sclk, 0);

    udelay(1);

    if(byData == 0)
    {
        gpio_set_value (spi_mosi, 0);
    }
    else
    {
        gpio_set_value (spi_mosi, 1);
    }
    gpio_set_value (spi_sclk, 1);//rising trigger
    udelay(1);

}

unsigned char s6e63m0_spi_read_bit(void)
{
    unsigned char dwValue;

    gpio_set_value (spi_sclk, 0);    
    udelay(1);   
    dwValue = !!gpio_get_value(spi_mosi);
    gpio_set_value (spi_sclk, 1);    
    udelay(1);   
    return dwValue;
}

unsigned char s6e63m0_spi_read_byte(void)
{
    unsigned char i =0;
    unsigned char byData=0;
    
    for(i = 0; i < 8; i++)
    {
        byData <<= 1;    
        byData |= s6e63m0_spi_read_bit();
    }

    return byData;
}

static void s6e63m0_spi_write_9bits(unsigned char byData, unsigned char dcx)
{
    unsigned char i, byMask=0x80;

    s6e63m0_spi_write_bit(dcx); // 0: command, 1: parameter
                   
    for(i = 0; i < 8; i++)
    {
        s6e63m0_spi_write_bit((unsigned char)(byData & byMask));
        byMask = byMask >>1;
    }
}


unsigned char s6e63m0_lcdc_remote_read(unsigned char cmd,unsigned char num)
{
    int index = 0;
    unsigned char byDate = 0;

    s6e63m0_spi_stop();
    gpio_set_value (spi_sclk, 1); // 0: command, 1: parameter
    mdelay(1);

    s6e63m0_spi_start();

    printk(KERN_INFO "s6e63m0_lcdc_remote_read() \n");
    // write command
    s6e63m0_spi_write_9bits(cmd, 0);

    gpio_tlmm_config(GPIO_CFG(spi_mosi, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, 
    	GPIO_CFG_2MA),GPIO_CFG_ENABLE);

    for ( index = 0; index < num; index++ )
    {
        byDate = s6e63m0_spi_read_byte();
        printk(KERN_INFO "s6e63m0_lcdc_remote_read() ,cmd = 0x%x,byDate = 0x%x\n",cmd,byDate);
    } 
    s6e63m0_spi_stop();
    gpio_tlmm_config(GPIO_CFG(spi_mosi, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, 
    GPIO_CFG_2MA),GPIO_CFG_ENABLE);
    mdelay(10);
    return byDate;
}


void s6e63m0_spi_write(unsigned char address,
	unsigned char command)
{

    s6e63m0_spi_stop();
    gpio_set_value (spi_sclk, 1); // 0: command, 1: parameter
    mdelay(1);
    s6e63m0_spi_start();

	if (address != DATA_ONLY)
      // write command
     s6e63m0_spi_write_9bits(address, 0);
	if (command != COMMAND_ONLY)
     // write parameters
    s6e63m0_spi_write_9bits(command, 1);

    s6e63m0_spi_stop();
    mdelay(1);

}

void s6e63m0_panel_send_sequence(const unsigned short *wbuf)
{
	int i = 0;

	//printk( KERN_INFO "[panel lcdc s6e63m0]: %s\n", __func__ );
	while ((wbuf[i] & DEFMASK) != ENDDEF) {
		if ((wbuf[i] & DEFMASK) != SLEEPMSEC)
			s6e63m0_spi_write(wbuf[i], wbuf[i+1]);
		else
			udelay(wbuf[i+1]*1000);
		i += 2;
	}

}

static void s6e63m0_disp_powerup(void)
{
	if (!s6e63m0_state.disp_powered_up && !s6e63m0_state.display_on) {
		/* Reset the hardware first */
		/* Include DAC power up implementation here */
	      s6e63m0_state.disp_powered_up = TRUE;
	}
}

static void _s6e63m0_gamma_ctl(const unsigned int *gamma)
{
	unsigned int i = 0;

	/* disable gamma table updating. */
	s6e63m0_spi_write(0xfa, 0x00);

	for (i = 0 ; i < GAMMA_TABLE_COUNT; i++) {
		s6e63m0_spi_write(DATA_ONLY, gamma[i]);		
	}

	/* update gamma table. */
	s6e63m0_spi_write(0xfa, 0x01);
}


static void s6e63m0_gamma_ctl(int gamma)
{

    printk( KERN_INFO "[panel lcdc s6e63m0]: %s gamma=%d\n", __func__ ,gamma);
    _s6e63m0_gamma_ctl(gamma_table.gamma_all_table[gamma]);
}

void lcdc_s6e63m0_initial(void)
{
	  int i, j;
	const unsigned short *init_seq_set_condition[] = {
		SEQ_PANEL_CONDITION_SET,
		SEQ_DISPLAY_CONDITION_SET,
	};
    const unsigned short *init_seq_dis_on[] = {
		SEQ_ETC_CONDITION_SET,
		SEQ_ACL_ON,
		SEQ_ELVSS_ON,
        SEQ_STAND_BY_OFF,
        SEQ_DISPLAY_ON,
	};

    printk( KERN_INFO "[panel lcdc s6e63m0]: %s lcd_backlight_level=%d\n", __func__ ,lcd_backlight_level);

    mutex_lock(&lcd_disp_on_lock);
	for (i = 0; i < ARRAY_SIZE(init_seq_set_condition); i++) {
		s6e63m0_panel_send_sequence(init_seq_set_condition[i]);
	}

    if(first_poweron){
        first_poweron = FALSE;
        s6e63m0_panel_send_sequence(SEQ_GAMMA_SETTING);
    }else
      s6e63m0_gamma_ctl(lcd_backlight_level);

	for (j = 0; j < ARRAY_SIZE(init_seq_dis_on); j++) {
		s6e63m0_panel_send_sequence(init_seq_dis_on[j]);
    }
     mutex_unlock(&lcd_disp_on_lock);
}


static int lcdc_s6e63m0_panel_on(struct platform_device *pdev)
{

    printk( KERN_INFO "[panel lcdc s6e63m0]: %s\n", __func__ );

    if(first_poweron)
    {
        s6e63m0_manufa_id=s6e63m0_lcdc_remote_read(0xda,1);
        /*Read ID1 0xfe*/
        if(0xfe != s6e63m0_manufa_id)
            return 0;
    }

    if (!s6e63m0_state.disp_initialized) {
        /* Configure reset GPIO that drives DAC */
        if (lcdc_s6e63m0_pdata->panel_config_gpio)
            lcdc_s6e63m0_pdata->panel_config_gpio(1);
            s6e63m0_disp_powerup();
        if (s6e63m0_state.disp_powered_up && !s6e63m0_state.display_on){
            mutex_init(&lcd_disp_on_lock);
            lcdc_s6e63m0_initial();                  
            s6e63m0_state.display_on = TRUE;
        }

        s6e63m0_state.disp_initialized = TRUE;
    }


    return 0;
}


static int lcdc_s6e63m0_panel_off(struct platform_device *pdev)
{
	printk( KERN_INFO "[panel lcdc s6e63m0]: %s\n", __func__ );
    if (s6e63m0_state.disp_powered_up && s6e63m0_state.display_on){
        s6e63m0_panel_send_sequence(SEQ_STAND_BY_ON);
        if (lcdc_s6e63m0_pdata->panel_config_gpio)
                lcdc_s6e63m0_pdata->panel_config_gpio(0);
            s6e63m0_state.display_on = FALSE;
            s6e63m0_state.disp_initialized = FALSE;   
    }

	return 0;
}

static void lcdc_s6e63m0_set_backlight(struct msm_fb_data_type *mfd)
{
	int bl_level;

	bl_level = mfd->bl_level;

	printk( KERN_INFO "[backlight s6e63m0]: %s, bl_level=%d \n", __func__,bl_level);

	if (lcdc_s6e63m0_pdata)
		s6e63m0_gamma_ctl(bl_level);
	else
		pr_err("%s(): Backlight level set failed", __func__);

    if(mfd->bl_level)
       lcd_backlight_level = mfd->bl_level;
}

static void s6e63m0_spi_pin_assign(void)
{
	/* Setting the Default GPIO's */
	spi_mosi  = *(lcdc_s6e63m0_pdata->gpio_num);
	spi_sclk  = *(lcdc_s6e63m0_pdata->gpio_num + 1);
	spi_cs  = *(lcdc_s6e63m0_pdata->gpio_num + 2);
}

static int __devinit s6e63m0_probe(struct platform_device *pdev)
{
	printk( KERN_INFO "[panel lcdc s6e63m0]: %s\n", __func__ );
	if (pdev->id == 0) {
		lcdc_s6e63m0_pdata = pdev->dev.platform_data;
		s6e63m0_spi_pin_assign();
		return 0;
	}
	msm_fb_add_device(pdev);
	return 0;
}

static struct platform_driver this_driver = {
	.probe  = s6e63m0_probe,
	.driver = {
		.name   = "lcdc_s6e63m0_wvga_pt",
	},
};

static struct msm_fb_panel_data s6e63m0_panel_data = {
	.on = lcdc_s6e63m0_panel_on,
	.off = lcdc_s6e63m0_panel_off,
	.set_backlight = lcdc_s6e63m0_set_backlight,
};

static struct platform_device this_device = {
	.name   = "lcdc_s6e63m0_wvga_pt",
	.id	= 1,
	.dev	= {
		.platform_data = &s6e63m0_panel_data,
	}
};

static int s6e63m0_proc_read(char *page, char **start, off_t off,
                             int count, int *eof, void *data)
{
    /*add for FTM code consistent*/
    unsigned char manufa_id = ~s6e63m0_manufa_id;
    printk("%s, manufa_id 0x%x\n", __FUNCTION__, manufa_id);
    return sprintf(page, "0x%x",manufa_id);
}

static int __init lcdc_s6e63m0_panel_init(void* hdl)
{
    int ret;
    struct msm_panel_info *pinfo;

    printk(KERN_INFO "[panel lcdc s6e63m0]: %s\n", __func__ );

    ret = msm_fb_detect_client("lcdc_s6e63m0_wvga_pt");
    if (ret){
        printk(KERN_INFO "[panel lcdc s6e63m0]: %s msm_fb_detect_client failed\n", __func__ );
            return 0;
    }

    ADD_DYN_PLATFORM_DRIVER(hdl, &this_driver);
    if (ret)
        return ret;

    pinfo = &s6e63m0_panel_data.panel_info;
    pinfo->xres = 480;
    pinfo->yres = 800;
    MSM_FB_SINGLE_MODE_PANEL(pinfo);
    pinfo->type = LCDC_PANEL;
    pinfo->pdest = DISPLAY_1;
    pinfo->wait_cycle = 0;
    pinfo->bpp = 24;
    pinfo->fb_num = 2;
    /* 30Mhz mdp_lcdc_pclk and mdp_lcdc_pad_pcl */
    pinfo->clk_rate = 23800000; /*clk *pixel_mdp_clk; */   /*clk *pixel_lcdc_clk;*/
    pinfo->bl_max = MAX_BRIGHTNESS;
    pinfo->bl_min = 1;

    pinfo->lcdc.h_back_porch = 10; /* HBP-HLW */
    pinfo->lcdc.h_front_porch = 10;
    pinfo->lcdc.h_pulse_width = 2;/* HLW = 2*DOT */
    pinfo->lcdc.v_back_porch = 1;/* VBP-VLW */
    pinfo->lcdc.v_front_porch = 32;
    pinfo->lcdc.v_pulse_width = 2;/* VLW = 2*HSYNC */
    pinfo->lcdc.border_clr = 0;     /* blk */
    pinfo->lcdc.underflow_clr = 0xff;       /* blue */
    pinfo->lcdc.hsync_skew = 0;

/* FIH, Panel_Size 3.97  */
	pinfo->width = 52;  // 51.85mm 
	pinfo->height = 87; // 86.4mm
/* FIH, Panel_Size 3.97 */

    s6e63m0_lcm_file=create_proc_entry("lcm_id", 0, NULL);	
    if(s6e63m0_lcm_file)
       s6e63m0_lcm_file->read_proc = s6e63m0_proc_read;

    ret = platform_device_register(&this_device);

    if (ret) {
        printk(KERN_ERR "%s not able to register the device\n",
               __func__);
        platform_driver_unregister(&this_driver);
    }
    return ret;
}

//device_initcall(lcdc_s6e63m0_panel_init);
DECLARE_DYN_LOADER(DRIVER_TYPE_LCM, 
                   LCM_SAMSUNG_S6E63M0X_V1, 
                   lcdc_s6e63m0_panel_init, LEVEL6);
