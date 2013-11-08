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
 */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <media/msm_camera.h>
#include <mach/camera.h>
#include <mach/gpio.h>
#include <fih/dynloader.h>
#include "mt9v115.h"

#undef  CDBG
#define CDBG    printk

/*=============================================================
	SENSOR REGISTER DEFINES
==============================================================*/
#define Q8    0x00000100

/* Omnivision8810 product ID register address */
#define REG_MT9V115_MODEL_ID_MSB                       0x0A
#define REG_MT9V115_MODEL_ID_LSB                       0x0B
#define REG_MT9V115_CHIP_VERSION                       0x0000

#define MT9V115_CHIP_VERSION                           0x2284
/* Omnivision8810 product ID */

/* Time in milisecs for waiting for the sensor to reset */
#define MT9V115_RESET_DELAY_MSECS    66
#define MT9V115_DEFAULT_CLOCK_RATE   24000000
/* Registers*/

/* Color bar pattern selection */
#define MT9V115_COLOR_BAR_PATTERN_SEL_REG     0x82
/* Color bar enabling control */
#define MT9V115_COLOR_BAR_ENABLE_REG           0x601
/* Time in milisecs for waiting for the sensor to reset*/
#define MT9V115_RESET_DELAY_MSECS    66

#define MT9V115_MSB_MASK			0xFF00
#define MT9V115_LSB_MASK			0x00FF

//vince modify for auto-frame rate+++
#define MT9V115_AUTO_FRAME_RATE
//vince modify for auto-frame rate---
/*============================================================================
							DATA DECLARATIONS
============================================================================*/

/* 22MHz
[Timing_settings]
REG = 0x300A, 0x01F9		//frame_length_lines = 958 > 505
REG = 0x300C, 0x02D6		//line_length_pck = 726
REG = 0x3010, 0x0012		//fine_correction = 18
REG = 0x9803, 0x07		    //stat_fd_zone_height = 7
REG = 0xA06E, 0x0098		//cam_fd_config_fdperiod_50hz = 152
REG = 0xA070, 0x007E		//cam_fd_config_fdperiod_60hz = 126
REG = 0xA072, 0x11		    //cam_fd_config_search_f1_50 = 17
REG = 0xA073, 0x13		    //cam_fd_config_search_f2_50 = 19
REG = 0xA074, 0x14		    //cam_fd_config_search_f1_60 = 20
REG = 0xA075, 0x16		    //cam_fd_config_search_f2_60 = 22
REG = 0xA076, 0x0014		//cam_fd_config_max_fdzone_50hz = 13(7.5fps) > 33(3fps) > 20(5fps)
REG = 0xA078, 0x0018		//cam_fd_config_max_fdzone_60hz = 16 > 40 > 24
REG = 0xA01A, 0x0014		//cam_ae_config_target_fdzone =   13 > 33 > 20
*/
/* 24MHz
[Timing_settings]
REG = 0x300A, 0x0204                    //frame_length_lines = 958 > 516
REG = 0x300C, 0x02D6                    //line_length_pck = 726
REG = 0x3010, 0x0012                     //fine_correction = 18
REG = 0x9803, 0x07                         //stat_fd_zone_height = 7
REG = 0xA06E, 0x009B                    //cam_fd_config_fdperiod_50hz = 155
REG = 0xA070, 0x0081                    //cam_fd_config_fdperiod_60hz = 129
REG = 0xA072, 0x11                         //cam_fd_config_search_f1_50 = 17
REG = 0xA073, 0x13                         //cam_fd_config_search_f2_50 = 19
REG = 0xA074, 0x15                         //cam_fd_config_search_f1_60 = 21
REG = 0xA075, 0x17                         //cam_fd_config_search_f2_60 = 23
REG = 0xA076, 0x0003                    //cam_fd_config_max_fdzone_50hz = 
REG = 0xA078, 0x0004                    //cam_fd_config_max_fdzone_60hz = 
REG = 0xA01A, 0x0003                   //cam_ae_config_target_fdzone =   
*/
/* TEST
[Timing_settings]
REG = 0x300A, 0x0226                    //frame_length_lines = 958 > 516
REG = 0x300C, 0x02D6                    //line_length_pck = 726
REG = 0x3010, 0x0012                     //fine_correction = 18
REG = 0x3040, 0x0041
REG = 0x9803, 0x07                         //stat_fd_zone_height = 7
REG = 0xA06E, 0x00A5                    //cam_fd_config_fdperiod_50hz = 155
REG = 0xA070, 0x008A                    //cam_fd_config_fdperiod_60hz = 129
REG = 0xA072, 0x12                         //cam_fd_config_search_f1_50 = 17
REG = 0xA073, 0x14                         //cam_fd_config_search_f2_50 = 19
REG = 0xA074, 0x16                         //cam_fd_config_search_f1_60 = 21
REG = 0xA075, 0x18                         //cam_fd_config_search_f2_60 = 23
REG = 0xA076, 0x0003                    //cam_fd_config_max_fdzone_50hz = 
REG = 0xA078, 0x0004                    //cam_fd_config_max_fdzone_60hz = 
REG = 0xA01A, 0x001A                   //cam_ae_config_target_fdzone =   
*/

struct reg_addr_val_pair_struct mt9v115_init_timing_array[] = {
// [Timing_settings]
#if 1 
    //MCLK=24MHz
    /*
    {0x300A, 0x0204 },      //frame_length_lines = 958 > 516                          
    {0x300C, 0x02D6 },      //line_length_pck = 726                                              
    {0x3010, 0x0012 },      //fine_correction = 18                                               
    {0x9803, 0x07	},      //stat_fd_zone_height = 7                                            
    {0xA06E, 0x009B },      //cam_fd_config_fdperiod_50hz = 155               
    {0xA070, 0x0081 },      //cam_fd_config_fdperiod_60hz = 129                          
    {0xA072, 0x11	},      //cam_fd_config_search_f1_50 = 17                                    
    {0xA073, 0x13	},      //cam_fd_config_search_f2_50 = 19                                    
    {0xA074, 0x15   },      //cam_fd_config_search_f1_60 = 21                                  
    {0xA075, 0x17   },      //cam_fd_config_search_f2_60 = 23                                    
    {0xA076, 0x0003 },      //cam_fd_config_max_fdzone_50hz = 
    {0xA078, 0x0004 },      //cam_fd_config_max_fdzone_60hz =                 
    {0xA01A, 0x0003 },      //cam_ae_config_target_fdzone =   
    */
    //TEST
//vince modify for auto-frame rate+++
#ifdef MT9V115_AUTO_FRAME_RATE
	// [Step2-PLL_Timing]2: LOAD=Timing_settings
	// [Timing_settings]1: REG=0x300A, 0x0226                       
	{0x300A, 0x01f9},            // FRAME_LENGTH_LINES
	// [Timing_settings]2: REG=0x300C, 0x02D6                      
	{0x300C, 0x02D6},           // LINE_LENGTH_PCK
	// [Timing_settings]3: REG=0x3010, 0x0012                       
	{0x3010, 0x0012},            // FINE_CORRECTION
	// [Timing_settings]4: REG=0x3040, 0x4041
	{0x3040, 0x0041},            // READ_MODE
	// [Timing_settings]5: REG=0x9803, 0x07                           
	{0x098E, 0x9803},            // LOGICAL_ADDRESS_ACCESS [STAT_FD_ZONE_HEIGHT]
	{0x9803, 0x07 },               // STAT_FD_ZONE_HEIGHT
	// [Timing_settings]6: REG=0xA06E, 0x00A5                      
	{0xA06E, 0x0098},           // CAM_FD_CONFIG_FDPERIOD_50HZ
	// [Timing_settings]7: REG=0xA070, 0x008A                      
	{0xA070, 0x007e },          // CAM_FD_CONFIG_FDPERIOD_60HZ
	// [Timing_settings]8: REG=0xA072, 0x12                           
	{0xA072, 0x11 },              // CAM_FD_CONFIG_SEARCH_F1_50
	// [Timing_settings]9: REG=0xA073, 0x14                           
	{0xA073, 0x13 } ,             // CAM_FD_CONFIG_SEARCH_F2_50
	// [Timing_settings]10: REG=0xA074, 0x16                         
	{0xA074, 0x14 } ,            // CAM_FD_CONFIG_SEARCH_F1_60
	// [Timing_settings]11: REG=0xA075, 0x18                         
	{0xA075, 0x16 } ,              // CAM_FD_CONFIG_SEARCH_F2_60
	{0xA076, 0x000d},           // CAM_FD_CONFIG_MAX_FDZONE_50HZ
	// [Timing_settings]13: REG=0xA078, 0x000A                    
	{0xA078, 0x0010} ,           // CAM_FD_CONFIG_MAX_FDZONE_60HZ
	// [Timing_settings]14: REG=0xA01A, 0x0008                    
	{0xA01A, 0x000d} ,          // CAM_AE_CONFIG_TARGET_FDZONE
#else
    {0x300A, 0x01f9 },      //frame_length_lines = 958 > 516                          
    {0x300C, 0x02D6 },      //line_length_pck = 726                                              
    {0x3010, 0x0012 },      //fine_correction = 18  
    {0x3040, 0x0041 },                                             
    {0x9803, 0x07  	},      //stat_fd_zone_height = 7                                            
    {0xA06E, 0x0098 },      //cam_fd_config_fdperiod_50hz = 155               
    {0xA070, 0x007e },      //cam_fd_config_fdperiod_60hz = 129                          
    {0xA072, 0x11   },      //cam_fd_config_search_f1_50 = 17                                    
    {0xA073, 0x13   },      //cam_fd_config_search_f2_50 = 19                                    
    {0xA074, 0x14   },      //cam_fd_config_search_f1_60 = 21                                  
    {0xA075, 0x16   },      //cam_fd_config_search_f2_60 = 23                                    
    {0xA076, 0x0003 },      //cam_fd_config_max_fdzone_50hz = 
    {0xA078, 0x0004 },      //cam_fd_config_max_fdzone_60hz =                 
    {0xA01A, 0x0003 },      //cam_ae_config_target_fdzone =  
#endif
//vince modify for auto-frame rate---
#else //MCLK=22MHz
    {0x300A, 0x01F9 },      //frame_length_lines = 958 > 505                                     
    {0x300C, 0x02D6 },      //line_length_pck = 726                                              
    {0x3010, 0x0012 },      //fine_correction = 18                                               
    {0x9803, 0x07	},      //stat_fd_zone_height = 7                                            
    {0xA06E, 0x0098 },      //cam_fd_config_fdperiod_50hz = 152                                  
    {0xA070, 0x007E },      //cam_fd_config_fdperiod_60hz = 126                                  
    {0xA072, 0x11	},      //cam_fd_config_search_f1_50 = 17                                    
    {0xA073, 0x13	},      //cam_fd_config_search_f2_50 = 19                                    
    {0xA074, 0x14   },      //cam_fd_config_search_f1_60 = 20                                    
    {0xA075, 0x16   },      //cam_fd_config_search_f2_60 = 22                                    
    {0xA076, 0x0014 },      //cam_fd_config_max_fdzone_50hz = 13(7.5fps) > 33(3fps) > 20(5fps)   
    {0xA078, 0x0018 },      //cam_fd_config_max_fdzone_60hz = 16 > 40 > 24                       
    {0xA01A, 0x0014 },      //cam_ae_config_target_fdzone =   13 > 33 > 20                       
#endif
};

/*
[Step3-Recommended]
LOAD=Char_settings
LOAD=load_and_go patch1
*/
struct reg_addr_val_pair_struct mt9v115_init_recommended_array[] = {
    // [Char_settings]
    {0x3168, 0x84F8},
    {0x316A, 0x028A},   //disable auto load for use with patch, Vrst_LO DAC Vrstlo=12(gain<=2), Vrstlo=8(gain>2) to fix Col FPn issue in low light
    {0x316C, 0xB477},
    {0x316E, 0x828A},   //eclipse setting recommended
    {0x3180, 0x87FF},   //DELTA_DK_CONTROL enabled
//    {0x31E0, 0x0000}, //pix_def_ID keep off
    {0x3E02, 0x0600},
    {0x3E04, 0x221C},
    {0x3E06, 0x3632},
    {0x3E08, 0x3204},
    {0x3E0A, 0x3106},
    {0x3E0C, 0x3025},
    {0x3E0E, 0x190B},
    {0x3E10, 0x0700},
    {0x3E12, 0x24FF},
    {0x3E14, 0x3731},
    {0x3E16, 0x0401},
    {0x3E18, 0x211E},
    {0x3E1A, 0x3633},
    {0x3E1C, 0x3107},
    {0x3E1E, 0x1A16},
    {0x3E20, 0x312D},
    {0x3E22, 0x3303},
    {0x3E24, 0x1401},
    {0x3E26, 0x0600},   //samp spare (Vaapix pulsing ON default) 
    {0x3E30, 0x0037},
    {0x3E32, 0x1638},
    {0x3E90, 0x0E05},
    {0x3E92, 0x1310},
    {0x3E94, 0x0904},
    {0x3E96, 0x0B00},
    {0x3E98, 0x130B},
    {0x3E9A, 0x0C06},
    {0x3E9C, 0x1411},
    {0x3E9E, 0x0E01},
    {0x3ECC, 0x4091},
    {0x3ECE, 0x430D},
    {0x3ED0, 0x1817},   //ecl_en
    {0x3ED2, 0x8504},   //Vaapix_PD=1 to fix col FPn in high light 
    
#if 1
    // FW patch sections.  Loaded from the Char section.
    // LOAD=a391soc_patch1
    // LOAD=patch1 thresholds
    // LOAD=enable patch1
    
    // [a391soc_patch1]
    {0x0982, 0},
    {0x098A, 0},
    
    {0x8251, 0x3C3C},
    {0x8253, 0xBDD1},
    {0x8255, 0xF2D6},
    {0x8257, 0x15C1},
    {0x8259, 0x126},
    {0x825B, 0x3ADC},
    {0x825D, 0xA30},
    {0x825F, 0xED02},
    
    {0x8261, 0xDC08},
    {0x8263, 0xED00},
    {0x8265, 0xFC01},
    {0x8267, 0xFCBD},
    {0x8269, 0xF5FC},
    {0x826B, 0x30EC},
    {0x826D, 0x2FD},
    {0x826F, 0x344},
    
    {0x8271, 0xB303},
    {0x8273, 0x4025},
    {0x8275, 0xDCC},
    {0x8277, 0x3180},
    {0x8279, 0xED00},
    {0x827B, 0xCCA0},
    {0x827D, 0xBD},
    {0x827F, 0xFBFB},
    
    {0x8281, 0x2013},
    {0x8283, 0xFC03},
    {0x8285, 0x44B3},
    {0x8287, 0x342},
    {0x8289, 0x220B},
    {0x828B, 0xCC31},
    {0x828D, 0x80ED},
    {0x828F, 0xCC},
    
    {0x8291, 0xA000},
    {0x8293, 0xBDFC},
    {0x8295, 0x1738},
    {0x8297, 0x3839},
    {0x8299, 0x3CD6},
    {0x829B, 0x15C1},
    {0x829D, 0x126},
    {0x829F, 0x70FC},
    
    {0x82A1, 0x344},
    {0x82A3, 0xB303},
    {0x82A5, 0x4025},
    {0x82A7, 0x13FC},
    {0x82A9, 0x7E26},
    {0x82AB, 0x83FF},
    {0x82AD, 0xFF27},
    {0x82AF, 0xBFC},
    
    {0x82B1, 0x7E26},
    {0x82B3, 0xFD03},
    {0x82B5, 0x4CCC},
    {0x82B7, 0xFFFF},
    {0x82B9, 0x2013},
    {0x82BB, 0xFC03},
    {0x82BD, 0x44B3},
    {0x82BF, 0x342},
    
    {0x82C1, 0x220E},
    {0x82C3, 0xFC7E},
    {0x82C5, 0x2683},
    {0x82C7, 0xFFFF},
    {0x82C9, 0x2606},
    {0x82CB, 0xFC03},
    {0x82CD, 0x4CFD},
    {0x82CF, 0x7E26},
    
    {0x82D1, 0xFC7E},
    {0x82D3, 0x2683},
    {0x82D5, 0xFFFF},
    {0x82D7, 0x2605},
    {0x82D9, 0xFC03},
    {0x82DB, 0x4A20},
    {0x82DD, 0x3FC },
    {0x82DF, 0x348 },
    
    {0x82E1, 0xFD7E},
    {0x82E3, 0xD0FC},
    {0x82E5, 0x7ED2},
    {0x82E7, 0x5F84},
    {0x82E9, 0xF030},
    {0x82EB, 0xED00},
    {0x82ED, 0xDC0A},
    {0x82EF, 0xB303},
    
    {0x82F1, 0x4625},
    {0x82F3, 0x10EC},
    {0x82F5, 0x27  },
    {0x82F7, 0xCFD },
    {0x82F9, 0x34E },
    {0x82FB, 0xFC7E},
    {0x82FD, 0xD284},
    {0x82FF, 0xFED },
    
    {0x8301, 0x20  },
    {0x8303, 0x19DC},
    {0x8305, 0xAB3 },
    {0x8307, 0x346 },
    {0x8309, 0x2415},
    {0x830B, 0xEC00},
    {0x830D, 0x8300},
    {0x830F, 0x26  },
    
    {0x8311, 0xEFC },
    {0x8313, 0x7ED2},
    {0x8315, 0x840F},
    {0x8317, 0xFA03},
    {0x8319, 0x4FBA},
    {0x831B, 0x34E },
    {0x831D, 0xFD7E},
    {0x831F, 0xD2BD},
    
    {0x8321, 0xD2AD},
    {0x8323, 0x3839},
    {0x098E, 0x0}, // back to logical addressing mode
    
    // [patch1 thresholds]
    {0x0982, 0x0},
    {0x098A, 0x0},// physical addressing mode
    {0x8340, 72},
    {0x8342, 64},
    {0x8344, 0},// gain*time thresholds, dark, bright, current, Vtxhi boost gain threshold, DAC_LD_4_5 bright, DAC_LD_4_5 dark
    {0x8346, 64},
    {0x8348, 0x1817},
    {0x834A, 0x1857},   
    {0x098E, 0x0},
    
    // [enable patch1]
    {0x0982, 0x0},
    {0x098A, 0x0},
    {0x824D, 0x0251},
    {0x824F, 0x0299},
    {0x098E, 0x0},
#endif

};
//vince fine-tune+++
struct reg_addr_val_pair_struct mt9v115_init_recommended2_array[] = {
{0x3210, 0x00B0}, 	// COLOR_PIPELINE_CONTROL         
// Lens correction 		                                                                                          
{0x3640, 0x0150},	// P_G1_P0Q0      
{0x3642, 0xB66A},	// P_G1_P0Q1                                    
{0x3644, 0x2990},	// P_G1_P0Q2      	                                                                                                          
{0x3646, 0x880B},	// P_G1_P0Q3      
{0x3648, 0x064E},	// P_G1_P0Q4      
{0x364A, 0x0190},	// P_R_P0Q0       
{0x364C, 0xA20B},	// P_R_P0Q1       
{0x364E, 0x3850},	// P_R_P0Q2       
{0x3650, 0xAD6D},	// P_R_P0Q3       
{0x3652, 0x07EF},	// P_R_P0Q4       
{0x3654, 0x00D0},	// P_B_P0Q0       
{0x3656, 0xE20A},	// P_B_P0Q1       
{0x3658, 0x1CF0},	// P_B_P0Q2       
{0x365A, 0x894D},	// P_B_P0Q3       
{0x365C, 0x31AE},	// P_B_P0Q4       
{0x365E, 0x03D0},	// P_G2_P0Q0      
{0x3660, 0x8FAA},	// P_G2_P0Q1      
{0x3662, 0x3050},	// P_G2_P0Q2      
{0x3664, 0x36E9},	// P_G2_P0Q3      
{0x3666, 0x006E},	// P_G2_P0Q4      
{0x3680, 0xDFAA},	// P_G1_P1Q0      
{0x3682, 0x16EA},	// P_G1_P1Q1      
{0x3684, 0x1CCE},	// P_G1_P1Q2      
{0x3686, 0x510C},	// P_G1_P1Q3      
{0x3688, 0x8A4D},	// P_G1_P1Q4      
{0x368A, 0x970B},	// P_R_P1Q0       
{0x368C, 0x116D},	// P_R_P1Q1       
{0x368E, 0x404E},	// P_R_P1Q2       
{0x3690, 0xB2CD},	// P_R_P1Q3       
{0x3692, 0xDF2E},	// P_R_P1Q4       
{0x3694, 0xE60B},	// P_B_P1Q0       
{0x3696, 0x386C},	// P_B_P1Q1       
{0x3698, 0x6A4D},	// P_B_P1Q2       
{0x369A, 0x820C},	// P_B_P1Q3       
{0x369C, 0xC02E},	// P_B_P1Q4       
{0x369E, 0xB62B},	// P_G2_P1Q0      
{0x36A0, 0x2CAB},	// P_G2_P1Q1      
{0x36A2, 0x23CE},	// P_G2_P1Q2      
{0x36A4, 0xD629},	// P_G2_P1Q3      
{0x36A6, 0xADAD},	// P_G2_P1Q4      
{0x36C0, 0x1A90},	// P_G1_P2Q0      
{0x36C2, 0xC56E},	// P_G1_P2Q1      
{0x36C4, 0x456E},	// P_G1_P2Q2      
{0x36C6, 0x2CEF},	// P_G1_P2Q3      
{0x36C8, 0x8FCD},	// P_G1_P2Q4      
{0x36CA, 0x2890},	// P_R_P2Q0       
{0x36CC, 0xB4CD},	// P_R_P2Q1       
{0x36CE, 0x6D90},	// P_R_P2Q2       
{0x36D0, 0x43B0},	// P_R_P2Q3       
{0x36D2, 0x8692},	// P_R_P2Q4       
{0x36D4, 0x0CD0},	// P_B_P2Q0       
{0x36D6, 0x8CCB},	// P_B_P2Q1       
{0x36D8, 0x23AE},	// P_B_P2Q2       
{0x36DA, 0x01AF},	// P_B_P2Q3       
{0x36DC, 0x8F4F},	// P_B_P2Q4       
{0x36DE, 0x1BD0},	// P_G2_P2Q0      
{0x36E0, 0xE90D},	// P_G2_P2Q1      
{0x36E2, 0xA3E7},	// P_G2_P2Q2      
{0x36E4, 0x4849},	// P_G2_P2Q3      
{0x36E6, 0x65EF},	// P_G2_P2Q4      
{0x3700, 0x6A8E},	// P_G1_P3Q0      
{0x3702, 0x6A0E},	// P_G1_P3Q1      
{0x3704, 0xCA4F},	// P_G1_P3Q2      
{0x3706, 0x8231},	// P_G1_P3Q3      
{0x3708, 0xC5EE},	// P_G1_P3Q4      
{0x370A, 0x278E},	// P_R_P3Q0       
{0x370C, 0x97ED},	// P_R_P3Q1       
{0x370E, 0xA4EF},	// P_R_P3Q2       
{0x3710, 0x9BCE},	// P_R_P3Q3       
{0x3712, 0x14D0},	// P_R_P3Q4       
{0x3714, 0x1BAE},	// P_B_P3Q0       
{0x3716, 0x45AA},	// P_B_P3Q1       
{0x3718, 0xA0CD},	// P_B_P3Q2       
{0x371A, 0x3DEF},	// P_B_P3Q3       
{0x371C, 0xDF2F},	// P_B_P3Q4       
{0x371E, 0x052F},	// P_G2_P3Q0      
{0x3720, 0x3F8E},	// P_G2_P3Q1      
{0x3722, 0xC22F},	// P_G2_P3Q2      
{0x3724, 0xAB10},	// P_G2_P3Q3      
{0x3726, 0x7AC9},	// P_G2_P3Q4      
{0x3740, 0x4A0F},	// P_G1_P4Q0      
{0x3742, 0x1EB0},	// P_G1_P4Q1      
{0x3744, 0x5FAE},	// P_G1_P4Q2      
{0x3746, 0xF691},	// P_G1_P4Q3      
{0x3748, 0xAC12},	// P_G1_P4Q4      
{0x374A, 0x3E0F},	// P_R_P4Q0       
{0x374C, 0x4ACE},	// P_R_P4Q1       
{0x374E, 0x99B2},	// P_R_P4Q2       
{0x3750, 0xDBF2},	// P_R_P4Q3       
{0x3752, 0x6CB3},	// P_R_P4Q4       
{0x3754, 0x408F},	// P_B_P4Q0       
{0x3756, 0x5D4C},	// P_B_P4Q1       
{0x3758, 0x22AF},	// P_B_P4Q2       
{0x375A, 0xE76E},	// P_B_P4Q3       
{0x375C, 0xC051},	// P_B_P4Q4       
{0x375E, 0x434F},	// P_G2_P4Q0      
{0x3760, 0x452F},	// P_G2_P4Q1      
{0x3762, 0x1911},	// P_G2_P4Q2      
{0x3764, 0xB3F0},	// P_G2_P4Q3      
{0x3766, 0x9713},	// P_G2_P4Q4      
                                                                                           
{0x3782, 0x0114},	// CENTER_ROW     
{0x3784, 0x012C},	// CENTER_COLUMN  
 
 
};

struct reg_addr_val_pair_struct mt9v115_init_recommended3_array[] = {
 //Step 5 AWB/CCM
{0x3210, 0x00B8}, 	// COLOR_PIPELINE_CONTROL                                                 
{0x3210, 0x00B8}, 	// COLOR_PIPELINE_CONTROL                                 
{0xA02F, 0x0176}, 	// CAM_AWB_CONFIG_CCM_L_0                                 
{0xA031, 0xFF62}, 	// CAM_AWB_CONFIG_CCM_L_1                                 
{0xA033, 0x0042}, 	// CAM_AWB_CONFIG_CCM_L_2                                 
{0xA035, 0xFF96}, 	// CAM_AWB_CONFIG_CCM_L_3                                 
{0xA037, 0x00FE}, 	// CAM_AWB_CONFIG_CCM_L_4                                 
{0xA039, 0x004F}, 	// CAM_AWB_CONFIG_CCM_L_5                                 
{0xA03B, 0xFFAB}, 	// CAM_AWB_CONFIG_CCM_L_6                                 
{0xA03D, 0xFF07}, 	// CAM_AWB_CONFIG_CCM_L_7                                 
{0xA03F, 0x01F1}, 	// CAM_AWB_CONFIG_CCM_L_8                                 
{0xA041, 0x001D}, 	// CAM_AWB_CONFIG_CCM_L_9                                 
{0xA043, 0x0046}, 	// CAM_AWB_CONFIG_CCM_L_10                                
{0xA045, 0x00C5}, 	// CAM_AWB_CONFIG_CCM_RL_0                                
{0xA047, 0xFFB7}, 	// CAM_AWB_CONFIG_CCM_RL_1                                
{0xA049, 0xFFD8}, 	// CAM_AWB_CONFIG_CCM_RL_2                                
{0xA04B, 0x002F}, 	// CAM_AWB_CONFIG_CCM_RL_3                                
{0xA04D, 0x005D}, 	// CAM_AWB_CONFIG_CCM_RL_4                                
{0xA04F, 0xFF9E}, 	// CAM_AWB_CONFIG_CCM_RL_5                                
{0xA051, 0x0058}, 	// CAM_AWB_CONFIG_CCM_RL_6                                
{0xA053, 0x006B}, 	// CAM_AWB_CONFIG_CCM_RL_7                                
{0xA055, 0xFFEE}, 	// CAM_AWB_CONFIG_CCM_RL_8                                
{0xA057, 0x0010}, 	// CAM_AWB_CONFIG_CCM_RL_9                                
{0xA059, 0xFFE0}, 	// CAM_AWB_CONFIG_CCM_RL_10                               
{0x2112, 0x051E}, 	// AWB_WEIGHT_R0                                          
{0x2114, 0x0000}, 	// AWB_WEIGHT_R1                                          
{0x2116, 0xAE40}, 	// AWB_WEIGHT_R2                                          
{0x2118, 0x7AE1}, 	// AWB_WEIGHT_R3                                          
{0x211A, 0x1996}, 	// AWB_WEIGHT_R4                                          
{0x211C, 0x0F5A}, 	// AWB_WEIGHT_R5                                          
{0x211E, 0xFDD8}, 	// AWB_WEIGHT_R6                                          
{0x2120, 0x5000}, 	// AWB_WEIGHT_R7                                          
{0xA065, 0x03}, 	// CAM_AWB_CONFIG_X_SCALE                                 
                                                                                                                                    
{0xA061, 0x0039}, 	//  cam_awb_config_x_shift_pre_adj  for AWB coolwhite tune
{0xA063, 0x0032}, 	//  cam_awb_config_Y_shift_pre_adj  forAWB coolwhite tune 
{0x9418, 0x2E}, 	// AWB_B_SCENE_RATIO_LOWER     
     
};


struct reg_addr_val_pair_struct mt9v115_init_recommended4_array[] = {
 //Step 6 AE, Denoise Gamma,FD 
{0x326E, 0x0006}, 	// LOW_PASS_YUV_FILTER                                                
{0x33F4, 0x000B}, 	// RESERVED_SOC1_33F4                                                                                                                                                        
{0xA07A, 0x10}, 	// CAM_LL_CONFIG_AP_THRESH_START                   
{0xA07C, 0x04}, 	// CAM_LL_CONFIG_AP_GAIN_START                     
{0xA081, 0x1E}, 	// CAM_LL_CONFIG_DM_EDGE_TH_START                  
{0xA082, 0x50}, 	// CAM_LL_CONFIG_DM_EDGE_TH_STOP                   
{0xA0B1, 0x10}, 	// CAM_LL_CONFIG_NR_RED_START                      
{0xA0B2, 0x2D}, 	// CAM_LL_CONFIG_NR_RED_STOP                       
{0xA0B3, 0x10}, 	// CAM_LL_CONFIG_NR_GREEN_START                    
{0xA0B4, 0x2D}, 	// CAM_LL_CONFIG_NR_GREEN_STOP                     
{0xA0B5, 0x10}, 	// CAM_LL_CONFIG_NR_BLUE_START                     
{0xA0B6, 0x2D}, 	// CAM_LL_CONFIG_NR_BLUE_STOP                      
{0xA0B7, 0x10}, 	// CAM_LL_CONFIG_NR_MIN_MAX_START                  
{0xA0B8, 0x2D}, 	// CAM_LL_CONFIG_NR_MIN_MAX_STOP                   
{0xA05F, 0x80}, 	// CAM_AWB_CONFIG_START_SATURATION                 
{0xA060, 0x05}, 	// CAM_AWB_CONFIG_END_SATURATION                   
{0xA0B9, 0x0026}, 	// CAM_LL_CONFIG_START_GAIN_METRIC                 
{0xA0BB, 0x00B4}, 	// CAM_LL_CONFIG_STOP_GAIN_METRIC                  
{0xA07E, 0x001E}, 	// CAM_LL_CONFIG_CDC_THRESHOLD_BM                  
 	                                                                                                                                   
{0x9C00, 0xBF}, 	// LL_MODE                                         
{0xA085, 0x0078}, 	// CAM_LL_CONFIG_FTB_AVG_YSUM_STOP                 
 		                                                                                                                           
{0xA087, 0x00}, 	// CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_0            
 	                                                                                                                                   
{0xA088, 0x07}, 	// CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_1            
{0xA089, 0x16}, 	// CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_2            
{0xA08A, 0x30}, 	// CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_3            
{0xA08B, 0x52}, 	// CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_4            
{0xA08C, 0x6D}, 	// CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_5            
{0xA08D, 0x86}, 	// CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_6            
{0xA08E, 0x9B}, 	// CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_7            
{0xA08F, 0xAB}, 	// CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_8            
{0xA090, 0xB9}, 	// CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_9            
{0xA091, 0xC5}, 	// CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_10           
{0xA092, 0xCF}, 	// CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_11           
{0xA093, 0xD8}, 	// CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_12           
{0xA094, 0xE0}, 	// CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_13           
{0xA095, 0xE7}, 	// CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_14           
{0xA096, 0xEE}, 	// CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_15           
{0xA097, 0xF4}, 	// CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_16           
{0xA098, 0xFA}, 	// CAM_LL_CONFIG_GAMMA_CONTRAST_CURVE_17           
{0xA0AD, 0x0005}, 	// CAM_LL_CONFIG_GAMMA_START_BM                    
{0xA0AF, 0x0021}, 	// CAM_LL_CONFIG_GAMMA_STOP_BM                     
{0xA020, 0x3C}, 	// CAM_AE_CONFIG_BASE_TARGET                       
{0xA027, 0x0040}, 	// CAM_AE_CONFIG_MIN_VIRT_AGAIN                    
{0xA029, 0x0080}, 	// CAM_AE_CONFIG_MAX_VIRT_AGAIN                    
{0xA01C, 0x0060}, 	// CAM_AE_CONFIG_TARGET_AGAIN                      
{0xA023, 0x0040}, 	// CAM_AE_CONFIG_MIN_VIRT_DGAIN                    
{0xA025, 0x0080}, 	// CAM_AE_CONFIG_MAX_VIRT_DGAIN                    
{0xA01E, 0x0060}, 	// CAM_AE_CONFIG_TARGET_DGAIN                      
                                                                                                                            
                                                                                                                
{0x098E, 0x9008},	// LOGICAL_ADDRESS_ACCESS [AE_TRACK_WEIGHT_TABLE_0]
{0x9007, 0x0a},                                                          
{0x9008, 0x19}, 	// AE_TRACK_WEIGHT_TABLE_0                         
{0x9009, 0x19}, 	// AE_TRACK_WEIGHT_TABLE_1                         
{0x900A, 0x19}, 	// AE_TRACK_WEIGHT_TABLE_2                         
{0x900B, 0x19}, 	// AE_TRACK_WEIGHT_TABLE_3                         
{0x900C, 0x19}, 	// AE_TRACK_WEIGHT_TABLE_4                         
{0x900D, 0x4B}, 	// AE_TRACK_WEIGHT_TABLE_5                         
{0x900E, 0x4B}, 	// AE_TRACK_WEIGHT_TABLE_6                         
{0x900F, 0x19}, 	// AE_TRACK_WEIGHT_TABLE_7                         
{0x9010, 0x19}, 	// AE_TRACK_WEIGHT_TABLE_8                         
{0x9011, 0x4B}, 	// AE_TRACK_WEIGHT_TABLE_9                         
{0x9012, 0x4B}, 	// AE_TRACK_WEIGHT_TABLE_10                        
{0x9013, 0x19}, 	// AE_TRACK_WEIGHT_TABLE_11                        
{0x9014, 0x19}, 	// AE_TRACK_WEIGHT_TABLE_12                        
{0x9015, 0x19}, 	// AE_TRACK_WEIGHT_TABLE_13                        
{0x9016, 0x19}, 	// AE_TRACK_WEIGHT_TABLE_14                        
{0x9017, 0x19}, 	// AE_TRACK_WEIGHT_TABLE_15                        
                                                                                                        
                                                                                                                                           
{0x8C03, 0x01}, 	// FD_STAT_MIN                                     
{0x8C04, 0x03}, 	// FD_STAT_MAX                                     
{0x8C05, 0x05}, 	// FD_MIN_AMPLITUDE  
 
 
};

//vince fine-tune--
#if 0
/*  96MHz PCLK @ 24MHz MCLK */
struct reg_addr_val_pair_struct mt9v115_init_settings_array[] = {
    // Register Wizard output for PLL and Timing
    // [PLL_settings]
//    {0x0010, 0x0110},   //pll_dividers = 272
//    {0x0012, 0x0600},   //pll_p_dividers = 1536
//    {0x0018, 0x0006},   //out of standby, start PLL, power up stop
//    {0xFFFF, 0x0010},   //delay 10 msec
    
    {0x0014, 0x2047},   //bypass PLL
    {0x0012, 0x0200},   //pll_p_dividers = 512
    {0x0014, 0x2046},   //use PLL output (bypass off)
    {0xFFFF, 0x0010},   //wait for lock (could poll for lock instead of wait)
    
//    {0x001E, 0x0107},   //This value must be tuned to suit the electrical characteristics of the target system 
    
    // [Timing_settings]
    {0x300A, 0x0204 }, //{0x300A, 0x03BE},   //frame_length_lines = 958
    {0x300C, 0x02D6 }, //{0x300C, 0x02EE},   //line_length_pck = 750 to fix lsb column issue
    {0x3010, 0x0012 }, //{0x3010, 0x0012},   //fine_correction = 18
    {0x9803, 0x07	}, //{0x9803, 0x07},     //stat_fd_zone_height = 7
    {0xA06E, 0x009B }, //{0xA06E, 0x0098},   //cam_fd_config_fdperiod_50hz = 152
    {0xA070, 0x0081 }, //{0xA070, 0x007E},   //cam_fd_config_fdperiod_60hz = 126
    {0xA072, 0x11	}, //{0xA072, 0x11},     //cam_fd_config_search_f1_50 = 17
    {0xA073, 0x13	}, //{0xA073, 0x13},     //cam_fd_config_search_f2_50 = 19
    {0xA074, 0x15   }, //{0xA074, 0x14},     //cam_fd_config_search_f1_60 = 20
    {0xA075, 0x17   }, //{0xA075, 0x16},     //cam_fd_config_search_f2_60 = 22
    {0xA076, 0x0003 }, //{0xA076, 0x000D},   //cam_fd_config_max_fdzone_50hz = 13
    {0xA078, 0x0004 }, //{0xA078, 0x0010},   //cam_fd_config_max_fdzone_60hz = 16
    {0xA01A, 0x0003 }, //{0xA01A, 0x000D},   //cam_ae_config_target_fdzone = 13
    
    // [Char_settings]
    {0x3168, 0x84F8},
    {0x316A, 0x828C},   //Vrst_LO DAC Vrstlo=12(gain<=2), Vrstlo=8(gain>2) to fix Col FPn issue in low light
    {0x316C, 0xB477},
    {0x316E, 0x828A},   //eclipse setting recommended
    {0x3180, 0x87FF},   //DELTA_DK_CONTROL enabled
//    {0x31E0, 0x0000}, //pix_def_ID keep off
    {0x3E02, 0x0600},
    {0x3E04, 0x221C},
    {0x3E06, 0x3632},
    {0x3E08, 0x3204},
    {0x3E0A, 0x3106},
    {0x3E0C, 0x3025},
    {0x3E0E, 0x190B},
    {0x3E10, 0x0700},
    {0x3E12, 0x24FF},
    {0x3E14, 0x3731},
    {0x3E16, 0x0401},
    {0x3E18, 0x211E},
    {0x3E1A, 0x3633},
    {0x3E1C, 0x3107},
    {0x3E1E, 0x1A16},
    {0x3E20, 0x312D},
    {0x3E22, 0x3303},
    {0x3E24, 0x1401},
    {0x3E26, 0x0600},   //samp spare (Vaapix pulsing ON default) 
    {0x3E30, 0x0037},
    {0x3E32, 0x1638},
    {0x3E90, 0x0E05},
    {0x3E92, 0x1310},
    {0x3E94, 0x0904},
    {0x3E96, 0x0B00},
    {0x3E98, 0x130B},
    {0x3E9A, 0x0C06},
    {0x3E9C, 0x1411},
    {0x3E9E, 0x0E01},
    {0x3ECC, 0x4091},
    {0x3ECE, 0x430D},
    {0x3ED0, 0x1817},   //ecl_en
    {0x3ED2, 0x8504},   //Vaapix_PD=1
    
    {0xA027, 0x0030},   //FIELD_WR= CAM_AE_CONFIG_MIN_VIRT_AGAIN, 0x0030

#if 1
    // FW patch sections.  Loaded from the Char section.
    // LOAD=a391soc_patch1
    // LOAD=patch1 thresholds
    // LOAD=enable patch1
    
    // [a391soc_patch1]
    {0x0982, 0},
    {0x098A, 0},
    
    {0x8251, 0x3C3C},
    {0x8253, 0xBDD1},
    {0x8255, 0xF2D6},
    {0x8257, 0x15C1},
    {0x8259, 0x126},
    {0x825B, 0x3ADC},
    {0x825D, 0xA30},
    {0x825F, 0xED02},
    
    {0x8261, 0xDC08},
    {0x8263, 0xED00},
    {0x8265, 0xFC01},
    {0x8267, 0xFCBD},
    {0x8269, 0xF5FC},
    {0x826B, 0x30EC},
    {0x826D, 0x2FD},
    {0x826F, 0x348},
    
    {0x8271, 0xB303},
    {0x8273, 0x4425},
    {0x8275, 0xDCC},
    {0x8277, 0x3180},
    {0x8279, 0xED00},
    {0x827B, 0xCCA0},
    {0x827D, 0xBD},
    {0x827F, 0xFBFB},
    
    {0x8281, 0x2013},
    {0x8283, 0xFC03},
    {0x8285, 0x48B3},
    {0x8287, 0x346},
    {0x8289, 0x220B},
    {0x828B, 0xCC31},
    {0x828D, 0x80ED},
    {0x828F, 0xCC},
    
    {0x8291, 0xA000},
    {0x8293, 0xBDFC},
    {0x8295, 0x1738},
    {0x8297, 0x3839},
    {0x8299, 0x3CD6},
    {0x829B, 0x15C1},
    {0x829D, 0x126},
    {0x829F, 0x6DFC},
    
    {0x82A1, 0x348},
    {0x82A3, 0xB303},
    {0x82A5, 0x4425},
    {0x82A7, 0x13FC},
    {0x82A9, 0x7E26},
    {0x82AB, 0x83FF},
    {0x82AD, 0xFF27},
    {0x82AF, 0xBFC},
    
    {0x82B1, 0x7E26},
    {0x82B3, 0xFD03},
    {0x82B5, 0x4CCC},
    {0x82B7, 0xFFFF},
    {0x82B9, 0x2013},
    {0x82BB, 0xFC03},
    {0x82BD, 0x48B3},
    {0x82BF, 0x346},
    
    {0x82C1, 0x220E},
    {0x82C3, 0xFC7E},
    {0x82C5, 0x2683},
    {0x82C7, 0xFFFF},
    {0x82C9, 0x2606},
    {0x82CB, 0xFC03},
    {0x82CD, 0x4CFD},
    {0x82CF, 0x7E26},
    
    {0x82D1, 0xFC7E},
    {0x82D3, 0xD25F},
    {0x82D5, 0x84F0},
    {0x82D7, 0x30ED},
    {0x82D9, 0xDC},
    {0x82DB, 0xAB3},
    {0x82DD, 0x34A},
    {0x82DF, 0x2510},
    
    {0x82E1, 0xEC00},
    {0x82E3, 0x270C},
    {0x82E5, 0xFD03},
    {0x82E7, 0x4EFC},
    {0x82E9, 0x7ED2},
    {0x82EB, 0x840F},
    {0x82ED, 0xED00},
    {0x82EF, 0x2019},
    
    {0x82F1, 0xDC0A},
    {0x82F3, 0xB303},
    {0x82F5, 0x4A24},
    {0x82F7, 0x15EC},
    {0x82F9, 0x83},
    {0x82FB, 0x0},
    {0x82FD, 0x260E},
    {0x82FF, 0xFC7E},
    
    {0x8301, 0xD284},
    {0x8303, 0xFFA},
    {0x8305, 0x34F},
    {0x8307, 0xBA03},
    {0x8309, 0x4EFD},
    {0x830B, 0x7ED2},
    {0x830D, 0xBDD2},
    {0x830F, 0xAD38},
    
    {0x8311, 0x3900},
    {0x098E, 0x0}, // back to logical addressing mode
    
    // [patch1 thresholds]
    {0x0982, 0x0},
    {0x098A, 0x0},// physical addressing mode
    {0x8344, 72},// gain*time thresholds, dark, bright, current, Vtxhi boost gain threshold
    {0x8346, 64},
    {0x8348, 0},
    {0x834A, 64},   
    {0x098E, 0x0},
    
    // [enable patch1]
    {0x0982, 0x0},
    {0x098A, 0x0},
    {0x824D, 0x0251},
    {0x824F, 0x0299},
    {0x098E, 0x0},
#endif

};

#endif

static bool MT9V115_CSI_CONFIG;
/* 816x612, 24MHz MCLK 96MHz PCLK */
uint32_t MT9V115_FULL_SIZE_WIDTH        = 640;
uint32_t MT9V115_FULL_SIZE_HEIGHT       = 480;

uint32_t MT9V115_QTR_SIZE_WIDTH         = 640;
uint32_t MT9V115_QTR_SIZE_HEIGHT        = 480;

uint32_t MT9V115_HRZ_FULL_BLK_PIXELS    = 16;
uint32_t MT9V115_VER_FULL_BLK_LINES     = 12;
uint32_t MT9V115_HRZ_QTR_BLK_PIXELS     = 16;
uint32_t MT9V115_VER_QTR_BLK_LINES      = 12;

struct mt9v115_work_t {
	struct work_struct work;
};
static struct  mt9v115_work_t *mt9v115_sensorw;
static struct  i2c_client *mt9v115_client;
struct mt9v115_ctrl_t {
	const struct  msm_camera_sensor_info *sensordata;
	uint32_t sensormode;
	uint32_t fps_divider;		/* init to 1 * 0x00000400 */
	uint32_t pict_fps_divider;	/* init to 1 * 0x00000400 */
	uint32_t fps;
	int32_t  curr_lens_pos;
	uint32_t curr_step_pos;
	uint32_t my_reg_gain;
	uint32_t my_reg_line_count;
	uint32_t total_lines_per_frame;
	enum mt9v115_resolution_t prev_res;
	enum mt9v115_resolution_t pict_res;
	enum mt9v115_resolution_t curr_res;
	enum mt9v115_test_mode_t  set_test;
	unsigned short imgaddr;
};
static struct mt9v115_ctrl_t *mt9v115_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(mt9v115_wait_queue);
DEFINE_MUTEX(mt9v115_mut);

/*=============================================================*/

static int mt9v115_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
		{
			.addr  = saddr,
			.flags = 0,
			.len   = 2,
			.buf   = rxdata,
		},
		{
			.addr  = saddr,
			.flags = I2C_M_RD,
			.len   = length,
			.buf   = rxdata,
		},
	};
	if (i2c_transfer(mt9v115_client->adapter, msgs, 2) < 0) {
		CDBG("mt9v115_i2c_rxdata failed!\n");
		return -EIO;
	}
	return 0;
}
static int32_t mt9v115_i2c_txdata(unsigned short saddr,
				unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		 },
	};
	if (i2c_transfer(mt9v115_client->adapter, msg, 1) < 0) {
		CDBG("mt9v115_i2c_txdata faild 0x%x\n", mt9v115_client->addr);
		return -EIO;
	}

	return 0;
}

static int32_t mt9v115_i2c_read(unsigned short raddr,
	unsigned short *rdata, int rlen)
{
	int32_t rc = 0;
	unsigned char buf[2];
	if (!rdata)
		return -EIO;
	memset(buf, 0, sizeof(buf));
	
	buf[0] = (raddr & MT9V115_MSB_MASK) >> 8;
	buf[1] = (raddr & MT9V115_LSB_MASK);
	
	rc = mt9v115_i2c_rxdata(mt9v115_client->addr, buf, rlen);
	if (rc < 0) {
		CDBG("%s: 0x%x failed!\n", __func__, raddr);
		return rc;
	}
	
	*rdata = (rlen == 2 ? buf[0] << 8 | buf[1] : buf[0]);
	return rc;
}

static int32_t mt9v115_i2c_write_b_sensor(uint16_t waddr, uint8_t bdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[3];
	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = bdata;
	CDBG("%s addr = 0x%x, val = 0x%x\n", __func__, waddr, bdata);
	rc = mt9v115_i2c_txdata(mt9v115_client->addr, buf, 3);
	if (rc < 0)
		CDBG("%s: failed, addr = 0x%x, val = 0x%x!\n", __func__, 
			waddr, bdata);
	return rc;
}

static int32_t mt9v115_i2c_write_w_sensor(uint16_t waddr, uint16_t wdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[4];
	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = (wdata & 0xFF00) >> 8;
	buf[3] = (wdata & 0x00FF);
	CDBG("%s: addr = 0x%x, val = 0x%x\n", __func__, waddr, wdata);
	rc = mt9v115_i2c_txdata(mt9v115_client->addr, buf, 4);
	if (rc < 0)
		CDBG("%s: failed, addr = 0x%x, val = 0x%x!\n", __func__,
			waddr, wdata);
	return rc;
}

static int32_t mt9v115_register_dump(void)
{
    int32_t rc = 0;
    //int32_t i = 0;
    uint16_t reg_addr, reg_data;
    
    //
    CDBG("Dump Core Registers:\n");
    for (reg_addr=0x3002; reg_addr<=0x300C; reg_addr+=2) {
        mt9v115_i2c_read(reg_addr, &reg_data, 2);
        CDBG("  Reg(0x%x) = 0x%x\n", reg_addr, reg_data);
    }
    for (reg_addr=0x3010; reg_addr<=0x3014; reg_addr+=2) {
        mt9v115_i2c_read(reg_addr, &reg_data, 2);
        CDBG("  Reg(0x%x) = 0x%x\n", reg_addr, reg_data);
    }
    reg_addr = 0x301E;
    mt9v115_i2c_read(reg_addr, &reg_data, 2);
    CDBG("  Reg(0x%x) = 0x%x\n", reg_addr, reg_data);
    for (reg_addr=0x3028; reg_addr<=0x3030; reg_addr+=2) {
        mt9v115_i2c_read(reg_addr, &reg_data, 2);
        CDBG("  Reg(0x%x) = 0x%x\n", reg_addr, reg_data);
    }
    reg_addr = 0x3040;
    mt9v115_i2c_read(reg_addr, &reg_data, 2);
    CDBG("  Reg(0x%x) = 0x%x\n", reg_addr, reg_data);
    for (reg_addr=0x3056; reg_addr<=0x305C; reg_addr+=2) {
        mt9v115_i2c_read(reg_addr, &reg_data, 2);
        CDBG("  Reg(0x%x) = 0x%x\n", reg_addr, reg_data);
    }
    reg_addr = 0x31FE;
    mt9v115_i2c_read(reg_addr, &reg_data, 2);
    CDBG("  Reg(0x%x) = 0x%x\n", reg_addr, reg_data);
    reg_addr = 0x3ECE;
    mt9v115_i2c_read(reg_addr, &reg_data, 2);
    CDBG("  Reg(0x%x) = 0x%x\n", reg_addr, reg_data);
    reg_addr = 0x3ED0;
    mt9v115_i2c_read(reg_addr, &reg_data, 2);
    CDBG("  Reg(0x%x) = 0x%x\n", reg_addr, reg_data);
    
    
    //
    CDBG("Dump SYSCTL Registers:\n");
    reg_addr = 0x0000;
    mt9v115_i2c_read(reg_addr, &reg_data, 2);
    CDBG("  Reg(0x%x) = 0x%x\n", reg_addr, reg_data);
    for (reg_addr=0x0010; reg_addr<=0x001E; reg_addr+=2) {
        mt9v115_i2c_read(reg_addr, &reg_data, 2);
        CDBG("  Reg(0x%x) = 0x%x\n", reg_addr, reg_data);
    }
    reg_addr = 0x0026;
    mt9v115_i2c_read(reg_addr, &reg_data, 2);
    CDBG("  Reg(0x%x) = 0x%x\n", reg_addr, reg_data);
    reg_addr = 0x002E;
    mt9v115_i2c_read(reg_addr, &reg_data, 2);
    CDBG("  Reg(0x%x) = 0x%x\n", reg_addr, reg_data);
    reg_addr = 0x0042;
    mt9v115_i2c_read(reg_addr, &reg_data, 2);
    CDBG("  Reg(0x%x) = 0x%x\n", reg_addr, reg_data);
    
    //
    CDBG("Dump TX_SS Registers:\n");
    reg_addr = 0x3C00;
    mt9v115_i2c_read(reg_addr, &reg_data, 2);
    CDBG("  Reg(0x%x) = 0x%x\n", reg_addr, reg_data);
    for (reg_addr=0x3C38; reg_addr<=0x3C5A; reg_addr+=2) {
        mt9v115_i2c_read(reg_addr, &reg_data, 2);
        CDBG("  Reg(0x%x) = 0x%x\n", reg_addr, reg_data);
    }
    
    return rc;
}
static int mt9v115_effect_mode = -1;
//vince, special effect setting++++
static int mt9v115_set_effect(int mode, int effect)
{
	long rc = 0;	
	printk("[cam_setting] effect = %d. mt9v115_effect_mode = %d.\n", effect, mt9v115_effect_mode);
	

    if(mt9v115_effect_mode==effect)
    	printk("mt9v115_effect_mode==effect \n") ;
//        return 0;

	mt9v115_effect_mode=effect; 

	switch (mode) 
	{
		case SENSOR_PREVIEW_MODE:
			/* Context A Special Effects */
			break;

		case SENSOR_SNAPSHOT_MODE:
			/* Context B Special Effects */
			break;

		default:
			break;
	}

	switch (effect) 
	{
		case CAMERA_EFFECT_OFF: 
		{
			printk("mt9v115: set CAMERA_EFFECT_OFF\n");
			rc = mt9v115_i2c_write_w_sensor(0x098E, 0x8400);
			rc = mt9v115_i2c_write_b_sensor(0x8400, 0x01);
			msleep(200);
			rc = mt9v115_i2c_write_b_sensor(0xA010, 0x00);
			rc = mt9v115_i2c_write_b_sensor(0x8400, 0x02);
			if (rc < 0)
				return rc;		
		}
		break;

		case CAMERA_EFFECT_MONO: 
		{
			printk("mt9v115: set CAMERA_EFFECT_MONO\n");
			rc = mt9v115_i2c_write_w_sensor(0x098E, 0x8400);
			rc = mt9v115_i2c_write_b_sensor(0x8400, 0x01);
			msleep(200);
			rc = mt9v115_i2c_write_b_sensor(0xA010, 0x01);
			rc = mt9v115_i2c_write_b_sensor(0x8400, 0x02);
			if (rc < 0)
				return rc;		
		}
		break;

		case CAMERA_EFFECT_SEPIA: 
		{
			printk("mt9v115: set CAMERA_EFFECT_SEPIA\n");
			rc = mt9v115_i2c_write_w_sensor(0x098E, 0x8400);
			rc = mt9v115_i2c_write_b_sensor(0x8400, 0x01);
			msleep(200);
			rc = mt9v115_i2c_write_b_sensor(0xA010, 0x02);
			rc = mt9v115_i2c_write_b_sensor(0x8400, 0x02);
			if (rc < 0)
				return rc;		
		}
		break;

		case CAMERA_EFFECT_NEGATIVE: 
		{
			printk("mt9v115: set CAMERA_EFFECT_NEGATIVE\n");
			rc = mt9v115_i2c_write_w_sensor(0x098E, 0x8400);
			rc = mt9v115_i2c_write_b_sensor(0x8400, 0x01);
			msleep(200);
			rc = mt9v115_i2c_write_b_sensor(0xA010, 0x03);
			rc = mt9v115_i2c_write_b_sensor(0x8400, 0x02);
			if (rc < 0)
				return rc;		
		}
		break;

		case CAMERA_EFFECT_SOLARIZE: 
		{
			printk("mt9v115: set CAMERA_EFFECT_SOLARIZE\n");
			rc = mt9v115_i2c_write_w_sensor(0x098E, 0x8400);
			rc = mt9v115_i2c_write_b_sensor(0x8400, 0x01);
			msleep(200);
			rc = mt9v115_i2c_write_b_sensor(0xA010, 0x04);
			rc = mt9v115_i2c_write_b_sensor(0x8400, 0x02);
			if (rc < 0)
				return rc;		
		}
		break;

		default: 
			return -EINVAL;
	}
	
	return rc;
}
//vince, special effect setting---

static int32_t mt9v115_sensor_setting(int update_type, int rt)
{
	int32_t i, array_length;
	int16_t R0x001A ;
	int32_t rc = 0;
	//uint16_t read_data;
	//uint8_t read_byte;
	
	struct msm_camera_csi_params mt9v115_csi_params;
	switch (update_type) {
	case REG_INIT:
		MT9V115_CSI_CONFIG = 0;
//		mt9v115_i2c_write_b_sensor(0x0e, 0x08);
/*
        mt9v115_i2c_write_w_sensor(0x0010, 0x0110);
        mt9v115_i2c_write_w_sensor(0x0012, 0x0600);
        mt9v115_i2c_write_w_sensor(0x0018, 0x0006);
        
        for (i=0; i<10; i++) {
            mt9v115_i2c_read(0x0018, &read_data, 2);
            CDBG("mt9v115 read_data(0x0018) = 0x%x\n", read_data);
            if (read_data & 0x4000) {
                CDBG("  FW_IN_STANDBY!!\n");
                break;
            }
            msleep(1);
        }
*/
		return rc;
		break;
	case UPDATE_PERIODIC:
		if (!MT9V115_CSI_CONFIG) {
			mt9v115_csi_params.lane_cnt = 1;
			mt9v115_csi_params.data_format = CSI_8BIT;
			mt9v115_csi_params.lane_assign = 0xe4;
			mt9v115_csi_params.dpcm_scheme = 0;
			mt9v115_csi_params.settle_cnt = 0x14;

			rc = msm_camio_csi_config(&mt9v115_csi_params);
			msleep(10);
#if 1
            
/*
            // [Step1-Reset]
            // REG= 0x001A, 0x0106 	// RESET_AND_MISC_CONTROL  // FIELD_WR= RESET_AND_MISC_CONTROL, RESET_PSEUDO_PIN, 1		
            // this bit is self-clearing, it is not necessary to re-write a zero
            mt9v115_i2c_write_w_sensor(0x001A, 0x0106);
            mdelay(20); //TEST
            // REG= 0x0042, 0xFFF3 	// COMMAND_RW   // FIELD_WR= COMMAND_RW, 0xFFF3 	// otpm_io_mode_override_{disable,select} = 0
            // REG= 0x3C00, 0x5004 	// TX_CONTROL  // FIELD_WR= TX_CONTROL, SEL_OUT_SOURCE, 4 	// sel_out_source = 8-bit Color pipe Mipi
            mt9v115_i2c_write_w_sensor(0x0042, 0xFFF3);
            mt9v115_i2c_write_w_sensor(0x3C00, 0x5004); // 4: Color-pipe MIPI (8-bit)
            
            // [Step2-PLL_Timing]
            // LOAD=PLL_settings
            // LOAD=Timing_settings

            // [PLL_settings]
            //22MHz
            // FIELD_WR= PLL_DIVIDERS, 0x0110 	// REG= 0x0010, 0x0110
            // FIELD_WR= PLL_P_DIVIDERS, 0x0200 	// REG= 0x0012, 0x0200
            //24MHz
            //mt9v115_i2c_write_w_sensor(0x0010, 0x031E); //pll_dividers = 798
            //mt9v115_i2c_write_w_sensor(0x0012, 0x0200); //pll_p_dividers = 512
            //TEST
*/
//vince modify for auto-frame rate+++
#ifdef MT9V115_AUTO_FRAME_RATE
/*
	    printk("MT9V115: auto-frame rate setting\n");
            mt9v115_i2c_write_w_sensor(0x0010, 0x0110); //pll_dividers = 272
            mt9v115_i2c_write_w_sensor(0x0012, 0x0200); //pll_p_dividers = 512
            
            // DO NOT REMOVE
            // FIELD_WR= STANDBY_CONTROL_AND_STATUS, 0x4503 	// REG= 0x0018, 0x4503
            mt9v115_i2c_write_w_sensor(0x0018, 0x4503);
            mdelay(20); //TEST
            // FIELD_WR= STANDBY_CONTROL_AND_STATUS, 0x4502 	// REG= 0x0018, 0x4502
            mt9v115_i2c_write_w_sensor(0x0018, 0x4502);
            mdelay(20); //TEST
            // FIELD_WR= PAD_SLEW, 0x0100 	// REG= 0x001E, 0x0100; IO Slew set to 0 to fix Column band on left of image
            mt9v115_i2c_write_w_sensor(0x001E, 0x0100);
            // FIELD_WR= OUTPUT_FORMAT_CONFIGURATION, 0x0040 	// REG= 0x332E, 0x0040
            mt9v115_i2c_write_w_sensor(0x332E, 0x0040);
            // FIELD_WR= OUTPUT_FORMAT_CONFIGURATION, 0x0000 	// REG= 0x332E, 0x0000
            mt9v115_i2c_write_w_sensor(0x332E, 0x0002);     // swaps chroma with luma
            // FIELD_WR= RESET_AND_MISC_CONTROL, 0x0300 	// REG= 0x001A, 0x0300
            mt9v115_i2c_write_w_sensor(0x001A, 0x0300);
            mdelay(20); //TEST
            // FIELD_WR= RESET_AND_MISC_CONTROL, 0x0344 	// REG= 0x001A, 0x0344
            mt9v115_i2c_write_w_sensor(0x001A, 0x0344);
            mdelay(20); //TEST
*/
            mt9v115_i2c_read(0x001A, &R0x001A, 2);
            CDBG("mt9v115 read_data(0x001A) = 0x%x\n", R0x001A);
            mt9v115_i2c_write_w_sensor(0x001A, (R0x001A|0x0106));
// [PLL_settings]2: FIELD_WR=RESET_AND_MISC_CONTROL, RESET_PSEUDO_PIN, 1
	    mt9v115_i2c_write_w_sensor(0x001a,0x0106);//vince add, 10/18
	    
            mt9v115_i2c_write_w_sensor(0x001a,0x0566);
	    mdelay(50);
	    mt9v115_i2c_write_w_sensor(0x0010,0x052c);
	    mt9v115_i2c_write_w_sensor(0x0012,0x0600);
	    mt9v115_i2c_write_w_sensor(0x0018,0x4506);  
	    mdelay(50);
	    mt9v115_i2c_write_w_sensor(0x0018,0x4500);
	    mt9v115_i2c_write_w_sensor(0x0042,0xfff3);
	  
	    mt9v115_i2c_write_w_sensor(0x3C00,0x5004);  // TX_CONTROL, vince add, 10/18
	 // mt9v115_i2c_write_w_sensor(0x0018,0x0006);
	    mt9v115_i2c_write_w_sensor(0x001a,0x0520);
	    mt9v115_i2c_write_w_sensor(0x001a,0x0564);
	    mdelay(200);
	    mt9v115_i2c_write_w_sensor(0x0012,0x0200);
	    mt9v115_i2c_write_w_sensor(0x332e,0x0040);
	    mt9v115_i2c_write_w_sensor(0x332e,0x0002);
            
      	    
#else
            mt9v115_i2c_write_w_sensor(0x0010, 0x0110); //pll_dividers = 272
            mt9v115_i2c_write_w_sensor(0x0012, 0x0200); //pll_p_dividers = 512
            
            // DO NOT REMOVE
            // FIELD_WR= STANDBY_CONTROL_AND_STATUS, 0x4503 	// REG= 0x0018, 0x4503
            mt9v115_i2c_write_w_sensor(0x0018, 0x4503);
            mdelay(20); //TEST
            // FIELD_WR= STANDBY_CONTROL_AND_STATUS, 0x4502 	// REG= 0x0018, 0x4502
            mt9v115_i2c_write_w_sensor(0x0018, 0x4502);
            mdelay(20); //TEST
            // FIELD_WR= PAD_SLEW, 0x0100 	// REG= 0x001E, 0x0100; IO Slew set to 0 to fix Column band on left of image
            mt9v115_i2c_write_w_sensor(0x001E, 0x0100);
            // FIELD_WR= OUTPUT_FORMAT_CONFIGURATION, 0x0040 	// REG= 0x332E, 0x0040
            mt9v115_i2c_write_w_sensor(0x332E, 0x0040);
            // FIELD_WR= OUTPUT_FORMAT_CONFIGURATION, 0x0000 	// REG= 0x332E, 0x0000
            mt9v115_i2c_write_w_sensor(0x332E, 0x0002);     // swaps chroma with luma
            // FIELD_WR= RESET_AND_MISC_CONTROL, 0x0300 	// REG= 0x001A, 0x0300
            mt9v115_i2c_write_w_sensor(0x001A, 0x0300);
            mdelay(20); //TEST
            // FIELD_WR= RESET_AND_MISC_CONTROL, 0x0344 	// REG= 0x001A, 0x0344
            mt9v115_i2c_write_w_sensor(0x001A, 0x0344);
            mdelay(20); //TEST
#endif            
//vince modify for auto-frame rate---
            // [Timing_settings]
            array_length = sizeof(mt9v115_init_timing_array) /
				sizeof(mt9v115_init_timing_array[0]);
			for (i = 0; i < array_length; i++) {
			    
			    if (mt9v115_init_timing_array[i].reg_addr == 0x9803) { // for write byte
			        mt9v115_i2c_write_b_sensor(
					mt9v115_init_timing_array[i].reg_addr,
					mt9v115_init_timing_array[i].reg_val);
                    continue;
                }
                if (mt9v115_init_timing_array[i].reg_addr == 0xA072) { // for write byte
			        mt9v115_i2c_write_b_sensor(
					mt9v115_init_timing_array[i].reg_addr,
					mt9v115_init_timing_array[i].reg_val);
                    continue;
                }
                if (mt9v115_init_timing_array[i].reg_addr == 0xA073) { // for write byte
			        mt9v115_i2c_write_b_sensor(
					mt9v115_init_timing_array[i].reg_addr,
					mt9v115_init_timing_array[i].reg_val);
                    continue;
                }
                if (mt9v115_init_timing_array[i].reg_addr == 0xA074) { // for write byte
			        mt9v115_i2c_write_b_sensor(
					mt9v115_init_timing_array[i].reg_addr,
					mt9v115_init_timing_array[i].reg_val);
                    continue;
                }
                if (mt9v115_init_timing_array[i].reg_addr == 0xA075) { // for write byte
			        mt9v115_i2c_write_b_sensor(
					mt9v115_init_timing_array[i].reg_addr,
					mt9v115_init_timing_array[i].reg_val);
                    continue;
                }
			    
			    
				rc = mt9v115_i2c_write_w_sensor(
					mt9v115_init_timing_array[i].reg_addr,
					mt9v115_init_timing_array[i].reg_val);
				
				if (rc < 0)
					return rc;
			}
            
            // [Step3-Recommended]
            array_length = sizeof(mt9v115_init_recommended_array) /
				sizeof(mt9v115_init_recommended_array[0]);
			for (i = 0; i < array_length; i++) {
				rc = mt9v115_i2c_write_w_sensor(
					mt9v115_init_recommended_array[i].reg_addr,
					mt9v115_init_recommended_array[i].reg_val);
				
				if (rc < 0)
					return rc;
			}
//vince fine-tune+++
            array_length = sizeof(mt9v115_init_recommended2_array) /
				sizeof(mt9v115_init_recommended2_array[0]);
			for (i = 0; i < array_length; i++) {
				rc = mt9v115_i2c_write_w_sensor(
					mt9v115_init_recommended2_array[i].reg_addr,
					mt9v115_init_recommended2_array[i].reg_val);
				
				if (rc < 0)
					return rc;
			}

            array_length = sizeof(mt9v115_init_recommended3_array) /
				sizeof(mt9v115_init_recommended3_array[0]);
			for (i = 0; i < array_length; i++) {
			    if (mt9v115_init_recommended3_array[i].reg_addr == 0xA065 ||
			    	mt9v115_init_recommended3_array[i].reg_addr == 0x9418) { // for write byte
			        mt9v115_i2c_write_b_sensor(
					mt9v115_init_recommended3_array[i].reg_addr,
					mt9v115_init_recommended3_array[i].reg_val);
                   			 continue;
                		}
				rc = mt9v115_i2c_write_w_sensor(
					mt9v115_init_recommended3_array[i].reg_addr,
					mt9v115_init_recommended3_array[i].reg_val);
				
				if (rc < 0)
					return rc;
			}

            array_length = sizeof(mt9v115_init_recommended4_array) /
				sizeof(mt9v115_init_recommended4_array[0]);
			for (i = 0; i < array_length; i++) {
			    if (mt9v115_init_recommended4_array[i].reg_addr == 0x326E ||
			    	mt9v115_init_recommended4_array[i].reg_addr == 0x33F4 ||
			    	mt9v115_init_recommended4_array[i].reg_addr == 0xA0B9 ||
			    	mt9v115_init_recommended4_array[i].reg_addr == 0xA0BB ||
			    	mt9v115_init_recommended4_array[i].reg_addr == 0xA07E ||
			    	mt9v115_init_recommended4_array[i].reg_addr == 0xA085 ||
			    	mt9v115_init_recommended4_array[i].reg_addr == 0xA0AD ||
			    	mt9v115_init_recommended4_array[i].reg_addr == 0xA0AF ||
			    	mt9v115_init_recommended4_array[i].reg_addr == 0xA027 ||
			    	mt9v115_init_recommended4_array[i].reg_addr == 0xA029 ||
			    	mt9v115_init_recommended4_array[i].reg_addr == 0xA01C ||
			    	mt9v115_init_recommended4_array[i].reg_addr == 0xA023 ||
			    	mt9v115_init_recommended4_array[i].reg_addr == 0xA025 ||
			    	mt9v115_init_recommended4_array[i].reg_addr == 0xA01E ||
			    	mt9v115_init_recommended4_array[i].reg_addr == 0x098E ) { // for write word
			        mt9v115_i2c_write_w_sensor(
					mt9v115_init_recommended4_array[i].reg_addr,
					mt9v115_init_recommended4_array[i].reg_val);
                   			 continue;
                		}
				rc = mt9v115_i2c_write_b_sensor(
					mt9v115_init_recommended4_array[i].reg_addr,
					mt9v115_init_recommended4_array[i].reg_val);
				
				if (rc < 0)
					return rc;
			}
//vince fine-tune---			
			// REG= 0x0018, 0x0002 	// STANDBY_CONTROL_AND_STATUS // FIELD_WR = STANDBY_CONTROL_AND_STATUS, 0x0002	
			// release power up stop, keep FW PLL startup
			mt9v115_i2c_write_w_sensor(0x0018, 0x0002);
//vince fine-tune+++
			mt9v115_i2c_write_w_sensor(0x3C00, 0x5004);
//vince fine-tune---
		    mdelay(20); //TEST

#else			
			// [Step1-Reset]
            // FIELD_WR= RESET_AND_MISC_CONTROL, RESET_PSEUDO_PIN, 1		
            // this bit is self-clearing, it is not necessary to re-write a zero
            mt9v115_i2c_read(0x001A, &read_data, 2);
            CDBG("mt9v115 read_data(0x001A) = 0x%x\n", read_data);
            mt9v115_i2c_write_w_sensor(0x001A, (read_data|0x0002));
			
			CDBG("Register Reset...\n");
			mt9v115_register_dump();
			
			/* [PLL_settings] { */
	        mt9v115_i2c_write_w_sensor(0x0010, 0x031E);     //pll_dividers = 798
            mt9v115_i2c_write_w_sensor(0x0012, 0x0200);     //pll_p_dividers = 512
            mt9v115_i2c_write_w_sensor(0x001E, 0x0107);     //DOUT slew = 7, FV/LV slew = 0, PIXCLK slew = 1
            
            mt9v115_i2c_write_w_sensor(0x0018, 0x0006);
            //mt9v115_i2c_write_w_sensor(0x0018, 0x0044);

            for (i=0; i<10; i++) {
                mt9v115_i2c_read(0x0018, &read_data, 2);
                CDBG("mt9v115 read_data(0x0018) = 0x%x\n", read_data);
                if (!(read_data & 0x4000)) {
                    CDBG("  Normal operation!!\n");
                    break;
                }
                else {
                    CDBG("  Firmware is in standby!!\n");   
                }
                msleep(1);
            }
			/* } [PLL_settings] */
			
			// FIELD_WR = RESET_AND_MISC_CONTROL, PARALLEL_ENABLE, 0
			mt9v115_i2c_read(0x001A, &read_data, 2);
            CDBG("mt9v115 read_data(0x001A) = 0x%x\n", read_data);
            mt9v115_i2c_write_w_sensor(0x001A, (read_data&0xFDFF)); 
            
            // FIELD_WR = STANDBY_CONTROL_AND_STATUS, 0x0002	//release power up stop, keep FW PLL startup
            mt9v115_i2c_write_w_sensor(0x0018, 0x0002); 
            
            //POLL_FIELD = SEQ_STATE, != 2, DELAY=1, TIMEOUT=10
            for (i=0; i<10; i++) {
                mt9v115_i2c_read(0x8401, &read_data, 1);
                CDBG("mt9v115 read_data(0x8401) = 0x%x\n", read_data);
                if ((read_data == 2)) {
                    CDBG("  streaming!!\n");
                    break;
                }
                else {
                    CDBG("  Not streaming!!\n");   
                }
                msleep(1);
            }
            
            // Put standby back to power up stop, FW starts PLL.  Then reset the MCU.
            //FIELD_WR = STANDBY_CONTROL_AND_STATUS, 0x0006	//out of standby, start PLL, power up stop
            mt9v115_i2c_write_w_sensor(0x0018, 0x0006); 
            
            // FIELD_WR = RESET_AND_MISC_CONTROL, MCU_SOFT_RST_I2C_N, 0
            mt9v115_i2c_read(0x001A, &read_data, 2);
            CDBG("mt9v115 read_data(0x001A) = 0x%x\n", read_data);
            mt9v115_i2c_write_w_sensor(0x001A, (read_data&0xFFFB)); 
            
            // FIELD_WR = MON_HEARTBEAT, 0					// set to zero so it can be polled later
            mt9v115_i2c_write_w_sensor(0x8003, 0); 
            
            // FIELD_WR = RESET_AND_MISC_CONTROL, MCU_SOFT_RST_I2C_N, 1
            mt9v115_i2c_read(0x001A, &read_data, 2);
            CDBG("mt9v115 read_data(0x001A) = 0x%x\n", read_data);
            mt9v115_i2c_write_w_sensor(0x001A, (read_data|0x0004)); 
            
            // POLL_FIELD = MON_HEARTBEAT, == 0, DELAY=1, TIMEOUT=20     
            // mon_heartbeat will start incrementing when FW is at power up stop point
            for (i=0; i<20; i++) {
                mt9v115_i2c_read(0x8003, &read_data, 2);
                CDBG("mt9v115 read_data(0x8003) = 0x%x\n", read_data);
                if ((read_data != 0)) {
                    CDBG("  MON_HEARTBEAT!=0!!\n");
                    break;
                }
                else {
                    CDBG("  MON_HEARTBEAT==0!!\n");   
                }
                msleep(1);
            }
        
            /* [TEST] { */
            /*
            mt9v115_i2c_read(0x001A, &read_data, 2);
            CDBG("mt9v115 read_data(0x001A) = 0x%x\n", read_data);
			mt9v115_i2c_write_w_sensor(0x001A, (read_data|0x0400)); // mipi_mode_enable
			mt9v115_i2c_read(0x001A, &read_data, 2);
            CDBG("mt9v115 read_data(0x001A) = 0x%x\n", read_data);
			mt9v115_i2c_write_w_sensor(0x3C00, 0x5006); // tx_control: Sensor MIPI (8-bit)
			*/
	         /* } [TEST] */
	
			array_length = sizeof(mt9v115_init_settings_array) /
				sizeof(mt9v115_init_settings_array[0]);
				
			for (i = 0; i < array_length; i++) {
			    if (mt9v115_init_settings_array[i].reg_addr == 0xFFFF) { // for delay
			        msleep(mt9v115_init_settings_array[i].reg_val);
                    continue;
                }
                if (mt9v115_init_settings_array[i].reg_addr == 0x9803) { // for write byte
			        mt9v115_i2c_write_b_sensor(0x9803, 0x07);
                    continue;
                }
                if (mt9v115_init_settings_array[i].reg_addr == 0xA072) { // for write byte
			        mt9v115_i2c_write_b_sensor(0xA072, 0x11);
                    continue;
                }
                if (mt9v115_init_settings_array[i].reg_addr == 0xA073) { // for write byte
			        mt9v115_i2c_write_b_sensor(0xA073, 0x13);
                    continue;
                }
                if (mt9v115_init_settings_array[i].reg_addr == 0xA074) { // for write byte
			        mt9v115_i2c_write_b_sensor(0xA074, 0x14);
                    continue;
                }
                if (mt9v115_init_settings_array[i].reg_addr == 0xA075) { // for write byte
			        mt9v115_i2c_write_b_sensor(0xA075, 0x16);
                    continue;
                }
                
				rc = mt9v115_i2c_write_w_sensor(
					mt9v115_init_settings_array[i].reg_addr,
					mt9v115_init_settings_array[i].reg_val);
				
				if (rc < 0)
					return rc;
			}
			
            /* [TEST] { */
            mt9v115_i2c_read(0x001A, &read_data, 2);
            CDBG("mt9v115 read_data(0x001A) = 0x%x\n", read_data);
            mt9v115_i2c_write_w_sensor(0x001A, (read_data&0xFDFF)|0x0400); 
            /* } [TEST] */
            
            			
			// STANDBY_CONTROL_AND_STATUS, 0x0002	//release power up stop, keep FW PLL startup
			mt9v115_i2c_write_w_sensor(0x0018, 0x0002);

#endif

			
			MT9V115_CSI_CONFIG = 1;
			msleep(20);
			
			// For Debugging...
			mt9v115_register_dump();
			
			/*
			mt9v115_i2c_read(0x001A, &read_data, 2);
            CDBG("mt9v115 read_data(0x001A) = 0x%x\n", read_data);
			mt9v115_i2c_write_w_sensor(0x001A, (read_data|0x0400)&0xFDFF); // mipi_mode_enable			
			mt9v115_register_dump();
			*/
			
			return rc;
		}
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}

static int32_t mt9v115_video_config(int mode)
{
	int32_t rc = 0;
	int rt;
	/* change sensor resolution if needed */
	rt = RES_PREVIEW;

	if (mt9v115_sensor_setting(UPDATE_PERIODIC, rt) < 0)
		return rc;
	mt9v115_ctrl->curr_res = mt9v115_ctrl->prev_res;
	mt9v115_ctrl->sensormode = mode;
	return rc;
}

static int32_t mt9v115_set_sensor_mode(int mode,
	int res)
{
	int32_t rc = 0;
	switch (mode) {
	case SENSOR_PREVIEW_MODE:
		rc = mt9v115_video_config(mode);
		break;
	case SENSOR_SNAPSHOT_MODE:
	case SENSOR_RAW_SNAPSHOT_MODE:
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}
static int32_t mt9v115_power_down(void)
{
    CDBG("%s\n", __func__);

    if (mt9v115_ctrl->sensordata) {
        CDBG("%s: standby pin = GPIO_%d\n", __func__, mt9v115_ctrl->sensordata->sensor_reset);
        gpio_request(mt9v115_ctrl->sensordata->sensor_reset, "mt9v115");

        gpio_direction_output(mt9v115_ctrl->sensordata->sensor_reset, 1);
        msleep(50);

        gpio_free(mt9v115_ctrl->sensordata->sensor_reset);
    }
	return 0;
}

static int mt9v115_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	uint16_t chip_version;
	int32_t rc = 0;
	/*The reset pin is not physically connected to the sensor.
	The standby pin will do the reset hence there is no need
	to request the gpio reset*/

    gpio_request(data->sensor_reset, "mt9v115");
	CDBG(" mt9e013_probe_init_sensor\n");

		CDBG("sensor_reset(%d) = %d\n", data->sensor_reset, rc);
		gpio_direction_output(data->sensor_reset, 1);
		msleep(50);
		gpio_set_value_cansleep(data->sensor_reset, 0);
		msleep(10);

    gpio_free(data->sensor_reset);

	/* Read sensor Model ID: */
	rc = mt9v115_i2c_read(REG_MT9V115_CHIP_VERSION, &chip_version, 2);
	if (rc < 0)
		goto init_probe_fail;

	CDBG("mt9v115 chip_version = 0x%x\n", chip_version);
	/* 4. Compare sensor ID to MT9V115 ID: */
	if (chip_version != MT9V115_CHIP_VERSION) {
		rc = -ENODEV;
		goto init_probe_fail;
	}
	goto init_probe_done;
init_probe_fail:
	pr_warning(" mt9v115_probe_init_sensor fails\n");
init_probe_done:
	CDBG(" mt9v115_probe_init_sensor finishes\n");
	return rc;
}

int mt9v115_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;

	CDBG("%s: %d\n", __func__, __LINE__);
	CDBG("Calling mt9v115_sensor_open_init\n");
	mt9v115_ctrl = kzalloc(sizeof(struct mt9v115_ctrl_t), GFP_KERNEL);
	if (!mt9v115_ctrl) {
		CDBG("mt9v115_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}
	mt9v115_ctrl->fps_divider = 1 * 0x00000400;
	mt9v115_ctrl->pict_fps_divider = 1 * 0x00000400;
	mt9v115_ctrl->fps = 30 * Q8;
	mt9v115_ctrl->set_test = TEST_OFF;
	mt9v115_ctrl->prev_res = QTR_SIZE;
	mt9v115_ctrl->pict_res = FULL_SIZE;
	mt9v115_ctrl->curr_res = INVALID_SIZE;

	if (data)
		mt9v115_ctrl->sensordata = data;

	/* enable mclk first */

	msm_camio_clk_rate_set(24000000);
//	msm_camio_clk_rate_set(12000000);
	msleep(20);

	rc = mt9v115_probe_init_sensor(data);
	mt9v115_effect_mode=-1;
	if (rc < 0) {
		CDBG("Calling mt9v115_sensor_open_init fail\n");
		goto init_fail;
	}

	rc = mt9v115_sensor_setting(REG_INIT, RES_PREVIEW);
	if (rc < 0)
		goto init_fail;
	else
		goto init_done;
init_fail:
	CDBG(" mt9v115_sensor_open_init fail\n");
	kfree(mt9v115_ctrl);
init_done:
	CDBG("mt9v115_sensor_open_init done\n");
	return rc;
}

static int mt9v115_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&mt9v115_wait_queue);
	return 0;
}

static const struct i2c_device_id mt9v115_i2c_id[] = {
	{"mt9v115", 0},
	{ }
};

static int mt9v115_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	CDBG("mt9v115_i2c_probe called!\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CDBG("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	mt9v115_sensorw = kzalloc(sizeof(struct mt9v115_work_t), GFP_KERNEL);
	if (!mt9v115_sensorw) {
		CDBG("kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, mt9v115_sensorw);
	mt9v115_init_client(client);
	mt9v115_client = client;

	CDBG("mt9v115_i2c_probe success! rc = %d\n", rc);
	return 0;

probe_failure:
	CDBG("mt9v115_i2c_probe failed! rc = %d\n", rc);
	return rc;
}

static int __exit mt9v115_remove(struct i2c_client *client)
{
	struct mt9v115_work_t_t *sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);
	mt9v115_client = NULL;
	kfree(sensorw);
	return 0;
}

static struct i2c_driver mt9v115_i2c_driver = {
	.id_table = mt9v115_i2c_id,
	.probe  = mt9v115_i2c_probe,
	.remove = __exit_p(mt9v115_i2c_remove),
	.driver = {
		.name = "mt9v115",
	},
};

int mt9v115_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;
	if (copy_from_user(&cdata,
		(void *)argp,
		sizeof(struct sensor_cfg_data)))
		return -EFAULT;
	mutex_lock(&mt9v115_mut);
	CDBG("mt9v115_sensor_config: cfgtype = %d\n",
	cdata.cfgtype);
	switch (cdata.cfgtype) {
	case CFG_SET_MODE:
		rc = mt9v115_set_sensor_mode(cdata.mode,
			cdata.rs);
		break;
	case CFG_PWR_DOWN:
		rc = mt9v115_power_down();
		break;
	case CFG_SET_EFFECT:
		rc = mt9v115_set_effect(cdata.mode,
					 cdata.cfg.effect);
		break;
	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(&mt9v115_mut);

	return rc;
}
static int mt9v115_sensor_release(void)
{
	int rc = -EBADF;
	mutex_lock(&mt9v115_mut);
	mt9v115_power_down();
	kfree(mt9v115_ctrl);
	mt9v115_ctrl = NULL;
	CDBG("mt9v115_release completed\n");
	mutex_unlock(&mt9v115_mut);

	return rc;
}

static int mt9v115_sensor_probe(const struct msm_camera_sensor_info *info,
		struct msm_sensor_ctrl *s)
{
	int rc = 0;
	rc = i2c_add_driver(&mt9v115_i2c_driver);
	if (rc < 0 || mt9v115_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_fail;
	}
	msm_camio_clk_rate_set(24000000);
//	msm_camio_clk_rate_set(12000000);
	rc = mt9v115_probe_init_sensor(info);
	if (rc < 0)
		goto probe_fail;
	s->s_init = mt9v115_sensor_open_init;
	s->s_release = mt9v115_sensor_release;
	s->s_config  = mt9v115_sensor_config;
	s->s_camera_type = FRONT_CAMERA_2D;
	s->s_mount_angle = info->sensor_platform_info->mount_angle;
	return rc;

probe_fail:
	CDBG("mt9v115_sensor_probe: SENSOR PROBE FAILS!\n");
	i2c_del_driver(&mt9v115_i2c_driver);
	return rc;
}

static int __mt9v115_probe(struct platform_device *pdev)
{

	return msm_camera_drv_start(pdev, mt9v115_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __mt9v115_probe,
	.driver = {
		.name = "msm_camera_mt9v115",
		.owner = THIS_MODULE,
	},
};

static int __init mt9v115_init(void* hdl)
{
	return platform_driver_register(&msm_camera_driver);
}

//module_init(mt9v115_init);
DECLARE_DYN_LOADER(DRIVER_TYPE_FRONT_CAM, 
			FRONT_CAM_MT9V115_V1, 
			mt9v115_init, LEVEL6);

MODULE_DESCRIPTION("OMNI VGA YUV sensor driver");
MODULE_LICENSE("GPL v2");
