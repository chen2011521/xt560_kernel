//======================================================================
//
// Copyright    2009 Chi Mei Communication Systems, Inc
//                                                                      
//----------------------------------------------------------------------
// PROJECT : FXX
// MODIFY BY : JamesKCTung
// MODIFY DATE : 2009, 12, 25
// CREATED BY : Will Chen
// CREATED DATE : 2008,06,21
//----------------------------------------------------------------------
//
// PURPOSE : Provides debug macros for other components, pair with library "kernel"
//

enum{
    FIH_DEBUG_ZONE_G0  = 1U << 0,
    FIH_DEBUG_ZONE_G1  = 1U << 1,
    FIH_DEBUG_ZONE_G2  = 1U << 2,
    FIH_DEBUG_ZONE_G3  = 1U << 3,
    FIH_DEBUG_ZONE_G4  = 1U << 4,
    FIH_DEBUG_ZONE_G5  = 1U << 5,
    FIH_DEBUG_ZONE_G6  = 1U << 6,
    FIH_DEBUG_ZONE_G7  = 1U << 7,
    FIH_DEBUG_ZONE_G8  = 1U << 8,
    FIH_DEBUG_ZONE_G9  = 1U << 9,
    FIH_DEBUG_ZONE_G10 = 1U << 10,
    FIH_DEBUG_ZONE_G11 = 1U << 11,
    FIH_DEBUG_ZONE_G12 = 1U << 12,
    FIH_DEBUG_ZONE_G13 = 1U << 13,
    FIH_DEBUG_ZONE_G14 = 1U << 14,
    FIH_DEBUG_ZONE_G15 = 1U << 15,
    FIH_DEBUG_ZONE_G16 = 1U << 16,
    FIH_DEBUG_ZONE_G17 = 1U << 17,
    FIH_DEBUG_ZONE_G18 = 1U << 18,
    FIH_DEBUG_ZONE_G19 = 1U << 19,
    FIH_DEBUG_ZONE_G20 = 1U << 20,
    FIH_DEBUG_ZONE_G21 = 1U << 21,
    FIH_DEBUG_ZONE_G22 = 1U << 22,
    FIH_DEBUG_ZONE_G23 = 1U << 23,
    FIH_DEBUG_ZONE_G24 = 1U << 24,
    FIH_DEBUG_ZONE_G25 = 1U << 25,
    FIH_DEBUG_ZONE_G26 = 1U << 26,
    FIH_DEBUG_ZONE_G27 = 1U << 27,
    FIH_DEBUG_ZONE_G28 = 1U << 28,
    FIH_DEBUG_ZONE_G29 = 1U << 29,
    FIH_DEBUG_ZONE_G30 = 1U << 30,
    FIH_DEBUG_ZONE_G31 = 1U << 31,
};

/*FIH, KanyYJLin, debugmask, 2011/09/22{*/
//#ifdef CONFIG_FIH_FXX
#define DEBUG_MASK_BASE            MSM_DEBUGMASK_BASE
/*#else
#define DEBUG_MASK_BASE            (MSM_PLOG_BASE + 0X80000)
#endif*/
/*FIH, KanyYJLin, debugmask, 2011/09/22}*/

/* FIH, JiaHao, 2011/04/28 { */
/* the follow define sequence must be the same as enum sequence in LogManager.c */
#define ALS_DEBUG_MASK_OFFSET      (DEBUG_MASK_BASE)
#define VB_DEBUG_MASK_OFFSET       (ALS_DEBUG_MASK_OFFSET + 4)
#define BT_DEBUG_MASK_OFFSET       (VB_DEBUG_MASK_OFFSET + 4)
#define WIFI_DEBUG_MASK_OFFSET     (BT_DEBUG_MASK_OFFSET + 4)
#define GS_DEBUG_MASK_OFFSET       (WIFI_DEBUG_MASK_OFFSET + 4)

#define UFN_DEBUG_MASK_OFFSET      (GS_DEBUG_MASK_OFFSET + 4)
#define SER_DEBUG_MASK_OFFSET      (UFN_DEBUG_MASK_OFFSET + 4)
#define CAM_DEBUG_MASK_OFFSET      (SER_DEBUG_MASK_OFFSET + 4)
#define SD_DEBUG_MASK_OFFSET       (CAM_DEBUG_MASK_OFFSET + 4)
#define KPD_DEBUG_MASK_OFFSET      (SD_DEBUG_MASK_OFFSET + 4)

#define TOUCH_DEBUG_MASK_OFFSET    (KPD_DEBUG_MASK_OFFSET + 4)
#define LCD_DEBUG_MASK_OFFSET      (TOUCH_DEBUG_MASK_OFFSET + 4)
#define BKL_DEBUG_MASK_OFFSET      (LCD_DEBUG_MASK_OFFSET + 4)
#define PWR_DEBUG_MASK_OFFSET      (BKL_DEBUG_MASK_OFFSET + 4)
#define WAV_DEBUG_MASK_OFFSET      (PWR_DEBUG_MASK_OFFSET + 4)

#define NAND_DEBUG_MASK_OFFSET     (WAV_DEBUG_MASK_OFFSET + 4)
#define BAT_DEBUG_MASK_OFFSET      (NAND_DEBUG_MASK_OFFSET + 4)
#define NLED_DEBUG_MASK_OFFSET     (BAT_DEBUG_MASK_OFFSET + 4)
#define MTB_DEBUG_MASK_OFFSET      (NLED_DEBUG_MASK_OFFSET + 4)
#define DCVS_DEBUG_MASK_OFFSET     (MTB_DEBUG_MASK_OFFSET + 4)

#define DIAGCHAR_DEBUG_MASK_OFFSET (DCVS_DEBUG_MASK_OFFSET + 4)
#define MSMRTC_DEBUG_MASK_OFFSET   (DIAGCHAR_DEBUG_MASK_OFFSET + 4)
#define ALARM_DEBUG_MASK_OFFSET    (MSMRTC_DEBUG_MASK_OFFSET + 4)
#define SC668_DEBUG_MASK_OFFSET    (ALARM_DEBUG_MASK_OFFSET + 4)	/* FIH, JiaHao, 2011/01/13 */
#define PRINTK_DEBUG_MASK_OFFSET   (SC668_DEBUG_MASK_OFFSET + 4)	/* FIH, JiaHao, 2011/04/26 */
#define SMD_RPCROUTER_DEBUG_MASK_OFFSET   (PRINTK_DEBUG_MASK_OFFSET + 4)	
/* FIH, JiaHao, 2011/04/28 } */

asmlinkage int fih_printk(uint32_t value, int cond, const char *fmt, ...);
