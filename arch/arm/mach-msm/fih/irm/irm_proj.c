#include "irm_proj.h"

//Div2-SW2-BSP, JOE HSU+[
#include <linux/fih_hw_info.h>  
#include "../../smd_private.h"     
extern void fxx_info_init(void); 
//Div2-SW2-BSP, JOE HSU+]

int __init IRM_init(void)
{
	//Div2-SW2-BSP,JOE HSU+[
	fih_get_oem_info(); 
	fih_get_host_oem_info(); // SW2-5-1-MP-HostOemInfo-00+
	//Div2-SW2-BSP,JOE HSU+]

#ifdef CONFIG_FIH_DEBUG
	ram_console_init();
	last_alog_init();
#endif	

#ifdef CONFIG_FIH_TOUCHSCREEN
	touchscreen_init();
#endif

#ifdef CONFIG_FIH_BATTERY
	battery_init();
#endif

#ifdef CONFIG_FIH_USB
	usb_init();
#endif

#ifdef CONFIG_FIH_LED
	led_init();
#endif

#ifdef CONFIG_FIH_KEYPAD
	keypad_init();
#endif

#ifdef CONFIG_FIH_IRM_AUDIO
	headset_init();
#endif
	
#ifdef CONFIG_FIH_SENSORS
	sensors_init();
#endif

#ifdef CONFIG_FIH_BT
	bt_init();
#endif

#ifdef CONFIG_FIH_WIFI
	wifi_init();
#endif

#ifdef CONFIG_FIH_VIBRATOR
	vibrator_init();
#endif

#ifdef CONFIG_FIH_CAM
	camera_init();
#endif

#ifdef CONFIG_FIH_NFC
    nfc_init();
#endif

	/*FIH-NJ-SD4, Alex, Add,*/ 
#if defined(CONFIG_FIH_TSIF) && defined(CONFIG_TSIF_NM32X_62X_DEV)
		dtv_init();
#endif
	/* add new type here */

	//Div2-SW2-BSP,JOE HSU+[
	/*call product id functions for init verification*/
	fih_get_product_id();
	fih_get_product_phase();
	fih_get_band_id();
	fxx_info_init();   
	//Div2-SW2-BSP,JOE HSU-]			
	return 0;
}
