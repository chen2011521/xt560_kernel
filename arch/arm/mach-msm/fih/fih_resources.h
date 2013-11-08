#ifndef FIH_RESOURCES_H
#define FIH_RESOURCES_H


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio_event.h>
#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <mach/vreg.h>
#include <mach/socinfo.h>

#include <linux/device.h>

#include <linux/i2c.h>
#include "../devices-msm7x2xa.h"

#include <fih/dynloader.h>

#if defined(CONFIG_FIH_WIFI)
#include "resources/wifi/fih_wifi.h"
#endif

#if defined(CONFIG_FIH_TOUCHSCREEN)
#include "resources/touchscreen/fih_touchscreen.h"
#endif

#if defined(CONFIG_FIH_BT)
#include "resources/bt/fih_bt.h"
#endif

#if defined(CONFIG_FIH_LCM)
#include "resources/lcm/fih_lcm.h"
#endif

#if defined(CONFIG_FIH_KEYPAD)
#include "resources/keypad/fih_keypad.h"
#endif

#if defined(CONFIG_FIH_LED)
#include "resources/led/fih_led.h"
#endif

#if defined(CONFIG_FIH_CAM)
#include "resources/cam/fih_cam.h"
#endif

#if defined(CONFIG_FIH_SENSORS)
#include "resources/sensors/fih_sensors.h"
#endif

#if defined(CONFIG_FIH_DEBUG)
#include "resources/debug/fih_dbg.h"
#endif

#if defined(CONFIG_FIH_USB)
#include "resources/usb/fih_usb.h"
#endif

#if defined(CONFIG_FIH_BATTERY)
#include "resources/battery/fih_battery.h"
#endif

#if defined(CONFIG_FIH_VIBRATOR)
#include "resources/vibrator/fih_vibrator.h"
#endif

#if defined(CONFIG_FIH_UART)
#include "resources/uart/fih_uart.h"
#endif

#if defined(CONFIG_FIH_SDCC)
#include "resources/sdcc/fih_sdcc.h"
#endif

#if defined(CONFIG_FIH_IRM_AUDIO)
#include "resources/headset/fih_headset.h"
#endif

/*Modified by Jiahao,2011-9-23*/
#if defined(CONFIG_FIH_NFC)
#include "resources/nfc/fih_nfc.h"
#endif

/*FIH-NJ-SD4, Alex, Add,*/ 
#if defined(CONFIG_FIH_TSIF) && defined(CONFIG_TSIF_NM32X_62X_DEV)
#include "resources/dtv/fih_dtv.h"
#endif
/* add new type here */
#endif /* FIH_RESOURCES_H */