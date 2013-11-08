/*
fih 1090_usb driver porting,add for HWID control
*/

int Linux_Mass=0;
bool is_switch=false;
#define NV_FIH_VERSION_I 8030
int fih_enable = 0;
bool switch_enable=false;
#include <linux/power_supply.h>  /* May adds, 2010.10.28 */
//static struct power_supply * g_ps_usb;
int USB_Connect=0;
int backup_USB_Connect=0;
extern unsigned int fih_host_usb_id;
void usb_chg_pid(bool usb_switch);
void cable_status(bool status);