/*
fih 1090_usb driver porting,add for HWID control
*/


bool isMoto = 1;                        //porting from DP2,set default as 1
static int switch_port_enable = 0;
static int _registered_function_count = 0;

module_param_named(switch_port_enable, switch_port_enable, int, S_IRUGO| S_IWUSR);

#ifdef CONFIG_FIH_PID_SWITCH            //CONFIG_FIH_PROJECT_IRM
#define     VENDOR_ID		    0x22B8
#define     PRODUCT_ID		    0x2DE7  //StevenCPHuang_20110905,change the default PID to 2DE7
#endif	                                // CONFIG_FIH_PROJECT_IRM

/* string IDs are assigned dynamically */

char *fih_version_buf;

static bool tethering_enabled = false;
static unsigned short desc_pid;         //FIH-StevenCPHuang_20110818-IRM1070_USB PID Switch
static unsigned short keep_desc_pid;         

#ifdef CONFIG_FIH_FTM
unsigned short orig_pid = 0xC003;
#else
unsigned short orig_pid = 0xC001;       //FIH-StevenCPHuang_20110818-IRM1070_USB PID Switch
#endif

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};
static char *usb_functions_custfull[] = {
	"usb_mass_storage",
	"diag",
	"adb",
	"modem",
};
static char *usb_functions_nodiag[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_msc[] = {
	"usb_mass_storage",
};
static char *usb_functions_modem[] = {
	"usb_mass_storage",
	"adb",
	"modem",
};
#ifdef CONFIG_FIH_FTM
static char *usb_functions_ftm[] = {
	"modem",
};
#endif

static struct android_usb_product usb_products[] = {
#ifdef CONFIG_FIH_FTM
//original pid is C003
	{
		.product_id	    = 0x2DE5,
		.num_functions  = ARRAY_SIZE(usb_functions_ftm),
		.functions      = usb_functions_ftm,
	},
#endif
//original pid is C000
	{
		.product_id     = 0x2DE6,
		.num_functions  = ARRAY_SIZE(usb_functions_custfull),
		.functions	    = usb_functions_custfull,
	},
//original pid is C004
	{
		.product_id	    = 0x2DE7,
		.num_functions	= ARRAY_SIZE(usb_functions_msc),
		.functions      = usb_functions_msc,
	},
//original pid is C001
	{
		.product_id     = 0x2DE8,
		.num_functions  = ARRAY_SIZE(usb_functions_nodiag),
		.functions      = usb_functions_nodiag,
	},
//original pid is C008
	{
		.product_id     = 0x2DE9,
		.num_functions  = ARRAY_SIZE(usb_functions_rndis_adb),
		.functions      = usb_functions_rndis_adb,
	},
//original pid is C00A
	{
		.product_id     = 0x2DEA,
		.num_functions  = ARRAY_SIZE(usb_functions_modem),
		.functions      = usb_functions_modem,
	},
}; 

