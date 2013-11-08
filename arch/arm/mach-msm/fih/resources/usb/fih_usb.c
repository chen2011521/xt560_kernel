/**
* USB Driver
******************************************/

#include "fih_usb.h"
#include <linux/usb/android_composite.h>
#include <mach/usb_gadget_fserial.h>
#include "../../../proc_comm.h"

//StevenCPHuang_20110812,add switch for QualComm PID setting as follows ++[
#ifdef CONFIG_FIH_PROJECT_IRM
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

static char *usb_functions_msc[] = {
	"usb_mass_storage",
};

static char *usb_functions_nodiag[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_modem[] = {
	"usb_mass_storage",
	"adb",
	"modem",
};

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
	"usb_mass_storage",
#ifdef CONFIG_USB_ANDROID_DIAG
	"diag",
#endif
	"adb",
#ifdef CONFIG_USB_F_SERIAL
	"modem",
	"nmea",
#endif
#ifdef CONFIG_USB_ANDROID_RMNET
	"rmnet",
#endif
};

static struct android_usb_product usb_products[] = {
	//original pid is c000
	{
		.product_id	= 0x2DE6,
		.num_functions	= ARRAY_SIZE(usb_functions_custfull),
		.functions	= usb_functions_custfull,
	},
	//original pid is c004
	{
		.product_id	= 0x2DE7,
		.num_functions	= ARRAY_SIZE(usb_functions_msc),
		.functions	= usb_functions_msc,
	},
	//original pid is c001
	// 0xC001 composition is removed diag port
	{
		.product_id	= 0x2DE8,
		.num_functions	= ARRAY_SIZE(usb_functions_nodiag),
		.functions	= usb_functions_nodiag,
	},
	//original pid is c008
	/* FIH, chandler_usbtethering 10.8.20 +++ */
	{
		.product_id	= 0x2DE9,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb),
		.functions	= usb_functions_rndis_adb,
	},
	{
		.product_id	= 0x2DEA,
		.num_functions	= ARRAY_SIZE(usb_functions_modem),
		.functions	= usb_functions_modem,
	},
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 2,
    /* +++ FIHTDC, AlbertYCFang, 2011.10.19 +++ */
	.vendor		= "GOOGLE",//"Qualcomm Incorporated",
    /* --- FIHTDC, AlbertYCFang, 2011.10.19 --- */
	.product	= "Mass storage",
	.release	= 0x0100,

};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x22B8,
	.product_id	= 0x2DE6,
	.version	= 0x0100,
	.product_name	= "Qualcomm HSUSB Device",
	.manufacturer_name = "Qualcomm Incorporated",
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
    /* +++ FIHTDC, AlbertYCFang, 2011.10.19 +++ */
	.serial_number = "123456789ABCDEF",//"1234567890ABCDEF",
    /* --- FIHTDC, AlbertYCFang, 2011.10.19 --- */
};

/* +++ FIHTDC, AlbertYCFang, 2011.10.19 +++ */
static int msm_read_serial_number_from_nvitem(void);

static int msm_read_serial_number_from_nvitem()
{
    uint32_t smem_proc_comm_oem_cmd1 = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1 = SMEM_PROC_COMM_OEM_PRODUCT_ID_READ;
    uint32_t smem_proc_comm_oem_data2 = 0;
    uint32_t product_id[40];

    if (msm_proc_comm_oem(smem_proc_comm_oem_cmd1, &smem_proc_comm_oem_data1, product_id, &smem_proc_comm_oem_data2) == 0)
    {
        printk(KERN_INFO "%s: [product_id=%s]\n", __func__, (char *)product_id);
        //memcpy(msm_hsusb_pdata.serial_number, product_id, 16);
        memcpy(android_usb_pdata.serial_number, product_id, 16);
    }
    return 1;
} 
/* --- FIHTDC, AlbertYCFang, 2011.10.19 --- */

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data = &android_usb_pdata,
	},
};

int __init usb_init(void)
{
	platform_device_register(&android_usb_device);
	platform_device_register(&usb_mass_storage_device);
    /* +++ FIHTDC, AlbertYCFang, 2011.10.19 +++ */
    msm_read_serial_number_from_nvitem();
    /* --- FIHTDC, AlbertYCFang, 2011.10.19 --- */
	return 0;	
}
#else
int __init usb_init(void)
{
	return 0;	
}
#endif
//StevenCPHuang_20110812,add switch for QualComm PID setting as follows ++]

