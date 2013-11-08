/*
FIH-StevenCPHuang_20110831,add this file for new HWID control 
*/

#include "../../../../arch/arm/mach-msm/proc_comm.h"
//FIH-StevenCPHuang_20110901,change to the other file location to read out build id ++

#define NV_FIH_VERSION_I        8030

#define BUILD_ID_0              "/proc/fver"
#define BUILD_ID_1              "/system/build_id"

//FIH-StevenCPHuang_20110901,change to the other file location to read out build id --
#ifdef CONFIG_FIH_USB_PORTING//CONFIG_FIH_PROJECT_IRM
#ifdef CONFIG_FIH_PID_SWITCH//FIH-StevenCPHuang_20110818-IRM1070_USB PID Switch  -00+{
//extern void scsi_set_adb_root(void);
//
extern bool is_switch;
extern void usb_chg_pid(bool usb_switch);
#endif//FIH-StevenCPHuang_20110818-IRM1070_USB PID Switch  -00+}
#endif	//CONFIG_FIH_USB_PORTING

static int do_switch_status(struct fsg_common *fsg, struct fsg_buffhd *bh) //static 
{
	char online[36] = "1", offline[36] = "0"; //,temp[]="1"; 	
	u8	*buf = (u8 *) bh->buf;
	
	memset(buf, 0, 36);	
	
	if (is_switch==true)
    {
	    sprintf(buf , online);
	} 
    else 
    {
	    sprintf(buf , offline);
	}
	return 36;
}

static int do_read_battery(struct fsg_common *fsg, struct fsg_buffhd *bh) //static 
{
    //struct 	* batt_info_if = get_batt_info_if(); //StevenCPHuang_20110712,disable it temporary
    int batt_capasity = 50;//StevenCPHuang_20110712,use the fixed fake value first( batt_info_if->get_batt_capacity();)
    u8 *buf = (u8 *) bh->buf;

    memset(buf, 0x0, 36);

    buf[0] = batt_capasity;
    //USBDBG("batt = %d%%", batt_capasity);
    return 36;
}

static int do_read_nv(struct fsg_common *fsg, struct fsg_buffhd *bh) //static 
{
    struct file *gMD_filp = NULL;
    u8 *buf = (u8 *) bh->buf;
    char text[20];

#ifdef CONFIG_FIH_PID_SWITCH//FIH-StevenCPHuang_20110829,use scsi command to read product id-00+{
    int32_t smem_proc_comm_oem_cmd1 = PCOM_CUSTOMER_CMD1;
    int32_t smem_proc_comm_oem_data1 = SMEM_PROC_COMM_OEM_NV_READ;
    int32_t smem_proc_comm_oem_data2= NV_FIH_VERSION_I;
    int32_t fih_version[32];
#endif//FIH-StevenCPHuang_20110829,use scsi command to read product id-00-}
    char ver_buf[128];
    mm_segment_t oldfs;

    memset(ver_buf, 0x0, sizeof(ver_buf));
//FIH-StevenCPHuang_20110829,enable for use scsi command to read product id-00+{
#ifdef CONFIG_FIH_PID_SWITCH
    if (msm_proc_comm_oem(smem_proc_comm_oem_cmd1, &smem_proc_comm_oem_data1, fih_version, &smem_proc_comm_oem_data2)==0)
    {
        memcpy(ver_buf, fih_version, sizeof(ver_buf));
        printk(KERN_INFO "%s : fih_version=%s\n", __func__, ver_buf);
    }
#endif
//FIH-StevenCPHuang_20110829,enable for use scsi command to read product id-00-}
    oldfs = get_fs();
    set_fs(KERNEL_DS);
    gMD_filp = filp_open(BUILD_ID_0, O_RDONLY, 0);

    if (!IS_ERR(gMD_filp)) 
    {
        printk(KERN_INFO "%s : BUILD_ID_0 = %s\n", __func__, BUILD_ID_0);
        gMD_filp->f_op->read(gMD_filp, text, sizeof(text), &gMD_filp->f_pos);
        filp_close(gMD_filp, NULL);

        printk(KERN_INFO "%s : text = %s\n", __func__, text);
        memcpy(ver_buf+1, &text[5], 1);
        memcpy(ver_buf+2, &text[7], 3);
        sprintf(buf, ver_buf);
        printk(KERN_INFO "%s : fih_version = %s\n", __func__, ver_buf);
    } 
    else 
    {
        gMD_filp = filp_open(BUILD_ID_1, O_RDONLY, 0);
        if (!IS_ERR(gMD_filp))
        {
            printk(KERN_INFO "%s : BUILD_ID_1 = %s \n", __func__, BUILD_ID_1);
            gMD_filp->f_op->read(gMD_filp, text, sizeof(text), &gMD_filp->f_pos);
            filp_close(gMD_filp, NULL);

            printk(KERN_INFO "%s : text = %s\n", __func__, text);
            memcpy(ver_buf+1, &text[5], 1);
            memcpy(ver_buf+2, &text[7], 3);
            sprintf(buf, ver_buf);
            printk(KERN_INFO "%s : fih_version = %s\n", __func__, ver_buf);
        }
        else
        {
            printk(KERN_INFO "%s : open build_id file fail\n", __func__);
        }
    }
    return 128;
}
