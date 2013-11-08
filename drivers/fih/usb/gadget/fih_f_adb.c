
#include "../../../../arch/arm/mach-msm/proc_comm.h"


#define NV_BSP_ADB_USER_RIGHT_I 8028

static bool scsi_adb_root = false;
static bool nv_adb_root = false;
void scsi_set_adb_root(void);
void nv_read_adb_root_right(void);

static ssize_t root_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    bool is_root = (scsi_adb_root || nv_adb_root);
    scsi_adb_root = false;
    return sprintf(buf, "%d\n", is_root?1:0);
}

DEVICE_ATTR(root, S_IRUGO | S_IWUSR, root_show, NULL);

int create_root_switch_attribute(struct adb_dev *dev)
{
    int rc = 0;
    rc = device_create_file(dev->function.dev, &dev_attr_root);
    if (rc<0)
    {
        pr_err("%s: Create attribute \"root\" failed!! <%d>", __func__, rc);
    }
    return rc;
}

void scsi_set_adb_root()
{
    scsi_adb_root = true;
}

void nv_read_adb_root_right()
{
    uint32_t smem_proc_comm_oem_data1 = SMEM_PROC_COMM_OEM_NV_READ;
    unsigned int allports_flag[1];
    unsigned smem_response;

    memset(allports_flag, 0, sizeof(allports_flag));
    allports_flag[0] = NV_BSP_ADB_USER_RIGHT_I;

    if (msm_proc_comm_oem_n(PCOM_CUSTOMER_CMD1, &smem_proc_comm_oem_data1, &smem_response, allports_flag, 1) == 0)
    {
        pr_debug("%s : allports_flag = 0x%x\n", __func__, allports_flag[0]);

        if ((allports_flag[0] & (1 << 0)) == (1 << 0))
        {
            nv_adb_root = false;
        }
        else
        {
            nv_adb_root = true;
        }
        pr_debug("%s: nv_adb_root = %s\n", __func__, (nv_adb_root)?"true":"false");
    }
    else
    {
        pr_debug("%s : Fail to read NV 8028!!\n", __func__);
    }
}
