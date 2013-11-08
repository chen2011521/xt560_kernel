#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>	// for kzalloc

#include <linux/proc_fs.h> 
#include <linux/seq_file.h> 
#include <linux/uaccess.h>

#include <fih/dynloader.h>
#include <linux/spinlock.h>

#include "../../arch/arm/mach-msm/proc_comm.h"
unsigned alarm_set_ms = 0;
//FIH, DerrickDRLiu add for Power off alarm+++
ssize_t rtc_get_alarm_time(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", alarm_set_ms);
} 

ssize_t rtc_set_alarm_time(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned time_set, time_set_ms;
	//pm_rtc_julian_type  sys_time;
   	unsigned smem_response;
	uint32_t oem_cmd=SMEM_PROC_COMM_OEM_SET_RTC_ALARM;
	int ret = -1;
	sscanf(buf, "%u\n", &time_set);
	time_set_ms = time_set * 1000;//to ,mili-second
	alarm_set_ms = time_set_ms;
	printk(KERN_ERR "%s: %u\n", __func__, time_set_ms);

	//pr_err("%d-%d-%d %d:%d:%d\n", 
		//sys_time.year, sys_time.month, sys_time.day, sys_time.hour, sys_time.minute, sys_time.second);
	ret = msm_proc_comm_oem(PCOM_CUSTOMER_CMD1, &oem_cmd, &smem_response, &time_set_ms);

	if(ret)
	{
		printk(KERN_ERR "smem set rtc alarm Failed \n");
	       printk(KERN_ERR "%d  %d\n", smem_response, ret);
	}else{
		printk(KERN_ERR "smem set rtc alarm Okay \n");
		printk(KERN_ERR "%d  %d\n", smem_response, ret);
	}

	return count;
}
DEVICE_ATTR(rtcset, 0664, rtc_get_alarm_time, rtc_set_alarm_time);


int create_rtcset_attribute(struct platform_device *pdev)
{
	int rc = 0;
	printk(KERN_ERR "%s rtcset %p  %p\n", __func__, pdev, &pdev->dev);
	rc=device_create_file(&pdev->dev, &dev_attr_rtcset);
	if (rc < 0)
	{
	    pr_err("%s: Create attribute \"rtcset\" failed!! <%d>", __func__, rc);
	}
	printk(KERN_ERR "rtcset\n");
	return rc;
}
EXPORT_SYMBOL(create_rtcset_attribute);
//FIH, DerrickDRLiu add for Power off alarm---