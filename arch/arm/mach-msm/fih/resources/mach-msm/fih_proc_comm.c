/* arch/arm/mach-msm/proc_comm.c
 *
 * Copyright (C) 2007-2008 Google, Inc.
 * Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <mach/msm_iomap.h>
#include <mach/system.h>

#include "../../../proc_comm.h"
#include "../../../smd_private.h"

static inline void notify_other_proc_comm(void)
{
	/* Make sure the write completes before interrupt */
	wmb();
#if defined(CONFIG_ARCH_MSM7X30)
	__raw_writel(1 << 6, MSM_GCC_BASE + 0x8);
#elif defined(CONFIG_ARCH_MSM8X60)
	__raw_writel(1 << 5, MSM_GCC_BASE + 0x8);
#else
	__raw_writel(1, MSM_CSR_BASE + 0x400 + (6) * 4);
#endif
}

#define APP_COMMAND 0x00
#define APP_STATUS  0x04
#define APP_DATA1   0x08
#define APP_DATA2   0x0C

#define MDM_COMMAND 0x10
#define MDM_STATUS  0x14
#define MDM_DATA1   0x18
#define MDM_DATA2   0x1C

static DEFINE_SPINLOCK(proc_comm_lock);

static int proc_comm_wait_for(unsigned addr, unsigned value)
{
	while (1) {
		/* Barrier here prevents excessive spinning */
		mb();
		if (readl_relaxed(addr) == value)
			return 0;

		if (smsm_check_for_modem_crash())
			return -EAGAIN;

		udelay(5);
	}
}

//Div2-SW2-BSP,JOE HSU,+++
int msm_proc_comm_oem(unsigned cmd, unsigned *data1, unsigned *data2, unsigned *cmd_parameter)
{
	unsigned base = (unsigned)MSM_SHARED_RAM_BASE;
	unsigned long flags;
	int ret;
	size_t sizeA, sizeB;
	smem_oem_cmd_data *cmd_buf;

         void* test;
	/* get share memory command address dynamically */
	int size;
	sizeA=40;
	sizeB=64;

	
	cmd_buf = smem_get_entry(SMEM_ID_VENDOR1, &size);
     //    test = cmd_buf+sizeof(unsigned int);

     /* read product id as serial number*/
     test= (unsigned*)&cmd_buf->cmd_data.cmd_parameter[0];

	spin_lock_irqsave(&proc_comm_lock, flags);

again:
	if (proc_comm_wait_for(base + MDM_STATUS, PCOM_READY))
		goto again;

	writel(cmd, base + APP_COMMAND);
	writel(data1 ? *data1 : 0, base + APP_DATA1);
	writel(data2 ? *data2 : 0, base + APP_DATA2);
	cmd_buf->cmd_data.check_flag = smem_oem_locked_flag;
	if(*data1==SMEM_PROC_COMM_OEM_PRODUCT_ID_WRITE)
	{
		//memcpy(cmd_buf->cmd_data.cmd_parameter[0],cmd_parameter,40);
	memcpy(test,(const void *)cmd_parameter,sizeA);

	}else if(*data1==SMEM_PROC_COMM_OEM_TEST_FLAG_WRITE)
	{
		//memcpy(cmd_buf->cmd_data.cmd_parameter[0],cmd_parameter,64);
	memcpy(test,(const void *)cmd_parameter,sizeB);
	}else if(*data1==SMEM_PROC_COMM_OEM_POWER_OFF)
	{
	    memcpy(test,(const void *)cmd_parameter, SMEM_OEM_CMD_BUF_SIZE*sizeof(int));
	}else if(*data1==SMEM_PROC_COMM_OEM_WRITE_FUSE)
	{
		cmd_buf->cmd_data.cmd_parameter[0] = cmd_parameter[0];
		cmd_buf->cmd_data.cmd_parameter[1] = cmd_parameter[1];
	}else if(*data1==SMEM_PROC_COMM_OEM_PM_SET_LED_INTENSITY)
	{
		cmd_buf->cmd_data.cmd_parameter[0] = cmd_parameter[0];
		cmd_buf->cmd_data.cmd_parameter[1] = cmd_parameter[1];
	}else if(*data1==SMEM_PROC_COMM_OEM_GPS_CN_TEST)
	{
		cmd_buf->cmd_data.cmd_parameter[0] = cmd_parameter[0];//GPS mode.
		cmd_buf->cmd_data.cmd_parameter[1] = cmd_parameter[1];//GPS satellite ID
		printk("GPS mode=%d,gps satellite=%d\n",cmd_buf->cmd_data.cmd_parameter[0],cmd_buf->cmd_data.cmd_parameter[1]);
	}
	else
	{
		// Set the parameter of OEM_CMD1
		cmd_buf->cmd_data.cmd_parameter[0] = cmd_parameter[0];
	}
	notify_other_proc_comm();

	if (proc_comm_wait_for(base + APP_COMMAND, PCOM_CMD_DONE))
		goto again;
	#if 0
	if (readl(base + APP_STATUS) == PCOM_CMD_SUCCESS) {
		if (data1)
			*data1 = readl(base + APP_DATA1);
		if (data2)
			*data2 = readl(base + APP_DATA2);
		ret = 0;
	} else {
		ret = -EIO;
	}
	#endif
	writel(PCOM_CMD_IDLE, base + APP_COMMAND);

	//spin_unlock_irqrestore(&proc_comm_lock, flags);


	/* read response value, Hanson Lin */
	while(!(cmd_buf->return_data.check_flag & smem_oem_unlocked_flag))
	{
		//waiting
	}
	ret = (cmd_buf->return_data.check_flag & 0x1111);
	//if(ret)
		//return ret;	
	if(!ret)
	{
		if(*data1==SMEM_PROC_COMM_OEM_PRODUCT_ID_READ)
		{
			//memcpy(data2,cmd_buf->return_data.return_value[0],40);
		memcpy((void *)data2, test,sizeA);
		
		}else if(*data1==SMEM_PROC_COMM_OEM_TEST_FLAG_READ)
		{
			//memcpy(data2,cmd_buf->return_data.return_value[0],64);
		memcpy((void *)data2,test,sizeB);
		/* FIH, WilsonWHLee, 2009/11/19 { */
        /* [FXX_CR], add for download tool */ 
		}else if(*data1==SMEM_PROC_COMM_OEM_NV_READ)
		{
			memcpy(data2,&cmd_buf->return_data.return_value[0],128); //WilsonWHLee, 2010/08/12 extend length for pc tool
		  //*test = cmd_buf->return_data.return_value;
		 // memcpy((void *)data2,test,32);
	    /* }FIH:WilsonWHLee 2009/11/19 */
	    }else if(*data1 == SMEM_PROC_COMM_OEM_GET_SYSTEM_TIME || *data1 == SMEM_PROC_COMM_OEM_GET_RTC_ALARM)
   	    {
            pm_rtc_julian_type * p_rtc = (pm_rtc_julian_type *) cmd_parameter;

            p_rtc->year = (unsigned short)cmd_buf->return_data.return_value[0] & 0x0000FFFF;
            p_rtc->month = (unsigned short)(cmd_buf->return_data.return_value[0]>>16);
            p_rtc->day = (unsigned short)cmd_buf->return_data.return_value[1] & 0x0000FFFF;
       	    p_rtc->hour = (unsigned short)(cmd_buf->return_data.return_value[1]>>16);
       	    p_rtc->minute = (unsigned short)cmd_buf->return_data.return_value[2] & 0x0000FFFF;
            p_rtc->second = (unsigned short)(cmd_buf->return_data.return_value[2]>>16);
		}else if(*data1==SMEM_PROC_COMM_OEM_GPS_CN_TEST)
		{
			printk("GPS return value cmd_buf->return_data.return_value[0]=%d\n",cmd_buf->return_data.return_value[0]);
			if (data2!=NULL)
				*data2=cmd_buf->return_data.return_value[0];//GPS CN value
		}
		else{
			*data2 = cmd_buf->return_data.return_value[0];
		}
	}else{
		/* Get the error code */
		if(*data1==SMEM_PROC_COMM_OEM_OTP_PROCESS)
			memcpy(data2,&cmd_buf->return_data.return_value[0],128);
	}
	
	
	//*data2 = cmd_buf->return_data.return_value[0];
	spin_unlock_irqrestore(&proc_comm_lock, flags);
	return ret;
	/* read response value, Hanson Lin */	
}
EXPORT_SYMBOL(msm_proc_comm_oem);

int proc_comm_set_led(unsigned id, unsigned level)
{
    unsigned led_parameter[2];
	unsigned smem_response;
	uint32_t oem_cmd = SMEM_PROC_COMM_OEM_PM_SET_LED_INTENSITY;
    led_parameter[0] = id;   
    led_parameter[1] = level;
	if(msm_proc_comm_oem(PCOM_CUSTOMER_CMD1, &oem_cmd, &smem_response, (unsigned *)led_parameter))
		return -1;
	return smem_response;
}
EXPORT_SYMBOL(proc_comm_set_led);


int proc_comm_req_RPLI(void)
{
	unsigned led_parameter[2];
	unsigned smem_response = 0;
	uint32_t oem_cmd = SMEM_PROC_COMM_OEM_PRIL_CHANGE_REQ;

	if(msm_proc_comm_oem(PCOM_CUSTOMER_CMD1, &oem_cmd, &smem_response, (unsigned *)led_parameter))
	return -1;
	return smem_response;
}
EXPORT_SYMBOL(proc_comm_req_RPLI);


int proc_comm_verify_result(int cmd)
{
	unsigned led_parameter[2];
	unsigned smem_response;
	uint32_t oem_cmd = SMEM_PROC_COMM_OEM_PRIL_VERIFY_RESULT;

	led_parameter[0] = cmd;
	led_parameter[1] = 0;

	if(msm_proc_comm_oem(PCOM_CUSTOMER_CMD1, &oem_cmd, &smem_response, (unsigned *)led_parameter))
	return -1;
	return smem_response;
}
EXPORT_SYMBOL(proc_comm_verify_result);

//FIHTDC-DerrickDRLiu-DbgCfgTool-00+[
int msm_proc_comm_oem_n(unsigned cmd, unsigned *data1, unsigned *data2, unsigned *cmd_parameter, int u32_para_size)
{
	unsigned base = (unsigned)MSM_SHARED_RAM_BASE;
	unsigned long flags;
	int ret;
	size_t sizeA, sizeB;
	smem_oem_cmd_data *cmd_buf;

         void* test;
	/* get share memory command address dynamically */
	int size;
	sizeA=40;
	sizeB=64;

	cmd_buf = smem_get_entry(SMEM_ID_VENDOR1, &size);

	if( (cmd_buf == 0) || (size < u32_para_size) )
	{
		printk(KERN_ERR "[SMEM_PROC_COMM] %s() LINE:%d, Can't get shared memory entry.(size %d,u32_para_size %d)\n", __func__, __LINE__, size, u32_para_size); 
		return -EINVAL;
	}

	test= (unsigned*)&cmd_buf->cmd_data.cmd_parameter[0];

	if( (cmd_parameter == NULL) || (u32_para_size == 0) )
	{
		printk(KERN_ERR "[SMEM_PROC_COMM] %s() LINE:%d, ERROR: u32_para_size %d.\n", __func__, __LINE__, u32_para_size); 
		return -EINVAL;
	}

	spin_lock_irqsave(&proc_comm_lock, flags);

again:
	
	if (proc_comm_wait_for(base + MDM_STATUS, PCOM_READY))
		goto again;

	writel(cmd, base + APP_COMMAND);
	writel(data1 ? *data1 : 0, base + APP_DATA1);
	writel(data2 ? *data2 : 0, base + APP_DATA2);
	cmd_buf->cmd_data.check_flag = smem_oem_locked_flag;
	memcpy(cmd_buf->cmd_data.cmd_parameter,cmd_parameter,sizeof(unsigned)*u32_para_size);
	notify_other_proc_comm();
	
	if (proc_comm_wait_for(base + APP_COMMAND, PCOM_CMD_DONE))
	{
		goto again;
	}
	
	writel(PCOM_CMD_IDLE, base + APP_COMMAND);
	
	/* read response value, Hanson Lin */
	while(!(cmd_buf->return_data.check_flag & smem_oem_unlocked_flag))
	{
		//waiting
	}
	
	/* Div6-D1-SY-FIHDBG-00*{ 
	 * Due to review return value in AMSS, we decide to modify the mask 
	 * from 0x1111 to 0xFFFF to identify the correct error type.
	 * Notice: Need to review the "check_flag" usage in both mARM and aARM.
	 */	
	ret = (cmd_buf->return_data.check_flag & 0xFFFF);
	/* Div6-D1-SY-FIHDBG-00*} */

	if(!ret)
	{
		*data2 = cmd_buf->return_data.return_value[0];

		/* Copy the returned value back to user "cmd_parameter" */
		memcpy(cmd_parameter, cmd_buf->return_data.return_value, sizeof(unsigned) * u32_para_size);
	}
	
	spin_unlock_irqrestore(&proc_comm_lock, flags);
	return ret;
}
EXPORT_SYMBOL(msm_proc_comm_oem_n);
//FIHTDC-DerrickDRLiu-DbgCfgTool-00+]

// SW-PHONE-NT-FTM_SIMTest-01 +[
int32_t proc_comm_phone_getsimstatus(void)
{
    unsigned int oem_cmd = SMEM_PROC_COMM_OEM_SIM_STATE; 
    unsigned int smem_response = 0;
    unsigned int oem_parameter = 1;  // SIM ID

    msm_proc_comm_oem(PCOM_CUSTOMER_CMD1, &oem_cmd, &smem_response, &oem_parameter);

    printk(KERN_INFO "%s (%d) - value: %d\n", __func__, __LINE__, smem_response);

    return smem_response;

#if 0
    // SF6 SIM Test mechanism
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_GET_CARD_MODE;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[2];
    
    memset((void*)data, 0x00, sizeof(data));
    data[0] = 5;
    
    printk(KERN_INFO "%s before call msm_proc_comm_oem_multi\n", __func__);
    msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1,
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    1);

    return data[0];
#endif
}
EXPORT_SYMBOL(proc_comm_phone_getsimstatus);
// SW-PHONE-NT-FTM_SIMTest-01 +]


int msm_proc_comm_oem_tcp_filter(void *cmd_data, unsigned cmd_size)
{
	unsigned cmd = PCOM_CUSTOMER_CMD1;
	unsigned oem_cmd = SMEM_PROC_COMM_OEM_UPDATE_TCP_FILTER;
	//unsigned oem_resp;
	unsigned *data1 = &oem_cmd;
	unsigned *data2 = NULL;
	unsigned base = (unsigned)MSM_SHARED_RAM_BASE;
	unsigned long flags;
	int ret;
	smem_oem_cmd_data *cmd_buf;

	void* ptr;
	int size;

#if 1
	unsigned short *content = (unsigned short *)cmd_data;
	for(ret=0; ret<cmd_size/2; ret++) {
		printk(KERN_INFO "tcp filter [%d, %4x]\n", ret, *(content+ret));
	}
#endif

	cmd_buf = smem_get_entry(SMEM_ID_VENDOR1, &size);

	ptr = (unsigned*)&cmd_buf->cmd_data.cmd_parameter[0];

	spin_lock_irqsave(&proc_comm_lock, flags);

again:
	if (proc_comm_wait_for(base + MDM_STATUS, PCOM_READY))
		goto again;

	writel(cmd, base + APP_COMMAND);
	writel(data1 ? *data1 : 0, base + APP_DATA1);
	writel(data2 ? *data2 : 0, base + APP_DATA2);
	cmd_buf->cmd_data.check_flag = smem_oem_locked_flag;
	{
		memcpy(ptr,(const void *)cmd_data,cmd_size);
	}
	notify_other_proc_comm();

	if (proc_comm_wait_for(base + APP_COMMAND, PCOM_CMD_DONE))
		goto again;

	writel(PCOM_CMD_IDLE, base + APP_COMMAND);

	/* read response value, Hanson Lin */
	while(!(cmd_buf->return_data.check_flag & smem_oem_unlocked_flag))
	{
		//waiting
	}
	ret = (cmd_buf->return_data.check_flag & 0x1111);

	spin_unlock_irqrestore(&proc_comm_lock, flags);

	if(ret) {
		printk(KERN_ERR "msm_proc_comm_oem_tcp_filter() returns %d\n", ret);
	}
	
	return ret;
}
EXPORT_SYMBOL(msm_proc_comm_oem_tcp_filter);

int msm_proc_comm_oem_for_nv(unsigned cmd, unsigned *data1, unsigned *data2, unsigned *cmd_parameter)
{
	unsigned base = (unsigned)MSM_SHARED_RAM_BASE;
	unsigned long flags;
	int ret;
	size_t sizeA, sizeB;
	smem_oem_cmd_data *cmd_buf;

         void* test;
	/* get share memory command address dynamically */
	int size;
	sizeA=40;
	sizeB=64;

	
	cmd_buf = smem_get_entry(SMEM_ID_VENDOR1, &size);
         test = cmd_buf+sizeof(unsigned int);
	spin_lock_irqsave(&proc_comm_lock, flags);

again:
	if (proc_comm_wait_for(base + MDM_STATUS, PCOM_READY))
		goto again;

	writel(cmd, base + APP_COMMAND);
	writel(data1 ? *data1 : 0, base + APP_DATA1);
	writel(data2 ? *data2 : 0, base + APP_DATA2);
	cmd_buf->cmd_data.check_flag = smem_oem_locked_flag;
	if(*data1==SMEM_PROC_COMM_OEM_PRODUCT_ID_WRITE)
	{
		//memcpy(cmd_buf->cmd_data.cmd_parameter[0],cmd_parameter,40);
	memcpy(test,(const void *)cmd_parameter,sizeA);

	}else if(*data1==SMEM_PROC_COMM_OEM_TEST_FLAG_WRITE)
	{
		//memcpy(cmd_buf->cmd_data.cmd_parameter[0],cmd_parameter,64);
	memcpy(test,(const void *)cmd_parameter,sizeB);
	}else
	{
		// Set the parameter of OEM_CMD1
		cmd_buf->cmd_data.cmd_parameter[0] = cmd_parameter[0];
		cmd_buf->cmd_data.cmd_parameter[1] = cmd_parameter[1];  //Added for new touch calibration by Stanley		
		cmd_buf->cmd_data.cmd_parameter[2] = cmd_parameter[2];  //Added for new touch calibration by Stanley
	}
	notify_other_proc_comm();

	if (proc_comm_wait_for(base + APP_COMMAND, PCOM_CMD_DONE))
		goto again;
	#if 0
	if (readl(base + APP_STATUS) == PCOM_CMD_SUCCESS) {
		if (data1)
			*data1 = readl(base + APP_DATA1);
		if (data2)
			*data2 = readl(base + APP_DATA2);
		ret = 0;
	} else {
		ret = -EIO;
	}
	#endif
	writel(PCOM_CMD_IDLE, base + APP_COMMAND);

	//spin_unlock_irqrestore(&proc_comm_lock, flags);


	/* read response value, Hanson Lin */
	while(!(cmd_buf->return_data.check_flag & smem_oem_unlocked_flag))
	{
		//waiting
	}
	ret = (cmd_buf->return_data.check_flag & 0x1111);
	//if(ret)
		//return ret;	
	if(!ret)
	{
		if(*data1==SMEM_PROC_COMM_OEM_PRODUCT_ID_READ)
		{
			//memcpy(data2,cmd_buf->return_data.return_value[0],40);
		memcpy((void *)data2, test,sizeA);
		
		}else if(*data1==SMEM_PROC_COMM_OEM_TEST_FLAG_READ)
		{
			//memcpy(data2,cmd_buf->return_data.return_value[0],64);
		memcpy((void *)data2,test,sizeB);
		}else{
			*data2 = cmd_buf->return_data.return_value[0];
		}
		//Michael add for RSD tool+++
		if(*data1==SMEM_PROC_COMM_OEM_NV_READ)
			memcpy(cmd_parameter, cmd_buf->return_data.return_value, 16);  
		else
		//Michael add for RSD tool---
			memcpy(cmd_parameter, cmd_buf->return_data.return_value, 8);  //Added for new touch calibration by Stanley		
	}
	//*data2 = cmd_buf->return_data.return_value[0];
	spin_unlock_irqrestore(&proc_comm_lock, flags);
	return ret;
	/* read response value, Hanson Lin */	
}
EXPORT_SYMBOL(msm_proc_comm_oem_for_nv);

void proc_comm_read_PID_testFlag( uint32_t oem_cmd,unsigned* data )
{
	unsigned cmd_parameter=0;
	/* cmd_parameter setting */
	msm_proc_comm_oem(PCOM_CUSTOMER_CMD1, &oem_cmd, data, &cmd_parameter );

}
EXPORT_SYMBOL( proc_comm_read_PID_testFlag);

void proc_comm_write_PID_testFlag( uint32_t oem_cmd,unsigned* data )
{
	unsigned cmd_parameter=0;
	/* cmd_parameter setting */
	msm_proc_comm_oem(PCOM_CUSTOMER_CMD1, &oem_cmd, &cmd_parameter, data );// data input to last parameter.
}
EXPORT_SYMBOL( proc_comm_write_PID_testFlag);

void proc_comm_read_hook_Flag( uint32_t oem_cmd,unsigned* data )
{
	unsigned cmd_parameter=0;
	/* cmd_parameter setting */
	msm_proc_comm_oem(PCOM_CUSTOMER_CMD1, &oem_cmd, data, &cmd_parameter );

}
EXPORT_SYMBOL( proc_comm_read_hook_Flag);

void proc_comm_ftm_wlanaddr_write(char* buf)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_NV_WRITE;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    int32_t data[3];

	printk(KERN_INFO "proc_comm_ftm_wlanaddr_write() '0x%x:%x:%x:%x:%x:%x'\n", buf[5], buf[4], buf[3], buf[2], buf[1], buf[0]);
   
    memset((void*)data, 0x00, sizeof(data));
    data[0] = (uint32_t)4678;	// NV_WLAN_MAC_ADDRESS_I
	memcpy((void *)&data[1],buf,6);
    
    if (0 != msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1, 
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    3)) {
        printk(KERN_ERR
                "%s: SMEM_PROC_COMM_OEM_FIH FAILED!!\n",
                __func__);
    }
}
EXPORT_SYMBOL(proc_comm_ftm_wlanaddr_write);

int proc_comm_ftm_wlanaddr_read(char * buf)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_NV_READ;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    int32_t data[3] = {0};
    int32_t ret = 0;

    memset((void*)data, 0x00, sizeof(data));
    data[0] = (uint32_t)4678;	// NV_WLAN_MAC_ADDRESS_I

    ret = msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1, 
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    3);
     if (ret != 0) {
        printk(KERN_ERR
                "%s: SMEM_PROC_COMM_OEM_FIH FAILED!!\n",
                __func__);
	  return ret;
    }
    
    memcpy(buf, &data[1], sizeof(char)*6);
    if ((data[1] == 0) && (data[2] == 0)) ret = -1;
    printk(KERN_INFO "proc_comm_ftm_wlanaddr_read() '0x%x:%x:%x:%x:%x:%x'\n", buf[5], buf[4],buf[3], buf[2],buf[1], buf[0]);
    return ret;
}
EXPORT_SYMBOL(proc_comm_ftm_wlanaddr_read);

//FIHTDC-DerrickDRLiu-DbgCfgTool-00+[
/*--------------------------------------------------------------------------*
 * Function    : fih_read_fihdbg_config_nv
 *
 * Description :
 *     Read fih debug configuration settings from nv item (NV_FIHDBG_I).
 *
 * Parameters  :
 *     String of 16 bytes fih debug configuration setting array in 
 *     unsigned char.
 *
 * Return value: Integer
 *     Zero     - Successful
 *     Not zero - Fail
 *--------------------------------------------------------------------------*/
int fih_read_fihdbg_config_nv( unsigned char* fih_debug )
{
    unsigned smem_response;
    uint32_t oem_cmd = SMEM_PROC_COMM_OEM_NV_READ;
    unsigned int cmd_para[FIH_DEBUG_CMD_DATA_SIZE];
    int ret = 0;

    printk(KERN_INFO "[SMEM_PROC_COMM] %s() LINE:%d \n", __func__, __LINE__);
    if ( fih_debug == NULL )
    {
        printk(KERN_ERR "[SMEM_PROC_COMM] %s() LINE:%d, ERROR: fih_debug is NULL.\n", __func__, __LINE__); 
        ret = -EINVAL;
    }
    else
    {
        cmd_para[0] = NV_FIHDBG_I;
        ret = msm_proc_comm_oem_n(PCOM_CUSTOMER_CMD1, &oem_cmd, &smem_response, cmd_para, FIH_DEBUG_CMD_DATA_SIZE);
        if(ret == 0)
        {
            memcpy( fih_debug, &cmd_para[0], FIH_DEBUG_CFG_LEN );
        }
        else
        {
            printk(KERN_ERR "[SMEM_PROC_COMM] %s() LINE:%d, ERROR: ret %d, cmd_para[0] %d.\n", __func__, __LINE__, ret, cmd_para[0]); 
        }
    }
    return ret;
}
EXPORT_SYMBOL(fih_read_fihdbg_config_nv);
 
/*--------------------------------------------------------------------------*
 * Function    : fih_write_fihdbg_config_nv
 *
 * Description :
 *     Write fih debug configuration settings into nv item (NV_FIHDBG_I).
 *
 * Parameters  :
 *     String of 16 bytes fih debug configuration setting array in 
 *     unsigned char.
 *
 * Return value: Integer
 *     Zero     - Successful
 *     Not zero - Fail
 *--------------------------------------------------------------------------*/
int fih_write_fihdbg_config_nv( unsigned char* fih_debug )
{
    unsigned smem_response;
    uint32_t oem_cmd = SMEM_PROC_COMM_OEM_NV_WRITE;
    unsigned int cmd_para[FIH_DEBUG_CMD_DATA_SIZE];
    int ret = 0;

    printk(KERN_INFO "[SMEM_PROC_COMM] %s() LINE:%d \n", __func__, __LINE__);
    if ( fih_debug == NULL )
    {
        printk(KERN_ERR "[SMEM_PROC_COMM] %s() LINE:%d, ERROR: fih_debug is NULL.\n", __func__, __LINE__); 
        ret = -EINVAL;
    }
    else
    {
        cmd_para[0] = NV_FIHDBG_I;
        memcpy( &cmd_para[1], fih_debug, FIH_DEBUG_CFG_LEN );
        ret = msm_proc_comm_oem_n(PCOM_CUSTOMER_CMD1, &oem_cmd, &smem_response, cmd_para, FIH_DEBUG_CMD_DATA_SIZE);
        if(ret != 0)
        {
            printk(KERN_ERR "[SMEM_PROC_COMM] %s() LINE:%d, ERROR: ret %d, cmd_para[0] %d.\n", __func__, __LINE__, ret, cmd_para[0]); 
        }
    }
    return ret;
}
EXPORT_SYMBOL(fih_write_fihdbg_config_nv);

//FIHTDC-DerrickDRLiu-Modem_Debug_NV-00+[
/*--------------------------------------------------------------------------*
 * Function    : fih_write_modem_debug_nv
 *
 * Description :
 *     Write modem debug NV correspondingly to make sure valid RAM dump can be generated.
 *
 * NV Item	Name						AutoDownload Action	Reset Action	No Action
 * 905		NV_ERR_FATAL_OPTIONS_I		0					1			2
 * 4399		NV_DETECT_HW_RESET_I		1					0			0
 * 6470		NV_CACHE_WT_I				1					0			0
 *
 * Parameters  :
 *     Value of NV_ERR_FATAL_OPTIONS_I
 *
 * Return value: Integer
 *     Zero     - Successful
 *     Not zero - Fail
 *--------------------------------------------------------------------------*/
int fih_write_modem_debug_nv( int value )
{
    unsigned smem_response;
    uint32_t oem_cmd = SMEM_PROC_COMM_OEM_NV_WRITE;
    unsigned int cmd_para[FIH_DEBUG_CMD_DATA_SIZE];
    int ret = 0;

    cmd_para[0] = NV_ERR_FATAL_OPTIONS_I;
    cmd_para[1] = value;
    ret = msm_proc_comm_oem_n(PCOM_CUSTOMER_CMD1, &oem_cmd, &smem_response, cmd_para, sizeof(int));
    if(ret != 0)
    {
    	printk(KERN_ERR
            "%s: SMEM_PROC_COMM_OEM_FIH FAILED!!\n",
            __func__);
    }
    printk(KERN_INFO "fih_write_modem_debug_nv() 0x%x\n", value);
	
    return ret;
}
EXPORT_SYMBOL(fih_write_modem_debug_nv);
//FIHTDC-DerrickDRLiu-Modem_Debug_NV-00+]
//FIHTDC-DerrickDRLiu-DbgCfgTool-00+]

int msm_proc_comm_oem_multi(unsigned cmd, unsigned *data1, unsigned *data2, unsigned *cmd_parameter, int number)
{
    unsigned base = (unsigned)MSM_SHARED_RAM_BASE;
    unsigned long flags;
    unsigned int size;
    int ret, index;

    smem_oem_cmd_data *cmd_buf;
    
    cmd_buf = (smem_oem_cmd_data *)smem_get_entry(SMEM_ID_VENDOR1, &size);
    
    //printk(KERN_INFO "%s: 0x%08x\n", __func__, (unsigned)cmd_buf);

    spin_lock_irqsave(&proc_comm_lock, flags);

again:
    if (proc_comm_wait_for(base + MDM_STATUS, PCOM_READY))
        goto again;

    writel(cmd, base + APP_COMMAND);
    writel(data1 ? *data1 : 0, base + APP_DATA1);
    writel(data2 ? *data2 : 0, base + APP_DATA2);
                        
    // Set the parameter of OEM_CMD1
    cmd_buf->cmd_data.check_flag = smem_oem_locked_flag;
    for( index = 0 ; index < number ; index++)
        cmd_buf->cmd_data.cmd_parameter[index] = cmd_parameter[index];
                        
    notify_other_proc_comm();
                        
    if (proc_comm_wait_for(base + APP_COMMAND, PCOM_CMD_DONE))
        goto again;
                        
    if (readl(base + APP_STATUS) == PCOM_CMD_SUCCESS) {
        if (data1)
            *data1 = readl(base + APP_DATA1);
        if (data2)
            *data2 = readl(base + APP_DATA2);
        
        ret = 0;
    } else {
        ret = -EIO;
    }
                        
    for (index = 0; index < number; index++) {
        cmd_parameter[index] = cmd_buf->return_data.return_value[index];
    }

    writel(PCOM_CMD_IDLE, base + APP_COMMAND);
                        
    while(!(cmd_buf->return_data.check_flag & smem_oem_unlocked_flag)) {
        //waiting
        mdelay(100);
        printk(KERN_INFO "%s: wait...... 0x%04x\n", __func__, cmd_buf->return_data.check_flag);
    }

    ret = (cmd_buf->return_data.check_flag & 0x1111);
    if(!ret) {
        *data2 = cmd_buf->return_data.return_value[0];
    }

    spin_unlock_irqrestore(&proc_comm_lock, flags);
    return ret;
}
EXPORT_SYMBOL(msm_proc_comm_oem_multi);

void proc_comm_ftm_bdaddr_write(char* buf)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_NV_WRITE;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    int32_t data[3];

	printk(KERN_INFO "proc_comm_ftm_bdaddr_write() '0x%x:%x:%x:%x:%x:%x'\n", buf[5], buf[4], buf[3], buf[2], buf[1], buf[0]);
   
    memset((void*)data, 0x00, sizeof(data));
    data[0] = (uint32_t)447;	// NV ID IMEI NV_BD_ADDR_I
	memcpy((void *)&data[1],buf,6);
    
    if (0 != msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1, 
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    3)) {
        printk(KERN_ERR
                "%s: SMEM_PROC_COMM_OEM_FIH FAILED!!\n",
                __func__);
    }
}
EXPORT_SYMBOL(proc_comm_ftm_bdaddr_write);

void proc_comm_ftm_bdaddr_read(char * buf)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_NV_READ;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    int32_t data[3];

    memset((void*)data, 0x00, sizeof(data));
    data[0] = (uint32_t)447;	// NV ID IMEI NV_BD_ADDR_I
    
    if (0 != msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1, 
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    3)) {
        printk(KERN_ERR
                "%s: SMEM_PROC_COMM_OEM_FIH FAILED!!\n",
                __func__);
    }
    
    memcpy(buf, &data[1], sizeof(char)*6);
    
    printk(KERN_INFO "proc_comm_ftm_bdaddr_read() '0x%x:%x:%x:%x:%x:%x'\n", buf[5], buf[4],buf[3], buf[2],buf[1], buf[0]);
}
EXPORT_SYMBOL(proc_comm_ftm_bdaddr_read);

void proc_comm_ftm_imei_write(unsigned* buf)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_NV_WRITE;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[10];
    
    //
    char data1[15] = {0};
    memcpy(data1, buf, 15);
    
    memset((void*)data, 0x00, sizeof(data));
    data[0] = (uint32_t)550;	// NV ID IMEI
    //
    data[1] = 0x8;
    data[2] = ((data1[0] << 4) & 0xf0) | 0xa;
    
    data[3] = ((data1[2] << 4) & 0xf0) | (data1[1] & 0xf);
    data[4] = ((data1[4] << 4) & 0xf0) | (data1[3] & 0xf);
    data[5] = ((data1[6] << 4) & 0xf0) | (data1[5] & 0xf);
    data[6] = ((data1[8] << 4) & 0xf0) | (data1[7] & 0xf);
    data[7] = ((data1[10] << 4) & 0xf0) | (data1[9] & 0xf);
    data[8] = ((data1[12] << 4) & 0xf0) | (data1[11] & 0xf);
    data[9] = ((data1[14] << 4) & 0xf0) | (data1[13] & 0xf);
    
    printk(KERN_INFO "proc_comm_ftm_imei_write() '0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x'\n", data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9]);
    
    if (0 != msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1, 
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    10)) {
        printk(KERN_ERR
                "%s: SMEM_PROC_COMM_OEM_FIH FAILED!!\n",
                __func__);
    }
}
EXPORT_SYMBOL(proc_comm_ftm_imei_write);

void proc_comm_ftm_imei_read(unsigned* buf)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_NV_READ;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[10];
    char data1[15] = {0};
    
    memset((void*)data, 0x00, sizeof(data));
    data[0] = (uint32_t)550;	// NV ID IMEI
    
    if (0 != msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1, 
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    10)) {
        printk(KERN_ERR
                "%s: SMEM_PROC_COMM_OEM_FIH FAILED!!\n",
                __func__);
    }
    
    data1[0] = (data[2] & 0xf0) >> 4;
    data1[1] = data[3] & 0xf;
    data1[2] = (data[3] & 0xf0) >> 4;
    data1[3] = data[4] & 0xf;
    data1[4] = (data[4] & 0xf0) >> 4;
    data1[5] = data[5] & 0xf;
    data1[6] = (data[5] & 0xf0) >> 4;
    data1[7] = data[6] & 0xf;
    data1[8] = (data[6] & 0xf0) >> 4;
    data1[9] = data[7] & 0xf;
    data1[10] = (data[7] & 0xf0) >> 4;
    data1[11] = data[8] & 0xf;
    data1[12] = (data[8] & 0xf0) >> 4;
    data1[13] = data[9] & 0xf;
    data1[14] = (data[9] & 0xf0) >> 4;
    
    memcpy(buf, data1, 15);
    
    printk(KERN_INFO "proc_comm_ftm_imei_read() IMEI = '%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d'\n", data1[0], data1[1], data1[2], data1[3], data1[4], data1[5], data1[6], data1[7], data1[8], data1[9], data1[10], data1[11], data1[12], data1[13], data1[14]);
}
EXPORT_SYMBOL(proc_comm_ftm_imei_read);

int proc_vib_action(int32_t reg, int32_t val)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_VIB;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t  data[2];

    memset((void*)data, 0x00, sizeof(data));

	data[0] = reg;
	data[1] = val;
    
    if (0 != msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1, 
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    2)) {
        printk(KERN_ERR
                "%s: SMEM_PROC_COMM_OEM_FIH FAILED!!\n",
                __func__);
		return 0;
    }
       
    return 1;

}
EXPORT_SYMBOL(proc_vib_action);


// MTD_MM_YW_FTM_AUDIO_SMEM+[
int proc_audio_adie_write(int32_t reg, int32_t val)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_AUDIO_ADIE_WRITE;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t  data[2];

    memset((void*)data, 0x00, sizeof(data));

	data[0] = reg;
	data[1] = val;
    
    if (0 != msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1, 
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    2)) {
        printk(KERN_ERR
                "%s: SMEM_PROC_COMM_OEM_FIH FAILED!!\n",
                __func__);
		return 0;
    }
       
    return 1;
}
EXPORT_SYMBOL(proc_audio_adie_write);

int proc_audio_adie_read(int32_t reg, int32_t *val)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_AUDIO_ADIE_READ;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t  data[1];
    
    memset((void*)data, 0x00, sizeof(data));
	
	data[0] = reg;

    if (0 != msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1, 
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    1)) {
        printk(KERN_ERR
                "%s: SMEM_PROC_COMM_OEM_FIH FAILED!!\n",
                __func__);
		return 0;
    }
    
	memcpy(val, data, 1);
    
    return 1;
}
EXPORT_SYMBOL(proc_audio_adie_read);

int proc_audio_loopback(int enable, int path)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_AUDIO_LOOPBACK;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t  data[2];
    
    memset((void*)data, 0x00, sizeof(data));
	
	data[0] = enable;
	data[1] = path;

    if (0 != msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1, 
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    2)) {
        printk(KERN_ERR
                "%s: SMEM_PROC_COMM_OEM_FIH FAILED!!\n",
                __func__);
		return 0;
    }    
    
    return 1;
}
EXPORT_SYMBOL(proc_audio_loopback);
// MTD_MM_YW_FTM_AUDIO_SMEM+]

//Div2-SW2-BSP,JOE HSU,---

// DIV2-SW2-BSP-IRM-FTM-CHG +[
int proc_comm_config_chg_current(bool on, int curr)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_CONFIG_CHG_CURRENT;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[2];
    int32_t ret = 0;
    
    memset((void*)data, 0x00, sizeof(data));
    data[0] = (uint32_t)on;
    data[1] = curr;
    
    ret = msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1, 
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    2);
    if (ret != 0){
        printk(KERN_ERR
                "%s: SMEM_PROC_COMM_OEM_CONFIG_CHG_CURRENT FAILED!!\n",
                __func__);
    }
    
    return ret;
}
EXPORT_SYMBOL(proc_comm_config_chg_current);
// DIV2-SW2-BSP-IRM-FTM-CHG+]

// DIV2-SW2-BSP-IRM-FTM-BACKUP-BATTERY+[
int proc_comm_config_coin_cell(int vset, int *voltage, int status)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_CONFIG_COIN_CELL;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[2];
    int32_t ret = 0;

    memset((void*)data, 0x00, sizeof(data));
    data[0] = vset;
    //status = 0:off / 1:on
    data[1] = status;
    
    ret = msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1,
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    2); 
    if (ret != 0){
        printk(KERN_ERR 
                "%s: SMEM_PROC_COMM_OEM_CONFIG_COIN_CELL FAILED!!\n",
                __func__);
    }
    
    *voltage = data[0];
    
    return ret; //mV
}
EXPORT_SYMBOL(proc_comm_config_coin_cell);
// DIV2-SW2-BSP-IRM-FTM-BACKUP-BATTERY+]

/* Debbie add 2011/08/23 { */
// DIV2-SW2-BSP-IRM-BATTERY+[
int proc_comm_read_battery_thermal(void)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_BATTERY_INFORMATION;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[2] = {0};
    int32_t ret = 0;
    
    ret = msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1,
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    2); 
    if (ret != 0){
        printk(KERN_ERR 
                "%s: SMEM_PROC_COMM_OEM_BATTERY_INFORMATION FAILED!!\n",
                __func__);
    }
    
    return data[1]; //degree
}
EXPORT_SYMBOL(proc_comm_read_battery_thermal);
// DIV2-SW2-BSP-IRM-BATTERY+]
/* Debbie add 2011/08/23 } */

/* Debbie add 2011/08/23 { */
// DIV2-SW2-BSP-IRM-BATTERY+[
int proc_comm_read_battery_voltage(void)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_BATTERY_INFORMATION;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[2] = {0};
    int32_t ret = 0;
    
    ret = msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1,
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    2); 
    if (ret != 0){
        printk(KERN_ERR 
                "%s: SMEM_PROC_COMM_OEM_BATTERY_INFORMATION FAILED!!\n",
                __func__);
    }

    return data[0]; //mv
}
EXPORT_SYMBOL(proc_comm_read_battery_voltage);
// DIV2-SW2-BSP-IRM-BATTERY+]
/* Debbie add 2011/08/23 } */

/*FIH-NJ-BSP Jerry add for HV battery dection. 2011/09/27  ++{ */
int proc_comm_hv_battery_dection(unsigned batt_type )
{
    unsigned batt_parameter[2];
	unsigned smem_response;
	uint32_t oem_cmd = SMEM_PROC_COMM_OEM_HIGH_BATTERY_DETECTION;
    batt_parameter[0] = batt_type;
    batt_parameter[1] = 1;  //reserved.
    //if batt_type = 1, battery is 4.35V; batt_type = 0, battery is 4.2V
	if(msm_proc_comm_oem(PCOM_CUSTOMER_CMD1, &oem_cmd, &smem_response, (unsigned *)batt_parameter))
		return -1;
	return smem_response;
}
EXPORT_SYMBOL(proc_comm_hv_battery_dection);
/*FIH-NJ-BSP Jerry add for HV battery dection. 2011/09/27  }++ */

/* FIHTDC, Div2-SW2-BSP CHHsieh, NVBackup { */
int proc_comm_ftm_backup_unique_nv(void)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_BACKUP_UNIQUE_NV;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[2];
    int32_t ret = 0;
    
    memset((void*)data, 0x00, sizeof(data));
    data[0] = 1;
    
    ret = msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1, 
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    0);
    if (ret != 0){
        printk(KERN_ERR
                "%s: SMEM_PROC_COMM_OEM_BACKUP_UNIQUE_NV FAILED!!\n",
                __func__);
                
    	return ret;
    }
    
    printk(KERN_INFO "proc_comm_ftm_backup_unique_nv()\n");
    
    return ret;
}
EXPORT_SYMBOL(proc_comm_ftm_backup_unique_nv);

int proc_comm_ftm_nv_flag(void)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_BACKUP_NV_FLAG;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[2];
    int32_t ret = 0;
    
    memset((void*)data, 0x00, sizeof(data));
    data[0] = 1;
    
    ret = msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1, 
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    0);
    if (ret != 0){
        printk(KERN_ERR
                "%s: SMEM_PROC_COMM_OEM_BACKUP_NV_FLAG FAILED!!\n",
                __func__);
                
    	return ret;
    }
    
    printk(KERN_INFO "proc_comm_ftm_nv_flag()\n");
    
    return ret;
}
EXPORT_SYMBOL(proc_comm_ftm_nv_flag);
/* } FIHTDC, Div2-SW2-BSP CHHsieh, NVBackup */

int proc_comm_ftm_change_modem_lpm(void)
{
	uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
	uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_EBOOT_SLEEP_REQ;
	uint32_t smem_proc_comm_oem_data2   = 0; //useless
	
	int32_t data[2];
	int		ret = 0;
	
	memset((void*)data, 0x00, sizeof(data));
	data[0] = 0;
	
	printk(KERN_INFO "%s before call msm_proc_comm_oem_multi\n", __func__);
	
    ret = msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1,
    								&smem_proc_comm_oem_data1,
    								&smem_proc_comm_oem_data2,
									data,
									0);
	if (ret != 1){
		printk(KERN_ERR
				"%s : %d : SMEM_PROC_COMM_OEM_CHG_MODE FAILED!!\n",
				__func__, ret);
		
		return ret;
	}
    
    printk(KERN_INFO "proc_comm_ftm_change_modem_lpm()\n");
    
    return ret;
}
EXPORT_SYMBOL(proc_comm_ftm_change_modem_lpm);

/* FIHTDC, CHHsieh, FD1 NV programming in factory { */
void proc_comm_ftm_customer_pid_write(unsigned* buf)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_NV_WRITE;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[10];
    
    memset((void*)data, 0x00, sizeof(data));
    data[0] = (uint32_t)8039;
    data[1] = 0x0;
    
    memcpy(&data[2], buf, 32);
    
    if (0 != msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1, 
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    10)) {
        printk(KERN_ERR
                "%s: SMEM_PROC_COMM_OEM_FIH FAILED!!\n",
                __func__);
    }
    
    printk(KERN_INFO "proc_comm_ftm_customer_pid_write() '%x %x %x'\n", data[1], data[2], data[3]);
}
EXPORT_SYMBOL(proc_comm_ftm_customer_pid_write);

void proc_comm_ftm_customer_pid_read(unsigned* buf)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_NV_READ;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[9];
    
    memset((void*)data, 0x00, sizeof(data));
    data[0] = (uint32_t)8039;
    data[1] = 0x0;
    
    if (0 != msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1, 
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    9)) {
        printk(KERN_ERR
                "%s: SMEM_PROC_COMM_OEM_FIH FAILED!!\n",
                __func__);
    }
    
    memcpy(buf, &data[0], 32);
    
    printk(KERN_INFO "proc_comm_ftm_customer_pid_read() '%x %x %x'\n", data[1], data[2], data[3]);
}
EXPORT_SYMBOL(proc_comm_ftm_customer_pid_read);

void proc_comm_ftm_customer_pid2_write(unsigned* buf)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_NV_WRITE;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[10];
    
    memset((void*)data, 0x00, sizeof(data));
    data[0] = (uint32_t)8039;
    data[1] = 0x1;
    
    memcpy(&data[2], buf, 32);
    
    if (0 != msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1, 
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    10)) {
        printk(KERN_ERR
                "%s: SMEM_PROC_COMM_OEM_FIH FAILED!!\n",
                __func__);
    }
    
    printk(KERN_INFO "proc_comm_ftm_customer_pid2_write() '%x %x %x'\n", data[1], data[2], data[3]);
}
EXPORT_SYMBOL(proc_comm_ftm_customer_pid2_write);

void proc_comm_ftm_customer_pid2_read(unsigned* buf)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_NV_READ;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[9];
    
    memset((void*)data, 0x00, sizeof(data));
    data[0] = (uint32_t)8039;
    data[1] = 0x1;
    
    if (0 != msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1, 
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    9)) {
        printk(KERN_ERR
                "%s: SMEM_PROC_COMM_OEM_FIH FAILED!!\n",
                __func__);
    }
    
    memcpy(buf, &data[0], 32);
    
    printk(KERN_INFO "proc_comm_ftm_customer_pid2_read()\n");
}
EXPORT_SYMBOL(proc_comm_ftm_customer_pid2_read);

void proc_comm_ftm_customer_swid_write(unsigned* buf)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_NV_WRITE;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[9];
    
    memset((void*)data, 0x00, sizeof(data));
    data[0] = (uint32_t)8039;
    data[1] = 0x2;
    
    memcpy(&data[2], buf, 32);
    
    if (0 != msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1, 
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    10)) {
        printk(KERN_ERR
                "%s: SMEM_PROC_COMM_OEM_FIH FAILED!!\n",
                __func__);
    }
    
    printk(KERN_INFO "proc_comm_ftm_customer_swid_write() '%x %x %x'\n", data[1], data[2], data[3]);
}
EXPORT_SYMBOL(proc_comm_ftm_customer_swid_write);

void proc_comm_ftm_customer_swid_read(unsigned* buf)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_NV_READ;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[9];
    
    memset((void*)data, 0x00, sizeof(data));
    data[0] = (uint32_t)8039;
    data[1] = 0x2;
    
    if (0 != msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1, 
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    9)) {
        printk(KERN_ERR
                "%s: SMEM_PROC_COMM_OEM_FIH FAILED!!\n",
                __func__);
    }
    
    memcpy(buf, &data[0], 32);
    
    printk(KERN_INFO "proc_comm_ftm_customer_swid_read()\n");
}
EXPORT_SYMBOL(proc_comm_ftm_customer_swid_read);

void proc_comm_ftm_dom_write(unsigned* buf)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_NV_WRITE;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[6];
    
    memset((void*)data, 0x00, sizeof(data));
    data[0] = (uint32_t)8039;
    data[1] = 0x3;
    
    memcpy(&data[2], buf, 14);
    
    if (0 != msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1, 
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    6)) {
        printk(KERN_ERR
                "%s: SMEM_PROC_COMM_OEM_FIH FAILED!!\n",
                __func__);
    }
    
    printk(KERN_INFO "proc_comm_ftm_dom_write() '%x %x %x'\n", data[1], data[2], data[3]);
}
EXPORT_SYMBOL(proc_comm_ftm_dom_write);

void proc_comm_ftm_dom_read(unsigned* buf)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_NV_READ;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[5];
    
    memset((void*)data, 0x00, sizeof(data));
    data[0] = (uint32_t)8039;
    data[1] = 0x3;
    
    if (0 != msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1, 
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    5)) {
        printk(KERN_ERR
                "%s: SMEM_PROC_COMM_OEM_FIH FAILED!!\n",
                __func__);
    }
    
    memcpy(buf, &data[0], 14);
    
    printk(KERN_INFO "proc_comm_ftm_dom_read()\n");
}
EXPORT_SYMBOL(proc_comm_ftm_dom_read);
/* } FIHTDC, CHHsieh, FD1 NV programming in factory */

/* FIH, JiaHao, 2011/08/16 { */
int proc_comm_read_hwid_table(struct st_hwid_table *p_hwid_table)
{
	unsigned oem_cmd = SMEM_PROC_COMM_OEM_FIH_CFGDATA_HWID_READ;
	unsigned smem_response;
	unsigned cmd_parameter[32]; // 128 Byte
	int ret;

	ret = msm_proc_comm_oem_n(PCOM_CUSTOMER_CMD1, &oem_cmd, &smem_response, cmd_parameter, 32);
	if (0 == ret) {
		memcpy(p_hwid_table, cmd_parameter, sizeof(struct st_hwid_table));
	} else {
		printk(KERN_ERR "%s : %d : SMEM_PROC_COMM_OEM_FIH_CFGDATA_HWID_READ FAILED!!\n", __func__, ret);
	}

	return ret;
}
EXPORT_SYMBOL(proc_comm_read_hwid_table);
/* FIH, JiaHao, 2011/08/16 } */

/* FIH, CHHsieh, 2011/08/16 { */
/* Moto CFC requirement */
unsigned int proc_comm_reset_chip(unsigned *cmd_parameter)
{
	unsigned smem_response;
	//unsigned *cmd_parameter;
	uint32_t oem_cmd = SMEM_PROC_COMM_OEM_RESET_CHIP_EBOOT;
	
	msm_proc_comm_oem(PCOM_CUSTOMER_CMD1, &oem_cmd, &smem_response, cmd_parameter);
	return oem_cmd;
}

int msm_proc_comm_oem_rw_NV(unsigned cmd, unsigned *data1, unsigned *data2, unsigned *cmd_parameter)
{
	unsigned base = (unsigned)MSM_SHARED_RAM_BASE;
	unsigned long flags;
	int ret, size;
	smem_oem_cmd_data *cmd_buf;
	int i;

	cmd_buf = smem_get_entry(SMEM_ID_VENDOR1, &size);

	spin_lock_irqsave(&proc_comm_lock, flags);

again:
	if (proc_comm_wait_for(base + MDM_STATUS, PCOM_READY))
		goto again;

	writel(cmd, base + APP_COMMAND);
	writel(data1 ? *data1 : 0, base + APP_DATA1);
	writel(data2 ? *data2 : 0, base + APP_DATA2);
	
	cmd_buf->cmd_data.check_flag = smem_oem_locked_flag;

	/* Insert cmd_parameter to shared memory (fixed size 128 bytes) */
	memcpy(cmd_buf->cmd_data.cmd_parameter, cmd_parameter, 128);

	for(i=0; i<32; i++)
		printk("%s - %d: 0x%x\n", __func__, i, cmd_buf->cmd_data.cmd_parameter[i]);
	
	notify_other_proc_comm();

	if (proc_comm_wait_for(base + APP_COMMAND, PCOM_CMD_DONE))
		goto again;

	writel(PCOM_CMD_IDLE, base + APP_COMMAND);

	/* read response value, Hanson Lin */
	while(!(cmd_buf->return_data.check_flag & smem_oem_unlocked_flag))
	{
		//waiting
	}
	printk("flag(0x%x)\n", cmd_buf->return_data.check_flag);
	ret = (cmd_buf->return_data.check_flag & 0x1111);

	/* Return data from modem */
	if(!ret)
	{
		if(*data1 == SMEM_PROC_COMM_OEM_NV_READ)
			memcpy(data2, cmd_buf->return_data.return_value, 128);
		else if(*data1 == SMEM_PROC_COMM_OEM_PRODUCT_ID_READ)
			memcpy(data2, cmd_buf->return_data.return_value, 128);
	}

	spin_unlock_irqrestore(&proc_comm_lock, flags);
	return ret;
	/* read response value, Hanson Lin */	
}
EXPORT_SYMBOL(msm_proc_comm_oem_rw_NV);

int proc_comm_oem_smem_process(int *buf, int oem_data1)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = oem_data1;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[32];
    int32_t ret = 0;
    
    memset((void*)data, 0x00, sizeof(data));
    memcpy(&data[0], buf, sizeof(data));
    printk(KERN_INFO "proc_comm_oem_smem_process() Before '0x%08x 0x%08x 0x%08x'\n", data[0], data[1], data[2]);
    
    ret = msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1, 
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    32);
	
    memcpy(buf, &data[0], sizeof(data));
    printk(KERN_INFO "proc_comm_oem_smem_process() After '0x%08x 0x%08x 0x%08x'\n", buf[0], buf[1], buf[2]);
	
    if (ret != 0){
        printk(KERN_ERR
                "%s: SMEM_PROC_COMM_OEM_FUSE_BOOT FAILED!!\n",
                __func__);
                
    	return ret;
    }
    
    printk(KERN_INFO "proc_comm_oem_smem_process() pass\n");
    
    return ret;
}
EXPORT_SYMBOL(proc_comm_oem_smem_process);
/*} FIH, CHHsieh, 2011/08/16 */

/* FIH, CHHsieh, 2011/10/19 { */
/* DUT can not enter FTM mode after flash */
static uint32_t restart_reason = 0x77665504;
void proc_comm_pm_restart(void)
{
	// FIHTDC, DerrickDRLiu, give oem shared memory command to modem {
	uint32_t oem_cmd = SMEM_PROC_COMM_OEM_RESET_CHIP_EBOOT;
	uint32_t smem_response = 0;
	// } FIHTDC, DerrickDRLiu, give oem shared memory command to modem
	
	// FIHTDC, DerrickDRLiu, give oem shared memory command to modem {	
	printk(KERN_INFO "%s : msm_proc_comm_oem before\n", __func__);
	msm_proc_comm_oem(PCOM_CUSTOMER_CMD1, &oem_cmd, &smem_response, &restart_reason);
	//msm_rpcrouter_close();
	//msm_proc_comm(PCOM_RESET_CHIP, &restart_reason, 0);
	// } FIHTDC, DerrickDRLiu, give oem shared memory command to modem
	//FIH-DerrickDRLiu-DbgCfgTool-00+[
	//#ifdef CONFIG_FIH_LAST_ALOG	
	//     alog_ram_console_sync_time(LOG_TYPE_ALL, SYNC_BEFORE);
	//#endif	
	//FIH-DerrickDRLiu-DbgCfgTool-00+]
	for (;;)
		;
}
/* } FIH, CHHsieh, 2011/10/19 */

/* FIH, CHHsieh, 2011/10/21 { */
/* IRE ftm command W/R NV through smem command */
//SHA-1 algorithm
typedef struct SHA1Context
{
    unsigned Message_Digest[5]; /* Message Digest (output)          */

    unsigned Length_Low;        /* Message length in bits           */
    unsigned Length_High;       /* Message length in bits           */

    unsigned char Message_Block[64]; /* 512-bit message blocks      */
    int Message_Block_Index;    /* Index into message block array   */

    int Computed;               /* Is the digest computed?          */
    int Corrupted;              /* Is the message digest corruped?  */
} SHA1Context;

#define SHA1CircularShift(bits,word) \
                ((((word) << (bits)) & 0xFFFFFFFF) | \
                ((word) >> (32-(bits))))

void SHA1ProcessMessageBlock(SHA1Context *);
void SHA1PadMessage(SHA1Context *);

void SHA1Reset(SHA1Context *context)
{
    context->Length_Low             = 0;
    context->Length_High            = 0;
    context->Message_Block_Index    = 0;

    context->Message_Digest[0]      = 0x67452301;
    context->Message_Digest[1]      = 0xEFCDAB89;
    context->Message_Digest[2]      = 0x98BADCFE;
    context->Message_Digest[3]      = 0x10325476;
    context->Message_Digest[4]      = 0xC3D2E1F0;

    context->Computed   = 0;
    context->Corrupted  = 0;
}

int SHA1Result(SHA1Context *context)
{

    if (context->Corrupted)
    {
        return 0;
    }

    if (!context->Computed)
    {
        SHA1PadMessage(context);
        context->Computed = 1;
    }

    return 1;
}

void SHA1Input(     SHA1Context         *context,
                    const unsigned char *message_array,
                    unsigned            length)
{
    if (!length)
    {
        return;
    }

    if (context->Computed || context->Corrupted)
    {
        context->Corrupted = 1;
        return;
    }

    while(length-- && !context->Corrupted)
    {
        context->Message_Block[context->Message_Block_Index++] =
                                                (*message_array & 0xFF);

        context->Length_Low += 8;
        /* Force it to 32 bits */
        context->Length_Low &= 0xFFFFFFFF;
        if (context->Length_Low == 0)
        {
            context->Length_High++;
            /* Force it to 32 bits */
            context->Length_High &= 0xFFFFFFFF;
            if (context->Length_High == 0)
            {
                /* Message is too long */
                context->Corrupted = 1;
            }
        }

        if (context->Message_Block_Index == 64)
        {
            SHA1ProcessMessageBlock(context);
        }

        message_array++;
    }
}

void SHA1ProcessMessageBlock(SHA1Context *context)
{
    const unsigned K[] =            /* Constants defined in SHA-1   */      
    {
        0x5A827999,
        0x6ED9EBA1,
        0x8F1BBCDC,
        0xCA62C1D6
    };
    int         t;                  /* Loop counter                 */
    unsigned    temp;               /* Temporary word value         */
    unsigned    W[80];              /* Word sequence                */
    unsigned    A, B, C, D, E;      /* Word buffers                 */

    /*
     *  Initialize the first 16 words in the array W
     */
    for(t = 0; t < 16; t++)
    {
        W[t] = ((unsigned) context->Message_Block[t * 4]) << 24;
        W[t] |= ((unsigned) context->Message_Block[t * 4 + 1]) << 16;
        W[t] |= ((unsigned) context->Message_Block[t * 4 + 2]) << 8;
        W[t] |= ((unsigned) context->Message_Block[t * 4 + 3]);
    }

    for(t = 16; t < 80; t++)
    {
       W[t] = SHA1CircularShift(1,W[t-3] ^ W[t-8] ^ W[t-14] ^ W[t-16]);
    }

    A = context->Message_Digest[0];
    B = context->Message_Digest[1];
    C = context->Message_Digest[2];
    D = context->Message_Digest[3];
    E = context->Message_Digest[4];

    for(t = 0; t < 20; t++)
    {
        temp =  SHA1CircularShift(5,A) +
                ((B & C) | ((~B) & D)) + E + W[t] + K[0];
        temp &= 0xFFFFFFFF;
        E = D;
        D = C;
        C = SHA1CircularShift(30,B);
        B = A;
        A = temp;
    }

    for(t = 20; t < 40; t++)
    {
        temp = SHA1CircularShift(5,A) + (B ^ C ^ D) + E + W[t] + K[1];
        temp &= 0xFFFFFFFF;
        E = D;
        D = C;
        C = SHA1CircularShift(30,B);
        B = A;
        A = temp;
    }

    for(t = 40; t < 60; t++)
    {
        temp = SHA1CircularShift(5,A) +
               ((B & C) | (B & D) | (C & D)) + E + W[t] + K[2];
        temp &= 0xFFFFFFFF;
        E = D;
        D = C;
        C = SHA1CircularShift(30,B);
        B = A;
        A = temp;
    }

    for(t = 60; t < 80; t++)
    {
        temp = SHA1CircularShift(5,A) + (B ^ C ^ D) + E + W[t] + K[3];
        temp &= 0xFFFFFFFF;
        E = D;
        D = C;
        C = SHA1CircularShift(30,B);
        B = A;
        A = temp;
    }

    context->Message_Digest[0] =
                        (context->Message_Digest[0] + A) & 0xFFFFFFFF;
    context->Message_Digest[1] =
                        (context->Message_Digest[1] + B) & 0xFFFFFFFF;
    context->Message_Digest[2] =
                        (context->Message_Digest[2] + C) & 0xFFFFFFFF;
    context->Message_Digest[3] =
                        (context->Message_Digest[3] + D) & 0xFFFFFFFF;
    context->Message_Digest[4] =
                        (context->Message_Digest[4] + E) & 0xFFFFFFFF;

    context->Message_Block_Index = 0;
}

void SHA1PadMessage(SHA1Context *context)
{
    /*
     *  Check to see if the current message block is too small to hold
     *  the initial padding bits and length.  If so, we will pad the
     *  block, process it, and then continue padding into a second
     *  block.
     */
    if (context->Message_Block_Index > 55)
    {
        context->Message_Block[context->Message_Block_Index++] = 0x80;
        while(context->Message_Block_Index < 64)
        {
            context->Message_Block[context->Message_Block_Index++] = 0;
        }

        SHA1ProcessMessageBlock(context);

        while(context->Message_Block_Index < 56)
        {
            context->Message_Block[context->Message_Block_Index++] = 0;
        }
    }
    else
    {
        context->Message_Block[context->Message_Block_Index++] = 0x80;
        while(context->Message_Block_Index < 56)
        {
            context->Message_Block[context->Message_Block_Index++] = 0;
        }
    }

    /*
     *  Store the message length as the last 8 octets
     */
    context->Message_Block[56] = (context->Length_High >> 24) & 0xFF;
    context->Message_Block[57] = (context->Length_High >> 16) & 0xFF;
    context->Message_Block[58] = (context->Length_High >> 8) & 0xFF;
    context->Message_Block[59] = (context->Length_High) & 0xFF;
    context->Message_Block[60] = (context->Length_Low >> 24) & 0xFF;
    context->Message_Block[61] = (context->Length_Low >> 16) & 0xFF;
    context->Message_Block[62] = (context->Length_Low >> 8) & 0xFF;
    context->Message_Block[63] = (context->Length_Low) & 0xFF;

    SHA1ProcessMessageBlock(context);
}

int proc_comm_ftm_meid_write(char* buf)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_NV_WRITE;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[4];
    int32_t ret = 0;
    char data1[7] = {0};
    SHA1Context sha;
    
    memcpy(data1, buf, 7);
    
    memset((void*)data, 0x00000000, sizeof(data));
    data[0] = (uint32_t)1943;	// NV ID IMEI
    //
    data[1] = ( ((data1[3] << 24) & 0xff000000) | ((data1[4] << 16) & 0xff0000) |
    			((data1[5] <<  8) & 0xff00)     |  (data1[6]        & 0xff) );
    data[2] = ( ((data1[0] << 16) & 0xff0000)   | ((data1[1] <<  8) & 0xff00) |
    			 (data1[2]        & 0xff) );
    
    
    SHA1Reset(&sha);
    SHA1Input(&sha, (const unsigned char *) buf, 7);

    if (!SHA1Result(&sha))
    {
        printk(KERN_ERR "%s: ERROR-- could not compute message digest\n", __func__);
    	
    	return 1;
    }
    else
    {
        data[3] = ( 0x80000000 | ( sha.Message_Digest[4] & 0x00ffffff ) );
    }
    
    printk(KERN_INFO "proc_comm_ftm_meid_write() '0x%08x, %08x'\n",
    											data[2], data[3]);
    
    if (ret != msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1,
    								&smem_proc_comm_oem_data1,
    								&smem_proc_comm_oem_data2,
    								data,
    								4)) {
        printk(KERN_ERR
        		"%s: SMEM_PROC_COMM_OEM_FIH FAILED!!\n",
                __func__);
    	
    	return ret;
    }
    
    printk(KERN_INFO "proc_comm_ftm_meid_write() success\n");
    
    return ret;
}
EXPORT_SYMBOL(proc_comm_ftm_meid_write);

int proc_comm_ftm_meid_read(unsigned* buf)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_NV_READ;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[3];
    int32_t ret = 0;
    
    char data1[14] = {0};
    

    memset((void*)data, 0x00000000, sizeof(data));
    data[0] = (uint32_t)1943;	// NV ID MEID
    
    if (ret != msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1, 
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    3)) {
        printk(KERN_ERR
                "%s: SMEM_PROC_COMM_OEM_FIH FAILED!!\n",
                __func__);
    	
    	return ret;
    }
    
    printk(KERN_ERR " meid = %08x   %08x",data[1],data[2]);
    
    data1[ 6] = ( data[1] & 0xf0000000 ) >> 28;
    data1[ 7] = ( data[1] & 0xf000000 )  >> 24;
    data1[ 8] = ( data[1] & 0xf00000 )   >> 20;
    data1[ 9] = ( data[1] & 0xf0000 )    >> 16;
    data1[10] = ( data[1] & 0xf000 )     >> 12;
    data1[11] = ( data[1] & 0xf00 )      >>  8;
    data1[12] = ( data[1] & 0xf0 )       >>  4;
    data1[13] = ( data[1] & 0xf );
    data1[ 0] = ( data[2] & 0xf00000 ) >> 20;
    data1[ 1] = ( data[2] & 0xf0000 )  >> 16;
    data1[ 2] = ( data[2] & 0xf000 )   >> 12;
    data1[ 3] = ( data[2] & 0xf00 )    >>  8;
    data1[ 4] = ( data[2] & 0xf0 )     >>  4;
    data1[ 5] = ( data[2] & 0xf );
    
    memcpy(buf, data1, 14);
    
    printk(KERN_INFO "proc_comm_ftm_meid_read() MEID = '0x%x,%x,%x,%x,%x,%x,%x,%x, %x, %x, %x, %x, %x, %x'\n",
    										data1[ 0], data1[ 1], data1[ 2], data1[ 3], data1[ 4],
    										data1[ 5], data1[ 6], data1[ 7], data1[ 8], data1[ 9],
    										data1[10], data1[11], data1[12], data1[13]);
    										
    return ret;
}
EXPORT_SYMBOL(proc_comm_ftm_meid_read);

int proc_comm_ftm_esn_read(unsigned* buf)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_NV_READ;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[2];
    int32_t ret = 0;
    
    char data1[8] = {0};
    

    memset((void*)data, 0x00000000, sizeof(data));
    data[0] = (uint32_t)0;	// NV ID MEID
    
    if (ret != msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1, 
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    3)) {
        printk(KERN_ERR
                "%s: SMEM_PROC_COMM_OEM_FIH FAILED!!\n",
                __func__);
    	
    	return ret;
    }
    
    printk(KERN_ERR " esn = %08x",data[1]);
    
    data1[ 0] = ( data[1] & 0xf0000000 ) >> 28;
    data1[ 1] = ( data[1] & 0xf000000 )  >> 24;
    data1[ 2] = ( data[1] & 0xf00000 )   >> 20;
    data1[ 3] = ( data[1] & 0xf0000 )    >> 16;
    data1[ 4] = ( data[1] & 0xf000 )     >> 12;
    data1[ 5] = ( data[1] & 0xf00 )      >>  8;
    data1[ 6] = ( data[1] & 0xf0 )       >>  4;
    data1[ 7] = ( data[1] & 0xf );
    
    memcpy(buf, data1, 8);
    
    printk(KERN_INFO "proc_comm_ftm_esn_read() ESN = '0x%x,%x,%x,%x,%x,%x,%x,%x\n",
    										data1[0], data1[1], data1[2], data1[3],
    										data1[4], data1[5], data1[6], data1[7]);
    										
    return ret;
}
EXPORT_SYMBOL(proc_comm_ftm_esn_read);

int proc_comm_ftm_akey1_write(unsigned* buf)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_NV_WRITE;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[4];
    int32_t ret = 0;
    
    char data1[16] = {0};
    
    memcpy(data1, buf, 16);
    
    memset((void*)data, 0x00000000, sizeof(data));
    data[0] = (uint32_t)25;	// NV ID IMEI
    
    data[1] = 0x0;
    data[2] = ( ((data1[ 8] << 28) & 0xf0000000) | ((data1[ 9] << 24) & 0xf000000) |
    			((data1[10] << 20) & 0xf00000)	 | ((data1[11] << 16) & 0xf0000) |
    			((data1[12] << 12) & 0xf000)     | ((data1[13] <<  8) & 0xf00) |
    			((data1[14] <<  4) & 0xf0)       |  (data1[15]        & 0xf) );
    data[3] = ( ((data1[ 0] << 28) & 0xf0000000) | ((data1[ 1] << 24) & 0xf000000) |
    			((data1[ 2] << 20) & 0xf00000)	 | ((data1[ 3] << 16) & 0xf0000) |
    			((data1[ 4] << 12) & 0xf000)     | ((data1[ 5] <<  8) & 0xf00) |
    			((data1[ 6] <<  4) & 0xf0)       |  (data1[ 7]        & 0xf) );
    
    printk(KERN_INFO "proc_comm_ftm_akey1_write() '0x%08x, %08x'\n",
    											data[2], data[3]);
    
    if (ret != msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1,
    								&smem_proc_comm_oem_data1,
    								&smem_proc_comm_oem_data2,
    								data,
    								4)) {
        printk(KERN_ERR
        		"%s: SMEM_PROC_COMM_OEM_FIH FAILED!!\n",
                __func__);
    	
    	return ret;
    }
    
    printk(KERN_INFO "proc_comm_ftm_akey1_write() success\n");
    
    return ret;
}
EXPORT_SYMBOL(proc_comm_ftm_akey1_write);

int proc_comm_ftm_akey2_write(unsigned* buf)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_NV_WRITE;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[4];
    int32_t ret = 0;
    
    char data1[16] = {0};
    
    memcpy(data1, buf, 16);
    
    memset((void*)data, 0x00000000, sizeof(data));
    data[0] = (uint32_t)25;	// NV ID IMEI
    
    data[1] = 0x1;
    data[2] = ( ((data1[ 8] << 28) & 0xf0000000) | ((data1[ 9] << 24) & 0xf000000) |
    			((data1[10] << 20) & 0xf00000)	 | ((data1[11] << 16) & 0xf0000) |
    			((data1[12] << 12) & 0xf000)     | ((data1[13] <<  8) & 0xf00) |
    			((data1[14] <<  4) & 0xf0)       |  (data1[15]        & 0xf) );
    data[3] = ( ((data1[ 0] << 28) & 0xf0000000) | ((data1[ 1] << 24) & 0xf000000) |
    			((data1[ 2] << 20) & 0xf00000)	 | ((data1[ 3] << 16) & 0xf0000) |
    			((data1[ 4] << 12) & 0xf000)     | ((data1[ 5] <<  8) & 0xf00) |
    			((data1[ 6] <<  4) & 0xf0)       |  (data1[ 7]        & 0xf) );
    
    printk(KERN_INFO "proc_comm_ftm_akey2_write() '0x%08x, %08x'\n",
    											data[2], data[3]);
    
    if (ret != msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1,
    								&smem_proc_comm_oem_data1,
    								&smem_proc_comm_oem_data2,
    								data,
    								4)) {
        printk(KERN_ERR
        		"%s: SMEM_PROC_COMM_OEM_FIH FAILED!!\n",
                __func__);
    	
    	return ret;
    }
    
    printk(KERN_INFO "proc_comm_ftm_akey2_write() success\n");
    
    return ret;
}
EXPORT_SYMBOL(proc_comm_ftm_akey2_write);

int proc_comm_ftm_akey1_read(char* buf)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_NV_READ;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[4];
    int32_t ret = 0;
    
    char data1[16] = {0};
    

    memset((void*)data, 0x00000000, sizeof(data));
    data[0] = (uint32_t)25;	// NV ID IMEI
    
    data[1] = 0x0;
    
    if (ret != msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1, 
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    4)) {
        printk(KERN_ERR
                "%s: SMEM_PROC_COMM_OEM_FIH FAILED!!\n",
                __func__);
    	
    	return ret;
    }
    
    printk(KERN_ERR " Akey1 = %08x, %08x, %08x, %08x", data[0], data[1], data[2], data[3]);
    
    data1[ 8] = ( data[2] & 0xf0000000 ) >> 28;
    data1[ 9] = ( data[2] & 0xf000000 )  >> 24;
    data1[10] = ( data[2] & 0xf00000 )   >> 20;
    data1[11] = ( data[2] & 0xf0000 )    >> 16;
    data1[12] = ( data[2] & 0xf000 )     >> 12;
    data1[13] = ( data[2] & 0xf00 )      >>  8;
    data1[14] = ( data[2] & 0xf0 )       >>  4;
    data1[15] = ( data[2] & 0xf );
    data1[ 0] = ( data[3] & 0xf0000000 ) >> 28;
    data1[ 1] = ( data[3] & 0xf000000 )  >> 24;
    data1[ 2] = ( data[3] & 0xf00000 )   >> 20;
    data1[ 3] = ( data[3] & 0xf0000 )    >> 16;
    data1[ 4] = ( data[3] & 0xf000 )     >> 12;
    data1[ 5] = ( data[3] & 0xf00 )      >>  8;
    data1[ 6] = ( data[3] & 0xf0 )       >>  4;
    data1[ 7] = ( data[3] & 0xf );
    
    memcpy(buf, data1, 16);
    
    printk(KERN_INFO "proc_comm_ftm_akey1_read() SPC = '0x%x,%x,%x,%x,%x,%x\n",
    										data1[0], data1[1], data1[2],
    										data1[3], data1[4], data1[5]);
    										
    return ret;
}
EXPORT_SYMBOL(proc_comm_ftm_akey1_read);

int proc_comm_ftm_akey2_read(char* buf)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_NV_READ;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[4];
    int32_t ret = 0;
    
    char data1[16] = {0};
    

    memset((void*)data, 0x00000000, sizeof(data));
    data[0] = (uint32_t)25;	// NV ID IMEI
    
    data[1] = 0x01;
    
    if (ret != msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1, 
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    4)) {
        printk(KERN_ERR
                "%s: SMEM_PROC_COMM_OEM_FIH FAILED!!\n",
                __func__);
    	
    	return ret;
    }
    
    printk(KERN_ERR " Akey1 = %08x, %08x, %08x, %08x", data[0], data[1], data[2], data[3]);
    
    data1[ 8] = ( data[2] & 0xf0000000 ) >> 28;
    data1[ 9] = ( data[2] & 0xf000000 )  >> 24;
    data1[10] = ( data[2] & 0xf00000 )   >> 20;
    data1[11] = ( data[2] & 0xf0000 )    >> 16;
    data1[12] = ( data[2] & 0xf000 )     >> 12;
    data1[13] = ( data[2] & 0xf00 )      >>  8;
    data1[14] = ( data[2] & 0xf0 )       >>  4;
    data1[15] = ( data[2] & 0xf );
    data1[ 0] = ( data[3] & 0xf0000000 ) >> 28;
    data1[ 1] = ( data[3] & 0xf000000 )  >> 24;
    data1[ 2] = ( data[3] & 0xf00000 )   >> 20;
    data1[ 3] = ( data[3] & 0xf0000 )    >> 16;
    data1[ 4] = ( data[3] & 0xf000 )     >> 12;
    data1[ 5] = ( data[3] & 0xf00 )      >>  8;
    data1[ 6] = ( data[3] & 0xf0 )       >>  4;
    data1[ 7] = ( data[3] & 0xf );
    
    memcpy(buf, data1, 16);
    
    printk(KERN_INFO "proc_comm_ftm_akey1_read() SPC = '0x%x,%x,%x,%x,%x,%x\n",
    										data1[0], data1[1], data1[2],
    										data1[3], data1[4], data1[5]);
    										
    return ret;
}
EXPORT_SYMBOL(proc_comm_ftm_akey2_read);


int proc_comm_ftm_spc_read(char* buf)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_NV_READ;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[7];
    int32_t ret = 0;
    
    char data1[6] = {0};
    

    memset((void*)data, 0x00000000, sizeof(data));
    data[0] = (uint32_t)85;	// NV ID MEID
    
    if (ret != msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1, 
                                    &smem_proc_comm_oem_data1,
                                    &smem_proc_comm_oem_data2,
                                    data,
                                    7)) {
        printk(KERN_ERR
                "%s: SMEM_PROC_COMM_OEM_FIH FAILED!!\n",
                __func__);
    	
    	return ret;
    }
    
    printk(KERN_ERR " SPC = %08x, %08x",data[1],data[2]);
    
    data1[ 0] = data[1] & 0xff;
    data1[ 1] = data[2] & 0xff;
    data1[ 2] = data[3] & 0xff;
    data1[ 3] = data[4] & 0xff;
    data1[ 4] = data[5] & 0xff;
    data1[ 5] = data[6] & 0xff;
    
    memcpy(buf, data1, 8);
    
    printk(KERN_INFO "proc_comm_ftm_spc_read() SPC = '0x%x,%x,%x,%x,%x,%x\n",
    										data1[0], data1[1], data1[2],
    										data1[3], data1[4], data1[5]);
    										
    return ret;
}
EXPORT_SYMBOL(proc_comm_ftm_spc_read);

int proc_comm_ftm_spc_write(unsigned* buf)
{
    uint32_t smem_proc_comm_oem_cmd1    = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_NV_WRITE;
    uint32_t smem_proc_comm_oem_data2   = 0; //useless
    
    int32_t data[7];
    int32_t ret = 0;
    
    char data1[6] = {0};
    
    memcpy(data1, buf, 6);
    
    memset((void*)data, 0x00000000, sizeof(data));
    data[0] = (uint32_t)85;	// NV ID IMEI
    
    
    data[1] = ( ( ( 0x3  <<  4) & 0xf0 ) | ( data1[0] & 0xf ) );
    data[2] = ( ( ( 0x3  <<  4) & 0xf0 ) | ( data1[1] & 0xf ) );
    data[3] = ( ( ( 0x3  <<  4) & 0xf0 ) | ( data1[2] & 0xf ) );
    data[4] = ( ( ( 0x3  <<  4) & 0xf0 ) | ( data1[3] & 0xf ) );
    data[5] = ( ( ( 0x3  <<  4) & 0xf0 ) | ( data1[4] & 0xf ) );
    data[6] = ( ( ( 0x3  <<  4) & 0xf0 ) | ( data1[5] & 0xf ) );
    
    printk(KERN_INFO "proc_comm_ftm_spc_write(): %02x, %02x, %02x, %02x, %02x, %02x",
    								data[1],data[2],data[3],data[4],data[5],data[6]);
    
    if (ret != msm_proc_comm_oem_multi(smem_proc_comm_oem_cmd1,
    								&smem_proc_comm_oem_data1,
    								&smem_proc_comm_oem_data2,
    								data,
    								7)) {
        printk(KERN_ERR
        		"%s: SMEM_PROC_COMM_OEM_FIH FAILED!!\n",
                __func__);
    	
    	return ret;
    }
    
    printk(KERN_INFO "proc_comm_ftm_spc_write() success\n");
    
    return ret;
}
EXPORT_SYMBOL(proc_comm_ftm_spc_write);
/* FIH, CHHsieh, 2011/10/21 } */

/* FIH, Square, 2011/11/03 { */
int proc_comm_read_product_id(unsigned int *product_id)
{    
    unsigned int NV_Buffer_w[32], NV_Buffer_r[32];
    int oem_cmd, ret;
    
    memset(NV_Buffer_w, 0, 32 * sizeof(unsigned int));
	memset(NV_Buffer_r, 0, 32 * sizeof(unsigned int));
	
	NV_Buffer_w[0] = 8001;
		
    oem_cmd = SMEM_PROC_COMM_OEM_PRODUCT_ID_READ;
    ret = msm_proc_comm_oem_rw_NV(PCOM_CUSTOMER_CMD1, &oem_cmd, NV_Buffer_r, &NV_Buffer_w[0]);
    
    if(ret != 0)
	{
		printk(KERN_INFO "msm_proc_comm_oem_rw_NV fail, error = %d\n", ret);
		return -EFAULT;
	}
	else
	{
	    if(NULL != product_id)
	    {
	        memcpy(product_id, NV_Buffer_r, 32 * sizeof(unsigned int));
	    }
	    else
	    {
	        printk(KERN_INFO "Parameter is null\n");
		    return -EFAULT;
	    }
	}
	
	return ret;	
}
EXPORT_SYMBOL(proc_comm_read_product_id);
/* FIH, Square, 2011/11/03 } */


int proc_comm_OEM_GPS_MODE(unsigned mode,unsigned satellite,unsigned  * CNvalue)//mode- 0:GPS off, 1:GPS on and satellite is at parameter., 2:get GPS value.
{
	unsigned parameter[2];
	uint32_t oem_cmd = SMEM_PROC_COMM_OEM_GPS_CN_TEST;
	int ret;
	parameter[0] = mode;
	parameter[1] = satellite;  //satellite ID.
	printk("GPS mode =%d, satellite ID =%d\n",parameter[0] ,parameter[1]);
	ret=msm_proc_comm_oem(PCOM_CUSTOMER_CMD1, &oem_cmd, CNvalue, parameter);
	printk("ret=%d\n",ret);
	if(ret !=0)
		return -1;
	return 0;
}
EXPORT_SYMBOL(proc_comm_OEM_GPS_MODE);  
  //FIHTDC, Henry added for change Modem mode ,20111109.--