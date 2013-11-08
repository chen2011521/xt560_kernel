/*
*     ftmgpio.c - GPIO access driver for FTM use
*
*     Copyright (C) 2009 Clement Hsu <clementhsu@tp.cmcs.com.tw>
*     Copyright (C) 2009 Chi Mei Communication Systems Inc.
*
*     This program is free software; you can redistribute it and/or modify
*     it under the terms of the GNU General Public License as published by
*     the Free Software Foundation; version 2 of the License.
*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/sysctl.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/cdev.h>
#include <linux/major.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <asm/ioctl.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <mach/gpio.h>
#include <mach/pmic.h>  /*FIHTDC, For enable bias2 for hook key, MayLi, 2011.11.10*/
#include "../../../arch/arm/mach-msm/proc_comm.h"
//FIH, MichaelKao, 2009/7/9++
/* [FXX_CR], Add for get Hardware version*/
#include <linux/io.h>
#include <mach/msm_iomap.h>
#include <mach/msm_smd.h>
//FIH, MichaelKao, 2009/7/9++
#include "ftmgpio.h"
#include <fih/leds-pmic8028-proc.h>
//FIH, WillChen, 2009/7/3++
/* [FXX_CR], Merge bsp kernel and ftm kernel*/
#ifdef CONFIG_FIH_FXX
extern int ftm_mode;
#endif
//FIH, WillChen, 2009/7/3--

//FIH, Henry Juang, 2009/08/10{++
/* [FXX_CR], Add for FTM PID and testflag*/
#define PID_length 128
char bufferPID_tsflag[PID_length];
char *pbuf;
#define CAL_length 128
unsigned bufferCAL[CAL_length];

//}FIH, Henry Juang, 2009/08/10--

//FIH,NeoChen, for read&write FUSE,20100727 ++
char FUSEvalue[1];
//FIH,NeoChen, for read&write FUSE,20100727 --

struct cdev * g_ftmgpio_cdev = NULL;
char ioctl_operation=0;
int gpio_pin_num=0;
int headset_key = 0;
//static int isReadHW=0,isJogball=0;

static const struct file_operations ftmgpio_dev_fops = {
        .open = ftmgpio_dev_open,                   
        .read = ftmgpio_dev_read,                   
        .write = ftmgpio_dev_write,                   
        .unlocked_ioctl = ftmgpio_dev_ioctl,                 
};                              

static struct miscdevice ftmgpio_dev = {
        MISC_DYNAMIC_MINOR,
        "ftmgpio",
        &ftmgpio_dev_fops
};

static int ftmgpio_dev_open( struct inode * inode, struct file * file )
{
	printk( KERN_INFO "FTM GPIO driver open\n" );

	if( ( file->f_flags & O_ACCMODE ) == O_WRONLY )
	{
		printk( KERN_INFO "FTM GPIO driver device node is readonly\n" );
		return -1;
	}
	else
		return 0;
}

static ssize_t ftmgpio_dev_read( struct file * file, char __user * buffer, size_t size, loff_t * f_pos )
{

	char *st;//,level,pk_status[5];
//	int HWID=0;
//	printk( KERN_INFO "FTM GPIO driver read, length=%d\n",size);
    int	level=0;
	unsigned CN_VALUE=0;

	if(ioctl_operation==IOCTL_GPIO_GET)	
	{
	printk( KERN_INFO "FTM GPIO driver read, length=%d\n",size);

		gpio_direction_input(gpio_pin_num);
		level = gpio_get_value(gpio_pin_num);
		printk( KERN_INFO "level:%d\n",level );
		st=kmalloc(sizeof(char),GFP_KERNEL);
		sprintf(st,"%d",level);
		if(copy_to_user(buffer,st,sizeof(char)))
		{
			kfree(st);
			printk( KERN_INFO "copy_to_user() fail!\n" );
			return -EFAULT;
		}
		kfree(st);
		return(size);
		
	}	

#if 0

#define GPIO_JOGBALL_LEFT  88
#define GPIO_JOGBALL_DOWN  90
#define GPIO_JOGBALL_RIGHT 91
#define GPIO_JOGBALL_UP    93
	else if(ioctl_operation==IOCTL_HV_GET)
	{
		if(!isReadHW)
		{
			printk( KERN_INFO "gpio_get_value(%d):%x, gpio_get_value(%d):%x\n",GPIO_JOGBALL_LEFT,gpio_get_value(GPIO_JOGBALL_LEFT),GPIO_JOGBALL_DOWN,gpio_get_value(GPIO_JOGBALL_DOWN) );
			printk( KERN_INFO "gpio_get_value(%d):%x, gpio_get_value(%d):%x\n",GPIO_JOGBALL_RIGHT,gpio_get_value(GPIO_JOGBALL_RIGHT),GPIO_JOGBALL_UP,gpio_get_value(GPIO_JOGBALL_UP) );

			if(!(gpio_get_value(GPIO_JOGBALL_LEFT)) && !(gpio_get_value(GPIO_JOGBALL_DOWN))
			&& !(gpio_get_value(GPIO_JOGBALL_RIGHT)) && !(gpio_get_value(GPIO_JOGBALL_UP)))
			{
				isJogball=1;		
			}
			isReadHW=1;
		}
		HWID = FIH_READ_ORIG_HWID_FROM_SMEM();
		if(isJogball)
		{
			HWID+=0x400;		
		}
		st=kmalloc(sizeof(char)*4,GFP_KERNEL);
		sprintf(st,"%04x",HWID);
		printk( KERN_INFO "HWID:%x%x%x%x\n",st[0],st[1],st[2],st[3] );
		if(copy_to_user(buffer,st,sizeof(char)*4))
		{
			kfree(st);
			printk( KERN_INFO "copy_to_user() fail!\n" );
			return -EFAULT;
		}
		kfree(st);
		return(size);
	}	
#endif
	else if(ioctl_operation==IOCTL_PID_GET)
	{
		pbuf=(char*)bufferPID_tsflag;

		if(copy_to_user(buffer,bufferPID_tsflag,sizeof(char)*40))
		{
			printk( KERN_INFO "copy_to_user() fail!\n" );
			return -EFAULT;
		}
		return 1;
	}
	else if(ioctl_operation==IOCTL_WLANADDR_GET)
	{
		proc_comm_ftm_wlanaddr_read((char *)bufferPID_tsflag);
		
		if(copy_to_user(buffer,bufferPID_tsflag,sizeof(char)*6))
		{
			printk( KERN_INFO "copy_to_user() fail!\n" );
			return -EFAULT;
		}
		return 1;
	}	
	else if(ioctl_operation==IOCTL_BDADDR_GET)
	{
		proc_comm_ftm_bdaddr_read((char *)bufferPID_tsflag);
		
		if(copy_to_user(buffer,bufferPID_tsflag,sizeof(char)*6))
		{
			printk( KERN_INFO "copy_to_user() fail!\n" );
			return -EFAULT;
		}
		return 1;
	}	
//MTD-BSP-AlwaysChen-SMEM_OEM_COMMAND-03+[
	else if(ioctl_operation==IOCTL_IMEI_GET)
	{
		pbuf=(char*)bufferPID_tsflag;

		if(copy_to_user(buffer,bufferPID_tsflag,sizeof(char)*15))
		{
			printk( KERN_INFO "copy_to_user() fail!\n" );
			return -EFAULT;
		}
		
		printk(KERN_INFO "[ftmgpio.c] IMEI = '%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d'\n", bufferPID_tsflag[0], bufferPID_tsflag[1], bufferPID_tsflag[2], bufferPID_tsflag[3], bufferPID_tsflag[4], bufferPID_tsflag[5], bufferPID_tsflag[6], bufferPID_tsflag[7], bufferPID_tsflag[8], bufferPID_tsflag[9], bufferPID_tsflag[10], bufferPID_tsflag[11], bufferPID_tsflag[12], bufferPID_tsflag[13], bufferPID_tsflag[14]);
		
		return 1;
	}	
//MTD-BSP-AlwaysChen-SMEM_OEM_COMMAND-03+]
#if 0	
	else if(ioctl_operation==IOCTL_TESTFLAG_GET)
	{
		pbuf=(char*)bufferPID_tsflag;
		
		if(copy_to_user(buffer,bufferPID_tsflag,sizeof(char)*64))
		{
			printk( KERN_INFO "copy_to_user() fail!\n" );
			return -EFAULT;
		}
		return 1;
	}

	/* FIH, Michael Kao, 2010/09/10{ */
	/* [FXX_CR], Add for FQC Calibration data test*/
	else if((ioctl_operation==IOCTL_CAL_STATUS1)||(ioctl_operation==IOCTL_CAL_STATUS2))
	{
		pbuf=(char*)bufferCAL;
		printk( KERN_INFO "[FTMGPIO_DEV_READ]IOCTL_CAL_STATUS\n" );
		if(copy_to_user(buffer,bufferCAL,sizeof(char)*8))
		{
			printk( KERN_INFO "copy_to_user() fail!\n" );
			return -EFAULT;
		}
		return 1;
	}
	/* FIH, Michael Kao, 2010/09/10{ */
	
	//FIH,NeoChen, for read&write FUSE,20100727 ++
	else if(ioctl_operation==IOCTL_READ_FUSE)
	{
		if(copy_to_user(buffer,FUSEvalue,sizeof(char)))
		{
			printk( KERN_INFO "copy_to_user() fail!\n" );
			return -EFAULT;
		}
		return 1;
	}
	//FIH,NeoChen, for read&write FUSE,20100727 --
	//FIH,Henry Juang, for GRE power key detection,20100816 ++
	else if(ioctl_operation==IOCTL_READ_PWR_KEY_DETECTION)
	{
		pk_status[0]=proc_comm_read_poc();
		if(copy_to_user(buffer,pk_status,sizeof(char)))
		{
			printk( KERN_INFO "IOCTL_READ_PWR_KEY_DETECTION copy_to_user() fail!\n" );
			return -EFAULT;
		}
		printk( KERN_INFO "pk_status[0]=%d\n",pk_status[0]);
		return 1;
	}
	//FIH,Henry Juang, for GRE power key detection,20100816 ++
#endif	
/* FIHTDC, CHHsieh, FD1 NV programming in factory { */
	else if(ioctl_operation==IOCTL_CUSTOMERPID_GET)
	{
		pbuf=(char*)bufferPID_tsflag;

		if(copy_to_user(buffer,bufferPID_tsflag,sizeof(char)*32))
		{
			printk( KERN_INFO "copy_to_user() fail!\n" );
			return -EFAULT;
		}
		
		printk(KERN_INFO "bufferPID_tsflag = '%s'", bufferPID_tsflag);
		
		return 1;
	}
	else if(ioctl_operation==IOCTL_CUSTOMERPID2_GET)
	{
		pbuf=(char*)bufferPID_tsflag;

		if(copy_to_user(buffer,bufferPID_tsflag,sizeof(char)*32))
		{
			printk( KERN_INFO "copy_to_user() fail!\n" );
			return -EFAULT;
		}
		
		printk(KERN_INFO "bufferPID_tsflag = '%s'", bufferPID_tsflag);
		
		return 1;
	}
	else if(ioctl_operation==IOCTL_CUSTOMERSWID_GET)
	{
		pbuf=(char*)bufferPID_tsflag;

		if(copy_to_user(buffer,bufferPID_tsflag,sizeof(char)*32))
		{
			printk( KERN_INFO "copy_to_user() fail!\n" );
			return -EFAULT;
		}
		
		printk(KERN_INFO "bufferPID_tsflag = '%s'", bufferPID_tsflag);
		
		return 1;
	}
	else if(ioctl_operation==IOCTL_DOM_GET)
	{
		pbuf=(char*)bufferPID_tsflag;

		if(copy_to_user(buffer,bufferPID_tsflag,sizeof(char)*14))
		{
			printk( KERN_INFO "copy_to_user() fail!\n" );
			return -EFAULT;
		}
		
		printk(KERN_INFO "bufferPID_tsflag = '%s'", bufferPID_tsflag);
		
		return 1;
	}
/* } FIHTDC, CHHsieh, FD1 NV programming in factory */

/* FIH, CHHsieh, 2011/08/16 { */
/* Moto CFC requirement */
	else if(ioctl_operation==IOCTL_NV_COMMAND)
	{
		unsigned int NV_Buffer_w[32], NV_Buffer_r[32];
		int oem_cmd, ret;

		memset(NV_Buffer_w, 0, 128);
		memset(NV_Buffer_r, 0, 128);

		/* Copy the written NV command/data from user space */
		if(copy_from_user(NV_Buffer_w, buffer, sizeof(char)*128))
		{
			printk(KERN_INFO "%s (%d) - copy_from_user() fail!\n", __func__, __LINE__ );
			return -EFAULT;
		}

		if(NV_Buffer_w[0]==8001)
		{
			oem_cmd = SMEM_PROC_COMM_OEM_PRODUCT_ID_READ;
			ret = msm_proc_comm_oem_rw_NV(PCOM_CUSTOMER_CMD1, &oem_cmd, NV_Buffer_r, &NV_Buffer_w[0]);
		}
		else
		{
			oem_cmd = SMEM_PROC_COMM_OEM_NV_READ;
			ret = msm_proc_comm_oem_rw_NV(PCOM_CUSTOMER_CMD1, &oem_cmd, NV_Buffer_r, NV_Buffer_w);
		}

		if(ret != 0)
		{
			printk(KERN_INFO "msm_proc_comm_oem_rw_NV fail, error = %d\n", ret);\
			return -EFAULT;
		}

		/* Write the read value back to the buffer */
		if(copy_to_user(buffer, NV_Buffer_r, sizeof(char)*128))
		{
			printk(KERN_INFO "%s (%d) - copy_to_user() fail!\n", __func__, __LINE__ );
			return -EFAULT;
		}

		return 1;
	}
	else if(ioctl_operation==IOCTL_HOOKKEY_GET)
	{
		if(copy_to_user(buffer,&headset_key,sizeof(char)*4))
		{
			printk( KERN_INFO "copy_to_user() fail!\n" );
			return -EFAULT;
		}
		printk( KERN_INFO "ftmgpio_dev_read copy_to_user() headset_key = %d!\n" ,headset_key);
		return 1;
	}
	else if(ioctl_operation==IOCTL_PRIL_CHANGE_REQ)
	{
		int rt;


		rt = proc_comm_req_RPLI();
        
		if(copy_to_user(buffer,&rt,sizeof(char)*4))
		{
			printk( KERN_INFO "copy_to_user() fail!\n" );
			return -EFAULT;
		}
		printk( KERN_INFO "ftmgpio_dev_read copy_to_user() PRIL_CHANGE_REQ = %d!\n" ,rt);
		return 1;
	}

	
/*} FIH, CHHsieh, 2011/08/16 */

/* FIH, CHHsieh, 2011/10/21 { */
/* IRE ftm command W/R NV through smem command */
	else if(ioctl_operation==IOCTL_MEID_GET)
	{
		pbuf=(char*)bufferPID_tsflag;

		if(copy_to_user(buffer,bufferPID_tsflag,sizeof(char)*14))
		{
			printk( KERN_INFO "copy_to_user() fail!\n" );
			return -EFAULT;
		}
		
		printk(KERN_INFO "[ftmgpio.c] MEID = '0x%x%x%x%x%x%x%x%x%x%x%x%x%x%x'\n",
				bufferPID_tsflag[ 0], bufferPID_tsflag[ 1], bufferPID_tsflag[ 2], bufferPID_tsflag[ 3],
				bufferPID_tsflag[ 4], bufferPID_tsflag[ 5], bufferPID_tsflag[ 6], bufferPID_tsflag[ 7],
				bufferPID_tsflag[ 8], bufferPID_tsflag[ 9], bufferPID_tsflag[10], bufferPID_tsflag[11],
				bufferPID_tsflag[12], bufferPID_tsflag[13]);
		
		return 1;
	}
	else if(ioctl_operation==IOCTL_ESN_GET)
	{
		pbuf=(char*)bufferPID_tsflag;

		if(copy_to_user(buffer,bufferPID_tsflag,sizeof(char)*8))
		{
			printk( KERN_INFO "copy_to_user() fail!\n" );
			return -EFAULT;
		}
		
		printk(KERN_INFO "[ftmgpio.c] ESN = '0x%x%x%x%x%x%x%x%x'\n",
				bufferPID_tsflag[0], bufferPID_tsflag[1], bufferPID_tsflag[2], bufferPID_tsflag[3],
				bufferPID_tsflag[4], bufferPID_tsflag[5], bufferPID_tsflag[6], bufferPID_tsflag[7]);
		
		return 1;
	}
	else if(ioctl_operation==IOCTL_AKEY1_GET)
	{
		memset(bufferPID_tsflag,0,16*sizeof(char));
		proc_comm_ftm_akey1_read((char *)bufferPID_tsflag);
		
		if(copy_to_user(buffer,bufferPID_tsflag,sizeof(char)*16))
		{
			printk( KERN_INFO "copy_to_user() fail!\n" );
			return -EFAULT;
		}
		
		printk(KERN_INFO "bufferPID_tsflag = '0x%02x%02x%02x%02x%02x%02x'\n",
				bufferPID_tsflag[0], bufferPID_tsflag[1], bufferPID_tsflag[2], bufferPID_tsflag[3],
				bufferPID_tsflag[4], bufferPID_tsflag[5]);
		
		return 1;
	}
	else if(ioctl_operation==IOCTL_AKEY2_GET)
	{
		memset(bufferPID_tsflag,0,16*sizeof(char));
		proc_comm_ftm_akey2_read((char *)bufferPID_tsflag);
		
		if(copy_to_user(buffer,bufferPID_tsflag,sizeof(char)*16))
		{
			printk( KERN_INFO "copy_to_user() fail!\n" );
			return -EFAULT;
		}
		
		printk(KERN_INFO "bufferPID_tsflag = '0x%02x%02x%02x%02x%02x%02x'\n",
				bufferPID_tsflag[0], bufferPID_tsflag[1], bufferPID_tsflag[2], bufferPID_tsflag[3],
				bufferPID_tsflag[4], bufferPID_tsflag[5]);
		
		return 1;
	}
	else if(ioctl_operation==IOCTL_SPC_GET)
	{
		memset(bufferPID_tsflag,0,6*sizeof(char));
		proc_comm_ftm_spc_read((char *)bufferPID_tsflag);
		
		if(copy_to_user(buffer,bufferPID_tsflag,sizeof(char)*6))
		{
			printk( KERN_INFO "copy_to_user() fail!\n" );
			return -EFAULT;
		}
		
		printk(KERN_INFO "bufferPID_tsflag = '0x%02x%02x%02x%02x%02x%02x'\n",
				bufferPID_tsflag[0], bufferPID_tsflag[1], bufferPID_tsflag[2], bufferPID_tsflag[3],
				bufferPID_tsflag[4], bufferPID_tsflag[5]);
		
		return 1;
	}
/* FIH, CHHsieh, 2011/10/21 } */
	else if(ioctl_operation==IOCTL_GPS_ON)
	{
	 	printk( KERN_INFO "IOCTL_GPS_ON: get cn value!\n" );
		 if(-1==proc_comm_OEM_GPS_MODE(2,(unsigned) gpio_pin_num,&CN_VALUE)){
		 	printk( KERN_INFO "proc_comm_OEM_GPS_MODE get CN value  failed!\n" );
			return -EFAULT;
		}
		
		if(copy_to_user(buffer,(char *)&CN_VALUE,sizeof(unsigned)))
		{
			printk( KERN_INFO "copy_to_user() fail!\n" );
			return -EFAULT;
		}
		
		return 1;
	}	
	return 0;
}

static int ftmgpio_dev_write( struct file * filp, const char __user * buffer, size_t length, loff_t * offset )
{

	int rc;
//	int micbais_en;
//	printk( KERN_INFO "FTM GPIO driver write, length=%d, data=%d\n",length , buffer[0] );
	if(ioctl_operation==IOCTL_GPIO_INIT)
	{
	printk( KERN_INFO "FTM GPIO driver write, length=%d, data=%d\n",length , buffer[0] );

		if(buffer[0]==GPIO_IN)
			rc = gpio_tlmm_config(GPIO_CFG(gpio_pin_num, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
		else
			rc = gpio_tlmm_config(GPIO_CFG(gpio_pin_num, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
			
		if (rc) 
		{
			printk(KERN_ERR
					"%s--%d: gpio_tlmm_config=%d\n",
					__func__,__LINE__, rc);
			return -EIO;
		}

	} 

	else if(ioctl_operation==IOCTL_GPIO_SET)	
	{
		gpio_direction_output(gpio_pin_num, buffer[0]);
	}
#if 0		
	else if(ioctl_operation==IOCTL_POWER_OFF)
	{
		msm_proc_comm(PCOM_POWER_DOWN, 0, 0);
		for (;;)
		;
	}
#endif
	else if(ioctl_operation==IOCTL_PID_SET)
	{
		if(copy_from_user(bufferPID_tsflag,buffer,sizeof(char)*40))
		{
			printk( KERN_INFO "copy_from_user() fail!\n" );
			return -EFAULT;
		}
		proc_comm_write_PID_testFlag( SMEM_PROC_COMM_OEM_PRODUCT_ID_WRITE,(unsigned*)bufferPID_tsflag);
		return 1;
	
	}
	else if(ioctl_operation==IOCTL_WLANADDR_SET)
	{
		if(copy_from_user(bufferPID_tsflag,buffer,6))
		{
			printk( KERN_INFO "copy_from_user() fail!\n" );
			return -EFAULT;
		}
		proc_comm_ftm_wlanaddr_write((char*)bufferPID_tsflag);
		return 1;
	}	
	else if(ioctl_operation==IOCTL_BDADDR_SET)
	{
		if(copy_from_user(bufferPID_tsflag,buffer,6))
		{
			printk( KERN_INFO "copy_from_user() fail!\n" );
			return -EFAULT;
		}
		proc_comm_ftm_bdaddr_write((char*)bufferPID_tsflag);
		return 1;
	}
//MTD-BSP-AlwaysChen-SMEM_OEM_COMMAND-03+[
	else if(ioctl_operation==IOCTL_IMEI_SET)
	{
		if(copy_from_user(bufferPID_tsflag,buffer,sizeof(char)*15))
		{
			printk( KERN_INFO "copy_from_user() fail!\n" );
			return -EFAULT;
		}
		proc_comm_ftm_imei_write((unsigned*)bufferPID_tsflag);
		return 1;
	}	
//MTD-BSP-AlwaysChen-SMEM_OEM_COMMAND-03+]
#if 0	
	else if(ioctl_operation==IOCTL_TESTFLAG_SET)
	{
		if(copy_from_user(bufferPID_tsflag,buffer,sizeof(char)*64))
		{
			printk( KERN_INFO "copy_from_user() fail!\n" );
			return -EFAULT;
		}
		proc_comm_write_PID_testFlag( SMEM_PROC_COMM_OEM_TEST_FLAG_WRITE,(unsigned*)bufferPID_tsflag);
		return 1;
	} 	
//FIH, Henry Juang, 2009/08/10--
	/* FIH, Michael Kao, 2009/08/28{ */
	/* [FXX_CR], Add to turn on Mic Bias */
	else if(ioctl_operation==IOCTL_MICBIAS_SET)
	{
		micbais_en=buffer[0];
		proc_comm_micbias_set(&micbais_en);
	}
	/* FIH, Michael Kao, 2009/08/28{ */
	else if(ioctl_operation==IOCTL_WRITE_NV_8029)
	{
		if(write_NV_single(gpio_pin_num,buffer[0])!=0)
		{
			printk( KERN_INFO "IOCTL_WRITE_NV_8029 fail! nv=%d,val=%d\n",gpio_pin_num,buffer[0]);
			return -EFAULT;
		}
	}
#endif
/* FIHTDC, CHHsieh, FD1 NV programming in factory { */
	else if(ioctl_operation==IOCTL_CUSTOMERPID_SET)
	{
		if(copy_from_user(bufferPID_tsflag,buffer,sizeof(char)*32))
		{
			printk( KERN_INFO "copy_from_user() fail!\n" );
			return -EFAULT;
		}
		printk(KERN_INFO "===========================> ftmgpio.c IOCTL_CUSTOMERPID_SET '%s'\n", bufferPID_tsflag);
		proc_comm_ftm_customer_pid_write((unsigned*)bufferPID_tsflag);
		return 1;
	}
	else if(ioctl_operation==IOCTL_CUSTOMERPID2_SET)
	{
		if(copy_from_user(bufferPID_tsflag,buffer,sizeof(char)*32))
		{
			printk( KERN_INFO "copy_from_user() fail!\n" );
			return -EFAULT;
		}
		printk(KERN_INFO "===========================> ftmgpio.c IOCTL_CUSTOMERPID2_SET '%s'\n", bufferPID_tsflag);
		proc_comm_ftm_customer_pid2_write((unsigned*)bufferPID_tsflag);
		return 1;
	}
	else if(ioctl_operation==IOCTL_CUSTOMERSWID_SET)
	{
		if(copy_from_user(bufferPID_tsflag,buffer,sizeof(char)*32))
		{
			printk( KERN_INFO "copy_from_user() fail!\n" );
			return -EFAULT;
		}
		printk(KERN_INFO "===========================> ftmgpio.c IOCTL_CUSTOMERSWID_SET '%s'\n", bufferPID_tsflag);
		proc_comm_ftm_customer_swid_write((unsigned*)bufferPID_tsflag);
		return 1;
	}
	else if(ioctl_operation==IOCTL_DOM_SET)
	{
		if(copy_from_user(bufferPID_tsflag,buffer,sizeof(char)*14))
		{
			printk( KERN_INFO "copy_from_user() fail!\n" );
			return -EFAULT;
		}
		printk(KERN_INFO "===========================> ftmgpio.c IOCTL_DOM_SET '%s'\n", bufferPID_tsflag);
		proc_comm_ftm_dom_write((unsigned*)bufferPID_tsflag);
		return 1;
	}
/* } FIHTDC, CHHsieh, FD1 NV programming in factory */

/* FIH, CHHsieh, 2011/08/16 { */
/* Moto CFC requirement */
	else if(ioctl_operation==IOCTL_NV_COMMAND)
	{
		unsigned int NV_Buffer_w[32];
		int oem_cmd, ret;

		memset(NV_Buffer_w, 0, 128);

		/* Copy the written NV command/data from user space */
		if(copy_from_user(NV_Buffer_w, buffer, sizeof(char)*128))
		{
			printk(KERN_INFO "%s (%d) - copy_from_user() fail!\n", __func__, __LINE__ );
			return -EFAULT;
		}

		if(NV_Buffer_w[0]==8001)
		{
			oem_cmd = SMEM_PROC_COMM_OEM_PRODUCT_ID_WRITE;
			ret = msm_proc_comm_oem_rw_NV(PCOM_CUSTOMER_CMD1, &oem_cmd, NULL, &NV_Buffer_w[1]);
		}
		else
		{
			oem_cmd = SMEM_PROC_COMM_OEM_NV_WRITE;
			ret = msm_proc_comm_oem_rw_NV(PCOM_CUSTOMER_CMD1, &oem_cmd, NULL, NV_Buffer_w);
		}


		if(ret != 0)
		{
			printk(KERN_INFO "msm_proc_comm_oem_rw_NV fail, error = %d\n", ret);\
			return -EFAULT;
		}

	}
	else if(ioctl_operation==IOCTL_OEM_SMEM_PROCESS)
	{
		unsigned int oem_cmd = gpio_pin_num; 
		unsigned int oem_parameter[32] = {0};
		
		memset(oem_parameter, 0, sizeof(oem_parameter));
		if(copy_from_user(oem_parameter, buffer, sizeof(oem_parameter)))
		{
			printk(KERN_INFO "%s (%d) - copy_from_user() fail!\n", __func__, __LINE__ );
			return -EFAULT;
		}
		if( proc_comm_oem_smem_process((int*)oem_parameter, oem_cmd) != 0 ){
			memcpy((int*)buffer, (int*)oem_parameter, sizeof(oem_parameter));
			printk(KERN_INFO "%s (%d) ==> ftmgpio.c '0x%02x 0x%02x 0x%02x'\n",
											__func__, __LINE__, buffer[0], buffer[4], buffer[8]);
			//if(copy_to_user(buffer,oem_parameter,sizeof(buffer)))
			//{
			//	printk( KERN_INFO "proc_comm_oem_smem_process() and copy_to_user() fail!\n" );
			//	return -EFAULT;
			//}
			printk( KERN_INFO "proc_comm_oem_smem_process() fail!\n" );
			return -EFAULT;
		}
		memcpy((int*)buffer, (int*)oem_parameter, sizeof(oem_parameter));
		printk(KERN_INFO "%s (%d) ==> ftmgpio.c '0x%02x 0x%02x 0x%02x'\n",
										__func__, __LINE__, buffer[0], buffer[4], buffer[8]);
		//if(copy_to_user(buffer,oem_parameter,sizeof(oem_parameter)))
		//{
		//	printk(KERN_INFO "%s (%d) - copy_to_user() fail!\n", __func__, __LINE__ );
		//	return -EFAULT;
		//}
		return 1;
	}
/*} FIH, CHHsieh, 2011/08/16 */

/* FIH, CHHsieh, 2011/10/21 { */
/* IRE ftm command W/R NV through smem command */
	else if(ioctl_operation==IOCTL_MEID_SET)
	{
		if(copy_from_user(bufferPID_tsflag,buffer,sizeof(char)*7))
		{
			printk( KERN_INFO "copy_from_user() fail!\n" );
			return -EFAULT;
		}
		if( proc_comm_ftm_meid_write((char*)bufferPID_tsflag) != 0 ){
			printk( KERN_INFO "proc_comm_ftm_meid_write() fail!\n" );
			return -EFAULT;
		}
		return 1;
	}
	else if(ioctl_operation==IOCTL_AKEY1_SET)
	{
		if(copy_from_user(bufferPID_tsflag,buffer,sizeof(char)*16))
		{
			printk( KERN_INFO "copy_from_user() fail!\n" );
			return -EFAULT;
		}
		if( proc_comm_ftm_akey1_write((unsigned*)bufferPID_tsflag) != 0 ){
			printk( KERN_INFO "proc_comm_ftm_akey1_write() fail!\n" );
			return -EFAULT;
		}
		return 1;
	}
	else if(ioctl_operation==IOCTL_AKEY2_SET)
	{
		if(copy_from_user(bufferPID_tsflag,buffer,sizeof(char)*16))
		{
			printk( KERN_INFO "copy_from_user() fail!\n" );
			return -EFAULT;
		}
		if( proc_comm_ftm_akey2_write((unsigned*)bufferPID_tsflag) != 0 ){
			printk( KERN_INFO "proc_comm_ftm_akey2_write() fail!\n" );
			return -EFAULT;
		}
		return 1;
	}
	else if(ioctl_operation==IOCTL_SPC_SET)
	{
		if(copy_from_user(bufferPID_tsflag,buffer,sizeof(char)*8))
		{
			printk( KERN_INFO "copy_from_user() fail!\n" );
			return -EFAULT;
		}
		if( proc_comm_ftm_spc_write((unsigned*)bufferPID_tsflag) != 0 ){
			printk( KERN_INFO "proc_comm_ftm_spc_write() fail!\n" );
			return -EFAULT;
		}
		return 1;
	}
/* FIH, CHHsieh, 2011/10/21 } */

	else
		printk( KERN_INFO "Undefined FTM GPIO driver IOCTL command\n" );
	
	return length;
}	

#define CLK_UART_NS_REG (MSM_CLK_CTL_BASE + 0x000000E0)

/* FIH, CHHsieh, 2011/08/16 { */
/* Moto CFC requirement */
#define A11_REBOOT_REASON_TO_DL_MODE	0x444C4F41
/*} FIH, CHHsieh, 2011/08/16 */

extern int proc_comm_set_led(unsigned id, unsigned level);

static long ftmgpio_dev_ioctl(struct file * filp, unsigned int cmd, unsigned long arg )
{

//	unsigned smem_response;
//	uint32_t oem_cmd;
//	unsigned oem_parameter=0;
//	int Cal_data;
#ifdef CONFIG_FIH_FTM
	unsigned long value_UART_NS_REG=0x0;
#endif
/* FIH, CHHsieh, 2011/08/16 { */
/* Moto CFC requirement */
	int reset_chip_reason = A11_REBOOT_REASON_TO_DL_MODE;
/*} FIH, CHHsieh, 2011/08/16 */

	printk( KERN_INFO "FTM GPIO driver ioctl, cmd = %d, arg = %ld\n", cmd , arg);
	
	switch(cmd)
	{
	
		case IOCTL_GPIO_INIT:		
		case IOCTL_GPIO_SET:		
		case IOCTL_GPIO_GET:	
			ioctl_operation = cmd;
			gpio_pin_num = arg;			
			break;			
#if 0			
		//FIH, MichaelKao, 2009/7/9++
		/* [FXX_CR], Add for get Hardware version*/
		case IOCTL_HV_GET:
		//FIH, MichaelKao, 2009/7/9++
		case IOCTL_POWER_OFF:
		/* FIH, Michael Kao, 2009/08/28{ */
		/* [FXX_CR], Add to turn on Mic Bias */
		case IOCTL_MICBIAS_SET:
		/* FIH, Michael Kao, 2009/08/28{ */
			//printk("[Ftmgoio.c][ftmgpio_dev_ioctl]power_off_test++++");
			ioctl_operation = cmd;
			gpio_pin_num = arg;			
			break;

#endif
		case IOCTL_PID_GET:
			memset(bufferPID_tsflag,0,40*sizeof(char));
			proc_comm_read_PID_testFlag( SMEM_PROC_COMM_OEM_PRODUCT_ID_READ,(unsigned*)bufferPID_tsflag);
			ioctl_operation = cmd;
			gpio_pin_num = arg;	
			break;

		case IOCTL_PID_SET:
			ioctl_operation = cmd;
			gpio_pin_num = arg;			
			break;

		case IOCTL_WLANADDR_GET:
		case IOCTL_WLANADDR_SET:
			ioctl_operation = cmd;
			gpio_pin_num = arg;
			break;			

		case IOCTL_BDADDR_GET:
		case IOCTL_BDADDR_SET:
			ioctl_operation = cmd;
			gpio_pin_num = arg;			
			break;		
//MTD-BSP-AlwaysChen-SMEM_OEM_COMMAND-03+[
		case IOCTL_IMEI_GET:
			memset(bufferPID_tsflag,0,15*sizeof(char));
			proc_comm_ftm_imei_read((unsigned*)bufferPID_tsflag);
			ioctl_operation = cmd;
			gpio_pin_num = arg;	
			break;
		case IOCTL_IMEI_SET:
			ioctl_operation = cmd;
			gpio_pin_num = arg;			
			break;			
//MTD-BSP-AlwaysChen-SMEM_OEM_COMMAND-03+]

/* FIH, CHHsieh, 2011/08/16 { */
/* Moto CFC requirement */
		case IOCTL_RESET:
			proc_comm_reset_chip(&reset_chip_reason);
			break;
		case IOCTL_NV_COMMAND:
		{
			ioctl_operation = cmd;
			gpio_pin_num = arg;
			break;
		}
		case IOCTL_OEM_SMEM_PROCESS:
		{
			printk( KERN_INFO"%s (%d) - ioctl code(%d)\n",__func__,__LINE__, cmd);
			ioctl_operation = cmd;
			gpio_pin_num = arg;
			break;
		}
/*} FIH, CHHsieh, 2011/08/16 */

//MTD-MM-YW-FTM-AUDIO+[
		case IOCTL_LOOPBACK_EN:
			printk("[FTM AUDIO] Enable Loopback\n");
			proc_audio_loopback(1, arg);		
			break;

		case IOCTL_LOOPBACK_DIS:
			printk("[FTM AUDIO] Disable Loopback\n");
			proc_audio_loopback(0, arg);		
			break;

		case IOCTL_MICBIAS_CTL:
			printk("[FTM AUDIO] Endble MICBIAS\n");
			//pmic_hsed_enable(PM_HSED_CONTROLLER_0, arg); /*FIHTDC, For enable bias2 for hook key, MayLi, 2011.11.10*/
			//pmic_hsed_enable(PM_HSED_CONTROLLER_1, arg);
			break;
//MTD-MM-YW-FTM-AUDIO+]			

// MTD-KERNEL-EL-VIB00+[
		case IOCTL_VIB_ACTION:
			printk("[FTM VIB] Vibrator Action\n");
			proc_vib_action(arg, 0);
			break;
// MTD-KERNEL-EL-VIB00+]
#ifdef CONFIG_FIH_FTM
		case IOCTL_UART_BAUDRATE_460800:
				//add for testing.	
				value_UART_NS_REG=0x7ff8fff & readl(CLK_UART_NS_REG);//set UART source clock to TCXO.
				printk("1******value_UART_NS_REG=0x%x%x,addr=0x%x\n",(unsigned int)(value_UART_NS_REG>>8)&0xff,(unsigned int)(value_UART_NS_REG & 0xff),(unsigned int)CLK_UART_NS_REG);	
				if(arg!=1){
					value_UART_NS_REG|=0x0001000; //roll back UART source clock to TCXO/4.
				}
				writel(value_UART_NS_REG,CLK_UART_NS_REG);	
				value_UART_NS_REG=readl(CLK_UART_NS_REG);	
				printk("2******value_UART_NS_REG=0x%x%x,addr=0x%x\n",(unsigned int)(value_UART_NS_REG>>8)&0xff,(unsigned int)(value_UART_NS_REG & 0xff),(unsigned int)CLK_UART_NS_REG);	
			break;
#endif
#if 0			
		case IOCTL_TESTFLAG_GET:
			memset(bufferPID_tsflag,0,64*sizeof(char));
			proc_comm_read_PID_testFlag( SMEM_PROC_COMM_OEM_TEST_FLAG_READ,(unsigned*)bufferPID_tsflag);
			ioctl_operation = cmd;
			gpio_pin_num = arg;	
			break;
		case IOCTL_TESTFLAG_SET:
			ioctl_operation = cmd;
			gpio_pin_num = arg;			
			break;
//}FIH, Henry Juang, 2009/08/10--
		
		//[+++]FIH, ChiaYuan, 2007/07/10
		// [FXX_CR] Put the modem into LPM mode for FTM test	
		case IOCTL_MODEM_LPM:
			oem_cmd = SMEM_PROC_COMM_OEM_EBOOT_SLEEP_REQ;
			msm_proc_comm_oem(PCOM_CUSTOMER_CMD1, &oem_cmd, &smem_response, &oem_parameter );
			break;
		//[---]FIH, ChiaYuan, 2007/07/10							
//FIH, Henry Juang, 20091116 ++
/*Add for modifing UART3 source clock*/
		case IOCTL_UART_BAUDRATE_460800:
				//add for testing.	
				value_UART_NS_REG=0x7ff8fff & readl(CLK_UART_NS_REG);//set UART source clock to TCXO.
				printk("1******value_UART_NS_REG=0x%x%x,addr=0x%x\n",(unsigned int)(value_UART_NS_REG>>8)&0xff,(unsigned int)(value_UART_NS_REG & 0xff),(unsigned int)CLK_UART_NS_REG);	
				if(arg!=1){
					value_UART_NS_REG|=0x0001000; //roll back UART source clock to TCXO/4.
				}
				writel(value_UART_NS_REG,CLK_UART_NS_REG);	
				value_UART_NS_REG=readl(CLK_UART_NS_REG);	
				printk("2******value_UART_NS_REG=0x%x%x,addr=0x%x\n",(unsigned int)(value_UART_NS_REG>>8)&0xff,(unsigned int)(value_UART_NS_REG & 0xff),(unsigned int)CLK_UART_NS_REG);	
			break;
//FIH, Henry Juang, 20091116 --
//FIH, Henry Juang, 20100426 ++
/*Add for write NV 8029.*/
		case IOCTL_WRITE_NV_8029:
			printk("select  nv_%d to write. \n",(int)arg);
			ioctl_operation = cmd;
			gpio_pin_num = arg;	
			
			break;
//FIH, Henry Juang, 20100426 --
		/* FIH, Michael Kao, 2010/09/10{ */
		/* [FXX_CR], Add for FQC Calibration data test*/
		case IOCTL_CAL_STATUS1:
			bufferCAL[0]=2499;
			if(proc_comm_read_nv((unsigned*)bufferCAL)==0)
				printk("[FTMGPIO.c]IOCTL_CAL_STATUS1=%x, %x\n",bufferCAL[0],bufferCAL[1]);
			else
			{
				printk("[FTMGPIO.c]IOCTL_CAL_STATUS1 read fail\n");
				return 0;
			}
			Cal_data=(bufferCAL[0]&0xff)*1000000+((bufferCAL[0]&0xff00)>>8)*10000+
				((bufferCAL[0]&0xff0000)>>16)*100+((bufferCAL[0]&0xff000000)>>24);
			printk("[FTMGPIO.c]Cal_data1=%d\n",Cal_data);
			ioctl_operation = cmd;
			gpio_pin_num = arg;
			return Cal_data;
			break;
		case IOCTL_CAL_STATUS2:
			bufferCAL[0]=2499;
			if(proc_comm_read_nv((unsigned*)bufferCAL)==0)
				printk("[FTMGPIO.c]IOCTL_CAL_STATUS1=%d, %d\n",bufferCAL[0],bufferCAL[1]);
			else
			{
				printk("[FTMGPIO.c]IOCTL_CAL_STATUS1 read fail\n");
				return 0;
			}
			Cal_data=(bufferCAL[1]&0xff)*1000000+((bufferCAL[1]&0xff00)>>8)*10000+
				((bufferCAL[1]&0xff0000)>>16)*100+((bufferCAL[1]&0xff000000)>>24);
			printk("[FTMGPIO.c]Cal_data2=%d\n",Cal_data);
			ioctl_operation = cmd;
			gpio_pin_num = arg;	
			return Cal_data;
			break;
		case IOCTL_CAL_STATUS3:
			bufferCAL[0]=2500;
			if(proc_comm_read_nv((unsigned*)bufferCAL)==0)
				printk("[FTMGPIO.c]IOCTL_CAL_STATUS2=0x%x\n",bufferCAL[0]);
			else
			{
				printk("[FTMGPIO.c]IOCTL_CAL_STATUS2 read fail\n");
				return 0;
			}
			Cal_data=(bufferCAL[0]&0xff)*1000000+((bufferCAL[0]&0xff00)>>8)*10000+
				((bufferCAL[0]&0xff0000)>>16)*100+((bufferCAL[0]&0xff000000)>>24);
			printk("[FTMGPIO.c]Cal_data3=%d\n",Cal_data);
			ioctl_operation = cmd;
			gpio_pin_num = arg;	
			return Cal_data;
			break;
		case IOCTL_CAL_STATUS4:
			bufferCAL[0]=2500;
			if(proc_comm_read_nv((unsigned*)bufferCAL)==0)
				printk("[FTMGPIO.c]IOCTL_CAL_STATUS2=0x%x\n",bufferCAL[0]);
			else
			{
				printk("[FTMGPIO.c]IOCTL_CAL_STATUS2 read fail\n");
				return 0;
			}
			Cal_data=(bufferCAL[1]&0xff)*1000000+((bufferCAL[1]&0xff00)>>8)*10000+
				((bufferCAL[1]&0xff0000)>>16)*100+((bufferCAL[1]&0xff000000)>>24);
			printk("[FTMGPIO.c]Cal_data4=%d\n",Cal_data);
			ioctl_operation = cmd;
			gpio_pin_num = arg;
			return Cal_data;
			break;
		/* FIH, Michael Kao, 2010/09/10{ */
//FIH,NeoChen, for read&write FUSE,20100727 ++
		case IOCTL_WRITE_FUSE:
			proc_comm_write_FUSE( SMEM_PROC_COMM_OEM_WRITE_FUSE);
			
			break;
			
		case IOCTL_READ_FUSE:
			memset(FUSEvalue,0,sizeof(char));
			ioctl_operation = cmd;
			proc_comm_read_FUSE( SMEM_PROC_COMM_OEM_READ_FUSE,(unsigned*)FUSEvalue);
			
			break;
//FIH,NeoChen, for read&write FUSE,20100727 --
//FIH,NeoChen, for GRE read&write FUSE,20100727 --

//FIH,Henry Juang, for GRE power key detection,20100816 ++
		case IOCTL_READ_PWR_KEY_DETECTION:
			ioctl_operation = cmd;
			break;
//FIH,Henry Juang, for GRE power key detection,20100816 --
#endif

/* 20101207, SquareCHFang, NVBackup { */
		case IOCTL_BACKUP_UNIQUE_NV:
			printk(KERN_INFO "===========================> ftmgpio.c IOCTL_BACKUP_UNIQUE_NV 2\n");
			if( proc_comm_ftm_backup_unique_nv() != 0 ){
				return -1;
			}
			break;
		case IOCTL_HW_RESET:
			printk(KERN_INFO "===========================> ftmgpio.c IOCTL_HW_RESET 2\n");
			//proc_comm_ftm_hw_reset();
			break;
		case IOCTL_BACKUP_NV_FLAG:
			printk(KERN_INFO "===========================> ftmgpio.c IOCTL_BACKUP_NV_FLAG 2\n");
			if( proc_comm_ftm_nv_flag() != 0){
				return -1;
			}
			break;
		case IOCTL_CHANGE_MODEM_LPM:
			printk(KERN_INFO "===========================> ftmgpio.c IOCTL_CHANGE_MODEM_LPM\n");
			if( proc_comm_ftm_change_modem_lpm() != 1){
				return -1;
			}
			break;
/* FIHTDC, CHHsieh, FD1 NV programming in factory { */
		case IOCTL_CUSTOMERPID_GET:
			memset(bufferPID_tsflag,0,32*sizeof(char));
			proc_comm_ftm_customer_pid_read((unsigned*)bufferPID_tsflag);
			ioctl_operation = cmd;
			gpio_pin_num = arg;	
			break;
		case IOCTL_CUSTOMERPID2_GET:
			memset(bufferPID_tsflag,0,32*sizeof(char));
			proc_comm_ftm_customer_pid2_read((unsigned*)bufferPID_tsflag);
			ioctl_operation = cmd;
			gpio_pin_num = arg;	
			break;
		case IOCTL_CUSTOMERSWID_GET:
			memset(bufferPID_tsflag,0,32*sizeof(char));
			proc_comm_ftm_customer_swid_read((unsigned*)bufferPID_tsflag);
			ioctl_operation = cmd;
			gpio_pin_num = arg;	
			break;
		case IOCTL_DOM_GET:
			memset(bufferPID_tsflag,0,14*sizeof(char));
			proc_comm_ftm_dom_read((unsigned*)bufferPID_tsflag);
			ioctl_operation = cmd;
			gpio_pin_num = arg;	
			break;
		case IOCTL_CUSTOMERPID_SET:
		case IOCTL_CUSTOMERPID2_SET:
		case IOCTL_CUSTOMERSWID_SET:
		case IOCTL_DOM_SET:
			ioctl_operation = cmd;
			gpio_pin_num = arg;			
			break;
		case IOCTL_HOOKKEY_GET:
			proc_comm_read_hook_Flag(SMEM_PROC_COMM_OEM_AUDIO_HESDKEY,(unsigned*)&headset_key);
			printk( KERN_INFO "IOCTL_HOOKKEY_GET headset_key=%d \n",headset_key );
			ioctl_operation = cmd;
			gpio_pin_num = arg;	
			break;
        case IOCTL_FLASHLED_ON:
            if(-1 == proc_comm_set_led(PMIC8028_ID_CAMERA_FLASH, 1)){
                return -1;
            }
            break;
        case IOCTL_FLASHLED_OFF:
            if(-1 == proc_comm_set_led(PMIC8028_ID_CAMERA_FLASH, 0)){
                return -1;
            }
            break;
/* } FIHTDC, CHHsieh, FD1 NV programming in factory */

/* FIH, CHHsieh, 2011/10/21 { */
/* IRE ftm command W/R NV through smem command */
		case IOCTL_MEID_GET:
			memset(bufferPID_tsflag,0,14*sizeof(char));
			proc_comm_ftm_meid_read((unsigned*)bufferPID_tsflag);
			ioctl_operation = cmd;
			gpio_pin_num = arg;	
			break;
		case IOCTL_MEID_SET:
			ioctl_operation = cmd;
			gpio_pin_num = arg;			
			break;
		case IOCTL_ESN_GET:
			memset(bufferPID_tsflag,0,8*sizeof(char));
			proc_comm_ftm_esn_read((unsigned*)bufferPID_tsflag);
			ioctl_operation = cmd;
			gpio_pin_num = arg;	
			break;
		case IOCTL_AKEY1_SET:
		case IOCTL_AKEY2_SET:
		case IOCTL_AKEY1_GET:
		case IOCTL_AKEY2_GET:
		case IOCTL_SPC_GET:
		case IOCTL_SPC_SET:
			ioctl_operation = cmd;
			gpio_pin_num = arg;			
			break;
/* FIH, CHHsieh, 2011/10/21 } */
		case IOCTL_GPS_ON:
			 gpio_pin_num = arg;	
			 printk( KERN_INFO "IOCTL_GPS_ON: satellite=%d!\n",gpio_pin_num );
			 if(-1==proc_comm_OEM_GPS_MODE(1,(unsigned) gpio_pin_num,(unsigned *)&reset_chip_reason)){
			 	printk( KERN_INFO "proc_comm_OEM_GPS_MODE IOCTL_GPS_ON failed!\n" );
				return -1;
			}
			ioctl_operation = cmd;
			gpio_pin_num = arg;	
			break;
		case IOCTL_GPS_OFF:
			 if(-1==proc_comm_OEM_GPS_MODE(0,(unsigned) arg,(unsigned *)&reset_chip_reason)){
			 	printk( KERN_INFO "proc_comm_OEM_GPS_MODE IOCTL_GPS_OFF failed!\n" );
				return -1;
			}
			ioctl_operation = cmd;
			gpio_pin_num = arg;	
			break;			

		case IOCTL_PRIL_CHANGE_REQ:
			ioctl_operation = cmd;
			break;			

		case IOCTL_PRIL_VERIFY_RESULT:
			proc_comm_verify_result(arg);
			break;			

		default:
			printk( KERN_INFO "Undefined FTM GPIO driver IOCTL command\n" );
			return -1;
	}


	return 0;
}

 static int __init ftmgpio_init(void)
{
        int ret;

        printk( KERN_INFO "FTM GPIO Driver init\n" );
//FIH, WillChen, 2009/7/3++
/* [FXX_CR], Merge bsp kernel and ftm kernel*/
/* FIH, Michael Kao, 2010/09/10{ */
/* [FXX_CR], Add for FQC Calibration data test*/
#if 0
#ifdef CONFIG_FIH_FXX
	if (ftm_mode == 0)
	{
		printk(KERN_INFO "This is NOT FTM mode. return without init!!!\n");
		return -1;
	}
#endif
#endif
/* FIH, Michael Kao, 2010/09/10{ */

//FIH, WillChen, 2009/7/3--
	ret = misc_register(&ftmgpio_dev);
	if (ret){
		printk(KERN_WARNING "FTM GPIO Unable to register misc device.\n");
		return ret;
	}

        return ret;
        
}

static void __exit ftmgpio_exit(void)                         
{                                                                              
        printk( KERN_INFO "FTM GPIO Driver exit\n" );          
        misc_deregister(&ftmgpio_dev);
}                                                        
                                                         
module_init(ftmgpio_init);                            
module_exit(ftmgpio_exit);                                
                                                         
MODULE_AUTHOR( "Clement Hsu <clementhsu@tp.cmcs.com.tw>" );
MODULE_DESCRIPTION( "FTM GPIO driver" );                
MODULE_LICENSE( "GPL" );                                 



















