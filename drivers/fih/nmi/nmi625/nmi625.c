/*****************************************************************************
 Copyright(c) 2010 NMI Inc. All Rights Reserved
 
 File name : nmi_hw.c
 
 Description : NM625 host interface
 
 History : 
 ----------------------------------------------------------------------
 2010/05/17 	ssw		initial
*******************************************************************************/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/miscdevice.h>
#include <linux/i2c.h>

#include <asm/uaccess.h>
#include <asm/io.h>

#include <linux/delay.h>

//#include <plat/gpio-cfg.h>
#include <mach/gpio.h>
#include <mach/msm_smd.h>

#include <linux/slab.h>	// Lewis 11.08.02

#include "nmi625.h"
#include "nmi625-i2c.h"
#include <mach/vreg.h>/**Jingbao add*/
#include <mach/pmic.h>/**Jingbao add*/
#include <fih/dynloader.h> /*Alex add*/


#define NMI_NAME		"dmb"
//#define NMI625_HW_CHIP_ID_CHECK

int i2c_init(void);

int dmb_open (struct inode *inode, struct file *filp);
long dmb_ioctl (struct file *filp, unsigned int cmd, unsigned long arg);
int dmb_release (struct inode *inode, struct file *filp);

/* GPIO Setting */
void dmb_hw_setting(void);
void dmb_hw_init(void);
void dmb_hw_deinit(void);
void dmb_hw_rst(unsigned long, unsigned long);

/* GPIO Setting For TV */
unsigned int gpio_isdbt_pwr_en;
unsigned int gpio_isdbt_rstn;	
unsigned int gpio_atv_rstn;

//extern unsigned int HWREV;
unsigned int HWREV=0;//chandler

static struct file_operations dmb_fops = 
{
	.owner		= THIS_MODULE,
	.unlocked_ioctl		= dmb_ioctl,
	.open		= dmb_open,
	.release	= dmb_release,
};

static struct miscdevice nmi_misc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = NMI_NAME,
    .fops = &dmb_fops,
};


extern void nmi625_i2c_read_chip_id(void); /* chandler-test */
extern void nmi625_i2c_ftm_chip_id(u8 *buf, int size);
	
int dmb_open (struct inode *inode, struct file *filp)
{
	DMB_OPEN_INFO_T *hOpen = NULL;

	printk("dmb open HANDLE : 0x%x\n", (unsigned int)hOpen);

	hOpen = (DMB_OPEN_INFO_T *)kmalloc(sizeof(DMB_OPEN_INFO_T), GFP_KERNEL);

	filp->private_data = hOpen;
	
	return 0;
}

int dmb_release (struct inode *inode, struct file *filp)
{
	//DMB_OPEN_INFO_T *hOpen;

	printk("dmb release \n");

	return 0;
}

long dmb_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
	long res = 1;

	ioctl_info info;

	int	err = 0, size = 0;
//	printk("%s : type(%c), nr(%x) \n",__func__ , _IOC_TYPE(cmd), _IOC_NR(cmd));
	
	if(_IOC_TYPE(cmd) != IOCTL_MAGIC) return -EINVAL;
	if(_IOC_NR(cmd) >= IOCTL_MAXNR) return -EINVAL;

	size = _IOC_SIZE(cmd);
//	printk("%s : size(%d)\n", __func__, size);
	
	if(size) {
		if(_IOC_DIR(cmd) & _IOC_READ)
			err = access_ok(VERIFY_WRITE, (void *) arg, size);
		else if(_IOC_DIR(cmd) & _IOC_WRITE)
			err = access_ok(VERIFY_READ, (void *) arg, size);
		if(!err) {
			printk("%s : Wrong argument! cmd(0x%08x) _IOC_NR(%d) _IOC_TYPE(0x%x) _IOC_SIZE(%d) _IOC_DIR(0x%x)\n", 
			__FUNCTION__, cmd, _IOC_NR(cmd), _IOC_TYPE(cmd), _IOC_SIZE(cmd), _IOC_DIR(cmd));
			return err;
		}
	}


	switch(cmd) 
	{
		case IOCTL_DMB_RESET:
			res = copy_from_user((void *)&info, (void *)arg, size);
			// reset the chip with the info[0], if info[0] is zero then isdbt reset, 
			// if info[0] is one then atv reset.
			printk("IOCTL_DMB_RESET enter.., info.buf[0] %d, info.buf[1] %d\n", info.buf[0], info.buf[1]);
			dmb_hw_rst(info.buf[0], info.buf[1]);
			break;
		case IOCTL_DMB_INIT:
			nmi625_i2c_init();
			break;
		case IOCTL_DMB_BYTE_READ:
		
			break;
		case IOCTL_DMB_WORD_READ:
			
			break;
		case IOCTL_DMB_LONG_READ:
			
			break;
		case IOCTL_DMB_BULK_READ:
			res = copy_from_user((void *)&info, (void *)arg, size);
			res = nmi625_i2c_read(0, (u16)info.dev_addr, (u8 *)(info.buf), info.size);
			res = copy_to_user((void *)arg, (void *)&info, size);
			break;
		case IOCTL_DMB_BYTE_WRITE:
			//copy_from_user((void *)&info, (void *)arg, size);
			//res = BBM_BYTE_WRITE(0, (u16)info.buff[0], (u8)info.buff[1]);
			break;
		case IOCTL_DMB_WORD_WRITE:
			
			break;
		case IOCTL_DMB_LONG_WRITE:
			
			break;
		case IOCTL_DMB_BULK_WRITE:
			res = copy_from_user((void *)&info, (void *)arg, size);
			nmi625_i2c_write(0, (u16)info.dev_addr, (u8 *)(info.buf), info.size);
			break;
		case IOCTL_DMB_TUNER_SELECT:
			
			break;
		case IOCTL_DMB_TUNER_READ:
			
			break;
		case IOCTL_DMB_TUNER_WRITE:
			
			break;
		case IOCTL_DMB_TUNER_SET_FREQ:
			
			break;
		case IOCTL_DMB_POWER_ON:
			printk("DMB_POWER_ON enter..\n");
			dmb_hw_init();
			break;
		case IOCTL_DMB_POWER_OFF:
			printk("DMB_POWER_OFF enter..\n");
			//nmi_i2c_deinit(NULL);
			dmb_hw_deinit();
			break;
/* FIH, Chandler, add for ftm support { */			
		case IOCTL_DMB_FTM_CHIPID:
			
			gpio_direction_output(gpio_isdbt_rstn,0);
			mdelay(50);
			gpio_direction_output(gpio_isdbt_rstn,1);
			mdelay(100);
			
			res = copy_from_user((void *)&info, (void *)arg, size);
			nmi625_i2c_ftm_chip_id((u8 *)(info.buf), info.size);
			res = copy_to_user((void *)arg, (void *)&info, size);
			break;
/* FIH, Chandler, add for ftm support } */			
		default:
			printk("dmb_ioctl : Undefined ioctl command\n");
			res = 1;
			break;
	}
	return res;
}

//#define REWORK_TEST
#ifdef REWORK_TEST
#define GPIO_ISDBT_PWR_EN 0xFF //no use
#define GPIO_ISDBT_RSTn 0xFF
#define GPIO_ATV_RSTn 124 //no use
#define GPIO_TV_CLK_EN 124 //no use
#else
#define GPIO_ISDBT_PWR_EN 131 //no use
#define GPIO_ISDBT_RSTn    88 //8 
#define GPIO_ATV_RSTn 124 //no use
#define GPIO_TV_CLK_EN 124 //no use
#endif
#if 1
#define GPIO_TV_VCORE_EN 93
#define GPIO_TV_VIO_EN   94
#endif

void dmb_hw_setting(void)
{
	int ret=0;

//added by vincent
#if 1
	ret = gpio_request(GPIO_TV_VCORE_EN, "TV_VCORE_EN");
	if(ret) 
        printk("%s(): request GPIO_TV_VCORE_EN failed,ret(%d)\n", __func__, ret);
    udelay(50);

    ret = gpio_direction_output(GPIO_TV_VCORE_EN, 1);
	if(ret) 
        printk("%s(): output GPIO_TV_VCORE_EN failed,ret(%d)\n", __func__, ret);
	ret = gpio_request(GPIO_TV_VIO_EN, "TV_VIO_EN");
	if(ret) 
        printk("%s(): request GPIO_TV_VIO_EN failed,ret(%d)\n", __func__, ret);
    udelay(50);
	ret = gpio_direction_output(GPIO_TV_VIO_EN, 1);
	if(ret) 
        printk("%s(): output GPIO_TV_VIO_EN failed,ret(%d)\n", __func__, ret);
#else
	static struct vreg *vreg_l6,*vreg_s3;
	int rc = 0;

	vreg_l6 = vreg_get(NULL, "wlan3");
	if (IS_ERR(vreg_l6)) {
	  pr_err("%s: vreg_get(%s) failed (%ld)\n",
		  __func__, "nmi_dtv_regulator", PTR_ERR(vreg_l6));
	  return;
	}

	rc = vreg_set_level(vreg_l6, 1200); //VREG_L6
	if (rc) {
	  pr_err("%s: VREG_L6 set_level failed (%d)\n",
		  __func__, rc);
	  return;
	}
	else
	{
		printk("dmb_hw_setting : VREG_L6 set_level \n");
	}


	vreg_s3 = vreg_get(NULL, "msme1");
	if (IS_ERR(vreg_s3)) {
	  pr_err("%s: vreg_get for S3 failed\n", __func__);
	  rc = PTR_ERR(vreg_s3);
	  return;
	}
 
	rc = vreg_set_level(vreg_s3, 1800);
	if (rc) {
	  pr_err("%s: vreg set level failed (%d) for S3\n",
			 __func__, rc);
	  return;
	}
    else
    {
		printk("dmb_hw_setting : vreg_get for S3\n");
	}
	
//end
#endif

	udelay(500);


#if 0
    if (HWREV >= 16)          // Derek: above rev1.0
	{
		gpio_isdbt_pwr_en = GPIO_ISDBT_PWR_EN_REV10;
		gpio_isdbt_rstn = GPIO_ISDBT_RSTn;
		gpio_atv_rstn = GPIO_ATV_RSTn_REV10;
	}
	else if(HWREV >= 13)
	{
		gpio_isdbt_pwr_en = GPIO_ISDBT_PWR_EN;
		gpio_isdbt_rstn = GPIO_ISDBT_RSTn;		
		gpio_atv_rstn = GPIO_ATV_RSTn_REV07;
	}
	else 
	{
		gpio_isdbt_pwr_en = GPIO_ISDBT_PWR_EN;
		gpio_isdbt_rstn = GPIO_ISDBT_RSTn;		
		gpio_atv_rstn = GPIO_ATV_RSTn;
	}
#else
       
		ret=gpio_tlmm_config(GPIO_CFG(GPIO_ISDBT_RSTn, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA), GPIO_CFG_ENABLE);

		if(ret)printk("%s(): gpio_tlmm_config(%d) failed.\n", __func__, ret);

		gpio_isdbt_pwr_en = GPIO_ISDBT_PWR_EN;
		gpio_isdbt_rstn = GPIO_ISDBT_RSTn;		
//		gpio_atv_rstn = GPIO_ATV_RSTn;

#endif

#if 0	
	///////////////////
	ret=gpio_request(gpio_isdbt_pwr_en,"ISDBT_EN");
	if(ret) printk("%s(): request gpio_isdbt_pwr_en failed,ret(%d)\n", __func__, ret);
	
	udelay(50);
	ret=gpio_direction_output(gpio_isdbt_pwr_en,1);
	if(ret) printk("%s(): output gpio_isdbt_pwr_en failed,ret(%d)\n", __func__, ret);
	
	///////////////////
#endif	
	ret=gpio_request(gpio_isdbt_rstn,"ISDBT_RSTn");
	if(ret) printk("%s(): request gpio_isdbt_rstn(%d) failed,ret(%d)\n", __func__, gpio_isdbt_rstn, ret);

	udelay(50);
	ret=gpio_direction_output(gpio_isdbt_rstn,1);
	if(ret) printk("%s(): output gpio_isdbt_rstn(%d) failed,ret(%d)\n", __func__, gpio_isdbt_rstn, ret);

	gpio_set_value(gpio_isdbt_rstn,1);
	mdelay(100);
	printk("%s(): get gpio_isdbt_rstn(%d) , get ret(%d)\n", __func__, gpio_isdbt_rstn, gpio_get_value(gpio_isdbt_rstn));


#if 0
	///////////////////
	ret=gpio_request(gpio_atv_rstn,"ATV_RSTn");
	if(ret) printk("%s(): request gpio_atv_rstn failed,ret(%d)\n", __func__, ret);

    udelay(50);
	ret=gpio_direction_output(gpio_atv_rstn,1);
	if(ret) printk("%s(): output gpio_atv_rstn failed,ret(%d)\n", __func__, ret);
#endif
}


void dmb_hw_setting_gpio_free(void)
{
    /* GPIO free*/
//	gpio_free(gpio_atv_rstn);
#if 1
    gpio_free(GPIO_TV_VCORE_EN);
    gpio_free(GPIO_TV_VIO_EN);
#endif
	gpio_free(gpio_isdbt_rstn);
	gpio_free(gpio_isdbt_pwr_en);	
	}
	
void dmb_hw_init(void)
{
#if 1
    int ret = 0;
	ret = gpio_request(GPIO_TV_VCORE_EN, "TV_VCORE_EN");
	if(ret) 
        printk("%s(): request GPIO_TV_VCORE_EN failed,ret(%d)\n", __func__, ret);
    udelay(50);
    ret = gpio_direction_output(GPIO_TV_VCORE_EN, 1);
	if(ret) 
        printk("%s(): output GPIO_TV_VCORE_EN failed,ret(%d)\n", __func__, ret);
	ret = gpio_request(GPIO_TV_VIO_EN, "TV_VIO_EN");
	if(ret) 
        printk("%s(): request GPIO_TV_VIO_EN failed,ret(%d)\n", __func__, ret);
    udelay(50);
	ret = gpio_direction_output(GPIO_TV_VIO_EN, 1);
	if(ret) 
        printk("%s(): output GPIO_TV_VIO_EN failed,ret(%d)\n", __func__, ret);
#endif
	gpio_set_value(gpio_isdbt_pwr_en, 1);  // ISDBT EN Enable

	if (HWREV == 16)          // Derek: above rev1.0
	{
		gpio_set_value(GPIO_TV_CLK_EN, 1); //Derek.ji 2010.017.27 NMI 26M clk Ensable			
	}
	mdelay(1);
}

void dmb_hw_rst(unsigned long _arg1, unsigned long _arg2)
{
	// _arg1 is the chip selection. if 0, then isdbt. if 1, then atv.
	// _arg2 is the reset polarity. if 0, then low. if 1, then high.
	if(0 == _arg1){
		gpio_set_value(gpio_isdbt_rstn, _arg2);  
		mdelay(10);
		printk("%s(): get gpio (%d)\n",__func__, gpio_get_value(gpio_isdbt_rstn));
	}else {
//		gpio_set_value(gpio_atv_rstn, _arg2); 

	}
}

void dmb_hw_deinit(void)
{
	gpio_set_value(gpio_isdbt_pwr_en, 0);  // ISDBT EN Disable

    if (HWREV == 16)          // Derek: above rev1.0
    	{
			gpio_set_value(GPIO_TV_CLK_EN, 0);	//Derek.ji 2010.017.27 NMI 26M clk Disable					
		}
#if 1
    gpio_direction_output(GPIO_TV_VCORE_EN, 0);
    gpio_direction_output(GPIO_TV_VIO_EN, 0);
    gpio_free(GPIO_TV_VCORE_EN);
    gpio_free(GPIO_TV_VIO_EN);
#endif
}

int dmb_init(void *hal)
{
	int result;

	printk("dmb dmb_init \n");

#if 0	// Lewis
#ifndef REWORK_TEST
	if(!(Project_DMT <=fih_read_product_id_from_orighwid() && fih_read_product_id_from_orighwid() <=Project_DMT)){

		printk("%s(): non-DTV HW, return\n",__func__ );
		return -ENODEV;

	}
#endif
#endif

//

	dmb_hw_setting();

#ifdef  NMI_TEST //def NMI625_HW_CHIP_ID_CHECK for testing
	dmb_hw_rst(0, 0);
	mdelay(100);
	dmb_hw_rst(0, 1);
	mdelay(100);
#else
	dmb_hw_rst(0, 0);
	dmb_hw_deinit();
#endif

	/*misc device registration*/
	result = misc_register(&nmi_misc_device);

//
	/* nmi i2c init	*/
	nmi625_i2c_init();

#ifdef  NMI_TEST //def NMI625_HW_CHIP_ID_CHECK for testing
	nmi625_i2c_read_chip_id();
#else
	dmb_hw_setting_gpio_free(); 
#endif

	if(result < 0)
		return result;

	return 0;
}

void dmb_exit(void)
{
	printk("dmb dmb_exit \n");

	nmi625_i2c_deinit();

	dmb_hw_deinit();

	/*misc device deregistration*/
	misc_deregister(&nmi_misc_device);
}

DECLARE_DYN_LOADER(DRIVER_TYPE_DTV_NMI, DTV_NMI_V1, dmb_init, LEVEL6);

module_exit(dmb_exit);

MODULE_LICENSE("Dual BSD/GPL");

