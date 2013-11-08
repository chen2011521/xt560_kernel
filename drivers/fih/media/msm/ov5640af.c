/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include <fih/dynloader.h>
#include "ov5640af.h"
#include <linux/slab.h>
#include <mach/vreg.h>
#include <mach/pmic.h>
/*flash function*/
#include <linux/completion.h>
#include <linux/hrtimer.h>

#include <fih/leds-pmic8028-proc.h>

int ov5640_m_ledmod=0;
int autoflash_en = 0;
static pid_t ov5640_thread_id;
struct hrtimer ov5640_flashled_timer;
int ov5640_ledtime=16;

int XVCLK = 2400;	// real clock/10000
int preview_sysclk, preview_HTS, preview_VTS,average;
int AE_Target = 52;
int AE_high, AE_low;

int effect_present = -1;
int brightness_present = -1;
int contrast_present = -1;
int ae_mode_present = -1;
int antibanding_present = -1;
int saturation_present = -1;
int sharpness_present = -1;
int wb_present = -1;
int scenemod_present = -1;
int exposure_compensat_present = -1;
int iso_present = -1;

#define OV5640_FLASHLED_DELAY 100// 570 //620 //750 //860 //520
DECLARE_COMPLETION(ov5640_flashled_comp);

extern int proc_comm_set_led(unsigned id, unsigned level);
static int ov5640_snapshot_config(void);
static int ov5640_torch_en(bool torch);
/*flash function*/

static int ov5640_pwdn_gpio;
static int ov5640_reset_gpio;
static struct vreg *vreg_l10= NULL;

#define FALSE 0
#define TRUE 1
#define FLASH_EN_PMU  9
#define TORCH_EN      6
#define OV7692_GPIO_PWD     116

static int OV5640_CSI_CONFIG = 0;

static int ov5640_m_60Hz = FALSE;

struct ov5640_work {
    struct work_struct work;
};
static struct ov5640_work *ov5640_sensorw;
static struct i2c_client    *ov5640_client;
static DECLARE_WAIT_QUEUE_HEAD(ov5640_wait_queue);
DEFINE_MUTEX(ov5640_mutex);
static u8 ov5640_i2c_buf[4];
static u8 ov5640_counter = 0;
static int16_t ov5640_effect = CAMERA_EFFECT_OFF;
static int is_autoflash =0;

struct __ov5640_ctrl 
{
    const struct msm_camera_sensor_info *sensordata;
    int sensormode;
    uint fps_divider; /* init to 1 * 0x00000400 */
    uint pict_fps_divider; /* init to 1 * 0x00000400 */
    u16 curr_step_pos;
    u16 curr_lens_pos;
    u16 init_curr_lens_pos;
    u16 my_reg_gain;
    u16 my_reg_line_count;
    enum msm_s_resolution prev_res;
    enum msm_s_resolution pict_res;
    enum msm_s_resolution curr_res;
    enum msm_s_test_mode  set_test;
};

static struct __ov5640_ctrl *ov5640_ctrl;
//int afinit = -1;
int af_finished = 0;

static int ov5640_i2c_remove(struct i2c_client *client);
static int ov5640_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id);
static int ov5640_sensor_start_af(void);
static int ov5640_set_touchaec(uint32_t x,uint32_t y);

static int ov5640_i2c_txdata(u16 saddr,u8 *txdata,int length)
{
    struct i2c_msg msg[] = {
        {
            .addr  = saddr,
            .flags = 0,
            .len = length,
            .buf = txdata,
        },
    };

    if (i2c_transfer(ov5640_client->adapter, msg, 1) < 0)    return -EIO;
    else return 0;
}

static int ov5640_i2c_write(unsigned short saddr, unsigned int waddr,
                            unsigned short bdata,u8 trytimes)
{
    int rc = -EIO;
    ov5640_counter = 0;
    ov5640_i2c_buf[0] = (waddr & 0xFF00)>>8;
    ov5640_i2c_buf[1] = (waddr & 0x00FF);
    ov5640_i2c_buf[2] = (bdata & 0x00FF);

   while ( (ov5640_counter<trytimes) &&(rc != 0) )
   {
      rc = ov5640_i2c_txdata(saddr, ov5640_i2c_buf, 3);
      if (rc < 0)
      {
          ov5640_counter++;
          printk(KERN_ERR"***Tom i2c_write_w failed,i2c addr=0x%x, command addr = 0x%x, val = 0x%x, s=%d, rc=%d!\n",saddr,waddr, bdata,ov5640_counter,rc);
          msleep(4);
      }
   }
   return rc;
}

static int ov5640_i2c_rxdata(unsigned short saddr,
                             unsigned char *rxdata, int length)
{
    struct i2c_msg msgs[] = {
        {
            .addr   = saddr,
            .flags = 0,
            .len   = 2,
            .buf   = rxdata,
        },
        {
            .addr   = saddr,
            .flags = I2C_M_RD,
            .len   = length,
            .buf   = rxdata,
        },
    };

    //printk(KERN_ERR"CAMERA: i2cR, saddr:0x%x, data:0x%x, length:%d",saddr,*rxdata,length);

    if (i2c_transfer(ov5640_client->adapter, msgs, 2) < 0) {
        printk(KERN_ERR"ov5640_i2c_rxdata failed!\n");
        printk(KERN_ERR"ov5640_i2c_rxdata failed!\n");
        return -EIO;
    }

    return 0;
}

static int32_t ov5640_i2c_read_byte(unsigned short   saddr,
                                    unsigned int raddr, unsigned int *rdata)
{
    int rc = 0;
    unsigned char buf[2];
    //printk(KERN_ERR"+ov5640_i2c_read\n");
    memset(buf, 0, sizeof(buf));

    buf[0] = (raddr & 0xFF00)>>8;
    buf[1] = (raddr & 0x00FF);

    rc = ov5640_i2c_rxdata(saddr, buf, 1);
    if (rc < 0) return rc;
    *rdata = buf[0];

    if (rc < 0) 
		printk(KERN_ERR"ov5640_i2c_read failed!\n");

    //printk(KERN_ERR"-ov5640_i2c_read\n");
    return rc;
}

static int32_t OV5640_WritePRegs(POV5640_WREG pTb,int32_t len)
{
    int32_t i,ret = 0;
    uint32_t regv;
    for(i=0;i<len;i++)
    {
        if(0==pTb[i].mask)
        {
            //printk(KERN_ERR"--CAMERA-- i2c write, addr:0x%x, data:0x%x\n",
            //    pTb[i].addr, pTb[i].data);    
            ov5640_i2c_write(ov5640_client->addr,pTb[i].addr,pTb[i].data,10);
        }
        else
        {
            //printk(KERN_ERR"--CAMERA-- i2c write with MASK, addr:0x%x, data:0x%x, MASK:0x%x\n",
            //    pTb[i].addr, pTb[i].data, pTb[i].mask);    
            ov5640_i2c_read_byte(ov5640_client->addr,pTb[i].addr,&regv);
            //printk(KERN_ERR"--===================CAMERA-- i2c read before write,addr:0x%x, data:0x%x\n",
                //pTb[i].addr, regv); 

            regv &= pTb[i].mask;
            regv |= (pTb[i].data & (~pTb[i].mask));
            ov5640_i2c_write(ov5640_client->addr,pTb[i].addr,regv,10);
            //for test
            // ov5640_i2c_read_byte(ov5640_client->addr,pTb[i].addr,&regv);
            //printk(KERN_ERR"--===================CAMERA-- i2c read after write,addr:0x%x, data:0x%x\n",
                //pTb[i].addr, regv);
        }
    }
    return ret;
}

static void camera_sw_power_onoff(u8 v)
{
    if (v == 0) {
        printk(KERN_ERR"camera_sw_power_onoff: down\n");
        ov5640_i2c_write(ov5640_client->addr, 0x4202, 0x0f, 10); //stream off
                     msleep(50);
    }
    else {
        printk(KERN_ERR"camera_sw_power_onoff: on\n");
        ov5640_i2c_write(ov5640_client->addr, 0x4202, 0x00, 10); //stream on
    }
}
/*
static void camera_power_onoff(u8 v)
{
    if (v==1)
    {
        printk(KERN_ERR"--CAMERA-- power on~!!\n");
        afinit = 1;    //af state
    }
    else
    {
        printk(KERN_ERR"--CAMERA-- power off~!!\n");
        afinit = 0;    //af state
    }
}
*/
static void ov5640_power_off(void)
{
    printk(KERN_ERR"--CAMERA-- %s ... (Start...)\n",__func__);

//    camera_power_onoff(0);
    /* SET PWDN & RESET TO LOW AFTER POWEROFF*/
    gpio_set_value(ov5640_pwdn_gpio, 1);
    gpio_set_value(ov5640_reset_gpio, 1);    

    printk(KERN_ERR"--CAMERA-- %s ... (End...)\n",__func__);
}

static void ov5640_power_on(void)
{
    int rc=0;
    printk(KERN_ERR"--CAMERA-- %s ... (Start...)\n",__func__);
    rc = gpio_tlmm_config(GPIO_CFG(OV7692_GPIO_PWD, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    gpio_request(OV7692_GPIO_PWD,"ov7692_pwd");
    /*powerdown = 1*/
    rc = gpio_direction_output(OV7692_GPIO_PWD, 1);
    gpio_free(OV7692_GPIO_PWD); 
    /* SET PWDN TO HIGH,RESET TO LOW BEFORE POWERON*/
    gpio_set_value(ov5640_pwdn_gpio, 1);
    gpio_set_value(ov5640_reset_gpio, 0);

//    camera_power_onoff(1);

    printk(KERN_ERR"--CAMERA-- %s ... (End...)\n",__func__);
}

static void ov5640_af_power_onoff(u8 v)
{
    int rc=0;
    printk(KERN_ERR"--CAMERA-- %s ... (Start...)\n",__func__);

    if (vreg_l10 == NULL) {
        vreg_l10 = vreg_get(NULL, "emmc");        
        if (IS_ERR(vreg_l10)) {
            pr_err("%s: vreg_get(%s) failed (%ld)\n",
                   __func__, "emmc", PTR_ERR(vreg_l10));
            return;
        }

        rc = vreg_set_level(vreg_l10, 3050);
        if (rc) {
            pr_err("%s: L10 set level failed (%d)\n",
                   __func__, rc);
        }
    }
    if (1==v) {
        rc = vreg_enable(vreg_l10);
        if (rc) {
            printk(KERN_ERR "%s: L10 enable failed (%d)\n",
                   __func__, rc);
        }
    }
    else {
        rc = vreg_disable(vreg_l10);
        if (rc) {
            pr_err("%s: L10 disable failed (%d)\n",
                   __func__, rc);
        }
    }

    printk(KERN_ERR"--CAMERA-- %s ... (End...)\n",__func__);
}

static int ov5640_probe_readID(const struct msm_camera_sensor_info *data)
{
    int rc = 0;    
    u32 device_id_high = 0;
    u32 device_id_low = 0;

    //printk(KERN_ERR"--CAMERA-- %s sensor poweron,begin to read ID!\n",__func__);

    // (6) Read Device ID
    rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x300A, &device_id_high);    //0x300A ,sensor ID register

    printk(KERN_ERR"--CAMERA-- %s ok , device_id_high = 0x%x\r\n",__func__ , device_id_high);
   if (rc < 0)
   {
       printk(KERN_ERR"--CAMERA-- %s ok , readI2C failed,rc = 0x%x\r\n",__func__ , rc);
       goto done;    
   }  

   rc = ov5640_i2c_read_byte(ov5640_client->addr, 0x300B, &device_id_low);    //0x300B ,sensor ID register
   printk(KERN_ERR"--CAMERA-- %s ok , device_id_low = 0x%x\r\n",__func__ , device_id_low);
   if (rc < 0)
   {
       printk(KERN_ERR"--CAMERA-- %s ok , readI2C failed,rc = 0x%x\r\n",__func__ , rc);
       goto done;  
   }   

   if((device_id_high<<8)+device_id_low != OV5640_SENSOR_ID)    //0x5640, ov5640 chip id
   {
       rc = -EINVAL;
       printk(KERN_ERR"--CAMERA-- %s ok , device id error, should be 0x%x\r\n",__func__ , OV5640_SENSOR_ID);
       goto done;  
   }
   else
   {
       printk(KERN_ERR"--CAMERA-- %s ok , device id=0x%x\n",__func__,OV5640_SENSOR_ID);
       goto done;  
    }
done:
        return rc;
}
/*
static int  OV5640_auto_focus(void)
{
	 //int temp;
	unsigned int REG0x3029;
	 // focus
			 	 //OV5640_write_i2c(0x3022, 0x03);
	ov5640_i2c_write(ov5640_client->addr,0x3022, 0x03, 10);

	 while(1)
	 {
		 // check status
		 //temp = OV5640_read_i2c(0x3029);
		ov5640_i2c_read_byte(ov5640_client->addr,0x3029,&REG0x3029);
		 if (REG0x3029 ==0x10) return 0;	// focus completed
	 }
	  return 1;

	  //msleep(100);
	 msleep(40);

}
*/
static int OV5640_get_shutter(void)
{
	 // read shutter, in number of line period
	 int shutter;
	 int REG3500,REG3501,REG3502;

	 
	 ov5640_i2c_read_byte(ov5640_client->addr,0x3500,&REG3500);
	 ov5640_i2c_read_byte(ov5640_client->addr,0x3501,&REG3501);
	 ov5640_i2c_read_byte(ov5640_client->addr,0x3502,&REG3502);
	 shutter = (REG3500&0x0F)<<12 ;
	 shutter += REG3501<<4 ;
	 shutter +=  REG3502>>4 ;
	 return shutter;
}

static int OV5640_set_shutter(int shutter)
{
	 // write shutter, in number of line period
	 int temp;
	 
	 shutter = shutter & 0xffff;

	 temp = shutter & 0x0f;
	 temp = temp<<4;
	 //OV5640_write_i2c(0x3502, temp);
	 ov5640_i2c_write(ov5640_client->addr,0x3502, temp, 0xff);

	 temp = shutter & 0xfff;
	 temp = temp>>4;
	 //OV5640_write_i2c(0x3501, temp);
	 ov5640_i2c_write(ov5640_client->addr,0x3501, temp, 0xff);

	 temp = shutter>>12;
	 //OV5640_write_i2c(0x3500, temp);
 	 ov5640_i2c_write(ov5640_client->addr,0x3500, temp, 0xff);

	 return 0;
}

static int OV5640_get_gain16(void)
{
	 // read gain, 16 = 1x
	 int gain16;
 	 int REG350a,REG350b;

	 ov5640_i2c_read_byte(ov5640_client->addr,0x350a,&REG350a);
	 ov5640_i2c_read_byte(ov5640_client->addr,0x350b,&REG350b);
	 gain16 = REG350a & 0x03;
	 gain16 = (gain16<<8) + REG350b;

	 return gain16;
}

static int OV5640_set_gain16(int gain16)
{
	 // write gain, 16 = 1x
	 int temp;
	 gain16 = gain16 & 0x3ff;

	 temp = gain16 & 0xff;
	 //OV5640_write_i2c(0x350b, temp);
	 ov5640_i2c_write(ov5640_client->addr,0x350b, temp, 0xff);

	 temp = gain16>>8;
	 //OV5640_write_i2c(0x350a, temp);
	 ov5640_i2c_write(ov5640_client->addr,0x350a, temp, 0xff);

	 return 0;
}

static int OV5640_get_light_frequency(void)
{
	 // get banding filter value
	 int temp, temp1, light_frequency;

	 //temp = OV5640_read_i2c(0x3c01);
	 ov5640_i2c_read_byte(ov5640_client->addr,0x3c01,&temp);

	 if (temp & 0x80) {
		 // manual
		 //temp1 = OV5640_read_i2c(0x3c00);
		 ov5640_i2c_read_byte(ov5640_client->addr,0x3c00,&temp1);
		 if (temp1 & 0x04) {
			 // 50Hz
			 light_frequency = 50;
		 }
		 else {
			 // 60Hz
			 light_frequency = 60;
		 }
	 }
	 else {
		 // auto
		 //temp1 = OV5640_read_i2c(0x3c0c);
		 ov5640_i2c_read_byte(ov5640_client->addr,0x3c0c,&temp1);
		 if (temp1 & 0x01) {
			 // 50Hz
			 light_frequency = 50;
		 }
		 else {
			 // 60Hz
		 }
	 }
	 return light_frequency;
}

static void OV5640_set_night_mode(int NightMode)
{

	 int temp;

	 switch (NightMode)
	 {

	 case 0://Off
		 //temp = OV5640_read_i2c(0x3a00);
		 ov5640_i2c_read_byte(ov5640_client->addr,0x3a00,&temp);
		 temp = temp & 0xfb;			// night mode off, bit[2] = 0
		 //OV5640_write_i2c(0x3a00, temp);
		 ov5640_i2c_write(ov5640_client->addr,0x3a00, temp, 0xff);
		 break;

	 case 1:// On
		 //temp = OV5640_read_i2c(0x3a00);
		 ov5640_i2c_read_byte(ov5640_client->addr,0x3a00,&temp);
		 temp = temp | 0x04;			// night mode on, bit[2] = 1
		// OV5640_write_i2c(0x3a00, temp);
		 ov5640_i2c_write(ov5640_client->addr,0x3a00, temp, 0xff);

		 break;

	 default:
		 break;

	 }
}

static int OV5640_get_HTS(void)
{
	 // read HTS from register settings
	 int HTS;
	 int REG380c,REG380d;


	 ov5640_i2c_read_byte(ov5640_client->addr,0x380c,&REG380c);
	 ov5640_i2c_read_byte(ov5640_client->addr,0x380d,&REG380d);

	 HTS = REG380c<<8; 
	 HTS += REG380d ;

	 return HTS;
}

static int OV5640_get_VTS(void)
{
	 // read VTS from register settings
	 int VTS;
	 int REG380e,REG380f;


	 ov5640_i2c_read_byte(ov5640_client->addr,0x380e,&REG380e);
	 ov5640_i2c_read_byte(ov5640_client->addr,0x380f,&REG380f);

	 VTS = (REG380e<<8) + REG380f;

	 return VTS;
}


static int OV5640_set_VTS(int VTS)
{
	 // write VTS to registers
	 int temp;

	 temp = VTS & 0xff;
	 //OV5640_write_i2c(0x380f, temp);
	 ov5640_i2c_write(ov5640_client->addr,0x380f,temp,0xff);

	 temp = VTS>>8;
	 //OV5640_write_i2c(0x380e, temp);
	 ov5640_i2c_write(ov5640_client->addr,0x380e,temp,0xff);

	 return 0;
}


static int OV5640_get_sysclk(void)
{
	 // calculate sysclk
	 int temp1, temp2;
	 int Multiplier, PreDiv, VCO, SysDiv, Pll_rdiv, sclk_rdiv, sysclk;
	 int Bit_div2x = 0;

	 int sclk_rdiv_map[] = {
		 1, 2, 4, 8};


	 ov5640_i2c_read_byte(ov5640_client->addr,0x3034, &temp1);
	

	 temp2 = temp1 & 0x0f;
	 if (temp2 == 8 || temp2 == 10) {
		 Bit_div2x = temp2 / 2;
	 }

	 //temp1 = OV5640_read_i2c(0x3035);
	 ov5640_i2c_read_byte(ov5640_client->addr,0x3035, &temp1);
	 SysDiv = temp1>>4;
	 if(SysDiv == 0) {
		 SysDiv = 16;
	 }

	 //temp1 = OV5640_read_i2c(0x3036);
	 ov5640_i2c_read_byte(ov5640_client->addr,0x3036, &temp1);
	 Multiplier = temp1;

	 //temp1 = OV5640_read_i2c(0x3037);
	 ov5640_i2c_read_byte(ov5640_client->addr,0x3037, &temp1);
	 PreDiv = temp1 & 0x0f;
	 Pll_rdiv = ((temp1 >> 4) & 0x01) + 1;

	 //temp1 = OV5640_read_i2c(0x3108);
         ov5640_i2c_read_byte(ov5640_client->addr,0x3108, &temp1);
	 temp2 = temp1 & 0x03;
	 sclk_rdiv = sclk_rdiv_map[temp2]; 

	 VCO = XVCLK * Multiplier / PreDiv;

	 sysclk = VCO / SysDiv / Pll_rdiv * 2 / Bit_div2x / sclk_rdiv;

	 return sysclk;
}


static int OV5640_set_AE_target(int target)
{
	 // stable in high
	 int fast_high, fast_low;
	 AE_low = target * 23 / 25;	// 0.92
	 AE_high = target * 27 / 25;	// 1.08

	 fast_high = AE_high<<1;
	 if(fast_high>255)
		 fast_high = 255;

	 fast_low = AE_low>>1;

	 ov5640_i2c_write(ov5640_client->addr,0x3a0f,AE_high,0xff);
	 ov5640_i2c_write(ov5640_client->addr,0x3a10,AE_low,0xff);
	 ov5640_i2c_write(ov5640_client->addr,0x3a1b,AE_high,0xff);
	 ov5640_i2c_write(ov5640_client->addr,0x3a1e,AE_low,0xff);
	 ov5640_i2c_write(ov5640_client->addr,0x3a11,fast_high,0xff);
	 ov5640_i2c_write(ov5640_client->addr,0x3a1f,fast_low,0xff);

	//ov5640_i2c_read_byte(ov5640_client->addr,0x3029,&REG0x3029);

	 return 0;
}

static void OV5640_set_bandingfilter(void)
{
	 int preview_VTS,temp;
	 int band_step60, max_band60, band_step50, max_band50;

	 // read preview PCLK
	 preview_sysclk = OV5640_get_sysclk();
	 printk(KERN_ERR"--CAMERA-- %s (Start...), preview_sysclk =%d \n",__func__,preview_sysclk );

	 // read preview HTS
	 preview_HTS = OV5640_get_HTS();

	 // read preview VTS
	 preview_VTS = OV5640_get_VTS();

	 // calculate banding filter
	 // 60Hz
	 band_step60 = preview_sysclk * 100/preview_HTS * 100/120;


	 ov5640_i2c_write(ov5640_client->addr,0x3a0a,(band_step60 >> 8),0xff);
	 ov5640_i2c_write(ov5640_client->addr,0x3a0b,(band_step60 & 0xff),0xff);

	 //max_band60 = int((preview_VTS-4)/band_step60);
	 max_band60 = (preview_VTS-4)/band_step60;
	 //OV5640_write_i2c(0x3a0d, max_band60);
	 ov5640_i2c_write(ov5640_client->addr,0x3a0d,max_band60,0xff);

	 // 50Hz
	 band_step50 = preview_sysclk * 100/preview_HTS; 

	 ov5640_i2c_write(ov5640_client->addr,0x3a08,(band_step50 >> 8),0xff);
 	 ov5640_i2c_write(ov5640_client->addr,0x3a09,(band_step50 & 0xff),0xff);

	 //max_band50 = int((preview_VTS-4)/band_step50);
	 max_band50 = (preview_VTS-4)/band_step50;
	 ov5640_i2c_write(ov5640_client->addr,0x3a0e,max_band50,0xff);

	ov5640_i2c_read_byte(ov5640_client->addr,0x3a08, &temp);
	printk(KERN_ERR"--CAMERA-- %s (Start...), 0x3a08=%x \n",__func__,temp);
	ov5640_i2c_read_byte(ov5640_client->addr,0x3a09, &temp);
	printk(KERN_ERR"--CAMERA-- %s (Start...), 0x3a09=%x \n",__func__,temp);
	ov5640_i2c_read_byte(ov5640_client->addr,0x3a0e, &temp);
	printk(KERN_ERR"--CAMERA-- %s (Start...), 0x3a0e=%x \n",__func__,temp);
}

static int ov5640_set_wb_oem(int param)
{
    int rc;
    printk(KERN_ERR"--CAMERA-- %s param=%d\r\n",__func__,param);

    if(param == CAMERA_WB_AUTO)
    {
        rc = OV5640Core_WritePREG(ov5640_wb_auto);
    }
    else if(param == CAMERA_WB_INCANDESCENT)
    {
        rc = OV5640Core_WritePREG(ov5640_wb_incandescent);
    }
    else if(param == CAMERA_WB_DAYLIGHT)
    {
        rc = OV5640Core_WritePREG(ov5640_wb_daylight);
    }
    else if(param == CAMERA_WB_FLUORESCENT)
    {
        rc = OV5640Core_WritePREG(ov5640_wb_fluorsent);
    }
    else if(param == CAMERA_WB_CLOUDY_DAYLIGHT)
    {
        rc = OV5640Core_WritePREG(ov5640_wb_cloudy);
    }
    return 1;

}
static int ov5640_video_config(void)
{
    int rc = 0;

    printk(KERN_ERR"--CAMERA-- ov5640_video_config\n");
	
	if(is_autoflash == 1)
	{
		pmic_flash_led_set_current(0);
	}    

    OV5640Core_WritePREG(ov5640_preview_tbl);
	//add by yisong
	// calculate banding filter
	 OV5640_set_bandingfilter();
     ov5640_set_wb_oem(wb_present);
	OV5640_set_AE_target(AE_Target);

    return rc;
}

#if 0
static int ov5640_get_preview_exposure_gain(void)
{
    int rc = 0;
    unsigned int ret_l,ret_m,ret_h;
            
    ov5640_i2c_write(ov5640_client->addr,0x3503, 0x07,10);

    //get preview exp & gain
    ret_h = ret_m = ret_l = 0;
    ov5640_preview_exposure = 0;
    ov5640_i2c_read_byte(ov5640_client->addr,0x3500, &ret_h);
    ov5640_i2c_read_byte(ov5640_client->addr,0x3501, &ret_m);
    ov5640_i2c_read_byte(ov5640_client->addr,0x3502, &ret_l);
    ov5640_preview_exposure = (ret_h << 12) + (ret_m << 4) + (ret_l >> 4);
    printk(KERN_ERR"preview_exposure=%d\n", ov5640_preview_exposure);

    ret_h = ret_m = ret_l = 0;
    ov5640_preview_maxlines = 0;
    ov5640_i2c_read_byte(ov5640_client->addr,0x380e, &ret_h);
    ov5640_i2c_read_byte(ov5640_client->addr,0x380f, &ret_l);
    ov5640_preview_maxlines = (ret_h << 8) + ret_l;
    printk(KERN_ERR"Preview_Maxlines=%d\n", ov5640_preview_maxlines);

    //Read back AGC Gain for preview
    ov5640_gain = 0;
    ov5640_i2c_read_byte(ov5640_client->addr,0x350b, &ov5640_gain);
    printk(KERN_ERR"Gain,0x350b=0x%x\n", ov5640_gain);

    return rc;
}

static int ov5640_set_capture_exposure_gain(void)
{
    int rc = 0;
   //calculate capture exp & gain
   
    unsigned char ExposureLow,ExposureMid,ExposureHigh;
    unsigned int ret_l,ret_m,ret_h,Lines_10ms;
    unsigned short ulCapture_Exposure,iCapture_Gain;
    unsigned int ulCapture_Exposure_Gain,Capture_MaxLines;

    ret_h = ret_m = ret_l = 0;
    ov5640_i2c_read_byte(ov5640_client->addr,0x380e, &ret_h);
    ov5640_i2c_read_byte(ov5640_client->addr,0x380f, &ret_l);
    Capture_MaxLines = (ret_h << 8) + ret_l;
    printk(KERN_ERR"Capture_MaxLines=%d\n", Capture_MaxLines);

    if(ov5640_m_60Hz == TRUE)
    {
    	Lines_10ms = g_Capture_Framerate * Capture_MaxLines/12000;
    }
    else
    {
    	Lines_10ms = g_Capture_Framerate * Capture_MaxLines/10000;
    }


    if(ov5640_preview_maxlines == 0)
    {
    	ov5640_preview_maxlines = 1;
    }

    ulCapture_Exposure = (ov5640_preview_exposure*(g_Capture_Framerate)*(Capture_MaxLines)) 
							/(((ov5640_preview_maxlines)*(g_Preview_FrameRate)));

    iCapture_Gain = ov5640_gain;

    ulCapture_Exposure_Gain = ulCapture_Exposure * iCapture_Gain; 

    if(ulCapture_Exposure_Gain < Capture_MaxLines*16)
    {
	    ulCapture_Exposure = ulCapture_Exposure_Gain/16;

	    if (ulCapture_Exposure > Lines_10ms)
	    {
	      ulCapture_Exposure /= Lines_10ms;
	      ulCapture_Exposure *= Lines_10ms;
	    }
    }
    else
    {
    	ulCapture_Exposure = Capture_MaxLines;
    }

    if(ulCapture_Exposure == 0)
    {
    	ulCapture_Exposure = 1;
    }

    iCapture_Gain = (ulCapture_Exposure_Gain * 2 / ulCapture_Exposure + 1) / 2;

    ExposureLow = ((unsigned char)ulCapture_Exposure) << 4;

    ExposureMid = (unsigned char)(ulCapture_Exposure >> 4) & 0xff;

    ExposureHigh = (unsigned char)(ulCapture_Exposure >> 12);


    // set capture exp & gain

    ov5640_i2c_write(ov5640_client->addr,0x350b, iCapture_Gain, 10);
	ov5640_i2c_write(ov5640_client->addr,0x3502, ExposureLow, 10);
    ov5640_i2c_write(ov5640_client->addr,0x3501, ExposureMid, 10);
    ov5640_i2c_write(ov5640_client->addr,0x3500, ExposureHigh, 10);
  
    printk(KERN_ERR"iCapture_Gain=%d\n", iCapture_Gain);
    printk(KERN_ERR"ExposureLow=%d\n", ExposureLow);
    printk(KERN_ERR"ExposureMid=%d\n", ExposureMid);
    printk(KERN_ERR"ExposureHigh=%d\n", ExposureHigh);

    msleep(250);

    return rc;
}
#endif 

#if 0
static int ov5640_snapshot_config(void)
{
    int rc = 0;
    unsigned int R0x350bpreview;
    unsigned int R0x5690preview;

    ov5640_get_preview_exposure_gain();//Step1: get preview exposure and gain


    ov5640_i2c_read_byte(ov5640_client->addr,0x5690,&R0x5690preview);
    ov5640_i2c_read_byte(ov5640_client->addr,0x350b,&R0x350bpreview);        
    printk(KERN_ERR"--CAMERA-- GAIN VALUE : 0x5690=0x%x 0x350b=0x%x\n", R0x5690preview, R0x350bpreview);
#if 0
    if (tmp2 > 0) {
        pmic_flash_led_set_current(0);
    }
    else {
        pmic_flash_led_set_current(100);
    }
#endif
     printk(KERN_ERR"--CAMERA-- ov5640_m_ledmod=%d \n",ov5640_m_ledmod);

    if ((ov5640_m_ledmod==2) ||autoflash_en == 1/*((ov5640_m_ledmod==1)&&(R0x5690preview <= 0x30))*/) {
			hrtimer_cancel(&ov5640_flashled_timer);
			hrtimer_start(&ov5640_flashled_timer,
					ktime_set(0, 0),
					HRTIMER_MODE_REL);
			if (hrtimer_active(&ov5640_flashled_timer))
				  printk(KERN_ERR"--CAMERA--hrtimer_active- \n");

            msleep(85);
    }

    OV5640Core_WritePREG(ov5640_capture_tbl);//Step2: change to full size
    ov5640_set_capture_exposure_gain();//Step3: calculate and set capture exposure and gain

    if ((ov5640_m_ledmod==2) ||autoflash_en == 1/*((ov5640_m_ledmod==1)&&(R0x5690preview <= 0x30))*/) {
            printk(KERN_ERR"--CAMERA-- flashlight -!!!!-- \n");
			hrtimer_start(&ov5640_flashled_timer,
				ktime_set(OV5640_FLASHLED_DELAY / 1000, (OV5640_FLASHLED_DELAY % 1000) * 1000000),
				HRTIMER_MODE_REL);

			if (hrtimer_active(&ov5640_flashled_timer))
				 printk(KERN_ERR"--CAMERA--hrtimer_active-111-- \n");
    }

    return rc;
}
#endif

static int ov5640_setting(enum msm_s_reg_update rupdate,enum msm_s_setting rt)
{
    int rc = -EINVAL;
    int tmp = 0;
	struct msm_camera_csi_params ov5640_csi_params;
    unsigned int af_ack;
    int i=0;

    printk(KERN_ERR"--CAMERA-- %s (Start...), rupdate=%d  rt = %d \n",__func__,rupdate , rt);

    rc = ov5640_i2c_write(ov5640_client->addr, 0x4800, 0x24, 10);
    msleep(10);

    camera_sw_power_onoff(0); //standby
    switch (rupdate) {
    case S_UPDATE_PERIODIC:



            if ((rt == S_RES_CAPTURE) ||
                (OV5640_CSI_CONFIG == 0) ) {
                msleep(66);
            }
            else{
            }

        if (!OV5640_CSI_CONFIG) {
        ov5640_csi_params.lane_cnt = 2;
        ov5640_csi_params.data_format = CSI_8BIT;
        ov5640_csi_params.lane_assign = 0xe4;
        ov5640_csi_params.dpcm_scheme = 0;
        ov5640_csi_params.settle_cnt = 0x6;

        printk(KERN_ERR"%s: msm_camio_csi_config\n", __func__);

        rc = msm_camio_csi_config(&ov5640_csi_params);              
        

        msleep(20);
        OV5640_CSI_CONFIG = 1;
        }
        if (S_RES_PREVIEW == rt) {
            rc = ov5640_video_config();
		camera_sw_power_onoff(1);
        }
        else if (S_RES_CAPTURE == rt) {
		camera_sw_power_onoff(1);
            rc = ov5640_snapshot_config();
        }
       // camera_sw_power_onoff(1); //on

 //       msleep(10);
        rc = ov5640_i2c_write(ov5640_client->addr, 0x4800, 0x04, 10);

        if (S_RES_PREVIEW == rt && (1==af_finished)) {
            //ov5640_i2c_write(ov5640_client->addr,OV5640_CMD_ACK,0x01,10);
            //ov5640_i2c_write(ov5640_client->addr,OV5640_CMD_MAIN,0x12,10);
            msleep(5);

            ov5640_i2c_write(ov5640_client->addr,OV5640_CMD_ACK,0x01,10);
            ov5640_i2c_write(ov5640_client->addr,OV5640_CMD_MAIN,0x08,10);
            for (i=0;i<50;i++) {
                ov5640_i2c_read_byte(ov5640_client->addr,OV5640_CMD_ACK,&af_ack);
                if (af_ack == 0)
                    break;
                msleep(50);
            }
            printk(KERN_ERR"--CAMERA AF release -- %s i=%d,af_ack = 0x%x\n", __func__, i,af_ack);
        }

        break; /* UPDATE_PERIODIC */

    case S_REG_INIT:
        printk(KERN_ERR"--CAMERA-- S_REG_INIT (Start)\n");
         rc = ov5640_i2c_write(ov5640_client->addr, 0x3103, 0x11, 10);
         rc = ov5640_i2c_write(ov5640_client->addr, 0x3008, 0x82, 10);
         msleep(5);
        //set sensor init setting
        printk(KERN_ERR"%s: set sensor init setting\n", __func__);
        rc = OV5640Core_WritePREG(ov5640_init_tbl);    
        //set image quality setting
        printk(KERN_ERR"%s: set image quality setting\n", __func__);
        rc = OV5640Core_WritePREG(ov5640_init_iq_tbl);

        //msleep(10);
       rc =ov5640_i2c_read_byte(ov5640_client->addr, 0x4740, &tmp);
       CDBG("--CAMERA-- init 0x4740 value=0x%x\n", tmp);

       if (tmp != 0x21) {
           rc = ov5640_i2c_write(ov5640_client->addr, 0x4740, 0x21, 10);
           msleep(10);
           rc =ov5640_i2c_read_byte(ov5640_client->addr, 0x4740, &tmp);
           CDBG("--CAMERA-- WG 0x4740 value=0x%x\n", tmp);
       }
       /*
        if (afinit == 1) {
            printk(KERN_ERR"%s--CAMERA-- AF_init~!!\n",__func__);
            rc = OV5640Core_WritePREG(ov5640_afinit_tbl_1);

            mdelay(5);
            rc = ov5640_i2c_txdata(ov5640_client->addr,(u8 *)ov5640_afinit_tbl_2,len);
            mdelay(5);
            rc=OV5640Core_WritePREG(ov5640_afinit_tbl_3);
            //ov5640_i2c_read_byte(ov5640_client->addr,OV5640_CMD_FW_STATUS,&af_st);
            //printk(KERN_ERR"--jenny CAMERA-- %s af_st = 0x%x\n", __func__, af_st);
            afinit = 0;
        }
        */
        /* reset fps_divider */
        ov5640_ctrl->fps_divider = 1 * 0x0400;
        printk(KERN_ERR"--CAMERA-- S_REG_INIT (End)\n");
        break; /* case REG_INIT: */

    default:
        break;
    } /* switch (rupdate) */

    printk(KERN_ERR"--CAMERA-- %s (End), rupdate=%d \n",__func__,rupdate);

    return rc;
}

static int ov5640_sensor_open_init(const struct msm_camera_sensor_info *data)
{
    int rc = -ENOMEM;
    printk(KERN_ERR"--CAMERA-- %s\n",__func__);

    ov5640_ctrl = kzalloc(sizeof(struct __ov5640_ctrl), GFP_KERNEL);
    if (!ov5640_ctrl)
    {
        printk(KERN_ERR"--CAMERA-- kzalloc ov5640_ctrl error !!\n");
        kfree(ov5640_ctrl);
        return rc;
    }
    ov5640_ctrl->fps_divider = 1 * 0x00000400;
    ov5640_ctrl->pict_fps_divider = 1 * 0x00000400;
    ov5640_ctrl->set_test = S_TEST_OFF;
    ov5640_ctrl->prev_res = S_QTR_SIZE;
    ov5640_ctrl->pict_res = S_FULL_SIZE;
    
    if (data) ov5640_ctrl->sensordata = data;

    //msm_camio_camif_pad_reg_reset();
    //mdelay(5);
    // 
    ov5640_af_power_onoff(1);
	// (4) Power On
//	ov5640_power_on();

	//printk(KERN_ERR"%s: msm_camio_clk_rate_set\n", __func__);

	/* SENSOR NEED MCLK TO DO I2C COMMUNICTION, OPEN CLK FIRST*/
	msm_camio_clk_rate_set(24000000);

	mdelay(10);

	//printk(KERN_ERR"%s: gpio pwd and reset\n", __func__);

	gpio_set_value(ov5640_pwdn_gpio, 0);   //enable power down pin
#if 0
	gpio_set_value(ov5640_reset_gpio, 1);   //reset camera reset pin
	mdelay(5);
	gpio_set_value(ov5640_reset_gpio, 0);
	mdelay(5);
	gpio_set_value(ov5640_reset_gpio, 1);
	mdelay(5);
#endif
/*
    rc = ov5640_probe_readID(data);
    if (rc < 0)
        goto init_fail;
*/
	ov5640_i2c_write(ov5640_client->addr,0x300E,0x45,10);
    if (ov5640_ctrl->prev_res == S_QTR_SIZE)
        rc = ov5640_setting(S_REG_INIT, S_RES_PREVIEW);
    else
        rc = ov5640_setting(S_REG_INIT, S_RES_CAPTURE);

    if (rc < 0)
        goto init_fail;

    OV5640_CSI_CONFIG = 0;
    printk(KERN_ERR"--CAMERA--re_init_sensor ok!!\n");
        return rc;
init_fail:
     printk(KERN_ERR"--CAMERA-- %s : init failed. rc = %d\n",__func__,rc);
     kfree(ov5640_ctrl);
    return rc;
}

static int ov5640_sensor_release(void)
{
    printk(KERN_ERR"--CAMERA--ov5640_sensor_release!!\n");

    mutex_lock(&ov5640_mutex);

    ov5640_i2c_write(ov5640_client->addr,0x300E,0x5d,10);

    ov5640_af_power_onoff(0);
    ov5640_power_off();
    ov5640_torch_en(0);
    complete(&ov5640_flashled_comp);
//    gpio_set_value(ov5640_ctrl->sensordata->sensor_reset, 1);
    
    kfree(ov5640_ctrl);
    ov5640_ctrl = NULL;

    OV5640_CSI_CONFIG = 0;

    mutex_unlock(&ov5640_mutex);
    return 0;
}


                   
static const struct i2c_device_id ov5640_i2c_id[] = {
    {"ov5640af", 0},{}
};

static int ov5640_i2c_remove(struct i2c_client *client)
{
   return 0;
}


static int ov5640_init_client(struct i2c_client *client)
{
   /* Initialize the MSM_CAMI2C Chip */
   init_waitqueue_head(&ov5640_wait_queue);
   return 0;
}


static long ov5640_set_effect(int mode, int effect)
{
    long rc = 0;

    printk(KERN_ERR"--CAMERA-- %s ...(Start)\n",__func__);

    switch (effect) {
    case CAMERA_EFFECT_OFF: {
            ov5640_i2c_write(ov5640_client->addr,0x4202,0x0f,10);
            msleep(50);
            OV5640Core_WritePREG(ov5640_effect_off_tbl);
            ov5640_i2c_write(ov5640_client->addr,0x4202,0x00,10);
        }
        break;

    case CAMERA_EFFECT_MONO: {
            ov5640_i2c_write(ov5640_client->addr,0x4202,0x0f,10);
            msleep(50);
            OV5640Core_WritePREG(ov5640_effect_mono_tbl);
            ov5640_i2c_write(ov5640_client->addr,0x4202,0x00,10);
        }
        break;

    case CAMERA_EFFECT_NEGATIVE: {
            ov5640_i2c_write(ov5640_client->addr,0x4202,0x0f,10);
            msleep(50);
            OV5640Core_WritePREG(ov5640_effect_negative_tbl);    
            ov5640_i2c_write(ov5640_client->addr,0x4202,0x00,10);
        }
        break;

    case CAMERA_EFFECT_SOLARIZE: {
            ov5640_i2c_write(ov5640_client->addr,0x4202,0x0f,10);
            msleep(50);
            OV5640Core_WritePREG(ov5640_effect_solarize_tbl); 
            
            ov5640_i2c_write(ov5640_client->addr,0x4202,0x00,10);
        }
        break;

    case CAMERA_EFFECT_SEPIA: {
            ov5640_i2c_write(ov5640_client->addr,0x4202,0x0f,10);
            msleep(50);
            OV5640Core_WritePREG(ov5640_effect_sepia_tbl);    
            ov5640_i2c_write(ov5640_client->addr,0x4202,0x00,10);
        }
        break;

    case CAMERA_EFFECT_POSTERIZE: {
            ov5640_i2c_write(ov5640_client->addr,0x4202,0x0f,10);
            msleep(50);
            OV5640Core_WritePREG(ov5640_effect_posterize_tbl);    
            ov5640_i2c_write(ov5640_client->addr,0x4202,0x00,10);
        }
        break;

    case CAMERA_EFFECT_AQUA: {
            ov5640_i2c_write(ov5640_client->addr,0x4202,0x0f,10);
            msleep(50);
            OV5640Core_WritePREG(ov5640_effect_aqua_tbl);    
            ov5640_i2c_write(ov5640_client->addr,0x4202,0x00,10);
        }
        break;

    default: {
            printk(KERN_ERR"--CAMERA-- %s ...(Not Support)\n",__func__);
        }
      break;
    }
    ov5640_effect = effect;
    /* Refresh Sequencer */
    return rc;
}

static int ov5640_set_brightness(int8_t brightness)
{
    long rc = 0;

    printk(KERN_ERR"--CAMERA-- %s ...brightness = %d\n",__func__ , brightness);

    switch (brightness)
    {
        case CAMERA_BRIGHTNESS_LV0:
            OV5640Core_WritePREG(ov5640_brightness_lv0_tbl);    
            
            break;
        case CAMERA_BRIGHTNESS_LV1:
            OV5640Core_WritePREG(ov5640_brightness_lv1_tbl);    

            break;
        case CAMERA_BRIGHTNESS_LV2:
            OV5640Core_WritePREG(ov5640_brightness_lv2_default_tbl);    

            break;
        case CAMERA_BRIGHTNESS_LV3:
            OV5640Core_WritePREG(ov5640_brightness_lv3_tbl);

            break;
        case CAMERA_BRIGHTNESS_LV4:
            OV5640Core_WritePREG(ov5640_brightness_lv4_tbl);

            break;
        case CAMERA_BRIGHTNESS_LV5:
            OV5640Core_WritePREG(ov5640_brightness_lv5_tbl);

            break;
        case CAMERA_BRIGHTNESS_LV6:
            OV5640Core_WritePREG(ov5640_brightness_lv6_tbl);

            break;
        default:
            break;
    }

    return rc;
}

static int ov5640_set_contrast(int contrast)
{
    long rc = 0;

    printk(KERN_ERR"--CAMERA-- %s ...contrast = %d\n",__func__ , contrast);

    switch (contrast) {
    case CAMERA_CONTRAST_LV0:
        OV5640Core_WritePREG(ov5640_contrast_lv0_tbl);

        break;
    case CAMERA_CONTRAST_LV1:
        OV5640Core_WritePREG(ov5640_contrast_lv1_tbl);

        break;
    case CAMERA_CONTRAST_LV2:
        OV5640Core_WritePREG(ov5640_contrast_lv2_tbl);

        break;
    case CAMERA_CONTRAST_LV3:
        OV5640Core_WritePREG(ov5640_contrast_default_lv3_tbl);

        break;
    case CAMERA_CONTRAST_LV4:
        OV5640Core_WritePREG(ov5640_contrast_lv4_tbl);

        break;
    case CAMERA_CONTRAST_LV5:
        OV5640Core_WritePREG(ov5640_contrast_lv5_tbl);

        break;
    case CAMERA_CONTRAST_LV6:
        OV5640Core_WritePREG(ov5640_contrast_lv6_tbl);

        break;
    case CAMERA_CONTRAST_LV7:
        OV5640Core_WritePREG(ov5640_contrast_lv7_tbl);

        break;
    case CAMERA_CONTRAST_LV8:
        OV5640Core_WritePREG(ov5640_contrast_lv8_tbl);

        break;
    case CAMERA_CONTRAST_LV9:
        OV5640Core_WritePREG(ov5640_contrast_lv9_tbl);

        break;
    case CAMERA_CONTRAST_LV10:
        OV5640Core_WritePREG(ov5640_contrast_lv10_tbl);

        break;
    default:
        printk(KERN_ERR"--CAMERA-- %s ...(Not Support)\n",__func__);
        break;
    }

    return rc;
}

static int ov5640_set_sharpness(int sharpness)
{
    long rc = 0;

    printk(KERN_ERR"--CAMERA-- %s ...sharpness = %d\n",__func__ , sharpness);

    switch (sharpness) {
    case CAMERA_SHARPNESS_LV0:
        OV5640Core_WritePREG(ov5640_sharpness_lv0_tbl);

        break;
    case CAMERA_SHARPNESS_LV1:
        OV5640Core_WritePREG(ov5640_sharpness_lv1_tbl);

        break;
    case CAMERA_SHARPNESS_LV2:
        OV5640Core_WritePREG(ov5640_sharpness_default_lv2_tbl);

        break;
    case CAMERA_SHARPNESS_LV3:
        OV5640Core_WritePREG(ov5640_sharpness_lv3_tbl);

        break;
    case CAMERA_SHARPNESS_LV4:
        OV5640Core_WritePREG(ov5640_sharpness_lv4_tbl);

        break;
    case CAMERA_SHARPNESS_LV5:
        OV5640Core_WritePREG(ov5640_sharpness_lv5_tbl);

        break;
    case CAMERA_SHARPNESS_LV6:
        OV5640Core_WritePREG(ov5640_sharpness_lv6_tbl);

        break;
    default:
        printk(KERN_ERR"--CAMERA-- %s ...(Not Support)\n",__func__);
        break;
    }

    return rc;
}

static int ov5640_set_saturation(int saturation)
{
    long rc = 0;

    printk(KERN_ERR"--CAMERA-- %s ...saturation = %d\n",__func__ , saturation);

    switch (saturation) {
    case CAMERA_SATURATION_LV0:
        OV5640Core_WritePREG(ov5640_saturation_lv0_tbl);                
        break;
    case CAMERA_SATURATION_LV1:
        OV5640Core_WritePREG(ov5640_saturation_lv1_tbl);               
        break;
    case CAMERA_SATURATION_LV2:
        OV5640Core_WritePREG(ov5640_saturation_lv2_tbl);                   
        break;
    case CAMERA_SATURATION_LV3:
        OV5640Core_WritePREG(ov5640_saturation_default_lv3_tbl);                    
        break;
    case CAMERA_SATURATION_LV4:
        OV5640Core_WritePREG(ov5640_saturation_lv4_tbl);                    
        break;
    case CAMERA_SATURATION_LV5:
        OV5640Core_WritePREG(ov5640_saturation_lv5_tbl);               
        break;   
    case CAMERA_SATURATION_LV6:
        OV5640Core_WritePREG(ov5640_saturation_lv6_tbl);
        break;
    case CAMERA_SATURATION_LV7:
        OV5640Core_WritePREG(ov5640_saturation_lv7_tbl);
        break;
    case CAMERA_SATURATION_LV8:
        OV5640Core_WritePREG(ov5640_saturation_lv8_tbl);
        break;
    case CAMERA_SATURATION_LV9:
        OV5640Core_WritePREG(ov5640_saturation_lv9_tbl);
        break;
    case CAMERA_SATURATION_LV10:
        OV5640Core_WritePREG(ov5640_saturation_lv10_tbl);
        break;

    default:
        printk(KERN_ERR"--CAMERA-- %s ...(Not Support)\n",__func__);
        break;
    }

    return rc;
}

static long ov5640_set_antibanding(int antibanding)
{
    long rc = 0;

    printk(KERN_ERR"--CAMERA-- %s ...antibanding = %d\n",__func__ , antibanding);

    switch (antibanding) {
    case CAMERA_ANTIBANDING_OFF:
        OV5640Core_WritePREG(ov5640_antibanding_off_tbl);
        ov5640_m_60Hz = 0;
        break;
    case CAMERA_ANTIBANDING_60HZ:
        OV5640Core_WritePREG(ov5640_antibanding_60z_tbl);
        ov5640_m_60Hz = 1;
        break;
    case CAMERA_ANTIBANDING_50HZ:
        OV5640Core_WritePREG(ov5640_antibanding_50z_tbl);
        ov5640_m_60Hz = 0;
        break;
    case CAMERA_ANTIBANDING_AUTO:
        OV5640Core_WritePREG(ov5640_antibanding_auto_tbl);
        ov5640_m_60Hz = 0;
        break;
    default:
        printk(KERN_ERR"--CAMERA-- %s ...(Not Support)\n",__func__);
        break;
    }

    return rc;
}

#if 0
static long ov5640_set_mirror_flip(int x_mirror, int y_flip)
{
    long rc = 0;

      //printk(KERN_ERR"--CAMERA-- %s ...(Start)\n",__func__);

      printk(KERN_ERR"--CAMERA-- %s ...x_mirror = %d\n, y_flip = %d",__func__ , x_mirror, y_flip);


      if(x_mirror==1)
      {
        if(y_flip==1)
        {
            //x_mirror=1, y_flip=1
            OV5640Core_WritePREG(ov5640_mirrorflip_mirror_flip);        
        }else
        {
            //x_mirror=1, y_flip=0
            OV5640Core_WritePREG(ov5640_mirrorflip_mirror);        
           
        }
      }
      else
      {
          if(y_flip==1)
          {
          //x_mirror=0, y_flip=1
          OV5640Core_WritePREG(ov5640_mirrorflip_flip);        

          
          }else
          {
          //x_mirror=0, y_flip=0
          OV5640Core_WritePREG(ov5640_mirrorflip_normal);        
          
          }      
      }
 
       printk(KERN_ERR"--CAMERA-- %s ...(End)\n",__func__);
    return rc;
}
#endif

static long ov5640_set_exposure_mode(int mode)
{
    long rc = 0;
    /*int i =0; */

      printk(KERN_ERR"--CAMERA-- %s ...mode = %d\n",__func__ , mode);

    switch (mode)
        {
            case CAMERA_SETAE_AVERAGE:
                OV5640Core_WritePREG(ov5640_ae_average_tbl);

                break;
            case CAMERA_SETAE_CENWEIGHT:
                OV5640Core_WritePREG(ov5640_ae_centerweight_tbl);

                break;
            case CAMERA_SETAE_CENTER_SPOT:
                OV5640Core_WritePREG(ov5640_ae_Center_Spot_tbl);

                break;
            default:
                printk(KERN_ERR"--CAMERA-- %s ...(Not Support)\n",__func__);
                break;
          }

    return rc;
}

static long ov5640_set_ledmod(int ledmod)
{
	long rc = 0;

	printk(KERN_ERR "--CAMERA-- %s ...ledmod = %d\n",__func__ , ledmod);

	switch (ledmod) {
	case LED_MODE_OFF: {
		ov5640_m_ledmod=0;
        ov5640_torch_en(0);
	}
			break;

	case LED_MODE_AUTO: {
		ov5640_m_ledmod=1;
	}
		break;

	case LED_MODE_ON: {
		ov5640_m_ledmod=2;
	}
		break;
    case LED_MODE_TORCH: {
		ov5640_m_ledmod=3;
        ov5640_torch_en(1);
	}
		break;
        
	default: {
        printk(KERN_ERR"--CAMERA-- %s ...(Not Support)\n",__func__);
		ov5640_m_ledmod=0;
		return -EFAULT;
	}
	}

	return rc;
}

static long ov5640_set_exposure_compentation(int exposure)
{
    long rc = 0;

    printk(KERN_ERR "--CAMERA-- %s ...exposure = %d\n",__func__ , exposure);

    switch (exposure) {
    case CAMERA_EVP_P2: {
            OV5640Core_WritePREG(OV5640SET_EV_P2);
        }
        break;

    case CAMERA_EVP_P1: {
            OV5640Core_WritePREG(OV5640SET_EV_P1);
        }
        break;

    case CAMERA_EVP_P0: {
            OV5640Core_WritePREG(OV5640SET_EV_P0);
        }
        break;

    case CAMERA_EVP_N1: {
            OV5640Core_WritePREG(OV5640SET_EV_N1);
        }
        break;

    case CAMERA_EVP_N2: {
            OV5640Core_WritePREG(OV5640SET_EV_N2);
        }
        break;
    default: {
        printk(KERN_ERR"--CAMERA-- %s ...(Not Support)\n",__func__);
        }
        break;
    }

	return rc;
}

static long ov5640_set_iso(int iso)
{
	long rc = 0;

	printk(KERN_ERR "--CAMERA-- %s ...iso = %d\n",__func__ , iso);

    switch (iso) {
    case CAMERA_ISO_AUTO: {
            OV5640Core_WritePREG(ov5640_iso_auto);
        }
        break;

    case CAMERA_ISO_100: {
            OV5640Core_WritePREG(ov5640_iso_100);
        }
        break;

    case CAMERA_ISO_200: {
            OV5640Core_WritePREG(ov5640_iso_200);
        }
        break;
    case CAMERA_ISO_400: {
            OV5640Core_WritePREG(ov5640_iso_400);
        }
        break;
    case CAMERA_ISO_800: {
            OV5640Core_WritePREG(ov5640_iso_800);
        }
        break;
    case CAMERA_ISO_1600: {
            OV5640Core_WritePREG(ov5640_iso_1600);
        }
        break;
    default: {
        printk(KERN_ERR"--CAMERA-- %s ...(Not Support)\n",__func__);
        }
        break;
    }

	return rc;
}

static enum hrtimer_restart ov5640_flashled_timer_func(struct hrtimer *timer)
{

    proc_comm_set_led(PMIC8028_ID_CAMERA_FLASH, 0);   //en_off
    gpio_set_value(TORCH_EN, 1);   //torch mode
    mdelay(1);
    proc_comm_set_led(PMIC8028_ID_CAMERA_FLASH, 1);   //en

	complete(&ov5640_flashled_comp);
	return HRTIMER_NORESTART;
}

static int ov5640_flashled_off_thread(void *arg)
{

    printk(KERN_ERR"---CAMERA-- %s\n",__func__);

	daemonize("ov5640_flashled_off_thread");

	while (1) {
		wait_for_completion(&ov5640_flashled_comp);
        printk(KERN_ERR"---CAMERA-get flash comp- %s \n",__func__);

		/* wait for flash on and turn off mpp */
        msleep(400);
        proc_comm_set_led(PMIC8028_ID_CAMERA_FLASH, 0);   //en_off
        gpio_set_value(TORCH_EN, 1);   //torch mode
        mdelay(1);
        proc_comm_set_led(PMIC8028_ID_CAMERA_FLASH, 0);   //off
	}
    return 0;
}


static int ov5640_torch_en(bool torch)
{
    if(torch) {
         proc_comm_set_led(PMIC8028_ID_CAMERA_FLASH, 0);   //en_off
         gpio_set_value(TORCH_EN, 0);   //torch mode en
         mdelay(1);
         proc_comm_set_led(PMIC8028_ID_CAMERA_FLASH, 1);   //en
    }
    else{
         proc_comm_set_led(PMIC8028_ID_CAMERA_FLASH, 0);   //en_off
         gpio_set_value(TORCH_EN, 0);   //torch mode en
         mdelay(1);
         proc_comm_set_led(PMIC8028_ID_CAMERA_FLASH, 0);   //off
    }
   
	return 0;
}

static int ov5640_snapshot_config(void)
{

	 int preview_shutter, preview_gain16;
	 int capture_shutter, capture_gain16;
	 int capture_sysclk, capture_HTS, capture_VTS;
	 int light_frequency, capture_bandingfilter, capture_max_band;
	 long capture_gain16_shutter;
	 int rc;
	 int preview_uv;

     int temp_awb_rgainH,temp_awb_rgainL,temp_awb_ggainH,temp_awb_ggainL,temp_awb_bgainH,temp_awb_bgainL;
     int temp_awb_rgain;
    unsigned int tmp;

     ov5640_i2c_read_byte(ov5640_client->addr, 0x3503, &tmp);
     CDBG("[kylin] 0x3503: %x\r\n", tmp);
    /*stop AEC/AGC here :write_i2c(0x3503 ,0x07) */
    /* Turn off 3A */
    rc = ov5640_i2c_write(ov5640_client->addr,0x3503, 0x07, 0xff);//yisong test 00
    /* MUST delay 30ms according to datasheet or sensor will crash */
    msleep(10);
  //  rc = ov5640_i2c_write(ov5640_client->addr,0x3406, 0x01, 0xff); //disable awb

  /*********************20120213 start*****************************/


        temp_awb_rgainH = 0;
        temp_awb_rgainL = 0;
        temp_awb_ggainH = 0;
        temp_awb_ggainL = 0;
        temp_awb_bgainH = 0;
        temp_awb_bgainL = 0;

        temp_awb_rgain = 0;

	rc = ov5640_i2c_write(ov5640_client->addr,0x3406, 0x01, 0xff); //disable awb

        ov5640_i2c_read_byte(ov5640_client->addr, 0x3400, &temp_awb_rgainH );
 	ov5640_i2c_read_byte(ov5640_client->addr, 0x3401, &temp_awb_rgainL );
 	ov5640_i2c_read_byte(ov5640_client->addr, 0x3402, &temp_awb_ggainH );
 	ov5640_i2c_read_byte(ov5640_client->addr, 0x3403, &temp_awb_ggainL );
 	ov5640_i2c_read_byte(ov5640_client->addr, 0x3404, &temp_awb_bgainH );
 	ov5640_i2c_read_byte(ov5640_client->addr, 0x3405, &temp_awb_bgainL );

        temp_awb_rgain = ((temp_awb_rgainH & 0x0f) << 8) + temp_awb_rgainL;

        temp_awb_rgain = (int)(temp_awb_rgain  * 95 / 100);

        temp_awb_rgainH = (temp_awb_rgain & 0x0f00) >> 8;

        temp_awb_rgainL = temp_awb_rgain  & 0x0ff;

 	rc = ov5640_i2c_write(ov5640_client->addr,0x3400,temp_awb_rgainH , 0xff);
     
	rc = ov5640_i2c_write(ov5640_client->addr,0x3401,temp_awb_rgainL , 0xff);



   /*********************20120213 start*****************************/
	//auto focus by yisong
	//OV5640_auto_focus();

// read preview shutter
	 preview_shutter = OV5640_get_shutter();
	 printk(KERN_ERR"--CAMERA CAPTURE-- preview_shutter  = %d\n",preview_shutter);
	 // read preview gain
	 preview_gain16 = OV5640_get_gain16();
	 printk(KERN_ERR"--CAMERA CAPTURE-- preview_gain16  = %d\n",preview_gain16);
	 ov5640_i2c_read_byte(ov5640_client->addr,0x558c, &preview_uv);
	 printk(KERN_ERR"--CAMERA-- 1 preview_uv  = %d\n",preview_uv);
	 

	 if (ov5640_effect == 0)
     {
          ov5640_i2c_read_byte(ov5640_client->addr, 0x5588, &tmp);
         tmp &= 0xbf;
         ov5640_i2c_write(ov5640_client->addr,0x5588, tmp,10);
    
         ov5640_i2c_read_byte(ov5640_client->addr, 0x558c, &tmp);
    
                 tmp = tmp*8/10;
         ov5640_i2c_write(ov5640_client->addr,0x5584, tmp,10);
         printk(KERN_ERR"--CAMERA-- 2 preview_uv  = %d\n",preview_uv);
     }

	 // turn off night mode for capture
	 OV5640_set_night_mode(0);

	 // turn off overlay
	  //OV5640_write_i2c(0x3022, 0x06);
	 ov5640_i2c_write(ov5640_client->addr,0x3022, 0x06, 0xff);

	 // Write capture setting
	 OV5640Core_WritePREG(ov5640_capture_tbl);//Step2: change to full size

	 // read capture VTS
	 capture_VTS = OV5640_get_VTS();
	 capture_HTS = OV5640_get_HTS();
	 capture_sysclk = OV5640_get_sysclk();

//neil add for capture outdoor
/*
         if(preview_shutter < 300)
         {
          	 ov5640_i2c_write(ov5640_client->addr,0x5193, 0x50, 0xff);
	         ov5640_i2c_write(ov5640_client->addr,0x5194, 0x50, 0xff);
	         ov5640_i2c_write(ov5640_client->addr,0x5195, 0x40, 0xff);
         }
*/
	 // calculate capture banding filter
	 light_frequency = OV5640_get_light_frequency();
	 if (light_frequency == 60) {
		 // 60Hz
		 capture_bandingfilter = capture_sysclk * 100 / capture_HTS * 100 / 120;
	 }
	 else {
		 // 50Hz
		 capture_bandingfilter = capture_sysclk * 100 / capture_HTS;
	 }
	 //capture_max_band = int((capture_VTS - 4)/capture_bandingfilter);
	 capture_max_band = (capture_VTS - 4)/capture_bandingfilter;
#if 1
	 // calculate capture shutter/gain16
	 if (average > AE_low && average < AE_high) {
		 // in stable range
		 capture_gain16_shutter = preview_gain16 * preview_shutter * capture_sysclk/preview_sysclk * preview_HTS/capture_HTS * AE_Target / average;
	 }
	 else {
		 capture_gain16_shutter = preview_gain16 * preview_shutter * capture_sysclk/preview_sysclk * preview_HTS/capture_HTS;
	 }

	 // gain to shutter
	 if(capture_gain16_shutter < (capture_bandingfilter * 16)) {
		 // shutter < 1/100
		 capture_shutter = capture_gain16_shutter/16;
		 if(capture_shutter <1)
			 capture_shutter = 1;

		 capture_gain16 = capture_gain16_shutter/capture_shutter;
		 if(capture_gain16 < 16)
			 capture_gain16 = 16;
	 }
	 else {
		 if(capture_gain16_shutter > (capture_bandingfilter*capture_max_band*16)) {
			 // exposure reach max
			 capture_shutter = capture_bandingfilter*capture_max_band;
			 capture_gain16 = capture_gain16_shutter / capture_shutter;
		 }
		 else {
			 // 1/100 < capture_shutter =< max, capture_shutter = n/100
			 //capture_shutter = (int (capture_gain16_shutter/16/capture_bandingfilter)) * capture_bandingfilter;
			 capture_shutter = (capture_gain16_shutter/16/capture_bandingfilter) * capture_bandingfilter;
			 capture_gain16 = capture_gain16_shutter / capture_shutter;
		 }
	 }
#else
capture_shutter =preview_shutter * capture_sysclk/preview_sysclk * preview_HTS/capture_HTS;
capture_gain16 = preview_gain16;
#endif

	 // write capture gain
	 OV5640_set_gain16(capture_gain16);

	 // write capture shutter
	 if (capture_shutter > (capture_VTS - 4)) {
		 capture_VTS = capture_shutter + 4;
		 OV5640_set_VTS(capture_VTS);
	 }
	 OV5640_set_shutter(capture_shutter);
	 printk(KERN_ERR"--CAMERA CAPTURE-- capture_shutter  = %d\n",capture_shutter);
	 printk(KERN_ERR"--CAMERA CAPTURE-- capture_gain16  = %d\n",capture_gain16);
	 #if 1
	 /*if (ov5640_effect == 0)
	 {
         
	 	ov5640_i2c_read_byte(ov5640_client->addr,0x5588,&rc);
	 	ov5640_i2c_write(ov5640_client->addr,0x5588, rc|0x40 , 0xff);	 

		ov5640_i2c_write(ov5640_client->addr,0x5583, preview_uv, 0xff);
		ov5640_i2c_write(ov5640_client->addr,0x5584, preview_uv, 0xff);
        
	 }*/

         #endif
         
	 // skip 2 vysnc
	 msleep(135);

	 // start capture at 3rd vsync
	 if ((ov5640_m_ledmod==2) ||(autoflash_en == 1)) {


        printk(KERN_ERR"--CAMERA-- flashlight -!!!!-- average = %d\n",average);
        hrtimer_cancel(&ov5640_flashled_timer);
        hrtimer_start(&ov5640_flashled_timer,
                      ktime_set(OV5640_FLASHLED_DELAY / 1000, (OV5640_FLASHLED_DELAY % 1000) * 1000000),
                      HRTIMER_MODE_REL);

        if (hrtimer_active(&ov5640_flashled_timer))
            printk(KERN_ERR"--CAMERA--hrtimer_active-111-- \n");

    }

	 return 0;



}


static int ov5640_set_sensor_mode(int mode, int res)
{
    int rc = 0;

    //printk(KERN_ERR"--CAMERA-- ov5640_set_sensor_mode mode = %d, res = %d\n", mode, res);

    switch (mode)
    {
        case SENSOR_PREVIEW_MODE:
			rc =ov5640_setting(S_UPDATE_PERIODIC, S_RES_PREVIEW);;
            break;
		
        case SENSOR_SNAPSHOT_MODE:
			rc =ov5640_setting(S_UPDATE_PERIODIC, S_RES_CAPTURE);;
            break;
			
        case SENSOR_RAW_SNAPSHOT_MODE:
        default:
            printk(KERN_ERR"--CAMERA--ov5640_set_sensor_mode  no support\n");
			rc = -EINVAL;
            break;
    }
    
    return rc;
}


#if 0
static int ov5640_set_touchaec(uint32_t x,uint32_t y)
{
    uint8_t aec_arr[8]={0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11};
    int idx = 0;
    int i;
    printk(KERN_ERR"--CAMERA-- %s x: %d ,y: %d\r\n",__func__ ,x,y);
    idx = x /2 + y *2;
    printk(KERN_ERR"--CAMERA-- idx: %d\r\n",idx);
    
    if(x %2 == 0)
    {
        aec_arr[idx] = 0x10 | 0x0a;
    }
    else
    {
        aec_arr[idx] = 0x01 | 0xa0;
    }
    
    /*for(i==0;i<8;i++) */
    for(i=0;i<8;i++) 
    {
        printk(KERN_ERR"write : %x val : %x ",0x5688+i,aec_arr[i]); 
        ov5640_i2c_write(ov5640_client->addr,0x5688+i,aec_arr[i],10);
    }

    return 1;
}
#endif

static long ov5640_set_scenemod(int scenemod)
{
    long rc = 0;

    printk(KERN_ERR "ov5640_set_scenemod,scenemod = %d\n",scenemod);

    switch (scenemod) {
    case CAMERA_BESTSHOT_OFF: {
            OV5640Core_WritePREG(ov5640_scene_auto_tbl);
            break;
        }

    case CAMERA_BESTSHOT_ACTION: {
            OV5640Core_WritePREG(ov5640_scene_action_tbl);
        }
        break;

    case CAMERA_BESTSHOT_PORTRAIT: {
            OV5640Core_WritePREG(ov5640_scene_portrait_tbl);
        }
        break;

    case CAMERA_BESTSHOT_LANDSCAPE: {
            OV5640Core_WritePREG(ov5640_scene_landscape_tbl);

            printk(KERN_ERR "LANDSCAPE Not implimented \n");

        }
        break;

    case CAMERA_BESTSHOT_NIGHT: {
            OV5640Core_WritePREG(ov5640_scene_night_tbl);
        }
        break;

    case CAMERA_BESTSHOT_NIGHT_PORTRAIT: {
            OV5640Core_WritePREG(ov5640_scene_nightportrait_tbl);
        }
        break;

    case CAMERA_BESTSHOT_THEATRE: {
            //OV5640Core_WritePREG(ov5640_scene_theatre_tbl);
        }
        break;

    case CAMERA_BESTSHOT_BEACH: {
            OV5640Core_WritePREG(ov5640_scene_beach_tbl);
        }
        break;

    case CAMERA_BESTSHOT_SNOW: {
            OV5640Core_WritePREG(ov5640_scene_snow_tbl);
        }
        break;

    case CAMERA_BESTSHOT_SUNSET: {
            OV5640Core_WritePREG(ov5640_scene_sunset_tbl);
        }
        break;
    case CAMERA_BESTSHOT_ANTISHAKE: {
            OV5640Core_WritePREG(ov5640_scene_antishake_tbl);
            printk(KERN_ERR "ANTISHAKE Not implimented \n");
        }
        break;
    case CAMERA_BESTSHOT_FIREWORKS: {
            OV5640Core_WritePREG(ov5640_scene_fireworks_tbl);
        }
        break;
    case CAMERA_BESTSHOT_SPORTS: {
            OV5640Core_WritePREG(ov5640_scene_sports_tbl);
        }
        break;

    case CAMERA_BESTSHOT_PARTY: {
            printk(KERN_ERR "PARTY Not implimented \n");
        }
        break;

    case CAMERA_BESTSHOT_CANDLELIGHT: {
            printk(KERN_ERR "CANDLELIGHT Not implimented \n");
        }
        break;

    case CAMERA_BESTSHOT_BACKLIGHT: {
            printk(KERN_ERR "BACKLIGHT Not implimented \n");
        }
        break;

    case CAMERA_BESTSHOT_FLOWERS: {
            printk(KERN_ERR "FLOWERS Not implimented \n");
        }
        break;

    case CAMERA_BESTSHOT_AR: {
            printk(KERN_ERR "AR Not implimented \n");
        }
        break;
    default: {
        printk(KERN_ERR"--CAMERA-- %s ...(Not Support)\n",__func__);
        }
       break;
    }

	return rc;
}

static int ov5640_sensor_config(void __user *argp)
{
    struct sensor_cfg_data cdata;
    long  rc = 0;

    if (copy_from_user(&cdata,(void *)argp,sizeof(struct sensor_cfg_data))) 
        return -EFAULT;

    printk(KERN_ERR"--CAMERA-- %s cfgtype=%d\n",__func__,cdata.cfgtype);

    //mdelay(200);
    mutex_lock(&ov5640_mutex);
    switch (cdata.cfgtype)
    {
        case CFG_SET_MODE:   // 0
            rc =ov5640_set_sensor_mode(cdata.mode, cdata.rs);
            break;
    case CFG_SET_EFFECT: // 1
            if(effect_present != cdata.cfg.effect)
            {
                effect_present = cdata.cfg.effect;

                rc = ov5640_set_effect(cdata.mode, cdata.cfg.effect);
            }
            break;
        case CFG_START:      // 2
            printk(KERN_ERR"--CAMERA-- CFG_START (Not Support) !!\n");
            // Not Support
            break;
        case CFG_PWR_UP:     // 3
            printk(KERN_ERR"--CAMERA-- CFG_PWR_UP (Not Support) !!\n");
            // Not Support
            break;
        case CFG_PWR_DOWN:   // 4
            ov5640_power_off();
            break;
        case CFG_SET_DEFAULT_FOCUS:  // 06
            printk(KERN_ERR"--CAMERA-- CFG_SET_DEFAULT_FOCUS (Not Implement) !!\n");
            break;        
        case CFG_MOVE_FOCUS:     //  07
            printk(KERN_ERR"--CAMERA-- CFG_MOVE_FOCUS (Not Implement) !!\n");
            break;
    case CFG_SET_BRIGHTNESS:     //  12
            if(brightness_present != cdata.cfg.brightness)
            {
                brightness_present = cdata.cfg.brightness;
            
                rc = ov5640_set_brightness(cdata.cfg.brightness);
            }
            break;
    case CFG_SET_CONTRAST:     //  13
            if(contrast_present != cdata.cfg.contrast)
            {
                contrast_present = cdata.cfg.contrast;
                rc = ov5640_set_contrast(cdata.cfg.contrast);
            }
            break;            
    case CFG_SET_EXPOSURE_MODE:     //  15 //it's be as metering //´ú¥ú
            if(ae_mode_present != cdata.cfg.ae_mode)
            {
                ae_mode_present = cdata.cfg.ae_mode;

                rc = ov5640_set_exposure_mode(cdata.cfg.ae_mode);
            }
            break;
    case CFG_SET_ANTIBANDING:     //  17
            if(antibanding_present != cdata.cfg.antibanding)
            {
                antibanding_present = cdata.cfg.antibanding;

                rc = ov5640_set_antibanding(cdata.cfg.antibanding);//mask
            }
            break;

    case CFG_SET_SATURATION:     //  30
            if(saturation_present != cdata.cfg.saturation)
            {
                saturation_present = cdata.cfg.saturation;

                rc = ov5640_set_saturation(cdata.cfg.saturation);//mask
            }
            break;
    case CFG_SET_SHARPNESS:     //  31
            if(sharpness_present != cdata.cfg.sharpness)
            {
                sharpness_present = cdata.cfg.sharpness;

                rc = ov5640_set_sharpness(cdata.cfg.sharpness);//mask
            }
            break;
    case CFG_SET_WB:   //  16
            if(wb_present != cdata.cfg.wb)
            {
                wb_present = cdata.cfg.wb;

                ov5640_set_wb_oem(cdata.cfg.wb);
            }
            rc = 0 ;
            break;

        case CFG_SET_MIRROR_FLIP:
            printk(KERN_ERR"--CAMERA-- CFG_SET_MIRROR_FLIP!!\n");
            //ov5640_set_mirror_flip(cdata.cfg.mirror_flip.x_mirror, cdata.cfg.mirror_flip.y_flip);
            rc = 0 ;
            break;
           
        case CFG_SET_TOUCHAEC:
            printk(KERN_ERR"--CAMERA-- CFG_SET_TOUCHAEC!!\n");
            rc = ov5640_set_touchaec(cdata.cfg.aec_cord.x,cdata.cfg.aec_cord.y);
            break;
        case CFG_SET_AUTO_FOCUS:
            printk(KERN_ERR"--CAMERA-- CFG_SET_AUTO_FOCUS !\n");
            rc = ov5640_sensor_start_af();
            break;        
        case CFG_SET_LEDMOD:  //  29		
    		rc = ov5640_set_ledmod(cdata.cfg.ledmod);
    		break;
    case CFG_SET_SCENEMOD:
            if(scenemod_present != cdata.cfg.scenemod)
            {
                scenemod_present = cdata.cfg.scenemod;

                rc = ov5640_set_scenemod(cdata.cfg.scenemod);//mask
            }
           break;
    case CFG_SET_EXPOSURE_COMPENSATION:
            if(exposure_compensat_present != cdata.cfg.exposure_compensat)
            {
                exposure_compensat_present = cdata.cfg.exposure_compensat;

                rc = ov5640_set_exposure_compentation(cdata.cfg.exposure_compensat);
            }
           break;
    case CFG_SET_ISO:
            if(iso_present != cdata.cfg.iso)
            {
                iso_present = cdata.cfg.iso;

                rc = ov5640_set_iso(cdata.cfg.iso);//mask
            }
           break;
		break;
        default:
            printk(KERN_ERR"--CAMERA-- %s: Command=%d (Not Implement)!!\n",__func__,cdata.cfgtype);
            rc = -EINVAL;
        break;    
    }
    mutex_unlock(&ov5640_mutex);
    return rc;    
}

static struct i2c_driver ov5640_i2c_driver = {
    .id_table = ov5640_i2c_id,
    .probe  = ov5640_i2c_probe,
    .remove = ov5640_i2c_remove,
    .driver = {
        .name = "ov5640af",
    },
};
static int ov5640_set_touchaec(uint32_t x,uint32_t y)
{
    u8 i;
    unsigned int af_ack;
    int rc = 1;

    printk(KERN_ERR"--CAMERA-- %s x=%d,y= %d\n", __func__, x,y);

    ov5640_i2c_write(ov5640_client->addr,0x3024,x,10);
    ov5640_i2c_write(ov5640_client->addr,0x3025,y,10);
    ov5640_i2c_write(ov5640_client->addr,OV5640_CMD_ACK,0x01,10);
    ov5640_i2c_write(ov5640_client->addr,OV5640_CMD_MAIN,0x81,10);

    for(i=0;i<50;i++)
    {
        ov5640_i2c_read_byte(ov5640_client->addr,OV5640_CMD_ACK,&af_ack);
        if(af_ack == 0)
        {
            printk(KERN_ERR"--CAMERA-- %s (REG3020 == 0)\n",__func__);
            rc = 0;
            break;
        }
        msleep(50);
    }

    printk(KERN_ERR"--CAMERA-- %s i=%d,af_ack = 0x%x\n", __func__, i,af_ack);
    return rc;

}
static int ov5640_sensor_start_af(void)
{
    u8 i;
    unsigned int af_st;
    unsigned int af_ack;
    int rc = 1;
//    int temp;
    printk(KERN_ERR"--CAMERA-- %s (Start...)\n",__func__);
           

		/**************add for test awb issue+++***************/

			rc = ov5640_i2c_write(ov5640_client->addr,0x3406, 0x01, 0xff); //disable awb

		/**************add for test awb issue---***************/
  

		    // get average
	 //average = OV5640_read_i2c(0x56a1);
//	 ov5640_i2c_read_byte(ov5640_client->addr,0x558c, &temp);
//	 printk(KERN_ERR"--CAMERA-- af preview_uv  = %d\n",temp);
	ov5640_i2c_read_byte(ov5640_client->addr,0x56a1,&average);
	printk(KERN_ERR"-- average = %d\n",average);

    if(((average <= 0x0a) && (ov5640_m_ledmod==1)) /*|| ov5640_m_ledmod == 2*/)
        autoflash_en = 1;
    else 
        autoflash_en = 0;

    if(autoflash_en ==1 )
    {
           //ov5640_torch_en(1);
        proc_comm_set_led(PMIC8028_ID_CAMERA_FLASH, 0);   //en_off
        gpio_set_value(TORCH_EN, 0);   //torch mode en
        mdelay(1);
        proc_comm_set_led(PMIC8028_ID_CAMERA_FLASH, 1);   //en
    }

    ov5640_i2c_write(ov5640_client->addr,OV5640_CMD_ACK,0x01,10);
    ov5640_i2c_write(ov5640_client->addr,OV5640_CMD_MAIN,0x03,10);

    for(i=0;i<50;i++)
    {
        ov5640_i2c_read_byte(ov5640_client->addr,OV5640_CMD_ACK,&af_ack);
        if(af_ack == 0)
        {
            printk(KERN_ERR"--CAMERA-- %s (REG3020 == 0)\n",__func__);
            rc = 0;
            break;
        }
        msleep(50);
	//msleep(10);
    }

    printk(KERN_ERR"--CAMERA-- %s i=%d,af_ack = 0x%x\n", __func__, i,af_ack);

    ov5640_i2c_read_byte(ov5640_client->addr,OV5640_CMD_FW_STATUS,&af_st);
    printk(KERN_ERR"--CAMERA-- %s af_st = 0x%x\n", __func__, af_st);

    if (af_st == 0x10) {
        printk(KERN_ERR"--CAMERA-- %s AF ok and release AF setting~!!\n", __func__);
        af_finished = 1;
    }
    else {
        printk(KERN_ERR"--CAMERA-- %s AF not ready!!\n", __func__);
    }
    ov5640_i2c_read_byte(ov5640_client->addr,0x56a1,&average);
	printk(KERN_ERR"-- average = %d\n",average);
/*
	ov5640_i2c_read_byte(ov5640_client->addr,0x3400,&temp);
	printk(KERN_ERR"--reg 0x3400 = 0x%x\n",temp);
	ov5640_i2c_read_byte(ov5640_client->addr,0x3401,&temp);
	printk(KERN_ERR"--reg 0x3401 = 0x%x\n",temp);
	ov5640_i2c_read_byte(ov5640_client->addr,0x3402,&temp);
	printk(KERN_ERR"--reg 0x3402 = 0x%x\n",temp);
	ov5640_i2c_read_byte(ov5640_client->addr,0x3403,&temp);
	printk(KERN_ERR"--reg 0x3403 = 0x%x\n",temp);
	ov5640_i2c_read_byte(ov5640_client->addr,0x3404,&temp);
	printk(KERN_ERR"--reg 0x3404 = 0x%x\n",temp);
	ov5640_i2c_read_byte(ov5640_client->addr,0x3405,&temp);
	printk(KERN_ERR"--reg 0x3405 = 0x%x\n",temp);
*/	


    if(autoflash_en ==1 )
    {
           //ov5640_torch_en(0);
        proc_comm_set_led(PMIC8028_ID_CAMERA_FLASH, 0);   //en_off
        gpio_set_value(TORCH_EN, 1);   //torch mode en
        mdelay(1);
        proc_comm_set_led(PMIC8028_ID_CAMERA_FLASH, 0);   //off
	
	ov5640_i2c_write(ov5640_client->addr,0x3400,0x06,10);
	ov5640_i2c_write(ov5640_client->addr,0x3401,0xE0,10);
	ov5640_i2c_write(ov5640_client->addr,0x3402,0x04,10);
	ov5640_i2c_write(ov5640_client->addr,0x3403,0x00,10);
	ov5640_i2c_write(ov5640_client->addr,0x3404,0x05,10);
	ov5640_i2c_write(ov5640_client->addr,0x3405,0x90,10);

    }

    return rc;
}


static int ov5640_probe_init_gpio(const struct msm_camera_sensor_info *data)
{
    int rc = 0;

    printk(KERN_ERR"--CAMERA-- %s Open CAMIO CLK,set to default clk rate!\n",__func__);

    ov5640_pwdn_gpio = data->sensor_pwd;
    ov5640_reset_gpio = data->sensor_reset;

    //printk(KERN_ERR"--CAMERA-- %s : sensor_pwd_pin=%d, sensor_reset_pin=%d\n",__func__,data->sensor_pwd,data->sensor_reset);

    rc = gpio_request(data->sensor_pwd, "ov5640");
    //printk(KERN_ERR"--CAMERA-- %s : gpio_request=%d, result is %d",__func__,data->sensor_pwd, rc);

    rc |= gpio_request(data->sensor_reset, "ov5640");
    //printk(KERN_ERR"--CAMERA-- %s : gpio_request=%d, result is %d",__func__,data->sensor_reset, rc);

    if(rc<0)
    {
        gpio_free(data->sensor_pwd);
        gpio_free(data->sensor_reset);
        rc = gpio_request(data->sensor_pwd, "ov5640");
        rc |= gpio_request(data->sensor_reset, "ov5640");
    }

    gpio_direction_output(data->sensor_reset, 1);
    gpio_direction_output(data->sensor_pwd, 1);

    return rc;
}



static int ov5640_sensor_probe(const struct msm_camera_sensor_info *info,struct msm_sensor_ctrl *s)
{
	int rc = -ENOTSUPP;
    int len = sizeof(ov5640_afinit_tbl_2)/sizeof(ov5640_afinit_tbl_2[0]);
	printk(KERN_ERR"--CAMERA-- %s (Start...)\n",__func__);
	rc = i2c_add_driver(&ov5640_i2c_driver);
	//printk(KERN_ERR"--CAMERA-- i2c_add_driver ret:0x%x,ov5640_client=0x%x\n",
	     //rc,(unsigned int)ov5640_client);
	if ((rc < 0 ) || (ov5640_client == NULL))
	{
	  printk(KERN_ERR"--CAMERA-- i2c_add_driver FAILS!!\n");
	   return rc;
	}


	rc = ov5640_probe_init_gpio(info);

	//Power On
	ov5640_power_on();

	/* SENSOR NEED MCLK TO DO I2C COMMUNICTION, OPEN CLK FIRST*/
	msm_camio_clk_rate_set(24000000);

	mdelay(5);

	gpio_set_value(info->sensor_pwd, 0);   //enable power down pin 
    mdelay(2);
	gpio_set_value(info->sensor_reset, 1);   //reset camera reset pin
	mdelay(5);
    /*
	gpio_set_value(info->sensor_reset, 0);
	mdelay(5);
	gpio_set_value(info->sensor_reset, 1);
	mdelay(1);
*/
    printk(KERN_ERR"--CAMERA-- %s, hrtimer_init ov5640_flashled_timer\n",__func__);
    ov5640_af_power_onoff(1);

	hrtimer_init(&ov5640_flashled_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ov5640_flashled_timer.function = ov5640_flashled_timer_func;
	ov5640_thread_id = kernel_thread(ov5640_flashled_off_thread, NULL, CLONE_FS | CLONE_FILES);

	rc |= ov5640_probe_readID(info);
	if (rc < 0)
	   goto probe_fail;
    else
    {
       rc = OV5640Core_WritePREG(ov5640_init_tbl);
       if (rc < 0)
       {
           printk("--CAMERA--probe failed %s  write 5640_init_table(End...)\n",__func__);
           goto probe_fail;
       }
    }


    rc = OV5640Core_WritePREG(ov5640_afinit_tbl_1);
    printk(KERN_ERR"--CAMERA-- AF INIT 1.rc = %d!!\n",rc);
    mdelay(5);
    rc = ov5640_i2c_txdata(ov5640_client->addr,(u8 *)ov5640_afinit_tbl_2,len);
    printk(KERN_ERR"--CAMERA-- AF INIT 2.rc = %d!!\n",rc);
    mdelay(5);
    rc=OV5640Core_WritePREG(ov5640_afinit_tbl_3);
    printk(KERN_ERR"--CAMERA-- AF INIT 3.rc = %d!!\n",rc);
     //ov5640_i2c_read_byte(ov5640_client->addr,OV5640_CMD_FW_STATUS,&af_st);
     //printk(KERN_ERR"--jenny CAMERA-- %s af_st = 0x%x\n", __func__, af_st);
	if (rc < 0)
	   goto probe_fail;

	s->s_init = ov5640_sensor_open_init;
	s->s_release = ov5640_sensor_release;
	s->s_config  = ov5640_sensor_config;
	s->s_camera_type = BACK_CAMERA_2D;
	s->s_mount_angle = info->sensor_platform_info->mount_angle;

    ov5640_power_off();
    return rc;
	printk(KERN_ERR"--CAMERA--probe ok %s (End...)\n",__func__);
probe_fail:
     ov5640_power_off();
	 i2c_del_driver(&ov5640_i2c_driver);
     printk(KERN_ERR"--CAMERA--probe failed %s (End...)\n",__func__);
	return rc;
}


static int ov5640_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	printk(KERN_ERR"--CAMERA-- %s ... (Start...)\n",__func__);


	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
	  printk(KERN_ERR"--CAMERA--i2c_check_functionality failed\n");
	  return -ENOMEM;
	}

	ov5640_sensorw = kzalloc(sizeof(struct ov5640_work), GFP_KERNEL);
	if (!ov5640_sensorw)
	{
	  printk(KERN_ERR"--CAMERA--kzalloc failed\n");
	  return -ENOMEM;
	}
	i2c_set_clientdata(client, ov5640_sensorw);
	ov5640_init_client(client);
	ov5640_client = client;

	printk(KERN_ERR"--CAMERA-- %s ... (End...)\n",__func__);
	return 0;
}

static int __ov5640_probe(struct platform_device *pdev)
{
    return msm_camera_drv_start(pdev, ov5640_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
    .probe = __ov5640_probe,
    .driver = {
        .name = "msm_camera_ov5640af",
        .owner = THIS_MODULE,
    },
};

static int __init ov5640_init_v1(void* hdl)
{
    ov5640_i2c_buf[0]=0x5A;
    printk(KERN_ERR"--CAMERA-- ov5640_init...\n");
    return platform_driver_register(&msm_camera_driver);
}

//module_init(ov5640_init);
DECLARE_DYN_LOADER(DRIVER_TYPE_MAIN_CAM, 
			MAIN_CAM_OV5640_V1, 
			ov5640_init_v1, LEVEL6);

static int __init ov5640_init_v2(void* hdl)
{
    ov5640_i2c_buf[0]=0x5A;
    printk(KERN_ERR"--CAMERA-- ov5640_init...\n");
    return platform_driver_register(&msm_camera_driver);
}
DECLARE_DYN_LOADER(DRIVER_TYPE_MAIN_CAM, 
			MAIN_CAM_OV5640_V2, 
			ov5640_init_v2, LEVEL6);
MODULE_DESCRIPTION("OV5640 YUV MIPI sensor driver");
MODULE_LICENSE("GPL v2");
