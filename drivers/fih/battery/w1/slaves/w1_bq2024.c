/*
 * 1-Wire implementation for the bq2024 EPROM chip
 *
 * Copyright © 2011-2012, Audi PC Huang <AudiPCHuang@fihtdc.com>
 *
 * Use consistent with the GNU GPL is permitted,
 * provided that this copyright notice is
 * preserved in its entirety in all copies and derived works.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/gfp.h>
#include <linux/delay.h>
#include <linux/w1-gpio.h>
#include <linux/mutex.h>
#include <asm/gpio.h>


#include "../../../../w1/w1.h"   //Debbie, 2011/09/05
#include "../../../../w1/w1_int.h"   //Debbie, 2011/09/05
#include "../../../../w1/w1_family.h"   //Debbie, 2011/09/05
#include "../../../../../include/fih/dynloader.h"  //FIH-NJ Jerry.
#include "../../../../../arch/arm/mach-msm/proc_comm.h"
#include "../../../../../arch/arm/mach-msm/fih/resources/battery/fih_battery.h" //Jerry, 2011/12/23.

#define W1_READ_MEMORY_PAGE_CRC		0xC3
/*FIH-NJ BSP Jerry add for HV battery. ++{*/
#define W1_READ_ROM		0x33   
static int battery_type_version = 1;
static struct w1_family *fent;  //use to module exit
/*FIH-NJ BSP Jerry add for HV battery. }++*/

enum {
	BATTERY_ID_UNKNOWN,
	BATTERY_ID_NOEPROM,
	BATTERY_ID_INVALID,
	BATTERY_ID_VALID,
};

//FIH-NJ-BSP Jerry add, 2011/12/22. +++

static int battery_type = BATTERY_TYPE_4V2;
//FIH-NJ-BSP Jerry add, 2011/12/22. ---

DEFINE_MUTEX(check_battery_id_lock);
static int is_battery_valid = BATTERY_ID_UNKNOWN;
static struct w1_slave *g_sl = NULL;

static const u8 battery_id_table[4][128]=
{
	//BP6X_Maxell
	{
		0xF8, 0x01, 0x98, 0x00, 0x00, 0x19, 0x14, 0x03, 0xE8, 0x23, 0x5A, 0x0E, 0x47, 0x95, 0xFF, 0x01, 
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0x7C, 0x67, 0x8F, 0x56, 0xD8, 0x00, 0x31, 0xFF, 0x5A, 0x40, 0xFF, 0x05, 0x01, 0x06, 0x0F, 0x00, 
		0x68, 0x01, 0xB0, 0x48, 0x9C, 0xA3, 0x11, 0x3C, 0x5F, 0x63, 0x67, 0x44, 0x52, 0x58, 0x5C, 0x1C,
		0xD7, 0xB0, 0x4A, 0x9C, 0xA4, 0x11, 0x3F, 0x6B, 0x6F, 0x72, 0x3F, 0x5F, 0x64, 0x68, 0x1B, 0x38, 
		0xC6, 0x00, 0xA4, 0xB3, 0x3A, 0xB6, 0x86, 0xD3, 0x97, 0x65, 0x6F, 0x7E, 0x90, 0x1C, 0x37, 0x64,
		0x43, 0x4F, 0x50, 0x52, 0x32, 0x30, 0x31, 0x30, 0x4D, 0x4F, 0x54, 0x4F, 0x52, 0x4F, 0x4C, 0x41, 
		0x20, 0x45, 0x2E, 0x50, 0x20, 0x43, 0x48, 0x41, 0x52, 0x47, 0x45, 0x20, 0x4F, 0x4E, 0x4C, 0x59
	},

	//BP6X_LGC
	{
		0xFA, 0x01, 0x98, 0x00, 0x00, 0x19, 0x14, 0x03, 0xE8, 0x23, 0x5A, 0x0E, 0x45, 0x95, 0xFF, 0x01, 
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0x81, 0x76, 0x8B, 0x56, 0xD8, 0x00, 0x31, 0xFF, 0x5A, 0x40, 0xFF, 0x05, 0x01, 0x06, 0x0F, 0x00, 
		0x68, 0x01, 0xB0, 0x48, 0x9C, 0xA3, 0x12, 0x3C, 0x5F, 0x63, 0x67, 0x44, 0x4B, 0x51, 0x59, 0x1C,
		0x0D, 0xB0, 0x4A, 0x9D, 0xA4, 0x12, 0x3F, 0x6A, 0x6F, 0x72, 0x3F, 0x5B, 0x64, 0x69, 0x1B, 0x33, 
		0xC6, 0x1C, 0xA9, 0xB6, 0x3A, 0x95, 0xA3, 0xB8, 0x97, 0x54, 0x5F, 0x76, 0x8B, 0x1C, 0x35, 0x60,
		0x43, 0x4F, 0x50, 0x52, 0x32, 0x30, 0x30, 0x39, 0x4D, 0x4F, 0x54, 0x4F, 0x52, 0x4F, 0x4C, 0x41, 
		0x20, 0x45, 0x2E, 0x50, 0x20, 0x43, 0x48, 0x41, 0x52, 0x47, 0x45, 0x20, 0x4F, 0x4E, 0x4C, 0x59
	},

	//BP5X_Maxell
	{
		0xF4, 0x01, 0x98, 0x00, 0x00, 0x19, 0x14, 0x03, 0xE8, 0x23, 0x5A, 0x0E, 0x4B, 0x95, 0xFF, 0x01, 
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0x66, 0x81, 0x96, 0x56, 0xD8, 0x00, 0x31, 0xFF, 0x5A, 0x40, 0xFF, 0x05, 0x01, 0x06, 0x0F, 0x00, 
		0x68, 0x01, 0xB0, 0x48, 0x9D, 0xA5, 0x0C, 0x3C, 0x5F, 0x63, 0x67, 0x44, 0x4E, 0x55, 0x5A, 0x1C,
		0xB0, 0xB0, 0x4A, 0x9E, 0xA6, 0x0C, 0x3F, 0x6B, 0x6F, 0x72, 0x3F, 0x5E, 0x62, 0x66, 0x1B, 0x46, 
		0xC6, 0x00, 0xA8, 0xB6, 0x35, 0xD6, 0x86, 0xD2, 0x97, 0x67, 0x70, 0x7E, 0x8E, 0x1C, 0x44, 0x54,
		0x43, 0x4F, 0x50, 0x52, 0x32, 0x30, 0x31, 0x30, 0x4D, 0x4F, 0x54, 0x4F, 0x52, 0x4F, 0x4C, 0x41, 
		0x20, 0x45, 0x2E, 0x50, 0x20, 0x43, 0x48, 0x41, 0x52, 0x47, 0x45, 0x20, 0x4F, 0x4E, 0x4C, 0x59
	},
    //HF5X - HV battery
    {
        0xE6, 0x01, 0x9E, 0x00, 0x00, 0x19, 0x14, 0x04, 0xE2, 0x23, 0x5A, 0x0E, 0x52, 0x95, 0x6B, 0x96, 
        0x9C, 0x5F, 0x31, 0x2F, 0x28, 0x1E, 0x15, 0x10, 0x11, 0x1E, 0x18, 0x0E, 0x18, 0x23, 0x35, 0x6A,
        0xD1, 0x75, 0xA5, 0x56, 0xD8, 0x00, 0x31, 0x7D, 0x5A, 0x40, 0xFF, 0x05, 0x01, 0x06, 0x0F, 0x00, 
        0x68, 0x08, 0xB9, 0x48, 0x9E, 0xA7, 0x10, 0x3C, 0x5F, 0x63, 0x69, 0x44, 0x4C, 0x52, 0x5A, 0x1C,
        0xBB, 0xB9, 0x4A, 0x9E, 0xA8, 0x10, 0x4A, 0x6A, 0x6F, 0x74, 0x4A, 0x5B, 0x62, 0x69, 0x1B, 0x39, 
        0xDC, 0x00, 0xA7, 0xBA, 0x4A, 0xA5, 0x85, 0xD2, 0x97, 0x5D, 0x68, 0x7C, 0x95, 0x1C, 0x32, 0x53,
        0x43, 0x4F, 0x50, 0x52, 0x32, 0x30, 0x31, 0x31, 0x4D, 0x4F, 0x54, 0x4F, 0x52, 0x4F, 0x4C, 0x41, 
        0x20, 0x45, 0x2E, 0x50, 0x20, 0x43, 0x48, 0x41, 0x52, 0x47, 0x45, 0x20, 0x4F, 0x4E, 0x4C, 0x59
    }
};

//NJDC-BSB jerry add the invalid family code, requirement of MOTO ++{
static const u8 invalid_family_code_talbe[4][8] = 
{
    {0x89,0x9b,0x8b,0xec,0x08,0x00,0x50,0x0c},
    {0x89,0xdc,0xc4,0x94,0x08,0x00,0x50,0xce},
    {0x89,0xf4,0xd6,0x98,0x08,0x00,0x50,0x34},
    {0x89,0x80,0xcb,0x27,0x09,0x00,0x50,0x40}
};
//NJDC-BSB jerry add the invalid family code, requirement of MOTO ++}

static int w1_bq2024_io(struct device *dev, u8 *cmd, u8 *buf, size_t count)
{
	struct w1_slave *sl = container_of(dev, struct w1_slave, dev);
	u8 crc = 0;

	if (!dev || !cmd || !buf || count < 0)
		return -1;

	if (!w1_reset_select_slave(sl)) {
		w1_write_8(sl->master, cmd[0]);
		w1_write_8(sl->master, cmd[1]);
		w1_write_8(sl->master, cmd[2]);
		crc = w1_read_8(sl->master);
		if (crc != w1_calc_crc8(cmd, 3)) {
			dev_err(dev, "%s: Write command failed!\n", __func__);
			return -1;
		}
		
		count = w1_read_block(sl->master, buf, count);
		crc = w1_read_8(sl->master);
		if (crc != w1_calc_crc8(buf, 32)) {
			dev_err(dev, "%s: Read data failed!\n", __func__);
			return -1;
		}
	}

	return count;
}

static int w1_bq2024_read_page(struct device *dev, u8 page_no, u8* page_data)
{
	u8 cmd[3];
	int rc = 0;

	// 2. Send Memory command, address low, and address high
	cmd[0] = W1_READ_MEMORY_PAGE_CRC;
	cmd[1] = 0x20 * page_no;
	cmd[2] = 0;
	rc = w1_bq2024_io(dev, cmd, page_data, 32);

	return rc;
}

/*FIH-NJ BSP Jerry add for HV battery. ++{*/
static int w1_bq2024_read_family_code(struct device *dev, u8* page_data)
{
    u8 cmd[1] = {W1_READ_ROM};
    u8 crc = 0, count = 0;
    struct w1_slave *sl = container_of(dev, struct w1_slave, dev);
    if (w1_reset_bus(sl->master))                    //should be reset slaver first
		return -1;
	w1_write_8(sl->master, cmd[0]);

    count = w1_read_block(sl->master, page_data, 8); //family code are 7 bytes + 
	crc = page_data[7];                              //CRC is 1 byte at the end
	if (crc != w1_calc_crc8(page_data, 7)) {
		dev_err(dev, "%s: Read family code CRC failed!\n", __func__);
		return -1;
	}
	return count;
}
/*FIH-NJ BSP Jerry add for HV battery. }++*/

static int w1_bq2024_check_battery_id_v1(struct w1_slave *sl)
{
	u8 eprom[128];
#if 0
	u8 test[128];
#endif
	int i, retries;

	// Clear EPROM buffer
	memset(eprom, 0, 128);
	
	// Read back page0 ~ page3
	for (i = 0; i < 4; i++) {
		retries = 0;
		if (w1_bq2024_read_page(&sl->dev, i, &eprom[i * 32]) < 0) {
			dev_err(&sl->dev, "%s: read page %d failed !!\n", __func__, i);
			while (1) { 
				mdelay(100);
				
				if(w1_bq2024_read_page(&sl->dev, i, &eprom[i * 32]) < 0) {
					dev_err(&sl->dev, "%s: [%d]read page %d failed again !!\n", __func__, retries, i);
					retries++;
				} else {
					dev_info(&sl->dev, "%s: read page %d successfully !!\n", __func__, i);
					break;
				}
				
				if (retries > 4)
					return -1;
			}
		} else
			dev_info(&sl->dev, "%s: read page %d successfully !!\n", __func__, i);
	}

	// Only check page4 byte 0~3 and byte 8~31.
	/* Debbie, 2011/08/30 { */
	/* modify for battery id detection */
	if (memcmp(&battery_id_table[0][96], &eprom[96], 4) != 0
		|| memcmp(&battery_id_table[0][104], &eprom[104], 24) != 0) {
	//if (memcmp(&battery_id_table[1][96], &eprom[96], 4) != 0
	//	|| memcmp(&battery_id_table[1][104], &eprom[104], 24) != 0) {
	/* Debbie, 2011/08/30 } */
		dev_err(&sl->dev, "%s: An invalid Battery is detected!\n", __func__);
		is_battery_valid = BATTERY_ID_INVALID;
		return 0;
	}

#if 0 // for testing
	for (i = 0; i < 128; i ++) {
		if (i % 16 == 0)
			sprintf(test, "%02x", eprom[i]);
		else
			sprintf(test, "%s, %02x", test, eprom[i]);
		if (i % 16 == 15)
			printk(KERN_INFO "%s\n", test);
	}
#endif

	dev_info(&sl->dev, "%s: A valid Battery is detected\n", __func__);
	is_battery_valid = BATTERY_ID_VALID;
    proc_comm_hv_battery_dection(0);
    battery_type = BATTERY_TYPE_4V2;
	
	return 0;
}

/*FIH-NJ BSP Jerry add for HV battery. ++{*/
static int w1_bq2024_check_battery_id_v2(struct w1_slave *sl)
{
	u8 eprom[128];
    u8 family_data[8] = {0};
#if 0
	u8 test[128];
#endif
	int i, retries, read_count = 0;

	// Clear EPROM buffer
	memset(eprom, 0, 128);
	
	// Read back page0 ~ page3
	for (i = 0; i < 4; i++) {
		retries = 0;
		if (w1_bq2024_read_page(&sl->dev, i, &eprom[i * 32]) < 0) {
			dev_err(&sl->dev, "%s: read page %d failed !!\n", __func__, i);
			while (1) { 
				mdelay(100);
				
				if(w1_bq2024_read_page(&sl->dev, i, &eprom[i * 32]) < 0) {
					dev_err(&sl->dev, "%s: [%d]read page %d failed again !!\n", __func__, retries, i);
					retries++;
				} else {
					dev_info(&sl->dev, "%s: read page %d successfully !!\n", __func__, i);
					break;
				}
				
				if (retries > 4)
					return -1;
			}
		} else
			dev_info(&sl->dev, "%s: read page %d successfully !!\n", __func__, i);
	}

    //Read out family code
    if (w1_bq2024_read_family_code(&sl->dev, &family_data[0]) < 0)
    {
        dev_err(&sl->dev, "%s: Read family code failed!\n", __func__);
        while(1)
        {
            mdelay(100);
            if(w1_bq2024_read_family_code(&sl->dev, &family_data[0]) < 0) 
            {
                dev_err(&sl->dev, "%s: [%d]Read family code failed again!\n", __func__,read_count); 
                read_count++;
            }
            else
            {
                dev_info(&sl->dev, "%s: Read family code successfully !!\n", __func__);
                break;
            }
            if(read_count > 4)
                return -1;
        }
    }
    else
        dev_info(&sl->dev, "%s: Read family code successfully !!\n", __func__);

#if 0  //for debug log.
    for(i=0;i<8;i++)
    {
		printk(KERN_ERR "%s: family_data[%d] = 0x%x\r\n", __func__, i, family_data[i]);

    }

    for(i=0;i<128;i++)
    {
		printk(KERN_ERR "%s: page[%d][%d] = 0x%x\r\n", __func__, (i/32),  (i%32), eprom[i]);

    }
#endif

    if((family_data[0] == 0x89) &&  //UID byte0
       (family_data[6] == 0x50) &&  //UID byte6
       ((family_data[5]&0xf0) == 0x0) &&  //UID byte5, 0x0X
       /*FIH-NJ-BSP Jerry modify to compatible with HW4X battery used by ITV. 2011/10/14 +{ */
       //(memcmp(&battery_id_table[3][32], &eprom[32], 1) == 0) &&    //page1 checksum 0xd1
       //(memcmp(&battery_id_table[3][64], &eprom[64], 2) == 0) &&    //page2 checksum 0xbb
       (memcmp(&battery_id_table[3][65], &eprom[65], 1) == 0) &&      //page2.2nd byte 0xb9, means max charge voltage
       /*FIH-NJ-BSP Jerry modify to compatible with HW4X battery used by ITV. 2011/10/14 +} */
       (memcmp(&battery_id_table[3][96], &eprom[96], 4) == 0) &&
       (memcmp(&battery_id_table[3][104], &eprom[104], 24) == 0)){  //check page3, byte 100~103 can be ignored.
       //if the above conditions are satisfied, can be consider as HV battery, except the following invalid family code.
        if((memcmp(&invalid_family_code_talbe[0][0], &family_data[0], 8) == 0) ||
           (memcmp(&invalid_family_code_talbe[1][0], &family_data[0], 8) == 0) ||
           (memcmp(&invalid_family_code_talbe[2][0], &family_data[0], 8) == 0) ||
           (memcmp(&invalid_family_code_talbe[3][0], &family_data[0], 8) == 0)  )
        {
            dev_err(&sl->dev, "%s: An invalid family code is detected!\n", __func__);
            is_battery_valid = BATTERY_ID_INVALID;

            #if 0  //for debug log
            for(i=0;i<8;i++)
            {
                printk(KERN_ERR "%s: family_data[%d] = 0x%x\r\n", __func__, i, family_data[i]);

            }
            #endif

            return 0;    
        }
        else{
            dev_info(&sl->dev, "%s: A valid Battery is detected\n", __func__);
            is_battery_valid = BATTERY_ID_VALID;
            proc_comm_hv_battery_dection(1);
            battery_type = BATTERY_TYPE_4V35;
        }

    }
    else{
        dev_err(&sl->dev, "%s: An invalid Battery is detected!\n", __func__);
		is_battery_valid = BATTERY_ID_INVALID;

        #if 0   //for debug log
        for(i=0;i<8;i++)
        {
            printk(KERN_ERR "%s: family_data[%d] = 0x%x\r\n", __func__, i, family_data[i]);

        }
        for(i=0;i<128;i++)
        {
            printk(KERN_ERR "%s: page[%d][%d] = 0x%x\r\n", __func__, (i/32),  (i%32), eprom[i]);

        }
        #endif
		return 0;
    }

#if 0  // for testing
	for (i = 0; i < 128; i ++) {
		if (i % 16 == 0)
			sprintf(test, "%02x", eprom[i]);
		else
			sprintf(test, "%s, %02x", test, eprom[i]);
		if (i % 16 == 15)
			printk(KERN_INFO "%s\n", test);
	}
#endif

	dev_info(&sl->dev, "%s: A valid Battery is detected\n", __func__);
	is_battery_valid = BATTERY_ID_VALID;
	
	return 0;
}
/*FIH-NJ BSP Jerry add for HV battery. }++*/

//FIH-NJ-BSP Jerry add, 2011/12/22. +++
static int w1_bq2024_check_battery_id_v3(struct w1_slave *sl)
{
	u8 eprom[128];
    u8 family_data[8] = {0};
#if 0
	u8 test[128];
#endif
	int i, retries;

	// Clear EPROM buffer
	memset(eprom, 0, 128);
	
	// Read back page0 ~ page3
	for (i = 0; i < 4; i++) {
		retries = 0;
		if (w1_bq2024_read_page(&sl->dev, i, &eprom[i * 32]) < 0) {
			dev_err(&sl->dev, "%s: read page %d failed !!\n", __func__, i);
			while (1) { 
				mdelay(100);
				
				if(w1_bq2024_read_page(&sl->dev, i, &eprom[i * 32]) < 0) {
					dev_err(&sl->dev, "%s: [%d]read page %d failed again !!\n", __func__, retries, i);
					retries++;
				} else {
					dev_info(&sl->dev, "%s: read page %d successfully !!\n", __func__, i);
					break;
				}
				
				if (retries > 4)
					return -1;
			}
		} else
			dev_info(&sl->dev, "%s: read page %d successfully !!\n", __func__, i);
	}

    //Read out family code
    if (w1_bq2024_read_family_code(&sl->dev, &family_data[0]) < 0) {
        dev_err(&sl->dev, "%s: Read family code failed!\n", __func__);
        mdelay(100);
        if(w1_bq2024_read_family_code(&sl->dev, &family_data[0]) < 0) {
            dev_err(&sl->dev, "%s: Read family code failed again!\n", __func__); 
        }
    }

    #if 0  //for debug log.
    for(i=0;i<8;i++)
    {
        printk(KERN_ERR "%s: family_data[%d] = 0x%x\r\n", __func__, i, family_data[i]);

    }

    for(i=0;i<128;i++)
    {
        printk(KERN_ERR "%s: page[%d][%d] = 0x%x\r\n", __func__, (i/32),  (i%32), eprom[i]);

    }
    #endif


    // 1)check moto copyright
	if (memcmp(&battery_id_table[3][96], &eprom[96], 4) != 0
		|| memcmp(&battery_id_table[3][104], &eprom[104], 24) != 0) 
    {
        goto battery_invalid;
	}
    // 2)check family code and manufacture ID
    if((family_data[0] != 0x89) ||      //UID byte0,family code
       (family_data[6] != 0x50) ||      //UID byte6,manufacture ID 0x500x
       ((family_data[5]&0xf0) != 0x0))  //UID byte5, 0x0X
    {
        goto battery_invalid;
    }

    // 3)check UID black list
    if((memcmp(&invalid_family_code_talbe[0][0], &family_data[0], 8) == 0) ||
       (memcmp(&invalid_family_code_talbe[1][0], &family_data[0], 8) == 0) ||
       (memcmp(&invalid_family_code_talbe[2][0], &family_data[0], 8) == 0) ||
       (memcmp(&invalid_family_code_talbe[3][0], &family_data[0], 8) == 0)  )
    {
        goto battery_invalid;
    }

    // 4) check page2 and page3 checksum and max charge voltage.
    if((eprom[32] == 0xE9) && (eprom[64] == 0xA9) && (eprom[65] == 0xB9))
    {
        printk(KERN_INFO "%s: battery type -- BATTERY_TYPE_4V35!\n", __func__);
        battery_type = BATTERY_TYPE_4V35;
        proc_comm_hv_battery_dection(1);  //inform modem the battery type is BATTERY_TYPE_4V35
        goto battery_valid;
    }
    else if ((eprom[65] == 0xB0) && 
             ((eprom[32] == 0x81)||(eprom[32] == 0x7C)||(eprom[32] == 0x51)||(eprom[32] == 0x60)||(eprom[32] == 0x59)) &&
             ((eprom[64] == 0x0D)||(eprom[64] == 0xD7)||(eprom[64] == 0xC1)||(eprom[64] == 0xD6)||(eprom[64] == 0x29)||(eprom[64] == 0xC8))
            )
    {
        printk(KERN_INFO "%s: battery type -- BATTERY_TYPE_4V2!\n", __func__);
        battery_type = BATTERY_TYPE_4V2;
        proc_comm_hv_battery_dection(0);  //inform modem the battery type is BATTERY_TYPE_4V2
        goto battery_valid;
    }
    else
    {
        goto battery_invalid;
    }
    
battery_valid:
	dev_info(&sl->dev, "%s: A valid Battery is detected\n", __func__);
	is_battery_valid = BATTERY_ID_VALID;
    return 0;

battery_invalid:
	dev_err(&sl->dev, "%s: An invalid Battery is detected!\n", __func__);
	is_battery_valid = BATTERY_ID_INVALID;
	return 0;
}
//FIH-NJ-BSP Jerry add, 2011/12/22. ---

bool w1_bq2024_is_battery_id_valid(void)
{
	int retries = 0;
    int ret_value;
	
	struct timespec ts;
	//refer to uptime
	do_posix_clock_monotonic_gettime(&ts);
	monotonic_to_bootbased(&ts);
	printk(KERN_INFO "%s\n", __func__);

	#ifndef CONFIG_FIH_FTM
	if(ts.tv_sec< 60)
	#else
	if(ts.tv_sec< 10)
	#endif
	{ 
		// let w1 search process have at least 6 times to add slave (default w1 polling interval is 10s).
        //FIH-NJ-Jerry, sometimes for an unknown reason, add slave at 54s. See TBE.B-1405.
		printk(KERN_INFO "%s(): %lu s,less than 60s, ignore check\n", __func__, ts.tv_sec);	
		return true; 
	}
	
	if (is_battery_valid == BATTERY_ID_UNKNOWN
		|| is_battery_valid == BATTERY_ID_INVALID) {

        /*if a battery has no EPROM, bq2024 add slave failed, the g_sl is a NULL point.
         FIH-NJ-BSP Jerry add, 2011/10/27. +++*/
        if( g_sl == NULL )
            return false;
        /*FIH-NJ-BSP Jerry add, 2011/10/27.---*/
			
		mutex_lock(&check_battery_id_lock);
		
		do {
            /*FIH-NJ BSP Jerry add for HV battery. {++*/
            if(battery_type_version == 1) {
                ret_value = w1_bq2024_check_battery_id_v1(g_sl);
            }
            else if(battery_type_version == 2) {
                ret_value = w1_bq2024_check_battery_id_v2(g_sl);
            }
            else {
                ret_value = w1_bq2024_check_battery_id_v3(g_sl);
            }
            /*FIH-NJ BSP Jerry add for HV battery. }++*/
			if (!ret_value)
				break;
			retries++;
			mdelay(200);
			printk(KERN_INFO "%s, retries %d\n", __func__, retries);
		} while (retries < 10);	
		
		mutex_unlock(&check_battery_id_lock);
	}

	
	/* FIH, Debbie, 2011/09/15 { */	
	if (is_battery_valid == BATTERY_ID_VALID )
	/* FIH, Debbie, 2011/09/15 } */	
		return true;
	else
		return false;
}
EXPORT_SYMBOL(w1_bq2024_is_battery_id_valid);

//FIH-NJ-BSP Jerry add, 2011/12/22. +++
int check_battery_type(void)
{
    return battery_type;

}
EXPORT_SYMBOL(check_battery_type);
//FIH-NJ-BSP Jerry add, 2011/12/22. ---

static ssize_t w1_bq2024_read_bin_v1(struct file *filp, struct kobject *kobj,
				  struct bin_attribute *bin_attr,
				  char *buf, loff_t off, size_t count)
{
	int retries = 0;
	
	do {
		if (!w1_bq2024_check_battery_id_v1(g_sl))
			break;
		retries++;
		mdelay(200);
		printk(KERN_INFO "%s, retries %d\n", __func__, retries);
	} while (retries < 10);

	return 0;
}

static ssize_t w1_bq2024_read_bin_v2(struct file *filp, struct kobject *kobj,
				  struct bin_attribute *bin_attr,
				  char *buf, loff_t off, size_t count)
{
	int retries = 0;
	
	do {
		if (!w1_bq2024_check_battery_id_v2(g_sl))
			break;
		retries++;
		mdelay(200);
		printk(KERN_INFO "%s, retries %d\n", __func__, retries);
	} while (retries < 10);

	return 0;
}

static ssize_t w1_bq2024_read_bin_v3(struct file *filp, struct kobject *kobj,
				  struct bin_attribute *bin_attr,
				  char *buf, loff_t off, size_t count)
{
	int retries = 0;
	
	do {
		if (!w1_bq2024_check_battery_id_v3(g_sl))
			break;
		retries++;
		mdelay(200);
		printk(KERN_INFO "%s, retries %d\n", __func__, retries);
	} while (retries < 10);

	return 0;
}

static struct bin_attribute w1_bq2024_bin_attr_v1 = {
	.attr = {
		.name = "w1_bq2024",
		.mode = S_IRUGO,
	},
	.read = w1_bq2024_read_bin_v1,
};
static struct bin_attribute w1_bq2024_bin_attr_v2 = {
	.attr = {
		.name = "w1_bq2024",
		.mode = S_IRUGO,
	},
	.read = w1_bq2024_read_bin_v2,
};

static struct bin_attribute w1_bq2024_bin_attr_v3 = {
	.attr = {
		.name = "w1_bq2024",
		.mode = S_IRUGO,
	},
	.read = w1_bq2024_read_bin_v3,
};

static int w1_bq2024_add_slave_v1(struct w1_slave *sl)
{
	int retries = 0;
	int ret;

	printk(KERN_INFO "%s\n",__func__);

	g_sl = sl;

	ret = sysfs_create_bin_file(&sl->dev.kobj, &w1_bq2024_bin_attr_v1);
	if (ret) {
		goto bin_attr_failed;
	}

	/*	FIH, Chandler, 11.11.13 
		issue IRM.B-3798:
		for a unknown reason, w1 master may re-add slave,and then can't check battery id success.
		At the same time, battery driver check battery id and cause kernel panic.
		
		workaround solution:
		the following case prevent w1 master from setting "is_battery_valid" to "BATTERY_ID_UNKNOWN" again.
	*/
	if(is_battery_valid == BATTERY_ID_VALID){
		goto success;
	}
	
	mutex_lock(&check_battery_id_lock);
	
	is_battery_valid = BATTERY_ID_UNKNOWN;
	do {
		if (!w1_bq2024_check_battery_id_v1(g_sl))
			break;
		retries++;
		mdelay(200);
		printk(KERN_INFO "%s, retries %d\n", __func__, retries);
	} while (retries < 10);
	
	mutex_unlock(&check_battery_id_lock);

	goto success;

bin_attr_failed:
success:
	return ret;
}

static int w1_bq2024_add_slave_v2(struct w1_slave *sl)
{
	int retries = 0;
	int ret;

	printk(KERN_INFO "%s\n",__func__);

	g_sl = sl;

	ret = sysfs_create_bin_file(&sl->dev.kobj, &w1_bq2024_bin_attr_v2);
	if (ret) {
		goto bin_attr_failed;
	}

    /* Njdc-BSP, Jerry add protection for issue of TBP.B-2390/2394, 2011/12/14. ++{ */
	if(is_battery_valid == BATTERY_ID_VALID){
		goto success;
	}
    /* Njdc-BSP, Jerry add protection for issue of TBP.B-2390/2394, 2011/12/14. }-- */

	mutex_lock(&check_battery_id_lock);
	
	is_battery_valid = BATTERY_ID_UNKNOWN;
	do {
        //Jerry add to resolve factory issue: can't to charge BF5X battery. 2012/01/06 ++++
        #ifndef CONFIG_FIH_FTM
		if (!w1_bq2024_check_battery_id_v2(g_sl))
			break;
        #else
		if (!w1_bq2024_check_battery_id_v1(g_sl))
			break;
        #endif
        //Jerry add to resolve factory issue: can't to charge BF5X battery. 2012/01/06 ----
		retries++;
		mdelay(200);
		printk(KERN_INFO "%s, retries %d\n", __func__, retries);
	} while (retries < 10);
	
	mutex_unlock(&check_battery_id_lock);

	goto success;

bin_attr_failed:
success:
	return ret;
}

static int w1_bq2024_add_slave_v3(struct w1_slave *sl)
{
	int retries = 0;
	int ret;

	printk(KERN_INFO "%s\n",__func__);

	g_sl = sl;

	ret = sysfs_create_bin_file(&sl->dev.kobj, &w1_bq2024_bin_attr_v3);
	if (ret) {
		goto bin_attr_failed;
	}

    /* Njdc-BSP, Jerry add protection for issue of TBP.B-2390/2394, 2011/12/14. ++{ */
	if(is_battery_valid == BATTERY_ID_VALID){
		goto success;
	}
    /* Njdc-BSP, Jerry add protection for issue of TBP.B-2390/2394, 2011/12/14. }-- */

	mutex_lock(&check_battery_id_lock);
	
	is_battery_valid = BATTERY_ID_UNKNOWN;
	do {
		if (!w1_bq2024_check_battery_id_v3(g_sl))
			break;
		retries++;
		mdelay(200);
		printk(KERN_INFO "%s, retries %d\n", __func__, retries);
	} while (retries < 10);
	
	mutex_unlock(&check_battery_id_lock);

	goto success;

bin_attr_failed:
success:
	return ret;
}

static void w1_bq2024_remove_slave_v1(struct w1_slave *sl)
{
	sysfs_remove_bin_file(&sl->dev.kobj, &w1_bq2024_bin_attr_v1);
}

static void w1_bq2024_remove_slave_v2(struct w1_slave *sl)
{
	sysfs_remove_bin_file(&sl->dev.kobj, &w1_bq2024_bin_attr_v2);
}

static void w1_bq2024_remove_slave_v3(struct w1_slave *sl)
{
	sysfs_remove_bin_file(&sl->dev.kobj, &w1_bq2024_bin_attr_v3);
}

static struct w1_family_ops w1_bq2024_fops_v1 = {
	.add_slave    = w1_bq2024_add_slave_v1,
	.remove_slave = w1_bq2024_remove_slave_v1,
};

static struct w1_family_ops w1_bq2024_fops_v2 = {
	.add_slave    = w1_bq2024_add_slave_v2,
	.remove_slave = w1_bq2024_remove_slave_v2,
};

static struct w1_family_ops w1_bq2024_fops_v3 = {
	.add_slave    = w1_bq2024_add_slave_v3,
	.remove_slave = w1_bq2024_remove_slave_v3,
};

static struct w1_family w1_bq2024_family_v1 = {
	.fid = W1_EPROM_BQ2024,
	.fops = &w1_bq2024_fops_v1,
};

static struct w1_family w1_bq2024_family_v2 = {
	.fid = W1_EPROM_BQ2024,
	.fops = &w1_bq2024_fops_v2,
};

static struct w1_family w1_bq2024_family_v3 = {
	.fid = W1_EPROM_BQ2024,
	.fops = &w1_bq2024_fops_v3,
};

/* FIH, Debbie, 2011/09/22 { */
/* update battery id status */
int w1_bq2024_get_battery_id_status(void)
{
	return is_battery_valid;
}
EXPORT_SYMBOL(w1_bq2024_get_battery_id_status);
/* FIH, Debbie, 2011/09/22 } */

static int __init w1_bq2024_init_v1(void* hdl)
{
    printk(KERN_INFO "1-Wire driver v1 for BQ2024 EPROM chip");
    battery_type_version = 1;
    fent = &w1_bq2024_family_v1;
	return w1_register_family(&w1_bq2024_family_v1);
}
DECLARE_DYN_LOADER(DRIVER_TYPE_BATTERY, 
                   BATTERY_V1, 
                   w1_bq2024_init_v1, 
                   LEVEL6);

static int __init w1_bq2024_init_v2(void* hdl)
{
    printk(KERN_INFO "1-Wire driver v2 for BQ2024 EPROM chip");
    //Jerry add to resolve factory issue: can't to charge BF5X battery. 2012/01/06 ++++
    #ifndef CONFIG_FIH_FTM
    battery_type_version = 2;
    #else
    battery_type_version = 1;
    #endif
    //Jerry add to resolve factory issue: can't to charge BF5X battery. 2012/01/06 ----
    fent = &w1_bq2024_family_v2;
	return w1_register_family(&w1_bq2024_family_v2);
}
DECLARE_DYN_LOADER(DRIVER_TYPE_BATTERY, 
                   BATTERY_V2, 
                   w1_bq2024_init_v2, 
                   LEVEL6);
//IRMPrime
static int __init w1_bq2024_init_v3(void* hdl)
{
    printk(KERN_INFO "1-Wire driver v3 for BQ2024 EPROM chip");
    battery_type_version = 3;
    fent = &w1_bq2024_family_v3;
	return w1_register_family(&w1_bq2024_family_v3);
}
DECLARE_DYN_LOADER(DRIVER_TYPE_BATTERY, 
                   BATTERY_V3, 
                   w1_bq2024_init_v3, 
                   LEVEL6);

static void __exit w1_bq2024_exit(void)
{
	w1_unregister_family(fent);
}

//module_init(w1_bq2024_init);
module_exit(w1_bq2024_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Audi Huang <AudiPCHuang@fihtdc.com>");
MODULE_DESCRIPTION("1-wire Driver TI BQ2024 EPROM chip");
