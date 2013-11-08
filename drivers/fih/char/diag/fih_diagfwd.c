/* Copyright (c) 2008-2011, Code Aurora Forum. All rights reserved.
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

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <linux/pm_runtime.h>
#include <linux/diagchar.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/smp_lock.h>
#ifdef CONFIG_DIAG_OVER_USB
#include <mach/usbdiag.h>
#endif
#include <mach/msm_smd.h>
#include <mach/socinfo.h>
#include <mach/restart.h>
#include "../../../char/diag/diagmem.h"
#include "../../../char/diag/diagchar.h"
#include "../../../char/diag/diagfwd.h"
#include "../../../char/diag/diagfwd_cntl.h"
#include "../../../char/diag/diagchar_hdlc.h"
#ifdef CONFIG_DIAG_SDIO_PIPE
#include "../../../char/diag/diagfwd_sdio.h"
#endif

/* FIH, JiaHao, 2011/09/14 { */
// G6: save BSP/fver into /data/fver
#define DIAG_READ_RETRY_COUNT    100  //100 * 5 ms = 500ms

enum 
{
EFS_ERR_NONE,
EFS_EPERM,
EFS_ENOENT,
EFS_EEXIST        =6, 
EFS_EBADF         =9,
EFS_ENOMEM        =12,
EFS_EACCES        =13,
EFS_EBUSY         =16,
EFS_EXDEV         =18,
EFS_ENODEV        =19,
EFS_ENOTDIR       =20,
EFS_EISDIR        =21,
EFS_EINVAL        =22,
EFS_EMFILE        =24,
EFS_ETXTBSY       =26,
EFS_ENOSPC        =28,
EFS_ESPIPE        =29,
EFS_FS_ERANGE     =34,
EFS_ENAMETOOLONG  =36,
EFS_ENOTEMPTY     =39,
EFS_ELOOP         =40,
EFS_ESTALE        =116,
EFS_EDQUOT        =122,
EFS_ENOCARD       =301,
EFS_EBADFMT       =302,
EFS_ENOTITM       =303,
EFS_EROLLBACK     =304,
EFS_ERR_UNKNOWN       =999,
};

enum
{
EFS_DIAG_TRANSACTION_OK = 0,
EFS_DIAG_TRANSACTION_NO_DATA,
EFS_DIAG_TRANSACTION_WRONG_DATA,
EFS_DIAG_TRANSACTION_NO_FILE_DIR,
EFS_DIAG_TRANSACTION_CMD_ERROR,
EFS_DIAG_TRANSACTION_ABORT,
};

#define READ_FILE_SIZE  0x200
#define EFS_BUFFER_SIZE (2*READ_FILE_SIZE)
#define EFS2_DIAG_OPEN_RES_SIZE  12     // Response 12 bytes
#define EFS2_DIAG_CLOSE_RES_SIZE  8     // Response 8 bytes

#define DIAG_DBG_MSG_G4(fmt, ...) \
        printk(fmt, ##__VA_ARGS__);
        //printk(diag_char_debug_mask, FIH_DEBUG_ZONE_G4, fmt, ##__VA_ARGS__);

#define DIAG_DBG_MSG_G6(fmt, ...) \
        printk(fmt, ##__VA_ARGS__);
        //printk(diag_char_debug_mask, FIH_DEBUG_ZONE_G6, fmt, ##__VA_ARGS__);

//uint32_t diag_char_debug_mask = 0;
static DEFINE_SPINLOCK(diag_smd_lock);
/* FIH, JiaHao, 2011/09/14 } */

/* FIH, JiaHao, 2011/09/14 { */
// efs_page = the index of page in fver which want to read
// efs_buf  = store the read back page of BSP/fver
static int efs_send_req(unsigned char * cmd_buf ,int cmd_size)
{
	unsigned long flags;
	int need, busy = -1, retry = 0;
	
	//paul// need = sizeof(cmd_buf);
	need = cmd_size;
	spin_lock_irqsave(&diag_smd_lock, flags);
	while (smd_write_avail(driver->ch) < need) {
		spin_unlock_irqrestore(&diag_smd_lock, flags);
		msleep(250);
		spin_lock_irqsave(&diag_smd_lock, flags);
	}

    busy = smd_write(driver->ch, cmd_buf, cmd_size);
    while (busy == -1 && retry < 10)
    {
        printk("[chhsieh]%s : smd_write = %d, retry = %d\n", __func__, busy, retry);
        spin_unlock_irqrestore(&diag_smd_lock, flags);
        msleep(10);
        spin_lock_irqsave(&diag_smd_lock, flags);
        busy = smd_write(driver->ch, cmd_buf, cmd_size);
        retry++;
    }
    spin_unlock_irqrestore(&diag_smd_lock, flags);
	//print_hex_dump(KERN_DEBUG, "", DUMP_PREFIX_OFFSET, 16, 1, cmd_buf, cmd_size, 0);
	
    return busy;
}

static int efs_recv_res(unsigned char * res_buf ,int res_size)
{
	unsigned long flags;
	//int need;
	static unsigned char responseBuf_temp[EFS_BUFFER_SIZE]; //Declaired as static to avoid compiler(gcc-4.4.0)'s warning
	struct diag_hdlc_decode_type hdlc;
	int ret = 0;//, type = 0;
    int sz;
    int gg = 0;
    int retry = 0;
    int rc = -EFS_DIAG_TRANSACTION_NO_DATA;
	hdlc.dest_ptr = res_buf;//driver->hdlc_buf;
	hdlc.dest_size = res_size;//USB_MAX_OUT_BUF;
	hdlc.src_ptr = responseBuf_temp;//data;
	//hdlc.src_size = res_size;//len;
	hdlc.src_size = sizeof(responseBuf_temp);//len;
	hdlc.src_idx = 0;
	hdlc.dest_idx = 0;
	hdlc.escaping = 0;
	
     memset(responseBuf_temp, 0, sizeof(responseBuf_temp));
	 for(retry = 0 ; retry<DIAG_READ_RETRY_COUNT ;)
	 {
        sz = smd_cur_packet_size(driver->ch);
        //printk(KERN_INFO "[wilson][WARNING]sz = %d\n", sz);
		if (sz == 0)
        {
            retry++;
            msleep(5);
			continue;
        }
		gg = smd_read_avail(driver->ch);
		//DIAG_DBG_MSG_G4(KERN_INFO "[wilson]smd_read_avail()= %d\n", gg);
		if (sz > gg)
		{
			DIAG_DBG_MSG_G4(KERN_INFO "[wilson][WARNING] smd_read_avail()= %d\n", gg);
			continue;
		}
		//if (sz > 535) {
		//	smd_read(driver->ch, 0, sz);
		//	continue;
		//}
		// sz should not be so large. So we take it as an abnormal case.
		if (sz >= sizeof(responseBuf_temp))
        {
            DIAG_DBG_MSG_G4(KERN_INFO "sz=%d, larger than our buffer!\n", sz);      
            goto lbExit;
        }
		spin_lock_irqsave(&diag_smd_lock, flags);//mutex_lock(&nmea_rx_buf_lock);
		if (smd_read(driver->ch, responseBuf_temp, sz) == sz) {
			spin_unlock_irqrestore(&diag_smd_lock, flags);//mutex_unlock(&nmea_rx_buf_lock);
			DIAG_DBG_MSG_G4(KERN_ERR "efs: not enough data?!\n");
			break;
		}
		//nmea_devp->bytes_read = sz;
		spin_unlock_irqrestore(&diag_smd_lock, flags);//mutex_unlock(&nmea_rx_buf_lock);
	 }
     if (retry >= DIAG_READ_RETRY_COUNT)
        goto lbExit;

	 ret = diag_hdlc_decode(&hdlc); //decode hdlc protocol

     rc = EFS_DIAG_TRANSACTION_OK;
lbExit:
    //print_hex_dump(KERN_DEBUG, "", DUMP_PREFIX_OFFSET, 16, 1, res_buf, res_size, 0);

    return rc;
}

static int efs_diag_transaction(unsigned char * TxBuf, int TxBufSize, unsigned char * RxBuf, int RxBufSize)
{
    int retry = 5;
    int rc = -1;
    int error_code = EFS_ERR_NONE;
    
    do
    {
        #ifdef SAVE_QXDM_LOG_TO_SD_CARD
        if (bLogStart || driver->usb_connected)
        #else
        if (driver->usb_connected)
        #endif
        {
            rc = -EFS_DIAG_TRANSACTION_ABORT;
            break;
        }
        
        if (efs_send_req(TxBuf, TxBufSize) == -1)
        {
            printk("[chhsieh]%s : efs_send_req fail\n", __func__);
            return -1;
        }
        
lbReadDataAgain:
        rc = efs_recv_res(RxBuf, RxBufSize);

        if (rc < 0)
        {
            DIAG_DBG_MSG_G4("Didn't get enough data! \n");
            if (retry-- < 0)
                break;
            else
                continue;
        }
        if (TxBuf[0] != RxBuf[0] || TxBuf[1] != RxBuf[1] || TxBuf[2] != RxBuf[2])
        {
            rc = -EFS_DIAG_TRANSACTION_WRONG_DATA;
            DIAG_DBG_MSG_G4("response not match. Read data again!\n");
            print_hex_dump(KERN_DEBUG, "", DUMP_PREFIX_OFFSET, 16, 1, RxBuf, RxBufSize, 0);
            if (retry-- < 0)
                break;
            else
                goto lbReadDataAgain;
        }
        else
        {
            if (RxBuf[0] == 0x4b && RxBuf[1] == 0x13)
            {
                switch (RxBuf[2])
                {
                    case 0x0d:  //Close dir
                    case 0x03:  //Close file
                        error_code = *(uint32_t *)&RxBuf[4];
                        break;
                    case 0x0c:  //Read dir
                        error_code = *(uint32_t *)&RxBuf[12];
                        break;
                    case 0x0b:  //Open dir
                    case 0x02:  //Open file
                        error_code = *(uint32_t *)&RxBuf[8];
                        break;
                    case 0x04:  //Read file
                        error_code = *(uint32_t *)&RxBuf[16];
                        break;
                    default:
                        error_code = EFS_ERR_UNKNOWN;
                        break;
                }
            }
            if (error_code)
            {
                if (error_code == EFS_ENOENT)
                {
                    DIAG_DBG_MSG_G4("No such file or directory," "Cmd:0x%x Error:0x%x\n", *(uint32_t *)&RxBuf[0], error_code);
                    rc = -EFS_DIAG_TRANSACTION_NO_FILE_DIR;
                    break;
                }
                else
                {
                    DIAG_DBG_MSG_G4("Cmd:0x%x Error:0x%x\n", *(uint32_t *)&RxBuf[0], error_code);
                    rc = -EFS_DIAG_TRANSACTION_CMD_ERROR;
                }
            }
        }
    }while(rc < 0 && retry-- > 0);

    return rc;
}

int read_efsfver(int efs_page, char *efs_buf)
{
	char src[] = "BSP/fver";

	char req_cmd[256];
	int  req_size;
	char res_dat[544];
	int  res_size;

	int rc;
	int fd;
	int offset;
	int bEOF;
	char *chr = NULL;
	int len;
	int total;

	char CMD_OpenFile[]  = {0x4b, 0x13, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x01, 0x00, 0x00};
	char CMD_CloseFile[] = {0x4b, 0x13, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00};
	char CMD_ReadFile[]  = {0x4b, 0x13, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	if (NULL == efs_buf) {
		DIAG_DBG_MSG_G6(KERN_ERR "read_efsfver: [WARNING] efs_buf is NULL\n");
		DIAG_DBG_MSG_G6(KERN_ERR "read_efsfver: [INFO] return the all byte(s) of %s\n", src);
	}
	DIAG_DBG_MSG_G6("read_efsfver: [INFO] efs_page = %d\n", efs_page);

	// open BSP/fver file
	DIAG_DBG_MSG_G6(KERN_ERR "read_efsfver: [INFO] Open %s (src)\n", src);
	memcpy(req_cmd, CMD_OpenFile, sizeof(CMD_OpenFile));
	memcpy((req_cmd + sizeof(CMD_OpenFile)), src, sizeof(src));
	req_size = sizeof(CMD_OpenFile) + sizeof(src) + 1; // +1 for null end of string
	memset(res_dat, sizeof(res_dat), 0);
	res_size = EFS2_DIAG_OPEN_RES_SIZE;

	rc = efs_diag_transaction(req_cmd, req_size, res_dat, res_size);
	if (rc < 0) {
		DIAG_DBG_MSG_G6(KERN_ERR "read_efsfver: [ERRPR] Open %s Failed #(-1)\n", src);
		total = (-1);
		return total;
	}
	fd = res_dat[4];
	offset = PAGE_SIZE * efs_page;
	DIAG_DBG_MSG_G6("read_efsfver: [INFO] offset = %d\n", offset);
	bEOF = 0;
	total = 0;

	// read BSP/fver file
	while (!bEOF)
	{
		memset(req_cmd, sizeof(req_cmd), 0);
		memcpy(req_cmd, CMD_ReadFile, sizeof(CMD_ReadFile));
		req_cmd[4] = fd;
		*(uint32_t *) &req_cmd[12] = offset;
		req_size = sizeof(CMD_ReadFile);
		memset(res_dat, sizeof(res_dat), 0);
		res_size = 544;

		rc = efs_diag_transaction(req_cmd, req_size, res_dat, res_size);
		if (rc < 0) {
			DIAG_DBG_MSG_G6(KERN_ERR "read_efsfver: [ERROR] Read %s Failed #(-2)\n", src);
			total = (-2);
			break;
		}

		offset += READ_FILE_SIZE;
		chr = &res_dat[20];
		len = (res_dat[13] << 8) + res_dat[12];
		DIAG_DBG_MSG_G6("read_efsfver: [INFO] len = %d\n", len);
		total += len;

		// write BSP/fver into efs_buf
		if (0 == len)
		{
			bEOF = 1;
		}
		else if (len < READ_FILE_SIZE)
		{
			bEOF = 1;
			if ((NULL != efs_buf)&&(total <= PAGE_SIZE)) {
				memcpy((efs_buf+(total-len)), chr, len);
			}
		}
		else
		{
			if ((NULL != efs_buf)&&(total <= PAGE_SIZE)) {
				memcpy((efs_buf+(total-len)), chr, len);
			}
		}

		if ((PAGE_SIZE <= total)&&(NULL != efs_buf)) {
			bEOF = 1;
		}
	} // while (!bEOF)

	// close BSP/fver file
	DIAG_DBG_MSG_G6("read_efsfver: [INFO] Close %s\n", src);
	memset(req_cmd, sizeof(req_cmd), 0);
	memcpy(req_cmd, CMD_CloseFile, sizeof(CMD_CloseFile));
	req_size = sizeof(CMD_CloseFile);
	memset(res_dat, sizeof(res_dat), 0);
	res_size = EFS2_DIAG_CLOSE_RES_SIZE;

	rc = efs_diag_transaction(req_cmd, req_size, res_dat, res_size);
	if (rc < 0) {
		DIAG_DBG_MSG_G6("read_efsfver: [ERROR] Close %s Failed\n", src);
	}

	DIAG_DBG_MSG_G6("read_efsfver: [INFO] total = %d\n", total);
	return total;
}
/* FIH, JiaHao, 2011/09/14 } */

