/*==========================================================================*
 * Copyright   2009 Chi Mei Communication Systems, Inc.
 *--------------------------------------------------------------------------*
 * PROJECT:
 * CREATED BY:  Lo, Chien Chung (chienchung.lo@gmail.com)
 * CREATED DATE: 04-16-2009
 *--------------------------------------------------------------------------*
 * PURPOSE: A char type driver of smd
 *==========================================================================*/

/*-------------------------------------------------------------------------*/
/*                         DEPENDANCY                                      */
/*-------------------------------------------------------------------------*/
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <asm/current.h>
#include <linux/crc-ccitt.h>
#include <linux/delay.h>
#include <mach/msm_smd.h>

#include "ftm_smd.h"
//FIH, WillChen, 2009/7/3++
/* [FXX_CR], Merge bsp kernel and ftm kernel*/
//#ifdef CONFIG_FIH_FXX
//extern int ftm_mode;
//#endif
//FIH, WillChen, 2009/7/3--
/*-------------------------------------------------------------------------*/
/*                         TYPE/CONSTANT/MACRO                             */
/*-------------------------------------------------------------------------*/
#if 1
#define D(x...) printk(x)
#else
#define D(x...)
#endif

/*-------------------------------------------------------------------------*/
/*                            VARIABLES                                    */
/*-------------------------------------------------------------------------*/
struct ftm_smd_char_dev *ftm_smd_driver;

/*-------------------------------------------------------------------------*/
/*                         Function prototype                             */
/*-------------------------------------------------------------------------*/
int ftm_smd_hdlc_decode(struct ftm_smd_hdlc_decode_type *hdlc);

static void ftm_smd_notify(void *ctxt, unsigned event);
static int ftm_smd_open(struct inode *inode, struct file *filp);
static int ftm_smd_release(struct inode *inode, struct file *filp);
static ssize_t ftm_smd_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
static ssize_t ftm_smd_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);
static unsigned int ftm_smd_poll(struct file *filp, struct poll_table_struct *wait);
static int ftm_smd_cleanup(void);
static int ftm_smd_setup_cdev(dev_t devno);
static void ftm_smd_init_mem(void);
static void ftm_smd_cleanup_mem(void);
static int __init ftm_smd_init(void);
static void __exit ftm_smd_exit(void);

/*-------------------------------------------------------------------------
 	Name 		: 	
	Description	:	
	Parameter 	: 	
	Return 		: 	
-------------------------------------------------------------------------*/
int ftm_smd_hdlc_decode(struct ftm_smd_hdlc_decode_type *hdlc)
{
	uint8_t *src_ptr = NULL, *dest_ptr = NULL;
	unsigned int src_length = 0, dest_length = 0;

	unsigned int len = 0;
	unsigned int i;
	uint8_t src_byte;

	int pkt_bnd = 0;

	if (hdlc && hdlc->src_ptr && hdlc->dest_ptr &&
	    (hdlc->src_size - hdlc->src_idx > 0) &&
	    (hdlc->dest_size - hdlc->dest_idx > 0)) {

		src_ptr = hdlc->src_ptr;
		src_ptr = &src_ptr[hdlc->src_idx];
		src_length = hdlc->src_size - hdlc->src_idx;

		dest_ptr = hdlc->dest_ptr;
		dest_ptr = &dest_ptr[hdlc->dest_idx];
		dest_length = hdlc->dest_size - hdlc->dest_idx;

		for (i = 0; i < src_length; i++) {

			src_byte = src_ptr[i];

			if (hdlc->escaping) {
				dest_ptr[len++] = src_byte ^ ESC_MASK;
				hdlc->escaping = 0;
			} else if (src_byte == ESC_CHAR) {
				if (i == (src_length - 1)) {
					hdlc->escaping = 1;
					i++;
					break;
				} else {
					dest_ptr[len++] = src_ptr[++i]
							  ^ ESC_MASK;
				}
			} else if (src_byte == CONTROL_CHAR) {
				dest_ptr[len++] = src_byte;
				pkt_bnd = 1;
				i++;
				break;
			} else {
				dest_ptr[len++] = src_byte;
			}

			if (len >= dest_length) {
				i++;
				break;
			}
		}

		hdlc->src_idx += i;
		hdlc->dest_idx += len;
	}

	return pkt_bnd;
}

/*-------------------------------------------------------------------------
 	Name 		: 	
	Description	:	
	Parameter 	: 	
	Return 		: 	
-------------------------------------------------------------------------*/
static void ftm_smd_notify(void *ctxt, unsigned event)
{
	//D(KERN_INFO "ftm_smd_notify! \n");

	if (event != SMD_EVENT_DATA)
		return;

	ftm_smd_driver->data_ready = 1;
	wake_up_interruptible(&ftm_smd_driver->wait_q);
}

/*-------------------------------------------------------------------------
 	Name 		: 	
	Description	:	
	Parameter 	: 	
	Return 		: 	
-------------------------------------------------------------------------*/
static int ftm_smd_open(struct inode *inode, struct file *filp)
{
	int result = 0;

	printk(KERN_INFO "ftm_smd_open! \n");
//FIH, WillChen, 2009/7/3++
/* [FXX_CR], Merge bsp kernel and ftm kernel*/
//#ifdef CONFIG_FIH_FXX
//	if (ftm_mode == 0)
//	{
//		printk(KERN_INFO "This is NOT FTM mode. return without open!!!\n");
//		return -ENOMEM;
//	}
//#endif
//FIH, WillChen, 2009/7/3--

	if (ftm_smd_driver) {
		mutex_lock(&ftm_smd_driver->ftm_smd_mutex);

		result = smd_open("DIAG", &ftm_smd_driver->ch, ftm_smd_driver, ftm_smd_notify);

		mutex_unlock(&ftm_smd_driver->ftm_smd_mutex);
		
		printk(KERN_INFO "diag opened SMD port ; r = %d\n", result);
		
		return result;
	}
	return -ENOMEM;	
}

/*-------------------------------------------------------------------------
 	Name 		: 	
	Description 	:	
	Parameter 	: 	
	Return 		: 	
-------------------------------------------------------------------------*/
static int ftm_smd_release(struct inode *inode, struct file *filp)
{
	D(KERN_INFO "ftm_smd_release! \n");

	mutex_lock(&ftm_smd_driver->ftm_smd_mutex);
	smd_close(ftm_smd_driver->ch);
	mutex_unlock(&ftm_smd_driver->ftm_smd_mutex);
	
	return 0;
}

/*-------------------------------------------------------------------------
 	Name 		: 	
	Description 	:	
	Parameter 	: 	
	Return 		: 	
-------------------------------------------------------------------------*/
static ssize_t ftm_smd_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	void *smd_buf;
	int len = 0;
	int err;
	//int i;

	//D(KERN_INFO "ftm_smd_read! \n");

	mutex_lock(&ftm_smd_driver->ftm_smd_mutex);

	len = smd_read_avail(ftm_smd_driver->ch);
	while(len <= 0){
		if(filp->f_flags & O_NONBLOCK) {
			mutex_unlock(&ftm_smd_driver->ftm_smd_mutex);
			return -EAGAIN;
		}

		if(wait_event_interruptible(ftm_smd_driver->wait_q, ftm_smd_driver->data_ready)) {
			mutex_unlock(&ftm_smd_driver->ftm_smd_mutex);
			return -ERESTARTSYS;
		}
		
		len = smd_read_avail(ftm_smd_driver->ch);
	}

	if (len > MAX_BUF) {
		printk(KERN_INFO "diag dropped num bytes = %d\n", len);
		ftm_smd_driver->data_ready = 0;
		mutex_unlock(&ftm_smd_driver->ftm_smd_mutex);
		return -EFAULT;
	}
	else {

		smd_buf = ftm_smd_driver->buf;
		if (!smd_buf) {
			
			printk(KERN_INFO "Out of diagmem for a9\n");

		} 
		else {
			smd_read_from_cb(ftm_smd_driver->ch, smd_buf, len);


			//if(len > 0){
			//	D("\n");
			//	D("ftm_smd_read: len=%x\n", len);
			//	for(i = 0; i < len; i++)
			//		D("%2x ", ftm_smd_driver->buf[i]);
			//	D("\n\n");
			//}

			err = copy_to_user(buf, smd_buf, len);
			if(err){
				printk(KERN_INFO "copy_to_user result = %d\n", len);
				ftm_smd_driver->data_ready = 0;
				mutex_unlock(&ftm_smd_driver->ftm_smd_mutex);
				return -EFAULT;
			}
		}
	}

	ftm_smd_driver->data_ready = 0;


	mutex_unlock(&ftm_smd_driver->ftm_smd_mutex);
	
	return len;
}

/*-------------------------------------------------------------------------
 	Name 		: 	
	Description 	:	
	Parameter 	: 	
	Return 		: 	
-------------------------------------------------------------------------*/
static ssize_t ftm_smd_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	struct ftm_smd_hdlc_decode_type hdlc;
	
	int ret;
	int err;
	int i;
	unsigned int len = 0;

	//D(KERN_INFO "ftm_smd_write! \n");

	mutex_lock(&ftm_smd_driver->ftm_smd_write_mutex);

	err = copy_from_user(ftm_smd_driver->buf, buf, count);
	if (err) {
		printk(KERN_INFO "ftm_smd : copy_from_user failed \n");
		mutex_unlock(&ftm_smd_driver->ftm_smd_write_mutex);
		return -EFAULT;
	}

	len = count;

	//D("\n");
	//D("ftm_smd_write: len=%x\n", len);
	//for(i = 0; i < len; i++)
	//	D("%2x ", ftm_smd_driver->buf[i]);
	//D("\n\n");

	hdlc.dest_ptr = ftm_smd_driver->hdlc_buf;
	hdlc.dest_size = MAX_BUF;
	hdlc.src_ptr = ftm_smd_driver->buf;
	hdlc.src_size = len;
	hdlc.src_idx = 0;
	hdlc.dest_idx = 0;
	hdlc.escaping = 0;

	ret = ftm_smd_hdlc_decode(&hdlc);

	if(!ret){
		printk("\n");
		printk("this will be discarded: len=%x\n", len);
		for(i = 0; i < len; i++)
			printk("%2x ", ftm_smd_driver->buf[i]);
		printk("\n\n");

		for(i = 0; i < hdlc.dest_idx; i++)
			printk("%2x ", ftm_smd_driver->hdlc_buf[i]);
		printk("\n\n");

		return count;
	}

	if ((ftm_smd_driver->ch) && (ret) && (hdlc.dest_idx > 3)){
		
		//D("\n");
		//D("HDLC: len=%x\n", hdlc.dest_idx);
		//for(i = 0; i < hdlc.dest_idx; i++)
		//	D("%2x ", ftm_smd_driver->hdlc_buf[i]);
		//D("\n\n");

		len = hdlc.dest_idx - 3;
		ret = smd_write_avail(ftm_smd_driver->ch);
		while (ret < len) {
			mutex_unlock(&ftm_smd_driver->ftm_smd_write_mutex);
			msleep(250);
			mutex_lock(&ftm_smd_driver->ftm_smd_write_mutex);
			ret = smd_write_avail(ftm_smd_driver->ch);
	}

		ret = smd_write(ftm_smd_driver->ch, ftm_smd_driver->hdlc_buf, len);
	
		if (ret != len) {
			printk(KERN_ERR "ERROR:%s:%i:%s: "
				"smd_write(ch,buf,count = %i) ret %i.\n",
				__FILE__,
				__LINE__,
				__func__,
				len,
				ret);
		}
		
	}
	
	mutex_unlock(&ftm_smd_driver->ftm_smd_write_mutex);
	
	return count;
}

/*-------------------------------------------------------------------------
 	Name 		: 	
	Description	:	
	Parameter 	: 	
	Return 		: 	
-------------------------------------------------------------------------*/
static unsigned int ftm_smd_poll(struct file *filp, struct poll_table_struct *wait)
{
	unsigned mask = 0;
	int len = 0;

	mutex_lock(&ftm_smd_driver->ftm_smd_mutex);
	len = smd_read_avail(ftm_smd_driver->ch);
	if(len > 0)
		mask |= POLLIN | POLLRDNORM;

	if (!mask) {
		poll_wait(filp, &ftm_smd_driver->wait_q, wait);
		len = smd_read_avail(ftm_smd_driver->ch);
		if(len > 0)
			mask |= POLLIN | POLLRDNORM;
	}
	mutex_unlock(&ftm_smd_driver->ftm_smd_mutex);

	return mask;
}

/*-------------------------------------------------------------------------*/
/*                         file operation                                      */
/*-------------------------------------------------------------------------*/
struct file_operations ftm_smd_fops = {
	.owner =	THIS_MODULE,	
	.read =	ftm_smd_read,
	.write =	ftm_smd_write,
	.poll =	ftm_smd_poll,
	.open =	ftm_smd_open,
	.release =	ftm_smd_release,
};

/*-------------------------------------------------------------------------
 	Name 		: 	
	Description	:	
	Parameter 	: 	
	Return 		: 	
-------------------------------------------------------------------------*/
static int ftm_smd_cleanup(void)
{
	if (ftm_smd_driver) {
		if (ftm_smd_driver->cdev) {
			/* TODO - Check if device exists before deleting */
			device_destroy(ftm_smd_driver->ftm_smd_class,
				MKDEV(ftm_smd_driver->major,
				ftm_smd_driver->minor));
			
			cdev_del(ftm_smd_driver->cdev);
		}
		
		if (!IS_ERR(ftm_smd_driver->ftm_smd_class))
			class_destroy(ftm_smd_driver->ftm_smd_class);
		
		kfree(ftm_smd_driver);
	}

	return 0;
}

/*-------------------------------------------------------------------------
 	Name 		: 	
	Description	:	
	Parameter 	: 	
	Return 		: 	
-------------------------------------------------------------------------*/
static int ftm_smd_setup_cdev(dev_t devno)
{
	int err;

	cdev_init(ftm_smd_driver->cdev, &ftm_smd_fops);

	ftm_smd_driver->cdev->owner = THIS_MODULE;
	ftm_smd_driver->cdev->ops = &ftm_smd_fops;

	err = cdev_add(ftm_smd_driver->cdev, devno, 1);

	if (err) {
		printk(KERN_INFO "ftm_smd cdev registration failed !\n\n");
		return -1;
	}

	ftm_smd_driver->ftm_smd_class = class_create(THIS_MODULE, "ftm_smd");

	if (IS_ERR(ftm_smd_driver->ftm_smd_class)) {
		printk(KERN_ERR "Error creating ftm_smd class.\n");
		return -1;
	}

	device_create(ftm_smd_driver->ftm_smd_class, NULL, devno,
		(void *)ftm_smd_driver, "ftm_smd");

	return 0;
}

/*-------------------------------------------------------------------------
 	Name 		: 	
	Description	:	
	Parameter 	: 	
	Return 		: 	
-------------------------------------------------------------------------*/
static void ftm_smd_init_mem(void)
{
	if (ftm_smd_driver->buf  == NULL &&
		(ftm_smd_driver->buf = kzalloc(MAX_BUF, GFP_KERNEL)) == NULL)
			goto err;
	
	if (ftm_smd_driver->hdlc_buf == NULL
		&& (ftm_smd_driver->hdlc_buf = kzalloc(HDLC_MAX, GFP_KERNEL)) == NULL)
			goto err;

	return;

err:
	printk(KERN_INFO "\n Could not initialize ftm_smd buffers \n");
	kfree(ftm_smd_driver->buf);
	kfree(ftm_smd_driver->hdlc_buf);
}

/*-------------------------------------------------------------------------
 	Name 		: 	
	Description	:	
	Parameter 	: 	
	Return 		: 	
-------------------------------------------------------------------------*/
static void ftm_smd_cleanup_mem(void)
{
	printk(KERN_INFO "ftm_smd_cleanup_mem!\n");

	ftm_smd_driver->ch = 0;

	kfree(ftm_smd_driver->buf);
	kfree(ftm_smd_driver->hdlc_buf);
}

/* FIH, CHHsieh, 2011/11/28 { */
/* setup fver for ftm mode */
#define ESF2_CMD_RETRY_CONT     100  //100 * 5 ms = 500ms
#define EFS2_CMD_REQUEST_SIZE   256
#define EFS2_CMD_RESPONSE_SIZE  544  // 512+32=544

#define EFS2_FILE_READ_SIZE  0x200
#define EFS2_BUFFER_SIZE (2*EFS2_FILE_READ_SIZE)

#define EFS2_DIAG_OPENDIR_RES_SIZE   12  // Response 12 bytes
#define EFS2_DIAG_READDIR_RES_SIZE   (40 + 64)  // Response 40 bytes + variable string
#define EFS2_DIAG_OPEN_RES_SIZE      12  // Response 12 bytes
#define EFS2_DIAG_READ_RES_SIZE      20  // Response 20 bytes + variable data
#define EFS2_DIAG_CLOSE_RES_SIZE     8   // Response 8 bytes
#define EFS2_DIAG_CLOSEDIR_RES_SIZE  8   // Response 8 bytes

static DEFINE_SPINLOCK(diag_smd_lock);

enum {
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
    EFS_ERR_UNKNOWN   =999,
};

enum {
    EFS_DIAG_TRANSACTION_OK = 0,
    EFS_DIAG_TRANSACTION_NO_DATA,
    EFS_DIAG_TRANSACTION_WRONG_DATA,
    EFS_DIAG_TRANSACTION_NO_FILE_DIR,
    EFS_DIAG_TRANSACTION_CMD_ERROR,
    EFS_DIAG_TRANSACTION_ABORT,
};

static int efs_send_req(unsigned char * cmd_buf ,int cmd_size)
{
    unsigned long flags;
    int need, busy = -1, retry = 0;

    need = cmd_size;
    spin_lock_irqsave(&diag_smd_lock, flags);

    while (smd_write_avail(ftm_smd_driver->ch) < need) {
        spin_unlock_irqrestore(&diag_smd_lock, flags);
        msleep(250);
        spin_lock_irqsave(&diag_smd_lock, flags);
    }

    busy = smd_write(ftm_smd_driver->ch, cmd_buf, cmd_size);
    while (busy == -1 && retry < 10)
    {
        //printk("[chhsieh]%s : smd_write = %d, retry = %d\n", __func__, busy, retry);
        spin_unlock_irqrestore(&diag_smd_lock, flags);
        msleep(10);
        spin_lock_irqsave(&diag_smd_lock, flags);
        busy = smd_write(ftm_smd_driver->ch, cmd_buf, cmd_size);
        retry++;
    }
    spin_unlock_irqrestore(&diag_smd_lock, flags);
    
    return busy;
}

static int efs_recv_res(unsigned char * res_buf ,int res_size)
{
    unsigned long flags;
    static unsigned char responseBuf_temp[EFS2_BUFFER_SIZE];
    struct ftm_smd_hdlc_decode_type hdlc;
    int ret = 0;
    int sz;
    int gg = 0;
    int retry = 0;
    int rc = -EFS_DIAG_TRANSACTION_NO_DATA;

    hdlc.dest_ptr = res_buf;
    hdlc.dest_size = res_size;
    hdlc.src_ptr = responseBuf_temp; //data;
    hdlc.src_size = sizeof(responseBuf_temp); //len
    hdlc.src_idx = 0;
    hdlc.dest_idx = 0;
    hdlc.escaping = 0;
    memset(responseBuf_temp, 0, sizeof(responseBuf_temp));

    for (retry = 0; retry < ESF2_CMD_RETRY_CONT; )
    {
        sz = smd_cur_packet_size(ftm_smd_driver->ch);
        if (sz == 0) {
            retry++;
            msleep(5);
            continue;
        }

        gg = smd_read_avail(ftm_smd_driver->ch);
        if (sz > gg) {
            continue;
        }

        if (sz >= sizeof(responseBuf_temp)) {
            goto lbExit_efs_recv_res;
        }

        spin_lock_irqsave(&diag_smd_lock, flags);
        if (smd_read(ftm_smd_driver->ch, responseBuf_temp, sz) == sz) {
            spin_unlock_irqrestore(&diag_smd_lock, flags);
            break;
        }
        spin_unlock_irqrestore(&diag_smd_lock, flags);
    }

    if (retry >= ESF2_CMD_RETRY_CONT) {
        goto lbExit_efs_recv_res;
    }

    ret = ftm_smd_hdlc_decode(&hdlc);
    rc = EFS_DIAG_TRANSACTION_OK;

lbExit_efs_recv_res:
    return rc;
}

static int efs_diag_func(unsigned char * TxBuf, int TxBufSize, unsigned char * RxBuf, int RxBufSize)
{
    int retry = 5;
    int rc = -1;
    int error_code = EFS_ERR_NONE;

    do
    {
        if (efs_send_req(TxBuf, TxBufSize) == -1)
        {
            printk("[chhsieh]%s : efs_send_req fail\n", __func__);
            return -1;
        }

lbReadDataAgain:
        rc = efs_recv_res(RxBuf, RxBufSize);

        if (rc < 0)
        {
            if (retry-- < 0)
                break;
            else
                continue;
        }

        if (TxBuf[0] != RxBuf[0] || TxBuf[1] != RxBuf[1] || TxBuf[2] != RxBuf[2])
        {
            rc = -EFS_DIAG_TRANSACTION_WRONG_DATA;
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
                    printk(KERN_ERR "No such file or directory," "Cmd:0x%x Error:0x%x\n", *(uint32_t *)&RxBuf[0], error_code);
                    rc = -EFS_DIAG_TRANSACTION_NO_FILE_DIR;
                    break;
                }
                else
                {
                    printk(KERN_ERR "Cmd:0x%x Error:0x%x\n", *(uint32_t *)&RxBuf[0], error_code);
                    rc = -EFS_DIAG_TRANSACTION_CMD_ERROR;
                }
            }
        }
    } while(rc < 0 && retry-- > 0);

    return rc;
}

// efs_page = the index of page in fver which want to read
// efs_buf  = store the read back page of BSP/fver 
int read_efsfver(int efs_page, char *efs_buf)
{
    char src[] = "BSP/fver";

    char req_cmd[EFS2_CMD_REQUEST_SIZE];
    int  req_size;
    char res_dat[EFS2_CMD_RESPONSE_SIZE];
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

    if (!ftm_smd_driver) {
        printk(KERN_ERR "read_efsfver: [ERROR] ftm_smd_driver is NULL\n");
        return (-3);
    }

    // open DIAG
    mutex_lock(&ftm_smd_driver->ftm_smd_mutex);
    smd_open("DIAG", &ftm_smd_driver->ch, ftm_smd_driver, ftm_smd_notify);
    mutex_unlock(&ftm_smd_driver->ftm_smd_mutex);

    if (!ftm_smd_driver->ch) {
        printk(KERN_ERR "read_efsfver: [ERROR] ftm_smd_driver->ch is NULL\n");
        return (-4);
    }

    if (NULL == efs_buf) {
        printk(KERN_ERR "read_efsfver: [WARNING] efs_buf is NULL\n");
        printk(KERN_ERR "read_efsfver: [INFO] return the all byte(s) of %s\n", src);
    }
    printk("read_efsfver: [INFO] efs_page = %d\n", efs_page);

    // open BSP/fver file
    printk(KERN_ERR "read_efsfver: [INFO] Open %s (src)\n", src);
    memcpy(req_cmd, CMD_OpenFile, sizeof(CMD_OpenFile));
    memcpy((req_cmd + sizeof(CMD_OpenFile)), src, sizeof(src));
    req_size = sizeof(CMD_OpenFile) + sizeof(src) + 1; // +1 for null end of string
    memset(res_dat, sizeof(res_dat), 0);
    res_size = EFS2_DIAG_OPEN_RES_SIZE;

    rc = efs_diag_func(req_cmd, req_size, res_dat, res_size);
    if (rc < 0) {
        printk(KERN_ERR "read_efsfver: [ERRPR] Open %s Failed #(-1)\n", src);
        total = (-1);
        goto lbExit_read_efsfver;
    }
    fd = res_dat[4];
    offset = PAGE_SIZE * efs_page;
    printk("read_efsfver: [INFO] offset = %d\n", offset);
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
        res_size = EFS2_CMD_RESPONSE_SIZE;

        rc = efs_diag_func(req_cmd, req_size, res_dat, res_size);
        if (rc < 0) {
            printk(KERN_ERR "read_efsfver: [ERROR] Read %s Failed #(-2)\n", src);
            total = (-2);
            break;
        }

        offset += EFS2_FILE_READ_SIZE;
        chr = &res_dat[20];
        len = (res_dat[13] << 8) + res_dat[12];
        printk("read_efsfver: [INFO] len = %d\n", len);
        total += len;

        // write BSP/fver into efs_buf
        if (0 == len)
        {
            bEOF = 1;
        }
        else if (len < EFS2_FILE_READ_SIZE)
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
    printk("read_efsfver: [INFO] Close %s\n", src);
    memset(req_cmd, sizeof(req_cmd), 0);
    memcpy(req_cmd, CMD_CloseFile, sizeof(CMD_CloseFile));
    req_size = sizeof(CMD_CloseFile);
    memset(res_dat, sizeof(res_dat), 0);
    res_size = EFS2_DIAG_CLOSE_RES_SIZE;

    rc = efs_diag_func(req_cmd, req_size, res_dat, res_size);
    if (rc < 0) {
        printk("read_efsfver: [ERROR] Close %s Failed\n", src);
    }

    printk("read_efsfver: [INFO] total = %d\n", total);

lbExit_read_efsfver:

    // close DIAG
    if (ftm_smd_driver->ch) {
        mutex_lock(&ftm_smd_driver->ftm_smd_mutex);
        smd_close(ftm_smd_driver->ch);
        mutex_unlock(&ftm_smd_driver->ftm_smd_mutex);
    }

    return total;
}
/* FIH, CHHsieh, 2011/11/28 } */

/*-------------------------------------------------------------------------
 	Name 		: 	
	Description 	:	
	Parameter 	: 	
	Return 		: 	
-------------------------------------------------------------------------*/
static int __init ftm_smd_init(void)
{
	dev_t devno;
	int error;
	
	printk(KERN_INFO "FTM_SMD driver initializing ...\n");
//FIH, WillChen, 2009/7/3++
/* [FXX_CR], Merge bsp kernel and ftm kernel*/
//#ifdef CONFIG_FIH_FXX
//	if (ftm_mode == 0)
//	{
//		printk(KERN_INFO "This is NOT FTM mode. return without init!!!\n");
//		return -1;
//	}
//#endif
//FIH, WillChen, 2009/7/3--

	ftm_smd_driver = kzalloc(sizeof(struct ftm_smd_char_dev), GFP_KERNEL);	

	if (ftm_smd_driver) {

		ftm_smd_driver->data_ready = 0;
		
		init_waitqueue_head(&ftm_smd_driver->wait_q);
		mutex_init(&ftm_smd_driver->ftm_smd_mutex);
		mutex_init(&ftm_smd_driver->ftm_smd_write_mutex);
		ftm_smd_init_mem();
		
		/* Get major number from kernel and initialize */
		error = alloc_chrdev_region(&devno, 0, 1, "ftm_smd");
		if (!error) {
			ftm_smd_driver->major = MAJOR(devno);
			ftm_smd_driver->minor = MINOR(devno);
		} else {
			printk(KERN_INFO "Major number not allocated \n");
			goto fail;
		}
		
		ftm_smd_driver->cdev = cdev_alloc();
		
		error = ftm_smd_setup_cdev(devno);
		if (error)
			goto fail;

	} else {
		printk(KERN_INFO "kzalloc failed\n");
		goto fail;
	}

	printk(KERN_INFO "FTM_SMD driver initialized ...\n");
	
	return 0;

fail:
	printk(KERN_INFO "Failed to initialize FTM_SMD driver ...\n");
	ftm_smd_cleanup();
	return -1;

}

/*-------------------------------------------------------------------------
 	Name 		: 	
	Description	:	
	Parameter 	: 	
	Return 		: 	
-------------------------------------------------------------------------*/
static void __exit ftm_smd_exit(void)
{
	printk(KERN_INFO "diagchar exiting ..\n");

	ftm_smd_cleanup_mem(); // This should be first before ftm_smd_cleanup().
	ftm_smd_cleanup();
	
	printk(KERN_INFO "done diagchar exit\n");
}

module_init(ftm_smd_init);
module_exit(ftm_smd_exit);
MODULE_DESCRIPTION("FTM SMD Driver");
MODULE_AUTHOR("Lo, Chien Chung <chienchung.lo@gmail.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.1");

