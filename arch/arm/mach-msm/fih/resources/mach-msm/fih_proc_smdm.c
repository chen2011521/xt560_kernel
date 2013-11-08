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

#define  SMDM_ADDR IOMEM(0xFB570000)	/* PHY : 0x2D370000~~0x2D380000 */

static int proc_comm_smdm(uint32_t * psmem_proc_comm_oem_data2)
{
    uint32_t smem_proc_comm_oem_data1   = SMEM_PROC_COMM_OEM_PRL_RESTORE;
    int32_t data[2];

	return msm_proc_comm_oem_multi(	PCOM_CUSTOMER_CMD1,
									&smem_proc_comm_oem_data1,
									psmem_proc_comm_oem_data2,
									data,
									0);
}

#include <linux/proc_fs.h>

static int g_newdata = 0;

/* Return Value
 *  0	modem do nothing
 *  1	PRL successfully write to NV item
 *  2	PRL fail to write to NV item
 */

static int smdm_read_proc
	(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	*eof = 1;

	if (g_newdata) {
		uint32_t smem_proc_comm_oem_data2 = 0; //useless
		int ret = proc_comm_smdm(&smem_proc_comm_oem_data2);
		g_newdata = 0;

		if (ret) {
			printk(KERN_INFO "smdm_read_proc fail : %d\n", ret);
			return snprintf(page, count, "%cOK!\n", (ret < 0) ? '0' : '2');
		}
		else {
			printk(KERN_INFO "smdm_read_proc : 0x%x\n", smem_proc_comm_oem_data2);
			return snprintf(page, count, "1OK!\n");
		}
	}
	else {
		printk(KERN_INFO "smdm_read_proc abort : without new data.\n");
		return snprintf(page, count, "0OK!\n");
	}
}

static int smdm_write_proc(struct file *file, const char __user *buffer,
	unsigned long count, void *data)
{
	printk(KERN_INFO "smdm_write_proc(buffer[%x], count(%lu), data[%x])\n", (unsigned int)buffer, count, (unsigned int)data);
	memcpy(SMDM_ADDR, buffer, count);
	g_newdata = 1;
	return count;
}

static int __init smdm_init(void)
{
	struct proc_dir_entry *d_entry;

	d_entry = create_proc_entry("smdm", S_IRWXUGO, NULL);
	if (d_entry) {
		d_entry->read_proc = smdm_read_proc;
		d_entry->write_proc = smdm_write_proc;
		d_entry->data = NULL;
	}

	return 0;
}

static void __exit smdm_exit(void)
{
	remove_proc_entry("smdm", NULL);
}

module_init(smdm_init);
module_exit(smdm_exit);

MODULE_DESCRIPTION("PROC_SMDM driver");

