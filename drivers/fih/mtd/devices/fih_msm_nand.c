#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/proc_fs.h>

struct flash_identification {
	uint32_t flash_id;
	uint32_t mask;
	uint32_t density;
	uint32_t widebus;
	uint32_t pagesize;
	uint32_t blksize;
	uint32_t oobsize;
	char    *device_name;
};

struct flash_identification supported_flash_table[] =
{
	/* Flash ID   ID Mask Density(MB)  Wid Pgsz   Blksz   oobsz    Manuf */
	{0x00000000, 0xFFFFFFFF,         0, 0,    0,         0,  0,  "ONFI"}, /*ONFI*/
	/* Note: Width flag is 0 for 8 bit Flash and 1 for 16 bit flash      */
	/* Note: The First row will be filled at runtime during ONFI probe   */
};

int supported_flash_index = 0;

static struct proc_dir_entry *proc_nandinfo;

extern char* fih_read_flash_name_from_smem(void);

//paul
static int msm_nand_read_proc (char *page, char **start, off_t off, int count,
			  int *eof, void *data_unused)
{
	//int len, l, i;
	int len;
        off_t   begin = 0;
	/* FIH, Debbie Sun, 2010/05/24 { */
	/* get device name form share memory */
	char * flash_name;
	flash_name = fih_read_flash_name_from_smem();
	/* FIH, Debbie Sun, 2010/05/24 } */

//	mutex_lock(&mtd_table_mutex);
#if 1
    len = sprintf(page, "Device name:%s\n", flash_name);
#else
    len = sprintf(page, "Device name:%s ID:0x%X\n",
        /* FIH, Debbie Sun, 2010/05/24 { */
        //supported_flash_table[supported_flash_index].device_name,
        flash_name,
        /* FIH, Debbie Sun, 2010/05/24 } */
        supported_flash_table[supported_flash_index].flash_id);
#endif

                if (len+begin > off+count)
                        goto done;
                if (len+begin < off) {
                        begin += len;
                        len = 0;
                }

        *eof = 1;

done:
//	mutex_unlock(&mtd_table_mutex);
        if (off >= len+begin)
                return 0;
        *start = page + (off-begin);
        return ((count < begin+len-off) ? count : begin+len-off);
}
//paul

void fih_msm_nand_init(void)
{
	if ((proc_nandinfo = create_proc_entry( "nandinfo", 0, NULL )))
		proc_nandinfo->read_proc = msm_nand_read_proc;
}
