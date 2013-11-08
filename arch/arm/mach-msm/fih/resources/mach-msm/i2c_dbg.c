#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <mach/msm_iomap.h>
#include <mach/system.h>

#include <linux/gpio.h>
#include <linux/proc_fs.h>

static int i2c_dbg_read_proc
	(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int i2c_stat = 0, o_i2c_stat = 0, i, j;
	for (i = 0, j = 0; (i < 100) && (j < 3); i++) {
		i2c_stat = 	gpio_get_value(60) +			/* I2C0_SCL */
					gpio_get_value(61) * 10 +		/* I2C0_SDA */
					gpio_get_value(131) * 100 +		/* I2C1_SCL */
					gpio_get_value(132) * 1000;		/* I2C1_SDA */
		if (i2c_stat == o_i2c_stat) j++;
		else {j = 0; o_i2c_stat = i2c_stat;}
		printk(KERN_INFO "[i2c] %d\n", i2c_stat);
		mdelay(1);
	}
	*eof = 1;
	return snprintf(page, count, "%d%s\n",
					(i < 100) ? i2c_stat : 1111,
					(i < 100) ? " " : "U");
}

static int i2c_dbg_write_proc(struct file *file, const char __user *buffer,
	unsigned long count, void *data)
{
	return count;
}

static int __init i2c_dbg_init(void)
{
	struct proc_dir_entry *d_entry;

	d_entry = create_proc_entry("i2c", S_IRUGO, NULL);
	if (d_entry) {
		d_entry->read_proc = i2c_dbg_read_proc;
		d_entry->write_proc = i2c_dbg_write_proc;
		d_entry->data = NULL;
	}

	return 0;
}

static void __exit i2c_dbg_exit(void)
{
	remove_proc_entry("i2c", NULL);
}

module_init(i2c_dbg_init);
module_exit(i2c_dbg_exit);

MODULE_DESCRIPTION("PROC_SMDM driver");

