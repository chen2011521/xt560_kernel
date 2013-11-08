#ifndef _FIH_TRAPS_H_
#define _FIH_TRAPS_H_

#include <linux/interrupt.h>
#include <linux/printk.h>

static inline bool ck_tasklet_func(struct tasklet_struct *t)
{
	if (t->func) return true;
	printk(KERN_ERR "[!] will causes Fatal exception in interrupt @ tasklet_action/tasklet_hi_action!!!\n");
	printk(KERN_ERR "[!] tasklet_struct[%08x] %08x %08lx %08x %08x %08lx\n",
		(unsigned int)t, (unsigned int)t->next, t->state, atomic_read(&t->count), (unsigned int)t->func, t->data);
	dump_stack();
	return false;
}

#endif	// _FIH_TRAPS_H_
