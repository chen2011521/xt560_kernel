#ifndef _PMDBG_H_
#define _PMDBG_H_

/*
 * suspend workqueue hanged tracker
 */
#include <linux/timer.h>

#define SWQ_TRACE_TIMER
#define SWQ_TRACE_POLLING_MS 55000

extern struct task_struct *gts_suspend;
extern struct timer_list g_suspend_trace_timer;
extern int g_en_swq_timer;
extern void swq_trace_timeout(unsigned long data);

static inline void InitTraceTimer(void) {setup_timer(&g_suspend_trace_timer, swq_trace_timeout, 0);}
static inline void InsertTraceTimer(void) {g_en_swq_timer = 1; mod_timer(&g_suspend_trace_timer, jiffies + msecs_to_jiffies(SWQ_TRACE_POLLING_MS));}
static inline void RemoveTraceTimer(void) {g_en_swq_timer = 0;}
#ifdef SWQ_TRACE_TIMER
static inline bool isInSwqTrace(void) {return (g_en_swq_timer == 1);}
#else	// SWQ_TRACE_TIMER
static inline bool isInSwqTrace(void) {return true;}
#endif	// SWQ_TRACE_TIMER

#define FIH_SWQ_TRACE_INIT_INSTACNCE \
struct timer_list g_suspend_trace_timer; \
int g_en_swq_timer = 0; \
void swq_trace_timeout(unsigned long data) \
{if (g_en_swq_timer) {printk(KERN_INFO "[PM] suspend work queue stack"); \
if (gts_suspend) {sched_show_task(gts_suspend);} \
mod_timer(&g_suspend_trace_timer, jiffies + msecs_to_jiffies(SWQ_TRACE_POLLING_MS));}}


/*
 * suspend state debug log mechanism
 */
//#define TRACE_CLK_DBGN ""		/* dbg_name of clk */

extern int g_fpmdbg;

/* kernel\arch\arm\mach-msm\gpio.c */
#define DB_GPIO_STATE	0x01	/*   1 */
extern void gpio_debug_print_state(void);

/* kernel\arch\arm\mach-msm\clock-debug.c */
#define DB_CLK_STATE	0x02	/*   2 */

/* kernel\arch\arm\mach-msm\vreg.c */
#define DB_VREG_CNT		0x04	/*   4 */
#define DB_VREG_MMV		0x08	/*   8 */
#define DB_VREG_STA		0x10	/*  16 */
extern void vreg_debug_print_state(void);

#define FPMDBG_INIT_VALUE	(DB_GPIO_STATE)

#endif	// _PMDBG_H_
