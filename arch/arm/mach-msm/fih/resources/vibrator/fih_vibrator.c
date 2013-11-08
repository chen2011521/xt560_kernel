/**
* Vibrator driver
**************************************/
#include "fih_vibrator.h"

//FIHTDC, Modify for vibration can not stop issue, MayLi, 2011.09.16 {++
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include "../../../../../../drivers/staging/android/timed_output.h"
#include <linux/sched.h>
#include "../../../pmic.h"
#include <mach/msm_rpcrouter.h>
/* FIH, KennyChu, 2009/07/23, solve vibration can't be stopped issue{*/
#include <linux/wakelock.h>
#include <linux/completion.h>
/* } FIH, KennyChu, 2009/07/23 */


#define PM_LIBPROG      0x30000061
#if (CONFIG_MSM_AMSS_VERSION == 6220) || (CONFIG_MSM_AMSS_VERSION == 6225)
#define PM_LIBVERS      0xfb837d0b
#else
#define PM_LIBVERS      0x10001
#endif

#define HTC_PROCEDURE_SET_VIB_ON_OFF	21
#define PMIC_VIBRATOR_LEVEL_1	(3000)     
#define PMIC_VIBRATOR_LEVEL_2	(2700)             //change the Voltage to 3.7v   NJDC-hjj-2012/3/21

static int VIB_LEVEL         = 0;     
static struct work_struct work_vibrator_on;
static struct work_struct work_vibrator_off;
static struct hrtimer vibe_timer;

/* FIH, KennyChu, 2009/07/23, solve vibration can't be stopped issue{ */
static struct wake_lock vibrator_suspend_wake_lock;
static pid_t thread_id;
DEFINE_MUTEX(vibrator_lock);
DECLARE_COMPLETION(vibrator_comp);
/* } FIH, KennyChu, 2009/07/23 */


#if 1
static void set_pmic_vibrator(int on)
{
	int rc;
	printk(KERN_ERR "%s[%d] +\n", __func__, on);

	rc = pmic_vib_mot_set_mode(PM_VIB_MOT_MODE__MANUAL);
	if (rc) {
		pr_err("%s: Vibrator set mode failed", __func__);
		return;
	}

    if (on)
    {
        if (!VIB_LEVEL)
            rc = pmic_vib_mot_set_volt(PMIC_VIBRATOR_LEVEL_1);
        else 
            rc = pmic_vib_mot_set_volt(PMIC_VIBRATOR_LEVEL_2);            
    }
    else
        rc = pmic_vib_mot_set_volt(0);

	if (rc)
		pr_err("%s: Vibrator set voltage level failed", __func__);
	printk(KERN_ERR "%s -\n", __func__);
}
#else
static void set_pmic_vibrator(int on)
{
	static struct msm_rpc_endpoint *vib_endpoint;
	struct set_vib_on_off_req {
		struct rpc_request_hdr hdr;
		uint32_t data;
	} req;
	printk(KERN_ERR "%s +\n", __func__);
	if (!vib_endpoint) {
		vib_endpoint = msm_rpc_connect(PM_LIBPROG, PM_LIBVERS, 0);
		if (IS_ERR(vib_endpoint)) {
			printk(KERN_ERR "%s, init vib rpc failed!\n", __func__);
			vib_endpoint = 0;
			return;
		}
	}


	if (on)
		req.data = cpu_to_be32(PMIC_VIBRATOR_LEVEL);
	else
		req.data = cpu_to_be32(0);

	msm_rpc_call(vib_endpoint, HTC_PROCEDURE_SET_VIB_ON_OFF, &req,
		sizeof(req), 5 * HZ);
	printk(KERN_ERR "%s -\n", __func__);
}
#endif

static void pmic_vibrator_on(struct work_struct *work)
{
	set_pmic_vibrator(1);
}

static void pmic_vibrator_off(struct work_struct *work)
{
	set_pmic_vibrator(0);
}

/* FIH, KennyChu, 2009/07/23, solve vibration can't be stopped issue{ */
static int pmic_vibrator_off_thread(void *arg)
{
    printk(KERN_ERR "%s, vib_off_thread running\n", __func__);

    daemonize("vib_off_thread");
    //allow_signal(SIGKILL);

    while (1) {        
        wait_for_completion(&vibrator_comp);
        printk(KERN_ERR "%s, Got complete signal\n", __func__);

        wake_lock(&vibrator_suspend_wake_lock);

        mutex_lock(&vibrator_lock);
        set_pmic_vibrator(0);
        mutex_unlock(&vibrator_lock);

        wake_unlock(&vibrator_suspend_wake_lock);

    }
	
    return 0;
}
/* } FIH, KennyChu, 2009/07/23 */

/* FIH, KennyChu, 2009/07/23, solve vibration can't be stopped issue{ */
#if 0
static void timed_vibrator_on(struct timed_output_dev *sdev)
{
	schedule_work(&work_vibrator_on);
}

static void timed_vibrator_off(struct timed_output_dev *sdev)
{
	schedule_work(&work_vibrator_off);
}
#endif
/* } FIH, KennyChu, 2009/07/23 */

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
/* FIH, KennyChu, 2009/07/23, solve vibration can't be stopped issue{ */
#if 1
	printk(KERN_ERR "%s, period=%d +\n", __func__, value);
	wake_lock(&vibrator_suspend_wake_lock);

	hrtimer_cancel(&vibe_timer);
	
	printk(KERN_ERR "%s, TIMER canceled\n", __func__);
	printk(KERN_ERR "%s: Vibration period %d ms\n", __func__, value);
	value = (value > 15000 ? 15000 : value);

	if (value != 0) {
		printk(KERN_ERR "%s, execute pmic_vibrator_on\n", __func__);

		// force to vibrate at least 20 ms
		if (0 < value && 20 > value){
			value = 20;
		}

		mutex_lock(&vibrator_lock);

		set_pmic_vibrator(1);

		hrtimer_start(&vibe_timer,
		ktime_set(value / 1000, (value % 1000) * 1000000), HRTIMER_MODE_REL);
		mutex_unlock(&vibrator_lock);

		if (hrtimer_active(&vibe_timer)){
			printk(KERN_ERR "%s, TIMER running\n", __func__);
		}

	}
	else	{
		printk(KERN_ERR "stop vibrator directly\n");
		mutex_lock(&vibrator_lock);
		set_pmic_vibrator(0);
		mutex_unlock(&vibrator_lock);

		wake_unlock(&vibrator_suspend_wake_lock);
	}
	printk(KERN_ERR "%s, period=%d -\n", __func__, value);
#else
    hrtimer_cancel(&vibe_timer);
	if (value == 0)
		timed_vibrator_off(dev);
	else {
		value = (value > 15000 ? 15000 : value);

		timed_vibrator_on(dev);

		hrtimer_start(&vibe_timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}
#endif
/* } FIH, KennyChu, 2009/07/023 */
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&vibe_timer)) {
		ktime_t r = hrtimer_get_remaining(&vibe_timer);
		printk(KERN_ERR "%s time=%d\n", __func__, (r.tv.sec * 1000 + r.tv.nsec / 1000000));
		return r.tv.sec * 1000 + r.tv.nsec / 1000000;
	} else
		return 0;
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	/* FIH, KennyChu, 2009/07/23, solve vibration can't be stopped issue{ */
	wake_unlock(&vibrator_suspend_wake_lock);
	complete(&vibrator_comp);
	/* } FIH, KennyChu, 2009/07/23 */

	return HRTIMER_NORESTART;
}

static struct timed_output_dev pmic_vibrator = {
	.name = "vibrator",
	.get_time = vibrator_get_time,
	.enable = vibrator_enable,
};

int msm_init_pmic_vibrator_Default(void* hdl)
{
	INIT_WORK(&work_vibrator_on, pmic_vibrator_on);
	INIT_WORK(&work_vibrator_off, pmic_vibrator_off);

	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer.function = vibrator_timer_func;

	timed_output_dev_register(&pmic_vibrator);
	
	/* FIH, KennyChu, 2009/07/23, solve vibration can't be stopped issue{ */
	wake_lock_init(&vibrator_suspend_wake_lock, WAKE_LOCK_SUSPEND, "vibrator_suspend_work");
	thread_id = kernel_thread(pmic_vibrator_off_thread, NULL, CLONE_FS | CLONE_FILES);

    return 0;
	/* } FIH, KennyChu, 2009/07/23 */
}

int msm_init_pmic_vibrator_v1(void* hdl)
{
    VIB_LEVEL         = 1;
	INIT_WORK(&work_vibrator_on, pmic_vibrator_on);
	INIT_WORK(&work_vibrator_off, pmic_vibrator_off);

	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer.function = vibrator_timer_func;

	timed_output_dev_register(&pmic_vibrator);
	
	/* FIH, KennyChu, 2009/07/23, solve vibration can't be stopped issue{ */
	wake_lock_init(&vibrator_suspend_wake_lock, WAKE_LOCK_SUSPEND, "vibrator_suspend_work");
	thread_id = kernel_thread(pmic_vibrator_off_thread, NULL, CLONE_FS | CLONE_FILES);

    return 0;
	/* } FIH, KennyChu, 2009/07/23 */
}

MODULE_DESCRIPTION("timed output pmic vibrator device");
MODULE_LICENSE("GPL");
//FIHTDC, Modify for vibration can not stop issue, MayLi, 2011.09.16 --}

int __init vibrator_init(void)
{
	printk(KERN_INFO"[%s] entry\n", __func__);

    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_VIB, 
                                     VIB_Default), 
                   "VIB_Default", msm_init_pmic_vibrator_Default, __FILE__, __LINE__);

    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_VIB, 
                                     VIB_V1), 
                   "VIB_V1", msm_init_pmic_vibrator_v1, __FILE__, __LINE__);
	
	return 0;
}
