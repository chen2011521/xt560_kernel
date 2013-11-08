#include <linux/proc_fs.h>
#include <linux/module.h>
#include <linux/fih_hw_info.h> //Div2-SW2-BSP, JOE HSU
#include "smd_private.h"       //Div2-SW2-BSP, JOE HSU
//FIH, KanyYJLin, debugmask, 2011/09/22 +++
/* Add dbgmask partition*/
#include <asm/uaccess.h>
#include <mach/msm_iomap.h>
#include <linux/io.h>
#define DBGMSKBUF	MSM_DEBUGMASK_BASE
char * debug_mask = (char *) DBGMSKBUF;
static struct proc_dir_entry *mask_file;
static struct proc_dir_entry *power_off_charger_alarm_file;
#define	MASKSIZE   	4096
//KanyYJLin, debugmask, 2011/09/22 ---

#define PHONESTATUSBUFLEN 32
char phone_status[32] = {0};
static struct proc_dir_entry *phone_status_file;

#include "proc_comm.h"

static int proc_read_phone_status(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	return sprintf(page, "%s\n", phone_status);
}


static int proc_write_phone_status(struct file *file, const char *buffer,
				   unsigned long count, void *data)
{
	if(count > PHONESTATUSBUFLEN)
		return -EINVAL;

	if(!buffer)
		return -EINVAL;
	
	if(copy_from_user(phone_status, buffer, count))
		return -EFAULT;
	
	return count;
}
static int proc_calc_metrics(char *page, char **start, off_t off,
				 int count, int *eof, int len)
{
	if (len <= off+count) *eof = 1;
	*start = page + off;
	len -= off;
	if (len>count) len = count;
	if (len<0) len = 0;
	return len;
}

static int band_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len;
	int pi = fih_get_band_id();
	char ver[40];

  switch (pi)
  {
    case GSM_900_1800_CDMA_BC0:
      strcpy( ver, "GSM_BAND_23, CDMA_BAND_0\n");
      break;
    case CDMA_BC0:
      strcpy( ver, "CDMA_BAND_0\n");
      break;
    case CDMA_BC01F:
      strcpy( ver, "CDMA_BAND_01F\n");
      break;
    case CDMA_BC01:
      strcpy( ver, "CDMA_BAND_01\n");
      break;
    case CDMA_BC1:
      strcpy( ver, "CDMA_BAND_1\n");
      break;
    case WCDMA_145_band:
      strcpy( ver, "GSM_BAND_1234, WCDMA_BAND_145\n");
      break;
    case WCDMA_148_band:
      strcpy( ver, "GSM_BAND_1234, WCDMA_BAND_148\n");
      break;
    case WCDMA_245_band:
      strcpy( ver, "GSM_BAND_1234, WCDMA_BAND_245\n");
      break;
    case WCDMA_15_band:
      strcpy( ver, "GSM_BAND_1234, WCDMA_BAND_15\n");
      break;
    case WCDMA_18_band:
      strcpy( ver, "GSM_BAND_1234, WCDMA_BAND_18\n");
      break;
    case WCDMA_25_band:
      strcpy( ver, "GSM_BAND_1234, WCDMA_BAND_25\n");
      break;
    case WCDMA_125_band:
      strcpy( ver, "GSM_BAND_1234, WCDMA_BAND_125\n");
      break;
    case WCDMA_128_band:
      strcpy( ver, "GSM_BAND_1234, WCDMA_BAND_128\n");
      break;
    case WCDMA_14_band:
      strcpy( ver, "GSM_BAND_1234, WCDMA_BAND_14\n");
      break;
    default:
      strcpy( ver, "Unkonwn RF band id\n");
      break;
    }
	len = snprintf(page, PAGE_SIZE, "%s\n",
		ver);
		
	return proc_calc_metrics(page, start, off, count, eof, len);
}

static int device_model_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len;
	int pi = fih_get_product_id();
	char ver[24];
		
	switch (pi){
		case Project_IRM:
			strcpy(ver, "IRM");
			break;
		case Project_IRE:
			strcpy(ver, "IRE");
			break;
		case Project_IMN:
			strcpy(ver, "IMN");
			break;
		case Project_ITV:
			strcpy(ver, "ITV");
			break;
		case Project_IT2:
			strcpy(ver, "IT2");
			break;
		case Project_TBP:
			strcpy(ver, "TBP");
			break;
		case Project_TBE:
			strcpy(ver, "TBE");
			break;
		case Project_TNQ:			/*Modefied By SW4-BSP,Jiahao,2011-10-13*/
			strcpy(ver, "TNQ");		/*Modefied By SW4-BSP,Jiahao,2011-10-13*/                 
			break;                          
		case Project_TQ2:
			strcpy(ver, "TQ2");
			break;
		case Project_IRQ:
			strcpy(ver, "IRQ");
			break;
		case Project_NPM:
			strcpy(ver, "NPM");
			break;
		case Project_IMP:
			strcpy(ver, "IMP");
			break;
		case Project_IPD:
			strcpy(ver, "IPD");
			break;
		case Project_TPP:
			strcpy(ver, "TPP");
			break;
		default:
			strcpy(ver, "Unkonwn Device Model");
			break;
	}

	len = snprintf(page, PAGE_SIZE, "%s\n",
		ver);
		
	return proc_calc_metrics(page, start, off, count, eof, len);	
}

static int baseband_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len;
	int pp = fih_get_product_phase();
	char ver[24];
	
	switch (pp){
  case Phase_PR0:
   strcpy(ver, "PR0");
	 break;		
  case Phase_PR1:
   strcpy(ver, "PR1");
	 break;
  case Phase_PR2:
   strcpy(ver, "PR2");
	 break;
  case Phase_PR3:
   strcpy(ver, "PR3");
	 break;
  case Phase_PR4:
   strcpy(ver, "PR4");
	 break;
  case Phase_PR5:
   strcpy(ver, "PR5");
	 break;
  case Phase_PR6:
   strcpy(ver, "PR6");
	 break;
  case Phase_MP:
   strcpy(ver, "MP");
	 break;
  case Phase_PR1p5:
	strcpy(ver, "PR1.5");
	 break;
  case Phase_PR2p5:
	strcpy(ver, "PR2.5");
	 break;
  case Phase_PR3p5:
	strcpy(ver, "PR3.5");
	 break;
  case Phase_PCR:
	strcpy(ver, "PCR");
	 break;
  default:
	strcpy(ver, "Unkonwn Baseband version");
	 break;
	}

	len = snprintf(page, PAGE_SIZE, "%s\n",
		ver);

	return proc_calc_metrics(page, start, off, count, eof, len);
}

static int oemrev_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len;
  char *pe;
	
  pe = fih_get_version();

	len = snprintf(page, PAGE_SIZE, "%s\n",
		pe);
		
	return proc_calc_metrics(page, start, off, count, eof, len);	
}

//Div2-SW2-BSP, HenryMCWang, get power on cause, +++
static int poweroncause_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len;
	char ver[24];
		
	sprintf(ver, "0x%x", fih_get_poweroncause_info());
	
	len = snprintf(page, PAGE_SIZE, "%s\n",
		ver);
		
	return proc_calc_metrics(page, start, off, count, eof, len);	
}
//Div2-SW2-BSP, HenryMCWang, get power on cause, +++
/* FIH, KanyYJLin, debugmask, 2011/09/22 { */
/* Add dbgmask partition*/
//#ifdef CONFIG_PRINTK

/*Added By SW4-BSP,Jiahao,2011-10-13*/
static int HW_band_info_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len;
    int pj = fih_get_band_id();
    char ver[40];

    switch (pj)
  {
    case GSM_900_1800_CDMA_BC0:
      strcpy( ver, "GSM900/1800, CDMA850\n");
      break;
    case CDMA_BC0:
      strcpy( ver, "CDMA850\n");
      break;
    case CDMA_BC01F:
        strcpy( ver, "CDMA850/1900/1700\n");
        break;
    case CDMA_BC01:
        strcpy( ver, "CDMA850/1900\n");
        break;
	case CDMA_BC1:
		strcpy( ver, "CDMA1900\n");
		break;
    case WCDMA_145_band:
      strcpy( ver, "WCDMA2100/1700/850\n");
      break;
    case WCDMA_148_band:
      strcpy( ver, "WCDMA2100/1700/900\n");
      break;
    case WCDMA_245_band:
      strcpy( ver, "WCDMA1900/1700/850\n");
      break;
    case WCDMA_15_band:
      strcpy( ver, "WCDMA2100/850\n");
      break;
    case WCDMA_18_band:
      strcpy( ver, "WCDMA2100/900\n");
      break;
    case WCDMA_25_band:
      strcpy( ver, "WCDMA1900/850\n");
      break;
    case WCDMA_125_band:
      strcpy( ver, "WCDMA2100/1900/850\n");
      break;
    case WCDMA_128_band:
      strcpy( ver, "WCDMA2100/1900/900\n");
      break;
    case WCDMA_14_band:
      strcpy( ver, "WCDMA2100/1700\n");
      break;
    default:
      strcpy( ver, "Unkonwn RF band id\n");
      break;
    }

	len = snprintf(page, PAGE_SIZE, "%s\n",
		ver);
    
	return proc_calc_metrics(page, start, off, count, eof, len);
}

static int proc_read_debug_mask(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{

	int i,j;
	
	for (i=0,j=0; i<MASKSIZE; i++,j=j+2){
		sprintf((page+j),"%02x",debug_mask[i]);
		printk("debug_mask = %02x/n",debug_mask[i]);
	}
	return MASKSIZE;
}
static int proc_write_debug_mask(struct file *file, const char *buffer, 
				 unsigned long count, void *data)
{
	//char mask[16];
	/*FIHTDC, WillChen add for debug mask { */
	printk("proc_write_debug_mask()\n");
	//if ( copy_from_user(debug_mask,buffer,count))
	//	return -EFAULT;
	if (buffer)
		memcpy(debug_mask,buffer,count);

	return count;
	/*FIHTDC, WillChen add for debug mask } */
}
//#endif
/* FIH, KanyYJLin, debugmask, 2011/09/22 } */

/* FIH, Square, 2011/11/03 { */
static int proc_read_product_id(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int len = 0;	
	int iRetVal = 0;
	unsigned int product_id[32];
	
	memset(product_id, 0, 32 * sizeof(unsigned int));
	
	iRetVal = proc_comm_read_product_id(product_id);

    if(iRetVal == 0)
    {
	    len = snprintf(page, PAGE_SIZE, "%s\n", (char *)product_id);
	}

	return proc_calc_metrics(page, start, off, count, eof, len);	
}

static int proc_read_track_id(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int len = 0;
	unsigned int track1_id[32];
	unsigned int track2_id[32];
	unsigned int track3_id[32];
	unsigned int track4_id[32];

	memset(track1_id, 0, 32 * sizeof(unsigned int));
	memset(track2_id, 0, 32 * sizeof(unsigned int));
	memset(track3_id, 0, 32 * sizeof(unsigned int));
	memset(track4_id, 0, 32 * sizeof(unsigned int));

	proc_comm_ftm_customer_pid_read(track1_id);
	proc_comm_ftm_customer_pid2_read(track2_id);
	proc_comm_ftm_customer_swid_read(track3_id);
	proc_comm_ftm_dom_read(track4_id);

    len = snprintf(page, PAGE_SIZE, "%s, %s, %s, %s\n", (char *)track1_id, (char *)track2_id, (char *)track3_id, (char *)track4_id);	

	return proc_calc_metrics(page, start, off, count, eof, len);  
}
/* FIH, Square, 2011/11/03 } */

/* FIH, Square, 2011/11/22 { */
unsigned int rtc_time_to_seconds(pm_rtc_julian_type  *rtc_time)
{
    return mktime(rtc_time->year, rtc_time->month, rtc_time->day, rtc_time->hour, rtc_time->minute, rtc_time->second);
}
#define APPS_REBOOT_RTC_ALARM                0x22694020
static int proc_comm_set_alarm_time(struct file *file, const char *buffer, 
				 unsigned long count, void *data)
{
	int   reset_chip_reason;
	reset_chip_reason=APPS_REBOOT_RTC_ALARM;
			/* Send a power on cause to notify Modem*/
			proc_comm_reset_chip(&reset_chip_reason);

	return count;
}
static int proc_comm_get_alarm_time(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int len = 0;
   	unsigned smem_response;
	uint32_t oem_cmd;
    pm_rtc_julian_type  alarm_time;
    pm_rtc_julian_type  current_time;
	/* cmd_parameter setting */
    unsigned int alarm_seconds;
    unsigned int current_secs;
    
    ///////////////////////////////////////////////////////////////////////////////
    //oem_cmd = SMEM_PROC_COMM_OEM_SET_RTC_ALARM;
    
    //memset (&alarm_time, 0, sizeof(alarm_time) );
    //msm_proc_comm_oem(PCOM_CUSTOMER_CMD1, &oem_cmd, &smem_response, (unsigned *)&alarm_time );
#if 1
    oem_cmd = SMEM_PROC_COMM_OEM_GET_RTC_ALARM;
    msm_proc_comm_oem(PCOM_CUSTOMER_CMD1, &oem_cmd, &smem_response, (unsigned *)&alarm_time );
    
    //printk("alarm: %d-%d-%d %d:%d:%d\n", 
    //alarm_time.year, alarm_time.month, alarm_time.day, alarm_time.hour, alarm_time.minute, alarm_time.second);

    alarm_seconds = rtc_time_to_seconds(&alarm_time);
#endif
    ///////////////////////////////////////////////////////////////////////////////
    
    
    oem_cmd = SMEM_PROC_COMM_OEM_GET_SYSTEM_TIME;
    
    msm_proc_comm_oem(PCOM_CUSTOMER_CMD1, &oem_cmd, &smem_response, (unsigned *)&current_time );
    //printk("current: %d-%d-%d %d:%d:%d\n", 
    //current_time.year, current_time.month, current_time.day, current_time.hour, current_time.minute, current_time.second);
    current_secs = rtc_time_to_seconds(&current_time);
    len = snprintf(page, PAGE_SIZE, "%d:%d\n", alarm_seconds, current_secs);	
        
    
    return proc_calc_metrics(page, start, off, count, eof, len);  
}
/* FIH, Square, 2011/11/22 } */

static struct {
		char *name;
		int (*read_proc)(char*,char**,off_t,int,int*,void*);
} *p, fxx_info[] = {
	{"devmodel",	device_model_read_proc},
	{"baseband",	baseband_read_proc},
	{"bandinfo",	band_read_proc},
	{"oemrev",	oemrev_proc},
	//Div2-SW2-BSP, HenryMCWang, get power on cause, +++
	{"poweroncause",	poweroncause_read_proc},
	//Div2-SW2-BSP, HenryMCWang, get power on cause, +++
    {"HWinfo",  HW_band_info_read_proc},
    //Added By SW4-BSP,Jiahao,2011-10-13
	/* FIH, Square, 2011/11/03 { */
	{"productid", proc_read_product_id},
	{"trackid", proc_read_track_id},
	/* FIH, Square, 2011/11/03 } */
	{NULL,},
};

void fxx_info_init(void)
{	
	for (p = fxx_info; p->name; p++)
		create_proc_read_entry(p->name, 0, NULL, p->read_proc, NULL);
		
	//FIH, KanyYJLin, debugmask, 2011/09/22 +++
	/* Add dbgmask partition*/
	//#ifdef CONFIG_PRINTK
	mask_file = create_proc_entry("debug_mask",0664,NULL);
	mask_file->read_proc = proc_read_debug_mask;
	mask_file->write_proc = proc_write_debug_mask;
	power_off_charger_alarm_file = create_proc_entry("battery_charger_alarm",0600,NULL);
	power_off_charger_alarm_file->read_proc = proc_comm_get_alarm_time;
	power_off_charger_alarm_file->write_proc = proc_comm_set_alarm_time;
	/* FIH, JiaHao, 2010/08/20 { */
	/* ./android/kernel/include/linux/proc_fs.h no define owner at 6030cs */
	//mask_file->owner = THIS_MODULE;
	/* FIH, JiaHao, 2010/08/20 } */
	//#endif
	//FIH, KanyYJLin, debugmask, 2011/09/22 ---
	//FIH, KanyYJLin, debugmask, 2011/09/22 ---
	//FIH-NJDC-BSP, Leok detect phone status, 2012/02/05
	phone_status_file = create_proc_entry("phone_status", 0777, NULL);
	phone_status_file->read_proc = proc_read_phone_status;
	phone_status_file->write_proc = proc_write_phone_status;
	//FIH-NJDC-BSP, Leok detect phone status, 2012/02/05
}
EXPORT_SYMBOL(fxx_info_init);

void fxx_info_remove(void)
{
	for (p = fxx_info; p->name; p++)
		remove_proc_entry(p->name, NULL);
}
EXPORT_SYMBOL(fxx_info_remove);
