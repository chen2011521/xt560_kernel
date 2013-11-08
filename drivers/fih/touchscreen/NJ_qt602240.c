#include <linux/module.h>
#include <linux/string.h>
#include <linux/fih_hw_info.h>


//#define CONFIG_FIH_FTM
//#define DEBUG
#ifdef DEBUG
#define TP_MSG(arg, ...) printk(format, arg, ## __VA_ARGS__)
#else
#define TP_MSG(arg, ...) 

#endif
static char test_result[20] = {0,};
static struct timer_list tp_timer;
static struct work_struct Timer_wqueue;
#ifdef CONFIG_FIH_FTM
static  uint8_t selftest_count = 0; 
#endif


static int atoi(const char *name)
{
	int val = 0;

	for (;; name++) {
		switch (*name) {
		case '0' ... '9':
			val = 10*val+(*name-'0');
			break;
		default:
			return val;
		}
	}
}


static void Timer_workqueue(struct work_struct *work)
{
#if 1
#ifdef WATER    
            int power_cfg_address;
            uint16_t Face_suppression_address = 0;

#endif
        power_cfg_address = g_RegAddr[GEN_ACQUISITIONCONFIG_T8];    
        

    
        switch(Project_Flag){
                case 1://TBP
                    Face_suppression_address=g_RegAddr[PROCI_GRIPFACESUPPRESSION_T20];
                    write_mem(power_cfg_address, sizeof(v10_T8), (void *)v10_T8 );
                    write_mem(Face_suppression_address, sizeof(v10_T20),(void *)v10_T20);
                    break;
                case 2://TNQ
                    Face_suppression_address=g_RegAddr[SPARE_T42];
                    write_mem(power_cfg_address, sizeof(v10_var13_T8), (void *)v10_var13_T8 );
                    write_mem(Face_suppression_address, sizeof(v10_var13_T42),(void *)v10_var13_T42);
                    break;
                case 3://ITV
                    Face_suppression_address=g_RegAddr[SPARE_T42];
                    printk("kami_write\n");
                    write_mem(power_cfg_address, sizeof(v10_var01_T8), (void *)v10_var01_T8 );
                    write_mem(Face_suppression_address, sizeof(v10_var01_T42),(void *)v10_var01_T42);
                    break;        
            }        
#endif
}


static void tp_timer_func(unsigned long arg)
{
    schedule_work(&Timer_wqueue);

    TP_MSG("tp_timer_func\n");
    
}




static void NPM_REG(int index_num, char ** obj)
{
    int i;
    
    switch (index_num)
    {
            case 7:
                    for (i=0; i<sizeof(v10_var01_T7_V2); i++)
                    v10_var01_T7_V2[i] = atoi(strsep(obj, ","));
                    break;              
            case 8:
                    for (i=0; i<sizeof(v10_var01_T8_V2); i++)
                    v10_var01_T8_V2[i] = atoi(strsep(obj, ","));
                    break;
            case 9:
                    for (i=0; i<sizeof(v10_var01_T9_V2); i++)
                    v10_var01_T9_V2[i] = atoi(strsep(obj, ","));
                    break;
            case 15:
                    for (i=0; i<sizeof(v10_var01_T15_V2); i++)
                    v10_var01_T15_V2[i] = atoi(strsep(obj, ","));
                    break;
            case 18:
                    for (i=0; i<sizeof(v10_var01_T18_V2); i++)
                    v10_var01_T18_V2[i] = atoi(strsep(obj, ","));
                    break;
            case 19:
                    for (i=0; i<sizeof(v10_var01_T19_V2); i++)
                    v10_var01_T19_V2[i] = atoi(strsep(obj, ","));
                    break;
            case 23:
                    for (i=0; i<sizeof(v10_var01_T23_V2); i++)
                    v10_var01_T23_V2[i] = atoi(strsep(obj, ","));
                    break;
            case 25:
                    for (i=0; i<sizeof(v10_var01_T25_V2); i++)
                    v10_var01_T25_V2[i] = atoi(strsep(obj, ","));
                    break;
            case 40:
                    for (i=0; i<sizeof(v10_var01_T40_V2); i++)
                    v10_var01_T40_V2[i] = atoi(strsep(obj, ","));
                    break;
            case 42:
                    for (i=0; i<sizeof(v10_var01_T42_V2); i++)
                    v10_var01_T42_V2[i] = atoi(strsep(obj, ","));
                    break;
            case 46:
                    for (i=0; i<sizeof(v10_var01_T46_V2); i++)
                    v10_var01_T46_V2[i] = atoi(strsep(obj, ","));
                    break;
            case 47:
                    for (i=0; i<sizeof(v10_var01_T47_V2); i++)
                    v10_var01_T47_V2[i] = atoi(strsep(obj, ","));
                    break;
            case 48:
                    for (i=0; i<sizeof(v10_var01_T48_V2); i++)
                    v10_var01_T48_V2[i] = atoi(strsep(obj, ","));
			break;    
			}

}


static void re_config_NPM(void)
{
    static struct file *filp = NULL;
	int c = 0;
	int i = 0;
	int bEOF = 0;
    int nRead = 0;
    char *buf;
    char *p = NULL;
    int reg_index = 0;

    mm_segment_t oldfs;

    oldfs = get_fs();
    set_fs(KERNEL_DS);
    
    filp = filp_open("/data/tp_cfg", O_RDONLY, 0);

    if (IS_ERR(filp))
    {
        filp = NULL;
        printk(KERN_ERR "Open tp_cfg error!\n");
        return;
    }

    p = kzalloc(1, GFP_KERNEL);
    buf = kzalloc(512, GFP_KERNEL);
    for (;;)
    {
    	c = 0;
    	memset(buf, 0 , 512);
    	// read 1 row
    	
    	do
    	{
    	        nRead = filp->f_op->read(filp, p, 1, &filp->f_pos);
    		if (nRead != 1)
    		{
			bEOF = 1;
			break;
		}

            if(*p == ' ')
            {   
                continue;
            }
            strcat(buf,p);
    	}while (*p != '\n');
    	// process 1 row
    	if (!bEOF)
    	{
        reg_index = atoi(strsep(&buf, "="));
        NPM_REG(reg_index, &buf);
    	}
     
    	else
    		break;        
    }

    kfree(p);
    kfree(buf);    
	// close ini file
    filp_close(filp, NULL);
    filp = NULL;
    set_fs(oldfs);


#if 1
    printk("v10_T7=");
    for (i=0; i<sizeof(v10_var01_T7_V2); i++)
		printk("%d,", v10_var01_T7_V2[i]);
	printk("\n");
    printk("v10_T8=");
    for (i=0; i<sizeof(v10_var01_T8_V2); i++)
		printk("%d,", v10_var01_T8_V2[i]);
	printk("\n");
    printk("v10_T9=");
    for (i=0; i<sizeof(v10_var01_T9_V2); i++)
		printk("%d,", v10_var01_T9_V2[i]);
	printk("\n");
    printk("v10_T15=");
    for (i=0; i<sizeof(v10_var01_T15_V2); i++)
		printk("%d,", v10_var01_T15_V2[i]);
	printk("\n");
    printk("v10_T18=");
    for (i=0; i<sizeof(v10_var01_T18_V2); i++)
		printk("%d,", v10_var01_T18_V2[i]);
	printk("\n");
    printk("v10_T19=");
    for (i=0; i<sizeof(v10_var01_T19_V2); i++)
		printk("%d,", v10_var01_T19_V2[i]);
	printk("\n");
    printk("v10_T23=");
    for (i=0; i<sizeof(v10_var01_T23_V2); i++)
		printk("%d,", v10_var01_T23_V2[i]);
	printk("\n");
    printk("v10_T25=");
    for (i=0; i<sizeof(v10_var01_T25_V2); i++)
		printk("%d,", v10_var01_T25_V2[i]);
	printk("\n");
    printk("v10_T40=");
    for (i=0; i<sizeof(v10_var01_T40_V2); i++)
		printk("%d,", v10_var01_T40_V2[i]);
	printk("\n");
    printk("v10_T42=");
    for (i=0; i<sizeof(v10_var01_T42_V2); i++)
		printk("%d,", v10_var01_T42_V2[i]);
	printk("\n");
    printk("v10_T46=");
    for (i=0; i<sizeof(v10_var01_T46_V2); i++)
		printk("%d,", v10_var01_T46_V2[i]);
	printk("\n");
    printk("v10_T47=");
    for (i=0; i<sizeof(v10_var01_T47_V2); i++)
		printk("%d,", v10_var01_T47_V2[i]);
	printk("\n");
    printk("v10_T48=");
    for (i=0; i<sizeof(v10_var01_T48_V2); i++)
		printk("%d,", v10_var01_T48_V2[i]);
	printk("\n");
#endif
    QT_InitConfig();
}

#if 0
static void QT_ConfigByGPIO(uint8_t arg)
{
	uint8_t vendor_ID =  (arg & 0x3C)>>2;

	uint8_t iLoop = 0, MaxOBJSize = 0;
	uint16_t addr;
	object_t *obj_index = NULL;
	MaxOBJSize = info_block->info_id->num_declared_objects;
	obj_index = info_block->objects;

        printk("vendor_ID:%d\n",vendor_ID);
        if(vendor_ID == 0x0D) {//SUCCESS GPIO IS 01 pull up read value is 1101
		uint8_t v10_T9_SUCCESS[]  = { 143, 0, 0, 16, 9, 0, 17, 45, 3, 3, 0, 1, 1, 14, 5, 10, 10, 10, 0x93, 0x3, 0xDF, 0x1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
             Vendor_Num = vendor_ID;
		for (iLoop=0; iLoop<MaxOBJSize; iLoop++)
		{
			TCH_MSG("Config: object_type = %d", obj_index->object_type);

			addr = obj_index->i2c_address;
			switch (obj_index->object_type)
			{
			case GEN_POWERCONFIG_T7 :
				write_mem(addr, sizeof(v10_T7), v10_T7);
				break;
			case GEN_ACQUISITIONCONFIG_T8 :
				write_mem(addr, sizeof(v10_T8_1), v10_T8_1);
				break;
			case TOUCH_MULTITOUCHSCREEN_T9 :
				write_mem(addr, sizeof(v10_T9_SUCCESS), v10_T9_SUCCESS);
				break;
			case TOUCH_KEYARRAY_T15 :
				write_mem(addr, sizeof(v10_T15), v10_T15);
				break;
			case SPT_COMCONFIG_T18 :
				write_mem(addr, sizeof(v10_T18), v10_T18);
				break;
			case SPT_GPIOPWM_T19 :
				write_mem(addr, sizeof(v10_T19), v10_T19);
				break;
			case PROCI_GRIPFACESUPPRESSION_T20 :
				write_mem(addr, sizeof(v10_T20), v10_T20);
				break;
			case PROCG_NOISESUPPRESSION_T22 :
				write_mem(addr, sizeof(v10_T22), v10_T22);
				break;
			case TOUCH_PROXIMITY_T23 :
				write_mem(addr, sizeof(v10_T23), v10_T23);
				break;
			case PROCI_ONETOUCHGESTUREPROCESSOR_T24 :
				write_mem(addr, sizeof(v10_T24), v10_T24);
				break;
			case SPT_SELFTEST_T25 :
				write_mem(addr, sizeof(v10_T25), v10_T25);
				break;
			case PROCI_TWOTOUCHGESTUREPROCESSOR_T27 :
				write_mem(addr, sizeof(v10_T27), v10_T27);
				break;
			case SPT_CTECONFIG_T28 :
				write_mem(addr, sizeof(v10_T28), v10_T28);
				break;
			case SPARE_T38 :
				//write_mem(addr, sizeof(v10_T38), v10_T38);
				break;
			default:
				TCH_MSG("Config: object_type %d not config data", obj_index->object_type);
				break;
			}
			obj_index++;
		}
	} else
	return;
}
#endif

//
void QT_read_reference(void)
{
        uint16_t addr;
        uint8_t data = 0x11;
        uint8_t T37_buf[130];
        uint8_t page = 0;
        uint8_t idx = 0;
        uint16_t val = 0;
        uint16_t try = 0;
        uint16_t low_level = 6500;
        uint16_t high_level = 11500;
        uint16_t x_count = 0, y_count = 0,total_count = 0, st_result_count = 0;
        uint16_t max_R = 0,min_R = 0;

        uint8_t v10_var01_T15[] = { 3, 0, 11, 18, 1, 0, 0, 55, 3, 0, 0 };
        uint8_t v10_var01_T15_with_gain[] = { 3, 0, 11, 18, 1, 0, 16, 55, 3, 0, 0 };
        

        switch(Project_Flag){
            case 1://TBP
                x_count = 14;
                y_count = 9;
                low_level = low_boundary;
                high_level = high_boundary;
                total_count = 224;
                break;
                
            case 2://TNQ
                x_count = 11;
                y_count = 8;
                low_level = low_boundary;
                high_level = high_boundary;        
                total_count = 88;                
                break;
                
            case 3://ITV
                x_count = 12;
                y_count = 11;
                low_level = low_boundary;
                high_level = high_boundary;        
                total_count = 216;   

                addr = g_RegAddr[TOUCH_KEYARRAY_T15];
                write_mem(addr, sizeof(v10_var01_T15), v10_var01_T15);
                mdelay(20);    
                
                break;
				
            case 4://IP
                	x_count = 12;
                	y_count = 11;
                	low_level = low_boundary;
                	high_level = high_boundary;		  
                	total_count = 216;								  
                	break;
				

        }


        if(!x_count || !y_count)
            return;    
    
            printk(KERN_INFO "[Touch] high_level=%d, low_level=%d\n", high_level, low_level);

        addr = g_RegAddr[GEN_COMMANDPROCESSOR_T6] + 5;
        write_mem(addr, 1, &data);

        msleep(50);

        for (page=0; page<4; page++)
        {
            read_mem(g_RegAddr[SPARE_T37], 2, T37_buf);
            while(!(T37_buf[0] == 0x11) )
            {
                if (try > 100)
                {
                    printk("faild");
                    break;
                }   
                msleep(5);
                try++;
                read_mem(g_RegAddr[SPARE_T37], 2, T37_buf);
            }
            memset(T37_buf, 0, sizeof(T37_buf));

            read_mem(g_RegAddr[SPARE_T37], sizeof(T37_buf), T37_buf);
//            printk(KERN_INFO "[Touch] mode = 0x%02x, page = %d\n",T37_buf[0], T37_buf[1]);

        	
        	for (idx=2; idx+1<sizeof(T37_buf); idx+=2)
        	{
        		val = T37_buf[idx+1];
        		val = T37_buf[idx] + (val << 8);
#if 1
                    if (qt_debug_level >= DB_LEVEL1) {
                            printk( "%d,",val);
                            if(!((page*64+(idx-2)/2+1)%x_count)  && idx != 2)
                                printk( "\n");    
                    }
#else
                    if (qt_debug_level >= DB_LEVEL1)
                    printk(KERN_INFO "[Touch] CH %d (X%d,Y%d) = %d",
                    		page*64+(idx-2)/2, (page*64+(idx-2)/2)/x_count,
                    		(page*64+(idx-2)/2)%x_count, val);
#endif


                    if (((page*64+(idx-2)/2)<total_count &&
                        ((((page*64+(idx-2)/2)%x_count) < y_count)))||(Project_Flag == 3 &&
                        (((page*64+(idx-2)/2) == 11) ||
                        ((page*64+(idx-2)/2) == 23) ||
                        ((page*64+(idx-2)/2) == 119) ||
                        ((page*64+(idx-2)/2) == 131))))
                    {
                        if(min_R == 0 || val < min_R)
                                min_R = val;
                        if(val > max_R)
                                max_R = val;
                    }


        		if (val < low_level || val > high_level)
        		{
                        if (((page*64+(idx-2)/2)<total_count &&
                        ((((page*64+(idx-2)/2)%x_count) < y_count)))||(Project_Flag == 3 &&
                            (((page*64+(idx-2)/2) == 11) ||
                            ((page*64+(idx-2)/2) == 23) ||
                            ((page*64+(idx-2)/2) == 119) ||
                            ((page*64+(idx-2)/2) == 131))))
                        {
                        	printk(KERN_INFO "[Touch] CH %d (X%d,Y%d) = %d, out of specification",
                        			page*64+(idx-2)/2, (page*64+(idx-2)/2)/x_count,
                        			(page*64+(idx-2)/2)%x_count, val);
                        		
                        st_result_count++;
                        }
        		}
        	}
        	
        	if (page < 3)
        	{
        		addr = g_RegAddr[GEN_COMMANDPROCESSOR_T6] + 5;
        		data = 0x1;		// page up
        		write_mem(addr, 1, &data);
        	}
        	
        	msleep(50);
        }
	
	//if (qt_debug_level >= DB_LEVEL1)
		printk(KERN_INFO "[Touch] Self test function complete, fail count = %d\n", st_result_count);
        printk(KERN_INFO "[Touch] Self test function complete, max:%d min:%d\n",max_R,min_R);

        if(Project_Flag == 3) {
            addr = g_RegAddr[TOUCH_KEYARRAY_T15];
            write_mem(addr, sizeof(v10_var01_T15_with_gain), v10_var01_T15_with_gain);
            mdelay(20);            
        }

        if(st_result_count == 0 && st_result == 0) {
            st_result = 1;
            sprintf(test_result,"self test success");                    
            return;
        }
            
        sprintf(test_result,"signal limit failed max:%d,min:%d",max_R,min_R);

}


void QT_SelfTest(void)
{
	uint16_t addr;
    uint8_t data = 0xFE; // Nonzero value
    uint8_t v10_var01_T15_0[] = { 3, 0, 11, 2, 1, 0, 0, 55, 3, 0, 0 };
    uint8_t v10_var01_T15_1[] = { 3, 9, 11, 2, 1, 0, 0, 55, 3, 0, 0 };


    if(Project_Flag == 3){
        addr = g_RegAddr[TOUCH_KEYARRAY_T15];
        write_mem(addr, sizeof(v10_var01_T15_0), v10_var01_T15_0);
        mdelay(20);    
        addr = g_RegAddr[SPT_SELFTEST_T25] + 1;    
        write_mem(addr, 1, &data);  // write one nonzero value;

        mdelay(50);
        
        addr = g_RegAddr[TOUCH_KEYARRAY_T15];
        write_mem(addr, sizeof(v10_var01_T15_1), v10_var01_T15_1);
        mdelay(20);    
    }
    
        addr = g_RegAddr[SPT_SELFTEST_T25] + 1;    
        write_mem(addr, 1, &data);  // write one nonzero value;
	
    TCH_MSG("Done.");
}


static void set_reference_level(int max_level, int min_level)
{
        unsigned char config_string[4] = {0,};
        uint8_t config_string1[8] = {0,};    
        int printcount;
        uint16_t addr;

        high_boundary = max_level;
        low_boundary = min_level;

        config_string[0] = max_level & 0xFF;
        config_string[1] = max_level >>8;        
        config_string[2] = min_level & 0xFF;     
        config_string[3] = min_level >>8;

        addr = g_RegAddr[SPT_SELFTEST_T25] + 2;    

        switch(Project_Flag){
            case 1://TBP
            case 2://TNQ
                write_mem(addr, 4, config_string);  

                addr = g_RegAddr[SPT_SELFTEST_T25];    
                read_mem(addr, 8, config_string1); 
                for(printcount=0;printcount<8;printcount++)
                    printk("SPT_SELFTEST_T25 %d:%x\n",printcount,config_string1[printcount]);
                
               break;
            case 3://ITV
                write_mem(addr, 4, config_string);  
                write_mem(addr+4, 4, config_string);        
                read_mem(addr, 12, config_string1); 
                for(printcount=0;printcount<12;printcount++)
                    printk("SPT_SELFTEST_T25 %d:%x\n",printcount,config_string1[printcount]);
                
               break;
           }

        printk("max_level:%d\n",max_level);
        printk("max_level:%d\n",min_level);
    
}

static ssize_t NJ_qt_reference_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    int max_level,min_level;
    char *string;


    string = kzalloc(20,GFP_KERNEL);
    strncpy(string, buf, 20);

    max_level = atoi(strsep(&string, " "));
    min_level =  atoi(strsep(&string, " "));

    set_reference_level(max_level, min_level);
        
    return 0;
}


static ssize_t NJ_qt_reference_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    unsigned char config_string[4] = {0,};
    int max_level,min_level;
    int max_level_high,min_level_high;
    uint16_t addr;

    addr = g_RegAddr[SPT_SELFTEST_T25] + 2;    
    read_mem(addr, 4, config_string); 

    max_level_high = config_string[1];
    min_level_high = config_string[3];

    max_level = config_string[0] + (max_level_high<<8);
    min_level = config_string[2] + (min_level_high<<8);
    
    return sprintf(buf, "max:%d,min:%d\n",max_level,min_level);
}


static ssize_t NJ_qt_result_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    return sprintf(buf, "%s\n", test_result);
}


static ssize_t NJ_qt_test_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    return sprintf(buf, "%d\n", st_result);
}

static ssize_t NJ_qt_test_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    int value = simple_strtoul(buf, NULL, 10);

    printk("NJ_qt_test_store:%d\n",value);
    if (value == 0)
    {
    	printk(KERN_INFO "[Touch] Run self test...\n");
    	QT_SelfTest();
    }
    else if (value == 1)
    {
        printk(KERN_INFO "[Touch] Read configuration from file...\n");
        re_config_NPM();
    }
    else if (value == 2)
    {
        printk(KERN_INFO "[Touch] calibration...\n");
        QT_Calibrate();
    }
    else if (value == 3)
    {
        printk(KERN_INFO "[Touch] read reference data...\n");
        QT_read_reference();
    }

    return count;
}

static struct device_attribute NJ_qt_dev_attr_enable = __ATTR(test, S_IRUGO|S_IWUSR|S_IWGRP,
        NJ_qt_test_show, NJ_qt_test_store);
static struct device_attribute NJ_qt_dev_attr_result = __ATTR(result, S_IRUGO|S_IWUSR|S_IWGRP,
        NJ_qt_result_show, NULL);
static struct device_attribute NJ_qt_dev_attr_reference = __ATTR(reference, S_IRUGO|S_IWUSR|S_IWGRP,
        NJ_qt_reference_show, NJ_qt_reference_store);


static struct attribute *NJ_qt_attributes[] = {
    &NJ_qt_dev_attr_enable.attr,   
    &NJ_qt_dev_attr_result.attr,
    &NJ_qt_dev_attr_reference.attr,
    NULL
};

static struct attribute_group NJ_qt_attribute_group = {
    .attrs = NJ_qt_attributes
};




//send command to receive check data
//NJ-BSP-simon TBP & TNQ & ITV
static void QT_DebugCommand(void)
{
    uint16_t addr = 0;
    uint8_t data = 0xF3; // Nonzero value

    addr = g_RegAddr[GEN_COMMANDPROCESSOR_T6] + 5;  // Call the calibrate field, command processor + 2
    write_mem(addr, 1, &data);  // Write one nonzero value.

    TCH_DBG(DB_LEVEL1, "Done.");
}



static void qt602240_Read_tch_atch(void)
{
    int i,j,check_mask;
    int tch_ch = 0, atch_ch = 0;
    int try=0;
    int Max_X_Number;    
    int Anti_Start_address;
    uint16_t Debug_addr = 0;
    uint8_t page_up_data = 0x01;

    
    QT_DebugCommand();
//    msleep(20);
    read_mem(g_RegAddr[SPARE_T37],82,(void *) g_TouchAntitouch);
    
     while(!((g_TouchAntitouch[0] == 0xF3) && (g_TouchAntitouch[1] == 0x00)))
     {
         if (try > 100)
         {
             printk("faild");
             break;
         }   
         msleep(5);
         try++;
         read_mem(g_RegAddr[SPARE_T37],2,(void *) g_TouchAntitouch);
     }
     read_mem(g_RegAddr[SPARE_T37],82,(void *) g_TouchAntitouch);

     switch(Project_Flag){
         case 1://TBP
            Max_X_Number=16;
            Anti_Start_address=20*2;
            break;
         case 2:
            Max_X_Number=14;
            Anti_Start_address=14*2;
            break;
         case 3:
            Max_X_Number=18;
            Anti_Start_address=20*2;
            break;
        }

     for(i=2 ;i<(Max_X_Number*2+2); i +=2)
     {
        for(j=0; j<8; j++)
        {
            check_mask = 1<<j;
            if(g_TouchAntitouch[i] & check_mask)
            {
                tch_ch++;
            }
            if(g_TouchAntitouch[i+1] & check_mask)
            {
                tch_ch++;
            }
            if(g_TouchAntitouch[Anti_Start_address+i] & check_mask)
            {
                atch_ch++;
            }
            if(g_TouchAntitouch[Anti_Start_address+1+i] & check_mask)
            {
                atch_ch++;
            }
        }    
     }                        
		printk("Antitouch>0 or tch_ch:%d, atch_ch:%d\n",tch_ch,atch_ch);

    Debug_addr = g_RegAddr[GEN_COMMANDPROCESSOR_T6] + 5;  // Call the calibrate field, command processor + 2
    write_mem(Debug_addr, 1, &page_up_data);  //page up

	if( (tch_ch>0) )
	{
	
		printk("in if tch_ch:%d, atch_ch:%d\n",tch_ch,atch_ch);
		/* cal was bad - must recalibrate and check afterwards */
		QT_Calibrate();

		cal_check_flag = 1;
		ponit_num = 0;
	}
	else
	{
		//printk("calibration was not decided yet\n");
		/* we cannot confirm if good or bad - we must wait for next touch 
		* message to confirm */
		cal_check_flag = 0;
	}
}



//NJ-BSP-Simon-Touch-TBP\TINQ\ITV
//fix the palm problem
//func for calibration after resume & power on
// 2011.10.28
static void qt602240_Check_Calibration(void)
{
        int i,j,check_mask;
        int tch_ch = 0, atch_ch = 0;
        int try=0;
        int Max_X_Number;    
        int Anti_Start_address;
        uint16_t Debug_addr = 0;
        uint8_t page_up_data = 0x01;
    
        
        QT_DebugCommand();
    //    msleep(20);
        read_mem(g_RegAddr[SPARE_T37],82,(void *) g_TouchAntitouch);
        
         while(!((g_TouchAntitouch[0] == 0xF3) && (g_TouchAntitouch[1] == 0x00)))
         {
             if (try > 100)
             {
                 printk("faild");
                 break;
             }   
             msleep(5);
             try++;
             read_mem(g_RegAddr[SPARE_T37],2,(void *) g_TouchAntitouch);
         }
         read_mem(g_RegAddr[SPARE_T37],82,(void *) g_TouchAntitouch);
    
         switch(Project_Flag){
             case 1://TBP
                Max_X_Number=16;
                Anti_Start_address=20*2;
                break;
             case 2:
                Max_X_Number=14;
                Anti_Start_address=14*2;
                break;
             case 3:
                Max_X_Number=18;
                Anti_Start_address=20*2;
                break;
            }
    
         for(i=2 ;i<(Max_X_Number*2+2); i +=2)
         {
            for(j=0; j<8; j++)
            {
                check_mask = 1<<j;
                if(g_TouchAntitouch[i] & check_mask)
                {
                    tch_ch++;
                }
                if(g_TouchAntitouch[i+1] & check_mask)
                {
                    tch_ch++;
                }
                if(g_TouchAntitouch[Anti_Start_address+i] & check_mask)
                {
                    atch_ch++;
                }
                if(g_TouchAntitouch[Anti_Start_address+1+i] & check_mask)
                {
                    atch_ch++;
                }
            }    
         }                        
            printk("Antitouch>0 or tch_ch:%d, atch_ch:%d\n",tch_ch,atch_ch);
    
        Debug_addr = g_RegAddr[GEN_COMMANDPROCESSOR_T6] + 5;  // Call the calibrate field, command processor + 2
        write_mem(Debug_addr, 1, &page_up_data);  //page up


	if( (atch_ch>7 || tch_ch>12) )
	{
	
		printk("in if tch_ch:%d, atch_ch:%d\n",tch_ch,atch_ch);
		/* cal was bad - must recalibrate and check afterwards */
		QT_Calibrate();

		cal_check_flag = 1;
		ponit_num = 0;
	}
	else
	{
		//printk("calibration was not decided yet\n");
		/* we cannot confirm if good or bad - we must wait for next touch 
		* message to confirm */
		cal_check_flag = 1;
	}
}

static void qt602240_isr_workqueue_V2(struct work_struct *work)
{
    struct qt602240_info *ts = container_of(work, struct qt602240_info, wqueue);

    uint16_t Buf_X;
    uint16_t Buf_Y;
    uint8_t /*count = 3,*/ MSG_TYPE = 0;
    int i,first_touch_id = qt602240->first_finger_id;
    uint8_t icount; 
//    int mulpoint_num;
    int go_out = 0;

#ifdef CONFIG_FIH_FTM
    uint16_t addr;
    uint8_t v10_var01_T15[] = { 3, 0, 11, 18, 1, 0, 16, 55, 3, 0, 0 };
#endif

#ifdef WATER    
//    int power_cfg_address;
#endif
    g_I2CAddress = g_RegAddr[GEN_MESSAGEPROCESSOR_T5];

    //printk("qt602240_isr_workqueue\n");


#if 0            
            while (read_mem(g_I2CAddress , 8, (void *) g_MsgData ) != 0)
            {
                TCH_ERR("Read data failed, Re-read.");
                mdelay(3);

                count--;
                if (count == 0)
                {
                    TCH_MSG("Can't Read/write data, reset chip.");
					if(goio_tp_rst_n)
                    TCH_MSG("goio_tp_rst_n = %d", gpio_get_value(goio_tp_rst_n));
                    TCH_MSG("gpio_tp_int_n = %d", gpio_get_value(gpio_tp_int_n));
                    qt602240_ts_reset();  // Re-try 3 times, can't read/write data, reset qt602240 chip
                    QT_GetConfigStatus();
                    QT_InitConfig();
                    QT_Calibrate();
                    if(Project_Flag)
                        goto work_out;
                    else
                        return;
                }
            }
#else
            read_mem(g_I2CAddress , 8, (void *) g_MsgData );
            
            for(icount=0;icount<2;icount++){
                TP_MSG("g_MsgData[%d]:%x\n",icount,g_MsgData[icount]);
                  }
#endif


#ifndef CONFIG_FIH_FTM
            if(g_MsgData[0]>1 && g_MsgData[0]<7)
                if(!(g_MsgData[1] == 0x30 || g_MsgData[1] == 0x20 || g_MsgData[1] == 0xC0 ||g_MsgData[1] == 0x90))
                    goto work_out;
#endif                

            
            if (qt_debug_level >= DB_LEVEL2)
            {
            	for (i=0; i<8; i++)
                	TCH_DBG(DB_LEVEL2, "g_MsgData[%d] = 0x%x", i, g_MsgData[i]);
            }

            MSG_TYPE = g_ReportID[g_MsgData[0]];
            if ((MSG_TYPE == 0xFF) || (MSG_TYPE == 0x00))   // No data, return
            {
                TCH_DBG(DB_LEVEL2, "No data, Leave ISR.");
                if(goio_tp_rst_n)				
                TCH_DBG(DB_LEVEL2, "goio_tp_rst_n = %d", gpio_get_value(goio_tp_rst_n));
                TCH_DBG(DB_LEVEL2, "gpio_tp_int_n = %d", gpio_get_value(gpio_tp_int_n));
                goto work_out;
            }
#ifdef CONFIG_FIH_FTM
            else if ((MSG_TYPE == 21) 
                     || (g_MsgData[0] == 21)  //TBP
                     || (g_MsgData[0] == 6)   //TNQ
                     || (g_MsgData[0] == 15)) //ITV
            {
                if (g_MsgData[1] == 0xFE)
                {
                    sprintf(test_result,"self test success");                    
                    printk(KERN_INFO "[Touch] Self test PASS!\n");
                    if(Project_Flag == 3){
                        if(++selftest_count == 2){
                            st_result = 1;
                            selftest_count = 0;
                            addr = g_RegAddr[TOUCH_KEYARRAY_T15];
                            write_mem(addr, sizeof(v10_var01_T15), v10_var01_T15);
                            mdelay(20);    
                        }
                    } else 
                        st_result = 1;
                } else if(g_MsgData[1] == 0x17) {
                        printk(KERN_INFO "[Touch] Self test FAIL (0x%x)!\n", g_MsgData[1]);
                        mdelay(50);
                        st_result = 0;
                        QT_read_reference();
                    }
                else if(g_MsgData[1] == 0x11) {
                    sprintf(test_result,"pin fault");
                    printk(KERN_INFO "[Touch] Self test FAIL (0x%x)!\n", g_MsgData[1]);
                    st_result = 0;
                } else {
                    printk(KERN_INFO "[Touch] Self test FAIL (0x%x)!\n", g_MsgData[1]);
                    sprintf(test_result,"failed other reason");                    
                }
            }
#endif            
            //Div2-D5-Peripheral-FG-TouchErrorHandle-00+[
            else if (MSG_TYPE == GEN_COMMANDPROCESSOR_T6)
            {
                if (g_MsgData[1] & 0x04)
                    TCH_ERR("I2C-Compatible Checksum Errors.");
                if (g_MsgData[1] & 0x08)
                    TCH_ERR("Configuration Errors.");
                if (g_MsgData[1] & 0x10)
                    TCH_MSG("Calibrating.");
                if (g_MsgData[1] & 0x20)
                    TCH_ERR("Signal Errors.");
                if (g_MsgData[1] & 0x40)
                    TCH_ERR("Overflow Errors.");
                if (g_MsgData[1] & 0x80)
                    TCH_MSG("Reseting.");
                if (g_MsgData[1] & 0x04 /*|| ((g_MsgData[1] & 0x10) && !gpio_get_value(gpio_tp_int_n))*/ || g_MsgData[1] & 0x20)
                {
                    TCH_MSG("Prepare to reset touch IC. Please wait a moment...");
                    qt602240_ts_reset();
                    QT_GetConfigStatus();
                    QT_InitConfig();
                    QT_Calibrate();
                    TCH_MSG("Finish to reset touch IC.");
                    if(goio_tp_rst_n)
                    TCH_MSG("goio_tp_rst_n = %d", gpio_get_value(goio_tp_rst_n));
                    TCH_MSG("gpio_tp_int_n = %d", gpio_get_value(gpio_tp_int_n));
                    goto work_out;
                }
            }
            //Div2-D5-Peripheral-FG-TouchErrorHandle-00+]
    //MTD-BSP-Colin-Touch03++[
    		else if ((MSG_TYPE == TOUCH_KEYARRAY_T15))
            {
                if(g_NVBackUp || ts->points[0].first_area == PRESS_TOUCH_AREA){
                    TCH_ERR("NVBackUp Not Down!");
                    goto work_out;
                }
                // Key1
                if (g_MsgData[2] & 0x01)
                {
                    if (!ts->key_state[0])
                    {
                        input_report_key(ts->keyevent_input, KEY_HOME, 1);
                        //TCH_MSG("Report HOME key Down.");
                        TCH_DBG(DB_LEVEL2, "Report HOME key Down.");
                        ts->key_state[0] = TRUE;
                    }
                }
                else
                {
                    if (ts->key_state[0])
                    {
                        input_report_key(ts->keyevent_input, KEY_HOME, 0);
                        //TCH_MSG("Report HOME key Up.");
                        TCH_DBG(DB_LEVEL2, "Report HOME key Up.");
                        ts->key_state[0] = FALSE;
                    }
                }
                // Key2
                if (g_MsgData[2] & 0x02)
                {
                    if (!ts->key_state[1])
                    {
                        input_report_key(ts->keyevent_input, KEY_MENU, 1);
                        //TCH_MSG("Report MENU key Down.");
                        TCH_DBG(DB_LEVEL2, "Report MENU key Down.");
                        ts->key_state[1] = TRUE;
                    }
                }
                else
                {
                    if (ts->key_state[1])
                    {
                        input_report_key(ts->keyevent_input, KEY_MENU, 0);
                        //TCH_MSG("Report MENU key Up.");
                        TCH_DBG(DB_LEVEL2, "Report MENU key Up.");
                        ts->key_state[1] = FALSE;
                    }
                }
                // Key3
                if (g_MsgData[3] & 0x02)
                {
                    if (!ts->key_state[2])
                    {
                        input_report_key(ts->keyevent_input, KEY_BACK, 1);
                        //TCH_MSG("Report BACK key Down.");
                        TCH_DBG(DB_LEVEL2, "Report BACK key Down.");
                        ts->key_state[2] = TRUE;
                    }
                }
                else
                {
                    if (ts->key_state[2])
                    {
                        input_report_key(ts->keyevent_input, KEY_BACK, 0);
                        //TCH_MSG("Report BACK key Up.");
                        TCH_DBG(DB_LEVEL2, "Report BACK key Up.");
                        ts->key_state[2] = FALSE;
                    }
                }
                // Key4
                if (g_MsgData[3] & 0x04)
                {
                    if (!ts->key_state[3])
                    {
                        input_report_key(ts->keyevent_input, KEY_SEARCH, 1);
                        //TCH_MSG("Report SEARCH key Down.");
                        TCH_DBG(DB_LEVEL2, "Report SEARCH key Down.");
                        ts->key_state[3] = TRUE;
                    }
                }
                else
                {
                    if (ts->key_state[3])
                    {
                        input_report_key(ts->keyevent_input, KEY_SEARCH, 0);
                        //TCH_MSG("Report SEARCH key Up.");
                        TCH_DBG(DB_LEVEL2, "Report SEARCH key Up.");
                        ts->key_state[3] = FALSE;
                    }
                }
            }

            else if(g_MsgData[0] ==16 ||g_MsgData[0] == 14||g_MsgData[0] == 7/*TInQ Face resigster ReportID=7*/)   //face suppression message
            {
                switch(Project_Flag){
                        case 1://TBP
                            if(g_MsgData[0] == 14)
                            {
                                TP_MSG("Face suppression Calibration------------------------\n");
                                QT_Calibrate();
                                cal_check_flag=1;
                            }
                            //write_mem(power_cfg_address, sizeof(v10_T8), (void *)v10_T8 );
                            break;
                        case 2://TNQ
                            if(g_MsgData[0] == 7)
                            {
                                TP_MSG("Face suppression Calibration------------------------\n");
                                QT_Calibrate();
                                cal_check_flag=1;
                            }
                            //write_mem(power_cfg_address, sizeof(v10_var13_T8), (void *)v10_var13_T8 );
                            break;
                        case 3://ITV
                            if(g_MsgData[0] == 16)
                            {
                                TP_MSG("Face suppression Calibration------------------------\n");
                                QT_Calibrate();
                                cal_check_flag=1;
                            }
                            break;        
                    }        
                }
    //MTD-BSP-Colin-Touch03++]
            else if ((MSG_TYPE == PROCI_GRIPFACESUPPRESSION_T20))
            {
                if(g_NVBackUp){
                    TCH_ERR("NVBackUp Not Down!");
                    goto work_out;
                }
                if (g_MsgData[1] & 0x01)
                {
                    TCH_MSG("Face Down, Status = 0x%x, ID = %d", g_MsgData[1], MSG_TYPE);
                    ts->facedetect = TRUE;
                    goto work_out;
                }
                else if (ts->facedetect)
                {
                    TCH_MSG("Face Up, Status = 0x%x, ID = %d", g_MsgData[1], MSG_TYPE);
                    ts->facedetect = FALSE;
                    for (i=0; i<TS_MAX_POINTS; i++)
                    {
                        if (ts->points[i].last_area > NO_TOUCH)
                        {   // Report a touch pen up event when touch detects finger down events in face detection(Face Up).
                            qt602240_report(ts, i, ts->points[i].x, ts->points[i].y, NO_TOUCH);
                            //TCH_MSG("POS%d Up, Status = 0x%x, X = %d, Y = %d, ID = %d", i+1, g_MsgData[1], ts->points[i].x, ts->points[i].y, MSG_TYPE);
                            input_sync(ts->touch_input);	//Div2-D5-Peripheral-FG-AddMultiTouch-00+
                        }
                    }
                }
            }
            else if (MSG_TYPE != TOUCH_MULTITOUCHSCREEN_T9)
            {
                TCH_DBG(DB_LEVEL1, "Get other report id = %d", MSG_TYPE);
                
                for (i=0; i<8; i++)
                    TCH_DBG(DB_LEVEL1, "Report T%d[%d] = %d", MSG_TYPE, i ,g_MsgData[i]);            
                goto work_out;
            }

            if (g_NVBackUp)
            {
                TCH_ERR("NVBackUp Not Down!");
                goto work_out;
            }
            
            Buf_X = (g_MsgData[2] << 2) | ((g_MsgData[4] & ~0x3f)>> 6);
            Buf_Y = (g_MsgData[3] << 2) | ((g_MsgData[4] & ~0xf3)>> 2) ;

            if (qt_debug_level >= DB_LEVEL1)
            {
            	if (g_MsgData[1] & 0xC0)
            		printk(KERN_INFO "POS %d, x = %d, y = %d, Down\n", g_MsgData[0] - first_touch_id, Buf_X, Buf_Y);
            		
            	if (g_MsgData[1] & 0x20)
            		printk(KERN_INFO "POS %d, x = %d, y = %d, Up\n", g_MsgData[0] - first_touch_id, Buf_X, Buf_Y);
            }

//NJ-BSP-Simon-Touch-TBP
// 2011.10.28
//fix palm problem in TBP & TNQ & ITV    	
            if(Project_Flag)	
            {
                if((cal_check_flag == 1) && (g_MsgData[0] == 0x02))
                {
                		qt602240_Check_Calibration();
                    	PositionX[ponit_num]=Buf_X;
                    	PositionY[ponit_num]=Buf_Y;
                    	TP_MSG("PositionX %2d, 0x%x\n",ponit_num,PositionX[ponit_num]);
                    	TP_MSG("PositionY %2d, 0x%x\n",ponit_num,PositionY[ponit_num]);

                    	ponit_num++;
                    	if(ponit_num==5) 
                    	{
                    	    ponit_num=0;
                    	    XY_Flag = 1;
                    	}

                }
                                
                if((cal_check_flag == 1) && (g_MsgData[0] == 0x02) && XY_Flag == 1)
                {
                    	XY_Flag = 0;
                    	if((PositionX[4]-PositionX[0])>90 ||(PositionX[0]-PositionX[4])>90
                    	    ||(PositionY[4]-PositionY[0])>60 ||(PositionY[0]-PositionY[4])>60)
                    	{	
                    	    cal_check_flag=0;
#ifdef WATER
#if 0
                        power_cfg_address = g_RegAddr[GEN_ACQUISITIONCONFIG_T8];    
                        switch(Project_Flag){
                            case 1://TBP
                                write_mem(power_cfg_address, sizeof(v10_T8), (void *)v10_T8 );
                                break;
                            case 2://TNQ
                                write_mem(power_cfg_address, sizeof(v10_var13_T8), (void *)v10_var13_T8 );
                                break;
                            case 3://ITV
                                printk("kami_write\n");
                                write_mem(power_cfg_address, sizeof(v10_var01_T8), (void *)v10_var01_T8 );
                                break;        
                            case 4://IP
                                printk("kami_write\n");
                                write_mem(power_cfg_address, sizeof(v10_var01_T8_V2), (void *)v10_var01_T8_V2 );
                                break;        
                                
                            }
#endif                        
                                mod_timer(&tp_timer,jiffies+20*HZ);
#endif
                   		}
                    	
                }
#if 0                
                 if((cal_check_flag == 1) && (g_MsgData[0] > 0x02)&& (g_MsgData[0] < 0x07) && 
                 g_MsgData[1] == 0xC0)
                {
                    printk("03 04 calibration\n");                     
                    QT_Calibrate();
                    ponit_num=0;
                    cal_check_flag=1;
                }       
#endif

            }
            TP_MSG("cal_check_flag:%d\n",cal_check_flag);  

#if 0
            mulpoint_num = (int)g_MsgData[0]-qt602240->first_finger_id;
            printk("mulpoint_num:%d\n",mulpoint_num );
            if((mulpoint_num>=0) && (mulpoint_num < 5))  
            {
                if((g_MsgData[1] & 0xC0) == 0xC0)
                    point_status_V[mulpoint_num].press++;
                else if((g_MsgData[1] & 0x20) && (point_status_V[mulpoint_num].press == 1))                
                    point_status_V[mulpoint_num].press--;
                
            }   
#endif
           //for(icount=0;icount<TS_MAX_POINTS;icount++)
            //printk("point_status[%d]:%d\n",icount,point_status_V[icount].press);

            if((g_MsgData[0] == 0x01) && (g_MsgData[1] == 0x10))
            {
                TP_MSG("Received Calibration message\n" );
                if(!cal_check_flag)
                mod_timer(&tp_timer,jiffies+20*HZ);

                cal_check_flag=1;
#if 0                
                for(icount=0;icount<TS_MAX_POINTS;icount++)
                {
                    if(point_status_V[icount].press > 0)
                    {
                    printk("fake_reease set bit\n");

                    point_status_V[icount].press = 0;
                    point_status_V[icount].fake_release = 1;
                    g_MsgData[0] = icount + qt602240->first_finger_id;
                    g_MsgData[1] = 0x20; 
                    qt602240_fake_release(ts);
                    go_out = 1;
                    }
                }
#endif       
                for (i=0; i<TS_MAX_POINTS; i++)
                {
                    if (qt602240->points[i].last_area > NO_TOUCH)
                        {
                        //TCH_MSG("POS%d Up, X = %d, Y = %d", i+1, qt602240->points[i].x, qt602240->points[i].y);
                            qt602240_report(qt602240, i, qt602240->points[i].x, qt602240->points[i].y, NO_TOUCH);
                            input_sync(qt602240->touch_input);
                            go_out = 1;
                            TP_MSG("fake_reease set bit\n");
                            
                        }
                }        
                if(go_out)
                    goto work_out;
            }

            
            
            for (i=0; i<TS_MAX_POINTS; i++)
            {
                if (g_MsgData[0] == first_touch_id + i)
                {
                    ts->points[i].x = Buf_X;
                    ts->points[i].y = Buf_Y;

                    if (g_MsgData[1] & 0xC0)
                    {	// Finger Down.
                        //Div2-D5-Peripheral-FG-AddMultiTouch-01*[
                        if (i == 0)
                        {
                            qt602240_report(ts, i, ts->points[i].x, ts->points[i].y, 255);
#ifdef QT602240_MT
                            if (ts->points[i+1].last_area > NO_TOUCH)
                                qt602240_report(ts, i+1, ts->points[i+1].x, ts->points[i+1].y, 255);
                            
                            if (ts->points[i+2].last_area > NO_TOUCH)
                                qt602240_report(ts, i+2, ts->points[i+2].x, ts->points[i+2].y, 255);
                            
                            if (ts->points[i+3].last_area > NO_TOUCH)
                                qt602240_report(ts, i+3, ts->points[i+3].x, ts->points[i+3].y, 255);
                            
                            if (ts->points[i+4].last_area > NO_TOUCH)
                                qt602240_report(ts, i+4, ts->points[i+4].x, ts->points[i+4].y, 255);
#endif
                            input_sync(ts->touch_input);
                        }
                        else if (i == 1)
                        {
#ifdef QT602240_MT
                            if (ts->points[i-1].last_area > NO_TOUCH)
                                qt602240_report(ts, i-1, ts->points[i-1].x, ts->points[i-1].y, 255);
                            
                            if (ts->points[i+1].last_area > NO_TOUCH)
                                qt602240_report(ts, i+1, ts->points[i+1].x, ts->points[i+1].y, 255);
                            
                            if (ts->points[i+2].last_area > NO_TOUCH)
                                qt602240_report(ts, i+2, ts->points[i+2].x, ts->points[i+2].y, 255);
                            
                            if (ts->points[i+3].last_area > NO_TOUCH)
                                qt602240_report(ts, i+3, ts->points[i+3].x, ts->points[i+3].y, 255);
#endif
                            qt602240_report(ts, i, ts->points[i].x, ts->points[i].y, 255);
                            
                            input_sync(ts->touch_input);
                        }
                        else if (i == 2)
                        {
#ifdef QT602240_MT
                            if (ts->points[i-2].last_area > NO_TOUCH)
                                qt602240_report(ts, i-2, ts->points[i-2].x, ts->points[i-2].y, 255);
                            
                            if (ts->points[i-1].last_area > NO_TOUCH)
                                qt602240_report(ts, i-1, ts->points[i-1].x, ts->points[i-1].y, 255);
                            
                            if (ts->points[i+1].last_area > NO_TOUCH)
                                qt602240_report(ts, i+1, ts->points[i+1].x, ts->points[i+1].y, 255);
                            
                            if (ts->points[i+2].last_area > NO_TOUCH)
                                qt602240_report(ts, i+2, ts->points[i+2].x, ts->points[i+2].y, 255);
#endif
                            qt602240_report(ts, i, ts->points[i].x, ts->points[i].y, 255);
                            
                            input_sync(ts->touch_input);
    					}
                        else if (i == 3)
                        {
#ifdef QT602240_MT
                            if (ts->points[i-3].last_area > NO_TOUCH)
                                qt602240_report(ts, i-3, ts->points[i-3].x, ts->points[i-3].y, 255);
                            
                            if (ts->points[i-2].last_area > NO_TOUCH)
                                qt602240_report(ts, i-2, ts->points[i-2].x, ts->points[i-2].y, 255);
                            
                            if (ts->points[i-1].last_area > NO_TOUCH)
                                qt602240_report(ts, i-1, ts->points[i-1].x, ts->points[i-1].y, 255);
                            
                            if (ts->points[i+1].last_area > NO_TOUCH)
                                qt602240_report(ts, i+1, ts->points[i+1].x, ts->points[i+1].y, 255);
#endif
                            qt602240_report(ts, i, ts->points[i].x, ts->points[i].y, 255);
                            
                            input_sync(ts->touch_input);
    					}
                        else if (i == 4)
                        {
#ifdef QT602240_MT
                            if (ts->points[i-4].last_area > NO_TOUCH)
                                qt602240_report(ts, i-4, ts->points[i-4].x, ts->points[i-4].y, 255);
                            
                            if (ts->points[i-3].last_area > NO_TOUCH)
                                qt602240_report(ts, i-3, ts->points[i-3].x, ts->points[i-3].y, 255);
                            
                            if (ts->points[i-2].last_area > NO_TOUCH)
                                qt602240_report(ts, i-2, ts->points[i-2].x, ts->points[i-2].y, 255);
                            
                            if (ts->points[i-1].last_area > NO_TOUCH)
                                qt602240_report(ts, i-1, ts->points[i-1].x, ts->points[i-1].y, 255);
#endif
                            qt602240_report(ts, i, ts->points[i].x, ts->points[i].y, 255);
                            
                            input_sync(ts->touch_input);
                        }
                        //input_sync(ts->touch_input);
                        //Div2-D5-Peripheral-FG-AddMultiTouch-01*]
                    }
                    else if ((g_MsgData[1] & 0x20) )
                    {	// Finger Up.
                        ponit_num = 0;
                        if (ts->points[i].last_area > NO_TOUCH)
                        {
                            if (i == 0)
                            {
                                qt602240_report(ts, i, ts->points[i].x, ts->points[i].y, NO_TOUCH);
#ifdef QT602240_MT
                                if (ts->points[i+1].last_area > NO_TOUCH)
                                    qt602240_report(ts, i+1, ts->points[i+1].x, ts->points[i+1].y, 255);
                                
                                if (ts->points[i+2].last_area > NO_TOUCH)
                                    qt602240_report(ts, i+2, ts->points[i+2].x, ts->points[i+2].y, 255);
                                
                                if (ts->points[i+3].last_area > NO_TOUCH)
                                    qt602240_report(ts, i+3, ts->points[i+3].x, ts->points[i+3].y, 255);
                                
                                if (ts->points[i+4].last_area > NO_TOUCH)
                                    qt602240_report(ts, i+4, ts->points[i+4].x, ts->points[i+4].y, 255);
#endif
                                input_sync(ts->touch_input);
#ifdef QT602240_MT
                                if (ts->points[i+1].last_area > NO_TOUCH)
                                    qt602240_report(ts, i+1, ts->points[i+1].x, ts->points[i+1].y, 255);
                                
                                if (ts->points[i+2].last_area > NO_TOUCH)
                                    qt602240_report(ts, i+2, ts->points[i+2].x, ts->points[i+2].y, 255);
                                
                                if (ts->points[i+3].last_area > NO_TOUCH)
                                    qt602240_report(ts, i+3, ts->points[i+3].x, ts->points[i+3].y, 255);
                                
                                if (ts->points[i+4].last_area > NO_TOUCH)
                                    qt602240_report(ts, i+4, ts->points[i+4].x, ts->points[i+4].y, 255);
#endif
                            }
                            else if (i == 1)
                            {
#ifdef QT602240_MT
                                if (ts->points[i-1].last_area > NO_TOUCH)
                                    qt602240_report(ts, i-1, ts->points[i-1].x, ts->points[i-1].y, 255);
                                
                                if (ts->points[i+1].last_area > NO_TOUCH)
                                    qt602240_report(ts, i+1, ts->points[i+1].x, ts->points[i+1].y, 255);
                                
                                if (ts->points[i+2].last_area > NO_TOUCH)
                                    qt602240_report(ts, i+2, ts->points[i+2].x, ts->points[i+2].y, 255);
                                
                                if (ts->points[i+3].last_area > NO_TOUCH)
                                    qt602240_report(ts, i+3, ts->points[i+3].x, ts->points[i+3].y, 255);
#endif
                                qt602240_report(ts, i, ts->points[i].x, ts->points[i].y, NO_TOUCH);
                                
                                input_sync(ts->touch_input);
#ifdef QT602240_MT
                                if (ts->points[i-1].last_area > NO_TOUCH)
                                    qt602240_report(ts, i-1, ts->points[i-1].x, ts->points[i-1].y, 255);
                                
                                if (ts->points[i+1].last_area > NO_TOUCH)
                                    qt602240_report(ts, i+1, ts->points[i+1].x, ts->points[i+1].y, 255);
                                
                                if (ts->points[i+2].last_area > NO_TOUCH)
                                    qt602240_report(ts, i+2, ts->points[i+2].x, ts->points[i+2].y, 255);
                                
                                if (ts->points[i+3].last_area > NO_TOUCH)
                                    qt602240_report(ts, i+3, ts->points[i+3].x, ts->points[i+3].y, 255);
#endif
                            }
                            else if (i == 2)
                            {
#ifdef QT602240_MT
                                if (ts->points[i-2].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-2, ts->points[i-2].x, ts->points[i-2].y, 255);

                                if (ts->points[i-1].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-1, ts->points[i-1].x, ts->points[i-1].y, 255);

                                if (ts->points[i+1].last_area > NO_TOUCH)
                                	qt602240_report(ts, i+1, ts->points[i+1].x, ts->points[i+1].y, 255);

                                if (ts->points[i+2].last_area > NO_TOUCH)
                                	qt602240_report(ts, i+2, ts->points[i+2].x, ts->points[i+2].y, 255);
#endif
                                qt602240_report(ts, i, ts->points[i].x, ts->points[i].y, NO_TOUCH);

                                input_sync(ts->touch_input);
#ifdef QT602240_MT
                                if (ts->points[i-2].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-2, ts->points[i-2].x, ts->points[i-2].y, 255);

                                if (ts->points[i-1].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-1, ts->points[i-1].x, ts->points[i-1].y, 255);

                                if (ts->points[i+1].last_area > NO_TOUCH)
                                	qt602240_report(ts, i+1, ts->points[i+1].x, ts->points[i+1].y, 255);

                                if (ts->points[i+2].last_area > NO_TOUCH)
                                	qt602240_report(ts, i+2, ts->points[i+2].x, ts->points[i+2].y, 255);
#endif
                            }
                            else if (i == 3)
                            {
#ifdef QT602240_MT
                                if (ts->points[i-3].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-3, ts->points[i-3].x, ts->points[i-3].y, 255);

                                if (ts->points[i-2].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-2, ts->points[i-2].x, ts->points[i-2].y, 255);

                                if (ts->points[i-1].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-1, ts->points[i-1].x, ts->points[i-1].y, 255);

                                if (ts->points[i+1].last_area > NO_TOUCH)
                                	qt602240_report(ts, i+1, ts->points[i+1].x, ts->points[i+1].y, 255);
#endif
                                qt602240_report(ts, i, ts->points[i].x, ts->points[i].y, NO_TOUCH);

                                input_sync(ts->touch_input);
#ifdef QT602240_MT
                                if (ts->points[i-3].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-3, ts->points[i-3].x, ts->points[i-3].y, 255);

                                if (ts->points[i-2].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-2, ts->points[i-2].x, ts->points[i-2].y, 255);

                                if (ts->points[i-1].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-1, ts->points[i-1].x, ts->points[i-1].y, 255);

                                if (ts->points[i+1].last_area > NO_TOUCH)
                                	qt602240_report(ts, i+1, ts->points[i+1].x, ts->points[i+1].y, 255);
#endif
                            }
                            else if (i == 4)
                            {
#ifdef QT602240_MT
                                if (ts->points[i-4].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-4, ts->points[i-4].x, ts->points[i-4].y, 255);

                                if (ts->points[i-3].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-3, ts->points[i-3].x, ts->points[i-3].y, 255);

                                if (ts->points[i-2].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-2, ts->points[i-2].x, ts->points[i-2].y, 255);

                                if (ts->points[i-1].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-1, ts->points[i-1].x, ts->points[i-1].y, 255);
#endif
                                qt602240_report(ts, i, ts->points[i].x, ts->points[i].y, NO_TOUCH);

                                input_sync(ts->touch_input);
#ifdef QT602240_MT
                                if (ts->points[i-4].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-4, ts->points[i-4].x, ts->points[i-4].y, 255);

                                if (ts->points[i-3].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-3, ts->points[i-3].x, ts->points[i-3].y, 255);

                                if (ts->points[i-2].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-2, ts->points[i-2].x, ts->points[i-2].y, 255);

                                if (ts->points[i-1].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-1, ts->points[i-1].x, ts->points[i-1].y, 255);
#endif
                            }
                            
                            input_sync(ts->touch_input);
                        }
                    }
                }
            }
work_out:    
        enable_irq(qt602240->irq);    
}

static irqreturn_t qt602240_isr_V2(int irq, void * handle)
{
    struct qt602240_info *ts = handle;

    disable_irq_nosync(qt602240->irq);

    schedule_work(&ts->wqueue);

    return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void qt602240_early_suspend_V2(struct early_suspend *h)
{
    uint16_t power_cfg_address = 0;
    uint8_t data[3];
    uint8_t rc = 0;


    TP_MSG("qt602240_early_suspend_V2\n");
    disable_irq(qt602240->irq);
        
    rc = cancel_work_sync(&qt602240->wqueue);
    if(rc){
        TP_MSG("qt602240_early_suspend enable_irq\n");
        enable_irq(qt602240->irq);
    }

    power_cfg_address = g_RegAddr[GEN_POWERCONFIG_T7];
    read_mem(power_cfg_address, 3, (void *) qt602240->T7);

    del_timer(&tp_timer);

    data[0] = 0;
    data[1] = 0;
    data[2] = 0;

    //QT_GetConfigStatus();  // Clear all data, avoid intrrupt no resume.

    rc = write_mem(power_cfg_address, 3, (void *) data);
    if (rc != 0)
        TCH_ERR("Driver can't enter deep sleep mode [%d].", rc);
    else
        TCH_MSG("Enter deep sleep mode.");

    qt602240->suspend_state = TRUE;
    //gpio_tlmm_config(GPIO_CFG(gpio_tp_int_n, 0, GPIO_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    if(goio_tp_rst_n)
    {
        gpio_tlmm_config(GPIO_CFG(goio_tp_rst_n, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
        gpio_set_value(goio_tp_rst_n, HIGH);
        TCH_MSG("goio_tp_rst_n = %d", gpio_get_value(goio_tp_rst_n));
    }
    
    TCH_MSG("gpio_tp_int_n = %d", gpio_get_value(gpio_tp_int_n));

    // gpio 117 low
    gpio_tlmm_config(GPIO_CFG(117, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    gpio_set_value(117, LOW);

    TCH_DBG(DB_LEVEL1, "Done.");
}

static void qt602240_late_resume_V2(struct early_suspend *h)
{
    uint16_t power_cfg_address = 0;
    uint16_t Face_suppression_address = 0;
    
    uint8_t i, rc = 0;


    // gpio 117 high
    gpio_tlmm_config(GPIO_CFG(117, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    gpio_set_value(117, HIGH);

    power_cfg_address = g_RegAddr[GEN_POWERCONFIG_T7];
    //gpio_tlmm_config(GPIO_CFG(gpio_tp_int_n, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    if(goio_tp_rst_n)
    {
        gpio_tlmm_config(GPIO_CFG(goio_tp_rst_n, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
        gpio_set_value(goio_tp_rst_n, HIGH);
    }
    qt602240->suspend_state = FALSE;

    //Div2-D5-Peripheral-FG-TouchErrorHandle-00*[

    for (i=0; i<TS_MAX_POINTS; i++)
    {
        if (qt602240->points[i].last_area > NO_TOUCH)
            {
            //TCH_MSG("POS%d Up, X = %d, Y = %d", i+1, qt602240->points[i].x, qt602240->points[i].y);
                qt602240_report(qt602240, i, qt602240->points[i].x, qt602240->points[i].y, NO_TOUCH);
                input_sync(qt602240->touch_input);
            }
    }        


    for (i=0;i<3;i++)
    {
            rc = write_mem(power_cfg_address, 3, (void *) qt602240->T7);
            if (rc != 0)
            printk(KERN_ERR "Driver can't return from deep sleep mode [%d].", rc);
            else
    	{
            printk(KERN_INFO "Return from sleep mode.\n");
            break;
    	}
    }    

    qt602240_Read_tch_atch();


#ifdef WATER
    power_cfg_address = g_RegAddr[GEN_ACQUISITIONCONFIG_T8];    
    
    TP_MSG(KERN_INFO "Project_Flag= %d\n",Project_Flag);

    switch(Project_Flag){
        case 1://TBP
            TP_MSG(KERN_INFO "TBP--------------------Project_Flag= %d\n",Project_Flag);
            Face_suppression_address=g_RegAddr[PROCI_GRIPFACESUPPRESSION_T20];
            if(cal_check_flag)
                {
                    TP_MSG(KERN_INFO "TBP--------------------cal_check_flag= %d\n",cal_check_flag);
                    rc = write_mem(power_cfg_address, sizeof(v10_T8_1), (void *)v10_T8_1 );
                    rc= write_mem(Face_suppression_address, sizeof(v10_T20_1),(void *)v10_T20_1);
                }
            
            else 
                {
                    rc = write_mem(power_cfg_address, sizeof(v10_T8), (void *)v10_T8 );
                    rc= write_mem(Face_suppression_address, sizeof(v10_T20),(void *)v10_T20);
                }

            break;
        case 2://TNQ
            Face_suppression_address=g_RegAddr[SPARE_T42];          
            if(cal_check_flag)
                {
                    rc = write_mem(power_cfg_address, sizeof(v10_var13_T8_1), (void *)v10_var13_T8_1 );
                    rc= write_mem(Face_suppression_address, sizeof(v10_var13_T42_1),(void *)v10_var13_T42_1);
                }
            
            else 
                {
                    rc = write_mem(power_cfg_address, sizeof(v10_var13_T8), (void *)v10_var13_T8 );
                    rc= write_mem(Face_suppression_address, sizeof(v10_var13_T42),(void *)v10_var13_T42);
                }
            break;
        case 3://ITV
            Face_suppression_address=g_RegAddr[SPARE_T42];
            if(cal_check_flag)
                {
                    rc = write_mem(power_cfg_address, sizeof(v10_var01_T8_1), (void *)v10_var01_T8_1 );
                    rc= write_mem(Face_suppression_address, sizeof(v10_var01_T42_1),(void *)v10_var01_T42_1);
                }
            
            else 
                {
                    rc = write_mem(power_cfg_address, sizeof(v10_var01_T8), (void *)v10_var01_T8 );
                    rc= write_mem(Face_suppression_address, sizeof(v10_var01_T42),(void *)v10_var01_T42);
                }
            break;        
    }
#endif
    msleep(25);

    //qt602240_ts_reset();
    //QT_GetConfigStatus();
    //QT_InitConfig();
    //QT_Calibrate();

//NJ-BSP-Simon-Touch-TBP & TNQ & ITV  
// 2011.10.28
//set flag & count to check calibration after resume   

    TP_MSG(KERN_INFO "set cal_check_flag.\n");
//    cal_check_flag = 1;
    ponit_num = 0;
    
    enable_irq(qt602240->irq);
    //Div2-D5-Peripheral-FG-TouchErrorHandle-00*]
    if(goio_tp_rst_n)
    TP_MSG(KERN_INFO "[Touch] qt602240_late_resume: goio_tp_rst_n = %d", gpio_get_value(goio_tp_rst_n));
    TP_MSG(KERN_INFO "[Touch] qt602240_late_resume: gpio_tp_int_n = %d", gpio_get_value(gpio_tp_int_n));
    TCH_DBG(DB_LEVEL1, "Done.");
}
#endif


#if 0
static void detect_tp_type(void)
{
	uint16_t addr;
	uint8_t v10_19[] = { 7, 0, 0, 0xFF, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    printk("kami [func]:%s [line]:%d \n",__func__,__LINE__);
	addr = g_RegAddr[SPT_GPIOPWM_T19];
	write_mem(addr, sizeof(v10_19), v10_19);
}
#endif


static int qt602240_probe_V2(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct input_dev *touch_input;
    struct input_dev *keyevent_input;
    //struct vreg *device_vreg;//MTD-BSP-Colin-Touch02++
    int i, rc = 0;	//MTD-BSP-Colin-Touch02++
    int irqrequest = 0;
    int rt;

    qt_debug_level = *(uint32_t *)TOUCH_DEBUG_MASK_OFFSET;

	
    TP_MSG(KERN_INFO"[qt602240] %s\n",__func__);

    /* FIHSPEC, AmberYang { */
    /* Patch: FixTouchCannotUseIssue */
    if(gpio_camreset != -1) 
    {
	gpio_request(gpio_camreset, "camera reset");
	gpio_set_value(gpio_camreset, 0);
	msleep(10);
	gpio_set_value(gpio_camreset, 1);
	msleep(10);
	gpio_set_value(gpio_camreset, 0);          
	gpio_free(gpio_camreset);
    }
    /* } FIHSPEC, AmberYang */

    g_NVBackUp = 0;
    
    // I2C Check
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        TCH_ERR("Check I2C functionality failed.");
        return -ENODEV;
    }

    qt602240 = kzalloc(sizeof(struct qt602240_info), GFP_KERNEL);
    if (qt602240 == NULL)
    {
        TCH_ERR("Can not allocate memory.");
        return -ENOMEM;
    }

    info_block = kzalloc(sizeof(info_block_t), GFP_KERNEL);
    if (info_block == NULL)
    {
        TCH_ERR("Can not allocate info block memory.");
        kfree(qt602240);
        return -ENOMEM;
    }
#if 0 //MTD-BSP-Colin-Touch02++[
    // Support Power
    device_vreg = vreg_get(0, TOUCH_DEVICE_VREG);
    if (!device_vreg) {
        TCH_ERR("driver %s: vreg get failed.", __func__);
        return -EIO;
    }

    vreg_set_level(device_vreg, 3000);
    rc = vreg_enable(device_vreg);
    TCH_DBG(0, "Power status = %d", rc);
#endif
    // Config GPIO
    //Div2-D5-Peripheral-FG-4Y6TouchPorting-00*[
    #if defined(CONFIG_FIH_PROJECT_SF4Y6)
    if ((pm8058_gpio_config(PM8058_gpio_tp_int_n, &touch_interrupt)) < 0)
        TCH_ERR("Config IRQ GPIO failed.");
    #else
//MTD-BSP-Colin-Touch03++[
    if(gpio_pwr != -1) 
    {
	gpio_tlmm_config(GPIO_CFG(gpio_pwr, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_set_value(gpio_pwr, HIGH);
	for (i=0; i<10; i++)
	{
        	mdelay(20);
		if (gpio_get_value(gpio_pwr) == LOW)
            	TCH_ERR("PULL 3V EN failed.");
    	}
    }
    gpio_tlmm_config(GPIO_CFG(gpio_tp_int_n, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    gpio_request(gpio_tp_int_n,"TP_INT_N");
	if (gpio_direction_input(gpio_tp_int_n) < 0)
       TCH_ERR("Direct IRQ GPIO failed.");
	//if (gpio_to_irq(gpio_tp_int_n) < 0)
       //TCH_ERR("Request IRQ GPIO failed.");
//MTD-BSP-Colin-Touch03++]
    #endif
    //Div2-D5-Peripheral-FG-4Y6TouchPorting-00*]
    //Div2-D5-Peripheral-FG-4Y6TouchFineTune-01+[
    // Init Work Queue and Register IRQ
    INIT_WORK(&qt602240->wqueue, qt602240_isr_workqueue_V2);
	//MTD-BSP-Colin-Touch02++]
    INIT_WORK(&Timer_wqueue, Timer_workqueue);

    
    //Div2-D5-Peripheral-FG-4Y6TouchFineTune-01+]
    if(goio_tp_rst_n)
    {    
        gpio_tlmm_config(GPIO_CFG(goio_tp_rst_n, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
        gpio_set_value(goio_tp_rst_n, HIGH);
    }
    qt602240_ts_reset();

    // Confirm Touch Chip
    qt602240->client = client;
    i2c_set_clientdata(client, qt602240);
    qt602240->client->addr = 0x4A;//MTD-BSP-Colin-Touch03++
    //Div2-D5-Peripheral-FG-4Y6TouchFineTune-02*[
    for (i=0; i<5; i++)
    {
        if (init_touch_driver(qt602240->client->addr) == DRIVER_SETUP_INCOMPLETE)
            TCH_ERR("Init touch driver failed %d times.", i+1);
        else
            break;
    }
    if (i==5)
        goto err1;
    //Div2-D5-Peripheral-FG-4Y6TouchFineTune-02*]



//MTD-BSP-Colin-Touch03++[
    if (0 == rc) {//ITV&TBP&TNQ use low trigger mode NJ-BSP-simon
        qt602240->irq = MSM_GPIO_TO_INT(gpio_tp_int_n);
        if (qt602240->irq) {
            if (request_irq(qt602240->irq, qt602240_isr_V2, IRQF_TRIGGER_LOW|IRQF_DISABLED, client->dev.driver->name, qt602240)) {
                TCH_ERR("Request IRQ failed.");
                goto err1;
            }
            else {
                irqrequest = 1;
            }
        }
        else {
            TCH_ERR("Request IRQ failed IRQ NULL.");
        }
    }

    touch_input = input_allocate_device();
    if (touch_input == NULL)
    {
        TCH_ERR("Can not allocate memory for touch input device.");
        goto err1;
    }

    keyevent_input = input_allocate_device();
    if (keyevent_input == NULL)
    {
        TCH_ERR("Can not allocate memory for key input device.");
        goto err1;
    }

    touch_input->name  = "qt602240touch";
    touch_input->phys  = "qt602240/input0";
    set_bit(EV_ABS, touch_input->evbit);
    set_bit(EV_SYN, touch_input->evbit);
//Div2-D5-Peripheral-FG-AddMultiTouch-01*[
#ifndef QT602240_MT
    set_bit(EV_KEY, touch_input->evbit);
    set_bit(BTN_TOUCH, touch_input->keybit);
    set_bit(BTN_2, touch_input->keybit);
    input_set_abs_params(touch_input, ABS_X, TS_MIN_X, TS_MAX_X, 0, 0);
    input_set_abs_params(touch_input, ABS_Y, TS_MIN_Y, TS_MAX_Y, 0, 0);
    input_set_abs_params(touch_input, ABS_HAT0X, TS_MIN_X, TS_MAX_X, 0, 0);
    input_set_abs_params(touch_input, ABS_HAT0Y, TS_MIN_Y, TS_MAX_Y, 0, 0);
    input_set_abs_params(touch_input, ABS_PRESSURE, 0, 255, 0, 0);
#else
    input_set_abs_params(touch_input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(touch_input, ABS_MT_POSITION_X, TS_MIN_X, TS_MAX_X, 0, 0);
    input_set_abs_params(touch_input, ABS_MT_POSITION_Y, TS_MIN_Y, TS_MAX_Y, 0, 0);
    input_set_abs_params(touch_input, ABS_X, TS_MIN_X, TS_MAX_X, 0, 0);
    input_set_abs_params(touch_input, ABS_Y, TS_MIN_Y, TS_MAX_Y, 0, 0);
#endif
//Div2-D5-Peripheral-FG-AddMultiTouch-01*]

    if(Project_Flag	== 4)
        keyevent_input->name  = "nonekey";
    else    
        keyevent_input->name  = "qt602240key";
    keyevent_input->phys  = "qt602240/input1";
    set_bit(EV_KEY, keyevent_input->evbit);
    set_bit(KEY_HOME, keyevent_input->keybit);
    set_bit(KEY_MENU, keyevent_input->keybit);
    set_bit(KEY_BACK, keyevent_input->keybit);
	set_bit(KEY_SEARCH, keyevent_input->keybit);//MTD-BSP-Colin-Touch03++
    input_set_abs_params(keyevent_input, ABS_X, 0, 0, 0, 0);

    qt602240->touch_input = touch_input;
    if (input_register_device(qt602240->touch_input))
    {
        TCH_ERR("Can not register touch input device.");
        goto err2;
    }

    qt602240->keyevent_input = keyevent_input;
    if (input_register_device(qt602240->keyevent_input))
    {
        TCH_ERR("Can not register key input device.");
        goto err3;
    }

    memset(qt602240->points, 0, sizeof(struct point_info));
    qt602240->suspend_state = FALSE;
    qt602240->facedetect = FALSE;
    qt602240->T7[0] = 32;
    qt602240->T7[1] = 10;
    qt602240->T7[2] = 50;
#ifdef CONFIG_HAS_EARLYSUSPEND
    // Register early_suspend
    qt602240->es.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
    qt602240->es.suspend = qt602240_early_suspend_V2;
    qt602240->es.resume = qt602240_late_resume_V2;

    register_early_suspend(&qt602240->es);
#endif

    //
    rt = sysfs_create_group(&touch_input->dev.kobj,
            &NJ_qt_attribute_group);
    
    if (rt)
        TCH_ERR("sysfs_create_group failed!");
        
//NJ-BSP-Simon-Touch-TBP & TNQ & ITV  
// 2011.10.28
//set flag to check calibration after power on
    cal_check_flag = 1;
    rt = sysfs_create_link(qt602240->touch_input->dev.kobj.parent, 
                            &qt602240->touch_input->dev.kobj, 
                            qt602240->touch_input->name);
    if (rt)
        goto err4;

#ifdef WATER
        init_timer(&tp_timer);
        tp_timer.function = tp_timer_func;  /* timer handler */
#endif

    
    TCH_DBG(0, "Done.");
    return 0;
err4:
    sysfs_remove_link(qt602240->touch_input->dev.kobj.parent, qt602240->touch_input->name);

err3:
    input_unregister_device(qt602240->touch_input);
//    input_unregister_device(qt602240->keyevent_input);
err2:
    input_free_device(touch_input);
    input_free_device(keyevent_input);
err1:
    //vreg_disable(device_vreg);//MTD-BSP-Colin-Touch02++
//MTD-BSP-Colin-Touch03++[
    if (qt602240->irq && irqrequest){
		free_irq(qt602240->irq, qt602240);
    }
//MTD-BSP-Colin-Touch03++]
    dev_set_drvdata(&client->dev, 0);
    kfree(qt602240);
    kfree(info_block);
    TCH_ERR("Failed.");
    return -1;
}

