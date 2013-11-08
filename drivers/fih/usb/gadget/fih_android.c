/*
fih 1090_usb driver porting,add for HWID control
*/

#include <mach/msm_smd.h>
#include "../../../../arch/arm/mach-msm/proc_comm.h"

#define NV_USB_PRODUCT_STRING   8053

static struct list_head _functions = LIST_HEAD_INIT(_functions);

static void android_set_default_product(int pid);

void android_usb_set_connected(int connected)
{
	if (_android_dev && _android_dev->cdev && _android_dev->cdev->gadget) {
		if (connected)
			usb_gadget_connect(_android_dev->cdev->gadget);
		else
			usb_gadget_disconnect(_android_dev->cdev->gadget);
	}
}

static struct android_usb_function *get_function(const char *name)
{
	struct android_usb_function	*f;
	list_for_each_entry(f, &_functions, list) {
		if (!strcmp(name, f->name))
			return f;
	}
	return 0;
}

static void bind_functions(struct android_dev *dev)
{
	struct android_usb_function	*f;
	char **functions = dev->functions;
	int i;

    pr_debug("%s : begin\n", __func__);
	for (i = 0; i < dev->num_functions; i++) {
		char *name = *functions++;
		f = get_function(name);
		if (f)
			f->bind_config(dev->config);
		else
			pr_err("%s: function %s not found\n", __func__, name);
	}

	/*
	 * set_alt(), or next config->bind(), sets up
	 * ep->driver_data as needed.
	 */
	usb_ep_autoconfig_reset(dev->cdev->gadget);
    pr_debug("%s : end\n", __func__);
}

static int __ref android_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;

    pr_debug("%s : begin\n", __func__);
	dev->config = c;

#ifdef CONFIG_FIH_PID_SWITCH
    	/* bind our functions if they have all registered */
	    if (_registered_function_count == dev->num_functions)
		    bind_functions(dev);

#endif
    pr_debug("%s : end\n", __func__);
	return 0;
}

static int android_setup_config(struct usb_configuration *c,
		const struct usb_ctrlrequest *ctrl);

static struct usb_configuration android_config_driver = {
	.label		= "android",
	.setup		= android_setup_config,
	.bConfigurationValue = 1,
	.bMaxPower	= 0xFA, /* 500ma */
};

static int android_setup_config(struct usb_configuration *c,
		const struct usb_ctrlrequest *ctrl)
{
	int i;
	int ret = -EOPNOTSUPP;

	for (i = 0; i < android_config_driver.next_interface_id; i++) {
		if (android_config_driver.interface[i]->setup) {
			ret = android_config_driver.interface[i]->setup(
				android_config_driver.interface[i], ctrl);
			if (ret >= 0)
				return ret;
		}
	}
	return ret;
}

static int product_has_function(struct android_usb_product *p,
		struct usb_function *f)
{
	char **functions = p->functions;
	int count = p->num_functions;
	const char *name = f->name;
	int i;

	for (i = 0; i < count; i++) {

    #ifdef CONFIG_FIH_PID_SWITCH
        if (!strcmp(name, *functions++))
		    return 1;
    #endif //FIH-StevenCPHuang_20110818-IRM1070_USB PID Switch  -00+}
	}
	return 0;
}

static int product_matches_functions(struct android_usb_product *p)
{
	struct usb_function		*f;
	list_for_each_entry(f, &android_config_driver.functions, list) {
		if (product_has_function(p, f) == !!f->disabled)
			return 0;
	}
	return 1;
}

static int get_product_id(struct android_dev *dev)
{
	struct android_usb_product *p = dev->products;
	int count = dev->num_products;
	int i;

    pr_debug("%s : begin\n", __func__);
	if (p) {
		for (i = 0; i < count; i++, p++) {
			if (product_matches_functions(p))
				return p->product_id;
		}
	}
    pr_debug("%s : end\n", __func__);
	/* use default product ID */
	return dev->product_id;
}

static int __devinit android_bind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct usb_gadget	*gadget = cdev->gadget;
	int			gcnum, id, product_id, ret;

    pr_debug("%s : begin\n", __func__);

	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */
	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_MANUFACTURER_IDX].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_PRODUCT_IDX].id = id;
	device_desc.iProduct = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_SERIAL_IDX].id = id;
#ifndef CONFIG_FIH_FTM
	device_desc.iSerialNumber = id;
#endif

	if (gadget_is_otg(cdev->gadget))
		android_config_driver.descriptors = otg_desc;

	/* register our configuration */
	ret = usb_add_config(cdev, &android_config_driver, android_bind_config);
	if (ret) {
		pr_err("%s: usb_add_config failed\n", __func__);
		return ret;
	}

	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum);
	else {
		/* gadget zero is so simple (for now, no altsettings) that
		 * it SHOULD NOT have problems with bulk-capable hardware.
		 * so just warn about unrcognized controllers -- don't panic.
		 *
		 * things like configuration and altsetting numbering
		 * can need hardware-specific attention though.
		 */
		pr_warning("%s: controller '%s' not recognized\n",
			longname, gadget->name);
		device_desc.bcdDevice = __constant_cpu_to_le16(0x9999);
	}

	usb_gadget_set_selfpowered(gadget);
	dev->cdev = cdev;
/* FIHTDC, UsbPorting { *///StevenCPHuang_20110812,USB porting

	if (desc_pid==0xC002)
    {
		product_id = 0xC002;
    }
	else if (desc_pid==0xC001)
    {
		(isMoto)? (product_id = 0x2DE7): (product_id = 0xC004);
    }
	else if (desc_pid == 0xC00A)
    {
		(isMoto)? (product_id = 0x2DEA): (product_id = 0xC000);
    }
	else
    {
		product_id = get_product_id(dev);
    }

    pr_debug("%s : product_id = 0x%x\n", __func__, product_id);

/* } FIHTDC, UsbPorting */
	device_desc.idProduct = __constant_cpu_to_le16(product_id);
	cdev->desc.idProduct = device_desc.idProduct;

    pr_debug("%s : end\n", __func__);
	return 0;
}

static struct usb_composite_driver android_usb_driver = {
	.name		= "android_usb",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.enable_function = android_enable_function,
};

void android_register_function(struct android_usb_function *f)
{
	struct android_dev *dev = _android_dev;

    pr_debug("%s : begin\n", __func__);
	pr_debug(KERN_INFO "android_register_function %s\n", f->name);

    // Disable usb port in recovery +++
	if (fih_read_usb_id_from_smem()==0xC002)
    {
        pr_debug("not supported in recovery mode\n");
        return;
    }
    // Disable usb port in recovery ---
	if (desc_pid==0xc002)
    {
	    if(strcmp(f->name, "diag"))
	    {
		    pr_debug("android_register_function this isnt diag\n");
		    return ;
	    }
	}
	list_add_tail(&f->list, &_functions);
	_registered_function_count++;

	/* bind our functions if they have all registered
	 * and the main driver has bound.
	 */
	//if (dev->config && _registered_function_count == dev->num_functions) {
	if (desc_pid==0xc002 || (dev->config && _registered_function_count == dev->num_functions))
    {
		bind_functions(dev);
        pr_debug("%s : dev->product_id = 0x%x\n", __func__, dev->product_id);
		android_set_default_product(dev->product_id);
	}
    pr_debug("%s : end\n", __func__);
}

//FIH-StevenCPHuang_20110818-IRM1070_USB PID Switch  -00+}
/**
 * android_set_function_mask() - enables functions based on selected pid.
 * @up: selected product id pointer
 *
 * This function enables functions related with selected product id.
 */
static void android_set_function_mask(struct android_usb_product *up)
{
    int index =0;
	struct usb_function *func;

    pr_debug("%s : begin\n", __func__);
	list_for_each_entry(func, &android_config_driver.functions, list) 
    {
		/* adb function enable/disable handled separetely */
//FIH-StevenCPHuang_20110824,disable for IRM1070_USB PID Switch  -00+{

        func->disabled = 1;
        for (index = 0; index < up->num_functions; index++) 
        {
            if (!strcmp(up->functions[index], func->name))
				func->disabled = 0;
        } 

        pr_debug("%s : desc_pid =0x%x\n", __func__, desc_pid);

        if (desc_pid==0xC000)
        {//
            if (strcmp(func->name, "usb_mass_storage") &&
                strcmp(func->name, "adb")&&
                strcmp(func->name, "diag")&&
                strcmp(func->name, "modem"))//open all when pid = c000
                func->disabled = 1;//if function isn't belong to C000 ,it will be removed out.
            else
                func->disabled = 0; //if function is belong to C000 ,it will be reserved.
            pr_debug("func->name = %s, func->disabled = %d\n", func->name, func->disabled);
        }
        else if (desc_pid==0xC00A)
        {
            if (strcmp(func->name, "usb_mass_storage") &&
                strcmp(func->name, "adb")&&
                strcmp(func->name, "modem"))//open all when pid = c000
                func->disabled = 1;//if function isn't belong to C00A ,it will be removed out.
            else
                func->disabled = 0;//if function is belong to C00A ,it will be reserved.
            pr_debug("func->name = %s, func->disabled = %d\n", func->name, func->disabled);
        }
        else if (desc_pid==0xC001)
        {
            if (tethering_enabled==false)
            {
                if (switch_port_enable)
                {
                    if (strcmp(func->name, "usb_mass_storage") &&
                        strcmp(func->name, "adb"))
                        func->disabled = 1;
                    else
                        func->disabled = 0;
                    pr_debug("func->name = %s, func->disabled = %d\n", func->name, func->disabled);
                }
                else
                {
                    if (strcmp(func->name, "usb_mass_storage")) 
                        func->disabled = 1;
                    else
                        func->disabled = 0;
                    pr_debug("func->name = %s, func->disabled = %d\n", func->name, func->disabled);
                }
            }
	    }
#ifdef CONFIG_FIH_FTM
        else if (desc_pid==0xC003)
        {
            if (strcmp(func->name, "modem"))
                func->disabled = 1;
            else
                func->disabled = 0; 
            pr_debug("func->name = %s, func->disabled = %d\n", func->name, func->disabled);
        }
#endif
	}
    pr_debug("%s : end\n", __func__);
//FIH-StevenCPHuang_20110824,disable for IRM1070_USB PID Switch  -00+}
}

/**
 * android_set_defaut_product() - selects default product id and enables
 * required functions
 * @product_id: default product id
 *
 * This function selects default product id using pdata information and
 * enables functions for same.
*/
static void android_set_default_product(int pid)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_product *up = dev->products;
	int index;

    pr_debug("%s : begin , desc_pid = 0x%x, pid = 0x%x\n", __func__, desc_pid, pid);
	for (index = 0; index < dev->num_products; index++, up++) {
		if (pid == up->product_id)
			break;
	}
	android_set_function_mask(up);
    pr_debug("%s : end , desc_pid = 0x%x, pid = 0x%x\n", __func__, desc_pid, pid);
}

/**
 * android_config_functions() - selects product id based on function need
 * to be enabled / disabled.
 * @f: usb function
 * @enable : function needs to be enable or disable
 *
 * This function selects first product id having required function.
 * RNDIS/MTP function enable/disable uses this.
*/
#ifdef CONFIG_USB_ANDROID_RNDIS
static void android_config_functions(struct usb_function *f, int enable)
{
	struct android_dev *dev = _android_dev;
	struct android_usb_product *up = dev->products;
	int index;
    
	#ifdef CONFIG_FIH_PID_SWITCH//FIH-StevenCPHuang_20110824,add for IRM1070_USB PID Switch  -00+{
	char **functions;
	#endif

    pr_debug("%s : begin\n", __func__);
    //FIH-StevenCPHuang_20110824 add for IRM1070_USB PID Switch  -00+}
	/* Searches for product id having function */
	if (enable) 
    {
		for (index = 0; index < dev->num_products; index++, up++) 
        {
#ifdef CONFIG_FIH_PID_SWITCH//FIH-StevenCPHuang_20110824,IRM1070_USB PID Switch  -00+{
			functions = up->functions;
			if (!strcmp(*functions, f->name))
				break;
#endif//FIH-StevenCPHuang_20110824,IRM1070_USB PID Switch  -00+}
		}
		android_set_function_mask(up);
	} 
    else
    {
		android_set_default_product(dev->product_id);
    }
    pr_debug("%s : end\n", __func__);
}
#endif

#ifdef CONFIG_FIH_PID_SWITCH
int android_enable_function(struct usb_function *f, int enable)
{
	struct android_dev *dev = _android_dev;
	int disable = !enable;
	int product_id;

    pr_debug("%s : begin\n", __func__);
    pr_info_ratelimited("%s: %s %s\n", __func__, enable ? "enable" : "disable", f->name);

	//desc_pid = orig_pid;//unsigned short)fih_read_usb_id_from_smem();
#ifdef CONFIG_FIH_FTM
    desc_pid = 0xC003;
#else
    desc_pid = fih_read_usb_id_from_smem();
#endif

//	pid_ptr= &desc_pid;
	pr_debug("%s: desc_pid=0x%x, f->disable = %d, !!(f->disabled) = %d ,disable = %d\n", 
              __func__, desc_pid, f->disabled, !!(f->disabled), disable);

	if (!!(f->disabled )!= disable) 
    {// if (disable==0) present as function enable

        pr_debug("%s : !!(f->disabled )!= disable\n", __func__);
	    if (!strcmp(f->name, "diag"))
        {
            desc_pid = 0xC000;
        }
//	if ((f->disabled )!= disable) {// if (disable==0) present as function enable //StevenCPHuang_20110823,roll back the above 
		f->disabled = disable;//original disable=1 then change to 0(enable)
	//	usb_function_set_enabled(f, !disable);
//WilsonWHLee porting from 7069 --
//#ifdef CONFIG_FIH_PID_SWITCH/This method could confirm whether our Kconfig_FIH is modified successfully or not
//#error fail/
//#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
		if (!strcmp(f->name, "rndis")) 
        {
       		struct usb_function		*func; 	//WilsonWHLee porting from 7069 ++
			/* We need to specify the COMM class in the device descriptor
			 * if we are using RNDIS.
			 */
            pr_debug("%s : rndis\n", __func__);
			if (enable) 
            {
#ifdef CONFIG_USB_ANDROID_RNDIS_WCEIS
				dev->cdev->desc.bDeviceClass = USB_CLASS_MISC;
				dev->cdev->desc.bDeviceSubClass      = 0x02;
				dev->cdev->desc.bDeviceProtocol      = 0x01;
#else
				dev->cdev->desc.bDeviceClass = USB_CLASS_COMM;
#endif			   
                list_for_each_entry(func, &android_config_driver.functions, list) 
                {
                    if (strcmp(func->name, "rndis") &&
                    	strcmp(func->name, "adb")) 
                    	func->disabled = 1;//if function isn't belong to C001 ,it will be removed out.
                    else
                    	func->disabled = 0;//if function is belong to C001 ,it will be reserved.
                }
                keep_desc_pid = desc_pid;
                tethering_enabled = true;
			} 
            else 
            {
				dev->cdev->desc.bDeviceClass = USB_CLASS_PER_INTERFACE;
				dev->cdev->desc.bDeviceSubClass      = 0;
				dev->cdev->desc.bDeviceProtocol      = 0;
                desc_pid = keep_desc_pid;
                tethering_enabled = false;
			}
//			pr_debug("StevenCPHuang_RNDIS_3\n");//StevenCPHuang_20110811,add for PID Switch 
			android_config_functions(f, enable);
		}
        else 
#endif 
        {
        		//if(desc_pid == 0xc002)
        		//	product_id = 0xc000;
        		//else
            pr_debug("%s : now desc_pid = 0x%x\n", __func__, desc_pid);

            if (tethering_enabled) return 0;

		    if (desc_pid==0xC002)
            {
                struct usb_function	*func;
            	list_for_each_entry(func, &android_config_driver.functions, list) 
                {
		    		if (strcmp(func->name, "diag")) //only diag enabled when pid = c002
		    			func->disabled = 1;
		    		else
		    			func->disabled = 0;
		    	    pr_debug("func->name = %s func->disabled = %d\n", func->name, func->disabled);
		    	}
	   	    }
	   	    else if (desc_pid==0xC001)
            { 
	    		if (switch_port_enable)
                {
	    	   	    struct usb_function	*func;
                	list_for_each_entry(func, &android_config_driver.functions, list) 
                    {
		    		    if (strcmp(func->name, "usb_mass_storage") &&
		    		    	strcmp(func->name, "adb"))//only ums adb enabled when pid = c001
		    		    	func->disabled = 1;
		    		    else
		    		    	func->disabled = 0;
		    		    pr_debug("func->name = %s func->disabled = %d\n", func->name, func->disabled);
		        	}
	        	}
	        	else
                {
	    			struct usb_function	*func;
               	    list_for_each_entry(func, &android_config_driver.functions, list) 
                    {
                		//pr_debug("func->name =%s func->disabled=%d\n",func->name,func->disabled);
		    			if (strcmp(func->name, "usb_mass_storage"))//only ums enabled when pid = c004
		    				func->disabled = 1;
		    			else
		    				func->disabled = 0;
		    		    pr_debug("func->name = %s func->disabled = %d\n", func->name, func->disabled);
		         	}
			    }
	    	}
		    else if (desc_pid==0xC000)
            {
	    		struct usb_function	*func;
           		list_for_each_entry(func, &android_config_driver.functions, list) 
                {
		    		if (strcmp(func->name, "usb_mass_storage") &&
		    			strcmp(func->name, "adb")&&
		    			strcmp(func->name, "diag")&&
		    			strcmp(func->name, "modem"))//open all when pid = c000
		    			func->disabled = 1;
		    		else
		    			func->disabled = 0;
		    	    pr_debug("func->name = %s func->disabled = %d\n", func->name, func->disabled);
		    	}
		    }
            else if (desc_pid==0xC00A)
            {
	    		struct usb_function	*func;
           		list_for_each_entry(func, &android_config_driver.functions, list) 
                {
		    		if (strcmp(func->name, "usb_mass_storage") &&
		    			strcmp(func->name, "adb")&&
		    			strcmp(func->name, "modem"))//open all when pid = c000
		    			func->disabled = 1;
		    		else
		    			func->disabled = 0;
		    	    pr_debug("func->name = %s func->disabled = %d\n", func->name, func->disabled);
		    	}
	    	}
#ifdef CONFIG_FIH_FTM
            else if (desc_pid==0xC003)
            {
                struct usb_function *func;
                list_for_each_entry(func, &android_config_driver.functions, list)
                {
                    if (strcmp(func->name, "modem"))//open all when pid = c003
                        func->disabled = 1;
                    else
                        func->disabled = 0;
		    	    pr_debug("func->name = %s func->disabled = %d\n", func->name, func->disabled);
                }
            }
#endif

        }
#ifdef CONFIG_USB_ANDROID_MTP
		if (!strcmp(f->name, "mtp"))
			android_config_functions(f, enable);
#endif

		product_id = get_product_id(dev);
		pr_debug("%s :  product_id = 0x%x\n", __func__, product_id);

		device_desc.idProduct = __constant_cpu_to_le16(product_id);
		if (dev->cdev)
			dev->cdev->desc.idProduct = device_desc.idProduct;
		//if(isMoto){
		//    if(product_id != 0x2DE7)
		        usb_composite_force_reset(dev->cdev);
		//}
		//else{
		//	if(product_id != 0xc004)//0x2DE7)
		//        usb_composite_force_reset(dev->cdev);
		//}
	}
    pr_debug("%s : end\n", __func__);
	return 0;
}
#endif

#ifdef CONFIG_DEBUG_FS
static int android_debugfs_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t android_debugfs_serialno_write(struct file *file, const char
				__user *buf,	size_t count, loff_t *ppos)
{
	char str_buf[MAX_STR_LEN];

	if (count > MAX_STR_LEN)
		return -EFAULT;

	if (copy_from_user(str_buf, buf, count))
		return -EFAULT;

	memcpy(serial_number, str_buf, count);

	if (serial_number[count - 1] == '\n')
		serial_number[count - 1] = '\0';

	strings_dev[STRING_SERIAL_IDX].s = serial_number;

	return count;
}
const struct file_operations android_fops = {
	.open	= android_debugfs_open,
	.write	= android_debugfs_serialno_write,
};

struct dentry *android_debug_root;
struct dentry *android_debug_serialno;

static int android_debugfs_init(struct android_dev *dev)
{
	android_debug_root = debugfs_create_dir("android", NULL);
	if (!android_debug_root)
		return -ENOENT;

	android_debug_serialno = debugfs_create_file("serial_number", 0220,
						android_debug_root, dev,
						&android_fops);
	if (!android_debug_serialno) {
		debugfs_remove(android_debug_root);
		android_debug_root = NULL;
		return -ENOENT;
	}
	return 0;
}

static void android_debugfs_cleanup(void)
{
       debugfs_remove(android_debug_serialno);
       debugfs_remove(android_debug_root);
}
#endif

static int msm_read_usb_p_string_from_nv8053(char* ret_buf, int buf_len)
{
    uint32_t smem_proc_comm_oem_cmd1 = PCOM_CUSTOMER_CMD1;
    uint32_t smem_proc_comm_oem_data1 = SMEM_PROC_COMM_OEM_NV_READ;
    uint32_t smem_proc_comm_oem_data2= NV_USB_PRODUCT_STRING;
    uint8_t buf[128];   // 32 x 4 = 128 bytes
    int rc = 0;

    rc = msm_proc_comm_oem(smem_proc_comm_oem_cmd1, &smem_proc_comm_oem_data1, (unsigned *)buf, &smem_proc_comm_oem_data2);
    if (rc==0)
    {
        pr_debug("%s : [p string=%s]\n", __func__, (char *) buf);
        // At least the fisrt character should be ASCCI visible symbol or letter
        if (buf[0]>=32 && buf[0]<=126)
        {
            memcpy(ret_buf , buf , buf_len);
        }
        else
        {
            pr_debug("%s : Not assci string\n", __func__);
        }
    }
    else
    {
        pr_debug("%s : nv8053 doesn't exist\n", __func__);
    }
    return rc;
}

static int __devinit android_probe(struct platform_device *pdev)
{
	struct android_usb_platform_data *pdata = pdev->dev.platform_data;
	struct android_dev *dev = _android_dev;
	int result;

#ifdef CONFIG_FIH_PID_SWITCH//FIH-StevenCPHuang_20110818-IRM1070_USB PID Switch  -00+{

	unsigned short pid = 0; //StevenCPHuang_20110818,PID switch
	char product_str[21]; //the length is related to board file
	char manufacturer_str[21];

    pr_debug("%s : begin", __func__);
	//pid = 0xC001;//(unsigned short)fih_read_usb_id_from_smem(); //get pid value from share memory
#ifdef CONFIG_FIH_FTM
	pid = 0xC003;//(unsigned short)fih_read_usb_id_from_smem(); //get pid value from share memory
#else
	pid = (unsigned short)fih_read_usb_id_from_smem();
#endif
	desc_pid = pid;
	pr_debug("%s : pid = 0x%x",__func__, pid);

	if (pid==0)
    {
		pid = 0xC000;
    }

	pr_debug("%s: isMoto = %d\n", __func__, isMoto);

	if (isMoto && pid != 0xC002)
    {
		snprintf(manufacturer_str , 9 ,"Motorola");
		memcpy(pdata->manufacturer_name , manufacturer_str , 21);
        msm_read_usb_p_string_from_nv8053(product_str, 21);
		memcpy(pdata->product_name , product_str , 21);
		pdata->vendor_id = 0x22B8;

		if (desc_pid==0xC000)
        {
  		    pdata->product_id = 0x2DE6;
        }
		else if (desc_pid==0xC00A)
        {
			pdata->product_id = 0x2DEA;
        }
#ifdef CONFIG_FIH_FTM
        else if (desc_pid==0xC003)
        {
			pdata->product_id = 0x2DE5;
        }
#endif
		pdata->num_products = ARRAY_SIZE(usb_products);
    	pdata->products = usb_products;
	}
   	// Disable diag port in recovery +++
	if (pid==0xC002)
   	{
        pr_debug("android_probe: not supporting in recovery mode"); 
       	return -ENODEV;
   	}
  	// Disable diag port in recovery ---

	pr_debug(KERN_INFO "android_probe pdata: %p\n", pdata);

#endif//FIH-StevenCPHuang_20110818-IRM1070_USB PID Switch  -00+}

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	result = pm_runtime_get(&pdev->dev);
	if (result < 0) {
		dev_err(&pdev->dev,
			"Runtime PM: Unable to wake up the device, rc = %d\n",
			result);
		return result;
	}

	if (pdata) {
		dev->products = pdata->products;
		dev->num_products = pdata->num_products;
		dev->functions = pdata->functions;
		dev->num_functions = pdata->num_functions;
		if (pdata->vendor_id)
			device_desc.idVendor =
				__constant_cpu_to_le16(pdata->vendor_id);
		if (pdata->product_id) 
        {
#ifdef CONFIG_FIH_PID_SWITCH  //FIH-StevenCPHuang_20110824,add for IRM1070_USB PID Switch  -00+{
			if (pid==0xC002) 
            {
				pdata->product_id = 0xC002;
            }
			else if (pid==0xC001)
            {
				if (isMoto)
					pdata->product_id = 0x2DE7;
				else
					pdata->product_id = 0xC001;           
           	}
#endif //FIH-StevenCPHuang_20110824 add for IRM1070_USB PID Switch  -00+}
			dev->product_id = pdata->product_id;

            pr_debug("%s : dev->product_id = 0x%x\n", __func__, dev->product_id);
			device_desc.idProduct =
				__constant_cpu_to_le16(pdata->product_id);
		}
		if (pdata->version)
			dev->version = pdata->version;

		if (pdata->product_name)
			strings_dev[STRING_PRODUCT_IDX].s = pdata->product_name;
		if (pdata->manufacturer_name)
			strings_dev[STRING_MANUFACTURER_IDX].s =
					pdata->manufacturer_name;
		if (pdata->serial_number)
			strings_dev[STRING_SERIAL_IDX].s = pdata->serial_number;
	}
#ifdef CONFIG_DEBUG_FS
	result = android_debugfs_init(dev);
	if (result)
		pr_debug("%s: android_debugfs_init failed\n", __func__);
#endif
    pr_debug("%s : end", __func__);
	return usb_composite_probe(&android_usb_driver, android_bind);
}

static int andr_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: suspending...\n");
	return 0;
}

static int andr_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: resuming...\n");
	return 0;
}

static struct dev_pm_ops andr_dev_pm_ops = {
	.runtime_suspend = andr_runtime_suspend,
	.runtime_resume = andr_runtime_resume,
};

static struct platform_driver android_platform_driver = {
	.driver = { .name = "android_usb", .pm = &andr_dev_pm_ops},
};
