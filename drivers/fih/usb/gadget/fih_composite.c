/*
fih 1090_usb driver porting,add for HWID control
*/
#ifdef CONFIG_FIH_PID_SWITCH //StevenCPHuang_20110824,add for PID Switch

struct device * dev_backup = NULL;
struct device * get_f_diag_dev(void)
{
    return dev_backup;
}

int fih_enable_Diag(struct device * pdev)
{
    char *buf = "1";
 //   printk("fih_enable_test_in\n");//
    enable_store(pdev, NULL, buf, 0);
 //   printk("fih_enable_test_out\n");//
	return 0;
}
#endif

int usb_add_function(struct usb_configuration *config,
		struct usb_function *function)
{
	struct usb_composite_dev	*cdev = config->cdev;
	int	value = -EINVAL;
	int index;
//         printk("StevenCPHuang_ usb_add_function\n"); //StevenCPHuang_20110824,add for debug

	DBG(cdev, "adding '%s'/%p to config '%s'/%p\n",
			function->name, function,
			config->label, config);

	if (!function->set_alt || !function->disable)
		goto done;

	index = atomic_inc_return(&cdev->driver->function_count);
	function->dev = device_create(cdev->driver->class, NULL,
		MKDEV(0, index), NULL, function->name);
//	printk("StevenCPHuang_ usb_add_function (function->dev)  = %p\n", (function->dev)); //StevenCPHuang_20110824,add for debug
	if (IS_ERR(function->dev))
		return PTR_ERR(function->dev);

	value = device_create_file(function->dev, &dev_attr_enable);
	if (value < 0) {
		device_destroy(cdev->driver->class, MKDEV(0, index));
		return value;
	}
	dev_set_drvdata(function->dev, function);

	function->config = config;
	list_add_tail(&function->list, &config->functions);

	/* REVISIT *require* function->bind? */
	if (function->bind) {
		value = function->bind(config, function);
		if (value < 0) {
			list_del(&function->list);
			function->config = NULL;
		}
	} else
		value = 0;

	/* We allow configurations that don't work at both speeds.
	 * If we run into a lowspeed Linux system, treat it the same
	 * as full speed ... it's the function drivers that will need
	 * to avoid bulk and ISO transfers.
	 */
	if (!config->fullspeed && function->descriptors)
		config->fullspeed = true;
	if (!config->highspeed && function->hs_descriptors)
		config->highspeed = true;

done:
	if (value)
		DBG(cdev, "adding '%s'/%p --> %d\n",
				function->name, function, value);
#ifdef CONFIG_FIH_PID_SWITCH//StevenCPHuang_20110824,add for PID Switch
    if(!dev_backup && !strcmp(function->name, "diag"))
    {
        dev_backup = function->dev;
        printk("dev_backup=%p", dev_backup);
    }
	return value;
#endif
}


