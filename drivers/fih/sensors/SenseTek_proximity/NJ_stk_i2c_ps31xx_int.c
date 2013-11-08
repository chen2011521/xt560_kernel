
static int32_t init_all_setting_V1(void);

static int32_t init_all_setting_V1(void)
{

	if (software_reset()<0)
    {
        ERR("STK PS : error --> device not found\n");
        return 0;
    }
    enable_ps(0);
    enable_als(0);
    set_ps_slp(STK_PS_SLEEP_TIME);
    set_ps_gc(0x09);
    set_ps_it(STK_PS_INTEGRAL_TIME);
    set_ps_led_driving_current(STK_PS_IRLED_DRIVING_CURRENT);
    set_als_gain(0x01); // x2
    //set_als_it(0x02); // x4
    set_als_it(0x0); // x1
    set_ps_thd_h(130);
    set_ps_thd_l(100);

    enable_ps_int(1);
    enable_als_int(1);
//	}
    return 1;
}

static void stk_oss_work_V1(struct work_struct *work)
{

    int32_t ret,reading;
    uint8_t disable_flag = 0;
    STK_LOCK(1);
    ret = get_status_reg();

	if(ret < 0)
	{
		STK_LOCK(0);
		ERR("stk_oss_work:get_status_reg fail, ret=%d", ret);
		msleep(30);
		enable_irq(pStkPsData->irq);
		return; 		
	}
	INFO("stk_oss_work:get_status_reg=%d", ret);
    if (ret&STK_PS_STATUS_ALS_INT_FLAG_MASK)
    {
		disable_flag = STK_PS_STATUS_ALS_INT_FLAG_MASK;
        reading = get_als_reading();
#ifndef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
        nLuxIndex = get_lux_interval_index(reading);
        set_als_thd_h(code_threshold_table[nLuxIndex]);
        set_als_thd_l(code_threshold_table[nLuxIndex-1]);
#else
        set_als_new_thd_by_reading(reading);
#endif //CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
        als_report_event(pStkPsData->als_input_dev,alscode2lux(reading));

    }
    if (ret&STK_PS_STATUS_PS_INT_FLAG_MASK)
    {
		
        reading = get_ps_reading();
        INFO("%s : ps code = %d\n",__func__,reading);
#ifdef CONFIG_STK_PS_ENGINEER_TUNING
        if (reading>=ps_code_high_thd)
#else
        if (reading>=CONFIG_STK_PS_CODE_HIGH_THRESHOLD)
#endif
        {
			disable_flag |= STK_PS_STATUS_PS_INT_FLAG_MASK;
            ps_report_event(pStkPsData->ps_input_dev,0);
        }
#ifdef CONFIG_STK_PS_ENGINEER_TUNING        
        else if (reading<ps_code_low_thd )
#else
		else  if (reading<CONFIG_STK_PS_CODE_LOW_THRESHOLD)
#endif        
        {	
			disable_flag |= STK_PS_STATUS_PS_INT_FLAG_MASK;		
            ps_report_event(pStkPsData->ps_input_dev,1);
        //    ps_report_event(pStkPsData->ps_input_dev,9);
		}
		else
			msleep(10);
    }
    
    reset_int_flag(ret,disable_flag);
    msleep(1);
    enable_irq(pStkPsData->irq);
	
 
    STK_LOCK(0);
}



static int stk_ps_probe_V1(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
    int err;
    struct stkps31xx_data* ps_data;
    int irq;
	struct FIH_lightsensor *data = (struct FIH_lightsensor *)client->dev.platform_data;
	
	 printk("STKPS -- %s: I2C is probing (%s)\n", __func__,id->name);
  
	client->addr = 0x90>>1;	// true i2c address	
	printk("stk_ps_probe -- %s:change i2c address to 0x48 successfully\n", __func__);
	
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
    {
        printk("STKPS -- No Support for I2C_FUNC_SMBUS_BYTE_DATA\n");
        return -ENODEV;
    }
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA))
    {
        printk("STKPS -- No Support for I2C_FUNC_SMBUS_WORD_DATA\n");
        return -ENODEV;
    }

#ifdef DETECT_CM3623		
	if(detect_cm3623(client) > 0)
	{
		is_cm3623_exist = 1;
		INFO("stk_i2c_ps31xx_init:is_cm3623_exist = 1\n");
	
		return -EINVAL;				
	}
	else
	{
		is_cm3623_exist = 0;
		INFO("stk_i2c_ps31xx_init:is_cm3623_exist = 0\n");
	}
#endif	/*#ifdef DETECT_CM3623*/			
	
	
#ifdef DETECT_CM3623
    if (id->driver_data == 0 && is_cm3623_exist == 0)
#else
    if (id->driver_data == 0)
	
#endif
    {

        ps_data = kzalloc(sizeof(struct stkps31xx_data),GFP_KERNEL);
        ps_data->client = client;
        i2c_set_clientdata(client,ps_data);
        mutex_init(&stkps_io_lock);

        ps_data->als_input_dev = input_allocate_device();
        ps_data->ps_input_dev = input_allocate_device();
        if ((ps_data->als_input_dev==NULL)||(ps_data->ps_input_dev==NULL))
        {
            if (ps_data->als_input_dev==NULL)
                input_free_device(ps_data->als_input_dev);
            if (ps_data->ps_input_dev==NULL)
                input_free_device(ps_data->ps_input_dev);
            ERR("%s: could not allocate input device\n", __func__);
            mutex_destroy(&stkps_io_lock);
            kfree(ps_data);
            return -ENOMEM;
        }
        ps_data->als_input_dev->name = ALS_NAME;
        ps_data->ps_input_dev->name = PS_NAME;
        set_bit(EV_ABS, ps_data->als_input_dev->evbit);
        set_bit(EV_ABS, ps_data->ps_input_dev->evbit);
        input_set_abs_params(ps_data->als_input_dev, ABS_MISC, 0, alscode2lux((1<<16)-1), 0, 0);
        input_set_abs_params(ps_data->ps_input_dev, ABS_DISTANCE, 0,1, 0, 0);
        err = input_register_device(ps_data->als_input_dev);
        if (err<0)
        {
            ERR("STK PS : can not register als input device\n");
            mutex_destroy(&stkps_io_lock);
            input_free_device(ps_data->als_input_dev);
            input_free_device(ps_data->ps_input_dev);
            kfree(ps_data);

            return err;
        }
        INFO("STK PS : register als input device OK\n");
        err = input_register_device(ps_data->ps_input_dev);
        if (err<0)
        {
            ERR("STK PS : can not register ps input device\n");
            input_unregister_device(ps_data->als_input_dev);
            mutex_destroy(&stkps_io_lock);
            input_free_device(ps_data->als_input_dev);
            input_free_device(ps_data->ps_input_dev);
            kfree(ps_data);
            return err;

        }
        pStkPsData = ps_data;
        ps_data->ps_delay = PS_ODR_DELAY;
        ps_data->als_delay = ALS_ODR_DELAY;

        stk_oss_work_queue = create_workqueue("stk_oss_wq");
        INIT_WORK(&ps_data->work, stk_oss_work_V1);
        enable_als(0);
        enable_ps(0);
#if ADDITIONAL_GPIO_CFG // Additional GPIO CFG
    INFO("gpio pin # = %d\n",(int)YAS_STK3101_PS_GPIO_OUT);		
		
    err = gpio_tlmm_config(GPIO_CFG(YAS_STK3101_PS_GPIO_OUT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 
                           GPIO_CFG_ENABLE);
    if (err) {
        printk(KERN_ERR "[stk3101]%s: YAS_STK3101_PS_GPIO_OUT config failed : gpio_tlmm_config=%d\n", 
               __func__, err);     
        return -EIO;
    }	
    	
     gpio_request(YAS_STK3101_PS_GPIO_OUT,"EINT");         /* Request for gpio pin */
     irq = MSM_GPIO_TO_INT(YAS_STK3101_PS_GPIO_OUT);
     client->irq = irq;
    //	gpio_direction_input(EINT_GPIO);		
		
#endif // Additional GPIO CFG
        err = request_irq(client->irq, stk_oss_irq_handler, STK_IRQF_MODE, DEVICE_NAME, ps_data);
        if (err < 0) {
            WARNING("%s: request_irq(%d) failed for (%d)\n",
                __func__, client->irq, err);
            return err;
        }
        pStkPsData->irq = client->irq;
#ifndef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
        init_code_threshold_table();
#endif


        if (!init_all_setting_V1())
        {
            input_unregister_device(pStkPsData->als_input_dev);
            input_unregister_device(pStkPsData->ps_input_dev);
            input_free_device(pStkPsData->als_input_dev);
            input_free_device(pStkPsData->ps_input_dev);
            kfree(pStkPsData);
            pStkPsData = NULL;
            return -EINVAL;
        } 
				
		als_transmittance = (int32_t)(data->setting);
		INFO("change transmittance to %d\n",als_transmittance);
        return 0;
    }

    return -EINVAL;
}



