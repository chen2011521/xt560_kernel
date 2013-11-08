/** 
* Battery/Charger Driver
*****************************************************/

#include <linux/delay.h>
/* Debbie, 2011/09/15 { */
/* add battery level*/
#include <linux/power_supply.h>
#include <mach/msm_battery.h>
/* Debbie, 2011/09/15 } */
#include "fih_battery.h"

// FIH,Debbie,2011/08/12,port battery driver +[
#ifdef CONFIG_W1_MASTER_GPIO
#include <linux/w1-gpio.h>	// DIV2-SW2-BSP-IRM-W1-MASTER

#define GPIO_W1_MASTER	130
#define GPIO_W1_MASTER_V1	26 //IRMPrime

//FIH-NJ-BSP, Jerry add, 2011/12/23 +++
VOLT_TO_PERCENT *p_Volt2percent;
extern int check_battery_type(void);
//FIH-NJ-BSP, Jerry add, 2011/12/23 ---

struct w1_gpio_platform_data w1_master_pdata = {
	.pin	= GPIO_W1_MASTER,
	.is_open_drain	= 1,
};

static struct platform_device w1_master_dev = {
	.name   = "w1-gpio",
	.id     = -1,
	.dev    = {
		.platform_data = &w1_master_pdata,
		},
};

static int config_w1_master_gpio(void *hdl)
{
	int rc = 0;
	
	rc = gpio_tlmm_config( GPIO_CFG( GPIO_W1_MASTER, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA ), GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s not able to get gpio %d\n", __func__, GPIO_W1_MASTER);

    return 0;
}

static int bq2024_reset(void *hdl)
{
	int ret;
	
	ret = gpio_request(GPIO_W1_MASTER, "w1");
	if (ret)
		pr_err("%s Request gpio %d failed.\n", __func__, GPIO_W1_MASTER);	
	
	//Pull a reset signal
	gpio_set_value(GPIO_W1_MASTER, 0);
	udelay(480); /*this should be more than 480us*/
	gpio_set_value(GPIO_W1_MASTER, 1);
	udelay(15);  /*this should be 15us~60us*/

	//Read whether slave has a presence pulse
	if (gpio_direction_input(GPIO_W1_MASTER))
		pr_info("w1_reset_bq2024: gpio_direction_input %d fails!!!\r\n", GPIO_W1_MASTER);
	udelay(60);   /*slave will pull low 60us~240us*/

	ret = gpio_get_value(GPIO_W1_MASTER) ? 1 : 0;
	udelay(480);  /*Do not small this value*/
	
	gpio_direction_output(GPIO_W1_MASTER, 1);
	gpio_free(GPIO_W1_MASTER);
	
	return ret;
}

//IRMPrime project

struct w1_gpio_platform_data w1_master_pdata_v1 = {
	.pin	= GPIO_W1_MASTER_V1,
	.is_open_drain	= 1,
};

static struct platform_device w1_master_dev_v1 = {
	.name   = "w1-gpio",
	.id     = -1,
	.dev    = {
		.platform_data = &w1_master_pdata_v1,
		},
};

static int config_w1_master_gpio_v1(void *hdl)
{
	int rc = 0;
	
	rc = gpio_tlmm_config( GPIO_CFG( GPIO_W1_MASTER_V1, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA ), GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s not able to get gpio %d\n", __func__, GPIO_W1_MASTER_V1);

     return 0;
}

static int bq2024_reset_v1(void *hdl)
{
	int ret;
	
	ret = gpio_request(GPIO_W1_MASTER_V1, "w1");
	if (ret)
		pr_err("%s Request gpio %d failed.\n", __func__, GPIO_W1_MASTER_V1);	
	
	//Pull a reset signal
	gpio_set_value(GPIO_W1_MASTER_V1, 0);
	udelay(480); /*this should be more than 480us*/
	gpio_set_value(GPIO_W1_MASTER_V1, 1);
	udelay(15);  /*this should be 15us~60us*/

	//Read whether slave has a presence pulse
	if (gpio_direction_input(GPIO_W1_MASTER_V1))
		pr_info("w1_reset_bq2024: gpio_direction_input %d fails!!!\r\n", GPIO_W1_MASTER_V1);
	udelay(60);   /*slave will pull low 60us~240us*/

	ret = gpio_get_value(GPIO_W1_MASTER_V1) ? 1 : 0;
	udelay(480);  /*Do not small this value*/
	
	gpio_direction_output(GPIO_W1_MASTER_V1, 1);
	gpio_free(GPIO_W1_MASTER_V1);
	
	return ret;
}
#endif	/* !CONFIG_W1_MASTER_GPIO */


static VOLT_TO_PERCENT g_Volt2Percent_v1[11] =
{
#if 0
/* IvanCHTang, 20110919 {*/
/* Bettery profile: BP6X */
  { 3450, 0},	   // empty,    Rx Table
  { 3567, 10},    // level 1
  { 3630, 20},    // level 2
  { 3665, 30},    // level 3
  { 3693, 40},    // level 4
  { 3732, 50},    // level 5
  { 3783, 60},    // level 6
  { 3849, 70},    // level 7
  { 3915, 80},    // level 8
  { 4000, 90},    // level 9
  { 4100, 100},   // full
#else
//use original mapping table
  { 3400, 0},	   // empty,    Rx Table
  { 3562, 10},    // level 1
  { 3616, 20},    // level 2
  { 3651, 30},    // level 3
  { 3678, 40},    // level 4
  { 3711, 50},    // level 5
  { 3756, 60},    // level 6
  { 3818, 70},    // level 7
  { 3887, 80},    // level 8
  { 3973, 90},    // level 9
  { 4100, 100},   // full
#endif
};

static VOLT_TO_PERCENT g_Volt2Percent_v2[11] =
{
	{3400, 0},  	 // empty,    Rx Table
	{3577, 10},      // level 1	
	{3644, 20},      // level 2	
	{3682, 30},      // level 3	
	{3717, 40},      // level 4	
	{3765, 50},      // level 5	
	{3832, 60},      // level 6	
	{3921, 70},      // level 7	
	{4027, 80},      // level 8	
	{4137, 90},      // level 9	
	{4300, 100},     // full
};


static u32 msm_calculate_batt_capacity_v1(u32 current_voltage);
static u32 msm_calculate_batt_capacity_v2(u32 current_voltage);
static u32 msm_calculate_batt_capacity_v3(u32 current_voltage);

static struct msm_psy_batt_pdata msm_psy_batt_data_v1 = {
	.voltage_min_design     =  3450,
	.voltage_max_design     =  4100,
	.avail_chg_sources      = AC_CHG | USB_CHG ,
	.batt_technology        = POWER_SUPPLY_TECHNOLOGY_LION,
	.calculate_capacity     = &msm_calculate_batt_capacity_v1,
};

static struct msm_psy_batt_pdata msm_psy_batt_data_v2 = {
	.voltage_min_design     = 3400,
	.voltage_max_design     = 4300,
	.avail_chg_sources      = AC_CHG | USB_CHG ,
	.batt_technology        = POWER_SUPPLY_TECHNOLOGY_LION,
	.calculate_capacity     = &msm_calculate_batt_capacity_v2,
};

static struct msm_psy_batt_pdata msm_psy_batt_data_v3 = {
	.voltage_min_design     = 3400,
	.voltage_max_design     = 4300,
	.avail_chg_sources      = AC_CHG | USB_CHG ,
	.batt_technology        = POWER_SUPPLY_TECHNOLOGY_LION,
	.calculate_capacity     = &msm_calculate_batt_capacity_v3,
};


static u32 msm_calculate_batt_capacity_v1(u32 current_voltage)
{
	int Volt_pec=0, pec_return;
	int iC;
	for(iC=0;iC<11;iC++)
	{
		if(current_voltage <= g_Volt2Percent_v1[iC].dwVolt)
			break;
	}
	if(iC==0)
		Volt_pec=0;
	else if(iC==11)
		Volt_pec=100;
	else if((iC>0)&&(iC<11))
	{
		Volt_pec=g_Volt2Percent_v1[iC-1].dwPercent +
			( current_voltage -g_Volt2Percent_v1[iC-1].dwVolt) * ( g_Volt2Percent_v1[iC].dwPercent -g_Volt2Percent_v1[iC-1].dwPercent)/( g_Volt2Percent_v1[iC].dwVolt -g_Volt2Percent_v1[iC-1].dwVolt);
	}
	printk(KERN_INFO "%s(): Vbat=%d, Volt_pec=%d \r\n", __func__, current_voltage, Volt_pec);
	/* Divide battery level (0~100) into 10 pieces*/
	if (Volt_pec == 0)
		pec_return = 0;
	else if (Volt_pec <= 10)
		pec_return = 5;
	else if (Volt_pec <= 15)
		pec_return = 10;
	else if (Volt_pec <= 20)
		pec_return = 15;
	else if (Volt_pec <= 29)
		pec_return = 20;
	else if (Volt_pec <= 38)
		pec_return = 30;
	else if (Volt_pec <= 47)
		pec_return = 40;
	else if (Volt_pec <= 56)
		pec_return = 50;
	else if (Volt_pec <= 65)
		pec_return = 60;
	else if (Volt_pec <= 74)
		pec_return = 70;
	else if (Volt_pec <= 85)
		pec_return = 80;
	else if (Volt_pec <= 99)
		pec_return = 90;
	else if (Volt_pec == 100)
		pec_return = 100;
	else
		return -1;
	
	printk(KERN_INFO "%s(): pec_return=%d\r\n", __func__, pec_return);
	
	return pec_return;
}

static u32 msm_calculate_batt_capacity_v2(u32 current_voltage)
{
	int Volt_pec=0, pec_return;
	int iC;
	for(iC=0;iC<11;iC++)
	{
		if(current_voltage <= g_Volt2Percent_v2[iC].dwVolt)
			break;
	}
	if(iC==0)
		Volt_pec=0;
	else if(iC==11)
		Volt_pec=100;
	else if((iC>0)&&(iC<11))
	{
		Volt_pec=g_Volt2Percent_v2[iC-1].dwPercent +
			( current_voltage -g_Volt2Percent_v2[iC-1].dwVolt) * ( g_Volt2Percent_v2[iC].dwPercent -g_Volt2Percent_v2[iC-1].dwPercent)/( g_Volt2Percent_v2[iC].dwVolt -g_Volt2Percent_v2[iC-1].dwVolt);
	}
	printk(KERN_INFO "%s(): Vbat=%d, Volt_pec=%d \r\n", __func__, current_voltage, Volt_pec);
	/* Divide battery level (0~100) into 10 pieces*/
	if (Volt_pec == 0)
		pec_return = 0;
	else if (Volt_pec <= 10)
		pec_return = 5;
	else if (Volt_pec <= 15)
		pec_return = 10;
	else if (Volt_pec <= 20)
		pec_return = 15;
	else if (Volt_pec <= 29)
		pec_return = 20;
	else if (Volt_pec <= 38)
		pec_return = 30;
	else if (Volt_pec <= 47)
		pec_return = 40;
	else if (Volt_pec <= 56)
		pec_return = 50;
	else if (Volt_pec <= 65)
		pec_return = 60;
	else if (Volt_pec <= 74)
		pec_return = 70;
	else if (Volt_pec <= 85)
		pec_return = 80;
	else if (Volt_pec <= 96)
		pec_return = 90;
    //MOTO new requirement--add 99% battery level for power off charging mode.FIH-NJ Jerry, 2011/12/26 +++
    else if (Volt_pec <= 99)
		pec_return = 99;
    //MOTO new requirement--add 99% battery level for power off charging mode.FIH-NJ Jerry, 2011/12/26 ---
	else if (Volt_pec == 100)
		pec_return = 100;
	else
		return -1;
	
	printk(KERN_INFO "%s(): pec_return=%d\r\n", __func__, pec_return);
	
	return pec_return;
}

static u32 msm_calculate_batt_capacity_v3(u32 current_voltage)
{
	int Volt_pec=0, pec_return;
	int iC;
    p_Volt2percent = g_Volt2Percent_v1;
    if(check_battery_type() == BATTERY_TYPE_4V35)
    {
        p_Volt2percent = g_Volt2Percent_v2;
    }
    printk(KERN_INFO "%s(): p_Volt2percent[10].dwVolt(%d) \r\n", __func__, p_Volt2percent[10].dwVolt);
	for(iC=0;iC<11;iC++)
	{
		if(current_voltage <= p_Volt2percent[iC].dwVolt)
			break;
	}
	if(iC==0)
		Volt_pec=0;
	else if(iC==11)
		Volt_pec=100;
	else if((iC>0)&&(iC<11))
	{
		Volt_pec=p_Volt2percent[iC-1].dwPercent +
			( current_voltage -p_Volt2percent[iC-1].dwVolt) * ( p_Volt2percent[iC].dwPercent -p_Volt2percent[iC-1].dwPercent)/( p_Volt2percent[iC].dwVolt -p_Volt2percent[iC-1].dwVolt);
	}
	printk(KERN_INFO "%s(): Vbat=%d, Volt_pec=%d \r\n", __func__, current_voltage, Volt_pec);
	/* Divide battery level (0~100) into 10 pieces*/
	if (Volt_pec == 0)
		pec_return = 0;
	else if (Volt_pec <= 10)
		pec_return = 5;
	else if (Volt_pec <= 15)
		pec_return = 10;
	else if (Volt_pec <= 20)
		pec_return = 15;
	else if (Volt_pec <= 29)
		pec_return = 20;
	else if (Volt_pec <= 38)
		pec_return = 30;
	else if (Volt_pec <= 47)
		pec_return = 40;
	else if (Volt_pec <= 56)
		pec_return = 50;
	else if (Volt_pec <= 65)
		pec_return = 60;
	else if (Volt_pec <= 74)
		pec_return = 70;
	else if (Volt_pec <= 85)
		pec_return = 80;
	else if (Volt_pec <= 96)
		pec_return = 90;
    else if (Volt_pec <= 99)  //power off charging need 99% to display.
		pec_return = 99;
	else if (Volt_pec == 100)
		pec_return = 100;
	else
		return -1;
	
	printk(KERN_INFO "%s(): pec_return=%d\r\n", __func__, pec_return);
	
	return pec_return;
}

static struct platform_device msm_batt_device_v1 = {
	.name               = "msm-battery",
	.id                 = -1,
	.dev.platform_data  = &msm_psy_batt_data_v1,
};

static struct platform_device msm_batt_device_v2 = {
	.name               = "msm-battery",
	.id                 = -1,
	.dev.platform_data  = &msm_psy_batt_data_v2,
};

static struct platform_device msm_batt_device_v3 = {
	.name               = "msm-battery",
	.id                 = -1,
	.dev.platform_data  = &msm_psy_batt_data_v3,
};
/* Debbie, 2011/09/15 } */

int __init battery_init(void)
{
	/* Debbie, 2011/09/15 { */
	/* add battery level*/
	//platform_device_register(&msm_batt_device);
    printk(KERN_INFO"[%s] entry\n", __func__);

    REGISTER_DYN_PLATFORM_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_BATTERY, 
                                                   BATTERY_V1), 
                                 &msm_batt_device_v1);

    REGISTER_DYN_PLATFORM_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_BATTERY, 
                                                   BATTERY_V2), 
                                 &msm_batt_device_v2);
    REGISTER_DYN_PLATFORM_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_BATTERY, 
                                                   BATTERY_V3), 
                                 &msm_batt_device_v3);
	//(void)msm_batt_device;
	/* Debbie, 2011/09/15 } */
#ifdef CONFIG_W1_MASTER_GPIO
	//config_w1_master_gpio(); 
	//bq2024_reset();	
	//platform_device_register(&w1_master_dev);
//IRM
    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_BATTERY, BATTERY_V1),
		                   "BATTERY_V1", config_w1_master_gpio, __FILE__, __LINE__);

    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_BATTERY, BATTERY_V1),
		                   "BATTERY_V1", bq2024_reset, __FILE__, __LINE__);

    REGISTER_DYN_PLATFORM_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_BATTERY, 
                                                   BATTERY_V1), 
                                 &w1_master_dev);
//TBP TNQ
    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_BATTERY, BATTERY_V2),
		                   "BATTERY_V2", config_w1_master_gpio, __FILE__, __LINE__);

    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_BATTERY, BATTERY_V2),
		                   "BATTERY_V2", bq2024_reset, __FILE__, __LINE__);

    REGISTER_DYN_PLATFORM_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_BATTERY, 
                                                   BATTERY_V2), 
                                 &w1_master_dev);

//IRMPrime
    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_BATTERY, BATTERY_V3),
		                   "BATTERY_V3", config_w1_master_gpio_v1, __FILE__, __LINE__);

    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_BATTERY, BATTERY_V3),
		                   "BATTERY_V3", bq2024_reset_v1, __FILE__, __LINE__);

    REGISTER_DYN_PLATFORM_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_BATTERY, 
                                                   BATTERY_V3), 
                                 &w1_master_dev_v1);
#endif	/* !CONFIG_W1_MASTER_GPIO */
	
	return 0;
}
