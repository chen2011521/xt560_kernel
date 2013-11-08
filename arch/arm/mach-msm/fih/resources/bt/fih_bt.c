/**
* Broadcom BlueTooth Driver
************************************************************/
#include <linux/delay.h>
#include "fih_bt.h"

//Thomas 2011.7.14 BT Porting ++[
#if defined(CONFIG_BROADCOM_BCM4330)
#define BT_MASK             0x01
#define FM_MASK             0x02

#define GPIO_BT_REG_ON      34
#define GPIO_BT_RST_N       32
#define GPIO_WL_REG_ON      33
#define BT_HOST_WAKE        27
#define HOST_WAKEUP_BT      31

#define GPIO_BTUART_RFR     43
#define GPIO_BTUART_CTS     44
#define GPIO_BTUART_RX      45
#define GPIO_BTUART_TX      46
#define GPIO_PCM_DIN        69
#define GPIO_PCM_DOUT       68
#define GPIO_PCM_SYNC       70
#define GPIO_PCM_BCLK       71

//static unsigned int bcm4330_power_status=0;

static struct platform_device bcm4330_bt_power_device = {
    .name = "bcm4330_bt_power",
    .id     = -1
};

/*static struct msm_gpio bt_config_power_on[] = {

    { GPIO_CFG(GPIO_BTUART_RFR, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA), "UART1DM_RFR" },
    { GPIO_CFG(GPIO_BTUART_CTS, 2, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,  GPIO_CFG_2MA), "UART1DM_CTS" },
    { GPIO_CFG(GPIO_BTUART_RX,  2, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,  GPIO_CFG_2MA), "UART1DM_RX" },
    { GPIO_CFG(GPIO_BTUART_TX,  2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,  GPIO_CFG_2MA), "UART1DM_TX" },
    
};*/

//static unsigned bt_config_power_on[] = {
	/*RFR*/
	//GPIO_CFG(43, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*CTS*/
	//GPIO_CFG(44, 2, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*RX*/
	//GPIO_CFG(45, 2, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*TX*/
	//GPIO_CFG(46, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
//};

static unsigned bt_config_pcm_on[] = {
	/*PCM_DOUT*/
	GPIO_CFG(68, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*PCM_DIN*/
	GPIO_CFG(69, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*PCM_SYNC*/
	GPIO_CFG(70, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	/*PCM_CLK*/
	GPIO_CFG(71, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};

static unsigned bt_config_pcm_off[] = {
	/*PCM_DOUT*/
	GPIO_CFG(68, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*PCM_DIN*/
	GPIO_CFG(69, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*PCM_SYNC*/
	GPIO_CFG(70, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/*PCM_CLK*/
	GPIO_CFG(71, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};
static struct msm_gpio bt_config_power_off[] = {
 
    { GPIO_CFG(GPIO_BTUART_RFR, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA), "UART1DM_RFR" },
    { GPIO_CFG(GPIO_BTUART_CTS, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA), "UART1DM_CTS" },
    { GPIO_CFG(GPIO_BTUART_RX,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA), "UART1DM_RX" },
    { GPIO_CFG(GPIO_BTUART_TX,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,  GPIO_CFG_2MA), "UART1DM_TX" },
    { GPIO_CFG(BT_HOST_WAKE,    0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,  	 GPIO_CFG_2MA), "BT_HOST_WAKE" },
    { GPIO_CFG(HOST_WAKEUP_BT,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,    GPIO_CFG_2MA), "BT_WAKE" }
};

static int bluetooth_power(int on)
{
    int rc=0;
    int pin = 0;
    printk("KERN_DEBUG %s: POWER %s\r\n", __FUNCTION__, on?"ON":"OFF");

    if (on)
    {
        /*rc = msm_gpios_enable(bt_config_power_on, ARRAY_SIZE(bt_config_power_on));
        if (rc<0)
        {
            printk(KERN_DEBUG "%s: Power ON bluetooth failed.\r\n", __FUNCTION__);
            return rc;
        }*/
	gpio_tlmm_config(GPIO_CFG(32, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),GPIO_CFG_ENABLE);	/* RFR */
	gpio_tlmm_config(GPIO_CFG(34, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_PULL_UP, GPIO_CFG_2MA),GPIO_CFG_ENABLE);	 
	gpio_tlmm_config(GPIO_CFG(43, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);	/* RFR */
	gpio_tlmm_config(GPIO_CFG(44, 2, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);	        /* CTS */
	gpio_tlmm_config(GPIO_CFG(45, 2, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);	        /* Rx */
	gpio_tlmm_config(GPIO_CFG(46, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);	/* Tx */
	gpio_tlmm_config(GPIO_CFG(27, 0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);	        /* BT_HOST_WAKE */
	gpio_tlmm_config(GPIO_CFG(31, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);	/* BT_WAKE */

	 for (pin = 0; pin < ARRAY_SIZE(bt_config_pcm_on);	pin++) {
		rc = gpio_tlmm_config(bt_config_pcm_on[pin],GPIO_CFG_ENABLE);
		if (rc < 0)
			return rc;
	}
	 //gpio_set_value(GPIO_BTUART_RFR, 0);
	 //mdelay(20);
        gpio_set_value(GPIO_BT_RST_N, 0); 
        mdelay(20);
        gpio_set_value(GPIO_BT_RST_N, 1); 
        mdelay(20);
        gpio_set_value(HOST_WAKEUP_BT, 1);
        mdelay(20);
        //printk(KERN_DEBUG "%s: PULL UP GPIO_BT_REG_ON\r\n", __FUNCTION__);
        gpio_set_value(GPIO_BT_REG_ON, 1);
        mdelay(100);

        printk(KERN_DEBUG "%s: GPIO_BT_RST    (%s)\r\n", __FUNCTION__, gpio_get_value(GPIO_BT_RST_N )?"HIGH":"LOW");
        printk(KERN_DEBUG "%s: GPIO_BT_REG_ON (%s)\r\n", __FUNCTION__, gpio_get_value(GPIO_BT_REG_ON)?"HIGH":"LOW");
        printk(KERN_DEBUG "%s: GPIO_BTUART_RX !(%s)\r\n", __FUNCTION__, gpio_get_value(GPIO_BTUART_RX)?"HIGH":"LOW");
        printk(KERN_DEBUG "%s: GPIO_BTUART_TX !(%s)\r\n", __FUNCTION__, gpio_get_value(GPIO_BTUART_TX)?"HIGH":"LOW");
        printk(KERN_DEBUG "%s: GPIO_BTUART_RFR !(%s)\r\n", __FUNCTION__, gpio_get_value(GPIO_BTUART_RFR)?"HIGH":"LOW");
        printk(KERN_DEBUG "%s: GPIO_BTUART_CTS !(%s)\r\n", __FUNCTION__, gpio_get_value(GPIO_BTUART_CTS)?"HIGH":"LOW");
        printk(KERN_DEBUG "%s: HOST_WAKEUP_BT !(%s)\r\n", __FUNCTION__, gpio_get_value(HOST_WAKEUP_BT)?"HIGH":"LOW");
    }
    else
    {
         rc = msm_gpios_enable(bt_config_power_off, ARRAY_SIZE(bt_config_power_off));
        if (rc<0)
        {
            printk(KERN_DEBUG "%s: Power OFF bluetooth failed.\r\n", __FUNCTION__);
            return rc;
        }
	 for (pin = 0; pin < ARRAY_SIZE(bt_config_pcm_off);	pin++) {
		rc = gpio_tlmm_config(bt_config_pcm_off[pin],GPIO_CFG_ENABLE);
		if (rc < 0)
			return rc;
	 }

        //printk(KERN_DEBUG "%s: PULL DOWN GPIO_BT_REG_ON\r\n", __FUNCTION__);
        gpio_set_value(GPIO_BT_REG_ON, 0);
        mdelay(100);
        gpio_set_value(HOST_WAKEUP_BT, 0);
        mdelay(20);
        gpio_set_value(GPIO_BT_RST_N, 0);
        mdelay(20);

        printk(KERN_DEBUG "%s: GPIO_BT_RST    (%s)\r\n", __FUNCTION__, gpio_get_value(GPIO_BT_RST_N )?"HIGH":"LOW");
        printk(KERN_DEBUG "%s: GPIO_BT_REG_ON (%s)\r\n", __FUNCTION__, gpio_get_value(GPIO_BT_REG_ON)?"HIGH":"LOW");
        printk(KERN_DEBUG "%s: GPIO_BTUART_RX !(%s)\r\n", __FUNCTION__, gpio_get_value(GPIO_BTUART_RX)?"HIGH":"LOW");
        printk(KERN_DEBUG "%s: GPIO_BTUART_TX !(%s)\r\n", __FUNCTION__, gpio_get_value(GPIO_BTUART_TX)?"HIGH":"LOW");
        printk(KERN_DEBUG "%s: GPIO_BTUART_RFR !(%s)\r\n", __FUNCTION__, gpio_get_value(GPIO_BTUART_RFR)?"HIGH":"LOW");
        printk(KERN_DEBUG "%s: GPIO_BTUART_CTS !(%s)\r\n", __FUNCTION__, gpio_get_value(GPIO_BTUART_CTS)?"HIGH":"LOW");
        printk(KERN_DEBUG "%s: HOST_WAKEUP_BT !(%s)\r\n", __FUNCTION__, gpio_get_value(HOST_WAKEUP_BT)?"HIGH":"LOW");
    }

    return 0;
}

static struct resource bluesleep_resources[] = {
    {
        .name = "gpio_host_wake",
        .start = BT_HOST_WAKE,
        .end = BT_HOST_WAKE,
        .flags = IORESOURCE_IO,
    },
    {
        .name = "gpio_ext_wake",
        .start = HOST_WAKEUP_BT,
        .end = HOST_WAKEUP_BT,
        .flags = IORESOURCE_IO,
    },
    {
        .name = "host_wake",
        .start = MSM_GPIO_TO_INT(BT_HOST_WAKE),
        .end = MSM_GPIO_TO_INT(BT_HOST_WAKE),
        .flags = IORESOURCE_IO,
    },
};

static struct platform_device msm_bluesleep_device = {
    .name = "bluesleep",
    .id = -1,
    .num_resources = ARRAY_SIZE(bluesleep_resources),
    .resource = bluesleep_resources,
};
static void __init bcm4330_bt_power_init(void)
{
    //gpio_set_value(GPIO_BT_REG_ON, 0);
    //gpio_set_value(GPIO_BT_RST_N, 0);
    gpio_tlmm_config(GPIO_CFG(43, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);	/* RFR */
    gpio_tlmm_config(GPIO_CFG(44, 2, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);	        /* CTS */
    gpio_tlmm_config(GPIO_CFG(45, 2, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);	        /* Rx */
    gpio_tlmm_config(GPIO_CFG(46, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE);	/* Tx */
    bcm4330_bt_power_device.dev.platform_data = &bluetooth_power;
}

inline int bt_init(void)
{
	bcm4330_bt_power_init();
	platform_device_register(&bcm4330_bt_power_device);
	platform_device_register(&msm_bluesleep_device);
	
	return 0;	
}
#else	/* !CONFIG_BROADCOM_BCM4330 */
inline int bt_init(void)
{
	return 0;	
}
#endif	/* !CONFIG_BROADCOM_BCM4330 */
//Thomas 2011.7.14 BT Porting ++]