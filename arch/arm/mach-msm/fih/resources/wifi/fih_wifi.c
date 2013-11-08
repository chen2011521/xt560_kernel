/**
* Broadcom Wifi Driver
************************************************************/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/skbuff.h>
#include "fih_wifi.h"

//# FIH,chandler,2012.8.12 broadcomm wifi +++[
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

#define PREALLOC_WLAN_NUMBER_OF_SECTIONS	4
#define PREALLOC_WLAN_NUMBER_OF_BUFFERS		160
#define PREALLOC_WLAN_SECTION_HEADER		24
#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 1024)
#define WLAN_SKB_BUF_NUM	16
static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];
typedef struct wifi_mem_prealloc_struct {
	void *mem_ptr;
	unsigned long size;
} wifi_mem_prealloc_t;
static wifi_mem_prealloc_t wifi_mem_array[PREALLOC_WLAN_NUMBER_OF_SECTIONS] = {
	{ NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER) }
};
void *fih_wifi_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_NUMBER_OF_SECTIONS)
		return wlan_static_skb;
	if ((section < 0) || (section > PREALLOC_WLAN_NUMBER_OF_SECTIONS))
		return NULL;
	if (wifi_mem_array[section].size < size)
		return NULL;
	return wifi_mem_array[section].mem_ptr;
}
EXPORT_SYMBOL(fih_wifi_mem_prealloc);
int __init fih_init_wifi_mem(void)
{
	int i;
	for(i=0;( i < WLAN_SKB_BUF_NUM );i++) {
		if (i < (WLAN_SKB_BUF_NUM/2))
			wlan_static_skb[i] = dev_alloc_skb(4096);
		else
			wlan_static_skb[i] = dev_alloc_skb(8192);
	}
	for(i=0;( i < PREALLOC_WLAN_NUMBER_OF_SECTIONS );i++) {
		wifi_mem_array[i].mem_ptr = kmalloc(wifi_mem_array[i].size,
							GFP_KERNEL);
		if (wifi_mem_array[i].mem_ptr == NULL)
			return -ENOMEM;
	}
	return 0;
}
static unsigned int bcm4330_power_status=0;

static struct platform_device bcm4330_wifi_power_device = {
    .name = "bcm4330_wifi_power",
    .id     = -1
};

int wifi_power(int on)
{
    printk(KERN_DEBUG "%s: POWER %s\r\n", __FUNCTION__, on?"ON":"OFF");

    if (on)
    {
        gpio_set_value(GPIO_WL_REG_ON, 0);
        mdelay(20);
        gpio_set_value(GPIO_WL_REG_ON, 1);
        mdelay(200);
        bcm4330_power_status = 1;

        printk(KERN_DEBUG "%s: GPIO_WL_REG_ON (%s)\r\n", __FUNCTION__, gpio_get_value(GPIO_WL_REG_ON)?"HIGH":"LOW");
    }
    else
    {
        bcm4330_power_status = 0;
        gpio_set_value(GPIO_WL_REG_ON, 0);
        mdelay(20);

        printk(KERN_DEBUG "%s: GPIO_WL_REG_ON (%s)\r\n", __FUNCTION__, gpio_get_value(GPIO_WL_REG_ON)?"HIGH":"LOW");
    }

    return 0;
}                                            
EXPORT_SYMBOL(wifi_power);

int bcm4330_wifi_resume(void)
{
    printk(KERN_DEBUG "%s: START bcm4330_power_status=0x%x\r\n", __func__, bcm4330_power_status);

    if (bcm4330_power_status == 0) {
        gpio_set_value(GPIO_WL_REG_ON, 1);
        mdelay(20);
        printk(KERN_DEBUG "%s: PULL UP GPIO_WL_REG_ON\r\n", __func__);
        bcm4330_power_status = 1;
    }

    return 0;
}
EXPORT_SYMBOL(bcm4330_wifi_resume);

int bcm4330_wifi_suspend(void)
{
    printk(KERN_DEBUG "%s: START bcm4330_power_status=0x%x\r\n", __func__, bcm4330_power_status);

    if (bcm4330_power_status == 1) {
        printk(KERN_DEBUG "%s: PULL DOWN GPIO_WL_REG_ON\r\n", __func__);
        gpio_set_value(GPIO_WL_REG_ON, 0);
        bcm4330_power_status = 0;
    }

    return 0;
}
EXPORT_SYMBOL(bcm4330_wifi_suspend);

static void __init bcm4330_wifi_power_init(void)
{
    gpio_set_value(GPIO_WL_REG_ON, 0);
    mdelay(20);
    bcm4330_wifi_power_device.dev.platform_data = &wifi_power;
}

inline int wifi_init(void)
{
	bcm4330_wifi_power_init();
	//add by leo 2012-1.2 using static buffer
	fih_init_wifi_mem();	
	//end add by leo 2012-1.2 using static buffer
	platform_device_register(&bcm4330_wifi_power_device);	
	
	return 0;
}
#else	/* !CONFIG_BROADCOM_BCM4330 */
inline int wifi_init(void)
{
	return 0;	
}
#endif	/* !CONFIG_BROADCOM_BCM4330 */
//# FIH,chandler,2012.8.12 broadcomm wifi +++]

