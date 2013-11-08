/**
* NFC Driver
******************************************/
/*Added by Jiahao,2011-9-23*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>

#include "fih_nfc.h"

#define PN544_I2C_SLAVE_ID 0x2B
#define GPIO_PN544_IRQ 19
#define GPIO_PN544_VEN 4
#define GPIO_PN544_FIRM 8

//IRMPrime
#define GPIO_PN544_IRQ_V2 19
#define GPIO_PN544_VEN_V2 49
#define GPIO_PN544_FIRM_V2 35

static struct pn544_i2c_platform_data pn544_data = {
    .irq_gpio = GPIO_PN544_IRQ,
    .ven_gpio = GPIO_PN544_VEN,
    .firm_gpio = GPIO_PN544_FIRM,
};

static struct pn544_i2c_platform_data pn544_data_v2 = {
    .irq_gpio = GPIO_PN544_IRQ_V2,
    .ven_gpio = GPIO_PN544_VEN_V2,
    .firm_gpio = GPIO_PN544_FIRM_V2,
};

static int config_nfc_gpios_v1(void *hdl) {
	int rc = 0;
	
	rc = gpio_tlmm_config( GPIO_CFG( GPIO_PN544_IRQ, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA ), GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s not able to get gpio %d\n", __func__, GPIO_PN544_IRQ);

	rc = gpio_tlmm_config( GPIO_CFG( GPIO_PN544_VEN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA ), GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s not able to get gpio %d\n", __func__, GPIO_PN544_VEN);
		
	rc = gpio_tlmm_config( GPIO_CFG( GPIO_PN544_FIRM, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA ), GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s not able to get gpio %d\n", __func__, GPIO_PN544_FIRM);
		

	return rc;
}

static int config_nfc_gpios_v2(void *hdl) {
	int rc = 0;
	
	rc = gpio_tlmm_config( GPIO_CFG( GPIO_PN544_IRQ_V2, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA ), GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s not able to get gpio %d\n", __func__, GPIO_PN544_IRQ_V2);

	rc = gpio_tlmm_config( GPIO_CFG( GPIO_PN544_VEN_V2, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA ), GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s not able to get gpio %d\n", __func__, GPIO_PN544_VEN_V2);
		
	rc = gpio_tlmm_config( GPIO_CFG( GPIO_PN544_FIRM_V2, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA ), GPIO_CFG_ENABLE);
	if (rc)
		pr_err("%s not able to get gpio %d\n", __func__, GPIO_PN544_FIRM_V2);
		

	return rc;
}

static struct i2c_board_info nfc_i2c_info[] __initdata = {
        {
                 I2C_BOARD_INFO("pn544", PN544_I2C_SLAVE_ID),
                .irq = MSM_GPIO_TO_INT(GPIO_PN544_IRQ),
       		    .platform_data  = &pn544_data,
        },
};
static struct i2c_board_info nfc_i2c_info_v2[] __initdata = {
        {
                 I2C_BOARD_INFO("pn544", PN544_I2C_SLAVE_ID),
                .irq = MSM_GPIO_TO_INT(GPIO_PN544_IRQ_V2),
       		    .platform_data  = &pn544_data_v2,
        },
};

int __init nfc_init(void)
{
     ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_NFC, NFC_V1),
                                MSM_GSBI1_QUP_I2C_BUS_ID,
                                nfc_i2c_info);

    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_NFC, NFC_V1),

                   "NFC_V1", config_nfc_gpios_v1, __FILE__, __LINE__);
    //IRMPrime
    ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_NFC, NFC_V2),
                            MSM_GSBI1_QUP_I2C_BUS_ID,
                            nfc_i2c_info_v2);

    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_NFC, NFC_V2),
                               "NFC_V2", config_nfc_gpios_v2, __FILE__, __LINE__);
        return 0;
}

