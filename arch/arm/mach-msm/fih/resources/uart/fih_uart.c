/**
* LCM Driver
******************************************/
#include <mach/board.h>
#include "../../../devices.h"
#include "fih_uart.h"

#if defined(CONFIG_SERIAL_MSM_HSL_CONSOLE) \
		&& defined(CONFIG_MSM_SHARED_GPIO_FOR_UART2DM)

/* UART_QUALCOMM_DEFAULT */
static struct msm_gpio uart2dm_gpios_v0[] = {
	{GPIO_CFG(19, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_rfr_n" },
	{GPIO_CFG(20, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_cts_n" },
	{GPIO_CFG(21, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_rx"    },
	{GPIO_CFG(108, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_tx"    },
};

int msm7x27a_cfg_uart2dm_serial_v0(void* hdl)
{
	int ret;

    printk(KERN_INFO"[fih_uart] %s\n",__func__);

	ret = msm_gpios_request_enable(uart2dm_gpios_v0,
					ARRAY_SIZE(uart2dm_gpios_v0));
	if (ret)
		pr_err("%s: unable to enable gpios for uart2dm\n", __func__);

    return ret;
}

/* UART_V1 */
static struct msm_gpio uart2dm_gpios_v1[] = {
	{GPIO_CFG(21, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_rx"    },
	{GPIO_CFG(108, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart2dm_tx"    },
};

int msm7x27a_cfg_uart2dm_serial_v1(void* hdl)
{
	int ret;

    printk(KERN_INFO"[fih_uart] %s\n",__func__);

	ret = msm_gpios_request_enable(uart2dm_gpios_v1,
					ARRAY_SIZE(uart2dm_gpios_v1));
	if (ret)
		pr_err("%s: unable to enable gpios for uart2dm\n", __func__);

    return ret;
}

/*Vincent for ITV Uart3 B*/
static struct msm_gpio uart3_gpios_v0[] = {
	{GPIO_CFG(111, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"uart3_tx_b"    },
	{GPIO_CFG(112, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
							"usbh_uart3_rx"    },
};

int msm7x27a_cfg_uart3_serial_v0(void* hdl)
{
	int ret;

    printk(KERN_INFO"[fih_uart] %s\n",__func__);

	ret = msm_gpios_request_enable(uart3_gpios_v0,
					ARRAY_SIZE(uart3_gpios_v0));
	if (ret)
		pr_err("%s: unable to enable gpios for uart3\n", __func__);

    return ret;
}
/*Vincent for ITV Uart3 E*/

/* uart init */
void __init fih_uart_init(void)
{
    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_UART, 
                                     UART_QUALCOMM_DEFAULT), 
                   "UART_QUALCOMM_DEFAULT", 
                   msm7x27a_cfg_uart2dm_serial_v0, 
                   __FILE__, __LINE__);

    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_UART, 
                                     UART_V1), 
                   "UART_V1", 
                   msm7x27a_cfg_uart2dm_serial_v1, 
                   __FILE__, __LINE__);
                   
    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_UART, 
                                     UART_V2), 
                   "UART_V2", 
                   msm7x27a_cfg_uart3_serial_v0, 
                   __FILE__, __LINE__);

}
#endif
