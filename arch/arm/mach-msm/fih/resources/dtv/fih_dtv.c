/** 
* DTV  Driver
****************************************************/
#include "fih_dtv.h"
#include <mach/msm_tsif.h>
#include <mach/dma.h>

#if defined(CONFIG_FIH_TSIF) || defined(CONFIG_TSIF_MODULE)

#define TSIF_B_SYNC     GPIO_CFG(108, 5, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_DATA     GPIO_CFG(21, 5, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_EN        GPIO_CFG(20, 5, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_CLK       GPIO_CFG(19, 5, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)

static const struct msm_gpio tsif_gpios[] = {
	{ .gpio_cfg = TSIF_B_CLK,  .label =  "tsif_clk", },
	{ .gpio_cfg = TSIF_B_EN,   .label =  "tsif_en", },
	{ .gpio_cfg = TSIF_B_DATA, .label =  "tsif_data", },
	{ .gpio_cfg = TSIF_B_SYNC, .label =  "tsif_sync", },
};

static struct msm_tsif_platform_data tsif_platform_data = {
	.num_gpios = ARRAY_SIZE(tsif_gpios),
//	.num_gpios = 4,
	.gpios = tsif_gpios,
	.tsif_clk = "tsif_clk",
	.tsif_pclk = "tsif_pclk",
	.tsif_ref_clk = "tsif_ref_clk",
};
#endif 

#if defined(CONFIG_FIH_TSIF) || defined(CONFIG_TSIF_MODULE)

#define MSM_TSIF_PHYS        (0xa0100000)
#define MSM_TSIF_SIZE        (0x200)

static struct resource tsif_resources[] = {
	[0] = {
		.flags = IORESOURCE_IRQ,
		.start = INT_TSIF_IRQ,
		.end   = INT_TSIF_IRQ,
	},
	[1] = {
		.flags = IORESOURCE_MEM,
		.start = MSM_TSIF_PHYS,
		.end   = MSM_TSIF_PHYS + MSM_TSIF_SIZE - 1,
	},
	[2] = {
		.flags = IORESOURCE_DMA,
		.start = DMOV_TSIF_CHAN,
		.end   = DMOV_TSIF_CRCI,
	},
};

static void tsif_release(struct device *dev)
{
	dev_info(dev, "release\n");
}

struct platform_device msm_device_tsif = {
	.name          = "msm_tsif",
	.id            = 0,
	.num_resources = ARRAY_SIZE(tsif_resources),
	.resource      = tsif_resources,
	.dev = {
		.release       = tsif_release,
	    .platform_data =&tsif_platform_data,
	},
};
#endif 



#ifdef  CONFIG_TSIF_NM32X_62X_DEV
//dtv_nmi326
static struct platform_device nm32x_62x_tsi_device = {
	.name      = "nm32x_62x-tsi",
	.id        = -1,
};
#endif

static struct i2c_board_info nmi625_i2c_info[] = {
{
	 I2C_BOARD_INFO("nmi625", 0x61>>1),    
	},
};

int __init dtv_init(void)
{

   void *hdl;
   hdl = DECL_VERSION_CTRL(DRIVER_TYPE_TSIF_NM32X_62X, TSIF_NM32X_62X_V1);
   REGISTER_DYN_PLATFORM_DEVICE(hdl, &nm32x_62x_tsi_device);

   hdl = DECL_VERSION_CTRL(DRIVER_TYPE_TSIF, TSIF_V1);
   REGISTER_DYN_PLATFORM_DEVICE(hdl, &msm_device_tsif);

   
   ADD_DYN_I2C_DEVICE(DECL_VERSION_CTRL(DRIVER_TYPE_DTV_NMI, DTV_NMI_V1), 
				MSM_GSBI1_QUP_I2C_BUS_ID,
				&nmi625_i2c_info[0]);

   return 0;
}

