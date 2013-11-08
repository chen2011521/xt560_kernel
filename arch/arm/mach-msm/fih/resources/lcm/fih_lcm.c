/**
* LCM Driver
******************************************/
#include <mach/board.h>
#include "../../../devices.h"
#include "fih_lcm.h"
#include <asm/io.h>

// FIH,Tiger,replace qualcomm default mipi dsi +[
#if defined(CONFIG_FB_MSM_MDP303) && defined(CONFIG_FIH_7X27A_PROJS)
#define MDP_303_VSYNC_GPIO 97

/*IRMPrime*/
#define	  AMOLED_GPIO_SPI_MDATA   83 /* spi_sdi */
#define	  AMOLED_GPIO_SPI_CLK     82   /* spi_clk */
#define	  AMOLED_GPIO_SPI_CS0_N   16 /* spi_cs  */

extern int proc_comm_set_led(unsigned id, unsigned level);
static bool lcm_firstpower = 1;
extern unsigned char *mipi_dsi_base;

enum {
	DSI_SINGLE_LANE_ = 1,
	DSI_TWO_LANES_,
};

static int fih_fb_get_lane_config(void)
{
	int rc = DSI_TWO_LANES_;

	if (cpu_is_msm7x25a() || cpu_is_msm7x25aa()) {
		rc = DSI_SINGLE_LANE_;
		pr_info("DSI Single Lane\n");
	} else {
		pr_info("DSI Two Lanes\n");
	}
	return rc;
}

static int fih_fb_dsi_client_reset(void)
{
	return 0;
}

#define LCM_RESET_GPIO 129
static unsigned lcm_reset_gpio =
	GPIO_CFG(LCM_RESET_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA); 

static struct vreg *vreg_l12_lcm_avdd = NULL;
static struct vreg *vreg_s3_lcm_iovdd = NULL;

extern int g_panel_id;

static int fih_mipi_dsi_panel_power(int on)
{
	int rc = 0;
	printk(KERN_INFO "[DISPLAY] %s(%d)\n", __func__, on);

	if (on) {
		/* reset LCM -- toggle reset pin -- gpio_129 */
		rc = gpio_tlmm_config(lcm_reset_gpio, GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
			__func__, lcm_reset_gpio, rc);
			return rc;
		}

//		gpio_set_value_cansleep(LCM_RESET_GPIO, 0);  /* bring reset line low to hold reset*/
		mdelay(1); /* wait 1 ms */
	}
	else if (g_panel_id == 1) { 	/* lcm off of Truly with Himax HX8363-A */
		/* Hard Reset, Reset Pin from high to low */
		gpio_set_value_cansleep(LCM_RESET_GPIO, 0);
		mdelay(10);
		gpio_set_value_cansleep(LCM_RESET_GPIO, 1);
		printk(KERN_INFO "[DISPLAY] %s: LCM_RESET_GPIO 1->0->1\n", __func__);
	}

	if (vreg_l12_lcm_avdd == NULL) {
		vreg_l12_lcm_avdd = vreg_get(NULL, "gp2");
		if (IS_ERR(vreg_l12_lcm_avdd)) {
			printk(KERN_ERR "[DISPLAY] %s: vreg_get(%s) failed (%ld)\n",
			__func__, "L12(gp2)", PTR_ERR(vreg_l12_lcm_avdd));
			return 0;
		}

		rc = vreg_set_level(vreg_l12_lcm_avdd, 2850);
		if (rc) {
			printk(KERN_ERR "[DISPLAY] %s: L12(GP2) set_level failed (%d)\n",
			__func__, rc);
			return 0;
		}
	}

	if (vreg_s3_lcm_iovdd == NULL) {
		vreg_s3_lcm_iovdd = vreg_get(NULL, "msme1");
		if (IS_ERR(vreg_s3_lcm_iovdd)) {
			printk(KERN_ERR "[DISPLAY] %s: vreg_get(%s) failed (%ld)\n",
			__func__, "S3(msme1)", PTR_ERR(vreg_s3_lcm_iovdd));
			return 0;
		}

		rc = vreg_set_level(vreg_s3_lcm_iovdd, 1800);
		if (rc) {
			printk(KERN_ERR "[DISPLAY] %s: S3(MSME1) set level failed (%d)\n",
			__func__, rc);
			return 0;
		}
	}

	if (on) {
		rc = vreg_enable(vreg_l12_lcm_avdd);
		if (rc) {
			printk(KERN_ERR "[DISPLAY] %s: L12(GP2) enable failed (%d)\n",
			__func__, rc);
			return 0;
		}
		msleep(5);
		rc = vreg_enable(vreg_s3_lcm_iovdd);
		if (rc) {
			printk(KERN_ERR "[DISPLAY] %s: S3(MSME1) enable failed (%d)\n",
			__func__, rc);
			return 0;
		}
		msleep(25);
	} else {
		rc = vreg_disable(vreg_s3_lcm_iovdd);
		if (rc) {
			printk(KERN_ERR "[DISPLAY] %s: S3(MSME1) disable failed (%d)\n",
			__func__, rc);
			return 0;
		}
		msleep(5);
		rc = vreg_disable(vreg_l12_lcm_avdd);
		if (rc) {
			printk(KERN_ERR "[DISPLAY] %s: L12(GP2) disable failed (%d)\n",
			__func__, rc);
			return 0;
		}
		msleep(5);
	}

	if (on) {
		gpio_set_value_cansleep(LCM_RESET_GPIO, 1);  /* bring reset line high */
		mdelay(20);      /* 10 msec before IO can be accessed */

		/* FIH, Ming, 2011/11/02 { */
		/* To make sure of a complete high->low->high pulse */
		gpio_set_value_cansleep(LCM_RESET_GPIO, 0);
		mdelay(10);
		gpio_set_value_cansleep(LCM_RESET_GPIO, 1);
		mdelay(10);
		gpio_set_value_cansleep(LCM_RESET_GPIO, 0);
		mdelay(10);
		gpio_set_value_cansleep(LCM_RESET_GPIO, 1);
		mdelay(20);
		printk(KERN_INFO "[DISPLAY] %s: LCM_RESET_GPIO 1->0->1->0->1\n", __func__);
		/* } FIH, Ming, 2011/11/02 */
	}

	return rc;
}
static int fih_toshiba_mipi_dsi_panel_power(int on)
{
	int rc = 0;
	printk(KERN_INFO "[DISPLAY] %s(%d)\n", __func__, on);

/* FIHNJDC, SW4-BSP Jenny, 2011/12/16{ */
/*the mipi clk line or data line has leaked current ,need disable dsi engine*/
    if (lcm_firstpower) {
        lcm_firstpower = 0;
        /* disbale dsi engine */
        writel(0,mipi_dsi_base + 0x0000);//DSI_CTRL
        mdelay(5);
    }
/* } FIHNJDC, SW4-BSP Jenny, 2011/12/16 */
	if (on) {
		/* reset LCM -- toggle reset pin -- gpio_129 */
		rc = gpio_tlmm_config(lcm_reset_gpio, GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
			__func__, lcm_reset_gpio, rc);
			return rc;
		}

		gpio_set_value_cansleep(LCM_RESET_GPIO, 0);  /* bring reset line low to hold reset*/
		mdelay(1); /* wait 1 ms */
	}
	else
	{
		/* Hard Reset, Reset Pin from high to low */
		gpio_set_value_cansleep(LCM_RESET_GPIO, 0); 
	}

	if (vreg_l12_lcm_avdd == NULL) {
		vreg_l12_lcm_avdd = vreg_get(NULL, "gp2");
		if (IS_ERR(vreg_l12_lcm_avdd)) {
			printk(KERN_ERR "[DISPLAY] %s: vreg_get(%s) failed (%ld)\n",
			__func__, "L12(gp2)", PTR_ERR(vreg_l12_lcm_avdd));
			return 0;
		}

		rc = vreg_set_level(vreg_l12_lcm_avdd, 2850);
		if (rc) {
			printk(KERN_ERR "[DISPLAY] %s: L12(GP2) set_level failed (%d)\n",
			__func__, rc);
			return 0;
		}
	}

	if (on) {
		rc = vreg_enable(vreg_l12_lcm_avdd);
		if (rc) {
			printk(KERN_ERR "[DISPLAY] %s: L12(GP2) enable failed (%d)\n",
			__func__, rc);
			return 0;
		}
		msleep(5);
	} else {
		rc = vreg_disable(vreg_l12_lcm_avdd);
		if (rc) {
			printk(KERN_ERR "[DISPLAY] %s: L12(GP2) disable failed (%d)\n",
			__func__, rc);
			return 0;
		}
		msleep(5);
	}

	if (on) {
		gpio_set_value_cansleep(LCM_RESET_GPIO, 1);  /* bring reset line high */
		mdelay(5);      /* 10 msec before IO can be accessed */ 
	}

	return rc;
}

static int fih_renesas_mipi_dsi_panel_power(int on)
{
	int rc = 0;
	printk(KERN_INFO "[DISPLAY] %s(%d)\n", __func__, on);

/* FIHNJDC, SW4-BSP Jenny, 2011/11/4{ */
    if (lcm_firstpower) {
        lcm_firstpower = 0;
        proc_comm_set_led(0,0);
       /* FIHNJDC, SW4-BSP Jenny, 2011/12/16{ */
        /*the mipi clk line or data line has leaked current ,need disable dsi engine*/
        writel(0,mipi_dsi_base + 0x0000);
        mdelay(5);
        /* FIHNJDC, SW4-BSP Jenny, 2011/12/16{ */
    }
/* } FIHNJDC, SW4-BSP Jenny, 2011/11/4 */

	if (on) {
		/* reset LCM -- toggle reset pin -- gpio_129 */
		rc = gpio_tlmm_config(lcm_reset_gpio, GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
			__func__, lcm_reset_gpio, rc);
			return rc;
		}

		gpio_set_value_cansleep(LCM_RESET_GPIO, 0);  /* bring reset line low to hold reset*/
		mdelay(1); /* wait 1 ms */
	}
	else
	{
		/* Hard Reset, Reset Pin from high to low */
		gpio_set_value_cansleep(LCM_RESET_GPIO, 0); 
	}

	if (vreg_l12_lcm_avdd == NULL) {
		vreg_l12_lcm_avdd = vreg_get(NULL, "gp2");
		if (IS_ERR(vreg_l12_lcm_avdd)) {
			printk(KERN_ERR "[DISPLAY] %s: vreg_get(%s) failed (%ld)\n",
			__func__, "L12(gp2)", PTR_ERR(vreg_l12_lcm_avdd));
			return 0;
		}

		rc = vreg_set_level(vreg_l12_lcm_avdd, 2850);
		if (rc) {
			printk(KERN_ERR "[DISPLAY] %s: L12(GP2) set_level failed (%d)\n",
			__func__, rc);
			return 0;
		}
	}	

	if (on) {
		rc = vreg_enable(vreg_l12_lcm_avdd);
		if (rc) {
			printk(KERN_ERR "[DISPLAY] %s: L12(GP2) enable failed (%d)\n",
			__func__, rc);
			return 0;
		}
		msleep(5);
	} else {
		rc = vreg_disable(vreg_l12_lcm_avdd);
		if (rc) {
			printk(KERN_ERR "[DISPLAY] %s: L12(GP2) disable failed (%d)\n",
			__func__, rc);
			return 0;
		}
		msleep(5);
	}

	if (on) {
		gpio_set_value_cansleep(LCM_RESET_GPIO, 1);  /* bring reset line high */
		mdelay(5);      /* 10 msec before IO can be accessed */ 
	}

	return rc;
}

static struct mipi_dsi_platform_data fih_mipi_dsi_pdata = {
	.vsync_gpio = MDP_303_VSYNC_GPIO,
	.dsi_power_save   = fih_mipi_dsi_panel_power,
	.dsi_client_reset = fih_fb_dsi_client_reset,
	.get_lane_config = fih_fb_get_lane_config,
};

#ifdef CONFIG_FB_MSM_LCDC_S6E63M0_WVGA_PT_PANEL

static uint32_t lcm_s6e63m0_on_gpio_table[] = {
	GPIO_CFG(AMOLED_GPIO_SPI_MDATA, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
	GPIO_CFG(AMOLED_GPIO_SPI_CLK, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
	GPIO_CFG(AMOLED_GPIO_SPI_CS0_N, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
};

static const char * const msm_fb_lcdc_s6e63m0_vreg[] = {
		"gp2",
		"msme1",
};

static const int msm_fb_lcdc_s6e63m0_vreg_mV[] = {
	2850,
	1800,
};

struct vreg *s6e63m0_lcdc_vreg[ARRAY_SIZE(msm_fb_lcdc_s6e63m0_vreg)];

static uint32_t lcdc_gpio_initialized;

static int lcm_spi_gpio_init_fake(void)
{

	int rc = 0, pin;

	/* skip tlmm setting */
	printk( KERN_ERR "%s()\n", __func__);	

    for (pin = 0; pin < ARRAY_SIZE(lcm_s6e63m0_on_gpio_table); pin++) {
        rc = gpio_tlmm_config(lcm_s6e63m0_on_gpio_table[pin], GPIO_CFG_ENABLE);
        if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config(%d)(%#x)=%d failed\n", __func__, (lcm_s6e63m0_on_gpio_table[pin]&0x3ff0)>>4,lcm_s6e63m0_on_gpio_table[pin], rc);
			return -EIO;
        }else{
			//printk(KERN_ERR "%s: gpio_tlmm_config(%d)(%#x)=%d success\n", __func__, (lcm_s6e63m0_on_gpio_table[pin]&0x3ff0)>>4,lcm_s6e63m0_on_gpio_table[pin], rc);
		}
    }

	return 0;
}

static void lcdc_s6e63m0_gpio_init(void)
{
	int i, rc = 0;

	//printk( KERN_INFO "[board s6e63m0]: %s\n", __func__ );
    lcm_spi_gpio_init_fake();

	if (!lcdc_gpio_initialized) {
		if (gpio_request(AMOLED_GPIO_SPI_CLK, "spi_clk")) {
			printk(KERN_ERR "failed to request gpio spi_clk\n");
			return;
		}
		if (gpio_request(AMOLED_GPIO_SPI_CS0_N, "spi_cs")) {
			printk(KERN_ERR "failed to request gpio spi_cs0_N\n");
			goto fail_gpio3;
		}
		if (gpio_request(AMOLED_GPIO_SPI_MDATA, "spi_mosi")) {
			printk(KERN_ERR "failed to request gpio spi_mosi\n");
			goto fail_gpio2;
		}

		for (i = 0; i < ARRAY_SIZE(msm_fb_lcdc_s6e63m0_vreg); i++) {
			s6e63m0_lcdc_vreg[i] = vreg_get(0, msm_fb_lcdc_s6e63m0_vreg[i]);

			rc = vreg_set_level(s6e63m0_lcdc_vreg[i],
						msm_fb_lcdc_s6e63m0_vreg_mV[i]);

			if (rc < 0) {
				printk(KERN_ERR "%s: set regulator level failed "
					"with :(%d)\n", __func__, rc);
				goto fail_gpio1;
			}
		}
		lcdc_gpio_initialized = 1;
	}

	return;

fail_gpio1:
	for (; i > 0; i--)
			vreg_put(s6e63m0_lcdc_vreg[i - 1]);

     gpio_free(AMOLED_GPIO_SPI_MDATA);
fail_gpio2:
	gpio_free(AMOLED_GPIO_SPI_CS0_N);
fail_gpio3:
	gpio_free(AMOLED_GPIO_SPI_CLK);
	lcdc_gpio_initialized = 0;

}

static uint32_t lcdc_gpio_table[] = {
    AMOLED_GPIO_SPI_MDATA,
    AMOLED_GPIO_SPI_CLK,
    AMOLED_GPIO_SPI_CS0_N,
};

static void config_lcdc_gpio_table(uint32_t *table, int len, unsigned enable)
{
	int n;

    if (lcdc_gpio_initialized) {
		/* All are IO Expander GPIOs */
		for (n = 0; n < (len - 1); n++)
			gpio_direction_output(table[n], 1);
	}
}

static void lcdc_s6e63m0_config_gpios(int enable)
{

	config_lcdc_gpio_table(lcdc_gpio_table,
		ARRAY_SIZE(lcdc_gpio_table), enable);

}

#define AMOLED_LCM_RESET_GPIO 107
static unsigned amoled_lcm_reset_gpio =
	GPIO_CFG(AMOLED_LCM_RESET_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA); 

static int msm_fb_lcdc_s6e63m0_power_save(int on)
{
	int i, rc = 0;

    printk( KERN_INFO "[board s6e63m0]: %s on=%d\n", __func__,on);

    /* Doing the init of the LCDC GPIOs very late as they are from
        an I2C-controlled IO Expander */
    lcdc_s6e63m0_gpio_init();
    if (lcdc_gpio_initialized) {

        if (on) {
            /* reset LCM -- toggle reset pin -- gpio_129 */
            rc = gpio_tlmm_config(amoled_lcm_reset_gpio, GPIO_CFG_ENABLE);
            if (rc) {
                printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
                       __func__, amoled_lcm_reset_gpio, rc);
                return rc;
            }

            gpio_set_value_cansleep(AMOLED_LCM_RESET_GPIO, 1);  /* bring reset line low to hold reset*/
            mdelay(1); /* wait 1 ms */
        }
        else {
            /* Hard Reset, Reset Pin from high to low */
            gpio_set_value_cansleep(AMOLED_LCM_RESET_GPIO, 0); 
        }

        for (i = 0; i < ARRAY_SIZE(msm_fb_lcdc_s6e63m0_vreg); i++) {
            if (on) {
                rc = vreg_enable(s6e63m0_lcdc_vreg[i]);

                if (rc) {
                    printk(KERN_ERR "vreg_enable: %s vreg"
                           "operation failed\n",
                           msm_fb_lcdc_s6e63m0_vreg[i]);
                    goto lcdc_vreg_fail;
                }
                msleep(30);

            }
            else {
                rc = vreg_disable(s6e63m0_lcdc_vreg[i]);

                if (rc) {
                    printk(KERN_ERR "vreg_disable: %s vreg "
                           "operation failed\n",
                           msm_fb_lcdc_s6e63m0_vreg[i]);
                    goto lcdc_vreg_fail;
                }
                msleep(30);
            }
        }
        if (on) {
            gpio_set_value_cansleep(AMOLED_LCM_RESET_GPIO, 0);  /* bring reset line high */
            mdelay(5);      /* 10 msec before IO can be accessed */ 
            gpio_set_value_cansleep(AMOLED_LCM_RESET_GPIO, 1);  /* bring reset line high */
            mdelay(10);      /* 10 msec before IO can be accessed */ 
        }

    }

    return rc;

    lcdc_vreg_fail:
    if (on) {
        for (; i > 0; i--)
            vreg_disable(s6e63m0_lcdc_vreg[i - 1]);
    }
    else {
        for (; i > 0; i--)
            vreg_enable(s6e63m0_lcdc_vreg[i - 1]);
    }
    return rc;

}

static struct lcdc_platform_data s6e63m0_lcdc_pdata = {
	.lcdc_gpio_config = NULL,
	.lcdc_power_save   = msm_fb_lcdc_s6e63m0_power_save,
};

static int lcd_panel_spi_gpio_num[] = {
        AMOLED_GPIO_SPI_MDATA,
        AMOLED_GPIO_SPI_CLK,
        AMOLED_GPIO_SPI_CS0_N,
};
static struct msm_panel_common_pdata lcdc_s6e63m0_panel_data = {
	.panel_config_gpio = lcdc_s6e63m0_config_gpios,
	.gpio_num	  = lcd_panel_spi_gpio_num,
};

static struct platform_device lcdc_s6e63m0_panel_device = {
	.name   = "lcdc_s6e63m0_wvga_pt",
	.id     = 0,
	.dev    = {
		.platform_data = &lcdc_s6e63m0_panel_data,
	}
};


int lcm_samsung_s6e63m0_v1(void *hdl)
{
    printk(KERN_INFO"[fih_lcm] %s\n",__func__);

	msm_fb_register_device("lcdc", &s6e63m0_lcdc_pdata);

	return 0;
}
#endif 

int lcm_novatek_nt35560_v1(void *hdl)
{
    printk(KERN_INFO"[fih_lcm] %s\n",__func__);

    fih_mipi_dsi_pdata.dsi_power_save = fih_mipi_dsi_panel_power;

	msm_fb_register_device("mipi_dsi", &fih_mipi_dsi_pdata);

	return 0;
}

int lcm_toshiba_upd161809_v1(void *hdl)
{
    printk(KERN_INFO"[fih_lcm] %s\n",__func__);

    fih_mipi_dsi_pdata.dsi_power_save = fih_toshiba_mipi_dsi_panel_power;

	msm_fb_register_device("mipi_dsi", &fih_mipi_dsi_pdata);

	return 0;
}

int lcm_renesas_r61531_v1(void *hdl)
{
    printk(KERN_INFO"[fih_lcm] %s\n",__func__);

    fih_mipi_dsi_pdata.dsi_power_save = fih_renesas_mipi_dsi_panel_power;

	msm_fb_register_device("mipi_dsi", &fih_mipi_dsi_pdata);

	return 0;
}

void __init fih_fb_device_init(void)
{

	void *hdl;

    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_LCM, 
                                     LCM_NOVATEK_NT35560_V1), 
                   "lcm_novatek_nt35560_v1", 
                   lcm_novatek_nt35560_v1, 
                   __FILE__, __LINE__);

    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_LCM, 
                                     LCM_NOVATEK_NT35560_V2), 
                   "lcm_novatek_nt35560_v2", 
                   lcm_novatek_nt35560_v1, 
                   __FILE__, __LINE__);

    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_LCM, 
                                     LCM_TOSHIBA_UPD161809_V1), 
                   "lcm_toshiba_upd161809_v1", 
                   lcm_toshiba_upd161809_v1, 
                   __FILE__, __LINE__);

    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_LCM, 
                                     LCM_RENESAS_R61531_V1), 
                   "lcm_renesas_r61531_v1", 
                   lcm_renesas_r61531_v1, 
                   __FILE__, __LINE__);
//IRMPrime

    add_dyn_module(DECL_VERSION_CTRL(DRIVER_TYPE_LCM, 
                                     LCM_SAMSUNG_S6E63M0X_V1), 
                   "lcm_samsung_s6e63m0_v1", 
                   lcm_samsung_s6e63m0_v1, 
                   __FILE__, __LINE__);

	hdl = DECL_VERSION_CTRL(DRIVER_TYPE_LCM, LCM_SAMSUNG_S6E63M0X_V1);

	REGISTER_DYN_PLATFORM_DEVICE(hdl, &lcdc_s6e63m0_panel_device);


	(void)hdl;
}
#endif
// FIH,Tiger,replace qualcomm default mipi dsi +]
