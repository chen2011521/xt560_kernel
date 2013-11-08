/** 
* Camera Driver
*********************************************************/
#include <mach/board.h>
#include <mach/msm_iomap.h>
#include "fih_cam.h"
#include <mach/vreg.h>
#include <mach/gpio.h>

#if defined(CONFIG_MSM_CAMERA)
static uint32_t camera_off_gpio_table[] = {
	GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};

static uint32_t camera_on_gpio_table[] = {
	GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG( 5, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG( 6, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(125, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};

static uint32_t camera_on_gpio_table_v2[] = {
	GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
};

#ifdef CONFIG_MSM_CAMERA_FLASH
static struct msm_camera_sensor_flash_src msm_flash_src = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_CURRENT_DRIVER,
	._fsrc.current_driver_src.led1 = 5,
	._fsrc.current_driver_src.led2 = 6,
};

static struct msm_camera_sensor_flash_src msm_flash_src_v2 = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_PMIC,
};
//IRMPrime
static struct msm_camera_sensor_flash_src msm_flash_src_v3 = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_CURRENT_DRIVER,
	._fsrc.current_driver_src.led1 = 89,
	._fsrc.current_driver_src.led2 = 88,
};
#endif
#ifdef CONFIG_MSM_CAMERA_FLASH_SC628A
static struct i2c_board_info flash_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("sc628a", 0x37),
	},
};
#endif
#if defined( CONFIG_FIH_OV5640AF ) || defined(CONFIG_FIH_OV7692)
static struct vreg *vreg_l17= NULL;
static struct vreg *vreg_l19= NULL;
int camera_power_on = 0;
static void msm_camera_vreg_config(int vreg_en)
{
	int rc = 0;

    printk(KERN_ERR "[camera] Enter %s vreg_en=%d \n", __func__,vreg_en);
	if (vreg_l17 == NULL) {
		vreg_l17 = vreg_get(NULL, "bt");
		if (IS_ERR(vreg_l17)) {
			pr_err("%s: vreg_get(%s) failed (%ld)\n",
				__func__, "bt", PTR_ERR(vreg_l17));
			return;
		}

		rc = vreg_set_level(vreg_l17, 1850);
		if (rc) {
			pr_err("%s: L17 set_level failed (%d)\n",
				__func__, rc);
		}
	}

	if (vreg_l19 == NULL) {
		vreg_l19 = vreg_get(NULL, "wlan4");
		if (IS_ERR(vreg_l19)) {
			pr_err("%s: vreg_get(%s) failed (%ld)\n",
				__func__, "wlan4", PTR_ERR(vreg_l19));
			return;
		}

		rc = vreg_set_level(vreg_l19, 2850);
		if (rc) {
			pr_err("%s: L19 set level failed (%d)\n",
				__func__, rc);
		}
	}

	if (vreg_en) {
		rc = vreg_enable(vreg_l17);
		if (rc) {
			pr_err("%s: L17 enable failed (%d)\n",
				__func__, rc);
		}

        msleep(1);
		rc = vreg_enable(vreg_l19);
		if (rc) {
			pr_err("%s: L19 enable failed (%d)\n",
				__func__, rc);           
		}
         msleep(5);
         camera_power_on = 1;

	} else {
		rc = vreg_disable(vreg_l19);
		if (rc) {
			pr_err("%s: L19 disable failed (%d)\n",
				__func__, rc);
		}

        msleep(1);
		rc = vreg_disable(vreg_l17);
		if (rc) {
			pr_err("%s: L17 disable failed (%d)\n",
				__func__, rc);
		}
         msleep(1);
         camera_power_on = 0;
	}
}
#endif

static struct vreg *vreg_l10= NULL;
static void msm_vcm_vreg_config(int vreg_en)
{
	int rc = 0;

    if (vreg_l10 == NULL) {
        vreg_l10 = vreg_get(NULL, "emmc");        
        if (IS_ERR(vreg_l10)) {
            pr_err("%s: vreg_get(%s) failed (%ld)\n",
                __func__, "emmc", PTR_ERR(vreg_l10));
            return;
        }

        rc = vreg_set_level(vreg_l10, 2850);
        if (rc) {
            pr_err("%s: L10 set level failed (%d)\n",
                __func__, rc);
        }
    }

    if (vreg_en) {
       rc = vreg_enable(vreg_l10);
       if (rc) 
           pr_err(KERN_ERR "%s: L10 enable failed (%d)\n",__func__, rc);
	} else {
            rc = vreg_disable(vreg_l10);
            if (rc) 
                pr_err("%s: L10 disable failed (%d)\n",__func__, rc);
	}
}

//NPM PR1.5 VCM vreg=->L19 ++
static void msm_vcm_vreg_config_v1(int vreg_en)
{
	int rc = 0;

    if (vreg_l19 == NULL) {
        vreg_l19 = vreg_get(NULL, "wlan4");        
        if (IS_ERR(vreg_l19)) {
            pr_err("%s: vreg_get(%s) failed (%ld)\n",
                __func__, "wlan4", PTR_ERR(vreg_l19));
            return;
        }

        rc = vreg_set_level(vreg_l19, 2800);
        if (rc) {
            pr_err("%s: L19 set level failed (%d)\n",
                __func__, rc);
        }
    }

    if (vreg_en) {
       rc = vreg_enable(vreg_l19);
       if (rc) 
           pr_err(KERN_ERR "%s: L19 enable failed (%d)\n",__func__, rc);
	} else {
            rc = vreg_disable(vreg_l19);
            if (rc) 
                pr_err("%s: L19 disable failed (%d)\n",__func__, rc);
	}
}
//NPM PR1.5 VCM vreg=->L19 --

static int config_gpio_table(uint32_t *table, int len)
{
	int rc = 0, i = 0;

	for (i = 0; i < len; i++) {
		rc = gpio_tlmm_config(table[i], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s not able to get gpio\n", __func__);
			for (i--; i >= 0; i--)
				gpio_tlmm_config(camera_off_gpio_table[i],
							GPIO_CFG_ENABLE);
			break;
		}
	}
	return rc;
}

static int config_camera_on_gpios_rear(void)
{
	int rc = 0;

	rc = config_gpio_table(camera_on_gpio_table,
			ARRAY_SIZE(camera_on_gpio_table));
	if (rc < 0) {
		pr_err("%s: CAMSENSOR gpio table request"
		"failed\n", __func__);
		return rc;
	}

	return rc;
}

static void config_camera_off_gpios_rear(void)
{
	config_gpio_table(camera_off_gpio_table,
			ARRAY_SIZE(camera_off_gpio_table));
}

static int config_camera_on_gpios_front(void)
{
	int rc = 0;

	rc = config_gpio_table(camera_on_gpio_table,
			ARRAY_SIZE(camera_on_gpio_table));
	if (rc < 0) {
		pr_err("%s: CAMSENSOR gpio table request"
			"failed\n", __func__);
		return rc;
	}

	return rc;
}

static void config_camera_off_gpios_front(void)
{
	config_gpio_table(camera_off_gpio_table,
			ARRAY_SIZE(camera_off_gpio_table));
}

static int config_camera_v2_on_gpios_rear(void)
{
	int rc = 0;
    if (camera_power_on == 0)
    {
        msm_camera_vreg_config(1);
    }
	rc = config_gpio_table(camera_on_gpio_table_v2,
			ARRAY_SIZE(camera_on_gpio_table_v2));
	if (rc < 0) {
		pr_err("%s: CAMSENSOR gpio table request"
		"failed\n", __func__);
		return rc;
	}

	return rc;
}

static void config_camera_v2_off_gpios_rear(void)
{
//    msm_camera_vreg_config(0);
	config_gpio_table(camera_off_gpio_table,
			ARRAY_SIZE(camera_off_gpio_table));
}

static int config_camera_v2_on_gpios_front(void)
{
	int rc = 0;
    if (camera_power_on == 0)
    {
        msm_camera_vreg_config(1);
    }
	rc = config_gpio_table(camera_on_gpio_table_v2,
			ARRAY_SIZE(camera_on_gpio_table_v2));
	if (rc < 0) {
		pr_err("%s: CAMSENSOR gpio table request"
			"failed\n", __func__);
		return rc;
	}

	return rc;
}

static void config_camera_v2_off_gpios_front(void)
{
//    msm_camera_vreg_config(0);
	config_gpio_table(camera_off_gpio_table,
			ARRAY_SIZE(camera_off_gpio_table));
}


static int config_camera_v3_on_gpios_rear(void)
{
	int rc = 0;

    pr_err("%s\n", __func__);

    rc = gpio_request(126, "mt9e013_r180");
    if (!rc) {
		rc = gpio_direction_output(126, 1);
		msleep(10);  //10
        gpio_free(126);
    }else{
		pr_err("%s: gpio_request failed\n", __func__);
	}

    msm_camera_vreg_config(1);
    msm_vcm_vreg_config(1);

    rc = config_gpio_table(camera_on_gpio_table,
			ARRAY_SIZE(camera_on_gpio_table));
	if (rc < 0) {
		pr_err("%s: CAMSENSOR gpio table request"
		"failed\n", __func__);
		return rc;
	}

	return rc;
}

static void config_camera_v3_off_gpios_rear(void)
{
	int rc = 0;

    pr_err("%s\n", __func__);
    rc = gpio_request(126, "mt9e013_r180");
    if (!rc) {
		rc = gpio_direction_output(126, 0);
		msleep(10);  //10
        gpio_free(126);
    }else{
		pr_err("%s: gpio_request failed\n", __func__);
	}

    msm_camera_vreg_config(0);
    msm_vcm_vreg_config(0);
	rc = config_gpio_table(camera_off_gpio_table,
			ARRAY_SIZE(camera_off_gpio_table));
	if (rc < 0) {
		pr_err("%s: config_gpio_table failed\n", __func__);
	}
}

static int config_camera_v3_on_gpios_front(void)
{
	int rc = 0;

    pr_err("%s\n", __func__);

    rc = gpio_request(126, "mt9e013_r180");/**To fix the main CMR lock SDA issue*/
    if (!rc) {
        rc = gpio_direction_output(126, 1);
        msleep(10);  //10
        gpio_free(126);
    }else{
        pr_err("%s: gpio_request failed\n", __func__);
    }

    rc = gpio_request(116, "ov7692_pwd");
    if (!rc) {
      rc = gpio_direction_output(116, 1);
      msleep(10);  //10
      gpio_free(116);
    }else{
      pr_err("%s: gpio_request failed\n", __func__);
    }
    msm_camera_vreg_config(1);

	rc = config_gpio_table(camera_on_gpio_table,
			ARRAY_SIZE(camera_on_gpio_table));
	if (rc < 0) {
		pr_err("%s: CAMSENSOR gpio table request"
			"failed\n", __func__);
	}
	return rc;
}
static void config_camera_v3_off_gpios_front(void)
{
    int rc = 0;

    pr_err("%s\n", __func__);
    
    rc = config_gpio_table(camera_off_gpio_table,
            ARRAY_SIZE(camera_off_gpio_table));
    if (rc < 0) {
        pr_err("%s: CAMSENSOR gpio table request"
            "failed\n", __func__);
    }

    msm_camera_vreg_config(0);

    rc = gpio_request(126, "mt9e013_r180");/**To fix the main CMR lock SDA issue*/
    if (!rc) {
        rc = gpio_direction_output(126, 0);
        msleep(10);  //10
        gpio_free(126);
    }else{
        pr_err("%s: gpio_request failed\n", __func__);
    }

}
//IRMPrime

static int config_camera_v4_on_gpios_rear(void)
{
	int rc = 0;

    msm_vcm_vreg_config(1);
	rc = config_gpio_table(camera_on_gpio_table_v2,
			ARRAY_SIZE(camera_on_gpio_table_v2));
	if (rc < 0) {
		pr_err("%s: CAMSENSOR gpio table request"
		"failed\n", __func__);
		return rc;
	}

	return rc;
}

static void config_camera_v4_off_gpios_rear(void)
{
    msm_vcm_vreg_config(0);
	config_gpio_table(camera_off_gpio_table,
			ARRAY_SIZE(camera_off_gpio_table));
}

//NPM PR1.5 ++
static int config_camera_v5_on_gpios_rear(void)
{
	int rc = 0;

    msm_vcm_vreg_config_v1(1);
	rc = config_gpio_table(camera_on_gpio_table_v2,
			ARRAY_SIZE(camera_on_gpio_table_v2));
	if (rc < 0) {
		pr_err("%s: CAMSENSOR gpio table request"
		"failed\n", __func__);
		return rc;
	}

	return rc;
}

static void config_camera_v5_off_gpios_rear(void)
{
    msm_vcm_vreg_config_v1(0);
	config_gpio_table(camera_off_gpio_table,
			ARRAY_SIZE(camera_off_gpio_table));
}
//NPM PR1.5 --

static int config_camera_v4_on_gpios_front(void)
{
	int rc = 0;

	rc = config_gpio_table(camera_on_gpio_table_v2,
			ARRAY_SIZE(camera_on_gpio_table_v2));
	if (rc < 0) {
		pr_err("%s: CAMSENSOR gpio table request"
			"failed\n", __func__);
		return rc;
	}

	return rc;
}

static struct msm_camera_device_platform_data msm_camera_device_data_rear = {
	.camera_gpio_on  = config_camera_on_gpios_rear,
	.camera_gpio_off = config_camera_off_gpios_rear,
	.ioext.csiphy = 0xA1000000,
	.ioext.csisz  = 0x00100000,
	.ioext.csiirq = INT_CSI_IRQ_1,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 192000000,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

static struct msm_camera_device_platform_data msm_camera_device_data_front = {
	.camera_gpio_on  = config_camera_on_gpios_front,
	.camera_gpio_off = config_camera_off_gpios_front,
	.ioext.csiphy = 0xA0F00000,
	.ioext.csisz  = 0x00100000,
	.ioext.csiirq = INT_CSI_IRQ_0,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 192000000,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};


static struct msm_camera_device_platform_data msm_camera_v2_device_data_rear = {
	.camera_gpio_on  = config_camera_v2_on_gpios_rear,
	.camera_gpio_off = config_camera_v2_off_gpios_rear,
	.ioext.csiphy = 0xA1000000,
	.ioext.csisz  = 0x00100000,
	.ioext.csiirq = INT_CSI_IRQ_1,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 192000000,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

static struct msm_camera_device_platform_data msm_camera_v2_device_data_front = {
	.camera_gpio_on  = config_camera_v2_on_gpios_front,
	.camera_gpio_off = config_camera_v2_off_gpios_front,
	.ioext.csiphy = 0xA0F00000,
	.ioext.csisz  = 0x00100000,
	.ioext.csiirq = INT_CSI_IRQ_0,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 192000000,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

static struct msm_camera_device_platform_data msm_camera_v3_device_data_rear = {
	.camera_gpio_on  = config_camera_v3_on_gpios_rear,
	.camera_gpio_off = config_camera_v3_off_gpios_rear,
	.ioext.csiphy = 0xA1000000,
	.ioext.csisz  = 0x00100000,
	.ioext.csiirq = INT_CSI_IRQ_1,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 192000000,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

static struct msm_camera_device_platform_data msm_camera_v3_device_data_front = {
	.camera_gpio_on  = config_camera_v3_on_gpios_front,
	.camera_gpio_off = config_camera_v3_off_gpios_front,
	.ioext.csiphy = 0xA0F00000,
	.ioext.csisz  = 0x00100000,
	.ioext.csiirq = INT_CSI_IRQ_0,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 192000000,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

//IRMPrime
static struct msm_camera_device_platform_data msm_camera_v4_device_data_rear = {
	.camera_gpio_on  = config_camera_v4_on_gpios_rear,
	.camera_gpio_off = config_camera_v4_off_gpios_rear,
	.ioext.csiphy = 0xA1000000,
	.ioext.csisz  = 0x00100000,
	.ioext.csiirq = INT_CSI_IRQ_1,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 192000000,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

//NPM PR1.5 ++
static struct msm_camera_device_platform_data msm_camera_v5_device_data_rear = {
	.camera_gpio_on  = config_camera_v5_on_gpios_rear,
	.camera_gpio_off = config_camera_v5_off_gpios_rear,
	.ioext.csiphy = 0xA1000000,
	.ioext.csisz  = 0x00100000,
	.ioext.csiirq = INT_CSI_IRQ_1,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 192000000,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};
//NPM PR1.5 --

static struct msm_camera_device_platform_data msm_camera_v4_device_data_front = {
	.camera_gpio_on  = config_camera_v4_on_gpios_front,
	.camera_gpio_off = config_camera_off_gpios_front,
	.ioext.csiphy = 0xA0F00000,
	.ioext.csisz  = 0x00100000,
	.ioext.csiirq = INT_CSI_IRQ_0,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 192000000,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

#ifdef CONFIG_FIH_OV5640AF
// TBP
static struct msm_camera_sensor_platform_info ov5640af_sensor_7627a_info_v1 = {
	.mount_angle = 90
};

static struct msm_camera_sensor_flash_data flash_ov5640af = {
	.flash_type             = MSM_CAMERA_FLASH_LED,
	.flash_src              = &msm_flash_src_v2
};

static struct msm_camera_sensor_info msm_camera_sensor_ov5640af_data_v1 = {
	.sensor_name    = "ov5640af",
	.sensor_reset_enable = 1,
	.sensor_reset   = 120,
	.sensor_pwd             = 126,
	.vcm_pwd                = 0,
	.vcm_enable             = 0,
	.pdata                  = &msm_camera_v2_device_data_rear,
	.flash_data             = &flash_ov5640af,
	.sensor_platform_info   = &ov5640af_sensor_7627a_info_v1,
	.csi_if                 = 1
};

static struct platform_device msm_camera_sensor_ov5640af_v1 = {
	.name   = "msm_camera_ov5640af",
	.dev    = {
		.platform_data = &msm_camera_sensor_ov5640af_data_v1,
	},
};

// TNQ
static struct msm_camera_sensor_platform_info ov5640af_sensor_7627a_info_v2 = {
	.mount_angle = 0
};

static struct msm_camera_sensor_info msm_camera_sensor_ov5640af_data_v2 = {
	.sensor_name    = "ov5640af",
	.sensor_reset_enable = 1,
	.sensor_reset   = 120,
	.sensor_pwd             = 126,
	.vcm_pwd                = 0,
	.vcm_enable             = 0,
	.pdata                  = &msm_camera_v2_device_data_rear,
	.flash_data             = &flash_ov5640af,
	.sensor_platform_info   = &ov5640af_sensor_7627a_info_v2,
	.csi_if                 = 1
};

static struct platform_device msm_camera_sensor_ov5640af_v2 = {
	.name   = "msm_camera_ov5640af",
	.dev    = {
		.platform_data = &msm_camera_sensor_ov5640af_data_v2,
	},
};
static struct i2c_board_info ov5640af_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("ov5640af", 0x78 >> 1),
	},
};
#endif

#ifdef CONFIG_FIH_OV7692
static struct msm_camera_sensor_platform_info ov7692_sensor_7627a_info = {
	.mount_angle = 0
};

static struct msm_camera_sensor_flash_data flash_ov7692 = {
	.flash_type             = MSM_CAMERA_FLASH_NONE,
	.flash_src              = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_ov7692_data = {
	.sensor_name    = "ov7692",
	.sensor_reset_enable = 0,
	.sensor_reset   = 1,
	.sensor_pwd             = 116,
	.vcm_pwd                = 1,
	.vcm_enable             = 0,
	.pdata                  = &msm_camera_v2_device_data_front,
	.flash_data             = &flash_ov7692,
	.sensor_platform_info   = &ov7692_sensor_7627a_info,
	.csi_if                 = 1
};

static struct platform_device msm_camera_sensor_ov7692 = {
	.name   = "msm_camera_ov7692",
	.dev    = {
		.platform_data = &msm_camera_sensor_ov7692_data,
	},
};

static struct msm_camera_sensor_platform_info ov7692_sensor_7627a_info_v2 = {
	.mount_angle = 270
};

static struct msm_camera_sensor_info msm_camera_sensor_ov7692_v2_data = {
	.sensor_name    = "ov7692",
	.sensor_reset_enable = 0,
	.sensor_reset   = 1,
	.sensor_pwd             = 116,
	.vcm_pwd                = 1,
	.vcm_enable             = 0,
	.pdata                  = &msm_camera_v3_device_data_front,
	.flash_data             = &flash_ov7692,
	.sensor_platform_info   = &ov7692_sensor_7627a_info_v2,
	.csi_if                 = 1
};

static struct platform_device msm_camera_sensor_ov7692_v2 = {
	.name   = "msm_camera_ov7692",
	.dev    = {
		.platform_data = &msm_camera_sensor_ov7692_v2_data,
	},
};

//TBP PR2.5 +
static struct msm_camera_sensor_platform_info ov7692_sensor_7627a_info_v3 = {
	.mount_angle = 270
};

static struct msm_camera_sensor_info msm_camera_sensor_ov7692_data_v3 = {
	.sensor_name    = "ov7692",
	.sensor_reset_enable = 0,
	.sensor_reset   = 1,
	.sensor_pwd             = 116,
	.vcm_pwd                = 1,
	.vcm_enable             = 0,
	.pdata                  = &msm_camera_v2_device_data_front,
	.flash_data             = &flash_ov7692,
	.sensor_platform_info   = &ov7692_sensor_7627a_info_v3,
	.csi_if                 = 1
};

static struct platform_device msm_camera_sensor_ov7692_v3 = {
	.name   = "msm_camera_ov7692",
	.dev    = {
		.platform_data = &msm_camera_sensor_ov7692_data_v3,
	},
};
//IRMPrime
static struct msm_camera_sensor_platform_info ov7692_sensor_7627a_info_v4 = {
	.mount_angle = 270
};


static struct msm_camera_sensor_info msm_camera_sensor_ov7692_data_v4 = {
	.sensor_name    = "ov7692",
	.sensor_reset_enable = 0,
	.sensor_reset   = 1,
	.sensor_pwd             = 91,//IRM2
	.vcm_pwd                = 1,
	.vcm_enable             = 0,
	.pdata                  = &msm_camera_v4_device_data_front,
	.flash_data             = &flash_ov7692,
	.sensor_platform_info   = &ov7692_sensor_7627a_info_v4,
	.csi_if                 = 1
};

static struct platform_device msm_camera_sensor_ov7692_v4 = {
	.name   = "msm_camera_ov7692",
	.dev    = {
		.platform_data = &msm_camera_sensor_ov7692_data_v4,
	},
};
//TBP PR2.5 -

static struct i2c_board_info ov7692_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("ov7692", 0x78),
	},
};
#endif

#if defined(CONFIG_MT9E013) && !defined(CONFIG_USE_QUALCOMM_DEFT_DEVICES)
static struct msm_camera_sensor_platform_info mt9e013_sensor_7627a_info = {
	.mount_angle = 90
};

static struct msm_camera_sensor_flash_data flash_mt9e013 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9e013_data = {
	.sensor_name    = "mt9e013",
	.sensor_reset   = 126, //0, // Patch: MT9E013CameraDriver
	.sensor_reset_enable = 1,
	.sensor_pwd     = 0, //85, // Patch: MT9E013CameraDriver
	.vcm_pwd        = 125, //1, // Patch: MT9E013CameraDriver
	.vcm_enable     = 1, //0,
	.pdata          = &msm_camera_device_data_rear,
	.flash_data     = &flash_mt9e013,
	.sensor_platform_info   = &mt9e013_sensor_7627a_info,
	.csi_if         = 1
};

static struct platform_device msm_camera_sensor_mt9e013 = {
	.name      = "msm_camera_mt9e013",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9e013_data,
	},
};

static struct i2c_board_info mt9e013_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("mt9e013", 0x6C >> 2),
	},
};
#endif /* CONFIG_MT9E013 && !CONFIG_USE_QUALCOMM_DEFT_DEVICES */

#if defined(CONFIG_FIH_MT9E013_R180) && !defined(CONFIG_USE_QUALCOMM_DEFT_DEVICES)
static struct msm_camera_sensor_platform_info mt9e013_sensor_7627a_info_v2 = {
	.mount_angle = 90
};

static struct msm_camera_sensor_flash_data flash_mt9e013_v2 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9e013_data_v2 = {
	.sensor_name    = "mt9e013_r180",
	.sensor_reset   = 126, //0, // Patch: MT9E013CameraDriver
	.sensor_reset_enable = 1,
	.sensor_pwd     = 0, //85, // Patch: MT9E013CameraDriver
	.vcm_pwd        = 125, //1, // Patch: MT9E013CameraDriver
	.vcm_enable     = 1, //0,
	.pdata          = &msm_camera_v3_device_data_rear,
	.flash_data     = &flash_mt9e013_v2,
	.sensor_platform_info   = &mt9e013_sensor_7627a_info_v2,
	.csi_if         = 1
};

static struct platform_device msm_camera_sensor_mt9e013_v2 = {
	.name      = "msm_camera_mt9e013_r180",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9e013_data_v2,
	},
};

static struct i2c_board_info mt9e013_i2c_info_v2[] __initdata = {
	{
		I2C_BOARD_INFO("mt9e013_r180", 0x6C >> 2),
	},
};
//IRMPrime

static struct msm_camera_sensor_platform_info mt9e013_sensor_7627a_info_v3 = {
	.mount_angle = 90
};

static struct msm_camera_sensor_flash_data flash_mt9e013_v3 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_v3
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9e013_data_v3 = {
	.sensor_name    = "mt9e013",
	.sensor_reset   = 90, 
	.sensor_reset_enable = 1,
	.sensor_pwd     = 0, //85, // Patch: MT9E013CameraDriver
	.vcm_pwd        = 0, 
	.vcm_enable     = 1, //0,
	.pdata          = &msm_camera_v4_device_data_rear,
	.flash_data     = &flash_mt9e013_v3,
	.sensor_platform_info   = &mt9e013_sensor_7627a_info_v3,
	.csi_if         = 1
};

static struct platform_device msm_camera_sensor_mt9e013_v3 = {
	.name      = "msm_camera_mt9e013",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9e013_data_v3,
	},
};

//NPM PR1.5 ++
static struct msm_camera_sensor_info msm_camera_sensor_mt9e013_data_v4 = {
	.sensor_name    = "mt9e013",
	.sensor_reset   = 90, 
	.sensor_reset_enable = 1,
	.sensor_pwd     = 0, //85, // Patch: MT9E013CameraDriver
	.vcm_pwd        = 0, 
	.vcm_enable     = 1, //0,
	.pdata          = &msm_camera_v5_device_data_rear,
	.flash_data     = &flash_mt9e013_v3,
	.sensor_platform_info   = &mt9e013_sensor_7627a_info_v3,
	.csi_if         = 1
};

static struct platform_device msm_camera_sensor_mt9e013_v4 = {
	.name      = "msm_camera_mt9e013",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9e013_data_v4,
	},
};
//NPM PR1.5 --

static struct i2c_board_info mt9e013_i2c_info_v3[] __initdata = {
	{
		I2C_BOARD_INFO("mt9e013", 0x6C >> 2),
	},
};
#endif

#ifdef CONFIG_MT9V115
static struct msm_camera_sensor_platform_info mt9v115_sensor_7627a_info = {
	.mount_angle = 270
};

static struct msm_camera_sensor_flash_data flash_mt9v115 = {
	.flash_type = MSM_CAMERA_FLASH_NONE,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9v115_data = {
	.sensor_name    = "mt9v115",
	.sensor_reset   = 116,
	.sensor_reset_enable = 1,
//	.sensor_pwd     = 0,
//	.vcm_pwd        = 0,
//	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data_front,
	.flash_data     = &flash_mt9v115,
	.sensor_platform_info   = &mt9v115_sensor_7627a_info,
	.csi_if         = 1
};

static struct platform_device msm_camera_sensor_mt9v115 = {
	.name      = "msm_camera_mt9v115",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9v115_data,
	},
};

static struct i2c_board_info mt9v115_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("mt9v115", 0x7A >> 1),
	},
};
#endif /* CONFIG_MT9V115 */

int __init camera_init(void)
{
	void *hdl;
#if defined(CONFIG_MT9E013) && !defined(CONFIG_USE_QUALCOMM_DEFT_DEVICES)
	hdl = DECL_VERSION_CTRL(DRIVER_TYPE_MAIN_CAM, MAIN_CAM_MT9E013_V1);

	REGISTER_DYN_PLATFORM_DEVICE(hdl, &msm_camera_sensor_mt9e013);

	ADD_DYN_I2C_DEVICE(hdl, MSM_GSBI0_QUP_I2C_BUS_ID, mt9e013_i2c_info);


//IRMPrime

    hdl = DECL_VERSION_CTRL(DRIVER_TYPE_MAIN_CAM, MAIN_CAM_MT9E013_V3);

	REGISTER_DYN_PLATFORM_DEVICE(hdl, &msm_camera_sensor_mt9e013_v3);

	ADD_DYN_I2C_DEVICE(hdl, MSM_GSBI0_QUP_I2C_BUS_ID, mt9e013_i2c_info_v3);

//NPM PR1.5
    hdl = DECL_VERSION_CTRL(DRIVER_TYPE_MAIN_CAM, MAIN_CAM_MT9E013_V4);

	REGISTER_DYN_PLATFORM_DEVICE(hdl, &msm_camera_sensor_mt9e013_v4);

	ADD_DYN_I2C_DEVICE(hdl, MSM_GSBI0_QUP_I2C_BUS_ID, mt9e013_i2c_info_v3);
#endif	

#if defined(CONFIG_FIH_MT9E013_R180) && !defined(CONFIG_USE_QUALCOMM_DEFT_DEVICES)

    hdl = DECL_VERSION_CTRL(DRIVER_TYPE_MAIN_CAM, MAIN_CAM_MT9E013_V2);

	REGISTER_DYN_PLATFORM_DEVICE(hdl, &msm_camera_sensor_mt9e013_v2);

	ADD_DYN_I2C_DEVICE(hdl, MSM_GSBI0_QUP_I2C_BUS_ID, mt9e013_i2c_info_v2);
#endif

#ifdef CONFIG_MT9V115
	hdl = DECL_VERSION_CTRL(DRIVER_TYPE_FRONT_CAM, FRONT_CAM_MT9V115_V1);

	REGISTER_DYN_PLATFORM_DEVICE(hdl, &msm_camera_sensor_mt9v115);

	ADD_DYN_I2C_DEVICE(hdl, MSM_GSBI0_QUP_I2C_BUS_ID, mt9v115_i2c_info);
#endif	

#ifdef CONFIG_FIH_OV5640AF
	hdl = DECL_VERSION_CTRL(DRIVER_TYPE_MAIN_CAM, MAIN_CAM_OV5640_V1);
	REGISTER_DYN_PLATFORM_DEVICE(hdl, &msm_camera_sensor_ov5640af_v1);
	ADD_DYN_I2C_DEVICE(hdl, MSM_GSBI0_QUP_I2C_BUS_ID, ov5640af_i2c_info);
	
	hdl = DECL_VERSION_CTRL(DRIVER_TYPE_MAIN_CAM, MAIN_CAM_OV5640_V2);
	REGISTER_DYN_PLATFORM_DEVICE(hdl, &msm_camera_sensor_ov5640af_v2);
	ADD_DYN_I2C_DEVICE(hdl, MSM_GSBI0_QUP_I2C_BUS_ID, ov5640af_i2c_info);
#endif

#ifdef CONFIG_FIH_OV7692
	hdl = DECL_VERSION_CTRL(DRIVER_TYPE_FRONT_CAM, FRONT_CAM_OV7692_V1);

	REGISTER_DYN_PLATFORM_DEVICE(hdl, &msm_camera_sensor_ov7692);

	ADD_DYN_I2C_DEVICE(hdl, MSM_GSBI0_QUP_I2C_BUS_ID, ov7692_i2c_info);

	hdl = DECL_VERSION_CTRL(DRIVER_TYPE_FRONT_CAM, FRONT_CAM_OV7692_V2);

	REGISTER_DYN_PLATFORM_DEVICE(hdl, &msm_camera_sensor_ov7692_v2);

	ADD_DYN_I2C_DEVICE(hdl, MSM_GSBI0_QUP_I2C_BUS_ID, ov7692_i2c_info);
	
//TBP PR2.5	
	hdl = DECL_VERSION_CTRL(DRIVER_TYPE_FRONT_CAM, FRONT_CAM_OV7692_V3);

	REGISTER_DYN_PLATFORM_DEVICE(hdl, &msm_camera_sensor_ov7692_v3);

	ADD_DYN_I2C_DEVICE(hdl, MSM_GSBI0_QUP_I2C_BUS_ID, ov7692_i2c_info);
//IRMPrime
	hdl = DECL_VERSION_CTRL(DRIVER_TYPE_FRONT_CAM, FRONT_CAM_OV7692_V4);

	REGISTER_DYN_PLATFORM_DEVICE(hdl, &msm_camera_sensor_ov7692_v4);

	ADD_DYN_I2C_DEVICE(hdl, MSM_GSBI0_QUP_I2C_BUS_ID, ov7692_i2c_info);
#endif
#ifdef CONFIG_MSM_CAMERA_FLASH_SC628A
	hdl = DECL_VERSION_CTRL(DRIVER_TYPE_FLASH_LIGHT, FLASH_LIGHT_V1);
	ADD_DYN_I2C_DEVICE(hdl, MSM_GSBI0_QUP_I2C_BUS_ID, flash_i2c_info);

	hdl = DECL_VERSION_CTRL(DRIVER_TYPE_FLASH_LIGHT, FLASH_LIGHT_V2);
	ADD_DYN_I2C_DEVICE(hdl, MSM_GSBI1_QUP_I2C_BUS_ID, flash_i2c_info);

    //for IRMPrime
    hdl = DECL_VERSION_CTRL(DRIVER_TYPE_FLASH_LIGHT, FLASH_LIGHT_V3);
	ADD_DYN_I2C_DEVICE(hdl, MSM_GSBI0_QUP_I2C_BUS_ID, flash_i2c_info);
#endif
	(void)hdl;

	return 0;	
}
#else	/* !CONFIG_MSM_CAMERA */
int __init camera_init(void)
{
	return 0;	
}
#endif	/* !CONFIG_MSM_CAMERA */
