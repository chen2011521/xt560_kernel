
#include <fih/dynloader.h>

static int __init mt9e013_init(void* hdl)
{
	return platform_driver_register(&msm_camera_driver);
}

DECLARE_DYN_LOADER(DRIVER_TYPE_MAIN_CAM, 
			MAIN_CAM_MT9E013_V1, 
			mt9e013_init, LEVEL6);
static int __init mt9e013_r180_init_v1(void* hdl)
{
     printk(KERN_INFO"[fih_cam] %s\n",__func__);

	return platform_driver_register(&msm_camera_driver);
}
DECLARE_DYN_LOADER(DRIVER_TYPE_MAIN_CAM, 
			MAIN_CAM_MT9E013_V3, 
			mt9e013_r180_init_v1, LEVEL6);
