#ifndef _qt602240_h_
#define _qt602240_h_

//Div2-D5-Peripheral-FG-AddMultiTouch-01*[
#ifndef CONFIG_FIH_FTM
#define QT602240_MT
#endif
//Div2-D5-Peripheral-FG-AddMultiTouch-01*]

#define GPIO_TP_INT_N               20
#define GPIO_TP_RST_N               7

#define DEFAULT_REG_NUM             64

#define MSB(uint16_t)  (((uint8_t* )&uint16_t)[1])
#define LSB(uint16_t)  (((uint8_t* )&uint16_t)[0])

#define TOUCH_DEVICE_NAME           "qt602240"
#define TOUCH_DEVICE_VREG           "emmc"
#define TOUCH_DEVICE_I2C_ADDRESS    0x4Au

#define LOW                         0
#define HIGH                        1
#define FALSE                       0
#define TRUE                        1

#define TS_MAX_POINTS               5

#define TS_MIN_Y                    0
#define TS_MIN_X                    0
#define TS_MAX_KEYS                 4

#define DB_LEVEL1                   1
#define DB_LEVEL2                   2
//#define DEBUG_TOUCH

#define TCH_ERR(msg, ...) printk("[TOUCH_ERR] %s : " msg "\n", __FUNCTION__, ## __VA_ARGS__)

#ifdef DEBUG_TOUCH
#define TCH_MSG(msg, ...) printk("[TOUCH] %s : " msg "\n", __FUNCTION__, ## __VA_ARGS__)
#else
#define TCH_MSG(msg, ...) 
#endif

struct point_info{
    int x;
    int y;
    int num;
    int first_area;
    int last_area;
};

struct qt602240_info {
    struct i2c_client *client;
    struct input_dev  *touch_input;
    struct input_dev  *keyevent_input;
    struct work_struct wqueue;
    struct point_info points[TS_MAX_POINTS];
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend es;
#endif
    int irq;
    bool suspend_state;
    int first_finger_id;
    bool facedetect;
	bool key_state[TS_MAX_KEYS];
    uint8_t T7[3];
} *qt602240;

enum ts_state {
    NO_TOUCH = 0,
    PRESS_TOUCH_AREA,
    PRESS_KEY1_AREA,
    PRESS_KEY2_AREA,
    PRESS_KEY3_AREA,
	PRESS_KEY4_AREA
};

#endif