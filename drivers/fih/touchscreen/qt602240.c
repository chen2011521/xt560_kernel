#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <linux/slab.h>

#include <fih/dynloader.h>
#include <fih/qt602240_info_block_driver.h>
#include <fih/qt602240_std_objects_driver.h>
#include <fih/qt602240_touch_driver.h>
#include <fih/qt602240.h>

#include <asm/uaccess.h>
#include <mach/msm_iomap.h>
#include <linux/cmdbgapi.h>

static info_block_t *info_block;
static uint16_t g_I2CAddress;
static uint16_t g_RegAddr[DEFAULT_REG_NUM];
static uint8_t g_MsgData[8];
static uint8_t g_TouchAntitouch[80];
static uint16_t PositionX[10];
static uint16_t PositionY[10];
static uint8_t g_ReportID[256];
static uint8_t g_NVBackUp;

static uint32_t TS_MAX_X            = 1023;
static uint32_t TS_MAX_Y            = 975;
static uint32_t TS_BUTTON_W         = 80;
static uint32_t TS_BUTTON_XBOUND    = 15;
static uint32_t TS_MENUKEYL_X       = 50;
static uint32_t TS_MENUKEYR_X       = 205;
static uint32_t TS_HOMEKEYL_X       = 459;
static uint32_t TS_HOMEKEYR_X       = 614;
static uint32_t TS_BACKKEYL_X       = 818;
static uint32_t TS_BACKKEYR_X       = 973;
static uint32_t TS_SEARCHKEYL_X     = 1024;
static uint32_t TS_SEARCHKEYR_X     = 1024;

    
static int gpio_camreset	    = -1;
static int gpio_pwr		    = -1;
static int cal_check_flag = 0; 
static uint16_t ponit_num = 0;	
static uint16_t XY_Flag = 0;
static uint16_t Project_Flag = 0;

static u16 low_boundary = 6000;
static u16 high_boundary = 13000;


static int st_result = 0;
static int gpio_tp_int_n    = 20;
static int goio_tp_rst_n = 7; //add by qumeng for IRM2
static unsigned int qt_debug_level;

static struct delayed_work touch_delayed_work; // FIH-Div2-SW2-BSP, Ming

#define WATER
#define YUSHUN         0x34
#if 0
struct point_status{
        int fake_release;
        int press;
};
static struct point_status point_status_V[TS_MAX_POINTS];
#endif

enum {
	TOUCH_DEBUG_COMMON		= 1U << 0,
	TOUCH_DEBUG_EVENT		= 1U << 1,
	TOUCH_DEBUG_I2C			= 1U << 2,
	TOUCH_DEBUG_ATMEL		= 1U << 3,
	TOUCH_DEBUG_ISR			= 1U << 4,
};

static int touch_debug_mask = 0;

module_param_named(
	debug_mask, touch_debug_mask, uint, 0644);

#define TCH_DBG(level, message, ...) \
    do { \
        if ( touch_debug_mask & level) \
            printk("[TOUCH] %s : " message "\n", __FUNCTION__, ## __VA_ARGS__); \
    } while (0)

module_param_named(debug_level, qt_debug_level, int, S_IRUGO | S_IWUSR | S_IWGRP);

static int cap_touch_fw_version;
module_param_named(fw_version, cap_touch_fw_version, int, S_IRUGO | S_IWUSR | S_IWGRP);

module_param_named(st_result, st_result, int, S_IRUGO | S_IWUSR | S_IWGRP);

static char *cap_touch_fw_variant_version;
module_param_named(fw_variant_version, cap_touch_fw_variant_version, charp, S_IRUGO | S_IWUSR | S_IWGRP);
static void qt602240_ts_reset(void)
{   // Active RESET pin to wakeup ts
    int i;
    if(goio_tp_rst_n)
    {
        if (gpio_get_value(goio_tp_rst_n) == LOW)
            gpio_set_value(goio_tp_rst_n, HIGH);
        gpio_set_value(goio_tp_rst_n, LOW);
        mdelay(10);
        gpio_set_value(goio_tp_rst_n, HIGH);
        for (i=0; i<10; i++)
        {
            mdelay(20);
            if (gpio_get_value(gpio_tp_int_n) == LOW)
                return ;
        }

        TCH_ERR("Failed.");
    }
    else
        return;
}

static void qt602240_report(struct qt602240_info *ts, int i, int x, int y, int pressure)
{
    TCH_DBG(TOUCH_DEBUG_EVENT, "POS%d, X = %d, Y = %d, pressure = %d", i+1, x, y, pressure);
    
    if (pressure == NO_TOUCH)
    {   // Finger Up.
        if (y >= TS_MAX_Y + 10)   //Key Area
        {   //Key Area
            if (ts->points[i].first_area == PRESS_KEY1_AREA)
            {
                input_report_key(ts->keyevent_input, KEY_MENU, 0);
                TCH_MSG("POS%d Report MENU key Up.", i+1);
            }
            else if (ts->points[i].first_area == PRESS_KEY2_AREA)
            {
                input_report_key(ts->keyevent_input, KEY_HOME, 0);
                TCH_MSG("POS%d Report HOME key Up.", i+1);
            }
            else if (ts->points[i].first_area == PRESS_KEY3_AREA)
            {
                input_report_key(ts->keyevent_input, KEY_BACK, 0);
                TCH_MSG("POS%d Report BACK key Up.", i+1);
            }            
            else if (ts->points[i].first_area == PRESS_KEY4_AREA)
            {
                input_report_key(ts->keyevent_input, KEY_SEARCH, 0);
                TCH_MSG("POS%d Report SEARCH key Up.", i+1);
            }
        }
        if (ts->points[i].first_area == PRESS_TOUCH_AREA)
        {   // Report touch pen up event only if first tap in touch area.
            if (i == 0)
            {
            //Div2-D5-Peripheral-FG-AddMultiTouch-01*[
#ifdef QT602240_MT
                input_report_abs(ts->touch_input, ABS_MT_TOUCH_MAJOR, 0);
                input_report_abs(ts->touch_input, ABS_MT_POSITION_X, x);
                input_report_abs(ts->touch_input, ABS_MT_POSITION_Y, y);
#else
                input_report_abs(ts->touch_input, ABS_X, x);
                input_report_abs(ts->touch_input, ABS_Y, y);
                input_report_abs(ts->touch_input, ABS_PRESSURE, pressure);
                input_report_key(ts->touch_input, BTN_TOUCH, 0);
#endif
            }
            if (i == 1)
            {
#ifdef QT602240_MT
                input_report_abs(ts->touch_input, ABS_MT_TOUCH_MAJOR, 0);
                input_report_abs(ts->touch_input, ABS_MT_POSITION_X, x);
                input_report_abs(ts->touch_input, ABS_MT_POSITION_Y, y);
#else
                input_report_abs(ts->touch_input, ABS_HAT0X, x);
                input_report_abs(ts->touch_input, ABS_HAT0Y, y);
                input_report_abs(ts->touch_input, ABS_PRESSURE, pressure);
                input_report_key(ts->touch_input, BTN_2, 0);
#endif
            }
            if (i == 2)
            {
#ifdef QT602240_MT
                input_report_abs(ts->touch_input, ABS_MT_TOUCH_MAJOR, 0);
                input_report_abs(ts->touch_input, ABS_MT_POSITION_X, x);
                input_report_abs(ts->touch_input, ABS_MT_POSITION_Y, y);
#else
                input_report_abs(ts->touch_input, ABS_X, x);
                input_report_abs(ts->touch_input, ABS_Y, y);
                input_report_abs(ts->touch_input, ABS_PRESSURE, pressure);
                input_report_key(ts->touch_input, BTN_TOUCH, 0);
#endif
            }
            if (i == 3)
            {
#ifdef QT602240_MT
                input_report_abs(ts->touch_input, ABS_MT_TOUCH_MAJOR, 0);
                input_report_abs(ts->touch_input, ABS_MT_POSITION_X, x);
                input_report_abs(ts->touch_input, ABS_MT_POSITION_Y, y);
#else
                input_report_abs(ts->touch_input, ABS_X, x);
                input_report_abs(ts->touch_input, ABS_Y, y);
                input_report_abs(ts->touch_input, ABS_PRESSURE, pressure);
                input_report_key(ts->touch_input, BTN_TOUCH, 0);
#endif
            }
            if (i == 4)
            {
#ifdef QT602240_MT
                input_report_abs(ts->touch_input, ABS_MT_TOUCH_MAJOR, 0);
                input_report_abs(ts->touch_input, ABS_MT_POSITION_X, x);
                input_report_abs(ts->touch_input, ABS_MT_POSITION_Y, y);
#else
                input_report_abs(ts->touch_input, ABS_X, x);
                input_report_abs(ts->touch_input, ABS_Y, y);
                input_report_abs(ts->touch_input, ABS_PRESSURE, pressure);
                input_report_key(ts->touch_input, BTN_TOUCH, 0);
#endif
            }
#ifdef QT602240_MT
            input_mt_sync(ts->touch_input);
#endif
            //Div2-D5-Peripheral-FG-AddMultiTouch-01*]
        }
        
        ts->points[i].num = 0;
        ts->points[i].first_area = NO_TOUCH;
        ts->points[i].last_area = NO_TOUCH;
    }
    else
    {   // Finger Down.
        if (y < TS_MAX_Y)
        {   //Touch Area
            if (ts->points[i].num == 0)
                ts->points[i].first_area = PRESS_TOUCH_AREA;
            if (ts->points[i].first_area == PRESS_KEY1_AREA)
            {   // Flick from home key area to touch area.
                input_report_key(ts->keyevent_input, KEY_MENU, 0);
                TCH_MSG("POS%d Report MENU key Up.", i+1);
                ts->points[i].first_area = NO_TOUCH;
            }
            else if (ts->points[i].first_area == PRESS_KEY2_AREA)
            {   // Flick from menu key area to touch area.
                input_report_key(ts->keyevent_input, KEY_HOME, 0);
                TCH_MSG("POS%d Report HOME key Up.", i+1);
                ts->points[i].first_area = NO_TOUCH;
            }
            else if (ts->points[i].first_area == PRESS_KEY3_AREA)
            {   // Flick from back key area to touch area.
                input_report_key(ts->keyevent_input, KEY_BACK, 0);
                TCH_MSG("POS%d Report BACK key Up.", i+1);
                ts->points[i].first_area = NO_TOUCH;
            }
            else if (ts->points[i].first_area == PRESS_KEY4_AREA)   // Flick from search key area to touch area.
            {
                input_report_key(ts->keyevent_input, KEY_SEARCH, 0);
                TCH_MSG("POS%d Report SEARCH key Up.", i+1);
                ts->points[i].first_area = NO_TOUCH;
            }
            else if (ts->points[i].first_area == PRESS_TOUCH_AREA)
            {
                if (i == 0)
                {
                //Div2-D5-Peripheral-FG-AddMultiTouch-01*[
#ifdef QT602240_MT
                    input_report_abs(ts->touch_input, ABS_MT_TOUCH_MAJOR, 255);
                    input_report_abs(ts->touch_input, ABS_MT_POSITION_X, x);
                    input_report_abs(ts->touch_input, ABS_MT_POSITION_Y, y);
#else
                    input_report_abs(ts->touch_input, ABS_X, x);
                    input_report_abs(ts->touch_input, ABS_Y, y);
                    input_report_abs(ts->touch_input, ABS_PRESSURE, pressure);
                    input_report_key(ts->touch_input, BTN_TOUCH, 1);
#endif
                }
                if (i == 1)
                {
#ifdef QT602240_MT
                    input_report_abs(ts->touch_input, ABS_MT_TOUCH_MAJOR, 255);
                    input_report_abs(ts->touch_input, ABS_MT_POSITION_X, x);
                    input_report_abs(ts->touch_input, ABS_MT_POSITION_Y, y);
#else
                    input_report_abs(ts->touch_input, ABS_HAT0X, x);
                    input_report_abs(ts->touch_input, ABS_HAT0Y, y);
                    input_report_abs(ts->touch_input, ABS_PRESSURE, pressure);
                    input_report_key(ts->touch_input, BTN_2, 1);
#endif
                }
                if (i == 2)
                {
#ifdef QT602240_MT
                    input_report_abs(ts->touch_input, ABS_MT_TOUCH_MAJOR, 255);
                    input_report_abs(ts->touch_input, ABS_MT_POSITION_X, x);
                    input_report_abs(ts->touch_input, ABS_MT_POSITION_Y, y);
#else
                    input_report_abs(ts->touch_input, ABS_X, x);
                    input_report_abs(ts->touch_input, ABS_Y, y);
                    input_report_abs(ts->touch_input, ABS_PRESSURE, pressure);
                    input_report_key(ts->touch_input, BTN_TOUCH, 1);
#endif
                }
                if (i == 3)
                {
#ifdef QT602240_MT
                    input_report_abs(ts->touch_input, ABS_MT_TOUCH_MAJOR, 255);
                    input_report_abs(ts->touch_input, ABS_MT_POSITION_X, x);
                    input_report_abs(ts->touch_input, ABS_MT_POSITION_Y, y);
#else
                    input_report_abs(ts->touch_input, ABS_X, x);
                    input_report_abs(ts->touch_input, ABS_Y, y);
                    input_report_abs(ts->touch_input, ABS_PRESSURE, pressure);
                    input_report_key(ts->touch_input, BTN_TOUCH, 1);
#endif
                }
                if (i == 4)
                {
#ifdef QT602240_MT
                    input_report_abs(ts->touch_input, ABS_MT_TOUCH_MAJOR, 255);
                    input_report_abs(ts->touch_input, ABS_MT_POSITION_X, x);
                    input_report_abs(ts->touch_input, ABS_MT_POSITION_Y, y);
#else
                    input_report_abs(ts->touch_input, ABS_X, x);
                    input_report_abs(ts->touch_input, ABS_Y, y);
                    input_report_abs(ts->touch_input, ABS_PRESSURE, pressure);
                    input_report_key(ts->touch_input, BTN_TOUCH, 1);
#endif
                }
#ifdef QT602240_MT
                input_mt_sync(ts->touch_input);
#endif
                //Div2-D5-Peripheral-FG-AddMultiTouch-01*]
            }
			
            ts->points[i].last_area = PRESS_TOUCH_AREA;
        }
        else if(y >= TS_MAX_Y + 10)  //Key Area
        {   //Key Area
            if (TS_MENUKEYL_X < x && x < TS_MENUKEYR_X)	//Div5-BSP-CW-TouchKey-00
            {
                if (ts->points[i].num == 0)
                {
                    input_report_key(ts->keyevent_input, KEY_MENU, 1);
                    TCH_MSG("POS%d Report MENU key Down.", i+1);
                    ts->points[i].first_area = PRESS_KEY1_AREA;
                }
                ts->points[i].last_area = PRESS_KEY1_AREA;
            }
            else if (TS_HOMEKEYL_X < x && x < TS_HOMEKEYR_X)	//Div5-BSP-CW-TouchKey-00
            {
                if (ts->points[i].num == 0)
                {
                    input_report_key(ts->keyevent_input, KEY_HOME, 1);
                    TCH_MSG("POS%d Report HOME key Down.", i+1);
                    ts->points[i].first_area = PRESS_KEY2_AREA;
                }
                ts->points[i].last_area = PRESS_KEY2_AREA;
            }
            else if (TS_BACKKEYL_X < x && x < TS_BACKKEYR_X)	//Div5-BSP-CW-TouchKey-00
            {
                if (ts->points[i].num == 0)
                {
                    input_report_key(ts->keyevent_input, KEY_BACK, 1);
                    TCH_MSG("POS%d Report BACK key Down.", i+1);
                    ts->points[i].first_area = PRESS_KEY3_AREA;
                }
                ts->points[i].last_area = PRESS_KEY3_AREA;
            }            
            else if (TS_SEARCHKEYL_X < x && x < TS_SEARCHKEYR_X)    //Div5-BSP-CW-TouchKey-00
            {
                if (ts->points[i].num == 0)
                {
                    input_report_key(ts->keyevent_input, KEY_SEARCH, 1);
                    TCH_MSG("POS%d Report SEARCH key Down.", i+1);
                    ts->points[i].first_area = PRESS_KEY4_AREA;
                }
                ts->points[i].last_area = PRESS_KEY4_AREA;
            }
//Div5-BSP-CW-touchkey-00+[
			else
			{
				ts->points[i].num--;
			}
//Div5-BSP-CW-touchkey-00+]
        }
        
        ts->points[i].num++;
    }
    //input_sync(ts->touch_input);	//Div2-D5-Peripheral-FG-AddMultiTouch-00-
}

uint8_t read_mem(uint16_t start, uint8_t size, uint8_t *mem)
{
    struct i2c_adapter *adap = qt602240->client->adapter;
    struct i2c_msg msgs[]= {
        [0] = {
                .addr = qt602240->client->addr,
                .flags = 0,
                .len = 2,
                .buf = (uint8_t*)&start,
        },

        [1] = {
                .addr = qt602240->client->addr,
                .flags = I2C_M_RD,/*MTD-BSP-Colin-Touch03++*/
                .len = size,
                .buf = mem,
        },
    };

    return (i2c_transfer(adap, msgs, 2) < 0) ? -EIO : 0;
}

uint8_t write_mem(uint16_t start, uint8_t size, uint8_t *mem)
{
    uint8_t data[256];  // max tranfer data size = 256-2 (16 bit reg address)
    uint8_t status = 0;

    struct i2c_adapter *adap = qt602240->client->adapter;
    struct i2c_msg msgs[]= {
        [0] = {
            .addr = qt602240->client->addr,
            .flags = 0,
            .len = size+2,
            .buf = data,
        }
    };

    if (size <= 0)
        return status;

    data[0] = LSB(start);
    data[1] = MSB(start);

    memcpy(data+2, mem, size);
    TCH_DBG(TOUCH_DEBUG_I2C, "write mem, start = 0x%x, size = %d", start, size);

    status = (i2c_transfer(adap, msgs, 1) < 0) ? -EIO : 0;
    if (status != 0)
        TCH_ERR("I2C transfer failed.");

    return status;
}

// brief Reads the id part of info block.
// Reads the id part of the info block (7 bytes) from touch IC to info_block struct.
uint8_t read_id_block(info_id_t *id)
{
    uint8_t status = 0;
    uint8_t data[7];

    status = read_mem(0, 7, (void *) data);
    if (status != READ_MEM_OK)
    {
        TCH_ERR("Read id information failed, status = %d", status);
        return status;
    }
    id->family_id            = data[0];
    id->variant_id           = data[1];
    id->version              = data[2];
    id->build                = data[3];
    id->matrix_x_size        = data[4];
    id->matrix_y_size        = data[5];
    id->num_declared_objects = data[6];

    TCH_DBG(TOUCH_DEBUG_COMMON, "Done.");
    return status;
}

// brief Reads the object table of info block.
// Reads the object table part of the info block from touch IC to info_block struct.
uint8_t read_object_table(object_t *obj, unsigned int num)
{
    uint8_t status = 0, iLoop = 0, jLoop = 0, id = 0;
    uint8_t data[DEFAULT_REG_NUM][6];

    if (num > DEFAULT_REG_NUM)
    {
        TCH_ERR("Please increase default reg number, REG_NUM > %d",DEFAULT_REG_NUM);
        return READ_MEM_FAILED;
    }

    status = read_mem(7, num*6, (void *) data);
    if (status != READ_MEM_OK)
    {
        TCH_ERR("Read object table information failed, status = %d", status);
        return status;
    }

    qt602240->first_finger_id = -1;
    for (iLoop=0; iLoop<num; iLoop++)
    {
        obj->object_type = data[iLoop][0];
        obj->i2c_address = ((uint16_t) data[iLoop][2] << 8) | data[iLoop][1];
        obj->size        = data[iLoop][3];
        obj->instances   = data[iLoop][4];
        obj->num_report_ids = data[iLoop][5];

        // Save the address to global variable
        if (obj->object_type < DEFAULT_REG_NUM)
        {
            g_RegAddr[obj->object_type] = obj->i2c_address;
        }

        // Save the Report ID information
        for (jLoop=0; jLoop<obj->num_report_ids; jLoop++)
        {
           id++;
           g_ReportID[id] = obj->object_type;

           // Save the first touch ID
           if ((obj->object_type == TOUCH_MULTITOUCHSCREEN_T9) && (qt602240->first_finger_id == -1))
               qt602240->first_finger_id = id;
        }

        obj++;
    }

    TCH_DBG(TOUCH_DEBUG_COMMON, "Done.");
    return status;
}

// Return 1: Face Down.
// Return 0: Face Up.
// Return -1: Failed.
int QT_FaceDetection(void)
{
    if (qt602240 == NULL)
    {
        TCH_ERR("qt602240 is a null pointer.");
        return -1;
    }
    else
    {
        TCH_DBG(TOUCH_DEBUG_COMMON, "facedetect = %d", qt602240->facedetect);
        return qt602240->facedetect;
    }
}

int Touch_FaceDetection(void)
{
    return QT_FaceDetection();
}
EXPORT_SYMBOL(Touch_FaceDetection);

void QT_GetConfigStatus(void)
{
    uint16_t addr;
    uint8_t data = 0x01; // Nonzero value
    uint8_t msg_data[8];
    uint8_t i;

    addr = g_RegAddr[GEN_COMMANDPROCESSOR_T6] + 3;  // Cal Report all field address.
                                                    // T6 address + 3
    write_mem(addr, 1, &data);  // write one nonzero value;
    mdelay(5);

    addr = g_RegAddr[GEN_MESSAGEPROCESSOR_T5];
    while (1)
    {
        read_mem(addr, 8, msg_data);
        TCH_DBG(TOUCH_DEBUG_I2C, "Message Address = 0x%x", addr);

        for (i=0; i<8; i++)
        {
            TCH_DBG(TOUCH_DEBUG_I2C, "Message Data = 0x%x", msg_data[i]);
        }

        if ((g_ReportID[msg_data[0]] == GEN_COMMANDPROCESSOR_T6) ||
            (g_ReportID[msg_data[0]] == 0xFF) ||
            (g_ReportID[msg_data[0]] == 0x00))
            break;
    }

    TCH_DBG(TOUCH_DEBUG_COMMON, "Done.");
}

uint32_t QT_CRCSoft24(uint32_t crc, uint8_t FirstByte, uint8_t SecondByte)
{
    uint32_t crcPoly;
    uint32_t Result;
    uint16_t WData;

    crcPoly = 0x80001b;

    WData = (uint16_t) ((uint16_t)(SecondByte << 8) | FirstByte);

    Result = ((crc << 1) ^ ((uint32_t) WData));
    if (Result & 0x1000000)
    {
        Result ^= crcPoly;
    }

    return Result;
}

uint32_t QT_CheckCRC(void)
{
    uint8_t crc_data[256];
    uint8_t iLoop = 0;
    uint32_t CRC = 0;
    object_t *obj_index;

    if(!info_block || !info_block->objects || !info_block->info_id){
        return 0;
    }
    crc_data[0] = info_block->info_id->family_id;
    crc_data[1] = info_block->info_id->variant_id;
    crc_data[2] = info_block->info_id->version;
    crc_data[3] = info_block->info_id->build;
    crc_data[4] = info_block->info_id->matrix_x_size;
    crc_data[5] = info_block->info_id->matrix_y_size;
    crc_data[6] = info_block->info_id->num_declared_objects;

    obj_index = info_block->objects;
    for (iLoop=0; iLoop<info_block->info_id->num_declared_objects; iLoop++)
    {
        crc_data[iLoop*6+7+0] = obj_index->object_type;
        crc_data[iLoop*6+7+1] = obj_index->i2c_address & 0xFF;
        crc_data[iLoop*6+7+2] = obj_index->i2c_address >> 8;
        crc_data[iLoop*6+7+3] = obj_index->size;
        crc_data[iLoop*6+7+4] = obj_index->instances;
        crc_data[iLoop*6+7+5] = obj_index->num_report_ids;

        obj_index++;
    }
    crc_data[iLoop*6+7] = 0x0;

    for (iLoop=0; iLoop<(info_block->info_id->num_declared_objects*3+4); iLoop++)
    {
        CRC = QT_CRCSoft24(CRC, crc_data[iLoop*2], crc_data[iLoop*2+1]);
    }
    CRC = CRC & 0x00FFFFFF;

    TCH_DBG(TOUCH_DEBUG_COMMON, "Done. CRC = 0x%x", CRC);
    return CRC;
}

void QT_BackupNV(void)
{
    uint16_t addr   = 0;
    uint8_t data    = 0x55; 
    g_NVBackUp = 1;
    addr = g_RegAddr[GEN_COMMANDPROCESSOR_T6] + 1;  // Call the BackNV field, command processor + 1
    write_mem(addr, 1, &data);
    msleep(10);
    addr = g_RegAddr[GEN_COMMANDPROCESSOR_T6] + 0;  // Call the Reset field, command processor + 0
    write_mem(addr, 1, &data);
    msleep(100);
    TCH_DBG(TOUCH_DEBUG_COMMON, "Reset Done.");
    data = 0x00;
    addr = g_RegAddr[GEN_COMMANDPROCESSOR_T6] + 1;  
    write_mem(addr, 1, &data);
    addr = g_RegAddr[GEN_COMMANDPROCESSOR_T6] + 0;  
    write_mem(addr, 1, &data);
    g_NVBackUp = 0;
    TCH_DBG(TOUCH_DEBUG_COMMON, "Done.");
}

/* FIH-Div2-SW2-BSP, Ming { */
void QT_WaterProofSwitch(int on)
{
    uint16_t addr = 0;
    uint8_t T8_WaterProofOn[]  = { 0, 1, 0, 0 };
    uint8_t T8_WaterProofOff[] = { 5, 30, 5, 192 };
    
    TCH_DBG(TOUCH_DEBUG_COMMON, " on=%d", on);
    
    if (on) {
        addr = g_RegAddr[GEN_ACQUISITIONCONFIG_T8] + 6;  // Last 4 bits in T8
        write_mem(addr, sizeof(T8_WaterProofOn), T8_WaterProofOn);
    } else {
        addr = g_RegAddr[GEN_ACQUISITIONCONFIG_T8] + 6;  // Last 4 bits in T8
        write_mem(addr, sizeof(T8_WaterProofOff), T8_WaterProofOff);
    }
    TCH_DBG(TOUCH_DEBUG_COMMON, "Done.");
}

static void water_proof_on(struct work_struct *work)
{
    QT_WaterProofSwitch(1);
}
/* } FIH-Div2-SW2-BSP, Ming */

void QT_Calibrate(void)
{
    uint16_t addr = 0;
    uint8_t data = 0x55; // Nonzero value

    if(!Project_Flag)//FIH-SD4-BSP, Simon
    QT_WaterProofSwitch(0);  // FIH-Div2-SW2-BSP, Ming

    addr = g_RegAddr[GEN_COMMANDPROCESSOR_T6] + 2;  // Call the calibrate field, command processor + 2
    write_mem(addr, 1, &data);  // Write one nonzero value.

    TCH_DBG(TOUCH_DEBUG_COMMON, "Done.");
    
	//QT_WaterProveSwitch(1);
    // FIH-Div2-SW2-BSP, Ming
    
    if(!Project_Flag)//FIH-SD4-BSP, Simon
    schedule_delayed_work(&touch_delayed_work, msecs_to_jiffies(5000));  // wait 5 secs for QT_WaterProofSwitch(1)
}



/*----------------------------------------------------------------------------------*/
    //MTD-BSP-Colin-Touch02++*[
//#if defined(CONFIG_FIH_PROJECT_IRM)
//MTD-BSP-Colin-Touch03++[
// 2011.10.7
uint8_t v16_T7[]  = { 32, 15, 50 };
uint8_t v16_T8[]  = { 10, 0, 15, 15, 0, 0, 0, 1, 0, 0 };
uint8_t v16_T9[]  = { 131, 0, 0, 18, 11, 1, 16, 40, 3, 3, 0, 0, 0, 14, 5, 5, 5, 15, 85, 3, 223, 1, 0, 0, 35, 35, 142, 60, 142, 68, 15, 15 };
uint8_t v16_T15[] = { 131, 0, 11, 11, 1, 1, 0, 40, 3, 0, 0 };
uint8_t v16_T18[] = { 0, 0 };
uint8_t v16_T19[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t v16_T20[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t v16_T22[] = { 5, 0, 0, 0, 0, 0, 0, 0, 20, 0, 0, 0, 10, 15, 20, 255, 0 };
uint8_t v16_T23[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t v16_T24[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t v16_T25[] = { 3, 0, 0xc8, 0x32, 0x70, 0x17, 0xc8, 0x32, 0xF4, 0x1, 0, 0, 0, 0 };
uint8_t v16_T28[] = { 0, 0, 2, 8, 16, 30 };
uint8_t v16_T38[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
/*
    uint8_t v16_T7[]  = { 32, 16, 50 };
    uint8_t v16_T8[]  = { 10, 5, 10, 10, 0, 0, 10, 30 };
    uint8_t v16_T9[]  = { 143, 0, 0, 18, 11, 1, 17, 40, 2, 3, 0, 5, 5, 32, 4, 10, 20, 3, 0x0, 0x0, 0x0, 0x0, 0, 0, 0, 0, 5, 180, 5, 180, 10 };
    uint8_t v16_T15[] = { 131, 0, 11, 11, 1, 1, 1, 35, 2, 0, 0 };
    uint8_t v16_T18[] = { 0, 0, };
    uint8_t v16_T19[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t v16_T20[] = { 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t v16_T22[] = { 5, 0, 0, 0x0, 0x0, 0x0, 0x0, 0, 15, 0, 3, 7, 10, 15, 20, 30, 0 };
    uint8_t v16_T23[] = { 0, 0, 0, 0, 0, 0, 0, 0x0, 0x0, 0, 0, 0x0, 0x0 };
    uint8_t v16_T24[] = { 0, 0, 0x0, 0x0, 0, 100, 100, 1, 10, 20, 40, 0x4B, 0, 0x02, 0, 0x64, 0, 0x19, 0 };
    uint8_t v16_T25[] = { 3, 0, 0xC8, 0x32, 0x70, 0x17, 0xC8, 0x32, 0xF4, 0x1, 0x0, 0x0, 0x0, 0x0 };
    uint8_t v16_T27[] = { 0, 2, 0, 0, 0, 0x0, 0x0 };
    uint8_t v16_T28[] = { 0, 0, 2, 8, 8, 30 };
*/
//MTD-BSP-Colin-Touch03++]
//#else
//    uint8_t v16_T7[]  = { 32, 10, 50 };
//    uint8_t v16_T8[]  = { 6, 0, 20, 20, 0, 0, 10, 15 };
//    uint8_t v16_T9[]  = { 143, 0, 0, 19, 10, 0, 32, 60, 3, 5, 0, 5, 5, 47, 2, 10, 25, 0, 0x0, 0x0, 0x0, 0x0, 0, 0, 0, 0, 0, 0, 0, 0, 30 };
//    uint8_t v16_T15[] = { 131, 15, 10, 4, 1, 0, 32, 60, 3, 0, 0 };
//    uint8_t v16_T18[] = { 0, 0, };
//    uint8_t v16_T19[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
//    uint8_t v16_T20[] = { 0, 100, 100, 100, 100, 0, 0, 30, 20, 4, 15, 5 };
//    uint8_t v16_T22[] = { 13, 0, 0, 0x1E, 0x0, 0xE2, 0xFF, 8, 50, 0, 0, 0, 10, 20, 40, 255, 8 };
//    uint8_t v16_T23[] = { 0, 0, 0, 0, 0, 0, 0, 0x0, 0x0, 0, 0, 0x0, 0x0 };
//    uint8_t v16_T24[] = { 0, 2, 0xFF, 0x03, 0, 100, 100, 1, 10, 20, 40, 0x4B, 0, 0x02, 0, 0x64, 0, 0x19, 0 };
//    uint8_t v16_T25[] = { 0, 0, 0xF8, 0x2A, 0x70, 0x17, 0x28, 0x23, 0x88, 0x13, 0x0, 0x0, 0x0, 0x0 };
//    uint8_t v16_T27[] = { 0, 1, 0, 224, 3, 0x23, 0x0 };
//    uint8_t v16_T28[] = { 0, 0, 3, 40, 16, 20 };
//#endif
    //MTD-BSP-Colin-Touch02++*]

static uint8_t v15_T7[]  = { 32, 16, 50};
static uint8_t v15_T8[]  = { 8, 5, 20, 20, 0, 0, 10, 15};
static uint8_t v15_T9[]  = { 143, 0, 0, 18, 10, 0, 33, 60, 2, 5, 0, 5, 5, 0, 2, 10, 10, 10, 0x0, 0x0, 0x0, 0x0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t v15_T15[] = { 0, 0, 10, 4, 1, 0, 33, 30, 2, 0, 0};
static uint8_t v15_T18[] = { 0, 0,};
static uint8_t v15_T19[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t v15_T20[] = { 7, 100, 100, 100, 100, 0, 0, 20, 15, 3, 12, 5};
static uint8_t v15_T22[] = { 5, 0, 0, 0x19, 0, 0xE7, 0xFF, 4, 50, 0, 1, 10, 15, 20, 25, 30, 4};
static uint8_t v15_T23[] = { 0, 0, 0, 0, 0, 0, 0, 0x0, 0x0, 0, 0, 0x0, 0x0};
static uint8_t v15_T24[] = { 0, 1, 0, 0, 0, 100, 100, 1, 10, 20, 40, 0x4B, 0, 0x02, 0, 0x64, 0, 0x19, 0};
static uint8_t v15_T25[] = { 3, 0, 0xE0, 0x2E, 0x58, 0x1B, 0xB0, 0x36, 0xF4, 0x01, 0x0, 0x0, 0x0, 0x0};
static uint8_t v15_T27[] = { 0, 2, 0, 0, 3, 0x23, 0x0};
static uint8_t v15_T28[] = { 0, 0, 2, 4, 8, 30};

static uint8_t v14_T7[]  = { 32, 16, 50};
static uint8_t v14_T8[]  = { 8, 5, 20, 20, 0, 0, 10, 15};
static uint8_t v14_T9[]  = { 131, 0, 0, 18, 10, 0, 65, 80, 2, 5, 0, 1, 1, 0, 2, 10, 10, 10, 0x0, 0x0, 0x0, 0x0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t v14_T15[] = { 0, 0, 0, 1, 1, 0, 65, 30, 2, 0, 0};
static uint8_t v14_T19[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t v14_T20[] = { 7, 100, 100, 100, 100, 0, 0, 15, 10, 3, 10, 5};
static uint8_t v14_T22[] = { 5, 0, 0, 0x19, 0, 0xE7, 0xFF, 4, 50, 0, 1, 10, 15, 20, 25, 30, 4};
static uint8_t v14_T23[] = { 0, 0, 0, 0, 0, 0, 0, 0x0, 0x0, 0, 0, 0x0, 0x0};
static uint8_t v14_T24[] = { 3, 10, 0xFF, 0x03, 0, 100, 100, 1, 10, 20, 40, 0x4B, 0, 0x02, 0, 0x64, 0, 0x19, 0};
static uint8_t v14_T25[] = { 0, 0, 0xE0, 0x2E, 0x58, 0x1B, 0xB0, 0x36, 0xF4, 0x01, 0x0, 0x0, 0x0, 0x0};
static uint8_t v14_T27[] = { 3, 2, 0, 224, 3, 0x23, 0x0};
static uint8_t v14_T28[] = { 0, 0, 2, 4, 8};

static uint8_t v12_T6[]  = { 0, 0, 0, 0, 0};
static uint8_t v12_T7[]  = { 32, 16, 50};
static uint8_t v12_T8[]  = { 8, 5, 20, 20, 0, 0};
static uint8_t v12_T9[]  = { 131, 0, 0, 14, 10, 0, 65, 80, 2, 5, 0, 1, 1, 0, 2, 10, 10, 10, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t v12_T15[] = { 0, 0, 0, 1, 1, 0, 65, 30, 2, 0, 0};
static uint8_t v12_T19[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t v12_T20[] = { 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0};
static uint8_t v12_T22[] = { 5, 0, 0, 0x19, 0, 0xE7, 0xFF, 4, 50, 0, 1, 0, 10, 20};
static uint8_t v12_T24[] = { 3, 10, 255, 1, 0, 100, 40, 1, 0, 0, 0, 200, 0, 5, 0, 50, 0, 100, 0};
static uint8_t v12_T25[] = { 0, 0, 176, 54, 244, 1, 176, 54, 244, 1, 3, 2};
static uint8_t v12_T26[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 255, 32, 101};
static uint8_t v12_T27[] = { 3, 2, 0, 224, 2, 20, 0, 4 };
static uint8_t v12_T28[] = { 9, 0, 2, 4, 8, 110, 32, 101 };


//NJ-BSP-Simon-Touch-TBP parameters
// 2011.10.28
//static uint8_t v10_T7[]  = { 32, 16, 50};
static uint8_t v10_T7[]  = { 20, 10, 20};
static uint8_t v10_T8[]  = { 8, 0, 5, 5, 0, 0, 255, 1};
#ifdef WATER
static uint8_t v10_T8_1[]  = { 8, 0, 5, 5, 10, 0, 0, 0};
#endif
static uint8_t v10_T9[]  = { 143, 0, 0, 16, 9, 0, 17, 45, 3, 1, 0, 1, 1, 14, 5, 10, 10, 10, 0x93, 0x3, 0xDF, 0x1, 0, 0, 0, 0, 
0, 0, 0, 0, 0};
static uint8_t v10_T15[] = { 1, 0, 9, 1, 5, 0, 17, 30, 2, 0, 0};
static uint8_t v10_T18[] = { 0, 0,};
static uint8_t v10_T19[] = { 3, 0, 0, 0xFF, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static uint8_t v10_T20[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t v10_T20_1[] = { 7, 0, 0, 0, 0, 0, 0, 20, 10, 2, 10, 1};

static uint8_t v10_T22[] = { 5, 0, 0, 0, 0, 0, 0, 0, 25, 0, 0, 6, 11, 16, 19, 21, 0};
static uint8_t v10_T23[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t v10_T24[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t v10_T25[] = { 3, 0, 0xEC, 0x2C, 0x70, 0x17, 0xE0, 0x2E, 0, 0, 0, 0, 0, 0};
static uint8_t v10_T27[] = { 0, 0, 0, 0, 0, 0, 0};
static uint8_t v10_T28[] = { 0, 0, 0, 32/*32*/, 32/*32*/, 10};
//static uint8_t v10_T38[] = { 0, 0, 0, 0, 0, 0, 0, 0 };



//NJ-BSP-Simon-Touch-TNQ parameters
// 2011.10.28
static uint8_t v10_var13_T7[]  = { 32, 16, 50};
static uint8_t v10_var13_T8[]  = { 18,0, 5, 5, 0, 0, 255,1, 0, 0};
#ifdef WATER
static uint8_t v10_var13_T8_1[]  = { 18,0, 5, 5, 10, 0, 0,0, 0, 0};
#endif
static uint8_t v10_var13_T9[]  = { 143, 0, 0, 11, 8, 0, 32, 80, 2, 2, 0, 5, 5, 0, 2, 10, 10, 10, 0xE0, 0x1, 0x40, 0x1, 0, 0, 0, 0, 0, 0, 0, 0,50, 15, 0, 0, 1};//2011-8-11
static uint8_t v10_var13_T18[] = { 0, 0};
static uint8_t v10_var13_T25[] = { 3, 0, 0x30, 0x75, 0x20, 0x4E, 0, 0, 0, 0, 0x0, 0x0, 0x0, 0x0};
//static uint8_t v10_var13_T38[] = { 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t v10_var13_T40[] = { 0, 0, 0, 0, 0};
static uint8_t v10_var13_T42[] = { 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t v10_var13_T42_1[] = { 3, 20, 20, 40, 128,0,0,0 };

static uint8_t v10_var13_T46[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t v10_var13_T47[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t v10_var13_T48[] = { 0, 140, 80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 6, 0, 0, 100, 4, 64, 10,  0, 10, 20, 0,  5,  0, 38, 0, 0, 0, 0, 0, 0, 48, 50, 2, 5, 5, 0, 2, 10, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//NJ-BSP-Simon-Touch-ITV parameters
// 2011.10.28
static uint8_t v10_var01_T7[]  = { 32, 16, 50 };
//static uint8_t v10_var01_T8[]  = { 30, 0, 10, 10, 0, 0, 0, 50,1,130 };
static uint8_t v10_var01_T8[]  = { 30, 0, 5, 5, 0, 0, 255, 1,0,0 };
#ifdef WATER
static uint8_t v10_var01_T8_1[]  = { 30, 0, 5,5 , 10, 0, 0, 0,0,0};
#endif
static uint8_t v10_var01_T9[]  = { 143, 0, 0, 18, 11, 0, 16, 50, 3, 3, 0, 10, 2, 0, 5, 0, 0, 10,0x4C, 3, 223,1,0,0,0,0,0,0,0,0,0,10,0,0,0}; 
static uint8_t v10_var01_T15[] = { 3, 0, 11, 18, 1, 0, 16, 55, 3, 0, 0 };
static uint8_t v10_var01_T18[] = { 0, 0, };
static uint8_t v10_var01_T19[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
//static uint8_t v10_var01_T20[] = { 0, 100, 100, 100, 100, 0, 0, 0, 0, 0, 0, 0 };
//static uint8_t v10_var01_T22[] = { 5, 0, 0, 0, 0, 0, 17, 0, 3, 7, 19, 30, 255, 255, 0 };
static uint8_t v10_var01_T23[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
//static uint8_t v10_var01_T24[] = { 0, 0, 0, 0, 100, 100, 1, 10, 20, 40, 75, 2, 100, 25 };
static uint8_t v10_var01_T25[] = { 3, 0, 0x30, 0x75, 0x20, 0x4E, 0x30, 0x75, 0x20, 0x4E, 0x0, 0x0, 0x0, 0x0};
//static uint8_t v10_var01_T27[] = { 0, 2, 0, 0, 0, 0 };
//static uint8_t v10_var01_T28[] = { 0, 0, 2, 4, 8, 30 };
//static uint8_t v10_var01_T38[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
static uint8_t v10_var01_T40[] = { 0, 0, 0, 0, 0 };
static uint8_t v10_var01_T42[] = { 0, 20, 20, 40, 128,0,0,0 };
static uint8_t v10_var01_T42_1[] = { 3, 20, 20, 40, 128,0,0,0 };


static uint8_t v10_var01_T46[] = { 0, 2, 0, 0, 0,0,0,0,0 };
static uint8_t v10_var01_T47[] = { 0, 0, 0, 0, 0,0,0,0,0,0 };
static uint8_t v10_var01_T48[] = { 1, 4, 0, 0, 0,0,0,0,0,0,0,0,0,6,6,0,0,100,4,64,10,0,20,5,0,38,0,8,0,0,0,0,0,0,16,40,3,10,2,0,5,20,25,
                                          0,0,0,0,0,0,0,0,0,0,0 };

//NJ-BSP-Simon-Touch-IP parameters
// 2011.11.30
static uint8_t v10_var01_T7_V2[]  = { 32, 16, 50 };
static uint8_t v10_var01_T8_V2[]  = { 30, 0, 5, 5, 0, 0, 255, 1,0,0 };
#ifdef WATER
//static uint8_t v10_var01_T8_1_V2[]  = { 30, 0, 5,5 , 0, 0, 0, 0,0,0};
#endif
static uint8_t v10_var01_T9_V2[]  = { 143, 0, 0, 19, 11, 0, 16, 40, 3, 7, 0, 10, 2, 0, 5, 0, 0, 10, 31, 3, 223,1,0,0,0,0,0,0,0,0,0,0,0,0,0}; 
static uint8_t v10_var01_T15_V2[] = { 0, 0, 11, 18, 1, 0, 0, 45, 3, 0, 0 };
static uint8_t v10_var01_T18_V2[] = { 0, 0, };
static uint8_t v10_var01_T19_V2[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static uint8_t v10_var01_T23_V2[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static uint8_t v10_var01_T25_V2[] = { 3, 0, 0x30, 0x75, 0x20, 0x4E, 0x30, 0x75, 0x20, 0x4E, 0x0, 0x0, 0x0, 0x0};
static uint8_t v10_var01_T40_V2[] = { 0, 0, 0, 0, 0 };
static uint8_t v10_var01_T42_V2[] = { 0, 20, 20, 40, 128,0,0,0 };
static uint8_t v10_var01_T46_V2[] = { 0, 3, 0, 0, 0,0,0,0,0 };
static uint8_t v10_var01_T47_V2[] = { 0, 0, 0, 0, 0,0,0,0,0,0 };
static uint8_t v10_var01_T48_V2[] = { 1, 132, 96, 0, 0,0,0,0,0,0,0,0,0,6,6,0,0,100,4,64,10,0,20,5,0,38,0,8,0,0,0,0,0,0,16,40,3,10,2,0,5,20,25,
	                                 0,0,0,0,0,0,0,0,0,0,0 };



void QT_InitConfig(void)
{
    uint8_t iLoop = 0, MaxOBJSize = 0;
    uint16_t addr;
    object_t *obj_index = NULL;
	
    if(!info_block || !info_block->objects || !info_block->info_id){
        return;
    }
    MaxOBJSize = info_block->info_id->num_declared_objects;
    obj_index = info_block->objects;
    cap_touch_fw_version = info_block->info_id->version;
    cap_touch_fw_variant_version = kzalloc(sizeof(char)*10,GFP_KERNEL);
    if(!cap_touch_fw_variant_version)
        {
            printk("kzalloc_err\n");
        }else
            sprintf(cap_touch_fw_variant_version,"%d_%d",cap_touch_fw_version,info_block->info_id->variant_id);

    if (info_block->info_id->version == 0x10)
    {
        TCH_MSG("Use 1.0 firmware config.");
        if (info_block->info_id->variant_id == 0x01)
        {
            if(Project_Flag == 3)
            {
            for (iLoop=0; iLoop<MaxOBJSize; iLoop++)
            {
                addr = obj_index->i2c_address;
                switch(obj_index->object_type)
                {
                case GEN_POWERCONFIG_T7 :
                    write_mem(addr, sizeof(v10_var01_T7), v10_var01_T7);
                    break;
                case GEN_ACQUISITIONCONFIG_T8 :
                    //write_mem(addr, sizeof(v10_var01_T8), v10_var01_T8);
                    write_mem(addr, sizeof(v10_var01_T8_1), v10_var01_T8_1);
     
                    break;
                case TOUCH_MULTITOUCHSCREEN_T9 :
                    write_mem(addr, sizeof(v10_var01_T9), v10_var01_T9);
                    break;
                case TOUCH_KEYARRAY_T15 :
                    write_mem(addr, sizeof(v10_var01_T15), v10_var01_T15);
                    break;
                case SPT_COMCONFIG_T18 :
                    write_mem(addr, sizeof(v10_var01_T18), v10_var01_T18);
                    break;
                case SPT_GPIOPWM_T19 :
                    write_mem(addr, sizeof(v10_var01_T19), v10_var01_T19);
                    break;                
                case TOUCH_PROXIMITY_T23 :
                    write_mem(addr, sizeof(v10_var01_T23), v10_var01_T23);
                    break;
                case SPT_SELFTEST_T25 :
                    write_mem(addr, sizeof(v10_var01_T25), v10_var01_T25);
                    break;
                case SPARE_T40 :
                    write_mem(addr, sizeof(v10_var01_T40), v10_var01_T40);
                    break;
                case SPARE_T42 :
                    write_mem(addr, sizeof(v10_var01_T42), v10_var01_T42);
                    break;
                case SPARE_T46 :
                    write_mem(addr, sizeof(v10_var01_T46), v10_var01_T46);
                    break;
                case SPARE_T47 :
                    write_mem(addr, sizeof(v10_var01_T47), v10_var01_T47);
                    break;
                case SPARE_T48 :
                    write_mem(addr, sizeof(v10_var01_T48), v10_var01_T48);
                    break;
                default:
                    break;
                }
                obj_index++;
            }
        }
            else if(Project_Flag == 4)
            {
                for (iLoop=0; iLoop<MaxOBJSize; iLoop++)
                {
                    addr = obj_index->i2c_address;
                    switch(obj_index->object_type)
                    {
                    case GEN_POWERCONFIG_T7 :
                        write_mem(addr, sizeof(v10_var01_T7_V2), v10_var01_T7_V2);
                        break;
                    case GEN_ACQUISITIONCONFIG_T8 :
                        write_mem(addr, sizeof(v10_var01_T8_V2), v10_var01_T8_V2);
                        break;
                    case TOUCH_MULTITOUCHSCREEN_T9 :
                        write_mem(addr, sizeof(v10_var01_T9_V2), v10_var01_T9_V2);
                        break;
                    case TOUCH_KEYARRAY_T15 :
                        write_mem(addr, sizeof(v10_var01_T15_V2), v10_var01_T15_V2);
                        break;
                    case SPT_COMCONFIG_T18 :
                        write_mem(addr, sizeof(v10_var01_T18_V2), v10_var01_T18_V2);
                        break;
                    case SPT_GPIOPWM_T19 :
                        write_mem(addr, sizeof(v10_var01_T19_V2), v10_var01_T19_V2);
                        break;                
                    case TOUCH_PROXIMITY_T23 :
                        write_mem(addr, sizeof(v10_var01_T23_V2), v10_var01_T23_V2);
                        break;
                    case SPT_SELFTEST_T25 :
                        write_mem(addr, sizeof(v10_var01_T25_V2), v10_var01_T25_V2);
                        break;
                    case SPARE_T40 :
                        write_mem(addr, sizeof(v10_var01_T40_V2), v10_var01_T40_V2);
                        break;
                    case SPARE_T42 :
                        write_mem(addr, sizeof(v10_var01_T42_V2), v10_var01_T42_V2);
                        break;
                    case SPARE_T46 :
                        write_mem(addr, sizeof(v10_var01_T46_V2), v10_var01_T46_V2);
                        break;
                    case SPARE_T47 :
                        write_mem(addr, sizeof(v10_var01_T47_V2), v10_var01_T47_V2);
                        break;
                    case SPARE_T48 :
                        write_mem(addr, sizeof(v10_var01_T48_V2), v10_var01_T48_V2);
                        break;
                    default:
                        break;
                    }
                    obj_index++;
                }
            }    
        }
        else if(info_block->info_id->variant_id == 0x03)
        {
                for (iLoop=0; iLoop<MaxOBJSize; iLoop++)
                {
                    TCH_MSG("Config: object_type = %d", obj_index->object_type);

                    addr = obj_index->i2c_address;
                    switch (obj_index->object_type)
                    {
                    case GEN_POWERCONFIG_T7 :
                        write_mem(addr, sizeof(v10_T7), v10_T7);
                        break;
                    case GEN_ACQUISITIONCONFIG_T8 :
                        write_mem(addr, sizeof(v10_T8_1), v10_T8_1);
                        break;
                    case TOUCH_MULTITOUCHSCREEN_T9 :
                        write_mem(addr, sizeof(v10_T9), v10_T9);
                        break;
                    case TOUCH_KEYARRAY_T15 :
                        write_mem(addr, sizeof(v10_T15), v10_T15);
                        break;
                    case SPT_COMCONFIG_T18 :
                        write_mem(addr, sizeof(v10_T18), v10_T18);
                        break;
                    case SPT_GPIOPWM_T19 :
                        write_mem(addr, sizeof(v10_T19), v10_T19);
                        break;
                    case PROCI_GRIPFACESUPPRESSION_T20 :
                        write_mem(addr, sizeof(v10_T20), v10_T20);
                        break;
                    case PROCG_NOISESUPPRESSION_T22 :
                        write_mem(addr, sizeof(v10_T22), v10_T22);
                        break;
                    case TOUCH_PROXIMITY_T23 :
                        write_mem(addr, sizeof(v10_T23), v10_T23);
                        break;
                    case PROCI_ONETOUCHGESTUREPROCESSOR_T24 :
                        write_mem(addr, sizeof(v10_T24), v10_T24);
                        break;
                    case SPT_SELFTEST_T25 :
                        write_mem(addr, sizeof(v10_T25), v10_T25);
                        break;
                    case PROCI_TWOTOUCHGESTUREPROCESSOR_T27 :
                        write_mem(addr, sizeof(v10_T27), v10_T27);
                        break;
                    case SPT_CTECONFIG_T28 :
                        write_mem(addr, sizeof(v10_T28), v10_T28);
                        break;
                    case SPARE_T38 :
                        //write_mem(addr, sizeof(v10_T38), v10_T38);
                        break;
                    default:
                        TCH_MSG("Config: object_type %d not config data", obj_index->object_type);
                        break;
                    }
                    obj_index++;
                }
        }
        else if(info_block->info_id->variant_id == 0x13)
        {
            for (iLoop=0; iLoop<MaxOBJSize; iLoop++)
            {
                TCH_MSG("Config: object_type = %d", obj_index->object_type);
                addr = obj_index->i2c_address;
                switch (obj_index->object_type)
                {
                case GEN_POWERCONFIG_T7 :
                    write_mem(addr, sizeof(v10_var13_T7), v10_var13_T7);
                    break;
                case GEN_ACQUISITIONCONFIG_T8 :
                    write_mem(addr, sizeof(v10_var13_T8_1), v10_var13_T8_1);
                    break;
                case TOUCH_MULTITOUCHSCREEN_T9 :
                    write_mem(addr, sizeof(v10_var13_T9), v10_var13_T9);
                    break;
                case SPT_COMCONFIG_T18 :
                    write_mem(addr, sizeof(v10_var13_T18), v10_var13_T18);
                    break;
                case SPT_SELFTEST_T25 :
                    write_mem(addr, sizeof(v10_var13_T25), v10_var13_T25);
                    break;
                case SPARE_T38:
//                    write_mem(addr, sizeof(v10_var13_T38), v10_var13_T38);
                    break;
                case SPARE_T40:
                    write_mem(addr, sizeof(v10_var13_T40), v10_var13_T40);
                    break;
                case SPARE_T42:
                    write_mem(addr, sizeof(v10_var13_T42), v10_var13_T42);
                    break;
                case SPARE_T46:
                    write_mem(addr, sizeof(v10_var13_T46), v10_var13_T46);
                    break;
                case SPARE_T47:
                    write_mem(addr, sizeof(v10_var13_T47), v10_var13_T47);
                    break;
                case SPARE_T48:
                    write_mem(addr, sizeof(v10_var13_T48), v10_var13_T48);
                    break;
               default:
                    TCH_MSG("Config: object_type %d not config data", obj_index->object_type);
                    break;
                }
                obj_index++;
            }
        }
    }
    else if (info_block->info_id->version == 0x12)
    {
        TCH_MSG("Use 1.2 firmware config.");
        for (iLoop=0; iLoop<MaxOBJSize; iLoop++)
        {
            addr = obj_index->i2c_address;
            switch(obj_index->object_type)
            {
            case GEN_COMMANDPROCESSOR_T6 :
                write_mem(addr, sizeof(v12_T6), v12_T6);
                break;
            case GEN_POWERCONFIG_T7 :
                write_mem(addr, sizeof(v12_T7), v12_T7);
                break;
            case GEN_ACQUISITIONCONFIG_T8 :
                write_mem(addr, sizeof(v12_T8), v12_T8);
                break;
            case TOUCH_MULTITOUCHSCREEN_T9 :
                write_mem(addr, sizeof(v12_T9), v12_T9);
                break;
            case TOUCH_KEYARRAY_T15 :
                write_mem(addr, sizeof(v12_T15), v12_T15);
                break;
            case SPT_GPIOPWM_T19 :
                write_mem(addr, sizeof(v12_T19), v12_T19);
                break;
            case PROCI_GRIPFACESUPPRESSION_T20 :
                write_mem(addr, sizeof(v12_T20), v12_T20);
                break;
            case PROCG_NOISESUPPRESSION_T22 :
                write_mem(addr, sizeof(v12_T22), v12_T22);
                break;
            case PROCI_ONETOUCHGESTUREPROCESSOR_T24 :
                write_mem(addr, sizeof(v12_T24), v12_T24);
                break;
            case SPT_SELFTEST_T25 :
                write_mem(addr, sizeof(v12_T25), v12_T25);
                break;
            case DEBUG_CTERANGE_T26 :
                write_mem(addr, sizeof(v12_T26), v12_T26);
                break;
            case PROCI_TWOTOUCHGESTUREPROCESSOR_T27 :
                write_mem(addr, sizeof(v12_T27), v12_T27);
                break;
            case SPT_CTECONFIG_T28 :
                write_mem(addr, sizeof(v12_T28), v12_T28);
                break;
            default:
                break;
            }
            obj_index++;
        }
    }
    else if (info_block->info_id->version == 0x14)
    {
        TCH_MSG("Use 1.4 firmware config.");
        for (iLoop=0; iLoop<MaxOBJSize; iLoop++)
        {
            addr = obj_index->i2c_address;
            switch(obj_index->object_type)
            {
            case GEN_POWERCONFIG_T7 :
                write_mem(addr, sizeof(v14_T7), v14_T7);
                break;
            case GEN_ACQUISITIONCONFIG_T8 :
                write_mem(addr, sizeof(v14_T8), v14_T8);
                break;
            case TOUCH_MULTITOUCHSCREEN_T9 :
                write_mem(addr, sizeof(v14_T9), v14_T9);
                break;
            case TOUCH_KEYARRAY_T15 :
                write_mem(addr, sizeof(v14_T15), v14_T15);
                break;
            case SPT_GPIOPWM_T19 :
                write_mem(addr, sizeof(v14_T19), v14_T19);
                break;
            case PROCI_GRIPFACESUPPRESSION_T20 :
                write_mem(addr, sizeof(v14_T20), v14_T20);
                break;
            case PROCG_NOISESUPPRESSION_T22 :
                write_mem(addr, sizeof(v14_T22), v14_T22);
                break;
            case TOUCH_PROXIMITY_T23 :
                write_mem(addr, sizeof(v14_T23), v14_T23);
                break;
            case PROCI_ONETOUCHGESTUREPROCESSOR_T24 :
                write_mem(addr, sizeof(v14_T24), v14_T24);
                break;
            case SPT_SELFTEST_T25 :
                write_mem(addr, sizeof(v14_T25), v14_T25);
                break;
            case PROCI_TWOTOUCHGESTUREPROCESSOR_T27 :
                write_mem(addr, sizeof(v14_T27), v14_T27);
                break;
            case SPT_CTECONFIG_T28 :
                write_mem(addr, sizeof(v14_T28), v14_T28);
                break;
            default:
                break;
            }
            obj_index++;
        }
    }
    else if (info_block->info_id->version == 0x15)
    {
        TCH_MSG("Use 1.5 firmware config.");
        for (iLoop=0; iLoop<MaxOBJSize; iLoop++)
        {
            addr = obj_index->i2c_address;
            switch(obj_index->object_type)
            {
            case GEN_POWERCONFIG_T7 :
                write_mem(addr, sizeof(v15_T7), v15_T7);
                break;
            case GEN_ACQUISITIONCONFIG_T8 :
                write_mem(addr, sizeof(v15_T8), v15_T8);
                break;
            case TOUCH_MULTITOUCHSCREEN_T9 :
                write_mem(addr, sizeof(v15_T9), v15_T9);
                break;
            case TOUCH_KEYARRAY_T15 :
                write_mem(addr, sizeof(v15_T15), v15_T15);
                break;
            case SPT_COMCONFIG_T18 :
                write_mem(addr, sizeof(v15_T18), v15_T18);
                break;
            case SPT_GPIOPWM_T19 :
                write_mem(addr, sizeof(v15_T19), v15_T19);
                break;
            case PROCI_GRIPFACESUPPRESSION_T20 :
                write_mem(addr, sizeof(v15_T20), v15_T20);
                break;
            case PROCG_NOISESUPPRESSION_T22 :
                write_mem(addr, sizeof(v15_T22), v15_T22);
                break;
            case TOUCH_PROXIMITY_T23 :
                write_mem(addr, sizeof(v15_T23), v15_T23);
                break;
            case PROCI_ONETOUCHGESTUREPROCESSOR_T24 :
                write_mem(addr, sizeof(v15_T24), v15_T24);
                break;
            case SPT_SELFTEST_T25 :
                write_mem(addr, sizeof(v15_T25), v15_T25);
                break;
            case PROCI_TWOTOUCHGESTUREPROCESSOR_T27 :
                write_mem(addr, sizeof(v15_T27), v15_T27);
                break;
            case SPT_CTECONFIG_T28 :
                write_mem(addr, sizeof(v15_T28), v15_T28);
                break;
            default:
                break;
            }
            obj_index++;
        }
    }
    else
    {
        TCH_MSG("Use 1.6 firmware config.");
        for (iLoop=0; iLoop<MaxOBJSize; iLoop++)
        {
            addr = obj_index->i2c_address;
            switch(obj_index->object_type)
            {
            case GEN_POWERCONFIG_T7 :
                write_mem(addr, sizeof(v16_T7), v16_T7);
                break;
            case GEN_ACQUISITIONCONFIG_T8 :
                write_mem(addr, sizeof(v16_T8), v16_T8);
                break;
            case TOUCH_MULTITOUCHSCREEN_T9 :
                write_mem(addr, sizeof(v16_T9), v16_T9);
                break;
            case TOUCH_KEYARRAY_T15 :
                write_mem(addr, sizeof(v16_T15), v16_T15);
                break;
            case SPT_COMCONFIG_T18 :
                write_mem(addr, sizeof(v16_T18), v16_T18);
                break;
            case SPT_GPIOPWM_T19 :
                write_mem(addr, sizeof(v16_T19), v16_T19);
                break;
            case PROCI_GRIPFACESUPPRESSION_T20 :
                write_mem(addr, sizeof(v16_T20), v16_T20);
                break;
            case PROCG_NOISESUPPRESSION_T22 :
                write_mem(addr, sizeof(v16_T22), v16_T22);
                break;
            case TOUCH_PROXIMITY_T23 :
                write_mem(addr, sizeof(v16_T23), v16_T23);
                break;
            case PROCI_ONETOUCHGESTUREPROCESSOR_T24 :
                write_mem(addr, sizeof(v16_T24), v16_T24);
                break;
            case SPT_SELFTEST_T25 :
                write_mem(addr, sizeof(v16_T25), v16_T25);
                break;
//            case PROCI_TWOTOUCHGESTUREPROCESSOR_T27 :
//                write_mem(addr, sizeof(v16_T27), v16_T27);
//                break;
            case SPT_CTECONFIG_T28 :
                write_mem(addr, sizeof(v16_T28), v16_T28);
                break;
			case SPARE_T38 :
				write_mem(addr, sizeof(v16_T38), v16_T38);
				break;
            default:
                break;
            }
            obj_index++;
        }
    }

    TCH_DBG(TOUCH_DEBUG_COMMON, "Done.");
}

//
void QT_SelfTest_V2(void)
{
	uint16_t addr;
	uint8_t data = 0x11;
	uint8_t T37_buf[130];
	uint8_t page = 0;
	uint8_t idx = 0;
	uint16_t val = 0;
	st_result = 0;
	
	if (qt_debug_level >= DB_LEVEL1)
		printk(KERN_INFO "[Touch] low_boundary=%d, high_boundary=%d\n", low_boundary, high_boundary);
		
	addr = g_RegAddr[GEN_COMMANDPROCESSOR_T6] + 5;
	write_mem(addr, 1, &data);
	
	msleep(50);
	
	for (page=0; page<4; page++)
	{
		memset(T37_buf, 0, sizeof(T37_buf));
		
		read_mem(g_RegAddr[SPARE_T37], sizeof(T37_buf), T37_buf);
		
		if (qt_debug_level >= DB_LEVEL1)
		{
			printk(KERN_INFO "[Touch] mode = 0x%02x, page = %d\n",
						T37_buf[0], T37_buf[1]);
		}
		
		for (idx=2; idx+1<sizeof(T37_buf); idx+=2)
		{
			val = T37_buf[idx+1];
			val = T37_buf[idx] + (val << 8);
			
			if (qt_debug_level >= DB_LEVEL1)
			{
				printk(KERN_INFO "[Touch] CH %d (X%d,Y%d) = %d",
						page*64+(idx-2)/2, (page*64+(idx-2)/2)/12,
						(page*64+(idx-2)/2)%12, val);
			}
			
			if (val < low_boundary || val > high_boundary)
			{
				if ((page*64+(idx-2)/2)<216 &&
					((((page*64+(idx-2)/2)%12) != 11) ||
					(((page*64+(idx-2)/2)/12) == 0) ||
					(((page*64+(idx-2)/2)/12) == 1) ||
					(((page*64+(idx-2)/2)/12) == 9) ||
					(((page*64+(idx-2)/2)/12) == 10)))
				{
					printk(KERN_INFO "[Touch] CH %d (X%d,Y%d) = %d, out of specification",
							page*64+(idx-2)/2, (page*64+(idx-2)/2)/12,
							(page*64+(idx-2)/2)%12, val);
					
					if ((page*64+(idx-2)/2) == 11)
					{
						printk(KERN_INFO "[Touch] Virtual key 'Menu' test FAIL!\n");
						st_result |= 2;
					}
					else if ((page*64+(idx-2)/2) == 23)
					{
						printk(KERN_INFO "[Touch] Virtual key 'Home' test FAIL!\n");
						st_result |= 4;
					}
					else if ((page*64+(idx-2)/2) == 119)
					{
						printk(KERN_INFO "[Touch] Virtual key 'Back' test FAIL!\n");
						st_result |= 8;
					}
					else if ((page*64+(idx-2)/2) == 131)
					{
						printk(KERN_INFO "[Touch] Virtual key 'Search' test FAIL!\n");
						st_result |= 16;
					}
					else
					{
						printk(KERN_INFO "[Touch] Touch area FAIL!\n");
						st_result |= 1;
					}
				}
			}
		}
		
		if (page < 3)
		{
			addr = g_RegAddr[GEN_COMMANDPROCESSOR_T6] + 5;
			data = 0x1;		// page up
			write_mem(addr, 1, &data);
		}
		
		msleep(50);
	}
	
	printk(KERN_INFO "[Touch] Self test function complete, result = %d\n", st_result);
}

static int my_atoi(const char *num)
{
	int minus = 0;
	unsigned char *ptr = (unsigned char *)num;
	int sum = 0;
		
	if (num[0] == 0)
    	return 0;
    
	if (*ptr == '-')
	{
		minus = 1;
		ptr++;
	}
	
	for (; *ptr != 0; ptr++)
	{
		if (*ptr < '0' || *ptr > '9')
			return 0;
		
		sum *= 10;
		sum += (*ptr - '0');
		
		if (sum < 0)
			return 0;
	}
	
	if (minus)
		sum = 0 - sum;
	
	return sum;
}

static void re_config(void)
{
    static struct file *filp = NULL;
	int c = 0;
	int i = 0;
	int j = 0;
	int reg_val = 0;
	int num_val = 0;
	int bEOF = 0;
    int nRead = 0;
    int f_reg = 0;
    int obj_num = 0;
    unsigned char buf[512];
    unsigned char num[10];
    uint8_t obj[64];
    unsigned char *p = NULL;
    mm_segment_t oldfs;

    oldfs = get_fs();
    set_fs(KERNEL_DS);
    
    filp = filp_open("/data/tp_cfg", O_RDONLY, 0);

    if (IS_ERR(filp))
    {
        filp = NULL;
        printk(KERN_ERR "Open tp_cfg error!\n");
        return;
    }

    p = kmalloc(1, GFP_KERNEL);
    
    for (;;)
    {
    	c = 0;
    	memset(buf, 0 , 512);
    
    	// read 1 row
    	do
    	{
    		nRead = filp->f_op->read(filp, p, 1, &filp->f_pos);
    		
    		if (nRead != 1)
    		{
				bEOF = 1;
				break;
			}
		
			buf[c++] = *p;
    	}while (*p != '\n');
    	
    	// process 1 row
    	if (!bEOF)
    	{
			memset(num, 0 , 10);
    		buf[c] = '\0';
    		j = 0;
    		f_reg = 0;
    		obj_num = 0;
    		
    		// i=1 for remove 'T'
    		for (i=1; i<c; i++)
    		{
    			// get obj type
    			if (buf[i] == '=')
    			{
    				num[j] = '\0';
    				reg_val = my_atoi(num);
    				obj[obj_num++] = reg_val;
    				//
    				f_reg = 1;
    				memset(num, 0, 10);
    				j = 0;
    			}
    			// get obj data
    			else if ((buf[i] == ',') && f_reg)
    			{
					num[j] = '\0';
					num_val = my_atoi(num);
					obj[obj_num++] = num_val;
					//
    				memset(num, 0, 10);
    				j = 0;
				}
    			else
    				num[j++] = buf[i];
    		}
    		
			//
			switch (obj[0])
			{
				case 7:
						for (i=1; i<obj_num; i++)
							v16_T7[i-1] = obj[i];
						break;
				case 8:
						for (i=1; i<obj_num; i++)
							v16_T8[i-1] = obj[i];
						break;
				case 9:
						for (i=1; i<obj_num; i++)
							v16_T9[i-1] = obj[i];
						break;
				case 15:
						for (i=1; i<obj_num; i++)
							v16_T15[i-1] = obj[i];
						break;
				case 18:
						for (i=1; i<obj_num; i++)
							v16_T18[i-1] = obj[i];
						break;
				case 19:
						for (i=1; i<obj_num; i++)
							v16_T19[i-1] = obj[i];
						break;
				case 20:
						for (i=1; i<obj_num; i++)
							v16_T20[i-1] = obj[i];
						break;
				case 22:
						for (i=1; i<obj_num; i++)
							v16_T22[i-1] = obj[i];
						break;
				case 23:
						for (i=1; i<obj_num; i++)
							v16_T23[i-1] = obj[i];
						break;
				case 24:
						for (i=1; i<obj_num; i++)
							v16_T24[i-1] = obj[i];
						break;
				case 25:
						for (i=1; i<obj_num; i++)
							v16_T25[i-1] = obj[i];
						break;
				case 28:
						for (i=1; i<obj_num; i++)
							v16_T28[i-1] = obj[i];
						break;
				case 38:
						for (i=1; i<obj_num; i++)
							v16_T38[i-1] = obj[i];
						break;
			}
    	}
    	else
    		break;
    }
    
	// close ini file
    filp_close(filp, NULL);
    filp = NULL;
    set_fs(oldfs);

	//
    printk("v16_T7=");
    for (i=0; i<sizeof(v16_T7); i++)
		printk("%d,", v16_T7[i]);
	printk("\n");
    printk("v16_T8=");
    for (i=0; i<sizeof(v16_T8); i++)
		printk("%d,", v16_T8[i]);
	printk("\n");
    printk("v16_T9=");
    for (i=0; i<sizeof(v16_T9); i++)
		printk("%d,", v16_T9[i]);
	printk("\n");
    printk("v16_T15=");
    for (i=0; i<sizeof(v16_T15); i++)
		printk("%d,", v16_T15[i]);
	printk("\n");
    printk("v16_T18=");
    for (i=0; i<sizeof(v16_T18); i++)
		printk("%d,", v16_T18[i]);
	printk("\n");
    printk("v16_T19=");
    for (i=0; i<sizeof(v16_T19); i++)
		printk("%d,", v16_T19[i]);
	printk("\n");
    printk("v16_T20=");
    for (i=0; i<sizeof(v16_T20); i++)
		printk("%d,", v16_T20[i]);
	printk("\n");
    printk("v16_T22=");
    for (i=0; i<sizeof(v16_T22); i++)
		printk("%d,", v16_T22[i]);
	printk("\n");
    printk("v16_T23=");
    for (i=0; i<sizeof(v16_T23); i++)
		printk("%d,", v16_T23[i]);
	printk("\n");
    printk("v16_T24=");
    for (i=0; i<sizeof(v16_T24); i++)
		printk("%d,", v16_T24[i]);
	printk("\n");
    printk("v16_T25=");
    for (i=0; i<sizeof(v16_T25); i++)
		printk("%d,", v16_T25[i]);
	printk("\n");
    printk("v16_T28=");
    for (i=0; i<sizeof(v16_T28); i++)
		printk("%d,", v16_T28[i]);
	printk("\n");
    printk("v16_T38=");
    for (i=0; i<sizeof(v16_T38); i++)
		printk("%d,", v16_T38[i]);
	printk("\n");
    
    //
    QT_InitConfig();
}

static ssize_t
qt_test_show(struct device *dev,
        struct device_attribute *attr,
        char *buf)
{
    return sprintf(buf, "%d\n", st_result);
}

static ssize_t
qt_test_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf,
        size_t count)
{
    int value = simple_strtoul(buf, NULL, 10);

    if (value == 0)
    {
    	printk(KERN_INFO "[Touch] [Cmd:%d] Run Self-test.\n", value);
    	QT_SelfTest_V2();
    }
    else if (value == 1)
    {
		printk(KERN_INFO "[Touch] [Cmd:%d] Read configuration from file.\n", value);
    	re_config();
    }
    else if (value == 2)
    {
		printk(KERN_INFO "[Touch] [Cmd:%d] Run touch calibration.\n", value);
    	QT_Calibrate();
    }
    else
    	printk(KERN_INFO "[Touch] [Cmd:%d] Unknow command.\n", value);
    
    return count;
}

static struct device_attribute qt_dev_attr_enable = __ATTR(qt_test, S_IRUGO|S_IWUSR|S_IWGRP,
        qt_test_show, qt_test_store);

static struct attribute *qt_attributes[] = {
    &qt_dev_attr_enable.attr,
    NULL
};

static struct attribute_group qt_attribute_group = {
    .attrs = qt_attributes
};

uint8_t init_touch_driver(uint8_t I2C_address)
{
    uint8_t iLoop = 0, status = 0;
    info_id_t *id;
    object_t *obj;
    uint8_t crc24[3];
    if(!info_block){
        return DRIVER_SETUP_INCOMPLETE;
    }

    // Read the info block data.
    id = kmalloc(sizeof(info_id_t), GFP_KERNEL);
    if (id == NULL)
    {
        return DRIVER_SETUP_INCOMPLETE;
    }

    if (read_id_block(id) != READ_MEM_OK)
    {
        TCH_ERR("read_id_block(id) != READ_MEM_OK");
        kfree(id);
        return DRIVER_SETUP_INCOMPLETE;
    }
    info_block->info_id = id;

    TCH_ERR("Family ID       = 0x%x", info_block->info_id->family_id);
    TCH_ERR("Variant ID      = 0x%x", info_block->info_id->variant_id);
    TCH_ERR("Version         = 0x%x", info_block->info_id->version);
    TCH_ERR("Build           = 0x%x", info_block->info_id->build);
    TCH_ERR("Matrix X size   = 0x%x", info_block->info_id->matrix_x_size);
    TCH_ERR("Matrix Y size   = 0x%x", info_block->info_id->matrix_y_size);
    TCH_ERR("Number of Table = 0x%x", info_block->info_id->num_declared_objects);

    // Read the Object Table data.
    obj = kmalloc(sizeof(object_t) * info_block->info_id->num_declared_objects, GFP_KERNEL);
    if (obj == NULL)
    {
        goto ERR_1;
    }

    if (read_object_table(obj, id->num_declared_objects) != READ_MEM_OK)
    {
        TCH_ERR("read_object_table(obj) != READ_MEM_OK");
        goto ERR_2;
    }
    info_block->objects = obj;

    for (iLoop=0; iLoop<info_block->info_id->num_declared_objects; iLoop++)
    {
        TCH_DBG(TOUCH_DEBUG_ATMEL, "type       = 0x%x", obj->object_type);
        TCH_DBG(TOUCH_DEBUG_ATMEL, "address    = 0x%x", obj->i2c_address);
        TCH_DBG(TOUCH_DEBUG_ATMEL, "size       = 0x%x", obj->size);
        TCH_DBG(TOUCH_DEBUG_ATMEL, "instances  = 0x%x", obj->instances);
        TCH_DBG(TOUCH_DEBUG_ATMEL, "report ids = 0x%x", obj->num_report_ids);
        TCH_DBG(TOUCH_DEBUG_ATMEL, "************************************************************");

        obj++;
    }

    // Read the Checksum info
    status = read_mem(info_block->info_id->num_declared_objects*6+7, 3, crc24);
    if (status != READ_MEM_OK)
    {
        TCH_ERR("Read CRC information failed, status = 0x%x", status);
        goto ERR_2;
    }
    info_block->CRC = (crc24[2] << 16) | (crc24[1] << 8) | crc24[0];
    TCH_DBG(TOUCH_DEBUG_COMMON, "Internal CRC = 0x%x", info_block->CRC);

    if (info_block->CRC != QT_CheckCRC())
    {
        TCH_ERR("Checksum failed.");
        goto ERR_2;
    }

    QT_GetConfigStatus();
    QT_InitConfig();
    QT_BackupNV();
    QT_Calibrate();

    TCH_DBG(TOUCH_DEBUG_COMMON, "Done.");
    return(DRIVER_SETUP_OK);

ERR_2:
   kfree(obj);
ERR_1:
   kfree(id);
   TCH_ERR("Failed.");
   return DRIVER_SETUP_INCOMPLETE;
}

static void qt602240_isr_workqueue(struct work_struct *work)
{
    struct qt602240_info *ts = container_of(work, struct qt602240_info, wqueue);

    uint16_t Buf_X;
    uint16_t Buf_Y;
    uint8_t count = 3, MSG_TYPE = 0;
    int i,first_touch_id = qt602240->first_finger_id;

    g_I2CAddress = g_RegAddr[GEN_MESSAGEPROCESSOR_T5];

    if (!ts->suspend_state)
    {//do once in project TBP & ITV & TNQ
        do // Read data until chip queue no data.
        {
            // If read/write I2C data faield, re-action 3 times.
            while (read_mem(g_I2CAddress , 8, (void *) g_MsgData ) != 0)
            {
                TCH_ERR("Read data failed, Re-read.");
                mdelay(3);

                count--;
                if (count == 0)
                {
                    TCH_MSG("Can't Read/write data, reset chip.");
                    if(goio_tp_rst_n)
                        TCH_MSG("goio_tp_rst_n = %d", gpio_get_value(goio_tp_rst_n));
                    TCH_MSG("gpio_tp_int_n = %d", gpio_get_value(gpio_tp_int_n));
                    qt602240_ts_reset();  // Re-try 3 times, can't read/write data, reset qt602240 chip
                    QT_GetConfigStatus();
                    QT_InitConfig();
                    QT_Calibrate();
                    return;
                }
            }
            
            MSG_TYPE = g_ReportID[g_MsgData[0]];
            
            if ((MSG_TYPE == 0xFF) || (MSG_TYPE == 0x00))   // No data, return
            {
                TCH_DBG(TOUCH_DEBUG_ISR, "No data, Leave ISR.");
                if(goio_tp_rst_n)
                    TCH_DBG(TOUCH_DEBUG_ISR, "goio_tp_rst_n = %d", gpio_get_value(goio_tp_rst_n));
                TCH_DBG(TOUCH_DEBUG_ISR, "gpio_tp_int_n = %d", gpio_get_value(gpio_tp_int_n));
                break;
            }
            //Div2-D5-Peripheral-FG-TouchErrorHandle-00+[
            else if (MSG_TYPE == GEN_COMMANDPROCESSOR_T6)
            {
                if (g_MsgData[1] & 0x04)
                    TCH_ERR("I2C-Compatible Checksum Errors.");
                if (g_MsgData[1] & 0x08)
                    TCH_ERR("Configuration Errors.");
                if (g_MsgData[1] & 0x10)
                    TCH_MSG("Calibrating.");
                if (g_MsgData[1] & 0x20)
                    TCH_ERR("Signal Errors.");
                if (g_MsgData[1] & 0x40)
                    TCH_ERR("Overflow Errors.");
                if (g_MsgData[1] & 0x80)
                    TCH_MSG("Reseting.");
                if (g_MsgData[1] & 0x04 /*|| ((g_MsgData[1] & 0x10) && !gpio_get_value(gpio_tp_int_n))*/ || g_MsgData[1] & 0x20)
                {
                    TCH_MSG("Prepare to reset touch IC. Please wait a moment...");
                    qt602240_ts_reset();
                    QT_GetConfigStatus();
                    QT_InitConfig();
                    QT_Calibrate();
                    TCH_MSG("Finish to reset touch IC.");
                    if(goio_tp_rst_n)
                        TCH_MSG("goio_tp_rst_n = %d", gpio_get_value(goio_tp_rst_n));
                    TCH_MSG("gpio_tp_int_n = %d", gpio_get_value(gpio_tp_int_n));
                    return;
                }
                return;
            }
            //Div2-D5-Peripheral-FG-TouchErrorHandle-00+]
    //MTD-BSP-Colin-Touch03++[
    		else if ((MSG_TYPE == TOUCH_KEYARRAY_T15))
            {
                if(g_NVBackUp){
                    TCH_ERR("NVBackUp Not Down!");
                    return;

                }
                // Key1
                if (g_MsgData[2] & 0x01)
                {
                    if (!ts->key_state[0])
                    {
                        input_report_key(ts->keyevent_input, KEY_HOME, 1);
                        TCH_DBG(TOUCH_DEBUG_EVENT, "Report HOME key Down.");
                        ts->key_state[0] = TRUE;
                    }
                }
                else
                {
                    if (ts->key_state[0])
                    {
                        input_report_key(ts->keyevent_input, KEY_HOME, 0);
                        TCH_DBG(TOUCH_DEBUG_EVENT, "Report HOME key Up.");
                        ts->key_state[0] = FALSE;
                    }
                }
                // Key2
                if (g_MsgData[2] & 0x02)
                {
                    if (!ts->key_state[1])
                    {
                        input_report_key(ts->keyevent_input, KEY_MENU, 1);
                        TCH_DBG(TOUCH_DEBUG_EVENT, "Report MENU key Down.");
                        ts->key_state[1] = TRUE;
                    }
                }
                else
                {
                    if (ts->key_state[1])
                    {
                        input_report_key(ts->keyevent_input, KEY_MENU, 0);
                        TCH_DBG(TOUCH_DEBUG_EVENT, "Report MENU key Up.");
                        ts->key_state[1] = FALSE;
                    }
                }
                // Key3
                if (g_MsgData[3] & 0x02)
                {
                    if (!ts->key_state[2])
                    {
                        input_report_key(ts->keyevent_input, KEY_BACK, 1);
                        TCH_DBG(TOUCH_DEBUG_EVENT, "Report BACK key Down.");
                        ts->key_state[2] = TRUE;
                    }
                }
                else
                {
                    if (ts->key_state[2])
                    {
                        input_report_key(ts->keyevent_input, KEY_BACK, 0);
                        TCH_DBG(TOUCH_DEBUG_EVENT, "Report BACK key Up.");
                        ts->key_state[2] = FALSE;
                    }
                }
                // Key4
                if (g_MsgData[3] & 0x04)
                {
                    if (!ts->key_state[3])
                    {
                        input_report_key(ts->keyevent_input, KEY_SEARCH, 1);
                        TCH_DBG(TOUCH_DEBUG_EVENT, "Report SEARCH key Down.");
                        ts->key_state[3] = TRUE;
                    }
                }
                else
                {
                    if (ts->key_state[3])
                    {
                        input_report_key(ts->keyevent_input, KEY_SEARCH, 0);
                        TCH_DBG(TOUCH_DEBUG_EVENT, "Report SEARCH key Up.");
                        ts->key_state[3] = FALSE;
                    }
                }
            }
    //MTD-BSP-Colin-Touch03++]
            else if ((MSG_TYPE == PROCI_GRIPFACESUPPRESSION_T20))
            {
                if(g_NVBackUp){
                    TCH_ERR("NVBackUp Not Down!");
                    return;

                }
                if (g_MsgData[1] & 0x01)
                {
                    TCH_MSG("Face Down, Status = 0x%x, ID = %d", g_MsgData[1], MSG_TYPE);
                    ts->facedetect = TRUE;
                    //continue;
                }
                else if (ts->facedetect)
                {
                    TCH_MSG("Face Up, Status = 0x%x, ID = %d", g_MsgData[1], MSG_TYPE);
                    ts->facedetect = FALSE;
                    for (i=0; i<TS_MAX_POINTS; i++)
                    {
                        if (ts->points[i].last_area > NO_TOUCH)
                        {   // Report a touch pen up event when touch detects finger down events in face detection(Face Up).
                            qt602240_report(ts, i, ts->points[i].x, ts->points[i].y, NO_TOUCH);
                            //TCH_MSG("POS%d Up, Status = 0x%x, X = %d, Y = %d, ID = %d", i+1, g_MsgData[1], ts->points[i].x, ts->points[i].y, MSG_TYPE);
                            input_sync(ts->touch_input);	//Div2-D5-Peripheral-FG-AddMultiTouch-00+
                        }
                    }
                }
                return;
            }
            else if (MSG_TYPE != TOUCH_MULTITOUCHSCREEN_T9)
            {
                TCH_DBG(TOUCH_DEBUG_ATMEL, "Get other report id = %d", MSG_TYPE);
                
                for (i=0; i<8; i++)
                    TCH_DBG(TOUCH_DEBUG_ATMEL, "Report T%d[%d] = %d", MSG_TYPE, i ,g_MsgData[i]);            
                //continue;
                return;
            }

            if (g_NVBackUp)
            {
                TCH_ERR("NVBackUp Not Down!");
                return;

            }
            
            Buf_X = (g_MsgData[2] << 2) | ((g_MsgData[4] & ~0x3f)>> 6);
            Buf_Y = (g_MsgData[3] << 2) | ((g_MsgData[4] & ~0xf3)>> 2);

			if (g_MsgData[1] & 0xC0)
				TCH_DBG(TOUCH_DEBUG_ISR, "POS %d, x = %d, y = %d, Down", g_MsgData[0] - first_touch_id + 1, Buf_X, Buf_Y);
            		
			if (g_MsgData[1] & 0x20)
				TCH_DBG(TOUCH_DEBUG_ISR, "POS %d, x = %d, y = %d, Up", g_MsgData[0] - first_touch_id + 1, Buf_X, Buf_Y);
                       
            for (i=0; i<TS_MAX_POINTS; i++)
            {
                if (g_MsgData[0] == first_touch_id + i)
                {
                    ts->points[i].x = Buf_X;
                    ts->points[i].y = Buf_Y;

                    if (g_MsgData[1] & 0xC0)
                    {	// Finger Down.
                        //Div2-D5-Peripheral-FG-AddMultiTouch-01*[
                        if (i == 0)
                        {
                            qt602240_report(ts, i, ts->points[i].x, ts->points[i].y, 255);
#ifdef QT602240_MT
                            if (ts->points[i+1].last_area > NO_TOUCH)
                                qt602240_report(ts, i+1, ts->points[i+1].x, ts->points[i+1].y, 255);
                            
                            if (ts->points[i+2].last_area > NO_TOUCH)
                                qt602240_report(ts, i+2, ts->points[i+2].x, ts->points[i+2].y, 255);
                            
                            if (ts->points[i+3].last_area > NO_TOUCH)
                                qt602240_report(ts, i+3, ts->points[i+3].x, ts->points[i+3].y, 255);
                            
                            if (ts->points[i+4].last_area > NO_TOUCH)
                                qt602240_report(ts, i+4, ts->points[i+4].x, ts->points[i+4].y, 255);
#endif
                            input_sync(ts->touch_input);
                        }
                        else if (i == 1)
                        {
#ifdef QT602240_MT
                            if (ts->points[i-1].last_area > NO_TOUCH)
                                qt602240_report(ts, i-1, ts->points[i-1].x, ts->points[i-1].y, 255);
                            
                            if (ts->points[i+1].last_area > NO_TOUCH)
                                qt602240_report(ts, i+1, ts->points[i+1].x, ts->points[i+1].y, 255);
                            
                            if (ts->points[i+2].last_area > NO_TOUCH)
                                qt602240_report(ts, i+2, ts->points[i+2].x, ts->points[i+2].y, 255);
                            
                            if (ts->points[i+3].last_area > NO_TOUCH)
                                qt602240_report(ts, i+3, ts->points[i+3].x, ts->points[i+3].y, 255);
#endif
                            qt602240_report(ts, i, ts->points[i].x, ts->points[i].y, 255);
                            
                            input_sync(ts->touch_input);
                        }
                        else if (i == 2)
                        {
#ifdef QT602240_MT
                            if (ts->points[i-2].last_area > NO_TOUCH)
                                qt602240_report(ts, i-2, ts->points[i-2].x, ts->points[i-2].y, 255);
                            
                            if (ts->points[i-1].last_area > NO_TOUCH)
                                qt602240_report(ts, i-1, ts->points[i-1].x, ts->points[i-1].y, 255);
                            
                            if (ts->points[i+1].last_area > NO_TOUCH)
                                qt602240_report(ts, i+1, ts->points[i+1].x, ts->points[i+1].y, 255);
                            
                            if (ts->points[i+2].last_area > NO_TOUCH)
                                qt602240_report(ts, i+2, ts->points[i+2].x, ts->points[i+2].y, 255);
#endif
                            qt602240_report(ts, i, ts->points[i].x, ts->points[i].y, 255);
                            
                            input_sync(ts->touch_input);
    					}
                        else if (i == 3)
                        {
#ifdef QT602240_MT
                            if (ts->points[i-3].last_area > NO_TOUCH)
                                qt602240_report(ts, i-3, ts->points[i-3].x, ts->points[i-3].y, 255);
                            
                            if (ts->points[i-2].last_area > NO_TOUCH)
                                qt602240_report(ts, i-2, ts->points[i-2].x, ts->points[i-2].y, 255);
                            
                            if (ts->points[i-1].last_area > NO_TOUCH)
                                qt602240_report(ts, i-1, ts->points[i-1].x, ts->points[i-1].y, 255);
                            
                            if (ts->points[i+1].last_area > NO_TOUCH)
                                qt602240_report(ts, i+1, ts->points[i+1].x, ts->points[i+1].y, 255);
#endif
                            qt602240_report(ts, i, ts->points[i].x, ts->points[i].y, 255);
                            
                            input_sync(ts->touch_input);
    					}
                        else if (i == 4)
                        {
#ifdef QT602240_MT
                            if (ts->points[i-4].last_area > NO_TOUCH)
                                qt602240_report(ts, i-4, ts->points[i-4].x, ts->points[i-4].y, 255);
                            
                            if (ts->points[i-3].last_area > NO_TOUCH)
                                qt602240_report(ts, i-3, ts->points[i-3].x, ts->points[i-3].y, 255);
                            
                            if (ts->points[i-2].last_area > NO_TOUCH)
                                qt602240_report(ts, i-2, ts->points[i-2].x, ts->points[i-2].y, 255);
                            
                            if (ts->points[i-1].last_area > NO_TOUCH)
                                qt602240_report(ts, i-1, ts->points[i-1].x, ts->points[i-1].y, 255);
#endif
                            qt602240_report(ts, i, ts->points[i].x, ts->points[i].y, 255);
                            
                            input_sync(ts->touch_input);
                        }
                        //input_sync(ts->touch_input);
                        //Div2-D5-Peripheral-FG-AddMultiTouch-01*]
                    }
                    else if ((g_MsgData[1] & 0x20) == 0x20)
                    {	// Finger Up.
                        if (ts->points[i].last_area > NO_TOUCH)
                        {
                            if (i == 0)
                            {
                                qt602240_report(ts, i, ts->points[i].x, ts->points[i].y, NO_TOUCH);
#ifdef QT602240_MT
                                if (ts->points[i+1].last_area > NO_TOUCH)
                                    qt602240_report(ts, i+1, ts->points[i+1].x, ts->points[i+1].y, 255);
                                
                                if (ts->points[i+2].last_area > NO_TOUCH)
                                    qt602240_report(ts, i+2, ts->points[i+2].x, ts->points[i+2].y, 255);
                                
                                if (ts->points[i+3].last_area > NO_TOUCH)
                                    qt602240_report(ts, i+3, ts->points[i+3].x, ts->points[i+3].y, 255);
                                
                                if (ts->points[i+4].last_area > NO_TOUCH)
                                    qt602240_report(ts, i+4, ts->points[i+4].x, ts->points[i+4].y, 255);
#endif
                                input_sync(ts->touch_input);
#ifdef QT602240_MT
                                if (ts->points[i+1].last_area > NO_TOUCH)
                                    qt602240_report(ts, i+1, ts->points[i+1].x, ts->points[i+1].y, 255);
                                
                                if (ts->points[i+2].last_area > NO_TOUCH)
                                    qt602240_report(ts, i+2, ts->points[i+2].x, ts->points[i+2].y, 255);
                                
                                if (ts->points[i+3].last_area > NO_TOUCH)
                                    qt602240_report(ts, i+3, ts->points[i+3].x, ts->points[i+3].y, 255);
                                
                                if (ts->points[i+4].last_area > NO_TOUCH)
                                    qt602240_report(ts, i+4, ts->points[i+4].x, ts->points[i+4].y, 255);
#endif
                            }
                            else if (i == 1)
                            {
#ifdef QT602240_MT
                                if (ts->points[i-1].last_area > NO_TOUCH)
                                    qt602240_report(ts, i-1, ts->points[i-1].x, ts->points[i-1].y, 255);
                                
                                if (ts->points[i+1].last_area > NO_TOUCH)
                                    qt602240_report(ts, i+1, ts->points[i+1].x, ts->points[i+1].y, 255);
                                
                                if (ts->points[i+2].last_area > NO_TOUCH)
                                    qt602240_report(ts, i+2, ts->points[i+2].x, ts->points[i+2].y, 255);
                                
                                if (ts->points[i+3].last_area > NO_TOUCH)
                                    qt602240_report(ts, i+3, ts->points[i+3].x, ts->points[i+3].y, 255);
#endif
                                qt602240_report(ts, i, ts->points[i].x, ts->points[i].y, NO_TOUCH);
                                
                                input_sync(ts->touch_input);
#ifdef QT602240_MT
                                if (ts->points[i-1].last_area > NO_TOUCH)
                                    qt602240_report(ts, i-1, ts->points[i-1].x, ts->points[i-1].y, 255);
                                
                                if (ts->points[i+1].last_area > NO_TOUCH)
                                    qt602240_report(ts, i+1, ts->points[i+1].x, ts->points[i+1].y, 255);
                                
                                if (ts->points[i+2].last_area > NO_TOUCH)
                                    qt602240_report(ts, i+2, ts->points[i+2].x, ts->points[i+2].y, 255);
                                
                                if (ts->points[i+3].last_area > NO_TOUCH)
                                    qt602240_report(ts, i+3, ts->points[i+3].x, ts->points[i+3].y, 255);
#endif
                            }
                            else if (i == 2)
                            {
#ifdef QT602240_MT
                                if (ts->points[i-2].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-2, ts->points[i-2].x, ts->points[i-2].y, 255);

                                if (ts->points[i-1].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-1, ts->points[i-1].x, ts->points[i-1].y, 255);

                                if (ts->points[i+1].last_area > NO_TOUCH)
                                	qt602240_report(ts, i+1, ts->points[i+1].x, ts->points[i+1].y, 255);

                                if (ts->points[i+2].last_area > NO_TOUCH)
                                	qt602240_report(ts, i+2, ts->points[i+2].x, ts->points[i+2].y, 255);
#endif
                                qt602240_report(ts, i, ts->points[i].x, ts->points[i].y, NO_TOUCH);

                                input_sync(ts->touch_input);
#ifdef QT602240_MT
                                if (ts->points[i-2].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-2, ts->points[i-2].x, ts->points[i-2].y, 255);

                                if (ts->points[i-1].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-1, ts->points[i-1].x, ts->points[i-1].y, 255);

                                if (ts->points[i+1].last_area > NO_TOUCH)
                                	qt602240_report(ts, i+1, ts->points[i+1].x, ts->points[i+1].y, 255);

                                if (ts->points[i+2].last_area > NO_TOUCH)
                                	qt602240_report(ts, i+2, ts->points[i+2].x, ts->points[i+2].y, 255);
#endif
                            }
                            else if (i == 3)
                            {
#ifdef QT602240_MT
                                if (ts->points[i-3].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-3, ts->points[i-3].x, ts->points[i-3].y, 255);

                                if (ts->points[i-2].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-2, ts->points[i-2].x, ts->points[i-2].y, 255);

                                if (ts->points[i-1].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-1, ts->points[i-1].x, ts->points[i-1].y, 255);

                                if (ts->points[i+1].last_area > NO_TOUCH)
                                	qt602240_report(ts, i+1, ts->points[i+1].x, ts->points[i+1].y, 255);
#endif
                                qt602240_report(ts, i, ts->points[i].x, ts->points[i].y, NO_TOUCH);

                                input_sync(ts->touch_input);
#ifdef QT602240_MT
                                if (ts->points[i-3].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-3, ts->points[i-3].x, ts->points[i-3].y, 255);

                                if (ts->points[i-2].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-2, ts->points[i-2].x, ts->points[i-2].y, 255);

                                if (ts->points[i-1].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-1, ts->points[i-1].x, ts->points[i-1].y, 255);

                                if (ts->points[i+1].last_area > NO_TOUCH)
                                	qt602240_report(ts, i+1, ts->points[i+1].x, ts->points[i+1].y, 255);
#endif
                            }
                            else if (i == 4)
                            {
#ifdef QT602240_MT
                                if (ts->points[i-4].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-4, ts->points[i-4].x, ts->points[i-4].y, 255);

                                if (ts->points[i-3].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-3, ts->points[i-3].x, ts->points[i-3].y, 255);

                                if (ts->points[i-2].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-2, ts->points[i-2].x, ts->points[i-2].y, 255);

                                if (ts->points[i-1].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-1, ts->points[i-1].x, ts->points[i-1].y, 255);
#endif
                                qt602240_report(ts, i, ts->points[i].x, ts->points[i].y, NO_TOUCH);

                                input_sync(ts->touch_input);
#ifdef QT602240_MT
                                if (ts->points[i-4].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-4, ts->points[i-4].x, ts->points[i-4].y, 255);

                                if (ts->points[i-3].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-3, ts->points[i-3].x, ts->points[i-3].y, 255);

                                if (ts->points[i-2].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-2, ts->points[i-2].x, ts->points[i-2].y, 255);

                                if (ts->points[i-1].last_area > NO_TOUCH)
                                	qt602240_report(ts, i-1, ts->points[i-1].x, ts->points[i-1].y, 255);
#endif
                            }
                            
                            input_sync(ts->touch_input);
                        }
                    }
                }
            }
            return;
        }while(1);
    }
}

static irqreturn_t qt602240_isr(int irq, void * handle)
{
    struct qt602240_info *ts = handle;
    
    if (!ts->suspend_state)
        schedule_work(&ts->wqueue);

    return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void qt602240_early_suspend(struct early_suspend *h)
{
    uint16_t power_cfg_address = 0;
    uint8_t data[3];
    uint8_t rc = 0;

    if(!Project_Flag)	//FIH-SD4-BSP, Simon
    cancel_delayed_work_sync(&touch_delayed_work); // FIH-Div2-SW2-BSP, Ming

    power_cfg_address = g_RegAddr[GEN_POWERCONFIG_T7];
    read_mem(power_cfg_address, 3, (void *) qt602240->T7);

    data[0] = 0;
    data[1] = 0;
    data[2] = 0;

    //QT_GetConfigStatus();  // Clear all data, avoid intrrupt no resume.

    rc = write_mem(power_cfg_address, 3, (void *) data);
    if (rc != 0)
        TCH_ERR("Driver can't enter deep sleep mode [%d].", rc);
    else
        TCH_MSG("Enter deep sleep mode.");

    disable_irq(qt602240->irq);
    qt602240->suspend_state = TRUE;
    //gpio_tlmm_config(GPIO_CFG(gpio_tp_int_n, 0, GPIO_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    if(goio_tp_rst_n)
    {
        gpio_tlmm_config(GPIO_CFG(goio_tp_rst_n, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
        gpio_set_value(goio_tp_rst_n, HIGH);
    
    
        TCH_MSG("goio_tp_rst_n = %d", gpio_get_value(goio_tp_rst_n));
    }
    TCH_MSG("gpio_tp_int_n = %d", gpio_get_value(gpio_tp_int_n));

    // gpio 117 low
    gpio_tlmm_config(GPIO_CFG(117, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    gpio_set_value(117, LOW);

    TCH_DBG(TOUCH_DEBUG_COMMON, "Done.");
}

void qt602240_late_resume(struct early_suspend *h)
{
    uint16_t power_cfg_address = 0;
    uint8_t i, rc = 0;

    // gpio 117 high
    gpio_tlmm_config(GPIO_CFG(117, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    gpio_set_value(117, HIGH);

    power_cfg_address = g_RegAddr[GEN_POWERCONFIG_T7];
    //gpio_tlmm_config(GPIO_CFG(gpio_tp_int_n, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    if(goio_tp_rst_n)
    {  
        gpio_tlmm_config(GPIO_CFG(goio_tp_rst_n, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
        gpio_set_value(goio_tp_rst_n, HIGH);
    }
    qt602240->suspend_state = FALSE;

    //Div2-D5-Peripheral-FG-TouchErrorHandle-00*[

    for (i=0; i<TS_MAX_POINTS; i++)
    {
        if (qt602240->points[i].last_area > NO_TOUCH)
		{
			//TCH_MSG("POS%d Up, X = %d, Y = %d", i+1, qt602240->points[i].x, qt602240->points[i].y);
			qt602240_report(qt602240, i, qt602240->points[i].x, qt602240->points[i].y, NO_TOUCH);
			input_sync(qt602240->touch_input);
		}
    }

    for (i=0;i<3;i++)
    {
            rc = write_mem(power_cfg_address, 3, (void *) qt602240->T7);
            if (rc != 0)
            printk(KERN_ERR "Driver can't return from deep sleep mode [%d].", rc);
            else
    	{
            printk(KERN_INFO "Return from sleep mode.\n");
            break;
    	}
    }
    
    //msleep(25);

    //qt602240_ts_reset();
    QT_GetConfigStatus();
    //QT_InitConfig();
    QT_Calibrate();

    //Div2-D5-Peripheral-FG-TouchErrorHandle-00*]
    enable_irq(qt602240->irq);

    if(goio_tp_rst_n)
    printk(KERN_INFO "[Touch] qt602240_late_resume: goio_tp_rst_n = %d", gpio_get_value(goio_tp_rst_n));
    printk(KERN_INFO "[Touch] qt602240_late_resume: gpio_tp_int_n = %d", gpio_get_value(gpio_tp_int_n));
    TCH_DBG(TOUCH_DEBUG_COMMON, "Done.");
}
#endif

static int qt602240_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct input_dev *touch_input;
    struct input_dev *keyevent_input;
    //struct vreg *device_vreg;//MTD-BSP-Colin-Touch02++
    int i, rc = 0;	//MTD-BSP-Colin-Touch02++
    int irqrequest = 0;
    int rt;
	
    printk(KERN_INFO"[qt602240] %s\n",__func__);
    
    INIT_DELAYED_WORK(&touch_delayed_work, water_proof_on); // FIH-Div2-SW2-BSP, Ming

    /* FIHSPEC, AmberYang { */
    /* Patch: FixTouchCannotUseIssue */
    if(gpio_camreset != -1) 
    {
	gpio_request(gpio_camreset, "camera reset");
	gpio_set_value(gpio_camreset, 0);
	msleep(10);
	gpio_set_value(gpio_camreset, 1);
	msleep(10);
	gpio_set_value(gpio_camreset, 0);          
	gpio_free(gpio_camreset);
    }
    /* } FIHSPEC, AmberYang */

    g_NVBackUp = 0;
    
    // I2C Check
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        TCH_ERR("Check I2C functionality failed.");
        return -ENODEV;
    }

    qt602240 = kzalloc(sizeof(struct qt602240_info), GFP_KERNEL);
    if (qt602240 == NULL)
    {
        TCH_ERR("Can not allocate memory.");
        return -ENOMEM;
    }

    info_block = kzalloc(sizeof(info_block_t), GFP_KERNEL);
    if (info_block == NULL)
    {
        TCH_ERR("Can not allocate info block memory.");
        kfree(qt602240);
        return -ENOMEM;
    }
#if 0 //MTD-BSP-Colin-Touch02++[
    // Support Power
    device_vreg = vreg_get(0, TOUCH_DEVICE_VREG);
    if (!device_vreg) {
        TCH_ERR("driver %s: vreg get failed.", __func__);
        return -EIO;
    }

    vreg_set_level(device_vreg, 3000);
    rc = vreg_enable(device_vreg);
    TCH_DBG(TOUCH_DEBUG_COMMON, "Power status = %d", rc);
#endif
    // Config GPIO
    //Div2-D5-Peripheral-FG-4Y6TouchPorting-00*[
    #if defined(CONFIG_FIH_PROJECT_SF4Y6)
    if ((pm8058_gpio_config(PM8058_gpio_tp_int_n, &touch_interrupt)) < 0)
        TCH_ERR("Config IRQ GPIO failed.");
    #else
//MTD-BSP-Colin-Touch03++[
    if(gpio_pwr != -1) 
    {
	gpio_tlmm_config(GPIO_CFG(gpio_pwr, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_set_value(gpio_pwr, HIGH);
	for (i=0; i<10; i++)
	{
        	mdelay(20);
		if (gpio_get_value(gpio_pwr) == LOW)
            	TCH_ERR("PULL 3V EN failed.");
    	}
    }
    gpio_tlmm_config(GPIO_CFG(gpio_tp_int_n, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    gpio_request(gpio_tp_int_n,"TP_INT_N");
	if (gpio_direction_input(gpio_tp_int_n) < 0)
       TCH_ERR("Direct IRQ GPIO failed.");
	//if (gpio_to_irq(gpio_tp_int_n) < 0)
       //TCH_ERR("Request IRQ GPIO failed.");
//MTD-BSP-Colin-Touch03++]
    #endif
    //Div2-D5-Peripheral-FG-4Y6TouchPorting-00*]
    //Div2-D5-Peripheral-FG-4Y6TouchFineTune-01+[
    // Init Work Queue and Register IRQ
    INIT_WORK(&qt602240->wqueue, qt602240_isr_workqueue);
	//MTD-BSP-Colin-Touch02++]
	
    //Div2-D5-Peripheral-FG-4Y6TouchFineTune-01+]
    if(goio_tp_rst_n)
    {
        gpio_tlmm_config(GPIO_CFG(goio_tp_rst_n, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
        gpio_set_value(goio_tp_rst_n, HIGH);
    }    
    qt602240_ts_reset();

    // Confirm Touch Chip
    qt602240->client = client;
    i2c_set_clientdata(client, qt602240);
    qt602240->client->addr = 0x4A;//MTD-BSP-Colin-Touch03++
    //Div2-D5-Peripheral-FG-4Y6TouchFineTune-02*[
    for (i=0; i<5; i++)
    {
        if (init_touch_driver(qt602240->client->addr) == DRIVER_SETUP_INCOMPLETE)
            TCH_ERR("Init touch driver failed %d times.", i+1);
        else
            break;
    }
    if (i==5)
        goto err1;
    //Div2-D5-Peripheral-FG-4Y6TouchFineTune-02*]



//MTD-BSP-Colin-Touch03++[
    if (0 == rc) {//ITV&TBP&TNQ use low trigger mode NJ-BSP-simon
        qt602240->irq = MSM_GPIO_TO_INT(gpio_tp_int_n);
        if (qt602240->irq) {
            if (request_irq(qt602240->irq, qt602240_isr, IRQF_TRIGGER_FALLING, client->dev.driver->name, qt602240)) {
                TCH_ERR("Request IRQ failed.");
                goto err1;
            }
            else {
                irqrequest = 1;
            }
        }
        else {
            TCH_ERR("Request IRQ failed IRQ NULL.");
        }
    }

    touch_input = input_allocate_device();
    if (touch_input == NULL)
    {
        TCH_ERR("Can not allocate memory for touch input device.");
        goto err1;
    }

    keyevent_input = input_allocate_device();
    if (keyevent_input == NULL)
    {
        TCH_ERR("Can not allocate memory for key input device.");
        goto err1;
    }

    touch_input->name  = "qt602240touch";
    touch_input->phys  = "qt602240/input0";
    set_bit(EV_ABS, touch_input->evbit);
    set_bit(EV_SYN, touch_input->evbit);
//Div2-D5-Peripheral-FG-AddMultiTouch-01*[
#ifndef QT602240_MT
    set_bit(EV_KEY, touch_input->evbit);
    set_bit(BTN_TOUCH, touch_input->keybit);
    set_bit(BTN_2, touch_input->keybit);
    input_set_abs_params(touch_input, ABS_X, TS_MIN_X, TS_MAX_X, 0, 0);
    input_set_abs_params(touch_input, ABS_Y, TS_MIN_Y, TS_MAX_Y, 0, 0);
    input_set_abs_params(touch_input, ABS_HAT0X, TS_MIN_X, TS_MAX_X, 0, 0);
    input_set_abs_params(touch_input, ABS_HAT0Y, TS_MIN_Y, TS_MAX_Y, 0, 0);
    input_set_abs_params(touch_input, ABS_PRESSURE, 0, 255, 0, 0);
#else
    input_set_abs_params(touch_input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(touch_input, ABS_MT_POSITION_X, TS_MIN_X, TS_MAX_X, 0, 0);
    input_set_abs_params(touch_input, ABS_MT_POSITION_Y, TS_MIN_Y, TS_MAX_Y, 0, 0);
    input_set_abs_params(touch_input, ABS_X, TS_MIN_X, TS_MAX_X, 0, 0);
    input_set_abs_params(touch_input, ABS_Y, TS_MIN_Y, TS_MAX_Y, 0, 0);
#endif
//Div2-D5-Peripheral-FG-AddMultiTouch-01*]

    keyevent_input->name  = "qt602240key";
    keyevent_input->phys  = "qt602240/input1";
    set_bit(EV_KEY, keyevent_input->evbit);
    set_bit(KEY_HOME, keyevent_input->keybit);
    set_bit(KEY_MENU, keyevent_input->keybit);
    set_bit(KEY_BACK, keyevent_input->keybit);
	set_bit(KEY_SEARCH, keyevent_input->keybit);//MTD-BSP-Colin-Touch03++
    input_set_abs_params(keyevent_input, ABS_X, 0, 0, 0, 0);

    qt602240->touch_input = touch_input;
    if (input_register_device(qt602240->touch_input))
    {
        TCH_ERR("Can not register touch input device.");
        goto err2;
    }

    qt602240->keyevent_input = keyevent_input;
    if (input_register_device(qt602240->keyevent_input))
    {
        TCH_ERR("Can not register key input device.");
        goto err3;
    }

    memset(qt602240->points, 0, sizeof(struct point_info));
    qt602240->suspend_state = FALSE;
    qt602240->facedetect = FALSE;
    qt602240->T7[0] = 32;
    qt602240->T7[1] = 10;
    qt602240->T7[2] = 50;
#ifdef CONFIG_HAS_EARLYSUSPEND
    // Register early_suspend
    //qt602240->es.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    qt602240->es.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;	// resume quickly
    qt602240->es.suspend = qt602240_early_suspend;
    qt602240->es.resume = qt602240_late_resume;

    register_early_suspend(&qt602240->es);
#endif

    //
    rt = sysfs_create_group(&touch_input->dev.kobj,
            &qt_attribute_group);
    
    if (rt)
        TCH_ERR("sysfs_create_group failed!");
        
//NJ-BSP-Simon-Touch-TBP & TNQ & ITV  
// 2011.10.28
//set flag to check calibration after power on
    cal_check_flag = 1;
	
    TCH_DBG(TOUCH_DEBUG_COMMON, "Done.");
    return 0;

err3:
    input_unregister_device(qt602240->touch_input);
//    input_unregister_device(qt602240->keyevent_input);
err2:
    input_free_device(touch_input);
    input_free_device(keyevent_input);
err1:
    //vreg_disable(device_vreg);//MTD-BSP-Colin-Touch02++
//MTD-BSP-Colin-Touch03++[
    if (qt602240->irq && irqrequest){
		free_irq(qt602240->irq, qt602240);
    }
//MTD-BSP-Colin-Touch03++]
    dev_set_drvdata(&client->dev, 0);
    kfree(qt602240);
    kfree(info_block);
    TCH_ERR("Failed.");
    return -1;
}

static int qt602240_remove(struct i2c_client * client)
{
//MTD-BSP-Colin-Touch03++[
	if(qt602240->irq)
    	free_irq(qt602240->irq, qt602240);
//MTD-BSP-Colin-Touch03++]

    cancel_work_sync(&qt602240->wqueue);
    kfree(&qt602240->wqueue);
    gpio_free(gpio_tp_int_n);

#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&qt602240->es);
#endif

    input_unregister_device(qt602240->touch_input);
    input_unregister_device(qt602240->keyevent_input);

    kfree(qt602240->touch_input);
    kfree(qt602240->keyevent_input);
    input_free_device(qt602240->touch_input);
    input_free_device(qt602240->keyevent_input);

    kfree(qt602240);
    kfree(info_block->info_id);
    kfree(info_block);

    dev_set_drvdata(&client->dev, 0);
    TCH_DBG(TOUCH_DEBUG_COMMON, "Done.");
    return 0;
}

#if 0
static int qt602240_suspend(struct i2c_client *client, pm_message_t state)
{
    TCH_DBG(TOUCH_DEBUG_COMMON, "Done.");
    return 0;
}

static int qt602240_resume(struct i2c_client *client)
{
    TCH_DBG(TOUCH_DEBUG_COMMON, "Done.");
    return 0;
}
#endif

static const struct i2c_device_id qt602240_id[] = {
    { TOUCH_DEVICE_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, qt602240_id);

static struct i2c_driver qt602240_i2c_driver = {
   .driver = {
      .name   = TOUCH_DEVICE_NAME,
   },
   .id_table   = qt602240_id,
   .probe      = qt602240_probe,
   .remove     = qt602240_remove,
   //.suspend    = qt602240_suspend,
   //.resume     = qt602240_resume,
};


#include "NJ_qt602240.c"

static struct i2c_driver qt602240_i2c_driver_V2 = {
   .driver = {
      .name   = TOUCH_DEVICE_NAME,
   },
   .id_table   = qt602240_id,
   .probe      = qt602240_probe_V2,
   .remove     = qt602240_remove,
   //.suspend    = qt602240_suspend,
   //.resume     = qt602240_resume,
};

static int __init qt602240_init_v1(void *hdl )
{
    printk(KERN_INFO"[qt602240] %s\n",__func__);

    /*IRM*/
    TS_MAX_X            = 1023;
    TS_MAX_Y            = 1024;
    
    gpio_camreset	= 126;
    gpio_pwr		= 117;

    TS_MENUKEYL_X       = 50;
    TS_MENUKEYR_X       = 205;
    TS_HOMEKEYL_X       = 459;
    TS_HOMEKEYR_X       = 614;
    TS_BACKKEYL_X       = 818;
    TS_BACKKEYR_X       = 973;


    return ADD_DYN_I2C_DRIVER(hdl, &qt602240_i2c_driver);
}
DECLARE_DYN_LOADER(DRIVER_TYPE_TOUCH, TOUCH_ATMEL_QT602240_V1, qt602240_init_v1, LEVEL6);

static int __init qt602240_init_v2(void *hdl )
{
    printk(KERN_INFO"[qt602240] %s\n",__func__);

    /*IRM*/
//    TS_MAX_X            = 1023;
//    TS_MAX_Y            = 1024;
    TS_MAX_X    = 480;
    TS_MAX_Y    = 854;
    
    gpio_camreset	= 126;
    gpio_pwr		= 117;

    TS_MENUKEYL_X       = 50;
    TS_MENUKEYR_X       = 205;
    TS_HOMEKEYL_X       = 459;
    TS_HOMEKEYR_X       = 614;
    TS_BACKKEYL_X       = 818;
    TS_BACKKEYR_X       = 973;

    return ADD_DYN_I2C_DRIVER(hdl, &qt602240_i2c_driver);
}
DECLARE_DYN_LOADER(DRIVER_TYPE_TOUCH, TOUCH_ATMEL_QT602240_V2, qt602240_init_v2, LEVEL6);

// TBP
static int __init qt602240_init_v3(void *hdl )
{
    printk(KERN_INFO"[qt602240] %s\n",__func__);

    /*TBP*/
    TS_MAX_X    = 480;
    TS_MAX_Y    = 854;

    TS_BUTTON_W         = ((TS_MAX_X + 1) /TS_MAX_KEYS);
    TS_BUTTON_XBOUND    = (15);                     
    TS_MENUKEYL_X       = (TS_BUTTON_W * 0 + TS_BUTTON_XBOUND);
    TS_MENUKEYR_X       = (TS_BUTTON_W * 1 - TS_BUTTON_XBOUND);
    TS_HOMEKEYL_X       = (TS_BUTTON_W * 1 + TS_BUTTON_XBOUND);
    TS_HOMEKEYR_X       = (TS_BUTTON_W * 2 - TS_BUTTON_XBOUND);
    TS_BACKKEYL_X       = (TS_BUTTON_W * 2 + TS_BUTTON_XBOUND);
    TS_BACKKEYR_X       = (TS_BUTTON_W * 3 - TS_BUTTON_XBOUND);
    TS_SEARCHKEYL_X     = (TS_BUTTON_W * 3 + TS_BUTTON_XBOUND);
    TS_SEARCHKEYR_X     = (TS_BUTTON_W * 4 - TS_BUTTON_XBOUND);

//NJ-BSP-Simon-Touch
// 2011.10.28
//TBP-ITV-TNQ do while once in irq_work & check Calibration for palm problem    
    Project_Flag = 1;
    high_boundary = 11500;
    low_boundary = 6000;

    return ADD_DYN_I2C_DRIVER(hdl, &qt602240_i2c_driver_V2);
}
DECLARE_DYN_LOADER(DRIVER_TYPE_TOUCH, TOUCH_ATMEL_QT602240_V3, qt602240_init_v3, LEVEL6);

//TNQ
static int __init qt602240_init_v4(void *hdl )
{
    printk(KERN_INFO"[qt602240] %s\n",__func__);

    /*TNQ*/
    TS_MAX_X    = 480;
    TS_MAX_Y    = 320;

    TS_BUTTON_W         = ((TS_MAX_X + 1) /TS_MAX_KEYS);
    TS_BUTTON_XBOUND    = (15);                     
    TS_MENUKEYL_X       = (TS_BUTTON_W * 0 + TS_BUTTON_XBOUND);
    TS_MENUKEYR_X       = (TS_BUTTON_W * 1 - TS_BUTTON_XBOUND);
    TS_HOMEKEYL_X       = (TS_BUTTON_W * 1 + TS_BUTTON_XBOUND);
    TS_HOMEKEYR_X       = (TS_BUTTON_W * 2 - TS_BUTTON_XBOUND);
    TS_BACKKEYL_X       = (TS_BUTTON_W * 2 + TS_BUTTON_XBOUND);
    TS_BACKKEYR_X       = (TS_BUTTON_W * 3 - TS_BUTTON_XBOUND);
    TS_SEARCHKEYL_X     = (TS_BUTTON_W * 3 + TS_BUTTON_XBOUND);
    TS_SEARCHKEYR_X     = (TS_BUTTON_W * 4 - TS_BUTTON_XBOUND);

//NJ-BSP-Simon-Touch
// 2011.10.28
//TBP-ITV-TNQ do while once in irq_work & check Calibration for palm problem    
    Project_Flag = 2;
    high_boundary = 30000;
    low_boundary = 20000;

    return ADD_DYN_I2C_DRIVER(hdl, &qt602240_i2c_driver_V2);
}
DECLARE_DYN_LOADER(DRIVER_TYPE_TOUCH, TOUCH_ATMEL_QT602240_V4, qt602240_init_v4, LEVEL6);

static int __init qt602240_init_v5(void *hdl )
{
    printk(KERN_INFO"[qt602240] %s\n",__func__);

    /*ITV*/
    TS_MAX_X    = 480;
    TS_MAX_Y    = 844;
    
    gpio_camreset	= 126;
    gpio_pwr		= 117;
    gpio_tp_int_n   = 38;

    TS_MENUKEYL_X       = 50;
    TS_MENUKEYR_X       = 205;
    TS_HOMEKEYL_X       = 459;
    TS_HOMEKEYR_X       = 614;
    TS_BACKKEYL_X       = 818;
    TS_BACKKEYR_X       = 973;

//NJ-BSP-Simon-Touch
// 2011.10.28
//TBP-ITV-TNQ do while once in irq_work & check Calibration for palm problem    
    Project_Flag = 3;
    high_boundary = 30000;
    low_boundary = 20000;


    return ADD_DYN_I2C_DRIVER(hdl, &qt602240_i2c_driver_V2);
}
DECLARE_DYN_LOADER(DRIVER_TYPE_TOUCH, TOUCH_ATMEL_QT602240_V5, qt602240_init_v5, LEVEL6);
static int __init qt602240_init_v6(void *hdl )
{
    printk(KERN_INFO"[qt602240] %s\n",__func__);

    /*IP*/
    TS_MAX_X    = 480;
    TS_MAX_Y    = 800;
    
//    gpio_camreset       = 38;//NULL GPIO
//    gpio_pwr               = 40;//NULL GPIO

//NJ-BSP-Simon-Touch
// 2011.11.02
//IronPrime do not have reset pin
    goio_tp_rst_n        = 0;
    

    TS_MENUKEYL_X       = 50;
    TS_MENUKEYR_X       = 205;
    TS_HOMEKEYL_X       = 459;
    TS_HOMEKEYR_X       = 614;
    TS_BACKKEYL_X       = 818;
    TS_BACKKEYR_X       = 973;

//NJ-BSP-Simon-Touch
// 2011.10.28
//TBP-ITV-TNQ do while once in irq_work & check Calibration for palm problem    
    Project_Flag = 4;

    return ADD_DYN_I2C_DRIVER(hdl, &qt602240_i2c_driver_V2);
}
DECLARE_DYN_LOADER(DRIVER_TYPE_TOUCH, TOUCH_ATMEL_QT602240_V6, qt602240_init_v6, LEVEL6);

static void __exit qt602240_exit( void )
{
    TCH_DBG(TOUCH_DEBUG_COMMON, "Done.");
    i2c_del_driver(&qt602240_i2c_driver);
}

#if 0
module_init(qt602240_init);
#endif

module_exit(qt602240_exit);

MODULE_DESCRIPTION("ATMEL QT602240 Touchscreen Driver");
MODULE_AUTHOR("FIH Div2 Dep5 Peripheral Team");
MODULE_VERSION("0.2.4");
MODULE_LICENSE("GPL");
