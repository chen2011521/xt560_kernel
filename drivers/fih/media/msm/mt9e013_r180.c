/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include "mt9e013_r180.h"
#include <fih/dynloader.h>
/*=============================================================
	SENSOR REGISTER DEFINES
==============================================================*/
#define REG_GROUPED_PARAMETER_HOLD		0x0104
#define GROUPED_PARAMETER_HOLD_OFF		0x00
#define GROUPED_PARAMETER_HOLD			0x01
/* Integration Time */
#define REG_COARSE_INTEGRATION_TIME		0x3012
/* Gain */
#define REG_GLOBAL_GAIN	0x305E
/* PLL registers */
#define REG_FRAME_LENGTH_LINES		0x0340
/* Test Pattern */
#define REG_TEST_PATTERN_MODE			0x0601
#define REG_VCM_NEW_CODE			0x30F2

/*============================================================================
							 TYPE DECLARATIONS
============================================================================*/

/* 16bit address - 8 bit context register structure */
#define Q8	0x00000100
#define Q10	0x00000400
#define MT9E013_R180_MASTER_CLK_RATE 24000000

/* AF Total steps parameters */
#define MT9E013_R180_TOTAL_STEPS_NEAR_TO_FAR    32

uint16_t mt9e013_r180_step_position_table[MT9E013_R180_TOTAL_STEPS_NEAR_TO_FAR+1];
uint16_t mt9e013_r180_nl_region_boundary1;
uint16_t mt9e013_r180_nl_region_code_per_step1;
uint16_t mt9e013_r180_l_region_code_per_step = 4;
uint16_t mt9e013_r180_damping_threshold = 10;
uint16_t mt9e013_r180_sw_damping_time_wait = 1;

struct mt9e013_r180_work_t {
	struct work_struct work;
};

static struct mt9e013_r180_work_t *mt9e013_r180_sensorw;
static struct i2c_client *mt9e013_r180_client;

struct mt9e013_r180_ctrl_t {
	const struct  msm_camera_sensor_info *sensordata;

	uint32_t sensormode;
	uint32_t fps_divider;/* init to 1 * 0x00000400 */
	uint32_t pict_fps_divider;/* init to 1 * 0x00000400 */
	uint16_t fps;

	uint16_t curr_lens_pos;
	uint16_t curr_step_pos;
	uint16_t my_reg_gain;
	uint32_t my_reg_line_count;
	uint16_t total_lines_per_frame;

	enum mt9e013_r180_resolution_t prev_res;
	enum mt9e013_r180_resolution_t pict_res;
	enum mt9e013_r180_resolution_t curr_res;
	enum mt9e013_r180_test_mode_t  set_test;
};


static bool CSI_CONFIG;
static struct mt9e013_r180_ctrl_t *mt9e013_r180_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(mt9e013_r180_wait_queue);
DEFINE_MUTEX(mt9e013_r180_mut);

static int cam_debug_init(void);
static struct dentry *debugfs_base;
/*=============================================================*/

static int mt9e013_r180_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
		{
			.addr  = saddr,
			.flags = 0,
			.len   = 2,
			.buf   = rxdata,
		},
		{
			.addr  = saddr,
			.flags = I2C_M_RD,
			.len   = 2,
			.buf   = rxdata,
		},
	};
	if (i2c_transfer(mt9e013_r180_client->adapter, msgs, 2) < 0) {
		CDBG("mt9e013_r180_i2c_rxdata faild 0x%x\n", saddr);
		return -EIO;
	}
	return 0;
}

static int32_t mt9e013_r180_i2c_txdata(unsigned short saddr,
				unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		 },
	};
	if (i2c_transfer(mt9e013_r180_client->adapter, msg, 1) < 0) {
		CDBG("mt9e013_r180_i2c_txdata faild 0x%x\n", saddr);
		return -EIO;
	}

	return 0;
}

static int32_t mt9e013_r180_i2c_read(unsigned short raddr,
	unsigned short *rdata, int rlen)
{
	int32_t rc = 0;
	unsigned char buf[2];
	if (!rdata)
		return -EIO;
	memset(buf, 0, sizeof(buf));
	buf[0] = (raddr & 0xFF00) >> 8;
	buf[1] = (raddr & 0x00FF);
	rc = mt9e013_r180_i2c_rxdata(mt9e013_r180_client->addr<<1, buf, rlen);
	if (rc < 0) {
		CDBG("mt9e013_r180_i2c_read 0x%x failed!\n", raddr);
		return rc;
	}
	*rdata = (rlen == 2 ? buf[0] << 8 | buf[1] : buf[0]);
	CDBG("mt9e013_r180_i2c_read 0x%x val = 0x%x!\n", raddr, *rdata);
	return rc;
}

static int32_t mt9e013_r180_i2c_write_w_sensor(unsigned short waddr, uint16_t wdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[4];
	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = (wdata & 0xFF00) >> 8;
	buf[3] = (wdata & 0x00FF);
	CDBG("i2c_write_b addr = 0x%x, val = 0x%x\n", waddr, wdata);
	rc = mt9e013_r180_i2c_txdata(mt9e013_r180_client->addr<<1, buf, 4);
	if (rc < 0) {
		CDBG("i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
			waddr, wdata);
	}
	return rc;
}

static int32_t mt9e013_r180_i2c_write_b_sensor(unsigned short waddr, uint8_t bdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[3];
	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00) >> 8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = bdata;
	CDBG("i2c_write_b addr = 0x%x, val = 0x%x\n", waddr, bdata);
	rc = mt9e013_r180_i2c_txdata(mt9e013_r180_client->addr<<1, buf, 3);
	if (rc < 0) {
		CDBG("i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
			waddr, bdata);
	}
	return rc;
}

static int32_t mt9e013_r180_i2c_write_w_table(struct mt9e013_r180_i2c_reg_conf const
					 *reg_conf_tbl, int num)
{
	int i;
	int32_t rc = -EIO;
	for (i = 0; i < num; i++) {
		rc = mt9e013_r180_i2c_write_w_sensor(reg_conf_tbl->waddr,
			reg_conf_tbl->wdata);
		if (rc < 0)
			break;
		reg_conf_tbl++;
	}
	return rc;
}

static void mt9e013_r180_group_hold_on(void)
{
	mt9e013_r180_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD,
						GROUPED_PARAMETER_HOLD);
}

static void mt9e013_r180_group_hold_off(void)
{
	mt9e013_r180_i2c_write_b_sensor(REG_GROUPED_PARAMETER_HOLD,
						GROUPED_PARAMETER_HOLD_OFF);
}

static void mt9e013_r180_start_stream(void)
{
	mt9e013_r180_i2c_write_w_sensor(0x301A, 0x8250);
	mt9e013_r180_i2c_write_w_sensor(0x301A, 0x8650);
	mt9e013_r180_i2c_write_w_sensor(0x301A, 0x8658);
	mt9e013_r180_i2c_write_b_sensor(0x0104, 0x00);
	mt9e013_r180_i2c_write_w_sensor(0x301A, 0x065C);
}

static void mt9e013_r180_stop_stream(void)
{
	mt9e013_r180_i2c_write_w_sensor(0x301A, 0x0058);
	mt9e013_r180_i2c_write_w_sensor(0x301A, 0x0050);
	mt9e013_r180_i2c_write_b_sensor(0x0104, 0x01);
}

static void mt9e013_r180_get_pict_fps(uint16_t fps, uint16_t *pfps)
{
	/* input fps is preview fps in Q8 format */
	uint32_t divider, d1, d2;

	d1 = mt9e013_r180_regs.reg_prev[E013_FRAME_LENGTH_LINES].wdata
		* 0x00000400/
		mt9e013_r180_regs.reg_snap[E013_FRAME_LENGTH_LINES].wdata;
	d2 = mt9e013_r180_regs.reg_prev[E013_LINE_LENGTH_PCK].wdata
		* 0x00000400/
		mt9e013_r180_regs.reg_snap[E013_LINE_LENGTH_PCK].wdata;
	divider = d1 * d2 / 0x400;

    /*Verify PCLK settings and frame sizes.*/
	*pfps = (uint16_t) (fps * divider / 0x400);
	/* 2 is the ratio of no.of snapshot channels
	to number of preview channels */
}

static uint16_t mt9e013_r180_get_prev_lines_pf(void)
{
	if (mt9e013_r180_ctrl->prev_res == QTR_SIZE)
		return mt9e013_r180_regs.reg_prev[E013_FRAME_LENGTH_LINES].wdata;
	else if (mt9e013_r180_ctrl->prev_res == FULL_SIZE)
		return mt9e013_r180_regs.reg_snap[E013_FRAME_LENGTH_LINES].wdata;
	else if (mt9e013_r180_ctrl->prev_res == HFR_60FPS)
		return mt9e013_r180_regs.reg_60fps[E013_FRAME_LENGTH_LINES].wdata;
	else if (mt9e013_r180_ctrl->prev_res == HFR_90FPS)
		return mt9e013_r180_regs.reg_120fps[E013_FRAME_LENGTH_LINES].wdata;
	else
		return mt9e013_r180_regs.reg_120fps[E013_FRAME_LENGTH_LINES].wdata;
}

static uint16_t mt9e013_r180_get_prev_pixels_pl(void)
{
	if (mt9e013_r180_ctrl->prev_res == QTR_SIZE)
		return mt9e013_r180_regs.reg_prev[E013_LINE_LENGTH_PCK].wdata;
	else if (mt9e013_r180_ctrl->prev_res == FULL_SIZE)
		return mt9e013_r180_regs.reg_snap[E013_LINE_LENGTH_PCK].wdata;
	else if (mt9e013_r180_ctrl->prev_res == HFR_60FPS)
		return mt9e013_r180_regs.reg_60fps[E013_LINE_LENGTH_PCK].wdata;
	else if (mt9e013_r180_ctrl->prev_res == HFR_90FPS)
		return mt9e013_r180_regs.reg_120fps[E013_LINE_LENGTH_PCK].wdata;
	else
		return mt9e013_r180_regs.reg_120fps[E013_LINE_LENGTH_PCK].wdata;
}

static uint16_t mt9e013_r180_get_pict_lines_pf(void)
{
	if (mt9e013_r180_ctrl->pict_res == QTR_SIZE)
		return mt9e013_r180_regs.reg_prev[E013_FRAME_LENGTH_LINES].wdata;
	else if (mt9e013_r180_ctrl->pict_res == FULL_SIZE)
		return mt9e013_r180_regs.reg_snap[E013_FRAME_LENGTH_LINES].wdata;
	else if (mt9e013_r180_ctrl->pict_res == HFR_60FPS)
		return mt9e013_r180_regs.reg_60fps[E013_FRAME_LENGTH_LINES].wdata;
	else if (mt9e013_r180_ctrl->pict_res == HFR_90FPS)
		return mt9e013_r180_regs.reg_120fps[E013_FRAME_LENGTH_LINES].wdata;
	else
		return mt9e013_r180_regs.reg_120fps[E013_FRAME_LENGTH_LINES].wdata;
}

static uint16_t mt9e013_r180_get_pict_pixels_pl(void)
{
	if (mt9e013_r180_ctrl->pict_res == QTR_SIZE)
		return mt9e013_r180_regs.reg_prev[E013_LINE_LENGTH_PCK].wdata;
	else if (mt9e013_r180_ctrl->pict_res == FULL_SIZE)
		return mt9e013_r180_regs.reg_snap[E013_LINE_LENGTH_PCK].wdata;
	else if (mt9e013_r180_ctrl->pict_res == HFR_60FPS)
		return mt9e013_r180_regs.reg_60fps[E013_LINE_LENGTH_PCK].wdata;
	else if (mt9e013_r180_ctrl->pict_res == HFR_90FPS)
		return mt9e013_r180_regs.reg_120fps[E013_LINE_LENGTH_PCK].wdata;
	else
		return mt9e013_r180_regs.reg_120fps[E013_LINE_LENGTH_PCK].wdata;
}

static uint32_t mt9e013_r180_get_pict_max_exp_lc(void)
{
	if (mt9e013_r180_ctrl->pict_res == QTR_SIZE)
		return mt9e013_r180_regs.reg_prev[E013_FRAME_LENGTH_LINES].wdata
			* 24;
	else if (mt9e013_r180_ctrl->pict_res == FULL_SIZE)
		return mt9e013_r180_regs.reg_snap[E013_FRAME_LENGTH_LINES].wdata
			* 24;
	else if (mt9e013_r180_ctrl->pict_res == HFR_60FPS)
		return mt9e013_r180_regs.reg_60fps[E013_FRAME_LENGTH_LINES].wdata
			* 24;
	else if (mt9e013_r180_ctrl->pict_res == HFR_90FPS)
		return mt9e013_r180_regs.reg_120fps[E013_FRAME_LENGTH_LINES].wdata
			* 24;
	else
		return mt9e013_r180_regs.reg_120fps[E013_FRAME_LENGTH_LINES].wdata
			* 24;
}

static int32_t mt9e013_r180_set_fps(struct fps_cfg   *fps)
{
	uint16_t total_lines_per_frame;
	int32_t rc = 0;
	if (mt9e013_r180_ctrl->curr_res == QTR_SIZE)
		total_lines_per_frame =
		mt9e013_r180_regs.reg_prev[E013_FRAME_LENGTH_LINES].wdata;
	else if (mt9e013_r180_ctrl->curr_res == FULL_SIZE)
		total_lines_per_frame =
		mt9e013_r180_regs.reg_snap[E013_FRAME_LENGTH_LINES].wdata;
	else if (mt9e013_r180_ctrl->curr_res == HFR_60FPS)
		total_lines_per_frame =
		mt9e013_r180_regs.reg_60fps[E013_FRAME_LENGTH_LINES].wdata;
	else if (mt9e013_r180_ctrl->curr_res == HFR_90FPS)
		total_lines_per_frame =
		mt9e013_r180_regs.reg_120fps[E013_FRAME_LENGTH_LINES].wdata;
	else
		total_lines_per_frame =
		mt9e013_r180_regs.reg_120fps[E013_FRAME_LENGTH_LINES].wdata;

	mt9e013_r180_ctrl->fps_divider = fps->fps_div;
	mt9e013_r180_ctrl->pict_fps_divider = fps->pict_fps_div;

	if (mt9e013_r180_ctrl->curr_res == FULL_SIZE) {
		total_lines_per_frame = (uint16_t)
		(total_lines_per_frame * mt9e013_r180_ctrl->pict_fps_divider/0x400);
	} else {
		total_lines_per_frame = (uint16_t)
		(total_lines_per_frame * mt9e013_r180_ctrl->fps_divider/0x400);
	}

	mt9e013_r180_group_hold_on();
	rc = mt9e013_r180_i2c_write_w_sensor(REG_FRAME_LENGTH_LINES,
							total_lines_per_frame);
	mt9e013_r180_group_hold_off();
	return rc;
}

static int32_t mt9e013_r180_write_exp_gain(uint16_t gain, uint32_t line)
{
	uint16_t max_legal_gain = 0xE7F;
	int32_t rc = 0;
	if (gain > max_legal_gain) {
		CDBG("Max legal gain Line:%d\n", __LINE__);
		gain = max_legal_gain;
	}

	if (mt9e013_r180_ctrl->curr_res != FULL_SIZE) {
		mt9e013_r180_ctrl->my_reg_gain = gain;
		mt9e013_r180_ctrl->my_reg_line_count = (uint16_t) line;
		line = (uint32_t) (line * mt9e013_r180_ctrl->fps_divider /
						   0x00000400);
	} else {
		line = (uint32_t) (line * mt9e013_r180_ctrl->pict_fps_divider /
						   0x00000400);
	}

	gain |= 0x1000;

	mt9e013_r180_group_hold_on();
	rc = mt9e013_r180_i2c_write_w_sensor(REG_GLOBAL_GAIN, gain);
	rc = mt9e013_r180_i2c_write_w_sensor(REG_COARSE_INTEGRATION_TIME, line);
	mt9e013_r180_group_hold_off();
	return rc;
}

static int32_t mt9e013_r180_set_pict_exp_gain(uint16_t gain, uint32_t line)
{
	int32_t rc = 0;
	rc = mt9e013_r180_write_exp_gain(gain, line);
	mt9e013_r180_i2c_write_w_sensor(0x301A, 0x065C|0x2);
	return rc;
}

#define DIV_CEIL(x, y) (x/y + (x%y) ? 1 : 0)

static int32_t mt9e013_r180_move_focus(int direction,
	int32_t num_steps)
{
	int16_t step_direction, dest_lens_position, dest_step_position;
	int16_t target_dist, small_step, next_lens_position;
	if (direction == MOVE_NEAR)
		step_direction = 1;
	else
		step_direction = -1;

	dest_step_position = mt9e013_r180_ctrl->curr_step_pos
						+ (step_direction * num_steps);

	if (dest_step_position < 0)
		dest_step_position = 0;
	else if (dest_step_position > MT9E013_R180_TOTAL_STEPS_NEAR_TO_FAR)
		dest_step_position = MT9E013_R180_TOTAL_STEPS_NEAR_TO_FAR;

	if (dest_step_position == mt9e013_r180_ctrl->curr_step_pos)
		return 0;

	dest_lens_position = mt9e013_r180_step_position_table[dest_step_position];
	target_dist = step_direction *
		(dest_lens_position - mt9e013_r180_ctrl->curr_lens_pos);

	if (step_direction < 0 && (target_dist >=
		mt9e013_r180_step_position_table[mt9e013_r180_damping_threshold])) {
		small_step = DIV_CEIL(target_dist, 10);
		mt9e013_r180_sw_damping_time_wait = 10;
	} else {
		small_step = DIV_CEIL(target_dist, 4);
		mt9e013_r180_sw_damping_time_wait = 4;
	}

	for (next_lens_position = mt9e013_r180_ctrl->curr_lens_pos
		+ (step_direction * small_step);
		(step_direction * next_lens_position) <=
		(step_direction * dest_lens_position);
		next_lens_position += (step_direction * small_step)) {
		mt9e013_r180_i2c_write_w_sensor(REG_VCM_NEW_CODE,
		next_lens_position);
		mt9e013_r180_ctrl->curr_lens_pos = next_lens_position;
		usleep(mt9e013_r180_sw_damping_time_wait*50);
	}

	if (mt9e013_r180_ctrl->curr_lens_pos != dest_lens_position) {
		mt9e013_r180_i2c_write_w_sensor(REG_VCM_NEW_CODE,
		dest_lens_position);
		usleep(mt9e013_r180_sw_damping_time_wait*50);
	}
	mt9e013_r180_ctrl->curr_lens_pos = dest_lens_position;
	mt9e013_r180_ctrl->curr_step_pos = dest_step_position;
	return 0;
}

static int32_t mt9e013_r180_set_default_focus(uint8_t af_step)
{
	int32_t rc = 0;
	if (mt9e013_r180_ctrl->curr_step_pos != 0) {
		rc = mt9e013_r180_move_focus(MOVE_FAR,
		mt9e013_r180_ctrl->curr_step_pos);
	} else {
		mt9e013_r180_i2c_write_w_sensor(REG_VCM_NEW_CODE, 0x00);
	}

	mt9e013_r180_ctrl->curr_lens_pos = 0;
	mt9e013_r180_ctrl->curr_step_pos = 0;

	return rc;
}

static void mt9e013_r180_init_focus(void)
{
	uint8_t i;
	mt9e013_r180_step_position_table[0] = 0;
	for (i = 1; i <= MT9E013_R180_TOTAL_STEPS_NEAR_TO_FAR; i++) {
		if (i <= mt9e013_r180_nl_region_boundary1) {
			mt9e013_r180_step_position_table[i] =
				mt9e013_r180_step_position_table[i-1]
				+ mt9e013_r180_nl_region_code_per_step1;
		} else {
			mt9e013_r180_step_position_table[i] =
				mt9e013_r180_step_position_table[i-1]
				+ mt9e013_r180_l_region_code_per_step;
		}

		if (mt9e013_r180_step_position_table[i] > 255)
			mt9e013_r180_step_position_table[i] = 255;
	}
}

//vince, OTP +++
//static ssize_t mt9e013_r180_otp_check(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
static int mt9e013_r180_otp_check(void)
{
	uint16_t counter = 0;
	unsigned short value=0, checksum;
	uint16_t R0x301A = 0, R0x3134 = 0, R0x304C = 0, R0x304A = 0;
	uint16_t page_reg[4] = {0x3000, 0x3100, 0x3200, 0x3300};
    	//unsigned type =0;
    	//int reg;
    	int i;

	CDBG("%s: %d\n", __func__, __LINE__);

	//Initialize OTPM for reading	
	mt9e013_r180_i2c_read(0x301A, &R0x301A, 2); //RESET_REGISTER_REG_ED_EN
	mt9e013_r180_i2c_read(0x3134, &R0x3134, 2);
	mt9e013_r180_i2c_read(0x304C, &R0x304C, 2);
	mt9e013_r180_i2c_read(0x304A, &R0x304A, 2);
	
	//sscanf(buf, "%x\n", &type);
	//sscanf(buf, "%x\n", &type);
	for(i=3; i>=0; i--)//from 4th PAGE
	{
		printk("OTP read page %d\n", i);
		mt9e013_r180_i2c_write_w_sensor(0x301A, R0x301A | 0x0020);
		mt9e013_r180_i2c_write_w_sensor(0x301A, 0x0610); //disable streaming
		mt9e013_r180_i2c_write_w_sensor(0x3134, 0xCD95); //timing paramerters for otp read
		//mt9e013_r180_i2c_write_w_sensor(0x304C, 0x3000); //record type 0x30
		mt9e013_r180_i2c_write_w_sensor(0x304C, page_reg[i]); //record type 0x30
		mt9e013_r180_i2c_write_w_sensor(0x304A, 0x0010); //only read single record at a time
		counter=0;
		while (value != 0x60 && counter < 10) { //auto read success if return 0x60
			mt9e013_r180_i2c_read(0x304A, &value, 2);
            printk("read value=%x\n",value);
			counter++;
		}
		mt9e013_r180_i2c_read(0x38D4, &value, 2);
		if((value&0xff)==0xff)
			break;
		else
			printk("%s: R0x38D4=0x%x \n",__func__,value);
			
	}

	mt9e013_r180_i2c_read(0x38D4, &checksum, 2);
	
	printk("checksum 0x38D4=0x%x\n",checksum);


    	return checksum>0?0:-1;
};

static int read_LSC(void)
{

	unsigned short value,i=0;
	uint16_t rc;
	printk("++++++read_LSC+++++++\n");
	for(i=0;i<106;i++)
	{
		rc = mt9e013_r180_i2c_read(mt9e013_r180_regs.reg_lsc[i].waddr, &value, 2);	
		printk("R0x%x = %x\n",mt9e013_r180_regs.reg_lsc[i].waddr,value);	
	}
	printk("------read_LSC-------\n");
	return rc;
}

static int read_OTP_value(void)
{

	unsigned short value,i=0;
	uint16_t rc=0;
	printk("++++++read_OTP_value+++++++\n");
	for(i=0x3800;i<=0x38D6;i+=2)
	{
		rc=mt9e013_r180_i2c_read(i, &value, 2);
		printk("R0x%x = 0x%04x\n",i,value);
	}
	printk("------read_OTP_value-------\n");
	return rc;
}

static int write_OTP_to_LSC(void)
{
	unsigned short value,i,j=0;
	uint16_t rc=0;
	for(i=0x3800;i<0x38D4;i+=2)
	{
		mt9e013_r180_i2c_read(i, &value, 2); 
		printk("R0x%x = 0x%04x\n",i,value);
        mt9e013_r180_regs.reg_lsc[j].wdata = value;
        j++;
	}
	rc = mt9e013_r180_i2c_write_w_table(mt9e013_r180_regs.reg_lsc,mt9e013_r180_regs.reg_lsc_size);
	//enable OTP
    mt9e013_r180_i2c_write_w_sensor(0x3780,0x8000);

	return rc;
}

static ssize_t mt9e013_r180_LSC_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	read_LSC();
	return 0;
}
DEVICE_ATTR(mt9e013_r180_lsc_table, 0664, mt9e013_r180_LSC_read, NULL);
static ssize_t mt9e013_r180_OTP_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	read_OTP_value();
	return 0;
}
DEVICE_ATTR(mt9e013_r180_otp_table, 0664, mt9e013_r180_OTP_read, NULL);
static int mt9e013_r180_create_attributes(struct i2c_client *client)
{
	int rc;

	rc = device_create_file(&client->dev, &dev_attr_mt9e013_r180_lsc_table);
	rc = device_create_file(&client->dev, &dev_attr_mt9e013_r180_otp_table);

	if (rc < 0) {
		dev_err(&client->dev, "%s: Create mt9e013_r180 attribute  failed!! <%d>", __func__, rc);	
		return rc; 
	}

	
	return rc;	
}
//vince, OTP ---

static int32_t mt9e013_r180_test(enum mt9e013_r180_test_mode_t mo)
{
	int32_t rc = 0;
	if (mo == TEST_OFF)
		return rc;
	else {
		/* REG_0x30D8[4] is TESBYPEN: 0: Normal Operation,
		1: Bypass Signal Processing
		REG_0x30D8[5] is EBDMASK: 0:
		Output Embedded data, 1: No output embedded data */
		if (mt9e013_r180_i2c_write_b_sensor(REG_TEST_PATTERN_MODE,
			(uint8_t) mo) < 0) {
			return rc;
		}
	}
	return rc;
}

static int32_t mt9e013_r180_sensor_setting(int update_type, int rt)
{

	int32_t rc = 0;
	struct msm_camera_csi_params mt9e013_r180_csi_params;
	uint8_t stored_af_step = 0;
	CDBG("sensor_settings\n");
	stored_af_step = mt9e013_r180_ctrl->curr_step_pos;
	mt9e013_r180_set_default_focus(0);
	mt9e013_r180_stop_stream();
	msleep(15);
	if (update_type == REG_INIT) {
		mt9e013_r180_i2c_write_w_table(mt9e013_r180_regs.reg_mipi,
			mt9e013_r180_regs.reg_mipi_size);
		mt9e013_r180_i2c_write_w_table(mt9e013_r180_regs.rec_settings,
			mt9e013_r180_regs.rec_size);
		if(!mt9e013_r180_otp_check())
		{
			printk("mt9e013_r180: read OTP completed\n");
			if(!write_OTP_to_LSC())
				printk("mt9e013_r180: write OTP to LSC completed\n");
			else
				printk("mt9e013_r180: write OTP to LSC fail\n");
		}
		else
			printk("mt9e013_r180: read OTP fail\n");
		cam_debug_init();
		CSI_CONFIG = 0;
	} else if (update_type == UPDATE_PERIODIC) {
		if (rt == QTR_SIZE) {
			mt9e013_r180_i2c_write_w_table(mt9e013_r180_regs.reg_pll,
				mt9e013_r180_regs.reg_pll_size);
			mt9e013_r180_i2c_write_w_table(mt9e013_r180_regs.reg_prev,
				mt9e013_r180_regs.reg_prev_size);
		} else if (rt == FULL_SIZE) {
			mt9e013_r180_i2c_write_w_table(mt9e013_r180_regs.reg_pll,
				mt9e013_r180_regs.reg_pll_size);
			mt9e013_r180_i2c_write_w_table(mt9e013_r180_regs.reg_snap,
				mt9e013_r180_regs.reg_snap_size);
		} else if (rt == HFR_60FPS) {
			mt9e013_r180_i2c_write_w_table(mt9e013_r180_regs.reg_pll_120fps,
				mt9e013_r180_regs.reg_pll_120fps_size);
			mt9e013_r180_i2c_write_w_sensor(0x0306, 0x0029);
			mt9e013_r180_i2c_write_w_table(mt9e013_r180_regs.reg_120fps,
				mt9e013_r180_regs.reg_120fps_size);
		} else if (rt == HFR_90FPS) {
			mt9e013_r180_i2c_write_w_table(mt9e013_r180_regs.reg_pll_120fps,
				mt9e013_r180_regs.reg_pll_120fps_size);
			mt9e013_r180_i2c_write_w_sensor(0x0306, 0x003D);
			mt9e013_r180_i2c_write_w_table(mt9e013_r180_regs.reg_120fps,
				mt9e013_r180_regs.reg_120fps_size);
		} else if (rt == HFR_120FPS) {
			msm_camio_vfe_clk_rate_set(266667000);
			mt9e013_r180_i2c_write_w_table(mt9e013_r180_regs.reg_pll_120fps,
				mt9e013_r180_regs.reg_pll_120fps_size);
			mt9e013_r180_i2c_write_w_table(mt9e013_r180_regs.reg_120fps,
				mt9e013_r180_regs.reg_120fps_size);
		}
		if (!CSI_CONFIG) {
			msm_camio_vfe_clk_rate_set(192000000);
			mt9e013_r180_csi_params.data_format = CSI_10BIT;
			mt9e013_r180_csi_params.lane_cnt = 2;
			mt9e013_r180_csi_params.lane_assign = 0xe4;
			mt9e013_r180_csi_params.dpcm_scheme = 0;
			mt9e013_r180_csi_params.settle_cnt = 0x18;
			rc = msm_camio_csi_config(&mt9e013_r180_csi_params);
			msleep(10);
			CSI_CONFIG = 1;
		}
		mt9e013_r180_start_stream();
		mt9e013_r180_move_focus(MOVE_NEAR, stored_af_step);
		
	}
	return rc;
}

static int32_t mt9e013_r180_video_config(int mode)
{

	int32_t rc = 0;

	CDBG("video config\n");
	/* change sensor resolution if needed */
	if (mt9e013_r180_sensor_setting(UPDATE_PERIODIC,
			mt9e013_r180_ctrl->prev_res) < 0)
		return rc;
	if (mt9e013_r180_ctrl->set_test) {
		if (mt9e013_r180_test(mt9e013_r180_ctrl->set_test) < 0)
			return  rc;
	}
    // Jos Add here 
    //Add this function to reset the AF while start preview. 
//  mt9e013_r180_set_default_focus(0);

	mt9e013_r180_ctrl->curr_res = mt9e013_r180_ctrl->prev_res;
	mt9e013_r180_ctrl->sensormode = mode;
	return rc;
}

static int32_t mt9e013_r180_snapshot_config(int mode)
{
	int32_t rc = 0;
	/*change sensor resolution if needed */
	if (mt9e013_r180_ctrl->curr_res != mt9e013_r180_ctrl->pict_res) {
		if (mt9e013_r180_sensor_setting(UPDATE_PERIODIC,
				mt9e013_r180_ctrl->pict_res) < 0)
			return rc;
	}

	mt9e013_r180_ctrl->curr_res = mt9e013_r180_ctrl->pict_res;
	mt9e013_r180_ctrl->sensormode = mode;
	return rc;
} /*end of mt9e013_r180_snapshot_config*/

static int32_t mt9e013_r180_raw_snapshot_config(int mode)
{
	int32_t rc = 0;
	/* change sensor resolution if needed */
	if (mt9e013_r180_ctrl->curr_res != mt9e013_r180_ctrl->pict_res) {
		if (mt9e013_r180_sensor_setting(UPDATE_PERIODIC,
				mt9e013_r180_ctrl->pict_res) < 0)
			return rc;
	}

	mt9e013_r180_ctrl->curr_res = mt9e013_r180_ctrl->pict_res;
	mt9e013_r180_ctrl->sensormode = mode;
	return rc;
} /*end of mt9e013_r180_raw_snapshot_config*/

static int32_t mt9e013_r180_set_sensor_mode(int mode,
	int res)
{
	int32_t rc = 0;
	switch (mode) {
	case SENSOR_PREVIEW_MODE:
	case SENSOR_HFR_60FPS_MODE:
	case SENSOR_HFR_90FPS_MODE:
	case SENSOR_HFR_120FPS_MODE:
		mt9e013_r180_ctrl->prev_res = res;
		rc = mt9e013_r180_video_config(mode);
		rc = mt9e013_r180_set_default_focus(0);
		if(!rc)
			printk("set mt9e013_r180_set_default_focus after start preview!!\n");
		else
			printk("set mt9e013_r180_set_default_focus fail!!\n");
		break;
	case SENSOR_SNAPSHOT_MODE:
		mt9e013_r180_ctrl->pict_res = res;
		rc = mt9e013_r180_snapshot_config(mode);
		break;
	case SENSOR_RAW_SNAPSHOT_MODE:
		mt9e013_r180_ctrl->pict_res = res;
		rc = mt9e013_r180_raw_snapshot_config(mode);
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}

static int32_t mt9e013_r180_power_down(void)
{
	return 0;
}

static int mt9e013_r180_probe_init_done(const struct msm_camera_sensor_info *data)
{
	CDBG("probe done\n");
	gpio_free(data->sensor_reset);
	return 0;
}

static int mt9e013_r180_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;
	uint16_t chipid = 0;
	int count=0;
	CDBG("%s: %d\n", __func__, __LINE__);
	rc = gpio_request(data->sensor_reset, "mt9e013_r180");
	CDBG(" mt9e013_r180_probe_init_sensor\n");
	if (!rc) 
	{
		CDBG("sensor_reset HIGH\n");
		gpio_direction_output(data->sensor_reset, 1);
		msleep(10);
		//vince test
		//msleep(100);
		#if 1
		/* enable mclk first */
		CDBG("MCLK enable\n");
		msm_camio_clk_rate_set(MT9E013_R180_MASTER_CLK_RATE);
		msleep(10);
		#endif
	
		CDBG("sensor_reset LOW\n");
		gpio_direction_output(data->sensor_reset, 0);
		msleep(10);
		
		CDBG("sensor_reset HIGH\n");
		gpio_set_value_cansleep(data->sensor_reset, 1);
		msleep(10);
		
		#if 0
		CDBG("MCLK enable\n");
		msm_camio_clk_rate_set(MT9E013_R180_MASTER_CLK_RATE);
		msleep(10);
		#endif
				
	} else {
		goto init_probe_done;
	}

	CDBG(" mt9e013_r180_probe_init_sensor is called\n");
	rc = mt9e013_r180_i2c_read(0x0000, &chipid, 2);
	CDBG("ID: %d\n", chipid);
	
#if 0 // Elit
        while(count < 40)
        {
		msleep(10);
                rc = mt9e013_r180_i2c_read(0x0000, &chipid, 2);
                count++;
                printk("mt9e013_r180_probe_init_sensor: retry %d, chipid=0x%x\n",count,chipid);                
        }
        if (chipid != 0x4B00)
           goto init_probe_fail;
#endif
	
#if 1
        while(chipid != 0x4B00 && count<10)
        {
                msleep(10);
                rc = mt9e013_r180_i2c_read(0x0000, &chipid, 2);
                count++;
                printk("mt9e013_r180_probe_init_sensor: retry %d, chipid=0x%x\n",count,chipid);
        }
        if (count > 9)
           goto init_probe_fail;
#endif

#if 0	
	/* 4. Compare sensor ID to MT9E013_R180 ID: */
	if (chipid != 0x4B00) {
		rc = -ENODEV;
		CDBG("mt9e013_r180_probe_init_sensor fail chip id doesnot match\n");
		goto init_probe_fail;
	}
#endif	

	mt9e013_r180_ctrl = kzalloc(sizeof(struct mt9e013_r180_ctrl_t), GFP_KERNEL);
	if (!mt9e013_r180_ctrl) {
		CDBG("mt9e013_r180_init failed!\n");
		rc = -ENOMEM;
	}
	mt9e013_r180_ctrl->fps_divider = 1 * 0x00000400;
	mt9e013_r180_ctrl->pict_fps_divider = 1 * 0x00000400;
	mt9e013_r180_ctrl->set_test = TEST_OFF;
	mt9e013_r180_ctrl->prev_res = QTR_SIZE;
	mt9e013_r180_ctrl->pict_res = FULL_SIZE;

	if (data)
		mt9e013_r180_ctrl->sensordata = data;

	goto init_probe_done;
init_probe_fail:
	CDBG(" mt9e013_r180_probe_init_sensor fails\n");
	gpio_set_value_cansleep(data->sensor_reset, 0);
	mt9e013_r180_probe_init_done(data);
init_probe_done:
	CDBG(" mt9e013_r180_probe_init_sensor finishes\n");
	return rc;
}
/* camsensor_mt9e013_r180_reset */

int mt9e013_r180_sensor_open_init(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;

	CDBG("%s: %d\n", __func__, __LINE__);
	CDBG("Calling mt9e013_r180_sensor_open_init\n");

	mt9e013_r180_ctrl = kzalloc(sizeof(struct mt9e013_r180_ctrl_t), GFP_KERNEL);
	if (!mt9e013_r180_ctrl) {
		CDBG("mt9e013_r180_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}
	mt9e013_r180_ctrl->fps_divider = 1 * 0x00000400;
	mt9e013_r180_ctrl->pict_fps_divider = 1 * 0x00000400;
	mt9e013_r180_ctrl->set_test = TEST_OFF;
	mt9e013_r180_ctrl->prev_res = QTR_SIZE;
	mt9e013_r180_ctrl->pict_res = FULL_SIZE;

	if (data)
		mt9e013_r180_ctrl->sensordata = data;
	if (rc < 0) {
		CDBG("Calling mt9e013_r180_sensor_open_init fail1\n");
		return rc;
	}
	CDBG("%s: %d\n", __func__, __LINE__);
	/* enable mclk first */
	//msm_camio_clk_rate_set(MT9E013_R180_MASTER_CLK_RATE);
	rc = mt9e013_r180_probe_init_sensor(data);
	if (rc < 0)
		goto init_fail;

    /* FIH, Ming, AF { */
	/* enable AF actuator */
	if (mt9e013_r180_ctrl->sensordata->vcm_enable) {
		CDBG("enable AF actuator, gpio = %d\n",
			 mt9e013_r180_ctrl->sensordata->vcm_pwd);
		rc = gpio_request(mt9e013_r180_ctrl->sensordata->vcm_pwd,
						"mt9e013_r180");
		if (!rc)
			gpio_direction_output(
				mt9e013_r180_ctrl->sensordata->vcm_pwd, 1);
		else {
			CDBG("mt9e013_r180_ctrl gpio request failed!\n");
			goto init_fail;
		}
	}
	/* } FIH, Ming, AF */

	CDBG("init settings\n");
	rc = mt9e013_r180_sensor_setting(REG_INIT, mt9e013_r180_ctrl->prev_res);
	mt9e013_r180_ctrl->fps = 30*Q8;
	mt9e013_r180_init_focus();
	if (rc < 0) {
		gpio_set_value_cansleep(data->sensor_reset, 0);
		goto init_fail;
	} else
		goto init_done;
init_fail:
	CDBG("init_fail\n");
	mt9e013_r180_probe_init_done(data);
init_done:
	CDBG("init_done\n");
	return rc;
} /*endof mt9e013_r180_sensor_open_init*/

static int mt9e013_r180_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&mt9e013_r180_wait_queue);
	return 0;
}

static const struct i2c_device_id mt9e013_r180_i2c_id[] = {
	{"mt9e013_r180", 0},
	{ }
};

static int mt9e013_r180_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	CDBG("mt9e013_r180_probe called!\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CDBG("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	mt9e013_r180_sensorw = kzalloc(sizeof(struct mt9e013_r180_work_t), GFP_KERNEL);
	if (!mt9e013_r180_sensorw) {
		CDBG("kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, mt9e013_r180_sensorw);
	mt9e013_r180_init_client(client);
	mt9e013_r180_client = client;


	CDBG("mt9e013_r180_probe successed! rc = %d\n", rc);
	return 0;

probe_failure:
	CDBG("mt9e013_r180_probe failed! rc = %d\n", rc);
	return rc;
}

static int mt9e013_r180_send_wb_info(struct wb_info_cfg *wb)
{
	return 0;

} /*end of mt9e013_r180_snapshot_config*/

static int __exit mt9e013_r180_remove(struct i2c_client *client)
{
	struct mt9e013_r180_work_t_t *sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);
	mt9e013_r180_client = NULL;
	kfree(sensorw);
	return 0;
}

static struct i2c_driver mt9e013_r180_i2c_driver = {
	.id_table = mt9e013_r180_i2c_id,
	.probe  = mt9e013_r180_i2c_probe,
	.remove = __exit_p(mt9e013_r180_i2c_remove),
	.driver = {
		.name = "mt9e013_r180",
	},
};

int mt9e013_r180_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;
	if (copy_from_user(&cdata,
		(void *)argp,
		sizeof(struct sensor_cfg_data)))
		return -EFAULT;
	mutex_lock(&mt9e013_r180_mut);
	CDBG("mt9e013_r180_sensor_config: cfgtype = %d\n",
	cdata.cfgtype);
		switch (cdata.cfgtype) {
		case CFG_GET_PICT_FPS:
			mt9e013_r180_get_pict_fps(
				cdata.cfg.gfps.prevfps,
				&(cdata.cfg.gfps.pictfps));

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PREV_L_PF:
			cdata.cfg.prevl_pf =
			mt9e013_r180_get_prev_lines_pf();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PREV_P_PL:
			cdata.cfg.prevp_pl =
				mt9e013_r180_get_prev_pixels_pl();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_L_PF:
			cdata.cfg.pictl_pf =
				mt9e013_r180_get_pict_lines_pf();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_P_PL:
			cdata.cfg.pictp_pl =
				mt9e013_r180_get_pict_pixels_pl();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_GET_PICT_MAX_EXP_LC:
			cdata.cfg.pict_max_exp_lc =
				mt9e013_r180_get_pict_max_exp_lc();

			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_SET_FPS:
		case CFG_SET_PICT_FPS:
			rc = mt9e013_r180_set_fps(&(cdata.cfg.fps));
			break;

		case CFG_SET_EXP_GAIN:
			rc =
				mt9e013_r180_write_exp_gain(
					cdata.cfg.exp_gain.gain,
					cdata.cfg.exp_gain.line);
			break;

		case CFG_SET_PICT_EXP_GAIN:
			rc =
				mt9e013_r180_set_pict_exp_gain(
				cdata.cfg.exp_gain.gain,
				cdata.cfg.exp_gain.line);
			break;

		case CFG_SET_MODE:
			rc = mt9e013_r180_set_sensor_mode(cdata.mode,
					cdata.rs);
			break;

		case CFG_PWR_DOWN:
			rc = mt9e013_r180_power_down();
			break;

		case CFG_MOVE_FOCUS:
			rc =
				mt9e013_r180_move_focus(
				cdata.cfg.focus.dir,
				cdata.cfg.focus.steps);
			break;

		case CFG_SET_DEFAULT_FOCUS:
			rc =
				mt9e013_r180_set_default_focus(
				cdata.cfg.focus.steps);
			break;

		case CFG_GET_AF_MAX_STEPS:
			cdata.max_steps = MT9E013_R180_TOTAL_STEPS_NEAR_TO_FAR;
			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct sensor_cfg_data)))
				rc = -EFAULT;
			break;

		case CFG_SET_EFFECT:
			rc = mt9e013_r180_set_default_focus(
				cdata.cfg.effect);
			break;


		case CFG_SEND_WB_INFO:
			rc = mt9e013_r180_send_wb_info(
				&(cdata.cfg.wb_info));
			break;

		default:
			rc = -EFAULT;
			break;
		}

	mutex_unlock(&mt9e013_r180_mut);

	return rc;
}

static int mt9e013_r180_sensor_release(void)
{
	int rc = -EBADF;
	mutex_lock(&mt9e013_r180_mut);
	mt9e013_r180_power_down();
	gpio_set_value_cansleep(mt9e013_r180_ctrl->sensordata->sensor_reset, 0);
	msleep(5);
	gpio_free(mt9e013_r180_ctrl->sensordata->sensor_reset);
	
	/* FIH, Ming, AF { */
	if (mt9e013_r180_ctrl->sensordata->vcm_enable) {
		gpio_direction_output(mt9e013_r180_ctrl->sensordata->vcm_pwd, 0);
		gpio_free(mt9e013_r180_ctrl->sensordata->vcm_pwd);
	}
	/* } FIH, Ming, AF */
	
	kfree(mt9e013_r180_ctrl);
	mt9e013_r180_ctrl = NULL;
	CDBG("mt9e013_r180_release completed\n");
	mutex_unlock(&mt9e013_r180_mut);

	return rc;
}

static int mt9e013_r180_sensor_probe(const struct msm_camera_sensor_info *info,
		struct msm_sensor_ctrl *s)
{
	int rc = 0;
	rc = i2c_add_driver(&mt9e013_r180_i2c_driver);
	if (rc < 0 || mt9e013_r180_client == NULL) {
		rc = -ENOTSUPP;
		CDBG("I2C add driver failed");
		goto probe_fail;
	}
	//msm_camio_clk_rate_set(MT9E013_R180_MASTER_CLK_RATE);
	rc = mt9e013_r180_probe_init_sensor(info);
	if (rc < 0)
		goto probe_fail;
	s->s_init = mt9e013_r180_sensor_open_init;
	s->s_release = mt9e013_r180_sensor_release;
	s->s_config  = mt9e013_r180_sensor_config;
	s->s_mount_angle = info->sensor_platform_info->mount_angle;
	gpio_set_value_cansleep(info->sensor_reset, 0);
	mt9e013_r180_probe_init_done(info);
	mt9e013_r180_create_attributes(mt9e013_r180_client);
	return rc;

probe_fail:
	CDBG("mt9e013_r180_sensor_probe: SENSOR PROBE FAILS!\n");
	return rc;
}

static int __mt9e013_r180_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, mt9e013_r180_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __mt9e013_r180_probe,
	.driver = {
		.name = "msm_camera_mt9e013_r180",
		.owner = THIS_MODULE,
	},
};

#ifndef CONFIG_USE_QUALCOMM_DEVICES 
/**ITV   */
static int __init mt9e013_r180_init(void* hdl)
{
	return platform_driver_register(&msm_camera_driver);
}

DECLARE_DYN_LOADER(DRIVER_TYPE_MAIN_CAM, 
			MAIN_CAM_MT9E013_V2, 
			mt9e013_r180_init, LEVEL6);
#else
static int __init mt9e013_r180_init(void)
{
	return platform_driver_register(&msm_camera_driver);
}

module_init(mt9e013_r180_init);
#endif

void mt9e013_r180_exit(void)
{
	i2c_del_driver(&mt9e013_r180_i2c_driver);
}
MODULE_DESCRIPTION("Aptina 8 MP Bayer sensor driver");
MODULE_LICENSE("GPL v2");

static bool streaming = 1;

static int mt9e013_r180_focus_test(void *data, u64 *val)
{
	int i = 0;
	mt9e013_r180_set_default_focus(0);

	for (i = 90; i < 256; i++) {
		mt9e013_r180_i2c_write_w_sensor(REG_VCM_NEW_CODE, i);
		msleep(5000);
	}
	msleep(5000);
	for (i = 255; i > 90; i--) {
		mt9e013_r180_i2c_write_w_sensor(REG_VCM_NEW_CODE, i);
		msleep(5000);
	}
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cam_focus, mt9e013_r180_focus_test,
			NULL, "%lld\n");

static int mt9e013_r180_step_test(void *data, u64 *val)
{
	int i = 0;
	mt9e013_r180_set_default_focus(0);

	for (i = 0; i < MT9E013_R180_TOTAL_STEPS_NEAR_TO_FAR; i++) {
		mt9e013_r180_move_focus(MOVE_NEAR, 1);
		msleep(5000);
	}

	mt9e013_r180_move_focus(MOVE_FAR, MT9E013_R180_TOTAL_STEPS_NEAR_TO_FAR);
	msleep(5000);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(cam_step, mt9e013_r180_step_test,
			NULL, "%lld\n");

static int cam_debug_stream_set(void *data, u64 val)
{
	int rc = 0;

	if (val) {
		mt9e013_r180_start_stream();
		streaming = 1;
	} else {
		mt9e013_r180_stop_stream();
		streaming = 0;
	}

	return rc;
}

static int cam_debug_stream_get(void *data, u64 *val)
{
	*val = streaming;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(cam_stream, cam_debug_stream_get,
			cam_debug_stream_set, "%llu\n");


static int cam_debug_init(void)
{
	struct dentry *cam_dir;
	debugfs_base = debugfs_create_dir("sensor", NULL);
	if (!debugfs_base)
		return -ENOMEM;

	cam_dir = debugfs_create_dir("mt9e013_r180", debugfs_base);
	if (!cam_dir)
		return -ENOMEM;

	if (!debugfs_create_file("focus", S_IRUGO | S_IWUSR, cam_dir,
							 NULL, &cam_focus))
		return -ENOMEM;
	if (!debugfs_create_file("step", S_IRUGO | S_IWUSR, cam_dir,
							 NULL, &cam_step))
		return -ENOMEM;
	if (!debugfs_create_file("stream", S_IRUGO | S_IWUSR, cam_dir,
							 NULL, &cam_stream))
		return -ENOMEM;

	return 0;
}



