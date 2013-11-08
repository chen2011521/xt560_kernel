/* Copyright (c) 2008-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef DIAGFWD_H
#define DIAGFWD_H

//#define SLATE_DEBUG


#define DIAG_MASK_CMD_SAVE                           0
#define DIAG_MASK_CMD_RESTORE                        1
#define DIAG_MASK_CMD_ADD_GET_RSSI                   2
#define DIAG_MASK_CMD_ADD_GET_STATE_AND_CONN_ATT     3
#define DIAG_MASK_CMD_ADD_GET_SEARCHER_DUMP          4

/* Values for pending slate log commands */
#define DIAG_LOG_CMD_TYPE_NONE                    0
#define DIAG_LOG_CMD_TYPE_GET_RSSI                1
#define DIAG_LOG_CMD_TYPE_GET_STATE_AND_CONN_ATT  2
#define DIAG_LOG_CMD_TYPE_RESTORE_LOG_MASKS       3
#define DIAG_LOG_CMD_TYPE_GET_1x_SEARCHER_DUMP 	  4

/* Log Types */
#define DIAG_LOG_TYPE_RSSI                        0
#define DIAG_LOG_TYPE_STATE                       1
#define DIAG_LOG_TYPE_CONN_ATT                    2
//FXPCAYM-87
#define DIAG_LOG_TYPE_SEARCHER_AND_FINGER         3
#define DIAG_LOG_TYPE_INTERNAL_CORE_DUMP          4
#define DIAG_LOG_TYPE_SRCH_TNG_SEARCHER_DUMP      5

/* FIH, CHHsieh, 2011/09/27 { */
// G6: save BSP/fver into /data/fver
#include "../../fih/char/diag/fih_diagfwd.h"
void diag_read_work_fn(struct work_struct *);
void diag_process_hdlc_fn(struct work_struct *);
/* } FIH, CHHsieh, 2011/09/27 */

#define NO_PROCESS	0
#define NON_APPS_PROC	-1

void diagfwd_init(void);
void diagfwd_exit(void);
void diag_process_hdlc(void *data, unsigned len);
void __diag_smd_send_req(void);
void __diag_smd_qdsp_send_req(void);
void __diag_smd_wcnss_send_req(void);
void diag_usb_legacy_notifier(void *, unsigned, struct diag_request *);
long diagchar_ioctl(struct file *, unsigned int, unsigned long);
int diag_device_write(void *, int, struct diag_request *);
int mask_request_validate(unsigned char mask_buf[]);
int chk_config_get_id(void);
/* State for diag forwarding */
#ifdef CONFIG_DIAG_OVER_USB
int diagfwd_connect(void);
int diagfwd_disconnect(void);
#endif
extern int diag_debug_buf_idx;
extern unsigned char diag_debug_buf[1024];

/* Routines added for SLATE support */
void diag_process_get_rssi_log(void);
void diag_process_get_stateAndConnInfo_log(void);
int  diag_log_is_enabled(unsigned char log_type);

#endif
