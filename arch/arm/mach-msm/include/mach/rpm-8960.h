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
 */

#ifndef __ARCH_ARM_MACH_MSM_RPM_8960_H
#define __ARCH_ARM_MACH_MSM_RPM_8960_H

/* RPM control message RAM enums */
enum {
	MSM_RPM_CTRL_VERSION_MAJOR,
	MSM_RPM_CTRL_VERSION_MINOR,
	MSM_RPM_CTRL_VERSION_BUILD,

	MSM_RPM_CTRL_REQ_CTX_0,
	MSM_RPM_CTRL_REQ_CTX_7 = MSM_RPM_CTRL_REQ_CTX_0 + 7,
	MSM_RPM_CTRL_REQ_SEL_0,
	MSM_RPM_CTRL_REQ_SEL_3 = MSM_RPM_CTRL_REQ_SEL_0 + 3,
	MSM_RPM_CTRL_ACK_CTX_0,
	MSM_RPM_CTRL_ACK_CTX_7 = MSM_RPM_CTRL_ACK_CTX_0 + 7,
	MSM_RPM_CTRL_ACK_SEL_0,
	MSM_RPM_CTRL_ACK_SEL_7 = MSM_RPM_CTRL_ACK_SEL_0 + 7,
};


/* RPM resource select enums defined for RPM core
   NOT IN SEQUENTIAL ORDER */
enum {
	MSM_RPM_SEL_NOTIFICATION				= 0,
	MSM_RPM_SEL_INVALIDATE					= 1,
	MSM_RPM_SEL_TRIGGER_TIMED				= 2,
	MSM_RPM_SEL_RPM_CTL					= 3,

	MSM_RPM_SEL_CXO_CLK					= 5,
	MSM_RPM_SEL_PXO_CLK					= 6,
	MSM_RPM_SEL_APPS_FABRIC_CLK				= 8,
	MSM_RPM_SEL_SYSTEM_FABRIC_CLK				= 9,
	MSM_RPM_SEL_MM_FABRIC_CLK				= 10,
	MSM_RPM_SEL_DAYTONA_FABRIC_CLK				= 11,
	MSM_RPM_SEL_SFPB_CLK					= 12,
	MSM_RPM_SEL_CFPB_CLK					= 13,
	MSM_RPM_SEL_MMFPB_CLK					= 14,
	MSM_RPM_SEL_EBI1_CLK					= 16,

	MSM_RPM_SEL_APPS_FABRIC_CFG_HALT			= 18,
	MSM_RPM_SEL_APPS_FABRIC_CFG_CLKMOD			= 19,
	MSM_RPM_SEL_APPS_FABRIC_CFG_IOCTL			= 20,
	MSM_RPM_SEL_APPS_FABRIC_ARB				= 21,

	MSM_RPM_SEL_SYS_FABRIC_CFG_HALT				= 22,
	MSM_RPM_SEL_SYS_FABRIC_CFG_CLKMOD			= 23,
	MSM_RPM_SEL_SYS_FABRIC_CFG_IOCTL			= 24,
	MSM_RPM_SEL_SYSTEM_FABRIC_ARB				= 25,

	MSM_RPM_SEL_MMSS_FABRIC_CFG_HALT			= 26,
	MSM_RPM_SEL_MMSS_FABRIC_CFG_CLKMOD			= 27,
	MSM_RPM_SEL_MMSS_FABRIC_CFG_IOCTL			= 28,
	MSM_RPM_SEL_MM_FABRIC_ARB				= 29,

	MSM_RPM_SEL_PM8921_S1					= 30,
	MSM_RPM_SEL_PM8921_S2					= 31,
	MSM_RPM_SEL_PM8921_S3					= 32,
	MSM_RPM_SEL_PM8921_S4					= 33,
	MSM_RPM_SEL_PM8921_S5					= 34,
	MSM_RPM_SEL_PM8921_S6					= 35,
	MSM_RPM_SEL_PM8921_S7					= 36,
	MSM_RPM_SEL_PM8921_S8					= 37,
	MSM_RPM_SEL_PM8921_L1					= 38,
	MSM_RPM_SEL_PM8921_L2					= 39,
	MSM_RPM_SEL_PM8921_L3					= 40,
	MSM_RPM_SEL_PM8921_L4					= 41,
	MSM_RPM_SEL_PM8921_L5					= 42,
	MSM_RPM_SEL_PM8921_L6					= 43,
	MSM_RPM_SEL_PM8921_L7					= 44,
	MSM_RPM_SEL_PM8921_L8					= 45,
	MSM_RPM_SEL_PM8921_L9					= 46,
	MSM_RPM_SEL_PM8921_L10					= 47,
	MSM_RPM_SEL_PM8921_L11					= 48,
	MSM_RPM_SEL_PM8921_L12					= 49,
	MSM_RPM_SEL_PM8921_L13					= 50,
	MSM_RPM_SEL_PM8921_L14					= 51,
	MSM_RPM_SEL_PM8921_L15					= 52,
	MSM_RPM_SEL_PM8921_L16					= 53,
	MSM_RPM_SEL_PM8921_L17					= 54,
	MSM_RPM_SEL_PM8921_L18					= 55,
	MSM_RPM_SEL_PM8921_L19					= 56,
	MSM_RPM_SEL_PM8921_L20					= 57,
	MSM_RPM_SEL_PM8921_L21					= 58,
	MSM_RPM_SEL_PM8921_L22					= 59,
	MSM_RPM_SEL_PM8921_L23					= 60,
	MSM_RPM_SEL_PM8921_L24					= 61,
	MSM_RPM_SEL_PM8921_L25					= 62,
	MSM_RPM_SEL_PM8921_L26					= 63,
	MSM_RPM_SEL_PM8921_L27					= 64,
	MSM_RPM_SEL_PM8921_L28					= 65,
	MSM_RPM_SEL_PM8921_L29					= 66,
	MSM_RPM_SEL_PM8921_CLK1					= 67,
	MSM_RPM_SEL_PM8921_CLK2					= 68,
	MSM_RPM_SEL_PM8921_LVS1					= 69,
	MSM_RPM_SEL_PM8921_LVS2					= 70,
	MSM_RPM_SEL_PM8921_LVS3					= 71,
	MSM_RPM_SEL_PM8921_LVS4					= 72,
	MSM_RPM_SEL_PM8921_LVS5					= 73,
	MSM_RPM_SEL_PM8921_LVS6					= 74,
	MSM_RPM_SEL_PM8921_LVS7					= 75,

	MSM_RPM_SEL_NCP						= 80,
	MSM_RPM_SEL_CXO_BUFFERS					= 81,
	MSM_RPM_SEL_USB_OTG_SWITCH				= 82,
	MSM_RPM_SEL_HDMI_SWITCH					= 83,

	MSM_RPM_SEL_LAST = MSM_RPM_SEL_HDMI_SWITCH,
};

/* RPM resource (4 byte) word ID enum */
enum {
	MSM_RPM_ID_NOTIFICATION_CONFIGURED_0			= 0,
	MSM_RPM_ID_NOTIFICATION_CONFIGURED_3 =
		MSM_RPM_ID_NOTIFICATION_CONFIGURED_0 + 3,

	MSM_RPM_ID_NOTIFICATION_REGISTERED_0			= 4,
	MSM_RPM_ID_NOTIFICATION_REGISTERED_3 =
		MSM_RPM_ID_NOTIFICATION_REGISTERED_0 + 3,

	MSM_RPM_ID_INVALIDATE_0					= 8,
	MSM_RPM_ID_INVALIDATE_7 =
		MSM_RPM_ID_INVALIDATE_0 + 7,

	MSM_RPM_ID_TRIGGER_TIMED_TO				= 16,
	MSM_RPM_ID_TRIGGER_TIMED_SCLK_COUNT			= 17,

	MSM_RPM_ID_RPM_CTL					= 18,

	/* TRIGGER_CLEAR/SET deprecated in these 24 RESERVED bytes */
	MSM_RPM_ID_RESERVED_0					= 19,
	MSM_RPM_ID_RESERVED_5 =
		MSM_RPM_ID_RESERVED_0 + 5,

	MSM_RPM_ID_CXO_CLK					= 25,
	MSM_RPM_ID_PXO_CLK					= 26,
	MSM_RPM_ID_APPS_FABRIC_CLK				= 27,
	MSM_RPM_ID_SYSTEM_FABRIC_CLK				= 28,
	MSM_RPM_ID_MM_FABRIC_CLK				= 29,
	MSM_RPM_ID_DAYTONA_FABRIC_CLK				= 30,
	MSM_RPM_ID_SFPB_CLK					= 31,
	MSM_RPM_ID_CFPB_CLK					= 32,
	MSM_RPM_ID_MMFPB_CLK					= 33,
	MSM_RPM_ID_EBI1_CLK					= 34,

	MSM_RPM_ID_APPS_FABRIC_CFG_HALT_0			= 35,
	MSM_RPM_ID_APPS_FABRIC_CFG_HALT_1			= 36,
	MSM_RPM_ID_APPS_FABRIC_CFG_CLKMOD_0			= 37,
	MSM_RPM_ID_APPS_FABRIC_CFG_CLKMOD_1			= 38,
	MSM_RPM_ID_APPS_FABRIC_CFG_CLKMOD_2			= 39,
	MSM_RPM_ID_APPS_FABRIC_CFG_IOCTL			= 40,
	MSM_RPM_ID_APPS_FABRIC_ARB_0				= 41,
	MSM_RPM_ID_APPS_FABRIC_ARB_11 =
		MSM_RPM_ID_APPS_FABRIC_ARB_0 + 11,

	MSM_RPM_ID_SYS_FABRIC_CFG_HALT_0			= 53,
	MSM_RPM_ID_SYS_FABRIC_CFG_HALT_1			= 54,
	MSM_RPM_ID_SYS_FABRIC_CFG_CLKMOD_0			= 55,
	MSM_RPM_ID_SYS_FABRIC_CFG_CLKMOD_1			= 56,
	MSM_RPM_ID_SYS_FABRIC_CFG_CLKMOD_2			= 57,
	MSM_RPM_ID_SYS_FABRIC_CFG_IOCTL				= 58,
	MSM_RPM_ID_SYSTEM_FABRIC_ARB_0				= 59,
	MSM_RPM_ID_SYSTEM_FABRIC_ARB_28 =
		MSM_RPM_ID_SYSTEM_FABRIC_ARB_0 + 28,

	MSM_RPM_ID_MMSS_FABRIC_CFG_HALT_0			= 88,
	MSM_RPM_ID_MMSS_FABRIC_CFG_HALT_1			= 89,
	MSM_RPM_ID_MMSS_FABRIC_CFG_CLKMOD_0			= 90,
	MSM_RPM_ID_MMSS_FABRIC_CFG_CLKMOD_1			= 91,
	MSM_RPM_ID_MMSS_FABRIC_CFG_CLKMOD_2			= 92,
	MSM_RPM_ID_MMSS_FABRIC_CFG_IOCTL			= 93,
	MSM_RPM_ID_MM_FABRIC_ARB_0				= 94,
	MSM_RPM_ID_MM_FABRIC_ARB_22 =
		MSM_RPM_ID_MM_FABRIC_ARB_0 + 22,

	MSM_RPM_ID_PM8921_S1_0					= 117,
	MSM_RPM_ID_PM8921_S1_1					= 118,
	MSM_RPM_ID_PM8921_S2_0					= 119,
	MSM_RPM_ID_PM8921_S2_1					= 120,
	MSM_RPM_ID_PM8921_S3_0					= 121,
	MSM_RPM_ID_PM8921_S3_1					= 122,
	MSM_RPM_ID_PM8921_S4_0					= 123,
	MSM_RPM_ID_PM8921_S4_1					= 124,
	MSM_RPM_ID_PM8921_S5_0					= 125,
	MSM_RPM_ID_PM8921_S5_1					= 126,
	MSM_RPM_ID_PM8921_S6_0					= 127,
	MSM_RPM_ID_PM8921_S6_1					= 128,
	MSM_RPM_ID_PM8921_S7_0					= 129,
	MSM_RPM_ID_PM8921_S7_1					= 130,
	MSM_RPM_ID_PM8921_S8_0					= 131,
	MSM_RPM_ID_PM8921_S8_1					= 132,
	MSM_RPM_ID_PM8921_L1_0					= 133,
	MSM_RPM_ID_PM8921_L1_1					= 134,
	MSM_RPM_ID_PM8921_L2_0					= 135,
	MSM_RPM_ID_PM8921_L2_1					= 136,
	MSM_RPM_ID_PM8921_L3_0					= 137,
	MSM_RPM_ID_PM8921_L3_1					= 138,
	MSM_RPM_ID_PM8921_L4_0					= 139,
	MSM_RPM_ID_PM8921_L4_1					= 140,
	MSM_RPM_ID_PM8921_L5_0					= 141,
	MSM_RPM_ID_PM8921_L5_1					= 142,
	MSM_RPM_ID_PM8921_L6_0					= 143,
	MSM_RPM_ID_PM8921_L6_1					= 144,
	MSM_RPM_ID_PM8921_L7_0					= 145,
	MSM_RPM_ID_PM8921_L7_1					= 146,
	MSM_RPM_ID_PM8921_L8_0					= 147,
	MSM_RPM_ID_PM8921_L8_1					= 148,
	MSM_RPM_ID_PM8921_L9_0					= 149,
	MSM_RPM_ID_PM8921_L9_1					= 150,
	MSM_RPM_ID_PM8921_L10_0					= 151,
	MSM_RPM_ID_PM8921_L10_1					= 152,
	MSM_RPM_ID_PM8921_L11_0					= 153,
	MSM_RPM_ID_PM8921_L11_1					= 154,
	MSM_RPM_ID_PM8921_L12_0					= 155,
	MSM_RPM_ID_PM8921_L12_1					= 156,
	MSM_RPM_ID_PM8921_L13_0					= 157,
	MSM_RPM_ID_PM8921_L13_1					= 158,
	MSM_RPM_ID_PM8921_L14_0					= 159,
	MSM_RPM_ID_PM8921_L14_1					= 160,
	MSM_RPM_ID_PM8921_L15_0					= 161,
	MSM_RPM_ID_PM8921_L15_1					= 162,
	MSM_RPM_ID_PM8921_L16_0					= 163,
	MSM_RPM_ID_PM8921_L16_1					= 164,
	MSM_RPM_ID_PM8921_L17_0					= 165,
	MSM_RPM_ID_PM8921_L17_1					= 166,
	MSM_RPM_ID_PM8921_L18_0					= 167,
	MSM_RPM_ID_PM8921_L18_1					= 168,
	MSM_RPM_ID_PM8921_L19_0					= 169,
	MSM_RPM_ID_PM8921_L19_1					= 170,
	MSM_RPM_ID_PM8921_L20_0					= 171,
	MSM_RPM_ID_PM8921_L20_1					= 172,
	MSM_RPM_ID_PM8921_L21_0					= 173,
	MSM_RPM_ID_PM8921_L21_1					= 174,
	MSM_RPM_ID_PM8921_L22_0					= 175,
	MSM_RPM_ID_PM8921_L22_1					= 176,
	MSM_RPM_ID_PM8921_L23_0					= 177,
	MSM_RPM_ID_PM8921_L23_1					= 178,
	MSM_RPM_ID_PM8921_L24_0					= 179,
	MSM_RPM_ID_PM8921_L24_1					= 180,
	MSM_RPM_ID_PM8921_L25_0					= 181,
	MSM_RPM_ID_PM8921_L25_1					= 182,
	MSM_RPM_ID_PM8921_L26_0					= 183,
	MSM_RPM_ID_PM8921_L26_1					= 184,
	MSM_RPM_ID_PM8921_L27_0					= 185,
	MSM_RPM_ID_PM8921_L27_1					= 186,
	MSM_RPM_ID_PM8921_L28_0					= 187,
	MSM_RPM_ID_PM8921_L28_1					= 188,
	MSM_RPM_ID_PM8921_L29_0					= 189,
	MSM_RPM_ID_PM8921_L29_1					= 190,
	MSM_RPM_ID_PM8921_CLK1_0				= 191,
	MSM_RPM_ID_PM8921_CLK1_1				= 192,
	MSM_RPM_ID_PM8921_CLK2_0				= 193,
	MSM_RPM_ID_PM8921_CLK2_1				= 194,
	MSM_RPM_ID_PM8921_LVS1					= 195,
	MSM_RPM_ID_PM8921_LVS2					= 196,
	MSM_RPM_ID_PM8921_LVS3					= 197,
	MSM_RPM_ID_PM8921_LVS4					= 198,
	MSM_RPM_ID_PM8921_LVS5					= 199,
	MSM_RPM_ID_PM8921_LVS6					= 200,
	MSM_RPM_ID_PM8921_LVS7					= 201,
	MSM_RPM_ID_NCP_0					= 202,
	MSM_RPM_ID_NCP_1					= 203,

	MSM_RPM_ID_CXO_BUFFERS					= 204,
	MSM_RPM_ID_USB_OTG_SWITCH				= 205,
	MSM_RPM_ID_HDMI_SWITCH					= 206,

	MSM_RPM_ID_LAST = MSM_RPM_ID_HDMI_SWITCH
};

/* RPM resources RPM_ID aliases */
enum {
	MSM_RPMRS_ID_RPM_CTL = MSM_RPM_ID_RPM_CTL,
	MSM_RPMRS_ID_PXO_CLK = MSM_RPM_ID_PXO_CLK,
	MSM_RPMRS_ID_VDD_DIG_0 = MSM_RPM_ID_PM8921_S3_0,
	MSM_RPMRS_ID_VDD_DIG_1 = MSM_RPM_ID_PM8921_S3_1,
	MSM_RPMRS_ID_VDD_MEM_0 = MSM_RPM_ID_PM8921_L24_0,
	MSM_RPMRS_ID_VDD_MEM_1 = MSM_RPM_ID_PM8921_L24_1,

	/* MSM8960 L2 cache power control not via RPM
	 * MSM_RPM_ID_LAST + 1 indicates invalid */
	MSM_RPMRS_ID_APPS_L2_CACHE_CTL = MSM_RPM_ID_LAST + 1
};

/* RPM status ID enum */
enum {
	MSM_RPM_STATUS_ID_VERSION_MAJOR				= 0,
	MSM_RPM_STATUS_ID_VERSION_MINOR				= 1,
	MSM_RPM_STATUS_ID_VERSION_BUILD				= 2,
	MSM_RPM_STATUS_ID_SUPPORTED_RESOURCES_0			= 3,
	MSM_RPM_STATUS_ID_SUPPORTED_RESOURCES_1			= 4,
	MSM_RPM_STATUS_ID_SUPPORTED_RESOURCES_2			= 5,
	MSM_RPM_STATUS_ID_RESERVED_SUPPORTED_RESOURCES_0	= 6,
	MSM_RPM_STATUS_ID_SEQUENCE				= 7,
	MSM_RPM_STATUS_ID_RPM_CTL				= 8,
	MSM_RPM_STATUS_ID_CXO_CLK				= 9,
	MSM_RPM_STATUS_ID_PXO_CLK				= 10,
	MSM_RPM_STATUS_ID_APPS_FABRIC_CLK			= 11,
	MSM_RPM_STATUS_ID_SYSTEM_FABRIC_CLK			= 12,
	MSM_RPM_STATUS_ID_MM_FABRIC_CLK				= 13,
	MSM_RPM_STATUS_ID_DAYTONA_FABRIC_CLK			= 14,
	MSM_RPM_STATUS_ID_SFPB_CLK				= 15,
	MSM_RPM_STATUS_ID_CFPB_CLK				= 16,
	MSM_RPM_STATUS_ID_MMFPB_CLK				= 17,
	MSM_RPM_STATUS_ID_EBI1_CLK				= 18,
	MSM_RPM_STATUS_ID_APPS_FABRIC_CFG_HALT			= 19,
	MSM_RPM_STATUS_ID_APPS_FABRIC_CFG_CLKMOD		= 20,
	MSM_RPM_STATUS_ID_APPS_FABRIC_CFG_IOCTL			= 21,
	MSM_RPM_STATUS_ID_APPS_FABRIC_ARB			= 22,
	MSM_RPM_STATUS_ID_SYS_FABRIC_CFG_HALT			= 23,
	MSM_RPM_STATUS_ID_SYS_FABRIC_CFG_CLKMOD			= 24,
	MSM_RPM_STATUS_ID_SYS_FABRIC_CFG_IOCTL			= 25,
	MSM_RPM_STATUS_ID_SYSTEM_FABRIC_ARB			= 26,
	MSM_RPM_STATUS_ID_MMSS_FABRIC_CFG_HALT			= 27,
	MSM_RPM_STATUS_ID_MMSS_FABRIC_CFG_CLKMOD		= 28,
	MSM_RPM_STATUS_ID_MMSS_FABRIC_CFG_IOCTL			= 29,
	MSM_RPM_STATUS_ID_MM_FABRIC_ARB				= 30,
	MSM_RPM_STATUS_ID_PM8921_S1_0				= 31,
	MSM_RPM_STATUS_ID_PM8921_S1_1				= 32,
	MSM_RPM_STATUS_ID_PM8921_S2_0				= 33,
	MSM_RPM_STATUS_ID_PM8921_S2_1				= 34,
	MSM_RPM_STATUS_ID_PM8921_S3_0				= 35,
	MSM_RPM_STATUS_ID_PM8921_S3_1				= 36,
	MSM_RPM_STATUS_ID_PM8921_S4_0				= 37,
	MSM_RPM_STATUS_ID_PM8921_S4_1				= 38,
	MSM_RPM_STATUS_ID_PM8921_S5_0				= 39,
	MSM_RPM_STATUS_ID_PM8921_S5_1				= 40,
	MSM_RPM_STATUS_ID_PM8921_S6_0				= 41,
	MSM_RPM_STATUS_ID_PM8921_S6_1				= 42,
	MSM_RPM_STATUS_ID_PM8921_S7_0				= 43,
	MSM_RPM_STATUS_ID_PM8921_S7_1				= 44,
	MSM_RPM_STATUS_ID_PM8921_S8_0				= 45,
	MSM_RPM_STATUS_ID_PM8921_S8_1				= 46,
	MSM_RPM_STATUS_ID_PM8921_L1_0				= 47,
	MSM_RPM_STATUS_ID_PM8921_L1_1				= 48,
	MSM_RPM_STATUS_ID_PM8921_L2_0				= 49,
	MSM_RPM_STATUS_ID_PM8921_L2_1				= 50,
	MSM_RPM_STATUS_ID_PM8921_L3_0				= 51,
	MSM_RPM_STATUS_ID_PM8921_L3_1				= 52,
	MSM_RPM_STATUS_ID_PM8921_L4_0				= 53,
	MSM_RPM_STATUS_ID_PM8921_L4_1				= 54,
	MSM_RPM_STATUS_ID_PM8921_L5_0				= 55,
	MSM_RPM_STATUS_ID_PM8921_L5_1				= 56,
	MSM_RPM_STATUS_ID_PM8921_L6_0				= 57,
	MSM_RPM_STATUS_ID_PM8921_L6_1				= 58,
	MSM_RPM_STATUS_ID_PM8921_L7_0				= 59,
	MSM_RPM_STATUS_ID_PM8921_L7_1				= 60,
	MSM_RPM_STATUS_ID_PM8921_L8_0				= 61,
	MSM_RPM_STATUS_ID_PM8921_L8_1				= 62,
	MSM_RPM_STATUS_ID_PM8921_L9_0				= 63,
	MSM_RPM_STATUS_ID_PM8921_L9_1				= 64,
	MSM_RPM_STATUS_ID_PM8921_L10_0				= 65,
	MSM_RPM_STATUS_ID_PM8921_L10_1				= 66,
	MSM_RPM_STATUS_ID_PM8921_L11_0				= 67,
	MSM_RPM_STATUS_ID_PM8921_L11_1				= 68,
	MSM_RPM_STATUS_ID_PM8921_L12_0				= 69,
	MSM_RPM_STATUS_ID_PM8921_L12_1				= 70,
	MSM_RPM_STATUS_ID_PM8921_L13_0				= 71,
	MSM_RPM_STATUS_ID_PM8921_L13_1				= 72,
	MSM_RPM_STATUS_ID_PM8921_L14_0				= 73,
	MSM_RPM_STATUS_ID_PM8921_L14_1				= 74,
	MSM_RPM_STATUS_ID_PM8921_L15_0				= 75,
	MSM_RPM_STATUS_ID_PM8921_L15_1				= 76,
	MSM_RPM_STATUS_ID_PM8921_L16_0				= 77,
	MSM_RPM_STATUS_ID_PM8921_L16_1				= 78,
	MSM_RPM_STATUS_ID_PM8921_L17_0				= 79,
	MSM_RPM_STATUS_ID_PM8921_L17_1				= 80,
	MSM_RPM_STATUS_ID_PM8921_L18_0				= 81,
	MSM_RPM_STATUS_ID_PM8921_L18_1				= 82,
	MSM_RPM_STATUS_ID_PM8921_L19_0				= 83,
	MSM_RPM_STATUS_ID_PM8921_L19_1				= 84,
	MSM_RPM_STATUS_ID_PM8921_L20_0				= 85,
	MSM_RPM_STATUS_ID_PM8921_L20_1				= 86,
	MSM_RPM_STATUS_ID_PM8921_L21_0				= 87,
	MSM_RPM_STATUS_ID_PM8921_L21_1				= 88,
	MSM_RPM_STATUS_ID_PM8921_L22_0				= 89,
	MSM_RPM_STATUS_ID_PM8921_L22_1				= 90,
	MSM_RPM_STATUS_ID_PM8921_L23_0				= 91,
	MSM_RPM_STATUS_ID_PM8921_L23_1				= 92,
	MSM_RPM_STATUS_ID_PM8921_L24_0				= 93,
	MSM_RPM_STATUS_ID_PM8921_L24_1				= 94,
	MSM_RPM_STATUS_ID_PM8921_L25_0				= 95,
	MSM_RPM_STATUS_ID_PM8921_L25_1				= 96,
	MSM_RPM_STATUS_ID_PM8921_L26_0				= 97,
	MSM_RPM_STATUS_ID_PM8921_L26_1				= 98,
	MSM_RPM_STATUS_ID_PM8921_L27_0				= 99,
	MSM_RPM_STATUS_ID_PM8921_L27_1				= 100,
	MSM_RPM_STATUS_ID_PM8921_L28_0				= 101,
	MSM_RPM_STATUS_ID_PM8921_L28_1				= 102,
	MSM_RPM_STATUS_ID_PM8921_L29_0				= 103,
	MSM_RPM_STATUS_ID_PM8921_L29_1				= 104,
	MSM_RPM_STATUS_ID_PM8921_CLK1_0				= 105,
	MSM_RPM_STATUS_ID_PM8921_CLK1_1				= 106,
	MSM_RPM_STATUS_ID_PM8921_CLK2_0				= 107,
	MSM_RPM_STATUS_ID_PM8921_CLK2_1				= 108,
	MSM_RPM_STATUS_ID_PM8921_LVS1				= 109,
	MSM_RPM_STATUS_ID_PM8921_LVS2				= 110,
	MSM_RPM_STATUS_ID_PM8921_LVS3				= 111,
	MSM_RPM_STATUS_ID_PM8921_LVS4				= 112,
	MSM_RPM_STATUS_ID_PM8921_LVS5				= 113,
	MSM_RPM_STATUS_ID_PM8921_LVS6				= 114,
	MSM_RPM_STATUS_ID_PM8921_LVS7				= 115,
	MSM_RPM_STATUS_ID_NCP_0					= 116,
	MSM_RPM_STATUS_ID_NCP_1					= 117,
	MSM_RPM_STATUS_ID_CXO_BUFFERS				= 118,
	MSM_RPM_STATUS_ID_USB_OTG_SWITCH			= 119,
	MSM_RPM_STATUS_ID_HDMI_SWITCH				= 120,

	MSM_RPM_STATUS_ID_LAST = MSM_RPM_STATUS_ID_HDMI_SWITCH
};

#endif /* __ARCH_ARM_MACH_MSM_RPM_8960_H */
