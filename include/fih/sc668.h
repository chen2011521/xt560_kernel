#ifndef __SC668_H__
#define __SC668_H__

#define SC668_B1			0
#define SC668_B2			1
#define SC668_B3			2
#define SC668_B4			3

/* Regs Map */
#define SC668_BLEN					0x00
#define SC668_BANKEN				0x01
#define	SC668_FADEEN_B1				0x02
#define	SC668_FADEEN_B2				SC668_FADEEN_B1 + SC668_B2
#define	SC668_FADEEN_B3				SC668_FADEEN_B1 + SC668_B3
#define	SC668_FADEEN_B4				SC668_FADEEN_B1 + SC668_B4
#define	SC668_BLINKEN_B1			0x06
#define	SC668_BLINKEN_B2			SC668_BLINKEN_B1 + SC668_B2
#define	SC668_BLINKEN_B3			SC668_BLINKEN_B1 + SC668_B3
#define	SC668_BLINKEN_B4			SC668_BLINKEN_B1 + SC668_B4
#define SC668_LIGHTING_EFFECT		0x0E
#define SC668_EFFECT_RATES			0x0F
#define SC668_TARGET_START_TIMES_G1	0x10
#define SC668_TARGET_START_TIMES_G2	0x11
#define SC668_ADP_OLE				0x16

/* Backlight Current */
#define SC668_BLCURR_0MA				0x00
#define SC668_BLCURR_0P05MA				0x01
#define SC668_BLCURR_0P1MA				0x02
#define SC668_BLCURR_0P2MA				0x03
#define SC668_BLCURR_0P5MA				0x04
#define SC668_BLCURR_1MA				0x05
#define SC668_BLCURR_1P5MA				0x06
#define SC668_BLCURR_2MA				0x07
#define SC668_BLCURR_2P5MA				0x08	
#define SC668_BLCURR_3MA				0x09
#define SC668_BLCURR_3P5MA				0x0A	
#define SC668_BLCURR_4MA				0x0B
#define SC668_BLCURR_4P5MA				0x0C
#define SC668_BLCURR_5MA				0x0D
#define SC668_BLCURR_6MA				0x0E
#define SC668_BLCURR_7MA				0x0F
#define SC668_BLCURR_8MA				0x10
#define SC668_BLCURR_9MA				0x11
#define SC668_BLCURR_10MA				0x12
#define SC668_BLCURR_11MA				0x13
#define SC668_BLCURR_12MA				0x14
#define SC668_BLCURR_13MA				0x15
#define SC668_BLCURR_14MA				0x16
#define SC668_BLCURR_15MA				0x17
#define SC668_BLCURR_16MA				0x18
#define SC668_BLCURR_17MA				0x19
#define SC668_BLCURR_18MA				0x1A
#define SC668_BLCURR_19MA				0x1B
#define SC668_BLCURR_20MA				0x1C
#define SC668_BLCURR_21MA				0x1D
#define SC668_BLCURR_23MA				0x1E
#define SC668_BLCURR_25MA				0x1F

/* Lighting Effect Reg */
#define SC668_LIGHTING_EFFECT_GRP0		3

#define SC668_GRP_B1_B234				0x0 << SC668_LIGHTING_EFFECT_GRP0
#define SC668_GRP_B12_B34				0x1 << SC668_LIGHTING_EFFECT_GRP0
#define SC668_GRP_B123_B4				0x2 << SC668_LIGHTING_EFFECT_GRP0
#define	SC668_GRP_B1234					0x3 << SC668_LIGHTING_EFFECT_GRP0

#define	SC668_BANK_BL18					0x0
#define	SC668_BANK_BL28_BL1				0x1
#define SC668_BANK_BL38_BL12			0x2
#define SC668_BANK_BL38_BL2_BL1			0x3
#define SC668_BANK_BL48_BL13			0x4
#define SC668_BANK_BL48_BL3_BL2_BL1		0x5
#define SC668_BANK_BL58_BL34_BL2_BL1	0x6
#define SC668_BANK_BL68_BL35_BL2_BL1	0x7

/* Effect Rate Reg */
#define SC668_EFFECT_RATES_GRP2			3

#define SC668_EFFRATES_SNAP				0x0
#define SC668_EFFRATES_B4F1MS			0x1
#define SC668_EFFRATES_B8F2MS			0x2
#define SC668_EFFRATES_B16F4MS			0x3
#define SC668_EFFRATES_B24F6MS			0x4
#define SC668_EFFRATES_B32F8MS			0x5
#define SC668_EFFRATES_B48F12MS			0x6
#define SC668_EFFRATES_B64F16MS			0x7

/* Target and Start Times Reg */
#define SC668_TARGET_START_TIMES_TT0	3

#define SC668_TIME_32MS					0x0
#define SC668_TIME_64MS					0x1
#define SC668_TIME_256MS				0x2
#define SC668_TIME_512MS				0x3
#define SC668_TIME_1024MS				0x4
#define SC668_TIME_2048MS				0x5
#define SC668_TIME_3072MS				0x6
#define SC668_TIME_4096MS				0x7

/* Other Lighting Effect */
#define SC668_OLE_PWM_BYP	0x0
#define SC668_OLE_OLEEN0	0x1

#define SC668_OLE_NO_EFFECT			0x0 << SC668_OLE_OLEEN0
#define SC668_OLE_AUTO_DIM_FULL		0x1 << SC668_OLE_OLEEN0
#define SC668_OLE_AUTO_DIM_PARTIAL	0x2 << SC668_OLE_OLEEN0

#define SC668_LED_RED		0x0
#define SC668_LED_GREEN		0x1
#define SC668_LED_BLUE		0x2
#define SC668_LED_TP		0x3
#define SC668_LED_MAX		0x4

#define SC668_MAX_LEDS_NUM	8

struct sc668_platform_data {
	unsigned char en_pin;
	unsigned char pwm_pin;
	unsigned char chg_led_ctrl_pin;
};

#endif
