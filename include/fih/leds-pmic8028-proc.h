/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, and the entire permission notice in its entirety,
 *    including the disclaimer of warranties.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * ALTERNATIVELY, this product may be distributed under the terms of
 * the GNU General Public License, version 2, in which case the provisions
 * of the GPL version 2 are required INSTEAD OF the BSD license.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, ALL OF
 * WHICH ARE HEREBY DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF NOT ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 */
#ifndef __LEDS_PMIC8028_PROC_H__
#define __LEDS_PMIC8028_PROC_H__

enum pmic8028_leds {
	PMIC8028_ID_LED_LCD_BACKLIGHT = 0,
	PMIC8028_ID_LED_RED,
	PMIC8028_ID_LED_GREEN,
	PMIC8028_ID_LED_BLUE,
	PMIC8028_ID_LED_KEYBOARD,
	PMIC8028_ID_LED_BUTTON,
    PMIC8028_ID_LED_AMBER,
    PMIC8028_ID_LED_JOGBALL,
    PMIC8028_ID_LED_EXT1,
    PMIC8028_ID_HEADSET_LOOP,
	PMIC8028_ID_LED_MAX,
    PMIC8028_ID_CAMERA_FLASH = 32,

    SMEM_LED_ONMS = 127,
    SMEM_LED_OFFMS,
    SMEM_LED_BLINKONCE,
    SMEM_RED_BLINK,
    SMEM_GREEN_BLINK,
    SMEM_BLUE_BLINK,
};

struct pmic8028_led {
	const char	*name;
	const char	*default_trigger;
	unsigned	max_brightness;
	int		id;
};

struct pmic8028_leds_platform_data {
	int	num_leds;
	struct pmic8028_led *leds;
};

#endif /* __LEDS_PMIC8028_PROC_H__ */
