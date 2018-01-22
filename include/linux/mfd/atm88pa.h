/*
 * include/mfd/atm88pa.h
 *
 * Public defines of ATMEGA88PA MFD driver
 *
 * Copyright (C) 2017 NanJing No.55 Research Institute Technology Development CO.,LTD.
 *
 * Author: Varphone Wong <varphone@qq.com>
 *
 */
#ifndef MFD_ATM88PA_H
#define MFD_ATM88PA_H

/*
 * Key bits:
 *   bit[7]: Power,
 *   bit[6]: F6
 *   bit[5]: F5
 *   bit[4]: F4
 *   bit[3]: Down
 *   bit[2]: Up
 *   bit[1]: Escape
 *   bit[0]: Enter
 * Values:
 *   0: Released
 *   1: Pressed
 */
#define ATM88PA_REG_KEYS	0x00 /* Key bit states */

/*
 * Status bits (Ver: 50x):
 *   bit[7]: Undefined
 *   bit[6]: Side camera fault, 1 fault, 0 normal
 *   bit[5]: Back camera fault, 1 fault, 0 normal
 *   bit[4]: Front camera fault, 1 fault, 0 normal
 *   bit[3]: Hall sensor, 1 closed, 0 opened
 *   bit[2]: Power source, 1 external, 0 super cap
 *   bit[1:0]: SuperCap state, undefined
 * Status bits (Ver 60x):
 *   bit[7]: Display power fault, 1 fault, 0 normal
 *   bit[6]: Back camera power fault, 1 fault, 0 normal
 *   bit[5]: Front camera power fault, 1 fault, 0 normal
 *   bit[4]: Right camera power fault, 1 fault, 0 normal
 *   bit[3]: Left camera power fault, 1 fault, 0 normal
 *   bit[2]: Power source, 1 external, 0 super cap
 *   bit[1:0] SuperCap state, undefined
 */
#define ATM88PA_REG_STATUS	0x01 /* R */

/* Temperature, 16 bit signed, precision: 0.1 Celsius */
#define ATM88PA_REG_TEMPERATURE	0x02 /* R */

/* SuperCap voltage, 16 bit unsigned, precision: 0.01V */
#define ATM88PA_REG_SUCAP_VOLT	0x04 /* R */

/* Ambient light, 16 bit unsigned, precision: undefined */
#define ATM88PA_REG_AMB_LIGHT	0x06 /* R/W */

/* ttymxc1 switch, 0 = internal, 1 = external */
#define ATM88PA_REG_TTYMXC1_SWITCH	0x08 /* R/W */

/* Camera switch, 1 back, 0 front */
#define ATM88PA_REG_CAM_SWITCH	0x08 /* R/W */

/* Supplementary lighting, 0 ~ 100 */
#define ATM88PA_REG_SUP_LIGHT	0x09 /* R/W */

/*
 * Indicator lighting, 0 ~ 100
 * NOTE: Ver 60x not supported
 */
#define ATM88PA_REG_IND_LIGHT	0x0A /* R/W */

/* LCD / Projector power control, 0 turn off, 1 turn on */
#define ATM88PA_REG_LCD_PWR_CTRL	0x0B /* R/W */

/* Front camera power control, 1 on, 0 off */
#define ATM88PA_REG_FC_PWR_CTRL	0x0C /* R/W */

/* Back camera power control, 1 on, 0 off */
#define ATM88PA_REG_BC_PWR_CTRL	0x0D /* R/W */

/* LCD / Projector lighting, 0 ~ 100 */
#define ATM88PA_REG_LCD_LIGHT	0x0E /* R/W */

/* Fault flags, 1 fault, 0 normal */
#define ATM88PA_REG_FAULT	0x0F /* R/W */

/* Poweroff control, 0x55 poweroff */
#define ATM88PA_REG_PWR_CTRL	0x10 /* R/W */

/*
 * Interrupts bits:
 *   bit[7:6]: Reserved
 *   bit[5]  : System status interrupt flag, clear after read
 *   bit[4]  : Keys interrupt flag, clear after read
 *   bit[3:2]: Reserved
 *   bit[1]  : Enable system status interrupt, 1 enabled, 0 disabled
 *   bit[0]  : Enable keys interrupt, 1 enabled, 0 disabled.
 */
#define ATM88PA_REG_INT_CTRL	0x11 /* R/W */

/* Left camera power control, 1 on, 0 off */
#define ATM88PA_REG_LC_PWR_CTRL	0x12 /* R/W */

/* Right camera power control, 1 on, 0 off */
#define ATM88PA_REG_RC_PWR_CTRL	0x13 /* R/W */

/* Keypad light, 0 ~ 255 */
#define ATM88PA_REG_KEYPAD_LIGHT	0x0A /* R/W */

/*
 * Software version, 16 bit unsigned
 * NOTE:
 *   501 = Ver 5.01, for CVR-MIL-V2 devices
 *   600 = Ver 6.00, for CVR-MIL-V2-A devices
 *   100 = Ver 1.00, for CVR-MIL-V2-B devices
 */
#define ATM88PA_REG_SW_VER	0x20 /* R */

#endif /* MFD_ATM88PA_H */
