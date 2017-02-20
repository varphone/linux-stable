/*
 * atmega_kp.h - Configuration for ATMEGA keypad driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation (version 2 of the License only).
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef __LINUX_ATMEGA_KP_H
#define __LINUX_ATMEGA_KP_H

#include <linux/types.h>
#include <linux/ioctl.h>

/*
 * Largest keycode that the chip can send, plus one,
 * so keys can be mapped directly at the index of the
 * ATMEGA keycode instead of subtracting one.
 */

#define ATMEGA_IOCTL_BASE  		'A'

/* backlight: 0 for power off, 1 for power on */
#define ATMEGA_S_BACKLIGHT_ON  		_IOW(ATMEGA_IOCTL_BASE, 82, u8)

#define ATMEGA_RESET_FPGA  		_IO(ATMEGA_IOCTL_BASE, 88)

/* set backlight value, varies from 1 to 10 */
#define ATMEGA_S_BACKLIGHT_VALUE  	_IOW(ATMEGA_IOCTL_BASE, 89, u8)

#define ATMEGA_G_BACKLIGHT_VALUE  	_IOR(ATMEGA_IOCTL_BASE, 90, u8*)

#endif /* __LINUX_ATMEGA_KP_H */
