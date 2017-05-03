/*
 * atmega_keypad.h - Configuration for ATMEGA keypad driver.
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

#ifndef __LINUX_ATMEGA_KEYPAD_H
#define __LINUX_ATMEGA_KEYPAD_H

#include <linux/types.h>
#include <linux/ioctl.h>

/*
 * Largest keycode that the chip can send, plus one,
 * so keys can be mapped directly at the index of the
 * ATMEGA keycode instead of subtracting one.
 */
#define ATMEGA_KEY_NUM		(5)
#define ATMEGA_KEYMAP_SIZE	(ATMEGA_KEY_NUM)


struct atmega_keypad_platform_data {
	int debounce_time; /* Time to watch for key bouncing, in ms. */
	int repeat;
	unsigned short keymap[ATMEGA_KEYMAP_SIZE];
};

#define	ATMEGA_IOCTL_BASE		'A'

/* temperature= temp value * 10, signed 16bit */
#define	ATMEGA_G_TEMPERATURE		_IOR(ATMEGA_IOCTL_BASE, 80, s16 *)

/* not using */
#define	ATMEGA_G_STATE			_IOR(ATMEGA_IOCTL_BASE, 81, u8 *)

/* obsoleted
#define	ATMEGA_G_TEMPERATURE_MODE	_IOW(ATMEGA_IOCTL_BASE, 88, u8 *)
*/

/* backlight: 0 for power off, 1 for power on */
#define	ATMEGA_S_BACKLIGHT_ON		_IOW(ATMEGA_IOCTL_BASE, 82, u8)

/* front camrea: 0 for power off, 1 for power on */
#define	ATMEGA_S_FRONTCAM_ON		_IOW(ATMEGA_IOCTL_BASE, 83, u8)

/* rear camrea: 0 for power off, 1 for power on */
#define	ATMEGA_S_REARCAM_ON		_IOW(ATMEGA_IOCTL_BASE, 84, u8)

/* 0 for rear camera, 1 for front camera */
#define	ATMEGA_S_CAM_LED		_IOW(ATMEGA_IOCTL_BASE, 85, u8)

/* 0 for normal, 1 for exception camera */
#define	ATMEGA_S_ALARM_LED		_IOW(ATMEGA_IOCTL_BASE, 86, u8)

/* obsolete
#define	ATMEGA_S_TEMPERATURE_MODE	_IOW(ATMEGA_IOCTL_BASE, 87, u8)
*/

/* set backlight value, varies from 1 to 10 */
#define	ATMEGA_S_BACKLIGHT_VALUE	_IOW(ATMEGA_IOCTL_BASE, 89, u8)

#endif /* __LINUX_ATMEGA_KEYPAD_H */
