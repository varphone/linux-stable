/*	--*- c -*--
 * Copyright (C) 2012 Enrico Scholz <enrico.scholz@sigma-chemnitz.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 and/or 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef H_LINUX_PLATFORM_DATA_MT9V02X_H
#define H_LINUX_PLATFORM_DATA_MT9V02X_H

#define MT9V02X_FEATURE_MONO		(0u << 24)
#define MT9V02X_FEATURE_COLOR		(1u << 24)

#define MT9V02X_FEATURE_LVDS		(1u << 23)
#define MT9V02X_FEATURE_LVDS_10BIT	(1u << 26)

#define MT9V02X_FEATURE_SKIP_PROBE	(1u << 25)

#endif	/* H_LINUX_PLATFORM_DATA_MT9V02X_H */
