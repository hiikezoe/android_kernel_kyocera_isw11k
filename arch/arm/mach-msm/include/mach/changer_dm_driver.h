/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 */
/*
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#ifndef CHANGER_DM_DRIVER_H
#define CHANGER_DM_DRIVER_H

#include <linux/ioctl.h>

#define CHANGER_DM_DRIVER_IOCTL_01                  0x10
#define CHANGER_DM_DRIVER_IOCTL_02                  0x11
#define CHANGER_DM_DRIVER_IOCTL_03                  0x12
#define CHANGER_DM_DRIVER_IOCTL_04                  0x13
#define CHANGER_DM_DRIVER_IOCTL_05                  0x14
#define CHANGER_DM_DRIVER_IOCTL_06                  0x15

#define CHANGER_DM_DRIVER_GET_SW_SET_MODE           0
#define CHANGER_DM_DRIVER_GET_DETECTION_ID_STATE    1
#define CHANGER_DM_DRIVER_GET_INTERRUPT_CASE        2
#define CHANGER_DM_DRIVER_GET_CHG_DET_MODE          3

extern u8 kc_changer_ic_get_dminfo(unsigned char cmd, int32_t *val);
extern u8 kc_changer_ic_set_chgdet_wait(int32_t *val);
extern u8 kc_hs_jack_cmd(int32_t *val);

#endif
