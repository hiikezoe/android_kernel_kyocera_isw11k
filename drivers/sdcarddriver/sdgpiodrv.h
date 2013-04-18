/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
                               <sdgpiodrv.h>
DESCRIPTION
  Driver for GPIO wrapper of SD card control.

EXTERNALIZED FUNCTIONS
  Operete gpio.

This software is contributed or developed by KYOCERA Corporation.
(C) 2011 KYOCERA Corporation
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
/*
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307 USA
 */

#ifndef __SD_GPIO_DRV__
#define __SD_GPIO_DRV__


#define SDGDRV_DEVICE_NAME "kc_sdgdrv"


#define SDG_GET_VALUE   1 
#define SDG_SET_VALUE   2 
#define SDG_TO_IRQ      3 

typedef struct sdgdrv_if
{
    int gpiofuncid;
    unsigned gpio;
    int value;
} SDGDRV_IF;

#endif

