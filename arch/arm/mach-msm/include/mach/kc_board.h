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

#ifndef _KC_BOARD_H_
#define _KC_BIARD_H_

typedef enum {
    OEM_BOARD_JTAG_TYPE = 0,
    OEM_BOARD_WS1_TYPE  = 1,
    OEM_BOARD_WS2_TYPE  = OEM_BOARD_WS1_TYPE,
    OEM_BOARD_WS3_TYPE  = 3,
    OEM_BOARD_RESERVE_TYPE = 4,
} oem_board_type;


typedef enum {
    OEM_KCC_F41_TYPE = 0,
    OEM_KCC_F54_TYPE,
    OEM_KCC_F55_TYPE
} oem_model_type;

oem_board_type OEM_get_board(void);

oem_model_type OEM_get_modeltype(void);

#endif 
