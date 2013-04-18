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
#ifndef GP2AP003A10F_H
#define GP2AP003A10F_H

#include <linux/ioctl.h>

#define D_MODE_ALS                  (0)
#define D_MODE_PROX                 (1)

#define D_ADDR_PROX     (0x00)
#define D_ADDR_GAIN     (0x01)
#define D_ADDR_HYS      (0x02)
#define D_ADDR_CYCLE    (0x03)
#define D_ADDR_OPMOD    (0x04)
#define D_ADDR_CON      (0x06)

#define D_PROX_DETECT_STATE_NON_DETECT      (0)
#define D_PROX_DETECT_STATE_DETECT          (1)

#define D_CMD_SENSOR_ACTIVE                 (0x00)
#define D_CMD_LIGHT_DATA_SET                (0x01)
#define D_CMD_CAM_TEMP_DEF_SET              (0x02)
#define D_CMD_LIGHT_CONV_DATA_SET           (0x03)

#define D_LIGHT_MILLI_VOLTAGE_MAX           (0x4476)

#define D_SENSOR_NEAR_SIZE                  (0x03)
#define D_SENSOR_FAR_SIZE                   (0x03)
#define D_SENSOR_DETECT_SIZE                (0x02)
#define D_SENSOR_PHOTO_SIZE                 (0x0A)
#define D_NV_DATA_MAX                       (0x0A+1)

typedef struct _t_gp2ap003a10f_ioctl_reg
{
    unsigned long ulAddress;
    unsigned long ulData;
}T_GP2AP003A10F_IOCTL_REG;

typedef struct _t_gp2ap003a10f_ioctl_sensor
{
    unsigned long ulSensor_mode;
    unsigned long ulData;
}T_GP2AP003A10F_IOCTL_SENSOR;

typedef struct _t_gp2ap003a10f_ioctl_command
{
    unsigned long ulCommand;
    unsigned long ulSensor_mode;
    unsigned long ulActive;
    unsigned long ulResponse;
    unsigned long ulData;
}T_GP2AP003A10F_IOCTL_COMMAND;

typedef struct _t_gp2ap003a10f_ioctl_nv
{
    unsigned long ulLength;
    unsigned char ucData[D_NV_DATA_MAX];
    unsigned long ulItem;
}T_GP2AP003A10F_IOCTL_NV;


#define GP2AP003A10F_IO             'G'

#define IOCTL_LIGHT_I2C_READ        _IOWR(GP2AP003A10F_IO, 0x01, T_GP2AP003A10F_IOCTL_REG)
#define IOCTL_PROXIMITY_I2C_READ    _IOWR(GP2AP003A10F_IO, 0x02, T_GP2AP003A10F_IOCTL_REG)
#define IOCTL_LIGHT_I2C_WRITE       _IOWR(GP2AP003A10F_IO, 0x03, T_GP2AP003A10F_IOCTL_SENSOR)
#define IOCTL_PROXIMITY_I2C_WRITE   _IOWR(GP2AP003A10F_IO, 0x04, T_GP2AP003A10F_IOCTL_SENSOR)
#define IOCTL_LIGHT_DATA_GET        _IOWR(GP2AP003A10F_IO, 0x05, T_GP2AP003A10F_IOCTL_SENSOR)
#define IOCTL_PROXIMITY_DATA_GET    _IOWR(GP2AP003A10F_IO, 0x06, T_GP2AP003A10F_IOCTL_SENSOR)
#define IOCTL_CLEAR_SENSOR_DATA     _IO(GP2AP003A10F_IO, 0x07)
#define IOCTL_PROXIMITY_COMMAND_SET _IOWR(GP2AP003A10F_IO, 0x08, T_GP2AP003A10F_IOCTL_COMMAND)
#define IOCTL_PROXIMITY_NV_DATA_SET _IOWR(GP2AP003A10F_IO, 0x09, T_GP2AP003A10F_IOCTL_NV)
#define IOCTL_PROXIMITY_NV_DATA_GET _IOWR(GP2AP003A10F_IO, 0x0A, T_GP2AP003A10F_IOCTL_NV)

enum {
    en_NV_PROXIMITY_SENSOR_NEAR = 0,
    en_NV_PROXIMITY_SENSOR_FAR,
    en_NV_PROXIMITY_DETECT,
    en_NV_PHOTO_SENSOR,
};
#endif
