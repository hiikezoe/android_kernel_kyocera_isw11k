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
#ifndef ___ALPS_IO_H_INCLUDED
#define ___ALPS_IO_H_INCLUDED

#include <linux/ioctl.h>

#define ALPSIO   0xAF

#define ALPSIO_SET_MAGACTIVATE   _IOW(ALPSIO, 0, int)
#define ALPSIO_SET_ACCACTIVATE   _IOW(ALPSIO, 1, int)
#define ALPSIO_SET_DELAY         _IOW(ALPSIO, 2, int)

#define ALPSIO_MAG_I2C_READ      _IOR(ALPSIO, 3, int)
typedef struct {
  int32_t  m_nAdrs;
  char m_cReadData;
}IoctlMagI2cRead;

#define ALPSIO_MAG_I2C_WRITE     _IOW(ALPSIO, 4, int)
typedef struct {
  int32_t  m_nAdrs;
  char m_cWriteData;
}IoctlMagI2cWrite;

#define ALPSIO_MAG_GET           _IOR(ALPSIO, 5, int)
typedef struct {
  int32_t  m_naData[3];
}IoctlMagDataGet;

#define ALPSIO_MAG_DM_READ       _IOR(ALPSIO, 6, int)

#define MAG_DM_R_RAW_CMND              0x00
#define MAG_DM_R_RAW_OFST_CMND       0x01
#define MAG_DM_R_IMITATION_CMND        0x02
#define MAG_DM_R_IMITATION_OFST_CMND 0x03
#define MAG_DM_R_RAW_AZIMUTH_CMND      0x04
#define MAG_DM_R_RAW_INCRINATION_CMND  0x05
#define MAG_DM_R_EXECUTING_STATUS              0xF0

typedef struct {
  int32_t m_nCommand;
  int32_t m_naData[3];
}IoCtlMagDataDmGetCmd;

#define ALPSIO_MAG_DM_WRITE      _IOW(ALPSIO, 7, int)

#define MAG_DM_W_X_CMND                0x10
#define MAG_DM_W_Y_CMND                0x11
#define MAG_DM_W_Z_CMND                0x12
#define MAG_DM_W_X_OFST_CMND           0x13
#define MAG_DM_W_Y_OFST_CMND           0x14
#define MAG_DM_W_Z_OFST_CMND           0x15

#define MAG_DM_W_REL_X_CMND            0x20
#define MAG_DM_W_REL_Y_CMND            0x21
#define MAG_DM_W_REL_Z_CMND            0x22
#define MAG_DM_W_REL_OFST_X_CMND       0x23
#define MAG_DM_W_REL_OFST_Y_CMND       0x24
#define MAG_DM_W_REL_OFST_Z_CMND       0x25
#define MAG_DM_W_EXECUTING_STATUS      0xf0

typedef struct {
  int32_t m_nCommand;
  int32_t m_nData;
}IoCtlMagDataDmSetCmd;

#define ALPSIO_ACC_I2C_READ      _IOR(ALPSIO, 8, int)
typedef struct {
  int32_t  m_nAdrs;
  char m_cReadData;
}IoctlAccI2cRead;

#define ALPSIO_ACC_I2C_WRITE     _IOW(ALPSIO, 9, int)
typedef struct {
  int32_t  m_nAdrs;
  char m_cWriteData;
}IoctlAccI2cWrite;

#define ALPSIO_ACC_GET           _IOR(ALPSIO,10, int)
typedef struct {
  int32_t  m_naData[4];
}IoctlAccDataGet;

#define ALPSIO_ACC_DM_READ       _IOR(ALPSIO,11, int)

#define ACC_DM_R_RAW_CMND              0x00
#define ACC_DM_R_IMITATION_CMND        0x01

#define ACC_DM_R_CALIB_MODE_CMND       0x40
#define ACC_DM_R_CALIB_STRT_CMND       0x41
#define ACC_DM_R_CALIB_STTS_CMND       0x42

#define ACC_DM_R_PEDOMETER_ENABLE      0x50
#define ACC_DM_R_PEDOMETER_SET_PARAM   0x51
#define ACC_DM_R_PEDOMETER_GET_VALUE   0x52
#define ACC_DM_R_PEDOMETER_GET_TIME    0x53
#define ACC_DM_R_PEDOMETER_CLEAR       0x54

typedef struct {
  int32_t m_nCommand;
  int32_t m_naData[7];
}IoCtlAccDataDmGetCmd;

#define ALPSIO_ACC_DM_WRITE      _IOW(ALPSIO,12, int)

#define ALPSIO_ACC_SET_FREQ      _IOW(ALPSIO,13, int)

#define ALPSIO_ACC_SET_OFFSET  _IOW(ALPSIO,14, int)
typedef struct {
  int32_t m_naOffset[6];
}IoCtlAccSetOffset;

#define ALPSIO_ACC_SUS_RES  _IOW(ALPSIO,15, int)
#define ALPSIO_MAG_SUS_RES  _IOW(ALPSIO,16, int)

#define ALPSIO_INVALID           _IOW(ALPSIO,17, int)

#define ACC_DM_W_X_CMND                0x10
#define ACC_DM_W_Y_CMND                0x11
#define ACC_DM_W_Z_CMND                0x12
#define ACC_DM_W_PITCH_CMND            0x13
#define ACC_DM_W_ROLL_CMND             0x14

#define ACC_DM_W_REL_X_CMND        0x20
#define ACC_DM_W_REL_Y_CMND        0x21
#define ACC_DM_W_REL_Z_CMND        0x22
#define ACC_DM_W_REL_PITCH_CMND    0x23
#define ACC_DM_W_REL_ROLL_CMND     0x24

typedef struct {
  int32_t m_nCommand;
  int32_t m_nData;
}IoCtlAccDataDmSetCmd;
#endif
