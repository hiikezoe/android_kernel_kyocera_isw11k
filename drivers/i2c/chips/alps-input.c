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
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/input-polldev.h>

#include <asm/uaccess.h> 
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/i2c/alps_io.h>
#include <linux/delay.h>
#include <linux/i2c.h>

#include <linux/earlysuspend.h>

#define EVENT_TYPE_ACCEL_X          ABS_X
#define EVENT_TYPE_ACCEL_Y          ABS_Y
#define EVENT_TYPE_ACCEL_Z          ABS_Z

#define EVENT_TYPE_MAGV_X           ABS_HAT0X
#define EVENT_TYPE_MAGV_Y           ABS_HAT0Y
#define EVENT_TYPE_MAGV_Z           ABS_BRAKE

#define ALPS_POLL_INTERVAL  1000
#define ALPS_INPUT_FUZZ  0
#define ALPS_INPUT_FLAT  0

#define POLL_STOP_TIME 1000

#define DEFAULT_DELAY  200

#define EVENT_ACC_DATA_MAX 8192
#define EVENT_MAG_DATA_MAX 4096

#define ACC_DATA_NUM        4
#define ACC_OFFSET_NUM      6
#define MAG_DATA_NUM        3
#define PITCH_ROLL_DATA_NUM 2

#define CALIB_START_MODE    0
#define CALIB_START_RESULT  1

#define CALIB_WAIT          0
#define CALIB_FINISH        1

#define INDEX_X            0
#define INDEX_Y            1
#define INDEX_Z            2
#define INDEX_SUM          3
#define INDEX_PITCH        4
#define INDEX_ROLL         5

#define ACT_UPDATE_FALSE   0
#define ACT_UPDATE_TRUE    1
#define ACC_POWER_ON       1
#define ACC_POWER_OFF      0
#define MAG_POWER_ON       1
#define MAG_POWER_OFF      0

#define MODE_0             0
#define MODE_1             1
#define MODE_2             2
#define MODE_3             3

#define PEDOMETER_ENA_PARM 0
#define PEDOMETER_ENA_RET  1

#define PEDOMETER_SET_P1   0
#define PEDOMETER_SET_P2   1
#define PEDOMETER_SET_RET  2

#define PEDOMETER_GET_P1   0
#define PEDOMETER_GET_P2   1
#define PEDOMETER_GET_RET  2

#define BYTE_MASK          0xff
#define REGISTER_LENGTH    1
#define I2C_WRITE_SIZE_MIN 2
#define INDEX_REG          0
#define INDEX_VAL          1

#define EVENT_KIND_ACC     1
#define EVENT_KIND_MAG     2

static DEFINE_MUTEX(alps_lock);

static struct platform_device *pdev;
static struct input_polled_dev *alps_idev;

static int32_t flgM = 0, flgA = 0;
static int32_t delay = DEFAULT_DELAY;
static int32_t poll_stop_cnt = 0;
static bool poll_stop_sleep = false;


static bool g_bIsAlreadyExistAccImitationXData       = false;
static bool g_bIsAlreadyExistAccImitationYData       = false;
static bool g_bIsAlreadyExistAccImitationZData       = false;
static bool g_bIsAlreadyExistAccImitationXYZData     = false;
static int32_t  g_naAccImitationData[ACC_DATA_NUM];

static bool g_bIsAlreadyExistAccImitationPitchData   = false;
static int32_t  g_nAccImitationPitchData;

static bool g_bIsAlreadyExistAccImitationRollData    = false;
static int32_t  g_nAccImitationRollData;

static bool g_bIsAlreadyExistMagImitationXData       = false;
static bool g_bIsAlreadyExistMagImitationYData       = false;
static bool g_bIsAlreadyExistMagImitationZData       = false;
static int32_t  g_naMagImitationData[MAG_DATA_NUM];

static bool g_bIsAlreadyExistMagImitationOffsetXData = false;
static bool g_bIsAlreadyExistMagImitationOffsetYData = false;
static bool g_bIsAlreadyExistMagImitationOffsetZData = false;
static int32_t  g_naMagImitationOffsetData[MAG_DATA_NUM];
static int32_t  g_naMagSaveOffsetData[MAG_DATA_NUM];
static int32_t  g_nAccCalibrationMode = -1;

static bool g_bIsMagDmExecuting = false;

static int32_t g_nDMAccPower = 0;


extern int32_t hscd_i2c_readm(u8 *rxData, int32_t length);
extern int32_t hscd_i2c_writem(u8 *txData, int32_t length);
extern void hscd_activate(int32_t flgatm, int32_t flg, int32_t dtime);
extern int32_t hscd_get_magnetic_field_data(int32_t *xyz);
extern int32_t hscd_get_magnetic_field_offset_data(int32_t *xyz);
extern int32_t hscd_set_magnetic_field_offset_Xdata(int32_t arg_nOffsetData);
extern int32_t hscd_set_magnetic_field_offset_Ydata(int32_t arg_nOffsetData);
extern int32_t hscd_set_magnetic_field_offset_Zdata(int32_t arg_nOffsetData);

extern int32_t accsns_i2c_readm(u8* arg_ucpReadData,int32_t arg_nLength );
extern int32_t accsns_i2c_writem(u8* arg_ucpWriteData,int32_t arg_nLength);
extern void accsns_activate(int32_t flgatm, int32_t flg);
extern int32_t accsns_get_acceleration_pitch_and_roll_data(int* ipData);
extern int32_t accsns_get_acceleration_data(int* ipXYZ);
extern void accsns_set_freq(uint8_t freq);
extern void accsns_set_offset(int32_t* offsets);
extern int32_t accsns_calibration_mode(void);
extern int32_t accsns_calibration_start(int32_t argMode);
extern int32_t accsns_calibration_is_wait(int32_t* argCal);

extern int32_t accsns_pedometer_get_time(void);
extern int32_t accsns_pedometer_clear(void);
extern int32_t accsns_pedometer_set_param(int32_t nStride, int32_t nWeight);
extern int32_t accsns_pedometer_enable(bool bEnable);
extern int32_t accsns_pedometer_get_value(int32_t* nSteps, int32_t* nCal);

extern void accsns_set_delay(int32_t delay);

extern int32_t accsns_get_i2c_err(void);
extern void accsns_error_reset(void);

extern int32_t accsns_resume (struct i2c_client *client);
extern int32_t accsns_suspend(struct i2c_client *client,pm_message_t mesg);
extern int32_t hscd_resume (struct i2c_client *client);
extern int32_t hscd_suspend(struct i2c_client *client,pm_message_t mesg);

static void alps_early_suspend(struct early_suspend *h);
static void alps_late_resume(struct early_suspend *h);

static void accsns_err_check(void)
{
  if (accsns_get_i2c_err() > 0) {
    accsns_error_reset();
  }
}

static int32_t accsns_dm_write
(
 IoCtlAccDataDmSetCmd* arg_stpDmWriteCommand
 )
{
  int32_t nRet = 0;
  
  switch (arg_stpDmWriteCommand->m_nCommand) {
  case ACC_DM_W_X_CMND:
    g_bIsAlreadyExistAccImitationXData = true;
    g_naAccImitationData[INDEX_X] = arg_stpDmWriteCommand->m_nData;
    break;
  case ACC_DM_W_Y_CMND:
    g_bIsAlreadyExistAccImitationYData = true;
    g_naAccImitationData[INDEX_Y] = arg_stpDmWriteCommand->m_nData;
    break;
  case ACC_DM_W_Z_CMND:
    g_bIsAlreadyExistAccImitationZData = true;
    g_naAccImitationData[INDEX_Z] = arg_stpDmWriteCommand->m_nData;
    break;
  case ACC_DM_W_PITCH_CMND:
    g_bIsAlreadyExistAccImitationPitchData = true;
    g_nAccImitationPitchData = arg_stpDmWriteCommand->m_nData;
    break;
  case ACC_DM_W_ROLL_CMND:
    g_bIsAlreadyExistAccImitationRollData = true;
    g_nAccImitationRollData = arg_stpDmWriteCommand->m_nData;
    break;
  case ACC_DM_W_REL_X_CMND:
    g_bIsAlreadyExistAccImitationXData = false;
    break;
  case ACC_DM_W_REL_Y_CMND:
    g_bIsAlreadyExistAccImitationYData = false;
    break;
  case ACC_DM_W_REL_Z_CMND:
    g_bIsAlreadyExistAccImitationZData = false;
    break;
  case ACC_DM_W_REL_PITCH_CMND:
    g_bIsAlreadyExistAccImitationPitchData = false;
    break;
  case ACC_DM_W_REL_ROLL_CMND:
    g_bIsAlreadyExistAccImitationRollData = false;
    break;
  default:
    printk("DM Command Error!! m_nCommand = 0x%02x\n",
           arg_stpDmWriteCommand->m_nCommand);
    nRet = -1;
    break;
  }
#ifdef ALPS_DEBUG
  printk("Imitation x:(%d)%03x y:(%d)%03x z:(%d)%03x p:(%d)%03x r:(%d)%03x\n",
         g_bIsAlreadyExistAccImitationXData, g_naAccImitationData[INDEX_X],
         g_bIsAlreadyExistAccImitationYData, g_naAccImitationData[INDEX_Y],
         g_bIsAlreadyExistAccImitationZData, g_naAccImitationData[INDEX_Z],
         g_bIsAlreadyExistAccImitationPitchData, g_nAccImitationPitchData,
         g_bIsAlreadyExistAccImitationRollData, g_nAccImitationRollData);
#endif
  return nRet;
}

static int32_t accsns_dm_read
(
 IoCtlAccDataDmGetCmd* arg_stpDmReadCommand
 )
{
  int32_t nRet = -1;
  
  switch (arg_stpDmReadCommand->m_nCommand) {
  case ACC_DM_R_RAW_CMND:
    if(flgA++ == 0) {
      accsns_activate(ACT_UPDATE_FALSE, ACC_POWER_ON);
      mutex_unlock(&alps_lock);
      msleep(delay + delay);
      mutex_lock(&alps_lock);
    }
    nRet = accsns_get_acceleration_data(arg_stpDmReadCommand->m_naData);
#ifdef ALPS_DEBUG
    if(nRet) {
      printk("accsns_get_acceleratio_data() return error\n");
    }
#endif
    if(nRet == 0) {
      nRet = accsns_get_acceleration_pitch_and_roll_data(&arg_stpDmReadCommand->m_naData[ACC_DATA_NUM]);
#ifdef ALPS_DEBUG
      if(nRet) {
        printk("accsns_get_acceleratio_data() return error\n");
      }
#endif
    }
    if(--flgA == 0) {
      accsns_activate(ACT_UPDATE_FALSE, ACC_POWER_OFF);
    }
    break;
    
  case ACC_DM_R_IMITATION_CMND:
    {
      int32_t naData[ACC_DATA_NUM+PITCH_ROLL_DATA_NUM];
      
      memcpy(arg_stpDmReadCommand->m_naData,
             g_naAccImitationData,
             sizeof(g_naAccImitationData));
      
      arg_stpDmReadCommand->m_naData[ACC_DATA_NUM] = g_nAccImitationPitchData;
      arg_stpDmReadCommand->m_naData[ACC_DATA_NUM+1] = g_nAccImitationRollData;
      
      if((g_bIsAlreadyExistAccImitationXData == false) ||
         (g_bIsAlreadyExistAccImitationYData == false) ||
         (g_bIsAlreadyExistAccImitationZData == false) ||
         (g_bIsAlreadyExistAccImitationXYZData == false)) {
        
        if(flgA++ == 0) {
          accsns_activate(ACT_UPDATE_FALSE, ACC_POWER_ON);
        }
        nRet = accsns_get_acceleration_data(naData);
        if(nRet == 0) {
          nRet = accsns_get_acceleration_pitch_and_roll_data(&naData[ACC_DATA_NUM]);
        }
        if(--flgA == 0) {
          accsns_activate(ACT_UPDATE_FALSE, ACC_POWER_OFF);
        }
        
        if(nRet == 0) {
          if(g_bIsAlreadyExistAccImitationXData == false) {
            arg_stpDmReadCommand->m_naData[INDEX_X] = naData[INDEX_X];
          }
          if(g_bIsAlreadyExistAccImitationYData == false) {
            arg_stpDmReadCommand->m_naData[INDEX_Y] = naData[INDEX_Y];
          }
          if(g_bIsAlreadyExistAccImitationZData == false) {
            arg_stpDmReadCommand->m_naData[INDEX_Z] = naData[INDEX_Z];
          }
          if(g_bIsAlreadyExistAccImitationXYZData == false) {
            arg_stpDmReadCommand->m_naData[INDEX_SUM] = naData[INDEX_SUM];
          }
          if(g_bIsAlreadyExistAccImitationPitchData == false) {
            arg_stpDmReadCommand->m_naData[INDEX_PITCH] = naData[INDEX_PITCH];
          }
          if(g_bIsAlreadyExistAccImitationRollData == false) {
            arg_stpDmReadCommand->m_naData[INDEX_ROLL] = naData[INDEX_ROLL];
          }
        }
      }
    }
    break;
  case ACC_DM_R_CALIB_MODE_CMND:
  {
    int32_t iRet = 0;
    iRet = accsns_calibration_mode();
    arg_stpDmReadCommand->m_naData[0] = iRet;
    nRet = 0;
    break;
  }
  case ACC_DM_R_CALIB_STRT_CMND:
  {
    int32_t iRet = 0;
    int32_t naData[ACC_OFFSET_NUM];
    iRet = accsns_calibration_is_wait(naData);

    if (iRet == 0) {
      nRet = 1;
      arg_stpDmReadCommand->m_naData[CALIB_START_RESULT] = 0xff;
    } else {
      if(g_nDMAccPower++ == 0) {
        if(flgA++ == 0) {
          accsns_activate(ACT_UPDATE_FALSE, ACC_POWER_ON);
        }
      }
      iRet = accsns_calibration_start(arg_stpDmReadCommand->m_naData[CALIB_START_MODE]);
      arg_stpDmReadCommand->m_naData[CALIB_START_RESULT] = iRet;
      g_nAccCalibrationMode = arg_stpDmReadCommand->m_naData[CALIB_START_MODE];
      if (iRet != 0) {
        if (--g_nDMAccPower == 0) {
          if (--flgA == 0) {
            accsns_activate(ACT_UPDATE_FALSE,ACC_POWER_ON);
          } else {
            if (flgA < 0) {
              printk("ACC Power state error %d\n", flgA);
              flgA = 0;
            }
          }
        } else {
          if (g_nDMAccPower < 0) {
            printk("g_nDMAccPower state error %d\n", g_nDMAccPower);
            g_nDMAccPower = 0;
          }
        }
      }
      nRet = 0;
    }
    printk("ACC POWER1:%d\n", flgA);
    break;
  }
  case ACC_DM_R_CALIB_STTS_CMND:
  {
    int32_t naData[ACC_OFFSET_NUM];
    int32_t iRet = 0;
    iRet = accsns_calibration_is_wait(naData);
    if (iRet == 0) {
      arg_stpDmReadCommand->m_naData[0] = CALIB_WAIT;
    } else {
      int32_t nIdxRes  = 0;
      int32_t nIdxRead = 0;
      arg_stpDmReadCommand->m_naData[nIdxRes++] = CALIB_FINISH;
      
      arg_stpDmReadCommand->m_naData[nIdxRes++] = naData[nIdxRead++];
      arg_stpDmReadCommand->m_naData[nIdxRes++] = naData[nIdxRead++];
      arg_stpDmReadCommand->m_naData[nIdxRes++] = naData[nIdxRead++];
      arg_stpDmReadCommand->m_naData[nIdxRes++] = naData[nIdxRead++];
      arg_stpDmReadCommand->m_naData[nIdxRes++] = naData[nIdxRead++];
      arg_stpDmReadCommand->m_naData[nIdxRes] = naData[nIdxRead];

      switch(g_nAccCalibrationMode) {
      case MODE_0:
      case MODE_1:
      case MODE_2:
      case MODE_3:
        if (--g_nDMAccPower == 0) {
          if (--flgA == 0) {
            accsns_activate(ACT_UPDATE_FALSE,ACC_POWER_OFF);
          } else {
            if (flgA < 0) {
              printk("ACC Power state error %d\n", flgA);
              flgA = 0;
            }
          }
        } else {
          if (g_nDMAccPower < 0) {
            printk("g_nDMAccPower state error %d\n", g_nDMAccPower);
            g_nDMAccPower = 0;
          }
        }
        break;
      default:
        printk("AccCalib: calibration mode err %d\n", g_nAccCalibrationMode);
        break;        
      }
    }
    nRet = 0;
    printk("ACC POWER2:%d\n", flgA);
    break;
  }
  case ACC_DM_R_PEDOMETER_ENABLE:
  {
    int32_t iRet = 0;
    bool bEnable = (arg_stpDmReadCommand->m_naData[0] != 0);
    printk("Pedometer enable %d\n", bEnable);

    if (bEnable) {
      if(g_nDMAccPower++ == 0) {
        if(flgA++ == 0) {
          accsns_activate(ACT_UPDATE_FALSE, ACC_POWER_ON);
        }
      }
    } else {
      if (--g_nDMAccPower == 0) {
        if (--flgA == 0) {
          accsns_activate(ACT_UPDATE_FALSE,ACC_POWER_OFF);
        } else {
          if (flgA < 0) {
            printk("ACC Power state error %d\n", flgA);
            flgA = 0;
          }
        }
      } else {
        if (g_nDMAccPower < 0) {
          printk("g_nDMAccPower state error %d\n", g_nDMAccPower);
          g_nDMAccPower = 0;
        }
      }
    }
    iRet = accsns_pedometer_enable(bEnable);
    arg_stpDmReadCommand->m_naData[PEDOMETER_ENA_RET] = iRet;
    nRet = 0;
    break;
  }
  case ACC_DM_R_PEDOMETER_SET_PARAM:
  {
    int32_t iRet = 0;
    printk("Pedometer set param\n");
    iRet = accsns_pedometer_set_param(
      arg_stpDmReadCommand->m_naData[PEDOMETER_SET_P1],
      arg_stpDmReadCommand->m_naData[PEDOMETER_SET_P2]
    );
    arg_stpDmReadCommand->m_naData[PEDOMETER_SET_RET] = iRet;
    nRet = 0;
    break;
  }
  case ACC_DM_R_PEDOMETER_GET_VALUE:
  {
    int32_t iRet = 0;
    printk("Pedometer get value\n");
    nRet = accsns_pedometer_get_value(
      &(arg_stpDmReadCommand->m_naData[PEDOMETER_GET_P1]),
      &(arg_stpDmReadCommand->m_naData[PEDOMETER_GET_P2])
    );
    arg_stpDmReadCommand->m_naData[PEDOMETER_GET_RET] = iRet;
    nRet = 0;
    break;
  }
  case ACC_DM_R_PEDOMETER_GET_TIME:
  {
    printk("Pedometer get time\n");
    arg_stpDmReadCommand->m_naData[0] = accsns_pedometer_get_time();
    nRet = 0;
    break;
  }
  case ACC_DM_R_PEDOMETER_CLEAR:
  {
    int32_t iRet = 0;
    printk("Pedometer clear\n");
    iRet = accsns_pedometer_clear();
    arg_stpDmReadCommand->m_naData[0] = iRet;
    nRet = 0;
    break;
  }
  default:
#ifdef ALPS_DEBUG
    printk("command errror!! iCommand = 0x%x\n",arg_stpDmReadCommand->m_nCommand);
#endif
    break;
  }
#ifdef ALPS_DEBUG
  printk("Imitation x:(%d)%03x y:(%d)%03x z:(%d)%03x p:(%d)%03x r:(%d)%03x\n",
         g_bIsAlreadyExistAccImitationXData, g_naAccImitationData[INDEX_X],
         g_bIsAlreadyExistAccImitationYData, g_naAccImitationData[INDEX_Y],
         g_bIsAlreadyExistAccImitationZData, g_naAccImitationData[INDEX_Z],
         g_bIsAlreadyExistAccImitationPitchData, g_nAccImitationPitchData,
         g_bIsAlreadyExistAccImitationRollData, g_nAccImitationRollData);
#endif
  return nRet;
}

static int32_t hscd_dm_write
(
 IoCtlMagDataDmSetCmd* arg_stpDmWriteCommand
 )
{
  int32_t nRet = 0;
  int32_t iaOffsetData[MAG_DATA_NUM];
  
  switch (arg_stpDmWriteCommand->m_nCommand) {
  case MAG_DM_W_X_CMND:
    g_bIsAlreadyExistMagImitationXData = true;
    g_naMagImitationData[INDEX_X] = arg_stpDmWriteCommand->m_nData;
    break;
  case MAG_DM_W_Y_CMND:
    g_bIsAlreadyExistMagImitationYData = true;
    g_naMagImitationData[INDEX_Y] = arg_stpDmWriteCommand->m_nData;
    break;
  case MAG_DM_W_Z_CMND:
    g_bIsAlreadyExistMagImitationZData = true;
    g_naMagImitationData[INDEX_Z] = arg_stpDmWriteCommand->m_nData;
    break;
  case MAG_DM_W_X_OFST_CMND:
    if(g_bIsAlreadyExistMagImitationOffsetXData == false) {
      g_bIsAlreadyExistMagImitationOffsetXData = true;
      hscd_get_magnetic_field_offset_data(&iaOffsetData[0]);
      g_naMagSaveOffsetData[INDEX_X] = iaOffsetData[INDEX_X];
    }
    g_naMagImitationOffsetData[INDEX_X] = arg_stpDmWriteCommand->m_nData;
    hscd_set_magnetic_field_offset_Xdata(g_naMagImitationOffsetData[INDEX_X]);
    break;
  case MAG_DM_W_Y_OFST_CMND:
    if(g_bIsAlreadyExistMagImitationOffsetYData == false) {
      g_bIsAlreadyExistMagImitationOffsetYData = true;
      hscd_get_magnetic_field_offset_data(&iaOffsetData[0]);
      g_naMagSaveOffsetData[INDEX_Y] = iaOffsetData[INDEX_Y];
    }
    g_naMagImitationOffsetData[INDEX_Y] = arg_stpDmWriteCommand->m_nData;
    hscd_set_magnetic_field_offset_Ydata(g_naMagImitationOffsetData[INDEX_Y]);
    break;
  case MAG_DM_W_Z_OFST_CMND:
    if(g_bIsAlreadyExistMagImitationOffsetZData == false) {
      g_bIsAlreadyExistMagImitationOffsetZData = true;
      hscd_get_magnetic_field_offset_data(&iaOffsetData[0]);
      g_naMagSaveOffsetData[INDEX_Z] = iaOffsetData[INDEX_Z];
    }
    g_naMagImitationOffsetData[INDEX_Z] = arg_stpDmWriteCommand->m_nData;
    hscd_set_magnetic_field_offset_Zdata(g_naMagImitationOffsetData[INDEX_Z]);
    break;
  case MAG_DM_W_REL_X_CMND:
    g_bIsAlreadyExistMagImitationXData = false;
    break;
  case MAG_DM_W_REL_Y_CMND:
    g_bIsAlreadyExistMagImitationYData = false;
    break;
  case MAG_DM_W_REL_Z_CMND:
    g_bIsAlreadyExistMagImitationZData = false;
    break;
  case MAG_DM_W_REL_OFST_X_CMND:
    if(g_bIsAlreadyExistMagImitationOffsetXData == true) {
      hscd_set_magnetic_field_offset_Xdata(g_naMagSaveOffsetData[INDEX_X]);
    }
    g_bIsAlreadyExistMagImitationOffsetXData = false;
    break;
  case MAG_DM_W_REL_OFST_Y_CMND:
    if(g_bIsAlreadyExistMagImitationOffsetYData == true) {
      hscd_set_magnetic_field_offset_Ydata(g_naMagSaveOffsetData[INDEX_Y]);
    }
    g_bIsAlreadyExistMagImitationOffsetYData = false;
    break;
  case MAG_DM_W_REL_OFST_Z_CMND:
    if(g_bIsAlreadyExistMagImitationOffsetZData == true) {
      hscd_set_magnetic_field_offset_Zdata(g_naMagSaveOffsetData[INDEX_Z]);
    }
    g_bIsAlreadyExistMagImitationOffsetZData = false;
    break;
  case MAG_DM_W_EXECUTING_STATUS:
    g_bIsMagDmExecuting = (bool)arg_stpDmWriteCommand->m_nData;
    break;
  default:
    nRet = -1;
    break;
  }
#ifdef ALPS_DEBUG
  printk("Imitation x:(%d)%03x y:(%d)%03x z:(%d)%03x ox:(%d)%03x oy:(%d)%03x oz:(%d)%03x\n",
         g_bIsAlreadyExistMagImitationXData,
         g_naMagImitationData[INDEX_X],
         g_bIsAlreadyExistMagImitationYData,
         g_naMagImitationData[INDEX_Y],
         g_bIsAlreadyExistMagImitationZData,
         g_naMagImitationData[INDEX_Z],
         g_bIsAlreadyExistMagImitationOffsetXData,
         g_naMagImitationOffsetData[INDEX_X],
         g_bIsAlreadyExistMagImitationOffsetYData,
         g_naMagImitationOffsetData[INDEX_Y],
         g_bIsAlreadyExistMagImitationOffsetZData,
         g_naMagImitationOffsetData[INDEX_Z]);
#endif
  return nRet;
}

static int32_t hscd_dm_read
(
 IoCtlMagDataDmGetCmd* arg_stpDmReadCommand
 )
{
  int32_t nRet = -1;
  
  switch (arg_stpDmReadCommand->m_nCommand) {
  case MAG_DM_R_RAW_CMND:
    if(flgM++ == 0) {
      hscd_activate(ACT_UPDATE_FALSE, MAG_POWER_ON, delay);
      mutex_unlock(&alps_lock);
      msleep(delay + delay);
      mutex_lock(&alps_lock);
    }
    nRet = hscd_get_magnetic_field_data(arg_stpDmReadCommand->m_naData);
    if(--flgM == 0) {
      hscd_activate(ACT_UPDATE_FALSE, MAG_POWER_OFF, delay);
    }
    break;
    
  case MAG_DM_R_RAW_OFST_CMND:
    nRet = hscd_get_magnetic_field_offset_data(arg_stpDmReadCommand->m_naData);
    break;
    
  case MAG_DM_R_IMITATION_CMND:
    {
      int32_t naData[MAG_DATA_NUM];
     
      nRet = 0;
      memcpy(arg_stpDmReadCommand->m_naData,
             g_naMagImitationData,
             sizeof(g_naMagImitationData));
      
      if((g_bIsAlreadyExistMagImitationXData == false) ||
         (g_bIsAlreadyExistMagImitationYData == false) ||
         (g_bIsAlreadyExistMagImitationZData == false)) {
        
        if(flgM++ == 0) {
          hscd_activate(ACT_UPDATE_FALSE, MAG_POWER_ON, delay);
          mutex_unlock(&alps_lock);
          msleep(delay + delay);
          mutex_lock(&alps_lock);
        }
        nRet = hscd_get_magnetic_field_data(naData);
        if(--flgM == 0) {
          hscd_activate(ACT_UPDATE_FALSE, MAG_POWER_OFF, delay);
        }
        
        if(nRet == 0) {
          if(g_bIsAlreadyExistMagImitationXData == false) {
            arg_stpDmReadCommand->m_naData[INDEX_X] = naData[INDEX_X];
          }
          if(g_bIsAlreadyExistMagImitationYData == false) {
            arg_stpDmReadCommand->m_naData[INDEX_Y] = naData[INDEX_Y];
          }
          if(g_bIsAlreadyExistMagImitationZData == false) {
            arg_stpDmReadCommand->m_naData[INDEX_Z] = naData[INDEX_Z];
          }
        }
      }
    }
    break;
    
  case MAG_DM_R_IMITATION_OFST_CMND:
    {
      int32_t iaOffsetData[MAG_DATA_NUM];
      
      nRet = 0;
      memcpy(arg_stpDmReadCommand->m_naData,
             g_naMagImitationOffsetData,
             sizeof(g_naMagImitationOffsetData));
      
      if((g_bIsAlreadyExistMagImitationOffsetXData == false) ||
         (g_bIsAlreadyExistMagImitationOffsetYData == false) ||
         (g_bIsAlreadyExistMagImitationOffsetZData == false)) {
        
        nRet = hscd_get_magnetic_field_offset_data(iaOffsetData);
        
        if(nRet == 0) {
          if(g_bIsAlreadyExistMagImitationOffsetXData == false) {
            arg_stpDmReadCommand->m_naData[INDEX_X] = iaOffsetData[INDEX_X];
          }
          if(g_bIsAlreadyExistMagImitationOffsetYData == false) {
            arg_stpDmReadCommand->m_naData[INDEX_Y] = iaOffsetData[INDEX_Y];
          }
          if(g_bIsAlreadyExistMagImitationOffsetZData == false) {
            arg_stpDmReadCommand->m_naData[INDEX_Z] = iaOffsetData[INDEX_Z];
          }
        }
      }
    }
    break;
  case MAG_DM_R_EXECUTING_STATUS:
    arg_stpDmReadCommand->m_naData[0] = (int32_t)g_bIsMagDmExecuting;
    nRet = 0;
    break;
  case MAG_DM_R_RAW_AZIMUTH_CMND:
  case MAG_DM_R_RAW_INCRINATION_CMND:
  default:
    break;
  }
#ifdef ALPS_DEBUG
  printk("Imitation x:(%d)%03x y:(%d)%03x z:(%d)%03x ox:(%d)%03x oy:(%d)%03x oz:(%d)%03x\n",
         g_bIsAlreadyExistMagImitationXData,
         g_naMagImitationData[INDEX_X],
         g_bIsAlreadyExistMagImitationYData,
         g_naMagImitationData[INDEX_Y],
         g_bIsAlreadyExistMagImitationZData,
         g_naMagImitationData[INDEX_Z],
         g_bIsAlreadyExistMagImitationOffsetXData,
         g_naMagImitationOffsetData[INDEX_X],
         g_bIsAlreadyExistMagImitationOffsetYData,
         g_naMagImitationOffsetData[INDEX_Y],
         g_bIsAlreadyExistMagImitationOffsetZData,
         g_naMagImitationOffsetData[INDEX_Z]);
#endif
  return nRet;
}

static int32_t alps_ioctl(struct inode* inode, struct file* filp, uint32_t cmd, unsigned long arg)
{
  void __user *argp = (void __user *)arg;
  int32_t ret = -1, tmpval = 0;
  
  switch (cmd) {
  case ALPSIO_SET_MAGACTIVATE:
    ret = copy_from_user(&tmpval, argp, sizeof(tmpval));
    if (ret) {
      printk("error : alps_ioctl(cmd = ALPSIO_SET_MAGACTIVATE)\n" );
      return -EFAULT;
    }
#ifdef ALPS_DEBUG
    printk("alps_ioctl(cmd = ALPSIO_SET_MAGACTIVATE), flgM = %d\n", tmpval);
#endif
    mutex_lock(&alps_lock);
    flgM += (tmpval != 0)? 1: -1;
    tmpval = (flgM == 0)? MAG_POWER_OFF : MAG_POWER_ON;
    hscd_activate(ACT_UPDATE_TRUE, tmpval, delay);
    mutex_unlock(&alps_lock);
    break;
    
  case ALPSIO_SET_ACCACTIVATE:
    ret = copy_from_user(&tmpval, argp, sizeof(tmpval));
    if (ret) {
      printk("error : alps_ioctl(cmd = ALPSIO_SET_ACCACTIVATE)\n");
      return -EFAULT;
    }
#ifdef ALPS_DEBUG
    printk("alps_ioctl(cmd = ALPSIO_SET_ACCACTIVATE), flgA = %d\n", tmpval);
#endif
    mutex_lock(&alps_lock);
    flgA += (tmpval != 0)? 1: -1;
    tmpval = (flgA == 0)? ACC_POWER_OFF : ACC_POWER_ON;
    accsns_activate(ACT_UPDATE_TRUE, tmpval);
    accsns_err_check();
    mutex_unlock(&alps_lock);
    break;
    
  case ALPSIO_SET_DELAY:
    ret = copy_from_user(&tmpval, argp, sizeof(tmpval));
    if (ret) {
      printk( "error : alps_ioctl(cmd = ALPSIO_SET_DELAY)\n" );
      return -EFAULT;
    }
#ifdef ALPS_DEBUG
    printk("alps_ioctl(cmd = ALPSIO_SET_DELAY)\n");
#endif
    if      (tmpval <=  10) tmpval =  10;
    else if (tmpval <=  20) tmpval =  20;
    else if (tmpval <=  60) tmpval =  50;
    else if (tmpval <= 100) tmpval = 100;
    else if (tmpval <= 200) tmpval = 200;
    else                    tmpval =1000;
    mutex_lock(&alps_lock);
    delay = tmpval;
    poll_stop_cnt = POLL_STOP_TIME / tmpval;
    hscd_activate(1, flgM, delay);
    accsns_set_delay(delay);
    mutex_unlock(&alps_lock);
#ifdef ALPS_DEBUG
    printk("     delay = %d\n", delay);
#endif
    break;
    
  case ALPSIO_MAG_I2C_READ:
    {
      IoctlMagI2cRead stMagI2cRead;
      ret = copy_from_user(&stMagI2cRead, argp, sizeof(stMagI2cRead));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_MAG_I2C_READ)\n" );
        return -EFAULT;
      }
      stMagI2cRead.m_cReadData = (char)(stMagI2cRead.m_nAdrs & BYTE_MASK);
      
      mutex_lock(&alps_lock);
      ret = hscd_i2c_readm(&stMagI2cRead.m_cReadData,REGISTER_LENGTH);
      mutex_unlock(&alps_lock);
      
      if(ret) {
        printk( "error(hscd_i2c_readm) : alps_ioctl(cmd = ALPSIO_MAG_I2C_READ)\n" );
        return -EFAULT;
      }
      ret = copy_to_user(argp,&stMagI2cRead,  sizeof(stMagI2cRead));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_MAG_I2C_READ)\n" );
        return -EFAULT;
      }
    }
    break;
    
  case ALPSIO_MAG_I2C_WRITE:
    {
      IoctlMagI2cWrite stMagI2cWrite = {0};
      char caWriteBuffer[I2C_WRITE_SIZE_MIN] = {0};
      
      ret = copy_from_user(&stMagI2cWrite, argp, sizeof(stMagI2cWrite));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_MAG_I2C_WRITE)\n" );
        return -EFAULT;
      }
      caWriteBuffer[INDEX_REG] = (char)(stMagI2cWrite.m_nAdrs & BYTE_MASK);
      caWriteBuffer[INDEX_VAL] = stMagI2cWrite.m_cWriteData;
      
      mutex_lock(&alps_lock);
      ret = hscd_i2c_writem(caWriteBuffer,I2C_WRITE_SIZE_MIN);
      mutex_unlock(&alps_lock);
      
      if(ret) {
        printk( "error(hscd_i2c_writem) : alps_ioctl(cmd = ALPSIO_MAG_I2C_WRITE)\n" );
        return -EFAULT;
      }
    }
    break;
    
  case ALPSIO_MAG_GET:
    {
      IoctlMagDataGet stMagData;
      memset(&stMagData, 0x00, sizeof(stMagData));
      
      mutex_lock(&alps_lock);
      if(flgM++ == 0) {
        hscd_activate(ACT_UPDATE_FALSE, MAG_POWER_ON, delay);
        mutex_unlock(&alps_lock);
        msleep(delay + delay);
        mutex_lock(&alps_lock);
      }
      ret = hscd_get_magnetic_field_data(&stMagData.m_naData[0]);
      if(--flgM == 0) {
        hscd_activate(ACT_UPDATE_FALSE, MAG_POWER_OFF, delay);
      }
#ifdef ALPS_DEBUG
      if(!ret)
        {
          printk("Mag_I2C, x:%d, y:%d, z:%d\n",stMagData.m_naData[INDEX_X], stMagData.m_naData[INDEX_Y], stMagData.m_naData[INDEX_Z]);
        }
#endif
      mutex_unlock(&alps_lock);
      
      if (ret) {
        printk( "error(hscd_get_magnetic_field_data) : alps_ioctl(cmd = ALPSIO_MAG_GET)\n" );
        return -EFAULT;
      }
      ret = copy_to_user(argp,&stMagData,  sizeof(stMagData));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_MAG_GET)\n" );
        return -EFAULT;
      }
    }
    break;
    
  case ALPSIO_MAG_DM_READ:
    {
      IoCtlMagDataDmGetCmd stDmReadCommand = {0};
      
      ret = copy_from_user(&stDmReadCommand, argp, sizeof(stDmReadCommand));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_MAG_DM_READ)\n" );
        return -EFAULT;
      }
#ifdef ALPS_DEBUG
      printk("[MAG]stDmReadCommand.m_nCommand=0x%x\n",stDmReadCommand.m_nCommand);
      printk("[MAG]stDmReadCommand.m_naData[0]=0x%x\n",stDmReadCommand.m_naData[0]);
#endif
      mutex_lock(&alps_lock);
      ret = hscd_dm_read(&stDmReadCommand);
      mutex_unlock(&alps_lock);
      
      if(ret) {
        printk( "error(hscd_dm_read) : alps_ioctl(cmd = ALPSIO_MAG_DM_READ)\n" );
        return -EFAULT;
      }
      ret = copy_to_user(argp,&stDmReadCommand,  sizeof(stDmReadCommand));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_MAG_DM_READ)\n" );
        return -EFAULT;
      }
    }
    break;
    
  case ALPSIO_MAG_DM_WRITE:
    {
      IoCtlMagDataDmSetCmd stDmWriteCommand = {0};
      
      ret = copy_from_user(&stDmWriteCommand, argp, sizeof(stDmWriteCommand));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_MAG_DM_WRITE)\n" );
        return -EFAULT;
      }
#ifdef ALPS_DEBUG
      printk("[MAG]stDmWriteCommand.m_nCommand=0x%x\n",stDmWriteCommand.m_nCommand);
      printk("[MAG]stDmWriteCommand.m_nData=0x%x\n",stDmWriteCommand.m_nData);
#endif
      mutex_lock(&alps_lock);
      ret = hscd_dm_write(&stDmWriteCommand);
      mutex_unlock(&alps_lock);
      
      if(ret) {
        printk( "error(hscd_dm_write) : alps_ioctl(cmd = ALPSIO_MAG_DM_WRITE)\n" );
        return -EFAULT;
      }
    }
    break;
    
  case ALPSIO_ACC_I2C_READ:
    {
      IoctlAccI2cRead stAccI2cRead = {0};
      ret = copy_from_user(&stAccI2cRead, argp, sizeof(stAccI2cRead));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_ACC_I2C_READ)\n" );
        return -EFAULT;
      }
      stAccI2cRead.m_cReadData = (char)(stAccI2cRead.m_nAdrs & BYTE_MASK);
      
      mutex_lock(&alps_lock);
      ret = accsns_i2c_readm(&stAccI2cRead.m_cReadData,REGISTER_LENGTH);
      accsns_err_check();
      mutex_unlock(&alps_lock);
      
      if(ret) {
        printk( "error(accsns_i2c_readm) : alps_ioctl(cmd = ALPSIO_ACC_I2C_READ)\n" );
        return -EFAULT;
      }
      ret = copy_to_user(argp,&stAccI2cRead,  sizeof(stAccI2cRead));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_ACC_I2C_READ)\n" );
        return -EFAULT;
      }
    }
    break;
    
  case ALPSIO_ACC_I2C_WRITE:
    {
      IoctlAccI2cWrite stAccI2cWrite = {0};
      char caWriteBuffer[I2C_WRITE_SIZE_MIN] = {0};
      
      ret = copy_from_user(&stAccI2cWrite, argp, sizeof(stAccI2cWrite));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_ACC_I2C_WRITE)\n" );
        return -EFAULT;
      }
      caWriteBuffer[INDEX_REG] = (char)(stAccI2cWrite.m_nAdrs & BYTE_MASK);
      caWriteBuffer[INDEX_VAL] = stAccI2cWrite.m_cWriteData;
      
      mutex_lock(&alps_lock);
      ret = accsns_i2c_writem(caWriteBuffer,I2C_WRITE_SIZE_MIN);
      accsns_err_check();
      mutex_unlock(&alps_lock);
      
      if(ret) {
        printk( "error(accns_i2c_writem) : alps_ioctl(cmd = ALPSIO_ACC_I2C_WRITE)\n" );
        return -EFAULT;
      }
    }
    break;
  case ALPSIO_ACC_GET:
    {
      IoctlAccDataGet stAccData;
      memset(&stAccData, 0x00, sizeof(stAccData));
      
      mutex_lock(&alps_lock);
      
      if(flgA++ == 0) {
        accsns_activate(ACT_UPDATE_FALSE, ACC_POWER_ON);
      }
      ret = accsns_get_acceleration_data(&stAccData.m_naData[0]);
#ifdef ALPS_DEBUG
      if(!ret)
        {
          printk("Acc_I2C, x:%d, y:%d, z:%d\n", 
                 *(stAccData.m_naData), *(stAccData.m_naData + INDEX_Y), *(stAccData.m_naData + INDEX_Z));
        }
#endif      
      if(--flgA == 0) {
        accsns_activate(ACT_UPDATE_FALSE, ACC_POWER_OFF);
      }
      accsns_err_check();
      mutex_unlock(&alps_lock);
      
      if (ret) {
        printk( "error(accns_get_acceleration_data) : alps_ioctl(cmd = ALPSIO_ACC_GET)\n" );
        return -EFAULT;
      }
      ret = copy_to_user(argp,&stAccData,  sizeof(stAccData));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_ACC_GET)\n" );
        return -EFAULT;
      }
    }
    break;
  case ALPSIO_ACC_DM_READ:
    {
      IoCtlAccDataDmGetCmd stDmReadCommand = {0};
      
      ret = copy_from_user(&stDmReadCommand, argp, sizeof(stDmReadCommand));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_ACC_DM_READ)\n" );
        return -EFAULT;
      }
#ifdef ALPS_DEBUG
      printk("[ACC]stDmReadCommand.m_nCommand=0x%x\n",stDmReadCommand.m_nCommand);
      printk("[ACC]stDmReadCommand.m_naData[0]=0x%x\n",stDmReadCommand.m_naData[0]);
#endif
      mutex_lock(&alps_lock);
      ret = accsns_dm_read(&stDmReadCommand);
      accsns_err_check();
      mutex_unlock(&alps_lock);
      
      if(ret) {
        printk( "error(accsns_dm_read) : alps_ioctl(cmd = ALPSIO_ACC_DM_READ)\n" );
        return -EFAULT;
      }
      ret = copy_to_user(argp,&stDmReadCommand,  sizeof(stDmReadCommand));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_ACC_DM_READ)\n" );
        return -EFAULT;
      }
    }
    break;
  case ALPSIO_ACC_DM_WRITE:
    {
      IoCtlAccDataDmSetCmd stDmWriteCommand = {0};
      
      ret = copy_from_user(&stDmWriteCommand, argp, sizeof(stDmWriteCommand));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_ACC_DM_WRITE)\n" );
        return -EFAULT;
      }
#ifdef ALPS_DEBUG
      printk("[ACC]stDmWriteCommand.m_nCommand=0x%x\n",stDmWriteCommand.m_nCommand);
      printk("[ACC]stDmWriteCommand.m_nData=0x%x\n",stDmWriteCommand.m_nData);
#endif
      mutex_lock(&alps_lock);
      ret = accsns_dm_write(&stDmWriteCommand);
      mutex_unlock(&alps_lock);
      
      if(ret) {
        printk( "error(accsns_dm_write) : alps_ioctl(cmd = ALPSIO_ACC_DM_WRITE)\n" );
        return -EFAULT;
      }
    }
    break;
  case ALPSIO_ACC_SET_FREQ:
    {
      int32_t rcv = 0;
      uint8_t freq = 0;

      ret = copy_from_user(&rcv,argp,sizeof(rcv));
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_ACC_SET_FREQ)\n" );
        return -EFAULT;
      }
      
      if (rcv > BYTE_MASK) {
        printk( "range over frequency %d\n", rcv);
        return -EFAULT;
      } else {
        freq = (uint8_t)(rcv & BYTE_MASK);
        accsns_set_freq(freq);
      }
    }
    break;
  case ALPSIO_ACC_SET_OFFSET:
    {
      IoCtlAccSetOffset ioc;
      memset(&ioc , 0x00, sizeof(ioc));
      ret = copy_from_user(&ioc,argp,sizeof(ioc));
      
      if (ret) {
        printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_ACC_SET_OFFSET)\n" );
        return -EFAULT;
      }
      accsns_set_offset(ioc.m_naOffset);
    }
    break;
  case ALPSIO_ACC_SUS_RES:
  {
    int32_t nArg = 0;
    ret = copy_from_user(&nArg,argp,sizeof(nArg));
    if (ret) {
      printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_ACC_SET_OFFSET)\n" );
      return -EFAULT;
    }
    
    if (nArg == 0) {
      pm_message_t mesg = {0};
      mesg.event = 0;
      accsns_suspend(NULL,mesg);
    } else if (nArg == 1) {
      accsns_resume(NULL);
    } else {
    }
    break;
  }
  case ALPSIO_MAG_SUS_RES:
  {
    int32_t nArg = 0;
    ret = copy_from_user(&nArg,argp,sizeof(nArg));
    if (ret) {
      printk( "error(copy_from_user) : alps_ioctl(cmd = ALPSIO_ACC_SET_OFFSET)\n" );
      return -EFAULT;
    }
    
    if (nArg == 0) {
      pm_message_t mesg = {0};
      mesg.event = 0;
      hscd_suspend(NULL,mesg);
    } else if (nArg == 1) {
      hscd_resume(NULL);
    } else {
    }
    break;
  }
    break;
  default:
    return -ENOTTY;
  }
  return 0;
}

static int32_t 
alps_io_open( struct inode* inode, struct file* filp )
{
  return 0;
}

static int32_t 
alps_io_release( struct inode* inode, struct file* filp )
{
  return 0;
}

static ssize_t accsns_position_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
  int32_t x = 0,y = 0,z = 0;
  int32_t xyz[INDEX_SUM] = {0};
  
  if(accsns_get_acceleration_data(xyz) == 0) {
    x = xyz[INDEX_X];
    y = xyz[INDEX_Y];
    z = xyz[INDEX_Z];
  } else {
    x = 0;
    y = 0;
    z = 0;
  }
  return snprintf(buf, PAGE_SIZE, "(%d %d %d)\n",x,y,z);
}

static ssize_t hscd_position_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
  int32_t x = 0,y = 0,z = 0;
  int32_t xyz[INDEX_SUM] = {0};
  
  if(hscd_get_magnetic_field_data(xyz) == 0) {
    x = xyz[INDEX_X];
    y = xyz[INDEX_Y];
    z = xyz[INDEX_Z];
  } else {
    x = 0;
    y = 0;
    z = 0;
  }
  return snprintf(buf, PAGE_SIZE, "(%d %d %d)\n",x,y,z);
}

static ssize_t alps_position_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
  size_t cnt = 0;
  mutex_lock(&alps_lock);
  cnt += accsns_position_show(dev,attr,buf);
  cnt += hscd_position_show(dev,attr,buf);
  mutex_unlock(&alps_lock);
  return cnt;
}
static DEVICE_ATTR(position, 0444, alps_position_show, NULL);

static struct file_operations alps_fops = {
  .owner   = THIS_MODULE,
  .open    = alps_io_open,
  .release = alps_io_release,
  .ioctl   = alps_ioctl,
};
static struct miscdevice alps_device = {
  .minor = MISC_DYNAMIC_MINOR,
  .name  = "alps_io",
  .fops  = &alps_fops,
};

static struct attribute *alps_attributes[] = {
  &dev_attr_position.attr,
  NULL,
};

static struct attribute_group alps_attribute_group = {
  .attrs = alps_attributes,
};

static struct early_suspend alps_early_suspend_handler = {
  .level	= EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
  .suspend	= alps_early_suspend,
  .resume 	= alps_late_resume,
}; 

static int32_t alps_probe(struct platform_device *dev)
{
  printk(KERN_INFO "alps: alps_probe\n");
  return 0;
}

static int32_t alps_remove(struct platform_device *dev)
{
  printk(KERN_INFO "alps: alps_remove\n");
  return 0;
}

static void alps_early_suspend(struct early_suspend *h)
{
  printk("alps: alps_early_suspend\n");
  mutex_lock(&alps_lock);
  poll_stop_sleep = true;
  mutex_unlock(&alps_lock);
}

static void alps_late_resume(struct early_suspend *h)
{
  printk("alps: alps_late_resume\n");
  mutex_lock(&alps_lock);
  poll_stop_sleep = false;
  mutex_unlock(&alps_lock);
}

static void accsns_poll(struct input_dev *idev)
{
  int32_t xyz[ACC_DATA_NUM] = {0};
  
  if(accsns_get_acceleration_data(xyz) == 0) {
    if(g_bIsAlreadyExistAccImitationXData) {
      xyz[INDEX_X] = g_naAccImitationData[INDEX_X];
    }
    if(g_bIsAlreadyExistAccImitationYData) {
      xyz[INDEX_Y] = g_naAccImitationData[INDEX_Y];
    }
    if(g_bIsAlreadyExistAccImitationZData) {
      xyz[INDEX_Z] = g_naAccImitationData[INDEX_Z];
    }
    input_report_abs(idev, EVENT_TYPE_ACCEL_X, xyz[INDEX_X]);
    input_report_abs(idev, EVENT_TYPE_ACCEL_Y, xyz[INDEX_Y]);
    input_report_abs(idev, EVENT_TYPE_ACCEL_Z, xyz[INDEX_Z]);
    idev->sync = 0;
    input_event(idev, EV_SYN, SYN_REPORT, EVENT_KIND_ACC);
  }
}

static void hscd_poll(struct input_dev *idev)
{
  int32_t xyz[MAG_DATA_NUM] = {0};
  
  if(hscd_get_magnetic_field_data(xyz) == 0) {

    if(g_bIsAlreadyExistMagImitationXData) {
      xyz[INDEX_X] = g_naMagImitationData[INDEX_X];
    }
    if(g_bIsAlreadyExistMagImitationYData) {
      xyz[INDEX_Y] = g_naMagImitationData[INDEX_Y];
    }
    if(g_bIsAlreadyExistMagImitationZData) {
      xyz[INDEX_Z] = g_naMagImitationData[INDEX_Z];
    }
    input_report_abs(idev, EVENT_TYPE_MAGV_X, xyz[INDEX_X]);
    input_report_abs(idev, EVENT_TYPE_MAGV_Y, xyz[INDEX_Y]);
    input_report_abs(idev, EVENT_TYPE_MAGV_Z, xyz[INDEX_Z]);
    idev->sync = 0;
    input_event(idev, EV_SYN, SYN_REPORT, EVENT_KIND_MAG);
  }
}

static void alps_poll(struct input_polled_dev *dev)
{
  struct input_dev *idev = dev->input;
  
  mutex_lock(&alps_lock);
  dev->poll_interval = delay;
  if (poll_stop_cnt-- < 0 && !poll_stop_sleep) {
    poll_stop_cnt = -1;
    if (flgM) hscd_poll(idev);
    if (flgA) accsns_poll(idev);
  }
  accsns_err_check();
  mutex_unlock(&alps_lock);
}

static struct platform_driver alps_driver = {
  .driver = {
    .name = "alps-input",
    .owner = THIS_MODULE,
  },
  .probe = alps_probe,
  .remove = alps_remove,
};

static int32_t __init alps_init(void)
{
  struct input_dev *idev = NULL;
  int32_t ret = 0;
  
  ret = platform_driver_register(&alps_driver);
  if (ret)
    goto out_region;
  printk(KERN_INFO "alps-init: platform_driver_register\n");
  
  pdev = platform_device_register_simple("alps", -1, NULL, 0);
  if (IS_ERR(pdev)) {
    ret = PTR_ERR(pdev);
    goto out_driver;
  }
  printk(KERN_INFO "alps-init: platform_device_register_simple\n");
  
  ret = sysfs_create_group(&pdev->dev.kobj, &alps_attribute_group);
  if (ret)
    goto out_device;
  printk(KERN_INFO "alps-init: sysfs_create_group\n");
  
  alps_idev = input_allocate_polled_device();
  if (!alps_idev) {
    ret = -ENOMEM;
    goto out_group;
  }
  printk(KERN_INFO "alps-init: input_allocate_polled_device\n");
  
  alps_idev->poll = alps_poll;
  alps_idev->poll_interval = ALPS_POLL_INTERVAL;
  
  idev = alps_idev->input;
  idev->name = "alps";
  idev->phys = "alps/input0";
  idev->id.bustype = BUS_HOST;
  idev->dev.parent = &pdev->dev;
  idev->evbit[0] = BIT_MASK(EV_ABS);
  
  input_set_abs_params(idev, EVENT_TYPE_ACCEL_X,
                       (-1 * EVENT_ACC_DATA_MAX), EVENT_ACC_DATA_MAX, 
                       ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);
  input_set_abs_params(idev, EVENT_TYPE_ACCEL_Y,
                       (-1 * EVENT_ACC_DATA_MAX), EVENT_ACC_DATA_MAX, 
                       ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);
  input_set_abs_params(idev, EVENT_TYPE_ACCEL_Z,
                       (-1 * EVENT_ACC_DATA_MAX), EVENT_ACC_DATA_MAX, 
                       ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);
  
  input_set_abs_params(idev, EVENT_TYPE_MAGV_X,
                       (-1 * EVENT_MAG_DATA_MAX), EVENT_MAG_DATA_MAX, 
                       ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);
  input_set_abs_params(idev, EVENT_TYPE_MAGV_Y,
                       (-1 * EVENT_MAG_DATA_MAX), EVENT_MAG_DATA_MAX,
                        ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);
  input_set_abs_params(idev, EVENT_TYPE_MAGV_Z,
                       (-1 * EVENT_MAG_DATA_MAX), EVENT_MAG_DATA_MAX,
                       ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);
  
  ret = input_register_polled_device(alps_idev);
  if (ret)
    goto out_idev;
  printk(KERN_INFO "alps-init: input_register_polled_device\n");
  
  ret = misc_register(&alps_device);
  if (ret) {
    printk("alps-init: alps_io_device register failed\n");
    goto exit_misc_device_register_failed;
  }
  printk("alps-init: misc_register\n");
  
  register_early_suspend(&alps_early_suspend_handler);
  
  return 0;
  
 exit_misc_device_register_failed:
 out_idev:
  input_free_polled_device(alps_idev);
  printk(KERN_INFO "alps-init: input_free_polled_device\n");
 out_group:
  sysfs_remove_group(&pdev->dev.kobj, &alps_attribute_group);
  printk(KERN_INFO "alps-init: sysfs_remove_group\n");
 out_device:
  platform_device_unregister(pdev);
  printk(KERN_INFO "alps-init: platform_device_unregister\n");
 out_driver:
  platform_driver_unregister(&alps_driver);
  printk(KERN_INFO "alps-init: platform_driver_unregister\n");
 out_region:
  return ret;
}

static void __exit alps_exit(void)
{
  unregister_early_suspend(&alps_early_suspend_handler);
  misc_deregister(&alps_device);
  printk(KERN_INFO "alps-exit: misc_deregister\n");
  input_unregister_polled_device(alps_idev);
  printk(KERN_INFO "alps-exit: input_unregister_polled_device\n");
  input_free_polled_device(alps_idev);
  printk(KERN_INFO "alps-exit: input_free_polled_device\n");
  sysfs_remove_group(&pdev->dev.kobj, &alps_attribute_group);
  printk(KERN_INFO "alps-exit: sysfs_remove_group\n");
  platform_device_unregister(pdev);
  printk(KERN_INFO "alps-exit: platform_device_unregister\n");
  platform_driver_unregister(&alps_driver);
  printk(KERN_INFO "alps-exit: platform_driver_unregister\n");
}

module_init(alps_init);
module_exit(alps_exit);

MODULE_DESCRIPTION("Alps Input Device");
MODULE_AUTHOR("ALPS");
MODULE_LICENSE("GPL v2");
