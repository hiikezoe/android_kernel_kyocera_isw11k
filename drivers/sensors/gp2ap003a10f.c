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
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/spinlock.h>

#include <asm/uaccess.h>

#include <linux/i2c/gp2ap003a10f.h>

#include <linux/hrtimer.h>

#define SENSOR_DEBUG        0

#if SENSOR_DEBUG
#define DEBUG_PRINT( arg... )   printk( KERN_INFO "GP2AP003A10F:" arg )
#else
#define DEBUG_PRINT( arg... )
#endif

#define GP2AP003A10F_DRV_NAME   "gp2ap003a10f"
#define DRIVER_VERSION          "1.0.0"

#define D_MEAS_STATE_STOP           (0x00)
#define D_MEAS_STATE_OPEN           (0x01)
#define D_MEAS_STATE_START          (0x02)
#define D_MEAS_STATE_ALS_START      (0x04)
#define D_MEAS_STATE_PROX_START     (0x08)
#define D_MEAS_STATE_PROX_INT       (0x10)
#define D_MEAS_STATE_DETECT         (0x20)


#define D_DATA_BIT_INT_CLEAR (0x80)

#define D_GAIN_SET_LED          (0x01<<3)
#define D_CYCLE_SET_CYCL        (0x04<<3)
#define D_CYCLE_SET_OSC         (0x01<<2)
#define D_OPMOD_SET_VCON        (0x01<<1)
#define D_OPMOD_SET_ASD         (0x00<<4)
#define D_OPMOD_SET_ASD_ON      (0x01<<4)
#define D_OPMOD_SET_SSD         (0x01)
#define D_OPMOD_SET_SD          (D_OPMOD_SET_ASD_ON|D_OPMOD_SET_VCON|D_OPMOD_SET_SSD)
#define D_OPMOD_SET_OFF         (D_OPMOD_SET_ASD_ON|D_OPMOD_SET_VCON)

#define D_CON_SET_OCON          (0x00<<3)
#define D_CON_SET_OCON_ON       (0x03<<3)

#define D_GAIN_SET              (D_GAIN_SET_LED)
#define D_CYCLE_SET             (D_CYCLE_SET_CYCL|D_CYCLE_SET_OSC)
#define D_HYS_SET_A             (0xC2)
#define D_CON_SET               (D_CON_SET_OCON)

#define GPIO_PROXIMITY_SENSOR_INT   (125)
#define GPIO_VSLED_ON               (103)
#if 0
#define GPIO_PHOTO_ON               (126)
#endif

#define D_CAM_TEMP_T2           (0x28)
#define D_CAM_TEMP_T1           (0x0A)

#define STATE_STOP_MASK(arg)        (arg&~(D_MEAS_STATE_OPEN|D_MEAS_STATE_PROX_INT))

#define D_DEV_WAIT_TIME         (1)
#define D_I2C_RETRIES           (5)
#define D_I2C_WAIT_TIME         (1)
#define D_RX_MSG_NUM            (2)
#define D_RX_MSG_LENGTH         (2)
#define D_TX_MSG_NUM            (1)
#define D_TX_MSG_LENGTH         (2)

#define D_PROX_NV_NEAR_DEF      (0x22)
#define D_PROX_NV_FAR_DEF       (0x00)
#define D_PROX_NV_DETECT_DEF    (0x0390)
#define D_PROX_NV_PHOTO_DEF     (0x40)
#define D_CAM_TEMP_DEF          (1)

#define GP2AP_IRQ_TIMER         125

struct GP2AP003A10F_data
{
    struct input_dev *input_dev_type;
    struct work_struct work_type;
};

static spinlock_t sense_data_spin_lock_type;

static struct i2c_client *this_client_type = NULL;

static unsigned long gul_als_measurement_data = 0;
static unsigned long gul_prox_measurement_data = 0;
static uint32_t gul_prox_timer_flag = 0;
static unsigned long gul_als_measurement_lux_data = 0;

static unsigned long gul_cam_temp_def_data = 0;
static uint8_t  guc_prox_nv_near[D_SENSOR_NEAR_SIZE]    = {0};
static uint8_t  guc_prox_nv_far[D_SENSOR_FAR_SIZE]      = {0};
static uint16_t gun_prox_nv_detect                      = 0;
static uint8_t  guc_als_nv_photo[D_SENSOR_PHOTO_SIZE]   = {0};
static uint8_t  guc_cam_temp_def        = 0;
static uint32_t gun_nv_status           = 0;

static uint32_t gun_meas_state = D_MEAS_STATE_STOP;
static int32_t gn_open_count = 0;
static uint32_t gun_meas_state_bk = D_MEAS_STATE_STOP;
static int32_t gn_active_prox_count = 0;
static int32_t gn_active_als_count = 0;

static struct hrtimer gp2ap_irq_timer;

static void gp2ap_irq_enable( void );
static irqreturn_t gp2ap_interrupt(int irq, void *pvDev_id);
static void gp2ap_input_report( int32_t nMode, unsigned long ulVal );

static int32_t i2c_rx_data(char *pcRxData, int32_t nLength)
{
    int32_t nRet = 0;
    int32_t nTries = 0;
    struct i2c_msg msgs_type[] =
    {
        {
            .addr = this_client_type->addr,
            .flags = 0,
            .len = 1,
            .buf = pcRxData,
        },
        {
            .addr = this_client_type->addr,
            .flags = I2C_M_RD,
            .len = nLength,
            .buf = pcRxData,
        },
    };

    DEBUG_PRINT( "i2c_rx_data(): rxData(0x%08X) rxData[0](%08X) rxData[1](%08X) nLength(%d) slave_addr(0x%02X)\n",
                     (unsigned int)pcRxData, pcRxData[0], pcRxData[1], nLength, this_client_type->addr );

    do {
        nRet = i2c_transfer(this_client_type->adapter, msgs_type, D_RX_MSG_NUM);
        if(nRet != D_RX_MSG_NUM)
        {
            msleep(D_I2C_WAIT_TIME);
        }
    } while ((nRet != D_RX_MSG_NUM) && (++nTries < D_I2C_RETRIES));

    if (nRet != D_RX_MSG_NUM)
    {
        printk(KERN_ERR "GP2AP003A10F GP2AP003A10F_RxData: transfer error\n");
        nRet = -EIO;
    } else
    {
        nRet = 0;
    }

    DEBUG_PRINT( "i2c_rx_data(): rxData[0](%08X) rxData[1](%08X) nLength(%d) slave_addr(0x%02X)\n",
                     pcRxData[0], pcRxData[1], nLength, this_client_type->addr );

    return nRet;
}

static int32_t i2c_tx_data(char *pcTxData, int32_t nLength)
{
    int32_t nRet = 0;
    int32_t nTries = 0;
    struct i2c_msg msg_type[] =
    {
        {
            .addr = this_client_type->addr,
            .flags = 0,
            .len = nLength,
            .buf = pcTxData,
        },
    };

    do {
        nRet = i2c_transfer(this_client_type->adapter, msg_type, D_TX_MSG_NUM);
        if(nRet != D_TX_MSG_NUM)
        {
            msleep(D_I2C_WAIT_TIME);
        }
    } while ((nRet != D_TX_MSG_NUM) && (++nTries < D_I2C_RETRIES));
    if(nRet != D_TX_MSG_NUM)
    {
        printk(KERN_ERR "GP2AP003A10F GP2AP003A10F_TxData: transfer error\n");
        nRet = -EIO;
    } else
    {
        nRet = 0;
    }

    return nRet;
}

static int32_t get_register( unsigned char ucAddr, unsigned char* pcval, int32_t nlen )
{
    int32_t nRet = 0;

    DEBUG_PRINT( "get_register() start: addr(%02X) pval(0x%08X) pval[0](%02X) len(%d)\n",
                     ucAddr, (unsigned int)pcval, pcval[0], nlen );

    pcval[0] = D_DATA_BIT_INT_CLEAR;

    nRet = i2c_rx_data( pcval, nlen );

    DEBUG_PRINT( "get_register() end: addr(%02X) pval(0x%08X) pval[0](%02X) pval[1](%02X) len(%d)\n",
                     ucAddr, (unsigned int)pcval, pcval[0], pcval[1], nlen );

    return nRet;
}

static int32_t set_register( unsigned char ucAddr, unsigned char ucVal )
{
    int32_t nRet = 0;

    unsigned char ucData[2] = {0,0};

    ucData[0] = ucAddr;

    ucData[1] = ucVal;

    nRet = i2c_tx_data( ucData, D_TX_MSG_LENGTH );

    return nRet;
}

#if 0
static void register_photo_on( uint8_t ucGain )
{
    DEBUG_PRINT("[IN]register_photo_on gain=%d",ucGain);
    gpio_set_value(GPIO_PHOTO_ON, ucGain);
    DEBUG_PRINT("[OUT]register_photo_on\n");
}
#endif

static void register_shutdown_enable( int32_t nMode )
{
    DEBUG_PRINT("[IN]register_shutdown_enable nMode=%d",nMode);
    spin_lock( &sense_data_spin_lock_type );
    if(!(gun_meas_state&D_MEAS_STATE_START))
    {
        DEBUG_PRINT("device on\n");
        gpio_set_value(GPIO_VSLED_ON, 1);
#if 0
        register_photo_on(1);
#endif
        gun_meas_state |= D_MEAS_STATE_START;
        spin_unlock( &sense_data_spin_lock_type );
        msleep(D_DEV_WAIT_TIME);
    } else
    {
        spin_unlock( &sense_data_spin_lock_type );
    }
    if(D_MODE_PROX == nMode)
    {
        set_register(D_ADDR_GAIN, D_GAIN_SET);
        set_register(D_ADDR_CYCLE, D_CYCLE_SET);
        set_register(D_ADDR_HYS, guc_prox_nv_near[guc_cam_temp_def]);
        set_register(D_ADDR_OPMOD, D_OPMOD_SET_SD);
        spin_lock( &sense_data_spin_lock_type );
        gun_meas_state |= D_MEAS_STATE_PROX_START;
        gul_prox_measurement_data = 0;
        spin_unlock( &sense_data_spin_lock_type );
        gp2ap_irq_enable();
        if( !(gun_meas_state&D_MEAS_STATE_DETECT) )
        {
            set_register(D_ADDR_CON, D_CON_SET_OCON);
        }
    } else
    {
        spin_lock( &sense_data_spin_lock_type );
        gun_meas_state |= D_MEAS_STATE_ALS_START;
        spin_unlock( &sense_data_spin_lock_type );

    }
    DEBUG_PRINT("[OUT]register_shutdown_enable\n");
}

static void register_shutdown_disable( int32_t nMode )
{
    DEBUG_PRINT("[IN]register_shutdown_disable nMode=%d\n",nMode);
    if(D_MODE_PROX == nMode)
    {
        set_register(D_ADDR_CON, D_CON_SET_OCON_ON);
        set_register(D_ADDR_OPMOD, D_OPMOD_SET_OFF);
    }
    if(D_MEAS_STATE_STOP == STATE_STOP_MASK(gun_meas_state))
    {
        DEBUG_PRINT("device off\n");
        gpio_set_value(GPIO_VSLED_ON, 0);
#if 0
        gpio_set_value(GPIO_PHOTO_ON, 0);
#endif
    }
    DEBUG_PRINT("[OUT]register_shutdown_disable\n");
}


static unsigned long set_cam_temp_revision(unsigned long ulTemp)
{
    int32_t nCount = 0;

    const struct adcToDef {
        unsigned long  ulCelsius_value;
        unsigned long  ulCamDef;
    } adcToDef[] = {
        {  D_CAM_TEMP_T2,  2 },
        {  D_CAM_TEMP_T1,  1 },
    };
    for(nCount=0; nCount<ARRAY_SIZE(adcToDef); nCount++)
    {
        if (ulTemp > adcToDef[nCount].ulCelsius_value) {
            return adcToDef[nCount].ulCamDef;
        }
    }
    return 0;
}

static int32_t register_ctrl( int32_t nMode, unsigned long ulArg )
{
    int32_t nRet = 0;
    T_GP2AP003A10F_IOCTL_REG* ioctl_data = (T_GP2AP003A10F_IOCTL_REG*)ulArg;
    unsigned char ucdata[2] = {0,0};

    switch( nMode )
    {
        case IOCTL_LIGHT_I2C_WRITE:
        case IOCTL_PROXIMITY_I2C_WRITE:
            nRet = set_register( ioctl_data->ulAddress, (unsigned char)ioctl_data->ulData );

            if( nRet != 0 )
            {
                printk(KERN_ERR "GP2AP003A10F register_ctrl: Can't set register data\n");
            }
            break;

        case IOCTL_LIGHT_I2C_READ:
        case IOCTL_PROXIMITY_I2C_READ:
            nRet = get_register( ioctl_data->ulAddress, (unsigned char*)ucdata, 2 );
            ioctl_data->ulData = ucdata[1];
            if( nRet != 0 )
            {
                printk(KERN_ERR "GP2AP003A10F register_ctrl: Can't get register data\n");
            }
            break;

        default :
            break;
    }
    return nRet;
}

static int32_t get_sensor_data( unsigned long ulArg )
{
    int32_t nRet = 0;
    T_GP2AP003A10F_IOCTL_SENSOR* sensor_data_type = (T_GP2AP003A10F_IOCTL_SENSOR*)ulArg;

    DEBUG_PRINT("[IN]get_sensor_data\n");

    if( !(gun_meas_state & D_MEAS_STATE_START) )
    {
        return -EAGAIN;
    }

    spin_lock( &sense_data_spin_lock_type );

    if( sensor_data_type->ulSensor_mode == D_MODE_ALS )
    {
        sensor_data_type->ulData = gul_als_measurement_data;
        sensor_data_type->ulSensor_mode = D_MODE_ALS;
    }
    else
    {
        sensor_data_type->ulData = gul_prox_measurement_data;
        sensor_data_type->ulSensor_mode = D_MODE_PROX;
    }

    spin_unlock( &sense_data_spin_lock_type );
    DEBUG_PRINT("[OUT]get_sensor_data nRet = %d\n",nRet);

    return nRet;
}

static void clear_sensor_data( void )
{
    DEBUG_PRINT("[IN]clear_sensor_data\n");
    spin_lock( &sense_data_spin_lock_type );
    gul_als_measurement_data = 0;
    gul_prox_measurement_data = 0;

    if(!gun_nv_status)
    {
        memset(&guc_prox_nv_near, D_PROX_NV_NEAR_DEF,
                        sizeof(guc_prox_nv_near));
        memset(&guc_prox_nv_far,  D_PROX_NV_FAR_DEF,
                        sizeof(guc_prox_nv_far));
        gun_prox_nv_detect = D_PROX_NV_DETECT_DEF;
        memset(&guc_als_nv_photo, D_PROX_NV_PHOTO_DEF,
                        sizeof(guc_als_nv_photo));
    }
    guc_cam_temp_def = D_CAM_TEMP_DEF;

    spin_unlock( &sense_data_spin_lock_type );
    DEBUG_PRINT("[OUT]clear_sensor_data\n");
    return;
}

static void set_sensor_active(int32_t nActive, int32_t nMode)
{
    uint32_t un_meas_state = 0;
    uint32_t unModeStart = 0;
    if(nActive)
    {
        spin_lock( &sense_data_spin_lock_type );
        un_meas_state = gun_meas_state;
        if(D_MODE_PROX == nMode)
        {
            gn_active_prox_count++;
            unModeStart = !(D_MEAS_STATE_PROX_START&un_meas_state);
        } else
        {
            gn_active_als_count++;
            unModeStart = !(D_MEAS_STATE_ALS_START&un_meas_state);
        }
        spin_unlock( &sense_data_spin_lock_type );
        if(unModeStart)
        {
            register_shutdown_enable(nMode);
        }
    }
    else
    {
        spin_lock( &sense_data_spin_lock_type );
        if(D_MODE_PROX == nMode)
        {
            if(--gn_active_prox_count <= 0)
            {
                gn_active_prox_count = 0;
                gun_meas_state &= ~D_MEAS_STATE_PROX_START;
                unModeStart = D_MEAS_STATE_PROX_START;
            }
        } else
        {
            if(--gn_active_als_count <= 0)
            {
                gn_active_als_count = 0;
                gun_meas_state &= ~D_MEAS_STATE_ALS_START;
                unModeStart = D_MEAS_STATE_ALS_START;
            }
        }
        gun_meas_state = (gun_meas_state&(
                    D_MEAS_STATE_PROX_START|D_MEAS_STATE_ALS_START))?
                    gun_meas_state:(gun_meas_state&~D_MEAS_STATE_START);
        spin_unlock( &sense_data_spin_lock_type );
        if(unModeStart)
        {
            register_shutdown_disable(nMode);
        }
    }
}

static void set_sensor_command( unsigned long ulArg )
{
    T_GP2AP003A10F_IOCTL_COMMAND* command_data_type
                            = (T_GP2AP003A10F_IOCTL_COMMAND*)ulArg;
    bool bInput_envnt = false;

    switch(command_data_type->ulCommand)
    {
    case D_CMD_SENSOR_ACTIVE:
        {
            DEBUG_PRINT("D_CMD_SENSOR_ACTIVE act = %d, mode = %d\n",
                (int)command_data_type->ulActive,(int)command_data_type->ulSensor_mode);
            set_sensor_active((int)command_data_type->ulActive,
                                        (int)command_data_type->ulSensor_mode);
            command_data_type->ulResponse = 0;
        }
        break;
    case D_CMD_LIGHT_DATA_SET:
        {
            spin_lock( &sense_data_spin_lock_type );
            DEBUG_PRINT("D_CMD_LIGHT_DATA lux = %ld\n",
                                command_data_type->ulData);
            if(gul_als_measurement_data != command_data_type->ulData)
            {
                gul_als_measurement_data = command_data_type->ulData;
            }
            command_data_type->ulResponse = 0;
            spin_unlock( &sense_data_spin_lock_type );

            if(gun_prox_nv_detect < (uint32_t)gul_als_measurement_data)
            {
                /* if(!(gun_meas_state&D_MEAS_STATE_DETECT)) */
                if((!(gun_meas_state&D_MEAS_STATE_DETECT)) &&
                   (gul_prox_measurement_data == 1) && !(gul_prox_timer_flag))
                {
                    set_register(D_ADDR_CON, D_CON_SET_OCON_ON);
                    set_register(D_ADDR_HYS, guc_prox_nv_near[guc_cam_temp_def]);
                    spin_lock( &sense_data_spin_lock_type );
                    gun_meas_state |= D_MEAS_STATE_DETECT;
                    gul_prox_measurement_data = 0;
                    spin_unlock( &sense_data_spin_lock_type );
                    DEBUG_PRINT("detect on:state=%x,als%ld\n",
                                        gun_meas_state,gul_als_measurement_data);
                    bInput_envnt = true;
                }
            }else
            {
                if(gun_meas_state&D_MEAS_STATE_DETECT)
                {
                    spin_lock( &sense_data_spin_lock_type );
                    gun_meas_state &= ~D_MEAS_STATE_DETECT;
                    spin_unlock( &sense_data_spin_lock_type );
                    DEBUG_PRINT("detect off:state=%x,als=%ld\n",
                                        gun_meas_state,gul_als_measurement_data);
                    gp2ap_irq_enable();
                    set_register(D_ADDR_CON, D_CON_SET_OCON);
                }
            }

            if(bInput_envnt)
            {
                gp2ap_input_report(D_MODE_PROX, gul_prox_measurement_data);
            }
        }
        break;
    case D_CMD_CAM_TEMP_DEF_SET:
        {
            spin_lock( &sense_data_spin_lock_type );
            DEBUG_PRINT("D_CMD_CAM_TEMP_DEF_SET: ADC = %ld\n",
                            command_data_type->ulData);
            gul_cam_temp_def_data = command_data_type->ulData;
            guc_cam_temp_def = set_cam_temp_revision(gul_cam_temp_def_data);
            DEBUG_PRINT("D_CMD_CAM_TEMP_DEF_SET: def = %d\n",guc_cam_temp_def);
            spin_unlock( &sense_data_spin_lock_type );
            command_data_type->ulResponse = 0;
        }
        break;
    case D_CMD_LIGHT_CONV_DATA_SET:
        {
            spin_lock( &sense_data_spin_lock_type );
            DEBUG_PRINT("D_CMD_LIGHT_CONV_DATA lux = %ld\n",
                                command_data_type->ulData);
            if(gul_als_measurement_lux_data != command_data_type->ulData)
            {
                gul_als_measurement_lux_data = command_data_type->ulData;
                bInput_envnt = true;
            }
            command_data_type->ulResponse = 0;
            spin_unlock( &sense_data_spin_lock_type );

            if(bInput_envnt)
            {
                gp2ap_input_report(D_MODE_ALS, gul_als_measurement_lux_data);
            }
        }
        break;
    default :
        command_data_type->ulResponse = -1;
        break;
    }
}

static int32_t get_sensor_nv( unsigned long ulArg )
{
    int32_t nRet = -1;
    T_GP2AP003A10F_IOCTL_NV* nv_data_type
                            = (T_GP2AP003A10F_IOCTL_NV*)ulArg;
    spin_lock( &sense_data_spin_lock_type );
    if(gun_nv_status & (0x01<<nv_data_type->ulItem))
    {
        switch(nv_data_type->ulItem)
        {
        case en_NV_PROXIMITY_SENSOR_NEAR:
            memcpy(nv_data_type->ucData,
                    &guc_prox_nv_near,sizeof(guc_prox_nv_near));
            nv_data_type->ulLength = sizeof(guc_prox_nv_near);
            nRet = 0;
            break;
        case en_NV_PROXIMITY_SENSOR_FAR:
            memcpy(nv_data_type->ucData,
                    &guc_prox_nv_far,sizeof(guc_prox_nv_far));
            nv_data_type->ulLength = sizeof(guc_prox_nv_far);
            nRet = 0;
            break;
        case en_NV_PROXIMITY_DETECT:
            memcpy(nv_data_type->ucData,
                    &gun_prox_nv_detect,sizeof(gun_prox_nv_detect));
            nv_data_type->ulLength = sizeof(gun_prox_nv_detect);
            nRet = 0;
            break;
        case en_NV_PHOTO_SENSOR:
            memcpy(nv_data_type->ucData,
                    &guc_als_nv_photo,sizeof(guc_als_nv_photo));
            nv_data_type->ulLength = sizeof(guc_als_nv_photo);
            nRet = 0;
            break;
        default :
            printk(KERN_ERR "GP2AP003A10F set_sensor_nv: Can't set nv data\n");
            break;
        }
    }
    spin_unlock( &sense_data_spin_lock_type );
    return nRet;
}

static void set_sensor_nv( unsigned long ulArg )
{
    T_GP2AP003A10F_IOCTL_NV* nv_data_type
                            = (T_GP2AP003A10F_IOCTL_NV*)ulArg;
    spin_lock( &sense_data_spin_lock_type );
    switch(nv_data_type->ulItem)
    {
    case en_NV_PROXIMITY_SENSOR_NEAR:
        gun_nv_status |= (0x01<<en_NV_PROXIMITY_SENSOR_NEAR);
        memcpy(&guc_prox_nv_near,
                nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        break;
    case en_NV_PROXIMITY_SENSOR_FAR:
        gun_nv_status |= (0x01<<en_NV_PROXIMITY_SENSOR_FAR);
        memcpy(&guc_prox_nv_far,
                nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        break;
    case en_NV_PROXIMITY_DETECT:
        gun_nv_status |= (0x01<<en_NV_PROXIMITY_DETECT);
        memcpy(&gun_prox_nv_detect,
                nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        break;
    case en_NV_PHOTO_SENSOR:
        gun_nv_status |= (0x01<<en_NV_PHOTO_SENSOR);
        memcpy(guc_als_nv_photo,
                nv_data_type->ucData,(size_t)nv_data_type->ulLength);
        break;
    default :
        printk(KERN_ERR "GP2AP003A10F set_sensor_nv: Can't set nv data\n");
        break;
    }
    spin_unlock( &sense_data_spin_lock_type );
}


static int32_t GP2AP003A10F_init_module(void)
{
    clear_sensor_data();
    set_register(D_ADDR_GAIN, D_GAIN_SET);
    set_register(D_ADDR_CYCLE, D_CYCLE_SET);
    set_register(D_ADDR_HYS, guc_prox_nv_near[guc_cam_temp_def]);
    set_register(D_ADDR_CON, D_CON_SET_OCON_ON);
    set_register(D_ADDR_OPMOD, D_OPMOD_SET_OFF);
    return 0;
}

static int GP2AP003A10F_open(struct inode *pInode_type,
    struct file *pFile_type)
{
    DEBUG_PRINT("[IN]GP2AP003A10F_open\n");
    spin_lock( &sense_data_spin_lock_type );

    gn_open_count++;

    if( gn_open_count > 0 )
    {
        gun_meas_state |= D_MEAS_STATE_OPEN;
    }

    DEBUG_PRINT("GP2AP003A10F_open open=%d\n",gn_open_count);

    spin_unlock( &sense_data_spin_lock_type );

    DEBUG_PRINT("[OUT]GP2AP003A10F_open\n");
    return 0;
}

static int GP2AP003A10F_release(struct inode *pInode_type,
    struct file *pFile_type)
{
    DEBUG_PRINT("[IN]GP2AP003A10F_release");
    spin_lock( &sense_data_spin_lock_type );

    gn_open_count--;

    if( gn_open_count <= 0 )
    {
        gn_open_count = 0;

        gun_meas_state &= D_MEAS_STATE_PROX_INT;

        spin_unlock( &sense_data_spin_lock_type );

        register_shutdown_disable(D_MODE_PROX);

        spin_lock( &sense_data_spin_lock_type );
    }
    else
    {
        gun_meas_state |= D_MEAS_STATE_OPEN;
    }
    DEBUG_PRINT("GP2AP003A10F_release open=%d",gn_open_count);
    spin_unlock( &sense_data_spin_lock_type );

    DEBUG_PRINT("[OUT]GP2AP003A10F_release\n");
    return 0;
}

static long
GP2AP003A10F_ioctl(struct file *pFile_type,
       uint32_t unCmd, unsigned long ulArg)
{
    int32_t nRet = -EINVAL;
    T_GP2AP003A10F_IOCTL_SENSOR sensor_data_type;
    T_GP2AP003A10F_IOCTL_COMMAND command_data_type;
    T_GP2AP003A10F_IOCTL_REG reg_data_type;
    DEBUG_PRINT("[IN]GP2AP003A10F_ioctl\n");

    memset((void*)&sensor_data_type, 0,
                        sizeof(T_GP2AP003A10F_IOCTL_SENSOR) );
    memset((void*)&command_data_type, 0,
                        sizeof(T_GP2AP003A10F_IOCTL_COMMAND) );
    memset((void*)&reg_data_type, 0,
                        sizeof(T_GP2AP003A10F_IOCTL_REG) );
    switch( unCmd )
    {
        case IOCTL_LIGHT_I2C_READ:
        case IOCTL_PROXIMITY_I2C_READ:
            nRet = copy_from_user(&reg_data_type,
                    (void __user *)ulArg, sizeof(T_GP2AP003A10F_IOCTL_REG) );
            if (nRet) {
                printk("error : GP2AP003A10F_ioctl(unCmd = I2C_READ_WRITE)\n" );
                return -EFAULT;
            }
            nRet = register_ctrl(unCmd, (unsigned long)&reg_data_type );
            nRet = copy_to_user((void *)(ulArg),
                     &reg_data_type, sizeof(T_GP2AP003A10F_IOCTL_REG) );
            if (nRet) {
                printk("error : GP2AP003A10F_ioctl(unCmd = I2C_READ_WRITE)\n" );
                return -EFAULT;
            }
            break;
        case IOCTL_LIGHT_I2C_WRITE:
        case IOCTL_PROXIMITY_I2C_WRITE:
            nRet = copy_from_user(&reg_data_type,
                    (void __user *)ulArg, sizeof(T_GP2AP003A10F_IOCTL_REG) );
            if (nRet) {
                printk("error : GP2AP003A10F_ioctl(unCmd = I2C_READ_WRITE)\n" );
                return -EFAULT;
            }
            nRet = register_ctrl(unCmd, (unsigned long)&reg_data_type );
            nRet = copy_to_user((void *)(ulArg),
                    &reg_data_type, sizeof(T_GP2AP003A10F_IOCTL_REG) );
            if (nRet) {
                printk("error : GP2AP003A10F_ioctl(unCmd = I2C_READ_WRITE)\n" );
                return -EFAULT;
            }
            break;
        case IOCTL_LIGHT_DATA_GET:
            nRet = copy_from_user(&sensor_data_type,
                    (void __user *)ulArg, sizeof(T_GP2AP003A10F_IOCTL_SENSOR) );
            if (nRet) {
                printk("error : GP2AP003A10F_ioctl(unCmd = I2C_READ_WRITE)\n" );
                return -EFAULT;
            }
            sensor_data_type.ulSensor_mode = D_MODE_ALS;
            nRet = get_sensor_data( (unsigned long)&sensor_data_type );
            nRet = copy_to_user((void __user *)ulArg,
                    &sensor_data_type, sizeof(T_GP2AP003A10F_IOCTL_SENSOR) );
            if (nRet) {
                printk("error : GP2AP003A10F_ioctl(unCmd = I2C_READ_WRITE)\n" );
                return -EFAULT;
            }
            break;
        case IOCTL_PROXIMITY_DATA_GET:
            nRet = copy_from_user(&sensor_data_type,
                    (void __user *)ulArg, sizeof(T_GP2AP003A10F_IOCTL_SENSOR) );
            if (nRet) {
                printk("error : GP2AP003A10F_ioctl(unCmd = I2C_READ_WRITE)\n" );
                return -EFAULT;
            }
            sensor_data_type.ulSensor_mode = D_MODE_PROX;
            nRet = get_sensor_data( (unsigned long)&sensor_data_type );
            nRet = copy_to_user((void __user *)ulArg,
                    &sensor_data_type, sizeof(T_GP2AP003A10F_IOCTL_SENSOR) );
            if (nRet) {
                printk("error : GP2AP003A10F_ioctl(unCmd = I2C_READ_WRITE)\n" );
                return -EFAULT;
            }
            break;
        case IOCTL_CLEAR_SENSOR_DATA:
            clear_sensor_data();
            nRet = 0;
            break;
        case IOCTL_PROXIMITY_NV_DATA_GET:
            {
                T_GP2AP003A10F_IOCTL_NV sensor_nv_type;
                memset((void*)&sensor_nv_type, 0,
                            sizeof(T_GP2AP003A10F_IOCTL_NV) );
                nRet = copy_from_user(&sensor_nv_type,
                        (void __user *)ulArg, sizeof(T_GP2AP003A10F_IOCTL_NV) );
                if(nRet)
                {
                    printk("error : GP2AP003A10F_ioctl(cmd = IOCTL_PROXIMITY_NV_DATA_GET)\n" );
                    return -EFAULT;
                }
                nRet = get_sensor_nv((unsigned long)&sensor_nv_type);
                if(!nRet)
                {
                    nRet = copy_to_user((void __user *)ulArg,
                        &sensor_nv_type, sizeof(T_GP2AP003A10F_IOCTL_NV) );
                } else
                {
                    printk("error : GP2AP003A10F_ioctl(cmd = IOCTL_PROXIMITY_NV_DATA_GET)\n" );
                    return -EFAULT;
                }
            }
            break;
        case IOCTL_PROXIMITY_NV_DATA_SET:
            {
                T_GP2AP003A10F_IOCTL_NV sensor_nv_type;
                memset((void*)&sensor_nv_type, 0,
                            sizeof(T_GP2AP003A10F_IOCTL_NV) );
                nRet = copy_from_user(&sensor_nv_type,
                        (void __user *)ulArg, sizeof(T_GP2AP003A10F_IOCTL_NV) );
                if(!nRet)
                {
                    set_sensor_nv((unsigned long)&sensor_nv_type);
                    nRet = 0;
                }
            }
            break;
        case IOCTL_PROXIMITY_COMMAND_SET:
            {
                nRet = copy_from_user(&command_data_type,
                        (void __user *)ulArg, sizeof(T_GP2AP003A10F_IOCTL_COMMAND) );
                if (nRet) {
                    printk("error : GP2AP003A10F_ioctl(cmd = IOCTL_PLOXIMIT_COMMAND_SET)\n" );
                    return -EFAULT;
                }
                set_sensor_command((unsigned long)&command_data_type);
                nRet = copy_to_user((void __user *)ulArg,
                        &command_data_type, sizeof(T_GP2AP003A10F_IOCTL_COMMAND) );
                if (nRet) {
                    printk("error : GP2AP003A10F_ioctl(cmd = IOCTL_PLOXIMIT_COMMAND_SET)\n" );
                    return -EFAULT;
                }
            }
            break;
        default:
            break;
    }
    DEBUG_PRINT("[OUT]GP2AP003A10F_ioctl nRet=%d\n",nRet);
    return nRet;
}

static struct file_operations GP2AP003A10F_fops = {
    .owner = THIS_MODULE,
    .open = GP2AP003A10F_open,
    .release = GP2AP003A10F_release,
    .unlocked_ioctl = GP2AP003A10F_ioctl,
};

static struct miscdevice GP2AP003A10F_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = GP2AP003A10F_DRV_NAME,
    .fops = &GP2AP003A10F_fops,
};

static int GP2AP003A10F_init_client(struct i2c_client *pClient_type)
{
    struct GP2AP003A10F_data *pData_type = NULL;
    struct input_dev        *pInput_dev = NULL;
    int nRet = -1;

    spin_lock_init( &sense_data_spin_lock_type );

    pData_type = i2c_get_clientdata(pClient_type);
    if(pData_type)
    {
        nRet = request_irq(pClient_type->irq, gp2ap_interrupt, IRQF_TRIGGER_LOW,
                            GP2AP003A10F_DRV_NAME, pData_type);

        gun_meas_state |= D_MEAS_STATE_PROX_INT;
    }
    if (nRet < 0)
    {
        printk(KERN_ERR "gp2ap003a10f_init_client: request irq failed\n");
        return nRet;
    }
    pInput_dev = input_allocate_device();

    if(pInput_dev)
    {
        pData_type->input_dev_type = pInput_dev;
        pInput_dev->name = GP2AP003A10F_DRV_NAME;
        pInput_dev->evbit[0] = BIT_MASK(EV_ABS);

        input_set_abs_params(pInput_dev, ABS_DISTANCE,
                                D_PROX_DETECT_STATE_NON_DETECT,
                                D_PROX_DETECT_STATE_DETECT, 0, 0);

        input_set_abs_params(pInput_dev, ABS_MISC,
                                0, D_LIGHT_MILLI_VOLTAGE_MAX, 0, 0);

        nRet = input_register_device(pInput_dev);
    }
    if (nRet < 0)
    {
        printk(KERN_ERR "gp2ap003a10f_init_client: input_register failed\n");
    }

    return nRet;
}

static unsigned long get_prox_sensor_val( void )
{
    unsigned long ulGet_data = 0;
    char ucData[D_RX_MSG_LENGTH] = {0,0};
    get_register(D_ADDR_PROX, ucData, D_RX_MSG_LENGTH);
    DEBUG_PRINT( "get_prox_sensor_val(): data = %d\n", (unsigned int)ucData[1] );
    ulGet_data = (ucData[1] & 0x01);
    return ( ulGet_data ) ;
}

static void gp2ap_input_report( int32_t nMode, unsigned long ulVal )
{
    struct GP2AP003A10F_data* pGP2AP_data = NULL;
    uint32_t unCode;
    DEBUG_PRINT("[IN]gp2ap_input_report\n");
    pGP2AP_data = i2c_get_clientdata(this_client_type);
    if(pGP2AP_data)
    {
        DEBUG_PRINT("Mode=%d ulVal=%ld\n", nMode, ulVal);

        unCode = (D_MODE_PROX == nMode) ? ABS_DISTANCE:ABS_MISC;
        input_report_abs(pGP2AP_data->input_dev_type,
                            unCode, ulVal);
        input_sync(pGP2AP_data->input_dev_type);
    }
    DEBUG_PRINT("[OUT]gp2ap_input_report\n");
}

static void gp2ap_work_func(struct work_struct *pWork_type)
{
    unsigned long  ulSensorVal = 0;
    uint32_t un_meas_state = 0;
    DEBUG_PRINT("[IN]gp2ap_work_func\n");

    spin_lock( &sense_data_spin_lock_type );

    un_meas_state = gun_meas_state;

    DEBUG_PRINT("gp2ap_work_func: old gul_prox_measurement_data=%d\n",
                    (int32_t)gul_prox_measurement_data);
    gul_prox_measurement_data = (gul_prox_measurement_data ^ 0x1)&0x1;
    DEBUG_PRINT("gp2ap_work_func: new gul_prox_measurement_data=%d\n",
                    (int32_t)gul_prox_measurement_data);

    spin_unlock( &sense_data_spin_lock_type );

    if(D_MEAS_STATE_DETECT&un_meas_state &&
            !(D_MEAS_STATE_START&un_meas_state))
    {
        DEBUG_PRINT("gp2ap_work_func:un_meas_state=%d\n",un_meas_state);
        DEBUG_PRINT("[OUT]gp2ap_work_func\n");
        return;
    }

    if(gul_prox_measurement_data)
    {
        set_register(D_ADDR_HYS, guc_prox_nv_far[guc_cam_temp_def]);
    } else
    {
        set_register(D_ADDR_HYS, guc_prox_nv_near[guc_cam_temp_def]);
    }

    set_register(D_ADDR_CON, D_CON_SET_OCON_ON);
    ulSensorVal = get_prox_sensor_val();

    if(ulSensorVal != gul_prox_measurement_data)
    {
        printk(KERN_ERR "gp2ap_work_func: mvo=%d != mo=%d\n",
                            (int32_t)ulSensorVal,
                            (int32_t)gul_prox_measurement_data);
        register_shutdown_disable(D_MODE_PROX);
        spin_lock( &sense_data_spin_lock_type );
        gul_prox_measurement_data = 0;
        spin_unlock( &sense_data_spin_lock_type );
        set_register(D_ADDR_GAIN, D_GAIN_SET);
        set_register(D_ADDR_CYCLE, D_CYCLE_SET);
        set_register(D_ADDR_OPMOD, D_OPMOD_SET_SD);
    }
    else
    {
        if(gul_prox_measurement_data)
        {
            gul_prox_timer_flag = 1;
            hrtimer_start(&gp2ap_irq_timer,
                        ktime_set(GP2AP_IRQ_TIMER / 1000,
                        (GP2AP_IRQ_TIMER % 1000) * 1000000),
                        HRTIMER_MODE_REL);
        }
        else
        {
            gp2ap_input_report(D_MODE_PROX, gul_prox_measurement_data);
        }
    }

    gp2ap_irq_enable();
    set_register(D_ADDR_CON, D_CON_SET_OCON);
    DEBUG_PRINT("[OUT]gp2ap_work_func\n");
}

static void gp2ap_irq_enable( void )
{
    uint32_t un_meas_state = 0;
    DEBUG_PRINT("[IN]gp2ap_irq_enable\n");
    spin_lock( &sense_data_spin_lock_type );
    un_meas_state = gun_meas_state;
    spin_unlock( &sense_data_spin_lock_type );
    if(!(un_meas_state&D_MEAS_STATE_PROX_INT))
    {
        DEBUG_PRINT("enable_irq\n");
        enable_irq(this_client_type->irq);
        spin_lock( &sense_data_spin_lock_type );
        gun_meas_state |= D_MEAS_STATE_PROX_INT;
        spin_unlock( &sense_data_spin_lock_type );
    }
    DEBUG_PRINT("[OUT]gp2ap_irq_enable\n");
}

static irqreturn_t gp2ap_interrupt(int32_t nIrq, void *pvDev_id)
{
    struct GP2AP003A10F_data *pData_type = pvDev_id;
    DEBUG_PRINT("[IN]gp2ap_interrupt\n");
    hrtimer_cancel( &gp2ap_irq_timer );
    gul_prox_timer_flag = 0;
    disable_irq_nosync(this_client_type->irq);
    spin_lock( &sense_data_spin_lock_type );
    gun_meas_state &= ~D_MEAS_STATE_PROX_INT;
    spin_unlock( &sense_data_spin_lock_type );
    schedule_work(&pData_type->work_type);
    DEBUG_PRINT("[OUT]gp2ap_interrupt\n");
    return IRQ_HANDLED;
}

static enum hrtimer_restart gp2ap_timer_func(struct hrtimer *timer)
{
    DEBUG_PRINT("[IN]gp2ap_timer_func\n");
    gul_prox_timer_flag = 0;
    gp2ap_input_report(D_MODE_PROX, gul_prox_measurement_data);
    DEBUG_PRINT("[OUT]gp2ap_timer_func\n");
    return HRTIMER_NORESTART;
}

static int32_t __devinit GP2AP003A10F_probe(struct i2c_client *pClient_type,
                   const struct i2c_device_id *id_type)
{
    struct GP2AP003A10F_data *pData_type = NULL;
    int32_t nErr = 0;

    DEBUG_PRINT("[IN]GP2AP003A10F_probe\n");

    pData_type = kzalloc(sizeof(struct GP2AP003A10F_data), GFP_KERNEL);
    if (!pData_type) {
        nErr = -ENOMEM;
        goto exit;
    }
    INIT_WORK(&pData_type->work_type, gp2ap_work_func);
    i2c_set_clientdata(pClient_type, pData_type);

    hrtimer_init( &gp2ap_irq_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
    gp2ap_irq_timer.function = gp2ap_timer_func;

    this_client_type = pClient_type;

    nErr = GP2AP003A10F_init_client(pClient_type);
    device_init_wakeup(&pClient_type->dev, 1);

    if (nErr)
        goto exit_kfree;

    GP2AP003A10F_init_module();

    nErr = misc_register(&GP2AP003A10F_device);

    if (nErr)
    {
        printk(KERN_ERR
               "GP2AP003A10F_probe: GP2AP003A10F register failed\n");
        goto exit_kfree;
    }

    DEBUG_PRINT("[OUT]GP2AP003A10F_probe\n");

    return 0;

exit_kfree:
    kfree(pData_type);
exit:
    return nErr;
}

static int32_t __devexit GP2AP003A10F_remove(struct i2c_client *pClient_type)
{
    struct GP2AP003A10F_data *pData_type = NULL;

    printk(KERN_INFO "[IN]GP2AP003A10F_remove\n");
    pData_type = i2c_get_clientdata(pClient_type);
    free_irq(pClient_type->irq, pData_type);
    input_unregister_device(pData_type->input_dev_type);
    misc_deregister(&GP2AP003A10F_device);
    i2c_release_client(pClient_type);

    kfree(i2c_get_clientdata(pClient_type));

    printk(KERN_INFO "[OUT]GP2AP003A10F_remove\n");

    return 0;
}

static int32_t GP2AP003A10F_suspend(struct i2c_client *pClient_type,
                    pm_message_t mesg_type)
{
    printk(KERN_INFO "[IN]GP2AP003A10F_suspend\n");

    if((gun_meas_state & D_MEAS_STATE_PROX_START) &&
       (device_may_wakeup(&pClient_type->dev)))
    {
        enable_irq_wake(this_client_type->irq);
        disable_irq(this_client_type->irq);
    }
    else
    {
        spin_lock( &sense_data_spin_lock_type );
        gun_meas_state_bk = gun_meas_state;
        gun_meas_state = D_MEAS_STATE_STOP;
        spin_unlock( &sense_data_spin_lock_type );
        register_shutdown_disable(D_MODE_PROX);
    }
    printk(KERN_INFO "[OUT]GP2AP003A10F_suspend\n");
    return 0;
}

static int32_t GP2AP003A10F_resume(struct i2c_client *pClient_type)
{
    int32_t nMode = 0;
    printk(KERN_INFO "[IN]GP2AP003A10F_resume\n");

    if((gun_meas_state & D_MEAS_STATE_PROX_START) &&
       (device_may_wakeup(&pClient_type->dev)))
    {
        disable_irq_wake(this_client_type->irq);
        enable_irq(this_client_type->irq);
    }
    else
    {
        if( gun_meas_state_bk&(D_MEAS_STATE_PROX_START|D_MEAS_STATE_ALS_START) )
        {
            nMode = (gun_meas_state_bk&D_MEAS_STATE_PROX_START) ?
                                            D_MODE_PROX:D_MODE_ALS;
            register_shutdown_enable(nMode);
        }
        spin_lock( &sense_data_spin_lock_type );
        gun_meas_state = gun_meas_state_bk;
        spin_unlock( &sense_data_spin_lock_type );
    }

    printk(KERN_INFO "[OUT]GP2AP003A10F_resume\n");
    return 0;
}

static const struct i2c_device_id GP2AP003A10F_id[] = {
    { GP2AP003A10F_DRV_NAME, 0 },
    { }
};

static struct i2c_driver GP2AP003A10F_driver = {
    .probe = GP2AP003A10F_probe,
    .remove = GP2AP003A10F_remove,
    .suspend    = GP2AP003A10F_suspend,
    .resume     = GP2AP003A10F_resume,
    .id_table = GP2AP003A10F_id,
    .driver = {
           .name = GP2AP003A10F_DRV_NAME,
           },
};

static int32_t __init GP2AP003A10F_init(void)
{
    printk(KERN_INFO "GP2AP003A10F_init: " GP2AP003A10F_DRV_NAME " driver ver." DRIVER_VERSION "\n" );

    return i2c_add_driver(&GP2AP003A10F_driver);
}

static void __exit GP2AP003A10F_exit(void)
{
    i2c_del_driver(&GP2AP003A10F_driver);
}

MODULE_AUTHOR("KYOCERA Co.,Ltd");
MODULE_DESCRIPTION("GP2AP003A10F driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(GP2AP003A10F_init);
module_exit(GP2AP003A10F_exit);
