/* This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 */
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/gpio.h>

#include <mach/camera.h>
#include <media/msm_camera.h>
#include <linux/interrupt.h>
#include <mach/vreg.h>
#include <mach/mpp.h>
#include <mach/gpio.h>
#include <linux/wait.h>
#include <linux/param.h>
#include <linux/clk.h>
#include "mlm1183.h"

#define  MLM1183_I2C_WR_PKT_HEADER  4
#define  MLM1183_INTERRUPT_PORT  33
#define  MLM1183_PMIC_PORT  6-1

//#define  MLM1183_LOG_ON
//#define  MLM1183_CALL_LOG_ON
//#define  MLM1183_DUMP_I2C_MESSAGE

#define  GPIO_LOW    0
#define  GPIO_HIGH   1
#define  PMIC_OFF    0
#define  PMIC_ON     1
#define  MCLK_OFF    0
#define  MCLK_ON     1

struct mlm1183_work_t {
    struct work_struct work;
};

static struct  mlm1183_work_t *mlm1183_sensorw;
static struct  i2c_client *mlm1183_client;

static DECLARE_WAIT_QUEUE_HEAD(mlm1183_wait_queue);
DEFINE_MUTEX(mlm1183_mut);

static const struct i2c_device_id mlm1183_i2c_id[] = {
    { "mlm1183", 2},
    { },
};

static int mlm1183_wait_flag = 0;

static struct msm_camera_sensor_info_mlm1183 *senser_info_mlm1183;

/*
* Functions
*/
static int mlm1183_i2c_probe(
    struct i2c_client *client,
    const struct i2c_device_id *id
);
static int __exit mlm1183_i2c_remove(struct i2c_client *client);
static void mlm1183_tasklet_proc( unsigned long data );
static irqreturn_t mlm1183_interrupt(int irq, void *handle);
static int mlm1183_sensor_init(
    const struct msm_camera_sensor_info* sensor_info
);
static int mlm1183_sensor_release(void);
static int mlm1183_sensor_i2c_write_transfer(
    unsigned short slave_addr,
    unsigned char *trans_buf,
    int len
);
static int mlm1183_sensor_i2c_read_transfer(
    unsigned short slave_addr,
    unsigned char *read_buf,
    int len
);
static int mlm1183_sensor_i2c_write(struct sensor_i2c_wr_cfg *i2c_wr_cfg);
static long mlm1183_sensor_i2c_read(struct sensor_i2c_rd_cfg *i2c_rd_cfg);
static int mlm1183_sensor_set_mclk( struct sensor_cfg_data* cfg_data );
static int mlm1183_sensor_set_pmic( struct sensor_cfg_data* cfg_data );
static int mlm1183_sensor_set_gpio( struct sensor_cfg_data* cfg_data );
static int mlm1183_sensor_wait_interrput( struct sensor_cfg_data* cfg_data );

/*
* mlm1183_i2c_probe
*/
static int mlm1183_i2c_probe(struct i2c_client *client,
    const struct i2c_device_id *id)
{
    int rc = 0;

    MLM1183_ENTER;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        rc = -ENOTSUPP;
    }

    if( !(rc < 0) ){
        mlm1183_sensorw = kzalloc(sizeof(struct mlm1183_work_t), GFP_KERNEL);
        if (!mlm1183_sensorw) {
            rc = -ENOMEM;
        }
    }

    if( !(rc < 0) ){
        i2c_set_clientdata(client, mlm1183_sensorw);
        mlm1183_client = client;
    }

    if( rc < 0 ){
        if( mlm1183_sensorw ){
            kfree(mlm1183_sensorw);
            mlm1183_sensorw = NULL;
        }
    }

    MLM1183_RETURN_N(rc);
}

/*
* mlm1183_i2c_remove
*/
static int __exit mlm1183_i2c_remove(struct i2c_client *client)
{
    struct s5k4eag_work_t *sensorw = i2c_get_clientdata(client);

    MLM1183_ENTER;

    free_irq(client->irq, sensorw);
    mlm1183_client = NULL;
    mlm1183_sensorw = NULL;
    kfree(sensorw);

    MLM1183_RETURN_N(0);
}

/*
* mlm1183_i2c_driver
*/
static struct i2c_driver mlm1183_i2c_driver = {
    .id_table = mlm1183_i2c_id,
    .probe  = mlm1183_i2c_probe,
    .remove = __exit_p(mlm1183_i2c_remove),
    .driver = {
        .name = "mlm1183",
    },
};

/*
* mlm1183_tasklet_proc
*/
static void mlm1183_tasklet_proc( unsigned long data )
{
    MLM1183_ENTER;

    if (!gpio_get_value(MLM1183_INTERRUPT_PORT)) {
        MLM1183_RETURN;
    }

    mlm1183_wait_flag = 1;
    wake_up_interruptible(&mlm1183_wait_queue);

    MLM1183_RETURN;
}

/*
* DECLARE_TASKLET
*/
DECLARE_TASKLET( mlm1183_tasklet, mlm1183_tasklet_proc, ( unsigned long )NULL );

/*
* mlm1183_interrupt
*/
static irqreturn_t mlm1183_interrupt(int irq, void *handle)
{
    MLM1183_ENTER;

    tasklet_schedule( &mlm1183_tasklet );

    MLM1183_RETURN_N(IRQ_HANDLED);
}

/*
* mlm1183_sensor_init
*  - It is called from OPEN.
*/
static int mlm1183_sensor_init(
    const struct msm_camera_sensor_info* sensor_info
)
{
    long rc = 0;
    struct sensor_cfg_data cfg_data;
    static struct vreg *vreg_cfg;

    MLM1183_ENTER;

    senser_info_mlm1183 = (struct msm_camera_sensor_info_mlm1183 *)sensor_info;
    mlm1183_wait_flag = 0;

    gpio_request(senser_info_mlm1183->sensor_info.sensor_pwd, "PWD");
    gpio_request(senser_info_mlm1183->sensor_info.sensor_reset, "RST");

    /* VCAML provide start */
    cfg_data.cfg.gpio_cfg.port = senser_info_mlm1183->sensor_info.sensor_pwd;
    cfg_data.cfg.gpio_cfg.ctl = GPIO_HIGH;
    rc = mlm1183_sensor_set_gpio( &cfg_data );
    if(rc != 0){
        MLM1183_RETURN_N(rc);
    }
    
    /* Wait:1ms */
    mdelay(1);
    
    /* VCAMD provide start */
    cfg_data.cfg.pmic_cfg.id = senser_info_mlm1183->vcmd_pwd;
    cfg_data.cfg.pmic_cfg.ctl =  PMIC_ON;
    rc = mlm1183_sensor_set_pmic( &cfg_data );
    if(rc != 0){
        MLM1183_RETURN_N(rc);
    }
    
    /* VCAMA provide start */
    cfg_data.cfg.pmic_cfg.id = senser_info_mlm1183->vcma_pwd;
    cfg_data.cfg.pmic_cfg.ctl =  PMIC_ON;
    vreg_cfg = vreg_get(NULL, cfg_data.cfg.pmic_cfg.id);
    vreg_set_level(vreg_cfg, 2800);
    rc = mlm1183_sensor_set_pmic( &cfg_data );
    if(rc != 0){
        MLM1183_RETURN_N(rc);
    }
    
    /* VCAMAF provide start */
    cfg_data.cfg.pmic_cfg.id = senser_info_mlm1183->vcmaf_pwd;
    cfg_data.cfg.pmic_cfg.ctl =  PMIC_ON;
    vreg_cfg = vreg_get(NULL, cfg_data.cfg.pmic_cfg.id);
    vreg_set_level(vreg_cfg, 2800);
    rc = mlm1183_sensor_set_pmic( &cfg_data );
    if(rc != 0){
        MLM1183_RETURN_N(rc);
    }
    
    /* Wait:1ms */
    msleep(1);
    
    /* CAM_CLK provide start */
    cfg_data.cfg.mclk_ctl = MCLK_ON;
    rc = mlm1183_sensor_set_mclk( &cfg_data );
    if(rc != 0){
        MLM1183_RETURN_N(rc);
    }

    /* Wait:5ms */
    msleep(5);

    /* Reset CAMIF PAD REG */
    msm_camio_camif_pad_reg_reset();
    
    /* Wait:5ms */
    msleep(5);
    
    /* reset cancel */
    cfg_data.cfg.gpio_cfg.port = senser_info_mlm1183->sensor_info.sensor_reset;
    cfg_data.cfg.gpio_cfg.ctl = GPIO_HIGH;
    rc = mlm1183_sensor_set_gpio( &cfg_data );
    if(rc != 0){
        MLM1183_RETURN_N(rc);
    }
    
    rc = request_irq( gpio_to_irq(MLM1183_INTERRUPT_PORT),
                      mlm1183_interrupt,
                      IRQF_TRIGGER_RISING,
                      "cam_trig_h",
                      0 );
    if(rc != 0){
        MLM1183_RETURN_N(rc);
    }
    
    MLM1183_RETURN_N(rc);
}

/*
* mlm1183_sensor_release
*  - It is called from CLOSE.
*/
static int mlm1183_sensor_release(void)
{
    int rc = 0;
    struct sensor_cfg_data cfg_data;

    MLM1183_ENTER;

    mutex_lock(&mlm1183_mut);

    /* VCAMD provide stop */
    cfg_data.cfg.pmic_cfg.id = senser_info_mlm1183->vcmd_pwd;
    cfg_data.cfg.pmic_cfg.ctl =  PMIC_OFF;
    rc = mlm1183_sensor_set_pmic( &cfg_data );
    if(rc != 0){
        MLM1183_RETURN_N(rc);
    }
    
    /* VCAMA provide stop */
    cfg_data.cfg.pmic_cfg.id = senser_info_mlm1183->vcma_pwd;
    cfg_data.cfg.pmic_cfg.ctl =  PMIC_OFF;
    rc = mlm1183_sensor_set_pmic( &cfg_data );
    if(rc != 0){
        MLM1183_RETURN_N(rc);
    }
    
    /* VCAMAF provide stop */
    cfg_data.cfg.pmic_cfg.id = senser_info_mlm1183->vcmaf_pwd;
    cfg_data.cfg.pmic_cfg.ctl =  PMIC_OFF;
    rc = mlm1183_sensor_set_pmic( &cfg_data );
    if(rc != 0){
        MLM1183_RETURN_N(rc);
    }
    
    /* Wait:1ms */
    msleep(1);
    
    /* VCAML provide stop */
    cfg_data.cfg.gpio_cfg.port = senser_info_mlm1183->sensor_info.sensor_pwd;
    cfg_data.cfg.gpio_cfg.ctl = GPIO_LOW;
    rc = mlm1183_sensor_set_gpio( &cfg_data );
    if(rc != 0){
        MLM1183_RETURN_N(rc);
    }
    
    /* Wait:15ms */
    msleep(15);
    
    /* reset setting */
    cfg_data.cfg.gpio_cfg.port = senser_info_mlm1183->sensor_info.sensor_reset;
    cfg_data.cfg.gpio_cfg.ctl = GPIO_LOW;
    rc = mlm1183_sensor_set_gpio( &cfg_data );
    if(rc != 0){
        MLM1183_RETURN_N(rc);
    }
    
    /* CAM_CLK provide stop */
    cfg_data.cfg.mclk_ctl = MCLK_OFF;
    rc = mlm1183_sensor_set_mclk( &cfg_data );
    if(rc != 0){
        MLM1183_RETURN_N(rc);
    }
    
    /* Wait:100ms */
    msleep(100);

    free_irq( gpio_to_irq(MLM1183_INTERRUPT_PORT), 0 );
    tasklet_kill( &mlm1183_tasklet );

    gpio_free(senser_info_mlm1183->sensor_info.sensor_reset);
    gpio_free(senser_info_mlm1183->sensor_info.sensor_pwd);

    mlm1183_wait_flag = 0;

    mutex_unlock(&mlm1183_mut);

    MLM1183_RETURN_N(rc);
}

/*
* mlm1183_sensor_i2c_transfer
*/
static int mlm1183_sensor_i2c_write_transfer(
    unsigned short slave_addr,
    unsigned char *trans_buf,
    int len
)
{
    int rc = 0;
#ifdef  MLM1183_DUMP_I2C_MESSAGE
    int i;
#endif

    struct i2c_msg msg[] = {
        {
            .addr = slave_addr,
            .flags = 0,
            .len = len,
            .buf = trans_buf,
        },
    };

    MLM1183_ENTER;

    if (i2c_transfer(mlm1183_client->adapter, msg, 1) < 0) {
        rc = -EIO;
    }

#ifdef  MLM1183_DUMP_I2C_MESSAGE
    MLM1183_LOG_DBG("slave_addr[0x%04x], len[0x%04x]", slave_addr, len);

    for(i=0; i<len; i++)
    {
        MLM1183_LOG_DBG("[0x%02x]", *(trans_buf+i));
    }
#endif

    MLM1183_RETURN_N(rc);
}

/*
* mlm1183_sensor_i2c_transfer
*/
static int mlm1183_sensor_i2c_read_transfer(
    unsigned short slave_addr,
    unsigned char *read_buf,
    int len
)
{
    int rc = 0;
#ifdef  MLM1183_DUMP_I2C_MESSAGE
    int i;
#endif

    struct i2c_msg msg[] = {
        {
            .addr = slave_addr,
            .flags = I2C_M_RD,
            .len = len,
            .buf = read_buf,
        },
    };

    MLM1183_ENTER;

    if (i2c_transfer(mlm1183_client->adapter, msg, 1) < 0) {
        rc = -EIO;
    }

#ifdef  MLM1183_DUMP_I2C_MESSAGE
    MLM1183_LOG_DBG("slave_addr[0x%04x], len[0x%04x]", slave_addr, len);

    for(i=0; i<len; i++)
    {
        MLM1183_LOG_DBG("[0x%02x]", *(read_buf+i));
    }
#endif

    MLM1183_RETURN_N(rc);
}

/*
* mlm1183_sensor_i2c_write
*/
static int mlm1183_sensor_i2c_write(struct sensor_i2c_wr_cfg *i2c_wr_cfg)
{
    int rc = 0;
    unsigned char *write_buf;

    MLM1183_ENTER;

    write_buf = (unsigned char *)kzalloc((size_t)i2c_wr_cfg->len,GFP_KERNEL);
    if( !write_buf ){
        rc = -EFAULT;
    }

    if( !(rc < 0) ){
        write_buf[0] = i2c_wr_cfg->len;
        write_buf[1] = i2c_wr_cfg->accsess_code;
        write_buf[2] = i2c_wr_cfg->id;
        write_buf[3] = i2c_wr_cfg->reg_addr;

        if ( copy_from_user( &write_buf[4],
                             (void *)(i2c_wr_cfg->write_data_ptr),
                             (i2c_wr_cfg->len - MLM1183_I2C_WR_PKT_HEADER) ) ){
            rc = -EFAULT;
        }
    }

    if( !(rc < 0) ){
        rc = mlm1183_sensor_i2c_write_transfer( mlm1183_client->addr,
                                                write_buf,
                                                i2c_wr_cfg->len );
    }

    if( write_buf )
        kfree(write_buf);

    MLM1183_RETURN_N(rc);
}
/*
* mlm1183_sensor_i2c_read
*/
static long mlm1183_sensor_i2c_read(struct sensor_i2c_rd_cfg *i2c_rd_cfg)
{
    int   rc = 0;
    unsigned char *read_buf = NULL;

    MLM1183_ENTER;

    read_buf = (unsigned char *)kzalloc((size_t)i2c_rd_cfg->len, GFP_KERNEL);
    if( !read_buf ){
        rc = -EFAULT;
    }

    if( !(rc < 0) ) {
        rc = mlm1183_sensor_i2c_read_transfer( mlm1183_client->addr,
                                               read_buf,
                                               i2c_rd_cfg->len );
    }

    if( !(rc < 0) ) {
        if (copy_to_user((void *)(i2c_rd_cfg->read_data_ptr),
                          &read_buf[0],
                          i2c_rd_cfg->len) ){
            rc = -EFAULT;
        }
    }

    if( read_buf )
        kfree(read_buf);

    MLM1183_RETURN_N(rc);
}

/*
* mlm1183_sensor_set_mclk
*/
static int mlm1183_sensor_set_mclk( struct sensor_cfg_data* cfg_data )
{
    int rc = 0;
    struct clk *clk = NULL;

    MLM1183_ENTER;

    if(cfg_data->cfg.mclk_ctl){
        gpio_tlmm_config(GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), GPIO_CFG_ENABLE);

        clk = clk_get(NULL, "cam_m_clk");
        msm_camio_clk_rate_set_2(clk, 24000000);
        clk_enable(clk);
    }

    MLM1183_RETURN_N(rc);
}

/*
* mlm1183_sensor_set_pmic
*/
static int mlm1183_sensor_set_pmic( struct sensor_cfg_data* cfg_data )
{
    int rc = 0;
    static struct vreg *vreg_cfg;

    MLM1183_ENTER;

    vreg_cfg = vreg_get(NULL, cfg_data->cfg.pmic_cfg.id);
    if(cfg_data->cfg.pmic_cfg.ctl){
        vreg_enable(vreg_cfg);
    }
    else{
        vreg_disable(vreg_cfg);
    }

    MLM1183_RETURN_N(rc);
}

/*
* mlm1183_sensor_set_gpio
*/
static int mlm1183_sensor_set_gpio( struct sensor_cfg_data* cfg_data )
{
    int rc = 0;

    MLM1183_ENTER;

    if(cfg_data->cfg.gpio_cfg.ctl){
        gpio_direction_output( cfg_data->cfg.gpio_cfg.port, 1 );
    }
    else{
        gpio_direction_output( cfg_data->cfg.gpio_cfg.port, 0 );
    }

    MLM1183_RETURN_N(rc);
}

/*
* mlm1183_sensor_wait_interrput
*/
static int mlm1183_sensor_wait_interrput( struct sensor_cfg_data* cfg_data )
{
    int rc = 0;
    unsigned long timeout = 0;
/*    float sys_time_cycle_ms = 0;*/
    unsigned int sys_time_cycle_ms = 0;

    MLM1183_ENTER;

    if( HZ != 0 ){
       sys_time_cycle_ms = 1000/HZ;
       if( (cfg_data->cfg.wait_timeout_ms) > sys_time_cycle_ms ){
            timeout = (cfg_data->cfg.wait_timeout_ms)/sys_time_cycle_ms;
       }
    }

    if( timeout != 0 ){
        rc = wait_event_interruptible_timeout(mlm1183_wait_queue,
                                              mlm1183_wait_flag != 0,
                                              timeout);
        mlm1183_wait_flag = 0;

        if(rc < 0){
            MLM1183_RETURN_N(-1);
        } else if(rc == 0){
            MLM1183_RETURN_N(-2);
        } else {
            rc = 0;
        }
    }

    MLM1183_RETURN_N(rc);
}

/*
* mlm1183_sensor_config
*  - It is called from IOCTL.
*/
static int mlm1183_sensor_config(void __user *argp)
{
    struct sensor_cfg_data cfg_data;
    int   rc = 0;

    MLM1183_ENTER;

    if ( copy_from_user( &cfg_data,
                         (void *)argp,
                         sizeof(struct sensor_cfg_data) ) )
        MLM1183_RETURN_N(-EFAULT);

    mutex_lock(&mlm1183_mut);

    switch (cfg_data.cfgtype) {
    case CFG_I2C_WRITE:
        rc = mlm1183_sensor_i2c_write( &cfg_data.cfg.i2c_wr_cfg );
        break;
    case CFG_I2C_READ:
        rc = mlm1183_sensor_i2c_read( &cfg_data.cfg.i2c_rd_cfg );
        break;
    case CFG_WAIT_INTERRPUT:
        rc = mlm1183_sensor_wait_interrput( &cfg_data );
        break;
    default:
        rc = -ENOTTY;
        break;
    }

    mutex_unlock(&mlm1183_mut);

    MLM1183_RETURN_N(rc);
}

/*
* mlm1183_sensor_probe
*
*/
static int mlm1183_sensor_probe(const struct msm_camera_sensor_info *info,
                struct msm_sensor_ctrl *s)
{
    int   rc = 0;

    MLM1183_ENTER;

    rc = i2c_add_driver(&mlm1183_i2c_driver);
    if (rc < 0) {
        rc = -ENOTSUPP;
        goto probe_fail;
    }

    s->s_init = mlm1183_sensor_init;
    s->s_release = mlm1183_sensor_release;
    s->s_config  = mlm1183_sensor_config;
    s->s_camera_type = BACK_CAMERA_2D;
    s->s_mount_angle = 0;

probe_fail:
    MLM1183_RETURN_N(0);
}

/*
* __mlm1183_probe
*/
static int __mlm1183_probe(struct platform_device *pdev)
{
    int rc;

    MLM1183_ENTER;
    rc = msm_camera_drv_start(pdev, mlm1183_sensor_probe);
    MLM1183_RETURN_N( rc );
}

/*
* msm_camera_driver
*/
static struct platform_driver msm_camera_driver = {
    .probe = __mlm1183_probe,
    .driver = {
        .name = "msm_camera_mlm1183",
        .owner = THIS_MODULE,
    },
};

/*
* mlm1183_init
*/
static int __init mlm1183_init(void)
{
    int rc;

    MLM1183_ENTER;
    rc = platform_driver_register(&msm_camera_driver);
    MLM1183_RETURN_N( rc );
}

/*
* MODULE_INIT
*/
module_init(mlm1183_init);

MODULE_DESCRIPTION("MLM183 sensor driver");
MODULE_LICENSE("GPL v2");
