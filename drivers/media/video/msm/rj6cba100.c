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
#include <linux/slab.h>
#include <linux/clk.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include <mach/vreg.h>
#include "rj6cba100.h"

//#define  RJ6CBA100_LOG_ON
//#define  RJ6CBA100_CALL_LOG_ON
//#define  RJ6CBA100_DUMP_I2C_MESSAGE

/*=============================================================
	SENSOR REGISTER DEFINES
==============================================================*/
#define PMIC_OFF    0
#define PMIC_ON     1

/*============================================================================
							DATA DECLARATIONS
============================================================================*/
struct rj6cba100_work_t {
	struct work_struct work;
};
static struct  rj6cba100_work_t *rj6cba100_sensorw;
static struct  i2c_client *rj6cba100_client;
struct rj6cba100_ctrl_t {
	const struct  msm_camera_sensor_info_rj6cba100 *sensordata;
};
static struct rj6cba100_ctrl_t *rj6cba100_ctrl;
static DECLARE_WAIT_QUEUE_HEAD(rj6cba100_wait_queue);
DEFINE_MUTEX(rj6cba100_mut);


/*============================================================================
							Functions
============================================================================*/
static int rj6cba100_i2c_rxdata(unsigned short saddr, unsigned char *rxdata, int length);
static int32_t rj6cba100_i2c_txdata(unsigned short saddr, unsigned char *txdata, int length);
static int32_t rj6cba100_i2c_read(uint8_t raddr, uint8_t *rdata, int rlen);
static int32_t rj6cba100_i2c_write_b_sensor(uint8_t waddr, uint8_t bdata);
static int32_t rj6cba100_power_down(void);
static int rj6cba100_probe_init_done(const struct msm_camera_sensor_info *data);
static int rj6cba100_probe_init_sensor(const struct msm_camera_sensor_info *data);
int rj6cba100_sensor_init(const struct msm_camera_sensor_info *data);
static int rj6cba100_init_client(struct i2c_client *client);
static int rj6cba100_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __exit rj6cba100_i2c_remove(struct i2c_client *client);
static int32_t rj6cba100_csi_setting(void);
int rj6cba100_sensor_config(void __user *argp);
static int rj6cba100_sensor_set_pmic( struct sensor_cfg_data* cfg_data, int vol );
static int rj6cba100_sensor_i2c_write(struct sensor_i2c_wr_cfg *i2c_wr_cfg);
static long rj6cba100_sensor_i2c_read(struct sensor_i2c_rd_cfg *i2c_rd_cfg);
static int rj6cba100_sensor_release(void);
static int rj6cba100_sensor_probe(const struct msm_camera_sensor_info *info, struct msm_sensor_ctrl *s);
static int __rj6cba100_probe(struct platform_device *pdev);
static int __init rj6cba100_init(void);

/*=============================================================*/

static int rj6cba100_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;

	RJ6CBA100_ENTER;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		RJ6CBA100_LOG_ERR("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	rj6cba100_sensorw = kzalloc(sizeof(struct rj6cba100_work_t), GFP_KERNEL);
	if (!rj6cba100_sensorw) {
		RJ6CBA100_LOG_ERR("kzalloc failed.\n");
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, rj6cba100_sensorw);
	rj6cba100_init_client(client);
	rj6cba100_client = client;

	mdelay(50);

	RJ6CBA100_LOG_DBG("rj6cba100_probe successed! rc = %d\n", rc);

	RJ6CBA100_RETURN_N(0);

probe_failure:
	RJ6CBA100_LOG_ERR("rj6cba100_probe failed! rc = %d\n", rc);

	RJ6CBA100_RETURN_N(rc);
}

static int __exit rj6cba100_i2c_remove(struct i2c_client *client)
{
	struct rj6cba100_work_t_t *sensorw;
	
	RJ6CBA100_ENTER;

	sensorw = i2c_get_clientdata(client);
	free_irq(client->irq, sensorw);
	rj6cba100_client = NULL;
	kfree(sensorw);

	RJ6CBA100_RETURN_N(0);
}

static int rj6cba100_init_client(struct i2c_client *client)
{
	RJ6CBA100_ENTER;

	/* Initialize the MSM_CAMI2C Chip */
	init_waitqueue_head(&rj6cba100_wait_queue);

	RJ6CBA100_RETURN_N(0);
}

static const struct i2c_device_id rj6cba100_i2c_id[] = {
	{"rj6cba100", 0},
	{ }
};

static int rj6cba100_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
{
#ifdef RJ6CBA100_DUMP_I2C_MESSAGE
    int i;
#endif

	struct i2c_msg msgs[] = {
		{
			.addr  = saddr,
			.flags = 0,
			.len   = 1,
			.buf   = rxdata,
		},
		{
			.addr  = saddr,
			.flags = I2C_M_RD,
			.len   = 1,
			.buf   = rxdata,
		},
	};

	RJ6CBA100_ENTER;

#ifdef RJ6CBA100_DUMP_I2C_MESSAGE
    RJ6CBA100_LOG_DBG("slave_addr[0x%04X], len[0x%04X]", saddr, length);
    RJ6CBA100_LOG_DBG("reg_addr[0x%04X]", *rxdata);
#endif

	if (i2c_transfer(rj6cba100_client->adapter, msgs, 2) < 0) {
		RJ6CBA100_LOG_ERR("rj6cba100_i2c_rxdata failed!\n");
		RJ6CBA100_RETURN_N(-EIO);
	}

#ifdef RJ6CBA100_DUMP_I2C_MESSAGE
    for(i=0; i<length; i++)
    {
        RJ6CBA100_LOG_DBG("[0x%02X]", *(rxdata+i));
    }
#endif
	
	RJ6CBA100_RETURN_N(0);
}
static int32_t rj6cba100_i2c_txdata(unsigned short saddr,
				unsigned char *txdata, int length)
{

	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = 2,
			.buf = txdata,
		},
	};

	RJ6CBA100_ENTER;

	if (i2c_transfer(rj6cba100_client->adapter, msg, 1) < 0) {
		RJ6CBA100_LOG_ERR("rj6cba100_i2c_txdata faild 0x%x\n", rj6cba100_client->addr);
		RJ6CBA100_RETURN_N(-EIO);
	}

	RJ6CBA100_RETURN_N(0);
}

static int32_t rj6cba100_i2c_read(uint8_t raddr,
	uint8_t *rdata, int rlen)
{
	int32_t rc = 0;
	unsigned char buf[1];

	RJ6CBA100_ENTER;

	if (!rdata)
		RJ6CBA100_RETURN_N(-EIO);
	memset(buf, 0, sizeof(buf));
	buf[0] = raddr;
	rc = rj6cba100_i2c_rxdata(rj6cba100_client->addr, buf, rlen);
	if (rc < 0) {
		RJ6CBA100_LOG_ERR("rj6cba100_i2c_read 0x%x failed!\n", raddr);
		RJ6CBA100_RETURN_N(rc);
	}
	*rdata = buf[0];
	
	RJ6CBA100_RETURN_N(rc);
}
static int32_t rj6cba100_i2c_write_b_sensor(uint8_t waddr, uint8_t bdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[2];

	RJ6CBA100_ENTER;

	memset(buf, 0, sizeof(buf));
	buf[0] = waddr;
	buf[1] = bdata;
	RJ6CBA100_LOG_DBG("i2c_write_b addr = 0x%x, val = 0x%x\n", waddr, bdata);
	rc = rj6cba100_i2c_txdata(rj6cba100_client->addr, buf, 2);
	if (rc < 0)
		RJ6CBA100_LOG_ERR("i2c_write_b failed, addr = 0x%x, val = 0x%x!\n",
			waddr, bdata);
	
	RJ6CBA100_RETURN_N(rc);
}

static int32_t rj6cba100_power_down(void)
{
	RJ6CBA100_ENTER;
	RJ6CBA100_RETURN_N(0);
}
static int rj6cba100_probe_init_done(const struct msm_camera_sensor_info *data)
{
	RJ6CBA100_ENTER;
	RJ6CBA100_RETURN_N(0);
}

static int rj6cba100_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;
	struct msm_camera_sensor_info_rj6cba100 *sensordata;
	struct sensor_cfg_data cfg_data;
	struct clk *clk = NULL;
	spinlock_t rj6cba100_spinlock;
	unsigned long flags;
	
	RJ6CBA100_ENTER;

	sensordata = (struct msm_camera_sensor_info_rj6cba100 *)data;

	spin_lock_init(&rj6cba100_spinlock);

	rc = gpio_request(sensordata->sensor_info.sensor_reset, "rj6cba100");
	if (!rc) {
		gpio_direction_output(sensordata->sensor_info.sensor_reset, 0);
		gpio_set_value(sensordata->sensor_info.sensor_reset, 1);
	} else {
		RJ6CBA100_LOG_ERR("gpio reset fail");
	}

	spin_lock_irqsave(&rj6cba100_spinlock, flags);

	/* VCAMD provide start */
	cfg_data.cfg.pmic_cfg.id = rj6cba100_ctrl->sensordata->vcmd_pwd;
	cfg_data.cfg.pmic_cfg.ctl =  PMIC_ON;
	rc = rj6cba100_sensor_set_pmic( &cfg_data, 1800 );
	if(rc != 0){
		RJ6CBA100_RETURN_N(rc);
	}

	udelay(100);
	
	/* VCAMA provide start */
	cfg_data.cfg.pmic_cfg.id = rj6cba100_ctrl->sensordata->vcma_pwd;
	cfg_data.cfg.pmic_cfg.ctl =  PMIC_ON;
	rc = rj6cba100_sensor_set_pmic( &cfg_data, 2800 );
	if(rc != 0){
		RJ6CBA100_RETURN_N(rc);
	}

	spin_unlock_irqrestore(&rj6cba100_spinlock, flags);

	mdelay(5);
	
	gpio_tlmm_config(GPIO_CFG(16, 3, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), GPIO_CFG_ENABLE);

	/* enable mclk */
	clk = clk_get(NULL, "gp_clk");
	msm_camio_clk_rate_set_2(clk, 24000000);
	if(!IS_ERR(clk))
		clk_enable(clk);

	RJ6CBA100_LOG_DBG("sensor_reset = %d\n", sensordata->sensor_info.sensor_reset);
	gpio_set_value(sensordata->sensor_info.sensor_reset, 0);
	mdelay(5);

	RJ6CBA100_RETURN_N(rc);
}

int rj6cba100_sensor_init(const struct msm_camera_sensor_info *data)
{
	int32_t rc = 0;

	RJ6CBA100_ENTER;
	RJ6CBA100_LOG_DBG("pdata: %p\n", data);

	rj6cba100_ctrl = kzalloc(sizeof(struct rj6cba100_ctrl_t), GFP_KERNEL);
	if (!rj6cba100_ctrl) {
		RJ6CBA100_LOG_ERR("rj6cba100_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	if (data)
		rj6cba100_ctrl->sensordata = (struct msm_camera_sensor_info_rj6cba100 *)data;

	rc = rj6cba100_probe_init_sensor(data);
	if (rc < 0) {
		RJ6CBA100_LOG_ERR("Calling rj6cba100_sensor_init fail\n");
		goto init_fail;
	} else
		goto init_done;
init_fail:
	RJ6CBA100_LOG_ERR(" rj6cba100_sensor_init fail\n");
	rj6cba100_probe_init_done(data);
	kfree(rj6cba100_ctrl);
init_done:

	RJ6CBA100_RETURN_N(rc);
}

static int rj6cba100_sensor_release(void)
{
	int rc = -EBADF;
	struct sensor_cfg_data cfg_data;

	RJ6CBA100_ENTER;

	mutex_lock(&rj6cba100_mut);
	rj6cba100_power_down();
	gpio_free(rj6cba100_ctrl->sensordata->sensor_info.sensor_reset);

	/* disable mclk */
	clk_disable(clk_get(NULL, "gp_clk"));

	mdelay(5);

	/* VCAMA provide stop */
	cfg_data.cfg.pmic_cfg.id = rj6cba100_ctrl->sensordata->vcma_pwd;
	cfg_data.cfg.pmic_cfg.ctl =  PMIC_OFF;
	rc = rj6cba100_sensor_set_pmic( &cfg_data, 0 );
	if(rc != 0){
		RJ6CBA100_RETURN_N(rc);
	}
	
	mdelay(5);

	/* VCAMD provide stop */
	cfg_data.cfg.pmic_cfg.id = rj6cba100_ctrl->sensordata->vcmd_pwd;
	cfg_data.cfg.pmic_cfg.ctl =  PMIC_OFF;
	rc = rj6cba100_sensor_set_pmic( &cfg_data, 0 );
	if(rc != 0){
		RJ6CBA100_RETURN_N(rc);
	}
	
	kfree(rj6cba100_ctrl);
	rj6cba100_ctrl = NULL;

	mutex_unlock(&rj6cba100_mut);

	mdelay(100);

	RJ6CBA100_RETURN_N(rc);
}

static struct i2c_driver rj6cba100_i2c_driver = {
	.id_table = rj6cba100_i2c_id,
	.probe  = rj6cba100_i2c_probe,
	.remove = __exit_p(rj6cba100_i2c_remove),
	.driver = {
		.name = "rj6cba100",
	},
};

static int32_t rj6cba100_csi_setting()
{
	int32_t rc = 0;
	struct msm_camera_csi_params rj6cba100_csi_params;

	RJ6CBA100_ENTER;

	rj6cba100_csi_params.lane_cnt = 1;
	rj6cba100_csi_params.data_format = CSI_8BIT;
	rj6cba100_csi_params.lane_assign = 0xe4;
	rj6cba100_csi_params.dpcm_scheme = 0;
	rj6cba100_csi_params.settle_cnt = 0x0a;

	rc = msm_camio_csi_config(&rj6cba100_csi_params);
	mdelay(10);

	RJ6CBA100_RETURN_N(rc);
}

int rj6cba100_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cdata;
	long   rc = 0;

	RJ6CBA100_ENTER;

	if (copy_from_user(&cdata,
		(void *)argp,
		sizeof(struct sensor_cfg_data)))
		RJ6CBA100_RETURN_N(-EFAULT);
	mutex_lock(&rj6cba100_mut);
	
	RJ6CBA100_LOG_DBG("rj6cba100_sensor_config: cfgtype = %d\n",cdata.cfgtype);
	
	switch (cdata.cfgtype) {
	case CFG_START:
		rc = rj6cba100_csi_setting();
		break;

	case CFG_I2C_WRITE:
		rc = rj6cba100_sensor_i2c_write( &cdata.cfg.i2c_wr_cfg );
		break;
	case CFG_I2C_READ:
		rc = rj6cba100_sensor_i2c_read( &cdata.cfg.i2c_rd_cfg );
		break;
	default:
		rc = -ENOTTY;
		break;
	}
	mutex_unlock(&rj6cba100_mut);

	RJ6CBA100_RETURN_N(rc);
}

/*
* rj6cba100_sensor_set_pmic
*/
static int rj6cba100_sensor_set_pmic( struct sensor_cfg_data* cfg_data, int vol )
{
    int rc = 0;
    static struct vreg *vreg_cfg;

    RJ6CBA100_ENTER;

    vreg_cfg = vreg_get(NULL, cfg_data->cfg.pmic_cfg.id);
    if(cfg_data->cfg.pmic_cfg.ctl){
        vreg_set_level(vreg_cfg, vol);
        vreg_enable(vreg_cfg);
    }
    else{
        vreg_disable(vreg_cfg);
    }

    RJ6CBA100_RETURN_N(rc);
}

/*
* rj6cba100_sensor_i2c_write
*/
static int rj6cba100_sensor_i2c_write(struct sensor_i2c_wr_cfg *i2c_wr_cfg)
{
	int rc = 0;
	unsigned char write_buf;

	RJ6CBA100_ENTER;

	if( !(rc < 0) ){
		if ( copy_from_user( &write_buf,
							 (void *)(i2c_wr_cfg->write_data_ptr),
							 1 ) ){
			rc = -EFAULT;
		}
	}

	if( !(rc < 0) ){
		rc = rj6cba100_i2c_write_b_sensor( i2c_wr_cfg->reg_addr,
											   write_buf);
	}

	RJ6CBA100_RETURN_N(rc);
}
/*
* rj6cba100_sensor_i2c_read
*/
static long rj6cba100_sensor_i2c_read(struct sensor_i2c_rd_cfg *i2c_rd_cfg)
{
	int   rc = 0;
	unsigned char *read_buf = NULL;

	RJ6CBA100_ENTER;

	read_buf = (unsigned char *)kzalloc((size_t)i2c_rd_cfg->len, GFP_KERNEL);
	if( !read_buf ){
		rc = -EFAULT;
	}

	if( !(rc < 0) ) {
		rc = rj6cba100_i2c_read( i2c_rd_cfg->reg_addr,
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

	RJ6CBA100_RETURN_N(rc);
}

static int rj6cba100_sensor_probe(const struct msm_camera_sensor_info *info,
		struct msm_sensor_ctrl *s)
{
	int rc = 0;

	RJ6CBA100_ENTER;

	rc = i2c_add_driver(&rj6cba100_i2c_driver);
	if (rc < 0 || rj6cba100_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_fail;
	}
	
	s->s_init = rj6cba100_sensor_init;
	s->s_release = rj6cba100_sensor_release;
	s->s_config  = rj6cba100_sensor_config;
	s->s_camera_type = FRONT_CAMERA_2D;
	s->s_mount_angle = 270;
	rj6cba100_probe_init_done(info);

	RJ6CBA100_RETURN_N(rc);

probe_fail:
	RJ6CBA100_LOG_ERR("rj6cba100_sensor_probe: SENSOR PROBE FAILS!\n");
	i2c_del_driver(&rj6cba100_i2c_driver);

	RJ6CBA100_RETURN_N(rc);
}

static int __rj6cba100_probe(struct platform_device *pdev)
{
	int rc;
	
	RJ6CBA100_ENTER;

	rc = msm_camera_drv_start(pdev, rj6cba100_sensor_probe);

	RJ6CBA100_RETURN_N(rc);
}

static struct platform_driver msm_camera_driver = {
	.probe = __rj6cba100_probe,
	.driver = {
		.name = "msm_camera_rj6cba100",
		.owner = THIS_MODULE,
	},
};

static int __init rj6cba100_init(void)
{
	int rc;
	
	RJ6CBA100_ENTER;

	rc = platform_driver_register(&msm_camera_driver);
	
	RJ6CBA100_RETURN_N(rc);
}

module_init(rj6cba100_init);

MODULE_DESCRIPTION("RJ6CBA100 VGA YUV sensor driver");
MODULE_LICENSE("GPL v2");

