/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
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

#include <linux/gpio.h>
/* For electrical power control for WS0 */
#include <mach/vreg.h>
#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"

/* GPIO control definition value for WS0 */
#define MDDI_RST_N 133
#define VMDDIC1_ON 143
#define VMDDIC2_ON 128
#define VRFSW_ON   157

#define DTS_LCD_WS21_INTER_HW_VER   0x30313032  /* WS2-1 internal HW version */
#define DTS_LCD_WS23_INTER_HW_VER   0x30313034  /* WS2-3 internal HW version */
#define DTS_LCD_WS3_INTER_HW_VER    0x30313035  /* WS2-4==WS3 internal HW version   */
#define DTS_LCD_POLLING_TIME            3000    /* ms */
#define LCD_BACKLIGHT_DOWN          2

/*
*       Two LCD panel(480x800)
*
*   Connection:
*    ====== QSD8650 ===================== | ======= others =================
*     MDP --------- DMA-S --- MDDI(host) --- MDDI(client) --- LCD (UPPER)
*      |
*      ---iLCDC --- DMA-P --- GPIO -------------------------- LCD (LOWER)
*
*           LCD Panel Driver is Renesas R63302
*           MDDI LCDC Driver is TOSHIBA TC358722XBG
*/

/*
*   The system interface of the LCD panel on the DOWN(MDP iLCDC) side is
*   controlled by using GPIO of QSD8650. 
*       1:use QSD GPIO 0:use MDDI
*/

/*
*   TOSHIBA TC358722XBG CONTROL RGISTER BASE ADDRESS
*/
#define MDDI_CLIENT_CORE_BASE   0x108000 /* Base0:MDC    block:32KB */
#define LCD_CONTROL_BLOCK_BASE  0x110000 /* Base1:LCDC   block:64KB */
#define SPI_BLOCK_BASE          0x120000 /* Base2:SPI    block:64KB */
/* #define I2C_BLOCK_BASE       0x130000  * Base3:I2C    block:64KB */
#define PWM_BLOCK_BASE          0x140000 /* Base4:PWM    block:64KB */
#define GPIO_BLOCK_BASE         0x150000 /* Base5:GPIO   block:64KB */
#define SYSTEM_BLOCK1_BASE      0x160000 /* Base6:SYSTEM1  block:64KB */
#define SYSTEM_BLOCK2_BASE      0x170000 /* Base7:SYSTEM2  block:64KB */

/*
*   Base0   MDC     block
*/
#define TTBUSSEL    (MDDI_CLIENT_CORE_BASE|0x18)
#define DPSET0      (MDDI_CLIENT_CORE_BASE|0x1C)
#define DPSET1      (MDDI_CLIENT_CORE_BASE|0x20)
#define DPSUS       (MDDI_CLIENT_CORE_BASE|0x24)
#define DPRUN       (MDDI_CLIENT_CORE_BASE|0x28)
#define SYSCKENA    (MDDI_CLIENT_CORE_BASE|0x2C)

#define BITMAP0     (MDDI_CLIENT_CORE_BASE|0x44)
#define BITMAP1     (MDDI_CLIENT_CORE_BASE|0x48)
#define BITMAP2     (MDDI_CLIENT_CORE_BASE|0x4C)
#define BITMAP3     (MDDI_CLIENT_CORE_BASE|0x50)
#define BITMAP4     (MDDI_CLIENT_CORE_BASE|0x54)

/*
*   Base1   LCDC    block
*/
#define SRST        (LCD_CONTROL_BLOCK_BASE|0x00)
#define PORT_ENB    (LCD_CONTROL_BLOCK_BASE|0x04)
#define START       (LCD_CONTROL_BLOCK_BASE|0x08)
#define PORT        (LCD_CONTROL_BLOCK_BASE|0x0C)
#define CMN         (LCD_CONTROL_BLOCK_BASE|0x10)

#define INTFLG      (LCD_CONTROL_BLOCK_BASE|0x18)
#define INTMSK      (LCD_CONTROL_BLOCK_BASE|0x1C)
#define MPLFBUF     (LCD_CONTROL_BLOCK_BASE|0x20)

#define PXL         (LCD_CONTROL_BLOCK_BASE|0x30)
#define HCYCLE      (LCD_CONTROL_BLOCK_BASE|0x34)
#define HSW         (LCD_CONTROL_BLOCK_BASE|0x38)
#define HDE_START   (LCD_CONTROL_BLOCK_BASE|0x3C)
#define HDE_SIZE    (LCD_CONTROL_BLOCK_BASE|0x40)
#define VCYCLE      (LCD_CONTROL_BLOCK_BASE|0x44)
#define VSW         (LCD_CONTROL_BLOCK_BASE|0x48)
#define VDE_START   (LCD_CONTROL_BLOCK_BASE|0x4C)
#define VDE_SIZE    (LCD_CONTROL_BLOCK_BASE|0x50)

#define WAKEUP      (LCD_CONTROL_BLOCK_BASE|0x54)
#define REGENB      (LCD_CONTROL_BLOCK_BASE|0x5C)
#define VSYNIF      (LCD_CONTROL_BLOCK_BASE|0x60)
#define WRSTB       (LCD_CONTROL_BLOCK_BASE|0x64)
#define RDSTB       (LCD_CONTROL_BLOCK_BASE|0x68)

#define ASY_DATA    (LCD_CONTROL_BLOCK_BASE|0x6C)
#define ASY_DATB    (LCD_CONTROL_BLOCK_BASE|0x70)
#define ASY_DATC    (LCD_CONTROL_BLOCK_BASE|0x74)
#define ASY_DATD    (LCD_CONTROL_BLOCK_BASE|0x78)
#define ASY_DATE    (LCD_CONTROL_BLOCK_BASE|0x7C)
#define ASY_DATF    (LCD_CONTROL_BLOCK_BASE|0x80)
#define ASY_DATG    (LCD_CONTROL_BLOCK_BASE|0x84)
#define ASY_DATH    (LCD_CONTROL_BLOCK_BASE|0x88)
#define ASY_CMDSET  (LCD_CONTROL_BLOCK_BASE|0x8C)
#define MONI        (LCD_CONTROL_BLOCK_BASE|0xB0)
#define VPOS        (LCD_CONTROL_BLOCK_BASE|0xC0)

/*
*   Base2   SPI     block
*/
#define SSICTL      (SPI_BLOCK_BASE|0x00)
#define SSITIME     (SPI_BLOCK_BASE|0x04)
#define SSITX       (SPI_BLOCK_BASE|0x08)
#define SSIRX       (SPI_BLOCK_BASE|0x0C)
#define SSIINTC     (SPI_BLOCK_BASE|0x10)
#define SSIINTS     (SPI_BLOCK_BASE|0x14)

/* DTS_LCD_005 */
#define SSIINTS_SIFACT  (0x00001000)
#define SSIINTS_INTSTXO (0x00000004)

/*
*   Base4   PWM     block
*/
#define TIMER0LOAD  (PWM_BLOCK_BASE|0x00)
#define TIMER0CTRL  (PWM_BLOCK_BASE|0x08)
#define PWM0OFF     (PWM_BLOCK_BASE|0x1C)
#define TIMER1LOAD  (PWM_BLOCK_BASE|0x20)
#define TIMER1CTRL  (PWM_BLOCK_BASE|0x28)
#define PWM1OFF     (PWM_BLOCK_BASE|0x3C)
#define TIMER2LOAD  (PWM_BLOCK_BASE|0x40)
#define TIMER2CTRL  (PWM_BLOCK_BASE|0x48)
#define PWM2OFF     (PWM_BLOCK_BASE|0x5C)
#define PWMCR       (PWM_BLOCK_BASE|0x68)

/*
*   Base5   GPIO    block
*/
#define GPIODATA    (GPIO_BLOCK_BASE|0x00)
#define GPIODIR     (GPIO_BLOCK_BASE|0x04)
#define GPIOIS      (GPIO_BLOCK_BASE|0x08)
#define GPIOIEV     (GPIO_BLOCK_BASE|0x10)
#define GPIOIC      (GPIO_BLOCK_BASE|0x20)
#define GPIOPC      (GPIO_BLOCK_BASE|0x28)

/*
*   Base6   SYSTEM1     block
*/
#define WKREQ       (SYSTEM_BLOCK1_BASE|0x00)
#define CLKENB      (SYSTEM_BLOCK1_BASE|0x04)
#define DRAMPWR     (SYSTEM_BLOCK1_BASE|0x08)
#define INTMASK     (SYSTEM_BLOCK1_BASE|0x0C)
#define CNT_DIS     (SYSTEM_BLOCK1_BASE|0x10)

/*
*   Base7   SYSTEM2     block
*/
#define GPIOSEL     (SYSTEM_BLOCK2_BASE|0x00)


/* SPI_BASE+0x00 bit functions */
#define SSICTL_SPOD     0x00080000 /* ON:PULL/L & TRISTATE */
#define SSICTL_LOSDIR   0x00040000 /* ON:DIN is not DOUT */
#define SSICTL_LOCSPL   0x00020000 /* ON:SPICS0 High Active */
#define SSICTL_LODMY    0x00010000 /* ON:Read Dummy Cycle in */

#define SSICTL_SPLOSEL  0x00000008 /* ON:SSI */
#define SSICTL_SETACT   0x00000002 /* ON:SSI Data Transfer */

#define SSI_TX_SETTING  (SSICTL_LOSDIR|SSICTL_SPLOSEL)
#define SSI_TX_START    (SSICTL_LOSDIR|SSICTL_SPLOSEL|SSICTL_SETACT)

/* SPI_BASE+0x08 bit functions */
#define SSITX_CONT      0x00080000 /* ON:TX Continue */
#define SSITX_WRITE     0x00000000 /* OFF:WRITE */
#define SSITX_READ      0x00040000 /* ON:READ */
#define SSITX_1BYTE     0x00000000 /* ON:1byte */
#define SSITX_2BYTE     0x00010000 /* ON:2bytes */
#define SSITX_3BYTE     0x00020000 /* ON:3bytes(RO) */
#define SSITX_4BYTE     0x00030000 /* ON:4bytes(RO) */

#define PANEL_WR(n)     (SSITX_WRITE|SSITX_1BYTE|n)
#define PANEL_WR2(n)    (SSITX_WRITE|SSITX_2BYTE|n)
#define PANEL_RD(n)     (SSITX_READ |SSITX_1BYTE|n)
#define PANEL_RD2(n)    (SSITX_READ |SSITX_2BYTE|n)
#define PANEL_RD3(n)    (SSITX_READ |SSITX_3BYTE|n)
#define PANEL_RD4(n)    (SSITX_READ |SSITX_4BYTE|n)

/*
*   UP/DOWN control
*/
#define DTS_LCD_ALL     0
#define DTS_LCD_UP      1
#define DTS_LCD_DOWN    2

#define HS_SPI_RETRY_CNT 5

#define WRITE_CLIENT_REG(reg_addr, reg_val) \
    mddi_host_register_write(reg_addr, reg_val, MDDI_DATA_PACKET_4_BYTES, TRUE, NULL, MDDI_HOST_PRIM)

#define DTS_LCD_OPEN        1
#define DTS_LCD_CLOSE       0
#define DTS_LCD_OPEN_CLOSE  -1

/*============================================================================
*       
============================================================================*/

extern uint8    dts_lcd_lcd_gamma[2][3][18];
void mddi_lcd_kernel_init( void );

uint32_t dts_lcd_internal_hw_version = DTS_LCD_WS3_INTER_HW_VER;

extern void lcdc_close_setting(void);
extern void lcdc_open_setting(void);
extern boolean mdp_dma_lcdc_status;

extern uint8_t g_disp_board_check_flag;

/*============================================================================
*       
============================================================================*/
/*
    LCD PANEL STATE MACHINE ()is specification's status name

    OFF-----------------NORMAL(DISPLAYED)
*/

typedef enum {
    TOSHIBA_STATE_OFF,
    TOSHIBA_STATE_STANDBY,
    TOSHIBA_STATE_READY,
    TOSHIBA_STATE_NORMAL_MODE,
    TOSHIBA_STATE_SUSPEND,
    TOSHIBA_STATE_INIT,
} mddi_toshiba_state_t;

static mddi_toshiba_state_t toshiba_state = TOSHIBA_STATE_OFF;

static struct msm_panel_common_pdata *mddi_dts_pdata;

int dts_lcd_flag_black_pict=0;
struct delayed_work dts_lcd_pict_chg_work;
void dts_lcd_set_nomal_pict(struct work_struct *work);
int dts_lcd_backlight_changing=0;
int dts_lcd_register_read_flg = FALSE;/* WS2-1:FALSE,WS2-2:TRUE */
int dts_lcd_check_panel_ready=0;

boolean dts_lcd_flag_under_panel_close = TRUE;

static void send_to_r63302(int command_or_parameter, int value, int updown);
static void lcd_panel_on_registr_settings(int updown);
boolean dts_lcd_flag_panel_restarting = FALSE;

boolean dts_lcd_flag_resume_start_backlight = FALSE;
boolean dts_lcd_flag_kernel_init_start = FALSE;
/*============================================================================
*       
============================================================================*/

/*
    The startup status of MDDI is confirmed.  DTS_LCD_133
*/
int dts_lcd_check_mddi_state(void)
{
    return (int)toshiba_state;
}

static inline void toshiba_reset_mddi(void)
{
    MDDI_MSG_DEBUG("%s %d %s \n",__FILE__ , __LINE__ , __func__);
    gpio_set_value(MDDI_RST_N, 0);
    msleep(4);
    gpio_set_value(MDDI_RST_N, 1);
    msleep(8);
}

static inline void toshiba_reset_lcd(void)
{
    MDDI_MSG_DEBUG("%s %d %s \n",__FILE__ , __LINE__ , __func__);
    WRITE_CLIENT_REG(GPIODATA,  0x00020000);
    mdelay(4);
    WRITE_CLIENT_REG(GPIODATA,  0x00020002);
    mdelay(8);
}

static inline void dts_reset_off(int updown)
{
    MDDI_MSG_DEBUG("%s %d %s \n",__FILE__ , __LINE__ , __func__);

    if(updown != DTS_LCD_DOWN)
    {
        WRITE_CLIENT_REG(GPIODATA,  0x00040000);  /* 2=L reset */
    }
    if(updown != DTS_LCD_UP)
    {
        /* gpio_set_value(GPIO_LCD2_RST, 0); */
        WRITE_CLIENT_REG(GPIODATA,  0x00020000);
    }
}

/*
    The state of the reset signal of lower side LCD panel is confirmed.
*/
int dts_lcd_check_down_lcd_reset(void)
{
    return 0;
}

static inline uint32 read_client_reg(uint32 reg_addr)
{
    /* uint32 reg_val; */
    uint32 reg_val=0xFFFFFFFF;

    MDDI_MSG_DEBUG("Before reg_addr = 0x%x  reg_val = 0x%x \n",(uint16)reg_addr,(uint16)reg_val);
    mddi_host_register_read(reg_addr, &reg_val, TRUE, MDDI_HOST_PRIM);/* DTS_LCD_129 */
    MDDI_MSG_DEBUG("After reg_addr = 0x%x  reg_val = 0x%x \n",(uint16)reg_addr,(uint16)reg_val);

    return reg_val;
}

#define LCD_PANEL_COMMAND   0
#define LCD_PANEL_PARAMETER 1

static uint32 send_to_r63302_read_panel(int value)
{
    uint32 val=0;
    int i;
    if (SSIINTS_SIFACT & read_client_reg(SSIINTS)) {/* DTS_LCD_005 */
        printk(KERN_ERR "ERROR!! MDDI set SSITX NG!  CS0 val=[0x%x]\n",value);
    }
    else
    {
    }    
    WRITE_CLIENT_REG(SSITX,     PANEL_RD(value));
    WRITE_CLIENT_REG(GPIODATA,  0x00400000);  /* 6=L CS0 */
    //if mddi_gpio_0 trigger for oscilloscope WRITE_CLIENT_REG(GPIODATA,  0x00410001);  /* 6=L CS0 ,0=H trigger */
    WRITE_CLIENT_REG(SSICTL,    SSI_TX_START);
    for(i=0; i<3; i++)
    {
        mddi_wait(1);
        val = read_client_reg(SSIINTS);
        if (val & 0x00000200) {
            val = read_client_reg(SSIRX);
            break;
        }else{
            printk(KERN_ERR "MEMO:                                 command 0x%X read NG i = [%d]\n", value, i);
        }
    }
    WRITE_CLIENT_REG(GPIODATA,  0x00400040);  /* 6=H CS0 */
    //if mddi_gpio_0 trigger for oscilloscope WRITE_CLIENT_REG(GPIODATA,  0x00410040);  /* 6=H CS0 ,0=L */

    WRITE_CLIENT_REG(SSICTL,    SSI_TX_SETTING);
    return val;
}

void dts_lcd_restart_panel(int updown)
{
    /* Reset release */
    WRITE_CLIENT_REG(GPIODATA,  0x00040004);  /* 2=H reset */
    if(updown != DTS_LCD_UP)
    {
        /* gpio_set_value(GPIO_LCD2_RST, 1); */
        WRITE_CLIENT_REG(GPIODATA,  0x00020002);
    }
    mddi_wait(10);
    /* LCD panel interface (SSI mode) setting. Only the upper part */
    WRITE_CLIENT_REG(SSICTL,    SSI_TX_SETTING);
    WRITE_CLIENT_REG(SSITIME,   0x00001000);
    WRITE_CLIENT_REG(CNT_DIS,   0x00000000);
    /* LCD panel start sequence */
    lcd_panel_on_registr_settings(updown);
    mddi_wait(100);
    dts_lcd_flag_panel_restarting = FALSE;/* Flag when being restarting */
}

/*
*   LCD panel register lead and restart conduct oneself
*/
void dts_lcd_panel_registar_read(void)
{
    boolean l_flag_go_restart = FALSE;
    int updown = DTS_LCD_ALL;

    dts_lcd_flag_panel_restarting = FALSE;/* Flag when being restarting */
    /*
    *   As for WS2-1, it is a return because it does the lead failure 
    *   without fail by the first leading. 
    *     dts_lcd_register_read_flg:WS2-1:FALSE,WS2-2:TRUE
    */
    if (!dts_lcd_register_read_flg)
    {
        printk(KERN_ERR "%s()dts_lcd_register_read_flg is FALSE not read and restart!!!\n", __func__);
        return;
    }
    if(dts_lcd_backlight_changing)
    {
        printk(KERN_ERR "%s()back light is setting now!!\n", __func__);
        return;
    }

    /*
    *   When the terminal closes, the lower side is led, and it doesn't restart. 
    */
    if(0x14 != send_to_r63302_read_panel(0x0A))
    {
        printk(KERN_ERR "%s()upper read error!! goto restart!!!\n", __func__);
        l_flag_go_restart = TRUE;
    }


    /*
    *   Restart processing
    */
    if(l_flag_go_restart)
    {
        printk(KERN_ERR "%s()do restart!!!\n", __func__);
        if(dts_lcd_flag_under_panel_close)
        {
            updown = DTS_LCD_UP;
        }
        /*
        *   The brightness of a present backlight is acquired. 
        */
        /*
        *   Off processing
        */
        mddi_wait(10);
        dts_lcd_flag_panel_restarting = TRUE;/* Flag when being restarting */
        WRITE_CLIENT_REG(SSICTL,    SSI_TX_SETTING);
        /* LCD panel OFF command(Disp Off) */
        send_to_r63302(LCD_PANEL_COMMAND, 0x28, updown);
        /* LCD panel OFF command(Sleep In) */
        send_to_r63302(LCD_PANEL_COMMAND, 0x10, updown);
        mddi_wait(200);
        dts_reset_off(updown);
        mddi_wait(4);
        /*
        *   On processing
        */
        dts_lcd_restart_panel(updown);
        /*
        *   The backlight brightness is restored. 
        */
    }
}

static void send_to_r63302(int command_or_parameter, int value, int updown)
{

    if(updown != DTS_LCD_DOWN) {
		if (SSIINTS_SIFACT & read_client_reg(SSIINTS)) {/* DTS_LCD_005 */
		    printk(KERN_ERR "ERROR!! MDDI set SSITX NG!  CS0 val=[0x%x]\n",value);
		} else {

		}    
    	if(command_or_parameter == LCD_PANEL_PARAMETER) {
    	    WRITE_CLIENT_REG(SSITX,     PANEL_WR(0x100 | value));
    	} else {
    	    WRITE_CLIENT_REG(SSITX,     PANEL_WR(value));
    	}
    	WRITE_CLIENT_REG(GPIODATA,  0x00400000);  /* 6=L CS0 */
    	WRITE_CLIENT_REG(SSICTL,    SSI_TX_START);
		udelay(100);
    	WRITE_CLIENT_REG(GPIODATA,  0x00400040);  /* 6=H CS0 */

    	WRITE_CLIENT_REG(SSICTL,    SSI_TX_SETTING);
	}

}

static void lcd_panel_set_factory_mode(int updown)
{
    /* Manufacture Command Access Protect Release */
    send_to_r63302(LCD_PANEL_COMMAND, 0xB0, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);
}

static void lcd_panel_exit_factory_mode(int updown)
{
    /* Manufacture Command Access Protect Effective */
    send_to_r63302(LCD_PANEL_COMMAND, 0xB0, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x03, updown);
}

/*
* function  :   set_gamma_data_to_panel
* input     :   up_or_down --- DTS_LCD_UP or DTS_LCD_DOWN only
*/
static void set_gamma_data_to_panel(int up_or_down)
{
    int i,j;

    /* Effective making independent gamma setting */
    send_to_r63302(LCD_PANEL_COMMAND, 0xCB, up_or_down);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x01, up_or_down);
    /* Independent gamma setting */
    for(i=0; i<3; i++)
    {
        send_to_r63302(LCD_PANEL_COMMAND, 0xC8+i, up_or_down);
        for(j=0; j<18; j++)
        {
            if(up_or_down == DTS_LCD_UP)
            {
                send_to_r63302(LCD_PANEL_PARAMETER, dts_lcd_lcd_gamma[0][i][j], up_or_down);
            }
            else
            {
                send_to_r63302(LCD_PANEL_PARAMETER, dts_lcd_lcd_gamma[1][i][j], up_or_down);
            }
        }
    }
}

void set_gamma_data_from_NV(int updown)
{
    if(updown != DTS_LCD_DOWN)
    {
        /* set register data to LCD panel */
        lcd_panel_set_factory_mode(DTS_LCD_UP);
        set_gamma_data_to_panel(DTS_LCD_UP);
        lcd_panel_exit_factory_mode(DTS_LCD_UP);
    }
    if(updown != DTS_LCD_UP)
    {
        /* set register data to LCD panel */
        lcd_panel_set_factory_mode(DTS_LCD_DOWN);
        set_gamma_data_to_panel(DTS_LCD_DOWN);
        lcd_panel_exit_factory_mode(DTS_LCD_DOWN);
    }
}

void dts_lcd_set_frame_mode(int updown)
{
    /* Frame reversing only of WS2-1 75Hz */
    if(dts_lcd_internal_hw_version == DTS_LCD_WS21_INTER_HW_VER)
    {
        if(updown != DTS_LCD_DOWN)
        {
            /* 75Hz */
            WRITE_CLIENT_REG(CLKENB,    0x000000CB);
            WRITE_CLIENT_REG(CLKENB,    0x0000A1CB);
        }
        /* Frame reversing */
        lcd_panel_set_factory_mode(updown);
        /* 18bitRGB setting */
        send_to_r63302(LCD_PANEL_COMMAND, 0x3A, updown);
        send_to_r63302(LCD_PANEL_PARAMETER, 0x60, updown);
        send_to_r63302(LCD_PANEL_COMMAND, 0xC1, updown);
        send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);
        send_to_r63302(LCD_PANEL_PARAMETER, 0x40, updown);
        send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);
        /* Because the flicker goes out the frame reversing, only WS2-1 
           sets the correspondence Gamma. 
         * It is a color tuning software. */
        set_gamma_data_from_NV(updown);

        lcd_panel_exit_factory_mode(updown);
    }
    else
    {
        /* Line reversing */
        lcd_panel_set_factory_mode(updown);
        if(dts_lcd_internal_hw_version < DTS_LCD_WS3_INTER_HW_VER)
        {
            /* 18bitRGB setting */
            send_to_r63302(LCD_PANEL_COMMAND, 0x3A, updown);
            send_to_r63302(LCD_PANEL_PARAMETER, 0x60, updown);
        }
        send_to_r63302(LCD_PANEL_COMMAND, 0xC1, updown);
        send_to_r63302(LCD_PANEL_PARAMETER, 0x80, updown);
        send_to_r63302(LCD_PANEL_PARAMETER, 0x40, updown);
        send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);
        lcd_panel_exit_factory_mode(updown);
    }
}

void dts_lcd_upper_panel_init(void)
{
    int updown = DTS_LCD_UP;
    MDDI_MSG_DEBUG("%s %d %s \n",__FILE__ , __LINE__ , __func__);

    if(dts_lcd_internal_hw_version == DTS_LCD_WS21_INTER_HW_VER)
    {
        MDDI_MSG_DEBUG("%s %d %s \n",__FILE__ , __LINE__ , __func__);

        /* 75Hz */
        WRITE_CLIENT_REG(CLKENB,    0x000000CB);
        WRITE_CLIENT_REG(CLKENB,    0x0000A1CB);
        /* LCD panel setting */
        lcd_panel_set_factory_mode(updown);
        /* 18bitRGB setting */
        send_to_r63302(LCD_PANEL_COMMAND, 0x3A, updown);
        send_to_r63302(LCD_PANEL_PARAMETER, 0x60, updown);
        /* Frame reversing */
        send_to_r63302(LCD_PANEL_COMMAND, 0xC1, updown);
        send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);
        send_to_r63302(LCD_PANEL_PARAMETER, 0x40, updown);
        send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);
        /* Because the flicker goes out the frame reversing, only 
         * WS2-1 sets the correspondence Gamma. 
         * It is a color tuning software.  */
        set_gamma_data_from_NV(updown);
        lcd_panel_exit_factory_mode(updown);
    }
    else
    {

        if(dts_lcd_internal_hw_version == DTS_LCD_WS23_INTER_HW_VER)
        {
            MDDI_MSG_DEBUG("%s %d %s \n",__FILE__ , __LINE__ , __func__);

            /* Line reversing */
            lcd_panel_set_factory_mode(updown);
            send_to_r63302(LCD_PANEL_COMMAND, 0xC1, updown);
            send_to_r63302(LCD_PANEL_PARAMETER, 0x80, updown);
            send_to_r63302(LCD_PANEL_PARAMETER, 0x40, updown);
            send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);
            lcd_panel_exit_factory_mode(updown);
        }
       else if(dts_lcd_internal_hw_version == DTS_LCD_WS3_INTER_HW_VER)
        {
            MDDI_MSG_DEBUG("%s %d %s \n",__FILE__ , __LINE__ , __func__);

            lcd_panel_set_factory_mode(updown);
            send_to_r63302(LCD_PANEL_COMMAND, 0xC1, updown);
            send_to_r63302(LCD_PANEL_PARAMETER, 0x80, updown);
            send_to_r63302(LCD_PANEL_PARAMETER, 0x40, updown);
            send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);
            lcd_panel_exit_factory_mode(updown);
        }
    }
}

static void lcd_panel_on_registr_settings(int updown)
{
    uint32 i,j;

    MDDI_MSG_DEBUG("%s %d %s \n",__FILE__ , __LINE__ , __func__);

    WRITE_CLIENT_REG(GPIODATA,  0x00400040);  /* 6=H CS0 */

    send_to_r63302(LCD_PANEL_COMMAND, 0xB0, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);

    /* Normal display for Driver Upper side */
    send_to_r63302(LCD_PANEL_COMMAND, 0x36, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x41, updown);

    /* Pixel format setting( 18bit color mode) */
    send_to_r63302(LCD_PANEL_COMMAND, 0x3A, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x60, updown);

    /* Source setting */
    send_to_r63302(LCD_PANEL_COMMAND, 0xB3, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x02, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);

    /* Pixel format setting( 18bit color mode) */
    send_to_r63302(LCD_PANEL_COMMAND, 0xB8, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown); /* PWM output OFF setting */
    send_to_r63302(LCD_PANEL_PARAMETER, 0x0D, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x08, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0xFF, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0xFF, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0xBA, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0xD9, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x0F, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x1F, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x10, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x10, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x37, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x5A, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x87, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0xBE, updown);

    /* RCCS(BLC) setting 2 */
    send_to_r63302(LCD_PANEL_COMMAND, 0xB9, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x01, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0xFF, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x02, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x08, updown);

    /* Driving setting */
    send_to_r63302(LCD_PANEL_COMMAND, 0xC0, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x01, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x31, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x0A, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);

    /* Display timing setting */
    send_to_r63302(LCD_PANEL_COMMAND, 0xC1, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x80, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x40, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);

    /* Display control setting */
    send_to_r63302(LCD_PANEL_COMMAND, 0xC2, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x08, updown);

    /* Driver setting */
    send_to_r63302(LCD_PANEL_COMMAND, 0xC3, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x08, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);

    /* Panel setting */
    send_to_r63302(LCD_PANEL_COMMAND, 0xC4, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);

    /* Panel timing setting */
    send_to_r63302(LCD_PANEL_COMMAND, 0xC5, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x0A, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x3C, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x04, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x38, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x40, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x0B, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x0E, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x0B, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x03, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x0D, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x02, updown);

    /* Gamma setting */
    for(i=0; i<3; i++)
    {
        send_to_r63302(LCD_PANEL_COMMAND, 0xC8+i, updown);
        for(j=0; j<18; j++)
        {
            send_to_r63302(LCD_PANEL_PARAMETER, dts_lcd_lcd_gamma[0][i][j], updown);
        }
    }

    /* Driver setting */
    send_to_r63302(LCD_PANEL_COMMAND, 0xCB, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x01, updown);

    /* Pressure circuit setting */
    send_to_r63302(LCD_PANEL_COMMAND, 0xD0, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x11, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0xC2, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x0F, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x41, updown);

    /* Pressure circuit setting */
    send_to_r63302(LCD_PANEL_COMMAND, 0xD1, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x1A, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x14, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x02, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x2C, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);

    /* AMP setting */
    send_to_r63302(LCD_PANEL_COMMAND, 0xD2, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0xBC, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);

    /* AMP */
    send_to_r63302(LCD_PANEL_COMMAND, 0xD3, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x33, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x03, updown);

    /* VCOM/VREG setting */
    send_to_r63302(LCD_PANEL_COMMAND, 0xD5, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x14, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x72, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x1E, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);

    /* Turning on timing setting */
    send_to_r63302(LCD_PANEL_COMMAND, 0xD7, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x89, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x04, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0xA9, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0xCD, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x78, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0xBC, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x6D, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x23, updown);

    /* Turning off timing setting */
    send_to_r63302(LCD_PANEL_COMMAND, 0xD8, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x24, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x37, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x82, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x25, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x49, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x0A, updown);

    /* ON/OFF setting */
    send_to_r63302(LCD_PANEL_COMMAND, 0xD9, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0xFB, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0xF5, updown);

    /* Internal clock setting */
    send_to_r63302(LCD_PANEL_COMMAND, 0xEA, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x0F, updown);

    /* Interface setting */
    send_to_r63302(LCD_PANEL_COMMAND, 0xF3, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x14, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x01, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x25, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x01, updown);

    /* Driver setting */
    send_to_r63302(LCD_PANEL_COMMAND, 0xFA, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x01, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);

    /* Driver setting */
    send_to_r63302(LCD_PANEL_COMMAND, 0xFE, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0xA0, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x02, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x03, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x20, updown);
    send_to_r63302(LCD_PANEL_PARAMETER, 0x00, updown);

    lcd_panel_exit_factory_mode(updown);

    send_to_r63302(LCD_PANEL_COMMAND, 0x11, updown);
	msleep(300);
    /* LCD panel start command(Disp On) */
    send_to_r63302(LCD_PANEL_COMMAND, 0x29, updown);
    
    MDDI_MSG_DEBUG("%s %d %s \n",__FILE__ , __LINE__ , __func__);
}


/*
    MDDI and LCD driver setting
*/
static void toshiba_common_initial_setup(struct msm_fb_data_type *mfd)
{
    int32 cnt;
    int32 i;
    uint32 val;
    int updown=DTS_LCD_ALL;

    MDDI_MSG_DEBUG("%s %d %s \n",__FILE__ , __LINE__ , __func__);

    for (cnt = 0; cnt < HS_SPI_RETRY_CNT; cnt++) {  /* It retries several times for the SPI overrunning evasion.  */

        /* Toshiba LCDC(TC358722XBG) DPLL initialization */
        WRITE_CLIENT_REG(DPSET0,    0x4F4200AC);  /* 002 Setup DPLL parameters */
        WRITE_CLIENT_REG(DPSET1,    0x0000010F);  /* 003 Setup DPLL parameters */
        WRITE_CLIENT_REG(DPSUS,     0x00000000);  /* 004 Set DPLL oscillation enable */
        WRITE_CLIENT_REG(DPRUN,     0x00000001);  /* 005 Release reset signal for DPLL */
        /* Toshiba LCDC(TC358722XBG) DPLL stabilization waiting */
        for (i = 0; i < 5; i++) {
            /* Wait is not shortened to 100mS or less. 
             * (The value of DPSUS doesn't change if Read is done at 25mS intervals. ) */
            msleep(100);
            val = read_client_reg(DPSUS);         /* 006 Wait for DPLL lock-up check */
            if (val & 0x00000010) {
                break;
            }
        }
        /* Toshiba LCDC(TC358722XBG) setting */
        WRITE_CLIENT_REG(SYSCKENA,  0x00000001);  /* 007 Enable system clock output */
        WRITE_CLIENT_REG(CLKENB,    0x000000CA);  /* CHG GPIOSEL */
        WRITE_CLIENT_REG(DRAMPWR,   0x00000001);  /* 020 eDRAM power up */
        mddi_wait(1);                                   /* 030 */
        /* GPIO CONTROL */
        WRITE_CLIENT_REG(GPIOSEL,   0x00000200);  /* GPIO 8 is used as a lease line. */
        WRITE_CLIENT_REG(GPIOPC,    0x00C100C0);
        WRITE_CLIENT_REG(GPIODATA,  0x00420040);
        WRITE_CLIENT_REG(GPIODIR,   0x0000034E);

        WRITE_CLIENT_REG(BITMAP0,   0x032001E0);  /* 240 Setup of PITCH size of Frame buffer0 (WVGA) */
        WRITE_CLIENT_REG(CLKENB,    0x000001CB);  /* 241 DCLK supply */
        WRITE_CLIENT_REG(PORT_ENB,  0x00000001);  /* 242 Synchronous port enable */
        WRITE_CLIENT_REG(PORT,      0x00000004);  /* 243 Polarity of DE is set to high active */

        WRITE_CLIENT_REG(PXL,       0x00000002);
        dts_lcd_flag_black_pict=0;

        dts_lcd_flag_resume_start_backlight = TRUE;

        WRITE_CLIENT_REG(MPLFBUF,   0x00000000);  /* 245 Select the reading buffer */

        /* Toshiba LCDC(TC358722XBG) setting LCD panel relation */
        WRITE_CLIENT_REG(HCYCLE,    0x000000FF);  /* 246 Setup to WVGA size */
        WRITE_CLIENT_REG(HSW,       0x00000004);  /* 247 Setup to WVGA size */
        WRITE_CLIENT_REG(HDE_START, 0x0000000A);  /* 248 Setup to WVGA size */
        WRITE_CLIENT_REG(HDE_SIZE,  0x000000EF);  /* 249 Setup to WVGA size */
        WRITE_CLIENT_REG(VCYCLE,    0x00000333);  /* 250 Setup to WVGA size */
        WRITE_CLIENT_REG(VSW,       0x00000001);  /* 251 Setup to WVGA size */
        WRITE_CLIENT_REG(VDE_START, 0x00000009);  /* 252 Setup to WVGA size */
        WRITE_CLIENT_REG(VDE_SIZE,  0x0000031F);  /* 253 Setup to WVGA size */
        /* It is previously started by the specification of LCD.  */
        WRITE_CLIENT_REG(START,     0x00000001);  /* 255 LCDC - Pixel data transfer start */
        /* Reset LCD */
        WRITE_CLIENT_REG(GPIODATA,  0x00020000);  /* 2=L reset */
	    mddi_wait(4);
        WRITE_CLIENT_REG(GPIODATA,  0x00020002);  /* 2=H reset */
	    mddi_wait(8);
        /* LCD panel interface (SSI mode) setting */
        WRITE_CLIENT_REG(SSICTL,    SSI_TX_SETTING);
        WRITE_CLIENT_REG(SSITIME,   0x00001000);
        WRITE_CLIENT_REG(CNT_DIS,   0x00000000);
        /* LCD panel start sequence */
        lcd_panel_on_registr_settings(updown); 
        /* Overrunning error confirmation */
        val = read_client_reg(SSIINTS);               /* 334 */
        if (val & SSIINTS_INTSTXO) {                  /* DTS_LCD_005 *//* There is an overrunning error. */
            WRITE_CLIENT_REG(SSIINTC,   0x00003000);  /* 335 Clear interrupt */
            WRITE_CLIENT_REG(SSIINTS,   0x0000000F);  /* 336 Clear interrupt */
        }
        else {                                        /* There is no overrunning error.  */
            break;
        }
    }
    MDDI_MSG_DEBUG("%s %d %s \n",__FILE__ , __LINE__ , __func__);
}

static void toshiba_lcd_off(struct msm_fb_data_type *mfd)
{
    int updown=DTS_LCD_ALL;

    MDDI_MSG_DEBUG("%s %d %s \n",__FILE__ , __LINE__ , __func__);
    /* LCD driver setting */
    WRITE_CLIENT_REG(SSICTL,    SSI_TX_SETTING);
    /* LCD panel OFF command(Disp Off) */
    /* LCD panel OFF command(Sleep In) */
    send_to_r63302(LCD_PANEL_COMMAND, 0x10, updown);
    /* 200ms wait */
    mddi_wait(200);
    /* LCD panel OFF command(Disp Off) */
    send_to_r63302(LCD_PANEL_COMMAND, 0x28, updown);
    /* 20ms wait */
    mddi_wait(20);
    /* Pixel data transfer start OFF */
    WRITE_CLIENT_REG(START,   0x00000000);
}

static void toshiba_lcd_stop(struct msm_fb_data_type *mfd)
{
    /* Toshiba LCDC(TC358722XBG) setting */
    WRITE_CLIENT_REG(PORT,      0x00000008);  /* #LCD.PORT   # DE output OFF */
    WRITE_CLIENT_REG(REGENB,    0x00000001);  /* #LCD.REGENB # LCDC reflects a setup by the next VSYNC */
    mddi_wait(16);
    WRITE_CLIENT_REG(PXL,       0x00000000);  /* #LCD.PXL.ACTMODE #LCDC sleep mode */
    dts_lcd_flag_black_pict=0;
    dts_lcd_flag_resume_start_backlight = FALSE;
    WRITE_CLIENT_REG(START,     0x00000000);  /* #LCD.START  #Synchronous I/F OFF */
    WRITE_CLIENT_REG(REGENB,    0x00000001);  /* #LCD.REGENB #LCDC reflects a setup by the next VSYNC */
    mddi_wait(32);
    WRITE_CLIENT_REG(CLKENB,    0x00000000);
    WRITE_CLIENT_REG(DRAMPWR,   0x00000000);
    WRITE_CLIENT_REG(SYSCKENA,  0x00000000);
    WRITE_CLIENT_REG(DPSUS,     0x00000001);
    WRITE_CLIENT_REG(DPRUN,     0x00000000);
}

static void mddi_toshiba_init(struct msm_fb_data_type *mfd)
{
    MDDI_MSG_DEBUG("%s %d %s \n",__FILE__ , __LINE__ , __func__);
    switch (toshiba_state) {
    case TOSHIBA_STATE_OFF:
        toshiba_reset_mddi();
        toshiba_state = TOSHIBA_STATE_STANDBY;
        /* fall through */
    case TOSHIBA_STATE_STANDBY:
        toshiba_common_initial_setup(mfd);
        toshiba_state = TOSHIBA_STATE_NORMAL_MODE;
        break;
    case TOSHIBA_STATE_INIT:
        mddi_lcd_kernel_init();
        toshiba_state = TOSHIBA_STATE_NORMAL_MODE;
        break;
    default:
        MDDI_MSG_ERR("ERROR!!%s() from state %d\n", __func__, toshiba_state);
    }
    MDDI_MSG_DEBUG("%s %d %s \n",__FILE__ , __LINE__ , __func__);
}

static void mddi_toshiba_lcd_powerdown(struct msm_fb_data_type *mfd)
{
    int updown=DTS_LCD_ALL;

    MDDI_MSG_DEBUG("%s %d %s \n",__FILE__ , __LINE__ , __func__);

    switch (toshiba_state) {
    case TOSHIBA_STATE_INIT:
    case TOSHIBA_STATE_NORMAL_MODE:
        toshiba_state = TOSHIBA_STATE_OFF;
        toshiba_lcd_off(mfd);
        dts_reset_off(updown);
        msleep(4);
        WRITE_CLIENT_REG(GPIODIR,  0x0000034E);
        toshiba_lcd_stop(mfd);
        break;
    default:
        MDDI_MSG_ERR("ERROR!!%s() from state %d\n", __func__, toshiba_state);
    }
}

static int mddi_tc358723xbg_on(struct platform_device *pdev)
{
    struct msm_fb_data_type *mfd;

    MDDI_MSG_DEBUG("mddi_dts_on():start-----\n");

    mfd = platform_get_drvdata(pdev);
    if (!mfd)
        return -ENODEV;
    if (mfd->key != MFD_KEY)
        return -EINVAL;

    mfd->dma->waiting = false;
    mfd->dma->busy = false;

    MDDI_MSG_DEBUG("%s %d %s \n",__FILE__ , __LINE__ , __func__);

    mddi_toshiba_init(mfd);

    MDDI_MSG_DEBUG(KERN_ERR "mddi_dts_on():end-----\n");

    return 0;
}

static int mddi_tc358723xbg_off(struct platform_device *pdev)
{
    MDDI_MSG_DEBUG("%s %d %s \n",__FILE__ , __LINE__ , __func__);
    mddi_toshiba_lcd_powerdown(platform_get_drvdata(pdev));

    return 0;
}

static int mddi_tc358723xbg_probe(struct platform_device *pdev)
{
    MDDI_MSG_DEBUG("%s %d %s \n",__FILE__ , __LINE__ , __func__);

    if (pdev->id == 0) {
        mddi_dts_pdata = pdev->dev.platform_data;
        return 0;
    }

    msm_fb_add_device(pdev);

    MDDI_MSG_DEBUG("%s %d %s \n",__FILE__ , __LINE__ , __func__);

    return 0;
}

static struct platform_driver this_driver = {
    .probe  = mddi_tc358723xbg_probe,
    .driver = {
        .name   = "mddi_dts",
    },
};

static struct msm_fb_panel_data mddi_dts_panel_data = {
    .on         = mddi_tc358723xbg_on,
    .off        = mddi_tc358723xbg_off,
};

/* int mddi_dts_device_register(struct msm_panel_info *pinfo) */
int mddi_tc358723xbg_device_register(struct msm_panel_info *pinfo)
{
    struct platform_device *pdev = NULL;
    int ret;
    int pdest = (int)pinfo->pdest;
    int type =(int)pinfo->type;

    MDDI_MSG_DEBUG("%s %d %s \n",__FILE__ , __LINE__ , __func__);

    if(type != MDDI_PANEL && type != LCDC_PANEL) {
        printk(KERN_ERR "ERROR!! invalid parameter for platform_device_alloc is type\n");
        
        return -EINVAL;
    }
    if(pdest != DISPLAY_1 && pdest != DISPLAY_2) {
        printk(KERN_ERR "ERROR!! invalid parameter for platform_device_alloc is pdest\n");
        return -EINVAL;
    }

    /* ID original is set.  */
    pdev = platform_device_alloc("mddi_dts", (int)((pdest<<16)|type));
    if (!pdev)
        return -ENOMEM;

    mddi_dts_panel_data.panel_info = *pinfo;

    ret = platform_device_add_data(pdev, &mddi_dts_panel_data,
                                    sizeof(mddi_dts_panel_data));
    if (ret) {
        printk(KERN_ERR "%s: platform_device_add_data failed!\n", __func__);
        goto err_device_put;
    }

    ret = platform_device_add(pdev);
    if (ret) {
        printk(KERN_ERR "%s: platform_device_register failed!\n", __func__);
        goto err_device_put;
    }

    MDDI_MSG_DEBUG(KERN_ERR "%s %d %s \n",__FILE__ , __LINE__ , __func__);

    return 0;

err_device_put:
    MDDI_MSG_DEBUG(KERN_ERR "%s %d %s \n",__FILE__ , __LINE__ , __func__);
    platform_device_put(pdev);
    return ret;
}

static int __init mddi_tc358723xbg_init(void)
{
    if( 0x00 != g_disp_board_check_flag )
    {
         /* DTS(WS0) is not detect */
         return 0;
    }

    MDDI_MSG_DEBUG("%s %d %s \n",__FILE__ , __LINE__ , __func__);

    INIT_DELAYED_WORK(&dts_lcd_pict_chg_work, dts_lcd_set_nomal_pict);

    return platform_driver_register(&this_driver);
}

module_init(mddi_tc358723xbg_init);

void dts_lcd_set_nomal_pict(struct work_struct *work)
{
    if(dts_lcd_flag_black_pict)
    {
        WRITE_CLIENT_REG(PXL,       0x00000002);
        WRITE_CLIENT_REG(REGENB,    0x00000001);
        dts_lcd_flag_black_pict=0;
        msleep(40);/* VSYNC Wait */
    }
}

/*----------------------------*/
/* L C D  F R E X  C H E C K  */
/*----------------------------*/
#define MDDI_TEST_PTN1 0x1
#define MDDI_TEST_PTN2 0xFF
#define MDDI_TEST_PTN3_1 0xFFFF
#define MDDI_TEST_PTN3_2 0x5555
#define MDDI_TEST_PTN3_3 0xAAAA
#define MDDI_TEST_PTN3_4 0x0000

#define MDDI_GRAM_ADR 0x32C3C0 /* Not Used Area */

boolean hs_lcd_testmode_frex_check_drv( void )
{
    boolean ret = TRUE;
    uint16 set,i;
    uint32 val,cnt;

    printk(KERN_ERR "%s:start \n", __func__);

    for( cnt = 0; cnt < 3 ;cnt++)
    {
        ret = TRUE;
        /* Test Pattern1 */
        set = MDDI_TEST_PTN1;
        for( i=0 ; i<16 ; i++ )
        {
            mddi_queue_register_write(MDDI_GRAM_ADR, set, TRUE, 0);
            mddi_queue_register_read(MDDI_GRAM_ADR, &val, TRUE, 0);
        if( val != set )
        {
            ret = FALSE;
        }
        set = set<<1;
        }
  
        /* Test Pattern2 */
        set = MDDI_TEST_PTN2;
        for( i=0 ; i<8 ; i++ )
        {
            mddi_queue_register_write(MDDI_GRAM_ADR, set, TRUE, 0);
            mddi_queue_register_read(MDDI_GRAM_ADR, &val, TRUE, 0);
            if( val != set )
            {
                ret = FALSE;
            }
            set = set<<1;
        }
  
        /* Test Pattern3 */
        mddi_queue_register_write(MDDI_GRAM_ADR, MDDI_TEST_PTN3_1, TRUE, 0);
        mddi_queue_register_read(MDDI_GRAM_ADR, &val, TRUE, 0);
        if( val != MDDI_TEST_PTN3_1 )
        {
            ret = FALSE;
        }
        mddi_queue_register_write(MDDI_GRAM_ADR, MDDI_TEST_PTN3_2, TRUE, 0);
        mddi_queue_register_read(MDDI_GRAM_ADR, &val, TRUE, 0);
        if( val != MDDI_TEST_PTN3_2 )
        {
            ret = FALSE;
        }
        mddi_queue_register_write(MDDI_GRAM_ADR, MDDI_TEST_PTN3_3, TRUE, 0);
        mddi_queue_register_read(MDDI_GRAM_ADR, &val, TRUE, 0);
        if( val != MDDI_TEST_PTN3_3 )
        {
            ret = FALSE;
        }
        mddi_queue_register_write(MDDI_GRAM_ADR, MDDI_TEST_PTN3_4, TRUE, 0);
        mddi_queue_register_read(MDDI_GRAM_ADR, &val, TRUE, 0);
        if( val != MDDI_TEST_PTN3_4 )
        {
            ret = FALSE;
        }
        if( ret == TRUE ) break;
    }

    printk(KERN_ERR "frex_check_drv %d:end \n", ret);
    return ret ;
}
/*------------------------------------------------------------------*/
/* LCD state acquisition */
boolean mddi_lcd_status_get( void )
{
  if(toshiba_state == TOSHIBA_STATE_NORMAL_MODE)
  {
    return(TRUE);
  }
  return(FALSE);
}

/* LCD initialization (Kernel side) */
void mddi_lcd_kernel_init( void )
{
    MDDI_MSG_DEBUG("%s %d %s \n",__FILE__ , __LINE__ , __func__);
    dts_lcd_flag_kernel_init_start = TRUE;
    dts_lcd_upper_panel_init();

    if(0x14 == send_to_r63302_read_panel(0x0A))
    {
        dts_lcd_register_read_flg = TRUE;
    }
    dts_lcd_check_panel_ready = 1;
}

