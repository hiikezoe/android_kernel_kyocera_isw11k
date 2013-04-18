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

#include "msm_fb.h"
#include "mddihost.h"

extern uint8_t g_disp_board_check_flag;

static int __init tc358723xbg_panel_init(void)
{
    int ret;
    struct msm_panel_info pinfo;

    if( 0x00 != g_disp_board_check_flag )
    {
         /* DTS(WS0) is not detect */
         return 0;
    }

    /* Panel information setting */
    pinfo.xres                      = 480;
    pinfo.yres                      = 800;
    pinfo.bpp                       = 18;

    pinfo.wait_cycle                = 0;        /* Waiting time to becoming of MDDI active */

    pinfo.bl_max                    = 0;        /* not used */
    pinfo.bl_min                    = 0;        /* not used */
    pinfo.fb_num                    = 2;

    MSM_FB_SINGLE_MODE_PANEL(&pinfo);

    pinfo.mddi.vdopkt               = MDDI_DEFAULT_PRIM_PIX_ATTR;

    /* Panel information setting (the above screen) */
    pinfo.pdest                     = DISPLAY_1;
    pinfo.type                      = MDDI_PANEL;

    pinfo.clk_rate                  = 192000000; /* MDDI TX rate Hz */
    pinfo.clk_min                   = 192000000; /* It makes it more than this value the lowest. */
    pinfo.clk_max                   = 200000000;

    pinfo.lcd.vsync_enable          = FALSE;
    pinfo.lcd.refx100               = 6244;     /* 100 times refresh rate */
    pinfo.lcd.v_back_porch          = 8;
    pinfo.lcd.v_front_porch         = 12;
    pinfo.lcd.v_pulse_width         = 5;
    pinfo.lcd.hw_vsync_mode         = FALSE;
    pinfo.lcd.vsync_notifier_period = (1 * HZ);

    /* Panel information registration (the above screen) */
    ret = mddi_tc358723xbg_device_register(&pinfo);

    if (ret) {
        printk(KERN_ERR "%s: failed to register device!<UP>\n", __func__);
        return ret;
    }

    return ret;
}

module_init(tc358723xbg_panel_init);

