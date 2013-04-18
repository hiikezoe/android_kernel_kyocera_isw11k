/* LINUX/kernel/include/linux/kc_security.h
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _KCLSMSECURITY_H
#define _KCLSMSECURITY_H

#if 0	/* disable debug log */
#define KC_DBGPRINT printk
#else
#define KC_DBGPRINT(...)
#endif

#ifdef INCLUDE_SECURITY_KCLSM	/* for kclsm.c */

#ifdef CONFIG_SECURITY_KCLSM_DEVICEFILEPERMISSION

typedef struct{
	const char* path;
	dev_t devnumber;
	mode_t mode;
	uid_t uid;
	gid_t gid;
} kclsm_devpermission;

static kclsm_devpermission kclsm_devpermission_list[] = {
	{NULL, 0, 0, 0, 0}
};

#endif	/* CONFIG_SECURITY_KCLSM_DEVICEFILEPERMISSION */


#ifdef CONFIG_SECURITY_KCLSM_MOUNT_DEVICE
static const char mntdev_namelist[][PATH_MAX] = {
	"/dev/block/mmcblk0p12",
	""
};
#endif	// CONFIG_SECURITY_KCLSM_MOUNT_DEVICE

#endif /* INCLUDE_SECURITY_KCLSM	// for kclsm.c */



#ifdef INCLUDE_SECURITY_KCSYS	/* for module.c */

#ifdef CONFIG_SECURITY_KCSYS_INSMOD
static const char* module_namelist[] = {
	"qcrypto",
	/* "WCN1314_rf", */	/* ->wlan */
	"WCN1314_rf_ftm",
	"qcedev",
	"hscd_i2c",
	"ansi_cprng",
	"cpaccess",
	"reset_modem",
	"mtd_stresstest",
	"mtd_torturetest",
	"dma_test",
	"kc_sdcarddrv",
	"alps_input",		/* alps-input.ko */
	"oprofile",
	"mtd_speedtest",
	"gp2ap003a10f",
	"wlan",			/* libra.ko, WCN1314_rf.ko */
	"qce",
	"mtd_erasepart",
	"scsi_wait_scan",
	"kwimax_onoff",
	"libra_ftm",
	/* "libra", */		/* ->wlan */
	"mtd_subpagetest",
	"mtd_pagetest",
	"mtd_nandecctest",
	"bcmsdio",
	"gspca_main",
	"mtd_readtest",
	"cls_flow",
	"bcmwimax",
	"felica",
	"evbug",
	"leds_ledlight",	/* leds-ledlight.ko */
	"dal_remotetest",
	"kxud9",
	"mtd_oobtest",
	"librasdioif",
	"kwimax_gpio_wrapper",
	"sch_dsmark",
	"xt_TCPMSS",
	"iptable_mangle",
	"kc_sdgdrv",
	NULL
};
#endif	// CONFIG_SECURITY_KCSYS_INSMOD

#endif /* INCLUDE_SECURITY_KCSYS	// for module.c */


#endif /* _KCLSMSECURITY_H */
