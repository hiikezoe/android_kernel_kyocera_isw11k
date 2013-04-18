/* security/kclsm.c  (kclsm LSM module)
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 *
 * _xx_is_valid(), _xx_encode(), _xx_realpath_from_path()
 * is ported from security/tomoyo/realpath.c in linux-2.6.32 
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/security.h>
#include <linux/moduleparam.h>
#include <linux/mount.h>
#include <linux/mnt_namespace.h>
#include <linux/fs_struct.h>
#include <linux/magic.h>

#include <linux/kdev_t.h>
#include <linux/stat.h>

#define INCLUDE_SECURITY_KCLSM
#undef INCLUDE_SECURITY_KCSYS
#include <linux/kc_security.h>


enum bootmode_type {
	BOOTMODE_NORMAL,
	BOOTMODE_UPDATE,
};
static enum bootmode_type boot_mode = BOOTMODE_NORMAL;

static int __init bootmode_setup(char *buf) {
	if (strcmp(buf, "f-ksg") == 0) {
		boot_mode = BOOTMODE_UPDATE;
	}
	printk(KERN_INFO "boot_mode : <%d>\n", boot_mode);
	return 0;
}

early_param("kcdroidboot.mode", bootmode_setup);

#ifdef CONFIG_SECURITY_KCLSM_PTRACE
static int kclsm_ptrace_access_check(struct task_struct *child, unsigned int mode)
{

	uid_t uid = current_uid();
	if (uid == 0){
		printk(KERN_ERR "%s: No permission pid=%d name=%s child=%s mode=%d\n", __FUNCTION__, current->pid, current->comm, child->comm, mode);

#ifdef	KCLSM_DEBUG_ENABLE
		return 0;
#else
		return -EPERM;
#endif
	}

	KC_DBGPRINT(KERN_INFO "%s: Permitted uid=%d pid=%d name=%s child=%s mode=%d\n", __FUNCTION__, uid, current->pid, current->comm, child->comm, mode);
	return 0;
}


static int kclsm_ptrace_traceme(struct task_struct *parent)
{

	uid_t uid = current_uid();
	if (uid == 0){
		printk(KERN_ERR "%s: No permission pid=%d name=%s parent=%s\n", __FUNCTION__, current->pid, current->comm, parent->comm);

#ifdef	KCLSM_DEBUG_ENABLE
		return 0;
#else
		return -EPERM;
#endif
	}

	KC_DBGPRINT(KERN_INFO "%s: Permitted uid = %d pid=%d name=%s parent=%s\n", __FUNCTION__, uid, current->pid, current->comm, parent->comm);
	return 0;
}
#endif	/* CONFIG_SECURITY_KCLSM_PTRACE */


static inline bool _xx_is_valid(const unsigned char c)
{
	return c > ' ' && c < 127;
}


static int _xx_encode(char *buffer, int buflen, const char *str)
{
	while (1) {
		const unsigned char c = *(unsigned char *) str++;

		if (_xx_is_valid(c)) {
			if (--buflen <= 0)
				break;
			*buffer++ = (char) c;
		if (c != '\\')
				continue;
			if (--buflen <= 0)
				break;
			*buffer++ = (char) c;
			continue;
		}
		if (!c) {
			if (--buflen <= 0)
				break;
			*buffer = '\0';
			return 0;
		}
		buflen -= 4;
		if (buflen <= 0)
			break;
		*buffer++ = '\\';
		*buffer++ = (c >> 6) + '0';
		*buffer++ = ((c >> 3) & 7) + '0';
		*buffer++ = (c & 7) + '0';
	}
	return -ENOMEM;
}


static int _xx_realpath_from_path(struct path *path, char *newname,
				  int newname_len)
{
	int error = -ENOMEM;
	struct dentry *dentry = path->dentry;
	char *sp;

	if (!dentry || !path->mnt || !newname || newname_len <= 2048)
		return -EINVAL;
	if (dentry->d_op && dentry->d_op->d_dname) {
		/* For "socket:[\$]" and "pipe:[\$]". */
		static const int offset = 1536;
		sp = dentry->d_op->d_dname(dentry, newname + offset,
					   newname_len - offset);
	} else {
		struct path ns_root = {.mnt = NULL, .dentry = NULL};

		spin_lock(&dcache_lock);
		/* go to whatever namespace root we are under */
		sp = __d_path(path, &ns_root, newname, newname_len);
		spin_unlock(&dcache_lock);
		/* Prepend "/proc" prefix if using internal proc vfs mount. */
		if (!IS_ERR(sp) && (path->mnt->mnt_flags & MNT_INTERNAL) &&
		    (path->mnt->mnt_sb->s_magic == PROC_SUPER_MAGIC)) {
			sp -= 5;
			if (sp >= newname)
				memcpy(sp, "/proc", 5);
			else
				sp = ERR_PTR(-ENOMEM);
		}
	}
	if (IS_ERR(sp))
		error = PTR_ERR(sp);
	else
		error = _xx_encode(newname, sp - newname, sp);
	/* Append trailing '/' if dentry is a directory. */
	if (!error && dentry->d_inode && S_ISDIR(dentry->d_inode->i_mode)
	    && *newname) {
		sp = newname + strlen(newname);
		if (*(sp - 1) != '/') {
			if (sp < newname + newname_len - 4) {
				*sp++ = '/';
				*sp = '\0';
			} else {
				error = -ENOMEM;
			}
		}
	}
	return error;
}


#ifndef CONFIG_SECURITY_KCLSM_SYSTEM_DIR_PATH
#define CONFIG_SECURITY_KCLSM_SYSTEM_DIR_PATH "/system/"
#endif

static int kclsm_sb_mount(char *dev_name, struct path *path,
			    char *type, unsigned long flags, void *data)
{
	static char realpath[PATH_MAX];
#ifdef CONFIG_SECURITY_KCLSM_MOUNT_DEVICE
	int i;
#endif
	int r;
	bool disable_mount_device;
	bool disable_remount;

	r = _xx_realpath_from_path(path, realpath, PATH_MAX-1);
	if (r != 0) {
		printk(KERN_ERR "%s: Invalid path\n", __FUNCTION__);
#ifdef	KCLSM_DEBUG_ENABLE
		return 0;
#else
		return r;
#endif
	}

	if (strncmp(realpath, CONFIG_SECURITY_KCLSM_SYSTEM_DIR_PATH,
		strlen(CONFIG_SECURITY_KCLSM_SYSTEM_DIR_PATH)) == 0)
	{
		if (strcmp(realpath, CONFIG_SECURITY_KCLSM_SYSTEM_DIR_PATH) == 0)
		{

#ifdef CONFIG_SECURITY_KCLSM_MOUNT_DEVICE
			disable_mount_device = true;
			i=0;
			while(strcmp(mntdev_namelist[i],"") != 0){
				if (strcmp(dev_name, mntdev_namelist[i]) == 0){
					disable_mount_device = false;
				}
				i++;
			}
#else
			disable_mount_device = false;
#endif	/* CONFIG_SECURITY_KCLSM_MOUNT_DEVICE */


			disable_remount = false;
#ifdef CONFIG_SECURITY_KCLSM_MOUNT_REMOUNT
			if (flags & MS_REMOUNT) {
				disable_remount = true;
			}
#endif	/* CONFIG_SECURITY_KCLSM_MOUNT_REMOUNT */

			if ( (!disable_mount_device) && (!disable_remount) ){
				KC_DBGPRINT(KERN_INFO "%s: Permitted(/system): %s\n", __FUNCTION__, realpath);
				return 0;
			}else{
				if (disable_mount_device){
					printk(KERN_ERR "%s: Invalid device: %s\n",
					__FUNCTION__, dev_name);
				}
				if (disable_remount){
					printk(KERN_ERR "%s: Remount no permission\n", __FUNCTION__);
				}
#ifdef	KCLSM_DEBUG_ENABLE
				return 0;
#else
				return -EPERM;
#endif
			}

		}
		else
		{
			if (current->pid == 1) {
				KC_DBGPRINT(KERN_INFO "%s: Permitted(%s) on init\n", __FUNCTION__, realpath);
				return 0;
			}
			printk(KERN_ERR "%s: No permittion(%s)\n", __FUNCTION__, realpath);
#ifdef	KCLSM_DEBUG_ENABLE
			return 0;
#else
			return -EPERM;
#endif
		}

	}

	KC_DBGPRINT(KERN_INFO "%s: Permitted(other): %s\n", __FUNCTION__, realpath);
	return 0;
}


#ifndef CONFIG_SECURITY_KCLSM_SYSTEM_MOUNT_POINT
#define CONFIG_SECURITY_KCLSM_SYSTEM_MOUNT_POINT "system"
#endif

#ifdef CONFIG_SECURITY_KCLSM_UMOUNT
static int kclsm_sb_umount(struct vfsmount *mnt, int flags)
{
	(void)flags;

	if(mnt && mnt->mnt_mountpoint && 
		strncmp(mnt->mnt_mountpoint->d_name.name, CONFIG_SECURITY_KCLSM_SYSTEM_MOUNT_POINT,
		strlen(CONFIG_SECURITY_KCLSM_SYSTEM_MOUNT_POINT)) == 0)
	{
		if(boot_mode == BOOTMODE_UPDATE) {
			printk(KERN_INFO "%s: umount permitted for update mode\n", __FUNCTION__);
			return 0;
		}
		printk(KERN_ERR "%s: No permission mountpoint=%s\n", __FUNCTION__, mnt->mnt_mountpoint->d_name.name);

#ifdef	KCLSM_DEBUG_ENABLE
		return 0;
#else
		return -EPERM;
#endif
	}

	KC_DBGPRINT(KERN_INFO "%s: Permitted=%s\n", __FUNCTION__, mnt->mnt_mountpoint->d_name.name);

	return 0;
}
#endif	/* CONFIG_SECURITY_KCLSM_UMOUNT */


#ifdef CONFIG_SECURITY_KCLSM_PIVOTROOT
static int kclsm_sb_pivotroot(struct path *old_path, struct path *new_path)
{
	printk(KERN_ERR "%s: No permission\n", __FUNCTION__);
#ifdef	KCLSM_DEBUG_ENABLE
	return 0;
#else
	return -EPERM;
#endif
}
#endif	/* CONFIG_SECURITY_KCLSM_PIVOTROOT */


#ifdef CONFIG_SECURITY_KCLSM_CHROOT
static int kclsm_path_chroot(struct path *path)
{
	printk (KERN_ERR "%s: No permission\n", __FUNCTION__);
#ifdef	KCLSM_DEBUG_ENABLE
	return 0;
#else
	return -EPERM;
#endif
}
#endif	/* CONFIG_SECURITY_KCLSM_CHROOT */


#ifdef CONFIG_SECURITY_KCLSM_DEVICEFILEPERMISSION

static int kclsm_path_mknod(struct path *dir, struct dentry *dentry, int mode,
			   unsigned int dev)
{

	char* realdir;
	dev_t tmpdev;
	int r;
	int i;
	int len;

	realdir = kmalloc(PATH_MAX, GFP_KERNEL);
	if (!realdir){
		printk(KERN_ERR "%s: No memory\n", __FUNCTION__);
		return -ENOMEM;
	}

	r = _xx_realpath_from_path(dir, realdir, PATH_MAX-1);
	if (r != 0) {
		printk(KERN_ERR "%s: Invalid path\n", __FUNCTION__);
		kfree(realdir);
#ifdef	KCLSM_DEBUG_ENABLE
		return 0;
#else
		return r;
#endif
	}

	len = strlen(realdir);
	len = len + strlen(dentry->d_name.name);
	if (len >= PATH_MAX) {
		printk(KERN_ERR "%s: Path too long: %d chars\n", __FUNCTION__, len);
		kfree(realdir);
#ifdef	KCLSM_DEBUG_ENABLE
		return 0;
#else
		return -EPERM;
#endif
	}

	strcat(realdir,dentry->d_name.name);

	tmpdev = new_decode_dev(dev);
	for (i=0 ; kclsm_devpermission_list[i].path != NULL ; i++){
		if (strcmp(realdir, kclsm_devpermission_list[i].path) == 0)
		{
			if (tmpdev == kclsm_devpermission_list[i].devnumber)
			{
				KC_DBGPRINT(KERN_INFO "%s: Permitted-1 devpath=%s i=%d\n", __FUNCTION__, realdir, i);
				kfree(realdir);
				return 0;
			}else{
				printk (KERN_ERR "%s: No permission(device number):%d devpath=%s i=%d\n", __FUNCTION__, tmpdev, realdir, i);
				kfree(realdir);
#ifdef	KCLSM_DEBUG_ENABLE
				return 0;
#else
				return -EPERM;
#endif
			}
		}else{
			
			if (tmpdev == kclsm_devpermission_list[i].devnumber)
			{
				printk (KERN_ERR "%s: No permission(device name):%s i=%d\n", __FUNCTION__, realdir, i);
				kfree(realdir);
#ifdef	KCLSM_DEBUG_ENABLE
				return 0;
#else
				return -EPERM;
#endif
			}
		}
	}

	KC_DBGPRINT(KERN_INFO "%s: Permitted-2 devpath=%s i=%d\n", __FUNCTION__, realdir, i);
	kfree(realdir);
	return 0;
}


static int kclsm_path_chmod(struct dentry *dentry, struct vfsmount *mnt,
			   mode_t mode)
{

	char* realpath;
	struct path tmppath;
	int r;
	int i;

	realpath = kmalloc(PATH_MAX, GFP_KERNEL);
	if (!realpath){
		printk(KERN_ERR "%s: No memory\n", __FUNCTION__);
		return -ENOMEM;
	}

	tmppath.dentry = dentry;
	tmppath.mnt = mnt;
	r = _xx_realpath_from_path(&tmppath, realpath, PATH_MAX-1);
	if (r != 0) {
		printk(KERN_ERR "%s: Invalid path\n", __FUNCTION__);
		kfree(realpath);
#ifdef	KCLSM_DEBUG_ENABLE
		return 0;
#else
		return r;
#endif
	}

	for (i=0 ; kclsm_devpermission_list[i].path != NULL ; i++){
		if (strcmp(realpath, kclsm_devpermission_list[i].path) == 0)
		{
			if (kclsm_devpermission_list[i].mode == mode)
			{
				KC_DBGPRINT(KERN_INFO "%s: Permitted-1 devpath=%s i=%d\n", __FUNCTION__, realpath, i);
				kfree(realpath);
				return 0;
			}else{
				printk (KERN_ERR "%s: No permission devpath=%s mode=0x%x i=%d\n", __FUNCTION__, realpath, mode, i);
				kfree(realpath);
#ifdef	KCLSM_DEBUG_ENABLE
				return 0;
#else
				return -EPERM;
#endif
			}
		}
	}

	KC_DBGPRINT(KERN_INFO "%s: Permitted-2 devpath=%s i=%d\n", __FUNCTION__, realpath, i);
	kfree(realpath);
	return 0;
}


static int kclsm_path_chown(struct path *path, uid_t uid, gid_t gid)
{

	char* realpath;
	int r;
	int i;

	realpath = kmalloc(PATH_MAX, GFP_KERNEL);
	if (!realpath){
		printk(KERN_ERR "%s: No memory\n", __FUNCTION__);
		return -ENOMEM;
	}

	r = _xx_realpath_from_path(path, realpath, PATH_MAX-1);
	if (r != 0) {
		printk(KERN_ERR "%s: Invalid path\n", __FUNCTION__);
		kfree(realpath);
#ifdef	KCLSM_DEBUG_ENABLE
		return 0;
#else
		return r;
#endif
	}

	for (i=0 ; kclsm_devpermission_list[i].path != NULL ; i++){
		if (strcmp(realpath, kclsm_devpermission_list[i].path) == 0)
		{
			if (((uid == kclsm_devpermission_list[i].uid) || (uid == -1)) &&
			    ((gid == kclsm_devpermission_list[i].gid) || (gid == -1))){
				KC_DBGPRINT(KERN_INFO "%s: Permitted-1 i=%d\n", __FUNCTION__, i);
				kfree(realpath);
				return 0;
			}else{
				printk (KERN_ERR "%s: No permission (%d:%d) i=%d\n", __FUNCTION__, uid, gid, i);
				kfree(realpath);
#ifdef	KCLSM_DEBUG_ENABLE
				return 0;
#else
				return -EPERM;
#endif
			}
		}
	}

	KC_DBGPRINT(KERN_INFO "%s: Permitted-2 devpath=%s uid=%d gid=%d\n", __FUNCTION__, realpath, uid, gid);
	kfree(realpath);
	return 0;
}

#endif	/* CONFIG_SECURITY_KCLSM_DEVICEFILEPERMISSION */


static struct security_operations kclsm_security_ops = {

#ifdef CONFIG_SECURITY_KCLSM_PTRACE
	.ptrace_access_check =	kclsm_ptrace_access_check,
	.ptrace_traceme =		kclsm_ptrace_traceme,
#endif	/* CONFIG_SECURITY_KCLSM_PTRACE */

	.sb_mount =			kclsm_sb_mount,

#ifdef CONFIG_SECURITY_KCLSM_UMOUNT
	.sb_umount =			kclsm_sb_umount,
#endif	/* CONFIG_SECURITY_KCLSM_UMOUNT */

#ifdef CONFIG_SECURITY_KCLSM_PIVOTROOT
	.sb_pivotroot =		kclsm_sb_pivotroot,
#endif	/* CONFIG_SECURITY_KCLSM_PIVOTROOT */

#ifdef CONFIG_SECURITY_KCLSM_CHROOT
	.path_chroot =		kclsm_path_chroot,
#endif	/* CONFIG_SECURITY_KCLSM_CHROOT */

#ifdef CONFIG_SECURITY_KCLSM_DEVICEFILEPERMISSION
	.path_mknod =		kclsm_path_mknod,
	.path_chmod =		kclsm_path_chmod,
	.path_chown =		kclsm_path_chown,
#endif	/* CONFIG_SECURITY_KCLSM_DEVICEFILEPERMISSION */


};

static int __init kclsm_init (void)
{
	int r;

	r = register_security (&kclsm_security_ops);
	if (r) {
		printk (KERN_ERR "Failure registering KCLSM LSM\n");
#ifdef	KCLSM_DEBUG_ENABLE
			return 0;
#else
			return -EINVAL;
#endif
	}

	printk (KERN_INFO "KCLSM LSM module initialized\n");

	return 0;
}

security_initcall (kclsm_init);
