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

#ifndef MLM1183_H
#define MLM1183_H

//#define MLM1183_CALL_LOG_ON
//#define MLM1183_LOG_ON

#include <mach/board.h>
#include <linux/kernel.h>

#ifdef MLM1183_CALL_LOG_ON
 #define MLM1183_ENTER   printk(KERN_INFO "MLM1183:%s enter\n", __func__)
 #define MLM1183_RETURN  { \
  printk(KERN_INFO "MLM1183:%s(%d) exit\n", __func__, __LINE__); \
  return; \
  }
 #define MLM1183_RETURN_N(x) { \
  printk(KERN_INFO "MLM1183:%s(%d) exit(%ld)\n", \
  __func__, __LINE__, (unsigned long)x); \
  return x; \
 }
#else
 #define MLM1183_ENTER
 #define MLM1183_RETURN             return
 #define MLM1183_RETURN_N(x)        return x
#endif /* MLM1183_CALL_LOG_ON */


#ifdef MLM1183_LOG_ON
#define MLM1183_LOG_OUT(label, fmt, ...) \
  printk(KERN_INFO "MLM1183:%s +%-4d %-28s "fmt"%s\n", \
  label, __LINE__, __func__, __VA_ARGS__)

#define MLM1183_LOG_ERR(...)  MLM1183_LOG_OUT("ERR", __VA_ARGS__, "")
#define MLM1183_LOG_DBG(...)  MLM1183_LOG_OUT("DBG", __VA_ARGS__, "")
#else
#define MLM1183_LOG_ERR(...)
#define MLM1183_LOG_DBG(...)
#endif /* MLM1183_LOG_ON */

#endif /* MLM1183 */

