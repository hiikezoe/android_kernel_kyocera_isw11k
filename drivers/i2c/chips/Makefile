#
# This software is contributed or developed by KYOCERA Corporation.
# (C) 2011 KYOCERA Corporation
#

#
# Makefile for miscellaneous I2C chip drivers.
#
# Do not add new drivers to this directory! It is DEPRECATED.
#
# Device drivers are better grouped according to the functionality they
# implement rather than to the bus they are connected to. In particular:
# * Hardware monitoring chip drivers go to drivers/hwmon
# * RTC chip drivers go to drivers/rtc
# * I/O expander drivers go to drivers/gpio
#

obj-$(CONFIG_MACH_MSM8655_KC_F41_ML8953B_HSDT004A) += alps-input.o hscd_i2c.o ml8953b.o
obj-$(CONFIG_MACH_MSM8655_KC_F41_KXUD9_HSDT004A) += alps-input.o hscd_i2c.o kxud9.o
obj-$(CONFIG_MACH_MSM8655_KC_F41_GP2AP003A10F)     += gp2ap003a10f.o
obj-$(CONFIG_SENSORS_MT9T013)		           += mt9t013.o

ifeq ($(CONFIG_I2C_DEBUG_CHIP),y)
EXTRA_CFLAGS += -DDEBUG
endif

