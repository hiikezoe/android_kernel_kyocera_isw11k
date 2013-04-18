KERNEL_DIR ?= $(ANDROID_BUILD_TOP)/../kernel
KERNEL_OUT ?= $(ANDROID_PRODUCT_OUT)/obj/KERNEL_OBJ

INSTALL_DIR ?= $(ANDROID_PRODUCT_OUT)/system/lib/modules

MODULES = kc_sdgdrv
obj-m := $(MODULES).o
$(MODULES)-objs += sdgpiodrv.o

MAKE_FLAGS := -C $(KERNEL_DIR) O=$(KERNEL_OUT) ARCH=arm CROSS_COMPILE=arm-eabi-
EXTRA_CFLAGS += $(addprefix -I, $(INC_PATH))

EXTRA_CFLAGS += -D_LINUX_KERNEL_

ifeq ($(SDAS_DBG), ON)
EXTRA_CFLAGS += -DSDASDBG
endif

ifeq ($(KERNEL_DBG), ON)
EXTRA_CFLAGS += -D_KERNEL_DBG_
endif

