ifneq ($(TARGET_SIMULATOR),true)

LOCAL_PATH:= $(call my-dir)

GME_FS_LIB=libGME_OSS
#export GMED_DEVICE_TYPE=303
export GMED_DEVICE_TYPE=306
#export GMED_DEVICE_TYPE=605
##### GME daemon ###############################################################
include $(CLEAR_VARS)

LOCAL_C_INCLUDES := \
	$(KERNEL_HEADERS) \
	$(LOCAL_PATH)/$(GME_FS_LIB)

LOCAL_SRC_FILES:= \
	$(GME_FS_LIB)/GMFS_AOC.c \
	$(GME_FS_LIB)/GMFS_Decomp.c \
	$(GME_FS_LIB)/GMFS_Device.c \
	$(GME_FS_LIB)/GMFS_Direction.c \
	$(GME_FS_LIB)/GMFS_VNorm.c \
	$(GME_FS_LIB)/gMfusion.c \
	$(GME_FS_LIB)/ossf_magnetic.c \
	GMFS_Driver.c \
	GMFS_APIs.c \
	GMFS_Disp.c \
	GMFS_FileIO.c \
	GMFS_Measure.c \
	main.c

LOCAL_CFLAGS += -Wall
LOCAL_CFLAGS += -DGMFS_OUTPUT_AVEC
LOCAL_CFLAGS += -DGME_VALUE_CHECK
LOCAL_CFLAGS += -DENABLE_GMEDEBUG=1
LOCAL_MULTILIB := 32

ifeq ($(GMED_DEVICE_TYPE), 303)
LOCAL_CFLAGS += -DGME_DEVICE_GMC303
endif

ifeq ($(GMED_DEVICE_TYPE), 306)
LOCAL_CFLAGS += -DGME_DEVICE_GMC306
endif

ifeq ($(GMED_DEVICE_TYPE), 605)
LOCAL_CFLAGS += -DGME_DEVICE_GME605
endif

LOCAL_MODULE := gmadfs306
LOCAL_MODULE_TAGS := eng
LOCAL_FORCE_STATIC_EXECUTABLE := false
LOCAL_SHARED_LIBRARIES := libc libm libcutils
include $(BUILD_EXECUTABLE)


endif  # TARGET_SIMULATOR != true

