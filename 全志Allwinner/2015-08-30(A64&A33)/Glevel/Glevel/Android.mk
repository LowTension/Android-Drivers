LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_MODULE_TAGS := optional

LOCAL_SRC_FILES := $(call all-java-files-under, src)

LOCAL_PACKAGE_NAME := GLevel
#LOCAL_SDK_VERSION := current

#include $(BUILD_PACKAGE)

#########################################################################
# Build JNI Shared Library
#########################################################################

#LOCAL_PATH:= $(LOCAL_PATH)/jni
#include $(CLEAR_VARS)
# Optional tag would mean it doesn't get installed by default
#LOCAL_CFLAGS := -Werror
#LOCAL_SRC_FILES:= Linuxc.c
#LOCAL_MODULE := gsensor
#LOCAL_ARM_MODE := arm
LOCAL_CERTIFICATE := platform
#include $(BUILD_SHARED_LIBRARY)
include $(BUILD_PACKAGE)
