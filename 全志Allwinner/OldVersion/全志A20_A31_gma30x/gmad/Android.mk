#
# CONFIDENTIAL
# Copyright (C) 2014 Globalmems Corporation
#
# test part#
#$(shell cp -rf $(LOCAL_PATH)/gss.sh /home/global/panda_work/android/out/target/product/panda/system/bin/gss.sh)
#PRODUCT_COPY_FILES:=$(LOCAL_PATH)/gss.sh:/system/bin/gss.sh
############
LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
	main.c

#LOCAL_CFLAGS := \
#    -O3 -D__ANDROID__
LOCAL_CFLAGS += \
	-Wall \
	-DENABLE_GMADEBUG=0 \
	-DOUTPUT_STDOUT=0 \
	-DDBG_LEVEL=0 \
	-O3 -D__ANDROID__
	
LOCAL_SHARED_LIBRARIES := libcutils
#LOCAL_SHARED_LIBRARIES := libc libm libutils libcutils
LOCAL_MODULE:= gmad
#LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_TAGS := eng

include $(BUILD_EXECUTABLE)






#build static libraries
#LOCAL_PREBUILT_LIBS := libA.a    /  
#                       libB.a  
#                          
#LOCAL_STATIC_LIBRARIES := libA    /  
#                        libB  
#   
#include $(BUILD_MULTI_PREBUILT) 

#build share libraries
#LOCAL_PATH := $(call my-dir)  
#include $(CLEAR_VARS)  
#   
#LOCAL_PREBUILT_LIBS := libA.so  
#                        libB.so  
#sssinclude $(BUILD_MULTI_PREBUILT) 

