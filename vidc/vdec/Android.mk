ifneq ($(BUILD_TINY_ANDROID),true)

ROOT_DIR := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_PATH:= $(ROOT_DIR)

# ---------------------------------------------------------------------------------
# 				Common definitons
# ---------------------------------------------------------------------------------

libOmxVdec-def := -D__alignx\(x\)=__attribute__\(\(__aligned__\(x\)\)\)
libOmxVdec-def += -D__align=__alignx
libOmxVdec-def += -Dinline=__inline
libOmxVdec-def += -g -O3
libOmxVdec-def += -DIMAGE_APPS_PROC
libOmxVdec-def += -D_ANDROID_
libOmxVdec-def += -DCDECL
libOmxVdec-def += -DT_ARM
libOmxVdec-def += -DNO_ARM_CLZ
libOmxVdec-def += -UENABLE_DEBUG_LOW
libOmxVdec-def += -DENABLE_DEBUG_HIGH
libOmxVdec-def += -DENABLE_DEBUG_ERROR
libOmxVdec-def += -UINPUT_BUFFER_LOG
libOmxVdec-def += -UOUTPUT_BUFFER_LOG
ifeq ($(call is-chipset-in-board-platform,msm7630),true)
libOmxVdec-def += -DMAX_RES_720P
libOmxVdec-def += -UPROCESS_EXTRADATA_IN_OUTPUT_PORT
endif
ifeq ($(call is-board-platform,msm8660),true)
libOmxVdec-def += -DMAX_RES_1080P
libOmxVdec-def += -DPROCESS_EXTRADATA_IN_OUTPUT_PORT
libOmxVdec-def += -DTEST_TS_FROM_SEI
endif
ifeq ($(call is-board-platform,msm8960),true)
libOmxVdec-def += -DMAX_RES_1080P
libOmxVdec-def += -DMAX_RES_1080P_EBI
libOmxVdec-def += -DPROCESS_EXTRADATA_IN_OUTPUT_PORT
endif

ifeq ($(call is-android-codename,HONEYCOMB),true)
libOmxVdec-def += -D_ANDROID_HONEYCOMB_
endif

ifeq ($(call is-android-codename,ICECREAM_SANDWICH),true)
libOmxVdec-def += -D_ANDROID_ICS_
endif

ifeq ($(TARGET_USES_ION),true)
libOmxVdec-def += -DUSE_ION
endif

# ---------------------------------------------------------------------------------
# 			Make the Shared library (libOmxVdec)
# ---------------------------------------------------------------------------------

include $(CLEAR_VARS)
LOCAL_PATH:= $(ROOT_DIR)

libmm-vdec-inc          := $(LOCAL_PATH)/inc
libmm-vdec-inc          += $(OMX_VIDEO_PATH)/vidc/common/inc

libmm-vdec-inc          += $(TARGET_OUT_HEADERS)/mm-core/omxcore
libmm-vdec-inc          += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include
#DRM include - Interface which loads the DRM library
libmm-vdec-inc	        += $(OMX_VIDEO_PATH)/DivxDrmDecrypt/inc

ifeq ($(call is-android-codename,ICECREAM_SANDWICH),true)
libmm-vdec-inc          += $(TOP)/hardware/qcom/display/libgralloc \
                        += $(TOP)/hardware/qcom/display/libgenlock
else
libmm-vdec-inc          += $(TOP)/hardware/msm7k/libgralloc-qsd8k
endif

LOCAL_MODULE                    := libOmxVdec
LOCAL_MODULE_TAGS               := optional
LOCAL_CFLAGS                    := $(libOmxVdec-def)
LOCAL_C_INCLUDES                := $(libmm-vdec-inc)
LOCAL_ADDITIONAL_DEPENDENCIES   := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

LOCAL_PRELINK_MODULE    := false
LOCAL_SHARED_LIBRARIES  := liblog libutils libbinder libcutils

ifeq ($(call is-android-codename,ICECREAM_SANDWICH),true)
LOCAL_SHARED_LIBRARIES += libgenlock
endif
LOCAL_SHARED_LIBRARIES  += libdivxdrmdecrypt

LOCAL_SRC_FILES         := src/frameparser.cpp
LOCAL_SRC_FILES         += src/h264_utils.cpp
LOCAL_SRC_FILES         += src/ts_parser.cpp
ifeq ($(call is-board-platform-in-list,msm8660 msm8960),true)
LOCAL_SRC_FILES         += src/mp4_utils.cpp
endif
LOCAL_SRC_FILES         += src/omx_vdec.cpp
LOCAL_SRC_FILES         += ../common/src/extra_data_handler.cpp
include $(BUILD_SHARED_LIBRARY)

# ---------------------------------------------------------------------------------
# 			Make the apps-test (mm-vdec-omx-test)
# ---------------------------------------------------------------------------------
include $(CLEAR_VARS)

mm-vdec-test-inc    := $(TARGET_OUT_HEADERS)/mm-core/omxcore
mm-vdec-test-inc    += $(LOCAL_PATH)/inc
mm-vdec-test-inc    += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include

LOCAL_MODULE                    := mm-vdec-omx-test
LOCAL_MODULE_TAGS               := optional
LOCAL_CFLAGS                    := $(libOmxVdec-def)
LOCAL_C_INCLUDES                := $(mm-vdec-test-inc)
LOCAL_ADDITIONAL_DEPENDENCIES   := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

LOCAL_PRELINK_MODULE      := false
LOCAL_SHARED_LIBRARIES    := libutils libOmxCore libOmxVdec libbinder

LOCAL_SRC_FILES           := src/queue.c
LOCAL_SRC_FILES           += test/omx_vdec_test.cpp

include $(BUILD_EXECUTABLE)

# ---------------------------------------------------------------------------------
# 			Make the driver-test (mm-video-driver-test)
# ---------------------------------------------------------------------------------
include $(CLEAR_VARS)

mm-vdec-drv-test-inc    := $(TARGET_OUT_HEADERS)/mm-core/omxcore
mm-vdec-drv-test-inc    += $(LOCAL_PATH)/inc
mm-vdec-drv-test-inc    += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include

LOCAL_MODULE                    := mm-video-driver-test
LOCAL_MODULE_TAGS               := optional
LOCAL_CFLAGS                    := $(libOmxVdec-def)
LOCAL_C_INCLUDES                := $(mm-vdec-drv-test-inc)
LOCAL_ADDITIONAL_DEPENDENCIES   := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr
LOCAL_PRELINK_MODULE            := false

LOCAL_SRC_FILES                 := src/message_queue.c
LOCAL_SRC_FILES                 += test/decoder_driver_test.c

include $(BUILD_EXECUTABLE)

endif #BUILD_TINY_ANDROID

# ---------------------------------------------------------------------------------
#                END
# ---------------------------------------------------------------------------------
