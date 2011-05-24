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
ifeq "$(findstring msm7630,$(QCOM_TARGET_PRODUCT))" "msm7630"
libOmxVdec-def += -DMAX_RES_720P
endif
ifeq "$(findstring msm8660,$(QCOM_TARGET_PRODUCT))" "msm8660"
libOmxVdec-def += -DMAX_RES_1080P
endif
ifeq "$(findstring msm8960,$(QCOM_TARGET_PRODUCT))" "msm8960"
libOmxVdec-def += -DMAX_RES_1080P
libOmxVdec-def += -DMAX_RES_1080P_EBI
endif

ifneq (, $(filter HONEYCOMB HMJ19, $(BUILD_ID)))
libOmxVdec-def += -D_ANDROID_HONEYCOMB_
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
libmm-vdec-inc          += $(TOP)/hardware/msm7k/libgralloc-qsd8k

LOCAL_MODULE                    := libOmxVdec
LOCAL_MODULE_TAGS               := optional
LOCAL_CFLAGS                    := $(libOmxVdec-def)
LOCAL_C_INCLUDES                := $(libmm-vdec-inc)
LOCAL_ADDITIONAL_DEPENDENCIES   := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

LOCAL_PRELINK_MODULE    := false
LOCAL_SHARED_LIBRARIES  := liblog libutils libbinder libcutils
LOCAL_SHARED_LIBRARIES  += libdivxdrmdecrypt

LOCAL_SRC_FILES         := src/frameparser.cpp
LOCAL_SRC_FILES         += src/h264_utils.cpp
ifeq "$(findstring msm8660,$(TARGET_PRODUCT))" "msm8660"
LOCAL_SRC_FILES         += src/mp4_utils.cpp
endif
ifeq "$(findstring msm8960,$(TARGET_PRODUCT))" "msm8960"
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
