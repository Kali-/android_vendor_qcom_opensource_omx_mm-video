OMX_VIDEO_PATH := $(call my-dir)
include $(CLEAR_VARS)

ifeq ($(BOARD_USES_QCOM_HARDWARE),true)

ifeq ($(TARGET_BOARD_PLATFORM),qsd8k)
    include $(OMX_VIDEO_PATH)/qdsp6/vdec/Android.mk
    include $(OMX_VIDEO_PATH)/qdsp6/venc/Android.mk
endif

ifeq ($(TARGET_BOARD_PLATFORM),msm7x30)
    include $(OMX_VIDEO_PATH)/vidc/vdec/Android.mk
    include $(OMX_VIDEO_PATH)/vidc/venc/Android.mk
    include $(OMX_VIDEO_PATH)/DivxDrmDecrypt/Android.mk
endif

ifeq ($(TARGET_BOARD_PLATFORM),msm8660)
    include $(OMX_VIDEO_PATH)/vidc/vdec/Android.mk
    include $(OMX_VIDEO_PATH)/vidc/venc/Android.mk
    include $(OMX_VIDEO_PATH)/DivxDrmDecrypt/Android.mk
endif

ifeq ($(TARGET_BOARD_PLATFORM),msm8960)
    include $(OMX_VIDEO_PATH)/vidc/vdec/Android.mk
    include $(OMX_VIDEO_PATH)/vidc/venc/Android.mk
    include $(OMX_VIDEO_PATH)/DivxDrmDecrypt/Android.mk
endif

endif #BOARD_USES_QCOM_HARDWARE
