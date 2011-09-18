OMX_VIDEO_PATH := $(call my-dir)
include $(CLEAR_VARS)

ifeq ($(call is-board-platform-in-list,$(QSD8K_BOARD_PLATFORMS)),true)
    include $(OMX_VIDEO_PATH)/qdsp6/vdec/Android.mk
    include $(OMX_VIDEO_PATH)/qdsp6/venc/Android.mk
endif

ifeq ($(call is-chipset-in-board-platform,msm7630),true)
    include $(OMX_VIDEO_PATH)/vidc/vdec/Android.mk
    include $(OMX_VIDEO_PATH)/vidc/venc/Android.mk
    include $(OMX_VIDEO_PATH)/DivxDrmDecrypt/Android.mk
endif

ifeq ($(call is-board-platform,msm8660),true)
    include $(OMX_VIDEO_PATH)/vidc/vdec/Android.mk
    include $(OMX_VIDEO_PATH)/vidc/venc/Android.mk
    include $(OMX_VIDEO_PATH)/DivxDrmDecrypt/Android.mk
endif

ifeq ($(call is-board-platform,msm8960),true)
    include $(OMX_VIDEO_PATH)/vidc/vdec/Android.mk
    include $(OMX_VIDEO_PATH)/vidc/venc/Android.mk
    include $(OMX_VIDEO_PATH)/DivxDrmDecrypt/Android.mk
endif
