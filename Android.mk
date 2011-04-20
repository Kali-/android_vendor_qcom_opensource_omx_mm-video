OMX_VIDEO_PATH := $(call my-dir)
include $(CLEAR_VARS)

ifeq "$(findstring qsd8250,$(QCOM_TARGET_PRODUCT))" "qsd8250"
    include $(OMX_VIDEO_PATH)/qdsp6/vdec/Android.mk
    include $(OMX_VIDEO_PATH)/qdsp6/venc/Android.mk
endif

ifeq "$(findstring qsd8650a,$(QCOM_TARGET_PRODUCT))" "qsd8650a"
    include $(OMX_VIDEO_PATH)/qdsp6/vdec/Android.mk
    include $(OMX_VIDEO_PATH)/qdsp6/venc/Android.mk
endif

ifeq "$(findstring msm7630,$(QCOM_TARGET_PRODUCT))" "msm7630"
    include $(OMX_VIDEO_PATH)/vidc/vdec/Android.mk
    include $(OMX_VIDEO_PATH)/vidc/venc/Android.mk
    include $(OMX_VIDEO_PATH)/DivxDrmDecrypt/Android.mk
endif

ifeq "$(findstring msm8660,$(QCOM_TARGET_PRODUCT))" "msm8660"
    include $(OMX_VIDEO_PATH)/vidc/vdec/Android.mk
    include $(OMX_VIDEO_PATH)/vidc/venc/Android.mk
    include $(OMX_VIDEO_PATH)/DivxDrmDecrypt/Android.mk
endif

ifeq "$(findstring msm8960,$(QCOM_TARGET_PRODUCT))" "msm8960"
    include $(OMX_VIDEO_PATH)/vidc/vdec/Android.mk
    include $(OMX_VIDEO_PATH)/vidc/venc/Android.mk
    include $(OMX_VIDEO_PATH)/DivxDrmDecrypt/Android.mk
endif
