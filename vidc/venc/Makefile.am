AM_CFLAGS = -Wall
AM_CFLAGS += -Wundef
AM_CFLAGS += -Wstrict-prototypes
AM_CFLAGS += -Wno-trigraphs
AM_CFLAGS += -g -O3

AM_CPPFLAGS = -Dlrintf=_ffix_r
AM_CPPFLAGS += -D__align=__alignx
AM_CPPFLAGS += -D__alignx\(x\)=__attribute__\(\(__aligned__\(x\)\)\)
AM_CPPFLAGS += -DT_ARM
AM_CPPFLAGS += -Dinline=__inline
AM_CPPFLAGS += -UENABLE_DEBUG_LOW
AM_CPPFLAGS += -DENABLE_DEBUG_HIGH
AM_CPPFLAGS += -DENABLE_DEBUG_ERROR
AM_CPPFLAGS += -UINPUT_BUFFER_LOG
AM_CPPFLAGS += -UOUTPUT_BUFFER_LOG
AM_CPPFLAGS += -USINGLE_ENCODER_INSTANCE
if TARGET_MSM7630
AM_CPPFLAGS += -DMAX_RES_720P
endif
if TARGET_MSM8660
AM_CPPFLAGS += -DMAX_RES_1080P
endif
AM_CPPFLAGS += -Iinc
AM_CPPFLAGS += -I../common/inc

c_sources = src/omx_video_base.cpp
c_sources += src/omx_video_encoder.cpp
c_sources += src/video_encoder_device.cpp
c_sources += ../common/src/extra_data_handler.cpp

lib_LTLIBRARIES = libOmxVenc.la
libOmxVenc_la_SOURCES = $(c_sources)
libOmxVenc_la_CFLAGS = $(AM_CFLAGS) -fPIC
libOmxVenc_la_LDLIBS = -lOmxcore -lstdc++ -lpthread
libOmxVenc_la_LDFLAGS = -shared -version-info $(OMXVIDEO_LIBRARY_VERSION)

bin_PROGRAMS = mm-venc-omx-test720p
bin_PROGRAMS += mm-video-encdrv-test
mm_video_encdrv_test_SOURCES := test/queue.c
mm_video_encdrv_test_SOURCES += test/video_encoder_test.c
mm_video_encdrv_test_LDADD = -lpthread

mm_venc_omx_test720p_SOURCES = test/venc_test.cpp
mm_venc_omx_test720p_SOURCES += test/camera_test.cpp
mm_venc_omx_test720p_SOURCES += test/venc_util.c
mm_venc_omx_test720p_SOURCES += test/fb_test.c
mm_venc_omx_test720p_LDADD = -ldl -lpthread -lOmxCore libOmxVenc.la
