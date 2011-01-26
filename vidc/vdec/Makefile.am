AM_CFLAGS = -Wall
AM_CFLAGS += -Wundef
AM_CFLAGS += -Wstrict-prototypes
AM_CFLAGS += -Wno-trigraphs
AM_CFLAGS += -g -O3

AM_CPPFLAGS := -D__alignx\(x\)=__attribute__\(\(__aligned__\(x\)\)\)
AM_CPPFLAGS += -D__align=__alignx
AM_CPPFLAGS += -Dinline=__inline
AM_CPPFLAGS += -DIMAGE_APPS_PROC
AM_CPPFLAGS += -DCDECL
AM_CPPFLAGS += -DT_ARM
AM_CPPFLAGS += -DNO_ARM_CLZ
AM_CPPFLAGS += -UENABLE_DEBUG_LOW
AM_CPPFLAGS += -DENABLE_DEBUG_HIGH
AM_CPPFLAGS += -DENABLE_DEBUG_ERROR
AM_CPPFLAGS += -UINPUT_BUFFER_LOG
AM_CPPFLAGS += -UOUTPUT_BUFFER_LOG
if TARGET_MSM7630
AM_CPPFLAGS += -DMAX_RES_720P
endif
if TARGET_MSM8660
AM_CPPFLAGS += -DMAX_RES_1080P
endif
AM_CPPFLAGS += -Iinc
AM_CPPFLAGS += -I../common/inc

c_sources = src/frameparser.cpp
c_sources += src/h264_utils.cpp
if TARGET_MSM8660
c_sources += src/mp4_utils.cpp
endif
c_sources += src/omx_vdec.cpp
c_sources += ../common/src/extra_data_handler.cpp

lib_LTLIBRARIES = libOmxVdec.la
libOmxVdec_la_SOURCES = $(c_sources)
libOmxVdec_la_CFLAGS = $(AM_CFLAGS) -fPIC
libOmxVdec_la_LDLIBS = -lOmxcore -lstdc++ -lpthread
libOmxVdec_la_LDFLAGS = -shared -version-info $(OMXVIDEO_LIBRARY_VERSION)

bin_PROGRAMS = mm-vdec-omx-test
bin_PROGRAMS += mm-vdec-drv-test

mm_vdec_omx_test_SOURCES := src/queue.c
mm_vdec_omx_test_SOURCES += test/omx_vdec_test.cpp
mm_vdec_omx_test_LDADD = -lOmxCore -ldl -lpthread libOmxVdec.la

mm_vdec_drv_test_SOURCES := src/message_queue.c
mm_vdec_drv_test_SOURCES += test/decoder_driver_test.c
mm_vdec_drv_test_LDADD = -lpthread
