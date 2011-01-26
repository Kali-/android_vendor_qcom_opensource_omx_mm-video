#                                               -*- Autoconf -*-

# configure.ac -- Autoconf script for mm-omxvideo
#

# Process this file with autoconf to produce a configure script.

AC_PREREQ(2.61)
AC_INIT([omxvideo],
	1.0.0)
AM_INIT_AUTOMAKE([-Wall -Werror gnu foreign])
AM_MAINTAINER_MODE
AC_CONFIG_HEADER([config.h])
AC_CONFIG_MACRO_DIR([m4])

#release versioning
OMXVIDEO_MAJOR_VERSION=1
OMXVIDEO_MINOR_VERSION=0
OMXVIDEO_MICRO_VERSION=0

OMXVIDEO_LIBRARY_VERSION=$OMXVIDEO_MAJOR_VERSION:$OMXVIDEO_MINOR_VERSION:$OMXVIDEO_MICRO_VERSION
AC_SUBST(OMXVIDEO_LIBRARY_VERSION)

# Checks for programs.
AC_PROG_CC
AC_PROG_CPP
AC_PROG_CXX
AM_PROG_CC_C_O
AC_PROG_LIBTOOL
AC_PROG_AWK
AC_PROG_INSTALL
AC_PROG_LN_S
AC_PROG_MAKE_SET

AC_ARG_ENABLE([target-msm7630],
	AC_HELP_STRING([--enable-target-msm7630],
		[Enable conditional compile for target msm7630 [default=no]]),
	[target_msm7630="${enableval}"],
	target_msm7630=no)

AC_ARG_ENABLE([target-msm8660],
	AC_HELP_STRING([--enable-target-msm8660],
		[Enable conditional compile for target msm8660 [default=no]]),
	[target_msm8660="${enableval}"],
	target_msm8660=no)

AM_CONDITIONAL([TARGET_MSM7630], [test x$target_msm7630 = xyes])
AM_CONDITIONAL([TARGET_MSM8660], [test x$target_msm8660 = xyes])

AC_ARG_WITH([sanitized-headers],
	[AS_HELP_STRING([--with-sanitized-headers=DIR],[location of the sanitized Linux headers])],
	[CPPFLAGS="$CPPFLAGS -I$withval"])

AC_CONFIG_FILES([     \
	Makefile \
	vidc/Makefile \
	vidc/vdec/Makefile \
	vidc/venc/Makefile \
	])
AC_OUTPUT
