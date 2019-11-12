#==============================================================
#
#  Name: Makefile to build EPICS Application
#
#  Facility: CAMAC Crate Verifier Controls
#
#  Auth: 30-Jun-2009, K. Luchini      (LUCHINI)
#  Rev:  dd-mmm-yyyy, First Lastname  (USERNAME)
#--------------------------------------------------------------
#  Mod:
#       dd-mmm-yyyy, First Lastname   (USERNAME):
#         comment
#
#==============================================================
#
TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#========================================

USR_CFLAGS += -DUSE_TYPED_RSET -DUSE_TYPE_DSET -DUSE_TYPED_DRVET

# Build an IOC support library
LIBRARY_IOC_RTEMS += CV

# Install include files
INC += devCV.h
INC += cratdef.h

# build database defintion file 
# CAMACSoft .dbd will be made up from these files:
#
DBD += CV.dbd
DBD += CVSub.dbd

# For gdb debug flags
USR_CFLAGS += -O0

# The following are compiled and added to the Support library
ifeq ($(OS_CLASS),RTEMS)
CV_SRCS += drvCV.c
CV_SRCS += devCV.c
CV_SRCS += CVTest.c
CV_LIBS += $(EPICS_BASE_IOC_LIBS)
endif

ARCH = linux-x86
LIBRARY_IOC += CVSub
CVSub_SRCS += CVSub.c
CVSub_LIBS += $(EPICS_BASE_IOC_LIBS)

#=======================================
include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
#
# End of file



