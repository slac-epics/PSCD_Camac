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
#       27-Feb-2023, K. Luchini       (LUCHINI):
#        only build libary for RTEMS
#
#==============================================================
#
TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#========================================
#
# Build an IOC support library
LIBRARY_IOC_RTEMS += CV CVSub

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
CV_SRCS += drvCV.c
CV_SRCS += devCV.c
CV_SRCS += CVTest.c

CVSub_SRCS  += CVSub.c
USR_LIBS += $(EPICS_BASE_IOC_LIBS)

#=======================================
include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
#
# End of file



