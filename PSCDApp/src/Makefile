TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC_RTEMS += PSCD

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord

INC += drvPSCDLib.h
INC += epicsPciWrapper.h

# install PSCD.dbd into <top>/dbd
DBD += PSCD.dbd

# specify all source files to be compiled and added to the library
PSCD_SRCS += epicsPciWrapper.c
PSCD_SRCS += drvPSCD.c
PSCD_SRCS += PSCDMain.cpp 

PSCD_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

