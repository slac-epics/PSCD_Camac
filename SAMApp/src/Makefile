TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC_RTEMS += SAM

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install SAM.dbd into <top>/dbd
DBD += SAM.dbd

# specify all source files to be compiled and added to the library
SAM_SRCS += drvSAM.c
SAM_SRCS += SAMTest.c
SAM_SRCS += devSAM.c

SAM_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

