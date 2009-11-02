TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC_RTEMS += PPOM

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install PPOM.dbd into <top>/dbd
DBD += PPOM.dbd

# specify all source files to be compiled and added to the library
PPOM_SRCS += devPPOM.c
PPOM_SRCS += drvPPOM.c

PPOM_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

