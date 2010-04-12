TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC_RTEMS += PDU

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install PDU.dbd into <top>/dbd
DBD += PDU.dbd
DBD += PDUII.dbd

# specify all source files to be compiled and added to the library
PDU_SRCS += PDU_Fiducial.c
#PDU_SRCS += PDU_F19.c
PDU_SRCS += drvPDUII.c
PDU_SRCS += devPDUII.c
PDU_SRCS += PDUII_360Task.c

PDU_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

