TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC_RTEMS += PDU

# Install include file
INC += devPDUII.h

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install PDU.dbd into <top>/dbd
# For test version of 360Hz fiducial
#DBD += PDU.dbd
DBD += PDUII.dbd
DBD += PDUDIAG.dbd

#For debugging
USR_CFLAGS += -O0

# specify all source files to be compiled and added to the library
# Test version of 360Hz fiducial
#PDU_SRCS += PDU_Fiducial.c
#PDU_SRCS += PDU_F19.c

PDU_SRCS += PDUIITest_Fiducial.c
PDU_SRCS += PDUIITest_F19.c

PDU_SRCS += drvPDUII.c
PDU_SRCS += devPDUII.c
PDU_SRCS += PDUII_360Task.c

PDU_SRCS += STBTest.c

#PDU Diagnostic
PDU_SRCS += devPDUDIAG.c
PDU_SRCS += drvPDUDIAG.c
PDU_SRCS += fidPDUDIAG.c

PDU_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

