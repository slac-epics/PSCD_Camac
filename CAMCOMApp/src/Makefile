TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC_RTEMS += CAMCOM

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install iCAMCOM.dbd into <top>/dbd
DBD += CAMCOM.dbd

# specify all source files to be compiled and added to the library
CAMCOM_SRCS += devCAMCOM.c
CAMCOM_SRCS += drvCAMCOM.c

# NOTE: To build SNL programs, SNCSEQ must be defined
# in the <top>/configure/RELEASE file
ifneq ($(SNCSEQ),)
#  Supress warnings of escaped C declarations
   CAMCOM_SNCFLAGS += -w
   CAMCOM_SRCS += Camcom_seq.stt
   CAMCOM_LIBS += seq pv
endif

CAMCOM_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

