TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================
USR_CFLAGS += -DUSE_TYPED_RSET -DUSE_TYPED_DSET -DUSE_TYPED_DRVET
#==================================================
# build a support library

LIBRARY_IOC_RTEMS += DAC

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install DAC.dbd into <top>/dbd
DBD += DAC.dbd

# specify all source files to be compiled and added to the library
DAC_SRCS += drvDAC.c
DAC_SRCS += devDAC.c
#DAC_SRCS += DACTest.c

DAC_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

