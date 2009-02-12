TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC_RTEMS += IDIM

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord

# install IDIM.dbd into <top>/dbd
DBD += IDIM.dbd

# specify all source files to be compiled and added to the library
IDIM_SRCS += devIDIM.c
IDIM_SRCS += DIMtest.c
IDIM_SRCS += CAMtest.c

IDIM_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

