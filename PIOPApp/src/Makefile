TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================
USR_CFLAGS += -DUSE_TYPED_RSET -DUSE_TYPED_DSET -DUSE_TYPED_DRVET
#==================================================
# build a support library

LIBRARY_IOC_RTEMS += PIOP

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install PIOP.dbd into <top>/dbd
DBD += PIOP.dbd

#USR_CFLAGS += -O0

# specify all source files to be compiled and added to the library
PIOP_SRCS += devPIOP.c
PIOP_SRCS += drvPIOP.c
PIOP_SRCS += iplPIOP.c
PIOP_SRCS += blockPIOP.c
PIOP_SRCS += msgPIOP.c
PIOP_SRCS += phasePIOP.c
PIOP_SRCS += fidPHASE.c
PIOP_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

