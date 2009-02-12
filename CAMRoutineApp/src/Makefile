TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC_RTEMS += CAMRoutine

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord

INC += slc_macros.h
INC += cam_proto.h

# install CAMRoutine.dbd into <top>/dbd
DBD += CAMRoutine.dbd

# specify all source files to be compiled and added to the library
CAMRoutine_SRCS += camgo.c
CAMRoutine_SRCS += camalo.c
CAMRoutine_SRCS += camalor.c
CAMRoutine_SRCS += camio.c
CAMRoutine_SRCS += camdel.c
CAMRoutine_SRCS += camadd.c
CAMRoutine_SRCS += cammod.c
#CAMRoutine_SRCS += pack.c
#CAMRoutine_SRCS += CAMtest.c
#CAMRoutine_SRCS += DIMtest.c

CAMRoutine_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

