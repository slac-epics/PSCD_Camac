TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================
USR_CFLAGS += -DUSE_TYPED_RSET -DUSE_TYPED_DSET -DUSE_TYPED_DRVET
#==================================================
# build a support library

LIBRARY_IOC_RTEMS += LDIM

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install LDIM.dbd into <top>/dbd
DBD += LDIM.dbd

# specify all source files to be compiled and added to the library
LDIM_SRCS += drvLDIM.c
LDIM_SRCS += devLDIM.c
LDIM_SRCS += LDIMTest.c
#LDIM_SRCS += xxx

LDIM_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

