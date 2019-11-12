TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================
USR_CFLAGS += -DUSE_TYPED_RSET -DUSE_TYPED_DSET -DUSE_TYPED_DRVET
#==================================================
# build a support library

LIBRARY_IOC_RTEMS += IDOM

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install IDOM.dbd into <top>/dbd
DBD += IDOM.dbd

# specify all source files to be compiled and added to the library
IDOM_SRCS += drvIDOM.c
IDOM_SRCS += devIDOM.c
IDOM_SRCS += IDOMTest.c

IDOM_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

