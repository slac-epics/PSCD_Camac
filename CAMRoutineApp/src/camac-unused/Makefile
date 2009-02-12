 objects = test.o pack.o stubs.o cammod.o camgo.o \
	camio.o camac_build_ctlw.o decode_control_word.o
# camadd.o camalor.o camalo.o camdel.o camgo.o camio.o 
 EPICS1 = /afs/slac/package/epics/vers4/R3.14.6/base/src/libCom/error
 EPICS2 = /afs/slac/package/epics/vers4/R3.14.6/base/src/libCom/osi/
 EPICS3 = /afs/slac/package/epics/vers4/R3.14.6/base/src/libCom/misc 
 EPICS4 = /afs/slac/package/epics/vers4/R3.14.6/base/include/os/Linux/
 RTEMS1 = /afs/slac/package/rtems/vol15/src/rtems/build-i386-rtems/i386-rtems/pc586/lib/include/
 CFLAGS = -gdwarf-2 -g3 -O0 -I. -I$(EPICS1) -I$(EPICS2) -I$(EPICS3) -I$(EPICS4)
 CC = gcc
 LDFLAGS = -L/afs/slac/package/epics/vers4/R3.14.6/base/lib/linux-x86/

 mytest:	$(objects) 
	gcc -o mytest $(CFLAGS) $(LDFLAGS)  $(objects)
 $(objects) : cam_proto.hm  camblkstruc.hm camdef.hc slc_macros.hc
#	camac.hm camac_standards.hc camac_types.hc camdef.inc camdef.hc
clean:
	rm -f mytest $(objects)
