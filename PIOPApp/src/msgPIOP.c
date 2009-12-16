/******************************************
 ** routines to process the PIOP message word
 ******************************************/

#include <devPIOP.h>
#include <drvPIOP.h>
#include <epicsExport.h>
#include <registryFunction.h>

long msgPIOPinit (struct subRecord *subr_p)
{
   printf ("Entered msgPIOPinit\n");
   return (0);
}

long msgPIOPproc (struct subRecord *subr_p)
{
   printf ("Entered msgPIOPproc\n");
   return (0);
}

epicsRegisterFunction(msgPIOPinit);
epicsRegisterFunction(msgPIOPproc);
