/* temp stubs for EPICS routines */

#include <stdio.h>
#include "epicsMutex.h"

typedef enum {errlogInfo, errlogMinor,errlogMajor,errlogFatal} errlogSevEnum;

static epicsMutexId dum = {1};
 
int errlogSevPrintf (const errlogSevEnum severity, const char* pformat, ...)
{
  printf ("Entered errlogSevPrintf\n");
    return 1;
}

epicsShareFunc epicsMutexLockStatus epicsShareAPI epicsMutexLock (epicsMutexId id)
{
  printf ("Entered epicsMutexLock\n");
    return 0;
}

epicsShareFunc void epicsShareAPI epicsMutexUnlock (epicsMutexId id)
{
  printf ("Entered epicsMutexUnlock\n");
    return;
}

epicsShareFunc epicsMutexId epicsShareAPI epicsMutexOsiCreate (const char *pfileName, int lineno)
{
  printf ("Entered epicsMutexCreate\n");
    return dum;
}

epicsShareFunc void epicsShareAPI epicsMutexDestroy (epicsMutexId id)
{
  int ret=0;
  printf ("Entered epicsMutexDestroy\n");
    return;
}
