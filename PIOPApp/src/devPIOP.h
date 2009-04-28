/*
** devPIOP.h
*/
#ifndef _DEV_PIOP_H_
#define _DEV_PIOP_H_

#define SAM_PIOP_VER_STRING	"CAMAC PIOP Device/Driver Support V1.0"

/*
** Include all of the standard C stuff needed by device & driver support
*/
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "errno.h"

/*
** Get all of the EPICS includes we might need.
*/
#include <epicsExport.h>
#include <alarm.h>
#include <dbCommon.h>
#include <dbDefs.h>
#include <dbAccess.h>
#include <dbStaticLib.h>
#include <dbScan.h>
#include <recSup.h>
#include <recGbl.h>
#include <devSup.h>
#include <drvSup.h>
#include <devLib.h>
#include <errlog.h>
#include <special.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsMutex.h>
#include <epicsMessageQueue.h>
#include <epicsThread.h>
#include <cantProceed.h>

/*
** We'll preserve/use VMS status words to pass back so we need the SUCCESS macro
*/

#include <slc_macros.h>
#include <klysdef.h>        /* VMS/micro error codes */

/*
** Recordtypes we support
*/
#include <stringoutRecord.h>
#include <biRecord.h>
#include <longinRecord.h>
#include <waveformRecord.h>

/******************************************************************************************/
/*********************       EPICS device support return        ***************************/
/******************************************************************************************/
#define CONVERT                 (0)
#define NO_CONVERT              (2)
#define MAX_FUNC_STRING_LEN	(40)
/******************************************************************************************/

/******************************************************************************************/
/*********************       Record types we support             ***************************/
/******************************************************************************************/
typedef enum EPICS_RECTYPE
{
    EPICS_RECTYPE_NONE,
    EPICS_RECTYPE_SO,
    EPICS_RECTYPE_BI,
    EPICS_RECTYPE_LI,
    EPICS_RECTYPE_WF
}   E_EPICS_RECTYPE;

/***************
 ** Params for MessageQueue array by crates/slots
***************/

#define MAX_CRATES 9
#define MAX_SLOTS  24
#define MAX_MODULES = MAX_CRATES * MAX_SLOTS

/**************************
 ** Driver private structure.
 **************************/
typedef struct
{
  vmsstat_t  status;  /* Status returned from driver */
} PIOP_PVT;

/************************************************************************
** Message sent to drvPIOP to execute the Camac PIOP function from devPIOP
** device support.
*************************************************************************/

typedef enum {IPL, PPNOFTP, LAST } FUNC_TE; /* Msgs to drvPIOP */

typedef struct
{
  short crate;
  short slot;
  char  fname[40];
} IPLSTRUC_TS;

/*
** Unify all parameters for the message to the PIOP thread.
*/
typedef union
{
  IPLSTRUC_TS ipl;
  int         dum;
} THREADPARM_U;

/*
** This is the general message. All functions get the record ptr.
*/
typedef struct
{
  dbCommon    *rec_p;
  FUNC_TE      func_e;
  THREADPARM_U parm_u;
} THREADMSG_TS;

/****************************************
** Prototypes for all driver modules in this App
*****************************************/

/* Thread that dispatches the Camac work */

void threadPIOP (void * msgQId);

void iplPIOPMain (PIOP_PVT *pvt_p,char *name, short crate, short slot);

int PIOPDriverInit (dbCommon *rec_p, struct camacio inout, enum EPICS_RECTYPE rtyp);

#endif
