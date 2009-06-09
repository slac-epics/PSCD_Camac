/*
** devPIOP.h
*/
#ifndef _DEV_PIOP_H_
#define _DEV_PIOP_H_

#define PIOP_VER_STRING	"CAMAC PIOP Device/Driver Support V1.0"

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
** So we can do Camadd in device support
*/
#include <cam_proto.h>
#include <cctlwmasks.h>

/*
** Recordtypes we support
*/
#include <stringoutRecord.h>
#include <biRecord.h>
#include <longinRecord.h>
#include <waveformRecord.h>
#include <mbbiRecord.h>

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
    EPICS_RECTYPE_WF,
    EPICS_RECTYPE_MBBI
}   E_EPICS_RECTYPE;

/******************************************************
 ** Params for MessageQueue array by crates/slots & PIOPs
 ****************************************************/

#define MAX_CRATES 9
#define MAX_SLOTS  24
#define MAX_MODULES = MAX_CRATES * MAX_SLOTS
#define MAX_PIOPS 9

/*
** Generic status/data structs
*/
typedef struct
{
    unsigned int stat;
    unsigned int data;
} STAT_DAT32;

typedef struct
{
    unsigned int stat;
    unsigned short data;
} STAT_DAT16;

/**************************
 ** Driver private structure.
 **************************/
typedef struct
{
  vmsstat_t  status; /* Status returned from driver */
  unsigned long val; /* Simple value returned */
} PIOP_PVT;

/************************************************************************
** Message sent to drvPIOP to execute the Camac PIOP function from devPIOP
** device support.
** Functions are for the PIOP thread unless preceeded by "SBIxxx" in which
** they're for the SBI thread. All threads handle the INIT message.
*************************************************************************/

typedef enum {INIT, IPL, PPN, SBIMSG, SBISTS, LAST } FUNC_TE; /* Msgs to drvPIOP threads */

typedef struct
{
  char  fname[40];
} IPLSTRUC_TS;

typedef struct
{
  char  *indat_p;
} PPSTRUC_TS;


/*
** Unify all parameters for the message to the PIOP thread.
*/
typedef union
{
  IPLSTRUC_TS ipl;
  PPSTRUC_TS  pp;
  int         dum;
} THREADPARM_U;

/*
** This is the general message. All functions get the record ptr,
** crate, slot & function.
*/
typedef struct
{
  dbCommon    *rec_p;
  short crate;
  short slot;
  FUNC_TE      func_e;
  THREADPARM_U parm_u;
} THREADMSG_TS;

/*****************************************************************
** Prototypes for driver modules referenced by devPIOP device support
*****************************************************************/

/* Threads that dispatch the Camac work */

void threadPIOP (void * msgQId);
void threadSBI  (void * msgQId);

/* Record-specific driver init */

int PIOPDriverInit (dbCommon *rec_p, struct camacio inout, enum EPICS_RECTYPE rtyp);

#endif
