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
#include "math.h"
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
#include <epicsEvent.h>
#include <cantProceed.h>

/*
** We'll preserve/use VMS status words to pass back so we need the SUCCESS macro
*/
#include <camdef.h>         /* VMS/micro error codes */

/*
** So we can do Camadd in device support
*/
#include <slc_macros.h>
#include <cam_proto.h>
#include <cctlwmasks.h>

/*
** Recordtypes we support
*/
#include <stringoutRecord.h>
#include <waveformRecord.h>
#include <mbbiRecord.h>
#include <biRecord.h>
#include <aiRecord.h>
#include <longinRecord.h>
#include <longoutRecord.h>
#include <subRecord.h>

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
    EPICS_RECTYPE_WF,
    EPICS_RECTYPE_MBBI,
    EPICS_RECTYPE_BI,
    EPICS_RECTYPE_LI,
    EPICS_RECTYPE_LO,
    EPICS_RECTYPE_AI,
}   E_EPICS_RECTYPE;

/******************************************************
 ** Params for MessageQueue array by crates/slots, 
 ** PIOPs and types of FTPs.
 ****************************************************/

#define MAX_CRATES 9
#define MAX_SLOTS  24
#define MAX_MODULES = MAX_CRATES * MAX_SLOTS
#define MAX_PIOPS 9
#define MAX_FTP_IDX 21

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

/************************************************************************
** Message sent to drvPIOP to execute the Camac PIOP function. CAMFUNC is
** saved in the driver private struct at record init time and
** tells the thread what Camac function to perform. CAMFUNC functions are 
** for the PIOP thread unless preceeded by "SBIxxx" in which case they're
** for the SBI thread.
*************************************************************************/

typedef enum {IPL, PPNOFTP, PPFTP, STATUSBLOCK, FTP, PAD, MK2, TRIMSLED, FOXHOME, 
              SBIMSGPIOP, SBISTATUS, SBIDELAY, INVALID } CAMFUNC_TE; /* Camac funcs to perform */

/********************************************
 ** Driver private structure for all record.
 ** Not all fields are used by all records.
 *******************************************/
typedef struct
{
   dbCommon     *rec_p;     /* Record pointer */
   vmsstat_t     status;    /* Status returned from driver */
   short         crate;
   short         slot;
   void         *val_p;     /* Local record's val field */
   CAMFUNC_TE    camfunc_e; /* Camac function for waveforms */
   int           phase_idx; /* Index into Piop_Phase_s for RT ai phase */
} PIOP_PVT;

/*
** This is the message send to the thread. All that's needed is 
** the record ptr. Other info is in driver private struct.
*/
typedef struct
{
  dbCommon     *rec_p;
} THREADMSG_TS;

/*****************************************************************
** Prototypes for driver modules referenced by devPIOP device support
*****************************************************************/

/* Threads that dispatch the Camac work */

void threadPIOP (void * msgQId);
void threadSBI  (void * msgQId);

/* Start phase read thread */

epicsThreadId fidPHASEStart(void);

/* Record-specific driver init */

void PIOPDriverInit (dbCommon *rec_p,  struct camacio *cam_ps, enum EPICS_RECTYPE rtyp);

#endif
