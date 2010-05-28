/*
** devPIOP.h
*/
#ifndef _DEV_CAMCOM_H_
#define _DEV_CAMCOM_H_

#define CAMCOM_VER_STRING	"CAMCOM Device/Driver Support V1.0"

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

/*
** So we can do Camadd in device support
*/
#include <cam_proto.h>
#include <cctlwmasks.h>
#include <camblkstruc.h>
#include <camdef.h>

/*
** Recordtypes we support
*/
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
    EPICS_RECTYPE_WF,
}   E_EPICS_RECTYPE;



/********************************************
 ** Driver private structure for each record.
 *******************************************/
typedef struct
{
  vmsstat_t     status;   /* Status returned from driver */
  void         *val_p;     /* Local record's val field */
} CAMCOM_PVT;

/*
** This is the message send to the thread. All that's needed is 
** the record ptr. Other info is in driver private struct.
*/
typedef struct
{
  dbCommon     *rec_p;
} THREADMSG_TS;

/*****************************************************************
** Prototypes for driver modules referenced by devCAMCOM device support
*****************************************************************/

/* Thread that does the Camac work */

void threadCAMCOM  (void * msgQId);

/* Record-specific driver init */

void CAMCOMDriverInit (dbCommon *rec_p, enum EPICS_RECTYPE rtyp);

#endif
