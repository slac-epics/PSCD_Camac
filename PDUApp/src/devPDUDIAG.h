/***************************************************************************\
 *   $Id: devPDUDIAG.h,v 1.2 2010/09/23 15:47:07 rcs Exp $
 *   File:		devPDUDIAG.h
 *   Author:		Robert C. Sass
 *   Email:		rcs@slac.stanford.edu
 *   Phone:		408-464-5853
 *   Company:		SLAC
 *   Date:		08/2010
 *   Version:		1.0
 *
 *   EPICS device support header file for PDU diagnostic 
 *
\***************************************************************************/

#ifndef _DEV_PDUDIAG_H_
#define _DEV_PDUDIAG_H_

#define PDUDIAG_DRV_VER_STRING	"CAMAC PDUDIAG Driver V1.0"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include <epicsVersion.h>
#if EPICS_VERSION>3 || (EPICS_VERSION==3 && EPICS_REVISION>=14)
#include <epicsExport.h>
#include <alarm.h>
#include <dbCommon.h>
#include <dbDefs.h>
#include <dbAccess.h>
#include <dbScan.h>
#include <recSup.h>
#include <recGbl.h>
#include <devSup.h>
#include <drvSup.h>
#include <link.h>
#include <ellLib.h>
#include <errlog.h>
#include <special.h>
#include <epicsTime.h>
#include <epicsEvent.h>
#include <epicsMessageQueue.h>
#include <epicsThread.h>
#include <cantProceed.h>
#include <devLib.h>
#include <special.h>
#include <cvtTable.h>

#include <longoutRecord.h>
#include <waveformRecord.h>

#else
#error "We need EPICS 3.14 or above to support OSI calls!"
#endif

#include "genType.h"

#include "evrTime.h"
#include "evrPattern.h"

/*
** We preserve/use VMS status words to pass back so we need the SUCCESS macro
*/

#include <slc_macros.h>
#include <cam_proto.h>

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************************/
/*********************       EPICS device support return        ***************************/
/******************************************************************************************/
#define CONVERT                 (0)
#define NO_CONVERT              (2)
#define MAX_FUNC_STRING_LEN	(40)

/******************************************************************************************/

/*** #define MAX_SAMPLES 360 ***/
#define MAX_SAMPLES 360

/*
** Prototypes for diagnostic called by 360Hz fiducial
*/
void fidPDUDIAGPreCam  (void *pkg_p);
void fidPDUDIAGPostCam (void);

/* 
** The data collected for each pulse
*/
typedef struct PDUDIAG_FIDD
{
   vmsstat_t       fidstatus;      /* status of this fid sample */
   epicsUInt32     matchstatus;    /* Save match status */
   epicsInt32	   measdelay;      /* Measured delay from STB */
   evrModifier_ta  modifier_a;     /* All modifiers */
   epicsTimeStamp  fidtimestamp;   /* timestamp has pulseID encoded */
} PDUDIAG_FIDD;

/*
** Waveform data struct
*/

typedef struct PDUDIAG_WFD
{
   epicsUInt32      crate;           /* Crate we're sampling */
   epicsUInt32      channel;         /* Channel */
   epicsTimeStamp   timestamp;       /* Time we start collecting */
   vmsstat_t        status;          /* Overall status of the collection */
   PDUDIAG_FIDD     fiddata[MAX_SAMPLES];
} PDUDIAG_WFD;

/*
** Msg sent to 360Hz comm thread
*/
typedef struct THREADMSG
{
   dbCommon     *rec_p;           /* Record pointer */
   UINT32        crate;           /* Crate we're sampling */
   UINT32        channel;         /* Channel */
   void         *wfb_p;           /* Waveform data buffer */
} THREADMSG;

/*
** Waveform record private data.
*/
typedef struct WF_PVT
{
   vmsstat_t        status;          /* status back from thread */
} WF_PVT;

/*
** Error statii for us in the manner of VMS. LSB = 1 = OK
*/
#define DIAG_OKOK        1
#define DIAG_BADPATTERN         0x10000000
#define DIAG_EVENTERR           0x20000000
#define DIAG_NOMEAS             0x30000000
#define DIAG_NOMATCH            0x40000000

/*
** Only STB mode we care about. Module is hard-coded in SCP.
*/
#define STB_MODE_ANY	7
#define STB_MODULE     21
#define STB_STOP_TIMER 0x0001000

/*
** Mode word shifts
*/
#define STB_CHAN_SHIFT 8
#define STB_MODE_SHIFT 13

/*
** Short and long Camac read structs
*/
typedef struct STAT_SDAT
{
    UINT32 stat;
    UINT16 sdat;
} STAT_SDAT;

typedef struct STAT_LDAT
{
    UINT32 stat;
    UINT32 ldat;
} STAT_LDAT;


#ifdef __cplusplus
}
#endif

#endif

