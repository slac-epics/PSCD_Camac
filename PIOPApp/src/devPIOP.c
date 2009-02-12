/***************************************************************************\
 *   $Id: devPIOP.c,v 0.1 2009/01/29 $
 *   File:		devPIOP.c
 *   Author:		Robert C. Sass
 *   Email:		bsassy@garlic.com
 *   Phone:		408-464-5853
 *   Company:		SLAC
 *   Date:		02/2009
 *   Version:		.01
 *
 *   EPICS device support file for PIOP 
 *
\***************************************************************************/
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "drvPSCD.h"

#include <epicsVersion.h>
#if EPICS_VERSION>=3 && EPICS_REVISION>=14
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
#include <epicsMutex.h>
#include <epicsInterrupt.h>
#include <epicsMessageQueue.h>
#include <epicsThread.h>
#include <cantProceed.h>
#include <osiSock.h>
#include <devLib.h>
#include <special.h>
#include <cvtTable.h>

#include <aiRecord.h>
#include <aoRecord.h>
#include <boRecord.h>
#include <longinRecord.h>
#include <longoutRecord.h>
#include <mbbiDirectRecord.h>
#include <stringinRecord.h>
#include <waveformRecord.h>

#else
#error "We need EPICS 3.14 or above to support OSI calls!"
#endif


/******************************************************************************************/
/*********************       EPICS device support return        ***************************/
/******************************************************************************************/
#define CONVERT                 (0)
#define NO_CONVERT              (2)
#define MAX_FUNC_STRING_LEN	(40)
/******************************************************************************************/

/******************************************************************************************/
/*********************       Record type we support             ***************************/
/******************************************************************************************/
typedef enum EPICS_RECTYPE
{
    EPICS_RECTYPE_NONE,
    EPICS_RECTYPE_AI,
    EPICS_RECTYPE_AO,
    EPICS_RECTYPE_BO,
    EPICS_RECTYPE_LI,
    EPICS_RECTYPE_LO,
    EPICS_RECTYPE_MBBID,
    EPICS_RECTYPE_SI,
    EPICS_RECTYPE_SO,
    EPICS_RECTYPE_WF
}   E_EPICS_RECTYPE;
/******************************************************************************************/


/******************************************************************************************/
/*********************              implementation              ***************************/
/******************************************************************************************/


/* global struct for devSup */
typedef struct {
        long            number;
        DEVSUPFUN       report;
        DEVSUPFUN       init;
        DEVSUPFUN       init_record;
        DEVSUPFUN       get_ioint_info;
        DEVSUPFUN       read_write;
        DEVSUPFUN       special_linconv;
} PIOP_DEV_SUP_SET;

