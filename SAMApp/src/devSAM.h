/***************************************************************************\
 *   $Id: devSAM.h,v 1.1.1.1 2009/02/12 20:08:59 pengs Exp $
 *   File:		devSAM.c
 *   Author:		Sheng Peng
 *   Email:		pengsh2003@yahoo.com
 *   Phone:		408-660-7762
 *   Company:		RTESYS, Inc.
 *   Date:		03/2007
 *   Version:		1.0
 *
 *   EPICS device support header file for SAM 
 *
\***************************************************************************/

#ifndef _DEV_SAN_H_
#define _DEV_SAN_H_

#define SAM_DRV_VER_STRING	"CAMAC SAM Driver V1.0"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"

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
#include <devLib.h>
#include <special.h>
#include <cvtTable.h>

#include <aiRecord.h>
#include <boRecord.h>

#else
#error "We need EPICS 3.14 or above to support OSI calls!"
#endif

#include "drvPSCDLib.h"

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

#define SAM_MIN_READ_INTERVAL	1.0	/* 1 second */

/* SAM module,  b,c,n define a unique module */
typedef struct SAM_MODULE
{
    ELLNODE                     node;   /* Link List Node */

    epicsMessageQueueId		msgQId;	/* so far all modules share one Q */

    short int			b;	/* branch */
    short int			c;	/* crate */
    short int			n;	/* node = slot */

    int				fwVer;	/* also used to indicate SAM in known state */

    epicsTimeStamp		lastReadTime;

    ELLLIST                     SAMDelayedReqList;

    unsigned int		startChannel; /* 0 ~ 31 */
    unsigned int		endChannel; /* 0 ~ 31 */
    unsigned short int		data[32];
    int                         lastErrCode;

    char			camacPreMsg[256];
} SAM_MODULE;

/******************************************************************************************/
/*********************       Record type we support             ***************************/
/******************************************************************************************/
typedef enum EPICS_RECTYPE
{
    EPICS_RECTYPE_NONE,
    EPICS_RECTYPE_AI,
    EPICS_RECTYPE_BO,
}   E_EPICS_RECTYPE;

/* define function flags */
typedef enum {
    SAM_AI_DATA,
    SAM_BO_RESET
} SAMFUNC;

static struct PARAM_MAP
{
    char param[MAX_FUNC_STRING_LEN];
    enum EPICS_RECTYPE rtyp;
    int  funcflag;
} param_map[] = {
    {"DATA",  EPICS_RECTYPE_AI, SAM_AI_DATA},
    {"RESET", EPICS_RECTYPE_BO, SAM_BO_RESET}
};
#define N_PARAM_MAP (sizeof(param_map)/sizeof(struct PARAM_MAP))

typedef struct SAM_REQUEST
{
    ELLNODE             node;   /* Link List Node */

    SAM_MODULE          *pSAMModule;
    dbCommon            *pRecord;

    int                 funcflag; /* read data/reset */
    short		a;
    short		f;

    epicsTimeStamp	reqTime;
    unsigned short	val;
    int		        errCode;
    int                 opDone;
} SAM_REQUEST;

#define SAM_REQUEST_NO_ERR	0

int SAMRequestInit(dbCommon * pRecord, struct camacio inout, enum EPICS_RECTYPE rtyp);

#ifdef __cplusplus
}
#endif

#endif

