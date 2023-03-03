/***************************************************************************\
 *   $Id: devSAM.h,v 1.8 2009/08/13 06:12:39 pengs Exp $
 *   File:		devSAM.h
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

#ifndef _DEV_SAM_H_
#define _DEV_SAM_H_

#define SAM_DRV_VER_STRING	"CAMAC SAM Driver V1.0"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include <epicsVersion.h>
#if EPICS_VERSION>3 || (EPICS_VERSION>=3 && EPICS_REVISION>=14)
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

#include "genType.h"
#include "drvPSCDLib.h"

#include "slc_macros.h"
#include "cam_proto.h"

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

#define SAM_MIN_READ_INTERVAL	0.8	/* 0.8 second */
#define SAM_NUM_OF_CHANNELS	32

/* SAM module,  b,c,n define a unique module */
typedef struct SAM_MODULE
{
    ELLNODE                     node;   /* Link List Node */

    UINT16			b;	/* branch */
    UINT16			c;	/* crate */
    UINT16			n;	/* node = slot */

    /* So one task/msgQ per module since reset takes long time */
    epicsMessageQueueId		msgQId;
    epicsThreadId               opTaskId;

    float			fwVer;	/* also used to indicate SAM in known state */

    UINT16			startChannel; /* 0 ~ 31 */
    UINT32			numChannels; /* 0 ~ 32 */

    ELLLIST                     SAMDelayedReqList;

    epicsTimeStamp		lastReadTime;
    float			data[SAM_NUM_OF_CHANNELS];
    UINT32			lastErrCode;

    /*char			camacPreMsg[256];*/	/* Could be done in init second pass */
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
    SAM_AI_FWVER,
    SAM_BO_RESET
} SAMFUNC;

static struct PARAM_MAP
{
    char param[MAX_FUNC_STRING_LEN];
    enum EPICS_RECTYPE rtyp;
    int  funcflag;
} param_map[] = {
    {"DATA",  EPICS_RECTYPE_AI, SAM_AI_DATA},
    {"VERSION", EPICS_RECTYPE_AI, SAM_AI_FWVER},
    {"RESET", EPICS_RECTYPE_BO, SAM_BO_RESET}
};
#define N_PARAM_MAP (sizeof(param_map)/sizeof(struct PARAM_MAP))

typedef struct SAM_REQUEST
{
    ELLNODE             node;   /* Link List Node */

    SAM_MODULE          *pSAMModule;
    dbCommon            *pRecord;

    int                 funcflag; /* read data/reset */
    UINT16		a;
    UINT16		f;

    epicsTimeStamp	reqTime;
    float		val;
    UINT32	        errCode;
    int                 opDone;
} SAM_REQUEST;

/* vmsstat_t uses low 28 bits */
#define SAM_REQUEST_NO_ERR	0
#define SAM_MODULE_NOT_EXIST	0x10000000
#define SAM_CAM_INIT_FAIL	0x20000000
#define SAM_RST_CAMIO_FAIL	0x30000000
#define SAM_SETUP_CAMIO_FAIL	0x40000000
#define SAM_CAM_ALLOC_FAIL	0x50000000
#define SAM_CAM_ADD_FAIL	0x60000000
#define SAM_CAM_GO_FAIL		0x70000000
#define SAM_CAM_DEL_FAIL	0x80000000
#define SAM_MODULE_UNKNOWN_MODE	0x90000000

int SAMRequestInit(dbCommon * pRecord, struct camacio inout, enum EPICS_RECTYPE rtyp);

#ifdef __cplusplus
}
#endif

#endif

