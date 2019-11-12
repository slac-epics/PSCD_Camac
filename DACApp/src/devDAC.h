/***************************************************************************\
 *   $Id: devDAC.h,v 1.1 2009/05/12 07:40:23 pengs Exp $
 *   File:		devDAC.h
 *   Author:		Sheng Peng
 *   Email:		pengsh2003@yahoo.com
 *   Phone:		408-660-7762
 *   Company:		RTESYS, Inc.
 *   Date:		03/2007
 *   Version:		1.0
 *
 *   EPICS device support header file for DAC 
 *
\***************************************************************************/

#ifndef _DEV_DAC_H_
#define _DEV_DAC_H_

#define DAC_DRV_VER_STRING	"CAMAC DAC Driver V1.0"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include <epicsVersion.h>
#if (EPICS_VERSION>=3 && EPICS_REVISION>=14) || EPICS_VERSION>3
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
#include <aoRecord.h>
#include <longinRecord.h>

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

#define DAC_MIN_READ_INTERVAL	1.0	/* 1 second */
#define DAC_NUM_OF_CHANNELS	16	/* 16 channels per module */

/* DAC module,  b,c,n define a unique module */
typedef struct DAC_MODULE
{
    ELLNODE                     node;   /* Link List Node */

    epicsMessageQueueId		msgQId;	/* so far all modules share one Q */

    UINT16			b;	/* branch */
    UINT16			c;	/* crate */
    UINT16			n;	/* node = slot */

    UINT16			moduleID;
} DAC_MODULE;

/******************************************************************************************/
/*********************       Record type we support             ***************************/
/******************************************************************************************/
typedef enum EPICS_RECTYPE
{
    EPICS_RECTYPE_NONE,
    EPICS_RECTYPE_AI,
    EPICS_RECTYPE_AO,
    EPICS_RECTYPE_LI,
}   E_EPICS_RECTYPE;

/* define function flags */
typedef enum {
    DAC_AI_DATA,
    DAC_AO_DATA,
    DAC_LI_MDID,
} DACFUNC;

static struct PARAM_MAP
{
    char param[MAX_FUNC_STRING_LEN];
    enum EPICS_RECTYPE rtyp;
    int  funcflag;
} param_map[] = {
    {"DATA",  EPICS_RECTYPE_AI, DAC_AI_DATA},
    {"DATA",  EPICS_RECTYPE_AO, DAC_AO_DATA},
    {"MDID",  EPICS_RECTYPE_LI, DAC_LI_MDID},
};
#define N_PARAM_MAP (sizeof(param_map)/sizeof(struct PARAM_MAP))

typedef struct DAC_REQUEST
{
    ELLNODE             node;   /* Link List Node */

    DAC_MODULE          *pDACModule;
    dbCommon            *pRecord;

    int                 funcflag; /* read data/write data/read mdid */
    UINT16		a;
    UINT16		f;

    epicsTimeStamp	actTime;
    UINT16		val;
    UINT32	        errCode;
    int                 opDone;
} DAC_REQUEST;

/* vmsstat_t uses low 28 bits */
#define DAC_REQUEST_NO_ERR	0
#define DAC_MODULE_NOT_EXIST	0x10000000
#define DAC_CAM_INIT_FAIL	0x20000000
#define DAC_WRT_CAMIO_FAIL	0x30000000
#define DAC_RDDAT_CAMIO_FAIL	0x40000000

int DACRequestInit(dbCommon * pRecord, struct camacio inout, enum EPICS_RECTYPE rtyp);

/* These functions do what the name defines, no matter pDACRequest->funcflag and f/a */
UINT32 DAC_WriteData(DAC_REQUEST  *pDACRequest);
UINT32 DAC_ReadData(DAC_REQUEST  *pDACRequest);
UINT32 DAC_ReadID(DAC_REQUEST  *pDACRequest);

#ifdef __cplusplus
}
#endif

#endif

