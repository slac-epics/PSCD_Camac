/***************************************************************************\
 *   $Id: devPPOM.h,v 1.2 2009/11/02 06:07:58 pengs Exp $
 *   File:		devPPOM.h
 *   Author:		Sheng Peng
 *   Email:		pengsh2003@yahoo.com
 *   Phone:		408-660-7762
 *   Company:		RTESYS, Inc.
 *   Date:		03/2007
 *   Version:		1.0
 *
 *   EPICS device support header file for PPOM 
 *
\***************************************************************************/

#ifndef _DEV_PPOM_H_
#define _DEV_PPOM_H_

#define PPOM_DRV_VER_STRING	"CAMAC PPOM Driver V1.0"

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

#include <boRecord.h>
#include <mbbiDirectRecord.h>
#include <mbboDirectRecord.h>

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

/* PPOM module,  b,c,n define a unique module */
typedef struct PPOM_MODULE
{
    ELLNODE                     node;   /* Link List Node */

    epicsMessageQueueId		msgQId;	/* so far all modules share one Q */

    UINT16			b;	/* branch */
    UINT16			c;	/* crate */
    UINT16			n;	/* node = slot */

/*    epicsTimeStamp		lastReadTime;
    UINT16			data[2];
    UINT32			lastErrCode;

    char			camacPreMsg[256];*/	/* Could be done in init second pass */
} PPOM_MODULE;

/******************************************************************************************/
/*********************       Record type we support             ***************************/
/******************************************************************************************/
typedef enum EPICS_RECTYPE
{
    EPICS_RECTYPE_NONE,
    EPICS_RECTYPE_BO,
    EPICS_RECTYPE_MBBID,
    EPICS_RECTYPE_MBBOD,
}   E_EPICS_RECTYPE;

/* define function flags */
typedef enum {
    PPOM_BO_RESET,
    PPOM_MBBID_DATA,
    PPOM_MBBOD_DATA,
} PPOMFUNC;

static struct PARAM_MAP
{
    char param[MAX_FUNC_STRING_LEN];
    enum EPICS_RECTYPE rtyp;
    int  funcflag;
} param_map[] = {
    {"RESET", EPICS_RECTYPE_BO,    PPOM_BO_RESET},
    {"DATA",  EPICS_RECTYPE_MBBID, PPOM_MBBID_DATA},
    {"DATA",  EPICS_RECTYPE_MBBOD, PPOM_MBBOD_DATA},
};
#define N_PARAM_MAP (sizeof(param_map)/sizeof(struct PARAM_MAP))

typedef struct PPOM_REQUEST
{
    ELLNODE             node;   /* Link List Node */

    PPOM_MODULE          *pPPOMModule;
    dbCommon            *pRecord;

    UINT16		a;
    UINT16		f;

    int                 funcflag; /* read data/write data/read mdid */

    epicsTimeStamp	actTime;
    UINT16		val;
    UINT32	        errCode;
    int                 opDone;
} PPOM_REQUEST;

/* vmsstat_t uses low 28 bits */
#define PPOM_REQUEST_NO_ERR	0
#define PPOM_MODULE_NOT_EXIST	0x10000000
#define PPOM_CAM_INIT_FAIL	0x20000000
#define PPOM_RSTCLR_CAMIO_FAIL	0x30000000
#define PPOM_READ_CAMIO_FAIL	0x40000000
#define PPOM_WRT_CAMIO_FAIL	0x50000000
#define PPOM_CAM_ALLOC_FAIL	0x60000000
#define PPOM_CAM_ADD_FAIL	0x70000000
#define PPOM_CAM_GO_FAIL	0x80000000
#define PPOM_CAM_DEL_FAIL	0x90000000

int PPOMRequestInit(dbCommon * pRecord, struct camacio inout, enum EPICS_RECTYPE rtyp);

UINT32 PPOM_RstClr(PPOM_REQUEST  *pPPOMRequest);
UINT32 PPOM_WriteData(PPOM_REQUEST  *pPPOMRequest);
UINT32 PPOM_ReadData(PPOM_REQUEST  *pPPOMRequest);

#ifdef __cplusplus
}
#endif

#endif

