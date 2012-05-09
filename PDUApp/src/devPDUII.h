/***************************************************************************\
 *   $Id: devPDUII.h,v 1.12 2011/02/23 07:57:15 rcs Exp $
 *   File:		devPDUII.h
 *   Author:		Sheng Peng
 *   Email:		pengsh2003@yahoo.com
 *   Phone:		408-660-7762
 *   Company:		RTESYS, Inc.
 *   Date:		03/2010
 *   Version:		1.0
 *
 *   EPICS device support header file for PDU 
 *
\***************************************************************************/

#ifndef _DEV_PDUII_H_
#define _DEV_PDUII_H_

#define PDUII_DRV_VER_STRING	"CAMAC PDUII Driver V1.0"

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
#include <longinRecord.h>
#include <longoutRecord.h>
#include <mbbiRecord.h>
#include <mbboRecord.h>
#include <mbbiDirectRecord.h>
#include <waveformRecord.h>

#else
#error "We need EPICS 3.14 or above to support OSI calls!"
#endif

#include "genType.h"

#include "drvPSCDLib.h"

#include "slc_macros.h"
#include "cam_proto.h"

#include "evrTime.h"
#include "evrPattern.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************************/
/*********************       EPICS device support return        ***************************/
/******************************************************************************************/
#define CONVERT                 (0)
#define NO_CONVERT              (2)
#define MAX_FUNC_STRING_LEN	(40)

#define N_CHNLS_PER_MODU	16
#define N_RULES_PER_CHNL	8	/* this is subject to change, but up to 32 since we use a F to indicate rule number */
/******************************************************************************************/

#define N_USHORTS_MASK	10	/* This has to be (MAX_EVR_MODIFIER-1)*2 */
#if 0	/* defined in evrTime.h */
#define MAX_EVR_MODIFIER  6
typedef epicsUInt32 evrModifier_ta[MAX_EVR_MODIFIER];
#endif

/* This structure defines a individual rule */
typedef struct PDUII_RULE
{
    evrModifier_ta	inclusionMask;
    evrModifier_ta	exclusionMask;
    unsigned long	beamCode;
    UINT32		pttDelay;
} PDUII_RULE;

#define PTT_ENTRY_UNKNOWN	0x80000000 /* For reset and init value, mark as unknown when camgo failed */
#define PTT_ENTRY_RELOADING	0x40000000 /* For 360T, Mark as updating, when camgo returns, clear */

#define CHNL_MODE_TRANSITING	0x80000000 /* Init value, mark as unknown when camgo failed */

#define CHNL_MODE_YY0	0
#define CHNL_MODE_PP0	1
#define CHNL_MODE_YY1	2
#define CHNL_MODE_PP1	3
#define CHNL_MODE_YY2	4
#define CHNL_MODE_PP2	5
#define CHNL_MODE_M36	6
#define CHNL_MODE_RUSE	7

/* PDUII module,  b,c,n define a unique module */
typedef struct PDUII_MODULE
{
    ELLNODE                     node;   /* Link List Node */

    UINT16			b;	/* branch, always 0 from record, but then readout from PSCD */
    UINT16			c;	/* crate */
    UINT16			n;	/* node = slot */

    epicsMessageQueueId		msgQId;	/* One Q per module */
    epicsThreadId               opTaskId;
    epicsMutexId		lockRule;/* lock the rules */
    epicsMutexId		lockModule;/* lock the module and pttCache, for reset */

    PDUII_RULE			rules[N_CHNLS_PER_MODU][N_RULES_PER_CHNL];

    UINT32			chnlModeRbk[N_CHNLS_PER_MODU];
    UINT32			chnlModeSet[N_CHNLS_PER_MODU];

    UINT32			pttCache[N_CHNLS_PER_MODU*256];

    UINT32			errorCount;	/* number of channels/entries missed */
/*    epicsTimeStamp		lastReadTime;
    UINT16			data[2];
    UINT32			lastErrCode;

    char			camacPreMsg[256];*/	/* Could be done in init second pass */
} PDUII_MODULE;

/******************************************************************************************/
/*********************       Record type we support             ***************************/
/******************************************************************************************/
typedef enum EPICS_RECTYPE
{
    EPICS_RECTYPE_NONE,
    EPICS_RECTYPE_BO,
    EPICS_RECTYPE_MBBID,
    EPICS_RECTYPE_MBBI,
    EPICS_RECTYPE_MBBO,
    EPICS_RECTYPE_LI,
    EPICS_RECTYPE_LO,
    EPICS_RECTYPE_WF
}   E_EPICS_RECTYPE;

/* define function flags */
typedef enum {
    PDUII_BO_RESET,
    PDUII_BO_UBPLN,
    PDUII_BO_SEQR,
    PDUII_MBBID_STATUS,

    /* Need channel number */
    PDUII_MBBI_MODEGET,
    PDUII_MBBO_MODESET,

    /* Need channel number and location number */
    PDUII_LI_PTTGET,
    PDUII_LO_PTTSET,

    /* Need channel number and rule number */
    PDUII_LO_DELAY,
    PDUII_WF_RULE
} PDUIIFUNC;

static struct PARAM_MAP
{
    char param[MAX_FUNC_STRING_LEN];
    enum EPICS_RECTYPE rtyp;
    int  funcflag;
} param_map[] = {
    {"RESET",		EPICS_RECTYPE_BO,    PDUII_BO_RESET},
    {"UBPLN",		EPICS_RECTYPE_BO,    PDUII_BO_UBPLN},
    {"SEQR",		EPICS_RECTYPE_BO,    PDUII_BO_SEQR},
    {"STATUS",		EPICS_RECTYPE_MBBID, PDUII_MBBID_STATUS},

    {"MODEGET",		EPICS_RECTYPE_MBBI,  PDUII_MBBI_MODEGET},
    {"MODESET",	        EPICS_RECTYPE_MBBO,  PDUII_MBBO_MODESET},

    {"PTTGET",	        EPICS_RECTYPE_LI,    PDUII_LI_PTTGET},
    {"PTTSET",	        EPICS_RECTYPE_LO,    PDUII_LO_PTTSET},

    {"DELAY",		EPICS_RECTYPE_LO,    PDUII_LO_DELAY},
    {"RULE",		EPICS_RECTYPE_WF,    PDUII_WF_RULE}
};

#define N_PARAM_MAP (sizeof(param_map)/sizeof(struct PARAM_MAP))

typedef struct PDUII_REQUEST
{
    ELLNODE             node;   /* Link List Node */

    PDUII_MODULE        *pPDUIIModule;
    dbCommon            *pRecord;

    UINT16		a;	/* always indicates channel number */
    UINT16		f;	/* not really used */

    UINT16		extra;	/* indicates rule number or PTT location */

    int                 funcflag;

    epicsTimeStamp	actTime;
    UINT32		val;
    UINT32	        errCode;
    int                 opDone;
} PDUII_REQUEST;

/* vmsstat_t uses low 28 bits */
#define PDUII_REQUEST_NO_ERR	0
#define PDUII_MODULE_NOT_EXIST	0x10000000
#define PDUII_CAM_INIT_FAIL	0x20000000
#define PDUII_RST_CAMIO_FAIL	0x30000000
#define PDUII_ENO_CAMIO_FAIL	0x40000000
#define PDUII_ENS_CAMIO_FAIL	0x50000000
#define PDUII_STS_CAMIO_FAIL	0x60000000
#define PDUII_READ_CAMIO_FAIL	0x70000000
#define PDUII_WRT_CAMIO_FAIL	0x80000000
#define PDUII_CAM_ALLOC_FAIL	0x90000000
#define PDUII_CAM_ADD_FAIL	0xA0000000
#define PDUII_CAM_GO_FAIL	0xB0000000
#define PDUII_CAM_DEL_FAIL	0xC0000000

typedef struct STAS_DAT
{
    UINT32 stat;
    UINT32 data;
} STAS_DAT;

typedef struct STAS_SDAT
{
    UINT32 stat;
    UINT16 sdata;
} STAS_SDAT;


int PDUII360TaskStart();
int PDUIIRequestInit(dbCommon * pRecord, struct camacio inout, enum EPICS_RECTYPE rtyp);

UINT32 PDUII_Reset(PDUII_MODULE *pPDUIIModule);
UINT32 PDUII_UpperBplnEnDis(PDUII_MODULE *pPDUIIModule, UINT32 enable);
UINT32 PDUII_SeqrEnDis(PDUII_MODULE *pPDUIIModule, UINT32 enable);
UINT32 PDUII_Status(PDUII_MODULE *pPDUIIModule, UINT32 *pStatus);

UINT32 PDUII_ModeGet(PDUII_REQUEST  *pPDUIIRequest);
UINT32 PDUII_ModeSet(PDUII_REQUEST  *pPDUIIRequest);

UINT32 PDUII_PTTGet(PDUII_REQUEST  *pPDUIIRequest);
UINT32 PDUII_PTTSet(PDUII_REQUEST  *pPDUIIRequest);

#ifdef __cplusplus
}
#endif

#endif

