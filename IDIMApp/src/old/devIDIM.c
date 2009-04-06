/***************************************************************************\
 *   $Id: devIDIM.c,v 1.1.1.1 2009/02/12 20:08:59 pengs Exp $
 *   File:		devIDIM.c
 *   Author:		Sheng Peng
 *   Email:		pengsh2003@yahoo.com
 *   Phone:		408-660-7762
 *   Company:		RTESYS, Inc.
 *   Date:		03/2007
 *   Version:		1.0
 *
 *   EPICS device support file for IDIM 
 *
\***************************************************************************/
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "drvPSCDLib.h"

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

#include "DIMtest.h"

extern struct PSCD_CARD pscd_card;
extern int PSCD_DRV_DEBUG;

extern DIM_TS dim_s;
extern void DIMtest (void);
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
    EPICS_RECTYPE_WF
}   E_EPICS_RECTYPE;
/******************************************************************************************/

/*      define function flags   */
typedef enum {
        IDIM_MID_DIMLO,
        IDIM_MID_DIMHI,
}       IDIMFUNC;


/*      define parameter check for convinence */
#define CHECK_MIDPARM(PARM,VAL)\
	if (!strncmp(pmbbid->inp.value.vmeio.parm,(PARM),strlen((PARM)))) {\
	          pmbbid->dpvt=(void *)VAL;\
	          return (0);\
	  }


/******************************************************************************************/
/*********************              implementation              ***************************/
/******************************************************************************************/

/* function prototypes */
static long init(int pass);
static long init_mid(struct mbbiDirectRecord *pmbbid);
static long read_mid(struct mbbiDirectRecord *pmbbid);

/* global struct for devSup */
typedef struct {
        long            number;
        DEVSUPFUN       report;
        DEVSUPFUN       init;
        DEVSUPFUN       init_record;
        DEVSUPFUN       get_ioint_info;
        DEVSUPFUN       read_write;
        DEVSUPFUN       special_linconv;
} IDIM_DEV_SUP_SET;

IDIM_DEV_SUP_SET devMiDIDIM = {6, NULL, init, init_mid, NULL, read_mid, NULL};

#if     EPICS_VERSION>=3 && EPICS_REVISION>=14
epicsExportAddress(dset, devMiDIDIM);
#endif

static long init(int pass)
{
    if(pass) return 0;

    epicsThreadMustCreate("DIMtest", epicsThreadPriorityHigh, 20480, DIMtest, (void *)0);
    return 0;
}


/*******        mbbiDirect record       ******/
static long init_mid(struct mbbiDirectRecord * pmbbid)
{
    int crate;
    int module;

    if(pmbbid->inp.type!=VME_IO)
    {
        recGblRecordError(S_db_badField, (void *)pmbbid, "devMiDIDIM Init_record, Illegal INP");
        pmbbid->pact=TRUE;
        return (S_db_badField);
    }

    /* we don't need pmbbid->mask and pmbbid->nobt */
    crate = pmbbid->inp.value.vmeio.card;
    module = pmbbid->inp.value.vmeio.signal;

    if(pscd_card.fwVer == 0)
    {
        recGblRecordError(S_db_badField, (void *)pmbbid, "devMiDIDIM Init_record, no IDIM");
        pmbbid->pact=TRUE;
        return (S_db_badField);
    }

    CHECK_MIDPARM("DIMLO", IDIM_MID_DIMLO)
    CHECK_MIDPARM("DIMHI", IDIM_MID_DIMHI)
    /* reach here, bad parm */
    recGblRecordError(S_db_badField, (void *)pmbbid, "devMiDIDIM Init_record, bad parm");
    pmbbid->pact=TRUE;
    return (S_db_badField);

    return (0);
}

static long read_mid(struct mbbiDirectRecord *pmbbid)
{
    switch ((int)pmbbid->dpvt)
    {
    case IDIM_MID_DIMLO:
        pmbbid->rval = (dim_s.data)&0xFFFF;
        break;
    case IDIM_MID_DIMHI:
        pmbbid->rval = (dim_s.data)>>16;
        break;
    default:
        return -1;
    }

    if(dim_s.stat != 0x52530000)
    {
        recGblSetSevr(pmbbid,READ_ALARM,INVALID_ALARM);
        return 0;
    }
    else
        return 0;
}


