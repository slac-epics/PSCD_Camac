/***************************************************************************\
 *   $Id: devLDIM.c,v 1.2 2009/04/06 02:54:26 pengs Exp $
 *   File:		devLDIM.c
 *   Author:		Sheng Peng
 *   Email:		pengsh2003@yahoo.com
 *   Phone:		408-660-7762
 *   Company:		RTESYS, Inc.
 *   Date:		03/2007
 *   Version:		1.0
 *
 *   EPICS device support file for LDIM 
 *
\***************************************************************************/
#include "drvPSCDLib.h"
#include "devLDIM.h"

extern int LDIM_DRV_DEBUG;

/******************************************************************************************/
/*********************              implementation              ***************************/
/******************************************************************************************/

/* function prototypes */
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
} LDIM_DEV_SUP_SET;

LDIM_DEV_SUP_SET devMiDLDIM = {6, NULL, NULL, init_mid, NULL, read_mid, NULL};

#if     EPICS_VERSION>=3 && EPICS_REVISION>=14
epicsExportAddress(dset, devMiDLDIM);
#endif

/*******        mbbiDirect record       ******/
static long init_mid(struct mbbiDirectRecord * pmbbid)
{
    int status;

    if(pmbbid->inp.type!=CAMAC_IO)
    {
        recGblRecordError(S_db_badField, (void *)pmbbid, "devMiDLDIM Init_record, Illegal INP");
        pmbbid->pact=TRUE;
        return (S_db_badField);
    }

    status = LDIMRequestInit((dbCommon *)pmbbid, pmbbid->inp.value.camacio, EPICS_RECTYPE_MBBID);

    /* we don't need pmbbid->mask and pmbbid->nobt */
    if(status)
    {
        recGblRecordError(S_db_badField, (void *)pmbbid, "devMiDLDIM Init_record failed");
        pmbbid->pact=TRUE;
        return (S_db_badField);
    }

    return (status);
}

static long read_mid(struct mbbiDirectRecord *pmbbid)
{
    LDIM_REQUEST  *pRequest = (LDIM_REQUEST *)(pmbbid->dpvt);
    int rtn = -1;

    if(!pRequest) return(-1);

    if(!pmbbid->pact)
    {/* pre-process */
        /* Clean up the request */
        epicsTimeGetCurrent(&(pRequest->reqTime));
        pRequest->errCode = LDIM_REQUEST_NO_ERR;
        pRequest->opDone = 0;

        if(epicsMessageQueueTrySend(pRequest->pLDIMModule->msgQId, (void *)&pRequest, sizeof(LDIM_REQUEST *)) == -1)
        {
            recGblSetSevr(pmbbid, READ_ALARM, INVALID_ALARM);
            errlogPrintf("Send Message to LDIM Operation Thread Error [%s]", pmbbid->name);
            rtn = -1;
        }
        else
        {
            pmbbid->pact = TRUE;
            rtn = NO_CONVERT;
        }

    }/* pre-process */
    else
    {/* post-process */
        if( (!pRequest->opDone) || pRequest->errCode )
        {
            recGblSetSevr(pmbbid, READ_ALARM, INVALID_ALARM);
            errlogPrintf("Record [%s] receive error code [0x%08x]!\n", pmbbid->name, pRequest->errCode);
            rtn = -1;
        }
        else
        {
            if(LDIM_DRV_DEBUG)   printf("Record [%s] receives val [0x%04X]!\n", pmbbid->name, pRequest->val);

            if(pmbbid->tse == epicsTimeEventDeviceTime)/* do timestamp by device support */
                pmbbid->time = pRequest->reqTime;
            pmbbid->val = pRequest->val;
            rtn = NO_CONVERT;
        }
    }/* post-process */
    return (rtn);

}

