/***************************************************************************\
 *   $Id: devSAM.c,v 1.2 2009/03/08 08:01:34 pengs Exp $
 *   File:		devSAM.c
 *   Author:		Sheng Peng
 *   Email:		pengsh2003@yahoo.com
 *   Phone:		408-660-7762
 *   Company:		RTESYS, Inc.
 *   Date:		03/2007
 *   Version:		1.0
 *
 *   EPICS device support file for SAM 
 *
\***************************************************************************/
#include "drvPSCDLib.h"
#include "devSAM.h"

extern int SAM_DRV_DEBUG;


/******************************************************************************************/
/*********************              implementation              ***************************/
/******************************************************************************************/

/* function prototypes */
static long init_ai(struct aiRecord *pai);
static long read_ai(struct aiRecord *pai);

static long init_bo(struct boRecord *pbo);
static long write_bo(struct boRecord *pbo);

/* global struct for devSup */
typedef struct {
        long            number;
        DEVSUPFUN       report;
        DEVSUPFUN       init;
        DEVSUPFUN       init_record;
        DEVSUPFUN       get_ioint_info;
        DEVSUPFUN       read_write;
        DEVSUPFUN       special_linconv;
} SAM_DRV_SUP_SET;

SAM_DRV_SUP_SET devAiSAM = {6, NULL, NULL, init_ai, NULL, read_ai, NULL};
SAM_DRV_SUP_SET devBoSAM = {6, NULL, NULL, init_bo, NULL, write_bo, NULL};

#if     EPICS_VERSION>=3 && EPICS_REVISION>=14
epicsExportAddress(dset, devAiSAM);
epicsExportAddress(dset, devBoSAM);
#endif

/*******        ai record       ******/
static long init_ai(struct aiRecord * pai)
{
    int status;

    if(pai->inp.type!=CAMAC_IO)
    {
        recGblRecordError(S_db_badField, (void *)pai, "devAiSAM Init_record, Illegal INP");
        pai->pact=TRUE;
        return (S_db_badField);
    }

    status = SAMRequestInit((dbCommon *)pai, pai->inp.value.camacio, EPICS_RECTYPE_AI);

    if(status)
    {
        recGblRecordError(S_db_badField, (void *)pai, "devAiSAM Init_record failed");
        pai->pact=TRUE;
        return (S_db_badField);
    }

    return (status);
}

static long read_ai(struct aiRecord *pai)
{
    SAM_REQUEST  *pRequest = (SAM_REQUEST *)(pai->dpvt);
    int rtn = -1;

    if(!pRequest) return(-1);

    if(pRequest->funcflag == SAM_AI_FWVER)
    {
        pai->val = pRequest->pSAMModule->fwVer;
        pai->udf = FALSE;
	if(pai->val <= 0.0)
            recGblSetSevr(pai, READ_ALARM, INVALID_ALARM);
        return NO_CONVERT;
    }

    if(!pai->pact)
    {/* pre-process */
        /* Clean up the request */
        epicsTimeGetCurrent(&(pRequest->reqTime));
        pRequest->errCode = SAM_REQUEST_NO_ERR;
        pRequest->opDone = 0;

        if(epicsMessageQueueTrySend(pRequest->pSAMModule->msgQId, (void *)&pRequest, sizeof(SAM_REQUEST *)) == -1)
        {
            recGblSetSevr(pai, READ_ALARM, INVALID_ALARM);
            errlogPrintf("Send Message to SAM Operation Thread Error [%s]", pai->name);
            rtn = -1;
        }
        else
        {
            pai->pact = TRUE;
            rtn = NO_CONVERT;
        }

    }/* pre-process */
    else
    {/* post-process */
        if( (!pRequest->opDone) || pRequest->errCode )
        {
            recGblSetSevr(pai, READ_ALARM, INVALID_ALARM);
            errlogPrintf("Record [%s] receive error code [0x%08x]!\n", pai->name, pRequest->errCode);
            rtn = -1;
        }
        else
        {
            if(SAM_DRV_DEBUG)   printf("Record [%s] receives val [%g]!\n", pai->name, pRequest->val);

            pai->udf = FALSE;
            if(pai->tse == epicsTimeEventDeviceTime)/* do timestamp by device support */
                pai->time = pRequest->reqTime;
            pai->val = pRequest->val;
            rtn = NO_CONVERT;
        }
    }/* post-process */
    return (rtn);

}

/*******        bo record       ******/
static long init_bo(struct boRecord * pbo)
{
    int status;

    if(pbo->out.type!=CAMAC_IO)
    {
        recGblRecordError(S_db_badField, (void *)pbo, "devBoSAM Init_record, Illegal INP");
        pbo->pact=TRUE;
        return (S_db_badField);
    }

    status = SAMRequestInit((dbCommon *)pbo, pbo->out.value.camacio, EPICS_RECTYPE_BO);

    if(status)
    {
        recGblRecordError(S_db_badField, (void *)pbo, "devBoSAM Init_record failed");
        pbo->pact=TRUE;
        return (S_db_badField);
    }

    return NO_CONVERT;
}

static long write_bo(struct boRecord *pbo)
{
    SAM_REQUEST  *pRequest = (SAM_REQUEST *)(pbo->dpvt);
    int rtn = -1;

    if(!pRequest) return(-1);

    if(!pbo->pact)
    {/* pre-process */
        /* Clean up the request */
        epicsTimeGetCurrent(&(pRequest->reqTime));
        pRequest->errCode = SAM_REQUEST_NO_ERR;
        pRequest->opDone = 0;

        if(epicsMessageQueueTrySend(pRequest->pSAMModule->msgQId, (void *)&pRequest, sizeof(SAM_REQUEST *)) == -1)
        {
            recGblSetSevr(pbo, WRITE_ALARM, INVALID_ALARM);
            errlogPrintf("Send Message to SAM Operation Thread Error [%s]", pbo->name);
            rtn = -1;
        }
        else
        {
            pbo->pact = TRUE;
            rtn = NO_CONVERT;
        }

    }/* pre-process */
    else
    {/* post-process */
        if( (!pRequest->opDone) || pRequest->errCode )
        {
            recGblSetSevr(pbo, WRITE_ALARM, INVALID_ALARM);
            errlogPrintf("Record [%s] receive error code [0x%08x]!\n", pbo->name, pRequest->errCode);
            rtn = -1;
        }
        else
        {
            if(SAM_DRV_DEBUG)   printf("Record [%s] succeed!\n", pbo->name);

            rtn = NO_CONVERT;
        }
    }/* post-process */
    return (rtn);

}

