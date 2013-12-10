/***************************************************************************\
 *   $Id: devDAC.c,v 1.4 2011/02/23 06:48:33 rcs Exp $
 *   File:		devDAC.c
 *   Author:		Sheng Peng
 *   Email:		pengsh2003@yahoo.com
 *   Phone:		408-660-7762
 *   Company:		RTESYS, Inc.
 *   Date:		03/2007
 *   Version:		1.0
 *
 *   EPICS device support file for DAC 
 *
\***************************************************************************/
#include "drvPSCDLib.h"
#include "devDAC.h"

extern int DAC_DRV_DEBUG;


/******************************************************************************************/
/*********************              implementation              ***************************/
/******************************************************************************************/

/* function prototypes */
static long init_ai(struct aiRecord *pai);
static long read_ai(struct aiRecord *pai);
static long ai_lincvt(struct aiRecord * pai, int after);

static long init_ao(struct aoRecord *pao);
static long write_ao(struct aoRecord *pao);
static long ao_lincvt(struct aoRecord * pao, int after);

static long init_li(struct longinRecord *pli);
static long read_li(struct longinRecord *pli);

/* global struct for devSup */
typedef struct {
        long            number;
        DEVSUPFUN       report;
        DEVSUPFUN       init;
        DEVSUPFUN       init_record;
        DEVSUPFUN       get_ioint_info;
        DEVSUPFUN       read_write;
        DEVSUPFUN       special_linconv;
} DAC_DRV_SUP_SET;

DAC_DRV_SUP_SET devAiDAC = {6, NULL, NULL, init_ai, NULL, read_ai, ai_lincvt};
DAC_DRV_SUP_SET devAoDAC = {6, NULL, NULL, init_ao, NULL, write_ao, ao_lincvt};
DAC_DRV_SUP_SET devLiDAC = {6, NULL, NULL, init_li, NULL, read_li, NULL};

#if     EPICS_VERSION>=3 && EPICS_REVISION>=14
epicsExportAddress(dset, devAiDAC);
epicsExportAddress(dset, devAoDAC);
epicsExportAddress(dset, devLiDAC);
#endif

/*******        ai record       ******/
static long init_ai(struct aiRecord * pai)
{
    int status;

    if(pai->inp.type!=CAMAC_IO)
    {
        recGblRecordError(S_db_badField, (void *)pai, "devAiDAC Init_record, Illegal INP");
        pai->pact=TRUE;
        return (S_db_badField);
    }

    status = DACRequestInit((dbCommon *)pai, pai->inp.value.camacio, EPICS_RECTYPE_AI);

    if(status)
    {
        recGblRecordError(S_db_badField, (void *)pai, "devAiDAC Init_record failed");
        pai->pact=TRUE;
        return (S_db_badField);
    }

    pai->eslo = (pai->eguf - pai->egul)/(float)0x10000;
    pai->roff = 0x0;

    return (status);
}

static long read_ai(struct aiRecord *pai)
{
    DAC_REQUEST  *pRequest = (DAC_REQUEST *)(pai->dpvt);
    int rtn = -1;

    if(!pRequest) return(-1);

    if(!pai->pact)
    {/* pre-process */
        /* Clean up the request */
        pRequest->errCode = DAC_REQUEST_NO_ERR;
        pRequest->opDone = 0;

        if(epicsMessageQueueTrySend(pRequest->pDACModule->msgQId, (void *)&pRequest, sizeof(DAC_REQUEST *)) == -1)
        {
            recGblSetSevr(pai, READ_ALARM, INVALID_ALARM);
            errlogPrintf("Send Message to DAC Operation Thread Error [%s]", pai->name);
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
            if(DAC_DRV_DEBUG)   printf("Record [%s] error %s!\n", pai->name, cammsg(pRequest->errCode));
            rtn = -1;
        }
        else
        {
            if(DAC_DRV_DEBUG)   printf("Record [%s] receives val [0x%X]!\n", pai->name, pRequest->val);

            if(pai->tse == epicsTimeEventDeviceTime)/* do timestamp by device support */
                pai->time = pRequest->actTime;
            pai->rval = pRequest->val;
            rtn = CONVERT;
        }
    }/* post-process */
    return (rtn);

}

static long ai_lincvt(struct aiRecord * pai, int after)
{
    if(!after) return(0);
    /* set linear conversion slope*/
    pai->eslo = (pai->eguf - pai->egul)/(float)0x10000;
    pai->roff = 0x0;
    return(0);
}

/*******        ao record       ******/
static long init_ao(struct aoRecord * pao)
{
    DAC_REQUEST  *pRequest;
    int status;

    if(pao->out.type!=CAMAC_IO)
    {
        recGblRecordError(S_db_badField, (void *)pao, "devAoDAC Init_record, Illegal INP");
        pao->pact=TRUE;
        return (S_db_badField);
    }

    status = DACRequestInit((dbCommon *)pao, pao->out.value.camacio, EPICS_RECTYPE_AO);

    if(status)
    {
        recGblRecordError(S_db_badField, (void *)pao, "devAoDAC Init_record failed");
        pao->pact=TRUE;
        return (S_db_badField);
    }

    pao->eslo = (pao->eguf - pao->egul)/(float)0x10000;
    pao->roff = 0x0;

    pRequest = (DAC_REQUEST *)(pao->dpvt);
    if(pRequest)
    {
        switch(pRequest->funcflag)
        {/* check funcflag */
            case DAC_AO_DATA: /* F16 An */
                if(DAC_ReadData(pRequest))
                    return -1;
		else
                {
                    pao->rval = pRequest->val;
     		    pao->udf = FALSE;
                    pao->stat = pao->sevr = NO_ALARM;
		}
                break;
	    default:
		break;
        }/* check funcflag */
    }

    return CONVERT;
}

static long write_ao(struct aoRecord *pao)
{
    DAC_REQUEST  *pRequest = (DAC_REQUEST *)(pao->dpvt);
    int rtn = -1;

    if(!pRequest) return(-1);

    if(!pao->pact)
    {/* pre-process */
        /* Clean up the request */
        pRequest->errCode = DAC_REQUEST_NO_ERR;
        pRequest->opDone = 0;
	if(pao->rval <= 0)
            pRequest->val = 0;
	else if(pao->rval >= 0xFFFF)
            pRequest->val = 0xFFFF;
	else
            pRequest->val = pao->rval;

        if(epicsMessageQueueTrySend(pRequest->pDACModule->msgQId, (void *)&pRequest, sizeof(DAC_REQUEST *)) == -1)
        {
            recGblSetSevr(pao, WRITE_ALARM, INVALID_ALARM);
            errlogPrintf("Send Message to DAC Operation Thread Error [%s]", pao->name);
            rtn = -1;
        }
        else
        {
            pao->pact = TRUE;
            rtn = NO_CONVERT;
        }

    }/* pre-process */
    else
    {/* post-process */
        if( (!pRequest->opDone) || pRequest->errCode )
        {
            recGblSetSevr(pao, WRITE_ALARM, INVALID_ALARM);
            if(DAC_DRV_DEBUG)   printf("Record [%s] error %s!\n", pao->name, cammsg(pRequest->errCode));
            rtn = -1;
        }
        else
        {
            if(DAC_DRV_DEBUG)   printf("Record [%s] succeed!\n", pao->name);

            rtn = NO_CONVERT;
        }
    }/* post-process */
    return (rtn);

}

static long ao_lincvt(struct aoRecord * pao, int after)
{
    if(!after) return(0);
    /* set linear conversion slope*/
    pao->eslo = (pao->eguf - pao->egul)/(float)0x10000;
    pao->roff = 0x0;
    return(0);
}

/*******        longin record       ******/
static long init_li(struct longinRecord * pli)
{
    int status;

    if(pli->inp.type!=CAMAC_IO)
    {
        recGblRecordError(S_db_badField, (void *)pli, "devLiDAC Init_record, Illegal INP");
        pli->pact=TRUE;
        return (S_db_badField);
    }

    status = DACRequestInit((dbCommon *)pli, pli->inp.value.camacio, EPICS_RECTYPE_LI);

    if(status)
    {
        recGblRecordError(S_db_badField, (void *)pli, "devLiDAC Init_record failed");
        pli->pact=TRUE;
        return (S_db_badField);
    }

    return (status);
}

static long read_li(struct longinRecord *pli)
{
    DAC_REQUEST  *pRequest = (DAC_REQUEST *)(pli->dpvt);
    int rtn = -1;

    if(!pRequest) return(-1);

    if(!pli->pact)
    {/* pre-process */
        /* Clean up the request */
        pRequest->errCode = DAC_REQUEST_NO_ERR;
        pRequest->opDone = 0;

        if(epicsMessageQueueTrySend(pRequest->pDACModule->msgQId, (void *)&pRequest, sizeof(DAC_REQUEST *)) == -1)
        {
            recGblSetSevr(pli, READ_ALARM, INVALID_ALARM);
            errlogPrintf("Send Message to DAC Operation Thread Error [%s]", pli->name);
            rtn = -1;
        }
        else
        {
            pli->pact = TRUE;
            rtn = NO_CONVERT;
        }

    }/* pre-process */
    else
    {/* post-process */
        if( (!pRequest->opDone) || pRequest->errCode )
        {
            recGblSetSevr(pli, READ_ALARM, INVALID_ALARM);
            if(DAC_DRV_DEBUG)   printf("Record [%s] error %s!\n", pli->name, cammsg(pRequest->errCode));
            rtn = -1;
        }
        else
        {
            if(DAC_DRV_DEBUG)   printf("Record [%s] receives val [%d]!\n", pli->name, pRequest->val);

            if(pli->tse == epicsTimeEventDeviceTime)/* do timestamp by device support */
                pli->time = pRequest->actTime;
            pli->val = pRequest->val;
            rtn = NO_CONVERT;
        }
    }/* post-process */
    return (rtn);

}

