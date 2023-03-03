/***************************************************************************\
 *   $Id: devPDUII.c,v 1.8 2013/12/10 18:22:38 sonya Exp $
 *   File:		devPDUII.c
 *   Author:		Sheng Peng
 *   Email:		pengsh2003@yahoo.com
 *   Phone:		408-660-7762
 *   Company:		RTESYS, Inc.
 *   Date:		09/2009
 *   Version:		1.0
 *
 *   EPICS device support file for PDUII 
 *
\***************************************************************************/
#include "drvPSCDLib.h"
#include "devPDUII.h"

extern int PDUII_DRV_DEBUG;

/******************************************************************************************/
/*********************              implementation              ***************************/
/******************************************************************************************/

/* function prototypes */
static long init(int pass);

static long init_bo(struct boRecord *pbo);
static long write_bo(struct boRecord *pbo);

static long init_mid(struct mbbiDirectRecord *pmbbid);
static long read_mid(struct mbbiDirectRecord *pmbbid);

static long init_mbbi(struct mbbiRecord *pmbbi);
static long read_mbbi(struct mbbiRecord *pmbbi);

static long init_mbbo(struct mbboRecord *pmbbo);
static long write_mbbo(struct mbboRecord *pmbbo);

static long init_li(struct longinRecord *pli);
static long read_li(struct longinRecord *pli);

static long init_lo(struct longoutRecord *plo);
static long write_lo(struct longoutRecord *plo);

static long init_wf(struct waveformRecord *pwf);
static long write_wf(struct waveformRecord *pwf);

/* global struct for devSup */
typedef struct {
        long            number;
        DEVSUPFUN       report;
        DEVSUPFUN       init;
        DEVSUPFUN       init_record;
        DEVSUPFUN       get_ioint_info;
        DEVSUPFUN       read_write;
        DEVSUPFUN       special_linconv;
} PDUII_DEV_SUP_SET;

PDUII_DEV_SUP_SET devBoPDUII =  {6, NULL, init, init_bo, NULL, write_bo, NULL};
PDUII_DEV_SUP_SET devMiDPDUII = {6, NULL, NULL, init_mid, NULL, read_mid, NULL};
PDUII_DEV_SUP_SET devMiPDUII = {6, NULL, NULL, init_mbbi, NULL, read_mbbi, NULL};
PDUII_DEV_SUP_SET devMoPDUII = {6, NULL, NULL, init_mbbo, NULL, write_mbbo, NULL};
PDUII_DEV_SUP_SET devLiPDUII =  {6, NULL, NULL, init_li, NULL, read_li, NULL};
PDUII_DEV_SUP_SET devLoPDUII =  {6, NULL, NULL, init_lo, NULL, write_lo, NULL};
PDUII_DEV_SUP_SET devWfPDUII = {6, NULL, NULL, init_wf, NULL, write_wf, NULL};

#if EPICS_VERSION>3 || (EPICS_VERSION==3 && EPICS_REVISION>=14)
epicsExportAddress(dset, devBoPDUII);
epicsExportAddress(dset, devMiDPDUII);
epicsExportAddress(dset, devMiPDUII);
epicsExportAddress(dset, devMoPDUII);
epicsExportAddress(dset, devLiPDUII);
epicsExportAddress(dset, devLoPDUII);
epicsExportAddress(dset, devWfPDUII);
#endif

static long init(int final)
{
    /* We only do things at the final round since then the PDU module linked list is ready */
    if(!final) return 0;
    else PDUII360TaskStart();
    return 0;
}

/*******        bo record       ******/
static long init_bo(struct boRecord * pbo)
{
    int status;

    if(pbo->out.type!=CAMAC_IO)
    {
        recGblRecordError(S_db_badField, (void *)pbo, "devBoPDUII Init_record, Illegal OUT");
        pbo->pact=TRUE;
        return (S_db_badField);
    }

    status = PDUIIRequestInit((dbCommon *)pbo, pbo->out.value.camacio, EPICS_RECTYPE_BO);

    if(status)
    {
        recGblRecordError(S_db_badField, (void *)pbo, "devBoPDUII Init_record failed");
        pbo->pact=TRUE;
        return (S_db_badField);
    }

    /* TODO, for UBPLN and SEQR, readback here, no, let autoSaveRestore do it */
    /* might not necesary, we depend on autoSaveRestore anyway */
    return (status);
}

static long write_bo(struct boRecord *pbo)
{
    PDUII_REQUEST  *pRequest = (PDUII_REQUEST *)(pbo->dpvt);
    int rtn = -1;

    if(!pRequest) return(-1);

    /* So far, all BO are async process */
    if(!pbo->pact)
    {/* pre-process */
        /* Clean up the request */
        epicsTimeGetCurrent(&(pRequest->actTime));
        pRequest->errCode = PDUII_REQUEST_NO_ERR;
        pRequest->opDone = 0;

        pRequest->val = pbo->val;

        if(epicsMessageQueueTrySend(pRequest->pPDUIIModule->msgQId, (void *)&pRequest, sizeof(PDUII_REQUEST *)) == -1)
        {
            recGblSetSevr(pbo, WRITE_ALARM, INVALID_ALARM);
            errlogPrintf("Send Message to PDUII Operation Thread Error [%s]", pbo->name);
            rtn = -1;
        }
        else
        {
            pbo->pact = TRUE;
            rtn = 0;
        }

    }/* pre-process */
    else
    {/* post-process */
        if( (!pRequest->opDone) || pRequest->errCode )
        {
            recGblSetSevr(pbo, WRITE_ALARM, INVALID_ALARM);
            if(PDUII_DRV_DEBUG)   printf("Record [%s] error %s!\n", pbo->name, cammsg(pRequest->errCode));
            rtn = -1;
        }
        else
        {
            if(PDUII_DRV_DEBUG > 1)   printf("Record [%s] succeed!\n", pbo->name);

            if(pbo->tse == epicsTimeEventDeviceTime)/* do timestamp by device support */
                pbo->time = pRequest->actTime;
            rtn = NO_CONVERT;
        }
    }/* post-process */
    return (rtn);

}

/*******        mbbiDirect record       ******/
static long init_mid(struct mbbiDirectRecord * pmbbid)
{
    int status;

    if(pmbbid->inp.type!=CAMAC_IO)
    {
        recGblRecordError(S_db_badField, (void *)pmbbid, "devMiDPDUII Init_record, Illegal INP");
        pmbbid->pact=TRUE;
        return (S_db_badField);
    }

    status = PDUIIRequestInit((dbCommon *)pmbbid, pmbbid->inp.value.camacio, EPICS_RECTYPE_MBBID);

    /* we don't need pmbbid->mask and pmbbid->nobt, record sets it anyway */
    if(status)
    {
        recGblRecordError(S_db_badField, (void *)pmbbid, "devMiDPDUII Init_record failed");
        pmbbid->pact=TRUE;
        return (S_db_badField);
    }

    return (status);
}

static long read_mid(struct mbbiDirectRecord *pmbbid)
{
    PDUII_REQUEST  *pRequest = (PDUII_REQUEST *)(pmbbid->dpvt);
    int rtn = -1;

    if(!pRequest) return(-1);

    /* So far, all MbbiDirect are async process */
    if(!pmbbid->pact)
    {/* pre-process */
        /* Clean up the request */
        epicsTimeGetCurrent(&(pRequest->actTime));
        pRequest->errCode = PDUII_REQUEST_NO_ERR;
        pRequest->opDone = 0;

        if(epicsMessageQueueTrySend(pRequest->pPDUIIModule->msgQId, (void *)&pRequest, sizeof(PDUII_REQUEST *)) == -1)
        {
            recGblSetSevr(pmbbid, READ_ALARM, INVALID_ALARM);
            errlogPrintf("Send Message to PDUII Operation Thread Error [%s]", pmbbid->name);
            rtn = -1;
        }
        else
        {
            pmbbid->pact = TRUE;
            rtn = 0;
        }

    }/* pre-process */
    else
    {/* post-process */
        if( (!pRequest->opDone) || pRequest->errCode )
        {
            recGblSetSevr(pmbbid, READ_ALARM, INVALID_ALARM);
            if(PDUII_DRV_DEBUG)   printf("Record [%s] error %s!\n", pmbbid->name, cammsg(pRequest->errCode));
            rtn = -1;
        }
        else
        {
            if(PDUII_DRV_DEBUG > 1)   printf("Record [%s] receives val [0x%04X]!\n", pmbbid->name, pRequest->val);

            if(pmbbid->tse == epicsTimeEventDeviceTime)/* do timestamp by device support */
                pmbbid->time = pRequest->actTime;
            pmbbid->val = pRequest->val;
            rtn = NO_CONVERT;
        }
    }/* post-process */
    return (rtn);

}

/*******        mbbi record       ******/
static long init_mbbi(struct mbbiRecord * pmbbi)
{
    int status;

    if(pmbbi->inp.type!=CAMAC_IO)
    {
        recGblRecordError(S_db_badField, (void *)pmbbi, "devMiPDUII Init_record, Illegal INP");
        pmbbi->pact=TRUE;
        return (S_db_badField);
    }

    status = PDUIIRequestInit((dbCommon *)pmbbi, pmbbi->inp.value.camacio, EPICS_RECTYPE_MBBI);

    /* we don't need pmbbi->mask and pmbbi->nobt, record sets it anyway */
    if(status)
    {
        recGblRecordError(S_db_badField, (void *)pmbbi, "devMiPDUII Init_record failed");
        pmbbi->pact=TRUE;
        return (S_db_badField);
    }

    return (status);
}

static long read_mbbi(struct mbbiRecord *pmbbi)
{
    PDUII_REQUEST  *pRequest = (PDUII_REQUEST *)(pmbbi->dpvt);
    int rtn = -1;

    if(!pRequest) return(-1);

    /* So far, all Mbbi are async process */
    if(!pmbbi->pact)
    {/* pre-process */
        /* Clean up the request */
        epicsTimeGetCurrent(&(pRequest->actTime));
        pRequest->errCode = PDUII_REQUEST_NO_ERR;
        pRequest->opDone = 0;

        if(epicsMessageQueueTrySend(pRequest->pPDUIIModule->msgQId, (void *)&pRequest, sizeof(PDUII_REQUEST *)) == -1)
        {
            recGblSetSevr(pmbbi, READ_ALARM, INVALID_ALARM);
            errlogPrintf("Send Message to PDUII Operation Thread Error [%s]", pmbbi->name);
            rtn = -1;
        }
        else
        {
            pmbbi->pact = TRUE;
            rtn = 0;
        }

    }/* pre-process */
    else
    {/* post-process */
        if( (!pRequest->opDone) || pRequest->errCode )
        {
            recGblSetSevr(pmbbi, READ_ALARM, INVALID_ALARM);
            if(PDUII_DRV_DEBUG)   printf("Record [%s] error %s!\n", pmbbi->name, cammsg(pRequest->errCode));
            rtn = -1;
        }
        else
        {
            if(PDUII_DRV_DEBUG > 1)   printf("Record [%s] receives val [0x%04X]!\n", pmbbi->name, pRequest->val);

            if(pmbbi->tse == epicsTimeEventDeviceTime)/* do timestamp by device support */
                pmbbi->time = pRequest->actTime;
            pmbbi->rval = pRequest->val;
            rtn = CONVERT;
        }
    }/* post-process */
    return (rtn);

}
/*******        mbbo record       ******/
static long init_mbbo(struct mbboRecord * pmbbo)
{
    int status;
    /*PDUII_REQUEST  *pRequest;*/

    if(pmbbo->out.type!=CAMAC_IO)
    {
        recGblRecordError(S_db_badField, (void *)pmbbo, "devMoPDUII Init_record, Illegal OUT");
        pmbbo->pact=TRUE;
        return (S_db_badField);
    }

    status = PDUIIRequestInit((dbCommon *)pmbbo, pmbbo->out.value.camacio, EPICS_RECTYPE_MBBO);

    /* we don't need pmbbo->mask and pmbbo->nobt */
    if(status)
    {
        recGblRecordError(S_db_badField, (void *)pmbbo, "devMoPDUII Init_record failed");
        pmbbo->pact=TRUE;
        return (S_db_badField);
    }

    /* TODO, do we want to read MODE here? no, let autoSaveRestore do it */
    /* I don't think we should shft mask here */

    return NO_CONVERT;
}

static long write_mbbo(struct mbboRecord *pmbbo)
{
    PDUII_REQUEST  *pRequest = (PDUII_REQUEST *)(pmbbo->dpvt);
    int rtn = -1;

    if(!pRequest) return(-1);

    /* So far, all Mbbo are async process */
    if(!pmbbo->pact)
    {/* pre-process */
        /* Clean up the request */
        epicsTimeGetCurrent(&(pRequest->actTime));
        pRequest->errCode = PDUII_REQUEST_NO_ERR;
        pRequest->opDone = 0;

        pRequest->val = pmbbo->rval;

        if(epicsMessageQueueTrySend(pRequest->pPDUIIModule->msgQId, (void *)&pRequest, sizeof(PDUII_REQUEST *)) == -1)
        {
            recGblSetSevr(pmbbo, WRITE_ALARM, INVALID_ALARM);
            errlogPrintf("Send Message to PDUII Operation Thread Error [%s]", pmbbo->name);
            rtn = -1;
        }
        else
        {
            pmbbo->pact = TRUE;
            rtn = 0;
        }

    }/* pre-process */
    else
    {/* post-process */
        if( (!pRequest->opDone) || pRequest->errCode )
        {
            recGblSetSevr(pmbbo, WRITE_ALARM, INVALID_ALARM);
            if(PDUII_DRV_DEBUG)   printf("Record [%s] error %s!\n", pmbbo->name, cammsg(pRequest->errCode));
            rtn = -1;
        }
        else
        {
            if(PDUII_DRV_DEBUG > 1)   printf("Record [%s] succeed!\n", pmbbo->name);

            if(pmbbo->tse == epicsTimeEventDeviceTime)/* do timestamp by device support */
                pmbbo->time = pRequest->actTime;
            rtn = NO_CONVERT;
        }
    }/* post-process */
    return (rtn);

}


/*******        longin record       ******/
static long init_li(struct longinRecord * pli)
{
    int status;

    if(pli->inp.type!=CAMAC_IO)
    {
        recGblRecordError(S_db_badField, (void *)pli, "devLiPDUII Init_record, Illegal INP");
        pli->pact=TRUE;
        return (S_db_badField);
    }

    status = PDUIIRequestInit((dbCommon *)pli, pli->inp.value.camacio, EPICS_RECTYPE_LI);

    if(status)
    {
        recGblRecordError(S_db_badField, (void *)pli, "devLiPDUII Init_record failed");
        pli->pact=TRUE;
        return (S_db_badField);
    }

    return (status);
}

static long read_li(struct longinRecord *pli)
{
    PDUII_REQUEST  *pRequest = (PDUII_REQUEST *)(pli->dpvt);
    int rtn = -1;

    if(!pRequest) return(-1);

    /* So far, all longin are async process */
    if(!pli->pact)
    {/* pre-process */
        /* Clean up the request */
        epicsTimeGetCurrent(&(pRequest->actTime));
        pRequest->errCode = PDUII_REQUEST_NO_ERR;
        pRequest->opDone = 0;

        if(epicsMessageQueueTrySend(pRequest->pPDUIIModule->msgQId, (void *)&pRequest, sizeof(PDUII_REQUEST *)) == -1)
        {
            recGblSetSevr(pli, READ_ALARM, INVALID_ALARM);
            errlogPrintf("Send Message to PDUII Operation Thread Error [%s]", pli->name);
            rtn = -1;
        }
        else
        {
            pli->pact = TRUE;
            rtn = 0;
        }

    }/* pre-process */
    else
    {/* post-process */
        if( (!pRequest->opDone) || pRequest->errCode )
        {
            recGblSetSevr(pli, READ_ALARM, INVALID_ALARM);
            if(PDUII_DRV_DEBUG)   printf("Record [%s] error %s!\n", pli->name, cammsg(pRequest->errCode));
            rtn = -1;
        }
        else
        {
            if(PDUII_DRV_DEBUG > 1)   printf("Record [%s] receives val [0x%04X]!\n", pli->name, pRequest->val);

            if(pli->tse == epicsTimeEventDeviceTime)/* do timestamp by device support */
                pli->time = pRequest->actTime;
            pli->val = pRequest->val;
            rtn = 0;
        }
    }/* post-process */
    return (rtn);

}

/*******        longout record       ******/
static long init_lo(struct longoutRecord * plo)
{
    int status;
    /*PDUII_REQUEST  *pRequest;*/

    if(plo->out.type!=CAMAC_IO)
    {
        recGblRecordError(S_db_badField, (void *)plo, "devLoPDUII Init_record, Illegal OUT");
        plo->pact=TRUE;
        return (S_db_badField);
    }

    status = PDUIIRequestInit((dbCommon *)plo, plo->out.value.camacio, EPICS_RECTYPE_LO);

    if(status)
    {
        recGblRecordError(S_db_badField, (void *)plo, "devLoPDUII Init_record failed");
        plo->pact=TRUE;
        return (S_db_badField);
    }

    /* TODO, do we want to read PTT entry here, no, let autoSaveRestore do it */

    return 0;
}

static long write_lo(struct longoutRecord *plo)
{
    PDUII_REQUEST  *pRequest = (PDUII_REQUEST *)(plo->dpvt);
    int rtn = -1;

    if(!pRequest) return(-1);

    if(pRequest->funcflag == PDUII_LO_DELAY)
    {
        pRequest->pPDUIIModule->rules[pRequest->a][pRequest->extra].pttDelay = plo->val;
        epicsTimeGetCurrent(&(pRequest->actTime));
        pRequest->errCode = PDUII_REQUEST_NO_ERR;
        pRequest->opDone = 1;
	return 0;
    }

    if(!plo->pact)
    {/* pre-process */
        /* Clean up the request */
        epicsTimeGetCurrent(&(pRequest->actTime));
        pRequest->errCode = PDUII_REQUEST_NO_ERR;
        pRequest->opDone = 0;

        pRequest->val = plo->val;

        if(epicsMessageQueueTrySend(pRequest->pPDUIIModule->msgQId, (void *)&pRequest, sizeof(PDUII_REQUEST *)) == -1)
        {
            recGblSetSevr(plo, WRITE_ALARM, INVALID_ALARM);
            errlogPrintf("Send Message to PDUII Operation Thread Error [%s]", plo->name);
            rtn = -1;
        }
        else
        {
            plo->pact = TRUE;
            rtn = 0;
        }

    }/* pre-process */
    else
    {/* post-process */
        if( (!pRequest->opDone) || pRequest->errCode )
        {
            recGblSetSevr(plo, WRITE_ALARM, INVALID_ALARM);
            if(PDUII_DRV_DEBUG)   printf("Record [%s] error %s!\n", plo->name, cammsg(pRequest->errCode));
            rtn = -1;
        }
        else
        {
            if(PDUII_DRV_DEBUG > 1)   printf("Record [%s] succeed!\n", plo->name);

            if(plo->tse == epicsTimeEventDeviceTime)/* do timestamp by device support */
                plo->time = pRequest->actTime;
            rtn = 0;
        }
    }/* post-process */
    return (rtn);

}

/*******        waveform record       ******/
static long init_wf(struct waveformRecord * pwf)
{
    int status;
    /*PDUII_REQUEST  *pRequest;*/

    if(pwf->inp.type!=CAMAC_IO)
    {
        recGblRecordError(S_db_badField, (void *)pwf, "devMoDPDUII Init_record, Illegal OUT");
        pwf->pact=TRUE;
        return (S_db_badField);
    }

    if(pwf->ftvl != DBF_USHORT || pwf->nelm != 2*N_USHORTS_MASK+1)
    {
        recGblRecordError(S_db_badField, (void *)pwf, "devWfKVI_BPM2 init_record, Illegal FTVL or NELM field");
        pwf->pact=TRUE;
        return (S_db_badField);
    }

    status = PDUIIRequestInit((dbCommon *)pwf, pwf->inp.value.camacio, EPICS_RECTYPE_WF);

    if(status)
    {
        recGblRecordError(S_db_badField, (void *)pwf, "devMoDPDUII Init_record failed");
        pwf->pact=TRUE;
        return (S_db_badField);
    }

    /* We don't want to read rules here since they are all 0 anyway */
    /* We depend on autoSaveRestore */

    return 0;
}

static long write_wf(struct waveformRecord *pwf)
{
    PDUII_REQUEST  *pRequest = (PDUII_REQUEST *)(pwf->dpvt);
    UINT16 *pData;
    int loop;

    if(!pRequest) return(-1);

    if(pwf->rarm)       pwf->rarm=0;    /* reset RARM */

    pData = (UINT16 *)(pwf->bptr);
    /* rule has to be updated as a whole */
    epicsMutexMustLock(pRequest->pPDUIIModule->lockRule);
    for(loop=0;loop<N_USHORTS_MASK/2;loop++)
    {/* mask[0] is ignored anyway */
	pRequest->pPDUIIModule->rules[pRequest->a][pRequest->extra].inclusionMask[loop+1] =
            pData[loop*2+1]<<16|pData[loop*2];
	pRequest->pPDUIIModule->rules[pRequest->a][pRequest->extra].exclusionMask[loop+1] =
            pData[N_USHORTS_MASK+loop*2+1]<<16|pData[N_USHORTS_MASK+loop*2];
    }
    pRequest->pPDUIIModule->rules[pRequest->a][pRequest->extra].beamCode = pData[N_USHORTS_MASK*2];
    epicsMutexUnlock(pRequest->pPDUIIModule->lockRule);

    epicsTimeGetCurrent(&(pRequest->actTime));
    pRequest->errCode = PDUII_REQUEST_NO_ERR;
    pRequest->opDone = 1;

    pwf->nord=2*N_USHORTS_MASK+1;
    pwf->udf=FALSE;

    return (0);
}

