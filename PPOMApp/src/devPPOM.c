/***************************************************************************\
 *   $Id: devPPOM.c,v 1.3 2013/12/10 18:22:38 sonya Exp $
 *   File:		devPPOM.c
 *   Author:		Sheng Peng
 *   Email:		pengsh2003@yahoo.com
 *   Phone:		408-660-7762
 *   Company:		RTESYS, Inc.
 *   Date:		09/2009
 *   Version:		1.0
 *
 *   EPICS device support file for PPOM 
 *
\***************************************************************************/
#include "drvPSCDLib.h"
#include "devPPOM.h"

extern int PPOM_DRV_DEBUG;

/******************************************************************************************/
/*********************              implementation              ***************************/
/******************************************************************************************/

/* function prototypes */
static long init_bo(struct boRecord *pbo);
static long write_bo(struct boRecord *pbo);

static long init_mid(struct mbbiDirectRecord *pmbbid);
static long read_mid(struct mbbiDirectRecord *pmbbid);

static long init_mod(struct mbboDirectRecord *pmbbod);
static long write_mod(struct mbboDirectRecord *pmbbod);

/* global struct for devSup */
typedef struct {
        long            number;
        DEVSUPFUN       report;
        DEVSUPFUN       init;
        DEVSUPFUN       init_record;
        DEVSUPFUN       get_ioint_info;
        DEVSUPFUN       read_write;
        DEVSUPFUN       special_linconv;
} PPOM_DEV_SUP_SET;

PPOM_DEV_SUP_SET devBoPPOM =  {6, NULL, NULL, init_bo, NULL, write_bo, NULL};
PPOM_DEV_SUP_SET devMiDPPOM = {6, NULL, NULL, init_mid, NULL, read_mid, NULL};
PPOM_DEV_SUP_SET devMoDPPOM = {6, NULL, NULL, init_mod, NULL, write_mod, NULL};

#if     EPICS_VERSION>=3 && EPICS_REVISION>=14
epicsExportAddress(dset, devBoPPOM);
epicsExportAddress(dset, devMiDPPOM);
epicsExportAddress(dset, devMoDPPOM);
#endif

/*******        bo record       ******/
static long init_bo(struct boRecord * pbo)
{
    int status;

    if(pbo->out.type!=CAMAC_IO)
    {
        recGblRecordError(S_db_badField, (void *)pbo, "devBoPPOM Init_record, Illegal OUT");
        pbo->pact=TRUE;
        return (S_db_badField);
    }

    status = PPOMRequestInit((dbCommon *)pbo, pbo->out.value.camacio, EPICS_RECTYPE_BO);

    if(status)
    {
        recGblRecordError(S_db_badField, (void *)pbo, "devBoPPOM Init_record failed");
        pbo->pact=TRUE;
        return (S_db_badField);
    }

    return (status);
}

static long write_bo(struct boRecord *pbo)
{
    PPOM_REQUEST  *pRequest = (PPOM_REQUEST *)(pbo->dpvt);
    int rtn = -1;

    if(!pRequest) return(-1);

    if(!pbo->pact)
    {/* pre-process */
        /* Clean up the request */
        epicsTimeGetCurrent(&(pRequest->actTime));
        pRequest->errCode = PPOM_REQUEST_NO_ERR;
        pRequest->opDone = 0;

        pRequest->val = pbo->val;

        if(epicsMessageQueueTrySend(pRequest->pPPOMModule->msgQId, (void *)&pRequest, sizeof(PPOM_REQUEST *)) == -1)
        {
            recGblSetSevr(pbo, WRITE_ALARM, INVALID_ALARM);
            if(PPOM_DRV_DEBUG)   printf("Send Message to PPOM Operation Thread Error [%s]", pbo->name);
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
            if(PPOM_DRV_DEBUG)   printf("Record [%s] error %s!\n", pbo->name, cammsg(pRequest->errCode));
            rtn = -1;
        }
        else
        {
            if(PPOM_DRV_DEBUG > 1)   printf("Record [%s] succeed!\n", pbo->name);

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
        recGblRecordError(S_db_badField, (void *)pmbbid, "devMiDPPOM Init_record, Illegal INP");
        pmbbid->pact=TRUE;
        return (S_db_badField);
    }

    status = PPOMRequestInit((dbCommon *)pmbbid, pmbbid->inp.value.camacio, EPICS_RECTYPE_MBBID);

    /* we don't need pmbbid->mask and pmbbid->nobt */
    if(status)
    {
        recGblRecordError(S_db_badField, (void *)pmbbid, "devMiDPPOM Init_record failed");
        pmbbid->pact=TRUE;
        return (S_db_badField);
    }

    return (status);
}

static long read_mid(struct mbbiDirectRecord *pmbbid)
{
    PPOM_REQUEST  *pRequest = (PPOM_REQUEST *)(pmbbid->dpvt);
    int rtn = -1;

    if(!pRequest) return(-1);

    if(!pmbbid->pact)
    {/* pre-process */
        /* Clean up the request */
        epicsTimeGetCurrent(&(pRequest->actTime));
        pRequest->errCode = PPOM_REQUEST_NO_ERR;
        pRequest->opDone = 0;

        if(epicsMessageQueueTrySend(pRequest->pPPOMModule->msgQId, (void *)&pRequest, sizeof(PPOM_REQUEST *)) == -1)
        {
            recGblSetSevr(pmbbid, READ_ALARM, INVALID_ALARM);
            if(PPOM_DRV_DEBUG)   printf("Send Message to PPOM Operation Thread Error [%s]", pmbbid->name);
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
            if(PPOM_DRV_DEBUG)   printf("Record [%s] error %s!\n", pmbbid->name, cammsg(pRequest->errCode));
            rtn = -1;
        }
        else
        {
            if(PPOM_DRV_DEBUG > 1)   printf("Record [%s] receives val [0x%04X]!\n", pmbbid->name, pRequest->val);

            if(pmbbid->tse == epicsTimeEventDeviceTime)/* do timestamp by device support */
                pmbbid->time = pRequest->actTime;
            pmbbid->val = pRequest->val;
            rtn = NO_CONVERT;
        }
    }/* post-process */
    return (rtn);

}

/*******        mbboDirect record       ******/
static long init_mod(struct mbboDirectRecord * pmbbod)
{
    int status;
    PPOM_REQUEST  *pRequest;

    if(pmbbod->out.type!=CAMAC_IO)
    {
        recGblRecordError(S_db_badField, (void *)pmbbod, "devMoDPPOM Init_record, Illegal OUT");
        pmbbod->pact=TRUE;
        return (S_db_badField);
    }

    status = PPOMRequestInit((dbCommon *)pmbbod, pmbbod->out.value.camacio, EPICS_RECTYPE_MBBOD);

    /* we don't need pmbbod->mask and pmbbod->nobt */
    if(status)
    {
        recGblRecordError(S_db_badField, (void *)pmbbod, "devMoDPPOM Init_record failed");
        pmbbod->pact=TRUE;
        return (S_db_badField);
    }

    pRequest = (PPOM_REQUEST *)(pmbbod->dpvt);
    if(pRequest)
    {
        switch(pRequest->funcflag)
        {/* check funcflag */
            case PPOM_MBBOD_DATA:
                if(PPOM_ReadData(pRequest))
                    return -1;
		else
                {
                    pmbbod->val = pRequest->val;
     		    pmbbod->udf = FALSE;
                    pmbbod->stat = pmbbod->sevr = NO_ALARM;
		}
                break;
	    default:
		break;
        }/* check funcflag */
    }

    return 2;
}

static long write_mod(struct mbboDirectRecord *pmbbod)
{
    PPOM_REQUEST  *pRequest = (PPOM_REQUEST *)(pmbbod->dpvt);
    int rtn = -1;

    if(!pRequest) return(-1);

    if(!pmbbod->pact)
    {/* pre-process */
        /* Clean up the request */
        epicsTimeGetCurrent(&(pRequest->actTime));
        pRequest->errCode = PPOM_REQUEST_NO_ERR;
        pRequest->opDone = 0;

        pRequest->val = pmbbod->val;

        if(epicsMessageQueueTrySend(pRequest->pPPOMModule->msgQId, (void *)&pRequest, sizeof(PPOM_REQUEST *)) == -1)
        {
            recGblSetSevr(pmbbod, WRITE_ALARM, INVALID_ALARM);
            if(PPOM_DRV_DEBUG)   printf("Send Message to PPOM Operation Thread Error [%s]", pmbbod->name);
            rtn = -1;
        }
        else
        {
            pmbbod->pact = TRUE;
            rtn = NO_CONVERT;
        }

    }/* pre-process */
    else
    {/* post-process */
        if( (!pRequest->opDone) || pRequest->errCode )
        {
            recGblSetSevr(pmbbod, READ_ALARM, INVALID_ALARM);
            if(PPOM_DRV_DEBUG)   printf("Record [%s] error %s!\n", pmbbod->name, cammsg(pRequest->errCode));
            rtn = -1;
        }
        else
        {
            if(PPOM_DRV_DEBUG > 1)   printf("Record [%s] succeed!\n", pmbbod->name);

            if(pmbbod->tse == epicsTimeEventDeviceTime)/* do timestamp by device support */
                pmbbod->time = pRequest->actTime;
            rtn = NO_CONVERT;
        }
    }/* post-process */
    return (rtn);

}

