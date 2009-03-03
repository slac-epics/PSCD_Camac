/***************************************************************************\
 *   $Id: drvSAM.c,v 1.1.1.1 2009/02/12 20:08:59 pengs Exp $
 *   File:		devSAM.c
 *   Author:		Sheng Peng
 *   Email:		pengsh2003@yahoo.com
 *   Phone:		408-660-7762
 *   Company:		RTESYS, Inc.
 *   Date:		03/2007
 *   Version:		1.0
 *
 *   EPICS driver for SAM 
 *
\***************************************************************************/
#include "drvPSCDLib.h"
#include "devSAM.h"

extern struct PSCD_CARD pscd_card;

int SAM_DRV_DEBUG = 0;

/* We only support one PSCD per IOC */
/* So all SAM transactions go thru one PSCD, only one task/msgQ needed */
static ELLLIST SAMModuleList = {{NULL, NULL}, 0};
static epicsMessageQueueId SAMmsgQId = NULL;
static epicsThreadId SAMOpId = NULL;

/*********************************************************************/
static SAM_MODULE * findSAMModuleByBCN(short b, short c, short n)
{/* we don't check if SAMModuleList is inited since drvInit should have done it */
    SAM_MODULE * pSAMModule = NULL;

    for( pSAMModule = (SAM_MODULE *)ellFirst(&SAMModuleList); pSAMModule; pSAMModule = (SAM_MODULE *)ellNext((ELLNODE *)pSAMModule) )
    {
        if(pSAMModule->b == b)
        {
            if(pSAMModule->c == c)
            {
                if(pSAMModule->n == n) return pSAMModule;
            }
        }
    }

    return NULL;
}

/* This function tries to reset SAM module */
/* It will read firmware version, set float value format, set normal mode as well */
/* It assumes b,c,n of pSAMModule is valid */
/* It will set fwVer, maybe camacPreMsg */
/* Return 0 means succeed, otherwise error code */
static int SAM_Reset(SAM_MODULE * pSAMModule)
{
    /* check if module exists */
    return 0;
}

/* This function tries to read SAM module */
/* It assumes b,c,n of pSAMModule is valid */
/* Return 0 means succeed, otherwise error code */
static int SAM_Read(SAM_MODULE * pSAMModule)
{
    /* check if module exists */
    return 0;
}

static int SAM_Operation(void * parg)
{
    int     msgQstatus;
    int     errCode;

    SAM_REQUEST  *pSAMRequest;

    epicsTimeStamp currentTime;

    if(SAMmsgQId == NULL)
    {
        errlogPrintf("Operation thread quit because no legal SAMmsgQId!\n");
        return -1;
    }

    while(TRUE)
    {
        msgQstatus = epicsMessageQueueReceive(SAMmsgQId, &pSAMRequest, sizeof(SAM_REQUEST *) );
        if(msgQstatus < 0)
        {/* we should never time out, so something wrong */
            errlogPrintf("SAM Operation msgQ timeout!\n");
            epicsThreadSleep(2.0);  /* Avoid super loop to starve CPU */
            /* TODO, add timeout for delayed process */
        }
        else
        {/* some requests come in, we deal it one by one, no dynamic combination */
            if(SAM_DRV_DEBUG) printf("SAM Operation task gets requests!\n");

            switch(pSAMRequest->funcflag)
            {/* check funcflag */
                case SAM_BO_RESET:
                    errCode = SAM_Reset(pSAMRequest->pSAMModule);

                    pSAMRequest->errCode = errCode;
                    pSAMRequest->opDone = 1;

                    /* process record */
                    if(pSAMRequest->pRecord)
                    {
                        dbScanLock(pSAMRequest->pRecord);
                        (*(pSAMRequest->pRecord->rset->process))(pSAMRequest->pRecord);
                        dbScanUnlock(pSAMRequest->pRecord);
                    }
                    break;
                case SAM_AI_DATA:
                    epicsTimeGetCurrent(&currentTime);
                    if(epicsTimeDiffInSeconds(&currentTime, &(pSAMRequest->pSAMModule->lastReadTime)) >= SAM_MIN_READ_INTERVAL)
                    {/* we can read SAM immediately */
                        errCode = SAM_Read(pSAMRequest->pSAMModule);

                        if(!errCode)
                        {
                            pSAMRequest->val = pSAMRequest->pSAMModule->data[pSAMRequest->a];
                            pSAMRequest->reqTime = pSAMRequest->pSAMModule->lastReadTime;
                        }
                        
                        pSAMRequest->errCode = errCode;
                        pSAMRequest->opDone = 1;

                        /* process record */
                        if(pSAMRequest->pRecord)
                        {
                            if(SAM_DRV_DEBUG > 1) printf("Got value for record [%s]=[%d]\n", pSAMRequest->pRecord->name, pSAMRequest->val);
                            dbScanLock(pSAMRequest->pRecord);
                            (*(pSAMRequest->pRecord->rset->process))(pSAMRequest->pRecord);
                            dbScanUnlock(pSAMRequest->pRecord);
                            if(SAM_DRV_DEBUG > 1) printf("Record [%s] processed\n", pSAMRequest->pRecord->name);
                        }
                    }
                    else
                    {/* No need to read SAM now, TODO, Delayed read */

                            pSAMRequest->val = pSAMRequest->pSAMModule->data[pSAMRequest->a];
                            pSAMRequest->reqTime = pSAMRequest->pSAMModule->lastReadTime;
                            pSAMRequest->errCode = pSAMRequest->pSAMModule->lastErrCode; /* snmp request timeout */
                            pSAMRequest->opDone = 1;

                            /* process record */
                            if(pSAMRequest->pRecord)
                            {
                                dbScanLock(pSAMRequest->pRecord);
                                (*(pSAMRequest->pRecord->rset->process))(pSAMRequest->pRecord);
                                dbScanUnlock(pSAMRequest->pRecord);
                            }
                    }
                    break;

            }/* check funcflag */
        }/* process requests */

    }/* infinite loop */

    /* We should never get here */
    return 0;
}

int SAMRequestInit(dbCommon * pRecord, struct camacio inout, enum EPICS_RECTYPE rtyp)
{
    SAM_MODULE * pSAMModule = NULL;
    int         funcflag = 0, loop;
    int		errCode;

    SAM_REQUEST * pSAMRequest = NULL;

    /* parameter check */
    if(!pRecord) return -1;

    for(loop=0; loop<N_PARAM_MAP; loop++)
    {
        if( 0 == strcmp(param_map[loop].param, inout.parm) )
        {
            if( rtyp == EPICS_RECTYPE_NONE || rtyp == param_map[loop].rtyp)
            {
                funcflag = param_map[loop].funcflag;
                break;
            }
        }
    }
    if(loop >= N_PARAM_MAP)
    {
        errlogPrintf("Record %s param %s is illegal!\n", pRecord->name, inout.parm);
        return -1;
    }

    /* Check if the SAM module is already in our list, or else add it */
    /* This should only happen in iocInit, single thread, so no mutex needed */
    pSAMModule = findSAMModuleByBCN(inout.b, inout.c, inout.n);

    if(!pSAMModule)
    {/* Did not find any existing matching SAM, create one */
        pSAMModule = callocMustSucceed(1, sizeof(SAM_MODULE), "calloc buffer for SAM_MODULE");
        /* no bzero needed due to calloc */

        pSAMModule->msgQId = SAMmsgQId;

        pSAMModule->b = inout.b;
        pSAMModule->c = inout.c;
        pSAMModule->n = inout.n;

        /* Reset SAM to known state */
        errCode = SAM_Reset(pSAMModule);
        if(errCode)
            errlogPrintf("Fail to reset SAM[%d,%d,%d], error 0x%08X\n", 
                pSAMModule->b, pSAMModule->c, pSAMModule->n, errCode);

        ellInit(&(pSAMModule->SAMDelayedReqList));
        if(SAM_DRV_DEBUG) printf("Add SAM[%d,%d,%d]\n",
            pSAMModule->b, pSAMModule->c, pSAMModule->n);
    }
    /* Done check if the SAM module is already in our list, or else add it */

    /* Request info prepare */
    pSAMRequest = (SAM_REQUEST *)callocMustSucceed(1, sizeof(SAM_REQUEST), "calloc SAM_REQUEST");
    /* no bzero needed due to calloc */

    pSAMRequest->pSAMModule = pSAMModule;
    pSAMRequest->pRecord = pRecord;

    pSAMRequest->funcflag = funcflag;
    pSAMRequest->a = inout.a;
    pSAMRequest->f = inout.f;

    /*pSAMRequest->reqTime*/
    pSAMRequest->val = 0;
    pSAMRequest->opDone = 0;
    pSAMRequest->errCode = SAM_REQUEST_NO_ERR;

    pRecord->dpvt = (void *)pSAMRequest;

    return 0;
}


/**************************************************************************************************/
/* Here we supply the driver report function for epics                                            */
/**************************************************************************************************/
static  long    SAM_EPICS_Init();
static  long    SAM_EPICS_Report(int level);

const struct drvet drvSAM = {2,                              /*2 Table Entries */
                             (DRVSUPFUN) SAM_EPICS_Report,  /* Driver Report Routine */
                             (DRVSUPFUN) SAM_EPICS_Init};   /* Driver Initialization Routine */

#if EPICS_VERSION>=3 && EPICS_REVISION>=14
epicsExportAddress(drvet,drvSAM);
#endif

/* implementation */
static long SAM_EPICS_Init()
{

    ellInit(&SAMModuleList);

    /* how many record could send request to SAM at same time, 1000 should be enough */
    SAMmsgQId = epicsMessageQueueCreate(1000, sizeof(struct SAM_REQUEST *));

    if (SAMmsgQId == NULL)
    {/* Fail to create messageQ */
        errlogPrintf("Failed to create messageQ for SAM operation!\n");
        epicsThreadSuspendSelf();
    }
    SAMOpId = epicsThreadMustCreate("SAM_OP", epicsThreadPriorityLow, 20480, (EPICSTHREADFUNC)SAM_Operation, (void *)0);

    return 0;
}

static long SAM_EPICS_Report(int level)
{
    SAM_MODULE  * pSAMModule;

    printf("\n"SAM_DRV_VER_STRING"\n\n");

    if(level > 0)   /* we only get into link list for detail when user wants */
    {
        for(pSAMModule=(SAM_MODULE *)ellFirst(&SAMModuleList); pSAMModule; pSAMModule = (SAM_MODULE *)ellNext((ELLNODE *)pSAMModule))
        {
            printf("\tSAM Module at b[%d]c[%d]n[%d]: \n", pSAMModule->b, pSAMModule->c, pSAMModule->n);
            if(level > 1)
            {
                printf("\tFirmware version is %d\n\n", pSAMModule->fwVer);
            }
        }
    }

    return 0;
}

