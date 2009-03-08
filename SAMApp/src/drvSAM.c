/***************************************************************************\
 *   $Id: drvSAM.c,v 1.3 2009/03/08 08:00:52 pengs Exp $
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
#include "slc_macros.h"
#include "cam_proto.h"

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
        /* if(pSAMModule->b == b) */
        {
            if(pSAMModule->c == (c&0xF))
            {
                if(pSAMModule->n == (n&0x1F)) return pSAMModule;
            }
        }
    }

    return NULL;
}

#if 0
       The format of the CAMAC control word, CTLW is as follows:
       (ref: SLC Basic Users Guide (BUG) for additional information)
       http://www.slac.stanford.edu/grp/cd/soft/wwwman/bug.www/chapter9.html

       Bits 0-3 = AAAA = Module subaddress.
              4 = RE_PACK = Re_pack data into smaller blocks.
              5 = P8   = 8-bit pack mode.  (See Pack Modes below).
              6 =  x   = Not used (yet).
           7-11 = NNNNN= Station address (slot)
          12-15 = CCCC = Crate address

          16-20 = FFFFF= CAMAC function code.
          21-23 = SSS  = Enables scan mode counters.
                         (See Scan Modes below.)
             21 = SA   = Enable subaddress counter.
             22 = SN   = Enable station address counter.
             23 = SC   = Enable crate counter.

          24-25 = II   = Scan Increment mode (see Scan Modes below).
             24 = ILQ  = Increment enabled counters only if Q=0.
             25 = IN   = Reset least significant enabled counter and
                         increment next most significant enabled
                         counter if X=0.
             26 = P24  = 24-bit pack mode. (See Pack Modes below).
          27-30 = XXQQ = Transfer/terminate mode (See Scan modes
                         below.)
             27 = QM2  = Terminate packet if Q=0.
             28 = QM1  = Transfer data if Q=1.
             29 = XM2  = Terminate packet if X=0.
             30 = XM1  = Transfer data if X=1.
             31 = MPC  = 0 if last packet of package; otherwise More
                                Packets Coming.

#endif

typedef struct STAS_DAT
{
    UINT32 stat;
    UINT32 data;
} STAS_DAT;

/* This function tries to reset SAM module */
/* It will read firmware version, set float value format, set normal mode as well */
/* It assumes b,c,n of pSAMModule is valid */
/* It will set fwVer */
/* Return 0 means succeed, otherwise error code */
static UINT32 SAM_Reset(SAM_MODULE * pSAMModule)
{/* This function is not thread-safe, but only used in one thread per system */

    if(!pSAMModule) return -1;

    /* check if module exists */
    if(isModuleExsit(pSAMModule->b, pSAMModule->c, pSAMModule->n))
    {/* b (Branch) is not used in SLAC system, a is not needed for SAM either */
        void *pkg_p;  /* A camac package */
        vmsstat_t iss;

        UINT32 samctlw = 0x0;
        UINT32 samstatdata[2] = {0,0x12345678};
        UINT16 bcnt = 4;
        UINT16 emask= 0xE0E0;

        if (!SUCCESS(iss = cam_ini ()))	/* no need, should be already done in PSCD driver */
            return (SAM_CAM_INIT_FAIL|iss);

        /* F9 to reset */
        samctlw = (pSAMModule->n << 7) | (pSAMModule->c << 12) | (9 << 16) | 0x04000000; /* 24-bit packed mode */;
        /* samctlw = 0x04095800; */
        if (!SUCCESS(iss = camio (NULL, &samctlw, &samstatdata[1], &bcnt, &samstatdata[0], &emask)))
        {
            errlogPrintf ("camio error 0x%08X for SAM reset\n", (unsigned int) iss);
            return (SAM_RST_CAMIO_FAIL|iss);
        }

        /* F16 to set module to know mode */
        samctlw = (pSAMModule->n << 7) | (pSAMModule->c << 12) | (16 << 16) | 0x04000000; /* 24-bit packed mode */;
        /* samctlw = 0x04105800; */
        samstatdata[1] = 0x00000004; /* 100b, IEEE, normal mode, no read firmware version, TODO */
        if (!SUCCESS(iss = camio (NULL, &samctlw, &samstatdata[1], &bcnt, &samstatdata[0], &emask)))
        {
            errlogPrintf ("camio error 0x%08X for SAM reset\n", (unsigned int) iss);
            return (SAM_SETUP_CAMIO_FAIL|iss);
        }

        return 0;
    }
    else
        return SAM_MODULE_NOT_EXIST;
}

/* This function tries to read SAM module */
/* It assumes b,c,n of pSAMModule is valid */
/* Return 0 means succeed, otherwise error code */
static UINT32 SAM_Read(SAM_MODULE * pSAMModule)
{/* This function is not thread safe, but only used in one thread */

    UINT32 rtn = 0;

    if(!pSAMModule) return -1;

    /* check if module exists */
    if(isModuleExsit(pSAMModule->b, pSAMModule->c, pSAMModule->n))
    {
        unsigned int loop;

        void *pkg_p;  /* A camac package */
        vmsstat_t iss;
        UINT16 bcnt = 4;
        UINT16 emask= 0xE0E0;

        UINT32 ctlwF17 = 0x0;
        UINT32 ctlwF0 = 0x0;

        STAS_DAT read_sam[SAM_NUM_OF_CHANNELS*2+1];	/* need to read twice for each channel, each read gets 16 bits of 32-bit float */
        UINT16 nops = 0;

        if (!SUCCESS(iss = cam_ini ()))	/* no need, should be already done in PSCD driver */
        {
            errlogPrintf("cam_ini error 0x%08X\n",(unsigned int) iss);
            rtn = (SAM_CAM_INIT_FAIL|iss);
            goto egress;
        }

        nops = 2 * pSAMModule->numChannels + 1;
 
        /** Allocate package for SAM reset */
        if (!SUCCESS(iss = camalol (NULL, &nops, &pkg_p)))
        {
            errlogPrintf("camalol error 0x%08X\n",(unsigned int) iss);
            rtn = (SAM_CAM_ALLOC_FAIL|iss);
            goto egress;
        }

        ctlwF17 = (pSAMModule->n << 7) | (pSAMModule->c << 12) | (17 << 16) | 0x04000000; /* 24-bit packed mode */;
        read_sam[0].data = pSAMModule->startChannel;
        if (!SUCCESS(iss = camadd (&ctlwF17, &read_sam[0], &bcnt, &emask, &pkg_p)))
        {
            errlogPrintf("camadd error 0x%08X\n",(unsigned int) iss);
            rtn = (SAM_CAM_ADD_FAIL|iss);
            goto release_campkg;
        }

        ctlwF0 = (pSAMModule->n << 7) | (pSAMModule->c << 12) | (0 << 16) | 0x04000000; /* 24-bit packed mode */;
        for (loop=1; loop <= (2 * pSAMModule->numChannels); loop++)
        {
            if (!SUCCESS(iss = camadd (&ctlwF0, &read_sam[loop], &bcnt, &emask, &pkg_p)))
            {
                errlogPrintf("camadd error 0x%08X\n",(unsigned int) iss);
                rtn = (SAM_CAM_ADD_FAIL|iss);
                goto release_campkg;
            }
        }

        if (!SUCCESS(iss = camgo (&pkg_p)))
        {
            errlogPrintf("camgo error 0x%08X\n",(unsigned int) iss);
            rtn = (SAM_CAM_GO_FAIL|iss);
            goto release_campkg;
        }


        for (loop=1; loop <= pSAMModule->numChannels; loop++)
        {
            UINT32 tempI;
            float tempF;
            tempI = (read_sam[loop*2-1].data) | (read_sam[loop*2].data << 16);
            tempF = *(float *) &tempI;
            pSAMModule->data[loop + pSAMModule->startChannel] = tempF;
        }
 
release_campkg: 

        if (!SUCCESS(iss = camdel (&pkg_p)))
            errlogPrintf("camdel error 0x%08X\n",(unsigned int) iss);
    }
    else
        rtn = SAM_MODULE_NOT_EXIST;

egress:
    epicsTimeGetCurrent(&(pSAMModule->lastReadTime)); 
    pSAMModule->lastErrCode = rtn;
    return rtn;
}

UINT32 SAM_Test()
{

    UINT32 rtn = 0;

    /* check if module exists */
    if(isModuleExsit(0, 5, 16))
    {
        unsigned int loop;

        void *pkg_p;  /* A camac package */
        vmsstat_t iss;
        UINT16 bcnt = 4;
        UINT16 emask= 0xE0E0;

        UINT32 ctlwF17 = 0x04115080;
        UINT32 ctlwF0 = 0x04005800;

        STAS_DAT read_sam[SAM_NUM_OF_CHANNELS*2 + 1];	/* need to read twice for each channel, each read gets 16 bits of 32-bit float */
        UINT16 nops = 0;

        if (!SUCCESS(iss = cam_ini ()))	/* no need, should be already done in PSCD driver */
        {
            errlogPrintf("cam_ini error 0x%08X\n",(unsigned int) iss);
            rtn = (SAM_CAM_INIT_FAIL|iss);
            goto egress;
        }

        nops = 65;
 
        /** Allocate package for SAM reset */
        if (!SUCCESS(iss = camalol (NULL, &nops, &pkg_p)))
        {
            errlogPrintf("camalol error 0x%08X\n",(unsigned int) iss);
            rtn = (SAM_CAM_ALLOC_FAIL|iss);
            goto egress;
        }

        read_sam[0].data = 0;
        if (!SUCCESS(iss = camadd (&ctlwF17, &read_sam[0], &bcnt, &emask, &pkg_p)))
        {
            errlogPrintf("camadd error 0x%08X\n",(unsigned int) iss);
            rtn = (SAM_CAM_ADD_FAIL|iss);
            goto release_campkg;
        }

        for (loop=1; loop <= 64; loop++)
        {
            if (!SUCCESS(iss = camadd (&ctlwF0, &read_sam[loop], &bcnt, &emask, &pkg_p)))
            {
                errlogPrintf("camadd error 0x%08X\n",(unsigned int) iss);
                rtn = (SAM_CAM_ADD_FAIL|iss);
                goto release_campkg;
            }
        }

        if (!SUCCESS(iss = camgo (&pkg_p)))
        {
            errlogPrintf("camgo error 0x%08X\n",(unsigned int) iss);
            rtn = (SAM_CAM_GO_FAIL|iss);
            goto release_campkg;
        }


        for (loop=1; loop <= 32; loop++)
        {
            UINT32 tempI;
            float tempF;
            tempI = (read_sam[loop*2-1].data) | (read_sam[loop*2].data << 16);
            tempF = *(float *) &tempI;
            printf("Ch[%d]: %g\n", loop, tempF);
        }
 
release_campkg: 

        if (!SUCCESS(iss = camdel (&pkg_p)))
            errlogPrintf("camdel error 0x%08X\n",(unsigned int) iss);
    }
    else
        rtn = SAM_MODULE_NOT_EXIST;
egress:
    return rtn;
}

static int SAM_Operation(void * parg)
{
    int     msgQstatus;
    UINT32  errCode;

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
                        SAM_Read(pSAMRequest->pSAMModule);

                        pSAMRequest->val = pSAMRequest->pSAMModule->data[pSAMRequest->a];
                        pSAMRequest->reqTime = pSAMRequest->pSAMModule->lastReadTime;
                        pSAMRequest->errCode = pSAMRequest->pSAMModule->lastErrCode;
                        pSAMRequest->opDone = 1;

                        /* process record */
                        if(pSAMRequest->pRecord)
                        {
                            if(SAM_DRV_DEBUG > 1) printf("Got value for record [%s]=[%g]\n", pSAMRequest->pRecord->name, pSAMRequest->val);
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
                            pSAMRequest->errCode = pSAMRequest->pSAMModule->lastErrCode;
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
    UINT32	errCode;

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

        pSAMModule->b = inout.b; /* SLAC system does not use branch */
        pSAMModule->c = inout.c & 0xF ;	
        pSAMModule->n = inout.n & 0x1F;

        /* Reset SAM to known state */
        pSAMModule->fwVer = -1.0;
        errCode = SAM_Reset(pSAMModule);
        if(errCode)
            errlogPrintf("Fail to reset SAM[%d,%d,%d], error 0x%08X\n", 
                pSAMModule->b, pSAMModule->c, pSAMModule->n, errCode);

        /*pSAMModule->lastReadTime;*/

        ellInit(&(pSAMModule->SAMDelayedReqList));

        pSAMModule->startChannel = 0;
        pSAMModule->numChannels = 0;
        /*pSAMModule->data;*/
        pSAMModule->lastErrCode = 0;

        ellAdd(&SAMModuleList, (ELLNODE *)pSAMModule);

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
    pSAMRequest->a = inout.a & 0xF;
    /* Here we use a to indicate which channel to read */
    if(pSAMRequest->a < pSAMModule->startChannel)
    {
        pSAMModule->numChannels += (pSAMModule->startChannel - pSAMRequest->a);
        pSAMModule->startChannel = pSAMRequest->a;
    }
    else if(pSAMRequest->a >= (pSAMModule->startChannel + pSAMModule->numChannels))
    {
        pSAMModule->numChannels = (pSAMRequest->a - pSAMModule->startChannel) + 1;
    }

    pSAMRequest->f = inout.f & 0x1F;

    /*pSAMRequest->reqTime*/
    pSAMRequest->val = 0.0;
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
                printf("\tFirmware version is %g\n\n", pSAMModule->fwVer);
            }
        }
    }

    return 0;
}

