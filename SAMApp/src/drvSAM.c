/***************************************************************************\
 *   $Id: drvSAM.c,v 1.17 2009/05/12 08:10:08 pengs Exp $
 *   File:		drvSAM.c
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
static ELLLIST SAMModuleList = {{NULL, NULL}, 0};

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
{/* This function is not thread-safe, but only used in one thread per module */

    if(!pSAMModule) return -1;

    /* check if module exists */
    if(isModuleExsit(pSAMModule->b, pSAMModule->c, pSAMModule->n))
    {/* b (Branch) is not used in SLAC system, a is not needed for SAM either */
        void *pkg_p;  /* A camac package */
        vmsstat_t iss;

        UINT32 samctlw = 0x0;
        STAS_DAT read_sam[3] = {{0,0}, {0,0}, {0,0}};
        UINT16 bcnt = 4;
        UINT16 emask= 0xE0E0;

	int loop;

        union I2F
        {
            UINT32 tempI;
            float tempF;
        } value;

        if (!SUCCESS(iss = cam_ini (&pscd_card)))	/* no need, should be already done in PSCD driver */
            return (SAM_CAM_INIT_FAIL|iss);

#if 1
        /* F9 to reset */
        samctlw = (pSAMModule->n << 7) | (pSAMModule->c << 12) | (9 << 16);
	bcnt = 0;
        if (!SUCCESS(iss = camio (&samctlw, NULL/*&(read_sam[0].data)*/, &bcnt, &(read_sam[0].stat), &emask)))
        {
            errlogPrintf ("camio error 0x%08X for SAM reset\n", (unsigned int) iss);
            return (SAM_RST_CAMIO_FAIL|iss);
        }
	epicsThreadSleep(10);
#endif

        /* F16 to set module to know mode */
        samctlw = (pSAMModule->n << 7) | (pSAMModule->c << 12) | (16 << 16);
	bcnt = 2;
        *((UINT16 *)(&(read_sam[0].data))) = 0x5;  /* 100b, IEEE, normal mode, read firmware version */
        /*read_sam[0].data = 0x00000005;*/
        if (!SUCCESS(iss = camio (&samctlw, &(read_sam[0].data), &bcnt, &(read_sam[0].stat), &emask)))
        {
            errlogPrintf ("camio error 0x%08X for SAM setup\n", (unsigned int) iss);
            return (SAM_SETUP_CAMIO_FAIL|iss);
        }
        epicsThreadSleep(1);

        /* F17 to set start channel */
        samctlw = (pSAMModule->n << 7) | (pSAMModule->c << 12) | (17 << 16);
        bcnt = 2;
        /*read_sam[0].data = 0;*/
        *((UINT16 *)(&(read_sam[0].data))) = 0x0;	/* Channel 0 */
        if (!SUCCESS(iss = camio (&samctlw, &(read_sam[0].data), &bcnt, &(read_sam[0].stat), &emask)))
        {
            errlogPrintf ("camio error 0x%08X for SAM F11\n", (unsigned int) iss);
	    return (SAM_RST_CAMIO_FAIL|iss);
        }

        /* F0 to read firmware version */
        samctlw = (pSAMModule->n << 7) | (pSAMModule->c << 12) | (0 << 16);
        bcnt = 4;
        if (!SUCCESS(iss = camio (&samctlw, &read_sam[1].data, &bcnt, &read_sam[1].stat, &emask)))
        {
            errlogPrintf ("camio error 0x%08X for SAM F0 fwVer\n", (unsigned int) iss);
            return (SAM_SETUP_CAMIO_FAIL|iss);
        }

        /*value.tempI = (read_sam[1].data & 0xFFFF) | (read_sam[2].data << 16);*/
        value.tempI = (read_sam[1].data & 0xFFFFFF00);
	pSAMModule->fwVer = value.tempF;
        printf("SAM at crate[%d] slot [%d] with firmware version: %f\n", pSAMModule->c, pSAMModule->n, value.tempF);
        /*printf("Firmware Version raw: L:0x%08X H:0x%08X\n", read_sam[1].data, read_sam[2].data);*/

        /* F16 to setup */
        samctlw = (pSAMModule->n << 7) | (pSAMModule->c << 12) | (16 << 16);
	bcnt = 2;
        *((UINT16 *)(&(read_sam[0].data))) = 0x4;  /* 100b, IEEE, normal mode, no read firmware version */
        /*read_sam[0].data = 0x00000004;*/
        if (!SUCCESS(iss = camio (&samctlw, &(read_sam[0].data), &bcnt, &(read_sam[0].stat), &emask)))
        {
            errlogPrintf ("camio error 0x%08X for SAM setup\n", (unsigned int) iss);
            return (SAM_SETUP_CAMIO_FAIL|iss);
        }

        epicsThreadSleep(1);

        return 0;
    }
    else
        return SAM_MODULE_NOT_EXIST;
}

/* This function tries to read SAM module */
/* It assumes b,c,n of pSAMModule is valid */
/* Return 0 means succeed, otherwise error code */
static UINT32 SAM_Read(SAM_MODULE * pSAMModule)
{/* This function is not thread safe, but only used in one thread per module */

    UINT32 rtn = 0;

    if(!pSAMModule) return -1;

    /* check if module exists */
    if(isModuleExsit(pSAMModule->b, pSAMModule->c, pSAMModule->n) && pSAMModule->fwVer > 0.0)
    {
        unsigned int loop;

        union I2F
        {
            UINT32 tempI;
            float tempF;
        } value;

        void *pkg_p;  /* A camac package */
        vmsstat_t iss;
        UINT16 bcnt = 4;
        UINT16 emask= 0xE0E0;

        UINT32 ctlwF17 = 0x0;
        UINT32 ctlwF0 = 0x0;

        STAS_DAT read_sam[SAM_NUM_OF_CHANNELS+1];	/* need to read twice for each channel, each read gets 16 bits of 32-bit float */
        UINT16 nops = 0;

        if (!SUCCESS(iss = cam_ini (&pscd_card)))	/* no need, should be already done in PSCD driver */
        {
            errlogPrintf("cam_ini error 0x%08X\n",(unsigned int) iss);
            rtn = (SAM_CAM_INIT_FAIL|iss);
            goto egress;
        }

        nops = pSAMModule->numChannels + 1;
 
        /** Allocate package for SAM reset */
        if (!SUCCESS(iss = camalol (&nops, &pkg_p)))
        {
            errlogPrintf("camalol error 0x%08X\n",(unsigned int) iss);
            rtn = (SAM_CAM_ALLOC_FAIL|iss);
            goto egress;
        }

        ctlwF17 = (pSAMModule->n << 7) | (pSAMModule->c << 12) | (17 << 16);
        /*read_sam[0].data = pSAMModule->startChannel;*/
	bcnt = 2;
        *((UINT16 *)(&(read_sam[0].data))) = pSAMModule->startChannel;
        if (!SUCCESS(iss = camadd (&ctlwF17, &read_sam[0], &bcnt, &emask, &pkg_p)))
        {
            errlogPrintf("camadd error 0x%08X\n",(unsigned int) iss);
            rtn = (SAM_CAM_ADD_FAIL|iss);
            goto release_campkg;
        }

        ctlwF0 = (pSAMModule->n << 7) | (pSAMModule->c << 12) | (0 << 16);
        for (loop=1; loop <= pSAMModule->numChannels; loop++)
        {
	    bcnt = 4;
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
            value.tempI = ((read_sam[loop].data)&0xFFFFFF00);
            /*value.tempI = ((read_sam[loop*2-1].data)&0xFFF0) | (read_sam[loop*2].data << 16);*/
            pSAMModule->data[loop - 1 + pSAMModule->startChannel] = value.tempF;
        }
 
release_campkg: 

        if (!SUCCESS(iss = camdel (&pkg_p)))
            errlogPrintf("camdel error 0x%08X\n",(unsigned int) iss);
    }
    else if(!isModuleExsit(pSAMModule->b, pSAMModule->c, pSAMModule->n))
        rtn = SAM_MODULE_NOT_EXIST;
    else
        rtn = SAM_MODULE_UNKNOWN_MODE;

egress:
    epicsTimeGetCurrent(&(pSAMModule->lastReadTime)); 
    pSAMModule->lastErrCode = rtn;
    return rtn;
}

static int SAM_Operation(void * parg)
{
    int     msgQstatus;
    UINT32  errCode;

    epicsTimeStamp currentTime;

    SAM_REQUEST  *pSAMRequest;
    SAM_MODULE * pSAMModule = (SAM_MODULE *) parg;

    if(pSAMModule->msgQId == NULL)
    {
        errlogPrintf("Operation thread quit because no legal SAMmsgQId!\n");
        return -1;
    }

    errCode = SAM_Reset(pSAMModule);
    if(errCode)
    {
        errlogPrintf("Fail to call SAM_Reset SAM[%d,%d,%d], error 0x%08X\n", 
            pSAMModule->b, pSAMModule->c, pSAMModule->n, errCode);
        return -1;
    }

    while(TRUE)
    {
        msgQstatus = epicsMessageQueueReceive(pSAMModule->msgQId, &pSAMRequest, sizeof(SAM_REQUEST *) );
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
	char opTaskName[256];
        pSAMModule = callocMustSucceed(1, sizeof(SAM_MODULE), "calloc buffer for SAM_MODULE");
        /* no bzero needed due to calloc */

#if 0

/* SAM module,  b,c,n define a unique module */
typedef struct SAM_MODULE
{
    ELLNODE                     node;   /* Link List Node */

    UINT16			b;	/* branch */
    UINT16			c;	/* crate */
    UINT16			n;	/* node = slot */

    /* So one task/msgQ per module since reset takes long time */
    epicsMessageQueueId		msgQId;
    epicsThreadId               opTaskId;

    float			fwVer;	/* also used to indicate SAM in known state */

    UINT16			startChannel; /* 0 ~ 31 */
    UINT32			numChannels; /* 0 ~ 32 */

    ELLLIST                     SAMDelayedReqList;

    epicsTimeStamp		lastReadTime;
    float			data[SAM_NUM_OF_CHANNELS];
    UINT32			lastErrCode;

    /*char			camacPreMsg[256];*/	/* Could be done in init second pass */
} SAM_MODULE;
#endif

        pSAMModule->b = inout.b; /* SLAC system does not use branch */
        pSAMModule->c = inout.c & 0xF ;	
        pSAMModule->n = inout.n & 0x1F;

        /* how many record could send request to SAM at same time, 1000 should be enough */
        pSAMModule->msgQId = epicsMessageQueueCreate(1000, sizeof(struct SAM_REQUEST *));

        if (pSAMModule->msgQId == NULL)
        {/* Fail to create messageQ */
            errlogPrintf("Failed to create messageQ for SAM[%d,%d,%d] operation!\n",pSAMModule->b, pSAMModule->c, pSAMModule->n);
            epicsThreadSuspendSelf();
        }
        sprintf(opTaskName, "%d-%dSAM", pSAMModule->c, pSAMModule->n);
        pSAMModule->opTaskId = epicsThreadMustCreate(opTaskName, epicsThreadPriorityLow, 20480, (EPICSTHREADFUNC)SAM_Operation, (void *)pSAMModule);

        pSAMModule->fwVer = -1.0;

        pSAMModule->startChannel = 0;
        pSAMModule->numChannels = 0;

        ellInit(&(pSAMModule->SAMDelayedReqList));

        /*pSAMModule->lastReadTime;*/
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

#if 0
typedef struct SAM_REQUEST
{
    ELLNODE             node;   /* Link List Node */

    SAM_MODULE          *pSAMModule;
    dbCommon            *pRecord;

    int                 funcflag; /* read data/reset */
    UINT16		a;
    UINT16		f;

    epicsTimeStamp	reqTime;
    float		val;
    UINT32	        errCode;
    int                 opDone;
} SAM_REQUEST;
#endif

    pSAMRequest->pSAMModule = pSAMModule;
    pSAMRequest->pRecord = pRecord;

    pSAMRequest->funcflag = funcflag;
    pSAMRequest->a = inout.a & 0x1F;	/* We are using a to indicate channel number here. So up to 31 */

    if(funcflag == SAM_AI_DATA)
    {
	if(pSAMModule->numChannels == 0)
        {/* first time */
            pSAMModule->startChannel = pSAMRequest->a;
	    pSAMModule->numChannels = 1;
	}
	else if(pSAMRequest->a < pSAMModule->startChannel)
	{/* Here we use a to indicate which channel to read */
            pSAMModule->numChannels += (pSAMModule->startChannel - pSAMRequest->a);
            pSAMModule->startChannel = pSAMRequest->a;
        }
        else if(pSAMRequest->a >= (pSAMModule->startChannel + pSAMModule->numChannels))
        {
            pSAMModule->numChannels = (pSAMRequest->a - pSAMModule->startChannel) + 1;
        }
    }

    pSAMRequest->f = inout.f & 0x1F;

    /*pSAMRequest->reqTime*/
    pSAMRequest->val = 0.0;
    pSAMRequest->errCode = SAM_REQUEST_NO_ERR;
    pSAMRequest->opDone = 0;

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
		printf("\tFrom ch[%d], %d channels are in use\n\n", pSAMModule->startChannel, pSAMModule->numChannels);
                printf("\tFirmware version is %g\n\n", pSAMModule->fwVer);
            }
        }
    }

    return 0;
}

