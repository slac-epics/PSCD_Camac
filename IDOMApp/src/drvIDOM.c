/***************************************************************************\
 *   $Id: drvIDOM.c,v 1.4 2009/11/11 01:02:31 pengs Exp $
 *   File:		drvIDOM.c
 *   Author:		Sheng Peng
 *   Email:		pengsh2003@yahoo.com
 *   Phone:		408-660-7762
 *   Company:		RTESYS, Inc.
 *   Date:		09/2009
 *   Version:		1.0
 *
 *   EPICS driver for IDOM 
 *
\***************************************************************************/

/* TODO, check X and Q, cctlwmasks.h:#define MBCD_STAT__Q     0x000010000 */
/* TODO, keep tracking number of errors */

#include "drvPSCDLib.h"
#include "devIDOM.h"

extern struct PSCD_CARD pscd_card;

int IDOM_DRV_DEBUG = 0;

/* We only support one PSCD per IOC */
/* So all IDOM transactions go thru one PSCD, only one task/msgQ needed */
static ELLLIST IDOMModuleList = {{NULL, NULL}, 0};
static epicsMessageQueueId IDOMmsgQId = NULL;
static epicsThreadId IDOMOpId = NULL;

/*********************************************************************/
static IDOM_MODULE * findIDOMModuleByBCN(short b, short c, short n)
{/* we don't check if IDOMModuleList is inited since drvInit should have done it */
    IDOM_MODULE * pIDOMModule = NULL;

    for( pIDOMModule = (IDOM_MODULE *)ellFirst(&IDOMModuleList); pIDOMModule; pIDOMModule = (IDOM_MODULE *)ellNext((ELLNODE *)pIDOMModule) )
    {
        /* if(pIDOMModule->b == b) */
        {
            if(pIDOMModule->c == (c&0xF))
            {
                if(pIDOMModule->n == (n&0x1F)) return pIDOMModule;
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

/* This function tries to reset or clear IDOM module */
/* It assumes b,c,n of pIDOMModule is valid */
/* Return 0 means succeed, otherwise error code */
UINT32 IDOM_RstClr(IDOM_REQUEST  *pIDOMRequest)
{
    UINT32 status = IDOM_REQUEST_NO_ERR;

    IDOM_MODULE * pIDOMModule = pIDOMRequest->pIDOMModule;
    if(!pIDOMRequest || !pIDOMModule) return -1; 

    /* check if module exists */
    if(isModuleExsit(pIDOMModule->b, pIDOMModule->c, pIDOMModule->n))
    {/* b (Branch) is not used in SLAC system */
        vmsstat_t iss;

        UINT16 bcnt = 0;
        UINT16 emask= 0xE0E0;

        UINT32 idomctlw = 0x0;
        STAS_DAT rstclr_idom = {0,0};

        if (!SUCCESS(iss = cam_ini (&pscd_card)))	/* no need, should be already done in PSCD driver */
	{
            errlogPrintf("cam_ini error 0x%08X\n",(unsigned int) iss);
            status = (IDOM_CAM_INIT_FAIL|iss);
	}
	else
	{
            /* F9A0 does reset, F10A0/1 clear */
            idomctlw = (pIDOMModule->n << 7) | (pIDOMModule->c << 12) |
		    (pIDOMRequest->f << 16) | pIDOMRequest->a;
	    bcnt = 0;
            if (!SUCCESS(iss = camio (&idomctlw, NULL, &bcnt, &(rstclr_idom.stat), &emask)))
            {
                errlogPrintf ("camio error %s for IDOM F%dA%d write.\n",
				cammsg(iss), pIDOMRequest->f, pIDOMRequest->a);
                status = (IDOM_RSTCLR_CAMIO_FAIL|iss);
            }
	}
    }
    else
        status = IDOM_MODULE_NOT_EXIST;

    epicsTimeGetCurrent(&(pIDOMRequest->actTime));
    pIDOMRequest->errCode = status;
    pIDOMRequest->opDone = 1;

    return status;
}

/* This function tries to write IDOM module */
/* It assumes b,c,n of pIDOMModule is valid */
/* Return 0 means succeed, otherwise error code */
UINT32 IDOM_WriteData(IDOM_REQUEST  *pIDOMRequest)
{
    UINT32 status = IDOM_REQUEST_NO_ERR;

    IDOM_MODULE * pIDOMModule = pIDOMRequest->pIDOMModule;
    if(!pIDOMRequest || !pIDOMModule) return -1; 

    /* check if module exists */
    if(isModuleExsit(pIDOMModule->b, pIDOMModule->c, pIDOMModule->n))
    {/* b (Branch) is not used in SLAC system */
        vmsstat_t iss;

        UINT16 bcnt = 2;
        UINT16 emask= 0xE0E0;

        UINT32 idomctlw = 0x0;
        STAS_DAT write_idom = {0,0};

        if (!SUCCESS(iss = cam_ini (&pscd_card)))	/* no need, should be already done in PSCD driver */
	{
            errlogPrintf("cam_ini error 0x%08X\n",(unsigned int) iss);
            status = (IDOM_CAM_INIT_FAIL|iss);
	}
	else
	{
            /* F16, F17, (F18, F21), (F19, F23) A0/1 */
            idomctlw = (pIDOMModule->n << 7) | (pIDOMModule->c << 12) |
		    (pIDOMRequest->f << 16) | pIDOMRequest->a;
	    bcnt = 2;
            *((UINT16 *)(&(write_idom.data))) = pIDOMRequest->val;
            if (!SUCCESS(iss = camio (&idomctlw, &(write_idom.data), &bcnt, &(write_idom.stat), &emask)))
            {
                errlogPrintf ("camio error %s for IDOM F%dA%d write.\n",
				cammsg(iss), pIDOMRequest->f, pIDOMRequest->a);
                status = (IDOM_WRT_CAMIO_FAIL|iss);
            }
	}
    }
    else
        status = IDOM_MODULE_NOT_EXIST;

    epicsTimeGetCurrent(&(pIDOMRequest->actTime));
    pIDOMRequest->errCode = status;
    pIDOMRequest->opDone = 1;

    return status;
}

/* This function tries to read IDOM module upon request */
/* It assumes b,c,n of pIDOMModule is valid */
/* Return 0 means succeed, otherwise error code */
UINT32 IDOM_ReadData(IDOM_REQUEST * pIDOMRequest)
{/* This function is not thread safe, but only used in one thread */

    UINT32 status = IDOM_REQUEST_NO_ERR;

    IDOM_MODULE * pIDOMModule = pIDOMRequest->pIDOMModule;
    if(!pIDOMRequest || !pIDOMModule) return -1;

    /* check if module exists */
    if(isModuleExsit(pIDOMModule->b, pIDOMModule->c, pIDOMModule->n))
    {

        vmsstat_t iss;
        UINT16 bcnt = 4;
        UINT16 emask= 0xE0E0;

        UINT32 idomctlw = 0x0;
        STAS_DAT read_idom[2];	/* We put 2 here for future optimization to combine two 16-bit read */

        if (!SUCCESS(iss = cam_ini (&pscd_card)))	/* no need, should be already done in PSCD driver */
        {
            errlogPrintf("cam_ini error 0x%08X\n",(unsigned int) iss);
            status = (IDOM_CAM_INIT_FAIL|iss);
        }
        else
        {
            /* F0 Aa to read IDOM, we don't use pIDOMRequest->f here to help init mbboDirect */
            idomctlw = (pIDOMModule->n << 7) | (pIDOMModule->c << 12) /*| (pIDOMRequest->f << 16)*/ | (pIDOMRequest->a);
            if(IDOM_DRV_DEBUG) printf("IDOM Operation control word is 0x%08x\n", idomctlw);
            bcnt = 4;

            if (!SUCCESS(iss = camio (&idomctlw, &read_idom[0].data, &bcnt, &read_idom[0].stat, &emask)))
            {
                errlogPrintf ("camio error %s for IDOM F%dA%d\n", cammsg(iss), pIDOMRequest->f, pIDOMRequest->a);
                status = (IDOM_READ_CAMIO_FAIL|iss);
            }
	    else
                pIDOMRequest->val = (read_idom[0].data)&0xFFFF;
	}
    }
    else
        status = IDOM_MODULE_NOT_EXIST;

    epicsTimeGetCurrent(&(pIDOMRequest->actTime)); 
    pIDOMRequest->errCode = status;
    pIDOMRequest->opDone = 1;
    return status;
}

static int IDOM_Operation(void * parg)
{
    int     msgQstatus;

    IDOM_REQUEST  *pIDOMRequest;

    if(IDOMmsgQId == NULL)
    {
        errlogPrintf("Operation thread quit because no legal IDOMmsgQId!\n");
        return -1;
    }

    while(TRUE)
    {
        msgQstatus = epicsMessageQueueReceive(IDOMmsgQId, &pIDOMRequest, sizeof(IDOM_REQUEST *) );
        if(msgQstatus < 0)
        {/* we should never time out, so something wrong */
            errlogPrintf("IDOM Operation msgQ timeout!\n");
            epicsThreadSleep(2.0);  /* Avoid super loop to starve CPU */
            /* TODO, add timeout for delayed process */
        }
        else
        {/* some requests come in, we deal it one by one, no dynamic combination */
            if(IDOM_DRV_DEBUG) printf("IDOM Operation task gets requests!\n");

            switch(pIDOMRequest->funcflag)
            {/* check funcflag */
                case IDOM_BO_RESET:
                case IDOM_BO_CLEAR:
                    IDOM_RstClr(pIDOMRequest);
                    break;
                case IDOM_MBBID_LATCHED:
                    IDOM_ReadData(pIDOMRequest);
                    break;
                case IDOM_MBBOD_LEVEL_LATCHED:
                case IDOM_MBBOD_LEVEL_SELECT:
                case IDOM_MBBOD_PULSE_SELECT:
                case IDOM_MBBOD_PULSE:
                    IDOM_WriteData(pIDOMRequest);
                    break;
            }/* check funcflag */

            /* process record */
            if(pIDOMRequest->pRecord)
            {
                if(IDOM_DRV_DEBUG > 1) printf("Got value for record [%s]=[0x%04X]\n", pIDOMRequest->pRecord->name, pIDOMRequest->val);
                dbScanLock(pIDOMRequest->pRecord);
                (*(pIDOMRequest->pRecord->rset->process))(pIDOMRequest->pRecord);
                dbScanUnlock(pIDOMRequest->pRecord);
                if(IDOM_DRV_DEBUG > 1) printf("Record [%s] processed\n", pIDOMRequest->pRecord->name);
            }
        }/* process requests */

    }/* infinite loop */

    /* We should never get here */
    return 0;
}

int IDOMRequestInit(dbCommon * pRecord, struct camacio inout, enum EPICS_RECTYPE rtyp)
{
    IDOM_MODULE * pIDOMModule = NULL;
    IDOM_REQUEST * pIDOMRequest = NULL;
    int         funcflag = 0, loop;

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

    /* Check if the IDOM module is already in our list, or else add it */
    /* This should only happen in iocInit, single thread, so no mutex needed */
    pIDOMModule = findIDOMModuleByBCN(inout.b, inout.c, inout.n);

    if(!pIDOMModule)
    {/* Did not find any existing matching IDOM, create one */
        pIDOMModule = callocMustSucceed(1, sizeof(IDOM_MODULE), "calloc buffer for IDOM_MODULE");
        /* no bzero needed due to calloc */

        pIDOMModule->msgQId = IDOMmsgQId;

        pIDOMModule->b = inout.b; /* SLAC system does not use branch */
        pIDOMModule->c = inout.c & 0xF ;	
        pIDOMModule->n = inout.n & 0x1F;

        ellAdd(&IDOMModuleList, (ELLNODE *)pIDOMModule);

        if(IDOM_DRV_DEBUG) printf("Add IDOM[%d,%d,%d]\n",
            pIDOMModule->b, pIDOMModule->c, pIDOMModule->n);
    }
    /* Done check if the IDOM module is already in our list, or else add it */

    /* Request info prepare */
    pIDOMRequest = (IDOM_REQUEST *)callocMustSucceed(1, sizeof(IDOM_REQUEST), "calloc IDOM_REQUEST");
    /* no bzero needed due to calloc */

    pIDOMRequest->pIDOMModule = pIDOMModule;
    pIDOMRequest->pRecord = pRecord;

    pIDOMRequest->funcflag = funcflag;

    pIDOMRequest->a = inout.a & 0xF;
    pIDOMRequest->f = inout.f & 0x1F;
    if(IDOM_DRV_DEBUG) printf("Add IDOM[%d,%d,%d] with F%d A%d\n",
        pIDOMModule->b, pIDOMModule->c, pIDOMModule->n, pIDOMRequest->f, pIDOMRequest->a);

    /*pIDOMRequest->actTime*/
    pIDOMRequest->val = 0u;
    pIDOMRequest->opDone = 0;
    pIDOMRequest->errCode = IDOM_REQUEST_NO_ERR;

    pRecord->dpvt = (void *)pIDOMRequest;

    return 0;
}


/**************************************************************************************************/
/* Here we supply the driver report function for epics                                            */
/**************************************************************************************************/
static  long    IDOM_EPICS_Init();
static  long    IDOM_EPICS_Report(int level);

const struct drvet drvIDOM = {2,                              /*2 Table Entries */
                             (DRVSUPFUN) IDOM_EPICS_Report,  /* Driver Report Routine */
                             (DRVSUPFUN) IDOM_EPICS_Init};   /* Driver Initialization Routine */

#if EPICS_VERSION>=3 && EPICS_REVISION>=14
epicsExportAddress(drvet,drvIDOM);
#endif

/* implementation */
static long IDOM_EPICS_Init()
{

    ellInit(&IDOMModuleList);

    /* how many record could send request to IDOM at same time, 1000 should be enough */
    IDOMmsgQId = epicsMessageQueueCreate(1000, sizeof(struct IDOM_REQUEST *));

    if (IDOMmsgQId == NULL)
    {/* Fail to create messageQ */
        errlogPrintf("Failed to create messageQ for IDOM operation!\n");
        epicsThreadSuspendSelf();
    }
    IDOMOpId = epicsThreadMustCreate("IDOM_OP", epicsThreadPriorityLow, 20480, (EPICSTHREADFUNC)IDOM_Operation, (void *)0);

    return 0;
}

static long IDOM_EPICS_Report(int level)
{
    IDOM_MODULE  * pIDOMModule;

    printf("\n"IDOM_DRV_VER_STRING"\n\n");

    if(level > 0)   /* we only get into link list for detail when user wants */
    {
        for(pIDOMModule=(IDOM_MODULE *)ellFirst(&IDOMModuleList); pIDOMModule; pIDOMModule = (IDOM_MODULE *)ellNext((ELLNODE *)pIDOMModule))
        {
            printf("\tIDOM Module at b[%d]c[%d]n[%d]: \n\n", pIDOMModule->b, pIDOMModule->c, pIDOMModule->n);
        }
    }

    return 0;
}

