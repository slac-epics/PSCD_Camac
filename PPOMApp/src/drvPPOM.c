/***************************************************************************\
 *   $Id: drvPPOM.c,v 1.2 2009/09/04 03:35:32 pengs Exp $
 *   File:		drvPPOM.c
 *   Author:		Sheng Peng
 *   Email:		pengsh2003@yahoo.com
 *   Phone:		408-660-7762
 *   Company:		RTESYS, Inc.
 *   Date:		09/2009
 *   Version:		1.0
 *
 *   EPICS driver for PPOM 
 *
\***************************************************************************/
#include "drvPSCDLib.h"
#include "devPPOM.h"
#include "slc_macros.h"
#include "cam_proto.h"

extern struct PSCD_CARD pscd_card;

int PPOM_DRV_DEBUG = 0;

/* We only support one PSCD per IOC */
/* So all PPOM transactions go thru one PSCD, only one task/msgQ needed */
static ELLLIST PPOMModuleList = {{NULL, NULL}, 0};
static epicsMessageQueueId PPOMmsgQId = NULL;
static epicsThreadId PPOMOpId = NULL;

/*********************************************************************/
static PPOM_MODULE * findPPOMModuleByBCN(short b, short c, short n)
{/* we don't check if PPOMModuleList is inited since drvInit should have done it */
    PPOM_MODULE * pPPOMModule = NULL;

    for( pPPOMModule = (PPOM_MODULE *)ellFirst(&PPOMModuleList); pPPOMModule; pPPOMModule = (PPOM_MODULE *)ellNext((ELLNODE *)pPPOMModule) )
    {
        /* if(pPPOMModule->b == b) */
        {
            if(pPPOMModule->c == (c&0xF))
            {
                if(pPPOMModule->n == (n&0x1F)) return pPPOMModule;
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

/* This function tries to reset or clear PPOM module */
/* It assumes b,c,n of pPPOMModule is valid */
/* Return 0 means succeed, otherwise error code */
UINT32 PPOM_RstClr(PPOM_REQUEST  *pPPOMRequest)
{
    UINT32 status = PPOM_REQUEST_NO_ERR;

    PPOM_MODULE * pPPOMModule = pPPOMRequest->pPPOMModule;
    if(!pPPOMRequest || !pPPOMModule) return -1; 

    /* check if module exists */
    if(isModuleExsit(pPPOMModule->b, pPPOMModule->c, pPPOMModule->n))
    {/* b (Branch) is not used in SLAC system */
        vmsstat_t iss;

        UINT16 bcnt = 0;
        UINT16 emask= 0xE0E0;

        UINT32 ppomctlw = 0x0;
        STAS_DAT rstclr_ppom = {0,0};

        if (!SUCCESS(iss = cam_ini (&pscd_card)))	/* no need, should be already done in PSCD driver */
	{
            errlogPrintf("cam_ini error 0x%08X\n",(unsigned int) iss);
            status = (PPOM_CAM_INIT_FAIL|iss);
	}
	else
	{
            /* F9A0 does reset, F10A0/1 clear */
            ppomctlw = (pPPOMModule->n << 7) | (pPPOMModule->c << 12) |
		    (pPPOMRequest->f << 16) | pPPOMRequest->a;
	    bcnt = 0;
            if (!SUCCESS(iss = camio (&ppomctlw, NULL, &bcnt, &(rstclr_ppom.stat), &emask)))
            {
                errlogPrintf ("camio error 0x%08X for PPOM F%dA%d write.\n",
				(unsigned int) iss, pPPOMRequest->f, pPPOMRequest->a);
                status = (PPOM_RSTCLR_CAMIO_FAIL|iss);
            }
	}
    }
    else
        status = PPOM_MODULE_NOT_EXIST;

    epicsTimeGetCurrent(&(pPPOMRequest->actTime));
    pPPOMRequest->errCode = status;
    pPPOMRequest->opDone = 1;

    return status;
}

/* This function tries to write PPOM module */
/* It assumes b,c,n of pPPOMModule is valid */
/* Return 0 means succeed, otherwise error code */
UINT32 PPOM_WriteData(PPOM_REQUEST  *pPPOMRequest)
{
    UINT32 status = PPOM_REQUEST_NO_ERR;

    PPOM_MODULE * pPPOMModule = pPPOMRequest->pPPOMModule;
    if(!pPPOMRequest || !pPPOMModule) return -1; 

    /* check if module exists */
    if(isModuleExsit(pPPOMModule->b, pPPOMModule->c, pPPOMModule->n))
    {/* b (Branch) is not used in SLAC system */
        vmsstat_t iss;

        UINT16 bcnt = 2;
        UINT16 emask= 0xE0E0;

        UINT32 ppomctlw = 0x0;
        STAS_DAT write_ppom = {0,0};

        if (!SUCCESS(iss = cam_ini (&pscd_card)))	/* no need, should be already done in PSCD driver */
	{
            errlogPrintf("cam_ini error 0x%08X\n",(unsigned int) iss);
            status = (PPOM_CAM_INIT_FAIL|iss);
	}
	else
	{
            /* F16, F17, (F18, F21), (F19, F23) A0/1 */
            ppomctlw = (pPPOMModule->n << 7) | (pPPOMModule->c << 12) |
		    (pPPOMRequest->f << 16) | pPPOMRequest->a;
	    bcnt = 2;
            *((UINT16 *)(&(write_ppom.data))) = pPPOMRequest->val;
            if (!SUCCESS(iss = camio (&ppomctlw, &(write_ppom.data), &bcnt, &(write_ppom.stat), &emask)))
            {
                errlogPrintf ("camio error 0x%08X for PPOM F%dA%d write.\n",
				(unsigned int) iss, pPPOMRequest->f, pPPOMRequest->a);
                status = (PPOM_WRT_CAMIO_FAIL|iss);
            }
	}
    }
    else
        status = PPOM_MODULE_NOT_EXIST;

    epicsTimeGetCurrent(&(pPPOMRequest->actTime));
    pPPOMRequest->errCode = status;
    pPPOMRequest->opDone = 1;

    return status;
}

/* This function tries to read PPOM module upon request */
/* It assumes b,c,n of pPPOMModule is valid */
/* Return 0 means succeed, otherwise error code */
UINT32 PPOM_ReadData(PPOM_REQUEST * pPPOMRequest)
{/* This function is not thread safe, but only used in one thread */

    UINT32 status = PPOM_REQUEST_NO_ERR;

    PPOM_MODULE * pPPOMModule = pPPOMRequest->pPPOMModule;
    if(!pPPOMRequest || !pPPOMModule) return -1;

    /* check if module exists */
    if(isModuleExsit(pPPOMModule->b, pPPOMModule->c, pPPOMModule->n))
    {

        vmsstat_t iss;
        UINT16 bcnt = 4;
        UINT16 emask= 0xE0E0;

        UINT32 ppomctlw = 0x0;
        STAS_DAT read_ppom[2];	/* We put 2 here for future optimization to combine two 16-bit read */

        if (!SUCCESS(iss = cam_ini (&pscd_card)))	/* no need, should be already done in PSCD driver */
        {
            errlogPrintf("cam_ini error 0x%08X\n",(unsigned int) iss);
            status = (PPOM_CAM_INIT_FAIL|iss);
        }
        else
        {
            /* F0 Aa to read PPOM, we don't use pPPOMRequest->f here to help init mbboDirect */
            ppomctlw = (pPPOMModule->n << 7) | (pPPOMModule->c << 12) /*| (pPPOMRequest->f << 16)*/ | (pPPOMRequest->a);
            if(PPOM_DRV_DEBUG) printf("PPOM Operation control word is 0x%08x\n", ppomctlw);
            bcnt = 4;

            if (!SUCCESS(iss = camio (&ppomctlw, &read_ppom[0].data, &bcnt, &read_ppom[0].stat, &emask)))
            {
                errlogPrintf ("camio error 0x%08X for PPOM F%dA%d\n", (unsigned int) iss, pPPOMRequest->f, pPPOMRequest->a);
                status = (PPOM_READ_CAMIO_FAIL|iss);
            }
	    else
                pPPOMRequest->val = (read_ppom[0].data)&0xFFFF;
	}
    }
    else
        status = PPOM_MODULE_NOT_EXIST;

    epicsTimeGetCurrent(&(pPPOMRequest->actTime)); 
    pPPOMRequest->errCode = status;
    pPPOMRequest->opDone = 1;
    return status;
}

static int PPOM_Operation(void * parg)
{
    int     msgQstatus;

    PPOM_REQUEST  *pPPOMRequest;

    if(PPOMmsgQId == NULL)
    {
        errlogPrintf("Operation thread quit because no legal PPOMmsgQId!\n");
        return -1;
    }

    while(TRUE)
    {
        msgQstatus = epicsMessageQueueReceive(PPOMmsgQId, &pPPOMRequest, sizeof(PPOM_REQUEST *) );
        if(msgQstatus < 0)
        {/* we should never time out, so something wrong */
            errlogPrintf("PPOM Operation msgQ timeout!\n");
            epicsThreadSleep(2.0);  /* Avoid super loop to starve CPU */
            /* TODO, add timeout for delayed process */
        }
        else
        {/* some requests come in, we deal it one by one, no dynamic combination */
            if(PPOM_DRV_DEBUG) printf("PPOM Operation task gets requests!\n");

            switch(pPPOMRequest->funcflag)
            {/* check funcflag */
                case PPOM_BO_RESET:
                case PPOM_BO_CLEAR:
                    PPOM_RstClr(pPPOMRequest);
                    break;
                case PPOM_MBBID_DATA:
                    PPOM_ReadData(pPPOMRequest);
                    break;
                case PPOM_MBBOD_DATA:
                case PPOM_MBBOD_CONF:
                    PPOM_WriteData(pPPOMRequest);
                    break;
            }/* check funcflag */

            /* process record */
            if(pPPOMRequest->pRecord)
            {
                if(PPOM_DRV_DEBUG > 1) printf("Got value for record [%s]=[0x%04X]\n", pPPOMRequest->pRecord->name, pPPOMRequest->val);
                dbScanLock(pPPOMRequest->pRecord);
                (*(pPPOMRequest->pRecord->rset->process))(pPPOMRequest->pRecord);
                dbScanUnlock(pPPOMRequest->pRecord);
                if(PPOM_DRV_DEBUG > 1) printf("Record [%s] processed\n", pPPOMRequest->pRecord->name);
            }
        }/* process requests */

    }/* infinite loop */

    /* We should never get here */
    return 0;
}

int PPOMRequestInit(dbCommon * pRecord, struct camacio inout, enum EPICS_RECTYPE rtyp)
{
    PPOM_MODULE * pPPOMModule = NULL;
    PPOM_REQUEST * pPPOMRequest = NULL;
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

    /* Check if the PPOM module is already in our list, or else add it */
    /* This should only happen in iocInit, single thread, so no mutex needed */
    pPPOMModule = findPPOMModuleByBCN(inout.b, inout.c, inout.n);

    if(!pPPOMModule)
    {/* Did not find any existing matching PPOM, create one */
        pPPOMModule = callocMustSucceed(1, sizeof(PPOM_MODULE), "calloc buffer for PPOM_MODULE");
        /* no bzero needed due to calloc */

        pPPOMModule->msgQId = PPOMmsgQId;

        pPPOMModule->b = inout.b; /* SLAC system does not use branch */
        pPPOMModule->c = inout.c & 0xF ;	
        pPPOMModule->n = inout.n & 0x1F;

        ellAdd(&PPOMModuleList, (ELLNODE *)pPPOMModule);

        if(PPOM_DRV_DEBUG) printf("Add PPOM[%d,%d,%d]\n",
            pPPOMModule->b, pPPOMModule->c, pPPOMModule->n);
    }
    /* Done check if the PPOM module is already in our list, or else add it */

    /* Request info prepare */
    pPPOMRequest = (PPOM_REQUEST *)callocMustSucceed(1, sizeof(PPOM_REQUEST), "calloc PPOM_REQUEST");
    /* no bzero needed due to calloc */

    pPPOMRequest->pPPOMModule = pPPOMModule;
    pPPOMRequest->pRecord = pRecord;

    pPPOMRequest->funcflag = funcflag;

    pPPOMRequest->a = inout.a & 0xF;
    pPPOMRequest->f = inout.f & 0x1F;

    /*pPPOMRequest->actTime*/
    pPPOMRequest->val = 0u;
    pPPOMRequest->opDone = 0;
    pPPOMRequest->errCode = PPOM_REQUEST_NO_ERR;

    pRecord->dpvt = (void *)pPPOMRequest;

    return 0;
}


/**************************************************************************************************/
/* Here we supply the driver report function for epics                                            */
/**************************************************************************************************/
static  long    PPOM_EPICS_Init();
static  long    PPOM_EPICS_Report(int level);

const struct drvet drvPPOM = {2,                              /*2 Table Entries */
                             (DRVSUPFUN) PPOM_EPICS_Report,  /* Driver Report Routine */
                             (DRVSUPFUN) PPOM_EPICS_Init};   /* Driver Initialization Routine */

#if EPICS_VERSION>=3 && EPICS_REVISION>=14
epicsExportAddress(drvet,drvPPOM);
#endif

/* implementation */
static long PPOM_EPICS_Init()
{

    ellInit(&PPOMModuleList);

    /* how many record could send request to PPOM at same time, 1000 should be enough */
    PPOMmsgQId = epicsMessageQueueCreate(1000, sizeof(struct PPOM_REQUEST *));

    if (PPOMmsgQId == NULL)
    {/* Fail to create messageQ */
        errlogPrintf("Failed to create messageQ for PPOM operation!\n");
        epicsThreadSuspendSelf();
    }
    PPOMOpId = epicsThreadMustCreate("PPOM_OP", epicsThreadPriorityLow, 20480, (EPICSTHREADFUNC)PPOM_Operation, (void *)0);

    return 0;
}

static long PPOM_EPICS_Report(int level)
{
    PPOM_MODULE  * pPPOMModule;

    printf("\n"PPOM_DRV_VER_STRING"\n\n");

    if(level > 0)   /* we only get into link list for detail when user wants */
    {
        for(pPPOMModule=(PPOM_MODULE *)ellFirst(&PPOMModuleList); pPPOMModule; pPPOMModule = (PPOM_MODULE *)ellNext((ELLNODE *)pPPOMModule))
        {
            printf("\tPPOM Module at b[%d]c[%d]n[%d]: \n\n", pPPOMModule->b, pPPOMModule->c, pPPOMModule->n);
        }
    }

    return 0;
}

