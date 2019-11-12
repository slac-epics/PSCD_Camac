/***************************************************************************\
 *   $Id: drvIDIM.c,v 1.5 2013/12/10 18:22:37 sonya Exp $
 *   File:		drvIDIM.c
 *   Author:		Sheng Peng
 *   Email:		pengsh2003@yahoo.com
 *   Phone:		408-660-7762
 *   Company:		RTESYS, Inc.
 *   Date:		03/2007
 *   Version:		1.0
 *
 *   EPICS driver for IDIM 
 *
\***************************************************************************/
#include "drvPSCDLib.h"
#include "devIDIM.h"

extern struct PSCD_CARD pscd_card;

int IDIM_DRV_DEBUG = 0;

/* We only support one PSCD per IOC */
/* So all IDIM transactions go thru one PSCD, only one task/msgQ needed */
static ELLLIST IDIMModuleList = {{NULL, NULL}, 0};
static epicsMessageQueueId IDIMmsgQId = NULL;
static epicsThreadId IDIMOpId = NULL;

/*********************************************************************/
static IDIM_MODULE * findIDIMModuleByBCN(short b, short c, short n)
{/* we don't check if IDIMModuleList is inited since drvInit should have done it */
    IDIM_MODULE * pIDIMModule = NULL;

    for( pIDIMModule = (IDIM_MODULE *)ellFirst(&IDIMModuleList); pIDIMModule; pIDIMModule = (IDIM_MODULE *)ellNext((ELLNODE *)pIDIMModule) )
    {
        /* if(pIDIMModule->b == b) */
        {
            if(pIDIMModule->c == (c&0xF))
            {
                if(pIDIMModule->n == (n&0x1F)) return pIDIMModule;
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

/* This function tries to read IDIM module upon request */
/* It assumes b,c,n of pIDIMModule is valid */
/* Return 0 means succeed, otherwise error code */
static UINT32 IDIM_Read(IDIM_REQUEST * pIDIMRequest)
{/* This function is not thread safe, but only used in one thread */

    UINT32 rtn = IDIM_REQUEST_NO_ERR;

    IDIM_MODULE * pIDIMModule;

    if(!pIDIMRequest || !(pIDIMRequest->pIDIMModule)) return -1;

    pIDIMModule = pIDIMRequest->pIDIMModule;

    /* check if module exists */
    if(isModuleExsit(pIDIMModule->b, pIDIMModule->c, pIDIMModule->n))
    {

        vmsstat_t iss;
        UINT16 bcnt = 4;
        UINT16 emask= 0xE0E0;

        UINT32 idimctlw = 0x0;
        STAS_DAT read_idim[2];	/* We put 2 here for future optimization to combine two 16-bit read */

        /* F0 Aa to read IDIM */
        idimctlw = (pIDIMModule->n << 7) | (pIDIMModule->c << 12) | (pIDIMRequest->f << 16) | (pIDIMRequest->a);
        if(IDIM_DRV_DEBUG > 1) printf("IDIM Operation control word is 0x%08x\n", idimctlw);
        bcnt = 4;

#if 1
        if (!SUCCESS(iss = camio (&idimctlw, &read_idim[0].data, &bcnt, &read_idim[0].stat, &emask)))
        {
            if(IDIM_DRV_DEBUG) printf ("camio error %s for IDIM F%dA%d\n", cammsg(iss), pIDIMRequest->f, pIDIMRequest->a);
            rtn = (IDIM_READ_CAMIO_FAIL|iss);
            goto egress;
        }
#else
	{
	    void * pkg_p;
	    UINT16 nops = 1;

            if (!SUCCESS(iss = camalol (&nops, &pkg_p)))
            {
                errlogPrintf("camalol error %s\n",cammsg(iss));
                rtn = (IDIM_CAM_ALLOC_FAIL|iss);
                goto egress;
            }

	    if (!SUCCESS(iss = camadd (&idimctlw, &read_idim[0], &bcnt, &emask, &pkg_p)))
	    {
	        errlogPrintf("camadd error %s\n",cammsg(iss));
	        rtn = (IDIM_CAM_ADD_FAIL|iss);
	        goto release_campkg;
	    }

	    if (!SUCCESS(iss = camgo (&pkg_p)))
	    {
	        if(IDIM_DRV_DEBUG) printf("camgo error %s\n",cammsg(iss));
	        rtn = (IDIM_CAM_GO_FAIL|iss);
	        goto release_campkg;
	    }

release_campkg:

            if (!SUCCESS(iss = camdel (&pkg_p)))
	        errlogPrintf("camdel error %s\n",cammsg(iss));
	}
#endif
        pIDIMRequest->val = (read_idim[0].data)&0xFFFF;
 
    }
    else
        rtn = IDIM_MODULE_NOT_EXIST;

egress:
    epicsTimeGetCurrent(&(pIDIMRequest->reqTime)); 
    pIDIMRequest->errCode = rtn;
    pIDIMRequest->opDone = 1;
    return rtn;
}

static int IDIM_Operation(void * parg)
{
    int     msgQstatus;

    IDIM_REQUEST  *pIDIMRequest;

    if(IDIMmsgQId == NULL)
    {
        errlogPrintf("Operation thread quit because no legal IDIMmsgQId!\n");
        return -1;
    }

    while(TRUE)
    {
        msgQstatus = epicsMessageQueueReceive(IDIMmsgQId, &pIDIMRequest, sizeof(IDIM_REQUEST *) );
        if(msgQstatus < 0)
        {/* we should never time out, so something wrong */
            errlogPrintf("IDIM Operation msgQ timeout!\n");
            epicsThreadSleep(2.0);  /* Avoid super loop to starve CPU */
            /* TODO, add timeout for delayed process */
        }
        else
        {/* some requests come in, we deal it one by one, no dynamic combination */
            if(IDIM_DRV_DEBUG >1) printf("IDIM Operation task gets requests!\n");

            IDIM_Read(pIDIMRequest);

            /* process record */
            if(pIDIMRequest->pRecord)
            {
                if(IDIM_DRV_DEBUG > 2) printf("Got value for record [%s]=[0x%04X]\n", pIDIMRequest->pRecord->name, pIDIMRequest->val);
                dbScanLock(pIDIMRequest->pRecord);
                (*(pIDIMRequest->pRecord->rset->process))(pIDIMRequest->pRecord);
                dbScanUnlock(pIDIMRequest->pRecord);
                if(IDIM_DRV_DEBUG > 2) printf("Record [%s] processed\n", pIDIMRequest->pRecord->name);
            }
        }/* process requests */

    }/* infinite loop */

    /* We should never get here */
    return 0;
}

int IDIMRequestInit(dbCommon * pRecord, struct camacio inout, enum EPICS_RECTYPE rtyp)
{
    IDIM_MODULE * pIDIMModule = NULL;
    IDIM_REQUEST * pIDIMRequest = NULL;

    /* parameter check */
    if(!pRecord) return -1;

    /* Check if the IDIM module is already in our list, or else add it */
    /* This should only happen in iocInit, single thread, so no mutex needed */
    pIDIMModule = findIDIMModuleByBCN(inout.b, inout.c, inout.n);

    if(!pIDIMModule)
    {/* Did not find any existing matching IDIM, create one */
        pIDIMModule = callocMustSucceed(1, sizeof(IDIM_MODULE), "calloc buffer for IDIM_MODULE");
        /* no bzero needed due to calloc */

        pIDIMModule->msgQId = IDIMmsgQId;

        pIDIMModule->b = inout.b; /* SLAC system does not use branch */
        pIDIMModule->c = inout.c & 0xF ;	
        pIDIMModule->n = inout.n & 0x1F;

        ellAdd(&IDIMModuleList, (ELLNODE *)pIDIMModule);

        if(IDIM_DRV_DEBUG > 1) printf("Add IDIM[%d,%d,%d]\n",
            pIDIMModule->b, pIDIMModule->c, pIDIMModule->n);
    }
    /* Done check if the IDIM module is already in our list, or else add it */

    /* Request info prepare */
    pIDIMRequest = (IDIM_REQUEST *)callocMustSucceed(1, sizeof(IDIM_REQUEST), "calloc IDIM_REQUEST");
    /* no bzero needed due to calloc */

    pIDIMRequest->pIDIMModule = pIDIMModule;
    pIDIMRequest->pRecord = pRecord;

    pIDIMRequest->a = inout.a & 0xF;
    pIDIMRequest->f = inout.f & 0x1F;

    /*pIDIMRequest->reqTime*/
    pIDIMRequest->val = 0u;
    pIDIMRequest->opDone = 0;
    pIDIMRequest->errCode = IDIM_REQUEST_NO_ERR;

    pRecord->dpvt = (void *)pIDIMRequest;

    return 0;
}


/**************************************************************************************************/
/* Here we supply the driver report function for epics                                            */
/**************************************************************************************************/
static  long    IDIM_EPICS_Init();
static  long    IDIM_EPICS_Report(int level);

#ifndef USE_TYPED_DRVET
const struct drvet drvIDIM = {2,                              /*2 Table Entries */
                             (DRVSUPFUN) IDIM_EPICS_Report,  /* Driver Report Routine */
                             (DRVSUPFUN) IDIM_EPICS_Init};   /* Driver Initialization Routine */
#else
const drvet drvIDIM = {2, IDIM_EPICS_Report, IDIM_EPICS_Init};
#endif

#if (EPICS_VERSION>=3 && EPICS_REVISION>=14) || EPICS_VERSION>3
epicsExportAddress(drvet,drvIDIM);
#endif

/* implementation */
static long IDIM_EPICS_Init()
{

    ellInit(&IDIMModuleList);

    /* how many record could send request to IDIM at same time, 1000 should be enough */
    IDIMmsgQId = epicsMessageQueueCreate(1000, sizeof(struct IDIM_REQUEST *));

    if (IDIMmsgQId == NULL)
    {/* Fail to create messageQ */
        errlogPrintf("Failed to create messageQ for IDIM operation!\n");
        epicsThreadSuspendSelf();
    }
    IDIMOpId = epicsThreadMustCreate("IDIM_OP", epicsThreadPriorityLow, 20480, (EPICSTHREADFUNC)IDIM_Operation, (void *)0);

    return 0;
}

static long IDIM_EPICS_Report(int level)
{
    IDIM_MODULE  * pIDIMModule;

    printf("\n"IDIM_DRV_VER_STRING"\n\n");

    if(level > 0)   /* we only get into link list for detail when user wants */
    {
        for(pIDIMModule=(IDIM_MODULE *)ellFirst(&IDIMModuleList); pIDIMModule; pIDIMModule = (IDIM_MODULE *)ellNext((ELLNODE *)pIDIMModule))
        {
            printf("\tIDIM Module at b[%d]c[%d]n[%d]: \n\n", pIDIMModule->b, pIDIMModule->c, pIDIMModule->n);
        }
    }

    return 0;
}

