/***************************************************************************\
 *   $Id: drvLDIM.c,v 1.4 2013/12/10 18:22:38 sonya Exp $
 *   File:		drvLDIM.c
 *   Author:		Sheng Peng
 *   Email:		pengsh2003@yahoo.com
 *   Phone:		408-660-7762
 *   Company:		RTESYS, Inc.
 *   Date:		03/2007
 *   Version:		1.0
 *
 *   EPICS driver for LDIM 
 *
\***************************************************************************/
#include "drvPSCDLib.h"
#include "devLDIM.h"
#include "slc_macros.h"
#include "cam_proto.h"

extern struct PSCD_CARD pscd_card;

int LDIM_DRV_DEBUG = 0;

/* We only support one PSCD per IOC */
/* So all LDIM transactions go thru one PSCD, only one task/msgQ needed */
static ELLLIST LDIMModuleList = {{NULL, NULL}, 0};
static epicsMessageQueueId LDIMmsgQId = NULL;
static epicsThreadId LDIMOpId = NULL;

/*********************************************************************/
static LDIM_MODULE * findLDIMModuleByBCN(short b, short c, short n)
{/* we don't check if LDIMModuleList is inited since drvInit should have done it */
    LDIM_MODULE * pLDIMModule = NULL;

    for( pLDIMModule = (LDIM_MODULE *)ellFirst(&LDIMModuleList); pLDIMModule; pLDIMModule = (LDIM_MODULE *)ellNext((ELLNODE *)pLDIMModule) )
    {
        /* if(pLDIMModule->b == b) */
        {
            if(pLDIMModule->c == (c&0xF))
            {
                if(pLDIMModule->n == (n&0x1F)) return pLDIMModule;
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

/* This function tries to read LDIM module upon request */
/* It assumes b,c,n of pLDIMModule is valid */
/* Return 0 means succeed, otherwise error code */
static UINT32 LDIM_Read(LDIM_REQUEST * pLDIMRequest)
{/* This function is not thread safe, but only used in one thread */

    UINT32 rtn = LDIM_REQUEST_NO_ERR;

    LDIM_MODULE * pLDIMModule;

    if(!pLDIMRequest || !(pLDIMRequest->pLDIMModule)) return -1;

    pLDIMModule = pLDIMRequest->pLDIMModule;

    /* check if module exists */
    if(isModuleExsit(pLDIMModule->b, pLDIMModule->c, pLDIMModule->n))
    {

        vmsstat_t iss;
        UINT16 bcnt = 4;
        UINT16 emask= 0xE0E0;

        UINT32 ldimctlw = 0x0;
        STAS_DAT read_ldim[2];	/* We put 2 here for future optimization to combine two 16-bit read */

        /* F0 Aa to read LDIM */
        ldimctlw = (pLDIMModule->n << 7) | (pLDIMModule->c << 12) | (pLDIMRequest->f << 16) | (pLDIMRequest->a);
        if(LDIM_DRV_DEBUG > 1) printf("LDIM Operation control word is 0x%08x\n", ldimctlw);
        bcnt = 4;

#if 1
        if (!SUCCESS(iss = camio (&ldimctlw, &read_ldim[0].data, &bcnt, &read_ldim[0].stat, &emask)))
        {
            if(LDIM_DRV_DEBUG) printf ("camio error %s for LDIM F%dA%d\n", cammsg(iss), pLDIMRequest->f, pLDIMRequest->a);
            rtn = (LDIM_READ_CAMIO_FAIL|iss);
            goto egress;
        }
#else
	{
	    void * pkg_p;
	    UINT16 nops = 1;

            if (!SUCCESS(iss = camalol (&nops, &pkg_p)))
            {
                errlogPrintf("camalol error %s\n",cammsg(iss));
                rtn = (LDIM_CAM_ALLOC_FAIL|iss);
                goto egress;
            }

	    if (!SUCCESS(iss = camadd (&ldimctlw, &read_ldim[0], &bcnt, &emask, &pkg_p)))
	    {
	        errlogPrintf("camadd error %s\n",cammsg(iss));
	        rtn = (LDIM_CAM_ADD_FAIL|iss);
	        goto release_campkg;
	    }

	    if (!SUCCESS(iss = camgo (&pkg_p)))
	    {
	        if(LDIM_DRV_DEBUG) printf("camgo error %s\n",cammsg(iss));
	        rtn = (LDIM_CAM_GO_FAIL|iss);
	        goto release_campkg;
	    }

release_campkg:

            if (!SUCCESS(iss = camdel (&pkg_p)))
	        errlogPrintf("camdel error %s\n",cammsg(iss));
	}
#endif
        pLDIMRequest->val = (read_ldim[0].data)&0xFFFF;
 
    }
    else
        rtn = LDIM_MODULE_NOT_EXIST;

egress:
    epicsTimeGetCurrent(&(pLDIMRequest->reqTime)); 
    pLDIMRequest->errCode = rtn;
    pLDIMRequest->opDone = 1;
    return rtn;
}

static int LDIM_Operation(void * parg)
{
    int     msgQstatus;

    LDIM_REQUEST  *pLDIMRequest;

    if(LDIMmsgQId == NULL)
    {
        errlogPrintf("Operation thread quit because no legal LDIMmsgQId!\n");
        return -1;
    }

    while(TRUE)
    {
        msgQstatus = epicsMessageQueueReceive(LDIMmsgQId, &pLDIMRequest, sizeof(LDIM_REQUEST *) );
        if(msgQstatus < 0)
        {/* we should never time out, so something wrong */
            errlogPrintf("LDIM Operation msgQ timeout!\n");
            epicsThreadSleep(2.0);  /* Avoid super loop to starve CPU */
            /* TODO, add timeout for delayed process */
        }
        else
        {/* some requests come in, we deal it one by one, no dynamic combination */
            if(LDIM_DRV_DEBUG > 1) printf("LDIM Operation task gets requests!\n");

            LDIM_Read(pLDIMRequest);

            /* process record */
            if(pLDIMRequest->pRecord)
            {
                if(LDIM_DRV_DEBUG > 2) printf("Got value for record [%s]=[0x%04X]\n", pLDIMRequest->pRecord->name, pLDIMRequest->val);
                dbScanLock(pLDIMRequest->pRecord);
                (*(pLDIMRequest->pRecord->rset->process))(pLDIMRequest->pRecord);
                dbScanUnlock(pLDIMRequest->pRecord);
                if(LDIM_DRV_DEBUG > 2) printf("Record [%s] processed\n", pLDIMRequest->pRecord->name);
            }
        }/* process requests */

    }/* infinite loop */

    /* We should never get here */
    return 0;
}

int LDIMRequestInit(dbCommon * pRecord, struct camacio inout, enum EPICS_RECTYPE rtyp)
{
    LDIM_MODULE * pLDIMModule = NULL;
    LDIM_REQUEST * pLDIMRequest = NULL;

    /* parameter check */
    if(!pRecord) return -1;

    /* Check if the LDIM module is already in our list, or else add it */
    /* This should only happen in iocInit, single thread, so no mutex needed */
    pLDIMModule = findLDIMModuleByBCN(inout.b, inout.c, inout.n);

    if(!pLDIMModule)
    {/* Did not find any existing matching LDIM, create one */
        pLDIMModule = callocMustSucceed(1, sizeof(LDIM_MODULE), "calloc buffer for LDIM_MODULE");
        /* no bzero needed due to calloc */

        pLDIMModule->msgQId = LDIMmsgQId;

        pLDIMModule->b = inout.b; /* SLAC system does not use branch */
        pLDIMModule->c = inout.c & 0xF ;	
        pLDIMModule->n = inout.n & 0x1F;

        ellAdd(&LDIMModuleList, (ELLNODE *)pLDIMModule);

        if(LDIM_DRV_DEBUG > 1) printf("Add LDIM[%d,%d,%d]\n",
            pLDIMModule->b, pLDIMModule->c, pLDIMModule->n);
    }
    /* Done check if the LDIM module is already in our list, or else add it */

    /* Request info prepare */
    pLDIMRequest = (LDIM_REQUEST *)callocMustSucceed(1, sizeof(LDIM_REQUEST), "calloc LDIM_REQUEST");
    /* no bzero needed due to calloc */

    pLDIMRequest->pLDIMModule = pLDIMModule;
    pLDIMRequest->pRecord = pRecord;

    pLDIMRequest->a = inout.a & 0xF;
    pLDIMRequest->f = inout.f & 0x1F;

    /*pLDIMRequest->reqTime*/
    pLDIMRequest->val = 0u;
    pLDIMRequest->opDone = 0;
    pLDIMRequest->errCode = LDIM_REQUEST_NO_ERR;

    pRecord->dpvt = (void *)pLDIMRequest;

    return 0;
}


/**************************************************************************************************/
/* Here we supply the driver report function for epics                                            */
/**************************************************************************************************/
static  long    LDIM_EPICS_Init();
static  long    LDIM_EPICS_Report(int level);

#ifndef USE_TYPED_DRVET
const struct drvet drvLDIM = {2,                              /*2 Table Entries */
                             (DRVSUPFUN) LDIM_EPICS_Report,  /* Driver Report Routine */
                             (DRVSUPFUN) LDIM_EPICS_Init};   /* Driver Initialization Routine */
#else
const drvet drvLDIM = {2, LDIM_EPICS_Report, LDIM_EPICS_Init};
#endif

#if (EPICS_VERSION>=3 && EPICS_REVISION>=14) || EPICS_VERSION>3
epicsExportAddress(drvet,drvLDIM);
#endif

/* implementation */
static long LDIM_EPICS_Init()
{

    ellInit(&LDIMModuleList);

    /* how many record could send request to LDIM at same time, 1000 should be enough */
    LDIMmsgQId = epicsMessageQueueCreate(1000, sizeof(struct LDIM_REQUEST *));

    if (LDIMmsgQId == NULL)
    {/* Fail to create messageQ */
        errlogPrintf("Failed to create messageQ for LDIM operation!\n");
        epicsThreadSuspendSelf();
    }
    LDIMOpId = epicsThreadMustCreate("LDIM_OP", epicsThreadPriorityLow, 20480, (EPICSTHREADFUNC)LDIM_Operation, (void *)0);

    return 0;
}

static long LDIM_EPICS_Report(int level)
{
    LDIM_MODULE  * pLDIMModule;

    printf("\n"LDIM_DRV_VER_STRING"\n\n");

    if(level > 0)   /* we only get into link list for detail when user wants */
    {
        for(pLDIMModule=(LDIM_MODULE *)ellFirst(&LDIMModuleList); pLDIMModule; pLDIMModule = (LDIM_MODULE *)ellNext((ELLNODE *)pLDIMModule))
        {
            printf("\tLDIM Module at b[%d]c[%d]n[%d]: \n\n", pLDIMModule->b, pLDIMModule->c, pLDIMModule->n);
        }
    }

    return 0;
}

