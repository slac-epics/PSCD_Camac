/***************************************************************************\
 *   $Id: drvDAC.c,v 1.5 2011/02/23 06:48:46 rcs Exp $
 *   File:		drvDAC.c
 *   Author:		Sheng Peng
 *   Email:		pengsh2003@yahoo.com
 *   Phone:		408-660-7762
 *   Company:		RTESYS, Inc.
 *   Date:		05/2009
 *   Version:		1.0
 *
 *   EPICS driver for DAC 
 *
\***************************************************************************/
#include "drvPSCDLib.h"
#include "devDAC.h"

extern struct PSCD_CARD pscd_card;

int DAC_DRV_DEBUG = 0;

/* We only support one PSCD per IOC */
/* So all DAC transactions go thru one PSCD, only one task/msgQ needed */
static ELLLIST DACModuleList = {{NULL, NULL}, 0};
static epicsMessageQueueId DACmsgQId = NULL;
static epicsThreadId DACOpId = NULL;

/*********************************************************************/
static DAC_MODULE * findDACModuleByBCN(short b, short c, short n)
{/* we don't check if DACModuleList is inited since drvInit should have done it */
    DAC_MODULE * pDACModule = NULL;

    for( pDACModule = (DAC_MODULE *)ellFirst(&DACModuleList); pDACModule; pDACModule = (DAC_MODULE *)ellNext((ELLNODE *)pDACModule) )
    {
        /* if(pDACModule->b == b) */
        {
            if(pDACModule->c == (c&0xF))
            {
                if(pDACModule->n == (n&0x1F)) return pDACModule;
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

/* This function tries to write DAC module */
/* It assumes b,c,n of pDACModule is valid */
/* Return 0 means succeed, otherwise error code */
UINT32 DAC_WriteData(DAC_REQUEST  *pDACRequest)
{
    UINT32 status = DAC_REQUEST_NO_ERR;
    DAC_MODULE * pDACModule = pDACRequest->pDACModule;
    if(!pDACRequest || !pDACModule) return -1; 

    /* check if module exists */
    if(isModuleExsit(pDACModule->b, pDACModule->c, pDACModule->n))
    {/* b (Branch) is not used in SLAC system, a is used as DAC channel */
        vmsstat_t iss;

        UINT32 dacctlw = 0x0;
        STAS_DAT op_dac = {0,0};
        UINT16 bcnt = 2;
        UINT16 emask= 0xE0E0;

        if (!SUCCESS(iss = cam_ini (&pscd_card)))	/* no need, should be already done in PSCD driver */
	{
            status = (DAC_CAM_INIT_FAIL|iss);
	}
	else
	{
            /* F16 to write data to DAC, we don't use pDACRequest->f here */
            dacctlw = (pDACModule->n << 7) | (pDACModule->c << 12) | (16 << 16) | pDACRequest->a;
	    bcnt = 2;
            *((UINT16 *)(&(op_dac.data))) = pDACRequest->val;
            if (!SUCCESS(iss = camio (&dacctlw, &(op_dac.data), &bcnt, &(op_dac.stat), &emask)))
            {
                if(DAC_DRV_DEBUG) printf ("camio error %s for DAC F16 write.\n", cammsg(iss));
                status = (DAC_WRT_CAMIO_FAIL|iss);
            }
	}
    }
    else
        status = DAC_MODULE_NOT_EXIST;

    epicsTimeGetCurrent(&(pDACRequest->actTime));
    pDACRequest->errCode = status;
    pDACRequest->opDone = 1;

    return status;
}

/* This function tries to read DAC module */
/* It assumes b,c,n of pDACModule is valid */
/* Return 0 means succeed, otherwise error code */
UINT32 DAC_ReadData(DAC_REQUEST  *pDACRequest)
{
    UINT32 status = DAC_REQUEST_NO_ERR;
    DAC_MODULE * pDACModule = pDACRequest->pDACModule;
    if(!pDACRequest || !pDACModule) return -1; 

    /* check if module exists */
    if(isModuleExsit(pDACModule->b, pDACModule->c, pDACModule->n))
    {
        vmsstat_t iss;

        UINT32 dacctlw = 0x0;
        STAS_DAT op_dac = {0,0};
        UINT16 bcnt = 2;
        UINT16 emask= 0xE0E0;

        if (!SUCCESS(iss = cam_ini (&pscd_card)))	/* no need, should be already done in PSCD driver */
	{
            status = (DAC_CAM_INIT_FAIL|iss);
	}
	else
	{
            /* F0 to read DAC setpoint register, we don't use pDACRequest->f here */
            dacctlw = (pDACModule->n << 7) | (pDACModule->c << 12) | (0 << 16) | pDACRequest->a;
            bcnt = 2;
            if (!SUCCESS(iss = camio (&dacctlw, &op_dac.data, &bcnt, &op_dac.stat, &emask)))
            {
                 if(DAC_DRV_DEBUG) printf ("camio error %s for DAC F0 read\n", cammsg(iss));
                 status = (DAC_RDDAT_CAMIO_FAIL|iss);
            }
            else
            {
                pDACRequest->val = *((UINT16 *)(&(op_dac.data)));
                if(DAC_DRV_DEBUG) printf("DAC at crate[%d] slot [%d] ch[%d] with value: 0x%X\n",
			    pDACModule->c, pDACModule->n, pDACRequest->a, pDACRequest->val);
	    }
	}
    }
    else
        status = DAC_MODULE_NOT_EXIST;

    epicsTimeGetCurrent(&(pDACRequest->actTime)); 
    pDACRequest->errCode = status;
    pDACRequest->opDone = 1;

    return status;
}

/* This function tries to read DAC module ID */
/* It assumes b,c,n of pDACModule is valid */
/* Return 0 means succeed, otherwise error code */
UINT32 DAC_ReadID(DAC_REQUEST  *pDACRequest)
{
    UINT32 status = DAC_REQUEST_NO_ERR;
    DAC_MODULE * pDACModule = pDACRequest->pDACModule;
    if(!pDACRequest || !pDACModule) return -1; 

    /* check if module exists */
    if(isModuleExsit(pDACModule->b, pDACModule->c, pDACModule->n))
    {
        vmsstat_t iss;

        UINT32 dacctlw = 0x0;
        STAS_DAT op_dac = {0,0};
        UINT16 bcnt = 2;
        UINT16 emask= 0xE0E0;

        if (!SUCCESS(iss = cam_ini (&pscd_card)))	/* no need, should be already done in PSCD driver */
	{
            status = (DAC_CAM_INIT_FAIL|iss);
	}
	else
	{
            /* F3 to read DAC module ID, we don't use pDACRequest->f and pDACRequest->a here */
            dacctlw = (pDACModule->n << 7) | (pDACModule->c << 12) | (3 << 16) | 0;
            bcnt = 2;
            if (!SUCCESS(iss = camio (&dacctlw, &op_dac.data, &bcnt, &op_dac.stat, &emask)))
            {
                 if(DAC_DRV_DEBUG) printf ("camio error %s for DAC F0 read\n", cammsg(iss));
                 status = (DAC_RDDAT_CAMIO_FAIL|iss);
            }

            pDACRequest->val = *((UINT16 *)(&(op_dac.data)));
            pDACModule->moduleID = pDACRequest->val;
            if(DAC_DRV_DEBUG) printf("DAC at crate[%d] slot [%d] ch[%d] with module ID: %d\n",
			    pDACModule->c, pDACModule->n, pDACRequest->a, pDACRequest->val);
	}
    }
    else
        status = DAC_MODULE_NOT_EXIST;

    epicsTimeGetCurrent(&(pDACRequest->actTime)); 
    pDACRequest->errCode = status;
    pDACRequest->opDone = 1;

    return status;
}

static int DAC_Operation(void * parg)
{
    int     msgQstatus;

    DAC_REQUEST  *pDACRequest;

    if(DACmsgQId == NULL)
    {
        errlogPrintf("Operation thread quit because no legal DACmsgQId!\n");
        return -1;
    }

    while(TRUE)
    {
        msgQstatus = epicsMessageQueueReceive(DACmsgQId, &pDACRequest, sizeof(DAC_REQUEST *) );
        if(msgQstatus < 0)
        {/* we should never time out, so something wrong */
            errlogPrintf("DAC Operation msgQ timeout!\n");
            epicsThreadSleep(2.0);  /* Avoid super loop to starve CPU */
        }
        else
        {/* some requests come in, we deal it one by one, no dynamic combination */
            if(DAC_DRV_DEBUG) printf("DAC Operation task gets requests!\n");

            switch(pDACRequest->funcflag)
            {/* check funcflag */
                case DAC_LI_MDID: /* F3 A0 */
                    DAC_ReadID(pDACRequest);
                    break;
                case DAC_AI_DATA: /* F0 An */
                    DAC_ReadData(pDACRequest);
                    break;
                case DAC_AO_DATA: /* F16 An */
                    DAC_WriteData(pDACRequest);
                    break;
            }/* check funcflag */

            /* process record */
            if(pDACRequest->pRecord)
            {
                if(DAC_DRV_DEBUG > 1) printf("Value for record [%s]=[0x%X]\n", pDACRequest->pRecord->name, pDACRequest->val);
                dbScanLock(pDACRequest->pRecord);
                (*(pDACRequest->pRecord->rset->process))(pDACRequest->pRecord);
                dbScanUnlock(pDACRequest->pRecord);
                if(DAC_DRV_DEBUG > 1) printf("Record [%s] processed\n", pDACRequest->pRecord->name);
            }
        }/* process requests */
    }/* infinite loop */

    /* We should never get here */
    return 0;
}

int DACRequestInit(dbCommon * pRecord, struct camacio inout, enum EPICS_RECTYPE rtyp)
{
    DAC_MODULE * pDACModule = NULL;
    int         funcflag = 0, loop;

    DAC_REQUEST * pDACRequest = NULL;

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

    /* Check if the DAC module is already in our list, or else add it */
    /* This should only happen in iocInit, single thread, so no mutex needed */
    pDACModule = findDACModuleByBCN(inout.b, inout.c, inout.n);

    if(!pDACModule)
    {/* Did not find any existing matching DAC, create one */
        pDACModule = callocMustSucceed(1, sizeof(DAC_MODULE), "calloc buffer for DAC_MODULE");
        /* no bzero needed due to calloc */
#if 0
/* DAC module,  b,c,n define a unique module */
typedef struct DAC_MODULE
{
    ELLNODE                     node;   /* Link List Node */

    epicsMessageQueueId		msgQId;	/* so far all modules share one Q */

    UINT16			b;	/* branch */
    UINT16			c;	/* crate */
    UINT16			n;	/* node = slot */

    UINT16			moduleID;
} DAC_MODULE;
#endif

        pDACModule->msgQId = DACmsgQId;

        pDACModule->b = inout.b; /* SLAC system does not use branch */
        pDACModule->c = inout.c & 0xF ;	
        pDACModule->n = inout.n & 0x1F;

        pDACModule->moduleID = 0xFFFF;

        ellAdd(&DACModuleList, (ELLNODE *)pDACModule);

        if(DAC_DRV_DEBUG) printf("Add DAC[%d,%d,%d]\n",
            pDACModule->b, pDACModule->c, pDACModule->n);
    }
    /* Done check if the DAC module is already in our list, or else add it */

    /* Request info prepare */
    pDACRequest = (DAC_REQUEST *)callocMustSucceed(1, sizeof(DAC_REQUEST), "calloc DAC_REQUEST");
    /* no bzero needed due to calloc */
#if 0
typedef struct DAC_REQUEST
{
    ELLNODE             node;   /* Link List Node */

    DAC_MODULE          *pDACModule;
    dbCommon            *pRecord;

    int                 funcflag; /* read data/write data/read mdid */
    UINT16		a;
    UINT16		f;

    epicsTimeStamp	actWriteTime;
    UINT16		val;
    UINT32	        errCode;
    int                 opDone;
} DAC_REQUEST;
#endif

    pDACRequest->pDACModule = pDACModule;
    pDACRequest->pRecord = pRecord;

    pDACRequest->funcflag = funcflag;
    pDACRequest->a = inout.a & 0xF;
    pDACRequest->f = inout.f & 0x1F;

    /*pDACRequest->actTime*/
    pDACRequest->val = 0;
    pDACRequest->opDone = 0;
    pDACRequest->errCode = DAC_REQUEST_NO_ERR;

    pRecord->dpvt = (void *)pDACRequest;

    return 0;
}


/**************************************************************************************************/
/* Here we supply the driver report function for epics                                            */
/**************************************************************************************************/
static  long    DAC_EPICS_Init();
static  long    DAC_EPICS_Report(int level);

const struct drvet drvDAC = {2,                              /*2 Table Entries */
                             (DRVSUPFUN) DAC_EPICS_Report,  /* Driver Report Routine */
                             (DRVSUPFUN) DAC_EPICS_Init};   /* Driver Initialization Routine */

#if EPICS_VERSION>=3 && EPICS_REVISION>=14
epicsExportAddress(drvet,drvDAC);
#endif

/* implementation */
static long DAC_EPICS_Init()
{

    ellInit(&DACModuleList);

    /* how many record could send request to DAC at same time, 1000 should be enough */
    DACmsgQId = epicsMessageQueueCreate(1000, sizeof(struct DAC_REQUEST *));

    if (DACmsgQId == NULL)
    {/* Fail to create messageQ */
        errlogPrintf("Failed to create messageQ for DAC operation!\n");
        epicsThreadSuspendSelf();
    }
    DACOpId = epicsThreadMustCreate("DAC_OP", epicsThreadPriorityLow, 20480, (EPICSTHREADFUNC)DAC_Operation, (void *)0);

    return 0;
}

static long DAC_EPICS_Report(int level)
{
    DAC_MODULE  * pDACModule;

    printf("\n"DAC_DRV_VER_STRING"\n\n");

    if(level > 0)   /* we only get into link list for detail when user wants */
    {
        for(pDACModule=(DAC_MODULE *)ellFirst(&DACModuleList); pDACModule; pDACModule = (DAC_MODULE *)ellNext((ELLNODE *)pDACModule))
        {
            printf("\tDAC Module at b[%d]c[%d]n[%d]: \n", pDACModule->b, pDACModule->c, pDACModule->n);
            if(level > 1)
            {
                printf("\tModule ID is %d\n\n", pDACModule->moduleID);
            }
        }
    }

    return 0;
}

