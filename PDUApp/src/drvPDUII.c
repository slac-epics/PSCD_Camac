/***************************************************************************\
 *   $Id: drvPDUII.c,v 1.9 2010/04/17 12:04:13 pengs Exp $
 *   File:		drvPDUII.c
 *   Author:		Sheng Peng
 *   Email:		pengsh2003@yahoo.com
 *   Phone:		408-660-7762
 *   Company:		RTESYS, Inc.
 *   Date:		03/2010
 *   Version:		1.0
 *
 *   EPICS driver for PDUII 
 *
\***************************************************************************/
#include "drvPSCDLib.h"
#include "devPDUII.h"
#include "slc_macros.h"
#include "cam_proto.h"

extern struct PSCD_CARD pscd_card;

int PDUII_DRV_DEBUG = 0;

/* We only support one PSCD per IOC */
ELLLIST PDUIIModuleList = {{NULL, NULL}, 0};

#if 0
#define LOCKMODULE epicsMutexMustLock(pPDUIIModule->lockModule)
#define UNLOCKMODULE epicsMutexUnlock(pPDUIIModule->lockModule)
#else
#define LOCKMODULE 
#define UNLOCKMODULE 
#endif

/*********************************************************************/
static PDUII_MODULE * findPDUIIModuleByBCN(short b, short c, short n)
{/* we don't check if PDUIIModuleList is inited since drvInit should have done it */
    PDUII_MODULE * pPDUIIModule = NULL;

    for( pPDUIIModule = (PDUII_MODULE *)ellFirst(&PDUIIModuleList); pPDUIIModule; pPDUIIModule = (PDUII_MODULE *)ellNext((ELLNODE *)pPDUIIModule) )
    {
        /* if(pPDUIIModule->b == b) */
        {
            if(pPDUIIModule->c == (c&0xF))
            {
                if(pPDUIIModule->n == (n&0x1F)) return pPDUIIModule;
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

/* This function tries to reset PDUII module */
/* It will clean PTT to all 0xFFFFF and disable upper backplane and sequencer */
/* We can use separate functions to load PTT and enable upper backplane and sequencer */
/* It assumes b,c,n of pPDUIIModule is valid */
/* Return 0 means succeed, otherwise error code */
UINT32 PDUII_Reset(PDUII_MODULE * pPDUIIModule)
{
    int loop;
    if(!pPDUIIModule) return -1;

    /* check if module exists */
    if(isModuleExsit(pPDUIIModule->b, pPDUIIModule->c, pPDUIIModule->n))
    {/* b (Branch) is not used in SLAC system, but mean PSCD port */
        vmsstat_t iss;

	STAS_DAT cmd_pduii = {0,0};
        UINT32 pduiictlw = 0x0;
        UINT16 bcnt = 0;
        UINT16 emask= 0xE0E0;

        if (!SUCCESS(iss = cam_ini (&pscd_card)))	/* no need, should be already done in PSCD driver */
            return (PDUII_CAM_INIT_FAIL|iss);

        /* 360Hz task will try lock to detect if module is going thru reset */
        epicsMutexMustLock(pPDUIIModule->lockModule);

        for(loop=0; loop<N_CHNLS_PER_MODU*256; loop++) pPDUIIModule->pttCache[loop] = PTT_ENTRY_UNKNOWN|0xFFFFF;

        /* F9A0 to reset, ignore-overwrite whatever a,f in record */
        pduiictlw = (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (9 << 16) | 0;
	bcnt = 0;
        if (!SUCCESS(iss = camio ((const unsigned long *)&pduiictlw, NULL, &bcnt, &(cmd_pduii.stat), &emask)))
        {/* Reset failed, Leave all pttCache invalidated. Unlock module. */
            epicsMutexUnlock(pPDUIIModule->lockModule); 
            errlogPrintf ("camio error 0x%08X for PDUII reset\n", (unsigned int) iss);
            return (PDUII_RST_CAMIO_FAIL|iss);
        }
        else
        {/* reset succeeded. Validated pttCache, Unlock module */
            for(loop=0; loop<N_CHNLS_PER_MODU*256; loop++) pPDUIIModule->pttCache[loop] = 0xFFFFF;

            epicsMutexUnlock(pPDUIIModule->lockModule);
	    epicsThreadSleep(1);
            return 0;
        }
    }
    else
        return PDUII_MODULE_NOT_EXIST;
}

/* This function tries to enable/disable PDUII output */
/* It assumes b,c,n of pPDUIIModule is valid */
/* Return 0 means succeed, otherwise error code */
UINT32 PDUII_UpperBplnEnDis(PDUII_MODULE *pPDUIIModule, UINT32 enable)
{
    if(!pPDUIIModule) return -1;

    /* check if module exists */
    if(isModuleExsit(pPDUIIModule->b, pPDUIIModule->c, pPDUIIModule->n))
    {/* b (Branch) is not used in SLAC system, but mean PSCD port */
        vmsstat_t iss;

	STAS_DAT cmd_pduii = {0,0};
        UINT32 pduiictlw = 0x0;
        UINT16 bcnt = 0;
        UINT16 emask= 0xE0E0;

        if (!SUCCESS(iss = cam_ini (&pscd_card)))	/* no need, should be already done in PSCD driver */
            return (PDUII_CAM_INIT_FAIL|iss);

        /* F24A1 to disable and F26A1 to enable, ignore-overwrite whatever a,f in record */
	if(enable)
            pduiictlw = (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (26 << 16) | 1;
	else
            pduiictlw = (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (24 << 16) | 1;

	bcnt = 0;
        if (!SUCCESS(iss = camio ((const unsigned long *)&pduiictlw, NULL, &bcnt, &(cmd_pduii.stat), &emask)))
        {
            errlogPrintf ("camio error 0x%08X for PDUII enable-disable output\n", (unsigned int) iss);
            return (PDUII_ENO_CAMIO_FAIL|iss);
        }

        return 0;
    }
    else
        return PDUII_MODULE_NOT_EXIST;
}

/* This function tries to enable/disable PDUII sequencer */
/* It assumes b,c,n of pPDUIIModule is valid */
/* Return 0 means succeed, otherwise error code */
UINT32 PDUII_SeqrEnDis(PDUII_MODULE *pPDUIIModule, UINT32 enable)
{
    if(!pPDUIIModule) return -1;

    /* check if module exists */
    if(isModuleExsit(pPDUIIModule->b, pPDUIIModule->c, pPDUIIModule->n))
    {/* b (Branch) is not used in SLAC system, but mean PSCD port */
        vmsstat_t iss;

	STAS_DAT cmd_pduii = {0,0};
        UINT32 pduiictlw = 0x0;
        UINT16 bcnt = 0;
        UINT16 emask= 0xE0E0;

        if (!SUCCESS(iss = cam_ini (&pscd_card)))	/* no need, should be already done in PSCD driver */
            return (PDUII_CAM_INIT_FAIL|iss);

        /* F24A2 to disable and F26A2 to enable, ignore-overwrite whatever a,f in record */
	if(enable)
            pduiictlw = (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (26 << 16) | 2;
	else
            pduiictlw = (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (24 << 16) | 2;

	bcnt = 0;
        if (!SUCCESS(iss = camio ((const unsigned long *)&pduiictlw, NULL, &bcnt, &(cmd_pduii.stat), &emask)))
        {
            errlogPrintf ("camio error 0x%08X for PDUII enable-disable sequencer\n", (unsigned int) iss);
            return (PDUII_ENS_CAMIO_FAIL|iss);
        }

        return 0;
    }
    else
        return PDUII_MODULE_NOT_EXIST;
}

/* This function tries to read PDUII module status */
/* It assumes b,c,n of pPDUIIModule is valid */
/* Return 0 means succeed, otherwise error code */
UINT32 PDUII_Status(PDUII_MODULE *pPDUIIModule, UINT32 *pStatus)
{
    if(!pPDUIIModule) return -1;

    /* check if module exists */
    if(isModuleExsit(pPDUIIModule->b, pPDUIIModule->c, pPDUIIModule->n))
    {/* b (Branch) is not used in SLAC system, but mean PSCD port */
        vmsstat_t iss;

	STAS_DAT read_pduii = {0,0};
        UINT32 pduiictlw = 0x0;
        UINT16 bcnt = 4;
        UINT16 emask= 0xE0E0;

        if (!SUCCESS(iss = cam_ini (&pscd_card)))	/* no need, should be already done in PSCD driver */
            return (PDUII_CAM_INIT_FAIL|iss);

        /* F2A2 to read status, ignore-overwrite whatever a,f in record */
        pduiictlw = (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (2 << 16) | 2;

	bcnt = 4;
        if (!SUCCESS(iss = camio ((const unsigned long *)&pduiictlw, &(read_pduii.data), &bcnt, &(read_pduii.stat), &emask)))
        {
            errlogPrintf ("camio error 0x%08X for PDUII enable-disable sequencer\n", (unsigned int) iss);
            return (PDUII_STS_CAMIO_FAIL|iss);
        }
        if(pStatus) *pStatus = read_pduii.data & 0xFF;
        return 0;
    }
    else
        return PDUII_MODULE_NOT_EXIST;
}

/* This function tries to read mode of PDUII module */
/* It assumes b,c,n of pPDUIIModule is valid */
/* Return 0 means succeed, otherwise error code */
UINT32 PDUII_ModeGet(PDUII_REQUEST  *pPDUIIRequest)
{/* This function is not thread safe, but only used in one thread per module */

    UINT32 status = PDUII_REQUEST_NO_ERR;

    PDUII_MODULE * pPDUIIModule = pPDUIIRequest->pPDUIIModule;
    if(!pPDUIIRequest || !pPDUIIModule) return -1;

    /* check if module exists */
    if(isModuleExsit(pPDUIIModule->b, pPDUIIModule->c, pPDUIIModule->n))
    {
        void *pkg_p;  /* A camac package */
        vmsstat_t iss;
        UINT16 bcnt = 4;
        UINT16 emask= 0xE0E0;

        UINT32 ctlwF17A0 = 0x0;
        UINT32 ctlwF1A0 = 0x0;

        STAS_DAT read_pduii[2];	/* need to set PTTP first, then read mode */
        UINT16 nops = 2;

        if (!SUCCESS(iss = cam_ini (&pscd_card)))	/* no need, should be already done in PSCD driver */
        {
            errlogPrintf("cam_ini error 0x%08X\n",(unsigned int) iss);
            status = (PDUII_CAM_INIT_FAIL|iss);
            goto egress;
        }

        /** Allocate package for PDUII mode read */
        if (!SUCCESS(iss = camalo (&nops, &pkg_p)))
        {
            errlogPrintf("camalo error 0x%08X\n",(unsigned int) iss);
            status = (PDUII_CAM_ALLOC_FAIL|iss);
            goto egress;
        }

	/* Write PTTP with channel number, location (PP/YY) field is 0 */
        ctlwF17A0 = (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (17 << 16) | 0;
	bcnt = 2;
        *((UINT16 *)(&(read_pduii[0].data))) = pPDUIIRequest->a << 8;
        if (!SUCCESS(iss = camadd ((const unsigned long *)&ctlwF17A0, &read_pduii[0], &bcnt, &emask, &pkg_p)))
        {
            errlogPrintf("camadd error 0x%08X\n",(unsigned int) iss);
            status = (PDUII_CAM_ADD_FAIL|iss);
            goto release_campkg;
        }

        ctlwF1A0 = (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (1 << 16) | 0;
        bcnt = 2;
        if (!SUCCESS(iss = camadd ((const unsigned long *)&ctlwF1A0, &read_pduii[1], &bcnt, &emask, &pkg_p)))
        {
            errlogPrintf("camadd error 0x%08X\n",(unsigned int) iss);
            status = (PDUII_CAM_ADD_FAIL|iss);
            goto release_campkg;
        }

        if (!SUCCESS(iss = camgo (&pkg_p)))
        {
            errlogPrintf("camgo error 0x%08X\n",(unsigned int) iss);
            status = (PDUII_CAM_GO_FAIL|iss);
            goto release_campkg;
        }

	pPDUIIRequest->val = *((UINT16 *)(&(read_pduii[1].data)));
        pPDUIIModule->chnlMode[pPDUIIRequest->a] = (((pPDUIIRequest->val)>>12) & 0x7);
 
release_campkg: 

        if (!SUCCESS(iss = camdel (&pkg_p)))
            errlogPrintf("camdel error 0x%08X\n",(unsigned int) iss);
    }
    else
        status = PDUII_MODULE_NOT_EXIST;

egress:
    epicsTimeGetCurrent(&(pPDUIIRequest->actTime));
    pPDUIIRequest->errCode = status;
    pPDUIIRequest->opDone = 1;

    return status;
}

/* This function tries to set mode of PDUII module */
/* It assumes b,c,n of pPDUIIModule is valid */
/* Return 0 means succeed, otherwise error code */
UINT32 PDUII_ModeSet(PDUII_REQUEST  *pPDUIIRequest)
{/* This function is not thread safe, but only used in one thread per module */

    UINT32 status = PDUII_REQUEST_NO_ERR;

    PDUII_MODULE * pPDUIIModule = pPDUIIRequest->pPDUIIModule;
    if(!pPDUIIRequest || !pPDUIIModule) return -1;

    /* check if module exists */
    if(isModuleExsit(pPDUIIModule->b, pPDUIIModule->c, pPDUIIModule->n))
    {
        void *pkg_p;  /* A camac package */
        vmsstat_t iss;
        UINT16 bcnt = 4;
        UINT16 emask= 0xE0E0;

        UINT32 ctlwF17A0 = 0x0;
        UINT32 ctlwF17A1 = 0x0;

        STAS_DAT write_pduii[2];	/* need to set PTTP first, then read mode */
        UINT16 nops = 2;

        if (!SUCCESS(iss = cam_ini (&pscd_card)))	/* no need, should be already done in PSCD driver */
        {
            errlogPrintf("cam_ini error 0x%08X\n",(unsigned int) iss);
            status = (PDUII_CAM_INIT_FAIL|iss);
            goto egress;
        }

        /** Allocate package for PDUII mode read */
        if (!SUCCESS(iss = camalo (&nops, &pkg_p)))
        {
            errlogPrintf("camalo error 0x%08X\n",(unsigned int) iss);
            status = (PDUII_CAM_ALLOC_FAIL|iss);
            goto egress;
        }

	/* Write PTTP with channel number, location field is 0 */
        ctlwF17A0 = (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (17 << 16) | 0;
	bcnt = 2;
        *((UINT16 *)(&(write_pduii[0].data))) = pPDUIIRequest->a << 8;
        if (!SUCCESS(iss = camadd ((const unsigned long *)&ctlwF17A0, &write_pduii[0], &bcnt, &emask, &pkg_p)))
        {
            errlogPrintf("camadd error 0x%08X\n",(unsigned int) iss);
            status = (PDUII_CAM_ADD_FAIL|iss);
            goto release_campkg;
        }

        ctlwF17A1 = (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (17 << 16) | 1;
        bcnt = 2;
        *((UINT16 *)(&(write_pduii[1].data))) = pPDUIIRequest->val & 0x7;
        if (!SUCCESS(iss = camadd ((const unsigned long *)&ctlwF17A1, &write_pduii[1], &bcnt, &emask, &pkg_p)))
        {
            errlogPrintf("camadd error 0x%08X\n",(unsigned int) iss);
            status = (PDUII_CAM_ADD_FAIL|iss);
            goto release_campkg;
        }

        /* Each entry is independent, and only one op thread per module, so no need to lock. */
        /* And 360Hz task has higher priority, this will never break into 360Hz task sequence */
        /* So be conservetive, marking as TRANSITING asap will be good enough */
        pPDUIIModule->chnlMode[pPDUIIRequest->a] = CHNL_MODE_TRANSITING|(pPDUIIRequest->val & 0x7);
        if (!SUCCESS(iss = camgo (&pkg_p)))
        {/* Fail to set mode, leave as invalid */
            errlogPrintf("camgo error 0x%08X\n",(unsigned int) iss);
            status = (PDUII_CAM_GO_FAIL|iss);
        }
        else
        {/* Succeed. Validate chnlMode */
            pPDUIIModule->chnlMode[pPDUIIRequest->a] = (pPDUIIRequest->val & 0x7);
        }

release_campkg: 

        if (!SUCCESS(iss = camdel (&pkg_p)))
            errlogPrintf("camdel error 0x%08X\n",(unsigned int) iss);
    }
    else
        status = PDUII_MODULE_NOT_EXIST;

egress:
    epicsTimeGetCurrent(&(pPDUIIRequest->actTime));
    pPDUIIRequest->errCode = status;
    pPDUIIRequest->opDone = 1;

    return status;
}

/* This function tries to read PTT of PDUII module */
/* It assumes b,c,n of pPDUIIModule is valid */
/* Return 0 means succeed, otherwise error code */
UINT32 PDUII_PTTGet(PDUII_REQUEST  *pPDUIIRequest)
{/* This function is not thread safe, but only used in one thread per module */

    UINT32 status = PDUII_REQUEST_NO_ERR;

    PDUII_MODULE * pPDUIIModule = pPDUIIRequest->pPDUIIModule;
    if(!pPDUIIRequest || !pPDUIIModule) return -1;

    /* check if module exists */
    if(isModuleExsit(pPDUIIModule->b, pPDUIIModule->c, pPDUIIModule->n))
    {
        void *pkg_p;  /* A camac package */
        vmsstat_t iss;
        UINT16 bcnt = 4;
        UINT16 emask= 0xE0E0;

        UINT32 ctlwF17A0 = 0x0;
        UINT32 ctlwF0A1 = 0x0;

        STAS_DAT read_pduii[2];	/* need to set PTTP first, then read mode */
        UINT16 nops = 2;

        if (!SUCCESS(iss = cam_ini (&pscd_card)))	/* no need, should be already done in PSCD driver */
        {
            errlogPrintf("cam_ini error 0x%08X\n",(unsigned int) iss);
            status = (PDUII_CAM_INIT_FAIL|iss);
            goto egress;
        }

        /** Allocate package for PDUII mode read */
        if (!SUCCESS(iss = camalo (&nops, &pkg_p)))
        {
            errlogPrintf("camalo error 0x%08X\n",(unsigned int) iss);
            status = (PDUII_CAM_ALLOC_FAIL|iss);
            goto egress;
        }

	/* Write PTTP with channel number, location field is 0 */
        ctlwF17A0 = (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (17 << 16) | 0;
	bcnt = 2;
        *((UINT16 *)(&(read_pduii[0].data))) = (pPDUIIRequest->a << 8) | (pPDUIIRequest->extra & 0xFF);
        if (!SUCCESS(iss = camadd ((const unsigned long *)&ctlwF17A0, &read_pduii[0], &bcnt, &emask, &pkg_p)))
        {
            errlogPrintf("camadd error 0x%08X\n",(unsigned int) iss);
            status = (PDUII_CAM_ADD_FAIL|iss);
            goto release_campkg;
        }

        ctlwF0A1 = (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (0 << 16) | 1;
        bcnt = 4;
        if (!SUCCESS(iss = camadd ((const unsigned long *)&ctlwF0A1, &read_pduii[1], &bcnt, &emask, &pkg_p)))
        {
            errlogPrintf("camadd error 0x%08X\n",(unsigned int) iss);
            status = (PDUII_CAM_ADD_FAIL|iss);
            goto release_campkg;
        }

        if (!SUCCESS(iss = camgo (&pkg_p)))
        {
            errlogPrintf("camgo error 0x%08X\n",(unsigned int) iss);
            status = (PDUII_CAM_GO_FAIL|iss);
        }
        else
        {
	    pPDUIIRequest->val = read_pduii[1].data & 0xFFFFF;
            pPDUIIModule->pttCache[(pPDUIIRequest->a << 8) | (pPDUIIRequest->extra & 0xFF)] = pPDUIIRequest->val;
        }
 
release_campkg: 

        if (!SUCCESS(iss = camdel (&pkg_p)))
            errlogPrintf("camdel error 0x%08X\n",(unsigned int) iss);
    }
    else
        status = PDUII_MODULE_NOT_EXIST;

egress:
    epicsTimeGetCurrent(&(pPDUIIRequest->actTime));
    pPDUIIRequest->errCode = status;
    pPDUIIRequest->opDone = 1;

    return status;
}

/* This function tries to set PTT of PDUII module */
/* It assumes b,c,n of pPDUIIModule is valid */
/* Return 0 means succeed, otherwise error code */
UINT32 PDUII_PTTSet(PDUII_REQUEST  *pPDUIIRequest)
{/* This function is not thread safe, but only used in one thread per module */

    UINT32 status = PDUII_REQUEST_NO_ERR;

    PDUII_MODULE * pPDUIIModule = pPDUIIRequest->pPDUIIModule;
    if(!pPDUIIRequest || !pPDUIIModule) return -1;

    /* check if module exists */
    if(isModuleExsit(pPDUIIModule->b, pPDUIIModule->c, pPDUIIModule->n))
    {
        void *pkg_p;  /* A camac package */
        vmsstat_t iss;
        UINT16 bcnt = 4;
        UINT16 emask= 0xE0E0;

        UINT32 ctlwF17A0 = 0x0;
        UINT32 ctlwF16A1 = 0x0;

        STAS_DAT write_pduii[2];	/* need to set PTTP first, then read mode */
        UINT16 nops = 2;

        if (!SUCCESS(iss = cam_ini (&pscd_card)))	/* no need, should be already done in PSCD driver */
        {
            errlogPrintf("cam_ini error 0x%08X\n",(unsigned int) iss);
            status = (PDUII_CAM_INIT_FAIL|iss);
            goto egress;
        }

        /** Allocate package for PDUII mode read */
        if (!SUCCESS(iss = camalo (&nops, &pkg_p)))
        {
            errlogPrintf("camalo error 0x%08X\n",(unsigned int) iss);
            status = (PDUII_CAM_ALLOC_FAIL|iss);
            goto egress;
        }

	/* Write PTTP with channel number, location field is 0 */
        ctlwF17A0 = (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (17 << 16) | 0;
	bcnt = 2;
        *((UINT16 *)(&(write_pduii[0].data))) = (pPDUIIRequest->a << 8) | (pPDUIIRequest->extra & 0xFF);
        if (!SUCCESS(iss = camadd ((const unsigned long *)&ctlwF17A0, &write_pduii[0], &bcnt, &emask, &pkg_p)))
        {
            errlogPrintf("camadd error 0x%08X\n",(unsigned int) iss);
            status = (PDUII_CAM_ADD_FAIL|iss);
            goto release_campkg;
        }

        ctlwF16A1 = (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (16 << 16) | 1;
        bcnt = 4;
        write_pduii[1].data = pPDUIIRequest->val & 0xFFFFF;
        if (!SUCCESS(iss = camadd ((const unsigned long *)&ctlwF16A1, &write_pduii[1], &bcnt, &emask, &pkg_p)))
        {
            errlogPrintf("camadd error 0x%08X\n",(unsigned int) iss);
            status = (PDUII_CAM_ADD_FAIL|iss);
            goto release_campkg;
        }

        /* Never modify 360T reloading channel here, and only one op thread per module, so no need to lock. */
        /* And 360Hz task has higher priority, this will never break into 360Hz task sequence */
        /* So be conservetive, marking as UNKNOWN asap will be good enough */
        pPDUIIModule->pttCache[(pPDUIIRequest->a << 8) | (pPDUIIRequest->extra & 0xFF)] = PTT_ENTRY_UNKNOWN|(pPDUIIRequest->val & 0xFFFFF);

        if (!SUCCESS(iss = camgo (&pkg_p)))
        {/* Failed, leave as invalid */
            errlogPrintf("camgo error 0x%08X\n",(unsigned int) iss);
            status = (PDUII_CAM_GO_FAIL|iss);
        }
        else
        {/* Succeed, validate it */
            pPDUIIModule->pttCache[(pPDUIIRequest->a << 8) | (pPDUIIRequest->extra & 0xFF)] = (pPDUIIRequest->val & 0xFFFFF);
        }

release_campkg: 

        if (!SUCCESS(iss = camdel (&pkg_p)))
            errlogPrintf("camdel error 0x%08X\n",(unsigned int) iss);
    }
    else
        status = PDUII_MODULE_NOT_EXIST;

egress:
    epicsTimeGetCurrent(&(pPDUIIRequest->actTime));
    pPDUIIRequest->errCode = status;
    pPDUIIRequest->opDone = 1;

    return status;
}

static int PDUII_Operation(void * parg)
{
    int     msgQstatus;
    UINT32  errCode;

    PDUII_REQUEST  *pPDUIIRequest;
    PDUII_MODULE * pPDUIIModule = (PDUII_MODULE *) parg;

    if(pPDUIIModule->msgQId == NULL)
    {
        errlogPrintf("Operation thread quit because no legal PDUIImsgQId!\n");
        return -1;
    }

#if 0
    errCode = PDUII_Reset(pPDUIIModule);
    if(errCode)
    {
        errlogPrintf("Fail to call PDUII_Reset PDUII[%d,%d,%d], error 0x%08X\n", 
            pPDUIIModule->b, pPDUIIModule->c, pPDUIIModule->n, errCode);
        return -1;
    }
#endif

    while(TRUE)
    {
        msgQstatus = epicsMessageQueueReceive(pPDUIIModule->msgQId, &pPDUIIRequest, sizeof(PDUII_REQUEST *) );
        if(msgQstatus < 0)
        {/* we should never time out, so something wrong */
            errlogPrintf("PDUII Operation msgQ timeout!\n");
            epicsThreadSleep(2.0);  /* Avoid super loop to starve CPU */
        }
        else
        {/* some requests come in, we deal it one by one, no dynamic combination */
            if(PDUII_DRV_DEBUG) printf("PDUII Operation task gets requests!\n");

            switch(pPDUIIRequest->funcflag)
            {/* check funcflag */
                case PDUII_BO_RESET:
                    errCode = PDUII_Reset(pPDUIIRequest->pPDUIIModule);

                    pPDUIIRequest->errCode = errCode;
                    pPDUIIRequest->opDone = 1;

                    break;
                case PDUII_BO_UBPLN:
                    errCode = PDUII_UpperBplnEnDis(pPDUIIRequest->pPDUIIModule, pPDUIIRequest->val);

                    pPDUIIRequest->errCode = errCode;
                    pPDUIIRequest->opDone = 1;

                    break;
                case PDUII_BO_SEQR:
                    errCode = PDUII_SeqrEnDis(pPDUIIRequest->pPDUIIModule, pPDUIIRequest->val);

                    pPDUIIRequest->errCode = errCode;
                    pPDUIIRequest->opDone = 1;

                    break;
                case PDUII_MBBID_STATUS:
                    errCode = PDUII_Status(pPDUIIRequest->pPDUIIModule, &(pPDUIIRequest->val));

                    pPDUIIRequest->errCode = errCode;
                    pPDUIIRequest->opDone = 1;

                    break;
                case PDUII_MBBI_MODEGET:
		    PDUII_ModeGet(pPDUIIRequest);
                    break;
                case PDUII_MBBO_MODESET:
		    PDUII_ModeSet(pPDUIIRequest);
                    break;
                case PDUII_LI_PTTGET:
		    PDUII_PTTGet(pPDUIIRequest);
                    break;
                case PDUII_LO_PTTSET:
		    PDUII_PTTSet(pPDUIIRequest);
                    break;

            }/* check funcflag */
            /* process record */
            if(pPDUIIRequest->pRecord)
            {
                dbScanLock(pPDUIIRequest->pRecord);
                (*(pPDUIIRequest->pRecord->rset->process))(pPDUIIRequest->pRecord);
                dbScanUnlock(pPDUIIRequest->pRecord);
            }

        }/* process requests */

    }/* infinite loop */

    /* We should never get here */
    return 0;
}

int PDUIIRequestInit(dbCommon * pRecord, struct camacio inout, enum EPICS_RECTYPE rtyp)
{
    PDUII_MODULE * pPDUIIModule = NULL;
    PDUII_MODULE * pPDUIIModuleTmp = NULL;
    UINT32	PDUIIPos;	/* used to sort linked list, PDUIIPos = B<<9 | C<<5 | N */
    int         funcflag = 0;
    int 	extra = 0;
    int		loop, loopch, looprule;

    PDUII_REQUEST * pPDUIIRequest = NULL;

    /* parameter check */
    if(!pRecord) return -1;

    for(loop=0; loop<N_PARAM_MAP; loop++)
    {
        if( 0 == strncmp(param_map[loop].param, inout.parm, strlen(param_map[loop].param)) )
        {
            if( rtyp == EPICS_RECTYPE_NONE || rtyp == param_map[loop].rtyp)
            {
                funcflag = param_map[loop].funcflag;
                if(strlen(param_map[loop].param) < strlen(inout.parm))
                {
                    extra = atoi(inout.parm + strlen(param_map[loop].param));
                    if(PDUII_DRV_DEBUG) printf("extra is %d for [%s]\n", extra, pRecord->name);
                }
                else
                    extra = 0;

                break;
            }
        }
    }
    if(loop >= N_PARAM_MAP)
    {
        errlogPrintf("Record %s param %s is illegal!\n", pRecord->name, inout.parm);
        return -1;
    }

    /* Check if the PDUII module is already in our list, or else add it */
    /* This should only happen in iocInit, single thread, so no mutex needed */
    pPDUIIModule = findPDUIIModuleByBCN(inout.b, inout.c, inout.n);

    if(!pPDUIIModule)
    {/* Did not find any existing matching PDUII, create one */
	char opTaskName[256];
        pPDUIIModule = callocMustSucceed(1, sizeof(PDUII_MODULE), "calloc buffer for PDUII_MODULE");
        /* no bzero needed due to calloc */

        pPDUIIModule->b = branchOfCrate(inout.c & 0xF); /* SLAC system does not use branch, but we do care PSCD port */
        pPDUIIModule->c = inout.c & 0xF ;	
        pPDUIIModule->n = inout.n & 0x1F;

        /* how many record could send request to PDUII at pduiie time, 1000 should be enough */
        pPDUIIModule->msgQId = epicsMessageQueueCreate(1000, sizeof(struct PDUII_REQUEST *));
        if (pPDUIIModule->msgQId == NULL)
        {/* Fail to create messageQ */
            errlogPrintf("Failed to create messageQ for PDUII[%d,%d,%d] operation!\n",pPDUIIModule->b, pPDUIIModule->c, pPDUIIModule->n);
            epicsThreadSuspendSelf();
        }

        sprintf(opTaskName, "%d-%dPDUII", pPDUIIModule->c, pPDUIIModule->n);
        pPDUIIModule->opTaskId = epicsThreadMustCreate(opTaskName, epicsThreadPriorityMedium, 20480, (EPICSTHREADFUNC)PDUII_Operation, (void *)pPDUIIModule);

        pPDUIIModule->lockRule = epicsMutexMustCreate();
        pPDUIIModule->lockModule = epicsMutexMustCreate();

        /* pPDUIIModule->rules, record autoSaveRestore will init it, by default, we have to disable all */
        for(loopch=0; loopch<N_CHNLS_PER_MODU; loopch++)
            for(looprule=0; looprule<N_RULES_PER_CHNL; looprule++)
	{
            for(loop=0; loop<MAX_EVR_MODIFIER; loop++)
	    {
                pPDUIIModule->rules[loopch][looprule].inclusionMask[loop]=0xFFFFFFFF;
                pPDUIIModule->rules[loopch][looprule].exclusionMask[loop]=0xFFFFFFFF;
	    }
            pPDUIIModule->rules[loopch][looprule].beamCode=0xFF;
            pPDUIIModule->rules[loopch][looprule].pttDelay=0xFFFFF;
        }

        /* pPDUIIModule->chnlMode, record autoSaveRestore will init it */
        for(loop=0; loop<N_CHNLS_PER_MODU*256; loop++)
        {
            pPDUIIModule->pttCache[loop] = PTT_ENTRY_UNKNOWN;
        }

        pPDUIIModule->errorCount = 0;

	/* sort linked list */
        PDUIIPos = (pPDUIIModule->b << 9)|(pPDUIIModule->c << 5)|pPDUIIModule->n;
        for( pPDUIIModuleTmp = (PDUII_MODULE *)ellFirst(&PDUIIModuleList); pPDUIIModuleTmp; pPDUIIModuleTmp = (PDUII_MODULE *)ellNext((ELLNODE *)pPDUIIModuleTmp) )
        {
            if(PDUIIPos < ((pPDUIIModuleTmp->b << 9)|(pPDUIIModuleTmp->c << 5)|pPDUIIModuleTmp->n)) break;
        }
        if(pPDUIIModuleTmp)
        {
            pPDUIIModuleTmp=(PDUII_MODULE *)ellPrevious((ELLNODE *)pPDUIIModuleTmp);
            ellInsert( &PDUIIModuleList, (ELLNODE *)pPDUIIModuleTmp, (ELLNODE *)pPDUIIModule );
        }
        else
        {/* Did not find anyone has bigger pos, so append at the end. True for empty list as well */
            ellAdd(&PDUIIModuleList, (ELLNODE *)pPDUIIModule);
        }

        if(PDUII_DRV_DEBUG) printf("Add PDUII[%d,%d,%d]\n",
            pPDUIIModule->b, pPDUIIModule->c, pPDUIIModule->n);
    }
    /* Done check if the PDUII module is already in our list, or else add it */

    /* Request info prepare */
    pPDUIIRequest = (PDUII_REQUEST *)callocMustSucceed(1, sizeof(PDUII_REQUEST), "calloc PDUII_REQUEST");
    /* no bzero needed due to calloc */

    pPDUIIRequest->pPDUIIModule = pPDUIIModule;
    pPDUIIRequest->pRecord = pRecord;

    pPDUIIRequest->a = inout.a & 0xF;	/* We are using a to indicate channel number here. So up to 15 */
    pPDUIIRequest->f = inout.f & 0x1F;	/* not used here */

    pPDUIIRequest->funcflag = funcflag;

    if(funcflag == PDUII_LI_PTTGET || funcflag == PDUII_LO_PTTSET)
        pPDUIIRequest->extra = (extra&0xFF);
    else if(funcflag == PDUII_LO_DELAY || funcflag == PDUII_WF_RULE)
        pPDUIIRequest->extra = min(extra,N_RULES_PER_CHNL-1);
    else
        pPDUIIRequest->extra = extra;

    /*pPDUIIRequest->actTime*/
    pPDUIIRequest->val = 0;
    pPDUIIRequest->errCode = PDUII_REQUEST_NO_ERR;
    pPDUIIRequest->opDone = 0;

    pRecord->dpvt = (void *)pPDUIIRequest;

    return 0;
}


/**************************************************************************************************/
/* Here we supply the driver report function for epics                                            */
/**************************************************************************************************/
static  long    PDUII_EPICS_Init();
static  long    PDUII_EPICS_Report(int level);

const struct drvet drvPDUII = {2,                              /*2 Table Entries */
                             (DRVSUPFUN) PDUII_EPICS_Report,  /* Driver Report Routine */
                             (DRVSUPFUN) PDUII_EPICS_Init};   /* Driver Initialization Routine */

#if EPICS_VERSION>=3 && EPICS_REVISION>=14
epicsExportAddress(drvet,drvPDUII);
#endif

/* implementation */
static long PDUII_EPICS_Init()
{

    ellInit(&PDUIIModuleList);

    return 0;
}

static long PDUII_EPICS_Report(int level)
{
    PDUII_MODULE  * pPDUIIModule;

    printf("\n"PDUII_DRV_VER_STRING"\n\n");

    if(level > 0)   /* we only get into link list for detail when user wants */
    {
        for(pPDUIIModule=(PDUII_MODULE *)ellFirst(&PDUIIModuleList); pPDUIIModule; pPDUIIModule = (PDUII_MODULE *)ellNext((ELLNODE *)pPDUIIModule))
        {
            printf("\tPDUII Module at b[%d]c[%d]n[%d]: \n", pPDUIIModule->b, pPDUIIModule->c, pPDUIIModule->n);
            if(level > 1)
            {
                /* More Info */
                printf("\tError (number of missed channels) count is [%d] \n", pPDUIIModule->errorCount);
            }
            printf("\n");
        }
    }

    return 0;
}

