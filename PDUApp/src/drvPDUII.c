/***************************************************************************\
 *   $Id: drvPDUII.c,v 1.15 2012/05/09 19:52:54 luchini Exp $
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
#include "cctlwmasks.h"
#include "camdef.h"          /* for CAM_READ_MISMATCH */

extern struct PSCD_CARD pscd_card;

int PDUII_DRV_DEBUG = 0;
static const UINT32 XM2QM1 = (CCTLW__XM2|CCTLW__QM1);  /* terminate X=0,Q=1 */



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

        /* 360Hz task will try lock to detect if module is going thru reset */
        epicsMutexMustLock(pPDUIIModule->lockModule);

        for(loop=0; loop<N_CHNLS_PER_MODU*256; loop++) pPDUIIModule->pttCache[loop] = PTT_ENTRY_UNKNOWN|0xFFFFF;

        /* F9A0 to reset, ignore-overwrite whatever a,f in record */
        pduiictlw = (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (9 << 16) | 0;
	bcnt = 0;
        if (!SUCCESS(iss = camio (&pduiictlw, NULL, &bcnt, &(cmd_pduii.stat), &emask)))
        {/* Reset failed, Leave all pttCache invalidated. Unlock module. */
            epicsMutexUnlock(pPDUIIModule->lockModule); 
            errlogPrintf ("camio error %s for PDUII reset\n", cammsg(iss));
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

        /* F24A1 to disable and F26A1 to enable, ignore-overwrite whatever a,f in record */
	if(enable)
            pduiictlw = (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (26 << 16) | 1;
	else
            pduiictlw = (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (24 << 16) | 1;

	bcnt = 0;
        if (!SUCCESS(iss = camio (&pduiictlw, NULL, &bcnt, &(cmd_pduii.stat), &emask)))
        {
            errlogPrintf ("camio error %s for PDUII enable-disable output\n", cammsg(iss));
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

        /* F24A2 to disable and F26A2 to enable, ignore-overwrite whatever a,f in record */
	if(enable)
            pduiictlw = (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (26 << 16) | 2;
	else
            pduiictlw = (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (24 << 16) | 2;

	bcnt = 0;
        if (!SUCCESS(iss = camio (&pduiictlw, NULL, &bcnt, &(cmd_pduii.stat), &emask)))
        {
            errlogPrintf ("camio error %s for PDUII enable-disable sequencer\n", cammsg(iss));
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

	STAS_SDAT read_pduii = {0,0};
        UINT32 pduiictlw = 0x0;
        UINT16 bcnt = 2; 
        UINT16 emask= 0xF2F2;   /* 0xE0E0; */

        /* F2A2 to read status, ignore-overwrite whatever a,f in record   */
        /* Note that when the PDU (not PPDU) is                           */
        /* busy and incapable of accepting or returning data, the PDU     */
        /* usually (but not always) indicates this condition by returning */
        /* Q = 0.  When this happens for the F2A2, the MBCD returns 0 as  */
        /* the status register contents, but the PDU resets the status    */
        /* register nonetheless                                           */
        pduiictlw = (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (2 << 16) | 2;

	bcnt = 2;
        if (!SUCCESS(iss = camio(&pduiictlw, &(read_pduii.sdata), &bcnt, &(read_pduii.stat), &emask)))
        {
            errlogPrintf ("camio error %s for PDUII enable-disable sequencer\n", cammsg(iss));
            return (PDUII_STS_CAMIO_FAIL|iss);
        }
        if(pStatus) *pStatus = read_pduii.sdata & 0xFF;
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
    dbCommon     * pRecord  = pPDUIIRequest->pRecord;
    PDUII_MODULE * pPDUIIModule = pPDUIIRequest->pPDUIIModule;
    if(!pPDUIIRequest || !pPDUIIModule) return -1;

    /* check if module exists */
    if(isModuleExsit(pPDUIIModule->b, pPDUIIModule->c, pPDUIIModule->n))
    {
        void *pkg_p;  /* A camac package */
        vmsstat_t iss;
        UINT16 bcnt = 4;
        UINT16 emask = 0xF3F3;   /* 0xE0E0 */

        UINT32 ctlwF17A0 = 0x0;
        UINT32 ctlwF1A0  = 0x0;

        STAS_SDAT write_pduii = {0,0};
        STAS_SDAT read_pduii[2] = {{0,0},{0,0}};   /* need to set PTTP first, then double read mode */
        UINT16    nops = 3;
        UINT16    read1, read2;    /* double read of mode */

        /** Allocate package for PDUII mode read */
        if (!SUCCESS(iss = camalo (&nops, &pkg_p)))
        {
            errlogPrintf("camalo error %s\n",cammsg(iss));
            status = (PDUII_CAM_ALLOC_FAIL|iss);
            goto egress;
        }

	/* Write PTTP with channel number, location (PP/YY) field is 0 */
        ctlwF17A0 = (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (17 << 16) | 0 | XM2QM1;
	bcnt = 2;
        write_pduii.sdata = pPDUIIRequest->a << 8;
        if (!SUCCESS(iss = camadd (&ctlwF17A0, &write_pduii, &bcnt, &emask, &pkg_p)))
        {
            errlogPrintf("camadd error %s\n",cammsg(iss));
            status = (PDUII_CAM_ADD_FAIL|iss);
            goto release_campkg;
        }

        /*
	** Read the pp/yy mode. We do a double read and ignore the result if they don't match. The PDU
	** has a bug which returns 0 ~1% of the time.
	*/
        ctlwF1A0 = (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (1 << 16) | 0 | XM2QM1;
        bcnt = 2;
        if (!SUCCESS(iss = camadd (&ctlwF1A0, &read_pduii[0], &bcnt, &emask, &pkg_p)))
        {
            errlogPrintf("camadd error %s\n",cammsg(iss));
            status = (PDUII_CAM_ADD_FAIL|iss);
            goto release_campkg;
        }

        /* Issue a second mode read */
        if (!SUCCESS(iss = camadd (&ctlwF1A0, &read_pduii[1], &bcnt, &emask, &pkg_p)))
        {
            errlogPrintf("camadd error %s\n",cammsg(iss));
            status = (PDUII_CAM_ADD_FAIL|iss);
            goto release_campkg;
        }

        if (!SUCCESS(iss = camgo (&pkg_p)))
        {
	    errlogPrintf("drvPDUII camgo error %s reading mode.\n",cammsg(iss));
            status = (PDUII_CAM_GO_FAIL|iss);
            goto release_campkg;
        }
        else
        {
	    read1 = ((read_pduii[0].sdata >>12) & 0x7);
	    read2 = ((read_pduii[1].sdata >>12) & 0x7);
            if ( read1!=read2 )
	    {
              errlogPrintf("drvPDUII failed mode read (rd1=0x%4.4hx rd2=0x%4.4hx) PDU n=%hd chan %hd %s\n",
			     read1,
                             read2,
			     pPDUIIModule->n,
                             pPDUIIRequest->a,
                             pRecord->name);
	    }
	    pPDUIIRequest->val = read_pduii[1].sdata;            /* last mode read */
            pPDUIIModule->chnlModeRbk[pPDUIIRequest->a] = read2;
        }

release_campkg: 

        if (!SUCCESS(iss = camdel (&pkg_p)))
            errlogPrintf("drvPDUII camdel error %s\n",cammsg(iss));
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
    dbCommon     * pRecord = pPDUIIRequest->pRecord;
    PDUII_MODULE * pPDUIIModule = pPDUIIRequest->pPDUIIModule;

    if(!pPDUIIRequest || !pPDUIIModule) return -1;

    /* check if module exists */
    if(isModuleExsit(pPDUIIModule->b, pPDUIIModule->c, pPDUIIModule->n))
    {
        void *pkg_p=NULL;  /* A camac package */
        vmsstat_t iss;
        UINT16 bcnt = 4;
        UINT16 emask= 0xF3F3;   /* 0xE0E0; */

        UINT32 ctlwF17A0 = 0x0;
        UINT32 ctlwF17A1 = 0x0;
        UINT32 ctlwF1A0  = 0x0;

        STAS_SDAT write_pduii[2] = {{0,0},{0,0}};	/* need to set PTTP first, then read mode */
        STAS_SDAT read_pduii[2]  = {{0,0},{0,0}};       /* read pduii data */
        UINT16    read1,read2;
        UINT16    nops = 4;
    

        /* Allocate package for PDUII mode write and read back */
        if (!SUCCESS(iss = camalo (&nops, &pkg_p)))
        {
            errlogPrintf("camalo error %s\n",cammsg(iss));
            status = (PDUII_CAM_ALLOC_FAIL|iss);
            goto egress;
        }

	/* Write PTTP with channel number, location field is 0 */
        ctlwF17A0 = (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (17 << 16) | 0 | XM2QM1;
	bcnt = 2;
        write_pduii[0].sdata = pPDUIIRequest->a << 8;
        if (!SUCCESS(iss = camadd (&ctlwF17A0, &write_pduii[0], &bcnt, &emask, &pkg_p)))
        {
            errlogPrintf("camadd error %s\n",cammsg(iss));
            status = (PDUII_CAM_ADD_FAIL|iss);
            goto release_campkg;
        }

        /* Set pp/yy mode for channel */
        ctlwF17A1 = (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (17 << 16) | 1 | XM2QM1;
        bcnt = 2;
        write_pduii[1].sdata = pPDUIIRequest->val & 0x7;
        if (!SUCCESS(iss = camadd (&ctlwF17A1, &write_pduii[1], &bcnt, &emask, &pkg_p)))
        {
            errlogPrintf("camadd error %s\n",cammsg(iss));
            status = (PDUII_CAM_ADD_FAIL|iss);
            goto release_campkg;
        }

        /*
	** Double read of the mode to make sure the write has latched.
	*/
        ctlwF1A0 = (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (1 << 16) | 0 | XM2QM1;
        bcnt = 2;
        if (!SUCCESS(iss = camadd (&ctlwF1A0, &read_pduii[0], &bcnt, &emask, &pkg_p)))
        {
            errlogPrintf("camadd error %s\n",cammsg(iss));
            status = (PDUII_CAM_ADD_FAIL|iss);
            goto release_campkg;
        }
        if (!SUCCESS(iss = camadd (&ctlwF1A0, &read_pduii[1], &bcnt, &emask, &pkg_p)))
        {
            errlogPrintf("camadd error %s\n",cammsg(iss));
            status = (PDUII_CAM_ADD_FAIL|iss);
            goto release_campkg;
        }

        /* Each entry is independent, and only one op thread per module, so no need to lock. */
        /* And 360Hz task has higher priority, this will never break into 360Hz task sequence */
        /* So be conservetive, marking as TRANSITING asap will be good enough */
        pPDUIIModule->chnlModeSet[pPDUIIRequest->a] = CHNL_MODE_TRANSITING|(pPDUIIRequest->val & 0x7);
        if (!SUCCESS(iss = camgo (&pkg_p)))
        {/* Fail to set mode, leave as invalid */
            errlogPrintf("drvPDUII camgo error %s setting mode\n",cammsg(iss));
            status = (PDUII_CAM_GO_FAIL|iss);
            goto release_campkg;  
        }
        else
        {   /* Set mode for 360Hz Task, used to set PP0 and PP1 PTTs */
            pPDUIIModule->chnlModeSet[pPDUIIRequest->a] = (pPDUIIRequest->val & 0x7);

            /* Verify that mode set succeed by making sure set point latched.*/
	    read1 = ((read_pduii[0].sdata)>>12) & 0x7;	    /* Strip off the PTTP in R12-1 */
	    read2 = ((read_pduii[1].sdata)>>12) & 0x7; 
            if ((read1!= write_pduii[1].sdata) && (read2!= write_pduii[1].sdata))
	    {/* Mode did not latch, write failed */
	      errlogPrintf("drvPDUII failed to set mode (wt=0x%4.4hx rd1=0x%4.4hx rd2=0x%4.4hx) in PDU n=%hd chan %hd %s\n",
			    write_pduii[1].sdata,
			    read1,
		 	    read2,
			    pPDUIIModule->n,
                            pPDUIIRequest->a,
                            pRecord->name);
              iss = CAM_READ_MISMATCH;
	      status = (PDUII_CAM_GO_FAIL|iss);
              goto release_campkg;
	    }
        }

release_campkg: 

        if (!SUCCESS(iss = camdel (&pkg_p)))
            errlogPrintf("camdel error %s\n",cammsg(iss));
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
        UINT16 emask= 0xF3F3;  /* 0xE0E0; */

        UINT32 ctlwF17A0 = 0x0;
        UINT32 ctlwF0A1 = 0x0;

        STAS_SDAT write_pduii[2]={{0,0},{0,0}}; /* set PTTP first before read */
        STAS_DAT read_pduii[2] = {{0,0},{0,0}};	/* then double read PTT val   */
        UINT16 nops = 4;

       dbCommon     * pRecord  = pPDUIIRequest->pRecord;

        /** Allocate package for PTT read */
        if (!SUCCESS(iss = camalo (&nops, &pkg_p)))
        {
            errlogPrintf("camalo error %s\n",cammsg(iss));
            status = (PDUII_CAM_ALLOC_FAIL|iss);
            goto egress;
        }

	/* Write PTTP with channel number, location field is 0 */
        ctlwF17A0 = (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (17 << 16) | 0 | XM2QM1;
	bcnt = 2;
        *((UINT16 *)(&(write_pduii[0].sdata))) = (pPDUIIRequest->a << 8) | (pPDUIIRequest->extra & 0xFF);
        if(PDUII_DRV_DEBUG) printf("PTTP is 0x[%x]\n", (pPDUIIRequest->a << 8) | (pPDUIIRequest->extra & 0xFF));
        if (!SUCCESS(iss = camadd (&ctlwF17A0, &write_pduii[0], &bcnt, &emask, &pkg_p)))
        {
            errlogPrintf("camadd error %s\n",cammsg(iss));
            status = (PDUII_CAM_ADD_FAIL|iss);
            goto release_campkg;
        }

        /*
	** Read the PTT. We do a double read and ignore the result if they don't match. The PDU
	** has a bug which returns 0 ~1% of the time.
	*/
        ctlwF0A1 = CCTLW__P24 | (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (0 << 16) | 1 | XM2QM1;
        bcnt = 4;
        if (!SUCCESS(iss = camadd (&ctlwF0A1, &read_pduii[0], &bcnt, &emask, &pkg_p)))
        {
            errlogPrintf("camadd error %s\n",cammsg(iss));
            status = (PDUII_CAM_ADD_FAIL|iss);
            goto release_campkg;
        }


	bcnt = 2;
        *((UINT16 *)(&(write_pduii[1].sdata))) = (pPDUIIRequest->a << 8) | (pPDUIIRequest->extra & 0xFF);
        if(PDUII_DRV_DEBUG) printf("PTTP is 0x[%x]\n", (pPDUIIRequest->a << 8) | (pPDUIIRequest->extra & 0xFF));
        if (!SUCCESS(iss = camadd (&ctlwF17A0, &write_pduii[1], &bcnt, &emask, &pkg_p)))
        {
            errlogPrintf("camadd error %s\n",cammsg(iss));
            status = (PDUII_CAM_ADD_FAIL|iss);
            goto release_campkg;
        } 

        /************** Double read ****************/
        bcnt = 4;
        if (!SUCCESS(iss = camadd (&ctlwF0A1, &read_pduii[1], &bcnt, &emask, &pkg_p)))
        {
            errlogPrintf("camadd error %s\n",cammsg(iss));
            status = (PDUII_CAM_ADD_FAIL|iss);
            goto release_campkg;
        }

        if (!SUCCESS(iss = camgo (&pkg_p)))
        {
	    errlogPrintf("drvPDUII camgo error %s reading the PTT\n",cammsg(iss));
            status = (PDUII_CAM_GO_FAIL|iss);
        }
        else if ( (read_pduii[0].data & 0xFFFF) == (read_pduii[1].data & 0xFFFF) )
        {
	    pPDUIIRequest->val = read_pduii[0].data & 0xFFFFF;
            pPDUIIModule->pttCache[(pPDUIIRequest->a << 8) | (pPDUIIRequest->extra & 0xFF)] = pPDUIIRequest->val;
        }
        else
	{
            errlogPrintf("drvPDUII failed pttp read (rd1=0x%8.8hx rd2=0x%8.8hx) PDU n=%hd chan %hd %s\n",
			  read_pduii[0].data,
                          read_pduii[1].data,
			  pPDUIIModule->n,
                          pPDUIIRequest->a,
                          pRecord->name);
	}
release_campkg: 

        if (!SUCCESS(iss = camdel (&pkg_p)))
            errlogPrintf("camdel error %s\n",cammsg(iss));
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
        UINT16 emask= 0xF3F3;  /* 0xE0E0; */

        UINT32 ctlwF17A0 = 0x0;
        UINT32 ctlwF16A1 = 0x0;
        UINT32 ctlwF0A1 = 0x0;

        STAS_DAT write_pduii[2] = {{0,0},{0,0}};	/* need to set PTTP */
        STAS_DAT read_pduii = {0,0};             	/* then read mode   */
        UINT32   read1 = 0;                             /* read data        */
        UINT16 nops = 3;

       dbCommon     * pRecord  = pPDUIIRequest->pRecord;

        /** Allocate package for PDUII mode read */
        if (!SUCCESS(iss = camalo (&nops, &pkg_p)))
        {
            errlogPrintf("camalo error %s\n",cammsg(iss));
            status = (PDUII_CAM_ALLOC_FAIL|iss);
            goto egress;
        }

	/* Write PTTP with channel number, location field is 0 */
        ctlwF17A0 = (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (17 << 16) | 0 | XM2QM1;
	bcnt = 2;
        *((UINT16 *)(&(write_pduii[0].data))) = (pPDUIIRequest->a << 8) | (pPDUIIRequest->extra & 0xFF);
        if(PDUII_DRV_DEBUG) printf("Set PTTP is 0x[%x]\n", (pPDUIIRequest->a << 8) | (pPDUIIRequest->extra & 0xFF));
        if (!SUCCESS(iss = camadd (&ctlwF17A0, &write_pduii[0], &bcnt, &emask, &pkg_p)))
        {
            errlogPrintf("camadd error %s\n",cammsg(iss));
            status = (PDUII_CAM_ADD_FAIL|iss);
            goto release_campkg;
        }

        ctlwF16A1 = CCTLW__P24 | (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (16 << 16) | 1 | XM2QM1;
        bcnt = 4;
        write_pduii[1].data = pPDUIIRequest->val & 0xFFFFF;
        if (!SUCCESS(iss = camadd (&ctlwF16A1, &write_pduii[1], &bcnt, &emask, &pkg_p)))
        {
            errlogPrintf("camadd error %s\n",cammsg(iss));
            status = (PDUII_CAM_ADD_FAIL|iss);
            goto release_campkg;
        }

        /*
        ** Verfiy that PTTP has latched.
        ** Note: The PDU has a bug which returns 0 ~1% of the time.
	*/
        ctlwF0A1 = CCTLW__P24 | (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (0 << 16) | 1 | XM2QM1;
        bcnt = 4;
        if (!SUCCESS(iss = camadd (&ctlwF0A1, &read_pduii, &bcnt, &emask, &pkg_p)))
        {
            errlogPrintf("camadd error %s\n",cammsg(iss));
            status = (PDUII_CAM_ADD_FAIL|iss);
            goto release_campkg;
        }

        /* Never modify 360T reloading channel here, and only one op thread per module, so no need to lock. */
        /* And 360Hz task has higher priority, this will never break into 360Hz task sequence               */
        /* So be conservetive, marking as UNKNOWN asap will be good enough                                  */
        pPDUIIModule->pttCache[(pPDUIIRequest->a << 8) | (pPDUIIRequest->extra & 0xFF)] = PTT_ENTRY_UNKNOWN|(pPDUIIRequest->val & 0xFFFFF);

        if (!SUCCESS(iss = camgo (&pkg_p)))
        {/* Failed, leave as invalid */
            errlogPrintf("drvPDUII camgo error %s writing PTT\n",cammsg(iss));
            status = (PDUII_CAM_GO_FAIL|iss);
        }
        else
        {/* Succeed, validate it */
            pPDUIIModule->pttCache[(pPDUIIRequest->a << 8) | (pPDUIIRequest->extra & 0xFF)] = (pPDUIIRequest->val & 0xFFFFF);
            read1 = read_pduii.data & 0xFFFFF;
            if ( read1 != write_pduii[1].data )
	    {
	      errlogPrintf("drvPDUII failed to set ptt (wt=0x%8.8hx rd=0x%8.8hx) in PDU n=%hd chan %hd %s\n",
			    write_pduii[1].data,
			    read1,
			    pPDUIIModule->n,
                            pPDUIIRequest->a,
                            pRecord->name);
	    }
        }

release_campkg: 

        if (!SUCCESS(iss = camdel (&pkg_p)))
            errlogPrintf("camdel error %s\n",cammsg(iss));
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
        errlogPrintf("Fail to call PDUII_Reset PDUII[%d,%d,%d], error %s\n", 
            pPDUIIModule->b, pPDUIIModule->c, pPDUIIModule->n, cammsg(errCode));
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

        /* pPDUIIModule->chnlModeSet, record autoSaveRestore will init it */
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

