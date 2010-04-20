/***************************************************************************\
 **   $Id: PDUII_360Task.c,v 1.8 2010/04/18 18:29:19 pengs Exp $
 **   File:              PDUII_360Task.c
 **   Author:            Sheng Peng
 **   Email:             pengsh2003@yahoo.com
 **   Phone:             408-660-7762
 **   Company:           RTESYS, Inc.
 **   Date:              03/2010
 **   Version:           1.0
 **
 **   handle high priority actions when fiducial
 **
\****************************************************************************/

/* TODO, check X and Q, cctlwmasks.h:#define MBCD_STAT__Q     0x000010000 */
/* TODO, keep tracking number of errors */

#include "drvPSCDLib.h"
#include "devPDUII.h"
#include "slc_macros.h"
#include "cam_proto.h"
#include "cctlwmasks.h"

extern struct PSCD_CARD pscd_card;
extern ELLLIST PDUIIModuleList;

int PDUII_360T_DEBUG = 0;
epicsExportAddress(int, PDUII_360T_DEBUG);

#define DEFAULT_EVR_TIMEOUT 0.2

#define MAX_PKTS_PER_BRANCH	20
#define MAX_NUM_OF_BRANCH	4

static int PDUIIFidu360Task(void * parg);
static epicsEventId EVRFidu360Event = NULL;

#ifndef vxWorks
static void binvert(char * pBuf, int nBytes)
{
        int loop;
        char temp;
        for(loop=0;loop<nBytes/2;loop++)
        {
                temp = pBuf[loop];
                pBuf[loop] = pBuf[nBytes-1-loop];
                pBuf[nBytes-1-loop] = temp;
        }
        return;
}
#endif

int PDUII360TaskStart()
{/* This funciton will be called in device support init during final round */

    /* scanIoInit(&ioscan); */
    /* need RTEMS native call to set higher priority */
    return (int)(epicsThreadMustCreate("PDUIIFidu360", epicsThreadPriorityMax, 20480, (EPICSTHREADFUNC)PDUIIFidu360Task, NULL));
}

static void EVRFidu360(void *parg)
{/* This funciton will be registered with EVR callback and be called at Fuducial rate*/

    /* This does nothing but send a semaphore to PDUIIFidu360Task */
    /* post event/release sema to wakeup PDUIIFidu360Task here */
    if(EVRFidu360Event) epicsEventSignal(EVRFidu360Event);

    return;
}

static int PDUIIFidu360Task(void * parg)
{
    unsigned int fiduCnt = 0;
    vmsstat_t iss;

    if (!SUCCESS(iss = cam_ini (&pscd_card)))	/* no need, should be already done in PSCD driver */
    {
        errlogPrintf("cam_ini error 0x%08X within PDUII 360Hz task\n",(unsigned int) iss);
        return -1;
    }

    /* Create event and register with EVR */
    EVRFidu360Event = epicsEventMustCreate(epicsEventEmpty);

    epicsThreadSleep(20.0); /* Wait for all save restore to finish */

    /* Register EVRFidu360 */
    evrTimeRegister((FIDUCIALFUNCTION)EVRFidu360, NULL);

    while(TRUE)
    {
        int stat;
        stat = epicsEventWaitWithTimeout(EVRFidu360Event, DEFAULT_EVR_TIMEOUT);
        if(stat != epicsEventWaitOK)
        {
            if(stat == epicsEventWaitTimeout)
            {
                if(PDUII_360T_DEBUG > 3) errlogPrintf("Wait EVR timeout, check timing?\n");
                continue;
            }
            else
            {
                errlogPrintf("Wait EVR Error, what happened? Let's sleep 2 seconds.\n");
                epicsThreadSleep(2.0);
                continue;
            }
        }
        else
        {/* Receive ficudical, do work */
            epicsTimeStamp time_s[3];
            evrModifier_ta modifier_a[3];
            unsigned long  patternStatus[3]; /* see evrPattern.h for values */
            int status[3];

            int resetModulo36Cntr = 0;	/* whenever we see MODULO720_MASK, reset, no hurt */
            int infoNext1OK = 0;
            unsigned long beamCode1 = 0;
            int infoNext2OK = 0;
            unsigned long beamCode2 = 0;

            void *F19pkg_p;
            UINT16 nops = MAX_PKTS_PER_BRANCH * MAX_NUM_OF_BRANCH;

            UINT16 emask= 0xE0E0;
            UINT16 bcnt = 2;

            STAS_DAT stat_data[MAX_PKTS_PER_BRANCH * MAX_NUM_OF_BRANCH];
            unsigned long ctlword[MAX_PKTS_PER_BRANCH * MAX_NUM_OF_BRANCH];

            PDUII_MODULE * pPDUIIModule = NULL;
            int currentBranch = -1;
            int currentCrate = -1;
            int totalPkts = 0;
            int numPktsCurBranch = 0;

            int	loopch, looprule;

	    fiduCnt++;

            /* Read pipeline for info for next pulse and the one after */
            status[1] = evrTimeGetFromPipeline(&time_s[1],  evrTimeNext1, modifier_a[1], &patternStatus[1], 0,0,0);
            status[2] = evrTimeGetFromPipeline(&time_s[2],  evrTimeNext2, modifier_a[2], &patternStatus[2], 0,0,0);

            if(status[1] ==0 && patternStatus[1] == PATTERN_OK)
            {
                infoNext1OK = 1;
                beamCode1 = BEAMCODE(modifier_a[1]);

                if(MODULO720_MASK & modifier_a[1][0])
                {
                    resetModulo36Cntr = 1;
                    /* So print only half hertz */
                    if(PDUII_360T_DEBUG >= 2) errlogPrintf("Got MODULO720,fiduCnt [%d]\n", fiduCnt);
                }
            }

            if(status[2] ==0 && patternStatus[2] == PATTERN_OK)
            {
                infoNext2OK = 1;
                beamCode2 = BEAMCODE(modifier_a[2]);
            }

            if(!infoNext1OK && !infoNext2OK) continue; /* no info, do nothing, EVR driver should report error */
            else
            {/* Get at least good info for one pulse */
		    nops=2;
                if(!SUCCESS(iss = camalo (&nops, &F19pkg_p)))
                {/* Allocate max possible slots */
                    if(PDUII_360T_DEBUG >= 1) errlogPrintf("camaloh error 0x%08X\n",(unsigned int) iss);
                    continue;
                }
            }

            /* Now start to go thru the linked list of modules */
            for( pPDUIIModule = (PDUII_MODULE *)ellFirst(&PDUIIModuleList); pPDUIIModule; pPDUIIModule = (PDUII_MODULE *)ellNext((ELLNODE *)pPDUIIModule) )
            {/* The linked list is sorted b,c,n */
                if(pPDUIIModule->b != currentBranch)
                {/* Catch branch change */
                    currentBranch = pPDUIIModule->b;
                    numPktsCurBranch = 0; /* start new branch, reset the counter for pkt per branch */
                    currentCrate = -1; /* ensure we will start a new crate */
                }/* Catch branch change */

                if(pPDUIIModule->c != currentCrate)
                {/* Catch module change to broadcast F19 */
                    currentCrate = pPDUIIModule->c;
                    /* do F19 */
                    /* broadcast to slot 31 
                       unsigned long ctlwF19A8 = (PDU_F19_CRATE << CCTLW__C_shc) | (31 << CCTLW__M_shc) | CCTLW__F19 | CCTLW__A8;
                       unsigned long ctlwF19A9 = (PDU_F19_CRATE << CCTLW__C_shc) | (31 << CCTLW__M_shc) | CCTLW__F19 | CCTLW__A8 | CCTLW__A1; */
                    if(infoNext1OK)
                    {/* Do F19A8 */
                        if(totalPkts >= MAX_PKTS_PER_BRANCH * MAX_NUM_OF_BRANCH || numPktsCurBranch >= MAX_PKTS_PER_BRANCH)
                        {
                            pPDUIIModule->errorCount++;
                        }
                        else
                        {
                            ctlword[totalPkts] = 0x00130F88|(currentCrate<<12); /* F19A8 */
                            *((UINT16 *)(&(stat_data[totalPkts].data))) = (beamCode1 << 8)|(resetModulo36Cntr?0xff:0);
                            bcnt = 2;
                            if(PDUII_360T_DEBUG >= 1) errlogPrintf("Add F19A8 for module[C%d,N%d] as No.%d packet\n", pPDUIIModule->c, pPDUIIModule->n, totalPkts);
                            if (!SUCCESS(iss = camadd (&(ctlword[totalPkts]), &(stat_data[totalPkts]), &bcnt, &emask, &F19pkg_p)))
                            {
                                if(PDUII_360T_DEBUG >= 1) errlogPrintf("camadd error 0x%08X\n",(unsigned int) iss);
                                goto release_campkg;
                            }
                            totalPkts++;
                            numPktsCurBranch++;
                        }
                    }

                    if(infoNext2OK)
                    {/* Do F19A9 */
                        if(totalPkts >= MAX_PKTS_PER_BRANCH * MAX_NUM_OF_BRANCH || numPktsCurBranch >= MAX_PKTS_PER_BRANCH)
                        {
                            pPDUIIModule->errorCount++;
                        }
                        else
                        {
                            ctlword[totalPkts] = 0x00130F89|(currentCrate<<12); /* F19A9 */
                            *((UINT16 *)(&(stat_data[totalPkts].data))) = (beamCode2 << 8);
                            bcnt = 2;
                            if(PDUII_360T_DEBUG >= 1) errlogPrintf("Add F19A9 for module[C%d,N%d] as No.%d packet\n", pPDUIIModule->c, pPDUIIModule->n, totalPkts);
                            if (!SUCCESS(iss = camadd (&(ctlword[totalPkts]), &(stat_data[totalPkts]), &bcnt, &emask, &F19pkg_p)))
                            {
                                if(PDUII_360T_DEBUG >= 1) errlogPrintf("camadd error 0x%08X\n",(unsigned int) iss);
                                goto release_campkg;
                            }
                            totalPkts++;
                            numPktsCurBranch++;
                        }
                    }
                }/* Catch module change to broadcast F19 */

#if 0
                /* Start to reload this module */
                if(epicsMutexLockOK != epicsMutexTryLock(pPDUIIModule->lockModule))
                {/* someone is resetting this module */
                    continue; /* next module */
                }
                else
                {/* Deal with each module */
                    for(loopch=0; loopch<N_CHNLS_PER_MODU; loopch++)
                    {/* Check each channel */
                        UINT32 pttDelayNew = 0xFFFFF; /*default is disable, user can enforce Rule 7 always match to give default */
                        unsigned long pttLocation = 256;
                        int matched = 0;

                        /* If MODE PP0, match infoNext1 */
                        if(pPDUIIModule->chnlMode[loopch] == CHNL_MODE_PP0 && infoNext1OK)
                        {
                            pttLocation = beamCode1;

                            for(looprule=0; looprule<N_RULES_PER_CHNL; looprule++)
                            {
                                epicsMutexMustLock(pPDUIIModule->lockRule);
                                matched = evrPatternCheck(pPDUIIModule->rules[loopch][looprule].beamCode, 0 /* any TS */,
                                              pPDUIIModule->rules[loopch][looprule].inclusionMask,
                                              pPDUIIModule->rules[loopch][looprule].exclusionMask,
                                              modifier_a[1]);
                                epicsMutexUnlock(pPDUIIModule->lockRule);
                                if(matched)
                                {
                                    pttDelayNew =  pPDUIIModule->rules[loopch][looprule].pttDelay;
                                    if(PDUII_360T_DEBUG >= 1) errlogPrintf("Matched rule[%d] module[C%d,N%d],Channel[%d]\n", looprule, pPDUIIModule->c, pPDUIIModule->n,loopch);
                                    continue; /* stop matching */
                                }
                            }
                        }

                        /* If MODE PP1, match infoNext2 */
                        if(pPDUIIModule->chnlMode[loopch] == CHNL_MODE_PP1 && infoNext2OK)
                        {
                            pttLocation = beamCode2;
                            for(looprule=0; looprule<N_RULES_PER_CHNL; looprule++)
                            {
                                epicsMutexMustLock(pPDUIIModule->lockRule);
                                matched = evrPatternCheck(pPDUIIModule->rules[loopch][looprule].beamCode, 0 /* any TS */,
                                              pPDUIIModule->rules[loopch][looprule].inclusionMask,
                                              pPDUIIModule->rules[loopch][looprule].exclusionMask,
                                              modifier_a[2]);
                                epicsMutexUnlock(pPDUIIModule->lockRule);
                                if(matched)
                                {
                                    pttDelayNew =  pPDUIIModule->rules[loopch][looprule].pttDelay;
                                    if(PDUII_360T_DEBUG >= 1) errlogPrintf("Matched rule[%d] module[C%d,N%d],Channel[%d]\n", looprule, pPDUIIModule->c, pPDUIIModule->n,loopch);
                                    continue; /* stop matching */
                                }
                            }
                        }

                        /* reloading, now pttDelayNew carries eithre matched one or default */
                        /* pttLocation < 256 means info OK, if info is not ok, we don't reload */
                        if(pttLocation < 256 && pttDelayNew != pPDUIIModule->pttCache[loopch*256+pttLocation])
                        {
                            if(totalPkts >= (MAX_PKTS_PER_BRANCH * MAX_NUM_OF_BRANCH - 1) || numPktsCurBranch >= (MAX_PKTS_PER_BRANCH - 1))
                            {/* we need two packets here */
                                pPDUIIModule->errorCount++;
                                continue; /* next module */
                            }

                            ctlword[totalPkts] = (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (17 << 16) | 0;
                            bcnt = 2;
                            *((UINT16 *)(&(stat_data[totalPkts].data))) = (loopch << 8) | (pttLocation & 0xFF);
                            if(PDUII_360T_DEBUG >= 1) errlogPrintf("PTTP 0x%08lX\n", (loopch << 8) | (pttLocation & 0xFF));
                            if (!SUCCESS(iss = camadd (&ctlword[totalPkts], &(stat_data[totalPkts]), &bcnt, &emask, &F19pkg_p)))
                            {
                                epicsMutexUnlock(pPDUIIModule->lockModule);
                                if(PDUII_360T_DEBUG >= 1) errlogPrintf("camadd error 0x%08X\n",(unsigned int) iss);
                                goto release_campkg;
                            }
                            totalPkts++;
                            numPktsCurBranch++;

                            ctlword[totalPkts] = CCTLW__P24 | (pPDUIIModule->n << 7) | (pPDUIIModule->c << 12) | (16 << 16) | 1;
                            bcnt = 4;
                            stat_data[totalPkts].data = pttDelayNew & 0xFFFFF;
                            if (!SUCCESS(iss = camadd (&ctlword[totalPkts], &(stat_data[totalPkts]), &bcnt, &emask, &F19pkg_p)))
                            {
                                epicsMutexUnlock(pPDUIIModule->lockModule);
                                errlogPrintf("camadd error 0x%08X\n",(unsigned int) iss);
                                goto release_campkg;
                            }
                            totalPkts++;
                            numPktsCurBranch++;

                            /* Invalid it during reloading */
                            pPDUIIModule->pttCache[loopch*256+pttLocation] = pttDelayNew|PTT_ENTRY_RELOADING;
                        }
                    }/* Deal with each channel */
                    epicsMutexUnlock(pPDUIIModule->lockModule);
                }/* Deal with each module */
#endif
            }/* go thru the linked list */

	    if(totalPkts > 0)
            {
                errlogPrintf("try camgo %d\n", totalPkts);
                if (!SUCCESS(iss = camgo (&F19pkg_p)))
                {/* Failed, leave as invalid */
                    errlogPrintf("camgo error 0x%08X\n",(unsigned int) iss);
                }
                else
                {/* Succeed, validate it */
                    int loop;
                    for(loop=0; loop<N_CHNLS_PER_MODU*256; loop++)
                        pPDUIIModule->pttCache[loop] &= ~(PTT_ENTRY_RELOADING);
                    errlogPrintf("try camgo ok %d\n", totalPkts);
                }
            }

release_campkg:
                errlogPrintf("try camdel %d\n", totalPkts);
            camdel(&F19pkg_p);
                errlogPrintf("ok camdel %d\n", totalPkts);
            /* scanIoRequest(ioscan); */
        }
    }

    /*Should never return from following call*/
    return(0);
}


