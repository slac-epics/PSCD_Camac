/***************************************************************************\
 **   $Id: PDUII_360Task.c,v 1.2 2010/04/13 00:17:14 pengs Exp $
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

    static unsigned int tsmod360 = 0; /* TODO */

    /* TODO, when to install EVR callback? */


#include "drvPSCDLib.h"
#include "devPDUII.h"
#include "slc_macros.h"
#include "cam_proto.h"

extern struct PSCD_CARD pscd_card;

int PDUII_360T_DEBUG = 0;
epicsExportAddress(int, PDUII_360T_DEBUG);

#define DEFAULT_EVR_TIMEOUT 0.02

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
    int		loop;
    int		rtncode;

    /* Create event and register with EVR */
    EVRFidu360Event = epicsEventMustCreate(epicsEventEmpty);

    epicsThreadSleep(20.0); /* Wait for all save restore to finish */

    /* Register EVRFidu360 */
    evrTimeRegister((REGISTRYFUNCTION)EVRFidu360, NULL);

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
            vmsstat_t iss;

            UINT16 emask= 0xE0E0;
            UINT16 bcnt = 2;

            STAS_DAT stat_data[MAX_PKTS_PER_BRANCH * MAX_NUM_OF_BRANCH];
            unsigned long ctlword[MAX_PKTS_PER_BRANCH * MAX_NUM_OF_BRANCH];

            PDUII_MODULE * pPDUIIModule = NULL;
            int currentBranch = -1;
            int currentCrate = -1;
            int totalPkts = 0;
            int numPktsCurBranch = 0;

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
                    if(PDUII_360T_DEBUG >= 2) errlogPrintf("Got MODULO720\n");
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
                if(!SUCCESS(iss = camaloh (&nops, &F19pkg_p)))
                {
                    if(PDUII_360T_DEBUG >= 1) errlogPrintf("camaloh error 0x%08X\n",(unsigned int) iss);
                    continue;
                }
            }

            /* Now start to go thru the linked list of modules */
            for( pPDUIIModule = (PDUII_MODULE *)ellFirst(&PDUIIModuleList); pPDUIIModule; pPDUIIModule = (PDUII_MODULE *)ellNext((ELLNODE *)pPDUIIModule) )
            {/* The linked list is sorted b,c,n */
                if(pPDUIIModule->b != currentBranch)
                {
                    currentBranch = pPDUIIModule->b;
                    numPktsCurBranch = 0; /* start new branch, reset the counter for pkt per branch */
                    currentCrate = -1; /* ensure we will start a new crate */
                }

                if(pPDUIIModule->c != currentCrate)
                {
                    currentCrate = pPDUIIModule->c;
                    /* do F19 */
                    /* broadcast to slot 31 
                       unsigned long ctlwF19A8 = (PDU_F19_CRATE << CCTLW__C_shc) | (31 << CCTLW__M_shc) | CCTLW__F19 | CCTLW__A8;
                       unsigned long ctlwF19A9 = (PDU_F19_CRATE << CCTLW__C_shc) | (31 << CCTLW__M_shc) | CCTLW__F19 | CCTLW__A8 | CCTLW__A1; */
                    if(infoNext1Ok)
                    {
                        ctlword[totalPkts] = 0x00130F88|(currentCrate<<12); /* F19A8 */
                        *((UINT16 *)(&(stat_data[totalPkts].data))) = (beamCode1 << 8)|(resetModulo36Cntr?0xff:0);
                        bcnt = 2;
                        if (!SUCCESS(iss = camadd (&ctlword[totalPkts], &(stat_data[totalPkts]), &bcnt, &emask, &F19pkg_p)))
                        {
                            if(PDUII_360T_DEBUG >= 1) errlogPrintf("camadd error 0x%08X\n",(unsigned int) iss);
                            goto release_campkg;
                        }
                        totalPkts++;
                        numPktsCurBranch++;
                    }
                    if(infoNext2Ok)
                    {
                        ctlword[totalPkts] = 0x00130F89|(currentCrate<<12); /* F19A9 */
                        *((UINT16 *)(&(stat_data[totalPkts].data))) = (beamCode2 << 8);
                        bcnt = 2;
                        if (!SUCCESS(iss = camadd (&ctlword[totalPkts], &(stat_data[totalPkts]), &bcnt, &emask, &F19pkg_p)))
                        {
                            if(PDUII_360T_DEBUG >= 1) errlogPrintf("camadd error 0x%08X\n",(unsigned int) iss);
                            goto release_campkg;
                        }
                        totalPkts++;
                        numPktsCurBranch++;
                    }
                }

                /* Deal with each module, channels */
            }

release_campkg:
            camdel(&F19pkg_p);
            /* scanIoRequest(ioscan); */
        }
    }

    /*Should never return from following call*/
    return(0);
}


