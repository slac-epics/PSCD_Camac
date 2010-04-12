/***************************************************************************\
 **   $Id: PDUII_360Task.c,v 1.1 2010/01/13 05:58:28 pengs Exp $
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

    /* This is 360Hz. So printk will screw timing */
    if(PDUII_360T_DEBUG >= 3) printk("Got fiducial\n");
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
        int status;
        status = epicsEventWaitWithTimeout(EVRFidu360Event, DEFAULT_EVR_TIMEOUT);
        if(status != epicsEventWaitOK)
        {
            if(status == epicsEventWaitTimeout)
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
            evrModifier_ta modifier_a[3];
            epicsTimeStamp time_s[3];
            unsigned long  patternStatus[3]; /* see evrPattern.h for values */
            int status[3];

            status[1] = evrTimeGetFromPipeline(&time_s[1],  evrTimeNext1, modifier_a[1], &patternStatus[1], 0,0,0);
            status[2] = evrTimeGetFromPipeline(&time_s[2],  evrTimeNext2, modifier_a[2], &patternStatus[2], 0,0,0);

            /* This is 3600Hz. So printf will screw timing */
            if(PDUII_360T_DEBUG >= 4) errlogPrintf("EVR fires\n");

            /* do F19 */
            if(PDUII_F19_CRATE)
                PDUII_F19(PDUII_F19_CRATE, 1, 0);
            if(PDUII_F19_DEBUG>=2) printf("Send F19\n");

            /* scanIoRequest(ioscan); */
        }
    }

    /*Should never return from following call*/
    return(0);
}


    if (!status)
    {/* check for LCLS beam and rate-limiting */
        if ((modifier_a[4] & MOD5_BEAMFULL_MASK) && (modifier_a[4] & rate_mask))
    }

