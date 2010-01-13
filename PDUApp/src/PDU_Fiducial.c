/***************************************************************************\
 **   $Id: PDU_Fiducial.c,v 1.4 2009/11/11 01:02:31 pengs Exp $
 **   File:              PDU_Fiducial.c
 **   Author:            Sheng Peng
 **   Email:             pengsh2003@yahoo.com
 **   Phone:             408-660-7762
 **   Company:           RTESYS, Inc.
 **   Date:              09/2009
 **   Version:           1.0
 **
 **   handle high priority actions when fiducial
 **
\****************************************************************************/

/* TODO, check X and Q, cctlwmasks.h:#define MBCD_STAT__Q     0x000010000 */
/* TODO, keep tracking number of errors */

#include "drvPSCDLib.h"
#include "devPDU.h"
#include "slc_macros.h"
#include "cam_proto.h"

#include "evrTime.h"
#include "evrPattern.h"

extern struct PSCD_CARD pscd_card;

int PDU_F19_CRATE = 5;
epicsExportAddress(int, PDU_F19_CRATE);

int PDU_F19_DEBUG = 0;
epicsExportAddress(int, PDU_F19_DEBUG);

#define DEFAULT_EVR_TIMEOUT 0.02

static int PDUFiducialTask(void * parg);

static epicsEventId EVRFiducialEvent = NULL;

int EVRFiducialStart()
{/* This funciton will be called in st.cmd after iocInit() */

    /* Create event and register with EVR */
    EVRFiducialEvent = epicsEventMustCreate(epicsEventEmpty);

    /* scanIoInit(&ioscan); */

    /* need RTEMS native call to set higher priority */
    return (int)(epicsThreadMustCreate("PDUFiducial", epicsThreadPriorityMax, 20480, (EPICSTHREADFUNC)PDUFiducialTask, NULL));
}

void EVRFiducial(void)
{/* This funciton will be registered with EVR callback */

    /* get the current pattern data - check for good status */
    evrModifier_ta modifier_a;
    epicsTimeStamp time_s;
    unsigned long  patternStatus; /* see evrPattern.h for values */

    int status = evrTimeGetFromPipeline(&time_s,  evrTimeCurrent, modifier_a, &patternStatus, 0,0,0);

    /* This is 120Hz. So printf will screw timing */
    if(PDU_F19_DEBUG >= 3) printk("Got fiducial\n");
    /* post event/release sema to wakeup worker task here */
    if(EVRFiducialEvent) epicsEventSignal(EVRFiducialEvent);
    return;
}

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


int PDU_F19(unsigned int crate, unsigned int PP0, unsigned int PP1);
static int PDUFiducialTask(void * parg)
{
    int		loop;
    int		rtncode;

    /* Register EVRFiducial */
    evrTimeRegister((REGISTRYFUNCTION)EVRFiducial);

    while(TRUE)
    {
        int status;
        status = epicsEventWaitWithTimeout(EVRFiducialEvent, DEFAULT_EVR_TIMEOUT);
        if(status != epicsEventWaitOK)
        {
            if(status == epicsEventWaitTimeout)
            {
                if(PDU_F19_DEBUG > 3) errlogPrintf("Wait EVR timeout, check timing?\n");
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
        {
            /* do F19 */
            if(PDU_F19_CRATE)
                PDU_F19(PDU_F19_CRATE, 1, 0);
            if(PDU_F19_DEBUG>=2) printf("Send F19\n");

            /* scanIoRequest(ioscan); */
        }
    }

    /*Should never return from following call*/
    return(0);
}

/**************************************************************************************************/
/* Here we supply the driver report function for epics                                            */
/**************************************************************************************************/
static long     PDU_EPICS_Init();
static  long    PDU_EPICS_Report(int level);

const struct drvet drvPDU = {2,                              /*2 Table Entries */
                              (DRVSUPFUN) PDU_EPICS_Report,  /* Driver Report Routine */
                              (DRVSUPFUN) PDU_EPICS_Init}; /* Driver Initialization Routine */

epicsExportAddress(drvet,drvPDU);

/* implementation */
static long PDU_EPICS_Init()
{
    EVRFiducialStart();
    return 0;
}

static long PDU_EPICS_Report(int level)
{
    printf("\n"PDU_DRV_VER_STRING"\n\n");

    return 0;
}


