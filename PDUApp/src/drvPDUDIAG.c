/***************************************************************************\
 *   $Id: drvPDUDIAG.c,v 1.4 2010/11/09 20:04:01 luchini Exp $
 *   File:		drvPDUDIAG.c
 *   Author:		Robert C. Sass
 *   Email:		rcs@slac.stanford.edu
 *   Phone:		408-464-5853
 *   Company:		SLAC
 *   Date:		08/2010
 *   Version:		1.0
 *
 *   EPICS driver for PDUDIAG 
 *
\***************************************************************************/
#include "devPDUDIAG.h"
#include "cctlwmasks.h"

/*********************************************************************************/
/*********************     Shared between dev/drv       **************************/
/*********************************************************************************/

int PDUDIAG_DRV_DEBUG = 0;

extern epicsMessageQueueId fidThread_msgQId;
extern PDUDIAG_WFD Wfd_s;    /* defined in fidPDUDIAG; 360 Hz Waveform data collected */

/*********************************************************************************/
/***   Shared between this thread and 360PDUDIAG_Fid data collection.  ***********/
/*********************************************************************************/

int          fidPDUDIAGActive = 0;     /* Set true to start data collection */
epicsEventId fidCollectionDone = NULL; /* fiducial data collection signals done */

/****************************************************************
** This is the driver thread that does all of the Communication
** with the fiducial data collection that runs as part of the 360Hz 
** PDU processing
*****************************************************************/

#define DIAG_TIMEOUT 4.2

void fidThread(void * msgQId)
{
   epicsMessageQueueId lmsgQ = msgQId;
   THREADMSG msg_s;
   int stat;
   dbCommon *reccom_p; /* Record pointer */
   /*----------------------------*/
   fidCollectionDone = epicsEventMustCreate(epicsEventEmpty); /* for 360 to signal done */
   while(TRUE)
   {
      if ((stat = epicsMessageQueueReceive (lmsgQ, &msg_s, sizeof(THREADMSG)) < 0))
      {
         errlogSevPrintf(errlogFatal,"PDUDIAG msgQ timeout for thread %s status %d. Suspending...\n",
                         epicsThreadGetNameSelf(), stat);
         epicsThreadSuspendSelf();
      }
      reccom_p = msg_s.rec_p;  /* Record pointer */
      /*
      ** Set crate, channel and flag to start data collection.
      ** Wait for event signaling that it's done.
      */
      Wfd_s.crate = msg_s.crate;
      Wfd_s.channel = msg_s.channel;
      fidPDUDIAGActive = 1;
      stat = epicsEventWaitWithTimeout(fidCollectionDone, DIAG_TIMEOUT);
      if(stat != epicsEventWaitOK)
      {
         errlogPrintf("drvPDUDIAG error %d waiting for data collection event\n", stat);
         Wfd_s.status = DIAG_EVENTERR;         
      }
      /*
      ** Finish record processing
      */
      dbScanLock(reccom_p);
      memcpy (msg_s.wfb_p, &Wfd_s, sizeof(PDUDIAG_WFD)); /* Copy whatever data we've collected */
      (*(reccom_p->rset->process))(reccom_p);
      dbScanUnlock(reccom_p);
   }
   /* We should never get here */
   return;
}

/**************************************************************************************************/
/* Here we supply the driver report function for epics                                            */
/**************************************************************************************************/
static  long    PDUDIAG_EPICS_Init();
static  long    PDUDIAG_EPICS_Report(int level);

const struct drvet drvPDUDIAG = {2,                              /*2 Table Entries */
                             (DRVSUPFUN) PDUDIAG_EPICS_Report,  /* Driver Report Routine */
                             (DRVSUPFUN) PDUDIAG_EPICS_Init};   /* Driver Initialization Routine */

#if EPICS_VERSION>3 || (EPICS_VERSION==3 && EPICS_REVISION>=14)
epicsExportAddress(drvet,drvPDUDIAG);
#endif

/* implementation */
static long PDUDIAG_EPICS_Init()
{
   return 0;
}

static long PDUDIAG_EPICS_Report(int level)
{
    printf("\n"PDUDIAG_DRV_VER_STRING"\n\n");
    if(level > 0)   /* we only get into link list for detail when user wants */
    {
      printf ("PDUDIAG Level %d\n",level);
    }
    return 0;
}

