/***************************************************************************\
 *   $Id: drvCAMCOM.c,v 1.6 2010/02/25 22:41:26 rcs Exp $
 *   File:		drvCAMCOM.c
 *   Author:		Robert C. Sass
 *   Email:		rcs@slac.stanford.edu
 *   Phone:		408-464-5853
 *   Company:		SLAC
 *   Date:		04/2010
 *   Version:		.01
 *
 *   EPICS driver CAMCOM; thread that does all of the Camac work 
 *
\***************************************************************************/

#include <devCAMCOM.h>

/**************************************
** MessageQueue for CAMCOM thread. One/IOC
***************************************/
epicsMessageQueueId camcom_msgQId;

/********************************************************************************************/
/* Here we supply the driver initialization & report functions for epics                    */
/********************************************************************************************/
static  long    CAMCOM_EPICS_Init();
static  long    CAMCOM_EPICS_Report(int level);

const struct drvet drvCAMCOM = {2,                             /*2 Table Entries */
                             (DRVSUPFUN) CAMCOM_EPICS_Report,  /* Driver Report Routine */
                             (DRVSUPFUN) CAMCOM_EPICS_Init};   /* Driver Initialization Routine */

epicsExportAddress(drvet,drvCAMCOM);


/********************************
 ** One time global driver init
 *******************************/
static long CAMCOM_EPICS_Init()
{
   /*-------------------------------*/
   /*
   ** Create the CAMCOM msgQ
   */
   if ((camcom_msgQId = epicsMessageQueueCreate (10,sizeof(THREADMSG_TS))) == NULL)
   {
      errlogSevPrintf(errlogFatal,
         "Failed to create CAMCOM message queue. Bummer.\n");
      goto egress;
   }
   epicsThreadMustCreate("CAMCOM", epicsThreadPriorityMedium, 20480,
                          threadCAMCOM, (void *)camcom_msgQId);
egress:
   return 0;
}


static long CAMCOM_EPICS_Report(int level)
{
   printf ("\nCAMAC CAMCOM Driver V1.0\n");
   return 0;
}

/*****************************************************
 ** Driver init for each record type called by devCAMCOM
 ****************************************************/

void CAMCOMDriverInit (dbCommon *pRec, enum EPICS_RECTYPE rtyp)
{
   CAMCOM_PVT *pvt_p;
   /*---------------------------------------------------*/ 
   pvt_p = callocMustSucceed (1, sizeof(CAMCOM_PVT), "calloc driver pvt");
   pvt_p->status = CAM_OKOK;   /* Init status to good */
   pRec->dpvt = (void *)pvt_p;
   return;
}

/*******************************************************************
** This is the driver thread that does all of the CAMCOM Camac work.
********************************************************************/
void threadCAMCOM (void * msgQId)
{
   vmsstat_t iss = CAM_OKOK;
   epicsMessageQueueId lmsgQ = msgQId;
   THREADMSG_TS msg_s;
   int msgQstat;
   mbcd_pkg_ts *wfpkg_p;  /* package in waveform */
   void *newpkg_p;        /* Created camcom package */
   unsigned short iops;   /* #ops in package */
   unsigned short emask = 0xFFFF; /* emask to return and report everything */
   unsigned short nbytes; /* #bytes this packet */
   mbcd_pkt_ts *wfpkt_p;  /* Packet pointer */
   char *wfdat_p;         /* Pointer to stat/data */
   int j;
   CAMCOM_PVT *pvt_p;    /* Driver private struct */
   dbCommon *reccom_p;   /* Record pointer */
   /*----------------------------*/
   while (TRUE)
   {
      if ((msgQstat = epicsMessageQueueReceive (lmsgQ, &msg_s, sizeof(THREADMSG_TS)) < 0))
      {
         errlogSevPrintf(errlogFatal,"CAMCOM msgQ timeout status %d. Suspending...\n", msgQstat);
         epicsThreadSuspendSelf();
      }
      reccom_p = msg_s.rec_p;   /* Record pointer for return to device support */
      pvt_p = (CAMCOM_PVT *)reccom_p->dpvt;   /* Local routines only know about driver private */
      /*****************************
       ** Build and execute package.
       ****************************/ 
      printf ("Entered CAMCOM thread\n");
      wfpkg_p = (mbcd_pkg_ts*) pvt_p->val_p;
      iops = wfpkg_p->hdr.iop;
      if (!SUCCESS(iss = camalo(&iops, &newpkg_p)))
	 goto egress;
      for (j=0; j<iops; j++)
      {
	 wfpkt_p = &(wfpkg_p->mbcd_pkt[j]);
         wfdat_p = (char *) wfpkg_p + (int) wfpkt_p->stad_p;
         nbytes =  wfpkt_p->wc_max << 1;
         if (!SUCCESS(iss = camadd(&(wfpkt_p->cctlw), &wfdat_p, &nbytes, &emask, &newpkg_p)))
	    goto egress; 
      }
      iss = camgo (&newpkg_p); 
   egress:
      pvt_p->status = iss;
      dbScanLock(reccom_p);
      (*(reccom_p->rset->process))(reccom_p);
      dbScanUnlock(reccom_p);
   }   /* End while (TRUE) */
}      /* End threadCAMCOM */
