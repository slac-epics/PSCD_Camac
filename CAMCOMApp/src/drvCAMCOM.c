/***************************************************************************\
 *   $Id: drvCAMCOM.c,v 1.2 2010/07/02 17:42:21 rcs Exp $
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

#ifndef USE_TYPED_DRVET
const struct drvet drvCAMCOM = {2,                             /*2 Table Entries */
                             (DRVSUPFUN) CAMCOM_EPICS_Report,  /* Driver Report Routine */
                             (DRVSUPFUN) CAMCOM_EPICS_Init};   /* Driver Initialization Routine */
#else
const drvet drvCAMCOM = {2, CAMCOM_EPICS_Report, CAMCOM_EPICS_Init};
#endif

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
   void *newpkg_p = NULL; /* Created camcom package */
   unsigned short iops;   /* #ops in package */
   unsigned short emask = 0; /* emask to return and report nothing */
   unsigned short nbytes; /* #bytes this packet */
   mbcd_pkt_ts *wfpkt_p;  /* Packet pointer */
   short *wfdat_p;        /* Pointer to stat/data */
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
      wfpkg_p = (mbcd_pkg_ts*) pvt_p->val_p;  /* Get package pointer */
#ifndef _X86_
      camSwapBytes (wfpkg_p, sizeof(mbcd_pkghdr_ts)); /* byte swap header */
#endif
      iops = wfpkg_p->hdr.iop;
      if (!SUCCESS(iss = camalo(&iops, &newpkg_p)))
	 goto egress;
      for (j=0; j<iops; j++)
      {
         wfpkt_p = &(wfpkg_p->mbcd_pkt[j]);
#ifndef _X86_
         camSwapBytes ( wfpkt_p, sizeof(*wfpkt_p));                     /* Byte swap packet */ 
         camSwapWords ( &(wfpkt_p->cctlw), sizeof(wfpkt_p->cctlw)/2 );  /* Word swap ctlw */ 
         camSwapWords ( &(wfpkt_p->stad_p), sizeof(wfpkt_p->stad_p)/2); /* Word swap stad_p */
#endif
         wfdat_p = (short *) wfpkg_p + (int) wfpkt_p->stad_p;
         nbytes =  wfpkt_p->wc_max << 1;
#ifndef _X86_
            camSwapBytes (wfdat_p+2, nbytes);     /* Byte swap data */ 
            camSwapWords (wfdat_p+2, nbytes/2 );  /* Word swap data */ 
#endif
         if (!SUCCESS(iss = camadd(&(wfpkt_p->cctlw), wfdat_p, &nbytes, &emask, &newpkg_p)))
	    goto egress; 
      }
      iss = camgo (&newpkg_p);
      /************** I/O Complete *********/
#ifndef _X86_
      /** Swap everything back  **/
      camSwapBytes (wfpkg_p, sizeof(mbcd_pkghdr_ts)); /* byte swap header back */
      for (j=0; j<iops; j++)
      {
         wfpkt_p = &(wfpkg_p->mbcd_pkt[j]);
         wfdat_p = (short *) wfpkg_p + (int) wfpkt_p->stad_p;
         nbytes =  wfpkt_p->wc_max << 1;
         camSwapBytes ( wfpkt_p, sizeof(*wfpkt_p));                     /* Byte swap packet */ 
         camSwapWords ( &(wfpkt_p->cctlw), sizeof(wfpkt_p->cctlw)/2 );  /* Word swap ctlw */ 
         camSwapWords ( &(wfpkt_p->stad_p), sizeof(wfpkt_p->stad_p)/2); /* Word swap stad_p */
         camSwapBytes (wfdat_p, nbytes+4);       /* Byte swap stat/data */ 
         camSwapWords (wfdat_p, (nbytes+4)/2 );  /* Word swap stat/data */
      }
#endif
   egress:
      camdel (&newpkg_p);
      pvt_p->status = iss;
      dbScanLock(reccom_p);
      (*(reccom_p->rset->process))(reccom_p);
      dbScanUnlock(reccom_p);
   }   /* End while (TRUE) */
}      /* End threadCAMCOM */

