/***************************************************************************\
 *   $Id: drvPIOP.c,v 1.1.1.1 2009/02/26 20:08:58 rcs Exp $
 *   File:		drvPIOP.c
 *   Author:		Robert C. Sass
 *   Email:		bsassy@garlic.com
 *   Phone:		408-464-5853
 *   Company:		SLAC
 *   Date:		02/2009
 *   Version:		.01
 *
 *   EPICS driver PIOP; thread that does all of the Camac work 
 *
\***************************************************************************/

#include <devPIOP.h>

/*******************************
** MessageQueue array for thread/PIOP
******************************/
epicsMessageQueueId module_msgQId[MAX_CRATES][MAX_SLOTS];

/**************************************************************************************************/
/* Here we supply the driver initialization & report functions for epics                          */
/**************************************************************************************************/
static  long    PIOP_EPICS_Init();
static  long    PIOP_EPICS_Report(int level);

const struct drvet drvPIOP = {2,                              /*2 Table Entries */
                             (DRVSUPFUN) PIOP_EPICS_Report,  /* Driver Report Routine */
                             (DRVSUPFUN) PIOP_EPICS_Init};   /* Driver Initialization Routine */

epicsExportAddress(drvet,drvPIOP);

/********************************
 ** One time global driver init
 *******************************/
static long PIOP_EPICS_Init()
{
   /* 
   ** Init array of msgQ pointers for each thread/PIOP
   */ 
   int c,s;
   for (c=0; c<MAX_CRATES; c++)
      for (s=0; s<MAX_SLOTS; s++) 
         module_msgQId[c][s] = NULL;
   return 0;
}


static long PIOP_EPICS_Report(int level)
{
   printf ("Entered PIOP_EPICS_Report level %d\n", level);
   return 0;
}

/*****************************************************
 ** Driver init for each record type called by devPIOP
 ****************************************************/

int PIOPDriverInit (dbCommon *pRec, struct camacio inout, enum EPICS_RECTYPE rtyp)
{
   PIOP_PVT *DrvPvt;
   /*---------------------------------------------------*/ 
   DrvPvt = callocMustSucceed (1, sizeof(PIOP_PVT), "calloc driver pvt");
   DrvPvt->status = KLYS_OKOK;   /* Init status to good */
   pRec->dpvt = (void *)DrvPvt;
   return 0;
}

/****************************************************************
** This is the driver thread/PIOP that does all of the Camac work
*****************************************************************/
void threadPIOP (void * msgQId)
{
   epicsMessageQueueId lmsgQ = msgQId;
   THREADMSG_TS msg;
   int msgQstat;
   /*----------------------------*/
   printf ("Entered PIOP module thread\n");
   while (TRUE)
   {
      if ((msgQstat = epicsMessageQueueReceive (lmsgQ, &msg, sizeof(THREADMSG_TS)) < 0))
      {
         errlogSevPrintf(errlogFatal,"PIOP msgQ timeout for thread %s status %d. Suspending...\n", 
                         epicsThreadGetNameSelf(), msgQstat);
         epicsThreadSuspendSelf();
      }
 
      dbCommon *reccom_p = msg.rec_p;    /* Record pointer for return to device support */
      PIOP_PVT *ppvt_p = reccom_p->dpvt; /* Local routines only know about driver private */

      switch (msg.func_e)
      {
         case IPL:
         {
            IPLSTRUC_TS iplmsg = msg.parm_u.ipl;
            printf ("In thread received IPL msg func = %d crate %d slot %d file %s\n", 
                     msg.func_e, iplmsg.crate, iplmsg.slot, iplmsg.fname);
            iplPIOPMain (ppvt_p, iplmsg.fname, iplmsg.crate, iplmsg.slot);
            dbScanLock(reccom_p);
            (*(reccom_p->rset->process))(reccom_p);
            dbScanUnlock(reccom_p);
            break;
         }
         case PPNOFTP:
         {
            IPLSTRUC_TS iplmsg = msg.parm_u.ipl;
            printf ("In thread received PPNOFTP msg func = %d crate %d slot %d \n", 
                     msg.func_e, iplmsg.crate, iplmsg.slot);
            dbScanLock(reccom_p);
            (*(reccom_p->rset->process))(reccom_p);
            dbScanUnlock(reccom_p);
            break;
         }
         default:
            errlogSevPrintf(errlogMinor, "Invalid function %d in threadPIOP\n",msg.func_e);
      }
   }   /* End while (TRUE) */
}
