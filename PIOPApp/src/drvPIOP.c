/***************************************************************************\
 *   $Id: drvPIOP.c,v 1.1 2009/04/28 06:09:45 pengs Exp $
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
#include <drvPIOP.h>

/************************************
** MessageQueue array for thread/PIOP
************************************/
epicsMessageQueueId piop_msgQId[MAX_CRATES][MAX_SLOTS];

/**************************************
** MessageQueue for SBI thread. One/IOC
***************************************/
epicsMessageQueueId sbi_msgQId;

/**************************************************************************************************/
/* Here we supply the driver initialization & report functions for epics                          */
/**************************************************************************************************/
static  long    PIOP_EPICS_Init();
static  long    PIOP_EPICS_Report(int level);

const struct drvet drvPIOP = {2,                             /*2 Table Entries */
                             (DRVSUPFUN) PIOP_EPICS_Report,  /* Driver Report Routine */
                             (DRVSUPFUN) PIOP_EPICS_Init};   /* Driver Initialization Routine */

epicsExportAddress(drvet,drvPIOP);

/*
** One camac package and data to read all of the PIOP message words.
** These are externed by device support which adds packets for each 
** MSGn record encountered.
*/

void *msgw_pkg_p = NULL;
STAT_DAT16 Piop_Msgs_s[MAX_PIOPS];

/********************************
 ** One time global driver init
 *******************************/
static long PIOP_EPICS_Init()
{
   vmsstat_t iss;
   unsigned short nops = MAX_PIOPS;
   /*-------------------------------*/
   if (!SUCCESS(iss = camalo (&nops, &msgw_pkg_p)))
   {
      errlogSevPrintf(errlogFatal,
         "PIOP global driver init failed to init Camac package with status %x\n", 
          (unsigned int) iss);
   }
   /*
   ** Init msg word status to -1
   */
   int i;
   for (i=0; i<MAX_PIOPS; i++)
      Piop_Msgs_s[i].stat = 0xFFFFFFFF;  
   /* 
   ** Init array of msgQ pointers for each thread/PIOP snd SBI thread
   */ 
   int c,s;
   for (c=0; c<MAX_CRATES; c++)
      for (s=0; s<MAX_SLOTS; s++) 
         piop_msgQId[c][s] = NULL;
   sbi_msgQId = NULL;
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
** for PIOPs. Device support creates a separate thread for each 
** PIOP IPL record detected.
*****************************************************************/
void threadPIOP (void * msgQId)
{
   epicsMessageQueueId lmsgQ = msgQId;
   THREADMSG_TS msg_s;
   int msgQstat;
   CAMBLOCKS_TS *camblocks_ps = NULL;
   vmsstat_t iss = KLYS_OKOK;
   /*----------------------------*/
   camblocks_ps = callocMustSucceed (1, sizeof(CAMBLOCKS_TS), "calloc camac blocks struct");
   while (TRUE)
   {
      if ((msgQstat = epicsMessageQueueReceive (lmsgQ, &msg_s, sizeof(THREADMSG_TS)) < 0))
      {
         errlogSevPrintf(errlogFatal,"PIOP msgQ timeout for thread %s status %d. Suspending...\n", 
                         epicsThreadGetNameSelf(), msgQstat);
         epicsThreadSuspendSelf();
      }
 
      dbCommon *reccom_p = msg_s.rec_p;   /* Record pointer for return to device support */
      PIOP_PVT *ppvt_p = reccom_p->dpvt;  /* Local routines only know about driver private */

      switch (msg_s.func_e)
      {
         case INIT:
         {
            if (!SUCCESS (iss = blockPIOPInit (camblocks_ps, msg_s.crate, msg_s.slot)))
	    {
               errlogSevPrintf(errlogFatal,
                     "PIOP failed to init Camac packages thread %s status %x. Suspending...\n", 
                      epicsThreadGetNameSelf(), (unsigned int) iss);
               epicsThreadSuspendSelf();
	    }
            break;
         }
         case IPL:
         {
            IPLSTRUC_TS iplmsg = msg_s.parm_u.ipl;
            iplPIOPMain (ppvt_p, iplmsg.fname, msg_s.crate, msg_s.slot);
            dbScanLock(reccom_p);
            (*(reccom_p->rset->process))(reccom_p);
            dbScanUnlock(reccom_p);
            break;
         }
         case PPN:
         {
	    ppvt_p->status = blockPIOPCblk (camblocks_ps, PIOP_CBLK_TKBITMAP, 
                        msg_s.parm_u.pp.indat_p);
            dbScanLock(reccom_p);
            (*(reccom_p->rset->process))(reccom_p);
            dbScanUnlock(reccom_p);
            break;
         }
         default:
            errlogSevPrintf(errlogMinor, "Invalid function %d in threadPIOP\n",msg_s.func_e);
      }
   }   /* End while (TRUE) */
}      /* End threadPIOP */


/****************************************************************
** This is the driver thread that does all of the SBI Camac work.
*****************************************************************/
void threadSBI (void * msgQId)
{
   vmsstat_t iss = KLYS_OKOK;
   epicsMessageQueueId lmsgQ = msgQId;
   THREADMSG_TS msg_s;
   int msgQstat;
   void *pkg_p;    /* Camac SBI package */
   void *stspkg_p; /* Camac SBI status only package */
   unsigned short nops;
   unsigned short emaskf3f3 = 0xF3F3;  /* emask */
   unsigned short twobytes  = 2;
   unsigned short fourbytes = 4;
   unsigned short zero      = 0;
   /*
   ** SBI data. First word is status.
   */
   STAT_DAT32 read_delay_s;
   STAT_DAT32 read_status_s, write_scan_s, read_scan_s;
   STAT_DAT16 read_status_only_s;   /* Just for periodic status read func SBISTS */
   /*----------------------------*/
   while (TRUE)
   {
      if ((msgQstat = epicsMessageQueueReceive (lmsgQ, &msg_s, sizeof(THREADMSG_TS)) < 0))
      {
         errlogSevPrintf(errlogFatal,"SBI msgQ timeout status %d. Suspending...\n", msgQstat);
         epicsThreadSuspendSelf();
      }
 
      dbCommon *reccom_p = msg_s.rec_p;   /* Record pointer for return to device support */
      PIOP_PVT *ppvt_p = reccom_p->dpvt;  /* Local routines only know about driver private */
      switch (msg_s.func_e)
      {
         case INIT:
         {
            /*
            ** Construct the Camac packages we'll need.
            */
            nops = 4;
	    iss = camalo(&nops, &pkg_p);
            nops = 1;
            if (SUCCESS(iss))
	       iss = camalo(&nops, &stspkg_p);

            unsigned long ploc = (msg_s.crate << CCTLW__C_shc) | (msg_s.slot << CCTLW__M_shc);
	    /*
	    ** First the status only package
	    */
            unsigned long ctlw = ploc | CCTLW__A1;
            if (SUCCESS(iss))
               iss = camadd(&ctlw, &read_status_only_s, &twobytes, &zero, &stspkg_p);
	    /*
	    ** Now the general package
	    **
	    ** F0 read of delay register
	    */
            iss = camadd(&ploc, &read_delay_s, &fourbytes, &emaskf3f3, &pkg_p);
	    /*
	    ** F0 A1 read status
	    */
            ctlw = ploc | CCTLW__A1;
            if (SUCCESS(iss))
               iss = camadd(&ctlw, &read_status_s, &fourbytes, &emaskf3f3, &pkg_p);
	    /*
	    ** F16 A0 SA write scan
	    */
            ctlw = ploc | CCTLW__F16 | CCTLW__SA;
            if (SUCCESS(iss))
               iss = camadd(&ctlw, &write_scan_s, &fourbytes, &emaskf3f3, &pkg_p);
	    /*
	    ** F0 A0 SA read scan
	    */
            ctlw = ploc | CCTLW__SA;
            if (SUCCESS(iss))
               iss = camadd(&ctlw, &read_scan_s, &fourbytes, &emaskf3f3, &pkg_p);
	    /*
	    ** Suspend us if we can't init Camac package
	    */
	    if (!SUCCESS(iss))
	    {
               errlogSevPrintf(errlogFatal,
                  "SBI thread failed to init Camac packages thread %s status %x. Suspending...\n", 
                   epicsThreadGetNameSelf(), (unsigned int) iss);
               epicsThreadSuspendSelf();
	    }
	    ppvt_p->status = iss;
            break;
         }
         case SBISTS:   /* Execute the SBI status package */
         {
            if (SUCCESS(iss = camgo (&stspkg_p)))
 	    {
               ppvt_p->val = read_status_only_s.data;
#ifndef _X86_
               ppvt_p->val = ppvt_p->val >> 16;
#endif
            }
            ppvt_p->status = iss;
            dbScanLock(reccom_p);
            (*(reccom_p->rset->process))(reccom_p);
            dbScanUnlock(reccom_p);
            break;
         }
         case SBIMSG:  /* Execute the PIOP MSG package that reads meg for all PIOPS */
         {
  	    iss = camgo (&msgw_pkg_p);
            ppvt_p->status = iss;
            dbScanLock(reccom_p);
            (*(reccom_p->rset->process))(reccom_p);
            dbScanUnlock(reccom_p);
            break;
         }
         default:
            errlogSevPrintf(errlogMinor, "Invalid function %d in threadSBI\n",msg_s.func_e);
      }
   }   /* End while (TRUE) */
}      /* End threadSBI */
