/***************************************************************************\
 *   $Id: drvPIOP.c,v 1.11 2011/02/07 15:21:35 rcs Exp $
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

/********************************************************************************************/
/* Here we supply the driver initialization & report functions for epics                    */
/********************************************************************************************/
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

void *Msgw_pkg_p = NULL;
STAT_DAT16 Piop_Msgs_s[MAX_PIOPS];

/*
** One camac package and data to read all of the PIOP phase words.
** These are externed by device support which adds packets for each 
** PHASE AI record encountered.
*/

void *Phase_pkg_p = NULL;
STAT_DAT16 Phase_s[MAX_PIOPS];
IOSCANPVT PhaseIoPvt;

/********************************
 ** One time global driver init
 *******************************/
static long PIOP_EPICS_Init()
{
   vmsstat_t iss;
   unsigned short nops = MAX_PIOPS;
   /*-------------------------------*/
   /*
   ** Allocate message word and phase camac packages.
   */
   if (!SUCCESS(iss = camalo (&nops, &Msgw_pkg_p)))
   {
      errlogSevPrintf(errlogFatal,
         "PIOP_EPICS_Init failed to alloc msg word Camac package with status %x\n", 
          (unsigned int) iss);
   }
   if (!SUCCESS(iss = camaloh (&nops, &Phase_pkg_p)))
   {
      errlogSevPrintf(errlogFatal,
         "PIOP_EPICS_init failed to init phase word Camac package with status %x\n", 
          (unsigned int) iss);
   }
   scanIoInit (&PhaseIoPvt);   /* All phase reads are in one Camac pkg */
   /*
   ** Init msg word status to -1
   */
   int i;
   for (i=0; i<MAX_PIOPS; i++)
   {
      Piop_Msgs_s[i].stat = 0xFFFFFFFF;  
      Piop_Msgs_s[i].data = 0xFFFF;  
   }
   /* 
   ** Init array of msgQ pointers for each thread/PIOP snd SBI thread
   */ 
   int c,s;
   for (c=0; c<MAX_CRATES; c++)
      for (s=0; s<MAX_SLOTS; s++) 
         piop_msgQId[c][s] = NULL;
   /*
   ** Create the SBI msgQ
   */
   if ((sbi_msgQId = epicsMessageQueueCreate (10,sizeof(THREADMSG_TS))) == NULL)
   {
      errlogSevPrintf(errlogFatal,
         "Failed to create SBI message queue. Don't expect much success from here on.\n");
      goto egress;
   }
   epicsThreadMustCreate("SBI", epicsThreadPriorityMedium, 20480,
                          threadSBI, (void *)sbi_msgQId);
egress:
   return 0;
}


static long PIOP_EPICS_Report(int level)
{
   printf ("\nCAMAC PIOP Driver V1.0\n");
   return 0;
}

/*****************************************************
 ** Driver init for each record type called by devPIOP
 ****************************************************/

void PIOPDriverInit (dbCommon *pRec, struct camacio *cam_ps, enum EPICS_RECTYPE rtyp)
{
   PIOP_PVT *pvt_p;
   /*---------------------------------------------------*/ 
   pvt_p = callocMustSucceed (1, sizeof(PIOP_PVT), "calloc driver pvt");
   pvt_p->status = KLYS_OKOK;   /* Init status to good */
   pvt_p->crate = cam_ps->c;
   pvt_p->slot =  cam_ps->n;
   pRec->dpvt = (void *)pvt_p;
   return;
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
   vmsstat_t iss;
   PIOP_PVT *pvt_p;    /* Driver private struct */
   dbCommon *reccom_p; /* Record pointer */
   int first_init = 0; /* Must do init first time */
   unsigned short beam_any[CBLK_LENW-1] = 
                          {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,0xFFFF, 0xFFFF, 0xFFFF, 
                           0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF,0xFFFF };
   unsigned short beam_one[CBLK_LENW-1] =    { 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
   unsigned short beam_noftp [CBLK_LENW-1] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
   unsigned short beam_loaded = 99;  /* 0=ANY 1=LCLS Beam. Force reload */
   FTP_WAVE_TS *ftp_wave_ps;
   FTP_CBLK_TS *ftp_cblk_ps;
   FTP_INFO_TS *ftp_info_ps;
   FTP_READ_TS *ftp_read_ps;
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
      reccom_p = msg_s.rec_p;  /* Record pointer */
      pvt_p = (PIOP_PVT *)reccom_p->dpvt;  /* Driver private pointer */
      /*************************************
      ** Check for first time initialization.
      **************************************/
      if (!first_init)
      {
         if (!SUCCESS (iss = blockPIOPInit (camblocks_ps, pvt_p->crate, pvt_p->slot)))
         {
            errlogSevPrintf(errlogFatal,
                  "PIOP failed to init Camac packages thread %s status %x. Suspending...\n", 
                   epicsThreadGetNameSelf(), (unsigned int) iss);
            epicsThreadSuspendSelf();
	 }
         first_init = 1;
      }
      /***************************************
      ** Execute Camac function from driver private struct.
      ***************************************/ 
      switch (pvt_p->camfunc_e)
      {
         case IPL:
         {
	    iss = iplPIOPMain (pvt_p, pvt_p->crate, pvt_p->slot);
            if (SUCCESS(iss))
	    {
               iss = blockPIOPCblk (camblocks_ps, PIOP_CBLK_TKBITMAP, 
			            &beam_noftp, CBLK_LENB-2, 2, 0.01 );
               epicsThreadSleep(.05);
	    }
            if (SUCCESS(iss))
            {
               iss = blockPIOPCblk (camblocks_ps, PIOP_CBLK_FTBITMAP, 
			            &beam_one, CBLK_LENB-2, 2, 0.01 );
               epicsThreadSleep(.05);
	    }
            if (SUCCESS(iss))
	       beam_loaded = 1;
            pvt_p->status = iss;
            break;
         }
         case PPNOFTP:
         {
	    pvt_p->status = blockPIOPCblk (camblocks_ps, PIOP_CBLK_TKBITMAP, 
                                           pvt_p->val_p, CBLK_LENB-2, 2, 0.01 );
            epicsThreadSleep(.05); /* Let PIOP digest bitmap */
            break;
         }           
         case PPFTP:
         {
	    pvt_p->status = blockPIOPCblk (camblocks_ps, PIOP_CBLK_FTBITMAP, 
                                           pvt_p->val_p, CBLK_LENB-2, 2, 0.01);
            epicsThreadSleep(.05); /* Let PIOP digest bitmap */
            break;
         }           
         case STATUSBLOCK:
         {
	    pvt_p->status = blockPIOPSblk (camblocks_ps,pvt_p->val_p, 3, 0.1);
            break;
         }           
         case FTP:
         {
	    iss = KLYS_OKOK;
	    ftp_wave_ps = (FTP_WAVE_TS*) pvt_p->val_p;
            ftp_cblk_ps = &(ftp_wave_ps->ftp_cblk_s);
            ftp_info_ps = &(ftp_wave_ps->ftp_info_s);
            ftp_read_ps = &(ftp_wave_ps->ftp_read_s);
	    /*
	    ** Insure we have the correct PP bitmap loaded and we're not 
            ** doing the psuedo FTP that reads the bitmap.
	    */
	    if ((ftp_info_ps->pp != beam_loaded) && (ftp_cblk_ps->channel != SUBSTPP_CHAN))
            {
	       if (ftp_info_ps->pp == 0)
                  iss = blockPIOPCblk (camblocks_ps, PIOP_CBLK_FTBITMAP, 
                                       beam_any, CBLK_LENB-2, 2, 0.01);
	       else
                  iss = blockPIOPCblk (camblocks_ps, PIOP_CBLK_FTBITMAP, 
                                       beam_one, CBLK_LENB-2, 2, 0.01);
               epicsThreadSleep(.05); 
               if (SUCCESS(iss))
	          beam_loaded = ftp_info_ps->pp;
	    }
	    /*
	    ** Send the CBLK and read the FTP data.
	    */
	    if (SUCCESS(iss))
	    {
               iss = blockPIOPCblk (camblocks_ps, PIOP_CBLK_FTP, ftp_cblk_ps, 
				    sizeof(FTP_CBLK_TS), 2, 0.01);
	       if (SUCCESS(iss))
                  iss = blockPIOPFblk (camblocks_ps, ftp_read_ps,
                                       ftp_info_ps->tries, ftp_info_ps->ms_per_try *.001);
	    }            
            pvt_p->status = iss; 
            break;
         }           
         case PAD:
         {           
	    pvt_p->status = blockPIOPCblk (camblocks_ps, PIOP_CBLK_PADPARAM, 
                                           pvt_p->val_p, CBLK_LENB-4, 2, 0.01);
            break;
         }           
         case MK2:
         {
	    pvt_p->status = blockPIOPCblk (camblocks_ps, PIOP_CBLK_MK2PARAM, 
                                           pvt_p->val_p, CBLK_LENB-4, 2, 0.01);
            break;
         }           
         case TRIMSLED:
         {
	    pvt_p->status = blockPIOPCblk (camblocks_ps, PIOP_CBLK_TRIMSLED, 
                                           pvt_p->val_p, 2, 2, 0.1);
            break;
         }           
         case FOXHOME:
         {
	    pvt_p->status = blockPIOPCblk (camblocks_ps, PIOP_CBLK_FOXHOME, 
                                           pvt_p->val_p, 0, 2, 0.1);
            break;
         }           
         default:
            errlogSevPrintf(errlogMinor, "Invalid camac function %d in threadPIOP\n",
                            pvt_p->camfunc_e);
      }   /* Switch on Camac function */
      /*
      ** Unlock and finish record processing
      */
      dbScanLock(reccom_p);
      (*(reccom_p->rset->process))(reccom_p);
      dbScanUnlock(reccom_p);

   }   /* End while (TRUE) */
   printf ("Never get here!!\n");
}   /* End threadPIOP */


/****************************************************************
** This is the driver thread that does all of the SBI Camac work.
** It also executes the Camac package that reads all of the PIOP
** message words.
*****************************************************************/
void threadSBI (void * msgQId)
{
   vmsstat_t iss = KLYS_OKOK;
   epicsMessageQueueId lmsgQ = msgQId;
   THREADMSG_TS msg_s;
   int msgQstat;
   void *delaypkg_p; /* Camac SBI delay & PSK enable package */
   void *stspkg_p;   /* Camac SBI status only package */
   unsigned short nops;
   unsigned short emaskf3f3 = 0xF3F3;  /* emask */
   unsigned short twobytes  = 2;
   unsigned short fourbytes = 4;
   unsigned short zero      = 0;
   STAT_DAT16 read_status_only_s;   /* Just for periodic status read func SBISTS */
   /*
   ** Structs toset/verify delay & PSK enable
   */
   STAT_DAT16 ro_delay_s;           /* Read only delay */
   STAT_DAT32 ro_stat_s;            /* Read only and delay */
   /*
   ** struct to write and read back the SBI delay and PSK enable. 
   ** On the readback the second word is a status.
   */ 
   STAT_DAT32 write_delay_s, read_delay_s;

   PIOP_PVT *pvt_p;    /* Driver private struct */
   dbCommon *reccom_p; /* Record pointer */
   int init_done = 0;  /* must do init first time */
   /*----------------------------*/
   while (TRUE)
   {
      if ((msgQstat = epicsMessageQueueReceive (lmsgQ, &msg_s, sizeof(THREADMSG_TS)) < 0))
      {
         errlogSevPrintf(errlogFatal,"SBI msgQ timeout status %d. Suspending...\n", msgQstat);
         epicsThreadSuspendSelf();
      }
      reccom_p = msg_s.rec_p;   /* Record pointer for return to device support */
      pvt_p = (PIOP_PVT *)reccom_p->dpvt;   /* Local routines only know about driver private */
      /*************************************
      ** Check for first time initialization.
      **************************************/
      if (!init_done)
      {
         /*
         ** Construct the Camac packages we'll need.
         */
         nops = 4;
	 iss = camalo(&nops, &delaypkg_p);
         nops = 1;
         if (SUCCESS(iss))
	    iss = camalo(&nops, &stspkg_p);

         unsigned long ploc = (pvt_p->crate << CCTLW__C_shc) | (pvt_p->slot << CCTLW__M_shc);
	 /*
	 ** First the status only package
	 */
         unsigned long ctlw = ploc | CCTLW__A1;
         if (SUCCESS(iss))
            iss = camadd(&ctlw, &read_status_only_s, &twobytes, &zero, &stspkg_p);
	 /*
	 ** Now the delay package. Read delay and 4 bytes status (A0 and A1 4 bytes read)
         ** to reset latched bits. Then write delay and PSK enable and readback delay and
         ** status. 
	 */
         ctlw = ploc;
         if (SUCCESS(iss))
            iss = camadd(&ctlw, &ro_delay_s, &twobytes, &emaskf3f3, &delaypkg_p);
         ctlw = ploc | CCTLW__A1;
         if (SUCCESS(iss))
            iss = camadd(&ctlw, &ro_stat_s, &fourbytes, &emaskf3f3, &delaypkg_p);
         ctlw = ploc | CCTLW__F16 | CCTLW__SA;
         if (SUCCESS(iss))
            iss = camadd(&ctlw, &write_delay_s, &fourbytes, &emaskf3f3, &delaypkg_p);
         ctlw = ploc | CCTLW__SA;
         if (SUCCESS(iss))
            iss = camadd(&ctlw, &read_delay_s, &fourbytes, &emaskf3f3, &delaypkg_p);
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
	 init_done = 1;
      }  /* End first time initialization */
      /***************************************
       ** Execute Camac function from driver private struct.
       ***************************************/ 
      switch (pvt_p->camfunc_e)
      {
         case SBIMSGPIOP:  /* Execute the PIOP MSG package that reads msg for all PIOPS */
         {
  	    iss = camgo (&Msgw_pkg_p);
            pvt_p->status = iss;
            dbScanLock(reccom_p);
            (*(reccom_p->rset->process))(reccom_p);
            dbScanUnlock(reccom_p);
            break;
         }
         case SBISTATUS:   /* Execute the SBI status package */
         {
            if (SUCCESS(iss = camgo (&stspkg_p)))
 	    {
               unsigned short *us_p; /* To put data into void *val_p */
	       us_p = pvt_p->val_p;
               *us_p = read_status_only_s.data;
            }
            pvt_p->status = iss;
            dbScanLock(reccom_p);
            (*(reccom_p->rset->process))(reccom_p);
            dbScanUnlock(reccom_p);
            break;
         }
         case SBIDELAY:   /* Update delay and PSK enable for the SBI */
         {
            memcpy (&(write_delay_s.data), pvt_p->val_p, sizeof(int));  /* Copy delay & psk_enable */
            if (SUCCESS(iss = camgo (&delaypkg_p)))
 	    {
	       if ( (write_delay_s.data & 0x0000FFFF) != (read_delay_s.data & 0x0000FFFF) )
	          iss = KLYS_SBIDELAY;
            }
            pvt_p->status = iss;
            dbScanLock(reccom_p);
            (*(reccom_p->rset->process))(reccom_p);
            dbScanUnlock(reccom_p);
            break;
         }
         default:
            errlogSevPrintf(errlogMinor, "Invalid function %d in threadSBI\n",pvt_p->camfunc_e);
      }
   }   /* End while (TRUE) */
}      /* End threadSBI */
