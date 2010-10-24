/****************************************************************************
 **   $Id: fidPDUDIAG.c,v 1.2 2010/09/23 15:47:07 rcs Exp $
 **   File:              fidPDUDIAG.c
 **   Author:            Robert C. Sass
 **   Email:             rcs@slac.stanford.edu
 **   Phone:             408-464-5853
 **   Company:           SLAC
 **   Date:              08/2010
 **   Version:           1.0
 **
 **   Collect actual PDU timing delays. This module has the following routines:
 ** 
 **   static void fidPDUDIAGDone - Called internally to terminate a collection.
 **   void fidPDUDIAGPreCam      - Called by 360Hz just before camgo.
 **   void fidPDUDIAGPostCam     - Called by 360Hz just after camgo.
 **
 ****************************************************************************/

#include "devPDUDIAG.h"
#include "slc_macros.h"
#include "cam_proto.h"
#include "cctlwmasks.h"

PDUDIAG_WFD         Wfd_s;             /* Collected waveform data struct */
extern int          fidPDUDIAGActive;  /* A collection is active. Defined in drvPDUDIAG */
extern epicsEventId fidCollectionDone; /* Set when collection is done. Defined in drvPDUDIAG */

/*
** Local state and info to control data collection. This function is 
** not reentrant and can only collect one data set at a time. It is
** started by the single waveform execution thread that waits for the
** event signaling that collection is complete.
*/
typedef enum {WAITING_SECOND, CHECK_MATCH, STORE_MODS, READ_REARM, SKIP_WRAP} STATES;
static STATES State = WAITING_SECOND; 
static UINT32         Sav_count   = 0;  /* Count of samples we've saved */
static UINT32         Dat_index   = 0;  /* Running index to data array */   
static UINT32         Pass_num    = 0;  /* Collection pass */   
static UINT32         Fids_wait   = 0;  /* Fids waiting for next second */
static UINT32         First_call  = 1;  /* First call of a new request */
   
static unsigned short Emask = 0xF300;
static unsigned long  Mode_ctlw;      /* ctlw to set mode */
static unsigned long  Read_ctlw;      /* ctlw to read delay */
static unsigned long  Avail_ctlw;     /* ctlw to check if data available */
static unsigned long  Match_ctlw;     /* ctlw to check if measurement on next fiducial */
static unsigned long  Match_stat;     /* Status to check if will measure next fiducial */
static unsigned long  Avail_stat;     /* Status to check if data is available */
static STAT_LDAT      Read_ldat_s;    /* Delay interval status/data */
static STAT_SDAT      Mode_sdat_s;    /* Mode status/data */

/******************************************
** Local routine to terminate a collection.
******************************************/
static void fidPDUDIAGDone (vmsstat_t iss)
{
   /*----------------------------*/
   /*
   ** End collection. iss indicates error or success.
   ** Set status, reset all counters, state and the active flag.
   ** Signal the done event.
   */
   Wfd_s.status = iss;
   fidPDUDIAGActive = 0;
   Sav_count = 0;
   Dat_index = 0;
   Pass_num = 0;
   First_call = 1;
   State = WAITING_SECOND; /* Next time we're active start here */
   epicsEventSignal(fidCollectionDone);
   return;
}

/*************************************************************************
** Called at 360Hz by the Fiducial processing before it's ready to issue
** the camgo to execute the F19 package.
**
**************************************************************************/ 
void fidPDUDIAGPreCam (void *pkg_p)
{
   int            stat;            /* TimeGet status */
   vmsstat_t      iss = DIAG_OKOK; /* cam* status */
   evrModifier_ta modifier_a;      /* Array of modifiers */
   unsigned long  patternStatus;   /* see evrPattern.h for values */
   UINT16         bcnt;            /* byte count for read delay camadd */
   /*----------------------------*/
   if (!fidPDUDIAGActive)
     return;
   /*
   ** switch based on state of data collection
   */
   switch (State)
   {
      case WAITING_SECOND:
         if (First_call)
	 {
            /*
            ** Set up ctlw's we'll need for the rest of the collection.
            */
            Read_ctlw =  CCTLW__F0  |  CCTLW__P24 | CCTLW__QM1 
                                    | (Wfd_s.crate << CCTLW__C_shc) | (STB_MODULE << CCTLW__M_shc);
            Avail_ctlw = CCTLW__F27 |  CCTLW__A2 | (Wfd_s.crate << CCTLW__C_shc) 
                                    | (STB_MODULE << CCTLW__M_shc);
            Match_ctlw = CCTLW__F27 |  CCTLW__A3 | (Wfd_s.crate << CCTLW__C_shc) 
                                    | (STB_MODULE << CCTLW__M_shc);
            Mode_ctlw = CCTLW__F17  | CCTLW__QM1 
                                    | (Wfd_s.crate << CCTLW__C_shc) | (STB_MODULE << CCTLW__M_shc);
            Mode_sdat_s.sdat  = (Wfd_s.channel << STB_CHAN_SHIFT)   | (STB_MODE_ANY << STB_MODE_SHIFT) 
                                                                    | 0xFF;
            /*
            ** Add packet to set read out interval from anythin previous and
	    ** set mode/channel and enable timer. Module is always 21.
            ** Enter non-zero PPYY even though we want to match ANY else it doesn't work.
            */
            bcnt = 4;
            if (!SUCCESS (iss = camadd (&Read_ctlw, &Read_ldat_s, &bcnt, &Emask, pkg_p)))
	    {
               fidPDUDIAGDone(iss);
               goto leave; /* Abort this collection on Camac error */
	    }
            bcnt = 2;
            if(!SUCCESS(iss = camadd (&Mode_ctlw, &Mode_sdat_s, &bcnt, &Emask, pkg_p)))
            {
               fidPDUDIAGDone(iss);
               goto leave; /* Abort this collection on Camac error */
            }
            Fids_wait  = 0;
	    First_call = 0;
	 }
         stat = evrTimeGetFromPipeline(&Wfd_s.timestamp,  evrTimeNext2, modifier_a, &patternStatus, 0,0,0);
         if(stat || patternStatus)
         {
               fidPDUDIAGDone(DIAG_BADPATTERN);
               goto leave; /* Abort this collection on Camac error */
	 }
         /*
	 ** If this fiducial is two before the second, read the interval to 
	 ** enable collection. Skip a fid so we start on Timeslot 1.
	 */
         if (    ((modifier_a[MOD5_IDX] & MOD5_1HZ_MASK)  != 0) 
              && ((modifier_a[MOD2_IDX] & TIMESLOT1_MASK) != 0) )
	 {
	    Wfd_s.status = DIAG_OKOK;
            /*
	    ** Read the previous value which re-arms the counter for fid+2
	    */
            bcnt = 4;
            if (!SUCCESS (iss = camadd (&Read_ctlw, &Read_ldat_s, &bcnt, &Emask, pkg_p)))
	    {
               fidPDUDIAGDone(iss);
               goto leave; /* Abort this collection on Camac error */
	    }
            State = SKIP_WRAP;
	 }
	 else
            Fids_wait++;   /* Count fids we wait */
	 break;
      case CHECK_MATCH:
         /*
	 ** Check that we'll match on the next fiducial
	 */
         bcnt = 0;
         if (!SUCCESS(iss = camadd (&Match_ctlw, &Match_stat, &bcnt, &Emask, pkg_p)))
	 {
	    fidPDUDIAGDone(iss);
	    goto leave;
	 }
         break;
      case STORE_MODS:
         /*
	 ** The STB time will measure this fid to be read out on the next.
	 ** Save modifiers & timestamp for this fid.
	 */
         stat = evrTimeGetFromPipeline(&(Wfd_s.fiddata[Dat_index].fidtimestamp),  evrTimeCurrent, 
                                        Wfd_s.fiddata[Dat_index].modifier_a, &patternStatus, 0,0,0);
         if(stat || patternStatus)
         {
            fidPDUDIAGDone(DIAG_BADPATTERN);
            goto leave; /* Abort this collection on pattern error */
	 }
	 break;
      case READ_REARM:
         /* 
	 ** READ_REARM Reads the actual delay from the previous fid which 
	 ** rearms the counter for fid+2. Make sure data is available. 
         */
         bcnt = 0;
         if (!SUCCESS (iss = camadd (&Avail_ctlw, &Avail_stat, &bcnt, &Emask, pkg_p)))
	 {
            fidPDUDIAGDone(iss);
            goto leave; /* Abort this collection on Camac error */
	 }
         bcnt = 4;
         if (!SUCCESS (iss = camadd (&Read_ctlw, &Read_ldat_s, &bcnt, &Emask, pkg_p)))
	 {
            fidPDUDIAGDone(iss);
            goto leave; /* Abort this collection on Camac error */
	 }
         /*
	 ** If this sample is the last for this pass then disable the interval timer
	 ** for one fid.
	 */
         if ( (Dat_index + 3) > MAX_SAMPLES-1) /* After this sample start the next pass */
	 {
            Mode_sdat_s.sdat  |= STB_STOP_TIMER;
            bcnt = 2;
            if(!SUCCESS(iss = camadd (&Mode_ctlw, &Mode_sdat_s, &bcnt, &Emask, pkg_p)))
            {
               fidPDUDIAGDone(iss);
               goto leave;
            }
            Mode_sdat_s.sdat  &= ~STB_STOP_TIMER;  /* Clear out the stop bit to re-enable */
	 }
	 break;
      case SKIP_WRAP:    /* Re-enable interval timer */
         bcnt = 2;
         if(!SUCCESS(iss = camadd (&Mode_ctlw, &Mode_sdat_s, &bcnt, &Emask, pkg_p)))
         {
            fidPDUDIAGDone(iss);
            goto leave; /* Abort this collection on Camac error */
         }
	 break;
   }
leave:
   return;
}

/********************************************
** This is called after the Camgo is complete.
********************************************/
void fidPDUDIAGPostCam (void)
{
   /*----------------------------*/
   if (!fidPDUDIAGActive)
     return;
   /*
   ** switch based on state of data collection
   */
   switch (State)
   {
      case WAITING_SECOND:   /* Nothing to do */
         break;  
      case CHECK_MATCH:
         Wfd_s.fiddata[Dat_index].matchstatus = Match_stat; /* Save match status */
         State = STORE_MODS;  /* Store modifiers for next state */
	 break;
      case STORE_MODS:    /* Just change state */
         State = READ_REARM;
	 break;
      case READ_REARM:
         State = CHECK_MATCH;   /* Assume we're in the middle of a pass */
         /*
         ** Check if we got a good measurement on the last read
         */
         if ((Avail_stat & MBCD_STAT__Q) != 0)
	 {
            Wfd_s.fiddata[Dat_index].measdelay =  Read_ldat_s.ldat;
	    Wfd_s.fiddata[Dat_index].fidstatus = DIAG_OKOK;
	 }
	 else
	 {
            Wfd_s.fiddata[Dat_index].measdelay =  -1;
	    Wfd_s.fiddata[Dat_index].fidstatus = DIAG_NOMEAS;
	 } 
	 Sav_count++;
         if(Sav_count >= MAX_SAMPLES)
	 {
               fidPDUDIAGDone(DIAG_OKOK);     /* This collection is done */
	       goto leave;
	 }
	 Dat_index+=3;  /* Asssume we're in the middle of a pass */
         /*
	 ** Start next pass if Dat_index is off the end.
	 */
         if (Dat_index > MAX_SAMPLES-1)
	 {
            Dat_index = ++Pass_num;  /* Start collecting the next pass */
            State = SKIP_WRAP;       /* End of a pass so skip the next fiducial */
	 }
         break;
      case SKIP_WRAP:     /* Just set next state */
         State = CHECK_MATCH;
	 break;
   }     /* End switch */
leave:
   return;
}
