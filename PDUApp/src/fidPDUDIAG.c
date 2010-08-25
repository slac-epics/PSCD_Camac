/****************************************************************************
 **   $Id: fidPDUIIDIAG.c,v 1.0 2010/08/21 rcs Exp $
 **   File:              fidPDUDIAG.c
 **   Author:            Robert C. Sass
 **   Email:             rcs@slac.stanford.edu
 **   Phone:             408-464-5853
 **   Company:           SLAC
 **   Date:              08/2010
 **   Version:           1.0
 **
 **   Collect actual PDU timing delays. 
 **   Routine called from the 360Hz fiducial processing.
 **
 ****************************************************************************/

#include "devPDUDIAG.h"
#include "slc_macros.h"
#include "cam_proto.h"
#include "cctlwmasks.h"

PDUDIAG_WFD         Wfd_s;             /* Collected waveform data struct */
extern int          fidPDUDIAGActive;  /* defined in drvPDUDIAG */
extern epicsEventId fidEventDone;      /* defined in drvPDUDIAG */

/*
** Local state and info to control data collection. This function is 
** not reentrant and can only collect one data set at a time. It is
** started by the single waveform execution thread who waits for the
** event signaling that collection is complete.
*/
typedef enum {WAITING_SECOND, SKIP_NEXT_FID, READ_REARM,STORE_PREV_DELAY} STATES;
static STATES State = WAITING_SECOND; 
static UINT32         Sav_count = 0;  /* Count of samples we've saved */
static UINT32         Dat_index = 0;  /* Running index to data array */   
static unsigned short Emask = 0xF300;
static unsigned long  Read_ctlw;      /* ctlw To read delay */
static STAT_LDAT      Read_ldat_s;    /* Delay interval status/data */

/*
** Called at 360Hz by the Fiducial processing before it's ready to issue
** the camgo to execute the F19 package.
**
** We run as a state machine so only once call is necessary from the 360Hz task.
*/ 
void fidPDUDIAG (void * pkg_p)
{
   int            stat;            /* TimeGet status */
   vmsstat_t      iss = DIAG_OKOK; /* cam* status */
   epicsTimeStamp time_s;          /* Timestamp */
   evrModifier_ta modifier_a;      /* Array of modifiers */
   unsigned long  patternStatus;   /* see evrPattern.h for values */
   unsigned long  mode_ctlw;       /* to set mode */
   STAT_SDAT      mode_sdat_s;     /* Mode status/data */
   UINT16         bcnt = 4;        /* byte count for read delay camadd */
   /*----------------------------*/
   if (!fidPDUDIAGActive)
     return;
   /*
   ** switch based on state of data collection
   */
   switch (State)
   {
      case WAITING_SECOND:
         stat = evrTimeGetFromPipeline(&time_s,  evrTimeNext1, modifier_a, &patternStatus, 0,0,0);
         if(stat !=0 || patternStatus != PATTERN_OK) goto leave; /* Skip if error. EVR reports it */
         /*
	 ** If next fiducial is the one before the second, set mode and read out the STB.
	 ** Set state to SKIP_NEXT so read out delay on next fid.
	 */
         if ((modifier_a[MOD5_IDX] & MOD5_1HZ_MASK) != 0)
	 {
            /*
            ** Add packets to set mode/channel and read/rearm interval. Module is always 21.
	    ** Enter non-zero PPYY even though we want to match ANY else it doesn't work.
            */
	    mode_ctlw = CCTLW__F17 | CCTLW__QM1 | (Wfd_s.crate << CCTLW__C_shc) 
                                   | (STB_MODULE << CCTLW__M_shc);
	    mode_sdat_s.sdat  = (Wfd_s.channel << STB_CHAN_SHIFT) | (STB_MODE_ANY << STB_MODE_SHIFT) | 1;
	    bcnt = 2;
	    iss = camadd (&mode_ctlw, &mode_sdat_s, &bcnt, &Emask, &pkg_p);
            Read_ctlw = CCTLW__F0 | CCTLW__QM1 | CCTLW__P24 | (Wfd_s.crate << CCTLW__C_shc) 
                                  | (STB_MODULE << CCTLW__M_shc);
            bcnt = 4;
            if (SUCCESS(iss))
               iss = camadd (&Read_ctlw, &Read_ldat_s, &bcnt, &Emask, &pkg_p);
            if (!SUCCESS(iss))
               goto done; 
            Sav_count = 0;
            Dat_index = 0;
	    Wfd_s.status = DIAG_OKOK;
            /*
	    ** Save timestamp & modifiers for first (next) pulse in new second. 
	    ** Actual delay read out 1 pulse after that and stored on the second pulse.
	    */
            stat = evrTimeGetFromPipeline(&(Wfd_s.fiddata[Dat_index].fidtimestamp),  evrTimeNext1, 
                                           Wfd_s.fiddata[Dat_index].modifier_a, &patternStatus, 0,0,0);
            if(stat !=0 || patternStatus != PATTERN_OK)
            {  
               iss = DIAG_BADPATTERN;
               goto done; /* Abort this collection on pattern error */
	    }
	    State = SKIP_NEXT_FID;  /* Timer will run on next fid so read on following one */
	 }
         goto leave;
	 break;
      case SKIP_NEXT_FID:
	 State = READ_REARM; /* Next state Reads the actual delay which rearms the counter for the net fid */
	 break;
      case READ_REARM:
         if (!SUCCESS (iss = camadd (&Read_ctlw, &Read_ldat_s, &bcnt, &Emask, &pkg_p)))
            goto leave;
	 if (Sav_count < MAX_SAMPLES/2)
         {
            /*
	    ** If we're in the middle of this half, get modifiers and timestamp from next pulse 
	    ** which is the actual delay measured. Indexes are from previous saved pulse.
	    */
            stat = evrTimeGetFromPipeline(&(Wfd_s.fiddata[Dat_index+2].fidtimestamp),  evrTimeNext1, 
                                            Wfd_s.fiddata[Dat_index+2].modifier_a, &patternStatus, 0,0,0);
            if(stat !=0 || patternStatus != PATTERN_OK)
            {  
	       iss = DIAG_BADPATTERN;
               goto done; /* Abort this collection on pattern error */
	    }
	 }
         State = STORE_PREV_DELAY; /* Store the data from this read on the next fid */
         break;
      case STORE_PREV_DELAY:
         Wfd_s.fiddata[Dat_index].measdelay =  Read_ldat_s.ldat;
	 Wfd_s.fiddata[Dat_index].fidstatus = DIAG_OKOK; 
	 Dat_index+=2;
	 Sav_count++;
         if(Sav_count >= MAX_SAMPLES)
	 {
            goto done;        /* This collection is done */
	 }
         /*
	 ** A bit tricky here. Store the delay from 2 pulses ago. If we're at the half-way
	 ** point, save the pattern and timestamp for the next pulse, readout the interval 
	 ** but skip the next pulse so we continue on to the second half.
	 */
         if (Sav_count >= MAX_SAMPLES/2)
         {
	    Dat_index = 1;     /* Start collecting the second half */
            /*
	    ** We're at the end of the first half. Get modifiers and timestamp from next pulse 
	    ** which is the actual delay measured and skip the next pulse.
	    */
            stat = evrTimeGetFromPipeline(&(Wfd_s.fiddata[Dat_index].fidtimestamp),  evrTimeNext1, 
                                            Wfd_s.fiddata[Dat_index].modifier_a, &patternStatus, 0,0,0);
            if(stat !=0 || patternStatus != PATTERN_OK)
            {  
	       iss = DIAG_BADPATTERN;
               goto done; /* Abort this collection on pattern error */
	    }
            if (!SUCCESS (iss = camadd (&Read_ctlw, &Read_ldat_s, &bcnt, &Emask, &pkg_p)))
               goto leave;
	    State = SKIP_NEXT_FID;  /* Start collecting second half */
	 }
         else
         {
            State = READ_REARM;  /* Keep collecting */
	 }
	 break;
   }
   goto leave;
done:
/*
** End collection. iss indicates error or success.
** Set status, reset all counters, state and the active flag.
** Signal the done event.
*/
  Wfd_s.status = iss;
  fidPDUDIAGActive = 0;
  Sav_count = 0;
  Dat_index = 0;
  State = WAITING_SECOND; /* Next time we're active start here */
  epicsEventSignal(fidEventDone);
leave:
   return;
}



