/***************************************************************************\
 **   $Id: fidPHASE.c,v 1.1 2011/02/07 15:24:59 rcs Exp $
 **   File:              fidPHASE.c
 **   Author:            Robert C. Sass
 **   Email:             rcs@slac.stanford.edu
 **   Phone:             408-464-5853
 **   Company:           SLAC
 **   Date:              01/2011
 **   Version:           1.0
 **
 **   Read PIOP phase on any beam crossing.
 **   This module has the following routines:
 **
 **   fidPHASEStart    - called from device support after all records are
 **                      initialized to start phase data collection.
 **   fidPHASETimerIsr - BSP timer ISR. Sets event for main routine.  
 **   fidPHASEEvr      - EVR callback. Sets event for main routine.
 **   fidPHASETask     - main task that reads the phase.
 **
\****************************************************************************/

#include "devPIOP.h"
#include "slc_macros.h"
#include "cam_proto.h"
#include "camblkstruc.h"
#include "evrTime.h"
#include "evrPattern.h"

/*
** For BSP fine granularity timer. RTEMS only.
*/
#ifdef __rtems__
#include <bsp/gt_timer.h>
#endif

static int fidPHASETask(void * parg);     /* The 360Hz task that does the work */ 
static epicsEventId fidPHASEEvent = NULL; /* Set by Evr callback to wake up task */
static epicsEventId fidIsrEvent = NULL;   /* Set by timer Isr to wake up task */

/*
** Move these inside loop when debug complete. ??
*/
unsigned long Delay_usec = 2000;  /* For BSP timer */

int Phasecambad  = 0;   /* !!?? Temp for debugging */
int Phasecamgud  = 0;

#define DEFAULT_EVR_TIMEOUT 0.2

extern void *Phase_pkg_p;     /* Package to read all of the phases i.e. status words */
extern IOSCANPVT PhaseIoPvt;  /* Scan list for phases */

/*************************************************************************************/
/******************              implementation              *************************/
/*************************************************************************************/

/* 
** This funciton will be called in Ai device support init during final round. 
*/
epicsThreadId fidPHASEStart()
{
    return (epicsThreadMustCreate("fidPHASETask", epicsThreadPriorityMax, 204800, 
                 (EPICSTHREADFUNC)fidPHASETask, NULL));
}

#ifdef __rtems__
/*
** BSP timer isr
*/
/*
** BSP RTEMS timer isr. Signal fidPHASETask to read the phases.
*/
static void fidPHASETimerIsr(void *arg)
{
    /*-----------------------------------*/
    epicsEventSignal(fidIsrEvent);  /* Signal fid task to do Camac */
    return;
}
#endif

/* 
** This funciton will be registered by fidPHASETask with EVR callback 
** and be called at Fuducial rate. It just sends a signal to fidPHASETask
** once it has been created.  
*/
static void fidPHASEEvr(void *parg)
{
    if(fidPHASEEvent) epicsEventSignal(fidPHASEEvent);
    return;
}

/*
** This is the task that does all of the work.
*/
static int fidPHASETask(void * parg)
{
    int eventstat;                /* Event wait status */
    int pipestat;                 /* get time from pipeline status */
    unsigned int timer_num = 0;   /* BSP timer number */
    unsigned int timer_delay;     /* Actual calculated delay */ 
    unsigned int clock_freq;      /* Board clock frequency */
    epicsTimeStamp time_s;
    evrModifier_ta modifier_a;
    unsigned long  patternStatus; /* see evrPattern.h for values */
    mbcd_pkghdr_ts *campkg_p = Phase_pkg_p;  /* To check if any ops in pkg */
    /*---------------------------------------*/
    /*
    ** Check that we have something to execute and we can get a BSP timer.
    */
    if (campkg_p->iop == 0)
    {
       errlogPrintf("No packets to execute in  fidPHASETask.\n");
       epicsThreadSuspendSelf();
    }
    /*
    ** Find a free timer, get board clock frequency and calc actual delay.
    */
    while (BSP_timer_setup(timer_num, fidPHASETimerIsr, 0, 0))
    {
       if (++timer_num >= BSP_timer_instances())
       {
           errlogSevPrintf(errlogFatal,
                       "No BSP timer found in fidPHASETask. We die now.\n");
           epicsThreadSuspendSelf();
       }
    }
    clock_freq = BSP_timer_clock_get(timer_num);
    /*
    ** Create events we need and register us with the EVR callback after some delay 
    ** for save/restore.
    */
    fidPHASEEvent = epicsEventMustCreate(epicsEventEmpty); /* Event used to signal 360 Hz */
    fidIsrEvent   = epicsEventMustCreate(epicsEventEmpty); /* Event used to signal timer */
    epicsThreadSleep(20.0);                                /* Wait for save/restore to finish */
    evrTimeRegister((FIDUCIALFUNCTION)fidPHASEEvr, NULL);  /* Register Evr callback */
    printf ("***********Starting fidPHASETask loop***************\n");
    while(TRUE)
    {
        eventstat = epicsEventWaitWithTimeout(fidPHASEEvent, DEFAULT_EVR_TIMEOUT);
        if(eventstat == epicsEventWaitOK)
        {
            /*
 	    ** Get current modifiers and check for beam crossing.
 	    */
            pipestat = evrTimeGetFromPipeline(&time_s,  evrTimeCurrent, modifier_a,
                                              &patternStatus, 0,0,0);
            if((pipestat == 0) && (patternStatus == PATTERN_OK))
            {
                if (MOD5_BEAMFULL_MASK & modifier_a[MOD5_IDX])
 	        {
                    /*
 		    ** We have beam. The beam will arrive in ~1ms & it takes the PIOP ~.5ms to
                    ** store its phase so wait ~2ms before we read it out.
		    ** Calc delay each time for now so can change it in debugger.
		    ** !!! Only do this once after debugging is complete.
 		    */
                    timer_delay = Delay_usec * (double)(clock_freq * 1.E-6);
                    BSP_timer_start (timer_num, timer_delay);
                    epicsEventMustWait(fidIsrEvent);
                    if (SUCCESS(camgo (&Phase_pkg_p)))
		    {
                        Phasecamgud++;
                        scanIoRequest(PhaseIoPvt);
		    }
                    else
                        Phasecambad++;
	        } 
            }
	}
        else if(eventstat == epicsEventWaitTimeout)
        {
            errlogSevPrintf(errlogMajor,
                      "Wait event timeout in fidPHASETask. Sleep 2 seconds & keep going.\n");
            epicsThreadSleep(2.0);
        }
	else   /* Some unknown error */
	{
            errlogSevPrintf(errlogFatal,
                   "Wait event error in fidPHASETask. We die now.\n");
            epicsThreadSuspendSelf();
	}
    }   /* while(TRUE) so never return from following call */
    return 0;
}
