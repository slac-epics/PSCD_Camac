/***************************************************************************\
 *   $Id: devCAMCOM.c,v 1.1.1.1 2010/05/28 23:13:13 rcs Exp $
 *   File:		devCAMCOM.c
 *   Author:		Robert C. Sass
 *   Email:		rcs@slac.stanford.edu
 *   Phone:		408-464-5853
 *   Company:		SLAC
 *   Date:		04/2010
 *   Version:		.01
 *
 *   EPICS device support file for CAMCOM 
 *
\***************************************************************************/

#include <slc_macros.h>
#include <devCAMCOM.h>    /* All other includes & CAMCOM definitions */

/******************************************************************************************/
/*********************      Externs defined in drvCAMCOM        ***************************/
/******************************************************************************************/

/*
** MessageQueue for camcom thread.
*/
extern epicsMessageQueueId camcom_msgQId;

/******************************************************************************************/
/***********************  local routine prototypes         ********************************/
/******************************************************************************************/

/*
** Waveform record support for CAMCOM
*/

static long Wf_init (int); 
static long Wf_init_record (struct waveformRecord *wfr_p);
static long Wf_read_write (struct waveformRecord *wfr_p);


/******************************************************************************************/
/*********************              implementation              ***************************/
/******************************************************************************************/

/* 
** Generic Struct for devSup
*/
typedef struct {
        long            number;
        DEVSUPFUN       report;
        DEVSUPFUN       init;
        DEVSUPFUN       init_record;
        DEVSUPFUN       get_ioint_info;
        DEVSUPFUN       read_write;
        DEVSUPFUN       special_linconv;
} DEV_SUP;

/*
** Waveform PIOP device support functions
*/
DEV_SUP devWfCAMCOM = {6, NULL, Wf_init, Wf_init_record, NULL, Wf_read_write, NULL};

epicsExportAddress(dset, devWfCAMCOM);

/*********************************************************
 *********** Waveform Record Support *********************
 ********************************************************/

/*
** Init for Waveform record
*/
static long Wf_init (int after)
{
   return 0;
}

/*
** Record init for Waveform
*/
static long Wf_init_record (struct waveformRecord *wfr_p)
{
   CAMCOM_PVT *pvt_p;
   /*------------------------------------------------*/
   CAMCOMDriverInit((dbCommon *)wfr_p, EPICS_RECTYPE_WF);
   pvt_p = (CAMCOM_PVT *)(wfr_p->dpvt);
   pvt_p->val_p = wfr_p->bptr; /* Buffer pointer for waveform */
   return (0);
}

/*
** Read/Write routine for Waveform record
*/
static long Wf_read_write (struct waveformRecord *wfr_p)
{
   THREADMSG_TS msg_s;
   CAMCOM_PVT *pvt_p = (CAMCOM_PVT *)(wfr_p->dpvt);
   int rtn = -1;        /* Assume bad */
   /*---------------------*/
   if (!pvt_p) return (rtn);   /* Bad. Should have a driver private area */
   /*
   ** If pre-process send msg to CAMCOM thread to execute the Camac operation.
   */
   if(!wfr_p->pact)
   {  /* Pre-process */
      msg_s.rec_p = (dbCommon *)wfr_p;
      if (epicsMessageQueueTrySend (camcom_msgQId, &msg_s, sizeof(msg_s)) == -1)
      {
         recGblSetSevr(wfr_p, WRITE_ALARM, INVALID_ALARM);
         errlogPrintf("CAMCOM Wf_read_write Thread Error [%s]", wfr_p->name);
      }
      else 
      {
         wfr_p->pact = TRUE;
         rtn = 0;     /* Return OK */
      }
   }
   else
   { /* post-process */
      rtn = 0;     /* Always return good status */
      if (!SUCCESS(pvt_p->status))  /* Check for different error options?? */
      {
         recGblSetSevr(wfr_p, WRITE_ALARM, INVALID_ALARM);
         errlogPrintf("Record [%s] receive error code [0x%08x]!\n", wfr_p->name, 
                      (unsigned int)pvt_p->status);
      }
   }   /* post-process */
   return (rtn);
}
