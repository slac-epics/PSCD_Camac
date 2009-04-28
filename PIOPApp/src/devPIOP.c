/***************************************************************************\
 *   $Id: devPIOP.c,v 1.1.1.1 2009/02/12 20:08:58 pengs Exp $
 *   File:		devPIOP.c
 *   Author:		Robert C. Sass
 *   Email:		bsassy@garlic.com
 *   Phone:		408-464-5853
 *   Company:		SLAC
 *   Date:		02/2009
 *   Version:		.01
 *
 *   EPICS device support file for PIOP 
 *
\***************************************************************************/

#include <slc_macros.h>
#include <devPIOP.h>    /* All other includes & PIOP definitions */

/******************************************************************************************/
/*********************               Global storage             ***************************/
/******************************************************************************************/

/*
** MessageQueue associated with the thread for each PIOP module.
*/
extern epicsMessageQueueId module_msgQId[MAX_CRATES][MAX_SLOTS];

/******************************************************************************************/
/***********************  local routine prototypes         ********************************/
/******************************************************************************************/

/*
** Stringout record support
*/

static long So_init (int); 
static long So_init_record (struct stringoutRecord *sor_p);
static long So_write (struct stringoutRecord *sor_p);

/*
** Waveform record support
*/

static long Wf_init (int); 
static long Wf_init_record (struct waveformRecord *sor_p);
static long Wf_write (struct waveformRecord *sor_p);

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
} PIOP_DEV_SUP;

/*
** Stringout device support functions
*/
PIOP_DEV_SUP devSoPIOP = {6, NULL, So_init, So_init_record, NULL, So_write, NULL};

epicsExportAddress(dset, devSoPIOP);

/*
** Waveform device support functions
*/
PIOP_DEV_SUP devWfPIOP = {6, NULL, Wf_init, Wf_init_record, NULL, Wf_write, NULL};

epicsExportAddress(dset, devWfPIOP);

/*********************************************************
 *********** Stringout Device Support ********************
 ********************************************************/
/*
** Stringout initialization. 
** Called twice during IOC initialization for module setup.
** after=0 called before database records are initialized
** after=1 called after database records were initialized
**
** We do any required local initialization here since there must be
** at least one stringout record/PIOP for IPL.
*/

static long So_init (int after)
{
   return 0;
}

/*
** Since stringout is used for PIOP IPL we create the thread and associated message queue
** for each PIOP module encountered.
*/

static long So_init_record (struct stringoutRecord *sor_p)
{
   struct camacio *cam_ps = &(sor_p->out.value.camacio);
   short crate = cam_ps->c;
   short slot  = cam_ps->n;
   int status;
   /*------------------------------------------------*/
   printf ("Entered so_init_record with crate %d slot %d.\n", crate,slot);

   /*************************
   ** First do error checking
   **************************/
   if(sor_p->out.type!=CAMAC_IO)
   {
      recGblRecordError(S_db_badField, (void *)sor_p, 
                         "devSoPIOP So_init_record, not CAMAC");
      sor_p->pact=TRUE;
      return (S_db_badField);
   }
   if( (crate > MAX_CRATES) || (slot > MAX_SLOTS) )
   {
      recGblRecordError(S_db_badField, (void *)sor_p, 
                        "devSoPIOP So_init_record, illegal crate or slot");
      sor_p->pact=TRUE;
      return (S_db_badField);
   }
   /*
   ** If this is an IPL record, create the MessageQueue & Thread for this module..
   */
   if (strcmp("IPL",cam_ps->parm) == 0)
   {
      printf ("Got IPL stringout record\n");
      if (module_msgQId[crate][slot] == NULL)
      {
         if ((module_msgQId[crate][slot] = epicsMessageQueueCreate (10,sizeof(THREADMSG_TS))) == NULL)
         {
            errlogPrintf("Failed to create messageQ for PIOP module thread. Suspending!\n");
            epicsThreadSuspendSelf();
         }
         char tname[10];     /* Constructed thread name */
         sprintf (tname, "threadPIOP%2.2u%2.2u",crate,slot);
         printf ("creating thread named %s\n",tname);
         epicsThreadMustCreate(tname, epicsThreadPriorityMedium, 20480,
                               threadPIOP, (void *)module_msgQId[crate][slot]);
      }
      else           /* >1 record for IPL error */
      {
         recGblRecordError(S_dbLib_recExists, (void *)sor_p, 
                           "Stringout PIOP So_init_record, duplicate IPL record for crate/slot");
         sor_p->pact=TRUE;
         return (S_dbLib_recExists);
      }
      status = PIOPDriverInit((dbCommon *)sor_p, *cam_ps, EPICS_RECTYPE_SO);
   }
   return (0);
}

/*
** Write routine for Stringout record
*/
static long So_write (struct stringoutRecord *sor_p)
{
   struct camacio *cam_ps = &(sor_p->out.value.camacio);
   short crate = cam_ps->c;
   short slot  = cam_ps->n;
   THREADMSG_TS msg_ts;
   PIOP_PVT *pvt_p = (PIOP_PVT *)(sor_p->dpvt);
   int rtn = -1;        /* Assume bad */
   /*---------------------*/
   if (!pvt_p) return (rtn);   /* Bad. Should have a driver private area */
   /*
   ** If this is an IPL record send a message to do it.
   */
   if (strcmp("IPL",cam_ps->parm) == 0)
   {
      printf ("IPL pre-process\n");
      if(!sor_p->pact)
      {  /* Pre-process */
         printf ("IPL pre-process\n");
         msg_ts.rec_p = (dbCommon *)sor_p;
         msg_ts.func_e = IPL;
         msg_ts.parm_u.ipl.crate = crate;
         msg_ts.parm_u.ipl.slot  = slot;
         strcpy (msg_ts.parm_u.ipl.fname, sor_p->val);
         printf ("Name from val %s\n",sor_p->val);
         if (epicsMessageQueueTrySend (module_msgQId[crate][slot], &msg_ts, sizeof(msg_ts)) == -1)
         {
            recGblSetSevr(sor_p, WRITE_ALARM, INVALID_ALARM);
            errlogPrintf("PIOP IPL Thread Error [%s]", sor_p->name);
         }
         else
         {
            sor_p->pact = TRUE;
            rtn = 0;     /* Return OK */
         }
      }
      else
      { /* post-process */
         rtn = 0;     /* Always return good status */
         printf ("IPL post-process\n");
         if (!SUCCESS(pvt_p->status))  /* Check for different error options?? */
         {
            recGblSetSevr(sor_p, WRITE_ALARM, INVALID_ALARM);
            errlogPrintf("Record [%s] receive error code [0x%08x]!\n", sor_p->name, 
                         (unsigned int)pvt_p->status);
         }
      }   /* post-process */
   } /* IPL stringout */ 
   return (rtn);
}

/*********************************************************
 *********** Waveform Device Support *********************
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
   struct camacio *cam_ps = &(wfr_p->inp.value.camacio);
   short crate = cam_ps->c;
   short slot  = cam_ps->n;
   int status;
   /*------------------------------------------------*/
   printf ("Entered Wf_init_record with crate %d slot %d.\n", crate,slot);
   if( (crate > MAX_CRATES) || (slot > MAX_SLOTS) )
   {
      recGblRecordError(S_db_badField, (void *)wfr_p, 
                        "devSoPIOP Wf_init_record, illegal crate or slot");
      wfr_p->pact=TRUE;
      return (S_db_badField);
   }
   status = PIOPDriverInit((dbCommon *)wfr_p, *cam_ps, EPICS_RECTYPE_WF);
   return (0);
}

/*
** Write routine for Waveform record
*/
static long Wf_write (struct waveformRecord *wfr_p)
{
   struct camacio *cam_ps = &(wfr_p->inp.value.camacio);
   short crate = cam_ps->c;
   short slot  = cam_ps->n;
   THREADMSG_TS msg_ts;
   PIOP_PVT *pvt_p = (PIOP_PVT *)(wfr_p->dpvt);
   int rtn = -1;        /* Assume bad */
   /*---------------------*/
   if (!pvt_p) return (rtn);   /* Bad. Should have a driver private area */
   /*
   ** If this is an IPL record send a message to do it.
   */
   if (strcmp("PPNOFTP",cam_ps->parm) == 0)
   {
      printf ("Waveform no FTP PP pre-process\n");
      if(!wfr_p->pact)
      {  /* Pre-process */
         printf ("IPL pre-process\n");
         msg_ts.rec_p = (dbCommon *)wfr_p;
         msg_ts.func_e = PPNOFTP;
         msg_ts.parm_u.ipl.crate = crate;
         msg_ts.parm_u.ipl.slot  = slot;
         if (epicsMessageQueueTrySend (module_msgQId[crate][slot], &msg_ts, sizeof(msg_ts)) == -1)
         {
            recGblSetSevr(wfr_p, WRITE_ALARM, INVALID_ALARM);
            errlogPrintf("PIOP IPL Thread Error [%s]", wfr_p->name);
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
         printf ("PPNOFTP post-process\n");
         if (!SUCCESS(pvt_p->status))  /* Check for different error options?? */
         {
            recGblSetSevr(wfr_p, WRITE_ALARM, INVALID_ALARM);
            errlogPrintf("Record [%s] receive error code [0x%08x]!\n", wfr_p->name, 
                         (unsigned int)pvt_p->status);
         }
      }   /* post-process */
   } /* PPNOFTP waveform */ 
}


