/***************************************************************************\
 *   $Id: devPDUDIAG.c,v 1.1 2010/08/25 22:43:43 rcs Exp $
 *   File:		devPDUDIAG.c
 *   Author:		Robert C. Sass
 *   Email:		rcs@slac.stanford.edu
 *   Company:		SLAC
 *   Date:		08/2010
 *   Version:		1.0
 *
 *   EPICS device support file for PDUDIAG PDU Diagnostic 
 *
\***************************************************************************/
#include "devPDUDIAG.h"


/***************************************************************************/
/*********************     Shared between dev/drv       ********************/
/***************************************************************************/

epicsMessageQueueId fidThread_msgQId;
int Crate, Chanl;
extern int PDUDIAG_DEV_DEBUG;

/***************************************************************************/
/*************************     drv local data       ************************/
/***************************************************************************/

static int CrateRecs = 0;         /* Only want one lo for crate number */
static int ChanlRecs = 0;         /* Only want one lo for channel number */
static int WavefRecs = 0;         /* Only want one wf for data collection */


/****************************************************************************/
/*********************              implementation            ***************/
/****************************************************************************/

/* function prototypes */

static long init_lo(struct longoutRecord *plo);
static long write_lo(struct longoutRecord *plo);

static long init_wf(struct waveformRecord *pwf);
static long read_wf(struct waveformRecord *pwf);

void fidThread (void * msgQId);   /* Eaveform execution thread for fid communication */

/* global struct for devSup */
typedef struct {
        long            number;
        DEVSUPFUN       report;
        DEVSUPFUN       init;
        DEVSUPFUN       init_record;
        DEVSUPFUN       get_ioint_info;
        DEVSUPFUN       read_write;
        DEVSUPFUN       special_linconv;
} PDUDIAG_DEV_SUP_SET;

PDUDIAG_DEV_SUP_SET devLoPDUDIAG = {6, NULL, NULL, init_lo, NULL, write_lo, NULL};
PDUDIAG_DEV_SUP_SET devWfPDUDIAG = {6, NULL, NULL, init_wf, NULL, read_wf,  NULL};

#if     EPICS_VERSION>=3 && EPICS_REVISION>=14
epicsExportAddress(dset, devLoPDUDIAG);
epicsExportAddress(dset, devWfPDUDIAG);
#endif

/*******        longout record       ******/
static long init_lo(struct longoutRecord * plo)
{
   struct camacio *cam_ps = &(plo->out.value.camacio);
   char *parm_p= cam_ps->parm;
   /*------------------------------------------------*/
   if(plo->out.type!=CAMAC_IO)
   {
      recGblRecordError(S_db_badField, (void *)plo,
                         "devLoPDUDIAG init_lo type not CAMAC");
      plo->pact=TRUE;
      return (S_db_badField);
   }
   if ( (strcmp("CRATE",parm_p) != 0) && (strcmp("CHANL",parm_p) != 0) )
   {
      recGblRecordError(S_db_badField, (void *)plo,
                        "devLoPDUDIAG init_lo parm not CRATE or CHANL");
      plo->pact=TRUE;
      return (S_db_badField);
   }
   /*
   ** Record has valid param. Insure only one of each CRATE & CHANL.
   */
   if (strcmp("CRATE",parm_p) == 0)
   {
      if (CrateRecs > 0)
      {
         recGblRecordError(S_db_badField, (void *)plo,
                          "devLoPDUDIAG init_lo > 1 CRATE record");
         plo->pact=TRUE;
         return (S_db_badField);
      }
      else
	 CrateRecs++;
   }
   if (strcmp("CHANL",parm_p) == 0)
   {
      if (ChanlRecs > 0)
      {
         recGblRecordError(S_db_badField, (void *)plo,
                          "devLoPDUDIAG init_lo > 1 CHANL record");
         plo->pact=TRUE;
         return (S_db_badField);
      }
      else
	 ChanlRecs++;
   }
   return 0;
}

static long write_lo(struct longoutRecord *plo)
{
   struct camacio *cam_ps = &(plo->out.value.camacio);
   char *parm_p= cam_ps->parm;
   int rval = plo->val;
#define MAX_CRATE	6
#define MAX_CHANL	16 
   /*------------------------------------------------*/
   /*
   ** Save crate or channel value locally.
   */
   if (strcmp("CRATE",parm_p) == 0)
   {
      if ( (rval > 0) && (rval < MAX_CRATE) )
         Crate = rval;
      else
      {
         recGblSetSevr(plo, WRITE_ALARM, INVALID_ALARM);
         errlogPrintf("Record [%s] invalid crate value [%d]\n", plo->name, rval);
      }
   }
   if (strcmp("CHANL",parm_p) == 0)
   {
      if ( (rval >= 0) && (rval < MAX_CHANL) )
         Chanl = rval;
      else
      {
         recGblSetSevr(plo, WRITE_ALARM, INVALID_ALARM);
         errlogPrintf("Record [%s] invalid channel value [%d]\n", plo->name, rval);
      }
   }
   return 0;
}

/*******        waveform record       ******/
static long init_wf(struct waveformRecord * pwf)
{
   WF_PVT *pvt_p = NULL;  /* Our private data */
   /*------------------------------------------------*/
   /*
   ** Only one waveform allowed.
   */
   if (WavefRecs > 0)
   {
      recGblRecordError(S_db_badField, (void *)pwf,
                        "devLoPDUDIAG init_wf > 1 waveform record");
      pwf->pact=TRUE;
      return (S_db_badField);
   }
   else
      WavefRecs++;
   /*
   ** Create the MessageQueue & Thread for the 360Hz fiducial data collection communication.
   */
   if ((fidThread_msgQId = epicsMessageQueueCreate (10,sizeof(THREADMSG))) == NULL)
   {
      recGblRecordError(S_db_noMemory, (void *)pwf,
                  "devWfPDUDIAG init_wf, can't create MessageQueue");
      pwf->pact=TRUE;
      return (S_db_noMemory);
   }
   epicsThreadMustCreate("fidThread", epicsThreadPriorityLow, 20480,
                         fidThread, (void *) (fidThread_msgQId));
   /*
   ** Create/init waveform private data.
   */
   pvt_p = callocMustSucceed (1, sizeof(WF_PVT), "calloc PDUDIAG driver pvt");
   pvt_p->status = DIAG_OKOK;   /* Init status to good */
   pwf->dpvt = pvt_p;           /* Save out private space in record */
   pwf->nord = sizeof(PDUDIAG_WFD)/sizeof(int);
   return 0;
}

static long read_wf(struct waveformRecord *pwf)
{
   THREADMSG msg_s;     /*  msg to send */
   WF_PVT *pvt_p = (WF_PVT *)(pwf->dpvt);
   int rtn = -1;        /* Assume bad */
   /*---------------------*/
   if (!pvt_p) return (rtn);   /* Bad. Should have a driver private area */
   /*
   ** Send msg to thread if not active else complete record processing.
   */
   if(!pwf->pact)
   {  /* Pre-process. */
      msg_s.rec_p = (dbCommon *) pwf;
      msg_s.crate = Crate;
      msg_s.channel = Chanl;
      msg_s.wfb_p = pwf->bptr;
      if (epicsMessageQueueTrySend (fidThread_msgQId, &msg_s, sizeof(msg_s)) == -1)
      {
         recGblSetSevr(pwf, WRITE_ALARM, INVALID_ALARM);
         errlogPrintf("read_wf msg queue send error [%s]", pwf->name);
      }
      else
      {
         pwf->pact = TRUE;
         rtn = 0;     /* Return OK */
      }
   }  /* end pre-process */
   else
   {  /* post-process */
      rtn = 0;     /* Always return good status */
      if (!SUCCESS(pvt_p->status))
      {
         recGblSetSevr(pwf, WRITE_ALARM, INVALID_ALARM);
         errlogPrintf("Record [%s] receive error code [0x%08x]!\n", pwf->name,
                      (unsigned int)pvt_p->status);
      }
   }  /* end post-process */
   return (0);
}
