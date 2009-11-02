/***************************************************************************\
 *   $Id: devPIOP.c,v 1.3 2009/06/09 22:27:05 pengs Exp $
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
/*********************      Externs defined in drvPIOP          ***************************/
/******************************************************************************************/

/*
** MessageQueue associated with the thread for each PIOP module.
*/
extern epicsMessageQueueId piop_msgQId[MAX_CRATES][MAX_SLOTS];

/*
** MessageQueue for SBI thread assume one per IOC.
*/
extern epicsMessageQueueId sbi_msgQId;

/*
** Camac package and stat/data array for PIOP message word from drvPIOP
*/
extern void *msgw_pkg_p;
extern STAT_DAT16 Piop_Msgs_s[MAX_PIOPS];


/******************************************************************************************/
/********************* globals for this module's eyes only      ***************************/
/******************************************************************************************/

static char *wave_parms = {"PPNOFTP","PPFTP", "STATUSBLOCK", "PAD", "MK2", "NEWPHASE", "TRIMSLED", 
                           "FOXHOME"}

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
static long Wf_init_record (struct waveformRecord *wfr_p);
static long Wf_read_write (struct waveformRecord *wfr_p);

/*
** Mbbi record support
*/

static long Mbi_init (int); 
static long Mbi_init_record (struct mbbiRecord *mbir_p);
static long Mbi_read (struct mbbiRecord *mbir_p);

/*
** bi record support
*/

static long Bi_init (int); 
static long Bi_init_record (struct biRecord *bir_p);
static long Bi_read (struct biRecord *bir_p);

/*
** longin record support
*/

static long Li_init (int); 
static long Li_init_record (struct longinRecord *lir_p);
static long Li_read (struct longinRecord *lir_p);

/*
** Local support routines
*/

static CAMFUNC_TE xlate_wf_parm (char *parm); 

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
** Stringout PIOP device support functions
*/
DEV_SUP devSoPIOP = {6, NULL, So_init, So_init_record, NULL, So_write, NULL};

epicsExportAddress(dset, devSoPIOP);

/*
** Waveform PIOP device support functions
*/
DEV_SUP devWfPIOP = {6, NULL, Wf_init, Wf_init_record, NULL, Wf_read_write, NULL};

epicsExportAddress(dset, devWfPIOP);

/*
** Longin device support functions
*/
DEV_SUP devLiPIOP = {6, NULL, Li_init, Li_init_record, NULL, Li_read, NULL};

epicsExportAddress(dset, devLiPIOP);

/*
** binary input device support functions
*/
DEV_SUP devBiPIOP = {6, NULL, Bi_init, Bi_init_record, NULL, Bi_read, NULL};

epicsExportAddress(dset, devBiPIOP);

/*
** MBBI SBI device support functions
*/
DEV_SUP devMbiSBI = {6, NULL, Mbi_init, Mbi_init_record, NULL, Mbi_read, NULL};

epicsExportAddress(dset, devMbiSBI);

/*********************************************************
 *********** Stringout Record Support *******************
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
   char *parm_p= cam_ps->parm;  
   PIOP_PVT *pvt_p;
   int status;
   /*------------------------------------------------*/
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
   if (strcmp("IPL",parm_p) != 0)
   {
      recGblRecordError(S_db_badField, (void *)sor_p, 
                        "devSoPIOP So_init_record, parm not IPL");
      sor_p->pact=TRUE;
      return (S_db_badField);
   }
   if (piop_msgQId[crate][slot] != NULL)
   {
      recGblRecordError(S_dbLib_recExists, (void *)sor_p, 
                        "Stringout PIOP So_init_record, duplicate IPL record for crate/slot");
      sor_p->pact=TRUE;
      return (S_dbLib_recExists);
   }
   /*
   ** Create the MessageQueue & Thread for this module.
   */
   if ((piop_msgQId[crate][slot] = epicsMessageQueueCreate (10,sizeof(THREADMSG_TS))) == NULL)
   {
      recGblRecordError(S_db_noMemory, (void *)sor_p, 
                  "devSoPIOP So_init_record, can't create MessageQueue");
      sor_p->pact=TRUE;
      return (S_db_noMemory);
   }
   char tname[10];     /* Constructed thread name */
   sprintf (tname, "threadPIOP%2.2u%2.2u",crate,slot);
   epicsThreadMustCreate(tname, epicsThreadPriorityMedium, 20480,
                         threadPIOP, (void *)piop_msgQId[crate][slot]);
   /*
   ** Init and set up common info in dpvt our private space for
   ** communication between record processing and the Camac thread.
   ** PIOPDriverInit sets crate, slot & inits status.
   */
   PIOPDriverInit((dbCommon *)sor_p, *cam_ps, EPICS_RECTYPE_SO);
   pvt_p = (PIOP_PVT *)(sor_p->dpvt);
   pvt_p->camfunc_e = IPL;
   pvt_p->val_p = sor_p->val;
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
   THREADMSG_TS msg_s;
   PIOP_PVT *pvt_p = (PIOP_PVT *)(sor_p->dpvt);
   int rtn = -1;        /* Assume bad */
   /*---------------------*/
   if (!pvt_p) return (rtn);   /* Bad. Should have a driver private area */
   /*
   ** Send msg to thread if not active else complete record processing.
   */
   if(!sor_p->pact)
   {  /* Pre-process. */
      msg_s.rec_p = (dbCommon *)sor_p;
      if (epicsMessageQueueTrySend (piop_msgQId[crate][slot], &msg_s, sizeof(msg_s)) == -1)
      {
         recGblSetSevr(sor_p, WRITE_ALARM, INVALID_ALARM);
         errlogPrintf("So_write msg queue send error [%s]", sor_p->name);
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
      if (!SUCCESS(pvt_p->status))  /* Check for different error options?? */
      {
         recGblSetSevr(sor_p, WRITE_ALARM, INVALID_ALARM);
         errlogPrintf("Record [%s] receive error code [0x%08x]!\n", sor_p->name, 
                      (unsigned int)pvt_p->status);
      }
   }   /* post-process */ 
   return (rtn);
}

/*********************************************************
 *********** Waveform Record Support *********************
 ********************************************************/


/*
** Local routine to translate Waveform parm into a camac function CAMFUNC_TE
**
** Note that we explicitly check the length as FTP has other chars after the "FTP".
*/
static struct ckparm_s
{
  char       *parm_p;
  int         lenparm;
  CAMFUNC_TE  func_e;
} ckparm_as[] = { {"PPNOFTP",sizeof("PPNOFTP"),PPNOFTP},
                  {"PPFTP",sizeof("PPFTP"),PPFTP},
                  {"STATUSBLOCK",sizeof("STATUSBLOCK"),STATUSBLOCK},
                  {"FTP",sizeof("FTP"),FTP},
                  {"PAD",sizeof("PAD"),PAD},
                  {"MK2",sizeof("MK2"),MK2FTP},
                  {"TRIMSLED",sizeof("TRIMDSLED"),TRIMSLEDP},
                  {"FOXHOME",sizeof("FOXHOME"),FOXHOME} };

static CAMFUNC_TE xlate_wf_parm (char *inparm_p, short *outidx_p);
{
   int j;
   CAMFUNC_TE camfunc_e = INVALID;
   /*------------------------------------------------*/
   for (j=0; j<(sizeof(ckparm_as)/sizeof(ckparm_s); j++) 
      if (strncmp(ckparm_as[j].parm_p, inparm_p, ckparm_as[j].lenparm) == 0)
      {
         camfunc_e = ckparm_as[j].func;
         break;
      }
   /*
   ** If FTP, validate index.
   */
   if (camfunc_e == FTP)
   {
      sprintf (&inparm_p[3], "%2.2u",*outidx_p);
      if (*outidx_p > MAX_FTP_IDX)
         camfunc_e = INVALID;
   }
   return (camfunc_e);
}

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
   char *parm_p= cam_ps->parm;
   PIOP_PVT      *pvt_p;
   CAMFUNC_TE     camfunc_e;
   short          ftpidx
   /*------------------------------------------------*/
   if(wfr_p->inp.type!=CAMAC_IO)
   {
      recGblRecordError(S_db_badField, (void *)wfr_p, 
                         "devWfPIOP Wf_init_record, not CAMAC");
      wfr_p->pact=TRUE;
      return(S_db_badField);
   }
   if( (crate > MAX_CRATES) || (slot > MAX_SLOTS) )
   {
      recGblRecordError(S_db_badField, (void *)wfr_p, 
                        "devWfPIOP Wf_init_record, illegal crate or slot");
      wfr_p->pact=TRUE;
      return(S_db_badField);
   }
   /*
   ** Insure the PARM is one we know about.If ftp then we also need it's index. 
   */
   if ((camfunc_e = xlate_wf_parm(parm_p, &ftpidx)) == INVALID)
   {   
      recGblRecordError(S_db_badField, (void *)wfr_p, 
                        "devWfPIOP Wf_init_record, invalid parameter");
      wfr_p->pact=TRUE;
      return (S_db_badField);
   }     
   status = PIOPDriverInit((dbCommon *)wfr_p, *cam_ps, EPICS_RECTYPE_WF);
   pvt_p = (PIOP_PVT *)(wfr_p->dpvt);
   pvt_p->camfunc_e = camfunc_e;
   pvt_p->ftpidx = ftpidx;
   pvt_p->val_p = sor_p->val;
egress:
   return (retval);
}

/*
** Read/Write routine for Waveform record
*/
static long Wf_read_write (struct waveformRecord *wfr_p)
{
   struct camacio *cam_ps = &(wfr_p->inp.value.camacio);
   short  crate = cam_ps->c;
   short  slot  = cam_ps->n;
   char  *parm_p= cam_ps->parm; 
   THREADMSG_TS msg_s;
   PIOP_PVT *pvt_p = (PIOP_PVT *)(wfr_p->dpvt);
   int rtn = -1;        /* Assume bad */
   /*---------------------*/
   if (!pvt_p) return (rtn);   /* Bad. Should have a driver private area */
   /*
   ** If pre-process send msg to PIOP thread to execute the Camac operation.
   */
   if(!wfr_p->pact)
   {  /* Pre-process */
      msg_s.rec_p = (dbCommon *)wfr_p;
      if (epicsMessageQueueTrySend (piop_msgQId[crate][slot], &msg_s, sizeof(msg_s)) == -1)
      {
         recGblSetSevr(wfr_p, WRITE_ALARM, INVALID_ALARM);
         errlogPrintf("Wf_read_write Thread Error [%s]", wfr_p->name);
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

/*********************************************************
 *********** Mbbi Record Support *************************
 ********************************************************/

/*
** Init for Mbbi record
*/
static long Mbi_init (int after)
{
   return 0;
}

/*
** Record init for Mbbi
*/
static long Mbi_init_record (struct mbbiRecord *mbir_p)
{
   struct camacio *cam_ps = &(mbir_p->inp.value.camacio);
   short crate = cam_ps->c;
   short slot  = cam_ps->n;
   PIOP_PVT *pvt_p;
   int status;
   /*------------------------------------------------*/
   if(mbir_p->inp.type!=CAMAC_IO)
   {
      recGblRecordError(S_db_badField, (void *)mbir_p, 
                         "devMbiSBI Mbi_init_record, not CAMAC");
      mbir_p->pact=TRUE;
      return (S_db_badField);
   }
   if( (crate > MAX_CRATES) || (slot > MAX_SLOTS) )
   {
      recGblRecordError(S_db_badField, (void *)mbir_p, 
                        "devMbiSBI Mbi_init_record, illegal crate or slot");
      mbir_p->pact=TRUE;
      return (S_db_badField);
   }
   if (strcmp("SBISTATUS",parm_p) != 0)
   {
       recGblRecordError(S_db_badField, (void *)mbir_p, 
                        "devMbiSBI Mbi_init_record, illegal parm string");
      mbir_p->pact=TRUE;
      return (S_db_badField);
  }
   /*
   ** Create SBI thread if not already done
   */   
   if (sbi_msgQId == NULL)
   {
      if ((sbi_msgQId = epicsMessageQueueCreate (10,sizeof(THREADMSG_TS))) == NULL)
      {
         recGblRecordError(S_db_noMemory, (void *)mbir_p, 
                        "devMbiSBI Mbi_init_record, can't create MessageQueue");
            mbir_p->pact=TRUE;
            return (S_db_noMemory);
      }
      epicsThreadMustCreate("threadSBI", epicsThreadPriorityMedium, 20480,
                            threadSBI, (void *)sbi_msgQId);
   }
   status = PIOPDriverInit((dbCommon *)mbir_p, *cam_ps, EPICS_RECTYPE_MBBI);
   pvt_p = (PIOP_PVT *)(mbir_p->dpvt);
   pvt_p->camfunc_e = SBISTATUS;
   pvt_p->val_p = mbir_p->val;
   return (0);
}

/*
** Read/Write routine for SBI Mbbi record
*/
static long Mbi_read (struct mbbiRecord *mbir_p)
{
   struct camacio *cam_ps = &(mbir_p->inp.value.camacio);
   short  crate = cam_ps->c;
   short  slot  = cam_ps->n;
   THREADMSG_TS msg_s;
   PIOP_PVT *pvt_p = (PIOP_PVT *)(mbir_p->dpvt);
   int rtn = -1;        /* Assume bad */
   /*---------------------*/
   if (!pvt_p) return (rtn);   /* Bad. Should have a driver private area */
   /*
   ** Send msg to thread if not active else complete record processing.
   */
   if(!mbir_p->pact)
   {  /* Pre-process */
      msg_s.rec_p = (dbCommon *)mbir_p;
      if (epicsMessageQueueTrySend (sbi_msgQId, &msg_s, sizeof(msg_s)) == -1)
      {
         recGblSetSevr(mbir_p, WRITE_ALARM, INVALID_ALARM);
         errlogPrintf("Record [%s] SBI STATUS can't send msg to thread", mbir_p->name);
      }
      else
      {
         mbir_p->pact = TRUE;
         rtn = 0;     /* Return OK */
      }
   }
   else
   { /* post-process */
      rtn = 0;     /* Always return good status */
      mbir_p->val = pvt_p->val;   /* Get value from driver */
      if (!SUCCESS(pvt_p->status))  /* Check for different error options?? */
      {
         recGblSetSevr(mbir_p, WRITE_ALARM, INVALID_ALARM);
         errlogPrintf("Record [%s] receive error code [0x%08x]!\n", mbir_p->name, 
                      (unsigned int)pvt_p->status);
      }
   }   /* post-process */
   return (rtn);
}


/*********************************************************
 *********** Longin Record Support ***********************
 ********************************************************/


static long Li_init (int after)
{
   return 0;
}


static long Li_init_record (struct longinRecord *lir_p)
{
   struct camacio *cam_ps = &(lir_p->inp.value.camacio);
   short crate = cam_ps->c;
   short slot  = cam_ps->n;
   char *parm_p= cam_ps->parm;  
   vmsstat_t iss = KLYS_OKOK;
   int msgidx;
   unsigned long ctlw;
   unsigned short twobytes = 2;
   unsigned short emaskzero = 0;
   long retval = 0;
   /*------------------------------------------------*/

   /**********************************
   ** First do standard error checking
   ********************************/
   if(lir_p->inp.type!=CAMAC_IO)
   {
      recGblRecordError(S_db_badField, (void *)lir_p, 
                         "devLiPIOP Li_init_record, not CAMAC");
      lir_p->pact=TRUE;
      retval = S_db_badField;
      goto egress;;
   }
   if( (crate > MAX_CRATES) || (slot > MAX_SLOTS) )
   {
      recGblRecordError(S_db_badField, (void *)lir_p, 
                        "devLiPIOP Li_init_record, illegal crate or slot");
      lir_p->pact=TRUE;
      retval = S_db_badField;
      goto egress;;
   }
   /*
   ** Check if is "MSG" param.
   */
   if (strncmp("MSG", parm_p, 3) != 0)
   {
      recGblRecordError(S_db_badField, (void *)lir_p, 
                        "devLiPIOP Li_init_record, illegal param name");
      lir_p->pact=TRUE;
      retval = S_db_badField;
      goto egress;;
   }
   msgidx = atoi(&parm_p[3]);      
   if (msgidx > MAX_PIOPS)
   {
      recGblRecordError(S_db_badField, (void *)lir_p, 
                        "devLiPIOP Li_init_record MSG, bad PIOP number");
      lir_p->pact=TRUE;
      retval = S_db_badField;
      goto egress;;
   }
   if (Piop_Msgs_s[msgidx].stat != 0xFFFFFFFF)
   {
      recGblRecordError(S_db_badField, (void *)lir_p, 
                        "devLiPIOP Li_init_record MSG, duplicate PIOP number");
      lir_p->pact=TRUE;
      retval = S_db_badField;
      goto egress;;
   }
   /*
   ** All params look OK. Add the packet pointing the stat/data to the specified index
   */
   ctlw = (crate << CCTLW__C_shc) | (slot << CCTLW__M_shc)
	              | CCTLW__A2 | CCTLW__A1 | CCTLW__F2;
   if (!SUCCESS(iss = camadd(&ctlw, &Piop_Msgs_s[msgidx], &twobytes, &emaskzero, &msgw_pkg_p)))
   {
      recGblRecordError(S_db_badField, (void *)lir_p, 
                        "devLiPIOP Li_init_record MSG, Camadd failure");
      lir_p->pact=TRUE;
      retval = S_db_badField;
      goto egress;;
   }
egress:
   return (retval);
}


/*
** Read routine for longin record
*/
static long Li_read (struct longinRecord *lir_p)
{
   struct camacio *cam_ps = &(lir_p->inp.value.camacio);
   char  *parm_p= cam_ps->parm; 
   int rtn = 0;
   int msgidx;
   /*---------------------*/
   /*
   ** Move the data from the Camac buffer to the record val.
   */
   msgidx = atoi(&parm_p[3]);
   lir_p->val = Piop_Msgs_s[msgidx].data;      
#ifndef _X86_
   lir_p->val = lir_p->val >> 16; /* Msg code to lower 16 bits of long word */
#endif
   return (rtn);
}


/*********************************************************
 *********** Binary input Record Support *****************
 ********************************************************/


static long Bi_init (int after)
{
   return 0;
}


static long Bi_init_record (struct biRecord *bir_p)
{
   struct camacio *cam_ps = &(bir_p->inp.value.camacio);
   char *parm_p= cam_ps->parm;  
   vmsstat_t iss = KLYS_OKOK;
   pvt_p = *PIOP_PVT;
   long retval = 0;
   /*------------------------------------------------*/
   /*************************
   ** Standard error checking
   **************************/
   if(bir_p->inp.type!=CAMAC_IO)
   {
      recGblRecordError(S_db_badField, (void *)bir_p, 
                         "devBiPIOP Bi_init_record, not CAMAC");
      bir_p->pact=TRUE;
      retval = S_db_badField;
      goto egress;;
   }
   /*
   ** Check if is "MSG" param.
   */
   if (strcmp("MSG", parm_p) != 0)
   {
      recGblRecordError(S_db_badField, (void *)bir_p, 
                        "devBiPIOP Bi_init_record, illegal param name");
      bir_p->pact=TRUE;
      retval = S_db_badField;
      goto egress;;
   }
   iss = PIOPDriverInit((dbCommon *)bir_p, *cam_ps, EPICS_RECTYPE_BI);
   pvt_p = (PIOP_PVT *)(bir_p->dpvt);
   pvt_p->camfunc_e = SBIMSGPIOP;
   pvt_p->val_p = sor_p->val;
egress:
   return (retval);
}

/*
** Read routine for binary input record
*/
static long Bi_read (struct biRecord *bir_p)
{
   struct camacio *cam_ps = &(bir_p->inp.value.camacio);
   short  crate = cam_ps->c;
   short  slot  = cam_ps->n;
   char  *parm_p= cam_ps->parm;
   THREADMSG_TS msg_s;
   PIOP_PVT *pvt_p = (PIOP_PVT *)(bir_p->dpvt);
   int rtn = -1;        /* Assume bad */
   /*---------------------*/
   if (!pvt_p) return (rtn);   /* Bad. Should have a driver private area */
   /*
   ** Send msg to thread if not active else complete record processing.
   */
   if(!bir_p->pact)
   {  /* Pre-process */
      /* 
      /** This camac pkg reads the msg word for all PIOPS. Use the single 
      ** SBI thread so we don't have to use a specific PIOP thread.
      */
      msg_s.rec_p = (dbCommon *)bir_p;
      if (epicsMessageQueueTrySend (sbi_msgQId, &msg_s, sizeof(msg_s)) == -1)
      {
         recGblSetSevr(bir_p, WRITE_ALARM, INVALID_ALARM);
         errlogPrintf("Record [%s] PIOP MSG can't send msg to thread", bir_p->name);
      }
      else
      {
         bir_p->pact = TRUE;
         rtn = 0;     /* Return OK */
      }
   }
   else
   { /* post-process */
      rtn = 0;     /* Always return good status */
      if (!SUCCESS(pvt_p->status))  /* Check for different error options?? */
      {
         recGblSetSevr(bir_p, WRITE_ALARM, INVALID_ALARM);
         errlogPrintf("Record [%s] receive error code [0x%08x]!\n", bir_p->name, 
                      (unsigned int)pvt_p->status);
      }
   }   /* post-process */
   return (rtn);
}
