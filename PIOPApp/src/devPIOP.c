/***************************************************************************\
 *   $Id: devPIOP.c,v 1.17 2016/05/09 22:30:17 luchini Exp $
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
/*********************         Externs de
fined in devPIOP          ************************/
/******************************************************************************************/
int PIOP_DEV_DEBUG;

/******************************************************************************************/
/*********************  Externs defined in drvPIOP or fidPHASE     ************************/
/******************************************************************************************/

extern int PIOP_DRV_DEBUG;

/*
** MessageQueue associated with the thread for each PIOP module.
*/
extern epicsMessageQueueId piop_msgQId[MAX_CRATES][MAX_SLOTS];

/*
** MessageQueue for SBI thread assume one per IOC.
*/
extern epicsMessageQueueId sbi_msgQId;

extern epicsThreadId fidPHASEStart (void);
/*
** Camac package and stat/data array for PIOP message word from drvPIOP
*/
extern void *Msgw_pkg_p;
extern STAT_DAT16 Piop_Msgs_s[MAX_PIOPS];

/*
** Camac package, stat/data array & IOSCANPVT for PIOP phase words from drvPIOP
*/
extern void *Phase_pkg_p;
extern STAT_DAT16 Phase_s[MAX_PIOPS];
extern IOSCANPVT PhaseIoPvt;
static int phase_idx = 0;     /* To assign phase index as records are initialized */

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
static long So_init_record (struct stringoutRecord *sor_p);
static long So_write (struct stringoutRecord *sor_p);
DEV_SUP devSoPIOP = {6, NULL, NULL, So_init_record, NULL, So_write, NULL};
epicsExportAddress(dset, devSoPIOP);

/*
** Waveform PIOP device support functions
*/
static long Wf_init_record (struct waveformRecord *wfr_p);
static long Wf_read_write (struct waveformRecord *wfr_p);
DEV_SUP devWfPIOP = {6, NULL, NULL, Wf_init_record, NULL, Wf_read_write, NULL};
epicsExportAddress(dset, devWfPIOP);

/*
** MBBI SBI device support functions
*/
static long Mbi_init_record (struct mbbiRecord *mbir_p);
static long Mbi_read (struct mbbiRecord *mbir_p);
DEV_SUP devMbiSBI = {6, NULL, NULL, Mbi_init_record, NULL, Mbi_read, NULL};
epicsExportAddress(dset, devMbiSBI);

/*
** binary input device support functions
*/
static long Bi_init_record (struct biRecord *bir_p);
static long Bi_read (struct biRecord *bir_p);
DEV_SUP devBiPIOP = {6, NULL, NULL, Bi_init_record, NULL, Bi_read, NULL};
epicsExportAddress(dset, devBiPIOP);

/*
** Longin device support functions
*/
static long Li_init_record (struct longinRecord *lir_p);
static long Li_read (struct longinRecord *lir_p);
DEV_SUP devLiPIOP = {6, NULL, NULL, Li_init_record, NULL, Li_read, NULL};
epicsExportAddress(dset, devLiPIOP);

/*
** Longout device support functions
*/
static long Lo_init_record (struct longoutRecord *lor_p);
static long Lo_write (struct longoutRecord *lor_p);
DEV_SUP devLoSBI = {6, NULL, NULL, Lo_init_record, NULL, Lo_write, NULL};
epicsExportAddress(dset, devLoSBI);

/*
** Analog input device support functions for phase read
*/
static long Ai_init (int); 
static long Ai_init_record (struct aiRecord *air_p);
static long Ai_ioint_info (int cmd, struct aiRecord *air_p, IOSCANPVT *iopvt); 
static long Ai_read (struct aiRecord *air_p);
DEV_SUP devAiPIOP = {6, NULL, Ai_init, Ai_init_record, Ai_ioint_info, Ai_read, NULL};
epicsExportAddress(dset, devAiPIOP);


/*********************************************************
 ******** Stringout Record Support; PIOP IPL *************
 ********************************************************/

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
   sprintf (tname, "PIOP%2.2u%2.2u",crate,slot);
   epicsThreadMustCreate(tname, epicsThreadPriorityMedium, 20480,
                         threadPIOP, (void *)piop_msgQId[crate][slot]);
   /*
   ** Init and set up common info in dpvt our private space for
   ** communication between record processing and the Camac thread.
   ** PIOPDriverInit sets crate, slot & inits status.
   */
   PIOPDriverInit((dbCommon *)sor_p, cam_ps, EPICS_RECTYPE_SO);
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
      if (!SUCCESS(pvt_p->status))
      {
         recGblSetSevr(sor_p, WRITE_ALARM, INVALID_ALARM);
         if (PIOP_DRV_DEBUG)   printf("Record [%s] error %s!\n", sor_p->name, cammsg(pvt_p->status));
      }
   }   /* post-process */ 
   return (rtn);
}

/*********************************************************
 *********** Waveform Record Support *********************
 ********************************************************/

/*
** Local routine to translate Waveform parm into a camac function CAMFUNC_TE.
*/
typedef struct
{
  char       *parm_p;
  CAMFUNC_TE  func_e;
} CKPARM_S;

static CKPARM_S ckparm_as[] = 
                { {"PPN",PPNOFTP},
                  {"PPF",PPFTP},
                  {"STSB",STATUSBLOCK},
                  {"FTP",FTP},
                  {"PAD",PAD},
                  {"MK2",MK2},
                  {"SLED",TRIMSLED},
                  {"FOX",FOXHOME}
                };

static CAMFUNC_TE xlate_wf_parm (char *inparm_p)
{
   int j;
   CAMFUNC_TE camfunc_e = INVALID; /* Assume error */
   /*------------------------------------------------*/
   for (j=0; j<(sizeof(ckparm_as)/sizeof(CKPARM_S)); j++)
   {
      if (strcmp(ckparm_as[j].parm_p, inparm_p) == 0)
      {
         camfunc_e = ckparm_as[j].func_e;
         break;
      }
   }
   return (camfunc_e);
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
   ** Insure the PARM is one we know about.If ftp also get its index. 
   */
   if ((camfunc_e = xlate_wf_parm(parm_p)) == INVALID)
   {
      recGblRecordError(S_db_badField, (void *)wfr_p, 
                        "devWfPIOP Wf_init_record, invalid parameter");
      wfr_p->pact=TRUE;
      return (S_db_badField);
   }     
   PIOPDriverInit((dbCommon *)wfr_p, cam_ps, EPICS_RECTYPE_WF);
   pvt_p = (PIOP_PVT *)(wfr_p->dpvt);
   pvt_p->camfunc_e = camfunc_e;
   pvt_p->val_p = wfr_p->bptr; /* Buffer pointer for waveform */
   return (0);
}

/*
** Read/Write routine for Waveform record
*/
static long Wf_read_write (struct waveformRecord *wfr_p)
{
   struct camacio *cam_ps = &(wfr_p->inp.value.camacio);
   short  crate = cam_ps->c;
   short  slot  = cam_ps->n;
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
      if (!SUCCESS(pvt_p->status))
      {
         recGblSetSevr(wfr_p, WRITE_ALARM, INVALID_ALARM);
         if(pvt_p->status != CAM_NGNG)
                     if (PIOP_DRV_DEBUG)   printf("Record [%s] error %s!\n", wfr_p->name, cammsg(pvt_p->status));
      }
      else if (pvt_p->camfunc_e == STATUSBLOCK)
      {
         wfr_p->nord = wfr_p->nelm;
      }
   }   /* post-process */
   return (rtn);
}

/*********************************************************
 *********** Mbbi Record Support *************************
 ********************************************************/

/*
** Record init for Mbbi
*/
static long Mbi_init_record (struct mbbiRecord *mbir_p)
{
   struct camacio *cam_ps = &(mbir_p->inp.value.camacio);
   short crate = cam_ps->c;
   short slot  = cam_ps->n;
   char *parm_p= cam_ps->parm;
   PIOP_PVT *pvt_p;
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
   if (strcmp("STS",parm_p) != 0)
   {
       recGblRecordError(S_db_badField, (void *)mbir_p, 
                        "devMbiSBI Mbi_init_record, illegal parm string");
      mbir_p->pact=TRUE;
      return (S_db_badField);
  }
   PIOPDriverInit((dbCommon *)mbir_p, cam_ps, EPICS_RECTYPE_MBBI);
   pvt_p = (PIOP_PVT *)(mbir_p->dpvt);
   pvt_p->camfunc_e = SBISTATUS;
   pvt_p->val_p = &(mbir_p->val);
   return (0);
}

/*
** Read/Write routine for SBI Mbbi record
*/
static long Mbi_read (struct mbbiRecord *mbir_p)
{
   THREADMSG_TS msg_s;
   PIOP_PVT *pvt_p = (PIOP_PVT *)(mbir_p->dpvt);
   unsigned short *mbival_p;  /* To retrieve value from dev pvt struct */
   int rtn = -1;   /* Assume bad */
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
         if (PIOP_DRV_DEBUG)   printf("Record [%s] SBI STATUS can't send msg to thread", mbir_p->name);
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
      mbival_p = pvt_p->val_p;  /* Value ptr from pvt */
      mbir_p->rval = *mbival_p;  /* Get value from driver */
      if (!SUCCESS(pvt_p->status))
      {
         recGblSetSevr(mbir_p, WRITE_ALARM, INVALID_ALARM);
         if (pvt_p->status != CAM_NGNG)
           if (PIOP_DRV_DEBUG)   printf("Record [%s] error %s!\n", mbir_p->name,cammsg(pvt_p->status));
      }
   }   /* post-process */
   return (rtn);
}


/*********************************************************
 *********** Binary input Record Support *****************
 ********************************************************/

static long Bi_init_record (struct biRecord *bir_p)
{
   struct camacio *cam_ps = &(bir_p->inp.value.camacio);
   char *parm_p= cam_ps->parm;  
   PIOP_PVT *pvt_p;
   /*------------------------------------------------*/
   /*************************
   ** Standard error checking
   **************************/
   if(bir_p->inp.type!=CAMAC_IO)
   {
      recGblRecordError(S_db_badField, (void *)bir_p, 
                         "devBiPIOP Bi_init_record, not CAMAC");
      bir_p->pact=TRUE;
      return (S_db_badField);
   }
   /*
   ** Check if is "MSG" param.
   */
   if (strcmp("MSG", parm_p) != 0)
   {
      recGblRecordError(S_db_badField, (void *)bir_p, 
                        "devBiPIOP Bi_init_record, illegal param name");
      bir_p->pact=TRUE;
      return (S_db_badField);
   }
   PIOPDriverInit((dbCommon *)bir_p, cam_ps, EPICS_RECTYPE_BI);
   pvt_p = (PIOP_PVT *)(bir_p->dpvt);
   pvt_p->camfunc_e = SBIMSGPIOP;
   pvt_p->val_p = &(bir_p->val);
   return (0);
}

/*
** Read routine for binary input record
*/
static long Bi_read (struct biRecord *bir_p)
{
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
      ** This camac pkg reads the msg word for all PIOPS. Use the single 
      ** SBI thread so we don't have to use a specific PIOP thread.
      */
      msg_s.rec_p = (dbCommon *)bir_p;
      if (epicsMessageQueueTrySend (sbi_msgQId, &msg_s, sizeof(msg_s)) == -1)
      {
         recGblSetSevr(bir_p, WRITE_ALARM, INVALID_ALARM);
         if (PIOP_DRV_DEBUG)   printf("Record [%s] PIOP MSG can't send msg to thread", bir_p->name);
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
      if (!SUCCESS(pvt_p->status))
      {
         recGblSetSevr(bir_p, WRITE_ALARM, INVALID_ALARM);
         if (pvt_p->status != CAM_NGNG)
            if (PIOP_DRV_DEBUG)   printf("Record [%s] error %s!\n", bir_p->name, cammsg(pvt_p->status));
      }
   }   /* post-process */
   return (rtn);
}


/*********************************************************
 *********** Longin Record Support ***********************
 ********************************************************/

static long Li_init_record (struct longinRecord *lir_p)
{
   struct camacio *cam_ps = &(lir_p->inp.value.camacio);
   short crate = cam_ps->c;
   short slot  = cam_ps->n;
   char *parm_p= cam_ps->parm;  
   vmsstat_t iss = CAM_OKOK;
   int msgidx;
   unsigned int ctlw;
   unsigned short twobytes = 2;
   unsigned short emaskzero = 0;

   /*------------------------------------------------*/

   /**********************************
   ** First do standard error checking
   ********************************/
   if(lir_p->inp.type!=CAMAC_IO)
   {
      recGblRecordError(S_db_badField, (void *)lir_p, 
                         "devLiPIOP Li_init_record, not CAMAC");
      lir_p->pact=TRUE;
      return (S_db_badField);
   }
   if( (crate > MAX_CRATES) || (slot > MAX_SLOTS) )
   {
      recGblRecordError(S_db_badField, (void *)lir_p, 
                        "devLiPIOP Li_init_record, illegal crate or slot");
      lir_p->pact=TRUE;
      return (S_db_badField);
   }
   /*
   ** Check if is "MSG" param.
   */
   if (strncmp("MSG", parm_p, 3) != 0)
   {
      recGblRecordError(S_db_badField, (void *)lir_p, 
                        "devLiPIOP Li_init_record, illegal param name");
      lir_p->pact=TRUE;
      return (S_db_badField);
   }
   msgidx = atoi(&parm_p[3]);      
   if (msgidx > MAX_PIOPS)
   {
      recGblRecordError(S_db_badField, (void *)lir_p, 
                        "devLiPIOP Li_init_record MSG, bad PIOP number");
      lir_p->pact=TRUE;
      return (S_db_badField);
   }

   if (Piop_Msgs_s[msgidx].stat != 0xFFFFFFFF)
   {
      recGblRecordError(S_db_badField, (void *)lir_p, 
                        "devLiPIOP Li_init_record MSG, duplicate PIOP number");
      lir_p->pact=TRUE;
      return (S_db_badField);
   }

   /*
   ** All params look OK. Add the packet pointing the stat/data to the specified index
   */
   ctlw = (crate << CCTLW__C_shc) | (slot << CCTLW__M_shc)
	              | CCTLW__A2 | CCTLW__A1 | CCTLW__F2;
   if (!SUCCESS(iss = camadd(&ctlw, &(Piop_Msgs_s[msgidx]), &twobytes, &emaskzero, &Msgw_pkg_p)))
   {
      recGblRecordError(S_db_badField, (void *)lir_p, 
                        "devLiPIOP Li_init_record MSG, Camadd failure");
      lir_p->pact=TRUE;
      return (S_db_noMemory);
   }
   return (0);
}


/*
** Read routine for longin record
*/
static long Li_read (struct longinRecord *lir_p)
{
   struct camacio *cam_ps = &(lir_p->inp.value.camacio);
   char   errmsg_c[256];
   char  *parm_p= cam_ps->parm; 
   unsigned int stat;
   PIOP_PVT *pvt_p= (PIOP_PVT *)lir_p->dpvt;

   int rtn = 0;
   int msgidx; 
   /*---------------------*/
   /*
   ** Move the data from the Camac buffer to the record val.
   *
   */
   msgidx = atoi(&parm_p[3]);
   
   /* should we check camac status? */
   if (!PIOP_DEV_DEBUG) 
     lir_p->val = lir_p->val = Piop_Msgs_s[msgidx].data; 
   /* Check if camgo was successful */
   else 
   {         
      /*
       * Check X, Q and BAR of camac status, lower byte only
       * The RMS software also checks crate online, but we don't here.
       */
       stat = Piop_Msgs_s[msgidx].stat;
       if ( PIOP_MSG_WD_OK(stat) ) {
         if ((lir_p->val==Piop_Msgs_s[msgidx].data) && (PIOP_DEV_DEBUG==2))
           epicsPrintf("devLiPIOP_MSG: %s  same fault as last time %d\n",lir_p->name,lir_p->val,(unsigned long int));
         lir_p->val = Piop_Msgs_s[msgidx].data;  
       }
       else if (PIOP_DEV_DEBUG==2)
         epicsPrintf(sprintf("devLiPIOP MSG, status=0x%lx data=%hd",(long unsigned int)stat,Piop_Msgs_s[msgidx].data );
   }
   return (rtn);
}


/*********************************************************
 *********** Longout Record Support ***********************
 ********************************************************/

static long Lo_init_record (struct longoutRecord *lor_p)
{
   struct camacio *cam_ps = &(lor_p->out.value.camacio);
   short crate = cam_ps->c;
   short slot  = cam_ps->n;
   char *parm_p= cam_ps->parm;
   PIOP_PVT *pvt_p = (PIOP_PVT *)(lor_p->dpvt);
   /*------------------------------------------------*/

   /**********************************
   ** First do standard error checking
   ********************************/
   if(lor_p->out.type!=CAMAC_IO)
   {
      recGblRecordError(S_db_badField, (void *)lor_p, 
                         "devLoSBI Lo_init_record, not CAMAC");
      lor_p->pact=TRUE;
      return (S_db_badField);
   }
   if( (crate > MAX_CRATES) || (slot > MAX_SLOTS) )
   {
      recGblRecordError(S_db_badField, (void *)lor_p, 
                        "devLoSBI Lo_init_record, illegal crate or slot");
      lor_p->pact=TRUE;
      return (S_db_badField);
   }
   /*
   ** Check if is "DELAY" param.
   */
   if (strcmp("DELAY", parm_p) != 0)
   {
      recGblRecordError(S_db_badField, (void *)lor_p, 
                        "devLoSBI Lo_init_record, illegal param name");
      lor_p->pact=TRUE;
      return (S_db_badField);
   }
   /*
   ** All params look OK. Init driver private struct.
   */
   PIOPDriverInit((dbCommon *)lor_p, cam_ps, EPICS_RECTYPE_LO);
   pvt_p = (PIOP_PVT *)(lor_p->dpvt);
   pvt_p->camfunc_e = SBIDELAY;
   pvt_p->val_p = &(lor_p->val);
  return (0);
}

/*
** Write routine for longout record
*/
static long Lo_write (struct longoutRecord *lor_p)
{
   THREADMSG_TS msg_s;
   PIOP_PVT *pvt_p = (PIOP_PVT *)(lor_p->dpvt);
   int rtn = -1;        /* Assume bad */
   /*---------------------*/
   if (!pvt_p) return (rtn);   /* Bad. Should have a driver private area */
   /*
   ** Send msg to SBI thread if not active else complete record processing.
   */
   if(!lor_p->pact)
   {  /* Pre-process */
     /* 
      ** This camac pkg writes the SBI delay and PSK enable to the SBI. 
      */
      msg_s.rec_p = (dbCommon *)lor_p;
      if (epicsMessageQueueTrySend (sbi_msgQId, &msg_s, sizeof(msg_s)) == -1)
      {
         recGblSetSevr(lor_p, WRITE_ALARM, INVALID_ALARM);
         if (PIOP_DRV_DEBUG)   printf("Record [%s] can't send msg to SBI thread", lor_p->name);
      }
      else
      {
         lor_p->pact = TRUE;
         rtn = 0;     /* Return OK */
      }
   }
   else
   { /* post-process */
      rtn = 0;     /* Always return good status */
      if (!SUCCESS(pvt_p->status))
      {
         recGblSetSevr(lor_p, WRITE_ALARM, INVALID_ALARM);
         if (pvt_p->status != CAM_NGNG)
            if (PIOP_DRV_DEBUG)   printf("Record [%s] error %s!\n", lor_p->name, cammsg(pvt_p->status));
      }
   }   /* post-process */
   return (rtn);
}

/*********************************************************
 ******* Analog input Record Support for phase read ******
 ********************************************************/
/*
** Called twice during IOC initialization for module setup.
** final=0 called before database records are initialized
** final=1 called after database records were initialized
**
** Start the task to read the phase on each beam crossing
** after all records initialized.
*/

static long Ai_init (int final)
{
   if (final)
      fidPHASEStart();  /* Start fiducial phase collection thread */
   return 0;
}

/*
** Record init for Analog input
*/
static long Ai_init_record (struct aiRecord *air_p)
{
   struct camacio *cam_ps = &(air_p->inp.value.camacio);
   short crate = cam_ps->c;
   short slot  = cam_ps->n;
   char *parm_p= cam_ps->parm;
   unsigned int ctlw;      /* Camac ctlw for camadd */
   unsigned short emaskf200 = 0xF200; /* emask no msgs; called from intrpt level */
   unsigned short bcnt = 2; /* 2 byte status word */
   PIOP_PVT      *pvt_p;
   /*------------------------------------------------*/
   if(air_p->inp.type!=CAMAC_IO)
   {
      recGblRecordError(S_db_badField, (void *)air_p, 
                         "devAiPIOP Ai_init_record, not CAMAC");
      air_p->pact=TRUE;
      return(S_db_badField);
   }
   if( (crate > MAX_CRATES) || (slot > MAX_SLOTS) )
   {
      recGblRecordError(S_db_badField, (void *)air_p, 
                        "devAiPIOP Ai_init_record, illegal crate or slot");
      air_p->pact=TRUE;
      return(S_db_badField);
   }
   /*
   ** Check if is "PHASE" param.
   */
   if (strcmp("PHASE", parm_p) != 0)
   {
      recGblRecordError(S_db_badField, (void *)air_p, 
                        "devAiPIOP Ai_init_record, illegal param name");
      air_p->pact=TRUE;
      return (S_db_badField);
   }
   /*
   ** Check that we don't have too many PHASE records.
   */
   if (phase_idx > MAX_PIOPS-1)
   {
      recGblRecordError(S_db_badField, (void *)air_p, 
                        "devAiPIOP Ai_init_record, too many PHASE records");
      air_p->pact=TRUE;
      return (S_db_badField);
   }
   /*
   ** Add camac packets for this phase.
   */
   ctlw = (crate << CCTLW__C_shc) | (slot << CCTLW__M_shc) | CCTLW__F2 | CCTLW__A2;
   if (!SUCCESS(camadd(&ctlw, &Phase_s[phase_idx], &bcnt, &emaskf200,  &Phase_pkg_p)))
   {
      recGblRecordError(S_db_badField, (void *)air_p, 
                        "devAiPIOP Ai_init_record, camadd error.");
      air_p->pact=TRUE;
      return (S_db_badField);
   } 
   /*
   ** All params look OK. Init driver private struct.
   */
   PIOPDriverInit((dbCommon *)air_p, cam_ps, EPICS_RECTYPE_AI);
   pvt_p = (PIOP_PVT *)(air_p->dpvt);
   pvt_p->val_p = &(air_p->val);
   pvt_p->phase_idx = phase_idx++;
   return (0);
}

/*
** Ioint_info routine to add this record to our IOSCANPVT list.
*/
static long Ai_ioint_info (int cmd, struct aiRecord *air_p, IOSCANPVT *iopvt)
{
   *iopvt = PhaseIoPvt;   /* Add this record to the list */
   return 0;
} 

/*********************************
** Read routine for PIOP ai record
**********************************/

static long Ai_read (struct aiRecord *air_p)
{
   PIOP_PVT *pvt_p = (PIOP_PVT *)(air_p->dpvt);
   /*---------------------*/
   if (!pvt_p) return (-1);   /* Bad. Should have a driver private area ??*/
   /*
   ** Record only activated if Camac is OK. 
   */
   air_p->rval = Phase_s[pvt_p->phase_idx].data;  /* Just save raw 16 bit value */
   return (0);
}

