/*
=============================================================

  Abs:  EPICS Device Support for the CAMAC Crate Verifier 

  Name: devCV.c

         Utilities:
         ----------
             CV_FindFuncIndex         - Find function index from string 
	 *   CV_RequestInit           - Device Support Initalization
         *   get_ioint_info           - Get I/O event list info

         Analog Input Device Support:
         ----------------------------
         *   init_ai                  - initialization
         *   read_ai                  - read analog input
         *   get_ioint_info_ai        - Get I/O event list info

         Binary Input Device Support:
         -----------------------------
         *   init_bo                  - initialization
         *   read_bo                  - read analog input

         Longinput Input Device Support:
         -----------------------------
         *   init_longin              - initialization
         *   read_longin              - read analog input
         *   get_ioint_info_longin    - Get I/O event list info

         Multibit-Binary Input Device Support:
         ------------------------------------
         *   init_mbbiDirect          - initialization
         *   read_mbbiDirect          - read analog input

         Waveform Device Support:
         -----------------------------
         *   init_wf                   - initialization
         *   read_wf                   - read analog input 
 

  Proto: None

  Auth: 19-Jul-2009, K. Luchini       (LUCHINI)
  Rev : dd-mmm-yyyy, Reviewer's Name  (USERNAME)

-------------------------------------------------------------
  Mod:
        dd-mmm-yyyy, First Lastname   (USERNAME):
          comment

=============================================================
*/

/* Header Files */
#include "drvPSCDLib.h"
#include "devCV.h"
#include "dbFldTypes.h"            /* for ULONG                   */
#include "aiRecord.h"              /* for aiRecord                */
#include "boRecord.h"              /* for boRecord                */
#include "longinRecord.h"          /* for longinRecord            */
#include "mbbiDirectRecord.h"      /* for struct mbbiDirectRecord */
#include "waveformRecord.h"        /* for struct waveform         */
#include "genSubRecord.h"          /* for struct genSubRecord     */
#include "drvCV_proto.h" 

/* Local Prototypes */
static long  CV_RequestInit(dbCommon * const rec_ps, struct camacio const * const inout_ps, cv_epics_rtyp_te rtyp_e );
static short CV_FindFuncIndex( char const * const req_c, cv_epics_rtyp_te rtyp_e );

static long init_ai(struct aiRecord * rec_ps);
static long read_ai(struct aiRecord * rec_ps);
static long init_bo(struct boRecord * rec_ps);
static long write_bo(struct boRecord * rec_ps);
static long init_longin(struct longinRecord * rec_ps);
static long read_longin(struct longinRecord * rec_ps);
static long init_mbbiDirect(struct mbbiDirectRecord * rec_ps);
static long read_mbbiDirect(struct mbbiDirectRecord * rec_ps);
static long init_wf(struct waveformRecord * rec_ps);
static long read_wf(struct waveformRecord * rec_ps);
static long get_ioint_info(     int cmd, dbCommon * rec_ps, IOSCANPVT * evt_pp );

/* Local variable */
static const double  slope = CV_ANLG_SLOPE;            /* 8-bits */
static const char   *opDoneStatus_ac[]={"No","Yes"};   /* Requeset operation completion status (0,1) */

/* Device Support structure */
typedef struct {
        long            number;
        DEVSUPFUN       report;
        DEVSUPFUN       init;
        DEVSUPFUN       init_record;
        DEVSUPFUN       get_ioint_info;
        DEVSUPFUN       read_write;
        DEVSUPFUN       special_linconv;
} DSET;


/*Device support entry table */
DSET      devAiCV                = {6, NULL, NULL, init_ai        , get_ioint_info , read_ai         , NULL };
DSET      devLonginCV            = {5, NULL, NULL, init_longin    , get_ioint_info , read_longin     , NULL };
DSET      devMbbiDirectCV        = {5, NULL, NULL, init_mbbiDirect, get_ioint_info , read_mbbiDirect , NULL };
DSET      devWfCV                = {5, NULL, NULL, init_wf        , get_ioint_info , read_wf         , NULL };
DSET      devBoCV                = {5, NULL, NULL, init_bo        , NULL           , write_bo        , NULL };

epicsExportAddress(dset, devAiCV);
epicsExportAddress(dset, devLonginCV);
epicsExportAddress(dset, devMbbiDirectCV);
epicsExportAddress(dset, devWfCV);
epicsExportAddress(dset, devBoCV);

/* Global variables */ 
int CV_DEV_DEBUG=0;


/*=============================================================

  Abs:  Analog Input Device Support initialization

  Name: init_ai

  Args: rec_ps                          Record information
          Use:  struct
          Type: aiRecord *
          Acc:  read-write access
          Mech: By reference

  Rem: This device support routine is called by the record
       support function init_record(). Its purpose it to
       initializes analog input records.

       The Camac information is extracted from the INP field,
       which includes: branch, crate and slot location of the
       crate verifier module. The module linked list is searched
       to see if the verifier module exists. If it does, then the
       PARM portion if the field (ie. after the @) is searched for 
       a valid verifier command for this record type.

       The analog input record example of for the INP field 
       is shown below.

        field(INP,  "CAMAC_IO - #B0 C1 N1 A$(A) F5 @VOLTS")

       where the macro A=0-7, inclusive

       Please see the structure CV_CAMAC_FUNC in devCV.h for a complete
       list of Camac functions for which device support has been provided. 

  Side: CAMAC_IO is the only bus type supported

  Ret: long
         NO_CONVERT      - Successful operation
         S_dev_badSignal - Failure due to invalid Camac command request, (ie. PARM field)
         S_dev_badbus    - Failure due to bus type not CAMAC_IO

=============================================================*/
static long init_ai(struct aiRecord * rec_ps)
{
    long             status     = NO_CONVERT;
    struct camacio  *camacio_ps = NULL;


    switch (rec_ps->inp.type) 
    {
       case CAMAC_IO:                  /* CAMAC bus */
  	  /* Check that signal range is valid */
	  camacio_ps = &rec_ps->inp.value.camacio;
          if ((camacio_ps->a >= CV_MIN_ANLG_SUBADR) && (camacio_ps->a<=CV_MAX_ANLG_SUBADR)) 
	  {
             status = CV_RequestInit((dbCommon *)rec_ps, camacio_ps, EPICS_RECTYPE_AI);
	     if (status==OK)
	     {
               rec_ps->eslo = ((rec_ps->eguf - rec_ps->egul)/CV_ANLG_MASK) * 1000; /* mv to volts */
	       rec_ps->roff = CV_ANLG_ZERO;
	     }
	  }
          else
             status = S_dev_badSignal;
          break;

       default:                       /* Bus type is not supported */
          status = S_dev_badBus;
          break;
    }/* End of switch */

    if ( status )
    {
       recGblRecordError(status,(void *)rec_ps ,"devAiCV(init): Illegal INP field");
       rec_ps->pact=TRUE;
    }
    
    return (status);
}

/*=============================================================

  Abs:  Device Support for io scanner init

  Name: get_ioint_info

  Args: cmd                        Command being performed
          Use:  integer
          Type: int
          Acc:  read-only
          Mech: By value

        rec_p                      Record information
          Use:  struct
          Type: void *
          Acc:  read-write access
          Mech: By reference

        evt_pp                     I/O scan event
          Use:  struct
          Type: IOSCANPVT *
          Acc:  read-write access
          Mech: By reference

  Rem:  This device support provides access to the IOSCANPVT
        structure associated with the specified adc card
        defined for this pv..

  Side: This routine can be called at interrupt level
        to process an event.

  Ret: long
            OK - Successful operation (always returned)

=============================================================*/
static long get_ioint_info( int cmd, dbCommon *rec_ps, IOSCANPVT *evt_pp )
{
    long                  status   = OK;         /* status return           */
    CV_REQUEST           *dpvt_ps  = NULL;       /* private device info     */
    cv_message_status_ts *mstat_ps = NULL;       /* function request status */


    if (rec_ps->dpvt) 
    {
       dpvt_ps = rec_ps->dpvt;
       mstat_ps = dpvt_ps->mstat_ps;
       if ( !mstat_ps->evt_p )
       {
         scanIoInit( &mstat_ps->evt_p );
         *evt_pp = mstat_ps->evt_p;
       }
       else
         *evt_pp = dpvt_ps->mstat_ps->evt_p;
    }
    else
      *evt_pp = NULL;
    return( status );
}


/*=============================================================

  Abs:  Analog Input device support

  Name: read_ai

  Args: rec_ps                      Record information
          Use:  struct
          Type: aiRecord *
          Acc:  read-write access
          Mech: By reference

  Rem: This routine processes a analog input record.
       The floating point read from the specified hardware
       memeory location, and stored into the VAL field.

       If an error occurs the STAT and SEVR fiels of the record
       are set accordingly.

  Side: Conversion from a raw value to engineering units
        will not be performed if the field "LINR" is zero.

  Ret: long
         NO_CONVERT - Successful operation ( no conversion)
         ERROR      - Failure occured during writ
         Otherwise, see return from

=============================================================*/
static long read_ai(struct aiRecord * rec_ps)
{
    long                   status    = NO_CONVERT;
    unsigned short         nsta      = READ_ALARM;
    unsigned short         nsev      = INVALID_ALARM;
    campkg_volts_ts       *cam_ps    = NULL;
    CV_REQUEST            *dpvt_ps   = (CV_REQUEST *)(rec_ps->dpvt);
    CV_MODULE             *module_ps = NULL;
    cv_message_status_ts  *mstat_ps  = NULL;
    char                   errmsg_c[40];


    if(!dpvt_ps || !dpvt_ps->module_ps) return(status);

    module_ps = dpvt_ps->module_ps;  
    cam_ps    = &module_ps->cam_s.rd_volts_s;
    mstat_ps  = dpvt_ps->mstat_ps; 
    if( (!mstat_ps->opDone) || !SUCCESS(mstat_ps->errCode) )
    { 
	if (module_ps->crate_s.stat_u._i & CRATE_STATUS_CTO_ERR) 
           nsta = TIMEOUT_ALARM;
        if ( recGblSetSevr(rec_ps,nsta,nsev) && 
             errVerbose                       && 
            (rec_ps->stat!=nsta ||rec_ps->sevr!=nsev) ) 
	{
	  /*sprintf(errmsg_c,"CV: error [0x%08lx] opDone=%s",
                   mstat_ps->errCode,
		   opDoneStatus_ac[mstat_ps->opDone]); */
           recGblRecordError(ERROR,(void *)rec_ps,errmsg_c);
	}
    }
    else
    {
        if (rec_ps->tse == epicsTimeEventDeviceTime)  
           rec_ps->time = mstat_ps->reqTime;
        rec_ps->val  = module_ps->crate_s.volts_a[dpvt_ps->a];
        rec_ps->udf  = FALSE;       
    }

    return(status);
}


/*=============================================================

  Abs:  Binary Output Device Support initialization

  Name: init_bo

  Args: rec_ps                          Record information
          Use:  struct
          Type: boRecord *
          Acc:  read-write access
          Mech: By reference

  Rem: This device support routine is called by the record
       support function init_record(). Its purpose it to
       initializes binary output records.

       The Camac information is extracted from the OUT field,
       which includes: branch, crate and slot location of the
       crate verifier module. The module linked list is searched
       to see if the verifier module exists. If it does, then the
       PARM portion if the field (ie. after the @) is searched for 
       a valid verifier command for this record type.

       The binary output record example of for the OUT field 
       is shown below.

           field(OUT, "CAMAC_IO - #B0 C1 N1 A0 F0 @VERIFY")
           field(OUT, "CAMAC_IO - #B0 C1 N1 A0 F0 @OFFLINE")

       The "VERIFY" requests a Camac crate dataway test be performed.
       The "OFFLINE" request is used to set the crate verifier module
       in-service (ie. 0) or out-of-service(ie. 1) in the database. 
       However, this is not currently supported.

       Please see the structure CV_CAMAC_FUNC in devCV.h for a complete
       list of Camac functions for which device support has been provided. 

  Side: CAMAC_IO is the only bus type supported

  Ret: long
         NO_CONVERT      - Successful operation
         S_dev_badSignal - Failure due to invalid Camac command request, (ie. PARM field)
         S_dev_badbus    - Failure due to bus type not CAMAC_IO

=============================================================*/
static long init_bo(struct boRecord * rec_ps)
{
    long status=OK;
  
    switch( rec_ps->out.type )
    {
        case CAMAC_IO:
          status = CV_RequestInit((dbCommon *)rec_ps, &rec_ps->out.value.camacio, EPICS_RECTYPE_BO);
          break;

        default:
          status = S_dev_badBus;
          break;
    }

    if(status)
    {
       recGblRecordError(status,(void *)rec_ps, "devBoCV Init_record failed");
       rec_ps->pact=TRUE;
    }

    return(OK);
}

/*=============================================================

  Abs:  Binary Output device support

  Name: read_bo

  Args: rec_ps                      Record information
          Use:  struct
          Type: boRecord *
          Acc:  read-write access
          Mech: By reference

  Rem: This routine processes a binary output record which
       supports the follwing requests:

         VERIFY   - send a request to the message queue for 
                    a Camac crate dataway test (ie. verifier test)

         OFFLINE -  set the crate verifier module out-of-service
                    or in-service from the database (not implimented yet)

       If an error occurs the STAT and SEVR fiels of the record
       are set accordingly.

  Side: None

  Ret: long
         NO_CONVERT  - Operation Successful (pre-processing/post-processing) 
         ERROR       - Operation failed
       
=============================================================*/
static long write_bo(struct boRecord * rec_ps)
{
    long                  status    = ERROR;
    unsigned short        nsta      = WRITE_ALARM;
    unsigned short        nsev      = INVALID_ALARM;
    cv_message_status_ts *mstat_ps  = NULL;
    CV_MODULE            *module_ps = NULL;
    CV_REQUEST           *dpvt_ps = (CV_REQUEST *)(rec_ps->dpvt);
    char                  errmsg_c[40];

    /* If device support doesn't exsist, then exit */
    if (!dpvt_ps) return(status);

    /* 
     *Otherwise, get the private device information
     * for this module and send a message to the queue
     * to request a Camac crate dataway test.
     */
    module_ps = dpvt_ps->module_ps;
    mstat_ps  = dpvt_ps->mstat_ps;
    if (!rec_ps->pact)
    {
        /* pre-processs. Clean up the request */
        CV_ClrMsgStatus( mstat_ps );
        if (epicsMessageQueueTrySend(module_ps->msgQId_ps,dpvt_ps,sizeof(CV_REQUEST)) == ERROR )
        {
            recGblSetSevr(rec_ps, nsta, nsev );
            errlogPrintf("Send Message to CV Operation Thread Error [%s]\n", rec_ps->name);
        }
        else
        { 
            rec_ps->pact = TRUE;
            status = NO_CONVERT;
        }
    }
    else
    {    /* post-process */
        if (CV_DEV_DEBUG) printf("devCV(write_bo): Post-processing of VERIFY Request\n"); 
        if( (!mstat_ps->opDone) || !SUCCESS(mstat_ps->errCode) )
        {
	    if (module_ps->crate_s.stat_u._i & CRATE_STATUS_CTO_ERR) 
               nsta = TIMEOUT_ALARM;
            if ( recGblSetSevr(rec_ps,nsta,nsev) && 
                 errVerbose                      && 
                 (rec_ps->stat!=nsta ||rec_ps->sevr!=nsev) ) 
	    {
	        sprintf(errmsg_c,"CV: error [0x%08lx] opDone=%s",
                        mstat_ps->errCode,
                        opDoneStatus_ac[mstat_ps->opDone]);
                recGblRecordError(ERROR,(void *)rec_ps,errmsg_c);
	    }
        }
        if (mstat_ps->opDone)
        {
            if(CV_DEV_DEBUG)  printf("Record [%s] succeed!\n",rec_ps->name);
            if (rec_ps->tse == epicsTimeEventDeviceTime) 
               rec_ps->time = mstat_ps->reqTime;
            status = NO_CONVERT;
        }

        /* Reset binary output state to idle, indicating request is complete */
        rec_ps->val = 0;
    }
    return (status);

}

/*=============================================================

  Abs:  Long input device support initialization

  Name: init_longin

  Args: rec_ps                      Record information
          Use:  struct
          Type: longinRecord *
          Acc:  read-write access
          Mech: By reference

  Rem: This routine performs the long input record initialization.

       The Camac information is extracted from the INP field,
       which includes: branch, crate and slot location of the
       crate verifier module. The module linked list is searched
       to see if the verifier module exists. If it does, then the
       PARM portion if the field (ie. after the @) is searched for 
       a valid verifier command for this record type.

       The long input record example of for the INP field 
       is shown below.

            field(INP, "CAMAC_IO - #B0 C1 N1 A3 F4 @ID")
            field(INP, "CAMAC_IO - #B0 C1 N1 A0 F4 @DATA")

       Please see the structure CV_CAMAC_FUNC in devCV.h for a complete
       list of Camac functions for which device support has been provided. 

  Side: None

  Ret: long
         OK   - Always
       
=============================================================*/
static long init_longin(struct longinRecord * rec_ps)
{
    long          status  = OK;
    CV_REQUEST   *dpvt_ps = NULL;

    switch( rec_ps->inp.type )
    {
        case CAMAC_IO:
          status = CV_RequestInit((dbCommon *)rec_ps, &rec_ps->inp.value.camacio, EPICS_RECTYPE_LI);
          if (!status)
	  {
            /* 
	     * If this is the id, set the alarm and warning limits.
	     * Note: the id MUST be set to the crate number
	     */
            dpvt_ps = (CV_REQUEST *)rec_ps->dpvt;
            if ( dpvt_ps->func_e == CAMAC_RD_ID )
	    {
              rec_ps->hihi = rec_ps->inp.value.camacio.c + 1;
              rec_ps->high = rec_ps->inp.value.camacio.c + 1;
	      rec_ps->low  = rec_ps->inp.value.camacio.c - 1;
	      rec_ps->lolo = rec_ps->inp.value.camacio.c - 1;
              rec_ps->hhsv = MAJOR_ALARM;
              rec_ps->hsv  = MAJOR_ALARM; 
              rec_ps->llsv = MAJOR_ALARM;
              rec_ps->lsv  = MAJOR_ALARM; 
	    }
	  }
          break;

        default:
          status = S_dev_badBus;
          break;
    }

    if(status)
    {
       recGblRecordError(status,(void *)rec_ps, "devlonginCV Init_record failed");
       rec_ps->pact=TRUE;
    }

    return(OK);
}


/*=============================================================

  Abs:  Long input device support

  Name: read_longin

  Args: rec_ps                      Record information
          Use:  struct
          Type: longinRecord *
          Acc:  read-write access
          Mech: By reference

  Rem: This routine processes a long input record for
       the crate verifier module, which supports the
       the following Camac request:

         ID     - Get the crate verifier id from the last read.
         DATA   - Get the data register from the crate online check

       If an error occurs the STAT and SEVR fiels of the record
       are set accordingly.

  Side: None

  Ret: long
         NO_CONVERT  - Operation Successful (pre-processing/post-processing) 
         ERROR       - Operation failed
       
=============================================================*/
static long read_longin(struct longinRecord * rec_ps)
{
    long                  status    = ERROR;
    unsigned short        nsta      = READ_ALARM;
    unsigned short        nsev      = INVALID_ALARM;
    cv_message_status_ts *mstat_ps  = NULL;
    CV_MODULE            *module_ps = NULL;
    CV_REQUEST           *dpvt_ps   = (CV_REQUEST *)(rec_ps->dpvt);
    char                  errmsg_c[40];


    if (!dpvt_ps || !dpvt_ps->module_ps) return(status);

    module_ps = dpvt_ps->module_ps;
    mstat_ps  = dpvt_ps->mstat_ps;

    if ( SUCCESS(mstat_ps->errCode) ) status = OK;
    if( (!mstat_ps->opDone) || !SUCCESS(mstat_ps->errCode) )
    {
	/* 
	 * If the Camac crate is offline then we have a
	 * timeout alarm status. Otherwise, we have a major alarm.
	 */
         if (module_ps->crate_s.stat_u._i & CRATE_STATUS_CTO_ERR)
            nsta = TIMEOUT_ALARM;
         if ( recGblSetSevr(rec_ps,nsta,nsev) && 
              errVerbose                      && 
              (rec_ps->stat!=nsta ||rec_ps->sevr!=nsev) ) 
	 {
	    /* 
            sprintf(errmsg_c,"CV: error [0x%08lx] opDone=%s",
                  mstat_ps->errCode,opDoneStatus_ac[mstat_ps->opDone]);
            */
            recGblRecordError(ERROR,(void *)rec_ps,errmsg_c);               
        }
    }
    if ( mstat_ps->opDone && (rec_ps->tse==epicsTimeEventDeviceTime))
        rec_ps->time = mstat_ps->reqTime;


    switch(dpvt_ps->func_e)  
    {
        case CAMAC_RD_ID:
	  status = OK;
	  rec_ps->val = module_ps->id;
	  break;

        case CAMAC_RD_DATA:
	  status = OK;
          rec_ps->val = module_ps->data;
          break;

        default:
          status = ERROR;
	  break;
    }/* End of switch statement */

    return(status);
}


/*=============================================================

  Abs:  Multi-bit Binary Direct device support initialization

  Name: init_mbbiDirect

  Args: rec_ps                      Record information
          Use:  struct
          Type: mbbiDirectRecord *
          Acc:  read-write access
          Mech: By reference

  Rem: This routine performs the multi-bit binary direct record 
       initialization.

       The Camac information is extracted from the INP field,
       which includes: branch, crate and slot location of the
       crate verifier module. The module linked list is searched
       to see if the verifier module exists. If it does, then the
       PARM portion if the field (ie. after the @) is searched for 
       a valid verifier command for this record type.

       The mbbiDirect input record example for the INP field 
       is shown below.

            field(INP, "CAMAC_IO - #B0 C1 N1 A0 F0 @CRATE")
            field(INP, "CAMAC_IO - #B0 C1 N1 A0 F0 @BUS")

       Please see the structure CV_CAMAC_FUNC in devCV.h for a complete
       list of Camac functions for which device support has been provided. 

  Side: None

  Ret: long
         OK   - Always
       
=============================================================*/
static long init_mbbiDirect(struct mbbiDirectRecord * rec_ps)
{
    long status = OK;

    switch( rec_ps->inp.type )
    {
        case CAMAC_IO:
          status = CV_RequestInit((dbCommon *)rec_ps, &rec_ps->inp.value.camacio, EPICS_RECTYPE_MBBI);
          break;

        default:
          status = S_dev_badBus;
          break;
    }

    if(status)
    {
       recGblRecordError(status,(void *)rec_ps, "devMbbiDirectCV Init_record failed");
       rec_ps->pact=TRUE;
    }
    return(OK);
}


/*=============================================================

  Abs:  Multi-bit Binary device support

  Name: read_mbbiDirect

  Args: rec_ps                      Record information
          Use:  struct
          Type: mbbiDirectRecord *
          Acc:  read-write access
          Mech: By reference

  Rem: This routine processes a multi-bit binary input record.
       This record reads the current crate status and  bus
       verification test status, which is kept in locally in 
       the driver support module information. 

        CRATE STAT - Camac crate online status
        BUS STAT   - Camac crate dataway verifier test status
  
      The VAL field is updated with this information for the 
      specified module. The RVAL field is not updated.

       If an error occurs the STAT and SEVR fiels of the record
       are set accordingly.

  Side: Conversion from a raw value to engineering units
        will not be performed if the field "LINR" is zero.

  Ret: long
         NO_CONVERT - Successful operation ( no conversion)
         ERROR      - Failure occured during writ
         Otherwise, see return from

=============================================================*/
static long read_mbbiDirect(struct mbbiDirectRecord * rec_ps)
{
    long                  status = ERROR;          /* status return    */
    unsigned short        nsta   = READ_ALARM;
    unsigned short        nsev   = INVALID_ALARM;
    cv_message_status_ts *mstat_ps   = NULL;
    CV_MODULE            *module_ps  = NULL;
    CV_REQUEST           *dpvt_ps    = (CV_REQUEST *)(rec_ps->dpvt);
    char                  errmsg_c[40];


    if(!dpvt_ps || !dpvt_ps->module_ps)  return(status);
 
    /* 
     * If the operation didn't complete or it completed
     * unsuccessfully, then set an error status and severity
     * for this pv.
     */
    module_ps = dpvt_ps->module_ps;
    mstat_ps  = dpvt_ps->mstat_ps;

    if ( (!mstat_ps->opDone) || !SUCCESS(mstat_ps->errCode) )
    { 
       nsev = MAJOR_ALARM;
       if ((dpvt_ps->func_e==CAMAC_RD_CRATE_STATUS) && (module_ps->crate_s.stat_u._i & CRATE_STATUS_CTO_ERR))
            nsta = TIMEOUT_ALARM;
       else if ((dpvt_ps->func_e==CAMAC_RD_BUS_STATUS) && (module_ps->crate_s.bus_stat_u._i & BUS_STATUS_CTO_ERR))
            nsta = TIMEOUT_ALARM;
       if ( recGblSetSevr(rec_ps,nsta,nsev) && 
            errVerbose                      && 
           (rec_ps->stat!=nsta ||rec_ps->sevr!=nsev) ) 
       {
	  /*	   
          sprintf(errmsg_c,"CV: error [0x%08lx] opDone=%s",
                  mstat_ps->errCode,
                  opDoneStatus_ac[mstat_ps->opDone]); 
          */
          recGblRecordError(ERROR,(void *)rec_ps,errmsg_c); 
       }
    }

    if (mstat_ps->opDone && (rec_ps->tse==epicsTimeEventDeviceTime))
       rec_ps->time = mstat_ps->reqTime;
    else
       rec_ps->udf = FALSE;
  

    if (dpvt_ps->func_e==CAMAC_RD_CRATE_STATUS)
      rec_ps->val = module_ps->crate_s.stat_u._i;
    else if (dpvt_ps->func_e==CAMAC_RD_BUS_STATUS)
      rec_ps->val = module_ps->crate_s.bus_stat_u._i;
    else
      rec_ps->val =0;

    if (CV_DEV_DEBUG)  
          printf("Record [%s] receives val [0x%04X]! iss=0x%8.8lX\n", rec_ps->name, rec_ps->val,mstat_ps->errCode);

    status = NO_CONVERT;
    return(status);
}


/*=============================================================

  Abs:  Waveform device support initialization

  Name: init_wf

  Args: rec_ps                      Record information
          Use:  struct
          Type: waveformRecord *
          Acc:  read-write access
          Mech: By reference

  Rem: This routine performs the waveform record initialization.

       The Camac information is extracted from the INP field,
       which includes: branch, crate and slot location of the
       crate verifier module. The module linked list is searched
       to see if the verifier module exists. If it does, then the
       PARM portion if the field (ie. after the @) is searched for 
       a valid verifier command for this record type.

       The waveform input record example of for the INP field 
       is shown below.

           field(INP, "CAMAC_IO - #B0 C1 N1 A0 F3 @CMD")
           field(INP, "CAMAC_IO - #B0 C1 N1 A0 F3 @RW")
           field(INP, "CAMAC_IO - #B0 C1 N1 A0 F3 @RW_PATTERN")

       Please see the structure CV_CAMAC_FUNC in devCV.h for a complete
       list of Camac functions for which device support has been provided. 

  Side: None

  Ret: long
         OK   - Always
       
=============================================================*/
static long init_wf(struct waveformRecord * rec_ps)
{
    long    status = OK;

    switch( rec_ps->inp.type )
    {
        case CAMAC_IO:
          status = CV_RequestInit((dbCommon *)rec_ps, &rec_ps->inp.value.camacio, EPICS_RECTYPE_WF); 
          break;

        default:
          status = S_dev_badBus;
          break;
    }

    if(status)
    {
       recGblRecordError(status,(void *)rec_ps, "devWfCV Init_record failed");
       rec_ps->pact=TRUE;
    }

    return(status);
}


/*=============================================================

  Abs:  Waveform device support

  Name: read_wf

  Args: rec_ps                      Record information
          Use:  struct
          Type: waveformRecord *
          Acc:  read-write access
          Mech: By reference

  Rem: This routine processes a waveform input record,
       which uploads the following Camac crate dataway
       verification test data  into a waveform record.
       This data is then later exracted, via sub-arrays
       and stuffed into individual pvs for display.
       Only the unsigned longword data type is supported

        CMD - Camac crate command line test data
        RW  - Camac crate read write line test data
  
      If an error occurs the STAT and SEVR fiels of the record
      are set accordingly.

  Side: Conversion from a raw value to engineering units
        will not be performed if the field "LINR" is zero.

  Ret: long
         NO_CONVERT - Successful operation ( no conversion)
         ERROR      - Failure occured during writ
         Otherwise, see return from

=============================================================*/
static long read_wf(struct waveformRecord *rec_ps)
{
    long                  status    = ERROR;          /* status return    */
    unsigned short        i         = 0;              /* index counter    */
    unsigned short        nsta      = READ_ALARM;     /* alarm status     */
    unsigned short        nsev      = INVALID_ALARM;  /* alarm severity   */
    unsigned long        *data_a    = NULL;           /* rw data          */
    unsigned long        *val_a     = NULL;           /* 32-bit data      */
    cv_message_status_ts *mstat_ps  = NULL;           /* message status   */
    campkg_dataway_ts    *cam_ps    = NULL;           /* camac info       */
    CV_MODULE            *module_ps = NULL;
    CV_REQUEST           *dpvt_ps   = (CV_REQUEST *)(rec_ps->dpvt);
    char                  errmsg_c[40];


    if(!dpvt_ps || !dpvt_ps->module_ps)
    {
       printf("%s not initalized\n",rec_ps->name);
       rec_ps->nord = 0;
       return(OK);
    }

    module_ps = dpvt_ps->module_ps;
    mstat_ps  = dpvt_ps->mstat_ps;
    if( (!mstat_ps->opDone) || !SUCCESS(mstat_ps->errCode) )
    {
      if ( module_ps->crate_s.stat_u._i & CRATE_STATUS_CTO_ERR )
           nsta = TIMEOUT_ALARM;
       if ( recGblSetSevr(rec_ps,nsta,nsev) && 
            errVerbose                      && 
           (rec_ps->stat!=nsta ||rec_ps->sevr!=nsev) ) 
       {
	 /* sprintf(errmsg_c,"CV: error [0x%08lx] opDone=%s",
                   mstat_ps->errCode,
                   opDoneStatus_ac[mstat_ps->opDone]); */
           recGblRecordError(ERROR,(void *)rec_ps,errmsg_c); 
       }
    }
    if (rec_ps->tse == epicsTimeEventDeviceTime)     /* do timestamp by device support */
      rec_ps->time = mstat_ps->reqTime;
       
    /* Copy data to the waveform record buffer */
    status = OK;
    cam_ps = &module_ps->cam_s.dataway_s;
    switch( dpvt_ps->func_e )
    {
          case CAMAC_TST_CMD:       /* command line data         */
            rec_ps->nord = MIN(CMD_LINE_NUM,rec_ps->nelm);
	    val_a = (unsigned long *)rec_ps->bptr;
            epicsMutexMustLock( module_ps->cmdLine_s.mlock );
            for (i=0; i<rec_ps->nord; i++)
	      val_a[i] = module_ps->cmdLine_s.data_a[i];
            epicsMutexUnlock( module_ps->cmdLine_s.mlock );
	    break;

          case CAMAC_TST_RW:            /* read-write lines W1-24 */ 
          case CAMAC_TST_RW_PATTERN:    /* read-write lines W1-24 pattern */ 
            rec_ps->nord = MIN(RW_LINE_NUM,rec_ps->nelm);
	    val_a = (unsigned long *)rec_ps->bptr;
            if (dpvt_ps->func_e==CAMAC_TST_RW_PATTERN) 
	      data_a = module_ps->rwLine_s.expected_data_a;
            else
              data_a = module_ps->rwLine_s.data_a;
            epicsMutexMustLock( module_ps->rwLine_s.mlock );
            for (i=0; i<rec_ps->nord; i++)
	      val_a[i] = data_a[i];
            epicsMutexUnlock( module_ps->rwLine_s.mlock );
	    break;

          default:
            status  = ERROR;
    }/* End of switch statement */

    return(status);
}


/*====================================================
 
  Abs:  Initialize the private devicec info for this record
 
  Name: CV_RequestInit
 
  Args: rec_ps                        Record information
          Type: struct         
          Use:  dbCommon * const
          Acc:  read-write
          Mech: By reference

        inout_ps                     Camac information from db
          Type: struc            
          Use:  struct camio const const *
          Acc:  read-only
          Mech: By reference

        rtyp_e                       EPICS record type
          Type: enum             
          Use:  cv_epics_rtype_te
          Acc:  read-only
          Mech: By value


  Rem:  The purpose of this function is to parse the INP/OUPT
        field of the specified record and initialize the
        private device information.

  Side: None
  
  Ret:  long 
            OK               - Operation succssful
            S_dev_badOutTyp  - Operation failed, due to invalid OUT field
            S_dev_badInpType - Operation failed, due to invalid INP field
            S_dev_badCard    - Operation failed, due to card not found in linked list 
            S_db_badField    - Operation failed, due to bad data type (wf only)
            ERROR            - Operation failed, record type not supported
            
=======================================================*/
static long CV_RequestInit(dbCommon             * const rec_ps, 
                           struct camacio const * const inout_ps, 
                           cv_epics_rtyp_te             rtyp_e )
{
    long               status    = ERROR; 
    cv_camac_func_te   func_e    = CAMAC_INVALID_OP;
    unsigned long int  num_a[2] = {CMD_LINE_NUM,RW_LINE_NUM};
    CV_MODULE         *module_ps = NULL;
    CV_REQUEST        *dpvt_ps   = NULL;
    waveformRecord    *wf_ps     = NULL;     


    /* parameter check */
    if ((rtyp_e<=EPICS_RECTYPE_NONE) || (rtyp_e>EPICS_RECTYPE_WF)) return(status);

    /* Is this a valid function for this record type? If not, return after issuing an error message */
    func_e = CV_FindFuncIndex( inout_ps->parm, rtyp_e );
    if (!func_e)
    {
       errlogPrintf("Record %s param %s is illegal!\n", rec_ps->name, inout_ps->parm);
       if (rtyp_e==EPICS_RECTYPE_BO)
	 status = S_dev_badOutType;
       else
         status = S_dev_badInpType; 
    }
    else
    { 
      switch(func_e)
      {
          case CAMAC_TST_CMD:
          case CAMAC_TST_RW:
          case CAMAC_TST_RW_PATTERN:
	    wf_ps = (waveformRecord *)rec_ps;
            if (wf_ps->ftvl!=DBF_ULONG)
	    {
              errlogPrintf("Record %s.FTVL is invalid, ULONG required\n",rec_ps->name);
              status = S_db_badField;
              break;
	    }
            else if ((func_e==CAMAC_TST_CMD) && (wf_ps->nelm<num_a[CMDLINE]))
	       errlogPrintf("Warning!! %s has %ld wf elements, expected %ld\n",
			    rec_ps->name,wf_ps->nelm,num_a[CMDLINE]);
	    else if (((func_e==CAMAC_TST_RW) || (func_e==CAMAC_TST_RW_PATTERN)) && (wf_ps->nelm<num_a[RWLINE]))
	       errlogPrintf("Warning!! %s has %ld wf elements, expected %ld\n",
			    rec_ps->name,wf_ps->nelm,num_a[RWLINE]);

         default:
            /* Is this module in the list? If not, then add to the list. */
            module_ps = CV_FindModuleByBCN(inout_ps->b, inout_ps->c, inout_ps->n);
            if (!module_ps)
            {
	       status  = S_dev_badCard;         /*Illegal or nonexistant module*/
               errlogPrintf("Module CV[b=%hd,c=%hd,n=%hd] not registerd by PSCD before iocInit\n", 
                            inout_ps->b,inout_ps->c,inout_ps->n );
            }
            else
            {
               dpvt_ps = (CV_REQUEST *)callocMustSucceed(1, sizeof(CV_REQUEST), "calloc CV_REQUEST");
               status  = CV_DeviceInit( func_e, "DSUP", rec_ps,module_ps,dpvt_ps );
               rec_ps->dpvt = dpvt_ps;
               dpvt_ps->a   = inout_ps->a;
               dpvt_ps->f   = inout_ps->f;
            }
            break;
      }/* End of switch statement */
    }
    return(status);
}


/*====================================================
 
  Abs:  Find requested command in the cv function table
 
  Name: CV_FindFuncIndex
 
  Args: req_c                        Requested string
          Type: ascii-string         Note:
          Use:  char const * const
          Acc:  read-only
          Mech: By reference

        rtyp_e                       EPICS record type
          Type: enum             
          Use:  cv_epics_rtype_te
          Acc:  read-only
          Mech: By value


  Rem:  The purpose of this function is to search the
        function table for a match of the requested function
        string, and then return the associated camac function
        used to submite a messsage request to the queue.

  Side: None
  
  Ret:  short 
              0 - Failed to find a match
              Otherwise, match found, function code returned.
                  
=======================================================*/
static short CV_FindFuncIndex( char const * const req_c, cv_epics_rtyp_te rtyp_e )
{
   CV_CAMAC_FUNC;
   short       i_func  = 0;
   short       found   = 0;
   short       nfunc   = sizeof(cv_camac_func_as)/sizeof(cv_camac_func_ts);


   for( i_func=0; (i_func<nfunc) && !found; i_func++ )
   {
       if( 0 == strcmp(cv_camac_func_as[i_func].func_c,req_c) )
       {  
          /* Found function, now check that this record type is expected. */
          if (rtyp_e == EPICS_RECTYPE_NONE || cv_camac_func_as[i_func].rtyp_e==rtyp_e) 
            found = cv_camac_func_as[i_func].func_e;
       }
   } /* End of FOR loop; */

   if (CV_DEV_DEBUG && !found )  
       printf("...%s (%hd) Not found\n",req_c,rtyp_e ); 
   return( found );
}
