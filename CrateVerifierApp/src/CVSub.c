/*============================================================
 
  File:  CVSub.c

  Abs:   This file contains all subroutine support for the
         CAMAC Crate Verifier module support.

 
         Functions:            Description
         ------------          ----------------------------
         CV_Init               General Initialization
         CV_Bus_Data_Init      Bus line data initialization 
         CV_RWline_Init        Read-Write line data initialization 
         CV_Stat               Crate Status Summary (worst case)
         CV_Limits             Alarm and warning limits for crate id
         CV_Bus_Data           Camac Bus Line data
         CV_RWline             Read/Write Line data (extracted from waveform)


         The genSubRecord places the return status into the VAL field

  Side:  Place all functions called by PVs into the file CV.dbd

  Proto: CVSub_proto.h.

  Auth:  23-May-2010, K. Luchini         (LUCHINI):
  Rev:   dd-mmm-yyyy, Reviewer's Name    (USERNAME)
--------------------------------------------------------------
  Mod:
         dd-mmm-yyyy, First Last Name    (USERNAME)
           comment

==============================================================*/

/* Header files */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>

#include "epicsVersion.h"
#if EPICS_VERSION>=3 && EPICS_REVISION>=14
#include "epicsExport.h"
#include "waveformRecord.h"        /* for struct waveform         */
#include "genSubRecord.h"          /* for struct genSubRecord     */
#include "longSubRecord.h"         /* for struct longSubRecord    */
#include "devCV.h" 
#include "registryFunction.h"    
#else
#error "We need EPICS 3.14 or above to support OSI calls!"
#endif

/* Maximum number of outputs in a genSubRecord */
#define MAX_ARGS 21 

/* Static prototypes */
static long CV_Init(dbCommon *sub_ps);
static long CV_Bus_Data_Init(genSubRecord *sub_ps);
static long CV_RWline_Init(longSubRecord *sub_ps);
static long CV_Stat(longSubRecord *sub_ps);
static long CV_Limits(genSubRecord *sub_ps);
static long CV_Bus_Data(genSubRecord *sub_ps);
static long CV_RWline(longSubRecord *sub_ps);

/* Global variables */ 
int CV_SUB_DEBUG=0;


/*=============================================================================

  Name: CV_Init

  Abs: This in a generic initialization function assuming that does nothing.

  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        dbCommon *          sub_ps      read/write  pointer to record

  Rem: This is a general purpose initialization required since all subroutine records
       require a non-NULL init routine even if no initialization is required.
       Note that most subRecords in this file use this routine as an init
       function.  If init logic is needed for a specific subroutine, create a
       new routine.

 
  Side: Do not modify this function may be used for muiltiple PV

  Ret:  long 
          OK - Successful (always)

==============================================================================*/
static long CV_Init(dbCommon *sub_ps)
{
  long status=OK;
  return(status);
}


/*=============================================================================

  Name: CV_Bus_Data_Init

  Abs: This in a generic initialization function for a waveform copy function

  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        genSubRecord *     sub_ps       read/write  pointer to record

        <device>:CMDLINES
        <device>:RWLINES

        Input: from CV_DatawayTest()
        --------
        INPA = waveform to camac bus data
        INPB = number of elements in waveform
        INPC = type of bus data  (0=cmdline, 1=rwline)
        INPD = index into waveform
        INPE = number of items to extract from waveform

        Output:
        --------
        DPVT = Pointer to a private device information

  Rem: This is a general purpose initialization required since all subroutine records
       require a non-NULL init routine even if no initialization is required.
       Note that most genSubRecords in this file use this routine as an init
       function.  If init logic is needed for a specific subroutine, create a
       new routine.

 
  Side: Do not modify this function may be used for muiltiple PV

  Ret:  long 
          OK - Successful (always)


==============================================================================*/
static long CV_Bus_Data_Init(genSubRecord *sub_ps)
{
  long            status  = ERROR;                                  /* return status       */
  unsigned long   type    = *(unsigned long *)sub_ps->c;            /* type of bus data    */
  cv_busdata_ts  *dpvt_ps = (cv_busdata_ts *)sub_ps->dpvt;          /* private device info */
  

  /* Check that this pv has not already been initialized */
  if (sub_ps->dpvt) return(status);
  
  /* Is the crate verifier set out-of-service in the database? */
  if ( sub_ps->fta!=DBF_ULONG )
      recGblRecordError(S_dev_badInpType,(void *)sub_ps,"CV_Data_Init: bad INP field");
  else if ((type<CMDLINE) || (type>RWLINE))
      recGblRecordError(S_dev_badInpType,(void *)sub_ps,"CV_Bus_Data_Init:  Invalid type of bus data");
  else
  {
      /* Create pointer for private device information to be used when record is processed */
      dpvt_ps = (cv_busdata_ts *)calloc(sizeof(cv_busdata_ts),1);
      if (!dpvt_ps) 
         recGblRecordError(S_rec_outMem,(void *)sub_ps,"CV_Data_Init: failed");
      else
      {
         status = OK;
         dpvt_ps->data_a =  (unsigned long *)sub_ps->a;                    /* ptr to local wf    */
         dpvt_ps->type   = *(unsigned long *)sub_ps->c;                    /* type of bus data   */
         dpvt_ps->nelem  = min(*(unsigned long *)sub_ps->b,sub_ps->noa);   /* # of data elements */
         if (type==CMDLINE)
	   dpvt_ps->nelem = min(dpvt_ps->nelem,CMD_LINE_NUM);
         else
 	   dpvt_ps->nelem = min(dpvt_ps->nelem,RW_LINE_NUM);
         dpvt_ps->mlock  = epicsMutexMustCreate();                         /* mutex to be used when copying waveform */
         sub_ps->dpvt    = dpvt_ps;
      }
  }
  return(status);
}

/*=============================================================================

  Name: CV_RWline_Init

  Abs: This in a generic initialization function assuming that does nothing.

  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        longSubRecord *     sub_ps      read/write  pointer to record

  Rem: This is a general purpose initialization required since all subroutine records
       require a non-NULL init routine even if no initialization is required.
       Note that most subRecords in this file use this routine as an init
       function.  If init logic is needed for a specific subroutine, create a
       new routine.

 
  Side: Do not modify this function may be used for muiltiple PV

  Ret:  long 
          OK - Successful (always)

==============================================================================*/
static long CV_RWline_Init(longSubRecord *sub_ps)
{
  long               status  = ERROR;
  cv_rwline_info_ts *dpvt_ps = NULL;
  DBADDR            *addr_ps = NULL;
  DBADDR            *eaddr_ps = NULL;


  addr_ps    = dbGetPdbAddrFromLink(&sub_ps->inpa);
  eaddr_ps   = dbGetPdbAddrFromLink(&sub_ps->inpb);
  if (!addr_ps || !eaddr_ps) return(ERROR);
  if ((addr_ps->no_elements<RW_LINE_NUM) || (eaddr_ps->no_elements<RW_LINE_NUM)) 
    return(status);
  
  /* Create pointer for private device information to be used when record is processed */
  dpvt_ps = (cv_rwline_info_ts *)calloc(sizeof(cv_rwline_info_ts),1);
  if (!dpvt_ps) 
      recGblRecordError(S_rec_outMem,(void *)sub_ps,"CV_RWline_Init: failed");
  else
  {
     dpvt_ps->mlock = epicsMutexMustCreate();
     dpvt_ps->data_a  = (unsigned long *)addr_ps->pfield;
     dpvt_ps->edata_a = (unsigned long *)eaddr_ps->pfield;
     sub_ps->dpvt = dpvt_ps;
     status= OK;
  }
  return(status);
}


/*=============================================================================

  Name: CV_Stat

  Abs: This function determines the summary message for the crate online
       status based on the input arguments.

       An bitmask will be generated based on the input information.
       This bitmask will be used by  <device>:STATMSG (mbbo) record.
       to provide a status summary (worst case) string.

        CAMC:<area>:<crate>:STAT_SUB

  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        longSubRecord *     sub_ps      read/write pointer to subroutine record
         
        Input:
        --------
        INPA = crate online status             CAMC:<area>:<crate>:STAT       
        INPB = crate bus line test status      CAMC:<area>:<crate>:BUSSTAT   
        INPC = crate voltage error             CAMC:<area>:<crate>:VOLTERR.SEVR      
        INPD = crate temperature severity      CAMC:<area>:<crate>:TEMP.SEVR           
        INPE = id register severity            MODU:<area>:<crate><slot>:ID.SEVR    
        INPF = data read register severity     MODU:<area>:<crate><slot>:DATA.SEVR  

        Output:
        ---------
        VAL = status summary value (worst case) 
 
        such that,
        VAL    Description 
        ----   ------------------
         0     Crate Power Off
         1     Crate Power On but not initalized
         2     Crate Online (power on and initlized)
         3     Voltage Error
         4     Temperature Error
         5     Bad Id
         6     Invalid data register pattern
         7     No module
         8     Voltage Warning
         9     Temperatore Warning
         10    Bus Error
         11    Cmd-Line Error
         12    RW-Line Error
         13    X-Response Warning
         14    Q-Response Warning
         15    Verify init failed

         VAL = crate summary message bitmask (worst case) used by <device>:STATMSG

  Rem: 
 
  Side: None

  Ret:  long 
          OK  - Alwasys

==============================================================================*/ 
static long CV_Stat(longSubRecord *sub_ps)
{
  long               status  = OK;                   /* return status */
  unsigned long      stat    = CRATE_STATSUMY_VPWROFF;
  unsigned long      crateOn = 0;
  cv_bus_status_tu   bus_stat_u;
  cv_crate_status_tu crate_stat_u;
  
  crateOn = sub_ps->a & CRATE_STATUS_GOOD;
  if (crateOn)
  {
      crate_stat_u._i = sub_ps->a & CRATE_STATUS_MASK;
      bus_stat_u._i   = sub_ps->b & BUS_STATUS_MASK;

      /* Check for an id register error */
      if ( sub_ps->e ) 
	 stat = CRATE_STATSUMY_BADID;

      /* Data verification errors */
      if ( BUS_ERR(bus_stat_u._i) )
      {
       
         /* Dataway verification test errors */
         if ( bus_stat_u._i & BUS_STATUS_INIT_ERR )
  	    stat = CRATE_STATSUMY_INITERR;    
         else if ( bus_stat_u._i & BUS_STATUS_CMD_ERR )
            stat = CRATE_STATSUMY_CMDERR;
         else if ( RWLINE_ERR(bus_stat_u._i) )
            stat = CRATE_STATSUMY_RWERR;
         else if ( bus_stat_u._i & BUS_STATUS_BUS_ERR )
            stat = CRATE_STATSUMY_BUSERR;

         /* Dataway verification test warnings */
         else if ( (bus_stat_u._i & BUS_STATUS_X_ERR) || (bus_stat_u._i & BUS_STATUS_NOX_ERR) )
            stat = CRATE_STATSUMY_XWARN;
         else if ( (bus_stat_u._i & BUS_STATUS_Q_ERR) || (bus_stat_u._i & BUS_STATUS_NOQ_ERR) )
            stat = CRATE_STATSUMY_QWARN;
         else if ( bus_stat_u._i & BUS_STATUS_INIT_ERR )
            stat = CRATE_STATSUMY_INITERR;
      }
      /* Voltage and Temperature errors */
      else if ( sub_ps->c==MAJOR_ALARM )
	  stat = CRATE_STATSUMY_VOLTERR;
      else if ( sub_ps->d==MAJOR_ALARM )
	  stat = CRATE_STATSUMY_TEMPERR;

      /* Voltage and Temperature warnings */
      else if ( sub_ps->c==MINOR_ALARM )
	  stat = CRATE_STATSUMY_VOLTWARN;
      else if ( sub_ps->d==MINOR_ALARM )
	  stat = CRATE_STATSUMY_TEMPWARN;
      else if (crateOn==CRATE_STATUS_ONINIT)
          stat = CRATE_STATSUMY_ONINIT;
       else
	  stat = CRATE_STATSUMY_VPWRON;
  }
      
  sub_ps->val = stat & CRATE_STATSUMY_MASK;;

  return(status);
}


/*=============================================================================

  Name: CV_Limits

  Abs: This function generates the alarm limits for the crate id
       based on the crate number

  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        genSubRecord *      sub_ps      read/write pointer to subroutine record

         <device>:LIMITS

        Input:
        --------
        INPA = Crate number          

        Output:
        ---------
        OUTA = Id Uppler warning limit   MODU:<area>:<crate><slot>:ID.LOLO 
        OUTB = Id Lower warning limit    MODU:<area>:<crate><slot>:ID.HIHI
        OUTC = Id Uppler alarm limit     MODU:<area>:<crate><slot>:ID.LOW
        OUTD = Id Lower alarm limit      MODU:<area>:<crate><slot>:ID.HIGH

  Rem: 
 
  Side: None

  Ret:  long 
          OK  - Alwasys

==============================================================================*/ 
static long CV_Limits(genSubRecord *sub_ps)
{
  long  status = OK;          /* return status */
   
  /* Set id alarm limits */
  *(unsigned short *)sub_ps->vala = *(unsigned long *)sub_ps->a - 1;
  *(unsigned short *)sub_ps->valb = *(unsigned long *)sub_ps->a + 1;

  /* Set warning limits (same as alarm limits) */
  *(unsigned short *)sub_ps->valc = *(unsigned long *)sub_ps->a - 1;
  *(unsigned short *)sub_ps->vald = *(unsigned long *)sub_ps->a + 1;
  return(status);
}


/*=============================================================================

  Name: CV_Bus_Data

  Abs: This function distributes the command line data to individual PVs


  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        genSubRecord *      sub_ps      read/write pointer to subroutine record

        Input: from CV_DatawayTest()
        --------
        INPA = Command line data from the CAMAC bus          <device>:CMDLINES       
        INPB = Number of waveform elements                   <device>:CMDLINES.NELM  
        INPC = type of bus data (0=cmdline, 1=rwline)
        INPD = index into waveform (starting from 0)
        INPE = number of items to extract from waveform
        DPVT = Pointer to a private device information (see: CV_Bus_Data_Init)

        Output 
        Command Line:
        ---------
        OUTA = <device>:N              N-line (slot) data
        OUTB = <device>:F1             Function line
        OUTC = <device>:F2             
        OUTD = <device>:F4            
        OUTE = <device>:F8           
        OUTF = <device>:F16           
        OUTG = <device>:A1             Subaddress line
        OUTH = <device>:A2
        OUTI = <device>:A4
        OUTJ = <device>:A8


        Reead Write Line:
        ---------
        OUTA = <device>:R1       <device>:R17   
        OUTB = <device>:R2       <device>:R18
        OUTC = <device>:R3       <device>:R19
        OUTD = <device>:R4       <device>:R20
        OUTE = <device>:R5       <device>:R21
        OUTF = <device>:R6       <device>:R22
        OUTG = <device>:R7       <device>:R23
        OUTH = <device>:R8       <device>:R24
        OUTI = <device>:R9       <device>:R25
        OUTJ = <device>:R10
        OUTK = <device>:R11           
        OUTL = <device>:R12          
        OUTM = <device>:R13  
        OUTN = <device>:R14
        OUTO = <device>:R15
        OUTP = <device>:R16

        where <device> = CAMC:<area>:<crate>
  Rem: 
 
  Side: None

  Ret:  long 
          OK  - Alwasys

==============================================================================*/ 
static long CV_Bus_Data(genSubRecord *sub_ps)
{
  long           status  = OK;                               /* return status              */
  unsigned long  i       = 0;                                /* index counter              */
  unsigned long  j       = *(unsigned long *)sub_ps->d;      /* index into data array      */
  unsigned long  k       = *(unsigned long *)sub_ps->e;      /* num of data items to get   */
  unsigned long **val_pp = (unsigned long **)&sub_ps->vala;  /* array of ptrs              */
  unsigned long *data_p  = NULL;                             /* output data ptr            */
  cv_busdata_ts *dpvt_ps = (cv_busdata_ts *)sub_ps->dpvt;    /* ptr to private device info */


  if (!sub_ps->dpvt) return(status);

  /* epicsMutexMustLock(dpvt_ps->mlock); */
  if (CV_SUB_DEBUG) printf("%s\n",sub_ps->name);
  for (i=0; (j<sub_ps->noa) && (i<MAX_ARGS) && (i<k); i++,j++,val_pp++)
  {
     data_p  = *val_pp;
     if (data_p)
       *data_p = dpvt_ps->data_a[j];
     if (CV_SUB_DEBUG)  printf("\t(%.2ld)0x%lx",j,*data_p);
  }
  if (CV_SUB_DEBUG) printf("\n");
 
  /* epicsMutexUnlock(dpvt_ps->mlock); */
  return(status);
}


/*=============================================================================

  Name: CV_RWline

  Abs: This function generates the alarm limits for the crate id
       based on the crate number

  Args: Type                Name        Access     Description
        ------------------- ----------- ---------- ----------------------------
        longSubRecord *      sub_ps      read/write pointer to subroutine record

         <device>:R<line>  where line is 1-24

        Input:
        --------
        INPA = Read-Write line test data              <device>:RWLINE
        INPB = Read-Write line test expected data     <device>:RWLINE_PATTERN
        INPC = Read-Write line number (0-24)
        INPD = Bus status bitmask       

        Output:
        ---------
        VAL - Read-Write line data

  Rem: 
 
  Side: None

  Ret:  long 
          OK  - Alwasys

==============================================================================*/ 
static long CV_RWline(longSubRecord *sub_ps)
{
  long               status   = ERROR;       /* return status        */
  unsigned long      line     = 0;           /* line number          */
  unsigned long      tst_data = 0;           /* expected data        */
  unsigned short     nsta     = READ_ALARM;  /* record status        */
  unsigned short     nsev     = MAJOR_ALARM; /* record severity      */
  cv_rwline_info_ts *dpvt_ps = NULL;         /* private device info  */

 
  if (!sub_ps->dpvt) return(status);

  status  = OK;
  dpvt_ps = (cv_rwline_info_ts *)sub_ps->dpvt; 
  line = sub_ps->c;
  if ( (line<0) || (line>RW_LINE_NUM) )
  {
     sub_ps->val = 0;
     if ( recGblSetSevr(sub_ps,nsta,nsev) && errVerbose && 
         (sub_ps->stat!=nsta ||sub_ps->sevr!=nsev) ) 
          recGblRecordError(ERROR,(void *)sub_ps,"Invalid RW Line number" );
  }
  else
  {
     /* Turn off alarms */
     sub_ps->hihi = sub_ps->hopr;
     sub_ps->high = sub_ps->hopr;
     sub_ps->lolo = sub_ps->lopr;
     sub_ps->low  = sub_ps->lopr;
     sub_ps->hhsv = NO_ALARM;
     sub_ps->llsv = NO_ALARM;
     sub_ps->hsv  = NO_ALARM;
     sub_ps->lsv  = NO_ALARM;
    
     /* Set the val field */
     epicsMutexMustLock(dpvt_ps->mlock);
     sub_ps->val = dpvt_ps->data_a[line];
     tst_data    = dpvt_ps->edata_a[line];
     epicsMutexUnlock(dpvt_ps->mlock);

     if ( (sub_ps->val != tst_data)  || RWLINE_BYPASSED(sub_ps->d) || CRATE_TIMEOUT(sub_ps->d) )
     {
       sub_ps->hihi = sub_ps->val - 1;
       sub_ps->high = sub_ps->val - 1;
       if ( RWLINE_BYPASSED(sub_ps->d) || CRATE_TIMEOUT(sub_ps->d) )
          nsev = INVALID_ALARM;
       sub_ps->hhsv = nsev;
       sub_ps->hsv  = nsev;
       sub_ps->llsv = nsev;
       sub_ps->lsv  = nsev;
     }
  }
  return(status);
}


epicsRegisterFunction(CV_Init);
epicsRegisterFunction(CV_Stat);
epicsRegisterFunction(CV_Limits);
epicsRegisterFunction(CV_Bus_Data_Init);
epicsRegisterFunction(CV_RWline_Init);
epicsRegisterFunction(CV_Bus_Data);
epicsRegisterFunction(CV_RWline);

/* Note - when adding a funciton, also add to CV.dbd */

