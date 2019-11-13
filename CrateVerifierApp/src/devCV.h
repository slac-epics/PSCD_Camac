/*
============================================================

  Abs:  CAMAC Crate Verifier device support header file

  Name: devCV.h

  Side:  None

  Auth: 19-Jul-2009, K. Luchini       (LUCHINI)
  Rev : dd-mmm-yyyy, Reviewer's Name  (USERNAME)

-------------------------------------------------------------
  Mod:
        24-Feb-2017, K. Luchini       (LUCHINI):
          add CAM_MBCD_NFG_MSG

=============================================================
*/
#ifndef _DEV_CV_H_
#define _DEV_CV_H_
#define CV_DRV_VER_STRING       "CAMAC CV Driver V1.0"

/*
** Include all of the standard C stuff needed by device & driver support
*/
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>

/*
** Get all of the EPICS includes we might need.
*/
#include "epicsVersion.h"
#if (EPICS_VERSION>=3 && EPICS_REVISION>=14) || EPICS_VERSION>3
#include "epicsExport.h"
#include "alarm.h"
#include "dbCommon.h"
#include "dbDefs.h"
#include "dbAccess.h"
#include "dbScan.h"
#include "recSup.h"
#include "recGbl.h"
#include "devSup.h"
#include "drvSup.h"
#include "link.h"
#include "ellLib.h"
#include "errlog.h"
#include "special.h"
#include "epicsTime.h"
#include "epicsMutex.h"
#include "epicsEvent.h"
#include "epicsInterrupt.h"
#include "epicsMessageQueue.h"
#include "epicsThread.h"
#include "cantProceed.h"
#include "devLib.h"
#include "special.h"
#include "cvtTable.h"
#include "menuConvert.h"
#include "registryFunction.h"
/*
** We'll preserve/use VMS status words to pass back so we need the SUCCESS macro
*/
#include "slc_macros.h"            /* for vmsstat_t               */
#include "cratdef.h"
#include "camdef.h"
#include "cctlwmasks.h"

#else
#error "We need EPICS 3.14 or above to support OSI calls!"
#endif

#include "genType.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* Minimum and maximum (removed from dbDef.h) */
#define MIN(x,y) ((x>y) ? y : x)
#define MAX(x,y) ((x>y) ? x : y)

/******************************************************************************************/
/*********************        VMS Status Message Codes          ***************************/
/*********************           see cratdef.hc                 ***************************/
/******************************************************************************************/

#define CV_MODU_MSG          "Crate Verifier modules (%d) have been registered with the PSCD\n"
#define CV_NOMODU_MSG        "No Crate Verifier Modules have been registerd with the PSCD\n"
#define CV_QCREATE_ERR_MSG   "Failed to create message Queue for CV Operation!\n"
#define CV_NOQ_MSG           "No messag Queue for CV Operation!\n"
#define CV_OPNOQ_MSG         "CV_OP thread exiting, no queue available!\n"
#define CV_QTMO_MSG          "%s thread error, message Queue timeout occurred - status %d. Suspending thread!\n"
#define CV_INVFUNC_MSG       "Invalid Camac function %d sent to CV Operations thread\n"
#define CV_ASYNSEND_MSG      "CV_ASYN sending periodic messages to queue!\n"
#define CV_THREADSTART_MSG   "%s thread starting, tid = %p\n"
#define CV_THREADFAIL_MSG    "%s thread failed to start!\n"
#define CV_THREADEXIT_MSG    "%s thread exiting thread!\n"
#define CV_WAITEVT_MSG       "%s thread waiting for event to occur\n"
#define CV_GOEVT_MSG         "%s thread processing after event signal received\n"


              /***************** From cratdef.h on SLC *************************/
/* Success Messages */
#define CRAT_INITSUCC_MSG       "Crate %.2d initialization done\n"
#define CRAT_OKOK_MSG           "CAMAC Successful\n"

/* Informational messages */
#define CRAT_CRATES_INIT_MSG    "Initalization of %d crates started at boot\n"
#define CRAT_ONLINE_MSG         "Crate %.2d is online\n"
#define CRAT_VERIFY_FAIL_MSG    "Crate %.2d failed verification\n"
#define CRAT_VERIFY_PASSES_MSG  "Crate %.2s passed verification, was 0x%8.8X\n"
#define CRAT_ZISSUED_MSG        "Camac Z (F26A8M28) issued to crate %.2d (power had been off)\n"
#define CRAT_CREXBGVF_MSG       "Start testing of crate %.2d verifier\n"
#define CRAT_CREXFNVF_MSG       "Stop testing of crate %.2d verifier\n"

/* Warning Messages */
#define CRAT_CHNGSTAT_MSG       "Crate %.2d status is now %s verifier status =0x%8.8X\n"
#define CRAT_OFFON_MSG          "Crate %.2d power has been OFF since last test, verifier data/status =0x%8.8lX/0x%8.8X\n"
#define CRAT_OFFLINE_MSG        "Crate %.2d is offline\n"

 /* Error Messages */
#define CRAT_CANTINIT_MSG       "Crate %.2d offline--unable to initialize\n"
#define CRAT_INITFAIL_MSG       "Crate %.2d initialization failed, verifier %s data/status =0x%8.8lX/0x%8.8X\n"
#define CRAT_OFFVER_MSG         "Crate %.2d verifier data(wt/rbk)=0x%8.8lX/0x%8.8lX\tstatus=0x%8.8X (Crate Offline or module is Bad)\n"
#define CRAT_VERDAT1_MSG        "Readout %.1d of two of verify crate %.2d  slot %.2d gave bad data/status 0x%8.8lX/0x%8.8X\n"
#define CRAT_VERDAT_MSG         "Both Readout to verify crate %.2d  slot %.2d gave bad data/status 0x%8.8lX/0x%8.8X  0x%8.8lX/0x%8.8X\n"
#define CRAT_ACBAD_MSG          "Crate watch found CAMAC problem in crate %.2d\n"
#define CRAT_CREXCTVF_MSG       "Crate %2.d verifier test: reps, errors: %.hd\n"
#define CRAT_CREXERV_MSG        "Verifier in crate %.2d gave %s; test expected %s\n"

/* Camac Error messages */
#define CAM_SOFT_TO_MSG         "CAMAC software timeout. No MBCD response.: Crate %.2d  N%.2d   A%ld  F%ld  status=0x%8.8X\n"
#define CAM_MBCD_NFG_MSG        "CAMAC MBCD Failure: Crate %.2d  N%.2d   A%ld  F%ld  status=0x%8.8X\n"
#define CAM_CRATE_TO_MSG        "CAMAC crate timeout: Crate %.2d  N%.2d  A%ld  F%ld  status=0x%8.8X\n"
#define CAM_NO_Q_MSG            "CAMAC no Q respns: Crate %.2d  N%.2d  A%ld  F%ld  status=0x%8.8X\n"
#define CAM_NO_X_MSG            "CAMAC no X respns: Crate %.2d  N%.2d  A%ld  F%ld  status=0x%8.8X\n"

/******************************************************************************************/
/*********************                   Status Codes           ***************************/
/******************************************************************************************/

/* Camac Crate Dataway Test Status Codes */
#define CV_STAT_FAIL            0
#define CV_STAT_OK              1
#define CV_STAT_BAD             100
#define CV_STAT_CRATE_OFF       100
#define CV_STAT_CAMSOFT_BAD     150
#define CV_STAT_MBCD_BAD        200
#define CV_STAT_NORESPONSE      300
#define CV_STAT_MICRO_OFF       400

#ifndef OK
#define OK                      (0)
#endif
#ifndef ERROR
#define ERROR                   (-1)
#endif

/* EPICS Device Support Status Codes */
#define CONVERT                 (0)   /* Analog device support, convert rval to val       */
#define NO_CONVERT              (2)   /* Analog device support, do not convert value      */

/******************************************************************************************/
/*********************       EPICS driver report levels         ***************************/
/******************************************************************************************/
/* 
 * The following definitions are the "level" that can be supplied to
 * the driver report function dbior("drvCV",<level>)
 */
#define REPORT_BASIC     0
#define REPORT_STANDARD  1
#define REPORT_DETAILED  3
#define REPORT_VOLTAGE   4
#define REPORT_EXPERT    5

/******************************************************************************************/
/******************************************************************************************/
/*********************                CAMAC                     ***************************/
/*********************            Functions Codes & Masks       ***************************/
/*********************               see cctlw.h                ***************************/
/******************************************************************************************/

#define CAMAC_CRATE_MASK            0xF              /* Camac crate number mask  */
#define CAMAC_SLOT_MASK             0x1F             /* Camac slot number mask   */
#define CAMAC_FUNC_MASK             0xF              /* Camac function code mask */
#define CAMAC_SUBADDR_MASK          0x1F             /* Camac subaddress mask    */

/* Error Masks */
#define CAMAC_EMASK_XQ              0xF300
#define CAMAC_EMASK_NBAR            0xE000
#define CAMAC_EMASK_NOX_NOQ_NOCTO   0xD000           /* NOX, NOQ and NO Crate-Time-Out */
#define CAMAC_EMASK_NOX_NOQ         0xF000           /* NOX and NOQ                    */
#define CAMAC_MBCD_CTO              0x0020

/* Control Functions */
#define F3A0           0x00030000
#define F4A0           0x00040000
#define F4A1           0x00040001    /* Read ROTATE register  and rotate patter left  */
#define F5A0           0x00050000
#define F20A0          0x00140000    /* Verifier load DATA register                   */
#define F20A3          0x00140003    /* Verifier set ROTATE register for walking zero */
#define F26A8          0x001A0008
#define F26A9          0x001A0009    /* Pulse the bus C-line    + M28 */
#define F24A9          0x00180009    /* turn off crate inhibit  + M30 */
#define A7             0x00000007
#define F5             0x00050000
#define M24            0x00000C00
#define M25            0x00000C80
#define M28            0x00000E00
#define M30            0x00000F00
#define P24            0x04000000

typedef struct camac_xq_status_s
{ 
  unsigned long was_one;
  unsigned long was_zero;
  bool          err;
}camac_xq_status_ts;

/******************************************************************************************/
/******************************************************************************************/
/*********************          Crate Verifier  Registers       ***************************/
/******************************************************************************************/

/*
 *             o  X = one for all valid commands.
 *             o  Q = one for all valid commands except F3 where Q = zero
 *
 *  Analog Voltage Register
 *
 *             o  The internal voltage referance must be set to 5.120 +/-  .005 volts.
 *             o  Read analog voltages n on R8-R1 with F5 An.
 *                   V(mv) = f(40*Vin - 5120)
 *             o  The absolute accuracy for voltage readings is +/-1%.
 *                n          source         f
 *                0           +24           6
 *                1           +12           3
 *                2            +6           1.5
 *                3          ground         1.0
 *                4            -6           1.5
 *                5           -12           3
 *                6           -24           6
 *                7         temperature     1
 *                8            P1           6
 *                9            P2           6
 *                10           P3           6
 *                11           P4           6
 *                12           P5           6
 *
 *             o  Temperature  =  40mv  per  degree  centigrade.   Zero  degree
 *                centigrade = zero volts.
 *             o  Absolute temperature accuracy is +/- 3 degrees.
 *
 */
#define CV_NUM_ANLG_CHANNELS	8           /* Number of data values     */
#define CV_MIN_ANLG_SUBADR      0           /* Minimum subaddress        */
#define CV_MAX_ANLG_SUBADR      7           /* Minimum subaddress        */
#define CV_ANLG_ZERO            5120        /* 8-bit analog data         */
#define CV_ANLG_SLOPE           40          /* convert from counts to mv */

typedef struct
{
  float   m1;
  short   m2;
  char   *label_c;
}cv_volt_mult_ts;

#define CV_TEMP_SUBADDR 7
#define CV_VOLT_MULT const cv_volt_mult_ts vmult_as[CV_NUM_ANLG_CHANNELS] = \
               {{0.0060,1,"+24V"}, {0.0030,1,"+12V"},{0.0015,1,"+6V"},\
                {0.0010,1,"VGND"}, {0.0015,1,"-6V"}, {0.0030,1,"-12V"},\
                {0.0060,1,"-24V"}, {0.0010,25,"Temp"}}

#define CV_VOLT_LABEL  const char *vlabel_a[] = \
              {"+24V","+12V","+6V ","GND "  ,"-6V " ,"-12V","-24V","Temp"}

/* Verifier Data Register */
#define CV_DATA_PATTERN        0x55         /* Data expected to be found in crate  */
#define MATCH(data,pattern)   ((data)==(pattern)?1:0)

/* Verifier Register Masks */
#define CV_ID_MASK             0x00ff       /* Id register      */
#define CV_ANLG_MASK           0x00ff       /* Analog registers */
#define CV_DATA_MASK           0x000000ff   /* Data register    */

/******************************************************************************************/
/*********************                    CAMAC Bus             ***************************/
/*********************               Command Line Test          ***************************/
/******************************************************************************************/

#define CMD_LINE_MASK 0x1fff
#define CMD_LINE_NUM 13
#define CMD_LINE_FUNC \
    const unsigned long  cmdLineFunc_a[CMD_LINE_NUM] = { 0,\
                                                    CCTLW__F1, CCTLW__F2, CCTLW__F4, CCTLW__F8,CCTLW__F16,\
                                                    CCTLW__A1, CCTLW__A2, CCTLW__A4, CCTLW__A8,\
                                                   (M28|F26A9), (M28|F26A8) , (M30|F26A9) }

#define CMD_LINE_OK \
    const unsigned long  cmdLineOk_a[CMD_LINE_NUM] = { 0,3,5,9,0x11,0x21,0x41,0x81,0x101,0x201,0x674,0xA34,0x1274 }

/******************************************************************************************/
/*********************                    CAMAC Bus             ***************************/
/*********************               Read-Write Line Test       ***************************/
/******************************************************************************************/

typedef enum cv_rwLine_nlines_e
{
   CAMAC_P16,
   CAMAC_P24
} cv_rwLine_nlines_te;

typedef enum cv_rwLine_access_e
{
  READ,
  WRITE
} cv_rwLine_access_te;

typedef enum cv_rwLine_type_e
{
  WALKING_ONE,
  WALKING_ZERO,
} cv_rwLine_type_te;

typedef struct cv_rwLine_test_s
{
  unsigned long        num;
  cv_rwLine_type_te    type_e;
  cv_rwLine_nlines_te  nlines_e;
  cv_rwLine_access_te  access_e;
}cv_rwLine_test_ts;


#define RW_LINE_NUM_TYPE  2
#define RW_LINE_NUM_TESTS 8
#define RW_LINE_NUM2      17
#define RW_LINE_NUM       25
#define RW_LINE_MASK      0xffffff

/* Test 3,4,7 & 8 are simulated */
#define RW_LINE_TEST \
    const cv_rwLine_test_ts rwLineTest_as[RW_LINE_NUM_TESTS] = {\
                  {1,WALKING_ONE ,CAMAC_P24,READ},\
		  {2,WALKING_ZERO,CAMAC_P24,READ},\
		  {3,WALKING_ONE ,CAMAC_P24,WRITE},\
                  {4,WALKING_ZERO,CAMAC_P24,WRITE},\
                  {5,WALKING_ONE ,CAMAC_P16,READ},\
                  {6,WALKING_ZERO,CAMAC_P16,READ},\
                  {7,WALKING_ONE ,CAMAC_P16,WRITE},\
		  {8,WALKING_ZERO,CAMAC_P16,WRITE}}

/*
 * The read line data (R1-16 and R1-24) have the following expected data.
 * The sets of data represent the read lines, and each element within a set is
 * the expected data. The first element in the set is the expected data from 
 * the walking one test, and the second element in the set is the expected data
 * from the walking zero test.
 */
#define RW_LINE_OK \
    const unsigned long rwLineOk_a[2][RW_LINE_NUM] = {{0,\
                                             0x1,        2,         4,         8,\
                                             0x10,       0x20,      0x40,      0x80,\
                                             0x100,      0x200,     0x400,     0x800,\
                                             0x1000,     0x2000,    0x4000,    0x8000,\
                                             0x10000,    0x20000,   0x40000,   0x80000,\
                                             0x100000  , 0x200000,  0x400000,  0x800000 },\
                                          {  0xffffff  ,\
                                             0xfffffe  , 0xfffffd , 0xfffffb , 0xfffff7,\
                                             0xffffef  , 0xffffdf , 0xffffbf , 0xffff7f,\
                                             0xfffeff  , 0xfffdff , 0xfffbff , 0xfff7ff,\
                                             0xffefff  , 0xffdfff , 0xffbfff , 0xff7fff,\
                                             0xfeffff  , 0xfdffff , 0xfbffff , 0xf7ffff,\
                                             0xefffff  , 0xdfffff , 0xbfffff , 0x7fffff }}

/******************************************************************************************/
/*********************        Subroutine Record Structures      ***************************/
/******************************************************************************************/

/* Private device support structure for subroutine record CV_RWline() */
typedef struct cv_rwline_info_s
{
  unsigned long *data_a;   /* camac data    */
  unsigned long *edata_a;  /* expected data */
  epicsMutexId   mlock;
}cv_rwline_info_ts;

/* Private device support structure for subroutine record CV_Cmdline() */
typedef struct cv_cmdline_info_s
{
  unsigned long *data_a;   /* camac data    */
  epicsMutexId   mlock;
}cv_cmdline_info_ts;


typedef enum cv_bus_type_e
{
   CMDLINE,      
   RWLINE,        
   RWLINE_P24   
} cv_bus_type_te;

typedef struct cv_busdata_s
{
  unsigned long   type;             /* type of bus data */
  unsigned long   nelem;            /* # items in array */
  unsigned long  *data_a;           /* data array       */
  unsigned long  *wt_data_a;        /* expected data    */

  /*
   * This lock should be used when accessing the data_a elements.
   */
  epicsMutexId    mlock;     /* Mutex lock       */ 
} cv_busdata_ts;


/******************************************************************************************/
/*********************                 Thread  Structure        ***************************/
/******************************************************************************************/

/* 
 * There are two threads that supports the crate verifier application:
 *    CV_OpThread - Operational, which process all camac messages from a queue
 *    CV_AsynThread - Sends periodic camac function request to the  messages queue 
 */
#define CV_OP_THREAD   0    /* CV_OpThread index into cvThread_as structure   */
#define CV_ASYN_THREAD 1    /* CV_AsynThread index into cvThread_as structure */
#define CV_NUM_THREADS 2    /* # of threads that support the crate verifier   */

/*
 * enum for interval for pocessing asyn messages. This is an index
 * into the asyn message list.
 */
#define CV_NUM_ASYN_PERIOD  2  /* number of asyn groupings */
typedef enum cv_interval_e
 {
   CV_10SEC, 
   CV_60SEC
 }cv_interval_te;


typedef struct cv_thread_s
 {        
   epicsThreadId         tid_ps;      /* task id                              */
   bool                  active;      /* indicate task active                 */
   bool                  stop;        /* indicate task should exit gracefully */
   epicsMessageQueueId   msgQId_ps;   /* message queue id                     */
   epicsEventId          evtId_ps;    /* event id                             */
} cv_thread_ts; 

/******************************************************************************************/
/*********************            Message Status Structure      ***************************/
/******************************************************************************************/

typedef struct cv_message_status_s
{
  epicsTimeStamp    reqTime;            /* camac request time           */
  unsigned long	    errCode;            /* status code                  */
  int               opDone;             /* Operation done flag          */
 
  epicsTimeStamp    opDoneTime;         /* time to finish camac request */
  double            elapsedTime;        /* elapsed time from request    */
                                        /* function was last requeested */
  epicsTimeStamp    lastTimeOk;         /* last time successful         */
  epicsTimeStamp    lastReqTime;        /* previous camac request time  */
  unsigned long	    lastErrCode;        /* previous error code          */

  IOSCANPVT         evt_p;              /* io scan event                */

  /*
   * This lock should be used when accessing anything within this data structure.
   * The functions CV_ClrMsgStatus() and CV_SetMsgStatus() should be used to
   * update the information in this structure. The function CV_ClrMsgStatus()
   * is called before a message is sent to the queue, and the function
   * CV_SetMsgStatus() is called after completion of the camac request.
   */
  epicsMutexId      mlock;              /* Mutex lock                   */ 
} cv_message_status_ts;

/******************************************************************************************/
/******************************************************************************************/
/*********************            Device Support Types          ***************************/
/******************************************************************************************/

/* Crate Verifier device support available */
#define MAX_EPICS_RTYP 7
typedef enum cv_epics_rtyp_e
{
    EPICS_RECTYPE_NONE,
    EPICS_RECTYPE_AI,
    EPICS_RECTYPE_BO,
    EPICS_RECTYPE_MBBI,
    EPICS_RECTYPE_LI,
    EPICS_RECTYPE_LO,
    EPICS_RECTYPE_WF
} cv_epics_rtyp_te;

#define CV_EPICS_RTYP_NAMES  \
   const char *epics_rtyp_names_ac[] = {"NONE","ai","bo","MbbiDirect","longin","longout","waveform"};


/******************************************************************************************/
/*********************          Camac Functions Supported       ***************************/
/******************************************************************************************/
#define MAX_STRING_LEN     (40)       /* Maximum param field string length */

typedef enum cv_camac_func_e
{
    CAMAC_INVALID_OP,
    CAMAC_RD_VOLTS,
    CAMAC_RD_CRATE_STATUS,
    CAMAC_RD_BUS_STATUS,
    CAMAC_RD_ID,
    CAMAC_RD_DATA,
    CAMAC_WT_DATA,
    CAMAC_TST_DATAWAY,
    CAMAC_TST_CMD,
    CAMAC_TST_RW,
    CAMAC_TST_RW_PATTERN
} cv_camac_func_te;

typedef struct 
{
    char              func_c[MAX_STRING_LEN]; /* function string    */
    cv_epics_rtyp_te  rtyp_e;                 /* record type        */ 
    cv_camac_func_te  func_e;                 /* function code      */
    dbfType           dtyp_e;                 /* db data type       */
    unsigned long     nelem;                  /* number of elements */
} cv_camac_func_ts;

#define MAX_CAMAC_FUNC_ASYN 3
#define MAX_CAMAC_FUNC     11
#define CV_CAMAC_FUNC \
    const cv_camac_func_ts  cv_camac_func_as[MAX_CAMAC_FUNC] = {\
    {"VOLTS"      , EPICS_RECTYPE_AI   , CAMAC_RD_VOLTS        },\
    {"STAT"       , EPICS_RECTYPE_MBBI , CAMAC_RD_CRATE_STATUS },\
    {"BUS"        , EPICS_RECTYPE_MBBI , CAMAC_RD_BUS_STATUS   },\
    {"ID"         , EPICS_RECTYPE_LI   , CAMAC_RD_ID           },\
    {"DATA"       , EPICS_RECTYPE_LI   , CAMAC_RD_DATA         },\
    {"DATA"       , EPICS_RECTYPE_LO   , CAMAC_WT_DATA         },\
    {"VERIFY"     , EPICS_RECTYPE_BO   , CAMAC_TST_DATAWAY     },\
    {"CMD"        , EPICS_RECTYPE_WF   , CAMAC_TST_CMD         },\
    {"RW"         , EPICS_RECTYPE_WF   , CAMAC_TST_RW          },\
    {"RW_PATTERN" , EPICS_RECTYPE_WF   , CAMAC_TST_RW_PATTERN  } }


typedef struct cv_asyn_types_s
{
   cv_camac_func_te func_e;
   cv_interval_te   interval_e;
} cv_asyn_types_ts;

#define CV_NUM_ASYN_FUNC    4
#define CV_ASYN_TYPES  \
    const cv_asyn_types_ts asynMsgs_as[CV_NUM_ASYN_FUNC] = { {CAMAC_RD_CRATE_STATUS,CV_10SEC},\
                                                             {CAMAC_RD_ID,CV_10SEC},\
                                                             {CAMAC_RD_VOLTS,CV_10SEC},\
                                                             {CAMAC_TST_DATAWAY,CV_60SEC} }

/******************************************************************************************/
/*********************       CAMAC Status/Data Structures       ***************************/
/******************************************************************************************/

#ifndef CAMBLKSTRUC_HM
/* cannot included both camblkstruc.h and cam_proto.h */
typedef struct
{   unsigned int  cctlw;         /* MBCD control word.               */
     void          *stad_p;      /* pointer to status/data.          */
     unsigned short wc_max,      /* Max wordcount.                   */
                   notused;      /* So consistent with old docs      */
} mbcd_pkt_ts;

typedef struct
{    unsigned short key,         /* Validation key.                   */
                    nops,        /* Max # of packets.                */
                    iop,         /* Current # of packets.            */
                    tbytes,      /* ?? */
                    bitsummary,
                    spare;
     void          *pkg_p;       /* pointer in dual ported mem to 1st packet. */
} mbcd_pkghdr_ts;
                          /*  Structure of an MBCD package.  */
typedef struct
{   mbcd_pkghdr_ts hdr;
    mbcd_pkt_ts    mbcd_pkt[1];
} mbcd_pkg_ts;
#endif /* CAMBLKSTRUC_HM */

typedef union
{
  unsigned short _a[2];
  unsigned int   _i;
} camstatd_tu;

typedef struct
{
   unsigned int      stat;
   short             data;
} statd_2_ts;

typedef struct
{
   unsigned int      stat;
   unsigned short    data;
} statd_2u_ts;

typedef struct
{
   unsigned int      stat;
   unsigned long     data;
} statd_4u_ts;

typedef struct
{
   unsigned int      stat;
   long              data;
} statd_4_ts;

/* 
 * This is the camac status data for the read-line test using
 * the ROTATE register for a walking one or zero bit test.
 * In this case, we are testing read lines R1-16, but we need to 
 * read two extra word .The first word, our 17th data element
 * is required because the first read for the walking one 
 * state will be zeros. The next will be have a one in R1 and zero's
 * in R24-R2. 
 *
 * The 18th word is required becasue a camac word block transfer is used,
 * which require a word swap. The side affects of this fact is that MUST
 * read an even number of words so that last word of data can be retrieved
 * otherwise, our last word of data is garbage.
 */
typedef struct campkt_statd_rw_s
{
   unsigned int     stat;
   unsigned short   data_a[RW_LINE_NUM2+1];       /* 18 words  - 36 bytes of data */
} campkt_statd_rw_ts;

typedef struct campkt_statd_4u_rw_s
{
   unsigned int    stat;
   unsigned long   data_a[RW_LINE_NUM];           /* 25 longwords data  (50 words)- 100 bytes of data */
} campkt_statd_4u_rw_ts;

/******************************************************************************************/
/*********************          CAMAC Packet Structures         ***************************/
/******************************************************************************************/

typedef struct campkg_nodata_s
{
  void               *pkg_p;            /* ptr to camac package          */
  unsigned int        stat;             /* status-data                   */
} campkg_nodata_ts;


/* A camac package with 2-byte data */
typedef struct campkg_s
{
  void                   *pkg_p;         /* ptr to camac package          */
  statd_2u_ts             statd_s;       /* status-data                   */
} campkg_ts;

/*  A camac package with 4-byte data (P24) */
typedef struct campkg_4u_s
{ 
  void                   *pkg_p;         /* ptr to camac package          */
  statd_4u_ts             statd_s;       /* status-data                   */
} campkg_4u_ts; 

/* 
 * Analog register camac package reading subsequent subaddress in
 * one packet using the SA camac control word option.
 */
typedef struct campkg_volts_s
{
  void                    *pkg_p;                             /* ptr to camac package */
  statd_2_ts               statd_as[CV_NUM_ANLG_CHANNELS] ;   /* status-data          */
} campkg_volts_ts;

/*
 * Data Register camac package. 
 * A read includes two reads to verify that the crates is online.
 * A write includes a write and a read to verify the setpoint latched.
 */
typedef struct campkg_data_s
{
  void                   *pkg_p;         /* ptr to camac package          */
  statd_4u_ts             statd_as[2];   /* status-data                   */
} campkg_data_ts;

/* Command Line test camac package */
typedef struct campkg_cmd_s
{
  void                   *pkg_p[2];             /* ptr to camac package   */
  statd_4u_ts             statd_s;
  unsigned int            stat_a[CMD_LINE_NUM];  
  statd_4u_ts             rd_statd_as[CMD_LINE_NUM+1]; 
  unsigned int            inhibit_stat; 
} campkg_cmd_ts;

/* 
 * Read line test using walking one bit without p24.
 * The read of the DATA register expectes 34 bytes of data.
 */
typedef struct campkg_rlines_walk1_s
{
  void                   *pkg_p;             /* ptr to camac package                          */
  unsigned int            stat;              /* clear registers on bus, pulse the C-line      */
  campkt_statd_rw_ts      rd_statd_s;        /* read DATA register                            */
} campkg_rlines_walk1_ts;

/* 
 * Read line test using walking zero bit without p24.
 * The read of the DATA register expects 100 bytes of data.
 */
typedef struct campkg_rlines_walk0_s
{
  void                   *pkg_p;             /* ptr to camac package                          */
  unsigned int            stat;              /* clear registers on bus, pulse the C-line      */
  unsigned int            wt_stat;           /* set ROTATE register for walking zeros         */
  campkt_statd_rw_ts      rd_statd_s;        /* read DATA register and rotate left            */
} campkg_rlines_walk0_ts;

/* 
 * Read line test using walking one bit with p24.
 * The read of the DATA register expectes 100 bytes of data.
 */
typedef struct campkg_rlines_walk1_4_s
{
  void                   *pkg_p;             /* ptr to camac package                          */
  unsigned int            stat;              /* clear registers on bus, pulse the C-line      */
  campkt_statd_4u_rw_ts   rd_statd_s;        /* read DATA register and rotate left            */
} campkg_rlines_walk1_4_ts;


typedef struct campkg_rlines_walk0_4_s
{
  void                   *pkg_p;             /* ptr to camac package                          */
  unsigned int            stat;              /* clear registers on bus, pulse the C-line       */
  unsigned int            wt_stat;           /* set ROTATE register for walking zero          */
  campkt_statd_4u_rw_ts   rd_statd_s;        /* read DATA register                            */
} campkg_rlines_walk0_4_ts;

/* 
 * Write line test with simulated walking ones and zeros.
 * This test is done with p24 on the write and read operations.
 */
typedef struct campkg_wlines_4_s
{
  void                   *pkg_p;             /* ptr to camac package  */
  statd_4u_ts             wt_statd_s;        /* set DATA register     */
  statd_4u_ts             rd_statd_s;        /* read DATA register    */
} campkg_wlines_4_ts;

/* 
 * Write line test with simulated walking ones and zeros.
 * This test is done without p24 on the write and with p24
 * on read operation
 */
typedef struct campkg_wlines_s
{
  void                   *pkg_p;             /* ptr to camac package               */
  statd_4u_ts             clr_statd_s;       /* clear out old data from            */
					     /* high order bytes of DATA registser */
  statd_2u_ts             wt_statd_s;        /* load DATA register                 */
  statd_4u_ts             rd_statd_s;        /* read DATA register                 */
} campkg_wlines_ts;

typedef struct campkg_rwlines_s
{
    /* Read line test with P24 */
    campkg_rlines_walk1_4_ts     test1_s;     /* walking ones bit with p24   */
    campkg_rlines_walk0_4_ts     test2_s;     /* walking zeros bit with p24  */

    /*  
     * Write line test with simulated alking ones and zeros. This test
     * is donw with P24 on both the read and write operations.
     * Note: teste 4 just uses test 3 package....setting the data 
     * with walking zero bit.
     */
    campkg_wlines_4_ts            test3_s;    /* simulated walking ones bit with p24  */

    /* Read line test without P24 */
    campkg_rlines_walk1_ts        test5_s;    /* walking ones bit without p24        */
    campkg_rlines_walk0_ts        test6_s;    /* walking zeros bit without p24       */   
 
    /*
     * Write line test with simulated walking ones and zeros.
     * This test is done without p24 on the write and with p24 on read operation.
     * Note: teste 8 just uses test 7 package....setting the data 
     * with walking zero bit.    
     */
    campkg_wlines_ts              test7_s;    /* simulated walking ones bit           */
   
}campkg_rwlines_ts;

/* 
 * A dataway verification test performs a series of camac functions,
 * whereby a separate camac package has been initialized for each
 * module. 
 */
typedef struct campkg_dataway_s
{

  campkg_nodata_ts         pulseC_s;      /* clear registers on bus        */
  campkg_nodata_ts         inhibit_s;     /* clear inhibit line I=0        */
  campkg_4u_ts             scc_s;         /* set into addressing mode      */
  campkg_cmd_ts            cmd_s;         /* command line package          */
  campkg_rwlines_ts        rwlines_s;     /* Read Write lines test         */

  bool                     init;          /* flag all packages initalized  */
  camac_xq_status_ts       Xstat_s;       /* X-response status             */
  camac_xq_status_ts       Qstat_s;       /* Q-response status             */
  bool                     cmdLineErr;    /* Command Line test status      */
  unsigned short           rwLineErr;     /* Read-Write Line test status   */
  bool                     timeout;       /* CAMAC crate timeout           */
} campkg_dataway_ts;

typedef struct 
{
   void                   *pkg_p;      
   unsigned int            pulseZ_stat; 
   unsigned int            inhibit_stat; 
   statd_4u_ts             wt_statd_s;
   statd_4u_ts             rd_statd_s;
}campkg_init_ts;

/******************************************************************************************/
/*********************             CAMAC Package Block          ***************************/
/******************************************************************************************/

/*
 * This structure contains all of the camac package that will be repeatly
 * used to access the crate verifier module registers and the serial crate
 * controller module.
 */ 
typedef struct camac_block_s
{
  campkg_ts               rd_id_s;      /* read id package               */
  campkg_data_ts          rd_data_s;    /* read data package             */
  campkg_data_ts          wt_data_s;    /* write data package            */
  campkg_volts_ts         rd_volts_s;   /* read voltage package          */
  campkg_init_ts          init_s;       /* crate online watch            */
  campkg_dataway_ts       dataway_s;    /* dataway test packages         */    
} camac_block_ts;

/******************************************************************************************/
/*********************               Crate Bus Status           ***************************/
/******************************************************************************************/

/*
 * Bit definitions for crate bus status:                                              
 *                                                                
 *     Bit  0 = Crate failed verification test (ie. Bus Error)
 *     Bit  1 = Command line test  
 *     Bit  2 = Read/write line test
 *     Bit  3 = unused
 *     Bit  4 = X-Response Error, was one when zero was expected             
 *     Bit  5 = X-Response Error, was zero when one was expected
 *     Bit  6 = Q-Response Error, was one when zero was expected
 *     Bit  7 = Q-Response Error, was zerio when one was expected
 *     Bit 8-15 = Failed verification test #, if zero test passed.
 * 
 * There are 8 bus line test performed. The test that failed is stored into
 * the upper byte of the bus status. If all 8 test pass this byte is
 * set to zero. The bus status is set by the dataway verification test
 * which takes place once a minute and on-demand (request from user).
 */

#define BUS_STATUS_MASK      0x1fff
#define BUS_STATUS_BUS_ERR   0x0001
#define BUS_STATUS_CMD_ERR   0x0002
#define BUS_STATUS_BADMODU   0x0004
#define BUS_STATUS_CTO_ERR   0x0008
#define BUS_STATUS_XQ_ERR    0x00f0
#define BUS_STATUS_X_ERR     0x0010
#define BUS_STATUS_NOX_ERR   0x0020
#define BUS_STATUS_Q_ERR     0x0040
#define BUS_STATUS_NOQ_ERR   0x0080
#define BUS_STATUS_RW_ERR    0x0f00
#define BUS_STATUS_INIT_ERR  0x1000


#define BUS_STATUS_RWERR_SHIFT 8
#define BUS_RWLINE_BYPASSED    0
#define BUS_RWLINE_PASSED      9



#define CRATE_TIMEOUT( status )    ( ((status) & BUS_STATUS_CTO_ERR) ? 1:0 )
#define BUS_ERR( status )          ( (status)  & BUS_STATUS_BUS_ERR  ? 1:0 )
#define CMDLINE_ERR(status)        ( (status)  & BUS_STATUS_CMD_ERR  ? 1:0 )
#define RWLINE_BYPASSED( status )  ( ((status) & BUS_STATUS_RW_ERR)==BUS_RWLINE_BYPASSED ? 1:0 )
#define RWLINE_SUCCESS( status )   ( (((status) & BUS_STATUS_RW_ERR)==0x900) ? 1:0 )
#define RWLINE_TEST( status )      ( ((status) & BUS_STATUS_RW_ERR) >> BUS_STATUS_RWERR_SHIFT )

#define RWLINE_ERR( test )         ( ((test)>BUS_RWLINE_BYPASSED) && ((test)<BUS_RWLINE_PASSED) ?1:0 )
#define RWLINE_P24( test )         ( ((test)>4) && ((test)<9) ? 0:1 )
#define WLINE_TEST( test )         ( (test)==3)||(test)==4)||(test)==7)||(test)==8)?1:0 )
#define RLINE_TEST( test )         ( (test)==1)||(test)==2)||(test)==5)||(test)==6)?1:0 ) 1:0 )  

typedef struct 
{
  unsigned int busErr            : 1;  /* bus error           */
  unsigned int cmdLineErr        : 1;  /* command line error  */
  unsigned int badModule         : 1;  /* bad verifier module */
  unsigned int camTimeoutErr     : 1;  /* camac crate timeout */

  unsigned int Xerr_was_one      : 1;
  unsigned int Xerr_was_zero     : 1;  /* nox error */
  unsigned int Qerr_was_one      : 1;
  unsigned int Qerr_was_zero     : 1;  /* noq error */

  unsigned int rwLineErr         : 4;  /* failed test number 0-8, success=0 */

  unsigned int initErr           : 1;  /* init failed */
  unsigned int unused            : 3;  /* spare       */
} cv_bus_status_ts;

typedef union
{
  cv_bus_status_ts   _s;
  unsigned short     _i;
} cv_bus_status_tu;


/******************************************************************************************/
/*********************      Crate Status Summary (worst case)   ***************************/
/******************************************************************************************/

/*
 * These bitmasks are used by the PV
 *  <device>:STATMSG
 * to provide a worst case status summary
 */
#define CRATE_STATSUMY_MASK      0xf
#define CRATE_STATSUMY_VPWROFF   0  
#define CRATE_STATSUMY_VPWRON    1
#define CRATE_STATSUMY_ONINIT    2   /* crate pwr on and init */
#define CRATE_STATSUMY_VOLTERR   3
#define CRATE_STATSUMY_TEMPERR   4
#define CRATE_STATSUMY_BADID     5
#define CRATE_STATSUMY_INVDATA   6
#define CRATE_STATSUMY_NOTFOUND  7
#define CRATE_STATSUMY_VOLTWARN  8
#define CRATE_STATSUMY_TEMPWARN  9
#define CRATE_STATSUMY_BUSERR   10
#define CRATE_STATSUMY_CMDERR   11
#define CRATE_STATSUMY_RWERR    12
#define CRATE_STATSUMY_XQERR    13
#define CRATE_STATSUMY_TIMEOUT  14
#define CRATE_STATSUMY_INITERR  15

/******************************************************************************************/
/*********************               Crate Online Status        ***************************/
/******************************************************************************************/

/* 
 * from RMS_INCLUDE: CRTDCTRS.PNC
 *
 * Bit definitions for longwords in CSTR:CRTS                    
 *   (crate status):                                              
 *                                                                
 *     Bit  0 = Crate online.                                     
 *     Bit  1 = Crate initialized.                                                                              
 *     Bit 10 = At last offline-to-online transition, verifier    
 *               module showed that crate had been powered off.   
 * 
 *
 *  Declare   ONBIT      literally '00000001H',
 *            ONINIT     literally '00000002H',
 *            PDUSTATBIT literally '00000004H', (not used for linac upgrade)
 *            BUS_FAIL   literally '00000008H', (see CAMC:<area>:<crate>00:BUS_STAT.SEVR
 *            VOLT_FAIL  literally '00000010H', ----> CAMC:<area>:<crate>00:VSTAT.SEVR
 *            TEMP_FAIL  literally '00000020H', ----> CAMC:<area>:<crate>00:TEMP.SEVR
 *            PDUREQBIT  literally '00000100H', (not used for linac upgrade)
 *            PDURCVBIT  literally '00000200H', (not used for linac upgrade)
 *            VPWROFFBIT literally '00000400H', 
 *            PAUSTATBIT literally '00000800H', (not used for linac upgrade)
 *            PSUSTATBIT literally '00001000H'; (not used for linac upgrade)
 */
#define CRATE_STATUS_ONLINE             0x0001  /* bit0 */
#define CRATE_STATUS_VPWRON             0x0001  /* bit0  (duplicate) */
#define CRATE_STATUS_INIT               0x0002  /* bit1 */
#define CRATE_STATUS_VPWROFF            0x0004  /* bit3 - crate pwr off to on transition */
#define CRATE_STATUS_CTO_ERR            0x0008  /* bit2 */
#define CRATE_STATUS_WDATA_ERR          0x0020  /* bit5 */
#define CRATE_STATUS_RDATA_ERR          0x00C0  /* bit6 & 7 */
#define CRATE_STATUS_CAM_ERR            0x0100  /* bit8 */

/* Combined bit status */
#define CRATE_STATUS_ONINIT             0x0003  /* bit0-1 */
#define CRATE_STATUS_GOOD               0x0003
#define CRATE_STATUS_R1DATA_ERR         0x0040 /* rd1 data error  */
#define CRATE_STATUS_R2DATA_ERR         0x0080 /* rd2 data error  */
#define CRATE_STATUS_MASK               0x01FF

#define CRATE_STATUS_RDATA              0x3

/* Crate status from periodic crate power online check */
typedef struct 
{
  unsigned int online           : 1;  /* crate online                           */
  unsigned int init             : 1;  /* crate initialized                      */
  unsigned int offOnTransition  : 1;  /* crate offline to online transition     */
  unsigned int camTimeoutErr    : 1;  /* camac timeout error                    */

  unsigned int spare            : 1;  /*  (not used)                            */
  unsigned int dataWtErr        : 1;  /* crate failed data write                */
  unsigned int dataRdErr        : 2;  /* crate failed data read                 */

  unsigned int camErr           : 1;  /* camac error                            */
  unsigned int unused           : 7;  /* (not used)                             */
} cv_crate_status_ts;

typedef union
{
  cv_crate_status_ts _s;
  unsigned short     _i;
} cv_crate_status_tu;

typedef enum cv_crate_flag_e
{
  CV_CRATEON  = 1,
  CV_CRATEOFF,
  CV_BADWREAD,
  CV_CRATENOINIT
} cv_crate_flag_te;
 
typedef struct cv_crate_online_status_s
{
       bool                     z_off;
       cv_crate_flag_te         flag_e;  
       bool                     idErr;   

       unsigned short           first_watch;
       unsigned short           reinit;
       unsigned short           nr_reinit;
       float                    volts_a[CV_NUM_ANLG_CHANNELS];

       cv_crate_status_tu       stat_u;        /* current crate status         */
       cv_crate_status_tu       prev_stat_u;   /* crate status from last check */
       cv_bus_status_tu         bus_stat_u;    /* current dataway test status  */
       epicsMutexId             mlock;        /* Mutex lock                    */ 
} cv_crate_online_status_ts;

/******************************************************************************************/
/*********************                 STAT Summary PV          ***************************/
/******************************************************************************************/

#define STATSUMY_GOOD            1
#define STATSUMY_CRATE_OFFLINE   2
#define STATSUMY_VOLT_ERR        3   
#define STATSUMY_TEMP_ERR        4
#define STATSUMY_ID_ERR          5
#define STATSUMY_INIT_ERR        6
#define STATSUMY_CRATE_OFF       7
#define STATSUMY_BUS_ERR         8  
#define STATSUMY_DATA_ERR        9  

/******************************************************************************************/
/*********************        Module Information Structure      ***************************/
/******************************************************************************************/

typedef struct cv_module_s
{
    ELLNODE                      node;                          /* Link List Node            */
    epicsMessageQueueId	         msgQId_ps;                     /* all modules share one Q   */  

    short	     	         b;  	                        /* CAMAC branch              */
    short	 	         c;	                        /* CAMAC crate               */
    short		         n;	                        /* CAMAC slot (ie node)      */
    short                        present;                       /* Module present  X=1       */
    unsigned long                pattern;                       /* write to data register    */

     /* Crate Verifierd data */
     unsigned long	   id;                                  /* module ID register        */
     unsigned long         data;                                /* DATA register pattern     */
     struct 
     {
          epicsMutexId          mlock;
          unsigned long         data_a[CMD_LINE_NUM];           /* command line data         */
     } cmdLine_s;

     struct 
     {
          epicsMutexId          mlock;
          unsigned short        test;                           /* test number (0-8)         */
          cv_rwLine_type_te     type_e;                         /* type of pattern           */
          unsigned long         err_a[RW_LINE_NUM];             /* read write line error     */
          unsigned long         data_a[RW_LINE_NUM];            /* read write line data      */
          unsigned long         expected_data_a[RW_LINE_NUM];   /* expected read write data  */
     } rwLine_s;

     /* Status */
     cv_crate_online_status_ts   prev_crate_s;              /* previous crate online status */
     cv_crate_online_status_ts   crate_s;                   /* crate online status          */
     cv_message_status_ts        mstat_as[MAX_CAMAC_FUNC];                /* message status */

    /* 
     * Camac package block. The packages for a module and operation are
     * allocated once and reused on subsequent Camac requset.
     */
     unsigned long               ctlw;           /* camac control word (crate+ slot) */
     camac_block_ts              cam_s;   

} cv_module_ts;

typedef cv_module_ts CV_MODULE;

/******************************************************************************************/
/*********************   Device Support Private Data Structure  ***************************/
/******************************************************************************************/

#define MAX_QUEUED_MSGS         (20)         /* max num of queued msgs                */
#define CV_MSG_ASYN             "ASYN"       /* message from asyn thread (periodic)   */
#define CV_MSG_DSUP             "DSUP"       /* message from device support on demand */

/* 
 * The private device support is also as the message sent to the queue.
 * Messages are sent by device support or from the CV_AsynThread(),
 * which send periodic camac function requests to the queue.
 * Device support provide on-Demand
 */
typedef struct cv_request_s
{
    ELLNODE                node;                         /* Link List Node           */
    char                   source_c[MAX_STRING_LEN];     /* source of message        */
    cv_camac_func_te       func_e;                       /* Camac function type      */

    CV_MODULE             *module_ps;                    /* ptr to module info       */
    void                  *cam_p;                        /* pointer to camac package */
    cv_message_status_ts  *mstat_ps;                     /* message status           */

    dbCommon              *rec_ps;                       /* ptr to record info       */    
    short                  a;                            /* camac subaddress code    */
    short                  f;                            /* camac function code      */
 
} cv_request_ts;
typedef cv_request_ts CV_REQUEST;

/******************************************************************************************/

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* _DEV_CV_H_  */
