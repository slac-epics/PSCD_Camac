/*
=============================================================

  Abs:  EPICS Driver Support for the CAMAC Crate Verifier 

  Name: drvCV.c
         EPICS Driver Support
         ---------------------
         *   drvCV_Init      - EPICS driver initialization
         *   drvCV_Report    - EPICS driver report

         Threads
         -------
         *  CV_OpThread       - Processes messages from the queue
	 *  CV_AsynThread     - Sends asynchronouse messages to the queue
         *  CV_AsynThreadStop - Force the Asynchronous thread to exit

        Message Utilities
        -------------------
	*   CV_AddMsg        - Add a request message to the asynronous message linked list.
            CV_ClrMsgStatus  - Message setup, performed prior to sending message to queue
        *   CV_SetMsgStatus  - Message completion, performed after messasge has completed
        *   CV_SendMsgs      - Submit messages from linked list provided to the queue
        *   CV_ProcessMsg    - Process message from the queue

        Miscellaneous
        ---------------
	    CV_Start           - Build module list,start threads and init camac bus for each crate
         *  CV_StartInit       - Initialize camac crate bus before iocInit
            CV_AddModule       - Add crate verifier module to the module linked list
            CV_FindModuleByBCN - Find a crate verifier module in the module linked list 
            CV_DeviceInit      - Initialize a requeset message 
            CV_SetBusStatus    - Set Camac Crate bus status based on verification test
            CV_SetCrateStatus  - Set Camac Crate status bitmask used as a pv
	    isCrateOnline      - Return crate online status with input argument of crate number

        I/O Functions
        ---------------
	*   CV_ReadVoltage     - Read the crate verifier analog registers (ie. subaddress 0-7)
        *   CV_ReadId          - Read the crate verifier identification register
	*   CV_ReadData        - Read the crate verifier data register
	*   CV_WriteData       - Set the crate verifier data register
	*   CV_TestDataway     - Test Camac Crate dataway test proceedure
	*   CV_CrateCmdLine    - Test the Camac Crate command lines
	*   CV_CrateRWLine     - Test the Camac Crate read write lines
	*   CV_IsCrateOnline   - Determine if the Camac Crate in phyciscallly online (ie. input argument CV_MODULE *) 
        
        Camac Package Initalization
        ------------------------------
	*   CV_ReadVoltageInit     - Initalize the Camac package to read the analog registers
	*   CV_ReadIdInit          - Initalize the Camac package to read the crate verifier identification register
	*   CV_ReadDataInit        - Initalize the Camac package to read the crate verifier data register
	*   CV_WriteDataInit       - Initalize the Camac package to set the crate verifier data register
	*   CV_TestDatawayInit     - Initalize the Camac package to test the Camac crate dataway
	*   CV_CratePulseCInit     - Initalize the Camac package to clear the Camac crate bus registers (pulse C-Line)
	*   CV_CrateClrInhibitInit - Initalize the Camac package to clear the Camac crate bus inhibit line (I-line)
	*   CV_CrateSCCInit        - Initalize the Camac package to reset the Camac crate to addresssing mode
	*   CV_CrateCmdLineInit    - Initalize the Camac package #1 to test the Camac crate command lines
	*   CV_CrateCmdLineInit2   - Initalize the Camac package #2 to test the Camac crate command lines
	*   CV_CrateRWLineInit     - Initalize the Camac package #1 to test the Camac crate read write lines (ie. test #1-4)
	*   CV_CrateRWLineInit2    - Initalize the Camac package #2 to test the Camac crate read write lines (ie. test #5-8)

	*   CV_DatawayInitData     - Initalize the dataway test local data
	*   CV_CheckReadData       - Check the data register data for errors 
	*   CV_CrateInit           - Camac crate initialization part #1
	*   CV_CrateInitInit       - Camac crate initialization part #2


  Note: * indicates static functions

  Proto: drvCV_proto.h

  Auth: 19-Jul-2009, K. Luchini       (LUCHINI)
  Rev : dd-mmm-yyyy, Reviewer's Name  (USERNAME)

-------------------------------------------------------------
  Mod:
        11-Feb-2017, K. Luchini       (LUCHINI):
          add second argument, slot to isCrateOnline()
        09-Nov-2016, K. Luchini       (LUCHINI):
          CV_IsCrateOnline() - clear camac timeout and 
          camac error in the camac status bitmask, 
          reflected in the pv CAMC:<sector>:<crate>:STAT

=============================================================
*/

/* Header Files */
#include "drvPSCDLib.h"
#include "devCV.h"
#include "cam_proto.h"         /* for camalo,camalo_reset,camadd,camio,camgo */
#include "drvCV_proto.h"
#include "CVTest_proto.h"

/* Local Prototypes for EPICS Driver Support Routines */
static long         drvCV_Init(void);
static long         drvCV_Report(int level);

/* Local Prototypes for Thread Routines */
static void         CV_OpThread(void); 
static void         CV_AsynThread(void);
static void         CV_StartInit(void);

/* Local Prototypes for Message Utilities */
static void         CV_SetMsgStatus( vmsstat_t status, cv_message_status_ts * const msgstat_ps );
static void         CV_AddMsg( cv_camac_func_te func_e,
                               cv_interval_te   interval_e, 
                               char           * const source_c,
                               CV_MODULE      * const module_ps );
static void         CV_SendMsgs( ELLLIST * const msgList_p );
static void         CV_ProcessMsg( CV_REQUEST * const  msgRecv_ps );

/* Local Prototypes for IO Routines */
static long         CV_ReadVoltage(   CV_MODULE * const module_ps );
static long         CV_ReadId(        CV_MODULE * const module_ps );
static long         CV_ReadData(      CV_MODULE * const module_ps );
static long         CV_WriteData(     CV_MODULE * const module_ps, unsigned long data );
static long         CV_TestDataway(   CV_MODULE * const module_ps );
static long         CV_IsCrateOnline( CV_MODULE * const module_ps );

static vmsstat_t    CV_ReadVoltageInit( short b, short c, short n, campkg_volts_ts   * const cam_ps );
static vmsstat_t    CV_ReadIdInit(      short b, short c, short n, campkg_ts         * const cam_ps );
static vmsstat_t    CV_ReadDataInit(    short b, short c, short n, campkg_data_ts    * const cam_ps );
static vmsstat_t    CV_WriteDataInit(   short b, short c, short n, campkg_data_ts    * const cam_ps );
static vmsstat_t    CV_TestDatawayInit( short b, short c, short n, campkg_dataway_ts * const cam_ps );
static vmsstat_t    CV_CratePulseCInit( short branch, short crate, short slot , campkg_nodata_ts * const cam_ps );
static vmsstat_t    CV_CrateClrInhibitInit( short branch, short crate, short slot , campkg_nodata_ts * const cam_ps );
static vmsstat_t    CV_CrateSCCInit( short branch, short crate, short slot , campkg_4u_ts * const cam_ps );
static vmsstat_t    CV_CrateCmdLineInit( short branch, short crate, short slot ,  campkg_cmd_ts * const cam_ps );
static vmsstat_t    CV_CrateCmdLineInit2( short branch, short crate, short slot , campkg_cmd_ts * const cam_ps );
static vmsstat_t    CV_CrateRWLineInit( short branch, short crate, short slot , campkg_dataway_ts * const cam_ps );
static vmsstat_t    CV_CrateRWLineInit2( short branch, short crate, short slot , campkg_dataway_ts * const cam_ps );
static void         CV_DatawayInitData( CV_MODULE * const module_ps );
static vmsstat_t    CV_CrateCmdLine( CV_MODULE * const module_ps );
static vmsstat_t    CV_CrateRWLine( CV_MODULE * const module_ps );

static vmsstat_t    CV_CheckReadData( CV_MODULE * const module_ps, vmsstat_t status );
static vmsstat_t    CV_CrateInit(     CV_MODULE * const module_ps, bool  pulzeZ_e );
static vmsstat_t    CV_CrateInitInit( CV_MODULE * const module_ps );
static vmsstat_t    CV_SetBusStatus(  CV_MODULE * const module_ps );
static void         CV_SetCrateStatus( CV_MODULE * const module_ps, unsigned short stat, unsigned short mask );


/* Global functions */
long         CV_Start( unsigned long crate_mask, short n );
CV_MODULE  * CV_AddModule( short b, short c, short n );
void         CV_AsynThreadStop(void);


/* Global variables */
int     CV_DRV_DEBUG = 0;
int     CV_DIAG_DISABLE = 1;
extern  struct PSCD_CARD pscd_card;

#ifndef USE_TYPED_DRVET
struct  drvet drvCV = {2, drvCV_Report, drvCV_Init};
#else
const drvet drvCV = {2, drvCV_Report, drvCV_Init};
#endif

/* 
 * All Crate Verifier Camac transactions go thru one PSCD.
 * We have only one task per msgQ that will be needed 
 */
static  int                     nmodules = 0;
static  ELLLIST                 moduleList_s  = {{NULL, NULL}, 0};
static  ELLLIST                 asynMsgList_as[2] = {{{NULL, NULL}, 0}, {{NULL, NULL}, 0}};
static  cv_thread_ts            threads_as[CV_NUM_THREADS] = {{NULL,0,0,NULL}, {NULL,0,0,NULL}};


/*====================================================
 
  Abs:  Initalization for all Crate Verifiers

  Name: CV_Start
 
  Args:  crate_mask             Bit mask of online crates 
          Type: integer         Note: 0-15, where bit 0 indictes
          Use:  unsigned long   is not used, bit1= crate1 , bit2=crate2
          Acc:  read-only       bit3=crate3, etc.
          Mech: By value        
 
        n                       Camac slot number (optional, default=1)
          Type: value           Note: 1-24, if the slot is zero, or    
          Use:  short           the argument is not supplied the 
          Acc:  read-only       default is used.
          Mech: By value

  Rem: This function is called from CV_Start, prior
       to iocInit to setup the linked list of modules
       and initalize the message queue id for each module 
       in the linked list.

       -----------------------------------------------
       Future:  (use string input)
       The input argument is a string representation of each
       crate verifier module location. The crate and slot
       are identified by "CxSy" where x represents the crate number
       and y represents the slot number. A comma is used to separate
       multiple crate verifier modules as follows:
 
         "C1S1,C2S1,C3S1"
 
       The above example, identifies a crate verifier module
       is crate 1 slot 1, crate 2 slot 1 and crate 3 slot 1.
       ----------------------------------------------- 
        
       There should be only ONE crate verifier module
       per crate, and this modules should be installed
       in slot 1 to provide a comprehensive dataway
       verification test, which takes place once per minute.

       This function performs the following tasks:
         1) creates the linked list
         2) creates the message queue
         3) start the message processing thread.
         4) initalize the crate verifier modules
            with the message queue id
         5) start the asyn message request thread       
 
  Side: This function should be called prior
        to any Camac driver initialization. Since
        epics R4-13-8-2 cannot ensure the order of
        driver initalization this function is called
        prior to iocInit.
 
  Ret:  None
 
=======================================================*/
long CV_Start( unsigned long crate_mask, short n )
{
    long                status     = OK;              /* status return     */
    short               bitNo      = 2;               /* bit number (1-15) */
    static const short  num        = MAX_CRATE_ADR;   /* no of crates      */
    static const short  branch     = 0;     /* branch number (0,1) ignored */
    short               crate      = 1;     /* crate number                */
    short               slot       = 1;     /* slot number (always=1)      */
    int                 maxMsgs    = MAX_QUEUED_MSGS;
    unsigned int        stackSize  = 20480;
    cv_thread_ts       *thread_ps  = &threads_as[CV_OP_THREAD];


    /* The serial crate controller uses the last 2-slots in the crate */
    if ((n<0) || (n>(MAX_CRATE_SLOT-2)))
    {
       printf("Invalid slot number, the Crate Verifier can only used slots 1-23\n");
       status = ERROR;
       return(status);
    }
    else if (!n)
      slot=1;  /* use default, slot 1 */
    else
      slot=n;
    printf("Crate Verifier is expected in slot %hd\n",slot);

    /* Build module linked list for available crates */
    for (crate=1; crate<num; bitNo<<=1,crate++)
    {
      if ( bitNo & crate_mask )     
	CV_AddModule(branch,crate,slot);
    }/* End of FOR loop */   

    /*
     * Keep track of the number of modules at this point in case other
     * modules are added after iocInit().
     */
    nmodules = ellCount(&moduleList_s); 
    if (!nmodules)
    {
      errlogSevPrintf(errlogInfo,CV_NOMODU_MSG);
      return(status);
    }
    errlogSevPrintf(errlogInfo,CV_MODU_MSG,nmodules);

    /* Create message queue */
    thread_ps->msgQId_ps = epicsMessageQueueCreate( maxMsgs,sizeof(CV_REQUEST));
    if ( thread_ps->msgQId_ps == NULL)
    {
        /* Fail to create messageQ */
        errlogSevPrintf(errlogFatal,CV_QCREATE_ERR_MSG);
        epicsThreadSuspendSelf();
    }
    else
    {
       /* Create thread to process messages from queue */      
       thread_ps->tid_ps = epicsThreadMustCreate("CV_OP",
						 epicsThreadPriorityLow,
                                                 stackSize,
                                                 (EPICSTHREADFUNC)CV_OpThread,
                                                 NULL );

       /* Create thread to send periodic (asyn) messages to the queue. */
       if (thread_ps->tid_ps)
       {  
          /* 
           * Perform initalization of camac crates before iocInit.
           * This MUST be done after the message queue has been created
           * so that the device init can be done.
           */
           CV_StartInit();

          /* 
	   * reate the event so that the asyn thread can be woken up  
	   * by the driver initialization (drvCV_Iinit) after epics
           * has been started.
	   */
           thread_ps = &threads_as[CV_ASYN_THREAD];
           thread_ps->evtId_ps = epicsEventMustCreate(epicsEventEmpty);
           thread_ps->tid_ps   = epicsThreadMustCreate("CV_ASYN",
                                                       epicsThreadPriorityLow,
                                                       stackSize,
                                                       (EPICSTHREADFUNC)CV_AsynThread,
                                                       NULL );       
           if (!thread_ps->tid_ps)
             errlogSevPrintf(errlogFatal,CV_THREADFAIL_MSG,"CV_ASYN");
       }
       else
         errlogSevPrintf(errlogFatal,CV_THREADFAIL_MSG,"CV_OP");
    }

    return(status);
}

/*====================================================
 
  Abs:  Initialize the Message Queue Id for each module
 
  Name: CV_StartInit
 
  Args: None
 
  Rem: This function is called from CV_Start, prior
       to iocInit, to initalize the  message queue
       id for each module in the linked  list.
 
  Side: None
 
  Ret:  None
 
=======================================================*/
static void CV_StartInit(void)
{ 
    CV_MODULE   *module_ps = NULL;     /* pointer to registered module   */
    vmsstat_t   iss = CRAT_OKOK;       /* local return status            */



     /* Initialize module in linked list with message queue id. */
     for ( module_ps = (CV_MODULE *)ellFirst(&moduleList_s);
	   module_ps; 
           module_ps =(CV_MODULE *)ellNext((ELLNODE *)module_ps) ) 
    {
       /* Initialize the message queue id */
       module_ps->msgQId_ps = threads_as[CV_OP_THREAD].msgQId_ps;
       if ( !module_ps->msgQId_ps )
          

       /* Dataway test */
       CV_ClrMsgStatus( &module_ps->mstat_as[CAMAC_TST_DATAWAY] );
       iss = CV_TestDataway(module_ps);

       /* Check crate online status */
       CV_ClrMsgStatus( &module_ps->mstat_as[CAMAC_RD_CRATE_STATUS] );
       iss = CV_IsCrateOnline(module_ps);

    }/* End of module FOR loop */

    return;
}


/*====================================================
 
  Abs:  Process message queue requests
 
  Name: CV_OpThread
 
  Args: None

  Rem:  The purpose of this thread is to process messages
        from the message Q and perform Camac transactions.

  Side: This thread process all Camac transactions for
        the crate verifier modules.
  
  Ret:  long
            OK - Successfully completed
        
            
=======================================================*/ 
static void CV_OpThread(void)
{
    int              msgQstat   = 0;                       /* status of message receive  */
    CV_REQUEST       msgRecv_s ;                           /* message received           */
    cv_thread_ts    *thread_ps  = &threads_as[CV_OP_THREAD];


   /*
    * Is a message queue available for crate verifier modules? 
    * If not, then exit and issue an error message 
    * to the log */
   if (thread_ps->msgQId_ps == NULL)
   {
      errlogSevPrintf(errlogInfo,CV_OPNOQ_MSG);
      return;
   }

   /* Indicate that this thread is active!*/
   thread_ps->active = true;
   errlogSevPrintf( errlogInfo,CV_THREADSTART_MSG,"CV_OP",thread_ps->tid_ps );

   /* 
    * Continuously process messages from queue. Exit only
    * if the stop flag indicates that we should.
    */
   while ( !thread_ps->stop )
   {
      /* Wait for a request message in queue */
      msgQstat = epicsMessageQueueReceive(thread_ps->msgQId_ps,&msgRecv_s,sizeof(CV_REQUEST));
      if (msgQstat<0)
      {
          /* We should never time out, so something wrong */
  	  errlogSevPrintf(errlogMajor,CV_QTMO_MSG,epicsThreadGetNameSelf(),msgQstat);
	  thread_ps->stop = true;
      } 
      else 
      {           
	  CV_ProcessMsg( &msgRecv_s );
      }
   } /* End of while statement */

   thread_ps->active = false;
   epicsMessageQueueDestroy( thread_ps->msgQId_ps );
   thread_ps->msgQId_ps = NULL;

   errlogSevPrintf( errlogInfo,CV_THREADEXIT_MSG,"CV_OP" );
   return;
}


/*=============================================================================

  Name: CV_AsynThread

  Abs:  The Crate Verifier Async thread 
        
  Args: None

  Rem: This thread sends periodic function request to the CV message queue,
       which is the only task to perform Camac operations.

  Side: None

  Ret:  None

==============================================================================*/
static void  CV_AsynThread(void)
{
  static const float      nsec         = 10.0;                  /* delay in seconds            */ 
  float                   max_interval = 60.0;                  /* maximum time interval       */
  float                   total        = 0;                     /* interval, 0=5sec 1=60sec    */
  bool                    first_pass   = true;                  /* first pass sending asyn msg */
  cv_thread_ts           *thread_ps    = &threads_as[CV_ASYN_THREAD];
 

  /* Indicate that this thread is active!*/
  thread_ps->active = true;
  errlogSevPrintf(errlogInfo,CV_THREADSTART_MSG,"CV_ASYN",thread_ps->tid_ps );
      

 /* Halt processing of thread until driver initialization has completed */
  errlogSevPrintf( errlogInfo,CV_WAITEVT_MSG,"CV_ASYN" );
  if ( thread_ps->evtId_ps )
     epicsEventMustWait(thread_ps->evtId_ps);

 /*
  * Event signal received from driver initalization (drvCV_Init) 
  * start sending periodic CV message requests to the queue.
  */
  errlogSevPrintf( errlogInfo,CV_GOEVT_MSG,"CV_ASYN" );


 /* start sendind periodic messages to the queue */
  errlogSevPrintf(errlogInfo,CV_ASYNSEND_MSG );
  while ( !thread_ps->stop && threads_as[CV_OP_THREAD].active && nmodules )
  {
     /* 
      * Submit all messages that take place every 10 seconds.
      * Note:  CV_OpThread() processes messages from the queue
      */
      for (total=0.0; total<max_interval; total+=nsec)
      {
         CV_SendMsgs( &asynMsgList_as[CV_10SEC] );
         /* 
	  * Process the asyn messages that cycle every 60 seconds
	  * on the first pass through the loop. This way we don't
	  * have to wait for a minute to pass before these records
	  * get processed in the database.
          */
         if (first_pass)
	 {
            CV_SendMsgs( &asynMsgList_as[CV_60SEC] );
	    first_pass = false;
         }

         /* wait for 5 seconds before sending next */
         epicsThreadSleep(nsec); 

      }/* End of interval */

      /* 
       * Submit all messages that take place every 60 seconds.
       * Note: CV_OpThread() processes messages from the queue
       */
       CV_SendMsgs( &asynMsgList_as[CV_60SEC] );
   
  }/* End of while statement */ 

  errlogSevPrintf( errlogInfo,CV_THREADEXIT_MSG,"CV_ASYN" );
  return;
}

/*=============================================================================

  Name: CV_AsyncThreadStop

  Abs:  Stop the Crate Verifier Asyn (message) thread  
        
  Args: None

  Rem: This function sets the stop flag of the asyn thread, which will
       cause the function CV_AsynThread() to exit.

  Side: None

  Ret:  None

==============================================================================*/
void  CV_AsynThreadStop(void)
{
  cv_thread_ts     *thread_ps = &threads_as[CV_ASYN_THREAD];
  thread_ps->stop = true;
  return;
}


/*=============================================================================

  Name: CV_SendMsgs

  Abs: Send list of messages to the queue
       
  Args: msgList_ps                Message list to cycle through
          Type: enum             
          Use:  ELLLIST * const   
          Acc:  read-only
          Mech: By value

  Rem: The purpose of this function is to submit all of the messages in the ilst
       provided, to the message queue.

  Side: This function is called by the asyn thread, CV_AsynThread()

  Ret:  None

==============================================================================*/
static void CV_SendMsgs( ELLLIST * const msgList_ps )
{  

   long                status    = OK;                      /* send msg return status      */
   unsigned long       i_msg     = 0;                       /* message counter             */
   unsigned long       nmsgs     = 0;                       /* # of messages in lined list */  
   cv_camac_func_te    func_e    = CAMAC_INVALID_OP;        /* function reuqest            */
   CV_REQUEST         *msg_ps    = NULL;                    /* message information         */
   CV_MODULE          *module_ps = NULL;                    /* module information          */
   CV_CAMAC_FUNC;
 
   
   nmsgs = ellCount( msgList_ps );
   for( i_msg=0, msg_ps = (CV_REQUEST *)ellFirst(msgList_ps);
        i_msg<nmsgs && msg_ps;
        i_msg++, msg_ps = (CV_REQUEST *)ellNext((ELLNODE *)msg_ps) )
   {
       module_ps = msg_ps->module_ps;
       if (module_ps->msgQId_ps && (CV_DRV_DEBUG!=2))
       {   
          func_e = msg_ps->func_e;
          CV_ClrMsgStatus( &module_ps->mstat_as[func_e] );  
          status = epicsMessageQueueTrySend(module_ps->msgQId_ps,msg_ps,sizeof(CV_REQUEST));
          if ((status==ERROR) && CV_DRV_DEBUG==3)
	      printf("CV_OP Message queue send error - %s func %s for CV[c=%hd n=%hd]\n",
                     msg_ps->source_c,
                     cv_camac_func_as[msg_ps->func_e].func_c,
                     module_ps->c,
                     module_ps->n );                
       }
   } /* End of FOR loop */

   return;
}

/*====================================================
 
  Abs:  Initialize all Crate Verifyer  module
 
  Name: drvCV_Init
  
  Args: None
 
  Rem: This function is called once during ioc
       initialization. The operations thread
       and the asyn thread are both started
       before iocInit(). At this poit there's
       nothing to do.
 
  Side: None
 
  Ret:  long
            OK  - Successful operation (always)
 
=======================================================*/
static long drvCV_Init(void)
{
    long           status    = OK;                          /* status return    */  
    cv_thread_ts  *thread_ps = &threads_as[CV_ASYN_THREAD]; /* Asyn thread info */

    if ( thread_ps->evtId_ps )
       epicsEventSignal( thread_ps->evtId_ps );
    return(status);
}

/*====================================================
 
  Abs:  Display data for all Crate Verifier Modules
 
  Name: drvCV_Report
 
  Args: level                        Level of info tobe
          Type: integer              displayed.
          Use:  int
          Acc:  read-only
          Mech: By value
 
  Rem:  The report routine is called by the function dbior,
        which is an ioc test utilities provided by EPICS.
        Thie function is responsible for producing a report 
        which is setnt to stdout, describing the Crate
        Verifier modules found at initialization.

        One input argument is passed, which describes the
        "level" of information to be displayed. The
         greater the "level", the more detail displayed.

        This report is output to the stdout
        when the following is entered from the
        target shell:
 
            dbior ("drvCV",<level>)

           
        Level  Report Informati Displayed
        -----  ---------------------------
         0     Driver version
         1     Additionally, module list listing branch, crate and slot
         2     Additionally, module id and data register with timestamp of last read.
         3     Additionally, crate voltages and temperatures

  Side: Report is sent to the standard output device
  
  Ret:  long
           OK  - Successful operation
            
=======================================================*/
static long drvCV_Report(int level)
{
    CV_VOLT_MULT;
    long                         status = OK;
    unsigned short               first  = 0;
    unsigned short               i      = 0;
    int                          qlevel = 0;
    statd_2_ts                  *statd_as   = NULL;
    CV_MODULE                   *module_ps  = NULL;
    campkg_dataway_ts           *dataway_ps = NULL;
    epicsMessageQueueId         msgQId_ps  = threads_as[CV_OP_THREAD].msgQId_ps;


    printf("\n"CV_DRV_VER_STRING"\n");
    for( module_ps = (CV_MODULE *)ellFirst(&moduleList_s);
         module_ps; 
	 module_ps = (CV_MODULE *)ellNext((ELLNODE *)module_ps))
   {
      switch( level )
      {
      case REPORT_BASIC:
            printf("\tCV Module[b%d c%d n%d]\n\n", module_ps->b, module_ps->c, module_ps->n );
        break;

      case REPORT_STANDARD:
           printf("\tCV Module[b%d c%d n%d]:\ttid=%ld data=0x%4.4lX    Crate: Pwr %s  %s\tFirst Watch (%s)\n",
	 	   module_ps->b,  module_ps->c, module_ps->n,
                   module_ps->id, module_ps->data,
                   (module_ps->crate_s.stat_u._i & CRATE_STATUS_ONLINE)?"On ":"Off",                  
                   (module_ps->crate_s.stat_u._i & CRATE_STATUS_INIT)?"Init Done  ":"Init Failed",   
		   (module_ps->crate_s.first_watch)?"Yes":"No");
           break;

      case REPORT_DETAILED:
           printf("\tCV Module[b%d c%d n%d]\n", module_ps->b, module_ps->c, module_ps->n);
           printf("\t\tid=%ld data=0x%4.4lX\n", module_ps->id, module_ps->data );
           printf("\t\tCrate: status=0x%4.4hx\tPwr %s\t%s\tInit Count=%d\n",
                 module_ps->crate_s.stat_u._i,
                 (module_ps->crate_s.stat_u._i & CRATE_STATUS_ONLINE)?"On":"Off", 
                 (module_ps->crate_s.stat_u._i & CRATE_STATUS_INIT)?"Init":"Not Init", 
                  module_ps->crate_s.nr_reinit );
           if (first)
           {
	      /* Print task IDs */
              printf("\tAsyn Thread:\t\tTask Id=%p\n",threads_as[CV_ASYN_THREAD].tid_ps);
              printf("\tOperational Thread:\tTask Id=%p\n",threads_as[CV_OP_THREAD].tid_ps);

	      /* Print the Message Queue IDs */
	      msgQId_ps = threads_as[CV_OP_THREAD].msgQId_ps;
              printf("\tMessage Queue (%p): \n\n",msgQId_ps);
   	      epicsMessageQueueShow(msgQId_ps,qlevel);
              first = 1;
	   }            
           break;

      case REPORT_VOLTAGE:
           printf("\tCV Module[b%d c%d n%d]\n", module_ps->b, module_ps->c, module_ps->n);
           printf("\t\tid=%ld\tdata=0x%lX\tstat=0x%4.4hX\t prev stat=0x%4.4hX\n", 
                   module_ps->id, module_ps->data,  
                   module_ps->crate_s.stat_u._i,
                   module_ps->crate_s.prev_stat_u._i);

           /* List Crate Voltages and Temperature */
           statd_as = module_ps->cam_s.rd_volts_s.statd_as;
           printf("\t\tCrate Voltages & Temperature: iss=%8.8lX\n",module_ps->mstat_as[CAMAC_RD_VOLTS].errCode);
           for (i=0; i<CV_NUM_ANLG_CHANNELS;i++) 
	     printf("\t\t%s\t%5.2f\t0x%4.4hX\tiss=%8.8X\n",
                    vmult_as[i].label_c,
                    module_ps->crate_s.volts_a[i],
                    statd_as[i].data,
                    statd_as[i].stat );
           break;
 
      default:
	    /* Command Line Test */
 	    dataway_ps = &module_ps->cam_s.dataway_s;
            printf("\tCV Module[b%d c%d n%d]\n", module_ps->b, module_ps->c, module_ps->n);
	    printf("\t\tCommand Line Test:  %s\n\t\t",
                  (dataway_ps->cmdLineErr)?"Failed":"Successful");
            for (i=0; i<CMD_LINE_NUM; i++)
	    {
	       printf("   0x%8.8lx",module_ps->cmdLine_s.data_a[i]);
               if (( i%4 )==0) printf("\n\t\t");
	    }
         
	    /* Read-Write test  */
            if (!dataway_ps->rwLineErr)
	      printf("\n\n\t\tRead Write Lines: Not Performed");
            else if (dataway_ps->rwLineErr!=BUS_RWLINE_PASSED)
	      printf("\n\n\t\tRead Write Lines: Failed    Test #%d\n\t\t",dataway_ps->rwLineErr);
	    else
	      printf("\n\n\t\tRead Write Lines: Successful\n\t\t");	     
            for (i=0; i<RW_LINE_NUM; i++)
	    {
	       printf("   0x%8.8lx",module_ps->rwLine_s.data_a[i]);
               if (( i%4 )==0) printf("\n\t\t");
	    } 
            printf("\n\n");
      	    break;

      }/* End of switch */	  
   }/* End of verifier module FOR loop */

   return(status);
}

/*====================================================
 
  Abs:  Find module in linked list and return pointer
 
  Name: CV_FindModuleByBCN
 
  Args: branch                       Camac Branch
          Type: integer              Note: ignored
          Use:  short
          Acc:  read-only
          Mech: By value

        crate                        Camac Crate Number
          Type: integer             
          Use:  short
          Acc:  read-only
          Mech: By value

        slot                         Camac Crate Slot Number
          Type: integer           
          Use:  short
          Acc:  read-only
          Mech: By value


  Rem:  The purpose of this function is to search the
        crate verifier linked list for the specified 
        module. If the module is located the pointer
        to the module infomation is returned. Otherwise,
        a NULL is returned.

  Side: None
  
  Ret:  CV_Module *
              NULL - If module not found
              Otherwise,  pointer to the module info in linked list
        
            
=======================================================*/ 
CV_MODULE * CV_FindModuleByBCN(short branch, short crate, short slot)
{
    CV_MODULE *module_ps = NULL;
    CV_MODULE *found_ps  = NULL;
 
    for( module_ps = (CV_MODULE *)ellFirst(&moduleList_s); 
         module_ps && !found_ps; 
         module_ps = (CV_MODULE *)ellNext((ELLNODE *)module_ps) )
    {
       if (module_ps->c == (crate & CAMAC_CRATE_MASK))
       {
          if (module_ps->n == (slot & CAMAC_SLOT_MASK)) 
            found_ps = module_ps;            
       }
    }/* End of FOR looop */

    if (!found_ps && CV_DRV_DEBUG)
      printf("CV Module was NOT found in b=%hd c=%hd n=%hd\n",branch,crate,slot); 
    else if (CV_DRV_DEBUG)
      printf("CV Module has been found in b=%hd c=%hd n=%hd\n",branch,crate,slot); 
    return( found_ps );
}   


/*====================================================
 
  Abs:  Process Camac requests
 
  Name: CV_ProcessMsg
 
  Args: msg_ps                    Message request received from queue
          Type: pointer             
          Use:  CV_REQUEST * const
          Acc:  read-write access
          Mech: By reference

  Rem:  The purpose of this function is to process a
         is to process messages
        from the message Q and perform Camac transactions.

  Side: This thread process all Camac transactions for
        the crate verifier modules.
  
  Ret:  long
            OK - Successfully completed
                    
=======================================================*/        
static void CV_ProcessMsg(  CV_REQUEST * const  msg_ps  )
{
    long                   status       = ERROR;      /* status return               */
    CV_MODULE             *module_ps    = NULL;       /* crate verifier module info  */
    dbCommon              *rec_ps       = NULL;       /* Record pointer              */
    cv_message_status_ts  *mstat_ps     = NULL;
    IOSCANPVT              volts_evt_p  = NULL;
    IOSCANPVT              id_evt_p     = NULL;
    IOSCANPVT              data_evt_p   = NULL;


    if (msg_ps==NULL)
    {   
        if (CV_DRV_DEBUG==2) printf("CV_ProcessMsg: Empty message received\n");
        return;   
    }

    if (CV_DRV_DEBUG==2) 
        printf("...Processing %s request message for camac func(%d)\n",msg_ps->source_c,msg_ps->func_e);

    /* Process Camac Request */ 
    module_ps = msg_ps->module_ps;                      /* ptr to module info    */
    mstat_ps  = msg_ps->mstat_ps;
    switch(msg_ps->func_e)
    { 
        /* 
	 * Check crate online status
	 * CAMC:<loca>:<crate>:STAT 
         */
        case CAMAC_RD_CRATE_STATUS:
	    status = CV_IsCrateOnline( module_ps );

            /* Process records waiting on an io scan event.*/
            if (mstat_ps && mstat_ps->evt_p) 
               scanIoRequest( mstat_ps->evt_p );

            data_evt_p = module_ps->mstat_as[CAMAC_RD_DATA].evt_p;
            if (data_evt_p) scanIoRequest( data_evt_p );
            break;

       /* 
	* Read module id register
	* MODU:<local>:<crate>slot>:ID 
        */
        case CAMAC_RD_ID:                               
            status = CV_ReadId( module_ps );

            /* Process records waiting on an io scan event.*/
            if (mstat_ps && mstat_ps->evt_p) 
               scanIoRequest( mstat_ps->evt_p );
            break; 
     
        /* 
	 * Read data register 
	 * MODU:<local>:<crate>slot>:DATA 
         */
        case CAMAC_RD_DATA:                     
	    status  = CV_ReadData( module_ps );

            /* Process records waiting on an io scan event.*/
            if (mstat_ps && mstat_ps->evt_p) 
               scanIoRequest( mstat_ps->evt_p );
            break;

        /* 
	 * Read crate voltage and temperatures 
         * CAMC:<loca>:<crate>:V24
         * CAMC:<loca>:<crate>:VMINUS24
         * CAMC:<loca>:<crate>:V6
         * CAMC:<loca>:<crate>:VMINUS6
         * CAMC:<loca>:<crate>:TEMP
         * CAMC:<loca>:<crate>:VGND
         */
        case CAMAC_RD_VOLTS:                    
            status = CV_ReadVoltage( module_ps );

             /* Process records waiting on an io scan event.*/
            if (mstat_ps && mstat_ps->evt_p) 
               scanIoRequest( mstat_ps->evt_p );
            break;

        /* 
	 * Perform Camac dataway checkout
	 * CAMC:<loca>:<crate>:BUSTAT
	 * CAMC:<loca>:<crate>:CMDLINE  
	 * CAMC:<loca>:<crate>:RWLINE
	 * CAMC:<loca>:<crate>:RWLINE_PATTERN
         */
        case CAMAC_TST_DATAWAY:                   
            status  = CV_TestDataway( module_ps );

            /* Process records waiting on an io scan event.*/
            if (mstat_ps && mstat_ps->evt_p) 
	    {
               scanIoRequest( mstat_ps->evt_p );
               volts_evt_p = module_ps->mstat_as[CAMAC_RD_VOLTS].evt_p;
               id_evt_p   =  module_ps->mstat_as[CAMAC_RD_ID].evt_p;
               if (volts_evt_p) scanIoRequest( volts_evt_p );
               if (id_evt_p)    scanIoRequest( id_evt_p ); 
	    }
  
           /*
	    * If this is an on-demand request, from a binary output record
	    * then make sure to callback the record to finish
	    * up with post-processing of the record.
	    */
            if ( msg_ps->rec_ps && strlen(msg_ps->source_c) && (strcmp("DSUP",msg_ps->source_c)==0))
            {
              rec_ps = msg_ps->rec_ps;
              dbScanLock(rec_ps);
              (*(rec_ps->rset->process))(rec_ps);
              dbScanUnlock(rec_ps);
            }
	    break;

        /*
	 * Set pattern in data register 
	 * MODU:<loca>:<crate><slot>:DATASETPT
         */
        case CAMAC_WT_DATA:
            status   = CV_WriteData( module_ps, module_ps->pattern );
            if ( msg_ps->rec_ps && strlen(msg_ps->source_c) && (strcmp("DSUP",msg_ps->source_c)==0))
            {
              rec_ps = msg_ps->rec_ps;
              dbScanLock(rec_ps);
              (*(rec_ps->rset->process))(rec_ps);
              dbScanUnlock(rec_ps);
            }
	    break;
	   
        /* Invalid function requested  */
        default:                 
	   status = ERROR;
           errlogSevPrintf(errlogMinor,CV_INVFUNC_MSG,msg_ps->func_e);
           break;

     } /* End of switch statement */

    return;
}

/*====================================================
 
  Abs:  Add a module to the linked list
 
  Name: CV_AddModule
 
  Args: branch                    Crate branch (not used)
          Type: value             Note: 0-3            
          Use:  short 
          Acc:  read-only
          Mech: By value

        crate                     Camac crate number
          Type: value             Note: 01-16          
          Use:  short 
          Acc:  read-only
          Mech: By value

        slot                      Camac slot number 
          Type: value             Note: 1-24           
          Use:  short 
          Acc:  read-only
          Mech: By value

  Rem:  The purpose of this function is to add the specified
        module to the linked list, first checking to see if
        is already in the list.

  Side: None

  Ret:  CV_MODULE
              address of module structure - Successfully completed
              NULL - Operation failed.
        
            
=======================================================*/ 
CV_MODULE * CV_AddModule( short branch, short crate, short slot )
{
    CV_ASYN_TYPES;
    unsigned short  i=1;               /* Index counter */
    CV_MODULE      *module_ps = NULL;  /* Pointer to module inforamtion structure  */

    /* 
     * Is this module already in the list?
     * If yes, return with the address.
     * If not, proceed with allocating the memory for
     * the structure.
     */
    module_ps = CV_FindModuleByBCN( branch, crate, slot );
    if (module_ps) return(module_ps);

    if(CV_DRV_DEBUG) 
       printf("Add Module CV[b=%d,c=%d,n=%d]\n",branch,crate,slot);

    /* 
     * This module is not in the list so allocate module information structure..
     */
    module_ps = callocMustSucceed(1,sizeof(CV_MODULE), "calloc buffer for CV_MODULE");
  
   /* Populate structure with basic info */
    module_ps->msgQId_ps = threads_as[CV_OP_THREAD].msgQId_ps;
    module_ps->b         = branch;                        /* SLAC system does not use branch */
    module_ps->c         = crate & CAMAC_CRATE_MASK;
    module_ps->n         = slot  & CAMAC_SLOT_MASK;
    module_ps->ctlw      = (crate << CCTLW__C_shc) | (slot << CCTLW__M_shc);
    module_ps->pattern   = CV_DATA_PATTERN;
    module_ps->present   = true;  
    module_ps->crate_s.mlock   = epicsMutexMustCreate();    /* used to lock flag_e and stat_u */
    module_ps->cmdLine_s.mlock = epicsMutexMustCreate();    /* cmdLine_s */
    module_ps->rwLine_s.mlock  = epicsMutexMustCreate();    /* rwLine_s  */
    module_ps->crate_s.first_watch = 1;

    if(CV_DRV_DEBUG) 
      printf("CV module present [c=%hd,n=%hd]\n",module_ps->c,module_ps->n);

    /*
     * Initialize locks so that both CV_AsynThread() and device support 
     * can write to cv_message_status_ts (i.e. mstat_s)
     */
    for (i=0; (i<MAX_CAMAC_FUNC); i++)
       module_ps->mstat_as[i].mlock = epicsMutexMustCreate();

    /* Now, add the new module to the linked list. */
    ellAdd(&moduleList_s, (ELLNODE *)module_ps);

    /* Add message to the proper processing list (ie. by interval) */
    for (i=0; i<CV_NUM_ASYN_FUNC; i++)
      CV_AddMsg(asynMsgs_as[i].func_e,asynMsgs_as[i].interval_e,"ASYN",module_ps);

    return(module_ps);
}


/*====================================================
 
  Abs:  Add a requeset message to the Asyn Message linked list

 Name: CV_AddMsg
 
  Args: func_e                    Camac request function
          Type: enum         
          Use:  cv_camac_func_te 
          Acc:  read-only
          Mech: By value

        interval_e                Interval to process asyn messages
          Type: enum         
          Use:  cv_interval_te 
          Acc:  read-only
          Mech: By value

        source_c                  Message source
          Type: ascii-string      Note: NULL terminated        
          Use:  char * const      TEST = Test functions
          Acc:  read-only         DSUP = device support
          Mech: By reference      ASYN = Asyn thread


        module_ps                 Module information       
          Type: pointer          
          Use:  CV_MODULE * const 
          Acc:  read-write
          Mech: By reference

  Rem:  The purpose of this function is to add a message for the
        specified request function to the Asynchronous message linked. 


  Side: None

  Ret:  None      
            
=======================================================*/ 
static void CV_AddMsg( cv_camac_func_te             func_e, 
                       cv_interval_te               interval_e,
                       char                 * const source_c,  
                       CV_MODULE            * const module_ps )
{
    CV_REQUEST   *msg_ps = NULL;  /* Pointer to message info used by the asyn thread */
    
    /* 
     * Initialize asyn message, which is used by CV_AsynThread() 
     * to send periodic camac function request to the queue.
     * Note that the messag Q pointer will be NULL if this function
     * has been called prior to CV_Start().
     *
     * Allocate memory for message block, used to send request to the queu
     * from the asyn task CV_AsynThread(). Device support will create their
     * own message block.
     */
     msg_ps = callocMustSucceed(1,sizeof(CV_REQUEST),"calloc buffer for async CV_MESSAGE");
     CV_DeviceInit( func_e,source_c,NULL,module_ps, msg_ps );
  
     /* Now, add the new module to the linked list.*/
     ellAdd(&asynMsgList_as[interval_e],(ELLNODE *)msg_ps);
  
     /* Print module location */
     if(CV_DRV_DEBUG) 
       printf("Add %s Message camac func(%d) for CV[b=%hd,c=%hd,n=%hd]\n",
               source_c,
               func_e,
               module_ps->b,
               module_ps->c,
               module_ps->n );
     return;
}

/*====================================================
 
  Abs:  Initialize Message contents
 
  Name: CV_DeviceInit
 
  Args: func_e                    Camac request function
          Type: enum         
          Use:  cv_camac_func_te 
          Acc:  read-only
          Mech: By value

        source_c                  Message source
          Type: ascii-string      Note: NULL terminated        
          Use:  char * const      TEST = Test functions
          Acc:  read-only         DSUP = device support
          Mech: By reference      ASYN = Asyn thread

        rec_ps                    Record Information
          Type: pointer           Note: Optional, NULL if not provided       
          Use:  dbCommon * const              
          Acc:  read-only       
          Mech: By reference 

        module_ps                 Module information       
          Type: pointer          
          Use:  CV_MODULE * const 
          Acc:  read-write
          Mech: By reference

        dpvt_ps                   Message 
          Type: pointer          
          Use:  CV_REQUEST * const 
          Acc:  read-write
          Mech: By reference


  Rem:  The purpose of this function is to initialize the
        message structure provided, based on the Camac request
        function.

  Side: None

  Ret:  long
            OK    - Operation successful
            ERROR - Operation failed. due to invalid function code
                  
=======================================================*/ 
long CV_DeviceInit( cv_camac_func_te         func_e, 
                    char       const * const source_c,
                    dbCommon         * const rec_ps,
                    CV_MODULE        * const module_ps,
                    CV_REQUEST       * const dpvt_ps   )
{
    long                  status  = OK;         /* return status              */
    campkg_dataway_ts    *cblk_ps = NULL;       /* ptr to dataway camac block */  
 
     /* 
      * Initalize the private device information structure, which 
      * is used to as the message sent to the queue 
      */
    dpvt_ps->module_ps = module_ps;
    dpvt_ps->rec_ps    = rec_ps;
    dpvt_ps->func_e    = func_e;
    sprintf(dpvt_ps->source_c,"%s",source_c);
    switch( func_e )
    {
        case CAMAC_RD_ID:
	   dpvt_ps->mstat_ps = &module_ps->mstat_as[func_e];
	   dpvt_ps->cam_p    = (void *)&module_ps->cam_s.rd_id_s;
           break;

        case CAMAC_RD_CRATE_STATUS:
	   dpvt_ps->mstat_ps = &module_ps->mstat_as[func_e];
           dpvt_ps->cam_p = (void *)&module_ps->cam_s.init_s;
           break;

        case CAMAC_RD_DATA:
           dpvt_ps->mstat_ps = &module_ps->mstat_as[func_e];
           dpvt_ps->cam_p    = (void *)&module_ps->cam_s.rd_data_s;
           break;

        case CAMAC_RD_VOLTS:
           dpvt_ps->mstat_ps = &module_ps->mstat_as[func_e];
           dpvt_ps->cam_p    = (void *)&module_ps->cam_s.rd_volts_s;
	   break;

        case CAMAC_WT_DATA:
           dpvt_ps->mstat_ps = &module_ps->mstat_as[func_e];
           dpvt_ps->cam_p    = (void *)&module_ps->cam_s.wt_data_s;
	   break;
   
        case CAMAC_RD_BUS_STATUS:  /* dataway verification status  */
        case CAMAC_TST_DATAWAY:    /* dataway verification test    */
        case CAMAC_TST_CMD:        /* command line test            */
        case CAMAC_TST_RW:         /* read write line test         */
        case CAMAC_TST_RW_PATTERN: /* read write line test pattern */
           dpvt_ps->mstat_ps = &module_ps->mstat_as[CAMAC_TST_DATAWAY];
	   cblk_ps           = &module_ps->cam_s.dataway_s;
           dpvt_ps->cam_p    = &cblk_ps->cmd_s;
           break;

         default:
           status = ERROR;
           dpvt_ps->mstat_ps = NULL;
           dpvt_ps->func_e   = CAMAC_INVALID_OP;
           break;

    }/* End of switch statement */

    return( status );
}


/*====================================================
 
  Abs:  Initlized Camac package to read the crate
        voltage and temperatures
 
  Name: CV_ReadVoltageInit
 
  Args: branch                    Crate branch (not used)
          Type: value             Note: 0-3            
          Use:  short 
          Acc:  read-only
          Mech: By value

        crate                     Camac crate number
          Type: value             Note: 01-16          
          Use:  short 
          Acc:  read-only
          Mech: By value

        slot                      Camac slot number 
          Type: value             Note: 1-24           
          Use:  short 
          Acc:  read-only
          Mech: By value

        cam_ps                    Camac block with package pointer 
          Type: struct            and status-data.        
          Use:  campkg_volts_ts * const
          Acc:  read-write access
          Mech: By reference


  Rem:  The purpose of this function is setup the Camac
        package to read the crate analog registers. There are
        thirteen registers F5An,where N=0-12, inclusive. A 
        Camac package with one packet is setup such that all
        subaddress 0-n, are read using the sequential subaddressing
        feature (SA) in the Camac control word. Please note that
        for the purposes of this application, only subaddress 0-7
        are used, and therefore subaddress 8-12 are not read.

        Subsequent calls to this fuction, with an already allocated
        package will resulte in the Camac package being reset and 
        then rebuilt using the module specification provided by
        the input arguments, crate and slot.

  Side: None
  
  Ret:  vmsstat_t
            CRAT_OKOK - Successfully completed
            Otherwise, see return codes from:
              camalo_reset()
              camalo()
              camadd()

=======================================================*/ 
static vmsstat_t CV_ReadVoltageInit( short branch, short crate, short slot, campkg_volts_ts * const cam_ps )
{
    vmsstat_t                    iss    = CRAT_OKOK;             /* return status         */
    vmsstat_t                    iss2   = CRAT_OKOK;             /* camdel return status  */
    static const unsigned short  emask  = 0xF000;                /* camac error mask      */
    unsigned long                subadr = 0; 
    unsigned int                 ctlw   = 0;                     /* Camac control word    */
    unsigned short               bcnt   = sizeof(short);         /* data byte count       */
    unsigned short               nops   = CV_NUM_ANLG_CHANNELS;  /* # of Camac operations */               

 
   /* 
    * Does CAMAC package exists? If so exit. Otherwise, allocate the 
    * package with a single packet to read the all analog voltages sequentially
    * in one packet, using the Camac scan subaddress option SA.
    */
    if (cam_ps->pkg_p) 
       return(iss);

    /* Allocate Camac package.*/
    iss = camalo(&nops,&cam_ps->pkg_p);
    if (SUCCESS(iss))
    {
      /* Add a Camac packet for each analog subaddress */
      for (subadr=0; (subadr<CV_NUM_ANLG_CHANNELS) && SUCCESS(iss); subadr++)
      {
        /* Build Camac control word */
        ctlw = (crate << CCTLW__C_shc) | (slot << CCTLW__M_shc) | F5 | subadr;

        /* Add a CAMAC packet */
        iss = camadd(&ctlw, &cam_ps->statd_as[subadr], &bcnt, &emask, &cam_ps->pkg_p);
        if (!SUCCESS(iss))
        {
          iss2 = camdel (&cam_ps->pkg_p);
          cam_ps->pkg_p = NULL;
        }
      }/* End of FOR loop */
    }

    return(iss);
}


/*====================================================
 
  Abs:  Initlized Camac package to read the id register
 
  Name: CV_ReadIdInit
 
  Args: branch                    Crate branch (not used)
          Type: value             Note: 0-3            
          Use:  short 
          Acc:  read-only
          Mech: By value

        crate                     Camac crate number
          Type: value             Note: 01-16          
          Use:  short 
          Acc:  read-only
          Mech: By value

        slot                      Camac slot number 
          Type: value             Note: 1-24           
          Use:  short 
          Acc:  read-only
          Mech: By value

        cam_ps                    Camac block with package pointer 
          Type: struct            and status-data.        
          Use:  campkg_ts * const
          Acc:  read-write access
          Mech: By reference


  Rem:  The purpose of this function is setup the Camac
        package to read the crate identification register. 

        The id  register is set from switches on the crate
        verifier module front panel. The id MUST match the id set on
        the Serial Crate Controller (SCC), which is located in slots
        24 5 25 within the same crate. The id of the SCC is set
        from a thumb wheel on the front panel of the module..

  Side: None
  
  Ret:  vmsstat_t
            CRAT_OKOK - Successfully completed
            Otherwise, see return codes from:
              camalo_reset()
              camalo()
              camadd()

=======================================================*/ 
static vmsstat_t   CV_ReadIdInit( short branch, short crate, short slot, campkg_ts * const cam_ps )
{
    vmsstat_t                    iss   = CRAT_OKOK;
    vmsstat_t                    iss2  = CRAT_OKOK;
    static const unsigned short  emask = 0xF000;                /* response X=0,Q=1 */
    unsigned int                 ctlw  = 0;
    unsigned short               bcnt  = sizeof(short);
    unsigned short               nops  = 1;


   /* 
    * Does CAMAC package exists? If so then reset the package. Otherwise, allocate the 
    * package with a single packet to read the module id register.
    */
    if ( cam_ps->pkg_p ) return(iss);

    /* Allocate CAMAC package */
    iss = camalo(&nops,&cam_ps->pkg_p);
    if (SUCCESS(iss))
    {
       /* build Camac control word to read id register */
       ctlw = (slot << CCTLW__M_shc) | (crate << CCTLW__C_shc) | CCTLW__F4  |  CCTLW__A3; 

       /* Add a CAMAC packet */
       if (!SUCCESS(iss = camadd(&ctlw, &cam_ps->statd_s, &bcnt, &emask, &cam_ps->pkg_p)))
       {
          iss2 = camdel (&cam_ps->pkg_p);
          cam_ps->pkg_p = NULL;
       }
    }
    return(iss);
}

/*====================================================
 
  Abs:  Initlized Camac package to read the data register
 
  Name: CV_ReadDataInit
 
  Args: branch                    Crate branch (not used)
          Type: value             Note: 0-3            
          Use:  short 
          Acc:  read-only
          Mech: By value

        crate                     Camac crate number
          Type: value             Note: 01-16          
          Use:  short 
          Acc:  read-only
          Mech: By value

        slot                      Camac slot number 
          Type: value             Note: 1-24           
          Use:  short 
          Acc:  read-only
          Mech: By value

        cam_ps                    Camac block with package pointer 
          Type: struct            and status-data.        
          Use:  campkg_data_ts * const
          Acc:  read-write access
          Mech: By reference


  Rem:  The purpose of this function is setup the Camac
        package with a double read of the data register.

  Side: None
  
  Ret:  vmsstat_t
            CRAT_OKOK - Successfully completed
            Otherwise, see return codes from:
              camalo_reset()
              camalo()
              camadd()

=======================================================*/ 
static vmsstat_t   CV_ReadDataInit( short branch, short crate, short slot, campkg_data_ts * const cam_ps )
{
    vmsstat_t                    iss    = CRAT_OKOK;
    vmsstat_t                    iss2   = CRAT_OKOK;
    static const unsigned short  emask  = 0xF000;      /*  CAMAC_EMASK_NOX_NOQ_NOCTO; */
    unsigned int                 ctlw   = 0;
    unsigned short               bcnt   = sizeof(long);
    unsigned short               nops   = 2;


   /* 
    * Does CAMAC package exists? If, so then we're done.Otherwise, allocate the 
    * package with a two packets, reading the data register twice in succession.
    * Both reads are required to match the expected pattern for a successful outcome.
    */
    if ( cam_ps->pkg_p ) return(iss);

    /* Allocate CAMAC package */
    iss = camalo(&nops,&cam_ps->pkg_p);
    if (SUCCESS(iss))
    {
       ctlw = (crate << CCTLW__C_shc)| (slot << CCTLW__M_shc) | (F4A0 | CCTLW__P24); 

       /* Add a CAMAC packets */
       cam_ps->statd_as[0].data = 0;
       cam_ps->statd_as[1].data = 0;
       iss = camadd(&ctlw, &cam_ps->statd_as[0], &bcnt, &emask, &cam_ps->pkg_p); 
       if (SUCCESS(iss))  
          iss = camadd(&ctlw, &cam_ps->statd_as[1], &bcnt, &emask, &cam_ps->pkg_p);
       if (!SUCCESS(iss))
       {
          iss2  = camdel (&cam_ps->pkg_p);
          cam_ps->pkg_p = NULL;
       }
    }

    return(iss);
}

/*====================================================
 
  Abs:  Initlized Camac package to set the data register
 
  Name: CV_WriteDataInit
 
  Args: branch                    Crate branch (not used)
          Type: value             Note: 0-3            
          Use:  short 
          Acc:  read-only
          Mech: By value

        crate                     Camac crate number
          Type: value             Note: 01-16          
          Use:  short 
          Acc:  read-only
          Mech: By value

        slot                      Camac slot number 
          Type: value             Note: 1-24           
          Use:  short 
          Acc:  read-only
          Mech: By value

        cam_ps                    Camac block with package pointer 
          Type: struct            and status-data.        
          Use:  campkg_data_ts * const
          Acc:  read-write access
          Mech: By reference


  Rem:  The purpose of this function is setup the Camac
        package with two packets. The first packet is to 
        set the data register with a pattern, and the second
        packet is to read the data register, so tht the user
        can verify that the pattern set latched. 

        This pattern written to the crate's verifier module's data 
        register is used to indicate if the crate has not been
        powered off since the last time the register was read.  

  Side: None
  
  Ret:  vmsstat_t
            CRAT_OKOK - Successfully completed
            Otherwise, see return codes from:
              camalo_reset()
              camalo()
              camadd()

=======================================================*/ 
static vmsstat_t  CV_WriteDataInit( short branch, short crate, short slot, campkg_data_ts  * const cam_ps  )
{
    vmsstat_t         iss     = CRAT_OKOK;
    vmsstat_t         iss2    = CRAT_OKOK;
    unsigned short    emask   = CAMAC_EMASK_NOX_NOQ;
    unsigned int      wt_ctlw = 0;
    unsigned int      rd_ctlw = 0;
    unsigned short    bcnt    = sizeof(long);
    unsigned short    nops    = 2;


    /* 
    * Does CAMAC package exists? If so then we're done. Otherwise, allocate the 
    * package with a two packets, setting the data register and then reading
    * back the register to verify that the write was successful.
    */
    if ( cam_ps->pkg_p ) return(iss);

    /* Allocate CAMAC package */
    iss = camalo(&nops,&cam_ps->pkg_p);
    if (SUCCESS(iss))
    {
       wt_ctlw = (crate << CCTLW__C_shc)| (slot << CCTLW__M_shc) | (F20A0 | CCTLW__P24); 
       rd_ctlw = (crate << CCTLW__C_shc)| (slot << CCTLW__M_shc) | (F4A0  | CCTLW__P24); 

       /* Add a CAMAC packets */
       cam_ps->statd_as[0].data = 0;   /* Set point  */
       cam_ps->statd_as[1].data = 0;   /* Read back  */
       iss = camadd(&wt_ctlw, &cam_ps->statd_as[0], &bcnt, &emask, &cam_ps->pkg_p); 
       if (SUCCESS(iss)) 
          iss = camadd(&rd_ctlw, &cam_ps->statd_as[1], &bcnt, &emask, &cam_ps->pkg_p);
       if (!SUCCESS(iss)) 
          iss2 = camdel (&cam_ps->pkg_p);
    }
    return(iss);
}

/*====================================================
 
  Abs:  Initalize Camac Crate 
 
  Name: CV_CrateInit
 
  Args: module_ps               Module information
          Type: pointer            
          Use:  CV_MODULE * const
          Acc:  read-write access
          Mech: By reference

        pulseZ _e               Z-line flag
          Type: unsigned short  0=don't pulse Z-line
          Use:  epicsBoolean    1=pulse X-line
          Acc:  read-only access
          Mech: By value

  Rem:  The purpose of this function is to verify that
        a Camac crate is physically online and attempt
        to bring those that are found offline due to
        a crate power cycle or at boot, back online.

        The crate's verifier module's data register contains  
        the expected data (CV_DATA_PATTERN), indicating    
        that the crate has not been powered off since the 
        last time we looked.  For those crates that we    
        have marked offline, attempt to write (and read   
        back) the same data into the verifier module,  
        considering the crate back online if we succeed. 

        SLC serial crate controller recognizes:
          F26A8 to module 28  = pulse the crate "Z" line. 
          F26A10 to module 30 = enable overall LAM in crate. 
          F24A9 to module 30  = deassert the crate "inhibit" line.

        Crate verifier module recognizes:
          F20A0P24 = write 24-bit data register
          F4A0P24  = read  24-bit data register.

        When to pulse the Z-line:
        If crate power has been off (i.e., if double-read verifier data 
        is wrong in both readouts), then issue crate "Z" pulse and 
        indicate same in crate status longword in database.  Make sure
        crate "inhibit" level is turned off.  (Re)write correct data  
        into verifier module.
       
  Side: before calling CV_CrateInit the camac crates 
        ONLINE bit in the crate status has been cleared.
  
  Ret:  vmsstat_t
            CRAT_NOTVALID - if module not registered 
            CRAT_OKOK     - Successful operations
            Otherwise, see return codes from calling function
               CV_CrateInitInit()

=======================================================*/ 
static vmsstat_t  CV_CrateInit( CV_MODULE * const module_ps, bool  pulseZ_e  )
{
   vmsstat_t  iss = CRAT_OKOK;


   if (!module_ps || !module_ps->present) 
      iss = CRAT_NOTVALID;
   else
   {
      if (pulseZ_e) 
      {
         printf("Z-Line On\n");
         module_ps->crate_s.z_off = false;
      }
      else	
         module_ps->crate_s.z_off = true;

      /* Initialize crate after crate power on or boot */
      iss = CV_CrateInitInit(module_ps);
       
   }
   return( iss );
}


/*====================================================
 
  Abs:  Camac crate initialization
 
  Name: CV_CrateInitInit
 
  Args: module_ps               Module information
          Type: pointer            
          Use:  CV_MODULE * const
          Acc:  read-write access
          Mech: By reference

  Rem:  The purpose of this function is initalize the crate
        after the power has been cycled from off to on.
        This was determined previously from a double-read of
        the verifier datra regsiter. If both  both readouts
        were found to be wrong the crate "Z-line" will be 
        pulsed. This will be indicated in the database pv

               CAMC:<area>:<crate>:STAT

       The crate "inhibit" level is turned off (dasserted), and a 
       a pattern is (re)written into the verifier data register.

  Side: None
  
  Ret:  vmsstat_t 
           CRAT_INITSUCC
           CRAT_INITFAIL   
           CRAT_CANTINIT - Failure due to Camac error
           Otherwise, see return codes from the functions:
              camalo_reset()
              camalo()
              camadd()
              camgo()

=======================================================*/ 
static vmsstat_t CV_CrateInitInit( CV_MODULE * const module_ps )
{
    vmsstat_t          iss    = CRAT_OKOK;
    unsigned int       ctlw   = 0;
    unsigned short     bcnt   = 0;
    unsigned short     nops   = 3;
    unsigned long      data   = 0;
    unsigned short     emask  = CAMAC_EMASK_NBAR;
    unsigned short     mask   = CRATE_STATUS_ONINIT | CRATE_STATUS_VPWROFF;
    unsigned short     crate_stat = 0;
    campkg_init_ts    *cam_ps = &module_ps->cam_s.init_s;

   
    if ( !cam_ps->pkg_p )
       iss = camalo( &nops, &cam_ps->pkg_p );
    else 
       iss = camalo_reset( &cam_ps->pkg_p );
    if (SUCCESS(iss))
    {

      /*
       * When a crate is first powered on the Serial Crate Controller (SCC) is
       * in an unaddressed state, such that I=0 and L is disabled, which is the
       * same as after a Z command. It is recommended that a Z operation be
       * performed to clear the modules in the crate after power has been
       * turned on. (see: SLC Camac Hardware Manual Chapter 46.5)
       */      
       if (!module_ps->crate_s.z_off)
       {
	  cam_ps->pulseZ_stat = 0;
          ctlw  = (module_ps->c << CCTLW__C_shc) | M28 | F26A8;
          iss   = camadd(&ctlw, &cam_ps->pulseZ_stat, &bcnt, &emask, &cam_ps->pkg_p);
       }
       if (SUCCESS(iss))
       {
	  cam_ps->inhibit_stat = 0;
	  ctlw  = (module_ps->c << CCTLW__C_shc) | M30 | F24A9;
          iss   = camadd(&ctlw, &cam_ps->inhibit_stat, &bcnt, &emask, &cam_ps->pkg_p);
	  if (SUCCESS(iss))
	  {
            cam_ps->wt_statd_s.stat = 0;
            cam_ps->wt_statd_s.data = module_ps->pattern;
            bcnt  = sizeof(long);
            emask = CAMAC_EMASK_XQ;
            ctlw  = (module_ps->c << CCTLW__C_shc) | (module_ps->n << CCTLW__M_shc) | (F20A0 | CCTLW__P24);
            iss   = camadd(&ctlw, &cam_ps->wt_statd_s, &bcnt, &emask, &cam_ps->pkg_p);
            if (SUCCESS(iss))
	    {
               iss = camgo( &cam_ps->pkg_p );
               if (SUCCESS(iss))
	       {
                 /*
		  * Just to be sure, retry reading the verifier module. 
                  * If the verifier data is now correct, then mark the crate        
                  * initialized and log a message.         
                  */
                 iss  = camalo_reset( &cam_ps->pkg_p );
                 ctlw = (module_ps->c << CCTLW__C_shc) | (module_ps->n << CCTLW__M_shc) | (F4A0 | CCTLW__P24);
                 iss  = camadd(&ctlw, &cam_ps->rd_statd_s, &bcnt, &emask, &cam_ps->pkg_p);
                 iss  = camgo( &cam_ps->pkg_p );
                 data = cam_ps->rd_statd_s.data & CV_DATA_MASK;
                 if (SUCCESS(iss) && (data==module_ps->pattern))
		 {
                    iss = CRAT_INITSUCC;
		    crate_stat = CRATE_STATUS_ONINIT;
                    module_ps->crate_s.nr_reinit++;
                    /* 
		     * If the Z-line has been pulsed and we haven't just booted
                     * then issue a message.
		     */
                    if (!module_ps->crate_s.z_off && !module_ps->crate_s.first_watch)
		       errlogSevPrintf(errlogInfo,CRAT_ZISSUED_MSG,module_ps->c);
                    if (module_ps->crate_s.first_watch)
		       errlogSevPrintf(errlogInfo,CRAT_INITSUCC_MSG,module_ps->c);
		 }
                 /*
                  * Reading the data register does not match pattern written, so 
                  * the crate must still be offline. Issue an error
                  * message indicating that the crate initialization has failed
		  * if we are booting up. Otherwise, return an error status
		  * that we still can't initialize the crate.            
                  */
		 else
		 {   
                     iss = CRAT_INITFAIL;
                     if (module_ps->crate_s.first_watch)
	               errlogSevPrintf(errlogMajor,
                                       CRAT_INITFAIL_MSG, 
                                       module_ps->c,"rd",
                                       cam_ps->rd_statd_s.data,
                                       cam_ps->rd_statd_s.stat);
	         }
	       }/* end of camgo status check */
	       else
	       {
                 iss = CRAT_INITFAIL;
                 if (module_ps->crate_s.first_watch)
	            errlogSevPrintf(errlogMajor,
                                    CRAT_INITFAIL_MSG, 
                                    module_ps->c,"wt",
                                    cam_ps->wt_statd_s.data,
                                    cam_ps->wt_statd_s.stat);
	       } 
            }
            /* camadd failure */
	  }
       }/* End of init */
    }

    CV_SetCrateStatus( module_ps,crate_stat,mask );
    return(iss);
}


/*====================================================
 
  Abs:  Initlized Camac package to set the data register
 
  Name: CV_TestDatawayInit
 
  Args: branch                    Crate branch (not used)
          Type: value             Note: 0-3            
          Use:  short 
          Acc:  read-only
          Mech: By value

        crate                     Camac crate number
          Type: value             Note: 01-16          
          Use:  short 
          Acc:  read-only
          Mech: By value

        slot                      Camac slot number 
          Type: value             Note: 1-24           
          Use:  short 
          Acc:  read-only
          Mech: By value

        cam_ps                    Camac block with package pointer 
          Type: struct            and status-data.        
          Use:  campkg_dataway_ts * const
          Acc:  read-write access
          Mech: By reference


  Rem:  The purpose of this function is setup the Camac
        packages to perform the following tests on the Camac 
        bus lines.

        1) Command lines
        2) Read-Write lines
        3) Id number
        4) Voltages and temperature
        5) X-Q response of module
 
 
  Side: None
  
  Ret:  vmsstat_t
            CRAT_OKOK - Successfully completed
            Otherwise, see return codes from:
              camalo_reset()
              camalo()
              camadd()

=======================================================*/ 
static vmsstat_t   CV_TestDatawayInit( short branch, short crate, short slot , campkg_dataway_ts * const cam_ps )
{
    vmsstat_t      iss = CRAT_OKOK;
 

    /*
     * Setup the Camac package to clear the registers on 
     * the bus by pulsing the C-line 
     */
    iss = CV_CratePulseCInit(branch,crate,slot,&cam_ps->pulseC_s);

    /* 
     * Setup the Camac package to clear the crate inhibit
     */
    if (SUCCESS(iss))
      iss = CV_CrateClrInhibitInit(branch,crate,slot,&cam_ps->inhibit_s);

    /* 
     * Setup the Camac package to write to the Serial Crate Controller
     * to put it into addressing mode. Expect X=0.
     */
    if (SUCCESS(iss))
      iss = CV_CrateSCCInit(branch,crate,slot,&cam_ps->scc_s);

    /*
     * Setup the Camac package to test the command lines.
     * in order of:
     *     N,F1,F2,F4,F8,F16,A1,A2,A4,A8,C,Z,I
     */
    if (SUCCESS(iss))
        iss = CV_CrateCmdLineInit(branch,crate,slot,&cam_ps->cmd_s);

    /* 
     * Setup Camac package to test the
     *   read lines using the walking one
     *   read lines using the walking zero
     *   write lines using the walking one
     *   write lines using the walking zero
     */
    if (SUCCESS(iss))
       iss = CV_CrateRWLineInit(branch,crate,slot,cam_ps);

    if (SUCCESS(iss))
      cam_ps->init = true;
     
    return(iss);
}

 
/*====================================================
 
  Abs:  Build Camac package to Pulse the C-Line (C=0)
 
  Name: CV_CratePulseCInit
 
  Args: branch                    Crate branch (not used)
          Type: value             Note: 0-3            
          Use:  short 
          Acc:  read-only
          Mech: By value

        crate                     Camac crate number
          Type: value             Note: 01-16          
          Use:  short 
          Acc:  read-only
          Mech: By value

        slot                      Camac slot number 
          Type: value             Note: 1-24           
          Use:  short 
          Acc:  read-only
          Mech: By value

        cam_ps                    Camac block with package pointer 
          Type: struct            and status-data.        
          Use:  campkg_nodata_ts * const
          Acc:  read-write access
          Mech: By reference

  Rem:  The purpose of this function is setup the Camac
        packages to cycle the Crate C-line, which clears the registers
        on the bus.

        This function is called as part of the Dataway Test

        CAMAC Function Codes

        F0 - F7   READ COMMANDS - USING R LINES
        F8 - F15  CONTROL COMMANDS
        F16- F23  WRITE COMMANDS - USING W LINES
        F24- F31  CONTROL COMMANDS

        These commands and  terminology  are  in  accordance  with  the  CAMAC
        Standard, and primarily refer to line usage in the CAMAC Crate.

        Special Commands of the SCC

        N31        :  ALL MODULES
        N30 F0 A0-7:  READ L SIGNALS, I LINE, L ENABLE
        N30 F24 A9  :  SET I = 0; RESPONSE Q=0, X=0
        N30 F26 A9  :  SET I = 1; RESPONSE Q=0, X=0
        N30 F24 A10 :  DISABLE OVERALL L IN CRATE; RESPONSE Q=0, X=0
        N30 F26 A10 :  ENABLE OVERALL L IN CRATE; RESPONSE Q=0, X=0
        N28 F26 A9  :  RUN CYCLE WITH C=1, RESPONSE Q=0, X=0
        N28 F26 A8  :  RUN CYCLE WITH Z=1, SET I=0, DISABLE L

  Side: None

  Ret:  vmsstat_t
            CRAT_OKOK - Successfully completed
            Otherwise, see return codes from:
              camalo()
              camadd()

=======================================================*/ 
static vmsstat_t   CV_CratePulseCInit( short branch, short crate, short slot, campkg_nodata_ts * const cam_ps )
{
    vmsstat_t       iss = CRAT_OKOK;
    unsigned int    ctlw   = 0;
    unsigned short  bcnt   = 0;
    unsigned short  nops   = 1;
    unsigned short  emask  = CAMAC_EMASK_NOX_NOQ;

   /*
    * Setup the Camac package to clear the bus registers 
    * by pulsing the C-line.
    */
    if (!cam_ps->pkg_p) 
    {
      iss = camalo(&nops,&cam_ps->pkg_p);
      if (SUCCESS(iss))
      {  
         /* Clear registers by pulsing the C-line */
         ctlw = (crate << CCTLW__C_shc) | M28 | F26A9;
         iss  = camadd(&ctlw, &cam_ps->stat, &bcnt, &emask, &cam_ps->pkg_p);
         if (!SUCCESS(iss))
         {
            camdel( &cam_ps->pkg_p );
            cam_ps->pkg_p = NULL;
         }
      }
    }
   return(iss);
}

/*====================================================
 
  Abs:  Build Camac package to Clear the Crate Inhibit Line (I=0)
 
  Name: CV_CrateClrInhibitInit
 
  Args: branch                    Crate branch (not used)
          Type: value             Note: 0-3            
          Use:  short 
          Acc:  read-only
          Mech: By value

        crate                     Camac crate number
          Type: value             Note: 01-16          
          Use:  short 
          Acc:  read-only
          Mech: By value

        slot                      Camac slot number 
          Type: value             Note: 1-24           
          Use:  short 
          Acc:  read-only
          Mech: By value

        cam_ps                    Camac block with package pointer 
          Type: struct            and status-data.        
          Use:  campkg_nodata_ts * const
          Acc:  read-write access
          Mech: By reference

  Rem:  The purpose of this function is setup the Camac
        packages to clear the Inhibit Line on the Camac bus.
        This is done by writting to the the Serial Crate Controller (SCC)
        installed in slots 24 and 25.

        This function is called as part of the Dataway Test

        CAMAC Function Codes

        F0 - F7   READ COMMANDS - USING R LINES
        F8 - F15  CONTROL COMMANDS
        F16- F23  WRITE COMMANDS - USING W LINES
        F24- F31  CONTROL COMMANDS

        These commands and  terminology  are  in  accordance  with  the  CAMAC
        Standard, and primarily refer to line usage in the CAMAC Crate.

        Special Commands of the SCC

        N31        :  ALL MODULES
        N30 F0 A0-7:  READ L SIGNALS, I LINE, L ENABLE
        N30 F24 A9  :  SET I = 0; RESPONSE Q=0, X=0
        N30 F26 A9  :  SET I = 1; RESPONSE Q=0, X=0
        N30 F24 A10 :  DISABLE OVERALL L IN CRATE; RESPONSE Q=0, X=0
        N30 F26 A10 :  ENABLE OVERALL L IN CRATE; RESPONSE Q=0, X=0
        N28 F26 A9  :  RUN CYCLE WITH C=1, RESPONSE Q=0, X=0
        N28 F26 A8  :  RUN CYCLE WITH Z=1, SET I=0, DISABLE L

  Side: None

  Ret:  vmsstat_t
            CRAT_OKOK - Successfully completed
            Otherwise, see return codes from:
              camalo()
              camadd()

=======================================================*/
static vmsstat_t   CV_CrateClrInhibitInit( short branch, short crate, short slot , campkg_nodata_ts * const cam_ps )
{
    vmsstat_t       iss   = CRAT_OKOK;
    unsigned int    ctlw  = 0;
    unsigned short  bcnt  = 0;
    unsigned short  nops  = 1;
    unsigned short  emask = CAMAC_EMASK_NOX_NOQ;

    /* 
     * Setup the Camac package to clear the bus inhibit line
     */
    if (!cam_ps->pkg_p) 
    {
       iss = camalo(&nops,&cam_ps->pkg_p);
       if (SUCCESS(iss))
       {  
          ctlw = (crate << CCTLW__C_shc) | M30 | F24A9;
          iss  = camadd(&ctlw, &cam_ps->stat, &bcnt, &emask, &cam_ps->pkg_p);
          if (!SUCCESS(iss))
          {
             camdel( &cam_ps->pkg_p );
             cam_ps->pkg_p = NULL;
          }
       }
    }
    return(iss);
}


/*====================================================
 
  Abs:  Build a Camac Package issue a READ command using R-Lines on the bus.
 
  Name: CV_CrateSCCInit
 
  Args: branch                    Crate branch (not used)
          Type: value             Note: 0-3            
          Use:  short 
          Acc:  read-only
          Mech: By value

        crate                     Camac crate number
          Type: value             Note: 01-16          
          Use:  short 
          Acc:  read-only
          Mech: By value

        slot                      Camac slot number 
          Type: value             Note: 1-24           
          Use:  short 
          Acc:  read-only
          Mech: By value

        cam_ps                    Camac block with package pointer 
          Type: struct            and status-data.        
          Use:  campkg_4u_ts * const
          Acc:  read-write access
          Mech: By reference

  Rem:  The purpose of this function is setup the Camac
        packages to issue a READ Command using the R Lines on the bus.
        This is done by writting to the the Serial Crate Controller (SCC)
        installed in slots 24 and 25.

        This function is called as part of the Dataway Test
        to check for a bad crate address or an offline crate.

        A crate controller once addressed remains in this state until another
        one on the line is addressed using a CAMAC command. Control bit C 
        selects 16 or 24 bit mode.  The crate controller remains in the  
        selected mode until re-addressed. The crate controller can implement 
        only SINGLE  ADDRESS  block transfers. The driver must check Q and/or X 
        to deal with termination or a word counter must be used.  There is a defined 
        response after every operation except after a WRITE command. Write data may 
        follow immediately after the WRITE command. Strings of WRITE data
        indicate a write block transfer.

        Read Data - 16 Bits or 24 Bits -

                  10X |    |    |    |    |    |    |
                  ----|----|----|----|----|----|----|----
                  ABCQ|XLRR|RRRR|RRRR|RRRR|RRRR|RRRR|RR
                      |  12|3456|7891|1111|1111|1222|22
                      |    |    |   0|1234|5678|9012|34

        CAMAC Function Codes

        F0 - F7   READ COMMANDS - USING R LINES
        F8 - F15  CONTROL COMMANDS
        F16- F23  WRITE COMMANDS - USING W LINES
        F24- F31  CONTROL COMMANDS

        These commands and  terminology  are  in  accordance  with  the  CAMAC
        Standard, and primarily refer to line usage in the CAMAC Crate.

        Special Commands of the SCC

        N31        :  ALL MODULES
        N30 F0 A0-7:  READ L SIGNALS, I LINE, L ENABLE
        N30 F24 A9  :  SET I = 0; RESPONSE Q=0, X=0
        N30 F26 A9  :  SET I = 1; RESPONSE Q=0, X=0
        N30 F24 A10 :  DISABLE OVERALL L IN CRATE; RESPONSE Q=0, X=0
        N30 F26 A10 :  ENABLE OVERALL L IN CRATE; RESPONSE Q=0, X=0
        N28 F26 A9  :  RUN CYCLE WITH C=1, RESPONSE Q=0, X=0
        N28 F26 A8  :  RUN CYCLE WITH Z=1, SET I=0, DISABLE L

  Side: The Crate number runs from 0 to 15 as set on front panel thumbwheel
        switch. Module  number  runs from 1 to 23 in standard CAMAC fashion;
        N=31 addresses all modules in given crate.
  
        When power is first turned on, the SCC is in the unaddressed state
        I=0,  and  L  is Disabled, the same as after a Z command as indicated.
        It is recommended that a Z operation be performed to clear the modules
        in the crate after power turn on.

  Ret:  vmsstat_t
            CRAT_OKOK - Successfully completed
            Otherwise, see return codes from:
              camalo()
              camadd()

=======================================================*/
static vmsstat_t   CV_CrateSCCInit( short branch, short crate, short slot , campkg_4u_ts * const cam_ps )
{
    vmsstat_t       iss   = CRAT_OKOK;             /* status return                    */
    vmsstat_t       iss2  = CRAT_OKOK;             /* local status                     */
    unsigned int    ctlw  = 0;                     /* Camac control word               */
    unsigned short  bcnt  = sizeof(long);          /* Camac data byte count            */
    unsigned short  nops  = 1;                     /* Number of Camac operations (pkt) */
    unsigned short  emask = CAMAC_EMASK_NOX_NOQ;   /* Camac error mask, returning      */
                                                   /* error on X=0 or Q=0              */

    /*
     * Setup the Camac package to read the command line F0
     * to check for a crate timeout .
     */
    if (!cam_ps->pkg_p) 
    {
       iss = camalo(&nops,&cam_ps->pkg_p);
       if (SUCCESS(iss))
       {  
          ctlw  = (crate << CCTLW__C_shc) | M24 | CCTLW__P24;
          iss   = camadd(&ctlw, &cam_ps->statd_s, &bcnt, &emask, &cam_ps->pkg_p);
          if (!SUCCESS(iss))
	  {
            iss2 = camdel(&cam_ps->pkg_p);
	    cam_ps->pkg_p = NULL;
          }
       }
    }
    return(iss);
}


/*====================================================
 
  Abs:  Initlized Camac package for Command Lines Test
 
  Name: CV_CrateCmdLineInit
 
  Args: branch                    Crate branch (not used)
          Type: value             Note: 0-3            
          Use:  short 
          Acc:  read-only
          Mech: By value

        crate                     Camac crate number
          Type: value             Note: 01-16          
          Use:  short 
          Acc:  read-only
          Mech: By value

        slot                      Camac slot number 
          Type: value             Note: 1-24           
          Use:  short 
          Acc:  read-only
          Mech: By value

        cam_ps                    Camac block with package pointer 
          Type: struct            and status-data.        
          Use:  campkg_cmd_ts * const
          Acc:  read-write access
          Mech: By reference


  Rem:  The purpose of this function is setup the Camac
        packages to test the command lines on the Camac bus
        in the following order:

        N,F1,F2,F4,F8,F16,A1,A2,A4,A8,C,Z,I     
 
        Note that the Z and I lines are set by the unaddress 
        commands:

           C: N(28) F26 A9  (C=1, Q=0, X=0)
           Z: N(28) F26 A8  (Z=1, I=0, Disable L)
           I: N(30) F26 A9  (I=1, Q=0, X=0)
   
       Two Camac packages are setup, each package with seven packets.
       The first package issues the command line test for:   N,F1,F2,F4,F8,F16,A1
       The second package issues the command line test for:  A2,A4,A8,C,Z,I

       Before the camac package is issued, the write data buffers will
       need to be setup and the read data buffers must be cleared.
 
  Side: None
  
  Ret:  vmsstat_t
            CRAT_OKOK - Successfully completed
            Otherwise, see return codes from:
              camalo_reset()
              camalo()
              camadd()

=======================================================*/
static vmsstat_t   CV_CrateCmdLineInit( short                 branch,
                                        short                 crate, 
                                        short                 slot, 
                                        campkg_cmd_ts * const cam_ps )
{
    CMD_LINE_FUNC;
    vmsstat_t          iss     = CRAT_OKOK;            /* return status                          */
    unsigned int       ctlw    = 0;                    /* Camac control word                     */  
    unsigned int       rd_ctlw = 0;                    /* Camac control word                     */   
    unsigned short     bcnt    = sizeof(long);         /* byte count                             */
    unsigned short     nobcnt  = 0;                    /* zero byte count                        */
    unsigned short     nops    = CMD_LINE_NUM+1;       /* Number of Camac operations             */
    unsigned short     npkts   = 0;                    /* Camac packet counter                   */
    unsigned short     emask   = CAMAC_EMASK_NOX_NOQ;  /* Camac error mask                       */
    unsigned short     i       = 0;                    /* Index to write-read stat-data pkts     */
    unsigned short     ipkg    = 0;                    /* Camac package pointer index            */

   
    /*
     * Setup the Camac package to test the command lines.
     * in order of:
     *     N,F1,F2,F4,F8,F16,A1,A2,A4,A8,C,Z,I
     */
    if (!cam_ps->pkg_p[ipkg])
    {
       /* Allocate Camac package for command line test */
       iss = camalo(&nops,&cam_ps->pkg_p[ipkg]);  
    
       /* 
	*  READ Command using the R-Lines (SCC)
        *
	* Note: A crate controller once addressed remains in this state until another
        * one on the line is addressed using a CAMAC command. Control bit C selects 16 or 24 bit mode. 
        * The crate controller remains in the selected mode until re-addressed.
	*/
       ctlw = (crate << CCTLW__C_shc) | M24;
       iss = camadd(&ctlw, &cam_ps->statd_s, &bcnt, &emask, &cam_ps->pkg_p[ipkg]);

       /* Read the verifier COMMAND register */
       rd_ctlw = (crate << CCTLW__C_shc) | (slot<<CCTLW__M_shc) | F3A0 | CCTLW__P24;  
       iss     = camadd(&rd_ctlw, &cam_ps->rd_statd_as[0], &bcnt, &emask, &cam_ps->pkg_p[ipkg]);

       /* 
	* READ Command Lines using R-Line, followed by a read of the verifier COMMAND register
        * for lines:  F1,F2,F4,F8,F16,A1,A2 
        *
	* Note: Since we are using the R-Lines to check that the function and subaddress codes are 
	* working properly, be aware that if if lines other than the F-lines and A-lines fail
	* the problem is a R-Line issue, but since F-lines and A-lines overlap R-Lines, of one
	* of the overlapping lines (F1,F2,F4,F8,F16,A1,A2), you don't know if the failure is
	* a R-Line, F-line or a A-line.
        */
       for (npkts=2,i=1; (npkts<=nops-2) && SUCCESS(iss); i++,npkts+=2)
       {
           ctlw = (crate << CCTLW__C_shc) | (slot << CCTLW__M_shc) | cmdLineFunc_a[i];
           iss  = camadd(&ctlw,    &cam_ps->stat_a[i],      &nobcnt, &emask, &cam_ps->pkg_p[ipkg]);
           iss  = camadd(&rd_ctlw, &cam_ps->rd_statd_as[i], &bcnt,   &emask, &cam_ps->pkg_p[ipkg]);
       }/* End of FOR loop */
    }
   
    if (SUCCESS(iss))
       iss = CV_CrateCmdLineInit2( branch,crate,slot,cam_ps );
    return(iss);
}



/*====================================================
 
  Abs:  Initlized Camac package #2 to test Camac Bus Command Lines
 
  Name: CV_CrateCmdLineInit2
 
  Args: branch                    Crate branch (not used)
          Type: value             Note: 0-3            
          Use:  short 
          Acc:  read-only
          Mech: By value

        crate                     Camac crate number
          Type: value             Note: 01-16          
          Use:  short 
          Acc:  read-only
          Mech: By value

        slot                      Camac slot number 
          Type: value             Note: 1-24           
          Use:  short 
          Acc:  read-only
          Mech: By value

        cam_ps                    Camac block with package pointer 
          Type: struct            and status-data.        
          Use:  campkg_cmd_ts * const
          Acc:  read-write access
          Mech: By reference


  Rem:  The purpose of this function is setup the second Camac
        package to test the command lines on the Camac bus
        in the following order:

        N,F1,F2,F4,F8,F16,A1,A2,A4,A8,C,Z,I      
 
        Note that the C,Z and I lines are set by the unaddress 
        commands:

           C: N(28) F26 A9
           Z: N(28) F26 A8
           I: N(30) F26 A9
   
       Two Camac packages are setup, each package with seven packets.
       The first package issues the command lines for:   N,F1,F2,F4,F8,F16,A1
       The second package issues the command lines for:  A2,A4,A8,C,Z,I

       Before the camac package is issued, the write data buffers will
       need to be setup and the read data buffers must be cleared.
 
  Side: None
  
  Ret:  vmsstat_t
            CRAT_OKOK - Successfully completed
            Otherwise, see return codes from:
              camalo_reset()
              camalo()
              camadd()

=======================================================*/
static vmsstat_t   CV_CrateCmdLineInit2( short branch, short crate, short slot , campkg_cmd_ts * const cam_ps )
{
    CMD_LINE_FUNC;
    vmsstat_t          iss     = CRAT_OKOK;            /* return status                          */
    unsigned int       ctlw    = 0;                    /* Camac control to write command register*/  
    unsigned int       rd_ctlw = 0;                    /* Camac control to read command register */                 
    unsigned short     bcnt    = sizeof(long);         /* byte count                             */
    unsigned short     nobcnt  = 0;                    /* zero byte count                        */
    unsigned short     nops    = CMD_LINE_NUM+1;       /* Number of Camac operations             */
    unsigned short     npkts   = 0;                    /* Camac packet counter                   */
    unsigned short     emask   = CAMAC_EMASK_NOX_NOQ;  /* Camac error mask                       */
    unsigned short     i       = 0;                    /* Index to write-read stat-data pkts     */
    unsigned short     ipkg    = 1;                    /* Camac package pointer index            */
    unsigned short     inhibit_cmd = 11;               /* inhibit command index (want to skip)   */

    if ( cam_ps->pkg_p[ipkg]) 
       return(iss);

    /* Allocate the Camac package to test the remainder of the bus command lines.*/
    iss = camalo(&nops,&cam_ps->pkg_p[ipkg]); 
    if (SUCCESS(iss))
    {  
       /* Build  the Camac control word to read the verifier COMMAND register */
       rd_ctlw = (crate << CCTLW__C_shc) | (slot << CCTLW__M_shc) | F3A0 | CCTLW__P24;  

       /* 
	* READ Command Lines using R-Line, followed by a read of the verifier COMMAND register
        * for lines:  A2,A4,A8,C,Z,I
        */
       for (i=7; (i<CMD_LINE_NUM) && SUCCESS(iss); i++,npkts+=2)
       {
         if ( i!=inhibit_cmd ) 
	 {
            ctlw = (crate << CCTLW__C_shc) | cmdLineFunc_a[i];

          /*
           * Are we testing bus lines C,Z or I? If not, then add the verifier module slot
           * to the control word so we can specify the R-Line before reading the verifier 
           * COMMAND register
           */
           if ((cmdLineFunc_a[i] & CCTLW__M)==0)  
              ctlw |= (slot << CCTLW__M_shc); 
           iss = camadd(&ctlw,    &cam_ps->stat_a[i],      &nobcnt, &emask, &cam_ps->pkg_p[ipkg]);
           iss = camadd(&rd_ctlw, &cam_ps->rd_statd_as[i], &bcnt,   &emask, &cam_ps->pkg_p[ipkg]);
	 }
       }/* End of FOR loop */
      
       /* Clear the inhibit line (ie. I=0). Expect response X=0 and Q=0 */
       ctlw = (crate << CCTLW__C_shc) | F24A9 | M30;
       iss  = camadd(&ctlw, &cam_ps->inhibit_stat, &nobcnt, &emask, &cam_ps->pkg_p[ipkg]);      
 
       /* Read the verifier COMMAND register, to setup for later */
       iss  = camadd(&rd_ctlw, &cam_ps->rd_statd_as[i], &bcnt, &emask, &cam_ps->pkg_p[ipkg]);
    }

    return(iss);
}


/*====================================================
 
  Abs:  Initlized Camac package for Camac Bus Read Line Test
 
  Name: CV_CrateRWLineInit
 
  Args: branch                    Crate branch (not used)
          Type: value             Note: 0-3            
          Use:  short 
          Acc:  read-only
          Mech: By value

        crate                     Camac crate number
          Type: value             Note: 01-16          
          Use:  short 
          Acc:  read-only
          Mech: By value

        slot                      Camac slot number 
          Type: value             Note: 1-24           
          Use:  short 
          Acc:  read-only
          Mech: By value

        cam_ps                    Camac block with package pointer 
          Type: struct            and status-data.        
          Use:  campkg_rw_ts * const
          Acc:  read-write access
          Mech: By reference


  Rem:  The purpose of this function is to setup the Camac
        package for the Camac bus read/write line test. 
        There are eight test in all, the sequence of which 
        is as follows:

           Perform the following with P24:
           1. Read line test using walking one bit
           2. Read line test using walking zero bit
           3. Write line test with simulated walking one bit
           4. Write line test with simulated walking zero bit

           Perform the following without P24:
           5. Read line test using walking one bit
           6. Read line test using walking zero bit

           Perform the following without P24 on write and with P24 on read:
           7. Write line test with simulated walking one bit
           8. Write line test with simulated walking zero bit

        Read Line:
        --------------
         Test #1,2,5,6
          The pattern will be rotate from R24 to R1 on the twenty-sixth read.
          The first read for the walking one state will be zero. The next read 
          will have a one in R1 and a zero's in R24-R2.

         Test #3,4,7 & 8
          The pattern will be rotate from R24 to R1 on the twenty-sixth read.
          The first read for the walking zero state will be all ones. The next read 
          will have a zero in R1 and a ones's in R24-R2. 
 
        Write Line Test
        ---------------
         Test #3,4,7& 8
           The pattern is simulated by writting to the DATA register and read back and verified.

  Side: None
  
  Ret:  vmsstat_t
            CRAT_OKOK - Successfully completed
            Otherwise, see return codes from:
              camalo_reset()
              camalo()
              camadd()

=======================================================*/
static vmsstat_t   CV_CrateRWLineInit( short branch, short crate, short slot , campkg_dataway_ts * const cam_ps )
{
    vmsstat_t                  iss      = CRAT_OKOK;  /* Camac status return                        */
    vmsstat_t                  iss2     = CRAT_OKOK;  /* Local Camac status return                  */
    unsigned int               clr_ctlw = 0;          /* Camac control word to clear the C-line     */
    unsigned int               wt_ctlw  = 0;          /* Camac control word set the ROTATE register */
    unsigned int               rd_ctlw  = 0;          /* Camac control word read the DATA register  */
    unsigned short             bcnt     = 0;          /* Camac data byte count                      */
    unsigned short             nobcnt   = 0;          /* Camac data byte count of zero              */
    unsigned short             nops     = 2;          /* Number of Camac operations (pkts)          */
    unsigned short             emask    = CAMAC_EMASK_NOX_NOQ; /* Camac error mask                  */
    campkg_rlines_walk1_4_ts  *test1_ps  = NULL;
    campkg_rlines_walk0_4_ts  *test2_ps  = NULL;
    campkg_wlines_4_ts        *wlines_ps = NULL;


    /*
     * Set some standard Camac control words used by the function,
     * The first is to clear the registers on the bus by pulsing the C-line,
     * second set the crate verifier ROTATE register for walking zero and last
     * read the crate verifier DATA register and rotate the pattern left.
     */
    clr_ctlw = (crate << CCTLW__C_shc) | M28 | F26A9;                                /* Run cycle with Z=1, Set I=0, Disable L        */
    wt_ctlw  = (crate << CCTLW__C_shc) | (slot << CCTLW__M_shc) | F20A3;             /* Set verifier ROTATE register for walking zero */
    rd_ctlw  = (crate << CCTLW__C_shc) | (slot << CCTLW__M_shc) | F4A1 | CCTLW__P24; /* Read ROTATE register and rotate pattern left  */

    /* Build the Camac package for test #1, walking one bit with p24. */
    if ( !cam_ps->rwlines_s.test1_s.pkg_p ) 
    {
      /* Allocate Camac packets */
      test1_ps = &cam_ps->rwlines_s.test1_s;
      iss = camalo(&nops,&test1_ps->pkg_p); 
      if (!SUCCESS(iss)) goto egress;

      /* Clear registers on the bus  */
      iss  = camadd(&clr_ctlw, &test1_ps->stat, &nobcnt, &emask, &test1_ps->pkg_p);
      if (SUCCESS(iss)) 
      {
         /* 
	  * Read ROTATE register and rotate pattern left.
	  * The pattern will rotate from R24 to R1 on the 26th read.
	  * the first read for walking one state will be zero. The next
	  * read will have a one in R1 and zeros in R24-R2.
	  * We expected to read 100 bytes of data.
          */
         bcnt = sizeof(test1_ps->rd_statd_s.data_a);
         iss  = camadd(&rd_ctlw, &test1_ps->rd_statd_s, &bcnt, &emask, &test1_ps->pkg_p); 
         if (!SUCCESS(iss))
         {
	    iss2 = camdel( &test1_ps->pkg_p);
            test1_ps->pkg_p = NULL;
            return(iss);
         }
      }
    }
    
    /* Build the Camac package for a read line test #2, walking zero bit with P24 */
    if ( !cam_ps->rwlines_s.test2_s.pkg_p ) 
    {
       /* Allocate Camac packets*/
        nops = 3;
        test2_ps = &cam_ps->rwlines_s.test2_s;
        iss = camalo(&nops,&test2_ps->pkg_p); 
        if (!SUCCESS(iss)) goto egress;
    
        /* Clear registers on the bus by pulsing the C-Line. */
        iss  = camadd(&clr_ctlw, &test2_ps->stat, &nobcnt, &emask, &test2_ps->pkg_p);

        /* Set ROTATE register for walking zero */
        if (SUCCESS(iss))
          iss = camadd(&wt_ctlw, &test2_ps->wt_stat, &nobcnt, &emask, &test2_ps->pkg_p); 
          
        /* Read DATA register and rotate pattern left.*/
        bcnt = sizeof(test2_ps->rd_statd_s.data_a);
        if (SUCCESS(iss))
          iss = camadd(&rd_ctlw, &test2_ps->rd_statd_s, &bcnt, &emask, &test2_ps->pkg_p); 
        if (!SUCCESS(iss))
	{
           iss2 = camdel(&test2_ps->pkg_p);
	   test2_ps->pkg_p = NULL;
           return(iss);
        }
    }

    /*
     * Build Camac package for test #3, write line 
     * test with simulated walking one/zero bit and p24
     */
    if ( !cam_ps->rwlines_s.test3_s.pkg_p )
    {
       /* Allocate Camac packets for write test */
        nops      = 2;
        wlines_ps = &cam_ps->rwlines_s.test3_s;
        iss       = camalo(&nops,&wlines_ps->pkg_p); 
        if (!SUCCESS(iss)) goto egress;

        /* Set the DATA register (P24) */
        bcnt = sizeof(wlines_ps->wt_statd_s.data);
        wt_ctlw = (crate << CCTLW__C_shc) |(slot << CCTLW__M_shc) | F20A0 | CCTLW__P24;
        iss  = camadd(&wt_ctlw, &wlines_ps->wt_statd_s, &bcnt, &emask, &wlines_ps->pkg_p); 

        /* Read the DATA register (P24) */
        bcnt = sizeof(wlines_ps->rd_statd_s.data);
        rd_ctlw  = (crate << CCTLW__C_shc) | (slot << CCTLW__M_shc) | F4A0 | CCTLW__P24;
        if (SUCCESS(iss))
           iss  = camadd(&rd_ctlw, &wlines_ps->rd_statd_s, &bcnt, &emask, &wlines_ps->pkg_p);
        if (!SUCCESS(iss))
        {
	   iss2 = camdel(&wlines_ps->pkg_p);
           wlines_ps->pkg_p = NULL;
           return(iss);
	}
    }

    if (SUCCESS(iss))
       iss = CV_CrateRWLineInit2(branch,crate,slot,cam_ps);

egress:
    return(iss);
}



/*====================================================
 
  Abs:  Initlized Camac package for Camac Bus Read Line Test
 
  Name: CV_CrateRWLineInit2
 
  Args: branch                    Crate branch (not used)
          Type: value             Note: 0-3            
          Use:  short 
          Acc:  read-only
          Mech: By value

        crate                     Camac crate number
          Type: value             Note: 01-16          
          Use:  short 
          Acc:  read-only
          Mech: By value

        slot                      Camac slot number 
          Type: value             Note: 1-24           
          Use:  short 
          Acc:  read-only
          Mech: By value

        cam_ps                    Camac block with package pointer 
          Type: struct            and status-data.        
          Use:  campkg_rw_ts * const
          Acc:  read-write access
          Mech: By reference


  Rem:  The purpose of this function is setup the Camac
        package for the read/write line test. The sequence 
        the test performed is as follows:

           Perform the following without P24:
           5. Read line test using perambulating bit
           6. Read line test using perambulating zero

           Perform the following without P24 on write and with P24 on read:
           7. Write line test with simulated walking one bit
           8. Write line test with simulated walking zero bit 
 
  Side: None
  
  Ret:  vmsstat_t
            CRAT_OKOK - Successfully completed
            Otherwise, see return codes from:
              camalo_reset()
              camalo()
              camadd()

=======================================================*/
static vmsstat_t   CV_CrateRWLineInit2( short branch, short crate, short slot , campkg_dataway_ts * const cam_ps )
{
    vmsstat_t                iss      = CRAT_OKOK;  /* Camac return status                        */
    vmsstat_t                iss2     = CRAT_OKOK;  /* Local Camac return status                  */
    unsigned int             clr_ctlw = 0;          /* Camac control word to clear the C-line     */
    unsigned int             wt_ctlw  = 0;          /* Camac control word set the ROTATE register */
    unsigned int             rd_ctlw  = 0;          /* Camac control word read the DATA register  */
    unsigned short           bcnt     = 0;          /* Camac data byte count                      */
    unsigned short           nobcnt   = 0;          /* Camac data byte count of zero              */
    unsigned short           nops     = 2;          /* Number of Camac operations (pkts)          */
    unsigned short           emask    = CAMAC_EMASK_NOX_NOQ;
    campkg_rlines_walk1_ts  *test5_ps = NULL;
    campkg_rlines_walk0_ts  *test6_ps = NULL;
    campkg_wlines_ts        *wlines_ps= NULL;

 
    /*
     * Set some standard Camac control words used by the function,
     * The first is to clear the registers on the bus by pulsing the C-line,
     * the second is to set the ROTATE register on the crate verifier and the 
     * last is to read the DATA register on the crate verifier module.
     */
    clr_ctlw = (crate << CCTLW__C_shc) | M28 | F26A9;
    wt_ctlw  = (crate << CCTLW__C_shc) | (slot << CCTLW__M_shc) | F20A3;
    rd_ctlw  = (crate << CCTLW__C_shc) | (slot << CCTLW__M_shc) | F4A1;
   
    /* Build the Camac package for test #5, walking one bit without p24. */
    if ( !cam_ps->rwlines_s.test5_s.pkg_p ) 
    {
      /* Allocate Camac packets */
      test5_ps = &cam_ps->rwlines_s.test5_s;
      iss = camalo(&nops,&test5_ps->pkg_p); 
      if (!SUCCESS(iss)) goto egress;

      /* Clear register on the bus  */
      iss  = camadd(&clr_ctlw, &test5_ps->stat, &nobcnt, &emask, &test5_ps->pkg_p);

     /* 
      * Read ROTATE register and rotate pattern left.
      * Read 34 bytes of data.
      */
      bcnt = sizeof(test5_ps->rd_statd_s.data_a);
      if (SUCCESS(iss))
         iss  = camadd(&rd_ctlw, &test5_ps->rd_statd_s, &bcnt, &emask, &test5_ps->pkg_p); 
      if (!SUCCESS(iss))
      {
 	  iss2 = camdel( &test5_ps->pkg_p);
          test5_ps->pkg_p = NULL;
          return(iss);
      }
    }
    
    /* Build the Camac package for test #6, walking zero bit without p24. */
    if ( !cam_ps->rwlines_s.test6_s.pkg_p ) 
    {
       /* Allocate Camac packets for setting the ROTATE register */
        nops = 3;
        test6_ps = &cam_ps->rwlines_s.test6_s;
        iss = camalo(&nops,&test6_ps->pkg_p); 
        if (!SUCCESS(iss)) goto egress;

        /* Clear register on the bus  */
        iss  = camadd(&clr_ctlw, &test6_ps->stat, &nobcnt, &emask, &test6_ps->pkg_p);

        /* Set the ROTATE register */
        if (SUCCESS(iss))
          iss  = camadd(&wt_ctlw, &test6_ps->wt_stat, &nobcnt, &emask, &test6_ps->pkg_p); 

        /* Read the DATA register. Read 18 words, 36 bytes of data */
        bcnt = sizeof(test6_ps->rd_statd_s.data_a);
        if (SUCCESS(iss))
          iss  = camadd(&rd_ctlw, &test6_ps->rd_statd_s, &bcnt, &emask,&test6_ps->pkg_p); 
        if (!SUCCESS(iss))
        {
	   iss2 = camdel( &test6_ps->pkg_p);
           test6_ps->pkg_p = NULL;
           return(iss);;
        }
    }

    /*
     * Build Camac package to issue a write and read of the DATA register.
     * This package will be used to test write lines.
     */
    if ( !cam_ps->rwlines_s.test7_s.pkg_p )
    {
       /* Allocate Camac packets for write test */
        nops      = 3;
        wlines_ps = &cam_ps->rwlines_s.test7_s;
        iss       = camalo(&nops,&wlines_ps->pkg_p); 
        if (!SUCCESS(iss)) goto egress;

        /* 
	 * Set the DATA register (P24) clearing out the old data from the high order bytes.
	 */
        bcnt    = sizeof(wlines_ps->clr_statd_s.data);
        wt_ctlw = (crate << CCTLW__C_shc) |(slot << CCTLW__M_shc) | F20A0 | CCTLW__P24;
        iss     = camadd(&wt_ctlw, &wlines_ps->clr_statd_s, &bcnt, &emask, &wlines_ps->pkg_p); 

        /* Set the DATA register */
        bcnt    = sizeof(wlines_ps->wt_statd_s.data);
        wt_ctlw = (crate << CCTLW__C_shc) |(slot << CCTLW__M_shc) | F20A0;
        iss     = camadd(&wt_ctlw, &wlines_ps->wt_statd_s, &bcnt, &emask, &wlines_ps->pkg_p); 

        /* Read the DATA register (P24) */
        bcnt = sizeof(wlines_ps->rd_statd_s.data);
        rd_ctlw  = (crate << CCTLW__C_shc) | (slot << CCTLW__M_shc) | F4A0 | CCTLW__P24;
        if (SUCCESS(iss)) 
           iss  = camadd(&rd_ctlw, &wlines_ps->rd_statd_s, &bcnt, &emask, &wlines_ps->pkg_p); 
        if (!SUCCESS(iss))
        {
	   iss2 = camdel( &wlines_ps->pkg_p);
           wlines_ps->pkg_p = NULL;
        }
    }

 egress:
    return(iss);
}


/*====================================================
 
  Abs:  Read Analog Voltage Registers
 
  Name: CV_ReadVoltage
 
  Args: module_ps               Module information
          Type: pointer            
          Use:  CV_MODULE * const
          Acc:  read-write access
          Mech: By reference

  Rem:  The purpose of this function is to read the
        analog voltage registers of the crate verifier,

  Side: Please note that subaddress 8-12 are not read,
        only 0-7
  
  Ret:  long 
            OK - Successful Operation
            ERROR - Operation failed,

            For detailed information on the failure see the
            the message error code status for this function:
            The SLC message codes from files cratdef.h and camdef.h
            are used:

            CRAT_OKOK      - Successful operation
	    CRAT_NOTVALID  - Failed operation, module not present
            Othewise, see error codes from the functions:
              camalo_reset()
              camalo()
              camadd()

=======================================================*/ 
static long  CV_ReadVoltage(CV_MODULE  * const module_ps )
{
    CV_VOLT_MULT;
    vmsstat_t           iss     = CRAT_NOTVALID;
    unsigned short      i       = 0;
    short               rval    = 0;
    float               slope   = CV_ANLG_SLOPE;
    float               zero    = CV_ANLG_ZERO;  
    campkg_volts_ts    *cam_ps  = NULL;

  
    if (!module_ps->present) 
       goto egress;

    /* Initalize Camac package status and data. */
    cam_ps = &module_ps->cam_s.rd_volts_s;
    memset(cam_ps->statd_as,0,sizeof(cam_ps->statd_as));
    memset(module_ps->crate_s.volts_a,0,sizeof(module_ps->crate_s.volts_a));

    /* Initialize the Camac package it it has not already been done */
    iss = CRAT_OKOK;
    if (!cam_ps->pkg_p)
       iss = CV_ReadVoltageInit( module_ps->b, module_ps->c, module_ps->n ,cam_ps );
   
    /* If a CAMAC package has been allocated...then issue the camac action. */
    if (SUCCESS(iss)) 
    {
       /* Read crate voltages, ground voltage and temperature */
       iss = camgo(&cam_ps->pkg_p); 
       if ( SUCCESS(iss))
       {
          for (i=0;(i<CV_NUM_ANLG_CHANNELS) && SUCCESS(iss);i++)
          {
             rval = cam_ps->statd_as[i].data & CV_ANLG_MASK;
             module_ps->crate_s.volts_a[i] = (slope * rval - zero)  * vmult_as[i].m1;
          }
          module_ps->crate_s.volts_a[A7] *= vmult_as[A7].m2;
       }
       else
          memset(module_ps->crate_s.volts_a,0,sizeof(module_ps->crate_s.volts_a));
    }

egress:
    /* Mark the message status complete. */
    CV_SetMsgStatus( iss,&module_ps->mstat_as[CAMAC_RD_VOLTS] );
    return( (SUCCESS(iss))?OK:ERROR );
}



/*====================================================
 
  Abs:  Read Identification Register
 
  Name: CV_ReadId
 
  Args: module_ps               Module information
          Type: pointer            
          Use:  CV_MODULE * const
          Acc:  read-write access
          Mech: By reference

  Rem:  The purpose of this function is to read the
        crate verifier id register. This register is
        set from switches on the front panel of the module.
        The switches should be set to match the Serial
        Crate Controller (SCC), which is a double-wide
        module installed in slot 24 and 25 of the same
        crate. The SCC is set from a thumb wheel on the
        front panel.

  Side: 
  
  Ret:  long 
            OK - Successful Operation
            ERROR - Operation failed,

            For detailed information on the failure see the
            the message error code status for this function:
            The SLC message codes from files cratdef.h and camdef.h
            are used:

            CRAT_OKOK      - Successful operation
	    CRAT_NOTVALID  - Failed operation, module not present
            Othewise, see error codes from the functions:
              CV_CamacIdInit()
              camgo()

=======================================================*/ 
static long  CV_ReadId(CV_MODULE * const module_ps )
{
    vmsstat_t      iss    = CRAT_NOTVALID;
    campkg_ts     *cam_ps = NULL;
    mbcd_pkg_ts   *campkg_ps = NULL;
    static const unsigned long  subadr = CCTLW__A3;
    static const unsigned long  func   = CCTLW__F4 >> CCTLW__F_shc;;
    long int     f=0,a=0;


    /* Does module exist? */
    if (!module_ps->present) 
      goto egress;

    module_ps->crate_s.idErr = true;  /* Set error as default */
 
    /* Initialize camac package first time through */
    iss    = CRAT_OKOK;
    cam_ps = &module_ps->cam_s.rd_id_s;
    if (!cam_ps->pkg_p)
       iss = CV_ReadIdInit( module_ps->b, module_ps->c, module_ps->n, cam_ps );

    /* If a CAMAC package has been allocated...then issue the camac action. */
    if (SUCCESS(iss))
    {
       cam_ps->statd_s.stat = 0;
       cam_ps->statd_s.data = 0;
       iss = camgo(&cam_ps->pkg_p);
       if (SUCCESS(iss))
       {
          module_ps->id = cam_ps->statd_s.data & CV_ID_MASK;
          if ( module_ps->c == module_ps->id ) 
            module_ps->crate_s.idErr = false;
       }
       else 
       {
          campkg_ps= module_ps->cam_s.rd_id_s.pkg_p;
          if (campkg_ps && campkg_ps->hdr.nops)
          {
             a = campkg_ps->mbcd_pkt[0].cctlw & CCTLW__A;
             f = (campkg_ps->mbcd_pkt[0].cctlw & CCTLW__F) >> CCTLW__F_shc;
          }
          module_ps->id = cam_ps->statd_s.data & CV_ID_MASK;
        
	  /* Check for a crate timeout */
          if ((iss==CAM_MBCD_NFG) || (iss==CAM_SOFT_TO) || (iss==CAM_CRATE_TO)) 
	  {
             module_ps->crate_s.stat_u._i |= CRATE_STATUS_CTO_ERR;
             module_ps->id = 0;

	     if (CV_DRV_DEBUG && (iss==CAM_SOFT_TO))       
               printf(CAM_SOFT_TO_MSG,(int)module_ps->c,(int)module_ps->n,a,f,cam_ps->statd_s.stat);
	     else if (CV_DRV_DEBUG && (iss==CAM_CRATE_TO)) 
               printf(CAM_CRATE_TO_MSG,module_ps->c,module_ps->n,a,f,cam_ps->statd_s.stat);
	     else if (CV_DRV_DEBUG && (iss==CAM_MBCD_NFG)) 
               printf(CAM_MBCD_NFG_MSG,(int)module_ps->c,(int)module_ps->n,a,f,cam_ps->statd_s.stat);
	  }

	  /* Check for Q=1. If we have a noq, log an error message. */
          else if ((cam_ps->statd_s.stat  & MBCD_STAT__Q)==0)
          {
             /* Set the NOQ error on in the bus status summary bitmask */
	     iss = CAM_NO_Q;
  	     module_ps->crate_s.bus_stat_u._i |= BUS_STATUS_NOQ_ERR;

             /* Issue a message to the log if this error did not occur last time */
	     if ( module_ps->mstat_as[CAMAC_RD_ID].lastErrCode!=iss )
	     {
	       errlogSevPrintf(errlogMinor,CAM_NO_Q_MSG,module_ps->c,module_ps->n,subadr,func,cam_ps->statd_s.stat);
	       if (CV_DRV_DEBUG) 
		 printf(CAM_NO_Q_MSG,module_ps->c,module_ps->n,a,f,cam_ps->statd_s.stat);
	     }
	  }else if (CV_DRV_DEBUG)
	    printf("CAMAC Error: Crate %.2d  N%.2d  A%ld  F%ld  status=0x%8.8X\n",(int)module_ps->c,(int)module_ps->n,a,f,cam_ps->statd_s.stat);
       }
    }

 egress:

    /* Mark the message status complete. */
    CV_SetMsgStatus(iss,&module_ps->mstat_as[CAMAC_RD_ID] );
    return( (SUCCESS(iss))?OK:ERROR );
}

/*====================================================
 
  Abs:  Read Data Register (double-read)
 
  Name: CV_ReadData
 
  Args: module_ps               Module information
          Type: pointer            
          Use:  CV_MODULE * const
          Acc:  read-write access
          Mech: By reference

  Rem:  The purpose of this function is to watch the
        status of the crate power by reading the data
        register and checking that a pattern written
        on the last watch is still in that register.
        If the data register does not match the pattern
        then it is assumed that the crate power has been
        cycled since it was last checked.

  Side: None
  
  Ret:  long 
            OK - Successful Operation
            ERROR - Operation failed,

=======================================================*/ 
static long  CV_ReadData(CV_MODULE * const module_ps)
{
    vmsstat_t           iss    = CRAT_NOTVALID;
    campkg_data_ts     *cam_ps = NULL;


    /* Does module exist? */  
    if (!module_ps->present) 
      goto egress;
 
    /*
     * Build the camack package to read the data register 
     * on the first pass...reuse the package on subsequent calls
     * to this function.
     */
    iss =  CRAT_OKOK;
    cam_ps = &module_ps->cam_s.rd_data_s;
    if (!cam_ps->pkg_p) 
      iss = CV_ReadDataInit( module_ps->b, module_ps->c, module_ps->n,cam_ps );
    if (SUCCESS(iss)) 
    {
       /*
	* Clear the Camac status-data in the packet, before
	* issuing the double-read of the DATA register
        */
       memset(cam_ps->statd_as,0,sizeof(cam_ps->statd_as));
       iss = camgo(&cam_ps->pkg_p);
         
       /* 
	* Check if the data register contains the expected
	* pattern, which indicates that the crate power had not been
	* cycled since our last check.
	*/
       iss = CV_CheckReadData( module_ps,iss );
       if (!SUCCESS(iss) && CV_DRV_DEBUG) printf("CAMAC Failed ReadData for Crate %.2d  N%.2d\n",module_ps->c,module_ps->n);
    }
    else
       module_ps->data = 0;
     
 egress:

    /* Mark the message status complete. */
    CV_SetMsgStatus(iss,&module_ps->mstat_as[CAMAC_RD_DATA]);
    return( (SUCCESS(iss))?OK:ERROR );
}

/*====================================================
 
  Abs:  Check the physical crate online status
 
  Name: isCrateOnline
 
  Args: crate                    Crate number
          Type: integer          Note: range 1-16            
          Use:  short   
          Acc:  read-only
          Mech: By value

        slot                     Slot number of crate 
          Type: value            verifier module.           
          Use:  short            (default=1)
          Acc:  read-only 
          Mech: By value


  Rem:  The purpose of this function is to return the
        crate physical online status.

  Side: None
  
  Ret:  long 
            ERROR - Invalid crate number or crate not found.
              -2  - Crate Verifier module not found
               0  - Crate Offline
               1  - Crate Online and not initialized
               2  - Illegal state
               3  - Crate Online and initalized

=======================================================*/
long isCrateOnline( short crate, short slot )
{
#define CV_NOMODU -2
  long               status = ERROR;  /* status return                 */
  static const short branch = 0;      /* branch, don't care            */
  short              n = 1;           /* default for crate verifier    */
  CV_MODULE         *module_ps = NULL;

  if (slot) n=slot;
  if ((crate>=MIN_CRATE_ADR) && (crate<=MAX_CRATE_ADR))
  {
     module_ps = CV_FindModuleByBCN( branch, crate, slot );
     if (module_ps) {
       status =  module_ps->crate_s.stat_u._i & CRATE_STATUS_GOOD;   
     }
     else
     { 
       printf(CV_NOMODU_MSG); 
       status = CV_NOMODU;
     }
  }  
  else
    printf("Camac Crate number %hd is invalid, (%hd-%hd)\n",crate,MIN_CRATE_ADR,MAX_CRATE_ADR);
  return(status);
}



/*====================================================
 
  Abs:  Check the physical crate online status
 
  Name: CV_IsCrateOnline
 
  Args: module_ps               Module information
          Type: pointer            
          Use:  CV_MODULE * const
          Acc:  read-write access
          Mech: By reference

  Rem:  The purpose of this function is to watch the
        physical online status of the crate power and
        to set the crate status flags. If the crate
        has been found to transition from an off to 
        and on status the crate is then initialized.
        and the status set appropriately.

  Side: None
  
  Ret:  long 
            OK - Successful Operation
            ERROR - Operation failed,

=======================================================*/ 
static long CV_IsCrateOnline( CV_MODULE * const module_ps )
{
    long                        status  = OK;
    vmsstat_t                   iss     = CRAT_NOTVALID;
    vmsstat_t                   rd_iss  = CRAT_OKOK;
    vmsstat_t                   wt_iss  = CRAT_OKOK;
    unsigned short              readErr = 0;
    static const unsigned short mask  = CRATE_STATUS_RDATA_ERR;
    static const unsigned short i_wt    = 0;
    static const unsigned short i_rbk   = 1;
    campkg_data_ts       *cam_ps  = NULL;


    /* Does module exist? */  
    if (!module_ps->present) 
      goto egress;

   /*
    * Save the previous crate status and clear current status 
    * saving only the last crate online and init status.
    */
    module_ps->crate_s.prev_stat_u._i = module_ps->crate_s.stat_u._i; 
    module_ps->crate_s.stat_u._i  &= ~CRATE_STATUS_VPWROFF;  /* Keep the crate off-to-on transition flag */

    /*
     * Build the camack package to read the data register 
     * on the first pass...reuse the package on subsequent calls
     * to this function.
     */
    iss = CRAT_OKOK;
    CV_ClrMsgStatus(&module_ps->mstat_as[CAMAC_RD_DATA]);
    status = CV_ReadData( module_ps );

    /*
     * If the crate is physically powered on, meaning that we didn't
     * receive an MBCD_CTO (ie. crate timeout), then check to see if
     * the crate was recently power cycled.
     */
    if (module_ps->crate_s.flag_e == CV_CRATEON)
    {
      /* Save the read data status */
       rd_iss = module_ps->mstat_as[CAMAC_RD_DATA].errCode;

      /*
       * At least we don't have crate timeout on both reads. Write
       * and read back the verifier module's data register, but
       * also retain what we just read.  See whether 
       * we can succeed once of at most 6 tries. 
       *
       * Please note, that even if the DATA register contains
       * the pattern we expect, on both reads, we still will
       * be writting the PATTERN to the DATA register and
       * verifying the readback.
       *
       * So don't mark the crate online yet...let's
       * see what happens later after we try to write
       * to the data register.  
       * 
       * Write a pattern to the data register 
       */
       CV_ClrMsgStatus(&module_ps->mstat_as[CAMAC_WT_DATA]);
       status = CV_WriteData( module_ps, module_ps->pattern ); 
       wt_iss = module_ps->mstat_as[CAMAC_WT_DATA].errCode;  
       if ( SUCCESS(wt_iss) && (module_ps->crate_s.flag_e==CV_CRATEON) )
       {
         /*
	  * The crate responded, so if we've just booted then initalize
	  * the crate with the Z-line pulsed. Otherwise, if both reads
	  * have failed, this indicates that the crate power has been
	  * cycled on since the last check, in which case initalize
	  * the crate but don't pulse the Z. 
	  */
         readErr = module_ps->crate_s.stat_u._i & mask;
	 if ( module_ps->crate_s.first_watch && readErr ) 
            iss = CV_CrateInit(module_ps,true);
	 else if (!module_ps->crate_s.first_watch && 
                  !(module_ps->crate_s.stat_u._i & CRATE_STATUS_INIT)  )        
	    iss = CV_CrateInit(module_ps,false); 
       }
       
      /*
       * Crate doesn't respond (maybe because it's powered off)
       * or verifier module is bad, and we are booting up, so 
       * complain.                                      
       */
       else if ((module_ps->crate_s.flag_e==CV_CRATEOFF) && module_ps->crate_s.first_watch)
       {
          iss = CRAT_CANTINIT;
          if ( module_ps->crate_s.first_watch )
             errlogSevPrintf(errlogMajor,CRAT_CANTINIT_MSG,module_ps->c); 
       }
      /*
       * Crate doesn't respond or verifier module is bad, and  
       * we are not booting up, and the crate is marked online,
       * so put it offline and complain.  Also notify timing   
       * subjob, which may set the 360-Hz interrupt handler's  
       * SKIP flag.                                            
       */
       else if ( !module_ps->crate_s.first_watch )
       {
         if (module_ps->crate_s.flag_e==CV_BADWREAD)
	 {
            iss = CRAT_OFFVER;
            cam_ps = &module_ps->cam_s.wt_data_s;
            /* 
	     * We don't want to flood the message log
	     * if the crate goes offline, so check that this
	     * crate wasn't off on the last check.
	     */
            if (!(module_ps->crate_s.prev_stat_u._i & CRATE_STATUS_WDATA_ERR))
	      errlogSevPrintf(errlogMajor,
                              CRAT_OFFVER_MSG,
                              module_ps->c,
                              cam_ps->statd_as[i_wt].data,
                              cam_ps->statd_as[i_rbk].data & CV_DATA_MASK,
                              cam_ps->statd_as[i_rbk].stat); 
	 }
         else
	 {
	    /* Looks like we chagned state from online to offline */
            iss = CRAT_CHNGSTAT;
            cam_ps = &module_ps->cam_s.wt_data_s;
            if (!(module_ps->crate_s.prev_stat_u._i & CRATE_STATUS_ONLINE))
	      errlogSevPrintf(errlogInfo,
                              CRAT_CHNGSTAT_MSG,
                              module_ps->c,
                              "ON",
                              cam_ps->statd_as[0].stat );
	 }
       }
    }
   /* 
    * Crate doesn't respond so it maybe powered off, a cable is loose,
    * the MBCD is bad or the verifier module is bad.
    */
    else 
       iss = CRAT_OFFLINE;	 

    /* If this is our first pass on boot, clear the first watch flag */
    if ( module_ps->crate_s.first_watch )
        module_ps->crate_s.first_watch = 0;
            
 egress:

    /* Mark the message status complete. */
    CV_SetMsgStatus(iss,&module_ps->mstat_as[CAMAC_RD_CRATE_STATUS]);
    return( (SUCCESS(iss))?OK:ERROR );
}

/*====================================================
 
  Abs:  Check Read Data Register
 
  Name: CV_CheckReadData
 
  Args: module_ps               Module information
          Type: pointer            
          Use:  CV_MODULE * const
          Acc:  read-write access
          Mech: By reference

        status                   Camac status from double-read status 
          Type: integer          of status register (ie. camgo return)
          Use:  vmsstat_t
          Acc:  read-only access
          Mech: By value

  Rem:  The purpose of this function is check the Camac status-data
        from a double-read of the data register to determine if the
        crate power has been physically offline since the last check.
        
        If the we don't get a crate timeout, meaning that the crate
        is online, and data register does not match the expected pattern,
        written on initialized or a crate power on, then it is assumed that the 
        crate power has been cycled.

        If a crate timeout has occured then the crate
        is considered offline. However, if 

          1) CRATE POWER IS ON AND THE CRATE HAS BEEN INITIALIZED:
            both reads of the data register produce the
            expected pattern, the crate is considered online
            and initialized. 

          2) CRATE POWER HAS TRANSITIONED FROM OFF TO ON:
             both reads If only one of the reads produces
             the pattern then.
          
          3) only one of the reads produces the expecte pattern.

  Side: CV_ReadData() is called prior to this function

  Ret:  vmsstat_t 
            CRAT_OKOK       - Successful operation
	    CRAT_OFFON      - Successful operation, crate power has transitioned from off to on 
            CRAT_OFFVER     - Failure, crate timeout or bad module, crate offline
            CRAT_VERDAT1    - Warning, only one read of data register produced expected pattern
            CRAT_CANTINIT   - Operation failed, unable to initalize crate, power off
            Othewise, see error codes from the functions:
              camgo()

=======================================================*/ 
static vmsstat_t CV_CheckReadData( CV_MODULE * const module_ps, vmsstat_t status )
{
    vmsstat_t             iss = status;
    unsigned short        i = 1;
    camstatd_tu           camstat1_u;        /* camac status  */
    camstatd_tu           camstat2_u;        /* camac status  */
    unsigned long         data1      = 0;    /* data read #1  */
    unsigned long         data2      = 0;    /* data read #2  */
    statd_4u_ts          *statd_as   = module_ps->cam_s.rd_data_s.statd_as;

     
    /*
     * Did we get a crate timeout? If yes, then the crate is
     * powered off, or the module is broken.
     */
     camstat1_u._i = statd_as[0].stat;   /* Get status read #1 return */
     camstat2_u._i = statd_as[1].stat;   /* Get status read #2 return */
 
     if ( SUCCESS(iss) )
     {   
         /*
	  * At least we don't have crate timeout or camac error on both reads.
	  * Now check that the data is what we expect.So set the creat online
	  * first and the clear the camac crate time-out and camac error bits
	  * in the status bitmask.
	  */
          epicsMutexMustLock(module_ps->crate_s.mlock );
          module_ps->crate_s.flag_e     = CV_CRATEON;
          module_ps->crate_s.stat_u._i |= CRATE_STATUS_ONLINE;
          module_ps->crate_s.stat_u._i &= ~(CRATE_STATUS_CTO_ERR | CRATE_STATUS_CAM_ERR); 
          epicsMutexUnlock(module_ps->crate_s.mlock );
    
         /*
          * Get verifier data from doubly-read of crate verifier register.
	  * If data doesn't matche expected pattern this indicates that the
          * crate has been powered off and initialization is needed.  
          */
          data1 = statd_as[0].data & CV_DATA_MASK;  /* Get 1st read from DATA register */
	  data2 = statd_as[1].data & CV_DATA_MASK;  /* Get 2nd read from DATA register */

         /* 
          * Yeah! So both reads were as expected...so the crate is happy, no power cycles
	  * occured since the last check. We can declare he crate online then.
	  * Note: the crate init happens in CV_IsCrateOnline()
	  */
          if ((data1==module_ps->pattern) && (data2==module_ps->pattern))
               module_ps->data = data1;
          else
	  {
	       /* Clear the crate init flag - something is wrong and set the 
		* crate power transition flag
                */
               epicsMutexMustLock(module_ps->crate_s.mlock );
	       module_ps->crate_s.stat_u._i &= ~CRATE_STATUS_INIT;  
               epicsMutexUnlock(module_ps->crate_s.mlock );
               if ((data1!=module_ps->pattern) && (data2!=module_ps->pattern))
	       {
		    module_ps->data = data1; /* save the date read from register */
                    epicsMutexMustLock(module_ps->crate_s.mlock );
                    module_ps->crate_s.stat_u._i |= CRATE_STATUS_RDATA_ERR;
                    epicsMutexUnlock(module_ps->crate_s.mlock );
                    /* 
		     * Issue a message to the log if this error didn't occur because we 
		     * have either just booted or the crate power was cycled since the last check,
                     * and this is a new error condition.		    
                     */
                    if ( !module_ps->crate_s.first_watch                           && 
                         (module_ps->crate_s.prev_stat_u._i & CRATE_STATUS_ONLINE) &&
			!(module_ps->crate_s.prev_stat_u._i & CRATE_STATUS_RDATA_ERR) )
	            	errlogSevPrintf( errlogMajor,
                                         CRAT_VERDAT_MSG,
                                         module_ps->c,
                                         module_ps->n,
                                         data1,
                                         camstat1_u._i,
                                         data2,
                                         camstat2_u._i );
	       }

               /* Ok, so did the first read have an unexpected pattern? */
	       else if ( data1!=module_ps->pattern )
	       {
	            /* 
	             * We have bad data, so save the data to the local module structure
	             * which will'll updated in the database. Log a message if we didn't
	             * see this error on our last check, unless we are booting.
	             */
	            module_ps->data = data1;
                    epicsMutexMustLock(module_ps->crate_s.mlock );
                    module_ps->crate_s.stat_u._i |= CRATE_STATUS_R1DATA_ERR;;
                    epicsMutexUnlock(module_ps->crate_s.mlock );
                    if ( !(module_ps->crate_s.prev_stat_u._i & CRATE_STATUS_R1DATA_ERR) )
                          errlogSevPrintf( errlogMajor,CRAT_VERDAT1_MSG,i,module_ps->c,module_ps->n,data1,camstat1_u._i );
	       }
              /* 
	       * The first read was good...how about the second read
	       * does it have the pattern we expect as well?
	       */
               else if ( data2!=module_ps->pattern )
	       {
	            /* 
	             * We have bad data, so save the data to the local module structure
	             * which will'll updated in the database. Log a message if we didn't
	             * see this error on our last check, unless we are booting.
	             */
                    i=2;
	            module_ps->data = data2;
                    epicsMutexMustLock(module_ps->crate_s.mlock );
                    module_ps->crate_s.stat_u._i |= CRATE_STATUS_R2DATA_ERR;;
                    epicsMutexUnlock(module_ps->crate_s.mlock );
                    if ( !(module_ps->crate_s.prev_stat_u._i & CRATE_STATUS_R2DATA_ERR) )
                         errlogSevPrintf( errlogMajor,CRAT_VERDAT1_MSG,i,module_ps->c,module_ps->n,data1,camstat2_u._i );
	       }
            
              /*
	       * If the crate is online but the data registers are not initalized with the expected data, and
               * the crate was offline during the last check, then log a message 
               * indicating that the crate has been powered from off to on.
	       * Please note, that we haven't initalized the crate yet. 
	       */
               if ( !(module_ps->crate_s.prev_stat_u._i & CRATE_STATUS_ONLINE) && !module_ps->crate_s.first_watch )
	       {
                  epicsMutexMustLock(module_ps->crate_s.mlock );
                  module_ps->crate_s.stat_u._i |= CRATE_STATUS_VPWROFF;
                  epicsMutexUnlock(module_ps->crate_s.mlock );
                  errlogSevPrintf(errlogInfo,CRAT_OFFON_MSG,module_ps->c,data1,camstat1_u._i );
	       }
	  }
     }
     else
     {
        /* 
         * OK, we've gotten a crate timeout so either the crate has been
         * powered off, or the verifier module is bad. Log a message
         * if we are booting or if the crate was online during our last
         * check.
         *
	 * Check for a crate timeout and flag the crate status bitmas accordingly.
         */
         epicsMutexMustLock( module_ps->crate_s.mlock );
         module_ps->crate_s.flag_e    = CV_CRATEOFF;
         if ((iss==CAM_MBCD_NFG) || (iss==CAM_SOFT_TO) || (iss==CAM_CRATE_TO)) 
	   module_ps->crate_s.stat_u._i |= CRATE_STATUS_CTO_ERR;
         else
	   module_ps->crate_s.stat_u._i |= CRATE_STATUS_CAM_ERR;
         epicsMutexUnlock( module_ps->crate_s.mlock );

         /* Issue a message to the log if we're booting or the crate was previously online */
         if ( module_ps->crate_s.first_watch )
	 {
           iss = CRAT_CANTINIT;
           errlogSevPrintf(errlogMajor,CRAT_CANTINIT_MSG,module_ps->c); 
	 }
         /* 
	  * The crate is not responding or the module is bad, and we
	  * aren't booting up, and the crate was previously marked online,
	  * so mark a crate timeout and issues and a message the a crate 
	  * timeout has occurred.
	  */
         else if ( module_ps->crate_s.prev_stat_u._i & CRATE_STATUS_ONLINE )
           iss = CRAT_OFFVER;
     }

     return(iss);
}

/*====================================================
 
  Abs:  Write patttern to data register
 
  Name: CV_WriteData
 
  Args: module_ps                Module information
          Type: pointer             
          Use:  CV_MODULE * const
          Acc:  read-write access
          Mech: By reference

        data                     Output data
          Type: integer            
          Use:  unsigned long
          Acc:  read-only access
          Mech: By value

  Rem:  The purpose of this function is to
        write the specified data to the data register
        and to verifiy that it has latched. A maximum
        of six tries are made to successfully set 
        the data register.

  Side: None

  Ret:  long
            OK    - Successfully completed
            ERROR - Operation failed
            
=======================================================*/ 
static long  CV_WriteData(CV_MODULE * const module_ps, unsigned long data )
{
    vmsstat_t                   iss      = CRAT_NOTVALID;
    unsigned short              i        = 0;
    unsigned long               rbk_data = 0;
    static const unsigned short max_trys = 6;
    static const unsigned short i_wt     = 0;
    static const unsigned short i_rbk    = 1;
    bool                        crateOn  = false;
    cv_crate_flag_te            flag_e   = CV_CRATEOFF;
    campkg_data_ts             *cam_ps   = NULL;;


    /* Does module exist? */    
    if (!module_ps->present)  
       goto egress;

    /* 
     * Build the camack package if it hasn't been done yet.
     * Reuse the package on subsequent calls.
     */
    iss    = CRAT_OKOK;
    cam_ps = &module_ps->cam_s.wt_data_s;
    if (!cam_ps->pkg_p) 
      iss = CV_WriteDataInit( module_ps->b, module_ps->c, module_ps->n,cam_ps );
    if (SUCCESS(iss)) 
    {
      /*  
       * Write and read back the  verifier module's data register,
       * Make up to 6 attempts to write the data successfully.
       */   
       cam_ps->statd_as[i_wt].data = data;       /* set the output data*/
       for (i=0; (i<max_trys) && !crateOn && SUCCESS(iss); i++)
       {
	 /*
	  * Attempt to set pattern in data register.
	  * clear the readback data first.
          */     
	  cam_ps->statd_as[i_wt].stat  = 0;  /* clear write operartion camac status    */    
	  cam_ps->statd_as[i_rbk].stat = 0;  /* clear read back operartion camac status */
	  cam_ps->statd_as[i_rbk].data = 0;  /* clear write operartion data             */
          iss = camgo(&cam_ps->pkg_p);
          if (SUCCESS(iss))
	  {
	     rbk_data = cam_ps->statd_as[i_rbk].data & CV_DATA_MASK;
             if (cam_ps->statd_as[i_wt].data==rbk_data)
	     {
	        crateOn = true;
                flag_e  = CV_CRATEON;
	     }
             else
                flag_e = CV_BADWREAD; 
	  }
          else
	     flag_e = CV_CRATEOFF;
       }/* End of FOR loop */

       epicsMutexMustLock( module_ps->crate_s.mlock );
       module_ps->crate_s.flag_e = flag_e;
       if (crateOn)
          module_ps->crate_s.stat_u._i |= CRATE_STATUS_ONLINE;
       else if (flag_e!=CV_BADWREAD)
       {
	 module_ps->crate_s.stat_u._i |=  CRATE_STATUS_CAM_ERR; /* set camac error */
         module_ps->crate_s.stat_u._i &= ~CRATE_STATUS_INIT;    /* clear init      */
       }
       else 
	  module_ps->crate_s.stat_u._i |= CRATE_STATUS_WDATA_ERR;
       epicsMutexUnlock( module_ps->crate_s.mlock ); 
    }

egress:

    /* Mark the message status complete. */
    CV_SetMsgStatus(iss,&module_ps->mstat_as[CAMAC_WT_DATA]);
    return( (SUCCESS(iss))?OK:ERROR );
}

/*====================================================
 
  Abs:  Perform Camac Crate Verification Test 
 
  Name: CV_TestDataway
 
  Args: module_ps                Module information
          Type: pointer             
          Use:  CV_MODULE * const
          Acc:  read-write access
          Mech: By reference


  Rem:  The purpose of this function is to perform
        the Camac crate verification test.

  Side: None

  Ret:  long
            OK    - Successfully completed
            ERROR - Opereation failed
           
=======================================================*/ 
static long  CV_TestDataway(CV_MODULE * const module_ps)
{ 
    vmsstat_t           iss     = CRAT_NOTVALID;      /* return status             */
    vmsstat_t           iss_wt  = CRAT_OKOK;          /* return status             */
    unsigned short      i_wt    = 0;                  /* index into wt stat-data   */
    long                status  = OK;                 /* local status              */
    unsigned int        stat    = 0;                  /* Camac status              */
    campkg_dataway_ts  *cam_ps = NULL;                /* pointer to camac info     */
    campkg_data_ts     *cam_data_ps = NULL;           /* write data register       */
 
    /* Does module exist? */
    if (!module_ps->present) 
       goto egress;
   
    iss = CRAT_OKOK;
    cam_ps = &module_ps->cam_s.dataway_s;

    /* 
     * Have all camac packages been build. If not, then do so. 
     * This should only need to be done on the first pass.
     */
    if ( !cam_ps->init )
      iss = CV_TestDatawayInit( module_ps->b, module_ps->c, module_ps->n ,cam_ps );

    /* If a CAMAC package has been allocated...then issue the camac action. */
    if (SUCCESS(iss)) 
    {
       /*
	* First, clear the data buffers for 
	* the tests that are to follow, which include the
	* Command Line, Read Line and Write Line tests
	*/
       CV_DatawayInitData(module_ps);
       
       /* Pulse the C-Line to clear the registers on the bus */
       iss = camgo(&cam_ps->pulseC_s.pkg_p);
       if (!SUCCESS(iss) && CV_DRV_DEBUG) 
	 printf("CV[%hd %hd %hd] Pulse C-Line failed, status=0x%8.8X\n",module_ps->b, module_ps->c, module_ps->n,cam_ps->pulseC_s.stat);

       /* Clear the Inhibit Line on the bus */
       iss = camgo(&cam_ps->inhibit_s.pkg_p);
       if (!SUCCESS(iss) && CV_DRV_DEBUG) 
	 printf("CV[%hd %hd %hd] Clear Inhibit Line failed, status=0x%8.8X\n",module_ps->b, module_ps->c, module_ps->n,cam_ps->inhibit_s.stat);

       /* Check for a bad crate address (id) or a crate offline. */
       iss = camgo(&cam_ps->scc_s.pkg_p);    
       if ((iss==CAM_MBCD_NFG) || (iss==CAM_SOFT_TO) || (iss==CAM_CRATE_TO)) 
       {
	  cam_ps->timeout = true;
          if (CV_DRV_DEBUG) 
	    printf("CV[%hd %hd %hd] Crate timeout, status=0x%8.8X\n",module_ps->b, module_ps->c, module_ps->n,cam_ps->scc_s.statd_s.stat);
          goto egress;
       }

       /* Check for the correct X-response, X=0.*/
       stat = cam_ps->scc_s.statd_s.stat;
       if ((stat & MBCD_STAT__X)==MBCD_STAT__X)
       {
	  cam_ps->Xstat_s.was_one = 1;  /* X-response returned "1" incorrectly */
          cam_ps->Xstat_s.err     = true;
          if (CV_DRV_DEBUG) 
	    printf("CV[%hd %hd %hd] CAMAC crate timeoutm status=0x%8.8X\n",module_ps->b, module_ps->c, module_ps->n,cam_ps->scc_s.statd_s.stat);
       }
       
       /* Perform Camac Bus Command Line Test */
       iss = CV_CrateCmdLine(module_ps);
       if (!SUCCESS(iss)) 
           goto egress;  

       /* Read the module Id register. Here we are checking for Q=1 */
       CV_ClrMsgStatus( &module_ps->mstat_as[CAMAC_RD_ID] );
       status = CV_ReadId(module_ps);
       if ( !SUCCESS(status) && (module_ps->mstat_as[CAMAC_RD_ID].errCode == CAM_NO_Q) )
       {
         cam_ps->Qstat_s.was_zero = 1;
         cam_ps->Qstat_s.err      = true;
       }
      
       /* Perform Camac Bus Read-Write Line Test */
       iss = CV_CrateRWLine(module_ps);

       /* Read crate voltages */
       CV_ClrMsgStatus( &module_ps->mstat_as[CAMAC_RD_VOLTS] );
       status = CV_ReadVoltage(module_ps);    
    }

egress:
    /* 
     * Leave test pattern in data register for periodic crate online status checking 
     * Don't bother checking the readback.
     */
    CV_ClrMsgStatus(&module_ps->mstat_as[CAMAC_WT_DATA]);
    cam_data_ps = &module_ps->cam_s.wt_data_s;
    if (!cam_data_ps->pkg_p) 
      iss_wt = CV_WriteDataInit( module_ps->b, module_ps->c, module_ps->n,cam_data_ps );
    if (SUCCESS(iss_wt)) 
    {
      /*  
       * Write and read back the  verifier module's data register,
       * Make up to 6 attempts to write the data successfully.
       */   
       cam_data_ps->statd_as[i_wt].data = module_ps->pattern & CV_DATA_MASK;       
       iss_wt = camgo(&cam_data_ps->pkg_p);
    }

    /* Set camac bus status summary bitmask (pv) */
    iss = CV_SetBusStatus( module_ps );  

    /* Mark the message status complete. */
    CV_SetMsgStatus(iss,&module_ps->mstat_as[CAMAC_TST_DATAWAY]);
    return( (SUCCESS(iss))?OK:ERROR );
}

/*====================================================
 
  Abs:  Set the Camac Crate Bus Status bitmask
 
  Name: CV_SetBusStatus
 
  Args: module_ps                Module information
          Type: pointer             
          Use:  CV_MODULE * const
          Acc:  read-write access
          Mech: By reference

  Rem:  The purpose of this function is to set the
        crate bus status bitmask based on the results
        of the dataway verification tests performed.

  Side: None

  Ret:  vmsstat_t
           CRAT_VERIFY_PASSES - Successful
           CRAT_VERIFY_FAIL   - Operation failed 
            
=======================================================*/ 
static vmsstat_t  CV_SetBusStatus( CV_MODULE * const module_ps )
{
    vmsstat_t           iss    = CRAT_VERIFY_PASSES;              /* return status             */
    campkg_dataway_ts  *cam_ps = &module_ps->cam_s.dataway_s;     /* ptr to dataway camac info */
    cv_bus_status_tu    stat_u;                                   /* bus status                */
    unsigned short      test = cam_ps->rwLineErr;

    stat_u._i = 0;
    stat_u._i = test << BUS_STATUS_RWERR_SHIFT;
    if ( RWLINE_ERR(test)    || 
         cam_ps->cmdLineErr  || 
         cam_ps->Xstat_s.err ||  cam_ps->Qstat_s.err || 
         cam_ps->timeout     || !cam_ps->init )
    {
        iss = CRAT_VERIFY_FAIL;
        if ( !cam_ps->init )  
          stat_u._i |= BUS_STATUS_INIT_ERR;
	else
	{
          if (cam_ps->cmdLineErr) stat_u._i |= BUS_STATUS_CMD_ERR;
          if (cam_ps->Xstat_s.was_one)  stat_u._i |= BUS_STATUS_X_ERR;
          if (cam_ps->Xstat_s.was_zero) stat_u._i |= BUS_STATUS_NOX_ERR;
          if (cam_ps->Qstat_s.was_one)  stat_u._i |= BUS_STATUS_Q_ERR;
          if (cam_ps->Qstat_s.was_zero) stat_u._i |= BUS_STATUS_NOQ_ERR;
          if (cam_ps->timeout) stat_u._i |= BUS_STATUS_CTO_ERR;
	}
    }
    module_ps->crate_s.bus_stat_u._i = stat_u._i & BUS_STATUS_MASK;
    return(iss); 
}


/*====================================================
 
  Abs:  Perform the Camac bus Command Line Test 
 
  Name: CV_CrateCmdLine
 
  Args: module_ps                Module information
          Type: pointer             
          Use:  CV_MODULE * const
          Acc:  read-write access
          Mech: By reference

  Rem:  The purpose of this function is to peform
        the Camac bus Comand Line test and verifiy that
        the lines were correct, and set and error flag
        in the module structure if a failure occurred.

  Side: None

  Ret:  vmsstat_t
           see return codes from
                camgo()
            
=======================================================*/ 
static vmsstat_t CV_CrateCmdLine( CV_MODULE * const module_ps )
{    CMD_LINE_OK;
    vmsstat_t                   iss    = CRAT_OKOK;     /* Return status                 */
    unsigned short              i      = 0;             /* index counter                 */
    unsigned long               data   = 0;             /* command line data             */
    unsigned int                stat   = 0;             /* camac status, for checking X  */
    unsigned long               lineNo = 0;             /* cmd line number               */
    static const unsigned long  mask   = CMD_LINE_MASK; /* data mask                     */
    static const unsigned short inhibit_cmd = 11;       /* inhibit command index         */
    campkg_dataway_ts          *cam_ps = NULL;          /* pointer to camac package info */
#define CMDL1_ERR_MSG "CV[%hd %hd %hd] failed Cmd Line Test #1\n"
#define CMDL2_ERR_MSG "CV[%hd %hd %hd] failed Cmd Line Test #2\n"


    /* Perform command line test */
    cam_ps = &module_ps->cam_s.dataway_s;
    iss = camgo(&cam_ps->cmd_s.pkg_p[0]);                    /* command line test pkg #1 */
    if (!SUCCESS(iss) && CV_DRV_DEBUG) 
      printf(CMDL1_ERR_MSG,module_ps->b,module_ps->c,module_ps->n);

    if (SUCCESS(iss)) iss = camgo(&cam_ps->cmd_s.pkg_p[1]);  /* command line test pkg #2 */
    if (!SUCCESS(iss) && CV_DRV_DEBUG) 
      printf(CMDL2_ERR_MSG,module_ps->b,module_ps->c,module_ps->n); 
    if (!SUCCESS(iss)) goto egress;

    /* Get read data from command line test */
    for (i=0,lineNo=1; i<CMD_LINE_NUM; i++,lineNo<<=1)
    {
        data = cam_ps->cmd_s.rd_statd_as[i].data & mask;
     
       /* 
	* Is the data correct? If not, then indicate a
        * command line error.
        */
       if (data!=cmdLineOk_a[i] && (i!=inhibit_cmd))
	 cam_ps->cmdLineErr = true;

       /* save the data */
       module_ps->cmdLine_s.data_a[i] = data;
    }/* End of FOR loop */

    /*
     * Next check the  X and Q response of the command line test.
     * Remember that the last Camac command issued was an 
     * F(3), which expects Q=0, X=1. Also, see if any
     * likely read line problems are causing spurious results, after
     * completing X and Q checks. This can be done by performing
     * a read of the id register.
     */
    stat = cam_ps->cmd_s.rd_statd_as[CMD_LINE_NUM].stat;
    if ((stat & MBCD_STAT__X)==0)
    {
       cam_ps->Xstat_s.was_zero = 1;
       cam_ps->Xstat_s.err = true;
    }
    /* Check Q-response */
    if (stat & MBCD_STAT__Q)
    {
       cam_ps->Qstat_s.was_one = 1;
       cam_ps->Qstat_s.err = true;
    }

egress:

    return(iss);
}


/*====================================================
 
  Abs:  Perform the Camac bus Read Write Line Test 
 
  Name: CV_CrateRWLine
 
  Args: module_ps                Module information
          Type: pointer             
          Use:  CV_MODULE * const
          Acc:  read-write access
          Mech: By reference

  Rem:  The purpose of this function is to peform
        the Camac bus Comand Line test and verifiy that
        the lines were correct, and set and error flag
        in the module structure if a failure occurred.

  Side: None

  Ret:  vmsstat_t
             CRAT_VERIFY_PASSES - Successful operation
             CRAT_VERIFY_FAIL   - Operation failed
            
=======================================================*/ 
static vmsstat_t CV_CrateRWLine( CV_MODULE * const module_ps )
{   
    RW_LINE_OK;
    vmsstat_t           iss        = CRAT_OKOK;      /* Return status                        */
    vmsstat_t           iss2       = CRAT_OKOK;      /* local return status                  */
    unsigned long       status     = OK;             /* local return status                  */
    unsigned short      i_bit      = 0;              /* index counter                        */
   
    unsigned short     *wt_sdata_p = NULL;           /* ptr to write data in write line test */ 
    unsigned long      *wt_data_p  = NULL;           /* ptr to write data in write line test */
    unsigned long      *rd_data_p  = NULL;           /* ptr to read data in write line test  */

    campkg_rwlines_ts  *cam_ps    = NULL;            /* pointer to camac package info        */
    campkg_dataway_ts *dataway_ps = NULL;            /* Dataway test camac packages          */
    unsigned short    *sdata_a    = NULL;            /* Pointer to 16-bit word array         */
    unsigned long     *data_a     = NULL;            /* Pointer to 32-bit word array         */
    unsigned long      stat       = 0;               /* Camac package status                 */    
    unsigned int       ctlw       = 0;               /* Camac control word                   */
    unsigned short     nobcnt     = 0;               /* Camac data byte count of zero        */
    unsigned short     emask      = CAMAC_EMASK_NOX_NOQ; /* Camac error mask, NOX and NOQ    */
    unsigned short     test       = 1;               /* Test number                          */
    unsigned int       nelem      = RW_LINE_NUM;     /* number of elements in block transfer */
    unsigned int       nbits      = RW_LINE_NUM;     /* number of bits for read-write test   */
    cv_rwLine_type_te  type_e     = WALKING_ONE;     /* type of bit test                     */
#define RWL1_ERR_MSG "CV[%hd %hd %hd] failed RW Line Test #1\n"

   /* 
    * Perform the Read Write  Line test #1 using walking one bit and P24
    */
    memset(module_ps->rwLine_s.data_a,0,sizeof(module_ps->rwLine_s.data_a));
    memset(module_ps->rwLine_s.err_a,0,sizeof(module_ps->rwLine_s.err_a));
    memset(module_ps->rwLine_s.expected_data_a,0,sizeof(module_ps->rwLine_s.expected_data_a));
    module_ps->rwLine_s.type_e = type_e;
    module_ps->rwLine_s.test   = test;

    dataway_ps = &module_ps->cam_s.dataway_s;
    cam_ps     = &dataway_ps->rwlines_s;
    iss        = camgo(&cam_ps->test1_s.pkg_p);
    if (!SUCCESS(iss)) 
    {
       if (CV_DRV_DEBUG) printf(CMDL2_ERR_MSG,module_ps->b,module_ps->c,module_ps->n); 
       goto egress;
    }

    data_a  = cam_ps->test1_s.rd_statd_s.data_a;
    for (i_bit=0; i_bit<nbits; i_bit++)
      module_ps->rwLine_s.data_a[i_bit] = data_a[i_bit];
    status = CV_RWDataGet(type_e, 
                          nbits, 
                          NULL, 
                          module_ps->rwLine_s.data_a,
                          module_ps->rwLine_s.err_a,
                          module_ps->rwLine_s.expected_data_a);
    if (CV_DRV_DEBUG && status)
    {
       printf("CV[%hd %hd %hd]\n",module_ps->b,module_ps->c,module_ps->n);
       printf("RW Line Test #%.2hd: %s\tstat=0x%8.8X  iss=0x%8.8lx\n",
               test,
              (status)?"Failed   ":"Successful",
              cam_ps->test1_s.rd_statd_s.stat,
              iss);
       for (i_bit=0; i_bit<nbits; i_bit++)
         printf("\t(%.2d):  data=0x%8.8lx  expected=0x%8.8lx\t%s\n",
               i_bit,
               module_ps->rwLine_s.data_a[i_bit],
               module_ps->rwLine_s.expected_data_a[i_bit],
              (module_ps->rwLine_s.err_a[i_bit])?"Error":"");
       printf("\n");
    }
    if (status || !SUCCESS(iss)) goto egress;

    /* 
     * Perform Read Write Line test #2 using walking zero bit and P24 
     */
    test++;
    type_e = WALKING_ZERO;
    memset(module_ps->rwLine_s.data_a,0,sizeof(module_ps->rwLine_s.data_a));
    memset(module_ps->rwLine_s.err_a,0,sizeof(module_ps->rwLine_s.err_a));
    memset(module_ps->rwLine_s.expected_data_a,0,sizeof(module_ps->rwLine_s.expected_data_a));
    module_ps->rwLine_s.type_e = type_e;
    module_ps->rwLine_s.test   = test;

    iss    = camgo(&cam_ps->test2_s.pkg_p);
 
    data_a = cam_ps->test2_s.rd_statd_s.data_a;
    for (i_bit=0; i_bit<nbits; i_bit++)
      module_ps->rwLine_s.data_a[i_bit] = data_a[i_bit];
    status = CV_RWDataGet(type_e, 
                          nbits, 
                          NULL, 
                          module_ps->rwLine_s.data_a,
                          module_ps->rwLine_s.err_a,
                          module_ps->rwLine_s.expected_data_a);
    if (CV_DRV_DEBUG && status)
    {
       printf("CV[%hd %hd %hd]\n",module_ps->b,module_ps->c,module_ps->n);
       printf("RW Line Test #%.2hd: %s\tstat=0x%8.8X  iss=0x%8.8lx\n",
              test,
              (status)?"Failed   ":"Successful",
              cam_ps->test2_s.rd_statd_s.stat,
              iss);
       for (i_bit=0; i_bit<nbits; i_bit++)
         printf("\t(%.2d):  data=0x%8.8lx  expected=0x%8.8lx\t%s\n",
               i_bit,
               module_ps->rwLine_s.data_a[i_bit],
               module_ps->rwLine_s.expected_data_a[i_bit],
              (module_ps->rwLine_s.err_a[i_bit])?"Error":"");
       printf("\n");
    }
    if (status || !SUCCESS(iss)) goto egress;

    /*
     * Perform Read Write Line test #3 simulated walking one bit and p24
     * Next perform the Read Write Line test #4 with simulated walking zero bit and p24.
     * Here we are setting the DATA register to simulate the bit test.
     */
    wt_data_p = &cam_ps->test3_s.wt_statd_s.data;
    rd_data_p = &cam_ps->test3_s.rd_statd_s.data;
    for (type_e=0; (type_e<RW_LINE_NUM_TYPE) && !status; type_e++)
    {
      test++;
      memset(module_ps->rwLine_s.data_a,0,sizeof(module_ps->rwLine_s.data_a));
      memset(module_ps->rwLine_s.err_a,0,sizeof(module_ps->rwLine_s.err_a));
      memset(module_ps->rwLine_s.expected_data_a,0,sizeof(module_ps->rwLine_s.expected_data_a));
      module_ps->rwLine_s.type_e = type_e;
      module_ps->rwLine_s.test   = test;

      for (i_bit=0; i_bit<nbits; i_bit++)
      {
        /* 
	 * Set the pattern to write to the DATA register
	 * for our bit test
         */
	 *wt_data_p = rwLineOk_a[type_e][i_bit];
         iss = camgo(&cam_ps->test3_s.pkg_p);  
         if (SUCCESS(iss))
            module_ps->rwLine_s.data_a[i_bit] = *rd_data_p;
         iss2 = max(iss,iss2);   
      } /* End of FOR loop (i_bit) */

      /* Check the data is valid */
      status = CV_RWDataGet(type_e,
                            nbits, 
                            NULL, 
                            module_ps->rwLine_s.data_a,
                            module_ps->rwLine_s.err_a,
                            module_ps->rwLine_s.expected_data_a);
      if (CV_DRV_DEBUG && status)
      {
         printf("CV[%hd %hd %hd]\n",module_ps->b,module_ps->c,module_ps->n);
         printf("RW Line Test #%.2hd: %s\tstat=0x%8.8X  iss=0x%8.8lx\n",
                 test,
                (status)?"Failed   ":"Successful",
                 cam_ps->test3_s.rd_statd_s.stat,
                 iss2);  
         for (i_bit=0; i_bit<nbits; i_bit++)
           printf("\t(%.2d):  data=0x%8.8lx  expected=0x%8.8lx\t%s\n",
		  i_bit,
                  module_ps->rwLine_s.data_a[i_bit],
                  module_ps->rwLine_s.expected_data_a[i_bit],
                 (module_ps->rwLine_s.err_a[i_bit])?"Error":"" );
         printf("\n");
      }
    }
    if (status || !SUCCESS(iss)) goto egress;

    /* 
     * Perform the Read Write Line test #5 using walking one bit without P24.
     * WARNING!!! the camac word block transfer data will need to be word swapped.
     */
    test++;
    type_e = WALKING_ONE;
    nbits  = RW_LINE_NUM2;
    nelem  = RW_LINE_NUM2+1;
    memset(module_ps->rwLine_s.data_a,0,sizeof(module_ps->rwLine_s.data_a));
    memset(module_ps->rwLine_s.err_a,0,sizeof(module_ps->rwLine_s.err_a));
    memset(module_ps->rwLine_s.expected_data_a,0,sizeof(module_ps->rwLine_s.expected_data_a));
    module_ps->rwLine_s.type_e = type_e;
    module_ps->rwLine_s.test   = test;

    iss = camgo(&cam_ps->test5_s.pkg_p);
 
    /* Word swap data from block transfer */
    sdata_a = cam_ps->test5_s.rd_statd_s.data_a;
    blockWordSwap(sdata_a,nelem);

    /* Check that the data is what we expect */
    status = CV_RWDataGet(type_e,
                          nbits, 
                          sdata_a, 
                          module_ps->rwLine_s.data_a, 
                          module_ps->rwLine_s.err_a,
                          module_ps->rwLine_s.expected_data_a);
    if (CV_DRV_DEBUG && status)
    {
       printf("RW Line Test #%.2hd: %s\tstat=0x%8.8X  iss=0x%8.8lx\n",
              test,
              (status)?"Failed   ":"Successful",
              cam_ps->test5_s.rd_statd_s.stat,
              iss);
       for (i_bit=0; i_bit<nbits; i_bit++)
         printf("\t(%.2d): data=0x%8.8lx  expected=0x%8.8lx\t%s\n",
                 i_bit,
                module_ps->rwLine_s.data_a[i_bit],
                module_ps->rwLine_s.expected_data_a[i_bit],
               (module_ps->rwLine_s.err_a[i_bit])?"Error":"");
      printf("\n");
    }
    if (status || !SUCCESS(iss))  goto egress;

    /* 
     * Perform Read Write Line test #6 using walking zero bit without P24. 
     * This read test should produce 36 bytes of data.
     */
    test++;
    type_e = WALKING_ZERO;
    nbits  = RW_LINE_NUM2;
    nelem  = RW_LINE_NUM2+1;
    memset(module_ps->rwLine_s.data_a,0,sizeof(module_ps->rwLine_s.data_a));
    memset(module_ps->rwLine_s.err_a,0,sizeof(module_ps->rwLine_s.err_a));
    memset(module_ps->rwLine_s.expected_data_a,0,sizeof(module_ps->rwLine_s.expected_data_a));
    module_ps->rwLine_s.type_e = type_e;
    module_ps->rwLine_s.test   = test;
  
    iss = camgo(&cam_ps->test6_s.pkg_p);

    /* Word swap the camac word block transfer data */
    sdata_a = cam_ps->test6_s.rd_statd_s.data_a;
    blockWordSwap(sdata_a,nelem);
    
    /* Check the data is what we expect. */
    status = CV_RWDataGet(type_e,
                          nbits,
                          sdata_a,
                          module_ps->rwLine_s.data_a,
                          module_ps->rwLine_s.err_a,
                          module_ps->rwLine_s.expected_data_a);
    if (CV_DRV_DEBUG && status)
    {
       printf("CV[%hd %hd %hd]\n",module_ps->b,module_ps->c,module_ps->n);
       printf("RW Line Test #%.2hd: %s\tstat=0x%8.8X  iss=0x%8.8lx\n",
              test,
              (status)?"Failed   ":"Successful",
              cam_ps->test6_s.rd_statd_s.stat,
              iss);
       for (i_bit=0; i_bit<nbits; i_bit++)
         printf("\t(%.2d): data=0x%8.8lx  expected=0x%8.8lx\t%s\n",
                 i_bit,
                module_ps->rwLine_s.data_a[i_bit],
                module_ps->rwLine_s.expected_data_a[i_bit],
               (module_ps->rwLine_s.err_a[i_bit])?"Error":"");
      printf("\n");
    }
    if (status || !SUCCESS(iss)) goto egress;

    /*
     * Perform Read Write Line test #7 with simulated walking one bit 
     * without p24 on the write and with p24 on the read.
     * After which perform the Read Write Line test #8 with simulated
     * walking zero bit without p24 on the write and with p24 on the read.
     *
     * Note: since the data is read as individual 32-bit words in these
     * two tests, the data will not need to be word swapped as in test
     * 5 and 6 above, which performed word block transfers.
     */
    wt_sdata_p = &cam_ps->test7_s.wt_statd_s.data;  /* write data without p24 */
    rd_data_p  = &cam_ps->test7_s.rd_statd_s.data;  /* read data with p24     */
    for (type_e=0; (type_e<RW_LINE_NUM_TYPE) && !status; type_e++)
    {
      test++;
      memset(module_ps->rwLine_s.data_a,0,sizeof(module_ps->rwLine_s.data_a));
      memset(module_ps->rwLine_s.err_a,0,sizeof(module_ps->rwLine_s.err_a));
      memset(module_ps->rwLine_s.expected_data_a,0,sizeof(module_ps->rwLine_s.expected_data_a));
      module_ps->rwLine_s.type_e = type_e;
      module_ps->rwLine_s.test   = test;

      for (i_bit=0; i_bit<nbits; i_bit++)
      {
	 wt_sdata_p[0] = (unsigned short)rwLineOk_a[type_e][i_bit];
         iss = camgo(&cam_ps->test7_s.pkg_p); 
         if (SUCCESS(iss))
            module_ps->rwLine_s.data_a[i_bit] = rd_data_p[0];
         iss2 = max(iss,iss2);
      }
     /* 
      * Check for errors. We want to read all of the data before
      * we check for errors so that we can print a summary of the
      * results, (ie. pass or fail).
      */
      status = CV_RWDataGet(type_e,
                            nbits,
                            NULL,
                            module_ps->rwLine_s.data_a,
                            module_ps->rwLine_s.err_a,
                            module_ps->rwLine_s.expected_data_a);
      if (CV_DRV_DEBUG && status)
      {
        printf("CV[%hd %hd %hd]\n",module_ps->b,module_ps->c,module_ps->n);
        printf("RW Line Test #%.2hd: %s\tstat=0x%8.8X  iss=0x%8.8lx\n",
	        test,
	       (status)?"Failed   ":"Successful",
	        cam_ps->test7_s.rd_statd_s.stat,
	        iss2);
        for (i_bit=0; i_bit<nbits; i_bit++)
           printf("\t(%.2d):  data=0x%8.8lx  expected=0x%8.8lx\t%s\n",
		  i_bit,
                  module_ps->rwLine_s.data_a[i_bit],
                  module_ps->rwLine_s.expected_data_a[i_bit],
                 (module_ps->rwLine_s.err_a[i_bit])?"Error":"" );
        printf("\n");

      }
    }/* End of type_e FOR loop */

    iss = iss2;

    /* If test completed successfully set the rwLine test to zero. */
    if ((status==OK) && SUCCESS(iss))  test++;
    module_ps->rwLine_s.test = test;

egress:
    
    /* 0=test not performed, 9=success, 1-8 is failed test */
    dataway_ps->rwLineErr = module_ps->rwLine_s.test;

   /*
    *  Clear the registers on the bus before 
    * exiting for clean up if we had a problem.
    */
    if (dataway_ps->rwLineErr!=BUS_RWLINE_PASSED)
    {
        /* 
         * Build Camac control word to pulse the C-line.
         * this fuction clears the registers on the bus
         * This is our way of cleaning up before we exit
         * if an error has occurred.
         */
         ctlw = (module_ps->c << CCTLW__C_shc) | M28 | F26A9;
         camio(&ctlw, 0 , &nobcnt, &stat, &emask );
         iss = CRAT_VERIFY_FAIL;
    }
    else if (SUCCESS(iss))
         iss = CRAT_VERIFY_PASSES;
    return( iss );
}


/*====================================================
 
  Abs:  Zero the data before the Camac Bus Dataway Test
 
  Name: CV_DatawayInitData
 
  Args: module_ps                Module information
          Type: pointer             
          Use:  CV_MODULE * const
          Acc:  read-write access
          Mech: By reference

  Rem:  The purpose of this function is zero the
        data buffers before the Camac Bus Verification
        test is performed.

  Side: None

  Ret:  None
            
=======================================================*/ 
static void CV_DatawayInitData( CV_MODULE * const module_ps)
{
   unsigned long      bcnt       = 0;
   campkg_dataway_ts *cam_ps     = NULL;
   campkg_rwlines_ts *rwlines_ps = NULL;


   /* Clear bus status bitmask */
   module_ps->crate_s.bus_stat_u._i = 0;

   /* Clear Command Line test results */
   bcnt = sizeof(module_ps->cmdLine_s.data_a);
   memset(module_ps->cmdLine_s.data_a,0,bcnt);

   /* Clear Read Write Line test results */
   bcnt = sizeof(module_ps->rwLine_s.data_a);
   memset(module_ps->rwLine_s.data_a,0,bcnt);
  
  /*
   * Clear the Camac package stat-data for 
   * the test that are to follow, which include the
   * Command Line, Read Line and Write Line tests
   */
   cam_ps = &module_ps->cam_s.dataway_s;  
   cam_ps->pulseC_s.stat  = 0;
   cam_ps->inhibit_s.stat = 0;
   memset(&cam_ps->scc_s.statd_s,0,sizeof(cam_ps->scc_s.statd_s));

   /* Clear Command Line test stat-data */
   memset(cam_ps->cmd_s.rd_statd_as,0,sizeof(cam_ps->cmd_s.rd_statd_as));
   memset(cam_ps->cmd_s.stat_a,0,sizeof(cam_ps->cmd_s.stat_a));
   memset(&cam_ps->cmd_s.statd_s,0,sizeof(cam_ps->cmd_s.statd_s));
   cam_ps->cmd_s.inhibit_stat = 0;

   /* 
    * Clear Read Write Line stat-data 
    */
   rwlines_ps = &cam_ps->rwlines_s;
   rwlines_ps->test1_s.stat = 0; 
   memset(&rwlines_ps->test1_s.rd_statd_s,0,sizeof(rwlines_ps->test1_s.rd_statd_s));

   rwlines_ps->test2_s.stat     = 0;
   rwlines_ps->test2_s.wt_stat  = 0;
   memset(&rwlines_ps->test2_s.rd_statd_s,0,sizeof(rwlines_ps->test2_s.rd_statd_s));

   memset(&rwlines_ps->test3_s.wt_statd_s,0,sizeof(rwlines_ps->test3_s.wt_statd_s));   
   memset(&rwlines_ps->test3_s.rd_statd_s,0,sizeof(rwlines_ps->test3_s.rd_statd_s));

   rwlines_ps->test5_s.stat = 0;
   memset(&rwlines_ps->test5_s.rd_statd_s,0,sizeof(rwlines_ps->test5_s.rd_statd_s));

   rwlines_ps->test6_s.stat    = 0;
   rwlines_ps->test6_s.wt_stat = 0;
   memset(&rwlines_ps->test6_s.rd_statd_s,0,sizeof(rwlines_ps->test6_s.rd_statd_s));

   memset(&rwlines_ps->test7_s.wt_statd_s,0,sizeof(rwlines_ps->test7_s.wt_statd_s));   
   memset(&rwlines_ps->test7_s.rd_statd_s,0,sizeof(rwlines_ps->test7_s.rd_statd_s));

   /* 
    * Clear the verifier X and Q response from the previous test.
    */
   memset(&cam_ps->Xstat_s,0,sizeof(cam_ps->Xstat_s));
   memset(&cam_ps->Qstat_s,0,sizeof(cam_ps->Qstat_s));
   cam_ps->cmdLineErr = false;
   cam_ps->rwLineErr  = 0;
   cam_ps->timeout    = false;
   return;
 }



/*====================================================
 
  Abs:  Clear the message status prior to a message request
 
  Name: CV_ClrMsgStatus
 
  Args: msgstat_ps                Message status 
          Type: pointer             
          Use:  cv_message_status_ts * const
          Acc:  read-write access
          Mech: By reference


  Rem:  The purpose of this function is to populate
        the message status structure when an requested
        Camac action has been completed.

  Side: None

  Ret:  long
            OK - Successfully completed
                  
=======================================================*/ 
void CV_ClrMsgStatus( cv_message_status_ts * const msgstat_ps )
{
    static const size_t  bcnt = sizeof(epicsTimeStamp);

    if (!msgstat_ps) return;
    /* epicsMutexMustLock( msgstat_ps->mlock ); */

    /* Save time of last request */
    memmove( (void *)&msgstat_ps->lastReqTime,(void *)&msgstat_ps->reqTime,bcnt );

    /* Save time of last successfully completed request */
    if (SUCCESS(msgstat_ps->errCode))
        memmove( (void *)&msgstat_ps->lastTimeOk,(void *)&msgstat_ps->reqTime,bcnt );

    msgstat_ps->lastErrCode = msgstat_ps->errCode;
    epicsTimeGetCurrent( &msgstat_ps->reqTime );   /* Get current time */
    msgstat_ps->errCode = CRAT_OKOK;               /* Set successful   */
    msgstat_ps->opDone  = 0;                       /* Set operations in progress */

    /* epicsMutexUnlock( msgstat_ps->mlock ); */
    return;
}

/*====================================================
 
  Abs:  Set the message status for complete operations
 
  Name: CV_SetMsgStatus
 
  Args: status                     Camac operation status return 
          Type: integer            
          Use:  vmsstat_t
          Acc:  read-only
          Mech: By value

         msgstat_ps                Message status 
          Type: pointer             
          Use:  cv_message_status_ts * const
          Acc:  read-write access
          Mech: By reference


  Rem:  The purpose of this function is to populate
        the message status structure when an requested
        Camac action has been completed.

  Side: None

  Ret:  long
            OK - Successfully completed
                    
=======================================================*/   
static void CV_SetMsgStatus( vmsstat_t status, cv_message_status_ts * const msgstat_ps )
{
  if ( !msgstat_ps ) return;

    /* epicsMutexMustLock( msgstat_ps->mlock ); */

    msgstat_ps->errCode = status;          /* Save Camac operation status */  

    /* Get time operation complete and calculate time to prpcess request. */
    epicsTimeGetCurrent( &msgstat_ps->opDoneTime ); 
    msgstat_ps->elapsedTime = epicsTimeDiffInSeconds( &msgstat_ps->opDoneTime, &msgstat_ps->reqTime);
 
    msgstat_ps->opDone  = 1;               /* Mark operation complete    */

    /* epicsMutexUnlock( msgstat_ps->mlock ); */
    return;
}

/*====================================================
 
  Abs:  Set the Camac Crate Status bitmask
 
  Name: CV_SetCrateStatus
 
  Args: module_ps                Module information
          Type: pointer             
          Use:  CV_MODULE * const
          Acc:  read-write access
          Mech: By reference

         stat                    Crate status bit mask
          Type: bitmask   
          Use:  unsigned short
          Acc:  read-only access
          Mech: By value

         mask                    Bits to mask off 
          Type: bitmask   
          Use:  unsigned short
          Acc:  read-only access
          Mech: By value

  Rem:  The purpose of this function is to update the 
        module crat status bitmask.

  Side: None

  Ret:  None
            
=======================================================*/ 
static void  CV_SetCrateStatus( CV_MODULE * const module_ps, unsigned short stat,unsigned short mask )
{
    if (module_ps)
    {
       epicsMutexMustLock( module_ps->crate_s.mlock );
       module_ps->crate_s.stat_u._i = (module_ps->crate_s.stat_u._i & ~mask) | stat;
       epicsMutexUnlock( module_ps->crate_s.mlock ); 
    }
    return; 
}


#if (EPICS_VERSION>=3 && EPICS_REVISION>=14) || EPICS_VERSION>3
epicsExportAddress(drvet,drvCV);
epicsRegisterFunction(isCrateOnline);
epicsRegisterFunction(CV_Start);
epicsRegisterFunction(CV_AsynThreadStop);
epicsRegisterFunction(CV_DeviceInit);
#endif

/* End of file */

