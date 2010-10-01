/***************************************************************************\
 **   $Id: PDU_Fiducial.c,v 1.3 2010/06/13 00:18:47 pengs Exp $
 **   File:              PDU_Fiducial.c
 **   Author:            Sheng Peng
 **   Email:             pengsh2003@yahoo.com
 **   Phone:             408-660-7762
 **   Company:           RTESYS, Inc.
 **   Date:              09/2009
 **   Version:           1.0
 **
 **   handle high priority actions when fiducial
 **
\****************************************************************************/

/* TODO, check X and Q, cctlwmasks.h:#define MBCD_STAT__Q     0x000010000 */
/* TODO, keep tracking number of errors */

#include "drvPSCDLib.h"
#include "devPDU.h"
#include "devPDUDIAG.h"
#include "slc_macros.h"
#include "cam_proto.h"
#include "cctlwmasks.h"

#include "evrTime.h"
#include "evrPattern.h"

int PDU_F19_CRATE = 2;
epicsExportAddress(int, PDU_F19_CRATE);

int PDU_F19_DEBUG = 0;
epicsExportAddress(int, PDU_F19_DEBUG);

#define DEFAULT_EVR_TIMEOUT 0.02

static int PDUFiducialTask(void * parg);

static epicsEventId EVRFiducialEvent = NULL;

/*
** externs for PDU_F19
*/
int TSmod360;
void *F19pkg_p;

typedef struct
{
    UINT32 stat;
    UINT16 data;
} STAT_DATA;

STAT_DATA stat_data_f19[2]; /* Status/data for F19A8 and F19A9 */

int Errs = 0;                   /* Print no more than MAXERRS */
int Msgs = 0;
#define MAXERRS 0
#define MAXMSGS 0
int Err1 = 0;   /* Next1 errors */
int Err2 = 0;   /* Next2 errors */
int Fids = 0;   /* Total Fiducials */
int Cams = 0;   /* Camac pkgs sent */
int Errc = 0;   /* Camac errors */

int EVRFiducialStart()
{/* This funciton will be called in st.cmd after iocInit() */
/**** moved to F19
    UINT16 nops = 2;
    UINT16 bcnt = 2;
    unsigned long ctlwF19A8 = (PDU_F19_CRATE << CCTLW__C_shc) | (31 << CCTLW__M_shc) | 
                               CCTLW__F19 | CCTLW__A8;
    unsigned long ctlwF19A9 = (PDU_F19_CRATE << CCTLW__C_shc) | (31 << CCTLW__M_shc) | 
                               CCTLW__F19 | CCTLW__A8 | CCTLW__A1;
    UINT16 emask= 0xE000;
    vmsstat_t iss;
********/
    /*-------------------------------*/
    /* Create event and register with EVR */
    EVRFiducialEvent = epicsEventMustCreate(epicsEventEmpty);
    TSmod360 = 0;
    /* scanIoInit(&ioscan); */
    /*
    ** Create/Init Camac package
    */
/************ move to F19
    if(!SUCCESS(iss = camaloh (&nops, &F19pkg_p)))
    {
        errlogPrintf("camalol error 0x%08X\n",(unsigned int) iss);
        goto egress;
    }
    if (!SUCCESS(iss = camadd (&ctlwF19A8, &stat_data_f19[0], &bcnt, &emask, &F19pkg_p)))
    {
        errlogPrintf("camadd error 0x%08X\n",(unsigned int) iss);
        goto release_campkg;
    }
    if (!SUCCESS(iss = camadd (&ctlwF19A9, &stat_data_f19[1], &bcnt, &emask, &F19pkg_p)))
    {
        errlogPrintf("camadd error 0x%08X\n",(unsigned int) iss);
        goto release_campkg;
    }
***********************/
    /* need RTEMS native call to set higher priority */
    return (int)(epicsThreadMustCreate("PDUFiducial", epicsThreadPriorityHigh+1, 20480, 
                                       (EPICSTHREADFUNC)PDUFiducialTask, NULL));
/***** move to F19
release_campkg:
    camdel(&F19pkg_p);
egress:
*******/
    return (0);
}

void EVRFiducial(void)
{/* This funciton will be registered with EVR callback */

    /* get the pattern data for the next 2 time slots - check for good status */
    evrModifier_ta modifier_a;
    epicsTimeStamp time_s;
    unsigned long  patternStatus; /* see evrPattern.h for values */
    int pipestatus;
    /*-------------------------------------*/
    if ( ((Fids % 28800) == 0) && Msgs < MAXMSGS)  /* Print stats every 80 seconds */
    {
       printk ("Evr Stats: Fiducials %d Cam pkgs %d Cam errs %d\n",
                Fids, Cams, Errc);
       Msgs++;
    }
    Fids++;
    pipestatus = evrTimeGetFromPipeline(&time_s,  evrTimeNext1, modifier_a, 
					&patternStatus, 0,0,0);
    if (pipestatus || patternStatus)
    {
       Err1++;
       if (Errs < MAXERRS)
       {
	  printk ("Err %x pipeline next1 time patternStatus %x\n",pipestatus, patternStatus);
          Errs++;
          goto egress;
       }
    }
    stat_data_f19[0].data = (BEAMCODE(modifier_a) << 8);
    pipestatus = evrTimeGetFromPipeline(&time_s,  evrTimeNext2, modifier_a, &patternStatus, 
                                        0,0,0);
    if (pipestatus || patternStatus)
    {
       Err2++;
       if (Errs < MAXERRS)
       {
	  printk ("Err %x pipeline next2 time patternStatus %x\n",pipestatus, patternStatus);
          Errs++;
          goto egress;
       }
    }
    stat_data_f19[1].data = (BEAMCODE(modifier_a) << 8);
    if (Msgs < MAXMSGS)
    {
       printk ("Beamcode A8 %x A9 %x\n",stat_data_f19[0].data, stat_data_f19[1].data);
       Msgs++;
    }
    /* This is 360Hz. So printf will screw timing */
    if(PDU_F19_DEBUG >= 3) printk("Got fiducial\n");
    /* post event/release sema to wakeup worker task here */
    if(EVRFiducialEvent) epicsEventSignal(EVRFiducialEvent);
egress:
    return;
}
   
int PDU_F19(unsigned int crate, unsigned int PP0, unsigned int PP1);
static int PDUFiducialTask(void * parg)
{
    vmsstat_t iss;
/****    int nowait = 0;  ****/
    void * dum = NULL;
    UINT16 nops = 5;
    UINT16 bcnt = 2;
    unsigned long ctlwF19A8 = (PDU_F19_CRATE << CCTLW__C_shc) | (31 << CCTLW__M_shc) | 
                               CCTLW__F19 | CCTLW__A8;
    unsigned long ctlwF19A9 = (PDU_F19_CRATE << CCTLW__C_shc) | (31 << CCTLW__M_shc) | 
                               CCTLW__F19 | CCTLW__A8 | CCTLW__A1;
    UINT16 emask= 0xE000;
    /*-------------------------------*/
    /* Register EVRFiducial */
    printf ("In PDU_F19 rebuild pkg each fiducial!!\n");
    evrTimeRegister((FIDUCIALFUNCTION)EVRFiducial, dum);
    while(TRUE)
    {
        int status;
        status = epicsEventWaitWithTimeout(EVRFiducialEvent, DEFAULT_EVR_TIMEOUT);
        if(status != epicsEventWaitOK)
        {
            if(status == epicsEventWaitTimeout)
            {
                if(PDU_F19_DEBUG > 3) errlogPrintf("Wait EVR timeout, check timing?\n");
                continue;
            }
            else
            {
                errlogPrintf("Wait EVR Error, what happened? Let's sleep 2 seconds.\n");
                epicsThreadSleep(2.0);
                continue;
            }
        }
        else
        {
            /* do F19 */
           if(!SUCCESS(iss = camaloh (&nops, &F19pkg_p)))
           {
              errlogPrintf("camalol error 0x%08X\n",(unsigned int) iss);
              continue;
           }
           if (!SUCCESS(iss = camadd (&ctlwF19A8, &stat_data_f19[0], &bcnt, &emask, &F19pkg_p)))
           {
              errlogPrintf("camadd error 0x%08X\n",(unsigned int) iss);
              goto release_campkg;
           }
           if (!SUCCESS(iss = camadd (&ctlwF19A9, &stat_data_f19[1], &bcnt, &emask, &F19pkg_p)))
           {
              errlogPrintf("camadd error 0x%08X\n",(unsigned int) iss);
              goto release_campkg;
           }
           fidPDUDIAGPreCam (&F19pkg_p);
           if(!SUCCESS(iss = camgo(&F19pkg_p)))
	      Errc++;
           fidPDUDIAGPostCam ();
	release_campkg:
	   if(!SUCCESS(iss = camdel(&F19pkg_p)))
              Errc++;
	  /*****************
          camgo_get_data(&F19pkg_p);
          if (stat_data_f19[0].stat != 0x2fd30000)
	  {
	      Errc++;
              if (Errc < MAXERRS)
              {
                  printk ("Previous camgo_start_pscd statii %x %x\n",stat_data_f19[0].stat, 
                          stat_data_f19[1].stat);
              }
	  }
	  iss = camgo_start_pscd(&F19pkg_p, nowait);
	  *************/
          Cams++;
	   /*****************
            if(PDU_F19_CRATE)
	    {
	       if ((TSmod360 % 36) == 0)
                  PDU_F19(PDU_F19_CRATE, 10, 0);
               else
                  PDU_F19(PDU_F19_CRATE, 0, 0); 
	    }            
            if(PDU_F19_DEBUG>=2) printk("Send F19\n");
	   **************/
	    if (TSmod360++ == 359)
	       TSmod360 = 0;
            /* scanIoRequest(ioscan); */
        }
    }

    /*Should never return from following call*/
    return(0);
}

/**************************************************************************************************/
/* Here we supply the driver report function for epics                                            */
/**************************************************************************************************/
static long     PDU_EPICS_Init();
static  long    PDU_EPICS_Report(int level);

const struct drvet drvPDU = {2,                              /*2 Table Entries */
                              (DRVSUPFUN) PDU_EPICS_Report,  /* Driver Report Routine */
                              (DRVSUPFUN) PDU_EPICS_Init}; /* Driver Initialization Routine */

epicsExportAddress(drvet,drvPDU);

/* implementation */
static long PDU_EPICS_Init()
{
    EVRFiducialStart();
    return 0;
}

static long PDU_EPICS_Report(int level)
{
    printf("\n"PDU_DRV_VER_STRING"\n\n");

    return 0;
}


