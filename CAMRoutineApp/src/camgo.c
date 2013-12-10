/*     **MEMBER**=SLCRMX:REENTRANT
       **CMS**=RMX_CAM
 
================================================================================
 
  Abs:  Execute PSCD package.
 
  Name: CAMGO (module)
        cam_ini (public)              Must be called once to init for PSCD access
        camgo_start_pscd (public)     Copy user data if write, Wait until PSCD's fifo 
                                      is empty and initiate execution of PSCD package.
        camgo  (public)               Execute PSCD package previously built by
                                      camalo and camadd, and check errors.
        camgo_get_data (public)       Copy status/data from dual-port memory after I/O
                                      is complete.
        cam_get_errcod (local)        Translate flag bits into VMS status code.
        camgo_spin_errs (local)       Wait for completion and summarize errors.
 
  Rem:
 
  Side: Operation of camac modules is affected.
 
  Proto: slcrmxi:cam_proto.h.
 
  Auth: ??-???-1982, Dave Sherden (DJS).
  Revw:
 
--------------------------------------------------------------------------------
 
  Mod:  
        09-Dec-2013, Sonya Hoobler (SONYA)
            Add flag to disable error messages; add check of flag to print statements
        16-Feb-2009, Robert C. Sass (RCS)
            Cahnge for PSCD card struct and interrupts.
        21-Jul-2008, Robert C. Sass (RCS)
            Modify for PSCD card running under EPICS
        10-Jun-1999, Tony Gromme (TEG):
            Translate from PLM386 to iC386.
        19-Dec-1996, Tony Gromme (TEG):
            For serious programming errors, log message containing unwound call
            offsets.
        11-Jul-1996, Tony Gromme (TEG):
            Add io_offset for Skater board.
        18-Jun-1993, Tony Gromme (TEG):
            Vary numerical spin count according to cpu;  spin-test only high
            half of PSCD status longword rather than entire longword.
        03-Dec-1990, Tony Gromme (TEG):
            Rewrite for USE32.  All memory allocation is now done by camalo and
            camadd.
        ??-Jan-1989, Robert C. Sass, (RCS):
            Major changes for campoola.
        ??-Jul-1984, Keith Jobe (RKJ):
            Re-write.
 
==============================================================================*/

#include <stdio.h>                 /* NULL                                    */ 
#include <string.h>                /* memcpy                                  */

#include "drvPSCDLib.h"            /* Registers etc. for PSCD access          */
#include "epicsMutex.h"            /* epicsMutexId, Create, Lock, Unlock      */
#include "epicsEvent.h"            /* For interrupt event                      */
#include "slc_macros.h"            /* vmsstat_t, SUCCESS.                     */
#include "cctlwmasks.h"           /* CCTLW__P8, CCTLW__F8, CCTLW__MPC        */
#include "camblkstruc.h"          /* mbcd_pkt_ts, mbcd_pkghdr_ts,            */
                                   /* mbcd_pkg_ts, mbcd_savep_ts,             */
                                   /* mbcd_stad_ts                            */
#include "cam_proto.h"             /* (Self).                                 */
#include "errlog.h"                /* errlogSevPrintf                         */
#include "camdef.h"               /* CAM_OKOK,...                            */
#include "epicsThread.h"

int MBCD_MODE = 0;   /* Global flag to disable crate/soft timeout msgs when in MBCD mode */
int CAM_DRV_DEBUG = 0; /* Global flag to disable/enable messages; default is to disable */

epicsExportAddress(int, MBCD_MODE);

 static unsigned long cam_tdv_busy_count = 0,
                      unwou_errf         = 0; 
 static const vmsstat_t camerrcods[9] = {CAM_OKOK,
                                         CAM_NO_Q,      CAM_NO_X,     CAM_NO_EOSTRM, CAM_NO_EOSTRM,
                                         CAM_NO_WCTERM, CAM_CRATE_TO, CAM_SOFT_TO,   CAM_MBCD_NFG};
/*
** Mutex for each priority level to start the PSCD.
*/

#define MAX_PRIOR 3
 static epicsMutexId   camgo_sem_toks[MAX_PRIOR] = {NULL, NULL, NULL};
 
/***********************************
 **  Prototypes of local procedures:
 ********************************* */
 static vmsstat_t cam_get_errcod(unsigned char eflagb);
 static vmsstat_t camgo_spin_errs(unsigned miop, const mbcd_pkt_ts mbcd_pkt_p[],
                                  const mbcd_savep_ts savep_p[]);
 void pack_w_to_bytes(const unsigned short sorc_p[],
	                             unsigned char dest_p[], unsigned bc);	
 void unpk_bytes_to_w(const unsigned char sorc_p[],
	 	                     unsigned short dest_p[], unsigned bc);
#ifndef _X86_
/* To or From Dual-port memory */
#define TODP 1
#define FROMDP 2
/*
** Big-endian word copy to insure that odd half-words go in the right place.
*/
void bewcpy (void *dest_p, void *wsrc_p, size_t wc,unsigned char dir );
#endif

/*********************
** Local procedures
**********************/

/*
**  Extract the low-order byte of each 16-bit word of a word array,
**    and pack the bytes into a byte array. 
*/

void pack_w_to_bytes(const unsigned short sorc_p[], 
                     unsigned char dest_p[], unsigned bc)
{
   int count = bc & 0xFFFF;   /* Just take low 16 bits of count */
   int i,j;
   unsigned char *sorcc_p = (unsigned char *) sorc_p; /* Sorc as char array */
/*---------------------------- Code --------------------------------------*/
  for (i=0, j=0; i<count; i++, j+=2)
     dest_p[i] = sorcc_p[j];
  return;
}

/*
** Unpack bytes from a byte array, zero-extending each byte to a
**        16-bit word, and store the words in a word array:
*/

void unpk_bytes_to_w(const unsigned char sorc_p[], 
                     unsigned short dest_p[], unsigned bc)
{
   int count = bc & 0xFFFF;   /* Just take low 16 bits of count */
   int i;
/*---------------------------- Code --------------------------------------*/
   for (i=0; i<count; i++)
   {
      dest_p[i] = 0;
      dest_p[i] = sorc_p[i];
   }   
  return;
}

#ifndef _X86_
/*
** Big-endian word copy. The PCI bus and hence dual-port memory is read
** little endian by the PSCD i.e. sequential addresses proceed from the 
** low-order byte. The PPC is big-endian i.e. sequential addresses proceed 
** from the high-order byte. The PSCD always does word transfers and the PLX 
** swaps the bytes but not the words which we must do here.
**
** Furthermore, how we do the swap depends on the direction.
**
** For TODP i.e. PPC -> DP;  PPC[0] -> DP[1]
** For FROMDP i.e. DP  -> PPC; DP[0]  -> PPC[1]
*/
void bewcpy (void *dest_p, void *src_p, size_t wc, unsigned char dir)
{
  unsigned short *lsrc_p  = (unsigned short *) src_p, 
                 *ldest_p = (unsigned short *) dest_p; /* Make ptrs *word */
  /*------------------ code ---------------*/
  /*
  ** If the word count is even this degerates into a simple memcpy because the 
  ** PLX chip handles the byte swapping.
  ** If it's odd, then do memcpy for the even number of words and put the last word
  ** in the correct half depending on the direction.
  */
  if (!(wc & 1))
  {
    memcpy (ldest_p, lsrc_p, (wc << 1));  /* Even word count = memcpy */
  }
  else            /* Odd word count. For 1 word the memcpy is a NOP. */
  {
    memcpy (ldest_p, lsrc_p, ((wc-1) << 1)); /* Copy all but the last odd word */
    lsrc_p =  &(lsrc_p[wc-1]);    /* Point to the last words */
    ldest_p = &(ldest_p[wc-1]);
    if (dir == TODP)
    {
      ldest_p[1] = lsrc_p[0];
    }
    else
    {
      ldest_p[0] = lsrc_p[1];
    }
  }
  return;
}
#endif
         /************************************************************/
         /*                                                          */
         /* Abs:  CAM routine initialization.                        */
         /* Name: cam_ini.                                           */
         /* Scop: Public;  must be called once from somewhere.       */
         /* Rem:  Creates semaphore for each priority & calls camioi */
         /* Arg:  PSCD card structure.                               */
         /* Ctxt: camgo_sem_toks - Mutex semaphore for each priority */
         /* Ret:  CAM_OKOK if successfull; possible bad status return*/
         /*       camioi CAM_NGNG if mutex can't be created.         */
         /*                                                          */
         /************************************************************/
 
 /**procedure**/
 vmsstat_t cam_ini(void *pPSCD)
 {
     int i;
     vmsstat_t      iss = CAM_OKOK;
 
     /*----------------------------------------------*/
 
     if (camgo_sem_toks[MAX_PRIOR-1] != NULL)
       goto egress;      /* Already initialized */

     for (i=0; i < MAX_PRIOR; i++)
     {
       if ((camgo_sem_toks[i] = epicsMutexCreate()) == NULL)
       {
         iss = CAM_NGNG;
         errlogSevPrintf (errlogFatal, "CAMGO - Cannot create EPICS mutex\n");
       }
     }
     iss = camioi();        /* Init camio */
 egress:
     return iss;
 }                                                             /* End cam_ini. */

        /**************************************************************/
        /*                                                            */
        /* Abs:  Copy user's data and initiate execution of the       */
        /*       PSCD package.                                        */
        /*       If the wait arg is true then wait via semaphore      */
        /*       for the completion interrupt else return after SIO.  */
        /* Name: camgo_start_pscd.                                    */
        /* Scop: Public;                                              */
        /* Rem:  Routine camgo_start_pscd initiates execution of one  */
        /*       camac package (one physically chained string of      */
        /*       camac packets), testing the PSCD's "TDV" register    */
        /*       beforehand to ensure that the PSCD's fifo will never */
        /*       contain more than one camac package pointer.         */
        /* Arg:  campkg_pp             pointer to camac package       */
        /*                                                            */
        /* Ret:  None.                                                */
        /*                                                            */
        /**************************************************************/
 
 /**procedure**/
 vmsstat_t camgo_start_pscd(const campkgp_t *campkg_pp, int wait)
 {
#define CAMBLK_p (*campkg_pp) 
#define TDV_BSYN_MSK 0x80
#define TDV_PEND_MSK 0x20
#define TDV_PTO_MSK  0x40 
#define TDV_DONE_MSK 0x1
     unsigned       j;
     unsigned int prior = CAMBLK_p->hdr.key;
     mbcd_savep_ts *savep_p = (void *) &CAMBLK_p->mbcd_pkt[CAMBLK_p->hdr.nops];
     mbcd_stad_ts  *stadh_p;
     unsigned miop = CAMBLK_p->hdr.iop; /* # of operations in package */
     unsigned jp;
     epicsMutexLockStatus oss;
     epicsEventWaitStatus ess;
     vmsstat_t iss = CAM_OKOK;
     extern volatile PSCD_CARD pscd_card;        /* PSCD card registers etc. */
     /*----------------------------------------------*/
     /* For each packet, clear the status longword                       */
     /* in case hardware is out to lunch, and, if the packet is a write  */
     /* and we are buffering the operation, then copy the caller's data  */
     /* into memory allocated by camadd.                                 */
 
     for (jp = 0;  jp < miop;  ++jp)
     {
         stadh_p = savep_p[jp].hrdw_p;
         stadh_p->camst_dw = 0;
         if (savep_p[jp].user_p != NULL &&
            (CAMBLK_p->mbcd_pkt[jp].cctlw & (CCTLW__F16 | CCTLW__F8)) == CCTLW__F16)
         {
             if ((CAMBLK_p->mbcd_pkt[jp].cctlw & CCTLW__P8) == 0)
             {
#ifdef _X86_
                 memcpy(&stadh_p->dat_w, (char *) savep_p[jp].user_p + 4,
                 CAMBLK_p->mbcd_pkt[jp].wc_max << 1);
#else
                 bewcpy(&stadh_p->dat_w, savep_p[jp].user_p + 4,
                         CAMBLK_p->mbcd_pkt[jp].wc_max, TODP);
#endif
             }
             else
                 unpk_bytes_to_w((unsigned char *) savep_p[jp].user_p + 4,
                                 stadh_p->dat_w, CAMBLK_p->mbcd_pkt[jp].wc_max);
         }
     }

     /* Serialize access to the registers for this priority */

     if ((oss = epicsMutexLock(camgo_sem_toks[prior])) != 0)
     {
         iss = CAM_NGNG;
         errlogSevPrintf (errlogFatal, "CAMGO - Unable to lock Mutex with EPICS code %x\n", oss);
         goto egress;
     }         
 
     /* 
     ** Spin while the PSCD is busy (BSYN is 1 when it's not busy) or pending.  
     ** We will spin for ~2 milliseconds and then signal an error.
     */

     for (j=4000; (!(*pscd_card.tdv_p[prior] & TDV_BSYN_MSK) || 
                    (*pscd_card.tdv_p[prior] & TDV_PEND_MSK)) && j > 0; --j)
             ;
     if (j == 0)
     {
         /* Don't emit errors forever if H/W dies */

         if (cam_tdv_busy_count < 99)
         {
                 errlogSevPrintf (errlogMajor, 
                     "CAMGO - PCSD board error -- TDV always busy.\n");
             ++cam_tdv_busy_count;
         }
         epicsMutexUnlock (camgo_sem_toks[prior]);  /* Free semaphore */
     }
     else
     {
 
     /* 
     ** Set wait for interrupt event condition and push given address into PSCD's 
     ** priority fifo to start execution of package and optionally wait for done.
     */

	pscd_card.waitSemSio[prior] = FALSE;
        if (wait)
	   pscd_card.waitSemSio[prior] = TRUE;
        *(pscd_card.sio_p[prior]) = MEMPARTADRS_TO_SIO((unsigned) CAMBLK_p->hdr.pkg_p);

     /*
     **  If requested, wait for I/O completion interrupt. Free mutex when done. 
     */

        if (wait)
        {
	   if ((ess = epicsEventWaitWithTimeout(pscd_card.semSio[prior], 0.1)) != epicsEventWaitOK)
	   {
              iss = CAM_NGNG;        
              if (CAM_DRV_DEBUG)   errlogSevPrintf (errlogMinor, "CAMGO - timeout waiting for interrupt %x\n",ess);
              pscd_card.waitSemSio[prior] = FALSE;  /* Tell ISR no longer waiting for interrupt */
	   }
           *pscd_card.tdv_p[prior] = TDV_DONE_MSK;  /* Insure clear for next I/O */
        }
        epicsMutexUnlock (camgo_sem_toks[prior]);
     }
 egress:
     return iss;
 }                                                   /* End camgo_start_pscd. */ 


        /**************************************************************/
        /*                                                            */
        /* Abs:  Copy the status and data from dual-port memory to    */
        /*       user's buffer. No error or completion check is made. */
        /* Name: camgo_get_data  .                                    */
        /* Scop: Public;                                              */
        /* Rem:  Routine camgo_get_data copies the status and data    */
        /*       from dual-port memory to the user's buffer.          */
        /*       It assumes that the I/O operation is complete.       */
        /* Arg:  campkg_pp             pointer to camac package       */
        /*                                                            */
        /* Ret:  None.                                                */
        /*                                                            */
        /**************************************************************/
 
 /**procedure**/
 void camgo_get_data(const campkgp_t *campkg_pp)
 {
     #define CAMBLK_p (*campkg_pp)
     unsigned  miop = CAMBLK_p->hdr.iop;
     unsigned  jp;
     mbcd_savep_ts *savep_p = (void *) &CAMBLK_p->mbcd_pkt[CAMBLK_p->hdr.nops];
     /*----------------------------------------------*/

     /* For each packet, if we are buffering the operation, then  */
     /* copy the status longword from memory allocated by camadd, */
     /* and if the packet is a read, then copy the data too.      */
     /* Note that if a packet is requesting P8, then we are       */
     /* buffering that operation.                                 */

     for (jp = 0;  jp < miop;  ++jp)
     {
         if (savep_p[jp].user_p != NULL)
         {
             if ((CAMBLK_p->mbcd_pkt[jp].cctlw & (CCTLW__F16 | CCTLW__F8)) == 0)
             {
                 if ((CAMBLK_p->mbcd_pkt[jp].cctlw & CCTLW__P8) == 0)
                 {
#ifdef _X86_
                     memcpy(savep_p[jp].user_p, savep_p[jp].hrdw_p,
                            (CAMBLK_p->mbcd_pkt[jp].wc_max << 1) + 4);
#else
                     bewcpy(savep_p[jp].user_p, savep_p[jp].hrdw_p,
                            (CAMBLK_p->mbcd_pkt[jp].wc_max + 2), FROMDP);
#endif
                 }
                 else
                 {
                     pack_w_to_bytes((unsigned short *) savep_p[jp].hrdw_p + 2,
                                     (unsigned char *) savep_p[jp].user_p + 4,
                                     CAMBLK_p->mbcd_pkt[jp].wc_max);
                     *(long *) savep_p[jp].user_p = *(long *) savep_p[jp].hrdw_p;
                 }
             }
             else
             {
                  *(long *) savep_p[jp].user_p = *(long *) savep_p[jp].hrdw_p;
             }
         }
     }
     return;
 } 
        /**************************************************************/
        /*                                                            */
        /* Abs:  Translate given error code byte into VMS status code.*/
        /*       index into camerrcods                                */
        /* Name: cam_get_errcod.                                      */
        /* Scop: Local;  called only from camgo_spin_errs.            */
        /* Rem:                                                       */
        /* Arg:  eflagb                Byte containing error bits     */
        /*                             derived from PSCD status and   */
        /*                             previously specified emask.    */
        /* Ret:  Translated VMS status code.                          */
        /*                                                            */
        /**************************************************************/
 
 /**procedure**/
 static vmsstat_t cam_get_errcod(unsigned char eflagb)
 {   unsigned       j;
 
               /*----------------------------------------------*/
 
     for (j = 8;  j > 0 && !(eflagb & 1 << (j-1));  --j)
         ;
     return camerrcods[j];
 }                                               /* End local cam_get_errcod. */
 
 
 
        /**************************************************************/
        /*                                                            */
        /* Abs:  For each packet in PSCD package being executed       */
        /*       summarize errors seen.                               */
        /* Name: camgo_spin_errs.                                     */
        /* Scop: Local;  called only from camgo.                      */
        /* Rem:                                                       */
        /* Args: miop               # of packets built in package.    */
        /*       mbcd_pkt_p         Pointer to 1st packet of package. */
        /*       savep_p            Pointer to 1st element of array   */
        /*                          built by camadd of structures     */
        /*                          containing buffering pointers and */
        /*                          emasks.                           */
        /* Ret:  CAM_OKOK, or VMS status code from cam_get_errcod for */
        /*       first packet in package showing any error.           */
        /*                                                            */
        /**************************************************************/
 
 /**procedure**/
 static vmsstat_t camgo_spin_errs(unsigned miop, const mbcd_pkt_ts mbcd_pkt_p[],
                                  const mbcd_savep_ts savep_p[])
 { 
     unsigned int    *mbcd_stat_p, mbcd_stat;
     unsigned        jp; 
     unsigned char   eflag;
     unsigned int    parmdw[4];
     vmsstat_t       iss, issx;
 
               /*----------------------------------------------*/
 
     /* For each packet in the package test status.  Note that      */
     /* camgo has zeroed each longword into which PSCD will store   */
     /* status, and high-order word of any status longword stored   */
     /* by PSCD must be nonzero. */
  
     iss = CAM_OKOK;
     for (jp = 0;  jp < miop;  ++jp)
     {
         mbcd_stat_p = savep_p[jp].hrdw_p;
         mbcd_stat = *mbcd_stat_p;     /* Read status into local memory */
 
         /* Assume package is complete.  Note that the PSCD         */
         /* stores the low-order status word (16 bits, including    */
         /* remaining wordcount) first, then the high-order word    */
         /* (including echo of crate & module address).             */ 

         /* Translate status conditioned by emask into error  */
         /* codes to be logged and error code to be returned. */

         if (mbcd_stat != 0)
            eflag = (((mbcd_stat >>16) ^ 0x1F) & 0x3F) | mbcd_stat >> 8;
         else           /* S/W timeout */
             eflag = 0x40;

         /* Low byte of emask conditions err_send's done here.  High byte */
         /* of emask conditions the code returned to caller.  Caller will */
         /* get first bad error code seen, or ok.                         */

         if (!SUCCESS(issx = cam_get_errcod(eflag & savep_p[jp].emask)))
         {
             parmdw[0] = (mbcd_pkt_p[jp].cctlw & CCTLW__C) >> CCTLW__C_shc;
             parmdw[1] = (mbcd_pkt_p[jp].cctlw & CCTLW__M) >> CCTLW__M_shc;
             parmdw[2] = mbcd_pkt_p[jp].cctlw & CCTLW__A;
             parmdw[3] = (mbcd_pkt_p[jp].cctlw & CCTLW__F) >> CCTLW__F_shc;
	     if (CAM_DRV_DEBUG) 
             { 
               switch (issx)
               {
                  case CAM_NO_Q:
                    errlogSevPrintf (errlogMinor,
                   "Camgo: no Q response C %x N %x A %x F %x Stat=%x\n",
                    parmdw[0], parmdw[1], parmdw[2], parmdw[3], mbcd_stat);
                    break;
                  case CAM_NO_X:
                    errlogSevPrintf (errlogMinor,
                   "Camgo: no X response C %x N %x A %x F %x Stat=%x\n",
                    parmdw[0], parmdw[1], parmdw[2], parmdw[3], mbcd_stat);
                    break;
                  case CAM_NO_EOSTRM:
                    errlogSevPrintf (errlogMinor,
                   "Camgo: failed to terminate on end of scan C %x N %x A %x F %x Stat=%x\n",
                    parmdw[0], parmdw[1], parmdw[2], parmdw[3], mbcd_stat);
                    break;
                  case CAM_NO_WCTERM:
                    errlogSevPrintf (errlogMinor,
                   "Camgo: failed to exhaust word count C %x N %x A %x F %x Stat=%x\n",
                    parmdw[0], parmdw[1], parmdw[2], parmdw[3], mbcd_stat);
                    break;
                  case CAM_CRATE_TO:
                    if (!MBCD_MODE)
		    {
                       errlogSevPrintf (errlogMinor,
                      "Camgo: crate timeout C %x N %x A %x F %x Stat=%x\n",
                       parmdw[0], parmdw[1], parmdw[2], parmdw[3], mbcd_stat);
                    }
                    break;
                  case CAM_SOFT_TO:
                    if (!MBCD_MODE)
		    {
                       errlogSevPrintf (errlogMinor,
                      "Camgo: software timeout. No PSCD response C %x N %x A %x F %x Stat=%x\n",
                       parmdw[0], parmdw[1], parmdw[2], parmdw[3], mbcd_stat);
		    }
                    break;
                  case CAM_MBCD_NFG:
                    errlogSevPrintf (errlogMinor,
                   "Camgo: PSCD failure C %x N %x A %x F %x Stat=%x\n",
                    parmdw[0], parmdw[1], parmdw[2], parmdw[3], mbcd_stat);
                    break;
               }
	   }

         }
         if (!SUCCESS(issx = cam_get_errcod(eflag & savep_p[jp].emask >> 8))  &&
             SUCCESS(iss))
         {
             iss = issx;
         }
 
         /* Complain if we are given a zero crate address.     */
 
         if ((mbcd_pkt_p[jp].cctlw & CCTLW__C) == 0 && !unwou_errf)
         {
             iss = CAM_BAD_C;
             errlogSevPrintf (errlogMinor, 
                 "CAMGO - Zero crate address is invalid.\n");
             ++unwou_errf;
         }
     }
     return iss;
 }                                              /* End local camgo_spin_errs. */
 
         /************************************************************/
         /*                                                          */
         /* Abs:  Execute given PSCD package and summarize errors    */
         /*       seen.                                              */
         /* Name: camgo.                                             */
         /* Scop: Public.                                            */
         /* Rem:  Routine camgo initiates execution of an already    */
         /*       prepared camac package, and spins until PSCD       */
         /*       completes.  We check all PSCD-stored status        */
         /*       longwords for errors.                              */
         /* Arg:  camtok_p              Pointer to selector of PSCD  */
         /*                             package created by camalo    */
         /*                             and by camadd.               */
         /* Ret:  CAM_OKOK if successful, or CAM_* error code        */
         /*       resulting from validity checks or from analyzing   */
         /*       status longwords stored by PSCD if package is      */
         /*       executed.                                          */
         /*                                                          */
         /************************************************************/
 
 /**procedure**/
 vmsstat_t camgo(const campkgp_t *campkg_pp)
 {
     #define CAMBLK_p (*campkg_pp)
     mbcd_savep_ts *savep_p;
     unsigned       miop;
     int            wait = 1;
     vmsstat_t      iss = CAM_OKOK;
    
     /*----------------------------------------------*/
 
     /* Validate PSCD package structure. */
 
     if (CAMBLK_p == NULL)
     {
         iss = CAM_NULL_PTR;
         errlogSevPrintf (errlogMinor, 
           "CAMGO - Unable to execute packet with NULL package pointer.\n");
         goto egress;
     }
     if ((miop = CAMBLK_p->hdr.iop) == 0 || miop > 127)
     {
         iss = CAM_NO_PKTS;
         errlogSevPrintf (errlogMinor, 
           "CAMGO - Number of packets %d in package is invalid.\n",miop);
         goto egress;
     }
 
     /* Copy data, start the PSCD operation and wait for completion. */
 
     iss = camgo_start_pscd(campkg_pp, wait);

     /* 
     ** Spin until completion and sort out the errors.
     ** Copy status/data to user's buffer.
     */

     savep_p = (void *) &CAMBLK_p->mbcd_pkt[CAMBLK_p->hdr.nops];
     iss = camgo_spin_errs(miop, CAMBLK_p->mbcd_pkt, savep_p);
     camgo_get_data(campkg_pp);
 egress:
     return iss;
 
     #undef CAMBLK_p
 }                                                              /* End camgo. */


