/*     **MEMBER**=SLCRMX:REENTRANT
       **CMS**=RMX_CAM
 
================================================================================
 
  Abs:  One-shot build + execute MBCD packet.
 
  Name: CAMIO (module)
        camio                        Construct MBCD package consisting of one
                                      packet, and execute it.
        camioi                       Module initialization.
 
  Rem:
 
  Side:
 
  Proto: slcrmxi:cam_proto.hm.
 
  Auth: ??-???-1982, Dave Sherden (DJS).
  Revw:
 
--------------------------------------------------------------------------------
 
  Mod:  26-Jul-2008, Robert C. Sass (RCS)
            Modify for PSCD card running under EPICS
        10-Jun-1999, Tony Gromme (TEG):
            Translate from PLM386 to iC386.
        07-Dec-1990, Tony Gromme (TEG):
            Rewrite for USE32;  add CAMIOI.
        ??-Jan-1989, Robert C. Sass (RCS):
            Changes for 286.
        ??-Jul-1984, Keith Jobe (RKJ):
            Re-write.
 
==============================================================================*/

#include <string.h>        /* memcpy                                  */
#include <stdio.h>         /* NULL                                    */ 

#include "rtems.h"         /* rtems_region_get/return_segment         */
#include "epicsMutex.h"    /* epicsMutexId, epicsMutexLockStatus,     */
                           /*  epicsMutexCreate, epicsMutexLock,      */
                           /*  epicsMutexUnlock, epicsMutexDestroy.   */
#include "slc_macros.h"   /* vmsstat_t, SUCCESS.                     */
#include "cctlwmasks.hm"   /* CCTLW__F16, CCTLW__F8.                  */
#include "camblkstruc.hm"  /* mbcd_pkt_ts, mbcd_pkghdr_ts,            */
                           /*  mbcd_pkg_ts, mbcd_stad_ts.             */
#include "cam_proto.h"    /* (Self), camalo, camalo_reset, camadd,   */
                           /*  camgo.                                 */
#include "camdef.hc"       /* CAM_NOHEAP                              */
#include "micrdef.hc"      /* MICR_EPMUTEXCREA.                       */
#include "errlog.h"        /* errlogSevPrintf                         */
#include "drvPSCDLib.h"       /* Registers etc. for PSCD access          */

extern PSCD_CARD pscd_card;    /* PSCD card registers etc. */     
 
#define CAMIO_DAT_DEFBC 128
 
 static epicsMutexId   camio_sem_tok;
 static campkgp_t      camio_pkg_tok = NULL;
 static void          *camio_stad_tok;
 
 static const unsigned short one_w = 1;
 
       /****************************************************************/
       /*                                                              */
       /* Abs:  One-shot build + execute MBCD packet.                  */
       /* Name: camio.                                                 */
       /* Scop: Public.                                                */
       /* Rem:  Routine camio provides a "shorthand" for camalo +      */
       /*       camadd + camgo + camdel.  Routine camio executes only  */
       /*       one packet.  Routine camio returns the status and data */
       /*       separately, and therefore is obliged to buffer them.   */
       /*       Routine camio uses a semaphore to serialize use of     */
       /*       its local package and status/data memory segment.      */
       /*       Routine camioi must be called once before camio can    */
       /*       ever be called.  All arguments are passed by           */
       /*       reference.  See Basic User's Guide (BUG) for exact     */
       /*       details.                                               */
       /* Args: brch_p           Pointer to branch name (ignored by    */
       /*                         version of camio that runs in micro).*/
       /*       cctlw_p          Pointer to 32-bit camac control       */
       /*                         longword.                            */
       /*       datau_p          Pointer to camac data if any (not     */
       /*                         including MBCD status).              */
       /*       bcnt_p           Pointer to word containing requested  */
       /*                         bytecount.                           */
       /*       statu_p          Pointer to longword where resulting   */
       /*                         MBCD status will be stored.          */
      /*       emask_p          Pointer to word containing flags      */
       /*                         controlling error reporting.         */
       /* Ctxt: camio_stad_tok  Pointer to reusable memory for         */
       /*                       buffering status/data.                 */
       /*       camio_pkg_tok   Identifier of reusable camac package.  */
       /*       camio_sem_tok   Identifier of our mutex semaphore.     */
       /* Ret:  CAM_OKOK if successful;  else possible bad status      */
       /*       return from camalo_reset, camadd,                      */
       /* rtems_region_get_segment, or (most likely) camgo.            */
       /*                                                              */
       /****************************************************************/
 
 /**procedure**/
 vmsstat_t camio(const char brch_p[4], const unsigned long *cctlw_p, void *datau_p,
                 const unsigned short *bcnt_p, void *statu_p, const unsigned short *emask_p)
 {  
     void          *stad_free_p;
     mbcd_stad_ts  *stad_locl_p;
     epicsMutexLockStatus oss;
     rtems_status_code rss;
     vmsstat_t      iss;
 
               /*----------------------------------------------*/
 
     /* Make sure returned MBCD status will be 0 if failure */
     /* before camgo.  Get memory if *bcnt_p > default.     */
 
     *(unsigned long *) statu_p = 0;
     if (*bcnt_p <= CAMIO_DAT_DEFBC)
     {
         stad_free_p = NULL;
         stad_locl_p = (void *) camio_stad_tok;
     }
     else     /* Request to big for pre-allocated package */
     {
         if ((rss = rtems_region_get_segment(pscd_card.memPartId, *bcnt_p+12, 
              RTEMS_NO_WAIT, 0, &stad_free_p)) != RTEMS_SUCCESSFUL)
	 {
	     iss = CAM_NOHEAP;
             errlogSevPrintf (errlogFatal, 
                    "CAMIO - Unable to (allocate dynamic memory with code %d\n", rss);
             goto egress;
         }
         stad_locl_p = stad_free_p;
     }
 
     /* Claim our semaphore, thus serializing use of our local */
     /* preallocated segments.                                 */
 
     if ((oss = epicsMutexLock(camio_sem_tok)) != 0)
     {
         iss = MICR_UNK_IRMXERR;
         errlogSevPrintf (errlogFatal, "CAMIO - Unable to lock Mutex with EPICS code %x\n", oss);
         goto egress;
     }         

     /*  Reset our local preallocated package to 0 packets, */
     /*  and build one packet therein.                      */
 
     if (SUCCESS(iss = camalo_reset(&camio_pkg_tok)) &&
         SUCCESS(iss = camadd(cctlw_p, stad_locl_p, bcnt_p, emask_p, &camio_pkg_tok)))
     {
 
         /* If write, then copy data from caller's area before executing packet. */
 
         if ((*cctlw_p & (CCTLW__F16 | CCTLW__F8)) == CCTLW__F16)
             memcpy(&stad_locl_p->dat_u, datau_p, *bcnt_p);
 
         /* Execute the packet. */
 
         iss = camgo(&camio_pkg_tok);
 
         /*  If read, then copy data to caller's area after executing packet. */
 
         if ((*cctlw_p & (CCTLW__F16 | CCTLW__F8)) == 0)
             memcpy(datau_p, &stad_locl_p->dat_u, *bcnt_p);
 
         /*  Copy status longword as generated by MBCD. */
 
         *(unsigned long *) statu_p = stad_locl_p->camst_u.camst_dw;
     }
 
     /*  Release our semaphore. */
 
     epicsMutexUnlock(camio_sem_tok);

 
     /*  If we allocated memory here, then deallocate it.  Then return. */
 
egress:
     if (stad_free_p != NULL)
     {
        if ((rss = rtems_region_return_segment (pscd_card.memPartId, 
                                                stad_free_p)) != RTEMS_SUCCESSFUL)
        {
           iss = CAM_NOHEAP;  
           errlogSevPrintf (errlogFatal, 
                          "CAMIO - Unable to free dual-port memory with code %d\n", rss);     
        }
     }
     return iss;
 }                                                              /* End camio. */
 
 
 
         /************************************************************/
         /*                                                          */
         /* Abs:  Module initialization.                             */
         /* Name: camioi.                                            */
         /* Scop: Public;  called only once from somewhere.          */
         /* Rem:  Before camio is ever called, camioi must be called */
         /*       to create camio's semaphore, reusable camac        */
         /*       package, and default status/data memory segment.   */
         /* Arg:  None.                                              */
         /* Ctxt: camio_sem_tok     Identifier of mutex semaphore.   */
         /*       camio_pkg_tok     Identifier of reusable camac     */
	 /*                         package.                         */
         /*       camio_stad_tok    Pointer to reusable memory for   */
	 /*                         buffering                        */
         /* Ret:  CAM_OKOK if successfull; else CAM_NOHEAP if no     */
         /*       memory or possible bad status return from camalo   */
         /*       or MICR_EPMUTEXCREA if mutex cant be created.      */
         /*                                                          */
         /************************************************************/
 
 /**procedure**/
 vmsstat_t camioi(void)
 {  
     rtems_status_code rss;
     vmsstat_t      iss = CAM_OKOK;
 
     /*----------------------------------------------*/
 
     if (camio_pkg_tok != NULL)
       goto egress;      /* Already initialized */

     if ((camio_sem_tok = epicsMutexCreate()) != NULL)
     {
         if (SUCCESS(iss = camalo(NULL, &one_w, &camio_pkg_tok)))
         {
             if ((rss = rtems_region_get_segment(pscd_card.memPartId, CAMIO_DAT_DEFBC+12, 
                  RTEMS_NO_WAIT, 0, &camio_stad_tok)) != RTEMS_SUCCESSFUL)
             {
	         iss = CAM_NOHEAP;
                 errlogSevPrintf (errlogFatal, 
                     "CAMIOI - Unable to allocate dynamic memory with code %x\n", rss);
                 camdel(&camio_pkg_tok);
                 epicsMutexDestroy(camio_sem_tok);
             }
         }
         else
         {
             epicsMutexDestroy(camio_sem_tok);
         }
     }
     else
     {
         iss = MICR_EPMUTEXCREA;
         errlogSevPrintf (errlogFatal, "CAMIOI - Cannot create EPICS mutex\n");
     }
 egress:
     return iss;
 }                                                             /* End camioi. */

