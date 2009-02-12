/*     **MEMBER**=SLCRMX:REENTRANT
       **CMS**=RMX_CAM
 
================================================================================
 
  Abs:  Reset contents of MBCD package structure to empty.
 
  Name: CAMALOR (module)
        camalo_reset                 Reset contents of MBCD package structure to
                                     empty.
 
  Rem:
 
  Side:
 
  Proto: slcrmxi:cam_proto.hm.
 
  Auth: ??-???-1982, Dave Sherden (DJS).
  Revw:
 
--------------------------------------------------------------------------------
 
  Mod:  15-Jul-2008, Robert C. Sass (RCS):
            Modify for PSCD card.
        09-Jun-1999, Tony Gromme (TEG):
            Translate from PLM386 to iC386.
        19-Dec-1996, Tony Gromme (TEG):
            If package trashed, log message containing unwound call offsets.
        03-Dec-1990, Tony Gromme (TEG):
            Convert to USE32.  Change memory deallocation.
        15-Sep-1989, Tony Gromme (TEG):
            Allow "FG" case (camaloi).
        03-Apr-1989, Robert C. Sass (RCS):
            Convert to 286.
        18-Sep-1987, Tony Gromme (TEG):
            Translated to PLM86; made it reentrant.
        ??-Jul-1984, Keith Jobe (RKJ):
            Re-write.
 
==============================================================================*/

#include <stdio.h>          /* NULL                           */
#include "slc_macros.h"    /* vmsstat_t, SUCCESS.            */
#include "camblkstruc.hm"   /* mbcd_pkt_ts, mbcd_pkghdr_ts,   */
                            /* mbcd_pkg_ts, mbcd_savep_ts.    */
#include "cam_proto.h"     /* (Self)                         */
#include "camdef.hc"        /* CAM_OKOK,...                   */
#include "errlog.h"         /* errlogSevPrintf                */
#include "drvPSCDLib.h"        /* Registers etc. for PSCD access */
 
 
      /******************************************************************/
      /*                                                                */
      /* Abs:  Reset MBCD package structure to empty.                   */
      /* Name: camalo_reset.                                            */
      /* Scop: Public.                                                  */
      /* Rem:  Clear counters in the given MBCD package structure       */
      /*       (created by camalo, camalol or camaloh), and release all */
      /*       memory allocated by camadd for that package, so that it  */
      /*       is as though no camadd's had ever been done to it.       */
      /* Arg:  camtok_pp                 Pointer to selector of MBCD    */
      /*                                 package created by camalo or   */
      /*                                 camaloi.                       */
      /* Ret:  CAM_OKOK if successful;  else CAM_CCB_NFG if key not     */
      /*       recognized.                                              */
      /*                                                                */
      /******************************************************************/
 
 /**procedure**/
 vmsstat_t camalo_reset(campkgp_t *campkg_pp)
 {
     #define CAMBLK_p (*campkg_pp)
     mbcd_savep_ts *savep_p;
     unsigned       j;
     rtems_status_code rss;
     vmsstat_t      iss = CAM_OKOK;
     extern volatile PSCD_CARD pscd_card;        /* PSCD card registers etc. */
 
               /*----------------------------------------------*/
 
     if (CAMBLK_p != NULL)
     {
         if (CAMBLK_p->hdr.key > KEY_LOW)
         {
             iss = CAM_CCB_NFG;
             errlogSevPrintf (errlogMajor, 
                               "Found CAMAC Control Block NFG in camalo_reset with key %d\n",
                               CAMBLK_p->hdr.key);
         }
         else
         {
             /* Release all memory allocated by camadd. */
 
             savep_p = (mbcd_savep_ts *) &CAMBLK_p->mbcd_pkt[CAMBLK_p->hdr.nops];
             for (j = 0;  j < CAMBLK_p->hdr.iop;  ++j)
             {
                 if (savep_p[j].user_p != NULL)
                 {
                     if ((rss = rtems_region_return_segment (pscd_card.memPartId, 
                                                   savep_p[j].hrdw_p)) != RTEMS_SUCCESSFUL)
                     {
                         iss = CAM_NOHEAP;  
                         errlogSevPrintf (errlogFatal, 
                          "CAMALORL - Unable to free dual-port memory with code %d\n", rss);
	             }
                 }
             }
             CAMBLK_p->hdr.bitsummary = 0;
             CAMBLK_p->hdr.iop = 0;
         }
     }
     return iss;
 
     #undef CAMBLK_p
 }                                                       /* End camalo_reset. */
