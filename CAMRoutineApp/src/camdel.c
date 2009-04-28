/*     **MEMBER**=SLCRMX:REENTRANT
       **CMS**=RMX_CAM
 
================================================================================
 
  Abs:  Deallocate MBCD package.
 
  Name: CAMDEL (module)
        camdel                       Deallocate MBCD package and any attached
                                      memory.
 
  Rem:
 
  Side:
 
  Proto: slcrmxi:cam_proto.h.
 
  Auth: ??-???-1982, Dave Sherden (DJS).
  Revw:
 
--------------------------------------------------------------------------------
 
  Mod:  14-Jul-2008, Robert C. Sass (RCS)
            Modify for PSCD card running under EPICS
        09-Jun-1999, Tony Gromme (TEG):
            Translate from PLM386 to iC386.
        03-Dec-1990, Tony Gromme (TEG):
            Converted to USE32;  call camalo_reset.
        ??-Mar-1989, Robert C. Sass (RCS):
            Changes for iRMX286.
        ??-Oct-1985, Keith Jobe (RKJ):
            Soft return if called with selector = 0.
        ??-Jul-1984, Keith Jobe (RKJ):
            Re-write.
 
==============================================================================*/

#include "rtems.h"          /* rtems_region_return_segment.            */
#include "slc_macros.h"    /* vmsstat_t, SUCCESS.                     */
#include "camblkstruc.h"   /* mbcd_pkt_ts, mbcd_pkghdr_ts             */
#include "cam_proto.h"     /* (Self), camalo_reset, campoold.         */
#include "camdef.h"        /* CAM_OKOK,...                            */
#include "errlog.h"         /* errlogSevPrintf                         */
#include "drvPSCDLib.h"    /* Registers etc. for PSCD access          */
 
         /************************************************************/
         /*                                                          */
         /* Abs:  Delete given MBCD package and any additional       */
         /*       memory allocated per packet by camadd.             */
         /* Name: camdel.                                            */
         /* Scop: Public.                                            */
         /* Rem:  If successful and caller's pointer was nonzero,    */
         /*       caller's pointer will be zeroed.If caller's pointer*/
         /*       was zero then we do nothing and return success.    */
         /* Arg:  camtok_p           Pointer to selector of package  */
         /*                          segment allocated by camalo or  */
         /*                          camaloi.                        */
         /* Ret:  CAM_OKOK if successful;  else possible bad status  */
         /*       return from camalo_reset.                          */
         /*                                                          */
         /************************************************************/
 
 /**procedure**/
 vmsstat_t camdel(campkgp_t *campkg_pp)
 {   
     rtems_status_code rss;
     vmsstat_t      iss = CAM_OKOK;
     extern PSCD_CARD pscd_card;        /* PSCD card registers etc. */
 
               /*----------------------------------------------*/
 
     if (*campkg_pp != NULL)
     {
         if (SUCCESS(iss = camalo_reset(campkg_pp)))
	 {
            if ((rss = rtems_region_return_segment (pscd_card.memPartId, 
                                                    *campkg_pp)) != RTEMS_SUCCESSFUL)
            {
               iss = CAM_NOHEAP;  
               errlogSevPrintf (errlogFatal, 
                        "CAMDEL - Unable to free dual-port memory with code %d\n", rss);
	    }
	 }     
     }
     return iss;
 }                                                             /* End camdel. */
