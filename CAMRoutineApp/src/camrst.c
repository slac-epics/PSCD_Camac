/*     **MEMBER**=SLCRMX:REENTRANT
       **CMS**=RMX_CAM
 
================================================================================
 
  Abs:  Reset contents of MBCD package structure to empty but don't deallocate
        any memory.
 
  Name: CAMRST (module)
        camrst                 Reset contents of MBCD package structure to
                               empty.
 
  Rem:
 
  Side:
 
  Proto: slcrmxi:cam_proto.h.
 
  Auth: 03-Mar-2011, Robert C. Sass (RCS).
  Rev:
 
--------------------------------------------------------------------------------
 
  Mod:  
 
==============================================================================*/

#include <stdio.h>         /* NULL                           */
#include "slc_macros.h"    /* vmsstat_t, SUCCESS.            */
#include "camblkstruc.h"   /* mbcd_pkt_ts, mbcd_pkghdr_ts,   */
                           /* mbcd_pkg_ts, mbcd_savep_ts.    */
#include "cam_proto.h"     /* (Self)                         */
#include "camdef.h"        /* CAM_OKOK,...                   */
#include "errlog.h"        /* errlogSevPrintf                */

#include <epicsExport.h>
#include <registryFunction.h>
 
      /******************************************************************/
      /*                                                                */
      /* Abs:  Reset MBCD package structure to empty.                   */
      /* Name: camrst.                                                  */
      /* Scop: Public.                                                  */
      /* Rem:  Clear iop counter in the given MBCD package structure    */
      /*       (created by camalo, camalol or camaloh) so that it       */
      /*       is as though no camadd's had ever been done to it.       */
      /* Arg:  camtok_pp                 Pointer to selector of MBCD    */
      /*                                 package created by camalo or   */
      /*                                 camaloi.                       */
      /* Ret:  CAM_OKOK if successful;  else CAM_CCB_NFG if key not     */
      /*       recognized.                                              */
      /*                                                                */
      /******************************************************************/
 
 /**procedure**/
 vmsstat_t camrst(campkgp_t *campkg_pp)
 {
     #define CAMBLK_p (*campkg_pp)
     vmsstat_t      iss = CAM_OKOK;
               /*----------------------------------------------*/
 
     if (CAMBLK_p != NULL)
     {
         if (CAMBLK_p->hdr.key > KEY_LOW)
         {
             iss = CAM_CCB_NFG;
             errlogSevPrintf (errlogMajor, 
                              "Found CAMAC Control Block NFG in camrst with key %d\n",
                              CAMBLK_p->hdr.key);
         }
         else
         {
	    CAMBLK_p->hdr.iop = 0;   /* Camgo checks this and won't execute pkg */
         }
     }
     return iss;
 
     #undef CAMBLK_p
 }                                                       /* End camrst. */

epicsRegisterFunction(camrst);
