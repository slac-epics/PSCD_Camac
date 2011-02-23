/*     **MEMBER**=SLCRMX:REENTRANT
       **CMS**=RMX_CAM
 
================================================================================
 
  Abs:  Modify and execute MBCD package.
 
  Name: CAMMOD (module)
        cammod                       Modify given MBCD package according to
                                      given cctlw and mask, and execute modified
                                      package.
 
  Rem:
 
  Side:
 
  Proto: slcrmxi:cam_proto.h.
 
  Auth: ??-???-1982, Dave Sherden (DJS).
  Revw:
 
--------------------------------------------------------------------------------
 
  Mod:  21-Jul-2008, Robert C. Sass (RCS)
            Modify for PSCD card running under EPICS
        10-Jun-1999, Tony Gromme (TEG):
            Translate from PLM386 to iC386.
        07-Dec-1990, Tony Gromme (TEG):
            Convert to USE32;  translate to PLM386.
        ??-Jul-1984, Keith Jobe (RKJ):
            Re-write.
 
==============================================================================*/

#include "slc_macros.h"    /* vmsstat_t.                              */
#include "cctlwmasks.h"    /* CCTLW__P8, CCTLW__P24, CCTLW__F8,       */
                            /*  CCTLW__MPC.                            */
#include "camblkstruc.h"   /* mbcd_pkt_ts, mbcd_pkghdr_ts,            */
                            /*  mbcd_pkg_ts.                           */
#include "cam_proto.h"     /* (Self), camgo.                          */
#include "errlog.h"         /* errlogSevPrintf                         */
#include "camdef.h"        /* CAM_ILMODCTLW.                          */
 
 
       /****************************************************************/
       /*                                                              */
       /* Abs:  Modify and execute MBCD package.                       */
       /* Name: cammod.                                                */
       /* Scop: Public.                                                */
       /* Rem:  Run through the packets in the given previously        */
       /*       constructed package, patching (only) the CCTLW in the  */
       /*       same way in each packet, and then execute the          */
       /*       resulting package by calling CAMGO.  The package       */
       /*       remains as altered.  In each packet, bits in the       */
       /*       packet's CCTLW will be replaced by bits from the given */
       /*       CCTLW corresponding to 1-bits in the given MASK.  All  */
       /*       arguments are passed by reference.  See Basic User's   */
       /*       Guide (BUG) for exact details.                         */
       /* Arg:  camtok_p             Pointer to token (selector) of    */
       /*                            package (previously allocated by  */
       /*                            camalo and constructed by calls   */
       /*                            to camadd).                       */
       /* Ret:  CAM_OKOK if successful;  else CAM_ILMODCTLW if         */
       /*       requested modification of cctlw is not allowed;  or    */
       /*       possible bad return from camgo.                        */
       /*                                                              */
       /****************************************************************/
 
 /**procedure**/
 vmsstat_t cammod(const unsigned int *cctlw_p, const unsigned int *mask_p,
                  const campkgp_t *camblk_pp)
 {
     #define CAMBLK_p (*camblk_pp)
     vmsstat_t      iss;
     unsigned       iop, j;
 
               /*----------------------------------------------*/
 
     iop = CAMBLK_p->hdr.iop;
     for (j = 0;  j < iop;  ++j)
     {
         if (((CAMBLK_p->mbcd_pkt[j].cctlw ^ *cctlw_p) & *mask_p &
              (CCTLW__F8 | CCTLW__P24 | CCTLW__P8 )) == 0)
         {
             CAMBLK_p->mbcd_pkt[j].cctlw = (CAMBLK_p->mbcd_pkt[j].cctlw &
                     (~*mask_p | CCTLW__MPC)) | (*cctlw_p & *mask_p & ~CCTLW__MPC);
         }
         else
         {
             iss = CAM_ILMODCTLW;
             errlogSevPrintf (errlogMajor, 
                "CAMMOD -  Illegal change %x to CCTLW %d.\n", (unsigned) *cctlw_p, j);
             return (iss);
         }
     }
     return (camgo(camblk_pp));  /* Execute package with modified ctlw */
 
     #undef CAMBLK_p
 }                                                             /* End cammod. */
