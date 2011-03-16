/*     **MEMBER**=SLCRMX:REENTRANT
       **CMS**=RMX_CAM
 
================================================================================
 
  Abs:  Allocate MBCD package.
 
  Name: CAMALO (module)
        camalo                       Allocate and initialize segment in which
                                     MBCD package (for execution by camgo using 
                                     the medium priority PSCD port) can be built.
        camalol                      Same as camalo except uses low priority port 
        camaloh                      Same as camalo except uses high priority port 
                                     for execution by interrupt-driven code, not by camgo
        camal_1 (local)              Code common to camalo, camalol and camaloh.
 
  Rem:
 
  Side:
 
  Proto: cam_proto.h.
 
  Auth: ??-???-1982, Dave Sherden (DJS).
  Revw:
 
--------------------------------------------------------------------------------
 
  Mod:  14-Jul-2008, Robert C. Sass (RCS)
            Modify for PSCD card running under EPICS
        09-Jun-1999, Tony Gromme (TEG):
            Translate from PLM386 to iC386.
        03-Dec-1990, Tony Gromme (TEG):
            Convert to USE32.
        07-Apr-1989, Tony Gromme (TEG):
            Provide CAMALOI;  translate to PLM86.
        11-Jan-1989, Robert C. Sass (RCS):
            Call campoola instead of RQ$CREATE$SEGMENT.
        ??-Jul-1984, Keith Jobe (RKJ):
            Re-write.
 
==============================================================================*/
#include <stdio.h>             /* NULL                                    */ 
#include "rtems.h"             /* rtems_region_get_segment.               */
#include "slc_macros.h"        /* vmsstat_t, SUCCESS.                     */
#include "camblkstruc.h"       /* mbcd_pkt_ts, mbcd_pkghdr_ts,            */
                               /* mbcd_pkg_ts, mbcd_savep_ts.             */
#include "cam_proto.h"         /* (Self)                                  */
#include "camdef.h"            /* CAM_NOPS_MAX.                           */ 
#include "errlog.h"            /* errlogSevPrintf                         */
#include "drvPSCDLib.h"        /* Registers etc. for PSCD access */
 
 
 /* Prototype of local procedure. */
 static vmsstat_t camal_1(unsigned camblk_bc, unsigned short key, unsigned nops,
                           campkgp_t *camblk_pp);
 
        /**************************************************************/
        /*                                                            */
        /* Abs:  Code common to camalo, camalol and camaloh.          */
        /* Name: camal_1.                                             */
        /* Scop: Local;  called only from camalo, camalol and camaloh.*/
        /* Rem:  Allocate and initialize memory in which a camac      */
        /*       package can be built.                                */
        /* Args: camblk_bc       Size in bytes of MBCD package        */
        /*                        segment to be allocated.            */
        /*       key             Validation key to be stored in       */
        /*                        package header structure.           */
        /*       nops            Max # (to be stored in package       */
        /*                        header) of packets that can be      */
        /*                        constructed in this package.        */
        /*       camblk_pp       Pointer to where we should store     */
        /*                        pointer to allocated package.       */
        /* Ret:  CAM_OKOK if successful;  CAM_NOHEAP if can't alloc   */
        /*       memory or CAM_NOPS_MAX if *nops_p is invalid.        */
        /*                                                            */
        /**************************************************************/
 
 /**procedure**/
 static vmsstat_t camal_1(unsigned camblk_bc, unsigned short key, unsigned nops,
                          campkgp_t *camblk_pp)
 {
     #define CAMBLK_p (*camblk_pp)
     rtems_status_code rss;
     vmsstat_t      iss = CAM_OKOK;
     mbcd_savep_ts *savep_p;
     int            j;
     extern volatile PSCD_CARD pscd_card;        /* PSCD card registers etc. */
 
               /*----------------------------------------------*/
 
     if (nops > 0 && nops <= 100)   /* Some random number? */
     {
         /* 
	 ** Allocate memory.  Store key by which package can later be validated, 
	 ** store max # of packets, store pointer to first packet for PSCD, 
	 ** zero mbcd and user packets and return package pointer to caller.
         */

         if ((rss = rtems_region_get_segment(pscd_card.memPartId, camblk_bc, 
              RTEMS_NO_WAIT, 0, (void **) camblk_pp)) == RTEMS_SUCCESSFUL)
         {
             CAMBLK_p->hdr.key = key;
             CAMBLK_p->hdr.nops = nops;
             CAMBLK_p->hdr.iop = 0;
             CAMBLK_p->hdr.tbytes = 0;
             CAMBLK_p->hdr.bitsummary = 0;
             CAMBLK_p->hdr.spare = 0;
             CAMBLK_p->hdr.pkg_p = CAMBLK_p->mbcd_pkt;
             /*
	     ** Zero hardware (PSCD DP memory) pointer
	     */
             for (savep_p = (mbcd_savep_ts *) &CAMBLK_p->mbcd_pkt[nops], j = 0;  j < nops;  ++j)
		savep_p[j].hrdw_p = NULL;
         }
         else
	 {
             *camblk_pp = NULL;
             iss = CAM_NOHEAP;
             errlogSevPrintf (errlogFatal, 
                           "CAMALO - Unable to allocate dual-port memory with code %d\n",rss);
	 }
     }
     else
     {
         *camblk_pp = NULL;
         iss = CAM_NOPS_MAX;
         errlogSevPrintf (errlogMajor, "CAMALO: Illegal request for %d > 100 packets.\n",
                           nops);
     }
     return iss;
 
     #undef CAMBLK_p
 }                                                      /* End local camal_1. */
 
        /**************************************************************/
        /*                                                            */
        /* Abs:  Allocate memory in which a camac package can be      */
        /*       constructed by one or more calls to camadd.          */
        /*       Sets key to the medium priority queue.               */
        /* Name: camalo.                                              */
        /* Scop: Public.                                              */
        /* Rem:  Note that since camadd may allocate additional       */
        /*       memory, the package can correctly be deleted only by */
        /*       camdel.  All arguments are passed by reference.  See */
        /*       Basic User's Guide (BUG) for more details.           */
        /* Args:                                                      */
        /*       nops_p          Pointer to word containing max # of  */
        /*                        packets that can be built in this   */
        /*                        package.                            */
        /*       camblk_pp       Pointer to where camal_1 will return */
        /*                        pointer to this package             */
        /*                        subsequent use.                     */
        /* Ret:  Returns from camal1                                  */
        /*                                                            */
        /**************************************************************/
 
 /**procedure**/
 vmsstat_t camalo(const unsigned short *nops_p, campkgp_t *camblk_pp)
 {
     return camal_1(sizeof(mbcd_pkghdr_ts) + *nops_p * (sizeof(mbcd_pkt_ts) +
                     sizeof(mbcd_savep_ts)), KEY_MED, *nops_p, camblk_pp);
 }                                                             /* End camalo. */
 
 
        /**************************************************************/
        /*                                                            */
        /* Abs:  Identical to camalo, except that packets will be     */
        /*       built for execution on the low priority port.        */
        /* Name: camalol.                                             */
        /* Scop: Public.                                              */
        /* Rem:  Note that since camadd may allocate additional       */
        /*       memory, the package can correctly be deleted only by */
        /*       camdel.  All arguments are passed by reference.  See */
        /*       Basic User's Guide (BUG) for more details.           */
        /* Args:                                                      */
        /*       nops_p          Pointer to word containing max # of  */
        /*                        packets that can be built in this   */
        /*                        package.                            */
        /*       camblk_pp       Pointer to where camal_1 will return */
        /*                        pointer identifying this package    */ 
	/*                        for all subsequent use.             */
        /* Ret:  Returns from camal1                                  */
        /*                                                            */
        /**************************************************************/
 
 /**procedure**/
 vmsstat_t camalol(const unsigned short *nops_p, campkgp_t *camblk_pp)
 {
     return camal_1(sizeof(mbcd_pkghdr_ts) + *nops_p * (sizeof(mbcd_pkt_ts) +
		    sizeof(mbcd_savep_ts)), KEY_LOW, *nops_p, camblk_pp);
 }                                                            /* End camalol. */

 
        /**************************************************************/
        /*                                                            */
        /* Abs:  Identical to camalo, except that packets will be     */
        /*       built for execution on the high priority port.       */
        /* Name: camaloh.                                             */
        /* Scop: Public.                                              */
        /* Rem:  Note that since camadd may allocate additional       */
        /*       memory, the package can correctly be deleted only by */
        /*       camdel.  All arguments are passed by reference.  See */
        /*       Basic User's Guide (BUG) for more details.           */
        /* Args:                                                      */
        /*       nops_p          Pointer to word containing max # of  */
        /*                        packets that can be built in this   */
        /*                        package.                            */
        /*       camblk_pp       Pointer to where camal_1 will return */
        /*                        pointer identifying this package    */ 
	/*                        for all subsequent use.             */
        /* Ret:  Returns from camal1                                  */
        /*                                                            */
        /**************************************************************/
 
 /**procedure**/
 vmsstat_t camaloh(const unsigned short *nops_p,
                   campkgp_t *camblk_pp)
 {
     return camal_1(sizeof(mbcd_pkghdr_ts) + *nops_p * (sizeof(mbcd_pkt_ts) +
                    sizeof(mbcd_savep_ts)), KEY_HI, *nops_p, camblk_pp);
 }                                                            /* End camaloh. */
