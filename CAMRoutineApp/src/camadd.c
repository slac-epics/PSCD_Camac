/*     **MEMBER**=SLCRMX:REENTRANT
       **CMS**=RMX_CAM
 
================================================================================
 
  Abs:  Add cctlw to MBCD package.
 
  Name: CAMADD (module)
        camadd                       Append (another) MBCD control word (cctlw)
                                      to MBCD package being built in segment
                                      previously allocated by camalo, camalol or 
                                      camaloh.
 
  Rem:
 
  Side:
 
  Proto: cam_proto.h.
 
  Auth: ??-???-1982, Dave Sherden (DJS).
  Revw:
 
--------------------------------------------------------------------------------
 
  Mod:   14-Jul-2008, Robert C. Sass (RCS)
            Modify for PSCD card running under EPICS
         24-Mar-2000, Tony Gromme (TEG):
            Unconditionally clear the "scan crate" and "scan module" bits in any
            packet.
        08-Jun-1999, Tony Gromme (TEG):
            Translate from PLM386 to iC386.
        19-Dec-1996, Tony Gromme (TEG):
            For serious programming errors, log message containing unwound call
            offsets.
        10-Dec-1990, Tony Gromme (TEG):
            Check that MBCD wont overshoot end of caller's status/data segment.
        03-Dec-1990, Tony Gromme (TEG):
            Rewrite for USE32.  All memory allocation is now done by camadd,
            none by camgo.
        07-Apr-1989, Tony Gromme (TEG):
            Changes for camaloi;  translated into PLM86.
        14-Mar-1989, Robert C. Sass (RCS):
            Changes for iRMX286.
        ??-Oct-1985, Nan Phinney (NP for RKJ):
            Add check for word-aligned data buffer.
        ??-Aug-1984, Keith Jobe (RKJ):
            Re-write.
 
==============================================================================*/
#include <stdio.h>          /* NULL                                    */ 
#include "rtems.h"          /* rtems_region_get_segment.               */
#include "slc_macros.h"    /* vmsstat_t, SUCCESS.                     */
#include "cctlwmasks.h"    /* CCTLW__P8, CCTLW__F8, CCTLW__RE_PACK,   */
                            /*  CCTLW__MPC.                            */
#include "camblkstruc.h"   /* mbcd_pkt_ts, mbcd_pkghdr_ts,            */
                            /*  mbcd_pkg_ts, mbcd_savep_ts,            */
                            /*  CAM_REPACK_WC.                         */
#include "cam_proto.h"     /* (Self)                                  */
#include "camdef.h"        /* CAM_OKOK,...                            */
#include "errlog.h"         /* errlogSevPrintf                         */
#include "drvPSCDLib.h"    /* Registers etc. for PSCD access */
  
/*
** If any errors found in packet to be added then key is set to something other than
** KEY_LOW, KEY_MED or KEY_HI and subsequent attempts to use the package result in returning
** CAM_CCB_NFG error code.
*/
 
       /****************************************************************/
       /*                                                              */
       /* Abs:  Add a(nother) packet to an MBCD package.               */
       /* Name: camadd.                                                */
       /* Scop: Public.                                                */
       /* Rem:  Memory segment for the package must have been          */
       /*       allocated by camalo, camalol or camaloh.               */
       /*       allocate additional memory.  All arguments are passed  */
       /*       by reference.  See Basic User's Guide (BUG) for        */
       /*       complete MBCD (Multibus camac driver) details.  Note   */
       /*       that reusing the same status/data buffer for more than */
       /*       one packet in the same package can lead to failure.    */
       /* Args: cctlw_p           Pointer to 32-bit camac control      */
       /*                          longword.                           */
       /*       stad_p            Pointer to buffer for status and     */
       /*                          data for packet.                    */
       /*       bcnt_p            Pointer to word containing requested */
       /*                          bytecount.                          */
       /*       emask_p           Pointer to word containing flags     */
       /*                          controlling error reporting.        */
       /*       camblk_pp         Pointer to memory allocated by camalo*/
       /*                          camalol or camaloh                  */
       /* Ret:  CAM_OKOK if successful; CAM_NULL_PTR if camblk_pp      */
       /*       points to NULL selector;  or CAM_NOHEAP if no memory.  */
       /*       CAM_CCB_NFG if package has been invalidated by         */
       /*       previous error;  or CAM_IOP_MAX if package already     */
       /*       contains max # of packets as determined by camalo;  or */
       /*       CAM_BC_ODD if requested bytecount is odd and "P8" bit  */
       /*       is not set in given cctlw;  or CAM_BC_ZNZ if requested */
       /*       bytecount is nonzero and "F8" bit is set in given      */
       /*       cctlw.                                                 */
       /*                                                              */
       /****************************************************************/
 
 /**procedure**/
 vmsstat_t camadd(const unsigned long *cctlw_p, const void *stad_p, 
                  const unsigned short *bcnt_p, const unsigned short *emask_p,
                  campkgp_t *camblk_pp)
 {
     #define CAMBLK_p (*camblk_pp)
     mbcd_savep_ts *savep_p;
     unsigned       bcp4, iop;
     unsigned short wcnt;
     rtems_status_code rss;
     vmsstat_t      iss = CAM_OKOK;
     extern volatile PSCD_CARD pscd_card;        /* PSCD card registers etc. */
 
               /*----------------------------------------------*/
 
     /* Validate the package pointer and some parameters. */
 
     if (CAMBLK_p == NULL)
     {
         iss = CAM_NULL_PTR;
         errlogSevPrintf (errlogMinor, 
           "CAMADD - Unable to add packet with NULL package pointer = 0\n");
         goto egress;
     }
     if (CAMBLK_p->hdr.key > KEY_LOW)
     {
         iss = CAM_CCB_NFG;
         errlogSevPrintf (errlogMajor, 
           "CAMADD - Found Camac Control Block NFG with key %d\n",CAMBLK_p->hdr.key);
         goto egress;
     }
     if (CAMBLK_p->hdr.iop >= CAMBLK_p->hdr.nops)
     {
         CAMBLK_p->hdr.key = KEY_BAD_OV;
         iss = CAM_IOP_MAX;
         errlogSevPrintf (errlogMinor, 
           "CAMADD - Attempt to add more packets than allocated.\n");
         goto egress;
     }
     if (*bcnt_p & 1 && !(*cctlw_p & CCTLW__P8))
     {
         CAMBLK_p->hdr.key = KEY_BAD_OD;
         iss = CAM_BC_ODD;
         errlogSevPrintf (errlogMinor, 
           "CAMADD - Odd byte count and no Pack 8.\n");
         goto egress;
     }
     if (*cctlw_p & CCTLW__F8 && *bcnt_p > 0)
     {
         CAMBLK_p->hdr.key = KEY_BAD_BC;
         iss = CAM_BC_ZNZ;
         errlogSevPrintf (errlogMinor,"CAMADD - Bytecount .gt. zero with F8 no data xfer.\n");
         goto egress;
     }

     /*
     ** We passed the first gauntlet of error checks. Now set appropriate allocation
     ** size in bytes and word count. For Pack 8 (P8) the word count transferred by the
     ** PSCD card = byte count passed in by the caller. Confusing but that's the way it is.
     */

     savep_p = (mbcd_savep_ts *) &CAMBLK_p->mbcd_pkt[CAMBLK_p->hdr.nops];
     iop = CAMBLK_p->hdr.iop;
     if (!(*cctlw_p & CCTLW__P8))
     {
         if ((wcnt = *bcnt_p >> 1) > 127)
         {
             CAMBLK_p->hdr.key = KEY_BAD_BM;
             iss = CAM_BC_MAX;
             errlogSevPrintf (errlogMinor, 
               "CAMADD - MBCD CAMAC byte count .gt. 256\n");
             goto egress;
         }
         bcp4 = (wcnt << 1) + 4 + 2;  /* Allow for stat/data and endian swap */
         if ((rss = rtems_region_get_segment(pscd_card.memPartId, bcp4, 
              RTEMS_NO_WAIT, 0, &savep_p[iop].hrdw_p)) != RTEMS_SUCCESSFUL)
         {
             CAMBLK_p->hdr.key = KEY_BAD_NH;
             iss = CAM_NOHEAP;
             errlogSevPrintf (errlogFatal, 
               "CAMADD - Unable to allocate dual-port memory with code %d\n",rss);
             goto egress;
         }
     }
     else        /* Is P8 pack option */
     {
         CAMBLK_p->hdr.bitsummary |= CCTLW__P8;
         wcnt = *bcnt_p;
         bcp4 = (wcnt << 1) + 4 + 2;  /* Status + endian swap */
         if ((rss = rtems_region_get_segment(pscd_card.memPartId, bcp4, 
              RTEMS_NO_WAIT, 0, &savep_p[iop].hrdw_p)) != RTEMS_SUCCESSFUL)
         {
             CAMBLK_p->hdr.key = KEY_BAD_NH;
             iss = CAM_NOHEAP;
             errlogSevPrintf (errlogFatal, 
               "CAMADD - Unable to allocate P8 dual-port memory\n");
             goto egress;
         }
     }

     /*
     ** Save user's data pointer and emask in packet
     */

     savep_p[iop].user_p = (void *) stad_p;
     savep_p[iop].emask = *emask_p;
 
     /* Store rest of new packet:  cctlw, hrdw_p, wordcount, and completion  */
     /* interrupt request word (always 0).  Clear the "more packets coming", */
     /* "scan module", and "scan crate" bits.  Set the "more packets coming" */
     /* bit in the preceding packet if any.  Increment count of packets      */
     /* built in this package, and return.                                   */
 
     CAMBLK_p->mbcd_pkt[iop].cctlw = *cctlw_p & ~(CCTLW__MPC | CCTLW__SM | CCTLW__SC);
     CAMBLK_p->mbcd_pkt[iop].stad_p = savep_p[iop].hrdw_p;
     CAMBLK_p->mbcd_pkt[iop].wc_max = wcnt;

     if (iop > 0)
         CAMBLK_p->mbcd_pkt[iop-1].cctlw |= CCTLW__MPC;
     ++CAMBLK_p->hdr.iop;
 egress:
     return iss;
 
     #undef CAMBLK_p
 }                                                             /* End camadd. */
