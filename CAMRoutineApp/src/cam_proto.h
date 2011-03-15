/*==============================================================================
 
  Abs:  Function prototypes for cam* routines in SLC micro's.
 
  Name: cam_proto.h.
 
  Prev: slc_macros.h               vmsstat_t.
        camblkstruc.h              (Optional) CAMPKGP_TDEF, campkgp_t.
 
  Rem:  This includefile is used only for compiling code that will run in FECC's
        host PC, not in FECC itself.
 
  Auth: 09-July-1990   R. Sass (RCS & TEG)
 
--------------------------------------------------------------------------------
 
  Mod:  15-Jul-2008, Robert C. Sass (RCS)
            Modify for PSCD card.
        03-Jan-2006, TEG: For conversion to FLAT linkage model + FECC3:  define
                          type for camac package "token";  eliminate camaloi,
                          campool*, and cam_tok_to_sio;  add camaloh.
        09-Jun-1999, TEG: Add campooli and a few const's;  eliminate campoolcps.
        14-Nov-1997, LUCHINI: Add camac_build_ctlw and decode_control_word.
        28-Jul-1997, RCS: Add campoolf.
        24-Mar-1993, RCS: Add campoolcps.
        30-Jul-1990, RCS: Delete nested includes.
 
==============================================================================*/
 
#ifndef CAM_PROTO_HM
 
 /* Definition of camac package "token" type (campkgp_t = 32 bits, not */
 /* 16 bits).  Stronger version provided by camblkstruc.h if latter   */
 /* is included and precedes any other includefile offering definition */
 /* of campkgp_t.                                                      */

#ifndef CAMPKGP_TDEF
 typedef void *campkgp_t;
#define CAMPKGP_TDEF
#endif
 
 /* Procedures camaloh and camalol are identical to camalo except that they */
 /* imply high and low priority on subsequent executions of the package     */
 /* by camgo. Procedure camio uses camalo not camaloh.                      */
 /* Procedure cam_ini must be called once before any of the others          */
 /* Procedure camrst is like camalo_reset except no memory is deleted       */
 
 vmsstat_t camadd(const unsigned int *cctlw_p, const void *stad_p, 
                  const unsigned short *bcnt_p, const unsigned short *emask_p,
                  campkgp_t *camblk_pp);
 
 vmsstat_t camalo(const unsigned short *nops_p, campkgp_t *camblk_pp);
 
 vmsstat_t camaloh(const unsigned short *nops_p, campkgp_t *camblk_pp);
 
 vmsstat_t camalol(const unsigned short *nops_p, campkgp_t *camblk_pp);
 
 vmsstat_t camalo_reset(campkgp_t *camblk_pp);

 vmsstat_t camrst(campkgp_t *camblk_pp);
 

 vmsstat_t camdel(campkgp_t *camblk_pp);
 
 vmsstat_t camgo(const campkgp_t *camblk_pp);
 
 vmsstat_t camio(const unsigned int *cctlw_p, void *datau_p,
                 const unsigned short *bcnt_p, void *statu_p, const unsigned short *emask_p);
 
 vmsstat_t cam_ini(void*);
 
 vmsstat_t camioi(void);
 
 vmsstat_t cammod(const unsigned int *cctlw_p, const unsigned int *cctlw_mask_p,
                  const campkgp_t *camblk_pp);
 
 char     *cammsg(vmsstat_t iss);

 vmsstat_t camgo_start_pscd(const campkgp_t *camblk_pp, int wait);
 
 void camgo_get_data(const campkgp_t *camblk_pp);

#ifndef _X86_

void camSwapBytes  (void *buf_p, int bsize);
void camSwapWords (void *buf_p, int wsize);

#endif
 
#define CAM_PROTO_HM
#endif
 
