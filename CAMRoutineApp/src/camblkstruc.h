 /*===========================================================================*/
 /*                                                                           */
 /*      **CMS**=RMX_INCLUDE                                                  */
 /*                                                                           */
 /* Abs:  This file defines the internal structure of the MBCD package in     */
 /*       the micro's.                                                        */
 /*                                                                           */
 /* Name: slcrmxi:camblkstruc.h.                                             */
 /*                                                                           */
 /* Prev: None.                                                               */
 /*                                                                           */
 /* Auth: 24-Mar-1992, T. Gromme  (Made from CAMBLKSTRUC.PNC)                 */
 /* Revw:                                                                     */
 /*                                                                           */
 /*---------------------------------------------------------------------------*/
 /*                                                                           */
 /* Mod:  15-Jul-2008, Robert C. Sass (RCS)                                   */
 /*           Change for PSCD card.                                           */
 /*       17-Jun-1996, A. Hunter (AFH):                                       */
 /*           Changes required for iC386.                                     */
 /*                                                                           */
 /*===========================================================================*/
 
 
#ifndef CAMBLKSTRUC_HM
 
 
             /*  Nops (specified in the call to CAMALO, CAMALOM or CAMALH) is */
             /*  the maximum number of packets that CAMADD can build  */
             /*  in the given package (without intervening calls to   */
             /*  CAMALO_RESET).  The packets themselves immediately   */
             /*  follow the package header (mbcd_pkghdr_ts).  In      */
             /*  order to be executable by the MBCD, the first packet */
             /*  of the package must begin on a physical byte address */
             /*  that is evenly divisible by sixteen;  that is why    */
             /*  the package header is exactly sixteen bytes long.    */
             /*  The array of mbcd_savep_ts's immediately follows the */
             /*  space for the last packet according to nops.         */
             /*  Automatic buffering of the user's status/data is     */
             /*  always done. When automatic buffering is being done, */
             /*  then user_p contains the saved pointer to the user's */
             /*  area.hrdw_p points to the status/data area directly  */
             /*  accessed by the MBCD.                                */

 
                          /*  Structure of a single MBCD packet.      */
 typedef struct
 {   unsigned long  cctlw;        /* MBCD control word.               */
     void          *stad_p;       /* pointer to status/data.          */
     unsigned short wc_max,       /* Max wordcount.                   */
                   notused;       /* So consistent with old docs      */
 } mbcd_pkt_ts;
 
                          /*  Structure of a single entry in list of    */
                          /*  saved pointers and emask's.               */
 typedef struct
 {   void          *hrdw_p;       /* pointer to dual ported mem status/data. */
     void          *user_p;       /* Possible pointer to user's status/data. */
     unsigned short emask;        /* Error conditions to test.               */
 } mbcd_savep_ts;
 
                     /* Structure of header portion of MBCD package.          */
                     /* Packages can be built to execute on the Low, Medium   */ 
                     /* or Hi priority ports. This goes into the validation   */
                     /* key of the package header.                            */
#define KEY_HI  0
#define KEY_MED 1
#define KEY_LOW 2
                     /* > KEY_LOW are various invalid codes. Need short so no enum */
#define KEY_BAD_OV 3
#define KEY_BAD_OD 4
#define KEY_BAD_BC 5
#define KEY_BAD_BM 6
#define KEY_BAD_NH 7

typedef struct
 {   unsigned short key,         /* Validation key.                   */
                    nops,        /* Max # of packets.                */
                    iop,         /* Current # of packets.            */
                    tbytes,      /* ?? */
                    bitsummary,
                    spare;
     void          *pkg_p;       /* pointer in dual ported mem to 1st packet. */
} mbcd_pkghdr_ts; 
                          /*  Structure of an MBCD package.  */
 typedef struct
 {   mbcd_pkghdr_ts hdr;
     mbcd_pkt_ts    mbcd_pkt[1];
 } mbcd_pkg_ts;
 
/* Definition of camac package "token" type (32 bits, not 16 bits).   */
/* Weaker version of mbcdpkg_ts is provided by cam_proto.h & certain */
/* others when this includefile is absent.                            */ 

#ifndef CAMPKGP_TDEF
 typedef mbcd_pkg_ts *campkgp_t;
#define CAMPKGP_TDEF
#endif
 
                          /*  Structure of an MBCD status/data block.  */
 typedef struct
 {   
   unsigned int  camst_dw;
   unsigned short dat_w[1];
 } mbcd_stad_ts;

#define CAMBLKSTRUC_HM
#endif
