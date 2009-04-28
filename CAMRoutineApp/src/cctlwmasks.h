 /*===========================================================================*/
 /*                                                                           */
 /*      **CMS**=RMX_INCLUDE                                                  */
 /*                                                                           */
 /* Abs:  Definitions of bits and fields in MBCD (Multibus camac driver)      */
 /*       control longword (cctlw) and in packet completion status longword   */
 /*       generated and stored by MBCD.                                       */
 /*                                                                           */
 /* Name: slcrmxi:cctlwmasks.h.                                              */
 /*                                                                           */
 /* Prev: None.                                                               */
 /*                                                                           */
 /* Auth:                                                                     */
 /* Revw:                                                                     */
 /*                                                                           */
 /*---------------------------------------------------------------------------*/
 /*                                                                           */
 /* Mod:  09-Jun-1999, Tony Gromme (TEG):                                     */
 /*           Translated from PLM386 to iC386.                                */
 /*                                                                           */
 /*===========================================================================*/
 
 
#ifndef CCTLWMASKS_HM
 
 
 /*  Masks and shift counts for bits and bit fields in  */
 /*  camac control longword portion of MBCD packet.     */
 
#define CCTLW__MPC       0x080000000
#define CCTLW__XM1       0x040000000
#define CCTLW__XM2       0x020000000
#define CCTLW__QM1       0x010000000
#define CCTLW__QM2       0x008000000
#define CCTLW__P24       0x004000000
#define CCTLW__IN        0x002000000
#define CCTLW__ILQ       0x001000000
#define CCTLW__SC        0x000800000
#define CCTLW__SM        0x000400000
#define CCTLW__SA        0x000200000
#define CCTLW__F         0x0001F0000
#define CCTLW__F27       0x0001B0000
#define CCTLW__F26       0x0001A0000
#define CCTLW__F24       0x000180000
#define CCTLW__F21       0x000150000
#define CCTLW__F19       0x000130000
#define CCTLW__F18       0x000120000
#define CCTLW__F17       0x000110000
#define CCTLW__F16       0x000100000
#define CCTLW__F9        0x000090000
#define CCTLW__F8        0x000080000
#define CCTLW__F4        0x000040000
#define CCTLW__F3        0x000030000
#define CCTLW__F2        0x000020000
#define CCTLW__F1        0x000010000
#define CCTLW__F0        0x000000000
#define CCTLW__C         0x00000F000
#define CCTLW__M         0x000000F80
#define CCTLW__P8        0x000000020
#define CCTLW__RE_PACK   0x000000010
#define CCTLW__A         0x00000000F
 
#define CCTLW__F_shc  16
#define CCTLW__C_shc  12
#define CCTLW__M_shc   7
#define CCTLW__A_shc   0

/*
** Low order CAMAC control word bits
*/
#define CCTLW__C8        0x00008000
#define CCTLW__C4        0x00004000
#define CCTLW__C2        0x00002000
#define CCTLW__C1        0x00001000
#define CCTLW__M16       0x00000800
#define CCTLW__M8        0x00000400
#define CCTLW__M4        0x00000200
#define CCTLW__M2        0x00000100
#define CCTLW__M1        0x00000080
#define CCTLW__A8        0x00000008
#define CCTLW__A4        0x00000004
#define CCTLW__A3        0x00000003
#define CCTLW__A2        0x00000002
#define CCTLW__A1        0x00000001
#define CCTLW__A0        0x00000000
 
 /*  Masks for bits and bit fields in camac status longword  */
 /*  generated and stored by MBCD.                           */
 
#define MBCD_STAT__C     0x0F0000000
#define MBCD_STAT__M     0x00F800000
#define MBCD_STAT__DNE   0x000400000
#define MBCD_STAT__CTO   0x000200000
#define MBCD_STAT__BAR   0x000100000
#define MBCD_STAT__EOS   0x000080000
#define MBCD_STAT__EMS   0x000040000
#define MBCD_STAT__X     0x000020000
#define MBCD_STAT__Q     0x000010000
#define MBCD_STAT__SHE   0x000008000
#define MBCD_STAT__LAM   0x000004000
#define MBCD_STAT__RWC   0x000003FFF
 
#define CCTLWMASKS_HM
#endif
