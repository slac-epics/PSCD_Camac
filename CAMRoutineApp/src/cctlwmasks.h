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
 
/* 
 * crate limits
*/
#define MIN_CRATE_ADR    1        /* minimum crate address                  */
#define MAX_CRATE_ADR   15        /* maximum crate address                  */
#define MAX_CRATE_SLOT  25        /* maximum crate slot number              */
#define MIN_CRATE_SLOT   1        /* minimum crate slot number              */
 
/*
 *  Masks and shift counts for bits and bit fields in
 *  camac control longword portion of MBCD packet.
 * 
 * ----------------------------------------------------------------------------
 * camac control word bits.
 * CTLW = MXXQ QPII SSSF FFFF CCCC MMMM MxPR AAAA
 *        PMMM M2NL CNA1 8421 8421 1842 1 8P 8421
 *        C121 24 Q    6           6       K
 * MPC = zero if last packet of package; otherwise more are coming.
 * XM1 = transfer data if X=1, else retry
 * XM2 = quit this packet if X=0
 * QM1 = transfer data if Q=1, else retry
 * QM2 = quit this packet if Q=0
 * P24 = 24-bit pack mode (see BUG section 9.1.1.2)
 * IN  = reset least significant enabled scan counter and increment most
 * ILQ = increment enabled counters only if Q=0
 * SC  = enable crate counter
 * SN  = enable station address counter (aka module or slot)
 * SA  = enable module subaddress counter
 * (for exact meaning of ctlw bits, see h/w manual for specific device)
 * F16 = ctlw bit: write operation
 * F17 = ctlw bit: read operation
 * F8  = ctlw bit: nondata xfer operation
 * F4  = ctlw bit:
 * F2  = ctlw bit: read and reset operation
 * F1  = ctlw bit:
 * C8, C4, C2, C1  =    (crate address).  Can't be zero.
 * M16, M8, M4, M2, M1 =    (Station/module/slot address). Can't be zero.
 * x   = not used
 * P8  = software-effect only 8-bit pack mode (see BUG section 9.1.1.2)
 * REPACK = repack data into smaller blocks.
 * A8, A4, A2, A1  =     (module subaddress).
 */
 
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
/*
** Shifts
*/ 
#define CCTLW__F_shc  16
#define CCTLW__C_shc  12
#define CCTLW__M_shc   7
#define CCTLW__A_shc   0

/*
 * ---------------------------------------------------------------
 * camac packet completion status bits
 * STAT = CCCC MMMM MDCB EEXQ SLWW WWWW WWWW WWWW
 *        8421 1842 1NTA OM   HA-- ---- ---- ----
 *             6     EOR SS   EM
 * C8, C4, C2, C1  =    (crate address).  Can't be zero unless there was an
 *                                        and camgo did nothing..
 * M16, M8, M4, M2, M1 =    (Station/module/slot address). Can't be zero,
 *                                        except as above
 * DNE = set when all packets have executed
 * CTO = set when camac crate timed out
 * BAR = set when word count exhausted
 * EOS = set when scan exceeded scan counter
 * EMS = set if packet terminated with X=0 or Q=0
 * X   = set if camac module received and liked the command
 * Q   = set if camac module successfully executed the command
 * SHE = set if MBCD problem
 * LAM = set if camac module is requesting attention (Look At Me)
 * WW WWWW WWWW WWWW = remaining word count of data not processed
 */
 
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

/*
 * ---------------------------------------------------------------
 * camac packet emask error-reporting bits
 * EMASK= SSCB EEXQ SSCB EEXQ 
 *        HTTA OM   HTTA OM
 *        EOOR SS   EOOR SS
 * The LSB of EMASK selects errors for which warning messages are
 * issued.  The MSB of EMASK selects conditions for which errors are
 * indicated in the return code.
 * SHE = MBCD summary hardware errors
 * STO = MBCD software timeout
 * CTO = camac crate hardware timeout
 * BAR = did not terminate on exhausted wordcount
 * EOS = did not terminate on end-of-scan (most significant counter overflow)
 * EMS = did not terminate on end-of-mode (xm2/qm2 conditions)
 * X   = no x reponse; camac crate didn't like command
 * Q   = no q response; camac crate couldn't execute command
 */
#define EMASK_HI_SHE 0x8000
#define EMASK_HI_STO 0x4000
#define EMASK_HI_CTO 0x2000
#define EMASK_HI_BAR 0x1000
#define EMASK_HI_EOS 0x0800
#define EMASK_HI_EMS 0x0400
#define EMASK_HI_X   0x0200
#define EMASK_HI_Q   0x0100
#define EMASK_LO_SHE 0x0080
#define EMASK_LO_STO 0x0040
#define EMASK_LO_CTO 0x0020
#define EMASK_LO_BAR 0x0010
#define EMASK_LO_EOS 0x0008
#define EMASK_LO_EMS 0x0004
#define EMASK_LO_X   0x0002
#define EMASK_LO_Q   0x0001
 
#define CCTLWMASKS_HM
#endif
