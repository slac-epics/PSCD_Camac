/*   **CMS**=C_INC   */

/*==============================================================================

  Abs:  CAMAC Control Specific Definitions. For further information see
        chapter 9 of the Basic Users Guide.

  Name: camac_types.hc

  Prev: required include file:
           slctxt: slc_macr.hc       ( int4, int2 ...)

  Auth: 13-AUG-1992, K. Underwood (KKU)
  Rev:  13-AUG-1992, D. Vanolst (VANOLST)

--------------------------------------------------------------------------------

  Mod:
        27-Apr-1997, Kristi Luchini (LUCHINI):
            chg elements in emask_tu,mbcd_stat_tu,ctlw_tu to unsigned
        19-Apr-1995, Kristi Luchini (LUCHINI):
            added _a[2] and to emask_tu and mbcd_stat_tu
        25-Jun-1993, Karey Krauter (KEK)
            added CAMAC_SHIFT_* for shifting the ctlw components to
            the least signifigant bits (esp _C and _N for passing
            into build_ctlw).
        22-Sep-1992, K. Luchini (LUCHINI)
            added typedef unions ctlw_tu, emask_tu, mbcd_stat_tu
            removed camac_header_ts ...similar typedef in camblkstruc.hm 
        21-Sep-1992, K. Luchini (LUCHINI)
            added camac_header_ts
        08-SEP-1992, K. Underwood (KKU)
            add sub-address, station, crate, and function code masks
        03-SEP-1992, K. Underwood (KKU)
            added stat and emask bit field structures

==============================================================================*/

#if !defined(CAMAC_TYPES_HC)
#define CAMAC_TYPES_HC

/* Define CTLW masks, primarily useful in CAMMOD calls */

#define CAMAC_MASK_A   0x0000000F /* sub-address mask */
#define CAMAC_MASK_RPK 0x00000010 /* repack mask */
#define CAMAC_MASK_P8  0x00000020 /* pack 8 bit mask */
#define CAMAC_MASK_N   0x00000F80 /* station address mask */
#define CAMAC_MASK_C   0x0000F000 /* crate address mask */
#define CAMAC_MASK_F   0x001F0000 /* function code mask */
#define CAMAC_MASK_SA  0x00200000 /* enable sub-address counter mask */
#define CAMAC_MASK_SN  0x00400000 /* enable station address counter mask */
#define CAMAC_MASK_SC  0x00800000 /* enable crate address counter mask */
#define CAMAC_MASK_ILQ 0x01000000 /* increment enabled counters mask */
#define CAMAC_MASK_IN  0x02000000 /* increment next counter mask */
#define CAMAC_MASK_P24 0x04000000 /* pack 24 bit mask */
#define CAMAC_MASK_QM2 0x08000000 /* terminate packet if Q==0 mask */
#define CAMAC_MASK_QM1 0x10000000 /* transfer data if Q==1 mask */
#define CAMAC_MASK_XM2 0x20000000 /* terminate packet if X==0 mask */
#define CAMAC_MASK_XM1 0x40000000 /* transfer data if X==1 mask */
#define CAMAC_MASK_MPC 0x80000000 /* more packets coming mask */

/* Define CTLW shift values */

#define CAMAC_SHIFT_A           0 /* sub-address mask */
#define CAMAC_SHIFT_RPK         4 /* repack mask */
#define CAMAC_SHIFT_P8          5 /* pack 8 bit mask */
#define CAMAC_SHIFT_N           7 /* station address mask */
#define CAMAC_SHIFT_C          12 /* crate address mask */
#define CAMAC_SHIFT_F          16 /* function code mask */
#define CAMAC_SHIFT_SA         21 /* enable sub-address counter mask */
#define CAMAC_SHIFT_SN         22 /* enable station address counter mask */
#define CAMAC_SHIFT_SC         23 /* enable crate address counter mask */
#define CAMAC_SHIFT_ILQ        24 /* increment enabled counters mask */
#define CAMAC_SHIFT_IN         25 /* increment next counter mask */
#define CAMAC_SHIFT_P24        26 /* pack 24 bit mask */
#define CAMAC_SHIFT_QM2        27 /* terminate packet if Q==0 mask */
#define CAMAC_SHIFT_QM1        28 /* transfer data if Q==1 mask */
#define CAMAC_SHIFT_XM2        29 /* terminate packet if X==0 mask */
#define CAMAC_SHIFT_XM1        30 /* transfer data if X==1 mask */
#define CAMAC_SHIFT_MPC        31 /* more packets coming mask */

/* Define the CAMAC ConTroL Word structure */

typedef struct
{
    unsigned A   : 4; /* module sub address */
    unsigned RPK : 1; /* repack data */
    unsigned P8  : 1; /* 8 bit pack mode */
    unsigned     : 1; /* unused */
    unsigned N   : 5; /* station address */
    unsigned C   : 4; /* crate address */
    unsigned F   : 5; /* function code */
    unsigned SA  : 1; /* enable sub address counter */
    unsigned SN  : 1; /* enable station address counter */
    unsigned SC  : 1; /* enable crate counter */
    unsigned ILQ : 1; /* increment enabled counters if Q==0 */
    unsigned IN  : 1; /* increment next counter if X==0 */
    unsigned P24 : 1; /* 24 bit pack mode */
    unsigned QM2 : 1; /* terminate packet if Q==0 */
    unsigned QM1 : 1; /* transfer data if Q==1 */
    unsigned XM2 : 1; /* terminate packet if X==0 */
    unsigned XM1 : 1; /* transfer data if X==1 */
    unsigned MPC : 1; /* more packets coming */
} ctlw_ts;

typedef union
{
  ctlw_ts _s;
  int4u   _i;
} ctlw_tu;


/* Define the CAMAC status word structure */

typedef struct
{
    unsigned WC  : 14; /* remaining word count */
    unsigned LAM :  1; /* LAM asserted */
    unsigned SHE :  1; /* summary hardware error */
    unsigned Q   :  1; /* Q on last CAMAC cycle */
    unsigned X   :  1; /* X on last CAMAC cycle */
    unsigned EMS :  1; /* packet terminated by QM2 or XM2 */
    unsigned EOS :  1; /* packet terminated by counter overflow */
    unsigned BAR :  1; /* terminated by remaining word count */
    unsigned CTO :  1; /* crate timeout */
    unsigned DNE :  1; /* CAMAC package is complete */
    unsigned N   :  5; /* station address */
    unsigned C   :  4; /* crate address */
} stat_ts;


typedef union 
{
  stat_ts  _s;
  int4u    _i;
  int2u    _a[2]; 
} mbcd_stat_tu;

/* Define the CAMAC error mask structure */

struct emask_s
{
    unsigned Q   :  1; /* no Q response */
    unsigned X   :  1; /* no X reponse */
    unsigned EMS :  1; /* did not terminate on QM2 or XM2 */
    unsigned EOS :  1; /* did not terminate on counter overflow */
    unsigned BAR :  1; /* did not terminate on word count */
    unsigned CTO :  1; /* crate timeout */
    unsigned STO :  1; /* software timeout */
    unsigned SHE :  1; /* summary hardware error */
};

typedef struct
{
    struct emask_s warn; /* selects conditions that issue warning messages */
    struct emask_s code; /* selects conditions to indicate in the return code */
} emask_ts;

typedef union
{
  emask_ts          _s;
  unsigned short    _i;
  unsigned char     _a[2];
} emask_tu;


#endif /* CAMAC_TYPES_HC */
