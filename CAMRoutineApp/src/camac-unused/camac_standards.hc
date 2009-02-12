/*   **CMS**=C_INC   */

/*==============================================================================

  Abs:  Standard CAMAC parameter include file

  Name: CAMAC_STANDARDS.HC

  Reference:  CAMAC Primer 
              Author: Peter Clout
              Los Alamos National Laboratory    
              LA-UR-82-2718

  Prev: required include files: 
            slctxt:slc_macros.h

  Auth: 17-Mar-1992, K. Luchini (LUCHINI)
  Rev:  05-Aug-1992, S. Levitt (LEVITT)

--------------------------------------------------------------------------------

  Mod:
        24-Apr-1997, K. Luchini (LUCHINI):
           Change function code and subaddress array to int2u to 
           correspond to camfag prototype change.
        19-Apr-1995, K. Luchini (LUCHINI)
           modify CAMAC_FUNCTION_CODES and CAMAC_ADDRESSES
           to int4u to int2 to correspond with camfag() and 
           camfag1() where function and subaddress is passed as shorts
        06-Sep-1994, K. Luchini (LUCHINI)
           Added separate defines for A0-A31
           and populated subaddress array A[]         
        21-Sep-1992, K. Luchini (LUCHINI)
           moved controller specific params to camac_param.hc

==============================================================================*/

#ifndef CAMAC_STANDARDS_HC  /* guard macro */
#define CAMAC_STANDARDS_HC  /* guard macro */



/* 
    Standard CAMAC Function Code indexes
    Note:function codes 4,6,12,14,28,30 are nonstandard
         function codes 5,7,13,15,29,31 are reserved 
*/
#define         RD1        0        /* read group 1 register                */
#define         RD2        1        /* read group 2 register                */
#define         RC1        2        /* read & clear group 1 register        */
#define         RCM        3        /* read complement of group 1 register  */ 
#define         NONSTD_4   4
#define         RESV_5     5
#define         NONSTD_6   6
#define         RESV_7     7
#define         TLM        8        /* test LAM                             */
#define         CL1        9        /* clear group 1 register               */ 
#define         CLM       10        /* clear LAM                            */
#define         CL2       11        /* clear group 2 register               */
#define         NONSTD_12 12        /* nonstandard function code            */
#define         RESV_13   13        /* reserved function code               */
#define         NONSTD_14 14        /* nonstandard function code            */
#define         RESV_15   15        /* reserved function code               */
#define         WT1       16        /* overwrite group 1 register           */
#define         WT2       17        /* overwrite group 2 register           */
#define         SS1       18        /* selective set group 1 register       */
#define         SS2       19        /* selective set group 2 register       */
#define         BS2       20        /* Nonstandard--bit set group 2 register*/
#define         SC1       21        /* selective clear group 1 register     */
#define         BC2       22        /* nonstandard--bit clear group 2 register */
#define         SC2       23        /* selective clear group 2 register     */
#define         DIS       24        /* disable                              */
#define         XEQ       25        /* execute                              */
#define         ENB       26        /* enable                               */
#define         TST       27        /* test status                          */
#define         NONSTD_28 28        /* nonstandard function code            */
#define         RESV_29   29        /* reserved function code               */
#define         NONSTD_30 30        /* nonstandard function code            */
#define         RESV_31   31        /* reserved function code               */


/* 
    Standard CAMAC Functions Codes
*/
#define CAMAC_FUNCTION_CODES \
        const int2u F[32]={  RD1,RD2,RC1,RCM,\
                             NONSTD_4,RESV_5,NONSTD_6,RESV_7,\
                             TLM,CL1,CLM,CL2,\
                             NONSTD_12,RESV_13,NONSTD_14,RESV_15,\
                             WT1,WT2,SS1,SS2,BS2,SC1,BC2,\
                             SC2,DIS,XEQ,ENB,TST,\
                             NONSTD_28,RESV_29,NONSTD_30,RESV_31 }


/* 
    Standard CAMAC subaddresses 
*/
#define A0  0
#define A1  1
#define A2  2
#define A3  3
#define A4  4
#define A5  5
#define A6  6
#define A7  7
#define A8  8
#define A9  9
#define A10 10
#define A11 11
#define A12 12
#define A13 13
#define A14 14
#define A15 15
#define A16 16
#define A17 17
#define A18 18
#define A19 19
#define A20 20
#define A21 21
#define A22 22
#define A23 23
#define A24 24
#define A25 25
#define A26 26
#define A27 27
#define A28 28
#define A29 29
#define A30 30
#define A31 31
#define CAMAC_SUBADDRESS \
        const int2u A[32]={ A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,\
                            A11,A12,A13,A14,A15,A16,A17,A18,A19,A20,\
                            A21,A22,A23,A24,A25,A26,A27,A28,A29,A30,\
                            A31}

#endif  /* CAMAC_STANDARDS_HC guard macro */
