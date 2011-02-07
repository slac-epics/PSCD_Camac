/*=====================================================================
**
** The phasePIOPproc routine converts the raw phase from the PIOP status
** word.
**
**=====================================================================
*/

#include <devPIOP.h>

#include <epicsExport.h>
#include <registryFunction.h>

static const float small_r_value = 1.0E-9;

/**************************************************
 *** Local support routines for this module *******
 **************************************************

phasePIOPvalid  -  return validated phase
phasePIOPmod    -  our verion of klys_mod
phasePIOPpoly   -  dbpoly; compute f(x)

*************************************************/

/**************************************************
  Our version of phase_valid.

  Selected comments from  RMX_KLYS:PHASEVALID.C86

  This routine merely makes sure that the given first argument floating
  value is larger than or equal to in absolute value the very small value
  1.E-09, and then copies the low-order two bits of the second argument
  floating mantissa over the low-order two bits of the first argument
  floating mantissa.

  For very small numbers, we subsitute 1.0E-9, since closer to ZERO
  the PAD cannot read.

  for IEEE s<---exp--><-------mantissa--------->WV  Wobbled, Valid
  Paste the wobble and valid bits from wobble to number.

 ***************************************************/

#define WOBBLE_VALID_BITS 0x03
static float phasePIOPvalid (float number, float wobble)
{
    float value = number;
    char *vc = (char *) &value;  /* value as char* */
    char *wc = (char *) &wobble; /* wobble as char* */
   /*--------------------*/
    if (value >= 0.0)
    {
        if (value <  1.0E-9)
            value =  1.0E-9;
    }
    else /* is negative */
    {
        if (value > -1.0E-9)
            value = -1.0E-9;
    }
    /*
    ** Or in the wobble and valid bits from the wobble number
    ** and return as the value
    */
#ifndef _X86_
    vc[3] = (vc[3] & ~WOBBLE_VALID_BITS) | (wc[3] & WOBBLE_VALID_BITS);
#else
    vc[0] = (vc[0] & ~WOBBLE_VALID_BITS) | (wc[0] & WOBBLE_VALID_BITS);
#endif
    return (value);
}

/**************************************************
*  Our version of klys_mod.
*
* Routine returns a value divisor/2. <= value < divisor/2.
*
* The "Wobble" and the "Valid" bits in the phase information
* are carefully left in the result.
*
* for IEEE s<---exp--><-------mantissa--------->WV  Wobbled, Valid
*
************************************************/
static float phasePIOPmod(float value, float divisor) 
{
    float r_2;
    /*----------------*/
    if ((value + divisor/2.0) >= 0)
    {
        r_2 = - divisor/2. + fmod(value + divisor/2., divisor);
    }
    else 
    {
        r_2 =   divisor/2. + fmod(value + divisor/2., divisor);
    }
    r_2 = phasePIOPvalid(r_2, value);
    return (r_2);
}

/*********************************************************
*  Version of dbpoly copied from Mike Z's implementation
*  on the soft IOC adapted from RMX dbpoly

;  Method:
;    Suppose N is the degree + 1, and the A's are the coefficients,
;    where A(1) is the constant term and A(N) is the highest-order
;    coefficient.  Then the algorithm is:
;
;        DF_DX = 0.
;        F = A(N)
;        DO K = N-1,1,-1
;          DF_DX = DF_DX*X + F
;          F = F*X + A(K)
;        ENDDO
;
;    Upon exit, F = value of polynomial at X, and
;               DF_DX = value of polynomial's 1st derivative at X.
************************************************/

static void phasePIOPpoly(float x, float coeff[], short cnt, float *fx, float *dfx)
{
    int j;
    float f;
    float df;
    /*------------------*/
    f = coeff[cnt-1];
    df = 0;

    for (j=cnt-2; j>=0; j=j-1) 
    {
        df = df * x + f;
        f = f * x + coeff[j];
    }
    *fx = f;
    *dfx = df;
}

/*******************************************
 ** Routines referenced by the sub record **
 *******************************************/
 
long phasePIOPinit (struct subRecord *subr_p)
{
    return (0);
}

/**************************************************************
 ************ Main Phase conversion routine *******************
 ******* Adapted from RMX_KLYS:KLYS_PHASE_CONVERT.C86 *********

 We assume the following inputs to the SUB record:
 a   = raw RT phase from ai record
 b   = gold offset
 c-h = 6 polynomial coefficients
 *************************************************************/

#define PHASE_CONSTANT (360.0/32768.0)  /* convert from counts to degrees */
#define PHASE_360 360.0                 /* 360 degrees */
#define PHASE_180 180.0                 /* 180 degrees */
#define PHASE_90  90.0                  /* 90 degrees */
#define WOBBLE_BIT 2                    /* check the wobble bit */

long phasePIOPproc (struct subRecord *subr_p)
{
    unsigned int rawphase = subr_p->a;  /* Get raw phase from input */ 
    float goldmstr        = subr_p->b;  /* Gold offset */
    float polycoeff[6]    = 
       {subr_p->c, subr_p->d, subr_p->e, subr_p->f, subr_p->g, subr_p->h};
    float temp_phase;           /* intermediate phase result */
    float praw;                 /* float/mod Raw phase from Ai input */
    float lppad, ldppad;        /* locally calculated & derivative */
    float sppad, sdppad;        /* KLYSTATUS ppad & derivative */
    float sphase;               /* Phase as calculated by KLYSTATUS.F86 */
    float temp2_phase;          /* intermediate phase result */
    int4u wobbled;              /* wobble bits to check */
    /*--------------------------------------------*/
    /*
    ** Convert to degrees, modulate between +- 180 deg
    ** and check the wobble bit in the raw data.
    */
    temp_phase=PHASE_CONSTANT * rawphase;
    praw = phasePIOPmod (temp_phase, PHASE_360);
    wobbled = (rawphase & WOBBLE_BIT);
    if (wobbled != 0)
        temp_phase = praw + PHASE_180;
    else
        temp_phase = praw;
    /*
    ** the angle +183.0 (unwobbled) has been represented as -177.  Likewise
    ** for -183.0 (wobbled). Force the over-extended goodies back where they
    ** belong.  in approx range of 0 ~<= R_2 ~<= 180.
    */
    temp2_phase = temp_phase - PHASE_90;
    temp_phase = PHASE_90 + phasePIOPmod(temp2_phase,PHASE_360);
    phasePIOPpoly(temp_phase, polycoeff, 6, &lppad, &ldppad);
    if (wobbled != 0)
    {
        lppad = lppad - PHASE_180;
    }
    lppad = phasePIOPmod(lppad, PHASE_360);
    /*
    ** Now duplicate the logic in KLYSTATUS.F86 that calcs PPAD and PHAS using
    ** the gold offset.
    */
    phasePIOPpoly(praw, polycoeff, 6, &sppad, &sdppad);
    sphase = phasePIOPmod (sppad - goldmstr, PHASE_360);
    /*
    ** Finish the calc as in KLYS_PHASE_CONVERT.F86
    */
    temp_phase = lppad + sphase - sppad;
    subr_p->val = phasePIOPmod(temp_phase, PHASE_360);
    return (0);
}

epicsRegisterFunction(phasePIOPinit);
epicsRegisterFunction(phasePIOPproc);
