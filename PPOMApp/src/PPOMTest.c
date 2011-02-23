/***************************************************************************\
 *   $Id: PPOMTest.c,v 1.2 2009/11/16 12:40:23 pengs Exp $
 *   File:		PPOMTest.c
 *   Author:		Sheng Peng
 *   Email:		pengsh2003@yahoo.com
 *   Phone:		408-660-7762
 *   Company:		RTESYS, Inc.
 *   Date:		09/2009
 *   Version:		1.0
 *
 *   EPICS driver for PPOM 
 *
\***************************************************************************/
#include "drvPSCDLib.h"
#include "devPPOM.h"
#include "slc_macros.h"
#include "cam_proto.h"
#include <registryFunction.h>

#if 0
       The format of the CAMAC control word, CTLW is as follows:
       (ref: SLC Basic Users Guide (BUG) for additional information)
       http://www.slac.stanford.edu/grp/cd/soft/wwwman/bug.www/chapter9.html

       Bits 0-3 = AAAA = Module subaddress.
              4 = RE_PACK = Re_pack data into smaller blocks.
              5 = P8   = 8-bit pack mode.  (See Pack Modes below).
              6 =  x   = Not used (yet).
           7-11 = NNNNN= Station address (slot)
          12-15 = CCCC = Crate address

          16-20 = FFFFF= CAMAC function code.
          21-23 = SSS  = Enables scan mode counters.
                         (See Scan Modes below.)
             21 = SA   = Enable subaddress counter.
             22 = SN   = Enable station address counter.
             23 = SC   = Enable crate counter.

          24-25 = II   = Scan Increment mode (see Scan Modes below).
             24 = ILQ  = Increment enabled counters only if Q=0.
             25 = IN   = Reset least significant enabled counter and
                         increment next most significant enabled
                         counter if X=0.
             26 = P24  = 24-bit pack mode. (See Pack Modes below).
          27-30 = XXQQ = Transfer/terminate mode (See Scan modes
                         below.)
             27 = QM2  = Terminate packet if Q=0.
             28 = QM1  = Transfer data if Q=1.
             29 = XM2  = Terminate packet if X=0.
             30 = XM1  = Transfer data if X=1.
             31 = MPC  = 0 if last packet of package; otherwise More
                                Packets Coming.
#endif

extern struct PSCD_CARD pscd_card;

typedef struct STAS_DAT
{
    UINT32 stat;
    UINT32 data;
} STAS_DAT;

UINT32 PPOM_Test()
{

    UINT32 rtn = 0;

    /* check if module exists */
    if(isModuleExsit(0, 5, 21))
    {

        void *pkg_p;  /* A camac package */
        vmsstat_t iss;
        UINT16 bcnt = 4;
        UINT16 emask= 0xE0E0;


        UINT32 ctlwF0A0 = 0x00005A80;

        STAS_DAT read_ppom[2];
        UINT16 nops = 0;

        nops = 1;
 
        /** Allocate package for PPOM reset */

        if (!SUCCESS(iss = camalol (&nops, &pkg_p)))
        {
            errlogPrintf("camalol error %s\n",cammsg(iss));
            rtn = (PPOM_CAM_ALLOC_FAIL|iss);
            goto egress;
        }

        read_ppom[0].data = 0;
	bcnt = 4;
        if (!SUCCESS(iss = camadd (&ctlwF0A0, &read_ppom[0], &bcnt, &emask, &pkg_p)))
        {
            errlogPrintf("camadd error %s\n",cammsg(iss));
            rtn = (PPOM_CAM_ADD_FAIL|iss);
            goto release_campkg;
        }

        if (!SUCCESS(iss = camgo (&pkg_p)))
        {
            errlogPrintf("camgo error %s\n",cammsg(iss));
            rtn = (PPOM_CAM_GO_FAIL|iss);
            goto release_campkg;
        }


       printf("Raw: 0x%08X\n", read_ppom[0].data);
 
release_campkg: 

        if (!SUCCESS(iss = camdel (&pkg_p)))
            errlogPrintf("camdel error %s\n",cammsg(iss));
    }
    else
        rtn = PPOM_MODULE_NOT_EXIST;
egress:
    return rtn;
}

epicsRegisterFunction(PPOM_Test);

