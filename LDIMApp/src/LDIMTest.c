/***************************************************************************\
 *   $Id: LDIMTest.c,v 1.1 2009/09/04 00:37:41 pengs Exp $
 *   File:		LDIMTest.c
 *   Author:		Sheng Peng
 *   Email:		pengsh2003@yahoo.com
 *   Phone:		408-660-7762
 *   Company:		RTESYS, Inc.
 *   Date:		03/2007
 *   Version:		1.0
 *
 *   EPICS driver for LDIM 
 *
\***************************************************************************/
#include "drvPSCDLib.h"
#include "devLDIM.h"
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

UINT32 LDIM_Test()
{

    UINT32 rtn = 0;

    /* check if module exists */
    if(isModuleExsit(0, 5, 16))
    {
        unsigned int loop;

        void *pkg_p;  /* A camac package */
        vmsstat_t iss;
        UINT16 bcnt = 4;
        UINT16 emask= 0xE0E0;


        UINT32 ctlwF0A0 = 0x00005200;
        UINT32 ctlwF0A1 = 0x00005201;

        STAS_DAT read_ldim[2];
        UINT16 nops = 0;

        if (!SUCCESS(iss = cam_ini (&pscd_card)))	/* no need, should be already done in PSCD driver */
        {
            errlogPrintf("cam_ini error 0x%08X\n",(unsigned int) iss);
            rtn = (LDIM_CAM_INIT_FAIL|iss);
            goto egress;
        }

        nops = 2;
 
        /** Allocate package for LDIM reset */

        if (!SUCCESS(iss = camalol (&nops, &pkg_p)))
        {
            errlogPrintf("camalol error 0x%08X\n",(unsigned int) iss);
            rtn = (LDIM_CAM_ALLOC_FAIL|iss);
            goto egress;
        }

        read_ldim[0].data = 0;
        read_ldim[1].data = 0;
	bcnt = 4;
        if (!SUCCESS(iss = camadd (&ctlwF0A0, &read_ldim[0], &bcnt, &emask, &pkg_p)))
        {
            errlogPrintf("camadd error 0x%08X\n",(unsigned int) iss);
            rtn = (LDIM_CAM_ADD_FAIL|iss);
            goto release_campkg;
        }

        if (!SUCCESS(iss = camadd (&ctlwF0A1, &read_ldim[1], &bcnt, &emask, &pkg_p)))
        {
            errlogPrintf("camadd error 0x%08X\n",(unsigned int) iss);
            rtn = (LDIM_CAM_ADD_FAIL|iss);
            goto release_campkg;
        }

        if (!SUCCESS(iss = camgo (&pkg_p)))
        {
            errlogPrintf("camgo error 0x%08X\n",(unsigned int) iss);
            rtn = (LDIM_CAM_GO_FAIL|iss);
            goto release_campkg;
        }


       printf("Raw: L:0x%08X H:0x%08X\n", read_ldim[0].data, read_ldim[1].data);
 
release_campkg: 

        if (!SUCCESS(iss = camdel (&pkg_p)))
            errlogPrintf("camdel error 0x%08X\n",(unsigned int) iss);
    }
    else
        rtn = LDIM_MODULE_NOT_EXIST;
egress:
    return rtn;
}

epicsRegisterFunction(LDIM_Test);

