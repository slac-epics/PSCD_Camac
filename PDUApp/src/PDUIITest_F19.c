/***************************************************************************\
 *   $Id: PDUIITest_F19.c,v 1.4 2010/04/20 12:19:51 pengs Exp $
 *   File:		PDUF19.c
 *   Author:		Sheng Peng
 *   Email:		pengsh2003@yahoo.com
 *   Phone:		408-660-7762
 *   Company:		RTESYS, Inc.
 *   Date:		03/2007
 *   Version:		1.0
 *
 *   PDU F19 test
 *
\***************************************************************************/
#include "drvPSCDLib.h"
#include "devPDUII.h"
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

int PDUII_Test_F19(unsigned int crate, unsigned int PP0, unsigned int PP1)
{

    UINT32 rtn = 0;

    /* check if module exists */
    if(TRUE)
    {
        void *pkg_p;  /* A camac package */
        vmsstat_t iss;
        UINT16 bcnt = 4;
        UINT16 emask= 0xE0E0;

        /* broadcast to module 31 */
	/*
        UINT32 ctlwF19A8 = 0x00130F88|(crate<<12);
        UINT32 ctlwF19A9 = 0x00130F89|(crate<<12);
	*/
        UINT32 ctlwF19A8 = 0x00130F88|(crate<<12);
        UINT32 ctlwF19A9 = 0x00130F89|(crate<<12);

        STAS_DAT pdu_f19[2];	/* only send F19A8 and F19A9 */
        UINT16 nops = 0;

        if (!SUCCESS(iss = cam_ini (&pscd_card)))	/* no need, should be already done in PSCD driver */
        {
            errlogPrintf("cam_ini error 0x%08X\n",(unsigned int) iss);
            rtn = iss;
            goto egress;
        }
	else
	{
#if 0
	    /* camio to do it */
	    bcnt = 2;
            *((UINT16 *)(&(pdu_f19[0].data))) = (PP0 << 8);
            if (!SUCCESS(iss = camio (&ctlwF19A8, &(pdu_f19[0].data), &bcnt, &(pdu_f19[0].stat), &emask)))
            {
                errlogPrintf ("camio error 0x%08X for PDU F19A8\n", (unsigned int) iss);
                return iss;
            }
#else
            nops = 2;
 
            /** Allocate package for F19 */
            if (!SUCCESS(iss = camalo (&nops, &pkg_p)))
            {
                errlogPrintf("camalol error 0x%08X\n",(unsigned int) iss);
                rtn = iss;
                goto egress;
            }

            *((UINT16 *)(&(pdu_f19[0].data))) = (PP0 << 8);
	    bcnt = 2;
            if (!SUCCESS(iss = camadd (&ctlwF19A8, &pdu_f19[0], &bcnt, &emask, &pkg_p)))
            {
                errlogPrintf("camadd error 0x%08X\n",(unsigned int) iss);
                rtn = iss;
                goto release_campkg;
            }

            *((UINT16 *)(&(pdu_f19[1].data))) = (PP1 << 8);
	    bcnt = 2;
            if (!SUCCESS(iss = camadd (&ctlwF19A9, &pdu_f19[1], &bcnt, &emask, &pkg_p)))
            {
                errlogPrintf("camadd error 0x%08X\n",(unsigned int) iss);
                rtn = iss;
                goto release_campkg;
            }

            if (!SUCCESS(iss = camgo (&pkg_p)))
            {
                errlogPrintf("camgo error 0x%08X\n",(unsigned int) iss);
                rtn = iss;
                goto release_campkg;
            }

release_campkg: 

            if (!SUCCESS(iss = camdel (&pkg_p)))
                errlogPrintf("camdel error 0x%08X\n",(unsigned int) iss);
#endif
        }
    }
    else
        rtn = -1;
egress:
    return rtn;
}


