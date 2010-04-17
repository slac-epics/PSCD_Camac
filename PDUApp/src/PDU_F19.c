/***************************************************************************\
 *   $Id: PDU_F19.c,v 1.1 2010/01/13 05:58:28 pengs Exp $
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
#include "devPDU.h"
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

extern int TSmod360;


int PDU_F19(unsigned int crate, unsigned int PP0, unsigned int PP1)
{

    UINT32 rtn = 0;

    /* check if module exists */
    if(TRUE)
    {
        vmsstat_t iss;

            *((UINT16 *)(&(pdu_f19[0].data))) = (PP0 << 8);
	    

            *((UINT16 *)(&(pdu_f19[1].data))) = (PP1 << 8);

            if (!SUCCESS(iss = camgo (&pkg_p)))
            {
                errlogPrintf("camgo error 0x%08X\n",(unsigned int) iss);
                rtn = iss;
                goto release_campkg;
            }

        }
    }
    else
        rtn = -1;
egress:
    return rtn;
}


