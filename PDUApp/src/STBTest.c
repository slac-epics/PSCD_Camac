/***************************************************************************\
 *   $Id: STBTest.c,v 1.5 2010/06/13 01:35:04 pengs Exp $
 *   File:		STBTest.c
 *   Author:		Sheng Peng
 *   Email:		pengsh2003@yahoo.com
 *   Phone:		408-660-7762
 *   Company:		RTESYS, Inc.
 *   Date:		05/2010
 *   Version:		1.0
 *
 *   STB test code to verify PDU driver
 *
\***************************************************************************/
#include "drvPSCDLib.h"
#include "slc_macros.h"
#include "cam_proto.h"
#include "cctlwmasks.h"
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

UINT32 STB_Test(int crate, int slot, int rst, int PPYY, int channel, int mode)
{

    UINT32 rtn = 0;

    if(crate == 0 && slot == 0 && rst == 0 && PPYY == 0 && channel == 0 && mode ==0)
    {
	printf("STB_Test(int crate, int slot, int rst, int PPYY, int channel, int mode)\n");
        return -1;
    }

    /* check if module exists */
    if(isModuleExsit(0, crate&0xF, slot&0x1F))
    {
        unsigned int loop;

        vmsstat_t iss;
        UINT16 bcnt = 4;
        UINT16 emask= 0xE0E0;


        UINT32 ctlwF9A0 = 0x00090000 | ((crate & 0xF)<<12) | ((slot & 0x1F) << 7);
        UINT32 ctlwF17A0 = 0x00110000 | ((crate & 0xF)<<12) | ((slot & 0x1F) << 7);
        UINT32 ctlwF1A0 = 0x00010000 | ((crate & 0xF)<<12) | ((slot & 0x1F) << 7);
        UINT32 ctlwF0A0 = 0x00000000 | ((crate & 0xF)<<12) | ((slot & 0x1F) << 7) | CCTLW__P24;

        STAS_DAT test_stb;

        if (!SUCCESS(iss = cam_ini (&pscd_card)))	/* no need, should be already done in PSCD driver */
        {
            errlogPrintf("cam_ini error 0x%08X\n",(unsigned int) iss);
            return (iss);
        }

	if(rst)
	{
            /* F9A0 to reset */
	    bcnt = 0;
            if (!SUCCESS(iss = camio (&ctlwF9A0, NULL/*&(test_stb.data)*/, &bcnt, &(test_stb.stat), &emask)))
            {
                errlogPrintf ("camio error 0x%08X for STB reset\n", (unsigned int) iss);
                return (iss);
            }

	    epicsThreadSleep(5.0);
	}

	/* F17A0 to setup */
	bcnt = 2;
        *((UINT16 *)(&(test_stb.data))) = ((mode & 0x7) << 13) | ((channel & 0xF) << 8) | (PPYY & 0xFF);
        if (!SUCCESS(iss = camio (&ctlwF17A0, &(test_stb.data), &bcnt, &(test_stb.stat), &emask)))
        {
            errlogPrintf ("camio error 0x%08X for STB setup\n", (unsigned int) iss);
            return (iss);
        }
	epicsThreadSleep(1.0);

        /* F1A0 to read the setting */
	bcnt = 2;
        if (!SUCCESS(iss = camio (&ctlwF1A0, &(test_stb.data), &bcnt, &(test_stb.stat), &emask)))
        {
            errlogPrintf ("camio error 0x%08X for STB F11\n", (unsigned int) iss);
            return (iss);
        }
        else
        {
           printf("Mode of Operation is 0x%X\n", *((UINT16 *)(&(test_stb.data))) );
        }

        do {
	    /* F0A0 to read counter */
            for (loop=0; loop < 10; loop++)
            {
	        bcnt = 4;
                if (!SUCCESS(iss = camio (&ctlwF0A0, &test_stb.data, &bcnt, &test_stb.stat, &emask)))
                {
                    errlogPrintf ("camio error 0x%08X for STB F0A0\n", (unsigned int) iss);
                    return (iss);
                }

                printf("Counter is: %d, raw readback is 0x%X \n", (test_stb.data)&0x3FFFFF, test_stb.data);
                epicsThreadSleep(0.02);
            }
            printf("q to quit, other key to continue:\n");
        } while('q' != getchar());
    }
    else
        rtn = -1;

    return rtn;
}

epicsRegisterFunction(STB_Test);

