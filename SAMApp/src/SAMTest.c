/***************************************************************************\
 *   $Id: SAMTest.c,v 1.7 2009/03/10 00:35:38 pengs Exp $
 *   File:		SAMTest.c
 *   Author:		Sheng Peng
 *   Email:		pengsh2003@yahoo.com
 *   Phone:		408-660-7762
 *   Company:		RTESYS, Inc.
 *   Date:		03/2007
 *   Version:		1.0
 *
 *   EPICS driver for SAM 
 *
\***************************************************************************/
#include "drvPSCDLib.h"
#include "devSAM.h"
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

UINT32 SAM_Test()
{

    UINT32 rtn = 0;

    union I2F
    {
        UINT32 tempI;
        float tempF;
    } value;

    /* check if module exists */
    if(isModuleExsit(0, 5, 16))
    {
        unsigned int loop;

        void *pkg_p;  /* A camac package */
        vmsstat_t iss;
        UINT16 bcnt = 4;
        UINT16 emask= 0xE0E0;


        /*UINT32 ctlwF17 = 0x04115800;
        UINT32 ctlwF0 = 0x04005800;*/
        UINT32 ctlwF17 = 0x00115800;
        UINT32 ctlwF16 = 0x00105800;
        UINT32 ctlwF9 = 0x00095800;
        UINT32 ctlwF0 = 0x00005800;

        STAS_DAT read_sam[SAM_NUM_OF_CHANNELS*2 + 1];	/* need to read twice for each channel, each read gets 16 bits of 32-bit float */
        UINT16 nops = 0;

        if (!SUCCESS(iss = cam_ini (&pscd_card)))	/* no need, should be already done in PSCD driver */
        {
            errlogPrintf("cam_ini error 0x%08X\n",(unsigned int) iss);
            rtn = (SAM_CAM_INIT_FAIL|iss);
            goto egress;
        }

#if 0
        /* F9 to reset */
	bcnt = 0;
        if (!SUCCESS(iss = camio (&ctlwF9, NULL/*&(read_sam[0].data)*/, &bcnt, &(read_sam[0].stat), &emask)))
        {
            errlogPrintf ("camio error 0x%08X for SAM reset\n", (unsigned int) iss);
            return (SAM_RST_CAMIO_FAIL|iss);
        }

	epicsThreadSleep(10);

#endif
	/* F16 to setup */
	bcnt = 2;
        read_sam[0].data = 0x00000005; /* 101b, IEEE, normal mode, read firmware version */
        if (!SUCCESS(iss = camio (&ctlwF16, &(read_sam[0].data), &bcnt, &(read_sam[0].stat), &emask)))
        {
            errlogPrintf ("camio error 0x%08X for SAM setup\n", (unsigned int) iss);
            return (SAM_SETUP_CAMIO_FAIL|iss);
        }

	epicsThreadSleep(10);

        /* F17 to set start channel */
        read_sam[0].data = 0;
	bcnt = 2;
        if (!SUCCESS(iss = camio (&ctlwF17, &(read_sam[0].data), &bcnt, &(read_sam[0].stat), &emask)))
        {
            errlogPrintf ("camio error 0x%08X for SAM F11\n", (unsigned int) iss);
            return (SAM_RST_CAMIO_FAIL|iss);
        }

	/* F0 to read firmware version */
        for (loop=1; loop <= 2; loop++)
        {
	    bcnt = 2;
            if (!SUCCESS(iss = camio (&ctlwF0, &read_sam[loop].data, &bcnt, &read_sam[loop].stat, &emask)))
            {
                errlogPrintf ("camio error 0x%08X for SAM F0-%d\n", (unsigned int) iss, loop);
                return (SAM_SETUP_CAMIO_FAIL|iss);
            }
	}

        value.tempI = (read_sam[1].data&0xFFFF) | (read_sam[2].data << 16);
        printf("Firmware Version: %f\n", value.tempF);
        printf("Firmware Version Raw: L:0x%08X H:0x%08X\n", read_sam[1].data, read_sam[2].data);
 
	/* F16 to setup */
	bcnt = 2;
        read_sam[0].data = 0x00000005; /* 101b, IEEE, normal mode, no read firmware version */
        if (!SUCCESS(iss = camio (&ctlwF16, &(read_sam[0].data), &bcnt, &(read_sam[0].stat), &emask)))
        {
            errlogPrintf ("camio error 0x%08X for SAM setup\n", (unsigned int) iss);
            return (SAM_SETUP_CAMIO_FAIL|iss);
        }

	epicsThreadSleep(10);

#if 1
        nops = 3;
 
        /** Allocate package for SAM reset */
        if (!SUCCESS(iss = camalol (&nops, &pkg_p)))
        {
            errlogPrintf("camalol error 0x%08X\n",(unsigned int) iss);
            rtn = (SAM_CAM_ALLOC_FAIL|iss);
            goto egress;
        }

        read_sam[0].data = 7;
        if (!SUCCESS(iss = camadd (&ctlwF17, &read_sam[0], &bcnt, &emask, &pkg_p)))
        {
            errlogPrintf("camadd error 0x%08X\n",(unsigned int) iss);
            rtn = (SAM_CAM_ADD_FAIL|iss);
            goto release_campkg;
        }

        for (loop=1; loop <= 2; loop++)
        {
            if (!SUCCESS(iss = camadd (&ctlwF0, &read_sam[loop], &bcnt, &emask, &pkg_p)))
            {
                errlogPrintf("camadd error 0x%08X\n",(unsigned int) iss);
                rtn = (SAM_CAM_ADD_FAIL|iss);
                goto release_campkg;
            }
        }

        if (!SUCCESS(iss = camgo (&pkg_p)))
        {
            errlogPrintf("camgo error 0x%08X\n",(unsigned int) iss);
            rtn = (SAM_CAM_GO_FAIL|iss);
            goto release_campkg;
        }


       value.tempI = ((read_sam[1].data)&0xFFFF) | (read_sam[2].data << 16);
       printf("Ch[%d]: %f\n", read_sam[0].data, value.tempF);
 
release_campkg: 

        if (!SUCCESS(iss = camdel (&pkg_p)))
            errlogPrintf("camdel error 0x%08X\n",(unsigned int) iss);
#endif
    }
    else
        rtn = SAM_MODULE_NOT_EXIST;
egress:
    return rtn;
}

epicsRegisterFunction(SAM_Test);

