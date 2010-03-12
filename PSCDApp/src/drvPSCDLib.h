/*********************************************************************/
/* $Id: drvPSCDLib.h,v 1.4 2009/08/13 06:15:45 pengs Exp $           */
/* This file defines the internal hw/sw struct of PSCD module        */
/* Author: Sheng Peng, pengs@slac.stanford.edu, 650-926-3847         */
/*********************************************************************/

#ifndef _INCLUDE_DRV_PSCD_LIB_H_
#define _INCLUDE_DRV_PSCD_LIB_H_

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include <epicsVersion.h>
#if EPICS_VERSION>=3 && EPICS_REVISION>=14

#include <epicsMutex.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsInterrupt.h>
#include <epicsMutex.h>
#include <epicsMessageQueue.h>
#include <epicsEvent.h>
#include <epicsInterrupt.h>
#include <cantProceed.h>
#include <epicsExport.h>
#include <drvSup.h>
#include <dbScan.h>
#include <ellLib.h>
#include <errlog.h>

#include "epicsPciWrapper.h"

#else
#error "You need EPICS 3.14 or above because we need OSI support!"
#endif

#define PSCD_DRV_VERSION "PSCD driver V1.0"

#ifndef BOOL
#define BOOL int
#endif

/* device driver ID structure, we will use array to control multi-devices, index is same as PCI index */
typedef struct PSCD_CARD
{
    epicsMutexId	lock;
    epicsEventId	semSio[3];
    BOOL	        waitSemSio[3];
    PCI_HEADER		pciHeader;
    UINT32		fwVer;	/* also indicate the card is in use */
    UINT32		fwDate;
    UINT32		fwID;

    UINT32		portMap[2];

    UINT32 *		sio_p[3];	/* for SIO, 0 is the highest priority */
    UINT32 *		tdv_p[3];	/* for TDV, 0 is the highest priority */
    unsigned char *	sram_p;
    UINT32 *		switch_p;	/* pointing to the register to handle PSCD MBCD switch */
    epicsMemPartId	memPartId;	/* Memory partition on PSCD */

    epicsMessageQueueId msgQId;
    epicsThreadId       opTaskId;       /* operation thread ID for this PSCD */

} PSCD_CARD;

typedef struct PSCD_CARD * PSCD_ID;

int pscdCreate(unsigned int portMap0, unsigned int portMap1);
int isModuleExsit(short b, short c, short n);
int branchOfCrate(unsigned short c);

#define MEMPARTADRS_TO_SIO(addr)        ((int)((char *)(addr)-(pscd_card.pciHeader.pciBaseAddr[2].pUserVirtualAddr)-0x100000))
#define SIO_TO_MEMPARTADRS(addr)        ((int)((char *)(addr)+(pscd_card.pciHeader.pciBaseAddr[2].pUserVirtualAddr)+0x100000))

#define PSCD_MEM_PUTC(addr, value)       out_8((volatile unsigned char *)(addr),(int)value);
#define PSCD_MEM_PUTS(addr, value)       out_be16((volatile unsigned short *)(addr),(int)value);
#define PSCD_MEM_PUTL(addr, value)       out_be32((volatile unsigned *)(addr),(int)value);
#define PSCD_MEM_GETC(addr)              in_8((volatile unsigned char *)(addr));
#define PSCD_MEM_GETS(addr)              in_be16((volatile unsigned short *)(addr));
#define PSCD_MEM_GETL(addr)              in_be32((volatile unsigned *)(addr));

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif

