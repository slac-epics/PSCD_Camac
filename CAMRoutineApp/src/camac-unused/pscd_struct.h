 
#ifndef PSCDCARD_H

#include "epicsMutex.h"            /* for PSCD extern                         */
#include "epicsMessageQueue.h"
#include "epicsThread.h"

/* dummies */
#define PCI_HEADER int
#define epicsMemPartId int
#define UINT32 unsigned int
#define MEMPARTADDRS_TO_SIO(arg) (arg & 0xFFFFFFFF)

typedef struct PSCD_CARD
{
    epicsMutexId        lock;
    epicsMessageQueueId msgQId;
    PCI_HEADER          pciHeader;
    UINT32              fwVer;  /* also indicate the card is in use */
    UINT32              fwDate;
    UINT32              fwID;
    unsigned           *sio_p[3];       /* for SIO, 0 is the highest priority */
    unsigned           *tdv_p[3];       /* for TDV, 0 is the highest priority */
    unsigned char *     sram_p;
    epicsMemPartId      memPartId;      /* Memory partition on PSCD */

    epicsThreadId       opTaskId;       /* operation thread ID for this PSCD */

} PSCD_CARD;

#define PSCDCARD_H
#endif
