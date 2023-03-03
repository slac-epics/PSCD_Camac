#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include <epicsVersion.h>
#if EPICS_VERSION>3 || (EPICS_VERSION==3 && EPICS_REVISION>=14)

#include <epicsMutex.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsInterrupt.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
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

/* So far we just hardcode sysPhysMemDesc to add the MMU entries. */
/* Need to addd sysPMC6SFPInit which does sysMmuMapAdd and be called in sysLib.c by or before pciConfigForeachFunc */

int epicsPciHeaderGet(int vendorId, int deviceId, int index, PCI_HEADER * pPciHeader)
{
    int bus, slot, func;
    int loop = 0;
    int status = 0;
    UINT32 BARTemp;
    UINT8 IntLine;
    int	lockKey;

    if(pPciHeader == NULL) return -1;
    bzero( (char *)pPciHeader, sizeof (PCI_HEADER) );

    status = epicsPciFindDevice(vendorId ,deviceId ,index ,&bus,&slot, &func);
    if(status != 0)
    {
        errlogPrintf("VID: 0x%04X; DID: 0x%04X; Index %d; Not found!\n", vendorId, deviceId, index);
        return -1;
    }

    pPciHeader->bus = bus;
    pPciHeader->slot = slot;
    pPciHeader->func = func;

    for(loop=0;loop<6;loop++)
    {/* read BAR start from PCI_CFG_BASE_ADDRESS_0 and test size */
     /* read BAR and save */

        BARTemp = 0;
        epicsPciConfigInLong(bus, slot, func, /* PCI_CFG_BASE_ADDRESS_0 */0x10 + loop*4, &BARTemp);

        if(BARTemp == 0)/* not in use */
        {
            pPciHeader->pciBaseAddr[loop].pciBAR = 0;
            pPciHeader->pciBaseAddr[loop].size = 0;
            pPciHeader->pciBaseAddr[loop].cpuPhysicalAddr = 0;
            pPciHeader->pciBaseAddr[loop].pUserVirtualAddr = NULL;
#if 0
            printf("BAR%d is 0!\n", loop);
#endif
        }
        else
        {/* detect size */
            lockKey = epicsInterruptLock();/* address transition is dangerous */

            pPciHeader->pciBaseAddr[loop].pciBAR = BARTemp;
            epicsPciConfigOutLong(bus, slot, func, /*PCI_CFG_BASE_ADDRESS_0*/0x10 + loop*4, 0xFFFFFFFF);
            epicsPciConfigInLong(bus, slot, func, /*PCI_CFG_BASE_ADDRESS_0*/0x10 + loop*4, &BARTemp);

            if( BARTemp & /*PCI_BAR_SPACE_MASK*/0x1 )
            {/* I/O; PCI_BAR_SPACE_IO */
                pPciHeader->pciBaseAddr[loop].size = 0xFFFFFFFF-(BARTemp & /*PCI_IOBASE_MASK*/0xFFFFFFFC)+1;

                /* set PciBusAddr, so far we put duplicated code here. We assume 1-to-1 virtual mem mapping here. TODO */
                pPciHeader->pciBaseAddr[loop].cpuPhysicalAddr = PCI2CPUADDR(pPciHeader->pciBaseAddr[loop].pciBAR);
                pPciHeader->pciBaseAddr[loop].pUserVirtualAddr = (volatile char *)(pPciHeader->pciBaseAddr[loop].cpuPhysicalAddr);
            }
            else
            {/* MEM: PCI_BAR_SPACE_MEM */
                if( (BARTemp & /* PCI_BAR_MEM_TYPE_MASK */0x6) >= /*PCI_BAR_MEM_ADDR64*/0x4 )
                {/* PCI_BAR_MEM_ADDR64 or PCI_BAR_MEM_RESERVED */
                    epicsInterruptUnlock(lockKey);
                    errlogPrintf("BAR%d[0x%08X] is not supported by this OS!\n", loop, BARTemp);
                    return -1;
                }

                if(BARTemp & /*PCI_BAR_MEM_PREF_MASK*/0x8)
                {/* PCI_BAR_MEM_PREFETCH */
                }
                else
                {/* PCI_BAR_MEM_NON_PREF */
                }

                pPciHeader->pciBaseAddr[loop].size = 0xFFFFFFFF-(BARTemp & /*PCI_MEMBASE_MASK*/0xFFFFFFF0)+1;
                /* set PciBusAddr, so far we put duplicated code here. We assume 1-to-1 virtual mem mapping here. TODO */
                pPciHeader->pciBaseAddr[loop].cpuPhysicalAddr = PCI2CPUADDR(pPciHeader->pciBaseAddr[loop].pciBAR);
                pPciHeader->pciBaseAddr[loop].pUserVirtualAddr = (volatile char *)(pPciHeader->pciBaseAddr[loop].cpuPhysicalAddr);
            }

            /* write saved BAR back */
            epicsPciConfigOutLong(bus, slot, func, /*PCI_CFG_BASE_ADDRESS_0*/0x10 + loop*4, (UINT32)(pPciHeader->pciBaseAddr[loop].pciBAR));
            epicsInterruptUnlock(lockKey);
#if 0
            printf("BAR%d[0x%08X] is mapped to %p!\n", loop, pPciHeader->pciBaseAddr[loop].pciBAR, pPciHeader->pciBaseAddr[loop].pUserVirtualAddr);
#endif
        }
    }

    epicsPciConfigInByte(bus, slot, func, /*PCI_CFG_DEV_INT_LINE*/0x3C, &IntLine );

    pPciHeader->intNum = IntLine;
    /*pPciHeader->isrIntHandler = NULL;
    pPciHeader->isrArg = NULL;*/

    return 0;
}

int epicsPciHeaderInfo(PCI_HEADER * pPciHeader)
{
#if 0
typedef struct PCI_BASE_ADDR
{
    UINT32                      pciBAR; /* Keep LSB info to know it is IO or MEM */
    UINT32                      size;   /* Size in byte */
    UINT32                      cpuPhysicalAddr;        /* integer, for DMA purpose */
    volatile char *             pUserVirtualAddr;
} PCI_BASE_ADDR;

typedef struct PCI_HEADER
{
    int                         bus;
    int                         slot;
    int                         func;

    PCI_BASE_ADDR               pciBaseAddr[6];

    UINT8                       intNum;         /* Interrupt Number */
} PCI_HEADER;
#endif
    int loop;
    printf("Found card on bus[%d], slot[%d], func[%d]; int num is [%d]\n", pPciHeader->bus, pPciHeader->slot, pPciHeader->func, pPciHeader->intNum);

    for(loop=0; loop<6; loop++)
    {
	printf("\tBAR[%d]:\n", loop);
	printf("\tBAR value is [0x%08X], size is [0x%08X], mapped address is [%p]\n", 
			pPciHeader->pciBaseAddr[loop].pciBAR, pPciHeader->pciBaseAddr[loop].size, pPciHeader->pciBaseAddr[loop].pUserVirtualAddr);
    }
    return 0;
}

