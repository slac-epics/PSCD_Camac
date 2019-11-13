/****************************************************************/
/* $Id: drvPSCD.c,v 1.8 2010/04/11 00:41:12 pengs Exp $         */
/* This file implements driver support for PSCD                 */
/* Author: Sheng Peng, pengs@slac.stanford.edu, 650-926-3847    */
/****************************************************************/

/* We only support one PSCD per CPU */

#include "drvPSCDLib.h"

#include "drvPSCD.h"

int     PSCD_DRV_DEBUG = 0;

struct PSCD_CARD pscd_card;

static int pscd_card_inited=0;

static void pscdIsr(void * parg);
/*static void pscdOpTaskLow(void *parm);*/

/* CAMRoutine init function */
extern unsigned int cam_ini(struct PSCD_CARD *pPSCD);

int pscdCreate (unsigned int portMap0, unsigned int portMap1)
{
    int index = 0;
    int status;
    unsigned int plxbigend;
    unsigned int INTCSRTemp ;
    rtems_status_code rtems_sts;

    if(!pscd_card_inited)
    {/* Initialize the PSCD_CARD arry  */
        bzero( (char *) (&pscd_card), sizeof(pscd_card));
        pscd_card_inited = 1;
    }

    if(pscd_card.fwVer)
    {
        errlogPrintf ("pscdCreate: PSCD is already initialized!\n");
        return -1;
    }

    for(;;)
    {
        int fwID = 0;
        status = epicsPciHeaderGet(PSCD_VID, PSCD_DX2002_DID, index, &(pscd_card.pciHeader));
        if(status)
        {/* can't find DX2002 */
            status = epicsPciHeaderGet(PSCD_VID, PSCD_DX502_DID, index, &(pscd_card.pciHeader));
            if(status)
	    {/* can't find DX502 */
               errlogPrintf ("pscdCreate: failed to get header info from PSCD!\n");
               return -1;
	    }
        }

        fwID = PCI_GETREG(PSCD_ID_OFFSET);

	/* check if the ID is "PSCD" in hex */
	if(1/*fwID == 0x50534344*/) break;
	else
	{
	    printf("Find a Acromag board with fwID 0x%08X\n", fwID);
	    index++;
	}
    }

    if(PSCD_DRV_DEBUG) epicsPciHeaderInfo(&(pscd_card.pciHeader));

    /* debug purpose only */
    if(PSCD_DRV_DEBUG)
    {
        INTCSRTemp = PCI_GETREG(PLX_INTCSR_OFFSET);
        printf("PLX Int CS Reg: 0x%08X\n", INTCSRTemp);
    }

    /* Turn the PSCD memory mapping to big endian */
    plxbigend = PCI_GETREG(PLX_BIGEND_OFFSET);
    PCI_SETREG(PLX_BIGEND_OFFSET, plxbigend|0x00000004);

    /* Init Port Mapping */
    PSCD_SETREG(PSCD_PMAP0_OFFSET, portMap0);
    pscd_card.portMap[0]=portMap0;
    PSCD_SETREG(PSCD_PMAP1_OFFSET, portMap1);
    pscd_card.portMap[1]=portMap1;

    PSCD_SETREG(PSCD_SIOTO0_OFFSET, 0xDB24);	/* Set high priority queue timeout to 1.7ms */

    /* assign sio_p, tdv_p and sram_p */
    pscd_card.sio_p[0] = (UINT32 *)(pscd_card.pciHeader.pciBaseAddr[2].pUserVirtualAddr + (PSCD_SIO0_OFFSET & 0x0FFFFFFF));
    pscd_card.sio_p[1] = (UINT32 *)(pscd_card.pciHeader.pciBaseAddr[2].pUserVirtualAddr + (PSCD_SIO1_OFFSET & 0x0FFFFFFF));
    pscd_card.sio_p[2] = (UINT32 *)(pscd_card.pciHeader.pciBaseAddr[2].pUserVirtualAddr + (PSCD_SIO2_OFFSET & 0x0FFFFFFF));
    pscd_card.tdv_p[0] = (UINT32 *)(pscd_card.pciHeader.pciBaseAddr[2].pUserVirtualAddr + (PSCD_TDV0_OFFSET & 0x0FFFFFFF));
    pscd_card.tdv_p[1] = (UINT32 *)(pscd_card.pciHeader.pciBaseAddr[2].pUserVirtualAddr + (PSCD_TDV1_OFFSET & 0x0FFFFFFF));
    pscd_card.tdv_p[2] = (UINT32 *)(pscd_card.pciHeader.pciBaseAddr[2].pUserVirtualAddr + (PSCD_TDV2_OFFSET & 0x0FFFFFFF));
    pscd_card.sram_p = (unsigned char *)(pscd_card.pciHeader.pciBaseAddr[2].pUserVirtualAddr + (PSCD_SRAM_OFFSET & 0x0FFFFFFF));
    pscd_card.switch_p = (UINT32 *)(pscd_card.pciHeader.pciBaseAddr[2].pUserVirtualAddr + (PSCD_SWITCH_OFFSET & 0x0FFFFFFF));

    if(PSCD_DRV_DEBUG)
    {
        printf("SIO 0,1,2: %p %p %p\n", pscd_card.sio_p[0], pscd_card.sio_p[1],pscd_card.sio_p[2]);
        printf("TDV 0,1,2: %p %p %p\n", pscd_card.tdv_p[0], pscd_card.tdv_p[1],pscd_card.tdv_p[2]);
        printf("SRAM: %p\n", pscd_card.sram_p);
    }

    /* 0x50534344 is "PSCD" in hex */
    rtems_sts = rtems_region_create(0x50534344, (void *)pscd_card.sram_p, 0x100000, 0x40, RTEMS_PRIORITY, &(pscd_card.memPartId));
    if(rtems_sts == RTEMS_SUCCESSFUL) printf("Successfully create memroy region on PSCD\n");
    else printf("Failed to create memroy region on PSCD\n");

    if(PSCD_DRV_DEBUG)
    {/* test memory region */
	char * segment_p;
        rtems_region_get_segment(pscd_card.memPartId, 256, RTEMS_WAIT, 2, (void *)&segment_p);
	printf("Get a segment %p, SIO 0x%x\n", segment_p, MEMPARTADRS_TO_SIO(segment_p));
    }


    pscd_card.lock = epicsMutexMustCreate();

    pscd_card.semSio[0] = epicsEventMustCreate(epicsEventEmpty);
    pscd_card.waitSemSio[0] = FALSE;
    pscd_card.semSio[1] = epicsEventMustCreate(epicsEventEmpty);
    pscd_card.waitSemSio[1] = FALSE;
    pscd_card.semSio[2] = epicsEventMustCreate(epicsEventEmpty);
    pscd_card.waitSemSio[2] = FALSE;

    pscd_card.msgQId = epicsMessageQueueCreate(10, sizeof(UINT32));
    if (pscd_card.msgQId == NULL)
    {/* Fail to create messageQ */
        errlogPrintf("Failed to create messageQ for PSCD!\n");
        epicsThreadSuspendSelf();
    }

    /* read out firmware version info */
    pscd_card.fwID = PSCD_GETREG(PSCD_ID_OFFSET);
    pscd_card.fwDate = PSCD_GETREG(PSCD_DATE_OFFSET);
    pscd_card.fwVer = PSCD_GETREG(PSCD_VERSION_OFFSET);
    printf("Found %c%c%c%c with Firmware Version %0x. Built date is %08X! \n", 
		    (pscd_card.fwID>>24)&0xFF, (pscd_card.fwID>>16)&0xFF, (pscd_card.fwID>>8)&0xFF, pscd_card.fwID&0xFF,
		    pscd_card.fwVer, pscd_card.fwDate);

    /* Create operation thread */
    /* pscd_card.opTaskId = epicsThreadMustCreate("PSCD_Poll", epicsThreadPriorityHigh, 20480, pscdOpTaskLow, (void *)index); */

    /* Enable Interrupt */
    if(PSCD_DRV_DEBUG)
    {
        printf("intNum is %d\n", pscd_card.pciHeader.intNum);
    }

    epicsPciIntConnect(pscd_card.pciHeader.intNum, pscdIsr,0);
    epicsPciIntEnable(pscd_card.pciHeader.intNum);

    PSCD_SETREG(PSCD_INTEN_OFFSET, 0x7);  /* Enable interrupt for all three SIO */

    /* Enable IRQ in PLX9056, BAR0 and BAR1 are already little endian */
    INTCSRTemp = PCI_GETREG(PLX_INTCSR_OFFSET);
    /* debug purpose only */
    if(PSCD_DRV_DEBUG)
    {
        printf("PLX Int CS Reg after swap: 0x%08X\n", INTCSRTemp);
    }

    INTCSRTemp |= 0x00000900u;	/* Bit11 and Bit 8 to 1 */
    PCI_SETREG(PLX_INTCSR_OFFSET, INTCSRTemp);

    cam_ini(&pscd_card);

    return 0;
}

static void pscdIsr(void * parg)
{
    int loopsio;
    unsigned int pscdIntStatus;

    if(PSCD_DRV_DEBUG >=2) printk ("Entered ISR\n");

    if(!pscd_card.fwVer)
    {
        epicsInterruptContextMessage("pscdIsr: pscd board is not initialized yet!\n");
        return;
    }

    /* Read interrupt status */
    pscdIntStatus = PSCD_GETREG(PSCD_INTREG_OFFSET);
    /* Clear interrupt */
    PSCD_SETREG(PSCD_INTREG_OFFSET,pscdIntStatus&0x7);

    for(loopsio=0; loopsio<3; loopsio++)
    {
	if((pscdIntStatus & (1<<loopsio) && pscd_card.waitSemSio[loopsio]))
	{
            epicsEventSignal(pscd_card.semSio[loopsio]);
            if(PSCD_DRV_DEBUG >=2)
	    {
                epicsInterruptContextMessage("pscdIsr: pscd sio");
		epicsInterruptContextMessage((loopsio==0?"0":(loopsio==1?"1":"2")));
		epicsInterruptContextMessage("interrupt!\n");
	        printk("pscdIsr: pscd sio %d interrupt!\n", loopsio);
	    }
	}
    }

    return;
}

int isModuleExsit(short b, short c, short n)
{/* TODO */
    return TRUE;
}

int branchOfCrate(unsigned short c)
{
    if(!pscd_card_inited) return -1;
    else
        return (pscd_card.portMap[(c&0xF)/8] >> (((c&0xF)%8)*4))&0x3;
}

static  long    PSCD_EPICS_Init();
static  long    PSCD_EPICS_Report(int level);

#ifndef USE_TYPED_DRVET
const struct drvet drvPSCD = {2,                              /*2 Table Entries */
                             (DRVSUPFUN) PSCD_EPICS_Report,      /* Driver Report Routine */
                             (DRVSUPFUN) PSCD_EPICS_Init};       /* Driver Initialization Routine */
#else
const drvet drvPSCD = {2, PSCD_EPICS_Report, PSCD_EPICS_Init};
#endif

#if     EPICS_VERSION>=3 && EPICS_REVISION>=14
epicsExportAddress(drvet,drvPSCD);
#endif

/* implementation */
static long PSCD_EPICS_Init()
{
    return  0;
}

static long PSCD_EPICS_Report(int level)
{
    printf("\n"PSCD_DRV_VERSION"\n\n");
    if(level)
    {
	epicsPciHeaderInfo(&(pscd_card.pciHeader));
        printf("\tFound %c%c%c%c with Firmware Version %0x. Built date is %08X! \n\n", 
		    (pscd_card.fwID>>24)&0xFF, (pscd_card.fwID>>16)&0xFF, (pscd_card.fwID>>8)&0xFF, pscd_card.fwID&0xFF,
		    pscd_card.fwVer, pscd_card.fwDate);
    }
    return 0;
}

