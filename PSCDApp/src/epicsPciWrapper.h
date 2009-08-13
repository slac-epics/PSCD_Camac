#ifndef _EPICS_PCI_H_
#define _EPICS_PCI_H_

#ifdef vxWorks
#include "vxWorks.h"
#include "ioLib.h"
#include "iosLib.h"
#include "stat.h"
#include "dirent.h"
#include "memLib.h"
#include "semLib.h"
#include "iv.h"
#include "errnoLib.h"
#include "drv/pci/pciConfigLib.h"
#include "drv/pci/pciIntLib.h" /* for pciIntConnect */
#include "vmLib.h"
#include "cacheLib.h"
#include "intLib.h"
#include "sysLib.h"
#include "taskLib.h"
#define HANDLE int
#define DIRHANDLE int

#elif defined(__rtems__)
#include <rtems.h>
#include <errno.h>
#include <bsp.h>
#include <bsp/pci.h>
#include <bsp/irq.h>
#include <bsp/bspExt.h>
#include <libcpu/io.h>
#include <libcpu/byteorder.h>
#include <sys/systm.h> 
#include <rtems/bspIo.h>	/* TODO, add vxWorks declaration for printk */
#include <rtems/rtems/region.h>	/* for memory partition. TODO, add vxWorks declaration and epics common macro */
#define HANDLE int
#define DIRHANDLE int

#else
#error "This driver is not implemented for this OS"
#endif

#include "genType.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* include system header files */
#ifndef OK
#define OK      0
#endif

#ifndef ERROR
#define ERROR   (-1)
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

typedef struct PCI_BASE_ADDR
{
    UINT32                      pciBAR;	/* Keep LSB info to know it is IO or MEM */
    UINT32                      size;	/* Size in byte */
    UINT32			cpuPhysicalAddr;	/* integer, for DMA purpose */
    volatile char *             pUserVirtualAddr;
} PCI_BASE_ADDR;

typedef struct PCI_HEADER
{
    int                         bus;
    int                         slot;
    int                         func;

    PCI_BASE_ADDR		pciBaseAddr[6];

    UINT8                       intNum;         /* Interrupt Number */
} PCI_HEADER;

int epicsPciHeaderGet(int vendorId, int deviceId, int index, PCI_HEADER * pPciHeader);
int epicsPciHeaderInfo(PCI_HEADER * pPciHeader);

/* Here we define some macro to cover difference between RTEMS and vxWorks */
#ifdef vxWorks

/*************************************************************************/
/* PCI configuration function                                            */
/* This part is idential for both x86 and ppc                            */
/*************************************************************************/
#define epicsPciFindDevice      pciFindDevice
#define epicsPciConfigInByte    pciConfigInByte
#define epicsPciConfigInWord    pciConfigInWord
#define epicsPciConfigInLong    pciConfigInLong
#define epicsPciConfigOutByte   pciConfigOutByte
#define epicsPciConfigOutWord   pciConfigOutWord
#define epicsPciConfigOutLong   pciConfigOutLong

/*************************************************************************/
/* Map I/O functions to underlying system functions.                     */
/* This part is different between x86 and ppc                            */
/* x86 use port to access IO, all others are memory mapped               */
/*************************************************************************/
#ifdef _X86_	/* _X86_ is defined in EPICS configuration file. Somehow WindRiver removed CPU_FAMILY */
#ifndef PCI_IO_OFFSET
#define PCI_IO_OFFSET       0
#endif
#define PCI_IO_PUTC(addr,value)		sysOutByte ((UINT32)(addr) + PCI_IO_OFFSET, (UINT8)(value))
#define PCI_IO_PUTS(addr,value)		sysOutWord ((UINT32)(addr) + PCI_IO_OFFSET, (UINT16)(value))
#define PCI_IO_PUTL(addr,value)		sysOutLong ((UINT32)(addr) + PCI_IO_OFFSET, (UINT32)(value))
#define PCI_IO_GETC(addr)		sysInByte ((UINT32)(addr) + PCI_IO_OFFSET)
#define PCI_IO_GETS(addr)		sysInWord ((UINT32)(addr) + PCI_IO_OFFSET)
#define PCI_IO_GETL(addr)		sysInLong ((UINT32)(addr) + PCI_IO_OFFSET)

#define PCI_MEM_PUTC(addr,value)	(*(volatile UINT8 *)((UINT32)(addr) + PCI_IO_OFFSET)) = (UINT8)(value)
#define PCI_MEM_PUTS(addr,value)	(*(volatile UINT16 *)((UINT32)(addr) + PCI_IO_OFFSET)) = (UINT16)(value)
#define PCI_MEM_PUTL(addr,value)	(*(volatile UINT32 *)((UINT32)(addr) + PCI_IO_OFFSET)) = (UINT32)(value)
#define PCI_MEM_GETC(addr)		(*(volatile UINT8 *)((UINT32)(addr) + PCI_IO_OFFSET))
#define PCI_MEM_GETS(addr)		(*(volatile UINT16 *)((UINT32)(addr) + PCI_IO_OFFSET))
#define PCI_MEM_GETL(addr)		(*(volatile UINT32 *)((UINT32)(addr) + PCI_IO_OFFSET))
#else	/* Both IO and MEM are memory mapped, so no difference */
#define PCI_IO_PUTC(addr, value)	sysPciOutByte((UINT32)(addr),(UINT8)value);
#define PCI_IO_PUTS(addr, value)	sysPciOutWord((UINT32)(addr),(UINT16)value);
#define PCI_IO_PUTL(addr, value)	sysPciOutLong((UINT32)(addr),(UINT32)value);
#define PCI_IO_GETC(addr)		sysPciInByte((UINT32)(addr));
#define PCI_IO_GETS(addr)		sysPciInWord((UINT32)(addr));
#define PCI_IO_GETL(addr)		sysPciInLong((UINT32)(addr));

#define PCI_MEM_PUTC(addr, value)	sysPciOutByte((UINT32)(addr),(UINT8)value);
#define PCI_MEM_PUTS(addr, value)	sysPciOutWord((UINT32)(addr),(UINT16)value);
#define PCI_MEM_PUTL(addr, value)	sysPciOutLong((UINT32)(addr),(UINT32)value);
#define PCI_MEM_GETC(addr)		sysPciInByte((UINT32)(addr));
#define PCI_MEM_GETS(addr)		sysPciInWord((UINT32)(addr));
#define PCI_MEM_GETL(addr)		sysPciInLong((UINT32)(addr));
#endif

/*************************************************************************/
/* Interrupt Enable/Disable Functions                                    */
/* This part is different between x86 and ppc                            */
/*************************************************************************/
#ifdef _X86_	/* _X86_ is defined in EPICS configuration file. Somehow WindRiver removed CPU_FAMILY */
#define epicsPciIntConnect(pciLine, isr, uarg)          pciIntConnect(INUM_TO_IVEC(INT_NUM_GET(pciLine)), isr, uarg)
#define epicsPciIntDisconnect(pciLine, isr, uarg)       pciIntDisconnect2(INUM_TO_IVEC(INT_NUM_GET(pciLine)), isr, uarg)
#define epicsPciIntEnable(pciLine)			sysIntEnablePIC(pciLine)
#define epicsPciIntDisable(pciLine)			sysIntDisablePIC(pciLine)
#else
#define epicsPciIntConnect(pciLine, isr, uarg)		pciIntConnect(INUM_TO_IVEC(pciLine), isr, uarg)
#define epicsPciIntDisconnect(pciLine, isr, uarg)	pciIntDisconnect2(INUM_TO_IVEC(pciLine), isr, uarg)
#define epicsPciIntEnable(pciLine)			intEnable(pciLine)
#define epicsPciIntDisable(pciLine)			intDisable(pciLine)
#endif

/*************************************************************************/
/* memory block function with cache disabled                             */
/*************************************************************************/
/* no need to define cacheDmaMalloc since it is native to vxWorks */
/* no need to define cacheDmaFree since it is native to vxWorks */
/* no need to define CACHE_DMA_INVALIDATE since it is native to vxWorks */
/* no need to define CACHE_DMA_FLUSH since it is native to vxWorks */

#ifdef _X86_	/* _X86_ is defined in EPICS configuration file. Somehow WindRiver removed CPU_FAMILY */
/* So far x86 is one to one mapping in MMU, we don't map port, still one to one */
#define PCI2CPUADDR(pciaddr)    ((pciaddr&0x1)?(pciaddr&0xFFFFFFFC):(pciaddr&0xFFFFFFF0))
#define MEM2PCIADDR(memaddr)    (memaddr)	/* TODO */
#else
/* PCI resources as seen by the CPU, TODO */
#define PCI2CPUADDR(pciaddr)    pciBusToLocalAdrs(pciaddr)
/* CPU memory as seen from the PCI bus, TODO */
#define MEM2PCIADDR(memaddr)    pciLocalToBusAdrs(memaddr)
#endif

#define epicsMemPartId		PART_ID
#elif defined(__rtems__)
/* TODO, how about x86 */
/*#define       epicsPciFindDevice      BSP_pciFindDevice*/
#define epicsPciFindDevice      pci_find_device
#define epicsPciConfigInByte    pci_read_config_byte
#define epicsPciConfigInWord    pci_read_config_word
#define epicsPciConfigInLong    pci_read_config_dword
#define epicsPciConfigOutByte   pci_write_config_byte
#define epicsPciConfigOutWord   pci_write_config_word
#define epicsPciConfigOutLong   pci_write_config_dword

/* st_le32, ld_le32, st_be32, ld_be32 are recommended for memory only */
/* in_le32, out_le32, in_be32, out_be32 are recommended for device */

#ifdef _X86_	/* _X86_ is defined in EPICS configuration file. TODO, test */
#ifndef PCI_IO_OFFSET
#define PCI_IO_OFFSET       0
#endif
#define PCI_IO_PUTC(addr,value)		out_8 ((UINT32)(addr) + PCI_IO_OFFSET, (UINT8)(value))
#define PCI_IO_PUTS(addr,value)		out_le16 ((UINT32)(addr) + PCI_IO_OFFSET, (UINT16)(value))
#define PCI_IO_PUTL(addr,value)		out_le32 ((UINT32)(addr) + PCI_IO_OFFSET, (UINT32)(value))
#define PCI_IO_GETC(addr)		in_8 ((UINT32)(addr) + PCI_IO_OFFSET)
#define PCI_IO_GETS(addr)		in_le16 ((UINT32)(addr) + PCI_IO_OFFSET)
#define PCI_IO_GETL(addr)		in_le32 ((UINT32)(addr) + PCI_IO_OFFSET)

#define PCI_MEM_PUTC(addr,value)	(*(volatile UINT8 *)((UINT32)(addr) + PCI_IO_OFFSET)) = (UINT8)(value)
#define PCI_MEM_PUTS(addr,value)	(*(volatile UINT16 *)((UINT32)(addr) + PCI_IO_OFFSET)) = (UINT16)(value)
#define PCI_MEM_PUTL(addr,value)	(*(volatile UINT32 *)((UINT32)(addr) + PCI_IO_OFFSET)) = (UINT32)(value)
#define PCI_MEM_GETC(addr)		(*(volatile UINT8 *)((UINT32)(addr) + PCI_IO_OFFSET))
#define PCI_MEM_GETS(addr)		(*(volatile UINT16 *)((UINT32)(addr) + PCI_IO_OFFSET))
#define PCI_MEM_GETL(addr)		(*(volatile UINT32 *)((UINT32)(addr) + PCI_IO_OFFSET))
#else	/* Both IO and MEM are memory mapped, so no difference */
#define PCI_IO_PUTC(addr, value)        out_8((volatile unsigned char *)(addr),(int)value);
#define PCI_IO_PUTS(addr, value)        out_le16((volatile unsigned short *)(addr),(int)value);
#define PCI_IO_PUTL(addr, value)        out_le32((volatile unsigned *)(addr),(int)value);
#define PCI_IO_GETC(addr)               in_8((volatile unsigned char *)(addr));
#define PCI_IO_GETS(addr)               in_le16((volatile unsigned short *)(addr));
#define PCI_IO_GETL(addr)               in_le32((volatile unsigned *)(addr));

#define PCI_MEM_PUTC(addr, value)       out_8((volatile unsigned char *)(addr),(int)value);
#define PCI_MEM_PUTS(addr, value)       out_le16((volatile unsigned short *)(addr),(int)value);
#define PCI_MEM_PUTL(addr, value)       out_le32((volatile unsigned *)(addr),(int)value);
#define PCI_MEM_GETC(addr)              in_8((volatile unsigned char *)(addr));
#define PCI_MEM_GETS(addr)              in_le16((volatile unsigned short *)(addr));
#define PCI_MEM_GETL(addr)              in_le32((volatile unsigned *)(addr));
#endif


#define epicsPciIntConnect(pciLine, isr, uarg)  \
        bspExtInstallSharedISR( (int)(BSP_PCI_IRQ_LOWEST_OFFSET + (pciLine)), isr, uarg, 0)
#define epicsPciIntDisconnect(pciLine, isr, uarg) \
        bspExtRemoveSharedISR( (int)(BSP_PCI_IRQ_LOWEST_OFFSET + (pciLine)), isr, uarg)
#define epicsPciIntEnable(pciLine)      BSP_enable_irq_at_pic(pciLine)
#define epicsPciIntDisable(pciLine)     BSP_disable_irq_at_pic(pciLine)

#define cacheDmaMalloc(size)    malloc(size)    /* do we need memalign ? */
#define cacheDmaFree(ptr)       free(ptr)
#define CACHE_DMA_INVALIDATE(pBuf, BUF_SIZE)    /* MVME6100 has snoop, how about other type of CPU */
#define CACHE_DMA_FLUSH(pBuf, BUF_SIZE)

/* PCI resources as seen by the CPU */
#define PCI2CPUADDR(pciaddr) ((unsigned long)(pciaddr) + PCI_MEM_BASE)
/* CPU memory as seen from the PCI bus */
#define MEM2PCIADDR(memaddr) ((unsigned long)(memaddr) + PCI_DRAM_OFFSET)

#define epicsMemPartId		rtems_id
#else
#error "This driver is not implemented for this OS"
#endif

#ifdef __cplusplus
}
#endif  /* __cplusplus */
#endif

