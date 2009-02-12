/*********************************************************************/
/* $Id: drvPSCD.h,v 1.5 2008/03/24 01:44:28 laser Exp $              */
/* This file defines the internal hw/sw struct of PSCD module        */
/* Author: Sheng Peng, pengs@slac.stanford.edu, 650-926-3847         */
/*********************************************************************/

#ifndef _INCLUDE_DRV_PSCD_H_
#define _INCLUDE_DRV_PSCD_H_

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

#define MAX_CA_STRING_LEN	40

/* Manufacturers assigned values */
#define PSCD_VID		0x16D5	/* Acromag */
#define PSCD_DX2002_DID		0x424B	/* DX2002 */
#define PSCD_DX502_DID		0x4357  /* DX502 */

/* All the offset defined as bar<<28 | offset */
/* Define PLX9056 Register */
#define PLX_LAS0BA_OFFSET		0x00000004
#define PLX_LAS0BA_VALUE		0x20000001

#define PLX_BIGEND_OFFSET		0x0000000C

#define PLX_INTCSR_OFFSET		0x00000068

#define PLX_LAS1BA_OFFSET		0x000000F4
#define PLX_LAS1BA_VALUE		0x40000001

/* Define PSCD function registers */
/* In BAR2 */
/* Global Control and Status Register */
#define PSCD_SIO0_OFFSET	0x20008000
#define PSCD_TDV0_OFFSET	0x20008004
#define PSCD_CNTR0_OFFSET	0x20008008
#define PSCD_TIMER0_OFFSET	0x2000800C
#define PSCD_SIO1_OFFSET	0x20008010
#define PSCD_TDV1_OFFSET	0x20008014
#define PSCD_CNTR1_OFFSET	0x20008018
#define PSCD_TIMER1_OFFSET	0x2000801C
#define PSCD_SIO2_OFFSET	0x20008020
#define PSCD_TDV2_OFFSET	0x20008024
#define PSCD_CNTR2_OFFSET	0x20008028
#define PSCD_TIMER2_OFFSET	0x2000802C
#define PSCD_PMAP0_OFFSET	0x20008030
#define PSCD_PMAP1_OFFSET	0x20008034
#define PSCD_VERSION_OFFSET	0x20008038
#define PSCD_UNUSED_OFFSET	0x2000803C
#define PSCD_SIOTO0_OFFSET	0x20008040
#define PSCD_SIOTO1_OFFSET	0x20008044
#define PSCD_SIOTO2_OFFSET	0x20008048
#define PSCD_ID_OFFSET		0x2000804C
#define PSCD_DATE_OFFSET	0x20008050
#define PSCD_INTREG_OFFSET	0x20008054
#define PSCD_INTEN_OFFSET	0x20008058
#define PSCD_RECDATA_OFFSET	0x2000805C
#define PSCD_RECCNT_OFFSET	0x20008060

#define PSCD_SRAM_OFFSET	0x20100000
#define PSCD_SRAM_SIZE		0x100000

#define PSCD_MEM_PUTC(addr, value)       out_8((volatile unsigned char *)(addr),(int)value);
#define PSCD_MEM_PUTS(addr, value)       out_be16((volatile unsigned short *)(addr),(int)value);
#define PSCD_MEM_PUTL(addr, value)       out_be32((volatile unsigned *)(addr),(int)value);
#define PSCD_MEM_GETC(addr)              in_8((volatile unsigned char *)(addr));
#define PSCD_MEM_GETS(addr)              in_be16((volatile unsigned short *)(addr));
#define PSCD_MEM_GETL(addr)              in_be32((volatile unsigned *)(addr));

#define PCI_SETREG(offset,value)	PCI_MEM_PUTL(pscd_card.pciHeader.pciBaseAddr[(offset)>>28].pUserVirtualAddr+((offset)&0x0fffffff), (value))
#define PCI_GETREG(offset)		PCI_MEM_GETL(pscd_card.pciHeader.pciBaseAddr[(offset)>>28].pUserVirtualAddr+((offset)&0x0fffffff))

#define PSCD_SETREG(offset,value)	PSCD_MEM_PUTL(pscd_card.pciHeader.pciBaseAddr[(offset)>>28].pUserVirtualAddr+((offset)&0x0fffffff), (value))
#define PSCD_GETREG(offset)		PSCD_MEM_GETL(pscd_card.pciHeader.pciBaseAddr[(offset)>>28].pUserVirtualAddr+((offset)&0x0fffffff))

#define MEMPARTADRS_TO_SIO(addr)        ((int)((char *)(addr)-(pscd_card.pciHeader.pciBaseAddr[2].pUserVirtualAddr)-0x100000))
#define SIO_TO_MEMPARTADRS(addr)        ((int)((char *)(addr)+(pscd_card.pciHeader.pciBaseAddr[2].pUserVirtualAddr)+0x100000))

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif

