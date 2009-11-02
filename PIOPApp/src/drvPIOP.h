/***************************************************************************\
 *   $Id: drvPIOP.h,v 1.1 2009/08/13 06:19:43 pengs Exp $
 *   File:		drvPIOP.h
 *   Author:		Robert C. Sass
 *   Email:		bsassy@garlic.com
 *   Phone:		408-464-5853
 *   Company:		SLAC
 *   Date:		02/2009
 *   Version:		.01
 *
 *   EPICS driver PIOP include. 
 *
\***************************************************************************/
#ifndef _DRV_PIOP_H_
#define _DRV_PIOP_H_

/*
** Control block functions for the PIOP
*/
#define PIOP_CBLK_FTP           0x01
#define PIOP_CBLK_VAX_FILENAME  0x02
#define PIOP_CBLK_STARTMODE     0x03
#define PIOP_CBLK_BOOTLOAD      0x04
#define PIOP_CBLK_STARTBOOT     0x05
#define PIOP_CBLK_READBYTE      0x06
#define PIOP_CBLK_READWORD      0x07
#define PIOP_CBLK_WRITEBYTE     0x08
#define PIOP_CBLK_WRITEWORD     0x09
#define PIOP_CBLK_READVOLTS     0x0A
#define PIOP_CBLK_READHEAD_1    0x0B
#define PIOP_CBLK_READHEAD_2    0x0C
#define PIOP_CBLK_CAMTEST       0x0D
#define PIOP_CBLK_RWREPEAT      0x0E
#define PIOP_CBLK_FTBITMAP      0x0F
#define PIOP_CBLK_TKBITMAP      0x10
 
#define PIOP_CBLK_PADPARAM      0x14
#define PIOP_CBLK_FOXHOME       0x15
#define PIOP_CBLK_MK2PARAM      0x16
#define PIOP_CBLK_TRIMSLED      0x17
#define PIOP_CBLK_NEWPHASE      0x18

/*************************************************************************
**
** Carry over these struct and comments from the old code for reference.
**
** Structure piop_cblk_ppbitmap_ts of PIOP control block for transmitting
** PP bitmap, like any other PIOP CBLK, cant be longer than 16 words. 
** Func word must contain either PIOP_CBLK_FTBITMAP (for conditioning FTP
** data) or PIOP_CBLK_TKBITMAP (for conditioning non-FTP (hsta_1beam) data
** collection).
**
** Note on structure piop_base_ppbitmap_ts:  Member bitmapb covers exactly
** all possible substitute beamcodes, which we assume to be 32-247, defined
** by RGBM_SUBSPP_MIN and RGBM_SUBSPP in rgbm_defines.hc.  We also
** assume N_BEAMS = 32.  Any element of base_pp having value 0 is ignored.
** If base_pp[0] >= 32 then base_pp[1] and base_pp[2] are ignored.
** If base_pp[0] = 32 then PIOP assumes all base beamcodes 1-31.
** If base_pp[0] > 32 then PIOP assumes all base beamcodes 1-31 plus
** beamcodes 0 and 248-255.  We dont have a way of specifying beamcode 0
** alone.  Note length of piop_base_ppbitmap_ts = exactly 30 bytes.

#define PPBITMAP_BASEPPS_MAXN  3            Max # of base beamcodes   
                                            specifiable in ppbitmap cblk.
 typedef struct
 {   unsigned char  base_pp[PPBITMAP_BASEPPS_MAXN],
                    bitmapb[30-PPBITMAP_BASEPPS_MAXN];
 } piop_base_ppbitmap_ts;

 typedef struct
 {   unsigned short        func;
     piop_base_ppbitmap_ts strng;
 } piop_cblk_ppbitmap_ts;
***********************************************************************/

/*
** Structure containing Camac package pointers and stat/data for
** all of the PIOP control block operations. Each thread allocates
** struct so that each thread gets it's own set of Camac packages.
*/

typedef struct
{
   void *cblk_pkg_p;    /* Control block package */
   void *ftpb_pkg_p;    /* FTP block pakage */
   void *sblk_pkg_p;    /* Status block package */
   unsigned short statdat1[2];
   unsigned short statdat2[69];
}  CAMBLOCKS_TS;

/*****************************************************************
** Prototypes for driver modules referenced internally by drvPIOP
*****************************************************************/
 
/* Init PIOP Camac packages for each PIOP thread */

vmsstat_t blockPIOPInit (CAMBLOCKS_TS *camblks_ps, unsigned short crate, unsigned short slot);

/* Write a PIOP control block */

vmsstat_t blockPIOPCblk (CAMBLOCKS_TS *camblocks_ps, unsigned short func, char *dat_p);

/* Main routine to IPL a PIOP */

void iplPIOPMain (PIOP_PVT *pvt_p,char *name, short crate, short slot);

#endif
