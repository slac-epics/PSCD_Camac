/***************************************************************************\
 *   $Id: drvPIOP.h,v 1.4 2010/02/08 19:35:08 rcs Exp $
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


/*******************************************************************
** Structure containing Camac package pointers and stat/data for
** all of the PIOP control block operations. Each thread allocates
** the struct so that each thread gets it's own set of Camac packages.
********************************************************************/

/*
** Length of the various PIOP blocks in bytes and words
*/
#define CBLK_LENB 32
#define FBLK_LENB 134
#define SBLK_LENB 64
#define CBLK_LENW CBLK_LENB/2
#define FBLK_LENW FBLK_LENB/2
#define SBLK_LENW SBLK_LENB/2

/*
** Generic control block sent to PIOP
** Note that the checksum goes after the last data word.
** That data after the function is function dependent.
*/
typedef struct
{
   unsigned int   stat32;
   unsigned short func;
   unsigned short dat[CBLK_LENW-2];
   unsigned short cksum;
} CAM_CBLK_TS;


/*
** Status block received from PIOP
*/
typedef struct
{
   unsigned int   stat32;
   unsigned short sid;
   unsigned short padid;
   unsigned short ts;
   unsigned short swrd;
   unsigned short spare1;
   short          bvlt;
   short          bcur;
   short          amp_mean;
   short          amp_jitter;
   short          phase_mean;
   short          phase_jitter;
   short          mk2_phase;
   unsigned short dsta[4];
   unsigned short mksu_id;
   unsigned short counter;
   unsigned short spare14[14];
} STS_BLK_TS;

/*
** FTP info sent in CBLK
*/
typedef struct
{
   short channel;
   short start_delay_out;
   short step_size_out;
} FTP_CBLK_TS;

/*
** FTP info for beam PP map and tries
*/
typedef struct
{
   short ms_per_try;
   short tries;
   short pp;    /* 0=ANY 1=LCLS Beam */
   short spare[7];
} FTP_INFO_TS;

/*
** FTP data read (past tense)
*/
typedef struct
{
   short status_in;
   short start_delay_in;
   short step_size_in;
   short dat[64];
} FTP_READ_TS;

/*
** FTP Camac read struct.
*/
typedef struct
{
  unsigned int stat32;
  FTP_READ_TS ftp_read_s;
} FTP_CAMAC_TS;

/*
** Waveform block for PIOP. First struct is info to
** set up the FTP in the CBLK. Second struct give PP map info
** and delay/tries for the read.
*/
typedef struct
{
   FTP_CBLK_TS ftp_cblk_s;
   FTP_INFO_TS ftp_info_s;
   FTP_READ_TS ftp_read_s;
} FTP_WAVE_TS;

/*
** All the camac packages we use. Since only one at a time can be active, all use
** the same stat32 for those packets that don't xfer data and statdat for those
** that do. We keep the last counter to insure that the PIOP hasn't died.
*/ 
typedef struct
{
   void *cblk_pkg_p;    /* Control block package */
   void *fblk_pkg_p;    /* FTP block pakage */
   void *sblk_pkg_p;    /* Status block package */
   unsigned int stat32;              /* Just status */
  /**
   ** Camac status + ftp or sblk data.
   ** Nota bene!! we add an extra word in case of big-endian word swap.
   */
   char  statdat[sizeof(FTP_CAMAC_TS)];
   short spare[1];
   unsigned short last_counter;  /* Last counter from status block */
}  CAMBLOCKS_TS;

/*****************************************************************
** Prototypes for driver modules referenced internally by drvPIOP
*****************************************************************/
 
/* Init PIOP Camac packages for each PIOP thread */

vmsstat_t blockPIOPInit (CAMBLOCKS_TS *camblocks_ps, unsigned short crate, unsigned short slot);

#ifndef _X86_
/* Generic routine to swap words                 */

void      blockPIOPSwap (void *buf_p, int numwords);
#endif

/* Main routine to IPL a PIOP */

vmsstat_t iplPIOPMain (PIOP_PVT *pvt_p, short crate, short slot);

/* Write a PIOP control block */

vmsstat_t blockPIOPCblk (CAMBLOCKS_TS *camblocks_ps, unsigned short func, void *indat_p,
                         int inlen, int tries, float delay);

/* Read a PIOP status block */

vmsstat_t blockPIOPSblk (CAMBLOCKS_TS *camblocks_ps, void *outdat_p,
			 int tries, float delay);

/* Read a PIOP FTP block */

vmsstat_t blockPIOPFblk (CAMBLOCKS_TS *camblocks_ps, void *outdat_p,
			 int tries, float delay);

#endif
