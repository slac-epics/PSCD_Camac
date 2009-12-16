/* 
** Contains:
**  iplPIOPIpl 
**  static iplPIOPRead
**  statuc iplPIOPDownload
**
** This module executes in the context of the device support thread.
**
*/

#include <devPIOP.h>
#include <drvPIOP.h>
#include <cam_proto.h>
#include <cctlwmasks.h>

/*
** Local prototypes for ipl
*/
static vmsstat_t iplPIOPRead (PIOP_PVT *pvt_p, char *img_name_p);
static vmsstat_t iplPIOPDownload (PIOP_PVT *pvt_p, short crate, short slot);

/*
** PIOP image Read once on first IPL.
*/
#define IMG_BLOCK_WC 70
#define IMG_MAX_BLOCKS 120
#define IMG_MAX_SIZE IMG_BLOCK_WC * IMG_MAX_BLOCKS * 2

char *ImagePIOP_p = NULL;  /* Pointer to PIOP image in memory */

/*************************************
 ** Top level IPL - Status returned to devPIOP via drvpvt 
 *************************************/
vmsstat_t iplPIOPMain (PIOP_PVT *pvt_p, short crate, short slot)
{
  char *name_p = (char *)pvt_p->val_p;
  vmsstat_t iss;
  /*------------------------------*/
  /*
  ** Read in the image if not already done.
  */
  if (!SUCCESS(iss = iplPIOPRead(pvt_p, name_p)))
    goto egress;
  iss = iplPIOPDownload(pvt_p, crate, slot);
egress:
  return (iss);
}

/*************************************************
** Read PIOP image into memory if not already done
*************************************************/
static vmsstat_t iplPIOPRead(PIOP_PVT *ppvt_p, char *img_name_p)
{
  FILE *piop_f = NULL;
  int idx = 0;         /* Index into array */
  unsigned short dum;  /* Throw away 2 bytes after length */
  union { char len[2]; int2u rlen; } rlen_u; /* In case we byte swap length */
#ifndef _X86_
  char tmp;           /* For byte swap */
  char *dat1_p;       /* Pointer for byte swap */
  int  i,j;           /* Indexs for byte/word swap */
  unsigned short stemp;    /* For word swap */
  unsigned short *short_p; /* Pointer for word swap */
#endif       
  int bytes_read;      /* Total bytes read */
  vmsstat_t iss = KLYS_OKOK;  /* Assume no error */
  char *fullpath_p;    /* Full path including env var PIOP_PATH */
  /*-------------------------------------*/
  if (ImagePIOP_p != NULL)    /* If already loaded then just exit with success */
    goto egress;
  if ( (fullpath_p = getenv("PIOP_PATH")) == NULL)
  {
    errlogSevPrintf (errlogMajor, 
                     "iplPIOPRead - Missing environment variable PIOP_PATH; no PIOP image\n");
    iss = KLYS_IPLNOIMG;
    goto egress;
  }
  fullpath_p = strcat(fullpath_p,img_name_p);
  if ((piop_f = fopen (fullpath_p, "r")) == NULL)
  {
    errlogSevPrintf (errlogMajor, 
                    "iplPIOPRead - Error opening PIOP image file %s with error %s\n",
                    fullpath_p, strerror(errno));
    iss = KLYS_IPLFNF;
    goto egress;
  }
  if ((ImagePIOP_p = calloc (1, IMG_MAX_SIZE)) == NULL)
  {
    errlogSevPrintf (errlogMajor, "iplPIOPRead - Cannot calloc for PIOP image.\n");
    iss = KLYS_IPLNOIMG;
    goto egress;
  }

  while (!feof (piop_f))
  {
    /*
    ** First read the 16 bit VMS record length followed by the data
    ** in each variable length record. 
    */
    bytes_read = fread (&rlen_u, 1, 2, piop_f);
#ifndef _X86_
    tmp = rlen_u.len[0];   /* We must byte swap the I*2 length from VMS. */
    rlen_u.len[0] = rlen_u.len[1];
    rlen_u.len[1] = tmp;
#endif
    if (feof (piop_f))  /* Hit eof reading record length after last record */
      goto swapbuf;
    /*
    ** Throw away the 2 bytes after the VMS RMX record length and decrement bytes to read.
    */
    bytes_read = fread (&dum, 1, 2, piop_f);
    rlen_u.rlen -= 2;
    /*
    ** Insure we don't exceed array size
    */
    if ( (idx + rlen_u.rlen) > IMG_MAX_SIZE)
    {
      errlogSevPrintf (errlogMajor, "iplPIOPRead - Max PIOP image size exceeded %d\n", idx);
      free (ImagePIOP_p);
      ImagePIOP_p = NULL;
      iss = KLYS_IPLNOEND;
      goto egress;
    }
    if ((bytes_read = fread (&ImagePIOP_p[idx], 1, rlen_u.rlen, piop_f)) != rlen_u.rlen)
    {
      errlogSevPrintf (errlogMajor, "iplPIOPRead - Cannot read %d bytes reading PIOP image\n", 
                       rlen_u.rlen);
      free (ImagePIOP_p);
      ImagePIOP_p = NULL;
      iss = KLYS_IPLFBAD;
      goto egress;
    }
#ifndef _X86_
    /*
    ** The PLX chip swaps the bytes on the way out so we need to swap them here.
    ** This also swaps the control word needed by the download.
    */
     dat1_p = &ImagePIOP_p[idx];
     for (j=0; j < bytes_read; j+=2)
     {
        tmp = *dat1_p;
        *dat1_p = *(dat1_p+1);
        *(dat1_p+1) = tmp;
        dat1_p += 2;
     }
#endif
    idx += rlen_u.rlen;
  }  /* EOF in data file */

swapbuf:
#ifndef _X86_
  /*
  ** The PCI bus in the PSCD reads 32 bit words little-endian so we also need 
  ** to swap the data words in each data block.
  */
  for (j=0; j<IMG_MAX_BLOCKS; j++)
  {
     short_p = (unsigned short *)&(ImagePIOP_p[j*IMG_BLOCK_WC*2]); /* Point to ctl word */
     short_p +=3;    /* Point to first data word in ftp or control block */
     for (i=0; i<FBLK_LENW/2; i++)
     { 
        stemp = *short_p;
        *short_p = *(short_p+1);
        *(short_p+1) = stemp;
        short_p += 2;
     }
  }
#endif
egress:
  fclose (piop_f);
  return (iss);
}

/***********************
** Download image to PIOP
***********************/
static vmsstat_t iplPIOPDownload(PIOP_PVT *pvt_p, short crate, short slot)
{
  unsigned long ploc;           /* PIOP crate/slot location */
  unsigned long                 /* Camac control words */
           ctlw1, ctlw2, ctlwa, ctlwb;
  vmsstat_t iss = KLYS_OKOK;    /* Assume good status */
  int      i;                   /* looper */
  unsigned int    ctlstat4;  /* Status from ctl op - 0 byte count */
  unsigned int   *statdat4_p;   /* Current stat/data as int4 */
  unsigned short *statdat2_p;   /* Current status/data as int2 */
  unsigned char  *byte_p;       /* byte pointer */
  int      block;               /* FTP/CTL block we're processing */
  int4u   statmsk = 0x007FBFFF; /* To check Camac status */
  int2u emaskf2e0 = 0xF2E0,     /* Various emasks */
        emaskf3e0 = 0xF3E0,
        emaske6e0 = 0xE6E0;
  unsigned short bcnt = 0,      /* running byte count and #ops in pkg */
                 nops = 2,      /* Operations in our package */
                 zero = 0;      /* Pass 0 bcnt */
  void *pkg_p = NULL;           /* Camac package for image download */
  /***************************************************
   ******* stat_data array for image download ********
   ******* cloned from the Klystron job in Fortrash **
   ***************************************************
   The structure of stat_data is:

    stat_data(0,i) = header.
                     high byte is entry number, except last is 'ff'x.
                     low  byte is type, 01 = ctl block,
                                        02 = ftp block.
    stat_data(1,i) = low status word returned by camac
    stat_data(2,i) = high   ''    ''     ''    ''   ''
    stat_data(3,i) = first word of camac data
    (.....)
     stat_data(69,i)= last  word of camac data

    ImagePIOP_p points to image in memory as array of charu.
    We recast it as a 2 dimensional array of I*2  
 
    blocks is the maximum number of blocks (of 65 i*2 words) used
    to transport the complete image -- bottom to top in one piece.

    blocks=120 will allow the following:
    image may reside between '0000'x and '3fff'x (16k bytes)
    and the image may be up to
         (120 - 2)*65 words long. (15340 bytes. 2 blocks are for header.)
	 
  ***************************************************/
  unsigned short *img_data2_p = (unsigned short *) ImagePIOP_p;
  /*-------------------------- code -------------------------*/
  /*
  ** CCTLW to reset PIOP
  */
  ploc  = (crate << CCTLW__C_shc) | (slot << CCTLW__M_shc);
  ctlw1 = ploc | CCTLW__F16 | CCTLW__F8 | CCTLW__F1;
  if (!SUCCESS(iss = camio (&ctlw1, NULL, &bcnt, &ctlstat4, &emaskf2e0)))
    goto egress;
  ctlstat4 &= statmsk;
  if ( (ctlstat4 << 16 != 0) || (ctlstat4 >> 16 != 0x0052) )
  {
     errlogPrintf ("iplPIOPDownload - Bad status from PIOP reset %08x\n", ctlstat4);
     iss = KLYS_NORESET;
     goto egress;
  }
  epicsThreadSleep(1.0);      /* Let PIOP finish reset */
  /*
  ** Get Camac pkg and download the image
  */
  if (!SUCCESS(iss = camalo (&nops, &pkg_p)))
     goto egress;
  ctlw1 = ploc | CCTLW__F8  | CCTLW__F2  | CCTLW__F1;
  ctlw2 = ploc | CCTLW__F16 | CCTLW__QM2 | CCTLW__XM2;

  for (i=0; i<IMG_MAX_BLOCKS; i++)
  {
     statdat2_p = &(img_data2_p[i*IMG_BLOCK_WC]);   /* Point to ctl word */
     block = *statdat2_p >> 8;
     if ((*statdat2_p & 0xFF) == 2)
     {
        ctlwa = ctlw1 | CCTLW__A1;
        ctlwb = ctlw2 | CCTLW__A1;
        bcnt = 2*67;
     }
     else 
     {
        ctlwa = ctlw1;
        ctlwb = ctlw2;
        bcnt = 2*16;
     }
     statdat2_p++;                 /* Increment past ctl word */
     byte_p = (unsigned char*)statdat2_p; /* Make char ptr */
     byte_p += 4; /* past status */
     statdat4_p = (unsigned int*) statdat2_p;   /* Actual stat/dat as int4u */
     if(!SUCCESS(iss = camadd(&ctlwa,&ctlstat4,&zero,&emaskf3e0,&pkg_p)))
        goto egress;
     if(!SUCCESS(iss = camadd(&ctlwb, statdat4_p, &bcnt, &emaske6e0,&pkg_p)))
        goto egress;
     if(!SUCCESS(iss = camgo(&pkg_p)))
        goto egress;
     ctlstat4    &= statmsk;
     *statdat4_p &= statmsk;
     if ( ((ctlstat4 & 0xBFFF)  != 0)    || (((ctlstat4 >> 16) & 0x7F) != 0x0053) ||
          ((*statdat4_p & 0xBFFF) != 0)  || (((*statdat4_p >> 16) & 0x7F)  != 0x0053) )
     {
        errlogPrintf ("iplPIOPDownload - Block %x bad status from PIOP download %8.8x %8.8x\n",
                      block, ctlstat4, *statdat4_p);
        iss = KLYS_NOIPL;
        goto egress;
     }
     /*
     ** Check if IPL is done
     */
     if ((*(statdat2_p-1) & 0xff00) == 0xff00)
        break;
     if (!SUCCESS(iss = camalo_reset(&pkg_p)))
        goto egress;

     epicsThreadSleep(0.02);
  }  /* End of loop to download the image */
  /*
  ** This PIOP seems now to be IPL'ed.Sleep a bit so PIOP can digest the image.
  */
  epicsThreadSleep(0.050);
egress:  
  camdel (&pkg_p);
  return (iss);
}
