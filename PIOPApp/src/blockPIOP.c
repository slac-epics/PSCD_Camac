/*
** Module to do piop FTP, Control and Status block Camac I/O.
** This module executes in the context of the device support thread.
**  
** Contains:
**  blockPIOPInit           Init Camac pkgs
**  static blockPIOPCksum   Calc checksum
**  blockPIOPCblk           Send control block
**  blockPIOPSblk           Read status block
**  blockPIOPFblk           Read FTP block
**
*/

#include <devPIOP.h>
#include <drvPIOP.h>

/*
** Construct the camac packages for the types of PIOP I/O we can do
*/
vmsstat_t blockPIOPInit (CAMBLOCKS_TS *camblocks_ps, unsigned short crate, unsigned short slot)
{
   vmsstat_t iss = KLYS_OKOK; /* Assume no error */
   unsigned short emaskf3c0 = 0xF3C0;  /* emask */
   unsigned short bcntcblk = CBLK_LENB, /* control block byte count */
                  bcntfblk = FBLK_LENB, /* FTP block byte count */
                  bcntsblk = SBLK_LENB, /* Status block byte count */
                  nops = 2,    /* Operations in each package */
                  zero = 0;    /* A 0 by reference */
   unsigned long ploc;         /* PIOP crate/slot location */
   unsigned long ctlw;         /* Control word */ 
   /*-------------------------------------*/

   ploc  = (crate << CCTLW__C_shc) | (slot << CCTLW__M_shc);
   /****** Control Block *****/
   if (!SUCCESS(iss = camalo(&nops, &(camblocks_ps->cblk_pkg_p))))
      goto egress;
   ctlw = ploc | CCTLW__F8 | CCTLW__F2 | CCTLW__F1 | CCTLW__A0;
   if (!SUCCESS(iss = camadd(&ctlw, &(camblocks_ps->stat32), &zero, 
                             &emaskf3c0, &(camblocks_ps->cblk_pkg_p))))
      goto egress;
   ctlw = ploc | CCTLW__F16 | CCTLW__A0 | CCTLW__QM2 | CCTLW__XM2;
   if (!SUCCESS(iss = camadd(&ctlw, &(camblocks_ps->statdat), &bcntcblk, 
                             &emaskf3c0, &(camblocks_ps->cblk_pkg_p)))) 
     goto egress;
   /******  FTP Block *****/
   if (!SUCCESS(iss = camalo(&nops, &(camblocks_ps->fblk_pkg_p))))
      goto egress;
   ctlw = ploc | CCTLW__F8 | CCTLW__F2 | CCTLW__F1 | CCTLW__A1;
   if (!SUCCESS(iss = camadd(&ctlw, &(camblocks_ps->stat32), &zero, 
                             &emaskf3c0, &(camblocks_ps->fblk_pkg_p))))
      goto egress;
   ctlw = ploc | CCTLW__F0 | CCTLW__A1 | CCTLW__QM2 | CCTLW__XM2;
   if (!SUCCESS(iss = camadd(&ctlw, &(camblocks_ps->statdat), &bcntfblk, 
                             &emaskf3c0, &(camblocks_ps->fblk_pkg_p)))) 
     goto egress;
   /****** Status Block *****/
   if (!SUCCESS(iss = camalo(&nops, &(camblocks_ps->sblk_pkg_p))))
      goto egress;
   ctlw = ploc | CCTLW__F8 | CCTLW__F2 | CCTLW__F1 | CCTLW__A2;
   if (!SUCCESS(iss = camadd(&ctlw, &(camblocks_ps->stat32), &zero, 
                             &emaskf3c0, &(camblocks_ps->sblk_pkg_p))))
      goto egress;
   ctlw = ploc | CCTLW__F0 | CCTLW__A2 | CCTLW__QM2 | CCTLW__XM2;
   if (!SUCCESS(iss = camadd(&ctlw, &(camblocks_ps->statdat), &bcntsblk, 
                             &emaskf3c0, &(camblocks_ps->sblk_pkg_p)))) 
     goto egress;
   camblocks_ps->last_counter = 0;
egress:
   return (iss);
}

/*
** Compute checksum for a block that's sent to the PIOP.
*/
static void blockPIOPCksum (short *wdat_p, int wlen)
{
   int i;
   /*-------------------------- code -------------------------*/
   wdat_p[wlen-1] = 0;
   for (i=0; i<wlen-1; i++)
      wdat_p[wlen-1] += wdat_p[i];
   return;
}

/*
** Send a control block with the requested function and data.
*/
vmsstat_t blockPIOPCblk (CAMBLOCKS_TS *camblocks_ps, unsigned short infunc, void *indat_p,
                         int datalenb, int tries, float delay)
{
   vmsstat_t iss;
   CAM_CBLK_TS *cam_cblk_ps = (CAM_CBLK_TS*) camblocks_ps->statdat;
   /*-------------------------- code -------------------------*/
   memset (cam_cblk_ps, 0, sizeof(CAM_CBLK_TS)); /* Clear the area */
   cam_cblk_ps->func = infunc;
   memcpy (&(cam_cblk_ps->dat), indat_p, datalenb);
   /*
   ** No checksum for bitmaps
   */
   if ((infunc != PIOP_CBLK_TKBITMAP) && (infunc != PIOP_CBLK_FTBITMAP))
     blockPIOPCksum ( (short *) &(cam_cblk_ps->func), CBLK_LENW);
   epicsThreadSleep(delay); /* Delay before first write attempt */

#ifndef _X86_
   camSwapWords ( &(cam_cblk_ps->func), CBLK_LENW); 
#endif
   do
   {
     if (!SUCCESS(iss = camgo (&(camblocks_ps->cblk_pkg_p))))
        epicsThreadSleep(delay);
   } while (!SUCCESS(iss) && (tries-- > 0));
   return (iss);
}

/*
** Read the status block.
*/
vmsstat_t blockPIOPSblk (CAMBLOCKS_TS *camblocks_ps, void *outdat_p,
                           int tries, float delay)
{
   vmsstat_t iss;
   STS_BLK_TS *cam_sblk_ps = (STS_BLK_TS*) camblocks_ps->statdat;
   /*-------------------------- code -------------------------*/
   memset (cam_sblk_ps, 0, sizeof(STS_BLK_TS)); /* Clear the area */
   do
   {
      /*
      ** Try 3 times in a row for each pass.
      */
      iss = camgo (&(camblocks_ps->sblk_pkg_p));
      if (!SUCCESS(iss))
         iss = camgo (&(camblocks_ps->sblk_pkg_p));
      if (!SUCCESS(iss))
         iss = camgo (&(camblocks_ps->sblk_pkg_p));
      if (!SUCCESS(iss))
         epicsThreadSleep(delay);
   } while (!SUCCESS(iss) && (tries-- > 0));
   if SUCCESS(iss)
   {
#ifndef _X86_
      camSwapWords (&(cam_sblk_ps->sid), SBLK_LENW); 
#endif
      memcpy (outdat_p, &(cam_sblk_ps->sid), SBLK_LENB); /* Data to record */
      /*
      ** Make sure PIOP is still alive and incrementing its counter.
      */
      if (cam_sblk_ps->counter != camblocks_ps->last_counter)
	camblocks_ps->last_counter = cam_sblk_ps->counter;  /* PIOP alive. Save last counter */
      else
      {
	iss = KLYS_PIOP_DEAD;
        camblocks_ps->last_counter = 0;
      } 
   }
   return (iss);
}

/*
** Read the FTP block.
*/
vmsstat_t blockPIOPFblk (CAMBLOCKS_TS *camblocks_ps, void *outdat_p,
                           int tries, float delay)
{
   vmsstat_t iss;
   FTP_CAMAC_TS *ftp_camac_ps = (FTP_CAMAC_TS*) camblocks_ps->statdat;

   /*-------------------------- code -------------------------*/
   epicsThreadSleep(delay); /* Delay before first read attempt */
   do
   {
     if (!SUCCESS(iss = camgo (&(camblocks_ps->fblk_pkg_p))))
        epicsThreadSleep(delay);
   } while (!SUCCESS(iss) && (tries-- > 0));
   if SUCCESS(iss)
   {
#ifndef _X86_
      camSwapWords (&(ftp_camac_ps->ftp_read_s),FBLK_LENW); 
#endif
      memcpy (outdat_p, &(ftp_camac_ps->ftp_read_s), sizeof(FTP_READ_TS)); /* Data to record */
   }
   return(iss);
}
