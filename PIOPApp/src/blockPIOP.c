/*
** Module to do piop FTP, Control and Status block Camac I/O.
** This module executes in the context of the device support thread.
**  
** Contains:
**  blockPIOPInit
**  blockPIOPCblk
**
*/

#include <devPIOP.h>
#include <drvPIOP.h>

/*
** Length of the various PIOP blocks
*/
#define CBLK_LEN 32
#define FTPB_LEN 134
#define SBLK_LEN 64

/*
** Construct the camac packages for the types of PIOP I/O we can do
*/
vmsstat_t blockPIOPInit (CAMBLOCKS_TS *camblks_ps, unsigned short crate, unsigned short slot)
{
   vmsstat_t iss = KLYS_OKOK; /* Assume no error */
   unsigned short emaskf3c0 = 0xF3C0;  /* emask */
   unsigned short bcntcblk = CBLK_LEN, /* control block byte count */
                  bcntftpb = FTPB_LEN, /* FTP block byte count */
                  bcntsblk = SBLK_LEN, /* Status block byte count */
                  nops = 2,       /* Operations in each package */
                  zero = 0;       /* A 0 by reference */
   unsigned long ploc;           /* PIOP crate/slot location */
   unsigned long ctlw;            /* Control word */ 
   /*-------------------------------------*/

   ploc  = (crate << CCTLW__C_shc) | (slot << CCTLW__M_shc);
   /****** Control Block *****/
   if (!SUCCESS(iss = camalo(&nops, &(camblks_ps->cblk_pkg_p))))
      goto egress;
   ctlw = ploc | CCTLW__F8 | CCTLW__F2 | CCTLW__F1 | CCTLW__A0;
   if (!SUCCESS(iss = camadd(&ctlw, &(camblks_ps->statdat1), &zero, 
                             &emaskf3c0, &(camblks_ps->cblk_pkg_p))))
      goto egress;
   ctlw = ploc | CCTLW__F16 | CCTLW__A0 | CCTLW__QM2 | CCTLW__XM2;
   if (!SUCCESS(iss = camadd(&ctlw, &(camblks_ps->statdat2), &bcntcblk, 
                             &emaskf3c0, &(camblks_ps->cblk_pkg_p)))) 
     goto egress;
   /******  FTP Block *****/
   if (!SUCCESS(iss = camalo(&nops, &(camblks_ps->ftpb_pkg_p))))
      goto egress;
   ctlw = ploc | CCTLW__F8 | CCTLW__F2 | CCTLW__F1 | CCTLW__A1;
   if (!SUCCESS(iss = camadd(&ctlw, &(camblks_ps->statdat1), &zero, 
                             &emaskf3c0, &(camblks_ps->ftpb_pkg_p))))
      goto egress;
   ctlw = ploc | CCTLW__F0 | CCTLW__A1 | CCTLW__QM2 | CCTLW__XM2;
   if (!SUCCESS(iss = camadd(&ctlw, &(camblks_ps->statdat2), &bcntftpb, 
                             &emaskf3c0, &(camblks_ps->ftpb_pkg_p)))) 
     goto egress;
   /****** Status Block *****/
   if (!SUCCESS(iss = camalo(&nops, &(camblks_ps->sblk_pkg_p))))
      goto egress;
   ctlw = ploc | CCTLW__F8 | CCTLW__F2 | CCTLW__F1 | CCTLW__A2;
   if (!SUCCESS(iss = camadd(&ctlw, &(camblks_ps->statdat1), &zero, 
                             &emaskf3c0, &(camblks_ps->sblk_pkg_p))))
      goto egress;
   ctlw = ploc | CCTLW__F0 | CCTLW__A2 | CCTLW__QM2 | CCTLW__XM2;
   if (!SUCCESS(iss = camadd(&ctlw, &(camblks_ps->statdat2), &bcntsblk, 
                             &emaskf3c0, &(camblks_ps->sblk_pkg_p)))) 
     goto egress;
egress:
   return (iss);
}

/*
** Send a control block with the requested function
*/
vmsstat_t blockPIOPCblk (CAMBLOCKS_TS *camblocks_ps, unsigned short func, char *dat_p)
{
   vmsstat_t iss = KLYS_OKOK;
   /*-------------------------- code -------------------------*/
   camblocks_ps->statdat2[2] = func;
   memcpy (&(camblocks_ps->statdat2[3]), dat_p,  CBLK_LEN-2);
   iss = camgo ( &(camblocks_ps->cblk_pkg_p));
   return (iss);
}
