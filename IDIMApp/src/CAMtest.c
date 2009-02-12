/* Test camac routines */
#include <stdio.h>

#include "slc_macros.h"
#include "cam_proto.h"

void CAMtest (void)
{
  void *pkg_p;  /* A camac package */
  vmsstat_t iss;

#define NWRITES 20
#define ITERS 1

  unsigned short nops = NWRITES*2;

  typedef struct
  {
    int stat;
    int outdat;
  } out_ts;

  typedef struct
  {
    int stat;
    int indat;
  } in_ts;

  out_ts write_s[NWRITES] = { {0,0x0034ABCD},{0,0x0034ABCD},{0,0x0034ABCD},{0,0x0034ABCD},
    {0,0x0034ABCD},{0,0x0034ABCD},{0,0x0034ABCD},{0,0x0034ABCD},{0,0x0034ABCD},{0,0x0034ABCD},
    {0,0x0034ABCD},{0,0x0034ABCD},{0,0x0034ABCD},{0,0x0034ABCD},{0,0x0034ABCD},{0,0x0034ABCD},
			 {0,0x0034ABCD},{0,0x0034ABCD},{0,0x0034ABCD},{0,0x0034ABCD} };

  in_ts read_s[NWRITES];
 
  unsigned long dimctlw = 0x00205380;
  unsigned int dimstatdata[2] = {0,0x12345678};
  unsigned short bcnt = 4;
  unsigned short emask= 0xE0E0;
  unsigned long ctlww = 0x04145080;
  unsigned long ctlwr = 0x04005100;
  unsigned int j,k;
  /*--------------------------------*/
  printf ("Begin  Camac tests.\n");

  if (!SUCCESS(iss = cam_ini ()))
    goto egress;

  /*
  ** Allocate package for Crate verifier write/read
  */
  if (!SUCCESS(iss = camalo (NULL, &nops, &pkg_p)))
    goto egress;

  for (j=0; j<NWRITES; j++)
  {
    if (!SUCCESS(iss = camadd (&ctlww, &write_s[j], &bcnt, &emask, &pkg_p)))
      goto egress;
    if (!SUCCESS(iss = camadd (&ctlwr, &read_s[j],  &bcnt, &emask, &pkg_p)))
      goto egress;
  }

  for (j=0; j<ITERS; j++)
  {
    for  (k=0; k<NWRITES; k++)
    {
      read_s[k].stat = 0;
      read_s[k].indat = 0xFCFCFCFC;
      write_s[k].stat = 0;
      write_s[k].outdat += k;
    }

    if (!SUCCESS(iss = camgo (&pkg_p)))
    {
      printf("camgo error %x\n",(unsigned int) iss);
    }
    
    for (k=0; k<NWRITES; k++)
    {
      if (read_s[k].indat != write_s[k].outdat)
        printf ("Data mismatch for iter %d index %d outdat = %x indat = %x instat = %x\n",
                j, k, write_s[k].outdat, read_s[k].indat, read_s[k].stat);
    }
  }
  
  if (!SUCCESS(iss = camdel (&pkg_p)))
    goto egress;
  /*
  ** Do camio for DIM read
  */

  for (j=100; j>0; j--)
  {
     if (!SUCCESS(iss = camio (NULL, &dimctlw, &dimstatdata[1], &bcnt, &dimstatdata[0], 
              &emask)))
     {
        printf ("camio error %x\n", (unsigned int) iss);
     }
  }
egress:
  return;
} 
