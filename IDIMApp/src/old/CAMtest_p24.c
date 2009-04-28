/* Test camac routines */
#include "drvPSCDLib.h"
#include "devIDIM.h"
#include <stdio.h>
#include "slc_macros.h"
#include "cam_proto.h"
#include <registryFunction.h>

UINT32 CAM_Test ()
{
  void *pkg_p;  /* A camac package */
  vmsstat_t iss;

#define NWRITES 20
#define ITERS 50

  unsigned short nops = NWRITES*2;

  typedef struct
  {
    unsigned int stat;
    unsigned int   outdat;
  } out_ts;

  typedef struct
  {
    int stat;
    int indat;
  } in_ts;

  out_ts write_s[20] = { {0,0x1234ABCD},{0,0x1234ABCD},{0,0x1234ABCD},
         {0,0x1234ABCD}, {0,0x1234ABCD},{0,0x1234ABCD},{0,0x1234ABCD},
         {0,0x1234ABCD}, {0,0x1234ABCD},{0,0x1234ABCD}, {0,0x1234ABCD},
         {0,0x1234ABCD}, {0,0x1234ABCD},{0,0x1234ABCD},{0,0x1234ABCD},
         {0,0x1234ABCD}, {0,0x1234ABCD},{0,0x1234ABCD},{0,0x1234ABCD},
         {0,0x1234ABCD} };

  in_ts read_s[NWRITES];
 
  unsigned long dimctlw = 0x00005200;
  unsigned int dimstatdata[2] = {0,0x12345678};
  unsigned short bcnt = 4;
  unsigned short emask= 0xE0E0;
  unsigned long ctlwwm = 0x84145080; /* Write with MPC */
  unsigned long ctlww  = 0x04145080;
  unsigned long ctlwrm = 0x84045080; /* Read with MPC */
  unsigned long ctlwr  = 0x04045080;
  unsigned int j,k;
  /*--------------------------------*/
  printf ("Begin  Camac tests.\n"); 
  /*
  ** Allocate package for Crate verifier write/read
  */
  if (!SUCCESS(iss = camalo (&nops, &pkg_p)))
    goto egress;

  for (j=0; j<NWRITES; j++)
  {
    if (!SUCCESS(iss = camadd (&ctlwwm, &write_s[j], &bcnt, &emask, &pkg_p)))
      goto egress;
    if (j < NWRITES-1)
    {
      if (!SUCCESS(iss = camadd (&ctlwrm, &read_s[j],  &bcnt, &emask, &pkg_p)))
        goto egress;
    }
    else
    {
      if (!SUCCESS(iss = camadd (&ctlwr, &read_s[j],  &bcnt, &emask, &pkg_p)))
        goto egress;
    }
   }

  for (j=0; j<ITERS; j++)
  {  
    for  (k=0; k<NWRITES; k++)
    {
      read_s[k].stat = 0;
      read_s[k].indat = 0xFCFCFCFC;
      write_s[k].stat = 0;
      write_s[k].outdat += (k);
    }
    printf ("Last data word written %08x\n",write_s[NWRITES-1].outdat);
    if (!SUCCESS(iss = camgo (&pkg_p)))
      goto egress;

    for (k=0; k<NWRITES; k++)
    {
      if ((read_s[k].indat & 0x00FFFFFF) != (write_s[k].outdat & 0x00FFFFFF))
        printf ("Data mismatch for iter %d index %d outdat = %x indat = %x instat = %x\n",
                j, k, write_s[k].outdat, read_s[k].indat, read_s[k].stat);
    }
  }  /* End ITERS loop */
  
  if (!SUCCESS(iss = camdel (&pkg_p)))
    goto egress;
  /*
  ** Do camio for DIM read
  */
  /**********************************************************
  for (j=1; j>0; j--)
  {
     dimstatdata[1] = 0x12345678;
     if (!SUCCESS(iss = camio (&dimctlw, &dimstatdata[1], &bcnt, &dimstatdata[0], 
              &emask)))
     {
        printf ("camio error %x\n", (unsigned int) iss);
     }
       printf ("DIM stat %x data %x\n", dimstatdata[0], dimstatdata[1]);
  }
  ************************************************************/
  /*
  ** Do write/read to Crate verifier with camio. use dim statdata.
  */
  /******************
  dimstatdata[1] = 0x1234ABCD;
  if (!SUCCESS(iss = camio (&ctlww, &dimstatdata[1], &bcnt, &dimstatdata[0], 
              &emask)))
  {
     printf ("camio CV write error %x\n", (unsigned int) iss);
  }
     printf ("CV stat %x\n", dimstatdata[0]);

  dimstatdata[1] = 0x12345678;
  if (!SUCCESS(iss = camio (&ctlwr, &dimstatdata[1], &bcnt, &dimstatdata[0], 
              &emask)))
  {
     printf ("camio CV read error %x\n", (unsigned int) iss);
  }
     printf ("CV stat %x data %x\n", dimstatdata[0], dimstatdata[1]);
  *****************************/
egress:
  return 0;
}

epicsRegisterFunction(CAM_Test); 
