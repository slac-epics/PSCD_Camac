/* Test camac routines */
#include <stdio.h>
#include "pscd_struct.h"       /* Registers etc. for PSCD access */

#include "slc_macros.hc"
#include "cam_proto.hm"

PSCD_CARD pscd_card;          /* Global PSCD struct */

/* epicsMemPartId is typedef of Object_Id. */

int main (int argc, char *argv[])
{
  void *pkg_p;  /* A camac package */
  vmsstat_t iss;
  char brch[4] = {'a','b','c','d'};
  unsigned short nops = 2;
  unsigned int outdat[3] = {0,0x0034ABCD, 0xCFCFCFCF};
  unsigned int indat[3]  = {0,0xFCFCFCFC, 0xA5A5A5A5};
  unsigned long dimctlw = 0x00201600;
  unsigned int dimstatdata[2];
  unsigned short bcnt = 4;
  unsigned short emask= 0xFFFF;
  unsigned long ctlww = 0x04141080;
  unsigned long ctlwr = 0x04041080;
  /*--------------------------------*/
  printf ("Begin  Camac tests.\n");

  if (!SUCCESS(iss = camioi ()))
    goto egress;
  /*
  ** Allocate package for Crate verifier write/read
  */
  if (!SUCCESS(iss = camalo (NULL, &nops, &pkg_p)))
    goto egress;
  if (!SUCCESS(iss = camadd (&ctlww, outdat, &bcnt, &emask, &pkg_p)))
    goto egress;
  if (!SUCCESS(iss = camadd (&ctlwr, indat,  &bcnt, &emask, &pkg_p)))
    goto egress;
  if (!SUCCESS(iss = camgo (&pkg_p)))
    goto egress;
  if (!SUCCESS(iss = camdel (&pkg_p)))
    goto egress;
  /*
  ** Do camio for DIM read
  */
  if (!SUCCESS(iss = camio (NULL, &dimctlw, &dimstatdata, &bcnt, &dimstatdata, &emask)))
    goto egress;

egress:
  return iss;
} 
