/* ead DIM modules */
#include <stdio.h>

#include "slc_macros.h"
#include "cam_proto.h"
#include "epicsThread.h"

#include "DIMtest.h"

DIM_TS dim_s;
 
void DIMtest (void)
{
  void *pkg_p;  /* A camac package */
  vmsstat_t iss;
  int j;
  unsigned short nops = 1;
  unsigned long dimctlw = 0x00205200; /* F0 A0 crate 5 slot 4 */
  unsigned short bcnt = 4;
  unsigned short emask= 0xE0E0;
  /*--------------------------------*/
  printf ("Begin DIM read.\n");

  if (!SUCCESS(iss = cam_ini ()))
    goto egress;

  /*
  ** Allocate package for DIM read
  */

  if (!SUCCESS(iss = camalol (NULL, &nops, &pkg_p)))
    goto egress;
  if (!SUCCESS(iss = camadd (&dimctlw, &dim_s, &bcnt, &emask, &pkg_p)))
    goto egress;

  /*for (j=0; j<1; j++)*/
  while(1)
  {
    dim_s.stat = 0xABCDEF89;
    dim_s.data = 0x12345678;
    if (!SUCCESS(iss = camgo (&pkg_p)))
    {
      printf("camgo error code %x stat word = %x\n",(unsigned int) iss, dim_s.stat);
    }
    printf ("Stat = %x Data = %x\n",dim_s.stat, dim_s.data);
  }
  if (!SUCCESS(iss = camdel (&pkg_p)))
    goto egress;
egress:
  return;
}
