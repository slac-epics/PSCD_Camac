/* Misc CAM tests */
#include <stdio.h>

#include "slc_macros.h"
#include "cam_proto.h"
#include "camblkstruc.hm"          /* mbcd_pkt_ts, mbcd_pkghdr_ts,            */
                                   /* mbcd_pkg_ts, mbcd_savep_ts,             */
                                   /* mbcd_stad_ts                            */
#include "epicsThread.h"


 
void MISCtest (void)
{
  void *pkg_p;  /* A camac package */
  mbcd_pkg_ts   *ipkg_p = pkg_p; /* Pointer to constructed package */
  mbcd_savep_ts *savep_p;
  unsigned int  *dpdata_p;

  vmsstat_t iss;
  int j;
  unsigned short nops = 1;
  unsigned long dimctlw = 0x00205200; /* F0 A0 crate 5 slot 4 */
  unsigned short bcnt = 4;
  unsigned short emask= 0xE0E0;
  unsigned int dimstatdata[2] = {0,0};
  /*--------------------------------*/
  printf ("Begin MISC test.\n");

  /*
  ** Allocate high priority package for DIM read
  */

  if (!SUCCESS(iss = camaloh (&nops, &pkg_p)))
    goto egress;
  if (!SUCCESS(iss = camadd (&dimctlw, &dimstatdata, &bcnt, &emask, &pkg_p)))
    goto egress;

  savep_p = &pkg_p->mbcd_pkt[pkg_p->hdr.nops];
  dpdata_p = (unsigned int) savep_p[0].hrdw_p; 


  for (j=0; j<1; j++)
  {
    if (!SUCCESS(iss = camgo_start_pscd (&pkg_p, 1)))
    {
      printf("Start with wait camgo error code %x stat word = %x\n",(unsigned int) iss, dpdata[0]);
    }
    printf ("With wait misc Stat = %x Misc Data = %x\n",dpdata[0], dpdata[1]);

    /* without wait */
    if (!SUCCESS(iss = camgo_start_pscd(&pkg_p, 0)))
    {
      printf("Start with wait camgo error code %x stat word = %x\n",(unsigned int) iss, dpdata[0]);
    }
    printf ("Without wait immediate return Stat = %x Misc Data = %x\n",dpdata[0], dpdata[1]);
    epicsThreadSleep(0.1);
    printf ("Without wait after sleep Stat = %x Misc Data = %x\n",dpdata[0], dpdata[1]);
    
  }
  if (!SUCCESS(iss = camdel (&pkg_p)))
    goto egress;
egress:
  return;
}
