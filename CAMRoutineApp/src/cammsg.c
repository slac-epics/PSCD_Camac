/*=====================================================================
**
** The cammsg routine takes the VMS style status word from a cam routine
** call and returns a text string.
**
**=====================================================================
*/

#include "slc_macros.h"        /* vmsstat_t */

#define CAM_FACILITY     0x0802
#define FACILITY_MASK    0x0FFF0000
#define FACILITY_RSHIFT  16
#define MSGNUM_MASK      0x00000FF8
#define MSGNUM_RSHIFT    3  
/*
** Last legal 0-based message number
*/
#define NUM_MSGS 40

static char *cam_msgs[NUM_MSGS] = 
{
  "CAMAC operation successful",
  "Found CAMAC Control Block NFG",
  "CAMADD: Attempt to add more packets than allocated",
  "CAMADD: MBCD CAMAC byte count .gt. 256",
  "CAMADD: CAMAC odd byte count",
  "CAMALO: Illegal request for too many packets",
  "CAMGO: software timeout. No PSCD response.",
  "CAMGO: PSCD failure",
  "CAMGO: crate timeout",
  "CAMGO: failed to exhaust word count",
  "CAMGO: failed to terminate on End-Of-Scan",
  "CAMGO: failed to terminate on End-Mode_Signal",
  "CAMGO: no X respnse",
  "CAMGO: no Q respnse",
  "CAMGO: Bad byte count from serial branch",
  "CAMVDRVR: NETHEADERLEN.GT.PKGLEN (Software problem)",
  "CAMGO_V ERROR: Special options not allowed with repeat",
  "VIRTUAL CAMAC -- ERROR: Repeat function not available due to lack of resources",
  "CAMADD/CAMGO: Unable to execute packet with Pointer = 0",
  "CAMGO: No packets specified in Control Block",
  "CAMGO: Data buffer is not word aligned",
  "CAMGO: PSCD board error -- TDV always busy",
  "CAMADD: Bytecount .gt. zero with no data xfer",
  "CAMADD: PSCD would run off end of segment containing status/data",
  "CAMMOD: Illegal change to CCTLW",
  "Request message from CAMGO_V has bad checksum",
  "CAMAC - Invalid handler command",
  "CAMAC - Invalid module ID",
  "CAMAC - No Trigger Received",
  "CAMAC - Disable LAM failure",
  "CAMAC - Enable LAM failure",
  "CAMAC - bad pointer to dynamic CAMAC data list",
  "CAMAC - Unable to (re)allocate dynamic memory",
  "CAMAC - Invalid crate address, unable to build control word",
  "CAMAC - Invalid crate slot #, unable to build control word",
  "CAMAC - Invalid function code, unable to build control word",
  "CAMAC - Invalid subaddress, unable to build control word",
  "CAMAC - Invalid mode, unable to build control word",
  "CAMAC - Invalid control word",
  "MISC  - Miscellaneous Non-Camac error"
};

/*
** Check the facility for CAM message and ignore severity and 
** other VMS msg fields. Extract and index the msg number and return
** the corresponding string.
*/
char *cammsg (vmsstat_t msgnum)
{
   char *retmsg;
   int   msgidx;
   /*--------------------------------------------*/
   if (( (msgnum & FACILITY_MASK) >>  FACILITY_RSHIFT) != CAM_FACILITY)
   {
      retmsg = "CAMMSG: Message number not Camac facility";
      goto egress;
   }
   /*
   ** VMS msg idx is 1-based
   */
   msgidx = ((msgnum & MSGNUM_MASK)  >> MSGNUM_RSHIFT) - 1 ;
   if (msgidx >= NUM_MSGS)
   {
      retmsg = "CAMMSG: Illegal message number";
      goto egress;
   }
   retmsg = cam_msgs[msgidx];
egress:
   return (retmsg);
}
