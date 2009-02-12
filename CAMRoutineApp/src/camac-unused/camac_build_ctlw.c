/*
       **MEMBER**=SLCRMX:REENTRANT
       **CMS**=RMX_CAM
===============================================================================

  Abs:  Build the CAMAC control word

  Name: CAMAC_BUILD_CTLW.C86

  Args:  crate_p                        CAMAC crate address
           Use:  integer                NOTE: optional
           Type: longword (unsigned) *
           Acc:  read-only      
	   Mech: By reference      

         slot_p                         CAMAC module slot number 
           Use:  integer                NOTE: optional
           Type: longword (unsigned) *
           Acc:  read-only      
	   Mech: By reference      

         function_p                     CAMAC function code
           Use:  integer                NOTE: optional
           Type: longword (unsigned) *
           Acc:  read-only      
	   Mech: By reference      

         subaddress_p                   CAMAC module subaddress 
           Use:  integer                NOTE: optional
           Type: longword (unsigned) *
           Acc:  read-only      
	   Mech: By reference      

         mode_p                         CAMAC driver mode
           Use:  integer                NOTE: optional
           Type: longword (unsigned) *
           Acc:  read-only      
	   Mech: By reference      

  Rem: This routine builds the address necessary to access a CAMAC module
       via the CAMAC controller. This would be called prior to
       performing a camac operations (ie: CAMIO,CAMADD,CAMMOD,etc. )

       The format of the CAMAC control word, CTLW is as follows: 
       (ref: SLC Basic Users Guide (BUG) for additional information)

         bits 0-3    subaddress
              4      re pack data into smaller blocks mode
              5      8-bit  pack mode
              6      unused
              7-11   slot #
              12-15  crate address
              16-20  function code
              21-23  Enable scan mode counters
              24-25  Scan increment mode
              26     24-bit pack mode
              27-30  Transfer/terminate mode
              31     0 if last packet of package; otherwise
                     more packets coming.
  
  Side: A check of the crate, slot, function code and subaddress
        to verify that they are within SLC MBCD as well as CAMAC 
        standards is done. 

        See the CAMAC Primer from Los Alamos (LA-UR-82-2718) for more
        information regarding CAMAC standards.
 
  Ret:  unsigned long -
            If successful, the CAMAC control word is returned
            Otherwise, a zero is returned.

  Proto: cam_proto.hm

  Auth: 07-Jun-1992, K. Luchini (LUCHINI)
  Rev:  05-Aug-1992, S. Levitt (LEVITT)

--------------------------------------------------------------------------------

  Mod:
        24-Jan-1997, K. Luchini (LUCHINI):
           convert to rmx version

==============================================================================*/

/*
** Include Files
*/
#include <stdio.h>
#include "camdef.hc"             /* CAMAC error codes           */
#include "slc_macros.hc"         /* slc macro definitions       */
#include "camac.hm"              /* CAMAC parameter defs        */
#include "camac_standards.hc"    /* CAMAC standards             */
#include "camac_types.hc"        /* CAMAC typedefs              */


unsigned long camac_build_ctlw( unsigned long const * crate_p, 
 	          	        unsigned long const * slot_p, 
	 	                unsigned long const * function_p, 
		                unsigned long const * subaddress_p, 
		                unsigned long const * mode_p )
{
  vmsstat_t     status = CAM_OKOK;    /* return status code    */
  unsigned long new_mode;             /* set camac mode        */
  unsigned long camac_modes =  CTLW_XM1 | CTLW_XM2 |
                               CTLW_QM1 | CTLW_QM2 |
                               CTLW_P24 |
                               CTLW_IN  | CTLW_ILQ |
                               CTLW_SC  | CTLW_SM  | CTLW_SA |
                               CTLW_P8  | CTLW_RE_PACK;
  ctlw_tu       cctlw_u;                 /* CAMAC control word            */
  static CAMAC_FUNCTION_CODES;           /* CAMAC standard function codes */
  static CAMAC_SUBADDRESS;               /* CAMAC subaddress'             */



  /*
  ** Add the CRATE addr
  */
  cctlw_u._i = 0;    
  if (  crate_p != NULL )
  {
     if ((*crate_p <= MAX_CRATE_ADR) && ( *crate_p >= MIN_CRATE_ADR) )
       cctlw_u._s.C = *crate_p;
     else
       status = CAM_BAD_C;
  }        
         
  /*
  ** Add the slot number
  */
  if ((  slot_p != NULL )  && SUCCESS(status) )
  { 
    if ((*slot_p >= MIN_CRATE_SLOT) && (*slot_p <= MAX_CRATE_SLOT)) 
       cctlw_u._s.N = *slot_p;
    else
       status = CAM_BAD_N;
  }
  /*
  ** Add the CAMAC function
  */
  if ((  function_p != NULL ) && SUCCESS(status) )
  {
     if ( (*function_p >= F[RD1]) && (*function_p <= F[RESV_31]))
       cctlw_u._s.F = *function_p;
     else
       status = CAM_BAD_F;
  }
  /* 
  ** Addd subaddress
  */
  if (( subaddress_p != NULL ) && SUCCESS(status) )
  {
     if ((*subaddress_p >= A[0]) && (*subaddress_p <= A[31]))
         cctlw_u._s.A = *subaddress_p;
     else 
         status = CAM_BAD_A;
  }

  /* 
  ** Add in mode (ie: scan modes and/or pack modes)
  ** Note: if no mode supplied then use default mode
  */
  if (( mode_p != NULL ) && SUCCESS(status) ) 
  {
      /* 
      ** Check for default mode 
      */
      if ( *mode_p != 0 )
      {
        new_mode = *mode_p & camac_modes;
        if ( new_mode != 0 )
          cctlw_u._i = cctlw_u._i | new_mode;
        else 
         status = CAM_BAD_MODE;
      }
  }

  /*
  ** If any invalid control word parameters return 0
  */
  if ( !SUCCESS(status) )
    cctlw_u._i = 0;

  return( cctlw_u._i );  

}

