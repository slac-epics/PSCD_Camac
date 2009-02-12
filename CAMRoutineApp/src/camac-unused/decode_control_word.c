/*
	**MEMBER**=SLCRMX:REENTRANT
        **CMS**=RMX_CAM
===============================================================================

  Abs:  This function decodes the CAMAC control word

  Name: DECODE_CONTROL_WORD.C86

  Args: cctlw_p                        CAMAC control word
          Use:  integer    
          Type: longword (unsigned) *
          Acc:  read-only      
	  Mech: By reference      

        crate_p                        CAMAC crate address
          Use:  integer                NOTE: optional 
          Type: longword (unsigned) *
          Acc:  write-access
	  Mech: By reference      

        slot_p                         CAMAC crate slot number 
          Use:  integer                NOTE: optional
          Type: longword (unsigned) *
          Acc:  write-access
	  Mech: By reference      

        function_p                     CAMAC function code
          Use:  integer                NOTE: optional
          Type: longword (unsigned) *
          Acc:  write-access
	  Mech: By reference      

        subaddress_p                   CAMAC subaddress 
          Use:  integer                NOTE: optional
          Type: longword (unsigned) *
          Acc:  write-access
	  Mech: By reference      

  Rem:  This routine will return the CAMAC crate address, slot number
        function code, and subaddress, given from the CAMAC control
        word provided.

  Side: A check for invalid crate,slot,function code and subaddress
        is not done.

  Ret:  vmsstat_t 
                CAM_OKOK - Successful operation (always)

  Proto: cam_proto.hm

  Auth: 12-Jun-1991, Mike Zelazny  (MIKE)
  Rev:  12-Jun-1991, Stephanie Allison (SAA)

--------------------------------------------------------------------------------

  Mod:
        24-Jan-1997, K. Luchini (LUCHINI):
           modify for rmx version from VAX version in camshr

==============================================================================*/

/*
** Include Files
*/
#include "camdef.hc"      /* CAMAC facility message codes */
#include "slc_macros.hc"  /* vmsstat_t,int4u,int2u,...    */
#include "camac_types.hc" /* ctlw_tu                      */


vmsstat_t decode_control_word( unsigned long const * cctlw_p,
    	 	               unsigned long * crate_p, 
 	 	               unsigned long * slot_p, 
	 	               unsigned long * function_p, 
	 	               unsigned long * subaddress_p )
{
  vmsstat_t     status = CAM_OKOK;       /* return status        */
  ctlw_tu       cctlw_u;                 /* CAMAC control word   */

  /*
  ** Disassemble CAMAC control word supplied
  */
  cctlw_u._i = *cctlw_p;   
  if ( crate_p )      *crate_p      = cctlw_u._s.C;
  if ( slot_p )       *slot_p       = cctlw_u._s.N;
  if ( function_p )   *function_p   = cctlw_u._s.F;
  if ( subaddress_p ) *subaddress_p = cctlw_u._s.A;

  return( status );
}
