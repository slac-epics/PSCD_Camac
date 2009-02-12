/*=============================================================================
 
  Name: pack.c
           pack_w_to_bytes
           unpk_bytes_to_w

  Abs:  Pack.unpack bytes and words. 

  Auth: 24-Apr-2006, Robert C. Sass (RCS) created
  Rev:  dd-mmm-200y,
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------  
  Mod:  (newest to oldest)  
        DD-MMM-YYYY, My Name:
           Changed such and such to so and so. etc. etc.
        DD-MMM-YYYY, Your Name:
           More changes ... The ordering of the revision history 
           should be such that the NEWEST changes are at the HEAD of
           the list.
 
=============================================================================*/


/*==========================================================================

  Name: pack_w_to_bytes

  Abs:  Extract the low-order byte of each 16-bit word of a word array,
        and pack the bytes into a byte array.  
        

  Args: Type		   Name	     Access	   Description
	---------------   --------  ---------- ----------------------------


  Rem:  

  Ret:  none
=============================================================================*/
void pack_w_to_bytes(const unsigned short sorc_p[], 
                     unsigned char dest_p[], unsigned bc)
{
   int count = bc & 0xFFFF;   /* Just take low 16 bits of count */
   int i,j;
   unsigned char *sorcc_p = (unsigned char *) sorc_p; /* Sorc as char array */
/*---------------------------- Code --------------------------------------*/
  for (i=0, j=0; i<count; i++, j+=2)
     dest_p[i] = sorcc_p[j];
  return;
}


/*==========================================================================

  Name: unpk_bytes_to_w

  Abs:  Unpack bytes from a byte array, zero-extending each byte to a
        16-bit word, and store the words in a word array:



  Args: Type               Name      Access        Description
        ---------------   --------  ---------- ----------------------------


  Rem:

  Ret:  none
=============================================================================*/
void unpk_bytes_to_w(const unsigned char sorc_p[], 
                     unsigned short dest_p[], unsigned bc)
{
   int count = bc & 0xFFFF;   /* Just take low 16 bits of count */
   int i;
/*---------------------------- Code --------------------------------------*/
   for (i=0; i<count; i++)
   {
      dest_p[i] = 0;
      dest_p[i] = sorc_p[i];
   }   
  return;
}

