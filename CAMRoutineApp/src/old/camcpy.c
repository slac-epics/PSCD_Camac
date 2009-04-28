/*     **MEMBER**=SLCRMX:REENTRANT
       **CMS**=RMX_CAM
 
================================================================================
 
  Abs:  Copy to/from dual-port memory.
 
  Name: CAMCPY (module)
 
  Rem:   Endian-aware copy to/from dual-port memory
 
  Side: None
 
  Proto: slcrmxi:cam_proto.h.
 
  Auth: 11-Apr-2009, Robert C. Sass (RCS).
  Revw:
 
--------------------------------------------------------------------------------
 
  Mod:  
        dd-mmm-yyyy, first last (uid)
==============================================================================*/

#include <stdio.h>                 /* NULL                                    */ 
#include <string.h>                /* memcpy                                  */

/*
** If this is a little-endian machine this degenerates to a memcpy.
** 
** If this is a big-endian machine the byte swap is handled by the PLX chip 
** on the PSCD card but we must handle the odd word.
**
** The PCI bus and hence dual-port memory is read little endian by the PSCD 
** i.e. sequential addresses proceed from the low-order byte. 
** The PPC is big-endian i.e. sequential addresses proceed 
** from the high-order byte. The PSCD always does word transfers and the PLX 
** swaps the bytes but not the words which we must do here.
**
** Furthermore, how we do the swap depends on the direction.
**
** For TODP i.e. PPC -> DP;  PPC[0] -> DP[1]
** For FROMDP i.e. DP  -> PPC; DP[0]  -> PPC[1]
*/
void camcpy (void *dest_p, void *src_p, size_t wc, unsigned char dir)
{
  unsigned short *lsrc_p  = (unsigned short *) src_p, 
                 *ldest_p = (unsigned short *) dest_p; /* Make ptrs *word */
  /*------------------ code ---------------*/
#ifdef _X86_
  memcpy (ldest_p, lsrc_p, (wc <<1));
#else
  /*
  ** If the word count is even only a simple memcpy is required.
  ** If it's odd, then do memcpy for the even number of words and put 
  ** the last word in the correct half depending on the direction.
  */
  if (!(wc & 1))
  {
    memcpy (ldest_p, lsrc_p, (wc << 1));  /* Even word count = memcpy */
  }
  else            /* Odd word count. For 1 word the memcpy is a NOP. */
  {
    memcpy (ldest_p, lsrc_p, ((wc-1) << 1)); /* Copy all but the last odd word */
    lsrc_p =  &(lsrc_p[wc-1]);    /* Point to the last words */
    ldest_p = &(ldest_p[wc-1]);
    if (dir == TODP)
    {
      ldest_p[1] = lsrc_p[0];
    }
    else
    {
      ldest_p[0] = lsrc_p[1];
    }
  }
  return;
}
#endif

