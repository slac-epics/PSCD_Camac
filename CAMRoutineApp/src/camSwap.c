/*     **MEMBER**=SLCRMX:REENTRANT
       **CMS**=RMX_CAM
 
================================================================================
 
  Abs:  Byte/Word swaps for big-endian machines.
 
  Name: camSwap (module)
        camSwapBytes                  Swap bytes.
        camSwapWords                  Swap words.
 
  Rem:
 
  Side:
 
  Proto: cam_proto.h.
 
  Auth: 01-July-2010, Robert C. Sass (RCS).
  Revw:
 
--------------------------------------------------------------------------------
 
  Mod:  
 
==============================================================================*/

/**********************************************
   Abs:   Swap bytes in a buffer
   Name:  camSwapBytes. 
   Scope: Global
   Rem:   See Abs
   Args:  buf_p           Pointer to buffer
          bsize           # bytes to swap
   Ret:   None
***********************************************/

void camSwapBytes  (void *buf_p, int bsize)
{
   char tmp;
   int  j;
   char *dat_p = buf_p;
   /*------------------------------------------*/
   for (j=0; j < bsize; j+=2)
   {
      tmp = *dat_p;
      *dat_p = *(dat_p+1);
      *(dat_p+1) = tmp;
      dat_p += 2;
   }
   return;
}

/***********************
   Abs:   Swap words (shorts) in a buffer
   Name:  camSwapWords. 
   Scope: Global
   Rem:   If #words is odd, don't swap the last 2
   Args:  buf_p           Pointer to buffer
          wsize           #words to swap
   Ret:   None
************************/

void camSwapWords (void *buf_p, int wsize)
{
   short tmp;
   int  j;
   short *dat_p = buf_p;
   /*------------------------------------------*/
   for (j=0; j < wsize-1; j+=2)
   {
      tmp = *dat_p;
      *dat_p = *(dat_p+1);
      *(dat_p+1) = tmp;
      dat_p += 2;
   }
   return;
}
