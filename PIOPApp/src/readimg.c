#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

FILE *piop_f;

#define IMG_MAX_BLOCKS 120
#define IMG_BLOCK_BC   240
#define IMG_MAX_SIZE IMG_MAX_BLOCKS * IMG_BLOCK_BC

unsigned char *piopimg_p = NULL;

int main(void)
{
  if ((piop_f = fopen ("mksu.img", "r")) == NULL)
  {
    printf ("Error opening file\n");
    goto egress;
  }
  if ((piopimg_p = calloc (1, IMG_MAX_SIZE)) == NULL)
  {
    printf ("Cannot calloc for image\n");
    goto egress;
  }

  int idx = 0; /* Index into array */
  unsigned short rlen; /* VMS 2 byte  record length */
  int bytes_read;
  int blocks = 0;
  
  while (!feof (piop_f))
  {
    /*
    ** First read the 16 bit VMS record length followed by the data
    ** in each variable length record.
    */
    bytes_read = fread (&rlen, 1, 2, piop_f);

    if (feof (piop_f))  /* Hit eof reading record length after last record */
      goto egress;

    printf ("Record length = %d total bytes read = %d\n",rlen, idx);

    /*
    ** Insure we don't exceed array size
    */
    if ( (idx + rlen) > IMG_MAX_SIZE)
    {
      printf ("Max image size exceeded %d\n", idx);
      goto egress;
    }

    if ((bytes_read = fread (&piopimg_p[idx], 1, rlen, piop_f)) != rlen)
    {
      printf ("Cannot read %d bytes\n", rlen);
      goto egress;
    }
    idx += rlen;
    blocks++;
  }
 egress:
  fclose (piop_f);
  free (piopimg_p);
  printf ("Read %d blocks total len %d\n",blocks,idx);
}
