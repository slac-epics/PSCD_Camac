/*   **CMS**=C_INC   */

/*==============================================================================

  Abs:  Common macros and typedef's for VAX & micro "C" code.

  Name: slc_macros.h

  Prev: stdio.h for true and false.

  Auth: 01-Aug-1990   R. Sass (RCS)

--------------------------------------------------------------------------------

  Mod: 15-Jul-2008    Robert C. Sass (RCS) Minimize for PSCD Camac changes
       21-Aug-1990    K. Krauter (KEK) add BOOLEAN
       01-Aug-1991    K. Krauter (KEK) add int1u
       13-Aug-1991    K. Krauter (KEK) support BOOL for 68K micros too
       20-jul-1992    D. Van Olst (VANOLST) move timestamp_ta to this file.
       21-jul-1992    G. R. White (GREG) add BOOL #define for gendefc.h
       29-Jul-1992    R. MacKenzie (RONM) Move logic here to slc_macr.hc and
                       just include slc_macr.hc here for DOS portability.

==============================================================================*/

#ifndef SLC_MACROS_HC

typedef short             int2;           /* An I*2 */
typedef unsigned short    int2u;          /* An unsigned I*2 */
typedef long int          int4;           /* An I*4 */
typedef unsigned long int int4u;          /* An unsigned I*4 */
typedef unsigned char     int1u;          /* For unsigned byte arithmetic */
typedef unsigned char     charu;          /* For unsigned byte arithmetic */

/* Give VMS style status its own type */

typedef int4u           vmsstat_t;      /* VMS type status stays 32 bits */

/* **  Check status return from standard error codes ** */

#define     SUCCESS( status)    ((status)&1)

#define SLC_MACROS_HC
#endif
