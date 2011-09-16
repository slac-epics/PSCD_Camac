/*
=============================================================

  Abs:  Crate Verifier Diagnostic Function prototypes

  Name: CVTest_proto.h
   
  Side: Includes the following header files
           slc_macros.h - for vmsstat_t

  Auth: 19-Jul-2009, K. Luchini       (LUCHINI)
  Rev : dd-mmm-yyyy, Reviewer's Name  (USERNAME)

-------------------------------------------------------------
  Mod:
        dd-mmm-yyyy, First Lastname   (USERNAME):
          comment

=============================================================
*/

#ifndef _CVTEST_PROTO_H_
#define _CVTEST_PROTO_H_

#include "slc_macros.h"        /* for vmsstat_t */


/* Diagnostic functions that do not use the message queue */
vmsstat_t CV_WtData( short branch, short crate, short slot, unsigned long *data_p );
vmsstat_t CV_RdData( short branch, short crate, short slot, unsigned long *data_p );
vmsstat_t CV_RdId( short branch, short crate, short slot,   unsigned long *data_p );
vmsstat_t CV_RdVoltage( short branch, short crate, short slot );
vmsstat_t CV_RW( short branch, short crate, short slot );
vmsstat_t CV_RW2( short branch, short crate, short slot );

/* Diagnostic functions that use the message queue */
long CV_SendMsg(   short b, short c, short n , cv_camac_func_te func_e);
long CV_RdIdMsg(   short b, short c, short n);
long CV_RdVoltageMsg(short b, short c, short n);
long CV_RdDataMsg( short b, short c, short n);
long CV_WtDataMsg( short b, short c, short n);
long CV_RdCrateStatusMsg(short b,short c,short n);

/* Utilities for Read Write Line Test */
void          blockWordSwap (void * const buf_p,unsigned int num_words);
unsigned long CV_RWDataGet(cv_rwLine_type_te      type_e, 
                           unsigned int           nelem,
                           unsigned short * const idata_a,
                           unsigned long  * const odata_a,
                           unsigned long  * const err_a,
                           unsigned long  * const expected_data_a);


#endif /*_CVTEST_PROTO_H_ */



