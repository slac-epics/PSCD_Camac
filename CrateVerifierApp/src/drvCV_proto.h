/*
=============================================================

  Abs:  CAMAC Crate Verifier EPICS Driver Support prototypes

  Name: drvCV_proto.h
   
  Side:  Must include the following header files
              devCV.h

  Auth: 19-Jul-2009, K. Luchini       (LUCHINI)
  Rev : dd-mmm-yyyy, Reviewer's Name  (USERNAME)

-------------------------------------------------------------
  Mod:
        dd-mmm-yyyy, First Lastname   (USERNAME):
          comment

=============================================================
*/
#ifndef _DRV_CV_PROTO_H_
#define _DRV_CV_PROTO_H_

long         IsCrateOnline( short c);
CV_MODULE  * CV_FindModuleByBCN(short b, short c, short n );
void         CV_ClrMsgStatus( cv_message_status_ts * const msgstat_ps );
long         CV_DeviceInit( cv_camac_func_te   func_e,
                            char const * const source_c,
                            dbCommon   * const rec_ps,
                            CV_MODULE  * const module_ps, 
                            CV_REQUEST * const dpvt_ps );

#endif /*_DRV_CV_PROTO_H_ */



