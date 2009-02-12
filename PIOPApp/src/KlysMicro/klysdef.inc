C Name: KLYSDEF.INC
C
C   (Generated  8-MAY-2008 15:27 by NEWMSG from corresponding .MSG file.)
C
C NOTE:  While this file may be stored in a CMS REFERENCE directory, it is
C        NOT a member a CMS library.  Any changes should be made not directly
C        to this file, but to its parent .MSG file, which is in a CMS library.
C
      INTEGER*4 KLYS_BAD_SIGMA,KLYS_BAD_SLOPE,KLYS_BAD_FALL
     >,KLYS_FIT_ERROR,KLYS_RISEFALL_AVG,KLYS_RISEFALL_PEAK
     >,KLYS_TOO_NARROW,KLYS_NOPEAK,KLYS_NOWAVEFORM,KLYS_DEV_OFF
     >,KLYS_OFFLINE,KLYS_CORRUPT,KLYS_CHK_TIMEOUT,KLYS_NETERR
     >,KLYS_TIMEOUT,KLYS_FATAL,KLYS_DB,KLYS_CAMAC,KLYS_ERR,KLYS_BADSTAT
     >,KLYS_NOUNITS,KLYS_WARN,KLYS_MSG,KLYS_NO_MSG,KLYS_SUCCESS
     >,KLYS_FTP_NULLSET,KLYS_FTP_DEACT,KLYS_FTP_ABORTED
     >,KLYS_FTP_NO_DRIVE,KLYS_FTP_LOW_RATE,KLYS_FTP_BAD_CABLE
     >,KLYS_FTP_TIMEOUT,KLYS_FTP_OKOK,KLYS_REQSATURFTP,KLYS_NOSATURFTP
     >,KLYS_NOHEXDUMP,KLYS_NONMIS,KLYS_TIMINGOFF,KLYS_BADPRIMI
     >,KLYS_BADUNITI,KLYS_PPLISTNON,KLYS_PPLISTBAD,KLYS_IPLNOEND
     >,KLYS_INIT,KLYS_NORESPONSE,KLYS_AUTOIPL,KLYS_NOCAMAC_SBLK
     >,KLYS_PIOP_DEAD,KLYS_BADSLEDVAL,KLYS_BADSLEDTIME
     >,KLYS_DIFFERENT_PMU,KLYS_HOMEERROR,KLYS_NOSATURATE
     >,KLYS_DATAREQUIRED,KLYS_SINGLEUNIT,KLYS_INCONSISTANT
     >,KLYS_BAD_POINTER,KLYS_LISTSHORT,KLYS_SBIDELAY,KLYS_SBIPSK
     >,KLYS_NOCONVERGE,KLYS_CNTRL,KLYS_SEQ_GIVEUP,KLYS_HOMEBAD
     >,KLYS_KNOBOFF,KLYS_NOAMPDRVFTP,KLYS_BADHSTA,KLYS_IMG2BIG
     >,KLYS_IMGSEQ,KLYS_IPLNOIMG,KLYS_IPLBAD,KLYS_INCONHW,KLYS_IPLFBAD
      INTEGER*4 KLYS_IPLFNOSTART,KLYS_IPLFNIPL,KLYS_IPLFNF,KLYS_NOIPL
     >,KLYS_NORESET,KLYS_READBACK_ERROR,KLYS_BADBAD,KLYS_BADUNIT
     >,KLYS_NOPAD,KLYS_CAMAC_ERROR,KLYS_PIOP_MAX,KLYS_KNOBCNFLCT
     >,KLYS_NOCLUSTER,KLYS_NOTYET,KLYS_NOFTP,KLYS_UNDEFINED_FUNC
     >,KLYS_AUTOTRIM_BIG,KLYS_CAMAC_CBLK,KLYS_CAMAC_VERBOSE
     >,KLYS_CAMAC_FASTEST,KLYS_CAMAC_FAST,KLYS_TRACEBACK,KLYS_PUSH
     >,KLYS_KNOBDASSN,KLYS_KNOBASSN,KLYS_ACCEPT,KLYS_ENTER_ASCII
     >,KLYS_CLEARERR,KLYS_ENTER,KLYS_SLED_REQUEST,KLYS_SEQ_REP2
     >,KLYS_SEQ_REP_OP,KLYS_HOMEOK,KLYS_IPLGOOD,KLYS_SEQ_OK,KLYS_IPL
     >,KLYS_OKOK,KLYS_FACILITY

      PARAMETER( KLYS_BAD_SIGMA                      = #0843A5C2H)
      PARAMETER( KLYS_BAD_SLOPE                      = #0843A5BAH)
      PARAMETER( KLYS_BAD_FALL                       = #0843A5B2H)
      PARAMETER( KLYS_FIT_ERROR                      = #0843A5AAH)
      PARAMETER( KLYS_RISEFALL_AVG                   = #0843A5A2H)
      PARAMETER( KLYS_RISEFALL_PEAK                  = #0843A59AH)
      PARAMETER( KLYS_TOO_NARROW                     = #0843A592H)
      PARAMETER( KLYS_NOPEAK                         = #0843A58AH)
      PARAMETER( KLYS_NOWAVEFORM                     = #0843A582H)
      PARAMETER( KLYS_DEV_OFF                        = #0843A0DCH)
      PARAMETER( KLYS_OFFLINE                        = #0843A0D4H)
      PARAMETER( KLYS_CORRUPT                        = #08439FE2H)
      PARAMETER( KLYS_CHK_TIMEOUT                    = #08439FDAH)
      PARAMETER( KLYS_NETERR                         = #08439FD2H)
      PARAMETER( KLYS_TIMEOUT                        = #08439FCAH)
      PARAMETER( KLYS_FATAL                          = #08439FC4H)
      PARAMETER( KLYS_DB                             = #08439FB2H)
      PARAMETER( KLYS_CAMAC                          = #08439FA2H)
      PARAMETER( KLYS_ERR                            = #08439F92H)
      PARAMETER( KLYS_BADSTAT                        = #08439F8AH)
      PARAMETER( KLYS_NOUNITS                        = #08439F82H)
      PARAMETER( KLYS_WARN                           = #08439F70H)
      PARAMETER( KLYS_MSG                            = #08439F63H)
      PARAMETER( KLYS_NO_MSG                         = #08439F53H)
      PARAMETER( KLYS_SUCCESS                        = #08439F43H)
      PARAMETER( KLYS_FTP_NULLSET                    = #0843993AH)
      PARAMETER( KLYS_FTP_DEACT                      = #08439930H)
      PARAMETER( KLYS_FTP_ABORTED                    = #0843992AH)
      PARAMETER( KLYS_FTP_NO_DRIVE                   = #08439922H)
      PARAMETER( KLYS_FTP_LOW_RATE                   = #0843991AH)
      PARAMETER( KLYS_FTP_BAD_CABLE                  = #08439912H)
      PARAMETER( KLYS_FTP_TIMEOUT                    = #0843990AH)
      PARAMETER( KLYS_FTP_OKOK                       = #08439901H)
      PARAMETER( KLYS_REQSATURFTP                    = #08438D63H)
      PARAMETER( KLYS_NOSATURFTP                     = #08438D5AH)
      PARAMETER( KLYS_NOHEXDUMP                      = #08438D52H)
      PARAMETER( KLYS_NONMIS                         = #08438D4AH)
      PARAMETER( KLYS_TIMINGOFF                      = #08438D42H)
      PARAMETER( KLYS_BADPRIMI                       = #08438D3AH)
      PARAMETER( KLYS_BADUNITI                       = #08438D32H)
      PARAMETER( KLYS_PPLISTNON                      = #08438D2AH)
      PARAMETER( KLYS_PPLISTBAD                      = #08438D22H)
      PARAMETER( KLYS_IPLNOEND                       = #08438C8CH)
      PARAMETER( KLYS_INIT                           = #08438C84H)
      PARAMETER( KLYS_NORESPONSE                     = #08438A9AH)
      PARAMETER( KLYS_AUTOIPL                        = #08438A92H)
      PARAMETER( KLYS_NOCAMAC_SBLK                   = #08438A8AH)
      PARAMETER( KLYS_PIOP_DEAD                      = #08438A82H)
      PARAMETER( KLYS_BADSLEDVAL                     = #08438A72H)
      PARAMETER( KLYS_BADSLEDTIME                    = #08438A6AH)
      PARAMETER( KLYS_DIFFERENT_PMU                  = #08438A62H)
      PARAMETER( KLYS_HOMEERROR                      = #08438A5AH)
      PARAMETER( KLYS_NOSATURATE                     = #08438A52H)
      PARAMETER( KLYS_DATAREQUIRED                   = #08438A4AH)
      PARAMETER( KLYS_SINGLEUNIT                     = #08438A42H)
      PARAMETER( KLYS_INCONSISTANT                   = #08438A3AH)
      PARAMETER( KLYS_BAD_POINTER                    = #08438A32H)
      PARAMETER( KLYS_LISTSHORT                      = #08438A2AH)
      PARAMETER( KLYS_SBIDELAY                       = #08438A22H)
      PARAMETER( KLYS_SBIPSK                         = #08438A1AH)
      PARAMETER( KLYS_NOCONVERGE                     = #08438A12H)
      PARAMETER( KLYS_CNTRL                          = #08438A0AH)
      PARAMETER( KLYS_SEQ_GIVEUP                     = #08438A02H)
      PARAMETER( KLYS_HOMEBAD                        = #084389FAH)
      PARAMETER( KLYS_KNOBOFF                        = #084389F2H)
      PARAMETER( KLYS_NOAMPDRVFTP                    = #084389EAH)
      PARAMETER( KLYS_BADHSTA                        = #084389E2H)
      PARAMETER( KLYS_IMG2BIG                        = #084389DAH)
      PARAMETER( KLYS_IMGSEQ                         = #084389D2H)
      PARAMETER( KLYS_IPLNOIMG                       = #084389CAH)
      PARAMETER( KLYS_IPLBAD                         = #084389C2H)
      PARAMETER( KLYS_INCONHW                        = #084389BAH)
      PARAMETER( KLYS_IPLFBAD                        = #084389B2H)
      PARAMETER( KLYS_IPLFNOSTART                    = #084389AAH)
      PARAMETER( KLYS_IPLFNIPL                       = #084389A2H)
      PARAMETER( KLYS_IPLFNF                         = #0843899AH)
      PARAMETER( KLYS_NOIPL                          = #08438992H)
      PARAMETER( KLYS_NORESET                        = #0843898AH)
      PARAMETER( KLYS_READBACK_ERROR                 = #08438982H)
      PARAMETER( KLYS_BADBAD                         = #0843897AH)
      PARAMETER( KLYS_BADUNIT                        = #08438972H)
      PARAMETER( KLYS_NOPAD                          = #0843896AH)
      PARAMETER( KLYS_CAMAC_ERROR                    = #08438962H)
      PARAMETER( KLYS_PIOP_MAX                       = #08438668H)
      PARAMETER( KLYS_KNOBCNFLCT                     = #08438660H)
      PARAMETER( KLYS_NOCLUSTER                      = #08438658H)
      PARAMETER( KLYS_NOTYET                         = #08438650H)
      PARAMETER( KLYS_NOFTP                          = #08438648H)
      PARAMETER( KLYS_UNDEFINED_FUNC                 = #08438640H)
      PARAMETER( KLYS_AUTOTRIM_BIG                   = #084383ABH)
      PARAMETER( KLYS_CAMAC_CBLK                     = #084383A3H)
      PARAMETER( KLYS_CAMAC_VERBOSE                  = #0843839BH)
      PARAMETER( KLYS_CAMAC_FASTEST                  = #08438393H)
      PARAMETER( KLYS_CAMAC_FAST                     = #0843838BH)
      PARAMETER( KLYS_TRACEBACK                      = #08438383H)
      PARAMETER( KLYS_PUSH                           = #0843837BH)
      PARAMETER( KLYS_KNOBDASSN                      = #08438373H)
      PARAMETER( KLYS_KNOBASSN                       = #0843836BH)
      PARAMETER( KLYS_ACCEPT                         = #08438363H)
      PARAMETER( KLYS_ENTER_ASCII                    = #0843835BH)
      PARAMETER( KLYS_CLEARERR                       = #08438353H)
      PARAMETER( KLYS_ENTER                          = #0843834BH)
      PARAMETER( KLYS_SLED_REQUEST                   = #08438343H)
      PARAMETER( KLYS_SEQ_REP2                       = #0843833BH)
      PARAMETER( KLYS_SEQ_REP_OP                     = #08438333H)
      PARAMETER( KLYS_HOMEOK                         = #0843832BH)
      PARAMETER( KLYS_IPLGOOD                        = #08438323H)
      PARAMETER( KLYS_SEQ_OK                         = #08438019H)
      PARAMETER( KLYS_IPL                            = #08438011H)
      PARAMETER( KLYS_OKOK                           = #08438009H)
      PARAMETER( KLYS_FACILITY                       = #00000843H)
                                             