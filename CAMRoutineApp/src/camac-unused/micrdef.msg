!   **MEMBER**=SLCLIBS:SLCMSGLIB
!   **ATTRIBUTES**=COMMON
.TITLE MICRDEF SLC Micro Messages ! (note: first word is module name)
.FACILITY MICR,20/PREFIX=MICR_
.IDENT   'VERSION 1.01'
!
!*************************************************************
!
!  06Jan2006  Tony Gromme  Add an RTEMS_specific bunch.
!  07Oct1996  Robert Sass  Add REGDUMP3. Too many chars for REGDUMP2.
!  08Apr1996  Robert Sass  Add UNEXPCOND. Add job and task to NPXENV386.
!                          Change NPXENV286 to REGDUMP1 and EXCPTN286
!                          to REGDUMP2.  All for revised exception handling.
!  18Jan1996  Tony Gromme  1. Add MBCDINIT.
!  26May1994  Robert Sass  1. Add NOPARAMOBJ.
!  19Apr1990  K. Krauter   1. Add PMDOK and PMDNG for ntwkshr:pmdcon.for.
!  21Apr1986  Alex Hunter  1. Add the missing MICR_CANTIPL message.
!
!*************************************************************
 
! The following codes are used to interpret iRMX error codes
!     and must have the base codes exactly as specified.
 
.SEVERITY SUCCESS
.BASE 0
 
OKOK            <'Micro operation successful'>
 
.SEVERITY ERROR
 
ETIME/warn      <'Time limit expired'>
EMEM            <'Insufficient memory available'>
EBUSY           <'Region (semaphore) is busy.'>
ELIMIT          <'Nucleus limit exceeded'>
ECONTEXT        <'Call issued out of context'>
EEXIST          <'No existing object for this token'>
ESTATE          <'Impossible transition state'>
ENOTCONF        <'System call not configured'>
EINTSAT/warn    <'Task has maximum interrupt requests'>
EINTOVF         <'Task exceeded maximum interrupt requests'>
ETRANSMIT       <'Bus timeout.'>
ESLOT           <'No GDT slots available.'>
 
.BASE 16
 
EZERDIV         <'Attempted divide by zero'>
EOVRFLW         <'Overflow interrupt occurred'>
ETYPE           <'Object not of required type'>
.BASE 20
EPARAM          <'Illegal parameter value'>
EBADCALL        <'Restricted software interrupt'>
EARRBNDS        <'Array bounds exceeded.'>
ENDPERR         <'80387 trap.'>
EINVOPCD        <'Invalid 80386 opcode.'>
EEMULAT         <'80387 not configured for this task.'>
EPASCASE        <'CASE bounds exceeded.'>
ENDPSEGL        <'Segment limit exceeded by 80387.'>
.BASE 29
EPROT           <'Memory protection violation.'>
ENOTPRES        <'Segment not present.'>
EBADADDR        <'Invalid pointer (bad selector or bad offset).'>
 
.BASE 32
 
NNOPORT         <'No port'>
NDUPNAM         <'Duplicate name'>
NUNDEF          <'Undefined'>
NSEQERR         <'Sequence number error'>
NTRUNCAT        <'Data truncated'>
NOJPO           <'No parameter object for ',A4,' job, token ',Z4,'. Job suspended.'>
 
.BASE 63
UNK_IRMXERR     <'Unrecognized iRMX error code',Z5.4>
!****************************************************************
 
.SEVERITY INFORMATIONAL
.BASE 64
 
IPLOK           <'IPLed and at your service'>
TEXT            <8A4>
MICRESET	<'Reset command sent by ',A4,' to ',A4>
REPLACE         <3A4,' Replaced in Root Directory'>
BLASTBEG        <'BLAST86 begun.'>
MBLASTBEG       <'MBLAST86 begun.'>
MD86RUN         <'MD86 RUN ',8A4>
STACKDUMP       <'@Stack dump',10Z5>
PMDOK           <'Dump successfully completed for ',A4>
STACKDUMP4      <'@Stack dump',Z9.8,':',9Z9>
ERR_METER_RESET <'Error message metering reset sent to micro ',A4>
UNWNDCALL       <'@Unwound call offsets:',8Z9.8>
 
.SEVERITY WARNING
.BASE 128
 
 
.SEVERITY ERROR
.BASE 192
 
MARKERR         <'Block pointers corrupted for database update'>
DBVERIFY        <'Database verify failed for supertype ',I2,' at offset ',Z5>
NOTIMOFDAY      <'SLCnet driver initialization couldnt get first time-of-day update.'>
 
.BASE 200
 
EXNET_INITFAIL   <'EXNET initialization failed.'>
EXNET_SOCKFAIL   <'EXNET TCP socket create failed.'>
EXNET_CONNFAIL   <'EXNET TCP socket connect (to proxy) failed.'>
EXNET_RECONNFAIL <'EXNET TCP socket reconnect (to proxy) failed.'>
EXNET_FWREGFAIL  <'EXNET TCP connection initial registration packet (to proxy) failed.'>
EXNET_SENDFAIL   <'EXNET TCP send (to proxy) failed.'>
EXNET_EOFERR     <'EXNET TCP eof indicated.'>
EXNET_RECVTMOUT  <'EXNET TCP recv timed out.'>
EXNET_RECVRESET  <'EXNET TCP (intentional) disconnect.'>
EXNET_RECVFAIL   <'EXNET TCP recv failed.'>
EXNET_FWDCRC     <'EXNET received fwd_hdr containing bad crc.'>
EXNET_FWDBCMAX   <'EXNET received fwd_hdr containing bytecount .gt. max.'>
EXNET_FWDBCSHT   <'EXNET received fwd_hdr containing apparently bad bytecount.'>
EXNET_BADACK     <'EXNET received ack bad or out of sequence.'>
 
.BASE 224
 
ROOTTOKEN/fatal/USER=8 <'Unable to get Root job token - Stat=',Z4>
NAMTRUNC/warn   <'Object name truncated to ',3A4>
CATALOG         <'Unable to catalog object ',3A4,' - Stat=',Z4>
LOOKUP          <'Unable to lookup object ',3A4,' - Stat=',Z4>
DBRDPORT/fatal/USER=8  <'Unable to allocate DB read port - Stat=',Z8>
DBSNDPORT/fatal/USER=8 <'Unable to allocate DB send port - Stat=',Z8>
DBNETRD         <'Error on DB Net_read - Stat=',Z8>
DBNETWRT        <'Error on DB Net_write - Stat=',Z8>
MSGRDPORT/fatal/USER=8 <'Unable to allocate MSG read port - Stat=',Z8>
MSGNETRD        <'Error on MSG Net_read - Stat=',Z8>
MSGNETWRT       <'Error on MSG Net_write - Stat=',Z8>
CREMBX/fatal/USER=8    <'Unable to create mailbox - ',I4,' - Stat=',Z4>
GETTOKEN        <'Unable to get token type ',I4,' - Stat=',Z4>
SENDMBX         <'Error on MBX ',I3,' send - Stat=',Z4>
MAINTOKEN       <'Unable to get Main task token - Stat=',Z8>
SUSTASK         <'Unable to suspend task - Stat='Z4>
NODBJOB/fatal/USER=8   <'Unable to create Database server job - Stat=',Z8>
CRESEG          <'Unable to create segment - Task=',I4,' - Stat=',Z4>
DELSEG          <'Unable to delete segment - Task=',I4,' - Stat=',Z4>
RESUMTSK        <'Unable to resume Micromain task - Stat=',Z4>
NOMSGJOB/fatal/USER=8  <'Unable to create Message server job - Stat=',Z8>
NOJOB           <'Unable to create ',A4,' job - Stat=',Z8>
MSGLNGTH        <'Message too long for buffer in task ',I3>
MSGWRTPORT      <'Unable to allo MSG wrt port - Task=',I4,' - Stat=',Z8>
DBCREUNIT/Fatal/USER=8 <'Unable to create DB semaphore for Net_write - Stat=',Z8>
DBRCVUNIT       <'Unable to acquire DB semaphore for Net_write - Stat=',Z8>
DBSNDUNIT       <'Unable to release DB semaphore for Net_write - Stat=',Z8>
DELTASK         <'Unable to delete task ',3A4,' - Stat=',Z4>
 
 
.SEVERITY FATAL
.BASE 256
 
EXCEPTN/USER=8         <'Micro exception detected'>
ERR_MAX/USER=8         <'ERRCHK: Too many errors: aborting'>
 
.SEVERITY ERROR
.BASE 258
 
SLEEPINPUT      <'Sleep input time out of bounds -- data=',Z8.8>
READ_MBX        <'Error on MBX for ',A4,' job.  Read - Stat=',Z4>
DEL_SEG         <'Unable to delete segment for ',A4,' job.  Stat=',Z4>
JOB_STOP/FATAL/USER=8  <A4,' job (',I3,') has suspended itself'>
BUFLENOVER	<'Large data transfer buffer has overflowed'>
BUFLENREQ       <'Invalid number of bytes=',I8,' requested for large data buffer'>
BUFFLAG		<'Large data buffer routines not initialized'>
BUFGETLEN       <'Requested BUF_GET data-length=',I8,' too long'>
CANTIPL/FATAL/USER=8	<'Unable to IPL the micro database.'> 
BAD_TASKID/FATAL/USER=8 <'Corrupt TASK_ID in MSGMAIL, value=',I4>
REGDUMP2/INFO   <'CPU regs part 2: eax,ecx,edx,ebx'4Z9.8>
BAD_SEGTOK      <'Bad message token from mailbox.'>
SHORTSEG        <'Short message (',I2,' bytes) from mailbox: ',5Z9.8>
PMDNG           <'Dump aborted for ',A4,' with errors'>
EXCEPTN386/FATAL<'RMXIII exception code ',Z5.4,' at cs:ip ',Z5.4,':',Z8.8,' task',Z5.4,' in ',A4,' job'>
ASYNCINIT       <'Unable to initialize async database updater.'>
MCOMINIT        <'Unable to initialize MCOM utility.'>
REGDUMP1/INFO   <'CPU regs part 1: cr2,gs,fs,es,ds,ss,sp'Z9.8,5Z5.4,Z9.8>
NPXENV386/INFO  <'80387 sw, cw, tagw, cs:eip',4Z5.4,':'Z8.8,' task',Z5.4,' in ',A4,' job'>
WAITNGINI       <A4,' awaiting bootup initialization, mask =',Z9.8>
EXCEPTNIMG/INFO <'RMXIII exception with image: ',8A4>
UNEXPCOND/FATAL/USER=8 <'Unexpected exception code ',Z4.4,' for task',Z5.4,' in ',A4,' job'>
REGDUMP3/INFO   <'CPU regs part 3: ebp,esi,edi,eflags'4Z9.8>
WAITNGTIM       <A4,' waiting to see valid time of day.'>
MSGBACKUP/WARN  <I6,' msg svc messages waiting to be processed by ',A4,:,' subjobx',I3>
BADRGNU/FATA    <'Wrong region usage in osdep.C86',I8>
 
.BASE 290
CANTREIPL       <'Supertype',Z5.4,' not receivable after micro IPL is complete.'>
BADSUPTYP       <'Invalid supertype',Z5.4,' received from DBEX.'>
DBWRNGMICR      <'Database update for ',A4,' sent to wrong micro.'>
DBNOMEM/FATAL   <'DBMAIN cant allocate memory for supertype',I2,', size =',I6,' words.'>
DBNEVRDWNLD     <'Database supertype',I2,' was never downloaded!'>
MSGMAINOMEMR    <'MSGMAIN cant allocate memory for receiving messages.  Retrying.'>
MSGMAINOMEMF/FA <'MSGMAIN cant allocate memory for receiving messages.  Giving up.'>
IPLDBDWNSEQ     <'Database download supertype sequence wrong:  expecting',I2,', got',I2>
DBBADMESSG      <'Invalid message from DBEX.  Bytecount =',I5,'.  Message:',6Z9.8>
 
.BASE 300
 
FLTX_INVAL      <'Invalid float value;  hex =',Z9.8,'.  ',3A4,2I8>
 
.BASE 320
 
MALLOC          <'Unable to allocate memory;  requested size =',I6,'.',:,' ',2A4>
MFREE           <'Unable to free memory (allocated by malloc?);  selector =',Z5.4>
REALLOC         <'Unable to reallocate memory (allocated by malloc?).'>
BADLENARG       <'Bad length arg.'>
 
.BASE 340
 
MBCDINIT/FATAL  <'Unable to load MBCD''s crate port map.',:,'  Read back',2Z9.8,', expected',2Z9.8>
FECCSERN        <'Reply message from FECC has func code',Z9.8,' expected',Z9.8>
FECCMSGLEN      <'Reply message from FECC has wrong length.'>
FECCNOMEM       <'Insufficient memory available in FECC.'>
FECCBADCTXT     <'More than one FECC initialization setup msg sent;  one is max.'>
FECCINVREQ      <'FECC received invalid request message.'>
 
! Following bunch (each beginning with "RT") are RTEMS-specific.
 
.BASE 360
UNK_RTEMSERR    <'Unrecognized RTEMS error code',Z9.8>
RTTASK_EXITTED             <'Returned from a thread.'>
RTMP_NOT_CONFIGURED        <'Multiprocessing not configured.'>
RTINVALID_NAME             <'Invalid object name.'>
RTINVALID_ID               <'Invalid object id.'>
RTTOO_MANY                 <'Too many.'>
RTTIMEOUT                  <'Timed out waiting.'>
RTOBJECT_WAS_DELETED       <'Object deleted while waiting.'>
RTINVALID_SIZE             <'Specified size is invalid.'>
RTINVALID_ADDRESS          <'Address specified is invalid.'>
RTINVALID_NUMBER           <'Number is invalid.'>
RTNOT_DEFINED              <'Item has not been initialized.'>
RTRESOURCE_IN_USE          <'Resources still outstanding.'>
RTUNSATISFIED              <'Request not satisfied.'>
RTINCORRECT_STATE          <'Thread is in wrong state.'>
RTALREADY_SUSPENDED        <'Thread already in state.'>
RTILLEGAL_ON_SELF          <'Illegal on calling thread.'>
RTILLEGAL_ON_REMOTE_OBJECT <'Illegal for remote object.'>
RTCALLED_FROM_ISR          <'Called from wrong environment.'>
RTINVALID_PRIORITY         <'Invalid thread priority.'>
RTINVALID_CLOCK            <'Invalid date/time.'>
RTINVALID_NODE             <'Invalid node id.'>
RTNOT_CONFIGURED           <'Directive not configured.'>
RTNOT_OWNER_OF_RESOURCE    <'Not owner of resource.'>
RTNOT_IMPLEMENTED          <'Directive not implemented.'>
RTINTERNAL_ERROR           <'RTEMS inconsistency detected.'>
RTNO_MEMORY                <'Could not get enough memory.'>
RTIO_ERROR                 <'Driver IO error.'>
RTPROXY_BLOCKING           <'Internal error only.'>
 
! Following bunch (each beginning with "EP") arise from Epics osii.
 
.BASE 420
EPTHREADCREA    <'Cant create Epics thread.'>
EPMUTEXCREA     <'Cant create Epics mutex.'>
EPEVENTCREA     <'Cant create Epics event.'>
EPMSGQCREA      <'Cant create Epics message queue.'>
.BASE 430
EPICSSYSSRVE    <'Error indication returned by epics',4A4>
 
! Socket-related errors.
 
.BASE 480
SOCKCREA        <'Cant get ',A3,' socket for ',A3>
SOCKCNCT        <'Connect failed.'>
SOCKISND        <'Initial send to proxy failed.'>
SOCKMSND        <'Message send to proxy failed (connection lost?).'>
SOCKRECV        <'Message recv from proxy failed (connection lost?).'>
PXYKEEPALV      <'Keepalive send to proxy failed.'>
PXYFWDHDR       <'Bad message forwarding header from proxy: ',3Z9.8>
NIXRECNCT       <'Reconnect not attempted because recv pending.'>
PXYRECNCT/INFO  <'Attempting to reconnect to proxy for ',A3>
 
! PCIL-related errors.
 
.BASE 500
OPENFAIL        <'Cant open ',4A4,'  errno=',I5>
IOCTLFAIL       <' ',4A4,' failed on ',4A4,'  errno=',I5>
READFAIL        <'Read failed on ',4A4,'  errno=',I5>
WRITEFAIL       <'Write failed on ',4A4,'  errno=',I5>
ADDRNMOD8       <'Given memory address (after adjustment)',Z9.8,' not on octabyte boundary.'>
ALLONMOD8       <'Allocated memory address',Z9.8,' not on octabyte boundary.',:,' ',2A4>
 
.END
                                                                                                                                                                                                                                                                                                                                                  