!       **MEMBER**=SLCLIBS:SLCMSGLIB
!       **ATTRIBUTES**=COMMON
.TITLE CAMDEF messages ! (note: first word is module name)
.FACILITY CAM,2
 
.SEVERITY SUCCESS
OKOK            <'CAMAC successful'>

.SEVERITY ERROR
CCB_NFG         <2A4,' found CAMAC Control Block NFG:',4Z9.8>
IOP_MAX         <'CAMADD: Attempt to add more packets than allocated'>
BC_MAX          <'CAMADD: MBCD CAMAC byte count .gt. 256'>
BC_ODD          <'CAMADD: CAMAC odd byte count'>
NOPS_MAX        <'CAMALO: Illegal request for',I4,' .gt. 84 packets.'>
SOFT_TO         <'CAMAC software timeout. No MBCD response.: C',I2,',N',I2,',A',I2,',F',I2,', Stat=',Z8.8>
MBCD_NFG        <'CAMAC MBCD failure: C',I2,', N',I2,', A',I2,', F',I2,', Stat=',Z8.8>
CRATE_TO        <'CAMAC crate timeout: C',I2,', N',I2,', A',I2,', F',I2,', Stat=',Z8.8>
NO_WCTERM       <'CAMAC - failed to exhaust word count: C',I2,', N',I2,', A',I2,', F',I2,', Stat=',Z8.8>
NO_EOSTRM       <'CAMAC - failed to terminate on End-Of-Scan: C',I2,', N',I2,', A',I2,', F',I2,', Stat=',Z8.8>
NO_EMSTRM       <'CAMAC failed to terminate on End-Mode_Signal:  C',I2,', N',I2,', A',I2,', F',I2,', Stat=',Z8.8>
NO_X            <'CAMAC - no X respns: C',I2,', N',I2,', A',I2,', F',I2,', Stat=',Z8.8>
NO_Q            <'CAMAC - no Q respns: C',I2,', N',I2,', A',I2,', F',I2,', Stat=',Z8.8>
BC_NFG          <2A4,': Bad byte count from serial branch'>
HDRLEN_BIG      <'CAMVDRVR: NETHEADERLEN.GT.PKGLEN (Software problem)'>
NOOPTION        <'CAMGO_V -- ERROR: Special options not allowed with repeat'>
NO_REPEAT       <'VIRTUAL CAMAC -- ERROR: Repeat function not available due to lack of resources'>
NULL_PTR        <'Unable to',2A4,'packet with Pointer = 0'>
NO_PKTS         <'No packets specified in Control Block'>
BAD_BUFF        <'Data buffer at',Z9,' is not word aligned'>
TDV_BUSY        <'@MBCD board error -- TDV always busy'>
BC_ZNZ          <'CAMADD: Bytecount .gt. zero with no data xfer.'>
OFFSEGEND       <'CAMADD: MBCD would run off end of segment containing status/data.'>
ILMODCTLW       <'CAMMOD: Illegal change to CCTLW.'>
CKSMER1         <'Request message from CAMGO_V has bad checksum.'>
INV_HANDLER_CMND <'CAMAC - Invalid handler command'>
INV_MODID       <'CAMAC - Invalid module ID: ',A4,1X,'C',I2,1X,'N',I2>
NOTRIG_RECVD    <'CAMAC - No Trigger Received'>
DIS_LAM_FAILED  <'CAMAC - Disable LAM failure'>
ENB_LAM_FAILED  <'CAMAC - Enable LAM failure'>
BADPTR          <'CAMAC - bad pointer to dynamic CAMAC data list'>
NOHEAP          <'CAMAC - Unable to (re)allocate dynamic memory'>
BAD_C           <'CAMAC - Invalid crate address, unable to build control word'>
BAD_N           <'CAMAC - Invalid crate slot #, unable to build control word'>
BAD_F           <'CAMAC - Invalid function code, unable to build control word'>
BAD_A           <'CAMAC - Invalid subaddress, unable to build control word'>
BAD_MODE        <'CAMAC - Invalid mode, unable to build control word'>
BAD_CTLW        <'CAMAC - Invalid control word'>
NGNG            <'CAMAC - General Bad Status'>

.BASE 100
.SEVERITY SUCCESS
MODID_OKOK      <'CAMAC - handler success ',A4,1X,'C',I2,1X,'N',I2,1X,'ID:',I2>
HWWRITE_OKOK    <'CAMAC - handler success ',A4,1X,'C',I2,1X,'N',I2,1X,'F',I2,1X,'A',I2,1X,I2>
HWINIT_OKOK     <'CAMAC - handler success ',A4,1X,'C',I2,1X,'N'I2>
 
.BASE 200 
.SEVERITY INFORMATIONAL
PRBS_INFO       <'@CAMAC PRBS - Cycles run',I10,'  Errors detected',I10,'  Error rate ',A1,E10.2>
PRBS_START      <'@CAMAC PRBS - Start test      Version no.',A4> 
READ_MISMATCH   <'CAMAC - READ does not match setpoint: ',A4,1X,'C',I2,1X,'N',I2>

!
! [300..309] reserved for CAMAC configuration INFORMATIONAL
!
.BASE 300
.SEVERITY INFORMATIONAL
CC_ATTEMPT       <'CAMAC - configuration request for ',A4,1X,A4,1X,A4,1X,'Crate:',1X,A4>
CC_ATTEMPT_DONE  <'CAMAC - configuration request completed for ',A4,1X,A4,1X,A4,1X,'Crate:'1X,A4>
CC_COMPLETE      <'CAMAC - configuration successful for ',A4,1X,A4,1X,A4,1X,'Crate:'1X,A4>
CC_MOD_OFFLINE   <'CAMAC -',A4,1X,A4,1X,A4,' Crate: ',A4,' NOT configured because module is offline'>
CC_CRATE_OFFLINE <'CAMAC -',A4,1X,A4,1X,A4,' Crate: ',A4,' NOT configured because crate is offline'>
CC_MICRO_OFFLINE <'CAMAC -',A4,1X,A4,1X,A4,' Crate: ',A4,' NOT configured because micro is offline'>

!
! [310..349] reserved for CAMAC configuration ERRORS
!
.BASE 310
.SEVERITY ERROR
CC_BAD_PRIMARY   <'CAMAC - unable to find configuration instructions for ',A4>
CC_NOT_ALLOWED   <'CAMAC - configuration request NOT allowed for ',A4,1X,A4,1X,A4,1X,'Crate:'1X,A4>
CC_NOT_SLOW_MGR  <'CAMAC - Only MPS Controller may change MPS channel configuration.'>      
CC_MPS_MISMATCH  <'CAMAC - MPS channel protection is not ', 3A4,', configuration not changed.'>
CC_LIST_OVERFLOW <'CAMAC - Internal software error; not enough room in output argument!!!'>
CC_NONE_TO_CONFIG <'CAMAC - No channels found for configuration request (type ', 3A4, ').'>
CC_INVALID_REQ_TYPE <'CAMAC - Invalid MPS request type specified.' >

.BASE 350
NO_ALL           <'CAMAC - ALL* not allowed for ',2A4 >
BAD_MOD_PRIM     < 'Invalid digital module primary: ', A4 >


.BASE 360
.SEVERITY INFORMATIONAL
ON_CHAN          <' for primary, micro, unit, channel: ',A4,',',A4,',',A4,',',A4  >
ON_UNIT          <' for primary, micro, unit: ',A4,',',A4,',',A4  >
ON_CHANCOMP      <' for primary, micro, unit, channel, component: ' ,A4,',',A4,',',A4,',',A4,',',2A4 >
 
.BASE 400
.SEVERITY ERROR
REPACK     <'CAMAC - REPACK Option only available with single packets'>
BC              <'CAMAC - Byte count of',I5,' exceeds limit of',I5>
MBCD_OVERRUN    <'CAMAC - remaining word count excessive'>
PROTECTION      <'CAMAC - ERROR: unable to write to argument',I2,' in ',2A4,' IS IT A CONSTANT INSTEAD OF A VARIABLE???'>
POOL_NOINIT     <'CAMAC - POOL for MBCD not initialized. CAMPOOLI must be called first.'>
POOL_BADSEG     <'CAMAC - Segment selector ',Z4,' has physical address ',Z8,' outside POOL or beyond 1 Mbyte.'>
POOL_BAD_DEF    <'CAMAC - Bad POOL definition. Starting address is ',Z8,' ending address is ',Z8>
POOL_NOMEM      <'CAMAC - Out of CAMAC memory pool.'>
POOL_FREE       <'CAMAC - Found ',I6,' free memory chunks at chunk ',I6,' that should be allocated. Software error.'>
POOL_DUPINIT    <'CAMAC - Duplicate initialization attempt of CAMAC memory pool.'>
POOL_ZREQ       <'CAMAC - POOL request for 0 bytes or for more than pool total.'>
POOL_DESCR      <'CAMAC - POOL requested and available but iRMX has run out of descriptor space.'>
 
.BASE 420
.SEVERITY ERROR
PAKALOI    <'CAMAC ctlw with P8 or REPACK illegal with CAMALOI.'>
DUPSTAD    <'Illegal reuse of stat/data block for same camac package.',Z12.8>
CKSMER3    <'CAMGO_V: Checksum failed 3 retries.'>
MBCDMAPL/FATAL  <'Unable to load MBCD''s crate port map.  Read back',2Z9.8,', expected',2Z9.8>
ZERCRATAD  <'CAMGO: Zero crate address is invalid.'>
INVDIAGRQ  <'Invalid MBCD diagnostic request.'>
 
.BASE 440
.SEVERITY ERROR
MOD_MANY_TYPES/W <'CAMAC - Only one type of data may be specified when setting channel component state; using 1st type specified.'>
MOD_BAD_TYPE     <'CAMAC - Unknown data type specified for setting channel component state.'>
MOD_UNSUP_TYPE   <'CAMAC - Unsupported data type specified for setting channel component state.'>
MOD_CHAN_NONE    <'CAMAC - No channels found.' >
MOD_CHAN_BAD     <'CAMAC - Bad channel: ', A4 >
MOD_COMP_NONE    <'CAMAC - No channel components found.' >
MOD_COMP_BAD     <'CAMAC - Component not configurable: ', 2A4 >
MOD_NOCTLW       <'CAMAC - Control word (CTLW) not found.'>
MOD_CHAN_ASSOC   <'CAMAC - Unable to determine associated database unit for module.'>
MOD_STATE_NONE   <'CAMAC - No states found for channel component.'>
MOD_STATE_INCONSIS <'CAMAC - Some states of channel component have a data type incompatible with other states of component.'>
MOD_STATE_WRONGNUM <'CAMAC - Wrong number of states found for a channel component.'>
MOD_STATE_BADLIM   <'CAMAC - Lower limit for valid values of channel component is greater than upper limit.'>
 
.BASE 480
.SEVERITY ERROR
OPENFAIL        <'Cant open ',4A4,'  errno=',I5>
IOCTLFAIL       <' ',4A4,' failed on ',4A4,'  errno=',I5>
READFAIL        <'Read failed on ',4A4,'  errno=',I5>
WRITEFAIL       <'Write failed on ',4A4,'  errno=',I5>
INVCCTLW        <'Invalid camac control command word',Z9.8>
BC_ZER          <'Zero camac databytecount (non-F8) not allowed.'>
LARGCYCC/INFO   <'Very large camac cyclecount',I5,' questionable.'>
 
.BASE 500
.SEVERITY ERROR
TRACE/ERROR     <'@ CTLW: ',Z8.8,' TDV:',Z4.2,' Stati: ',8(Z8.8,:,2X)>
 
.END
                                                                                                                                                                                     