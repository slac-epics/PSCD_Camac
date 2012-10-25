/*
=============================================================

  Abs:  EPICS Device Support for PDU 360 Task 

  Name: devPDUII_360Task.c

         Long Integer Output Device Support:
         -------------------------------------------------------

	 devLongoutPDU360
         -------------

         *   init_pdu360_longout_record   - Record initialization
         *   write_pdu360_longout         - Write long integer output
	 *   report_pdu360                - Device support report

=============================================================
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include <alarm.h>
#include <dbAccess.h>
#include <errlog.h>
#include <alarm.h>
#include <dbDefs.h>
#include <dbAccess.h>
#include <recGbl.h>
#include <recSup.h>
#include <devSup.h>
#include <longoutRecord.h>
#include <epicsExport.h>

extern int PDU_F19_DELAY_US;

/* Device support prototypes */
static long init_pdu360_longout_record(struct longoutRecord *plongout);
static long write_pdu360_longout(struct longoutRecord *plongout);
static long report_pdu360(int level);

/* global struct for devSup */
typedef struct {
        long            number;
        DEVSUPFUN       report;
        DEVSUPFUN       init;
        DEVSUPFUN       init_record;
        DEVSUPFUN       get_ioint_info;
        DEVSUPFUN       read_write;
        DEVSUPFUN       special_linconv;
} PDU360_DEV_SUP_SET;

/* Add reporting */
PDU360_DEV_SUP_SET devLongoutPDU360 = {6, report_pdu360, NULL, init_pdu360_longout_record,    NULL, write_pdu360_longout, NULL};

epicsExportAddress(dset, devLongoutPDU360);

static long
init_pdu360_longout_record(struct longoutRecord *plongout)
{
long   status = 0;
struct camacio *camio;
char  *parm, *str;

        if ( CAMAC_IO != plongout->out.type ) {
		status = S_dev_badBus;
		plongout->pact = TRUE;
		return status;
	}

	camio = &(plongout->out.value.camacio);
	parm = camio->parm;

       	if ( strcmp( parm, "DELAY" ) ) {
	       	sprintf( str, "Unknown task parameter %s", parm);
	       	status = S_dev_badSignal;
       	}

	if ( status ) {
	       recGblRecordError( status, (void *)plongout , (const char *)str );
	       plongout->pact = TRUE;
	}

	return status;
}

static long 
write_pdu360_longout(struct longoutRecord *plongout)
{

	PDU_F19_DELAY_US = plongout->val;

       	plongout->udf = FALSE;
       	return 0;
}

static long 
report_pdu360(int level)
{
      errlogPrintf("Device support for adjustable delay for PDU 360 Hz task\n");
      return 0;
}
