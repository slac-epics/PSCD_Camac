/***************************************************************************/
/* Filename: devPSCD.c                                                     */
/* Description: EPICS device support for PMC SLAC Camac Driver             */
/***************************************************************************/
#include <stdlib.h>

#include "drvPSCDLib.h"

#include <alarm.h>
#include <dbCommon.h>
#include <dbDefs.h>
#include <recSup.h>
#include <recGbl.h>
#include <devSup.h>
#include <devLib.h>
#include <link.h>
#include <dbScan.h>
#include <dbAccess.h>
#include <special.h>
#include <cvtTable.h>
#include <cantProceed.h>
#include <ellLib.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsThread.h>

#include <biRecord.h>
#include <boRecord.h>
#include <longinRecord.h>
#include <stringinRecord.h>
#include <epicsVersion.h>

#if EPICS_VERSION>=3 && EPICS_REVISION>=14
#include <epicsExport.h>
#endif


extern int PSCD_DRV_DEBUG;
extern struct PSCD_CARD pscd_card;

/* Device support implementation */

static long init_bi(struct biRecord *pbi);
static long read_bi(struct biRecord *pbi);
static long init_bo(struct boRecord *pbo);
static long write_bo(struct boRecord *pbo);
static long init_li(struct longinRecord *pli);
static long read_li(struct longinRecord *pli);
#if 0
static long init_si(struct stringinRecord *psi);
static long read_si(struct stringinRecord *psi);
#endif

/* global struct for devSup */
typedef struct {
    long		number;
    DEVSUPFUN	report;
    DEVSUPFUN	init;
    DEVSUPFUN	init_record;
    DEVSUPFUN	get_ioint_info;
    DEVSUPFUN	read_write;
    DEVSUPFUN	special_linconv;
} PSCD_DEV_SUP_SET;

PSCD_DEV_SUP_SET devBiPSCD=   {6, NULL, NULL, init_bi,  NULL, read_bi,  NULL};
PSCD_DEV_SUP_SET devBoPSCD=   {6, NULL, NULL, init_bo,  NULL, write_bo,  NULL};
PSCD_DEV_SUP_SET devLiPSCD=   {6, NULL, NULL, init_li,  NULL, read_li,  NULL};
#if 0
PSCD_DEV_SUP_SET devSiPSCD=   {6, NULL, NULL, init_si,  NULL, read_si,  NULL};
#endif

#if	EPICS_VERSION>=3 && EPICS_REVISION>=14
epicsExportAddress(dset, devBiPSCD);
epicsExportAddress(dset, devBoPSCD);
epicsExportAddress(dset, devLiPSCD);
#if 0
epicsExportAddress(dset, devSiPSCD);
#endif
#endif

typedef enum
{
    EPICS_RTYPE_NONE,
    EPICS_RTYPE_AI,
    EPICS_RTYPE_AO,
    EPICS_RTYPE_BI,
    EPICS_RTYPE_BO,
    EPICS_RTYPE_LI,
    EPICS_RTYPE_LO,
    EPICS_RTYPE_MBBI,
    EPICS_RTYPE_MBBO,
    EPICS_RTYPE_MBBID,
    EPICS_RTYPE_MBBOD,
    EPICS_RTYPE_SI,
    EPICS_RTYPE_SO,
    EPICS_RTYPE_WF
}   E_EPICS_RTYPE;

typedef enum {
    PSCD_BI_LOCALRMT=1,	/* avoid dpvt is 0 */
    PSCD_BI_SWITCH,
    PSCD_BO_SWITCH,
    PSCD_LI_FWVER,
    PSCD_LI_FWDATE,
    PSCD_SI_ID
} E_PSCD_FUNC;

static struct PARAM_MAP
{
        char param[40];
        E_EPICS_RTYPE rtype;;
        E_PSCD_FUNC funcflag;
} param_map[] = {
    {"LOCALRMT",	EPICS_RTYPE_BI,	PSCD_BI_LOCALRMT},
    {"SWITCH",		EPICS_RTYPE_BI,	PSCD_BI_SWITCH},
    {"SWITCH",		EPICS_RTYPE_BO,	PSCD_BO_SWITCH},
    {"FWVER",		EPICS_RTYPE_LI,	PSCD_LI_FWVER},
    {"FWDATE",		EPICS_RTYPE_LI,	PSCD_LI_FWDATE},
    {"ID",		EPICS_RTYPE_SI,	PSCD_SI_ID}
};
#define N_PARAM_MAP (sizeof(param_map)/sizeof(struct PARAM_MAP))

/* This function will be called by all device support */
/* The memory for PSCD_DEVDATA will be malloced inside */
static int PSCD_DevData_Init(dbCommon * precord, E_EPICS_RTYPE rtype, char * ioString)
{
    E_PSCD_FUNC    funcflag = 0;

    int     loop;

    /* param check */
    if(precord == NULL || ioString == NULL)
    {
        if(!precord) errlogPrintf("No legal record pointer!\n");
        if(!ioString) errlogPrintf("No INP/OUT field for record %s!\n", precord->name);
        return -1;
    }

    /* analyze INP/OUT string */
    for(loop=0; loop<N_PARAM_MAP; loop++)
    {
        if( 0 == strcmp(param_map[loop].param, ioString) && param_map[loop].rtype == rtype)
        {
            funcflag = param_map[loop].funcflag;
            break;
        }
    }
    if(loop >= N_PARAM_MAP)
    {
        errlogPrintf("Record %s INP/OUT %s is illegal!\n", precord->name, ioString);
        return -1;
    }

    precord->dpvt = (void *)funcflag;
    return 0;
}


/********* bi record *****************/
static long init_bi( struct biRecord * pbi)
{
    pbi->dpvt = NULL;

    if (pbi->inp.type!=INST_IO)
    {
        recGblRecordError(S_db_badField, (void *)pbi, "devBiPSCD Init_record, Illegal INP");
        pbi->pact=TRUE;
        return (S_db_badField);
    }

    pbi->mask = 0;

    if( PSCD_DevData_Init((dbCommon *)pbi, EPICS_RTYPE_BI, pbi->inp.value.instio.string) != 0 )
    {
        errlogPrintf("Fail to init devdata for record %s!\n", pbi->name);
        recGblRecordError(S_db_badField, (void *) pbi, "Init devdata Error");
        pbi->pact = TRUE;
        return (S_db_badField);
    }

    return 0;
}

static long read_bi(struct biRecord * pbi)
{
    E_PSCD_FUNC    funcflag = 0;

    if(!(pbi->dpvt)) return -1;

    funcflag = (E_PSCD_FUNC)(pbi->dpvt);

    switch(funcflag)
    {
    case PSCD_BI_LOCALRMT:
        if(pscd_card.switch_p)
        {
            UINT32 temp;
            temp = PSCD_MEM_GETL(pscd_card.switch_p);
            pbi->rval = (temp&0x1)?1:0;
        }
        else
        {
            recGblSetSevr(pbi,READ_ALARM,INVALID_ALARM);
        }
        break;
    case PSCD_BI_SWITCH:
        if(pscd_card.switch_p)
        {
            UINT32 temp;
            temp = PSCD_MEM_GETL(pscd_card.switch_p);
            pbi->rval = (temp&0x2)?1:0;
        }
        else
        {
            recGblSetSevr(pbi,READ_ALARM,INVALID_ALARM);
        }
        break;
    default:
        return -1;
    }

    return 0;	/* do conversion */
}

/********* bo record *****************/
static long init_bo( struct boRecord * pbo)
{
    E_PSCD_FUNC    funcflag = 0;

    pbo->dpvt = NULL;

    if (pbo->out.type!=INST_IO)
    {
        recGblRecordError(S_db_badField, (void *)pbo, "devBoPSCD Init_record, Illegal OUT");
        pbo->pact=TRUE;
        return (S_db_badField);
    }

    pbo->mask = 0;

    if( PSCD_DevData_Init((dbCommon *)pbo, EPICS_RTYPE_BO, pbo->out.value.instio.string) != 0 )
    {
        errlogPrintf("Fail to init devdata for record %s!\n", pbo->name);
        recGblRecordError(S_db_badField, (void *) pbo, "Init devdata Error");
        pbo->pact = TRUE;
        return (S_db_badField);
    }

    funcflag = (E_PSCD_FUNC)(pbo->dpvt);

    switch(funcflag)
    {
    case PSCD_BO_SWITCH:
        if(pscd_card.switch_p)
        {
            UINT32 temp;
            temp = PSCD_MEM_GETL(pscd_card.switch_p);
            pbo->rval = (temp&0x2)?1:0;
            pbo->udf = FALSE;
            pbo->stat = pbo->sevr = NO_ALARM;
        }
        break;
    default:
        break;
    }
    return 0;
}

static long write_bo(struct boRecord * pbo)
{
    E_PSCD_FUNC    funcflag = 0;

    if(!(pbo->dpvt)) return -1;

    funcflag = (E_PSCD_FUNC)(pbo->dpvt);

    switch(funcflag)
    {
    case PSCD_BO_SWITCH:
        if(pscd_card.switch_p)
        {
            if(pbo->rval != 0)
	    {
                PSCD_MEM_PUTL(pscd_card.switch_p, 0x4);
	    }
	    else
	    {
                PSCD_MEM_PUTL(pscd_card.switch_p, 0x0);
	    }
        }
        else
        {
            recGblSetSevr(pbo,WRITE_ALARM,INVALID_ALARM);
        }
        break;
    default:
        return -1;
    }

    return 0;
}

/********* li record *****************/
static long init_li( struct longinRecord * pli)
{
    pli->dpvt = NULL;

    if (pli->inp.type!=INST_IO)
    {
        recGblRecordError(S_db_badField, (void *)pli, "devLiPSCD Init_record, Illegal INP");
        pli->pact=TRUE;
        return (S_db_badField);
    }

    if( PSCD_DevData_Init((dbCommon *)pli, EPICS_RTYPE_LI, pli->inp.value.instio.string) != 0 )
    {
        errlogPrintf("Fail to init devdata for record %s!\n", pli->name);
        recGblRecordError(S_db_badField, (void *) pli, "Init devdata Error");
        pli->pact = TRUE;
        return (S_db_badField);
    }

    return 0;
}

static long read_li(struct longinRecord * pli)
{
    E_PSCD_FUNC    funcflag = 0;

    if(!(pli->dpvt)) return -1;

    funcflag = (E_PSCD_FUNC)(pli->dpvt);

    switch(funcflag)
    {
    case PSCD_LI_FWDATE:
        pli->val = pscd_card.fwDate;
        break;
    case PSCD_LI_FWVER:
        pli->val = pscd_card.fwVer;
        break;
    default:
        return -1;
    }

    return 0;
}


