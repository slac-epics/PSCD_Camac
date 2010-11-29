/*=====================================================================
**
** The msgPIOPproc routine takes the unaltered error word from the PIOP
** error buffer and:
**
** Translates the PIOP msg word into the VMS Msg Symbol and
** the single character ERROR_BYTE mneumonic
**
**=====================================================================
*/

#include <devPIOP.h>
#include <drvPIOP.h>

#include <epicsExport.h>
#include <registryFunction.h>

/*
** Last legal 0-based message number
*/
#define LAST_MSG 46

/*******************************************************
** N.B. -- The ORDER of the following data is HARD WIRED
**         into the firmware of the PIOP.  Add to the
**         END of the list only.
********************************************************/

typedef struct
{
  char    *msg_p;
  int      printdata;
} MSG_S;

static MSG_S msg_as[LAST_MSG+1] = 
{
  {"PIOP Processor Crashed. Reset Required",1},  /* PIOP_DEADMANTIMEOUT */
  {"PIOP Undefined Message.",0},                 /* PIOP_UNDEFINEDMSG */ 
  {"Checksum error in Camac transfer.",1},       /* PIOP_CMND_CSUM */ 
  {"PAD DAC FAILED on range test.",1},           /* PIOP_DAC_RANGE */ 
  {"PAD DAC FAILED on differental linearity test.",1}, /* PIOP_DAC_MONO */ 
  {"PIOP Undefined Message.",0},                 /* PIOP_place_205 */ 
  {"No response from remote head",1},            /* PIOP_LINK */ 
  {"Foreign Command. -- Command Was ",1},        /* PIOP_CMND_ERROR */ 
  {"PIOP Undefined Message.",0},                 /* PIOP_place_312 */ 
  {"PAD protocol error detected.",1},            /* PIOP_PAD_PROTOCOL */ 
  {"PIOP Undefined Message.",0},                 /* PIOP_place_206 */ 
  {"NO RF DRIVE :",1},                           /* PIOP_DRIVEFAULT */ 
  {"Klystron Beam Current Exceeds Limit,",1},    /* PIOP_BEAMCUR */ 
  {"Klystron Beam Voltage Exceeds Limit",1},     /* PIOP_BEAMVOLTS */ 
  {"Reflected Power Fault.",1},                  /* PIOP_REPOWER */ 
  {"Battery Failure:",1},                        /* PIOP_BATTERY */ 
  {"WaveGuide Vacuum Fault.",1},                 /* PIOP_VACUUM */ 
  {"PIOP Undefined Message.",0},                 /* PIOP_place_303 */ 
  {"PIOP Undefined Message.",0},                 /* PIOP_place_302 */ 
  {"PIOP Undefined Message.",0},                 /* PIOP_place_301 */ 
  {"PIOP Undefined Message.",0},                 /* PIOP_place_300 */ 
  {"Trigger Disabled:",1},                       /* PIOP_TRIGGEROFF */ 
  {"RF Amplitude Jitter",1},                     /* PIOP_AMPJITTER */ 
  {"RF Amplitude Drift",1},                      /* PIOP_AMPDRIFT */ 
  {"Phase Jitter Exceeds Tolerance",1},          /* PIOP_PHASEJITTER */ 
  {"Phase Drift Exceeds Tolerance",1},           /* PIOP_PHASEDRIFT */ 
  {"Trigger Enabled:",1},                        /* PIOP_TRIGGERON */ 
  {"Operation sucessfully completed.",0},        /* PIOP_OKOK */ 
  {"SLED Cavity Tune Fault.",1},                 /* PIOP_SLED */ 
  {" Delta Temperature Fault:",1},               /* PIOP_KLYSTEMP */ 
  {"FOCUS Supply Fault.",1},                     /* PIOP_FOCUS */ 
  {"WATER ACC1 Fault.",1},                       /* PIOP_WATER_ACC1 */ 
  {"WATER ACC2 Fault.",1},                       /* PIOP_WATER_ACC2 */ 
  {"WATER WG1 Fault.",1},                        /* PIOP_WATER_WG1 */ 
  {"WATER WG2 Fault.",1},                        /* PIOP_WATER_WG2 */ 
  {"WATER KLYS Fault.",1},                       /* PIOP_WATER_KLYS */ 
  {"Modulator Fault.",1},                        /* PIOP_MOD_FAULT */ 
  {"WATER SUMMARY Fault.",1},                    /* PIOP_WATER_SUMMARY */ 
  {"WaveGuide Vacuum Fault.",1},                 /* PIOP_WG_VACUUM */  
  {"Klystron Vacuum Fault.",1},                  /* PIOP_KLYS_VACUUM */ 
  {"Low Beam Current.",1},                       /* PIOP_LOW_BEAM */ 
  {"Station has Timed Out",1},                   /* PIOP_STATION_FAULT */ 
  {"Re-sync required in PIOP.",1},               /* PIOP_RESYNC_TRAP */ 
  {"PPYY Timeout in PIOP.",1},                   /* PIOP_NOPPYY_TRAP */ 
  {"Low trigger rate detected in PIOP.",1},      /* PIOP_LOWRATE_TRAP */ 
  {"Modulator TOC Fault.",1},                    /* PIOP_TOC_FAULT */ 
  {"PIOP Undefined Message.",0}                  /* PIOP_place_336 */ 
};

long msgPIOPinit (struct subRecord *subr_p)
{
   subr_p->val = 0;  /* We save previous msg word here */
   return (0);
}

long msgPIOPproc (struct subRecord *subr_p)
{
   /*
   ** Form of error word is:
   **
   ** Bit 15   -- Optional Argument Token
   ** Bit 14-8 -- MSG Number
   ** Bit 7-0  -- Optional Argument Data
   */
   unsigned short piopword = subr_p->a;  /* PIOP msg word */
   unsigned short msgnum;
   unsigned short msgdat = 0;
   /*--------------------------------------------*/
   if (subr_p->a == subr_p->val)
      goto egress;              /* Don't log if same as last time */

   subr_p->val = piopword;     /* Save for next time */
   if (piopword == 0x8181)     /* Ignore nonsense message */
      goto egress;

   /*
   ** Decode message and optional argument
   */
   msgnum = (piopword &  0x7F00) >> 8;
   if (msgnum > LAST_MSG)
   {
      msgnum = 1;  /* Set to undefined message */
   }   
   else if ((piopword & 0x8000) != 0)
   {
      msgdat = piopword & 0x00ff;
   }
   /*
   ** Log the new error/status.
   */
   if ( msg_as[msgnum].printdata && ((piopword & 0x8000) != 0) )
      errlogPrintf("Record %s: %s: data = %2.2x\n", subr_p->name, 
                    msg_as[msgnum].msg_p, msgdat);
   else
      errlogPrintf("Record %s: %s\n", subr_p->name, msg_as[msgnum].msg_p);
egress:
   return (0);
}

epicsRegisterFunction(msgPIOPinit);
epicsRegisterFunction(msgPIOPproc);
