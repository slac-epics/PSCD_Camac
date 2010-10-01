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
** The last message/param we sent so we only send once.
*/
short lastmsg = 0;
short lastdat = 0;

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
  int      success;
} MSG_S;

static MSG_S msg_as[LAST_MSG+1] = 
{
  {"PIOP Processor Crashed. Reset Required",1,0},  /* PIOP_DEADMANTIMEOUT */
  {"PIOP Undefined Message.",0,0},                 /* PIOP_UNDEFINEDMSG */ 
  {"Checksum error in Camac transfer.",1,0},       /* PIOP_CMND_CSUM */ 
  {"PAD DAC FAILED on range test.",1,0},           /* PIOP_DAC_RANGE */ 
  {"PAD DAC FAILED on differental linearity test.",1,0}, /* PIOP_DAC_MONO */ 
  {"PIOP Undefined Message.",0,0},                 /* PIOP_place_205 */ 
  {"No response from remote head",1,0},            /* PIOP_LINK */ 
  {"Foreign Command. -- Command Was ",1,0},        /* PIOP_CMND_ERROR */ 
  {"PIOP Undefined Message.",0,0},                 /* PIOP_place_312 */ 
  {"PAD protocol error detected.",1,0},            /* PIOP_PAD_PROTOCOL */ 
  {"PIOP Undefined Message.",0,0},                 /* PIOP_place_206 */ 
  {"NO RF DRIVE :",1,0},                           /* PIOP_DRIVEFAULT */ 
  {"Klystron Beam Current Exceeds Limit,",1,0},    /* PIOP_BEAMCUR */ 
  {"Klystron Beam Voltage Exceeds Limit",1,0},     /* PIOP_BEAMVOLTS */ 
  {"Reflected Power Fault.",1,0},                  /* PIOP_REPOWER */ 
  {"Battery Failure:",1,0},                        /* PIOP_BATTERY */ 
  {"WaveGuide Vacuum Fault.",1,0},                 /* PIOP_VACUUM */ 
  {"PIOP Undefined Message.",0,0},                 /* PIOP_place_303 */ 
  {"PIOP Undefined Message.",0,0},                 /* PIOP_place_302 */ 
  {"PIOP Undefined Message.",0,0},                 /* PIOP_place_301 */ 
  {"PIOP Undefined Message.",0,0},                 /* PIOP_place_300 */ 
  {"Trigger Disabled:",1,0},                       /* PIOP_TRIGGEROFF */ 
  {"RF Amplitude Jitter",1,0},                     /* PIOP_AMPJITTER */ 
  {"RF Amplitude Drift",1,0},                      /* PIOP_AMPDRIFT */ 
  {"Phase Jitter Exceeds Tolerance",1,0},          /* PIOP_PHASEJITTER */ 
  {"Phase Drift Exceeds Tolerance",1,0},           /* PIOP_PHASEDRIFT */ 
  {"Trigger Enabled:",1,1},                        /* PIOP_TRIGGERON */ 
  {"Operation sucessfully completed.",0,1},        /* PIOP_OKOK */ 
  {"SLED Cavity Tune Fault.",1,0},                 /* PIOP_SLED */ 
  {" Delta Temperature Fault:",1,0},               /* PIOP_KLYSTEMP */ 
  {"FOCUS Supply Fault.",1,0},                     /* PIOP_FOCUS */ 
  {"WATER ACC1 Fault.",1,0},                       /* PIOP_WATER_ACC1 */ 
  {"WATER ACC2 Fault.",1,0},                       /* PIOP_WATER_ACC2 */ 
  {"WATER WG1 Fault.",1,0},                        /* PIOP_WATER_WG1 */ 
  {"WATER WG2 Fault.",1,0},                        /* PIOP_WATER_WG2 */ 
  {"WATER KLYS Fault.",1,0},                       /* PIOP_WATER_KLYS */ 
  {"Modulator Fault.",1,0},                        /* PIOP_MOD_FAULT */ 
  {"WATER SUMMARY Fault.",1,0},                    /* PIOP_WATER_SUMMARY */ 
  {"WaveGuide Vacuum Fault.",1,0},                 /* PIOP_WG_VACUUM */  
  {"Klystron Vacuum Fault.",1,0},                  /* PIOP_KLYS_VACUUM */ 
  {"Low Beam Current.",1,0},                       /* PIOP_LOW_BEAM */ 
  {"Station has Timed Out",1,0},                   /* PIOP_STATION_FAULT */ 
  {"Re-sync required in PIOP.",1,0},               /* PIOP_RESYNC_TRAP */ 
  {"PPYY Timeout in PIOP.",1,0},                   /* PIOP_NOPPYY_TRAP */ 
  {"Low trigger rate detected in PIOP.",1,0},      /* PIOP_LOWRATE_TRAP */ 
  {"Modulator TOC Fault.",1,0},                    /* PIOP_TOC_FAULT */ 
  {"PIOP Undefined Message.",0,0}                  /* PIOP_place_336 */ 
};

long msgPIOPinit (struct subRecord *subr_p)
{
   return (0);  /* Nothing required right now */
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
   unsigned short msgdat = 0xffff;
   /*--------------------------------------------*/
/*   printf ("In msg sub got data %4.4x\n",piopword); */
   
   msgnum = (piopword &  0x7F00) >> 8;
  if (msgnum > LAST_MSG)
   {
      msgnum = 1;  /* Set to undefined message */
   }   
   else if ((piopword & 0x8000) != 0)
   {
      msgdat = piopword & 0x00ff;
   }
   if ((msgnum == lastmsg) && (msgdat == lastdat))
      goto egress;         /* Same as last time so ignore */

   lastmsg = msgnum;
   lastdat = msgdat;
   if (msg_as[msgnum].success)
      goto egress;         /* Don't log if success */
   /*
   ** Log the new error/status.
   */
/**************** Temp
   if (msg_as[msgnum].printdata)
      errlogPrintf("Record %s PIOP message: %s data = %0x04x\n", subr_p->name, 
                    msg_as[msgnum].msg_p, msgdat);
   else
      errlogPrintf("Record %s PIOP message: %s\n", subr_p->name, 
                    msg_as[msgnum].msg_p);
******************/
egress:
   return (0);
}

epicsRegisterFunction(msgPIOPinit);
epicsRegisterFunction(msgPIOPproc);
