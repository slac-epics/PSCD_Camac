/*
** Message queues and message sent to drvPIOP to execute the Camac PIOP function from devPIOP
** device support.
*/

/***********************************
** Message for IPL function
***********************************/

typedef enum {IPL, LAST } FUNC_TE; /* Msgs to drvPIOP */

typedef struct
{
  short crate;
  short slot;
  char  fname[40];
} IPLSTRUC_TS;

/*
** Unify all parameters for the message to the PIOP thread.
*/
typedef union
{
  IPLSTRUC_TS ipl;
  int         dum;
} THREADPARM_U;

/*
** This is the general message. All functions get the record ptr.
*/
typedef struct
{
  void        *rec_p;
  FUNC_TE      func_e;
  THREADPARM_U parm_u;
} THREADMSG_TS;
