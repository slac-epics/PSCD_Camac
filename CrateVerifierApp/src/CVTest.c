/*
=============================================================

  Abs: Crate Verifier Diagnostic Functions

  Name: CVTest.c

         Message:
         --------
         CV_SendMessage - send camac request message to queue

         Test Read/Write Functions: 
         -------------------------
         CV_RdData           - read data register
         CV_RdId             - read id register
         CV_RdVoltage        - read analog voltage register
         CV_WtData           - set data register
         CV_RW               - read write line test #1,2,3 & 4 (RW1-24)
         CV_RW2              - read write line test #5,6,7 & 7 (RW1-16)

         Send Read/Write Message to Queue:
         ---------------------------------
         CV_RdDataMsg        - send msg request to read data register
         CV_RdIdMsg          - send msg request to read id register
         CV_RdVoltageMsg     - send msg request to read crate voltages
         CV_RdCrateStatusMsg - send msg request to read crate voltages
         CV_WtDataMsg        - send msg request to set data register

         Utilities:
         -------------
         blockWordSwap       - Swap word data read in a camac word block transfer
         CV_RWDataGet        - Save the read-write line word data and check for errors


  Note: * indicates static functions

  Proto: CVTest_proto.h

  Auth: 23-May-2010, K. Luchini       (LUCHINI)
  Rev : dd-mmm-yyyy, Reviewer's Name  (USERNAME)
-------------------------------------------------------------
  Mod:
        dd-mmm-yyyy, First Lastname   (USERNAME):
          comment

=============================================================
*/

/* Header files */
#include "drvPSCDLib.h"
#include "slc_macros.h"        /* for vmsstat_t */
#include "cam_proto.h"         /* for camalo,camalo_reset,camadd,camio,camgo */
#include "devCV.h"
#include "drvCV_proto.h"
#include "CVTest_proto.h"

int CV_TEST_DEBUG=0;


/*====================================================
 
  Abs:  Set the Data Register
 
  Name: CV_WtData
 
  Args: branch                       Camac Branch
          Type: integer              Note: ignored
          Use:  short
          Acc:  read-only
          Mech: By value

        crate                        Camac Crate Number
          Type: integer             
          Use:  short
          Acc:  read-only
          Mech: By value

        slot                         Camac Crate Slot Number
          Type: integer           
          Use:  short
          Acc:  read-only
          Mech: By value

        data_p                       Data register data
          Type: pointer              Note: optional argument               
          Use:  unsigned long        If set to NULL the default
          Acc:  read-write           data pattern CV_DATA_PATTERN
          Mech: By reference         will be output.
                                     

  Rem:  The purpose of this funciton is to set the data regiser
        with the value specified in the data_p argument. If the 
        data_p argument is not supplied (i.e. NULL) the the default
        pattern CV_DATA_PATTERN will be output to the data register.
        A readback of the register is done to verify that the output
        has latched.

  Side: None
  
  Ret:  vmsstat_t
            CRAT_OKOK         - Successfully completed
            CRAT_INITFAIL     - Write failed, output data did not latch
            CAM_READ_MISMATCH - Read does not match setpoint
            Otherwise, see return codes from
               camio()             
            
=======================================================*/
vmsstat_t CV_WtData( short branch, short crate, short slot, unsigned long *data_p  )
{
   vmsstat_t      iss   = CRAT_OKOK;                  /* return status                */
   unsigned int   ctlw  = 0;                          /* Camac control word           */
   unsigned short emask = 0xF3F3;                     /* return on NOX and NOQ        */
   unsigned short bcnt   =  sizeof(long);             /* byte counte of data          */
   statd_4u_ts    wt_statd_s = {0,CV_DATA_PATTERN};   /* output data to data register */
   statd_4u_ts    rbk_statd_s = {0,0};                /* readback data register       */


   if (data_p) 
      wt_statd_s.data = *data_p & CV_DATA_MASK;
   ctlw = (crate << CCTLW__C_shc) | (slot << CCTLW__M_shc) | F20A0 | CCTLW__P24;
   iss = camio (&ctlw, &wt_statd_s.data, &bcnt, &wt_statd_s.stat, &emask);
   if (!SUCCESS(iss))
     printf("CV[%hd %hd %hd]: Failed to write 0x%8.8lx to Data Register - stat=0x%8.8lX\n",
            branch,crate,slot,wt_statd_s.data,iss);
   else 
   {
     ctlw = (crate << CCTLW__C_shc) | (slot << CCTLW__M_shc) | F4A0 | CCTLW__P24;
     iss  = camio (&ctlw, &rbk_statd_s, &bcnt, &rbk_statd_s.stat, &emask);
     if (!SUCCESS(iss))
        printf("CV[%hd %hd %hd]: Failed to Read Data Register - stat=0x%8.8lX\n",branch,crate,slot,iss);
     else
     {
        if ( rbk_statd_s.data!=wt_statd_s.data )
           iss = CRAT_INITFAIL;  
	else
           iss = CRAT_OKOK; 
        rbk_statd_s.data = rbk_statd_s.data & CV_DATA_MASK;
        printf("CV[%hd %hd %hd]: Data Register failed to latch, wt=0x%8.8lx rbk=0x%8.8lx\n",
               branch,crate,slot,
               wt_statd_s.data,
               rbk_statd_s.data);
     }
   }
  return(iss);
}


/*====================================================
 
  Abs:  Read Data Register
 
  Name: CV_RdData
 
  Args: branch                       Camac Branch
          Type: integer              Note: ignored
          Use:  short
          Acc:  read-only
          Mech: By value

        crate                        Camac Crate Number
          Type: integer             
          Use:  short
          Acc:  read-only
          Mech: By value

        slot                         Camac Crate Slot Number
          Type: integer           
          Use:  short
          Acc:  read-only
          Mech: By value

        data_p                       Data register data
          Type: pointer              Note: optional argument               
          Use:  unsigned long        if set to NULL
          Acc:  read-write
          Mech: By reference


  Rem:  The purpose of this to read the data register of the
        speficied crate verifier module and the data if the
        argument data_p has been  supplied (not NULL).

  Side: None
  
  Ret:  vmsstat_t
            CRAT_OKOK         - Successfully completed
            CRAT_VERDATA1     - Data does not match expected pattern
            Otherwise, see return codes from
               camio()             
            
=======================================================*/
vmsstat_t CV_RdData( short branch, short crate, short slot, unsigned long *data_p )
{
   vmsstat_t      iss   = CRAT_OKOK;            /* return status               */
   unsigned int   ctlw  = 0;                    /* Camac control word          */
   unsigned short emask = 0xF000;               /* return on NOX and NOQ       */
   unsigned short bcnt  = sizeof(long);         /* byte counte of data         */
   statd_4u_ts    statd_s = {0,0};              /* Camac status-data           */


   if (data_p) *data_p=0;
   ctlw = (crate << CCTLW__C_shc) | (slot << CCTLW__M_shc) | (CCTLW__F4 | CCTLW__P24);
   iss = camio (&ctlw, &statd_s.data, &bcnt, &statd_s.stat, &emask);
   if (!SUCCESS(iss))
     printf("CV[%hd %hd %hd]: Failed to read Data Register - stat=0x%8.8lX\n",branch,crate,slot,iss);
   else 
   {
     if (statd_s.data != CV_DATA_PATTERN)
     {
        iss = CRAT_VERDAT1;
        printf("CV[%hd %hd %hd]: Failed to read Data Register - data=0x%4.4lX stat=0x%8.8lX\n",
                branch,crate,slot,statd_s.data,iss);
     }
     else
     {
        iss = CRAT_OKOK;
        statd_s.data = statd_s.data & CV_DATA_MASK;
        if (data_p) *data_p = statd_s.data;
        printf("CV[%hd %hd %hd]: Data Register 0x%4.4lX\n",branch,crate,slot,statd_s.data);
     }
  }
  return(iss);
}


/*====================================================
 
  Abs:  Read Identification Register
 
  Name: CV_RdId
 
  Args: branch                       Camac Branch
          Type: integer              Note: ignored
          Use:  short
          Acc:  read-only
          Mech: By value

        crate                        Camac Crate Number
          Type: integer             
          Use:  short
          Acc:  read-only
          Mech: By value

        slot                         Camac Crate Slot Number
          Type: integer           
          Use:  short
          Acc:  read-only
          Mech: By value

        data_p                       Module id
          Type: pointer              Note: optional argument               
          Use:  unsigned long        if set to NULL
          Acc:  read-write
          Mech: By reference


  Rem:  The purpose of this to read the module id register
        of the specified crate verifier modules and return
        the data if the data_p argument has been supplied (not NULL).
       
        It is a requirment that the crate verifier id register, which
        is set by switches on the front panel, match the crate number
        specified by the Serial Crate Controller module within the crate.
        The Serial Crate Controller module crate number is set by a thumb
        wheel on the front panel.

  Side: None
  
  Ret:  vmsstat_t
            CRAT_OKOK     - Successfully completed
            CAM_INV_MODID - Invalid module id
            Otherwise, see return codes from
               camio()             
            
=======================================================*/
vmsstat_t CV_RdId( short branch, short crate, short slot, unsigned long *data_p )
{
   vmsstat_t      iss   = CRAT_OKOK;         /* return status               */
   unsigned int   ctlw  = 0;                 /* Camac control word          */
   unsigned short emask = 0xF2F2;            /* return on NOX, expect Q=0   */
   unsigned short bcnt  = sizeof(long);      /* byte counte of data         */
   statd_4u_ts    statd_s = {0,0};           /* Camac status-data           */


   if (data_p) *data_p=0;
   ctlw = (crate << CCTLW__C_shc) | (slot << CCTLW__M_shc) | (CCTLW__F4 | CCTLW__A3) | CCTLW__P24;
   iss = camio (&ctlw, &statd_s.data, &bcnt, &statd_s.stat, &emask);
   if (!SUCCESS(iss))
     printf("CV[%hd %hd %hd]: Failed to read ID Register - stat=0x%8.8lX\n",branch,crate,slot,iss);
   else
   {
     statd_s.data = statd_s.data & CV_ID_MASK;
     if (statd_s.data != crate)
     {
        iss = CAM_INV_MODID;
        printf("CV[%hd %hd %hd]: Warning ID (%ld) does not match crate number\n",branch,crate,slot,statd_s.data);
     }
     else
     {
        iss = CRAT_OKOK;
        printf("CV[%hd %hd %hd]: ID=%ld\n",branch,crate,slot,statd_s.data);
     }

     if (data_p) *data_p = statd_s.data;
   }
   return(iss);
}

/*====================================================
 
  Abs:  Read Analog Register
 
  Name: CV_RdVoltage
 
  Args: branch                       Camac Branch
          Type: integer              Note: ignored
          Use:  short
          Acc:  read-only
          Mech: By value

        crate                        Camac Crate Number
          Type: integer             
          Use:  short
          Acc:  read-only
          Mech: By value

        slot                         Camac Crate Slot Number
          Type: integer           
          Use:  short
          Acc:  read-only
          Mech: By value


  Rem:  The purpose of this to read the analog voltage register
        of the specified crate verifier modules. If the data_a
        argument is supplied (not NULL) then the raw data requested
        by the argument first_chan and nchans is returned

  Side: None
  
  Ret:  vmsstat_t
            CAM_OKOK  - Successfully completed
            CAM_BAD_A - Number of channels requested is illegal, bad subaddress.
            Otherwise, see return codes from
               camio()             
            
=======================================================*/
vmsstat_t CV_RdVoltage( short branch, short crate, short slot )
{
   CV_VOLT_MULT;
   vmsstat_t        iss   = CAM_OKOK;                     /* return status          */
   unsigned int     ctlw  = 0;                            /* Camac control word     */
   unsigned short   emask = 0xF000;                       /* return on NOX and NOQ  */ 
   unsigned short   bcnt  = sizeof(short);                /* byte counte of data    */
   unsigned long    subadr = 0;                           /* index counter          */
   short            rdata = 0;
   float            fdata = 0.0;
   float            zero  = CV_ANLG_ZERO;
   float            slope = CV_ANLG_SLOPE;
   statd_2_ts       statd_s;                              /* status data            */


   printf("\t\tCrate Voltages & Temperature:\n");
   for (subadr=0; (subadr<CV_NUM_ANLG_CHANNELS) && SUCCESS(iss); subadr++)
   {
     statd_s.data = 0;
     ctlw = (crate << CCTLW__C_shc) | (slot << CCTLW__M_shc) | F5 | subadr;
     iss = camio(&ctlw, &statd_s.data, &bcnt, &statd_s.stat, &emask);
     if (!SUCCESS(iss))
       printf("CV[%hd %hd %hd]: Failed to read Analog Register - stat=0x%8.8lX\n",branch,crate,slot,iss);
     else
     {
        rdata  = statd_s.data & CV_ANLG_MASK;
        fdata  = (slope * rdata - zero)  * vmult_as[subadr].m1;
        fdata *= vmult_as[subadr].m2;
        printf("\t\t%s\t%5.2f\t0x%4.4hX stat=0x%8.8lX\n",vmult_as[subadr].label_c,fdata,rdata,iss);
     }     
   }/* End of FOR loop */

   return(iss);
}

/*====================================================
 
  Abs:  Camac Crate Read Write Lines Test (R1-24)
 
  Name: CV_RW
 
  Args: branch                       Camac Branch
          Type: integer              Note: ignored
          Use:  short
          Acc:  read-only
          Mech: By value

        crate                        Camac Crate Number
          Type: integer             
          Use:  short
          Acc:  read-only
          Mech: By value

        slot                         Camac Crate Slot Number
          Type: integer           
          Use:  short
          Acc:  read-only
          Mech: By value

  Rem:  The purpose of this function is to setup the Camac
        package for the Camac bus read/write line test. 
        There are eight test in all, the sequence of which 
        is as follows:

           Perform the following with P24:
           1. Read line test using walking one bit
           2. Read line test using walking zero bit
           3. Write line test with simulated walking one bit
           4. Write line test with simulated walking zero bit


        Read line test:
          1) walking one bit, testing R1-24. The pattern
             will rotate from R24-R1 on the twenty-sixth read. The first
             read for the walking one bit will be zeros. The next read 
             will have a one in R1 and zero's in R24-R2

          2) walking zero bit, testing R1-24. The pattern
             will rotate from R24-R1 on the twenty-sixth read. The first
             read for the walking zero pattern will be all ones. The next read 
             will have a zero in R1 and one's in R24-R2.

         Write line test:
          3) Simulated walking one

          4) Simulated walking zero

  Side: None
  
  Ret:  vmsstat_t
            CAM_OKOK  - Successfully completed
            Otherwise, see return codes from
               camio()             
            
=======================================================*/
vmsstat_t CV_RW( short branch, short crate, short slot )
{
    RW_LINE_OK;
    vmsstat_t            iss      = CAM_OKOK;     /* return status                              */
    unsigned long        status   = OK;           /* local return status                        */
    unsigned int         clr_ctlw = 0;            /* Camac control word to clear the C-line     */
    unsigned int         wt_ctlw  = 0;            /* Camac control word set the ROTATE register */
    unsigned int         rd_ctlw  = 0;            /* Camac control word read the DATA register  */
    unsigned short       emask    = 0xF000;       /* return on NOX and NOQ                      */ 
    unsigned short       bcnt     = 0;            /* byte counte of data                        */
    unsigned short       nobcnt   = 0;            /* byte count of zero                         */ 
    unsigned short       i        = 0;            /* index counter                              */
    unsigned short       j        = 0;            /* index counter                              */
    unsigned int         stat     = 0;            /* pulse the C-line                           */
    unsigned int         wt_stat;                 /* set the ROTATE register for walking zeros  */ 
    statd_4u_ts          wt_data_statd_s;         /* set DATA register with   P24               */
    statd_4u_ts          rd_data_statd_s;         /* read DATA register with P24                */
    campkt_statd_4u_rw_ts rd_statd_s;              /* read DATA register and rotate left         */   
    cv_rwLine_type_te    type_e = WALKING_ONE;    /* Type of read write bit test                */
    unsigned int         nelem= RW_LINE_NUM;      /* number of words in array                   */
    unsigned long        ldata_a[RW_LINE_NUM];    /* read write data saved as 32-bit words      */
    unsigned long        err_a[RW_LINE_NUM];      /* Error flag for read write data             */
    unsigned long        expected_data_a[RW_LINE_NUM];  /* expected data results                */


    /*
     * Set some standard Camac control words used by the function,
     * The first is to clear the registers on the bus by pulsing the C-line,
     * the second is to set the ROTATE register on the crate verifier and the 
     * last is to read the DATA register on the crate verifier module.
     */
     clr_ctlw = (crate << CCTLW__C_shc) | M28 | F26A9;
     wt_ctlw  = (crate << CCTLW__C_shc) | (slot << CCTLW__M_shc) | F20A3;
     rd_ctlw  = (crate << CCTLW__C_shc) | (slot << CCTLW__M_shc) | F4A1 | CCTLW__P24;

     /*---------------------------------------------------------------------------- 
      * Ok, here we are begining Test #1, which is the walking one bit test.
      * of read lines. The first read for the walking one bit test 
      * will be zeros. The next read will have a one in R1 and zero's
      * in R24-R2. 
      *
      * Now, clear register on the bus by pulsing the C-line.
      */
     if (!SUCCESS(iss = camio(&clr_ctlw, 0 , &nobcnt, &stat, &emask )))
     {
       printf("CV[%hd %hd %hd]: RW Lines failed to clear the ROTATE reg. (pulse C-line) - stat=0x%8.8x iss=0x%8.8lX\n",
               branch,crate,slot,stat,iss);
        goto egress;
     }

    /* 
     * Read ROTATE register and rotate pattern left.
     * We expect to read 100 bytes of data, first read and then
     * the read lines R1-24.
     */
     bcnt = sizeof(rd_statd_s.data_a);
     memset(&rd_statd_s,0,sizeof(rd_statd_s));
     if (!SUCCESS(iss = camio(&rd_ctlw, rd_statd_s.data_a, &bcnt, &rd_statd_s.stat, &emask )))
     {
        printf("CV[%hd %hd %hd]: RW Lines failed to read the ROTATE reg - stat=0x%8.8x  iss=0x%8.8lX\n",
               branch,crate,slot,stat,iss);
        goto egress;
     }

     /* Check data for errors */
     status = CV_RWDataGet(type_e, nelem, NULL, rd_statd_s.data_a, err_a,expected_data_a);

     printf("RW Line Test #1: %s\tstat=0x%8.8X  iss=0x%8.8lx\n",(status)?"Failed   ":"Successful",rd_statd_s.stat,iss);
     for (i=0; i<nelem; i++)
       printf("...(%.2d):  data=0x%8.8lx  expected=0x%8.8lx\t%s\n",
              i,
              rd_statd_s.data_a[i],
              expected_data_a[i],
              (err_a[i])?"Error":"");
     printf("\n");   

     /*---------------------------------------------------------------------------- 
      * Ok, here we are beginnign Test #2, which is the walking zero bit test.
      * of the read lines. The first read for the walking zero test
      * will be ones. The next will have a zero in R1 and one's in R24-R2.
      *
      * Now, clear register on the bus by pulsing the C-line.
      */
      if (!SUCCESS(iss = camio(&clr_ctlw, 0 , &nobcnt, &stat, &emask )))
      {
          printf("CV[%hd %hd %hd]: RW Lines failed to clear the ROTATE reg. (pulse C-line). stat=0x%8.8x iss=0x%8.8lX\n",
                  branch,crate,slot,stat,iss);
          goto egress;
      }

      if (!SUCCESS(iss = camio(&wt_ctlw, 0, &nobcnt, &wt_stat, &emask )))
      {
         printf("CV[%hd %hd %hd]: RW Lines failed to set the ROTATE reg for walking zero.  stat=0x%8.8x  iss=0x%8.8lX\n",
                 branch,crate,slot,wt_stat,iss);
         goto egress;
      }
 
      /* Read ROTATE register and rotate pattern left. */
      bcnt = sizeof(rd_statd_s.data_a);
      memset(&rd_statd_s,0,sizeof(rd_statd_s));
      if (!SUCCESS(iss = camio(&rd_ctlw, rd_statd_s.data_a, &bcnt, &rd_statd_s.stat, &emask )))
      {
          printf("CV[%hd %hd %hd]: RW Lines failed to read the ROTATE reg. stat=0x%8.8x  iss=0x%8.8lX\n",
              branch,crate,slot,stat,iss);
          goto egress;
      }

     /* Swap word data crom camac block transfer.*/
     type_e = WALKING_ZERO;
     status = CV_RWDataGet(type_e,nelem,NULL,rd_statd_s.data_a,err_a,expected_data_a);

     printf("RW Line Test #2: %s\tstat=0x%8.8X  iss=0x%8.8lx\n",(status)?"Failed   ":"Successful",rd_statd_s.stat,iss);
     for (i=0; i<nelem; i++)
       printf("...(%.2d): data=0x%8.8lx  expected=0x%8.8lx\t%s\n",
              i,
              rd_statd_s.data_a[i],
              expected_data_a[i],
              (err_a[i])?"Error":"");
      printf("\n");

     /*----------------------------------------------------------------------------
      * Ok, here we are beginnign Test #3 and #4, which are 
      *
      *       simulated walking one bit test with   P24
      *       simulated  walking zero bit test with P24
      *
      * where the wrote does not use P24 and the read does use P24.
      *
      * Now, clear register on the bus by pulsing the C-line.
      */
     if (!SUCCESS(iss = camio(&clr_ctlw, 0 , &nobcnt, &stat, &emask )))
     {
        printf("CV[%hd %hd %hd]: RW Lines failed to clear the ROTATE reg. (pulse C-lin2) - stat=0x%8.8x iss=0x%8.8lX\n",
                 branch,crate,slot,stat,iss);
        goto egress;
     }

     /* Set the DATA register (P24) clearing out the old data from the high order bytes. */
     bcnt = sizeof(long);
     memset(&wt_data_statd_s,0,sizeof(wt_data_statd_s));
     wt_ctlw = (crate << CCTLW__C_shc) |(slot << CCTLW__M_shc) | F20A0 | CCTLW__P24;
     if (!SUCCESS(iss = camio(&wt_ctlw, &wt_data_statd_s.data, &bcnt, &wt_data_statd_s.stat, &emask )))
     {
         printf("CV[%hd %hd %hd]: RW Lines failed to set the DATA reg (P24) - stat=0x%8.8x  iss=0x%8.8lX\n",
              branch,crate,slot,stat,iss);
         goto egress;
     }

     wt_data_statd_s.data = 0;
     for (j=0; j<RW_LINE_NUM_TYPE; j++)
     {
        /* Clear out local data buffers */
        memset(ldata_a,0,sizeof(ldata_a));
        memset(err_a,0,sizeof(err_a));

        /* Read all of the data */
        for (i=0; i<nelem; i++)
        { 
           /* Set the DATA register (P24)*/
           bcnt    = sizeof(long);
           wt_data_statd_s.stat = 0;
           wt_data_statd_s.data = rwLineOk_a[j][i];
           wt_ctlw = (crate << CCTLW__C_shc) |(slot << CCTLW__M_shc) | F20A0 | CCTLW__P24;
           if (!SUCCESS(iss= camio(&wt_ctlw, &wt_data_statd_s.data, &bcnt, &wt_data_statd_s.stat, &emask )))
	   {
              printf("CV[%hd %hd %hd]: RW Lines failed to set the DATA reg - stat=0x%8.8x  iss=0x%8.8lX\n",
                     branch,crate,slot,stat,iss);
              goto egress;
	   }
 
           /* Read the DATA register (P24) */
           rd_data_statd_s.stat = 0;
           rd_data_statd_s.data = 0;
           rd_ctlw  = (crate << CCTLW__C_shc) | (slot << CCTLW__M_shc) | F4A0 | CCTLW__P24;
           if(!SUCCESS(iss = camio(&rd_ctlw, &rd_data_statd_s.data, &bcnt, &rd_data_statd_s.stat, &emask )))
	   {
              printf("CV[%hd %hd %hd]: RW Lines failed to read the DATA reg - stat=0x%8.8x  iss=0x%8.8lX\n",
                     branch,crate,slot,stat,iss);
              goto egress;
	   }
           /* Save status and data in separte arrays */
           ldata_a[i] = rd_data_statd_s.data;
	}
        /* 
	 * Check for errors. We want to read all of the data before
	 * we check for errors so that we can print a summary of the
	 * results, (ie. pass or fail).
	 */
        status = CV_RWDataGet(j,nelem,NULL,ldata_a,err_a,expected_data_a);
        printf("RW Line Test #%d: %s\tstat=0x%8.8X  iss=0x%8.8lx\n",
                j+3,(status)?"Failed   ":"Successful",rd_statd_s.stat,iss);  
        for (i=0; i<nelem; i++)
           printf("...(%.2d):  data=0x%8.8lx  expected=0x%8.8lx\t%s\n",
		  i,
                  ldata_a[i],
                  expected_data_a[i],
                  (err_a[i])?"Error":"" );
        printf("\n");

     } /* End of j FOR loop */
     printf("\n");

egress:

    /* Clear the registers on the bus before exiting for clean up. */
     camio(&clr_ctlw, 0 , &nobcnt, &stat, &emask );

   return(iss);
}


/*====================================================
 
  Abs:  Camac Crate Read Write Lines Test (R1-16)
 
  Name: CV_RW2
 
  Args: branch                       Camac Branch
          Type: integer              Note: ignored
          Use:  short
          Acc:  read-only
          Mech: By value

        crate                        Camac Crate Number
          Type: integer             
          Use:  short
          Acc:  read-only
          Mech: By value

        slot                         Camac Crate Slot Number
          Type: integer           
          Use:  short
          Acc:  read-only
          Mech: By value

  Rem:  The purpose of this function is to perform the Camac Crate
        Read line test
          1) walking one bit, testing R1-16. The pattern
             will rotate from R24-R1 on the twenty-sixth read. The first
             read for the walking one bit will be zeros. The next read 
             will have a one in R1 and zero's in R24-R2

          2) walking zero bit, testing R1-16. The pattern
             will rotate from R24-R1 on the twenty-sixth read. The first
             read for the walking zero pattern will be all ones. The next read 
             will have a zero in R1 and one's in R24-R2.

          3) Simulated walking one

          4) Simulated walking zero

  Side: Camac word block transfer REQUIRE that the data is WORD swapped.
        It is important to be aware that an EVEN number of WORDS must be read
        so that the last data word can be read and swapped from the upper byte.
        If this additional word is not read, the last data point expected will
        be garbage.
  
  Ret:  vmsstat_t
            CAM_OKOK  - Successfully completed
            Otherwise, see return codes from
               camio()             
            
=======================================================*/
vmsstat_t CV_RW2( short branch, short crate, short slot )
{
    RW_LINE_OK;
    vmsstat_t           iss      = CAM_OKOK;     /* return status                              */
    unsigned long       status   = OK;           /* local return status                        */
    unsigned int        nelem= RW_LINE_NUM2+1;  /* number of words in array                   */
    unsigned int        clr_ctlw = 0;            /* Camac control word to clear the C-line     */
    unsigned int        wt_ctlw  = 0;            /* Camac control word set the ROTATE register */
    unsigned int        rd_ctlw  = 0;            /* Camac control word read the DATA register  */
    unsigned short      emask    = 0xF000;       /* return on NOX and NOQ                      */ 
    unsigned short      bcnt     = 0;            /* byte counte of data                        */
    unsigned short      nobcnt   = 0;            /* byte count of zero                         */ 
    unsigned short      i        = 0;            /* index counter                              */
    unsigned short      j        = 0;            /* index counter                              */
    unsigned long       ldata_a[RW_LINE_NUM2];   /* read write data saved as 32-bit words      */
    unsigned long       err_a[RW_LINE_NUM2];     /* Error flag for read write data             */
    unsigned long       expected_data_a[RW_LINE_NUM2]; /* expected data results                */
    unsigned int        stat     = 0;            /* pulse the C-line                           */
    unsigned int        wt_stat;                 /* set the ROTATE register for walking zeros  */ 
    statd_2u_ts         wt_sdata_statd_s;        /* set DATA register without P24              */
    statd_4u_ts         wt_data_statd_s;         /* set DATA register with   P24               */
    statd_4u_ts         rd_data_statd_s;         /* read DATA register with P24                */
    campkt_statd_rw_ts  rd_statd_s;              /* read DATA register and rotate left         */   
    cv_rwLine_type_te   type_e = WALKING_ONE;    /* Type of read write bit test                */


    /*
     * Set some standard Camac control words used by the function,
     * The first is to clear the registers on the bus by pulsing the C-line,
     * the second is to set the ROTATE register on the crate verifier and the 
     * last is to read the DATA register on the crate verifier module.
     */
     clr_ctlw = (crate << CCTLW__C_shc) | M28 | F26A9;
     wt_ctlw  = (crate << CCTLW__C_shc) | (slot << CCTLW__M_shc) | F20A3;
     rd_ctlw  = (crate << CCTLW__C_shc) | (slot << CCTLW__M_shc) | F4A1;

     /*---------------------------------------------------------------------------- 
      * Ok, here we are begining Test #5, which is the walking one bit test.
      * of read lines R1-16. We need to read two extra word. The first word,
      * our 17th, for the walking one bit test will be all zeros.
      * The next will have a one in R1 and zero's in R24-R2.
      * The 18th word is required becasue this is a camac word
      * block transfer which will require a word swap. This means that we need
      * to read an even number of words so that last word of data can be acquired.
      *
      * Now, clear register on the bus by pulsing the C-line.
      */
     if (!SUCCESS(iss = camio(&clr_ctlw, 0 , &nobcnt, &stat, &emask )))
     {
       printf("CV[%hd %hd %hd]: RW Lines failed to clear the ROTATE reg. (pulse C-line) - stat=0x%8.8x iss=0x%8.8lX\n",
               branch,crate,slot,stat,iss);
        goto egress;
     }

    /* 
     * Read ROTATE register and rotate pattern left.
     * We want 17 words of data (34 bytes) but don't forget the extra word
     */
     bcnt = sizeof(rd_statd_s.data_a);
     memset(&rd_statd_s,0,sizeof(rd_statd_s));
     if (!SUCCESS(iss = camio(&rd_ctlw, rd_statd_s.data_a, &bcnt, &rd_statd_s.stat, &emask )))
     {
        printf("CV[%hd %hd %hd]: RW Lines failed to read the ROTATE reg - stat=0x%8.8x  iss=0x%8.8lX\n",
               branch,crate,slot,stat,iss);
        goto egress;
     }

     /* Word swap data from block transfer */
     blockWordSwap(rd_statd_s.data_a,nelem);

     /* Check data for errors */
     nelem = RW_LINE_NUM2;
     status = CV_RWDataGet(type_e, nelem, rd_statd_s.data_a, ldata_a, err_a,expected_data_a);

     printf("RW Line Test #5: %s\tstat=0x%8.8X  iss=0x%8.8lx\n",(status)?"Failed   ":"Successful",rd_statd_s.stat,iss);
     for (i=0; i<RW_LINE_NUM2; i++)
       printf("...(%.2d):  data=0x%8.8lx  expected=0x%8.8lx\t%s\n",
              i,
              ldata_a[i],
              expected_data_a[i],
              (err_a[i])?"Error":"");
     printf("\n");   

     /*---------------------------------------------------------------------------- 
      * Ok, here we are beginnign Test #6, which is the walking zero bit test.
      * of read lines R1-16. We need to read two extra word .The first word,
      * our 17th, is for the walking one bit test will be all ones.
      * The next will be have a zero in R1 and one's in R24-R2.
      * The 18th word is required becasue this is a camac word
      * block transfer which will require a word swap. This means that we need
      * to read an even number of words so that last word of data can be acquired.
      *
      * Now, clear register on the bus by pulsing the C-line.
      */
      if (!SUCCESS(iss = camio(&clr_ctlw, 0 , &nobcnt, &stat, &emask )))
      {
          printf("CV[%hd %hd %hd]: RW Lines failed to clear the ROTATE reg. (pulse C-line). stat=0x%8.8x iss=0x%8.8lX\n",
                  branch,crate,slot,stat,iss);
          goto egress;
      }

      if (!SUCCESS(iss = camio(&wt_ctlw, 0, &nobcnt, &wt_stat, &emask )))
      {
         printf("CV[%hd %hd %hd]: RW Lines failed to set the ROTATE reg for walking zero.  stat=0x%8.8x  iss=0x%8.8lX\n",
                 branch,crate,slot,wt_stat,iss);
         goto egress;
      }
 
      /* Read ROTATE register and rotate pattern left. */
      bcnt = sizeof(rd_statd_s.data_a);
      memset(&rd_statd_s,0,sizeof(rd_statd_s));
      if (!SUCCESS(iss = camio(&rd_ctlw, rd_statd_s.data_a, &bcnt, &rd_statd_s.stat, &emask )))
      {
          printf("CV[%hd %hd %hd]: RW Lines failed to read the ROTATE reg. stat=0x%8.8x  iss=0x%8.8lX\n",
              branch,crate,slot,stat,iss);
          goto egress;
      }

     /* Swap word data crom camac block transfer.*/
     type_e = WALKING_ZERO;
     nelem  = RW_LINE_NUM2+1;
     blockWordSwap(rd_statd_s.data_a,nelem);

     nelem = RW_LINE_NUM2;
     status = CV_RWDataGet(type_e,nelem,rd_statd_s.data_a,ldata_a,err_a,expected_data_a);

     printf("RW Line Test #6: %s\tstat=0x%8.8X  iss=0x%8.8lx\n",(status)?"Failed   ":"Successful",rd_statd_s.stat,iss);
     for (i=0; i<nelem; i++)
       printf("...(%.2d): data=0x%8.8lx  expected=0x%8.8lx\t%s\n",
              i,
              ldata_a[i],
              expected_data_a[i],
              (err_a[i])?"Error":"");
      printf("\n");

     /*----------------------------------------------------------------------------
      * Ok, here we are beginnign Test #7 and #8, which are 
      *
      *       simulated walking one bit test  
      *       simulated  walking zero bit test
      *
      * where the wrote does not use P24 and the read does use P24.
      *
      * Now, clear register on the bus by pulsing the C-line.
      */
     if (!SUCCESS(iss = camio(&clr_ctlw, 0 , &nobcnt, &stat, &emask )))
     {
        printf("CV[%hd %hd %hd]: RW Lines failed to clear the ROTATE reg. (pulse C-lin2) - stat=0x%8.8x iss=0x%8.8lX\n",
                 branch,crate,slot,stat,iss);
        goto egress;
     }

     /* Set the DATA register (P24) clearing out the old data from the high order bytes. */
     bcnt = sizeof(long);
     memset(&wt_data_statd_s,0,sizeof(wt_data_statd_s));
     wt_ctlw = (crate << CCTLW__C_shc) |(slot << CCTLW__M_shc) | F20A0 | CCTLW__P24;
     if (!SUCCESS(iss = camio(&wt_ctlw, &wt_data_statd_s.data, &bcnt, &wt_data_statd_s.stat, &emask )))
     {
         printf("CV[%hd %hd %hd]: RW Lines failed to set the DATA reg (P24) - stat=0x%8.8x  iss=0x%8.8lX\n",
              branch,crate,slot,stat,iss);
         goto egress;
     }

     wt_data_statd_s.data = 0;
     for (j=0; j<RW_LINE_NUM_TYPE; j++)
     {
        /* Clear out local data buffers */
        memset(ldata_a,0,sizeof(ldata_a));
        memset(err_a,0,sizeof(err_a));

        /* Read all of the data */
        for (i=0; i<nelem; i++)
        {
           /* Set the DATA register (P24) clearing out the old data from the high order bytes. */
           wt_data_statd_s.stat = 0;
           bcnt    = sizeof(long);
           wt_ctlw = (crate << CCTLW__C_shc) |(slot << CCTLW__M_shc) | F20A0 | CCTLW__P24;
           if (!SUCCESS(iss = camio(&wt_ctlw, &wt_data_statd_s.data, &bcnt, &wt_data_statd_s.stat, &emask )))
	   {
              printf("CV[%hd %hd %hd]: RW Lines failed to set the DATA reg (P24) - stat=0x%8.8x  iss=0x%8.8lX\n",
                     branch,crate,slot,stat,iss);
              goto egress;
	   }
 
           /* Set the DATA register */
           bcnt    = sizeof(short);
           wt_sdata_statd_s.stat = 0;
           wt_sdata_statd_s.data = rwLineOk_a[j][i];
           wt_ctlw = (crate << CCTLW__C_shc) |(slot << CCTLW__M_shc) | F20A0;
           if (!SUCCESS(iss= camio(&wt_ctlw, &wt_sdata_statd_s.data, &bcnt, &wt_sdata_statd_s.stat, &emask )))
	   {
              printf("CV[%hd %hd %hd]: RW Lines failed to set the DATA reg - stat=0x%8.8x  iss=0x%8.8lX\n",
                     branch,crate,slot,stat,iss);
              goto egress;
	   }
 
           /* Read the DATA register (P24) */
           bcnt     = sizeof(long);
           rd_data_statd_s.stat = 0;
           rd_data_statd_s.data = 0;
           rd_ctlw  = (crate << CCTLW__C_shc) | (slot << CCTLW__M_shc) | F4A0 | CCTLW__P24;
           if(!SUCCESS(iss = camio(&rd_ctlw, &rd_data_statd_s.data, &bcnt, &rd_data_statd_s.stat, &emask )))
	   {
              printf("CV[%hd %hd %hd]: RW Lines failed to read the DATA reg - stat=0x%8.8x  iss=0x%8.8lX\n",
                     branch,crate,slot,stat,iss);
              goto egress;
	   }
           /* Save status and data in separte arrays */
           ldata_a[i] = rd_data_statd_s.data;
	}
        /* 
	 * Check for errors. We want to read all of the data before
	 * we check for errors so that we can print a summary of the
	 * results, (ie. pass or fail).
	 */
        status = CV_RWDataGet(j,nelem,NULL,ldata_a,err_a,expected_data_a);
        printf("RW Line Test #%d: %s\tstat=0x%8.8X  iss=0x%8.8lx\n",
                j+7,(status)?"Failed   ":"Successful",rd_statd_s.stat,iss);  
        for (i=0; i<nelem; i++)
           printf("...(%.2d):  data=0x%8.8lx  expected=0x%8.8lx\t%s\n",
		  i,
                  ldata_a[i],
                  expected_data_a[i],
                  (err_a[i])?"Error":"" );
        printf("\n");

     } /* End of j FOR loop */
     printf("\n");

egress:

    /* Clear the registers on the bus before exiting for clean up. */
     camio(&clr_ctlw, 0 , &nobcnt, &stat, &emask );

   return(iss);
}

/*====================================================
 
  Abs:  Extract the Read Write word data from the Camac Packet

  Name: CV_RWDataGet
 
  Args: type_e                        Type of bit test
          Type: enum                  Note: WALKING_ONE
          Use:  cv_rwLine_type_te           WALKING_ZERO
          Acc:  read-only
          Mech: By value

        nelem                         Number of data elements.
          Type: integer               
          Use:  unsigned int
          Acc:  read-only
          Mech: By value
      
        idata_a                       Input data from Camac packet 
          Type: pointer to array      Note: data is alread word swapped.
          Use:  unsigned short * const
          Acc:  read-write                
          Mech: By reference             

        odata_a                        Output data 
          Type: ptr to array           data moved to an array of 32-bits
          Use:  unsigned long * const  array must have RW_LINE_NUM2 elements
          Acc:  read-write
          Mech: By reference

        err_a                          Error flag array. 
          Type: ptr to array           array must have RW_LINE_NUM2 elements
          Use:  unsigned long * const
          Acc:  read-write
          Mech: By reference

        expected_a                     Expected data array
          Type: ptr to array          array must have RW_LINE_NUM2 elements
          Use:  unsigned long * const 
          Acc:  read-write
          Mech: By reference

  Rem:  The data from the input data buffer is masked and stored in the 
        output data buffer, odata_a. The output data elements are then
        compared to the expected data and the error flag set accordingly 
        in err_a.

  Side: The data in the idata_a array has already been word swapped
        if necessary.
  
  Ret:  unsigned long
            OK    - Successful, data is valid
            ERROR - Read Write data error         
            
=======================================================*/
unsigned long CV_RWDataGet(cv_rwLine_type_te      type_e, 
                           unsigned int           nelem,
                           unsigned short * const idata_a,
                           unsigned long  * const odata_a,
                           unsigned long  * const err_a,
                           unsigned long  * const expected_data_a )
{
     RW_LINE_OK;
     unsigned long    status = OK;
     unsigned short   i      = 0;
     unsigned long    mask   = 0x0000ffff;
     unsigned long    ldata  = 0;   
     unsigned long    bcnt   = nelem * sizeof(unsigned long);


    /* Clear output data and error flag array */ 
    memset(err_a,0,bcnt);
    memset(expected_data_a,0,bcnt);  
   
   /* 
    * If an input data array is specified then we want to 
    * word swap the data and
    */
    switch(nelem)
    {
       case RW_LINE_NUM2:
         if ( idata_a ) 
         {
            memset(odata_a,0,bcnt);
            for (i=0; i<nelem; i++)
            {
               /* Get expected data */
               expected_data_a[i]= rwLineOk_a[type_e][i];
               if (type_e==WALKING_ZERO) expected_data_a[i] &=mask;

              /* 
	       * Mask off the data and check if it's what we expect.
	       * If not, the flag an error
	       * and set the return status accordingly
               */
               ldata = (unsigned long)idata_a[i];
               ldata  &= mask;
               odata_a[i] = ldata;
               if (odata_a[i] != expected_data_a[i])
	       {
                  err_a[i] = 1;
                 status = ERROR;
               }
            }/* End of FOR loop */
         }
         else
         {
            /* No input buffer, so let's check the data in the output buffer */
            for (i=0; i<nelem; i++)
            {
                 /* Get expected data  */
                 expected_data_a[i]= rwLineOk_a[type_e][i];
                 if (type_e==WALKING_ZERO) expected_data_a[i] &=mask;

                /* 
		 * Mask off the data and check if it's what we expect.
		 * If not, the flag an error
		 * and set the return status accordingly
                 */
                odata_a[i] &= mask;
                if (odata_a[i] != expected_data_a[i])
                {
                   err_a[i] = 1;
                   status = ERROR;
	        }
            }/* End of FOR loop */
         }       
         break;

       case RW_LINE_NUM:
         /* No input buffer so check the output buffer data */
         for (i=0; i<nelem; i++)
         {
            /* Get expected data  */
            expected_data_a[i]= rwLineOk_a[type_e][i];
 
           /* 
	    * Mask off the data and check if it's what we expect.
      	    * If not, the flag an error
	    * and set the return status accordingly
            */
            odata_a[i] &= RW_LINE_MASK;
            if (odata_a[i] != expected_data_a[i])
            {
              err_a[i] = 1;
              status = ERROR;
            }
	 }           
         break;

       default:
         status = ERROR;
         break;

    }  /* End of switch statement */

    return(status);
}


/*====================================================
 
  Abs:  Send a Camac function request message to the queue
 
  Name: CV_SendMsg
 
  Args: branch                       Camac Branch
          Type: integer              Note: ignored
          Use:  short
          Acc:  read-only
          Mech: By value

        crate                        Camac Crate Number
          Type: integer             
          Use:  short
          Acc:  read-only
          Mech: By value

        slot                         Camac Crate Slot Number
          Type: integer           
          Use:  short
          Acc:  read-only
          Mech: By value

        func_e                       Camac function request code
          Type: enum           
          Use:  cv_camac_func_te
          Acc:  read-only
          Mech: By value


  Rem:  The purpose of this function is to send a camac
        function request to the message queue. 

  Side: None
  
  Ret:  long
            OK - Successfully completed
            Otherwise, failure due to
               No module found
               see return code from epicsMessageQueueTrySend()      
            
=======================================================*/
long CV_SendMsg(short branch, short crate, short slot, cv_camac_func_te func_e)
{
     long        status    = ERROR;
     CV_MODULE  *module_ps = NULL;
     CV_REQUEST  msg_s;


     module_ps = CV_FindModuleByBCN(branch,crate,slot);
     if( !module_ps )
        printf("No module found\n");
     else
     { 
       /* Is this module already in the list?
        * If yes, return with the address.
        * If not, proceed with allocating the memory for
        * the structure.
        */
       status = CV_DeviceInit( func_e,"TEST",NULL, module_ps, &msg_s ); 
       if (!status && module_ps->msgQId_ps)
       {
          CV_ClrMsgStatus( msg_s.mstat_ps );
          status = epicsMessageQueueTrySend(module_ps->msgQId_ps,&msg_s,sizeof(CV_REQUEST));
       }
    }
    return(status);
}

/*====================================================
 
  Abs:  Send a request message to read the id register
 
  Name: CV_RdIdMsg
 
  Args: branch                       Camac Branch
          Type: integer              Note: ignored
          Use:  short
          Acc:  read-only
          Mech: By value

        crate                        Camac Crate Number
          Type: integer             
          Use:  short
          Acc:  read-only
          Mech: By value

        slot                         Camac Crate Slot Number
          Type: integer           
          Use:  short
          Acc:  read-only
          Mech: By value


  Rem:  The purpose of this function is to send a camac
        function request to the message queue to read
        the id register. 

  Side: None
  
  Ret:  long
            OK        - Successfully completed
            Otherwise -  Operations failed.
               See return error returns from:
                  CV_SendMsg()     
            
=======================================================*/
long CV_RdIdMsg(short b,short c,short n)
{ 
   return(CV_SendMsg(b,c,n,CAMAC_RD_ID));
} 


/*====================================================
 
  Abs:  Send a request message to read the analog register
 
  Name: CV_RdVoltageMsg
 
  Args: branch                       Camac Branch
          Type: integer              Note: ignored
          Use:  short
          Acc:  read-only
          Mech: By value

        crate                        Camac Crate Number
          Type: integer             
          Use:  short
          Acc:  read-only
          Mech: By value

        slot                         Camac Crate Slot Number
          Type: integer           
          Use:  short
          Acc:  read-only
          Mech: By value


  Rem:  The purpose of this function is to send a camac
        function request to the message queue to read
        the analog register. 

  Side: None
  
  Ret:  long
            OK        - Successfully completed
            Otherwise -  Operations failed.
               See return error returns from:
                  CV_SendMsg()     
            
=======================================================*/
long CV_RdVoltageMsg(short b,short c,short n)  
{ 
  return(CV_SendMsg(b,c,n,CAMAC_RD_VOLTS));
}


/*====================================================
 
  Abs:  Send a request message to read the data register
 
  Name: CV_RdDataMsg
 
  Args: branch                       Camac Branch
          Type: integer              Note: ignored
          Use:  short
          Acc:  read-only
          Mech: By value

        crate                        Camac Crate Number
          Type: integer             
          Use:  short
          Acc:  read-only
          Mech: By value

        slot                         Camac Crate Slot Number
          Type: integer           
          Use:  short
          Acc:  read-only
          Mech: By value


  Rem:  The purpose of this function is to send a camac
        function request to the message queue to read
        the data register. This register should contain
        a data pattern written during boot.

  Side: None
  
  Ret:  long
            OK        - Successfully completed
            Otherwise -  Operations failed.
               See return error returns from:
                  CV_SendMsg()     
            
=======================================================*/
long CV_RdDataMsg(short b,short c,short n)
{ 
  return(CV_SendMsg(b,c,n,CAMAC_RD_DATA));
}


/*====================================================
 
  Name: CV_RdCrateStatusMsg
 
  Args: branch                       Camac Branch
          Type: integer              Note: ignored
          Use:  short
          Acc:  read-only
          Mech: By value

        crate                        Camac Crate Number
          Type: integer             
          Use:  short
          Acc:  read-only
          Mech: By value

        slot                         Camac Crate Slot Number
          Type: integer           
          Use:  short
          Acc:  read-only
          Mech: By value

  Rem:  The purpose of this function is to send a camac
        function request to the message queue to read
        the data register, which should contain a
        pattern written to this register during startup.

  Side: None
  
  Ret:  long
            OK        - Successfully completed
            Otherwise -  Operations failed.
               See return error returns from:
                  CV_SendMsg()     
            
=======================================================*/
long CV_RdCrateStatusMsg(short b,short c,short n)
{ 
  return(CV_SendMsg(b,c,n,CAMAC_RD_CRATE_STATUS)); 
}


/*====================================================
 
  Abs:  Send a request message to set the data register
 
  Name: CV_WtDataMsg
 
  Args: branch                       Camac Branch
          Type: integer              Note: ignored
          Use:  short
          Acc:  read-only
          Mech: By value

        crate                        Camac Crate Number
          Type: integer             
          Use:  short
          Acc:  read-only
          Mech: By value

        slot                         Camac Crate Slot Number
          Type: integer           
          Use:  short
          Acc:  read-only
          Mech: By value

  Rem:  The purpose of this function is to send a camac
        function request to the message queue to write
        the default pattern to the data register.

  Side: None
  
  Ret:  long
            OK        - Successfully completed
            Otherwise -  Operations failed.
               See return error returns from:
                  CV_SendMsg()     
            
=======================================================*/
long CV_WtDataMsg(short b,short c,short n)  
{ 
  return(CV_SendMsg(b,c,n,CAMAC_WT_DATA));
}


/*====================================================
 
  Abs:  Word swap data in the buffer provided. 
 
  Name: CV_blockWordSwap
 
  Args: buf_p                      Pointer to a data array
          Type: integer             
          Use:  void * const
          Acc:  read-write
          Mech: By reference

        num_words                   Number of words in the array
          Type: integer             
          Use:  unsigned int
          Acc:  read-only
          Mech: By value

  Rem:  The purpose of this function is to swap the words
        in the array provided.  If numwords is odd we don't 
        swap the last word

  Side: Code copied from Bob Sass's blockPIOP.c
  
  Ret:  void    
            
=======================================================*/
void blockWordSwap(void * const buf_p, unsigned int num_words)
{
   unsigned short *data_p = buf_p;
   unsigned short  temp;            /* Temporary data for swapping */
   unsigned int    i;               /* index counter                */


   for (i=0; i<num_words; i+=2)
   {
       temp       = *data_p;      /* lower word data                       */
       *data_p    = *(data_p+1);  /* stuff upper word data into lower word */
      *(data_p+1) = temp;         /* stuff lower word data into upper word */
      data_p      += 2;           /* got the next word                      */
   }
   return;
}


epicsRegisterFunction(CV_WtData);
epicsRegisterFunction(CV_RdData);
epicsRegisterFunction(CV_RdId);
epicsRegisterFunction(CV_RdVoltage);
epicsRegisterFunction(CV_RW);
epicsRegisterFunction(CV_RW2);

epicsRegisterFunction(blockWordSwap);
epicsRegisterFunction(CV_RWDataGet);

epicsRegisterFunction(CV_SendMsg);
epicsRegisterFunction(CV_RdDataMsg);
epicsRegisterFunction(CV_RdIdMsg);
epicsRegisterFunction(CV_RdVoltageMsg);
epicsRegisterFunction(CV_WtDataMsg);
epicsRegisterFunction(CV_RdCrateStatusMsg);

/* End of file */
