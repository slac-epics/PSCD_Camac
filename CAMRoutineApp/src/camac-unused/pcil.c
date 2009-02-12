/*=============================================================================

  Name: pcil.c

  Abs:  All driver functions and support routines for PCIL-FECC real-time front-end CAMAC 
        (and others) I/O subsystem.
  
  Auth: 11-Apr-2006, Robert C. Sass (RCS) rewritten from RMX driver by 
                     Eric Siskind (EJS)
  Rev:  dd-mmm-200y,
-----------------------------------------------------------------------------*/
/**???? Uncomment when in standard make  #include "copyright_SLAC.h"  */
/*-----------------------------------------------------------------------------
  Mod:  (newest to oldest)
        DD-MMM-YYYY, My Name:
           Changed such and such to so and so. etc. etc.
        DD-MMM-YYYY, Your Name:
           More changes ... The ordering of the revision history
           should be such that the NEWEST changes are at the HEAD of
           the list.

=============================================================================*/
/*
** These defines are necessary for Linux Kernel modules.
*/

#define __KERNEL__
#define MODULE

/*
** Macro to turn conditionally print debugging info.
*/

#ifdef PCIL_DEBUG
#  define PDEBUG(fmt, args...) printk (KERN_DEBUG "PCIL: " fmt, ##args)
#else
#  define PDEBUG(fmt, args...)  /* No debug messages */
#endif

/*
** Macro to execute the mfence instruction to complete all memory reads/writes
** before advancing the IP.
*/
#define MFENCE asm("mfence");


/*
** We use a static major device number.
*/
#define PCIL_MAJOR 253

/*
** Les includes
*/
#include <linux/modversions.h>
#include <linux/config.h>

#include <linux/module.h>      /* All loadable modules need this */
#include <linux/errno.h>       /* Linux error codes */
#include <linux/pci.h>         /* pci_dev device block and pci protos */
#include <linux/mm.h>          /* kmalloc etc. */
#include <linux/sched.h>       /* interrupt handler etc.*/
#include <linux/fs.h>          /* file ops etc.*/
#include <linux/init.h>        /* module_init, module_exit */
#include <linux/ioctl.h>       /* command bitfields etc. */
#include <linux/string.h>      /* memset */
#include <asm/uaccess.h>       /* copy_to/from_user etc. */
#include <asm/page.h>          /* __pa() to get physical from logical address */
#include <asm/pgtable.h>       /* Page table struct etc. */
#include <asm/semaphore.h>     /* semaphores */

#include "slc_macros.hc"       /* int2u BOOL etc. */
#include "pcil_hw.h"           /* Hardware port offsets, masks etc. */
#include "pcil_usr.h"          /* Structs and defs for application code */

MODULE_AUTHOR ("Robert C. Sass");
MODULE_DESCRIPTION ("PCIL driver to support PCIL2/FECC3");

/* 
** This was taken from the scsi 3w.xxx driver and is what we need for a 
** wait with timeout which seems to be sorely lacking in 2.4. 
*/
#define pcil_wait_event_interruptible_timeout(wq, condition, ret)         \
do {                                                                    \
        wait_queue_t __wait;                                            \
        init_waitqueue_entry(&__wait, current);                         \
                                                                        \
        add_wait_queue(&wq, &__wait);                                   \
        for (;;) {                                                      \
                set_current_state(TASK_INTERRUPTIBLE);                  \
                if (condition)                                          \
                        break;                                          \
                if (!signal_pending(current)) {                         \
                        ret = schedule_timeout(ret);                    \
                        if (!ret)                                       \
                                break;                                  \
                        continue;                                       \
                }                                                       \
                ret = -ERESTARTSYS;                                     \
                break;                                                  \
        }                                                               \
        current->state = TASK_RUNNING;                                  \
        remove_wait_queue(&wq, &__wait);                                \
} while (0)
      
/*
** Local typedefs
*/

typedef struct                       /* communication area */
{
   u32    response_ring_low_a[RESPONSE_RING_LENGTH]; /* low priority response ring */
   u32    response_ring_high_a[RESPONSE_RING_LENGTH];/* high priority response ring */
   u32    command_ring_low_a[COMMAND_RING_LENGTH];   /* low priority command ring */
   u32    command_ring_high_a[COMMAND_RING_LENGTH];  /* high priority command ring */
} commarea_ts;

typedef enum                         /* PCIL function                       */
{
   MBCD_EMULATE = 0,                 /* emulate MBCD operation              */
   SEND_TO_FECC = 1,                 /* send message to FECC user code      */
   SEND_TO_PC   = 2,                 /* send message from FECC user code    */
   REMOTE_IO    = 3,                 /* FECC remote PC buffer access        */
   DIAG_PCIL    = 4,                 /* PCIL diagnostic/maintenance request */
   DIAG_FECC    = 5,                 /* FECC diagnostic/maintenance request */
   BCOM_EMULATE = 6,                 /* emulate bitbus master operation     */
   ERROR_REPORT = 7                  /* error detected report               */
} function_e;

typedef enum                         /* current ring location               */
{
   COMMAND_RING  = 0,                /* on command ring                     */
   RESPONSE_RING = 1                 /* on response ring                    */
} ring_e;

typedef union                        /* function code in pieces             */
{
   u32            funct_code;        /* as bit-masked field                 */
   function_e     opcode;            /* as an operation code enumeration    */
} function_u;


typedef struct                       /* A wait queue with condition */
{
   wait_queue_head_t wq;
   int               cond;
} wqcond_ts;

typedef struct request_s             /* request block                  */
{
   struct request_s *link_p;         /* pointer to next request block     */
   wqcond_ts         wqcond_s;       /* wait queue and condition from ISR */
   function_u        function;       /* function code                     */
   u32               fwstatus;       /* overall execution status          */
   u32               pci_size;       /* pci buffer length (bytes)         */
   u32               pci_addr;       /* pci buffer address                */
   u32               msg_size;       /* message buffer length (bytes)     */
   void             *msg_p;          /* message buffer pointer            */
} request_ts;

typedef struct                       /* request queue header                 */
{
   request_ts    *head_pointer_p;    /* pointer to first request block       */
   request_ts   **tail_pointer_pp;   /* pointer to link_p of last request    */
} requestqueue_ts;

typedef struct                       /* response id                          */
{
   int2u          index;             /* index into array of response id's    */
   int2u          use_count;         /* cumulative number of uses            */
} rspid_ts;

typedef struct flow_s                /* flow block                           */
{
   struct flow_s *link_p;            /* pointer to next flow block           */
   request_ts    *request_ps;        /* pointer to request_ts                */
   rspid_ts       rspid_s;           /* response id                          */
} flow_ts;

typedef struct                       /* flow queue header                    */
{
   flow_ts       *head_pointer_p;    /* pointer to first flow block          */
   flow_ts      **tail_pointer_pp;   /* pointer to link_p of last flow block */
} flowqueue_ts;

typedef struct message_s             /* message block                        */
{
   struct message_s *link_p;         /* pointer to next message block        */
   int1u             buff_no;        /* buffer number                        */
   int1u             ring_index;     /* ring index                           */
   ring_e            ring_type;      /* current ring type                    */
   u32               ring_element;   /* buffer descriptor for ring insertion */
   u32               xfer_size;      /* overall buffer length (bytes)        */
   function_u        function;       /* function code                        */
   rspid_ts          rspid_s;        /* response id                          */
   u32               fwstatus;       /* overall execution status             */
   u32               pci_size;       /* pci buffer length (bytes)            */
   u32               pci_offset;     /* pci buffer offset (bytes)            */
   u32               pci_addr;       /* pci buffer address                   */
   u32               msg_size;       /* message buffer length (bytes)        */
   u32               message[MESSAGE_LENGTH];
                                     /* message buffer                       */
} message_ts;

typedef struct                       /* message queue header                 */
{
   message_ts    *head_pointer_p;    /* pointer to first message block       */
   message_ts   **tail_pointer_pp;   /* pointer to link_p of last message    */
} messagequeue_ts;

typedef struct                               /* Device data block              */
{
   struct semaphore request_region;          /* request queue mutex            */
   spinlock_t        message_region;         /* message queue spinlock         */
   struct semaphore message_semaphore;       /* message wait semaphore         */
   struct semaphore flow_semaphore;          /* flow wait semaphore            */
   struct semaphore synch_semaphore;         /* synchronization semaphore      */
   struct semaphore camac_semaphore;         /* CAMAC allocation semaphore     */
   struct semaphore bitbus_semaphore;        /* BITbus allocation semaphore    */
   int1u            message_wait_count;      /* requests awaiting message_ts   */
   int1u            flow_wait_count;         /* messages awaiting flow_ts      */
   int1u            command_ring_index;      /* next command slot              */
   int1u            command_ring_poll;       /* next command poll slot         */
   int1u            response_ring_index;     /* next response slot             */
   int1u            response_ring_poll;      /* next response poll slot        */
   int1u            command_ring_count;      /* number of active commands      */
   int1u            response_ring_count;     /* number of active responses     */
   requestqueue_ts  request_queue_s;         /* free request block queue       */
   flowqueue_ts     flow_queue_s;            /* free flow block queue          */
   messagequeue_ts  message_queue_s;         /* free message block queue       */
   messagequeue_ts  send_queue_s;            /* send message queue             */
   u32             *doorbell_register_p;     /* pointer to doorbell register   */
   u32             *command_ring_p;          /* pointer to command ring        */
   u32             *response_ring_p;         /* pointer to response ring       */
   flow_ts         *flow_array_ps;           /* pointer to flow_ts array       */
   message_ts      *command_contents_ps[COMMAND_RING_LENGTH];
                                             /* pointers to command ring messages   */
   message_ts      *response_contents_ps[RESPONSE_RING_LENGTH];
                                            /* pointers to response ring messages  */
} deviceblock_ts;

typedef struct                              /* PCIL hardware registers             */
{
   u32             doorbell_low;            /* low priority doorbell               */
   u32             unused0;                 /* (unused)                            */
   u32             acknowledge_low;         /* low priority interrupt acknowledge  */
   u32             unused1;                 /* (unused)                            */
   u32             doorbell_high;           /* high priority doorbell              */
   u32             unused2;                 /* (unused)                            */
   u32             acknowledge_high;        /* high priority interrupt acknowledge */
   u32             unused3;                 /* (unused)                            */
   u32             doorbell_pattern;        /* pattern received doorbell           */
   u32             unused4;                 /* (unused)                            */
   u32             acknowledge_pattern;     /* pattern rcvd interrupt acknowledge  */
   u32             unused5[5];              /* (unused)                            */
   u32             boot_control;            /* boot initiation and address         */
   u32             unused6;                 /* (unused)                            */
   u32             commun_init_low;         /* communications initialization low   */
   u32             commun_init_high;        /* communications initialization high  */
   u32             unused7[4];              /* (unused)                            */
   u32             code_buffer[4];          /* PCIL code buffer descriptor         */
   u32             unused8[4];              /* (unused)                            */
   u32             pattern[2*PATTERN_LENGTH];
                                            /* trigger pattern array               */
} registers2_ts;

/*
** Minor device numbers to discriminate the different write & read functions. 
** These match up with the device name files followed by pciln created by mknod.
** Devices 1-7 are runtine application functions. Devices 8-22 are diagnostic.
*/

typedef enum
{
   /*
   ** Runtime functions
   */
   MBCDLO =  1,     /* PCILnMBCDLO Write low priority MBCD message */
   MBCDHI =  2,     /* PCILnMBCDHI Write hi priority MBCD message */
   BBUSLO =  3,     /* PCILnBBUSLO Write low priority BITbus message */
   BBUSHI =  4,     /* PCILnBBUSHI Write hi priority BITbus message */
   BBUSRD =  5,     /* PCILnBBUSRD Read BITbus message as hi priority */
   AMFELO =  6,     /* PCILnA2FELO Write/Read low priority application msg to/from FECC */
   AMFEHI =  7,     /* PCILnA2FEHI Write/Read high priority application msg to/from FECC */
   SNDPAT =  8,     /* PCILnSNDPAT Write (Send) pattern */
   /*
   ** Diagnostic functions
   */
   GFGCNT =  9,     /* PCILnGETCNT Read (get) fine granularity timer count from PCIL */
   LDFECR = 10,     /* PCILnLDFECR Write (load) FECC command register */
   LDLLIR = 11,     /* PCILnLDLLIR Write (load) local link initialization register */
   LDRLIR = 12,     /* PCILnLDRLIR Write (load) remote link initialization register */
   DPCMEM = 13,     /* PCILnDPCMEM Read (dump) PCIL memory */
   GPCVER = 14,     /* PCILnGPCVER Read (get) PCIL hardware/software version */
   GPCCPU = 15,     /* PCILnGPCCPU Read (get) PCIL CPU use statistics */
   GPCLMC = 16,     /* PCILnGPCLMC Read (get) PCIL2 link maintenance counters */
   DFEMEM = 17,     /* PCILnDFEMEM Read (dump) FECC memory */
   LFEMEM = 18,     /* PCILnLFEMEM Write (load) FECC memory */
   GFEVER = 19,     /* PCILnGFEVER Read (get) FECC hardware/software version */
   GFECPU = 20,     /* PCILnGFECPU Read (get) FECC CPU use statistics */
   GFELMC = 21,     /* PCILnGFELMC Read (get) FECC3 link maintenance counters */
   LFECCM = 22,     /* PCILnLFECMP Write (load) FECC CAMAC crate map */
   LFECSR = 23,     /* PCILnLFECSR Write (load) FECC3 CAMAC standard register */
} mdev_te;

/*
** Interrupt handler & tasklets for high and low priority.
*/

static void pcil_isr (int irq, void *devp, struct pt_regs *regs);
void PCIL_IO_Receive_High (unsigned long);
void PCIL_IO_Receive_Low (unsigned long);
DECLARE_TASKLET (TaskletHigh, PCIL_IO_Receive_High, 0);
DECLARE_TASKLET (TaskletLow,  PCIL_IO_Receive_Low, 0);

/*
** Static variables
*/

DECLARE_WAIT_QUEUE_HEAD (sleep_queue);  /* Dummy queue for fixed sleeps. */
DECLARE_WAIT_QUEUE_HEAD (init_queue);   /* Used just for initialization signal from ISR. */
static int init_wait = 0;               /* To Tell ISR we're waiting on init complete */

static commarea_ts    *PCIL_IO_Comm_Area_ps = NULL;  /* Communication area pointer         */
static registers2_ts  *PCIL_IO_Registers2_ps = NULL; /* PCIL2 hardware registers pointer   */
static deviceblock_ts *PCIL_IO_Device_Block_Low_ps = NULL;  /* Low priority device block pointer  */
static deviceblock_ts *PCIL_IO_Device_Block_High_ps = NULL; /* High priority device block pointer */

/*
** Registers used to communicate with the PCIL.
*/
static u32     *PCIL_IO_Doorbell_Low_p = NULL;       /* Low priority doorbell pointer       */
static u32     *PCIL_IO_Doorbell_High_p = NULL;      /* High priority doorbell pointer      */
static u32     *PCIL_IO_Doorbell_Pattern_p = NULL;   /* Pattern forwarding doorbell pointer */
static u32     *PCIL_IO_Interrupt_Type_p = NULL;     /* Interrupt type register pointer     */
static u32     *PCIL_IO_Acknowledge_Low_p = NULL;    /* Low priority acknowledge pointer    */
static u32     *PCIL_IO_Acknowledge_High_p = NULL;   /* High priority acknowledge pointer   */
static u32     *PCIL_IO_Boot_Control_p = NULL;       /* Boot initiation and address pointer */
static u32     *PCIL_IO_Commun_Init_Low_p = NULL;    /* Communications init low pointer     */
static u32     *PCIL_IO_Commun_Init_High_p = NULL;   /* Communications init high pointer    */
static u32     *PCIL_IO_Code_Buffer_ap = NULL;       /* PCIL code buffer descriptor pointer */
static u32     *PCIL_IO_Pattern_ap = NULL;           /* Trigger pattern array pointer       */
static u32     *PCIL_IO_Counter_p = NULL;            /* PCI cycle counter register pointer  */
static u32      PCIL_IO_Comm_Area_a = 0;             /* Linear address of comm area         */
static u32      PCIL_IO_PCIL_Device_Version = 0;     /* Device hardware/software version    */
u32             PCIL_IO_Pattern_Length = 0;          /* Trigger pattern array length        */

/*
***********************************************************************
** Psuedo Unibus (PU) virtual memory map pagetable pointers to map 
** non-contiguous data areas for DMA by PCIL.
** Number of pages to allocate in power of 2 pages PTEPWR
** e.g. 3 = 2^4 = 16 pages. PTEMAX is # of u32   entries allocated.
** Struct to manage the map.
***********************************************************************
*/
#define PTEPWR 4
#define PTEMAX ( ((1 << PTEPWR)*PAGE_SIZE)/sizeof(u32  ) ) 
/*
**Get a page table entry for a giver user virtual address.
*/
#define GETPTE(pte, virt) \
         pte = pte_offset_map (pmd_offset (pgd_offset (current->mm, virt), virt),virt)

typedef struct
{
  u32              *pu_ps;  /* An array of Psuedo Unibus page table entries for the PCIL */
  u32               lux;    /* Index to last used entry */
  struct semaphore  pusem;  /* For access to lux & pu_ps */
} vmap_ts;

static vmap_ts vmap_s;  /* struct for PCIL virtual memory map. */

/*
** Struct for array and index to keep track of all wait queues allocated in ioctl function 
** IOCTL_PCIL_NEWFECCMBX. These are freed in ioctl function IOCTL_PCIL_INIT.
*/
#define WAITMAX 100
typedef struct
{
  wqcond_ts        **wqcond_paps; /* Array of pointers to wait queue structs for the user */
  u32                wux;         /* Index to next available entry */
  struct semaphore   semq;        /* For access to wux & wqcond_paps */
} uwait_ts;

static uwait_ts uwait_s;

/*
** Local static flags
*/
static BOOL     PCIL_Load_OK = FALSE_F; /* We got through pcil_init called at driver load */
static BOOL     PCIL_Init_Complete = FALSE_F;  /* IOCTL_PCIL_INIT function successful */

/*
** PCI device info.
*/
struct pci_dev *pcil_device_ps = NULL;  /* PCI device block pointer */
int             dev_id = 1;             /* Dummy device id for now. Will be pci index later */

/***************************************************************************
** Local support routines
***************************************************************************/

/*=============================================================================

  Abs:  Check if virtual address + length is contiguous in physical memory.

  Name: PCIL_Is_Contiguous

  Args:
       *pcil_addr_ps
         Use:  Struct to test virtual address
         Type: pcil_addr_ts*
         Acc:  read
         Mech: by reference

  Rem:  Loop through # pages checking that they are PAGE_SIZE apart in physical
        memory.

  Side: None.

  Ret:  BOOL

==============================================================================*/

BOOL PCIL_Is_Contiguous (pcil_addr_ts *addr_ps)
{
   BOOL   stat = TRUE_T;                 /* Assume is contiguous */
   u32    cvirt = (int) addr_ps->virtaddr_p; /* Current virtual address */
   int    npages = ((cvirt & ~PAGE_MASK) + addr_ps->len) >> PAGE_SHIFT;
   pte_t *mypte_p;                       /* Page table entry */
   int    i = 0;                         /* Loop index */
   u32    curpphys, nxtpphys;            /* Current and next page physical addr */
   /*---------------------------------------------------------------------------*/
   /*
   ** If we're within one page then npages = 0 and we return TRUE_T immediately.
   ** Otherwise we have to check that all pages are contiguous. Note that 
   ** npages = 1 means that we check 2 pages; current and next etc.
   */
   GETPTE (mypte_p, cvirt);
   curpphys = (mypte_p->pte_low & PAGE_MASK);
   while (i++ < npages)
   {
      cvirt += PAGE_SIZE;  /* Next virtual page */
      GETPTE (mypte_p, cvirt);
      nxtpphys = (mypte_p->pte_low & PAGE_MASK);
      if (nxtpphys  != (curpphys + PAGE_SIZE)) /* check if physically contiguous */
      {
	 stat = FALSE_F;
	 PDEBUG ("Non-contiguous pages curpphys = %x nxtpphys = %x\n", curpphys, 
                 nxtpphys);
         break;
      }
      curpphys = nxtpphys;
   }
   return stat;
}

/*=============================================================================

  Abs:   Find free PU mapping entries.

  Name:  PCIL_GetFreePUMap

  Args:
        npages
         Use:  number of entries to get
         Type: int
         Acc:  read
         Mech: by value

       *mapx_p
         Use:  Where to return map index found
         Type: int
         Acc:  write
         Mech: by reference

  Rem:  Find npages free pages in our Psuedo-Unibus (PU) mapping table.

  Side: Update last used mapping index.

  Ret:  BOOL

==============================================================================*/

BOOL PCIL_GetFreePUMap (int npages, int *mapx_p)
{
   int  stat = npages;     /* Assume we'll find that space */
   int  ci;                /* Current index */
   int  li;                /* Last index */
   int  npi;               /* number of pages index */
   BOOL found = FALSE_F;   /* Found required free pages */
   BOOL wrap  = FALSE_F;   /* We wrapped around the array */
   /*---------------------------------------------------------------------------*/
   if (down_interruptible (&vmap_s.pusem))   /* Get access to vmap_s */
   {
      printk (KERN_ERR "PCIL: Signal interrupt in PCIL_GetGreePUMap\n");
      stat = -ERESTARTSYS;
      goto egress;
   }
   li = vmap_s.lux; /* last used index */
   ci = li + 1;     /* current index; this means we don't start using entry 0. BFD. */ 
   while (!found && !wrap)  /* Exit if found || wrap */
   {
      /*
      ** Find npages contiguous free (==0) slots. Check for end of pu_ps and wrap
      */
      for (npi=0; npi < npages; npi++)
      {
         if (ci >= PTEMAX)
	 {
	    ci = 0; /* Start over */
            break;
	 }
         if (ci == li)
         {
	    wrap = TRUE_T;
	    stat = -ENOSPC;
            printk (KERN_ERR "PCIL: Psuedo-Unibus map table full finding %d entries\n",
                    npages);
	    break;
         } 
	 if (vmap_s.pu_ps[ci++] != 0)
	    break;
      }
      if (npi >= npages)
	found = TRUE_T;
   }  /* while (!found && !wrap) */
   if (found)
   {
      vmap_s.lux = ci - 1;    /* Point to last used index */
      *mapx_p = ci - npages;  /* Index of first allocated page */
      PDEBUG ("In GetFreePUMap found virtual map index %d pages %d\n",
              *mapx_p, npages);
   }
   up (&vmap_s.pusem);  /* Free the semaphore */    
egress:
   return stat;
}

/*=============================================================================

  Abs:  Get physical address and optionally map it for PCIL.

  Name: PCIL_GetAddr

  Args:
        *pcil_addr_ps
         Use:  Struct go return mapping info
         Type: pcil_addr_ts*
         Acc:  write
         Mech: by reference
 
  Rem:  If request is physically contiguous enter the physical address.
        Otherwise allocate and fill in the PU mapping registers and enter
        the PU virtual address of the first entry.  

  Side: PU map registers allocated and filled in.

  Ret:  status

==============================================================================*/
int PCIL_GetAddr (pcil_addr_ts *addr_ps)
{
   pte_t *mypte_p;                                   /* Page table entry */
   u32    virtaddr = (u32 ) addr_ps->virtaddr_p;    /* User's virtual address */
   int    stat = 0;                                  /* Status */
   int    mapx = -1;                                 /* Index to map entry */
   /*---------------------------------------------------------------------------*/
   /* 
   ** If request is physically contiguous then we can just return the physical address.
   */
   if (PCIL_Is_Contiguous (addr_ps))
   {
      GETPTE (mypte_p, virtaddr);
      addr_ps->physaddr =  (mypte_p->pte_low & PAGE_MASK) | (virtaddr & ~PAGE_MASK);
      /*
      ** No mapping index or registers allocated.
      */
      addr_ps->mapn = 0;
   }
   else
   {
      /*
      ** Request is not physically contiguous. Map # pages required and return 
      ** psuedo-Unibus (PU) virtual address).
      */
      int npages = (((virtaddr & ~PAGE_MASK) + addr_ps->len) >> PAGE_SHIFT) + 1;
      if ( (stat = PCIL_GetFreePUMap (npages, &mapx)) < 0)
         goto egress;
      /*
      ** Now map the PU entries.
      */
      addr_ps->mapn = npages;  /* Save # entries in user's struct */
      int p;     /* PU loop index */
      int puidx; /* PU index */ 
      for (p=0, puidx=mapx; p < npages; p++)
      {
	 GETPTE (mypte_p, virtaddr);  /* Get next page table entry */
	 PDEBUG ("Adding pte %lx at index %d\n",mypte_p->pte_low, puidx);
         vmap_s.pu_ps[puidx++] = mypte_p->pte_low; 
         virtaddr += PAGE_SIZE;       /* Next virtual page */        
      }
      /*
      ** Construct PU virtual address of first PU entry as the physical address.
      */
      addr_ps->physaddr = 0x80000000 | (mapx  << PAGE_SHIFT) | 
                                       ((u32  )addr_ps->virtaddr_p & ~PAGE_MASK);
   }
egress:
   PDEBUG ("In GetAddr address returned = %x mapx = %d mapn = %d\n",
           addr_ps->physaddr, mapx, addr_ps->mapn);
   return (stat);
}


/*=============================================================================

  Abs:  Release virtual address.

  Name: PCIL_RelAddr

  Args:
        *pcil_addr_ps
         Use:  Struct go get/return virtual/physical addresses
         Type: pcil_addr_ts*
         Acc:  read/write
         Mech: by reference

  Rem:  If this is a PU virtual address then release the entries.

  Side: Map entries freed (zeroed).

  Ret:  -ERESTARTSYS if signal waiting for semaphore.

==============================================================================*/

int PCIL_RelAddr (pcil_addr_ts * addr_ps)
{
   int   stat = 0;    /* Assume all OK */
   int   i;           /* Loop index */
   int   mx;          /* Map index */
   /*---------------------------------------------------------------------------*/
   /*
   ** If address was physical there are no map registers to be released so we're done.
   */
   if ( ((u32  )addr_ps->physaddr & 0x80000000) == 0) goto egress;
   /*
   ** Get the semaphore and release the entries.
   */
   if (down_interruptible (&vmap_s.pusem))   /* Get vmap_s semaphore */
   {
      printk (KERN_ERR "PCIL: Signal interrupt in PCIL_RelAddr.\n");
      stat = -ERESTARTSYS;
      goto egress;
   }
   mx = ((u32  )addr_ps->physaddr & 0x7FFFFFFF) >> PAGE_SHIFT; /* Map table index */
   for (i=0; i < addr_ps->mapn; i++)
      vmap_s.pu_ps[mx++]=0;  /* Free the register */
   PDEBUG ("In RelAddr freed %d entries starting at %d.\n",addr_ps->mapn, 
            mx - addr_ps->mapn);
   up (&vmap_s.pusem);       /* Free vmap_s semaphore */
egress:
   return stat;
}

/*=============================================================================

  Abs:  Insert message in command ring.

  Name: PCIL_IO_Insert_in_Command_Ring

  Args:
       *device_block_ps
         Use:  device database for this priority
         Type: deviceblock_ts
         Acc:  write
         Mech: by reference

       *message_ps
         Use:  message to insert
         Type: message_ts
         Acc:  write
         Mech: by reference

  Rem:  None

  Side: PCIL doorbell interrupt is generated.

  Ret:  None

==============================================================================*/

void PCIL_IO_Insert_in_Command_Ring (deviceblock_ts *device_block_ps,
                                     message_ts *message_ps)
{
   int1u        cindex;                /* command ring index              */
   /*---------------------------------------------------------------------------*/
   /*
   ** Make sure that the message is visible from the PCI.
   */
   MFENCE;
   /*
   ** Insert request into next command ring element.
   */
   device_block_ps->
      command_ring_p[cindex = device_block_ps->command_ring_index & COMMAND_RING_MASK] =
      message_ps->ring_element;
   /*
   ** Make sure that the command ring element is visible from the PCI.
   */
   MFENCE;
   /*
   ** Interrupt the PCIL by writing into the doorbell register.
   */
   *device_block_ps->doorbell_register_p = 0;
   /*
   ** Make sure that the doorbell ring happens immediately.
   */
   MFENCE;
   /*
   ** Update counters.
   */
   device_block_ps->command_ring_index++;
   device_block_ps->command_ring_count++;
   /*
   ** Save message location.
   */
   device_block_ps->command_contents_ps[cindex] = message_ps;
   /*
   ** Specify message location for ease of debugging.
   */
   message_ps->ring_index = cindex;
   message_ps->ring_type = COMMAND_RING;
   return;
}


/*=============================================================================

  Abs:  Insert message in response ring.

  Name: PCIL_IO_Insert_in_Response_Ring

  Args:
       *device_block_ps
         Use:  device database for this priority
         Type: deviceblock_ts
         Acc:  write
         Mech: by reference

       *message_ps
         Use:  message to insert
         Type: message_ts
         Acc:  write
         Mech: by reference

  Rem:  None

  Side: None

  Ret:  None

==============================================================================*/

void PCIL_IO_Insert_in_Response_Ring (deviceblock_ts *device_block_ps,
                                      message_ts *message_ps)
{
  int1u        rindex;                /* response ring index             */
  /*---------------------------------------------------------------------------*/
  /*
  ** Insert request into next response ring element.
  */
  device_block_ps->
      response_ring_p[rindex = device_block_ps->response_ring_index & RESPONSE_RING_MASK] =
      message_ps->ring_element;
  /*
  ** Make sure that the response ring element is visible from the PCI.
  */
  MFENCE;
  /*
  ** Update counters.
  */
  device_block_ps->response_ring_index++;
  device_block_ps->response_ring_count++;
  /*
  ** Save message location.
  */
  device_block_ps->response_contents_ps[rindex] = message_ps;
  /*
  ** Specify message location for ease of debugging.
  */
  message_ps->ring_index = rindex;
  message_ps->ring_type = RESPONSE_RING;
  return;
}


/*=============================================================================

  Abs:  Enqueue message_ts on singly linked queue with tail pointer.

  Name: PCIL_IO_Enqueue_Message

  Args:
       *message_queue_ps
         Use:  message queue header
         Type: messagequeue_ts
         Acc:  write
         Mech: by reference

       *message_ps
         Use:  message to enqueue
         Type: message_ts
         Acc:  write
         Mech: by reference

  Rem:

  Side: None

  Ret:  None

==============================================================================*/

void PCIL_IO_Enqueue_Message (messagequeue_ts *message_queue_ps,
                              message_ts      *message_ps)
{
   /*---------------------------------------------------------------------------*/
   /*
   ** Add message to queue at tail.
   */
   *message_queue_ps->tail_pointer_pp = message_ps;
   /*
   ** Mark end of queue.
   */
   message_ps->link_p = NULL;
   /*
   ** Update tail pointer.
   */
   message_queue_ps->tail_pointer_pp = &message_ps->link_p;
   return;
}


/*=============================================================================

  Abs:  Dequeue message_ts from singly linked queue with tail pointer.

  Name: PCIL_IO_Dequeue_Message

  Args:
       *message_queue_ps
         Use:  message queue header
         Type: messagequeue_ts
         Acc:  write
         Mech: by reference

  Rem:

  Side: None

  Ret:  pointer to message_ts.

==============================================================================*/

message_ts *PCIL_IO_Dequeue_Message (messagequeue_ts *message_queue_ps)
{
   message_ts  *message_ps;     /* message pointer    */
   /*---------------------------------------------------------------------------*/
   /*
   ** Get message pointer from queue header.
   */
   if ( ( message_ps = message_queue_ps->head_pointer_p ) != NULL )
   {
      /*
      **  Remove message from queue and rebuild tail pointer if queue is now empty.
      */
      if ( ( message_queue_ps->head_pointer_p = message_ps->link_p ) == NULL )
             message_queue_ps->tail_pointer_pp = &message_queue_ps->head_pointer_p;
   }
   /*
   ** Return pointer to dequeued message.
   */
   return message_ps;
}


/*=============================================================================

  Abs:  Release message after use.

  Name: PCIL_IO_Release_Message

  Args:
       *device_block_ps
         Use:  device database for this priority
         Type: deviceblock_ts
         Acc:  write
         Mech: by reference

       *message_ps
         Use:  message to release
         Type: message_ts
         Acc:  write
         Mech: by reference

  Rem:  Insert message in response ring if there is a free slot available.
        If not, put the message on the free queue.

  Side: None

  Ret:  None

==============================================================================*/

void PCIL_IO_Release_Message (deviceblock_ts *device_block_ps,
                              message_ts *message_ps)
{
   /*---------------------------------------------------------------------------*/
   /*
   ** Check for a free response ring slot.
   */
   if ( device_block_ps->response_ring_count < RESPONSE_RING_LENGTH )
      /*
      ** Insert request into next response ring element.
      */
      PCIL_IO_Insert_in_Response_Ring (device_block_ps, message_ps);
   /*
   ** Otherwise put the message on the free queue.
   */
   else
   {
      PCIL_IO_Enqueue_Message (&device_block_ps->message_queue_s, message_ps);
      /*
      ** Unstall the transmit task if it is waiting for a message.
      */
      if ( device_block_ps->message_wait_count > 0 )
      {
         up (&device_block_ps->message_semaphore);
         device_block_ps->message_wait_count--;
      }
   }
   return;
}


/*=============================================================================

  Abs:  Reclaim unused command ring entries.

  Name: PCIL_IO_Poll_Command_Ring

  Args:
       *device_block_ps
         Use:  device database for this priority
         Type: deviceblock_ts
         Acc:  write
         Mech: by reference

  Rem:  Remove command ring entries which have been accepted by the PCIL.
        Whenever an entry is reclaimed, try to replace it with a request which
        is waiting for an entry.

  Side: None

  Ret:  None

==============================================================================*/

void PCIL_IO_Poll_Command_Ring (deviceblock_ts *device_block_ps)
{ 
   int1u        cindex;                /* command ring index              */
   message_ts  *message_ps;            /* message pointer                 */
   /*---------------------------------------------------------------------------*/
   /*
   ** Process each command ring element released by the PCIL.
   */
   while ( ( device_block_ps->command_ring_count > 0 ) &&
         ( (device_block_ps->command_ring_p[cindex = device_block_ps->
            command_ring_poll & COMMAND_RING_MASK] & PCIL_OWNED) == 0 ) )
   {
      /*
      ** Retrieve the message pointer and update counters
      */
      message_ps = device_block_ps->command_contents_ps[cindex];
      device_block_ps->command_ring_poll++;
      device_block_ps->command_ring_count--;
      /*
      ** Release the message
      */
      PCIL_IO_Release_Message (device_block_ps, message_ps);
      /*
      ** Replace the released element with a waiting message if one is available.
      */
      if ( (message_ps = PCIL_IO_Dequeue_Message (&device_block_ps->send_queue_s)) != NULL )
         PCIL_IO_Insert_in_Command_Ring (device_block_ps, message_ps);
   }
   return;
}


/*=============================================================================

  Abs:  Send message to PCIL.

  Name: PCIL_IO_Send_to_PCIL

  Args:
       *device_block_ps
         Use:  device database for this priority
         Type: deviceblock_ts
         Acc:  write
         Mech: by reference

       *message_ps
         Use:  message to send
         Type: message_ts
         Acc:  write
         Mech: by reference

  Rem:  Insert message in command ring if there is a free slot available.
        If not, put the message on the send queue.

  Side: None

  Ret:  None

==============================================================================*/

void PCIL_IO_Send_to_PCIL (deviceblock_ts *device_block_ps,
                           message_ts *message_ps)
{
   /*---------------------------------------------------------------------------*/
   /*
   ** Check for a free command ring slot.
   */
   if ( device_block_ps->command_ring_count < COMMAND_RING_LENGTH )
   /*
   ** Insert request into next command ring element.
   */
      PCIL_IO_Insert_in_Command_Ring (device_block_ps, message_ps);
   /*
   ** Otherwise put the message on the send queue and reclaim empty space.
   */
   else
   {
      PCIL_IO_Enqueue_Message (&device_block_ps->send_queue_s, message_ps);
      PCIL_IO_Poll_Command_Ring (device_block_ps);
   }
   return;
}

/*=============================================================================

  Abs:  Transmit messages to PCIL.

  Name: PCIL_IO_Transmit

  Args:
       *device_block_ps
         Use:  device database for this priority
         Type: deviceblock_ts
         Acc:  write
         Mech: by reference
  
       *request_ps
         Use:  freed request
         Type: request_ts
         Acc:  write

  Rem:  Allocate a message_ts in which to construct the message to the PCIL and a 
        unit of flow control bundled with a response id in the form of a flow block.
        Construct the message and send it to the PCIL.

  Side: None

  Ret:  status

==============================================================================*/
int PCIL_IO_Transmit (deviceblock_ts *device_block_ps, request_ts *request_ps)
{
   flow_ts      *flow_ps;        /* flow block pointer */
   message_ts   *message_ps;     /* message pointer    */
   function_u    loc_function;   /* local function union */
   int           stat = 0;       /* return status */
   /*---------------------------------------------------------------------------*/
   /*
   ** Obtain the message spinlock and prevent execution of tasklet (bottom half) 
   ** of the ISR. This spinlock serializes access to both the free and
   ** transmit message queues, the free flow block queue, and the ring counters 
   ** between us and the ISR tasklet.
   */
   spin_lock_bh (&device_block_ps->message_region);
   PDEBUG ("Got message_region in IO_transmit\n");
   /*
   ** Allocate a message.
   */
   while ( ( message_ps = PCIL_IO_Dequeue_Message (&device_block_ps->message_queue_s) ) == NULL )
   {
      /*
      ** If not sucessful on the first attempt, poll the command ring for unused 
      ** messages and try again.
      */
      PCIL_IO_Poll_Command_Ring(device_block_ps);
      if ( ( message_ps = PCIL_IO_Dequeue_Message (&device_block_ps->message_queue_s) ) == NULL )
      {
         /*
         ** If still unsuccessful, increment the wait count and wait for the receive 
         ** interrupt to free up a message. While waiting, the message spinlock must be released 
         ** or else the ISR tasklet will never run.
         */
         device_block_ps->message_wait_count++;
         spin_unlock_bh (&device_block_ps->message_region);
         schedule();  /* Let tasklet run if an interrupt happened. Necessary?? */
         if (down_interruptible (&device_block_ps->message_semaphore))
         {
            printk (KERN_ERR "PCIL: Signal interrupt on message_semaphore in PCIL_IO_Transmit\n");
            stat = -ERESTARTSYS;
            goto egress;
         }
         spin_lock_bh(&device_block_ps->message_region);
      }  /* if (message_ps == NULL) */
   }     /* while (message_ps == NULL) */
   /*
   ** Allocate a flow block.
   */
   flow_ps = NULL;
   while ( flow_ps == NULL )
   {
      /*
      ** Get flow block pointer from queue header.
      */
      if ( ( flow_ps = device_block_ps->flow_queue_s.head_pointer_p ) != NULL )
      {
         /*
         ** Remove flow block from queue and rebuild tail pointer if queue is now empty.
         */
         if ( ( device_block_ps->flow_queue_s.head_pointer_p = flow_ps->link_p ) == NULL )
                device_block_ps->flow_queue_s.tail_pointer_pp =
               &device_block_ps->flow_queue_s.head_pointer_p;
      }
      else
      {
         /*
         ** If unsuccessful, increment the wait count and wait for the receive task 
         ** to free up a flow block. While waiting, the message spinlock must be released 
         ** to allow the ISR tasklet to get message_region.
         */
         device_block_ps->flow_wait_count++;
         spin_unlock_bh (&device_block_ps->message_region);
         schedule();  /* Let tasklet run if an interrupt happened. Necessary? */
         if (down_interruptible (&device_block_ps->flow_semaphore))
         {
	    up (&device_block_ps->message_semaphore); /* From previous while loop */
            printk (KERN_ERR "PCIL: Signal interrupt on flow_semaphore in PCIL_IO_Transmit\n");
            stat = -ERESTARTSYS;
            goto egress;
         }
         spin_lock_bh (&device_block_ps->message_region);
      }    /* else */
   }       /* while flow_ps == NULL */
   /*
   ** Save the request pointer in the flow block.
   */
   flow_ps->request_ps = request_ps;
   /*
   ** Construct the message.
   */
   message_ps->xfer_size =
      sizeof (function_u) + sizeof (rspid_ts) + (6 * sizeof (u32  ));
   message_ps->function = loc_function = request_ps->function;
   loc_function.funct_code &= OPCODE_MASK;
   message_ps->rspid_s = flow_ps->rspid_s;
   message_ps->fwstatus = 0;
   message_ps->pci_size = stat = request_ps->pci_size;
   message_ps->pci_offset = 0;
   message_ps->pci_addr = request_ps->pci_addr;
   if ( ( message_ps->msg_size = request_ps->msg_size ) > 0 &&
        (  loc_function.opcode != BCOM_EMULATE ||
        ( (request_ps->function.funct_code & READ_MESSAGE) == 0) ) )
   {
      /*
      ** Check if request_ps->msg_p is user space. If so copy_from_user else memcpy.
      */
      if (__range_ok( request_ps->msg_p, request_ps->msg_size) == 0)
         copy_from_user (message_ps->message, request_ps->msg_p, (size_t) request_ps->msg_size);
      else
	 memcpy (message_ps->message, request_ps->msg_p, request_ps->msg_size);

      message_ps->xfer_size += request_ps->msg_size;
   }
   /*
   ** Send the message to the PCIL.
   */
   PCIL_IO_Send_to_PCIL (device_block_ps, message_ps);
   /*
   ** Release the message spinlock.
   */
   spin_unlock_bh (&device_block_ps->message_region);
egress:
   return stat;
}

/*=============================================================================

  Abs:  Construct new request_ts.

  Name: PCIL_IO_Construct_Request

  Args:
       *device_block_ps
         Use:  device database for this priority
         Type: deviceblock_ts
         Acc:  write
         Mech: by reference

  Rem:  kmalloc a new request_ts.  Init the wait queue for this request_ts.
        We don't need the device_block_ps for Linux but keep it for now.

  Side: None

  Ret:  Pointer to request_ts.

==============================================================================*/

request_ts *PCIL_IO_Construct_Request (deviceblock_ts  *device_block_ps)
{
   request_ts  *request_ps;         /* pointer to allocated request  */
   /*---------------------------------------------------------------------------*/
   /*
   ** Allocate request_ts and save self pointer to it.
   */
   request_ps = (request_ts *) kmalloc(sizeof(request_ts), GFP_KERNEL);
   /*
   ** Initialize the wait queue and condition.
   */
   init_waitqueue_head (&request_ps->wqcond_s.wq);
   request_ps->wqcond_s.cond = 0;
   return request_ps;
}

/*=============================================================================

  Abs:  Obtain request_ts from free queue or construct new request_ts.

  Name: PCIL_IO_Obtain_Request

  Args:
       *device_block_ps
         Use:  device database for this priority
         Type: deviceblock_ts
         Acc:  write
         Mech: by reference

  Rem:  Obtain request_ts from free queue with header in device block.  If none
        is available, construct a new request_ts.

  Side: Possible kmalloc for new request_ts.

  Ret:  Pointer to request_ts.

==============================================================================*/

request_ts *PCIL_IO_Obtain_Request (deviceblock_ts  *device_block_ps)
{
   request_ts  *request_ps = NULL;  /* pointer to allocated request    */
   /*---------------------------------------------------------------------------*/
   /*
   ** Obtain the request mutex.
   */
   if (down_interruptible (&device_block_ps->request_region))
   {
      printk (KERN_ERR "PCIL: Signal interrupt on mutex in PCIL_IO_Obtain_Request\n");
      goto egress;
   }
   /*
   ** Get request pointer from queue header.
   */
   if ( ( request_ps = device_block_ps->request_queue_s.head_pointer_p) != NULL )
   {
   /*
   ** Remove request from queue and rebuild tail pointer if queue is now empty.
   */
      if ( (device_block_ps->request_queue_s.head_pointer_p = request_ps->link_p ) == NULL )
            device_block_ps->request_queue_s.tail_pointer_pp =
	    &device_block_ps->request_queue_s.head_pointer_p;
   }
   /*
   ** Release the request mutex.
   */
   up (&device_block_ps->request_region);
   /*
   ** Construct a new request_ts if none was available.
   */
   if ( request_ps == NULL )
      request_ps = PCIL_IO_Construct_Request(device_block_ps);
   /*
   ** Return pointer to dequeued or constructed request.
   */
egress:
   return request_ps;
}


/*=============================================================================

  Abs:  Return request_ts to free queue.

  Name: PCIL_IO_Release_Request

  Args:
       *device_block_ps
         Use:  device database for this priority
         Type: deviceblock_ts
         Acc:  write
         Mech: by reference

       *request_ps
         Use:  freed request
         Type: request_ts
         Acc:  write
         Mech: by reference

  Rem:  Return request_ts to free queue with header in device block.

  Side: None

  Ret:  None

==============================================================================*/

void PCIL_IO_Release_Request (deviceblock_ts  *device_block_ps,
                              request_ts      *request_ps)
{
   /*---------------------------------------------------------------------------*/
   /*
   ** Obtain the request mutex. Free it if we got a signal.
   */
   if (down_interruptible (&device_block_ps->request_region))
   {
      printk (KERN_ERR "PCIL: Signal interrupt on request_region in PCIL_IO_Release_Request\n");
      kfree (request_ps);
      goto egress;
   }
   /*
   ** Add request_ts to queue at tail.
   */
   *device_block_ps->request_queue_s.tail_pointer_pp = request_ps;
   /*
   ** Mark end of queue.
   */
   request_ps->link_p = NULL;
   /*
   ** Update tail pointer.
   */
   device_block_ps->request_queue_s.tail_pointer_pp = &request_ps->link_p;
   /*
   ** Release the request mutex.
   */
   up (&device_block_ps->request_region);
egress:
   return;
}


/*=============================================================================

  Abs:  Allocate and initialize device block.

  Name: PCIL_IO_Create_Device_Block

  Args:
      **device_block_ps
         Use:  pointer to created device block
         Type: deviceblock_ts
         Acc:  write
         Mech: pointer by reference

       *doorbell_register_p
         Use:  PCIL doorbell register for this priority
         Type: u32  
         Acc:  read
         Mech: by reference

       *command_ring_p
         Use:  Command ring for this priority
         Type: u32   array
         Acc:  read
         Mech: by reference

       *response_ring_p
         Use:  Response ring for this priority
         Type: u32   array
         Acc:  read
         Mech: by reference

  Rem:  Allocate the device block and the flow blocks.
        Initialize all queues.  Place the flow blocks on the free queue;
	allocate the initial set of request blocks and place on the free queue.
	Create the transmit task input mailbox, and the request and message mutexes.
	Create additional semaphores and initialize counters.

  Side: None

  Ret:  None

==============================================================================*/

void PCIL_IO_Create_Device_Block (deviceblock_ts **device_block_pps,
                                  u32             *doorbell_register_p,
                                  u32             *command_ring_p,
                                  u32             *response_ring_p)
{
   deviceblock_ts *device_block_ps;  /* device block pointer */
   request_ts  *request_ps;          /* request pointer      */
   flow_ts     *flow_ps;             /* flow block pointer   */
   int2u        findex;              /* flow block index     */
   /*---------------------------------------------------------------------------*/
   /*
   ** Allocate and zero the device block.
   */
   *device_block_pps =
     device_block_ps = (deviceblock_ts *) kmalloc (sizeof (deviceblock_ts), GFP_KERNEL);
   memset (device_block_ps, 0x00, sizeof (deviceblock_ts));
   /*
   ** Allocate the flow blocks.
   */
   device_block_ps->flow_array_ps = 
      flow_ps = (flow_ts *) kmalloc ( (MAX_FLOW * sizeof (flow_ts)), GFP_KERNEL);
   /*
   ** Create the request mutex message spinlock.
   */
   init_MUTEX     (&device_block_ps->request_region);
   spin_lock_init (&device_block_ps->message_region);
   /*
   ** Create the message, flow, and synch mutexes, camac and bitbus semaphores.
   */
   init_MUTEX (&device_block_ps->message_semaphore);
   init_MUTEX (&device_block_ps->flow_semaphore);
   init_MUTEX (&device_block_ps->synch_semaphore);
   sema_init (&device_block_ps->camac_semaphore, MAX_CAMAC);
   sema_init (&device_block_ps->bitbus_semaphore, MAX_BITBUS);
   /*
   ** Initialize queue headers.
   */
   device_block_ps->request_queue_s.head_pointer_p = NULL;
   device_block_ps->request_queue_s.tail_pointer_pp = 
     &device_block_ps->request_queue_s.head_pointer_p;
   device_block_ps->flow_queue_s.head_pointer_p = NULL;
   device_block_ps->flow_queue_s.tail_pointer_pp = 
     &device_block_ps->flow_queue_s.head_pointer_p;
   device_block_ps->message_queue_s.head_pointer_p = NULL;
   device_block_ps->message_queue_s.tail_pointer_pp = 
     &device_block_ps->message_queue_s.head_pointer_p;
   device_block_ps->send_queue_s.head_pointer_p = NULL;
   device_block_ps->send_queue_s.tail_pointer_pp = 
     &device_block_ps->send_queue_s.head_pointer_p;
   /*
   ** Build and queue all of the flow blocks.
   */
   for (findex = 0; findex < MAX_FLOW; flow_ps++, findex++)
   {
     flow_ps->rspid_s.index = findex;
     flow_ps->rspid_s.use_count = 0;
     flow_ps->request_ps = NULL;
     flow_ps->link_p = NULL;
     *device_block_ps->flow_queue_s.tail_pointer_pp = flow_ps;
     device_block_ps->flow_queue_s.tail_pointer_pp = &flow_ps->link_p;
   }
   /*
   ** Allocate and queue the initial set of request blocks.
   */
   for (findex = 0; findex < INITIAL_REQUEST; findex++)
   {
     request_ps = PCIL_IO_Construct_Request (device_block_ps);
     PCIL_IO_Release_Request (device_block_ps, request_ps);
   }
   /*
   ** Save the doorbell register and ring pointers.
   */
   device_block_ps->doorbell_register_p = doorbell_register_p;
   device_block_ps->command_ring_p = command_ring_p;
   device_block_ps->response_ring_p = response_ring_p;
   return;
}


/*=============================================================================

  Abs:  Interrupt handler for the PCIL input interrupt.

  Name: pcil_isr

  Args: 
        irq
         Use:  Command ring for this priority
         Type: int
         Acc:  read
         Mech: by value

       *devp
         Use:  User data
         Type: void *
         Acc:  read
         Mech: by reference

       *regs
         Use:  Command ring for this priority
         Type: struct pt_regs*
         Acc:  read
         Mech: by reference

  Rem:  Acknowledge the high or low priority interrupt and schedule the appropriate
        tasklet. Do special init check if low priority interrupt. 

  Ret:  None

==============================================================================*/

static void pcil_isr (int irq, void *dev_p, struct pt_regs *regs_ps)
{
   u32          interrupt_register;     /* interrupt type register storage */
   /*---------------------------------------------------------------------------*/
   PDEBUG ("ISR entered\n");
   /*
   ** Read interrupt descriptor register and check for high priority interrupt.
   */
   if ( ((interrupt_register = readl(PCIL_IO_Interrupt_Type_p)) 
          & HIGH_PRIORITY_INTERRUPT) != 0 )
   {
      /*
      ** Clear the interrupt by writing the acknowledge register.
      */
      writel (0, PCIL_IO_Acknowledge_High_p);
      MFENCE;
      /*
      ** Schedule tasklet for high priority queue.
      */
      tasklet_schedule (&TaskletHigh);
   }
   /*
   ** Check for low priority interrupt.
   */
   if ( (interrupt_register & LOW_PRIORITY_INTERRUPT) != 0 )
   {
      /*
      ** Clear the interrupt by writing the acknowledge register.
      */
      writel (0,PCIL_IO_Acknowledge_Low_p);
      MFENCE;
      /*
      ** If init is not waiting for the interrupt then schedule the low priority tasklet
      ** else send to the init_queue to let init know we got the interrupt.
      */
      if (init_wait == 0)
      {
         tasklet_schedule (&TaskletLow);
      }
      else
      {
         init_wait = 0;
         wake_up_interruptible (&init_queue);
      }
   }
/* printk (KERN_DEBUG "Dismiss interrupt\n");*/
}


/*=============================================================================

  Abs:  Called by high or low priority Tasklet to process incoming messages from PCIL.

  Name: PCIL_IO_Receive

  Args:
       *device_block_ps
         Use:  device database for this priority
         Type: deviceblock_ts
         Acc:  write
         Mech: by reference

  Rem:  This is the guts of the Tasklet or bottom-half of the interrupt processing
        that does all of the real work. Lots of comments in the code below.

  Side: None

  Ret:  void

==============================================================================*/
void PCIL_IO_Receive (deviceblock_ts *device_block_ps)
{
   int1u        rindex;                 /* response ring index             */
   function_u   loc_function;           /* local function union            */
   function_u   req_function;           /* request function union          */
   request_ts  *request_ps;             /* request pointer                 */
   flow_ts     *flow_ps;                /* flow block pointer              */
   message_ts  *message_ps;             /* response message pointer        */
   message_ts  *replacement_message_ps; /* replacement message pointer     */
   /*---------------------------------------------------------------------------*/
   PDEBUG ("ISR Tasklet entered.\n");
   /*
   ** Obtain the message spinlock.  This spinlock serializes access to both the free and
   ** transmit message queues, the free flow block queue, and the ring counters.
   */
   spin_lock (&device_block_ps->message_region);
   /*
   ** For each response ring element released by the PCIL, process the
   ** indicated request.
   */
   while ( ( device_block_ps->response_ring_count > 0 ) &&
         ( ((device_block_ps->
            response_ring_p[rindex = device_block_ps->response_ring_poll & RESPONSE_RING_MASK])
           & PCIL_OWNED) == 0 ) )
   {
      /*
      ** Retrieve the response message pointer and update counters
      */
      message_ps = device_block_ps->response_contents_ps[rindex];
      device_block_ps->response_ring_count--;
      device_block_ps->response_ring_poll++;
      /*
      ** Try to locate a replacement message to insert into the response ring.
      ** If a replacement message is available on the free queue then insert it
      ** into the response ring.
      */
      if (  ( replacement_message_ps =
	  PCIL_IO_Dequeue_Message (&device_block_ps->message_queue_s) ) != NULL )
          PCIL_IO_Insert_in_Response_Ring (device_block_ps, replacement_message_ps);
      /*
      ** Otherwise poll the command ring for unused messages.
      */
      else
         PCIL_IO_Poll_Command_Ring(device_block_ps);
      /*
      ** Process the response message by opcode.  The confusing thing here is that the PCIL
      ** code routes its messages by opcode.  Some opcodes require further processing
      ** by the FECC after initial processing by the PCIL and thus get sent to the FECC.  The FECC
      ** processes these and sets the REMOTE_PROCESSING_COMPLETE bit in the funct_code before
      ** returning them to the PCIL.  In the PCIL, some messages are processed by the same task
      ** that initially processed them, but the presence of the REMOTE_PROCESSING_COMPLETE bit
      ** is recognized and so the processing is different.  In order to ensure that the messages
      ** now are routed back to the PC, the opcodes are changed(!!!!!) at this point.
      ** So . . . MBCD_EMULATE is converted into SEND_TO_FECC with the REMOTE_PROCESSING_COMPLETE
      ** bit set.  Similarly, DIAG_FECC is converted into DIAG_PCIL with REMOTE_PROCESSING_COMPLETE.
      */
      loc_function.funct_code = message_ps->function.funct_code & OPCODE_MASK;
      /*
      ** In the case of SEND_TO_PC without REMOTE_PROCESSING_COMPLETE, the message is truly a 
      ** message from the FECC to the PC that was not solicited (at this level) by the PC. 
      ** This message contains a pointer to a kernel semaphore that was obtained by the user
      ** via the IOCTL_PCIL_NEWFECCMBX function and passed in by a read. The DMA address and 
      ** the address of this semaphore was set up in the outgoing appmsg so this just notifies
      ** the waiting read that data has arrived.
      */
      if ( ( loc_function.opcode == SEND_TO_PC ) &&
           ( (message_ps->function.funct_code & REMOTE_PROCESSING_COMPLETE) == 0 ) )
      {
         PDEBUG ("Got Send to PC message in Tasklet\n");
         /*
         ** Wake up the waiting reader.
         */
	 PDEBUG ("In Tasklet message data = %x %x %x\n",
                  message_ps->message[0], message_ps->message[1], message_ps->message[2]);
         if (message_ps->message[0] != 0)
	 {
	   /*
	    wqcond_ts *wqcond_ps = (wqcond_ts *)message_ps->message[0];
	    wqcond_ps->cond++;
	    wake_up_interruptible (&wqcond_ps->wq);
	   */
	 }
         /*
         ** Construct the response to the FECC's request in the existing message buffer.
	 ** We only use message[0] here but message[1] might also be used so send the size
	 ** back for both of them.
         */
         message_ps->function.funct_code |= REMOTE_PROCESSING_COMPLETE;
         message_ps->xfer_size -= 2 * sizeof (message_ps->message[0]);
         message_ps->msg_size -=  2 * sizeof (message_ps->message[0]);
         message_ps->pci_size = 0;
         /*
         ** Send this response message back to the FECC via the PCIL.
         */
         PCIL_IO_Send_to_PCIL (device_block_ps, message_ps);
         /*
         ** Clear the message pointer so it won't be deallocated in this routine.
         */
         message_ps = NULL;
      }
      /*
      ** In the case of SEND_TO_FECC or BCOM_EMULATE with REMOTE_PROCESSING_COMPLETE
      ** or DIAG_PCIL, the message is a completion message for a previous request.
      ** The response id (rspid) within the message is used to locate the flow block,
      ** which in turn points to the original request_ts.
      */
      else if ( ( loc_function.opcode == DIAG_PCIL ) ||
              ( (loc_function.opcode == SEND_TO_FECC ||
                loc_function.opcode == BCOM_EMULATE ) &&
              ( (message_ps->function.funct_code & REMOTE_PROCESSING_COMPLETE) != 0 ) ) )
      {
         flow_ps = &device_block_ps->
               flow_array_ps[(message_ps->rspid_s.index) & (MAX_FLOW - 1)];
         /*
         ** The use count in the rspid is employed to validate the index.
         */
         if ( message_ps->rspid_s.use_count == flow_ps->rspid_s.use_count )
         {
            /*
            ** Get the request_ts pointer, copy the PCIL execution status, 
	    ** transfer any BITbus read message and wake the wait_queue.
            */
            request_ps = flow_ps->request_ps;
            request_ps->fwstatus = message_ps->fwstatus;
            req_function.funct_code = request_ps->function.funct_code & OPCODE_MASK;
            if ( ( message_ps->msg_size = request_ps->msg_size ) > 0 &&
                req_function.opcode == BCOM_EMULATE &&
                ( (request_ps->function.funct_code & READ_MESSAGE) != 0) )
	    {
               /*
               ** Check if request_ps->msg_p is user space. If so copy_to_user else memcpy.
               */
               if (__range_ok( request_ps->msg_p, request_ps->msg_size) == 0)
                  copy_to_user (request_ps->msg_p, message_ps->message, 
                                (size_t) request_ps->msg_size);
               else
	          memcpy (request_ps->msg_p, message_ps->message, request_ps->msg_size);
	    }
            PDEBUG ("Waking up caller from ISR.\n");
            request_ps->wqcond_s.cond++;
            wake_up_interruptible (&request_ps->wqcond_s.wq);
            /*
            ** Update the flow block use count and put the flow block on the free queue.
            */
            flow_ps->rspid_s.use_count++;
            *device_block_ps->flow_queue_s.tail_pointer_pp = flow_ps;
            /*
            ** Mark end of queue.
            */
            flow_ps->link_p = NULL;
            /*
            ** Update tail pointer.
            */
            device_block_ps->flow_queue_s.tail_pointer_pp = &flow_ps->link_p;
            /*
            ** Unstall the transmit task if it is waiting for a flow block.
            */
            if ( device_block_ps->flow_wait_count > 0 )
            {
               up (&device_block_ps->flow_semaphore);
               device_block_ps->flow_wait_count--;
            }
         }  /* if message use_count == flow use_count */
         else
         {
            printk (KERN_ERR "PCIL: Bad RSPID in Interrupt Tasklet. Msg use %d flow use %d \n",
                    message_ps->rspid_s.use_count, flow_ps->rspid_s.use_count);
         }
      }
      /*
      ** Error reports are generated when a hardware error (i.e. PCI or link) is detected by PCIL
      ** software when it is not processing a request message from the PC.  This generally means 
      ** that the PCIL was either forwarding a pattern to the FECC or processing a remote I/O 
      ** request from the FECC. The overall PCIL execution status element of the message is
      ** sent to the Linux kernel message log.
      */
      else if ( loc_function.opcode == ERROR_REPORT )
      {
         printk (KERN_ERR "PCIL: internal error. Status %x\n", message_ps->fwstatus);
      }
      /*
      ** Everything else is unexpected.  Log an error message.
      */
      else
      {
         printk (KERN_ERR "PCIL: uexpected message. Function code %x\n", 
                 message_ps->function.funct_code);
      }
      /*
      ** Release the message if it wasn't already sent back to the PCIL.
      */
      if ( message_ps != NULL )
           PCIL_IO_Release_Message (device_block_ps, message_ps);
   }
   /*
   ** Release the message spinlock.
   */
   spin_unlock (&device_block_ps->message_region);
}


/*=============================================================================

  Abs:  Tasklet scheduled by ISR to process incoming high priority messages from PCIL.

  Name: PCIL_IO_Receive_High

  Args: None

  Rem:  Wrapper to call PCIL_IO_Receive with high-priority device block.

  Side: None

  Ret:  Never

==============================================================================*/

void PCIL_IO_Receive_High (unsigned long dummy)
{
   /*---------------------------------------------------------------------------*/
   /*
   ** PCIL_IO_Receive does all of the work.
   */
   PCIL_IO_Receive (PCIL_IO_Device_Block_High_ps);
}


/*=============================================================================

  Abs:  Tasklet scheduled by ISR to process incoming low priority messages from PCIL.

  Name: PCIL_IO_Receive_Low

  Args: None

  Rem:  Wrapper to call PCIL_IO_Receive with low-priority device block.

  Side: None

  Ret:  Never

==============================================================================*/

void PCIL_IO_Receive_Low (unsigned long dummy)
{
   /*---------------------------------------------------------------------------*/
   /*
   ** PCIL_IO_Receive does all of the work.
   */
   PCIL_IO_Receive (PCIL_IO_Device_Block_Low_ps);
}

/*=============================================================================

  Abs:  Allocate messages and pre-load response ring.

  Name: PCIL_IO_Initialize_Response_Ring

  Args:
       *device_block_ps
         Use:  device database for this priority
         Type: deviceblock_ts
         Acc:  write
         Mech: by reference

  Rem:  Kmalloc space for the messages.  Fill the response ring with messages,
        and place the remainder on the free queue.

  Side: None

  Ret:  None

==============================================================================*/

void PCIL_IO_Initialize_Response_Ring (deviceblock_ts *device_block_ps)
{
   message_ts  *message_ps;             /* message pointer         */
   int1u        mindex;                 /* message index           */
   /*---------------------------------------------------------------------------*/
   /*
   ** Allocate and zero the message area.
   */
   message_ps = (message_ts *) kmalloc((MAX_MESSAGE * sizeof (message_ts)), GFP_KERNEL);
   memset (message_ps, 0x00, (size_t) (MAX_MESSAGE * sizeof (message_ts)));
   /*
   ** Build and release all of the messages.
   */
   for (mindex = 0; mindex < MAX_MESSAGE; message_ps++, mindex++)
   {
      message_ps->buff_no = mindex;
      message_ps->ring_element = __pa(&message_ps->xfer_size) | PCIL_OWNED;
      PCIL_IO_Release_Message (device_block_ps, message_ps);
   }
   return;
}


/*=============================================================================

  Abs:  Transmit message to PCIL and wait for completion.

  Name: PCIL_IO_Xmit_Wait

  Args:
       *device_block_ps
         Use:  device database for this priority
         Type: deviceblock_ts
         Acc:  write
         Mech: by reference

       *request_ps
         Use:  request message
         Type: message_ts
         Acc:  read
         Mech: by reference

  Rem:  Use PCIL_IO_Transmit to send the message and wait on the wait queue.

  Side: 

  Ret:  Stat from PCIL_IO_Transmit.

==============================================================================*/

int PCIL_IO_Xmit_Wait (deviceblock_ts *device_block_ps, request_ts *request_ps)
{
   int stat;
   /*---------------------------------------------------------------------------*/
   /*
   ** Transmit request to PCIL.
   */
   if ( (stat = PCIL_IO_Transmit (device_block_ps, request_ps)) < 0 )
      goto egress;
   /*
   ** Wait for interrupt and request completion.
   */
   wait_event_interruptible (request_ps->wqcond_s.wq, request_ps->wqcond_s.cond);
   request_ps->wqcond_s.cond = 0;    /* Clear condition */
   /*
   ** Check the PCIL execution status.
   */
   if ( request_ps->fwstatus != 0 )
   {
      stat = -ECOMM;
      printk (KERN_ERR "PCIL: Error status from PCIL = %x in PCIL_IO_Xmit_Wait\n", 
              request_ps->fwstatus);
   }
egress:
   return stat;
}


/*=============================================================================

  Abs:  Send request message to MBCD emulator.

  Name: PCIL_IO_Send_MBCD

  Args:
        package_addr
         Use:  physical address of MBCD package
         Type: u32  
         Acc:  read
         Mech: by value

        package_size
         Use:  length of MBCD package (bytes)
         Type: u32  
         Acc:  read
         Mech: by value
 
       *device_block_ps
         Use:  device database for this priority
         Type: deviceblock_ts
         Acc:  write
         Mech: by reference

  Rem:  Assume all pointers have already been converted to physical addresses.
        Allocate a request_ts and fill it in. Transmit request to PCIL.
        Check final status.

  Side: Possible allocation of new request_ts.

  Ret:  Status

==============================================================================*/

int PCIL_IO_Send_MBCD (u32              package_addr,
                       u32              package_size,
                       deviceblock_ts  *device_block_ps)
{
   request_ts  *request_ps = NULL;        /* pointer to request */
   int          stat= 0;                  /* Default to OK */
   BOOL         MBCD_allocated = FALSE_F; /* MBCD clone allocated flag  */
   /*---------------------------------------------------------------------------*/
   /*
   ** Obtain a unit from the camac semaphore, representing the use of one of the clones of
   ** the FECC's MBCD emulator.
   */
   if (down_interruptible (&device_block_ps->camac_semaphore))
   {
      stat = -ERESTARTSYS;
      goto egress;
   }
   MBCD_allocated = TRUE_T;
   /*
   ** Obtain a request_ts.
   */
   if ( ( request_ps = PCIL_IO_Obtain_Request (device_block_ps) ) != NULL )
   {
      /*
      ** Fill in the request_ts.
      */
      request_ps->function.opcode = MBCD_EMULATE;
      request_ps->function.funct_code |= ( PREFETCH_PACKAGE | PREFETCH_WRITE_DATA );
      request_ps->pci_size = stat = package_size;
      request_ps->pci_addr = package_addr;
      request_ps->msg_size = 0;
      request_ps->msg_p = NULL;
      /*
      ** Transmit request to PCIL and wait for execution to complete.
      */
      stat = PCIL_IO_Xmit_Wait (device_block_ps, request_ps);
   }  /* if (Obtain_request_ps != NULL) */
   /*
   ** Release the the CAMAC semaphore unit and request_ts.
   */
egress:
   if ( MBCD_allocated )
      up (&device_block_ps->camac_semaphore);
   if ( request_ps != NULL )
      PCIL_IO_Release_Request (device_block_ps, request_ps);
   return stat;
}

/*=============================================================================

  Abs:  Send transmit request message to BITbus master emulator.

  Name: PCIL_IO_Send_BITbus

  Args:
       *transmit_buffer_p
         Use:  buffer holding contents of transmit message
         Type: u32  
         Acc:  write
         Mech: by reference

        transmit_buffer_size
         Use:  length of transmit message (bytes)
         Type: u32  
         Acc:  read
         Mech: by value

        master_unit_number
         Use:  BITbus master unit number
         Type: u32  
         Acc:  read
         Mech: by value
 
       *device_block_ps
         Use:  device database for this priority
         Type: deviceblock_ts
         Acc:  write
         Mech: by reference

  Rem:  Allocate a request_ts and fill it in. Transmit to PCIL.
        Check final status.

  Side: Possible allocation of new request_ts.

  Ret:  status

==============================================================================*/

int PCIL_IO_Send_BITbus  (u32             *transmit_buffer_p,
                          u32              transmit_buffer_size,
                          u32              master_unit_number,
                          deviceblock_ts  *device_block_ps)
{
   int          stat= 0;                    /* Default to OK */
   request_ts  *request_ps = NULL;          /* pointer to request */
   BOOL         BITbus_allocated = FALSE_F; /* BITbus queue slot allocated flag */
   /*---------------------------------------------------------------------------*/
   /*
   ** Obtain a unit from the BITbus semaphore, representing the use of one of the 
   ** queue slots in the FECC3's BITbus master emulator.
   */
   if (down_interruptible (&device_block_ps->bitbus_semaphore))
   {
      stat = -ERESTARTSYS;
      goto egress;
   }
   BITbus_allocated = TRUE_T;
   /*
   ** Obtain a request_ts.
   */
   if ( ( request_ps = PCIL_IO_Obtain_Request (device_block_ps) ) != NULL )
   {
      /*
      ** Fill in the request_ts.
      */
      request_ps->function.opcode = BCOM_EMULATE;
      request_ps->function.funct_code |= 
           ( (master_unit_number & BITBUS_UNIT_MASK) << BITBUS_UNIT_SHIFT );
      request_ps->pci_size = 0;
      request_ps->pci_addr = 0;
      request_ps->msg_size = transmit_buffer_size;
      request_ps->msg_p = transmit_buffer_p;
      /*
      ** Transmit rquest to PCIL and wait for execution to complete.
      */
      stat = PCIL_IO_Xmit_Wait (device_block_ps, request_ps);
   }  /* if (Obtain_Request != NULL) */
   /*
   ** Release the the BITbus semaphore unit and request_ts.
   */
egress:
   if ( BITbus_allocated )
      up (&device_block_ps->bitbus_semaphore);
   if ( request_ps != NULL )
      PCIL_IO_Release_Request (device_block_ps, request_ps);
   return stat;
}

/*=============================================================================

  Abs:  Send application message forwarding request to PCIL.

  Name: PCIL_IO_Send_Appmsg

  Args:
       *appmsg_ps
         Use:  common application message header at beginning of message
         Type: void *
         Acc:  read
         Mech: by reference

        appmsg_size
         Use:  length of appmsg (bytes)
         Type: u32  
         Acc:  read
         Mech: by value

       *device_block_ps
         Use:  device database for this priority
         Type: deviceblock_ts
         Acc:  write
         Mech: by reference

  Rem:  Allocate a request_ts and fill it in. Transmit to PCIL.
        Check final status.

  Side: Possible allocation of new request_ts.

  Ret:  status

==============================================================================*/

int PCIL_IO_Send_Appmsg (void *appmsg_ps, u32   size,
                         deviceblock_ts  *device_block_ps)
{
   int           stat= 0;             /* Default to OK */
   request_ts   *request_ps = NULL;   /* pointer to request   */
   pcil_addr_ts  addr_s = {0};        /* Struct to map/release user's address */
   /*---------------------------------------------------------------------------*/
   /*
   ** Obtain a request_ts.
   */
   if ( ( request_ps = PCIL_IO_Obtain_Request (device_block_ps) ) != NULL )
   {
      /*
      ** Fill in the request_ts.
      */
      request_ps->function.opcode = SEND_TO_FECC;
      request_ps->function.funct_code |= READ_MESSAGE;
      request_ps->pci_size = size;
      addr_s.virtaddr_p = appmsg_ps;
      addr_s.len = request_ps->pci_size;
      if ( (stat = PCIL_GetAddr (&addr_s)) < 0)
         goto egress;
      request_ps->pci_addr = addr_s.physaddr;
      request_ps->msg_size = 0;
      request_ps->msg_p = NULL;
      /*
      ** Transmit request to PCIL and wait for execution to complete.
      */
      stat = PCIL_IO_Xmit_Wait (device_block_ps, request_ps);
   }  /* if (request_ps != NULL) */
egress:
   /*
   ** Release the request_ts & address map.
   */
   if ( request_ps != NULL )
   {
      PCIL_IO_Release_Request (device_block_ps, request_ps);
      PCIL_RelAddr (&addr_s);
   }
   return stat;
}

/*=============================================================================

  Abs:  Receive high priority BITbus message.

  Name: PCIL_IO_Receive_BITbus

  Args:
       *receive_buffer_p
         Use:  buffer to receive contents of response message
         Type: u32  
         Acc:  read
         Mech: by reference

        receive_buffer_size
         Use:  maximum length of receive message (bytes)
         Type: u32  
         Acc:  read
         Mech: by value

        master_unit_number
         Use:  BITbus master unit number
         Type: u32  
         Acc:  read
         Mech: by value
 
  Rem:  Allocate a request_ts and fill it in. Transmit to PCIL.
        Check final status.

  Side: Possible allocation of new request_ts.

  Ret:  # bytes transfered or error

==============================================================================*/

int PCIL_IO_Receive_BITbus (u32             *receive_buffer_p,
                            u32              receive_buffer_size,
                            u32              master_unit_number)
{
   int          stat= 0;             /* Default to OK */
   request_ts  *request_ps = NULL;   /* pointer to request   */
   /*---------------------------------------------------------------------------*/
   /*
   ** Obtain a request_ts.
   */
   if ( ( request_ps = PCIL_IO_Obtain_Request (PCIL_IO_Device_Block_High_ps) ) != NULL )
   {
      /*
      ** Fill in the request_ts.
      */
      request_ps->function.opcode = BCOM_EMULATE;
      request_ps->function.funct_code |= 
         ( (master_unit_number & BITBUS_UNIT_MASK) << BITBUS_UNIT_SHIFT )
                                      | READ_MESSAGE;
      request_ps->pci_size = 0;
      request_ps->pci_addr = 0;
      request_ps->msg_size = receive_buffer_size;
      request_ps->msg_p = receive_buffer_p;
      /*
      ** Transmit request to PCIL and wait for execution to complete.
      */
      stat = PCIL_IO_Xmit_Wait (PCIL_IO_Device_Block_High_ps, request_ps);
   }  /* if (request_ps != NULL) */
   /*
   ** Release the request_ts.
   */
   if ( request_ps != NULL )
      PCIL_IO_Release_Request (PCIL_IO_Device_Block_High_ps, request_ps);
   return stat;
}


/*=============================================================================

  Abs:  Dump PCIL memory.

  Name: PCIL_IO_Dump_PCIL_Memory

  Args:
       *dump_buffer_p
         Use:  buffer to hold contents of dump
         Type: u32  
         Acc:  write
         Mech: by reference

        dump_size
         Use:  length of dump buffer (bytes)
         Type: u32  
         Acc:  read
         Mech: by value

        dump_address
         Use:  offset within PCIL memory from which dump occurs
         Type: u32  
         Acc:  read
         Mech: by value

  Rem:  Allocate a request_ts and fill it in. Transmit to PCIL.
        Check final status.

  Side: Possible allocation of new new request_ts.

  Ret:  # bytes transfered
 
==============================================================================*/

int PCIL_IO_Dump_PCIL_Memory (u32   *dump_buffer_p,
                              u32    dump_size,
	                      u32    dump_address)
{
   int           stat= 0;            /* Default to OK */
   request_ts   *request_ps = NULL;  /* pointer to request   */
   pcil_addr_ts  addr_s = {0};       /* Struct to map/release user's address */
   /*---------------------------------------------------------------------------*/
   /*
   ** Obtain a request_ts.
   */
   if ( ( request_ps = PCIL_IO_Obtain_Request (PCIL_IO_Device_Block_Low_ps) ) != NULL )
   {
      /*
      ** Fill in the request_ts.
      */
      request_ps->function.opcode = DIAG_PCIL;
      request_ps->function.funct_code |= COPY_PCIL_MEMORY;
      request_ps->pci_size = dump_size;
      addr_s.virtaddr_p = dump_buffer_p;
      addr_s.len = request_ps->pci_size;
      if ( (stat = PCIL_GetAddr (&addr_s)) < 0)
         goto egress;
      request_ps->pci_addr = addr_s.physaddr;
      request_ps->msg_size = sizeof(u32  );
      request_ps->msg_p =  &dump_address;
      /*
      ** Transmit request to PCIL and wait for execution to complete.
      */
      stat = PCIL_IO_Xmit_Wait (PCIL_IO_Device_Block_Low_ps, request_ps);
   }  /* if (request_ps != NULL) */
   /*
   ** Release the request_ts and physical mapping.
   */
egress:
   if ( request_ps != NULL )
   {
      PCIL_IO_Release_Request (PCIL_IO_Device_Block_Low_ps, request_ps);
      PCIL_RelAddr (&addr_s);
   }
   return stat;
}


/*=============================================================================

  Abs:  Get fine granularity timer count from PCIL.

  Name: PCIL_IO_Get_Counter

  Args:
       *counter_p
         Use:  Current counter contents
         Type: u32  
         Acc:  write
         Mech: by reference

  Rem:  Return current contents of 32 bit counter clocked by PCI bus clock.
 
  Side: None

  Ret:  # bytes returned
 
==============================================================================*/

int PCIL_IO_Get_Counter (u32   *counter_p)
{
   int stat = sizeof(int);
   /*---------------------------------------------------------------------------*/
   /*
   ** Read and return counter contents.
   */
   copy_to_user (counter_p, PCIL_IO_Counter_p, sizeof(u32  ));
   return stat;
}

/*=============================================================================

  Abs:  Get PCIL hardware/software version.

  Name: PCIL_IO_Get_PCIL_Version
  Args:
       *version_p
         Use:  current PCIL hardware/software version
         Type: u32  
         Acc:  write
         Mech: by reference

  Rem:  PCIL version is read into static by PCIL_IO_Init.

  Side: None

  Ret:  # bytes returned
 
==============================================================================*/

int PCIL_IO_Get_PCIL_Version (u32   *version_p)
{
   int stat = sizeof(int);
   /*---------------------------------------------------------------------------*/
   /*
   ** PCIL version was read into static by PCIL_IO_Init.
   */
   copy_to_user (version_p, &PCIL_IO_PCIL_Device_Version, sizeof(u32  ));
   return stat;
}

/*=============================================================================

  Abs:  Get PCIL CPU use statistics.

  Name: PCIL_IO_Get_PCIL_CPU_Stats

  Args:
       *stats_buffer_p
         Use:  buffer to hold CPU use statistics
         Type: u32  
         Acc:  write
         Mech: by reference

        stats_size
         Use:  length of stats buffer (bytes)
         Type: u32  
         Acc:  read
         Mech: by value

	monitor_mode_p
         Use:  new CPU statistics monitoring mode
         Type: u32  
         Acc:  read
         Mech: by value

  Rem:  Allocate a request_ts and fill it in. Transmit to PCIL.
        Check final status.

  Side: Possible allocation of new request_ts.

  Ret:  # bytes transfered
 
==============================================================================*/

int PCIL_IO_Get_PCIL_CPU_Stats (u32   *stats_buffer_p,
                                u32    stats_size,
                                u32   *monitor_mode_p)
{
   int           stat= 0;            /* Default to OK */
   request_ts   *request_ps = NULL;  /* pointer to request   */
   pcil_addr_ts  addr_s = {0};       /* Struct to map/release user's address */
   /*---------------------------------------------------------------------------*/
   /*
   ** Obtain a request_ts.
   */
   if ( ( request_ps = PCIL_IO_Obtain_Request (PCIL_IO_Device_Block_Low_ps) ) != NULL )
   {
      /*
      ** Fill in the request_ts.
      */
      request_ps->function.opcode = DIAG_PCIL;
      request_ps->function.funct_code |= PCIL_GET_CPU_STATS;
      request_ps->pci_size = stats_size;
      addr_s.virtaddr_p = stats_buffer_p;
      addr_s.len = request_ps->pci_size;
      if ( (stat = PCIL_GetAddr (&addr_s)) < 0)
         goto egress;
      request_ps->pci_addr = addr_s.physaddr;
      request_ps->msg_size = sizeof(u32  );
      request_ps->msg_p = &monitor_mode_p;
      /*
      ** Transmit request to PCIL and wait for execution to complete.
      */
      stat = PCIL_IO_Xmit_Wait (PCIL_IO_Device_Block_Low_ps, request_ps);
   }  /* if (request_ps != NULL) */
   /*
   ** Release the request_ts & address map.
   */
egress:
   if ( request_ps != NULL )
   {
      PCIL_IO_Release_Request (PCIL_IO_Device_Block_Low_ps, request_ps);
      PCIL_RelAddr (&addr_s);
   }
   return stat;
}


/*=============================================================================

  Abs:  Get PCIL2 link maintenance counters.

  Name: PCIL_IO_Get_PCIL2_Maintenance_Counters

  Args:
       *counters_buffer_p
         Use:  buffer to hold link maintenance counters
         Type: int2u
         Acc:  write
         Mech: by reference

        counters_size
         Use:  length of link maintenance counters buffer (bytes)
         Type: u32  
         Acc:  read
         Mech: by value

  Rem:  Allocate a request_ts and fill it in. Transmit to PCIL.
        Check final status.

  Side: Possible allocation of new request_ts.

  Ret:  # bytes transfered
 
==============================================================================*/

int PCIL_IO_Get_PCIL2_Maintenance_Counters (int2u *counters_buffer_p,
                                            u32    counters_size)
{
   int           stat= 0;            /* Default to OK */
   request_ts   *request_ps = NULL;  /* pointer to request   */
   pcil_addr_ts  addr_s = {0};       /* Struct to map/release user's address */
   /*---------------------------------------------------------------------------*/
   /*
   ** Obtain a request_ts.
   */
   if ( ( request_ps = PCIL_IO_Obtain_Request (PCIL_IO_Device_Block_Low_ps) ) != NULL )
   {
      /*
      ** Fill in the request_ts.
      */
      request_ps->function.opcode = DIAG_PCIL;
      request_ps->function.funct_code |= PCIL2_READ_MAINT_COUNTERS;
      request_ps->pci_size = counters_size;
      addr_s.virtaddr_p = counters_buffer_p;
      addr_s.len = request_ps->pci_size;
      if ( (stat = PCIL_GetAddr (&addr_s)) < 0)
         goto egress;
      request_ps->pci_addr = addr_s.physaddr;
      request_ps->msg_size = 0;
      request_ps->msg_p = NULL;
      /*
      ** Transmit request to PCIL and wait for execution to complete.
      */
      stat = PCIL_IO_Xmit_Wait (PCIL_IO_Device_Block_Low_ps, request_ps);
   }  /* if (request_ps != NULL) */
egress:
   /*
   ** Release the request_ts & address map if allocated.
   */
   if ( request_ps != NULL )
   {
      PCIL_IO_Release_Request (PCIL_IO_Device_Block_Low_ps, request_ps);
      PCIL_RelAddr (&addr_s);
   }
   return stat;
}


/*=============================================================================

  Abs:  Dump FECC memory.

  Name: PCIL_IO_Dump_FECC_Memory

  Args:
       *dump_buffer_p
         Use:  buffer to hold contents of dump
         Type: u32  
         Acc:  write
         Mech: by reference

        dump_size
         Use:  length of dump buffer (bytes)
         Type: u32  
         Acc:  read
         Mech: by value

        dump_address_p
         Use:  offset within FECC memory from which dump occurs
         Type: u32  
         Acc:  read
         Mech: by reference

  Rem:  Allocate a request_ts and fill it in. Transmit to PCIL.
        Check final status.

  Side: Possible allocation of new request_ts.

  Ret:  stat
 
==============================================================================*/

int PCIL_IO_Dump_FECC_Memory (u32   *dump_buffer_p,
                              u32    dump_size,
		              u32   *dump_address_p)
{
   int           stat= 0;            /* Default to OK */
   request_ts   *request_ps = NULL;  /* pointer to request   */
   pcil_addr_ts  addr_s = {0};       /* Struct to map/release user's address */
   /*---------------------------------------------------------------------------*/
   /*
   ** Obtain a request_ts.
   */
   if ( ( request_ps = PCIL_IO_Obtain_Request (PCIL_IO_Device_Block_Low_ps) ) != NULL )
   {
      /*
      ** Fill in the request_ts.
      */
      request_ps->function.opcode = DIAG_FECC;
      request_ps->function.funct_code |= FECC_PEEK;
      request_ps->pci_size = dump_size;
      addr_s.virtaddr_p = dump_buffer_p;
      addr_s.len = request_ps->pci_size;
      if ( (stat = PCIL_GetAddr (&addr_s)) < 0)
         goto egress;
      request_ps->pci_addr = addr_s.physaddr;
      request_ps->msg_size = sizeof(u32  );
      request_ps->msg_p = dump_address_p;
      /*
      ** Transmit request to PCIL and wait for execution to complete.
      */
      stat = PCIL_IO_Xmit_Wait (PCIL_IO_Device_Block_Low_ps, request_ps);
   }  /* if (request_ps != NULL) */
   /*
   ** Release the request_ts & address map if allocated.
   */
egress:
   if ( request_ps != NULL )
   {
      PCIL_IO_Release_Request (PCIL_IO_Device_Block_Low_ps, request_ps);
      PCIL_RelAddr (&addr_s);
   }
   return stat;
}


/*=============================================================================

  Abs:  Get FECC hardware/software version.

  Name: PCIL_IO_Get_FECC_Version

  Args:
       *version_p
         Use:  current FECC hardware/software version
         Type: u32  
         Acc:  read
         Mech: by reference

  Rem:  Allocate a request_ts and fill it in. Transmit to PCIL.
        Check final status.

  Side: Possible allocation of new request_ts.

  Ret:  stat
 
==============================================================================*/

int PCIL_IO_Get_FECC_Version (u32   *version_p)
{
   int           stat= 0;            /* Default to OK */
   request_ts   *request_ps = NULL;  /* pointer to request   */
   pcil_addr_ts  addr_s = {0};       /* Struct to map/release user's address */
   /*---------------------------------------------------------------------------*/
   /*
   ** Obtain a request_ts.
   */
   if ( ( request_ps = PCIL_IO_Obtain_Request (PCIL_IO_Device_Block_Low_ps) ) != NULL )
   {
      /*
      ** Fill in the request_ts.
      */
      request_ps->function.opcode = DIAG_FECC;
      request_ps->function.funct_code |= FECC_GET_VERSION;
      request_ps->pci_size = sizeof(u32  );
      addr_s.virtaddr_p = version_p;
      addr_s.len = request_ps->pci_size;
      if ( (stat = PCIL_GetAddr (&addr_s)) < 0)
         goto egress;
      request_ps->pci_addr = addr_s.physaddr;
      request_ps->msg_size = 0;
      request_ps->msg_p = NULL;
      /*
      ** Transmit request to PCIL and wait for execution to complete.
      */
      stat = PCIL_IO_Xmit_Wait (PCIL_IO_Device_Block_Low_ps, request_ps);
   }  /* if (request_ps != NULL) */
egress:
   /*
   ** Release the request_ts & address map if allocated.
   */
   if ( request_ps != NULL )
   {
      PCIL_IO_Release_Request (PCIL_IO_Device_Block_Low_ps, request_ps);
      PCIL_RelAddr (&addr_s);
   }
   return stat;
}


/*=============================================================================

  Abs:  Get FECC CPU use statistics.

  Name: PCIL_IO_Get_FECC_CPU_Stats

  Args:
       *stats_buffer_p
         Use:  buffer to hold CPU use statistics
         Type: u32  
         Acc:  write
         Mech: by reference

        stats_size
         Use:  length of stats buffer (bytes)
         Type: u32  
         Acc:  read
         Mech: by value

        monitor_mode_p
         Use:  new CPU statistics monitoring mode
         Type: u32  
         Acc:  read
         Mech: by value

  Rem:  Allocate a request_ts and fill it in. Transmit to PCIL.
        Check final status.

  Side: Possible allocation of new request_ts.

  Ret:  stat
 
==============================================================================*/

int PCIL_IO_Get_FECC_CPU_Stats (u32   *stats_buffer_p,
                                u32    stats_size,
                                u32   *monitor_mode_p)
{
   int           stat= 0;           /* Default to OK */
   request_ts   *request_ps = NULL; /* pointer to request   */
   pcil_addr_ts  addr_s = {0};      /* Struct to map/release user's address */
   /*---------------------------------------------------------------------------*/
   /*
   ** Obtain a request_ts.
   */
   if ( ( request_ps = PCIL_IO_Obtain_Request (PCIL_IO_Device_Block_Low_ps) ) != NULL )
   {
      /*
      ** Fill in the request_ts.
      */
      request_ps->function.opcode = DIAG_FECC;
      request_ps->function.funct_code |= FECC_GET_CPU_STATS;
      request_ps->pci_size = stats_size;
      addr_s.virtaddr_p = stats_buffer_p;
      addr_s.len = request_ps->pci_size;
      if ( (stat = PCIL_GetAddr (&addr_s)) < 0)
         goto egress;
      request_ps->pci_addr = addr_s.physaddr;
      request_ps->msg_size = sizeof(u32  );
      request_ps->msg_p = monitor_mode_p;
      /*
      ** Transmit request to PCIL and wait for execution to complete.
      */
      stat = PCIL_IO_Xmit_Wait (PCIL_IO_Device_Block_Low_ps, request_ps);
   }  /* if (request_ps != NULL) */
egress:
   /*
   ** Release the request_ts & address map.
   */
   if ( request_ps != NULL )
   {
      PCIL_IO_Release_Request (PCIL_IO_Device_Block_Low_ps, request_ps);
      PCIL_RelAddr (&addr_s);
   }
   return stat;
}


/*=============================================================================

  Abs:  Get FECC3 link maintenance counters.

  Name: PCIL_IO_Get_FECC3_Maintenance_Counters
  Args:
       *counters_buffer_p
         Use:  buffer to hold link maintenance counters
         Type: int2u
         Acc:  write
         Mech: by reference

        counters_size
         Use:  length of link maintenance counters buffer (bytes)
         Type: u32  
         Acc:  read
         Mech: by value

  Rem:  Allocate a request_ts and fill it in. Transmit to PCIL.
        Check final status.

  Side: Possible allocation of new request_ts.

  Ret:  # bytes transfered
 
==============================================================================*/

int PCIL_IO_Get_FECC3_Maintenance_Counters (int2u *counters_buffer_p,
                                            u32    counters_size)
{
   int           stat= 0;            /* Default to OK */
   request_ts   *request_ps = NULL;  /* pointer to request   */
   pcil_addr_ts  addr_s = {0};       /* Struct to map/release user's address */
   /*---------------------------------------------------------------------------*/
   /*
   ** Obtain a request_ts.
   */
   if ( ( request_ps = PCIL_IO_Obtain_Request (PCIL_IO_Device_Block_Low_ps) ) != NULL )
   {
      /*
      ** Fill in the request_ts.
      */
      request_ps->function.opcode = DIAG_FECC;
      request_ps->function.funct_code |= FECC3_READ_MAINT_COUNTERS;
      request_ps->pci_size = counters_size;
      addr_s.virtaddr_p = counters_buffer_p;
      addr_s.len = request_ps->pci_size;
      if ( (stat = PCIL_GetAddr (&addr_s)) < 0)
         goto egress;
      request_ps->pci_addr = addr_s.physaddr;
      request_ps->msg_size = 0;
      request_ps->msg_p = NULL;
      /*
      ** Transmit request to PCIL and wait for execution to complete.
      */
      stat = PCIL_IO_Xmit_Wait (PCIL_IO_Device_Block_Low_ps, request_ps);
   }  /* if (request_ps != NULL) */
egress:
   /*
   ** Release the request_ts & address map.
   */
   if ( request_ps != NULL )
   {
      PCIL_IO_Release_Request (PCIL_IO_Device_Block_Low_ps, request_ps);
      PCIL_RelAddr (&addr_s);
   }
   return stat;
}

/*=============================================================================

  Abs:  Send trigger pattern to FECC.

  Name: PCIL_IO_Send_Pattern

  Args:
       *pattern_p
         Use:  Trigger pattern array to send
         Type: u32  [16] (PCIL/FECC) or u32 [32] (PCIL2/FECC3)
         Acc:  read
         Mech: by reference user virtual address

  Rem:  Load pattern array into PCIL registers and ring pattern doorbell.
 
  Side: None

  Ret: # bytes transfered
 
==============================================================================*/

int PCIL_IO_Send_Pattern (u32   *pattern_p)
{
    int  stat;         /* Request status */
   /*   u32      index;          copy looper */
   /*---------------------------------------------------------------------------*/
   /*
   ** Load pattern array into PCIL registers.
   */
   /* !!?? For the moment, avoid memcpy since it seems to generate a PCI block transfer
   ** that occasionally causes the PC to crash.  Use a simple copy loop instead, whcih
   ** generates a succession of individual longword register writes to the PCIL on the PCI.
   */
   copy_from_user (PCIL_IO_Pattern_ap, pattern_p, (size_t) PCIL_IO_Pattern_Length * sizeof (u32  ));
   stat =  PCIL_IO_Pattern_Length * sizeof (u32  );  /* # bytes transfered */
   /*   for (index = 0; index < PCIL_IO_Pattern_Length; index++)
      *(PCIL_IO_Pattern_ap + index) = pattern_p[index];
   */
   /*
   ** Make sure that the register load happens immediately.
   */
   MFENCE;
   /*
   ** Ring the pattern doorbell to transfer the register contents to the FECC.
   */
   *PCIL_IO_Doorbell_Pattern_p = 0;
   /*
   ** Make sure that the doorbell ring happens immediately.
   */
   MFENCE;
   return stat;
}


/*=============================================================================

  Abs:  Load FECC command register.

  Name: PCIL_IO_Load_FECC_Command

  Args:
       *command_word_p
         Use:  Write data for load of FECC command register
         Type: u32  
         Acc:  read
         Mech: by reference user virtual address

  Rem:  Allocate a request_ts and fill it in. Transmit to PCIL.
        Check final status.

  Side: Possible allocation of new request_ts.

  Ret:  # bytes transfered
 
==============================================================================*/

int PCIL_IO_Load_FECC_Command (u32   *command_word_p)
{
   int           stat= 0;           /* Default to OK */
   request_ts   *request_ps = NULL; /* pointer to request   */
   /*---------------------------------------------------------------------------*/
   /*
   ** Obtain a request_ts.
   */
   if ( ( request_ps = PCIL_IO_Obtain_Request (PCIL_IO_Device_Block_Low_ps) ) != NULL )
   {
      /*
      ** Fill in the request_ts.
      */
      request_ps->function.opcode = DIAG_PCIL;
      request_ps->function.funct_code |= PCIL_LOAD_FECC_COMMAND;
      request_ps->pci_size = 0;
      request_ps->pci_addr = 0;
      request_ps->msg_size = sizeof(u32  );
      request_ps->msg_p = command_word_p;
     /*
      ** Transmit request to PCIL and wait for execution to complete.
      */
      stat = PCIL_IO_Xmit_Wait (PCIL_IO_Device_Block_Low_ps, request_ps);
   }  /* if (request_ps != NULL) */
   /*
   ** Release the request_ts & address map.
   */
   if ( request_ps != NULL )
      PCIL_IO_Release_Request (PCIL_IO_Device_Block_Low_ps, request_ps);
   return stat;
}


/*=============================================================================

  Abs:  Load PCIL2 local link initialization register.

  Name: PCIL_IO_Load_Local_Link_Initialization

  Args:
       *initialization_word_p
         Use:  Write data for load of PCIL2 local link initialization register
         Type: u32  
         Acc:  read
         Mech: by reference user virtual address

  Rem:  Allocate a request_ts and fill it in. Transmit to PCIL.
        Check final status.

  Side: Possible allocation of new request_ts.

  Ret:  # bytes transfered
 
==============================================================================*/

int PCIL_IO_Load_Local_Link_Initialization (u32   *initialization_word_p)
{
   int           stat= 0;             /* Default to OK */
   request_ts   *request_ps = NULL;   /* pointer to request   */
   /*---------------------------------------------------------------------------*/
   /*
   ** Obtain a request_ts.
   */
   if ( ( request_ps = PCIL_IO_Obtain_Request (PCIL_IO_Device_Block_Low_ps) ) != NULL )
   {
      /*
      ** Fill in the request_ts.
      */
      request_ps->function.opcode = DIAG_PCIL;
      request_ps->function.funct_code |= PCIL2_LOAD_LOCAL_INIT;
      request_ps->pci_size = 0;
      request_ps->pci_addr = 0;
      request_ps->msg_size = sizeof(u32  );
      request_ps->msg_p = initialization_word_p;
     /*
      ** Transmit request to PCIL and wait for execution to complete.
      */
      stat = PCIL_IO_Xmit_Wait (PCIL_IO_Device_Block_Low_ps, request_ps);
   }  /* if (request_ps != NULL) */
   /*
   ** Release the request_ts & address map.
   */
   if ( request_ps != NULL )
      PCIL_IO_Release_Request (PCIL_IO_Device_Block_Low_ps, request_ps);
   return stat;
}


/*=============================================================================

  Abs:  Load PCIL2 remote link initialization register.

  Name: PCIL_IO_Load_Remote_Link_Initialization

  Args:
       *initialization_word_p
         Use:  Write data for load of PCIL2 remote link initialization register
         Type: u32  
         Acc:  read
         Mech: by reference e user virtual address

  Rem:  Allocate a request_ts and fill it in. Transmit to PCIL.
        Check final status.

  Side: Possible allocation of new request_ts.

  Ret:  # bytes transfered
 
==============================================================================*/

int PCIL_IO_Load_Remote_Link_Initialization (u32   *initialization_word_p)
{
   int           stat= 0;            /* Default to OK */
   request_ts   *request_ps = NULL;  /* pointer to request   */
   /*---------------------------------------------------------------------------*/
   /*
   ** Obtain a request_ts.
   */
   if ( ( request_ps = PCIL_IO_Obtain_Request (PCIL_IO_Device_Block_Low_ps) ) != NULL )
   {
      /*
      ** Fill in the request_ts.
      */
      request_ps->function.opcode = DIAG_PCIL;
      request_ps->function.funct_code |= PCIL2_LOAD_REMOTE_INIT;
      request_ps->pci_size = 0;
      request_ps->pci_addr = 0;
      request_ps->msg_size = sizeof(u32  );
      request_ps->msg_p = initialization_word_p;
     /*
      ** Transmit request to PCIL and wait for execution to complete.
      */
      stat = PCIL_IO_Xmit_Wait (PCIL_IO_Device_Block_Low_ps, request_ps);
   }  /* if (request_ps != NULL) */
   /*
   ** Release the request_ts & address map.
   */
   if ( request_ps != NULL )
      PCIL_IO_Release_Request (PCIL_IO_Device_Block_Low_ps, request_ps);
   return stat;
}


/*=============================================================================

  Abs:  Load FECC memory.

  Name: PCIL_IO_Load_FECC_Memory

  Args:
       *load_buffer_p
         Use:  buffer holding load data
         Type: u32  
         Acc:  read
         Mech: by reference

        load_size
         Use:  length of load buffer (bytes)
         Type: u32  
         Acc:  read
         Mech: by value

        load_address
         Use:  offset within FECC memory to which load occurs
         Type: u32  
         Acc:  read
         Mech: by value

  Rem:  Allocate a request_ts and fill it in. Transmit to PCIL.
        Check final status.

  Side: Possible allocation of new request_ts.

  Ret:  # bytes transfered
 
==============================================================================*/

int PCIL_IO_Load_FECC_Memory (u32   *load_buffer_p,
                              u32    load_size,
			      u32    load_address)
{
   int           stat= 0;            /* Default to OK */
   request_ts   *request_ps = NULL;  /* pointer to request   */
   pcil_addr_ts  addr_s = {0};       /* Struct to map/release user's address */
   /*---------------------------------------------------------------------------*/
   /*
   ** Obtain a request_ts.
   */
   if ( ( request_ps = PCIL_IO_Obtain_Request (PCIL_IO_Device_Block_Low_ps) ) != NULL )
   {
      /*
      ** Fill in the request_ts.
      */
      request_ps->function.opcode = DIAG_FECC;
      request_ps->function.funct_code |= ( FECC_POKE | PREFETCH_WRITE_DATA );
      request_ps->pci_size = load_size;
      addr_s.virtaddr_p = load_buffer_p;
      addr_s.len = request_ps->pci_size;
      if ( (stat = PCIL_GetAddr (&addr_s)) < 0)
         goto egress;
      request_ps->pci_addr = addr_s.physaddr;
      request_ps->msg_size = sizeof(u32  );
      request_ps->msg_p = &load_address;
      /*
      ** Transmit request to PCIL and wait for execution to complete.
      */
      stat = PCIL_IO_Xmit_Wait (PCIL_IO_Device_Block_Low_ps, request_ps);
   }  /* if (request_ps != NULL) */
egress:
   /*
   ** Release the request_ts & address map.
   */
   if ( request_ps != NULL )
   {
      PCIL_IO_Release_Request (PCIL_IO_Device_Block_Low_ps, request_ps);
      PCIL_RelAddr (&addr_s);
   }
   return stat;
}


/*=============================================================================

  Abs:  Load FECC CAMAC crate map.

  Name: PCIL_IO_Load_FECC_Crate_Map

  Args:
       *crate_map_p
         Use:  CAMAC crate map (one longword per crate)
         Type: u32  
         Acc:  read
         Mech: by reference

        crate_map_size
         Use:  length of crate map (bytes)
         Type: u32  
         Acc:  read
         Mech: by value

  Rem:  Allocate a request_ts and fill it in. Transmit to PCIL.
        Check final status.

  Side: Possible allocation of new request_ts.

  Ret:  # bytes transfered
 
==============================================================================*/

int PCIL_IO_Load_FECC_Crate_Map (u32   *crate_map_p,
                                 u32    crate_map_size)
{
   int           stat= 0;            /* Default to OK */
   request_ts   *request_ps = NULL;  /* pointer to request   */
   /*---------------------------------------------------------------------------*/
   /*
   ** Obtain a request_ts.
   */
   if ( ( request_ps = PCIL_IO_Obtain_Request (PCIL_IO_Device_Block_Low_ps) ) != NULL )
   {
      /*
      ** Fill in the request_ts.
      */
      request_ps->function.opcode = DIAG_FECC;
      request_ps->function.funct_code |= FECC_LOAD_CRATE_MAP;
      request_ps->pci_size = 0;
      request_ps->pci_addr = 0;
      request_ps->msg_size = crate_map_size;
      request_ps->msg_p = crate_map_p;
      /*
      ** Transmit request to PCIL and wait for execution to complete.
      */
      stat = PCIL_IO_Xmit_Wait (PCIL_IO_Device_Block_Low_ps, request_ps);
   }  /* if (request_ps != NULL) */
   /*
   ** Release the request_ts & address map.
   */
   if ( request_ps != NULL )
      PCIL_IO_Release_Request (PCIL_IO_Device_Block_Low_ps, request_ps);
   return stat;
}


/*=============================================================================

  Abs:  Load FECC3 CAMAC standard register.

  Name: PCIL_IO_Load_FECC3_CAMAC_Standard

  Args:
       *CAMAC_standard_p
         Use:  Write data for load of FECC3 CAMAC standard register
         Type: u32  
         Acc:  read
         Mech: by reference

  Rem:  Allocate a request_ts and fill it in. Transmit to PCIL.
        Check final status.

  Side: Possible allocation of new request_ts.

  Ret:  # bytes transfered
 
==============================================================================*/

int PCIL_IO_Load_FECC3_CAMAC_Standard (u32   *CAMAC_standard_p)
{
   int           stat= 0;            /* Default to OK */
   request_ts   *request_ps = NULL;  /* pointer to request   */
   /*---------------------------------------------------------------------------*/
   /*
   ** Obtain a request_ts.
   */
   if ( ( request_ps = PCIL_IO_Obtain_Request (PCIL_IO_Device_Block_Low_ps) ) != NULL )
   {
      /*
      ** Fill in the request_ts.
      */
      request_ps->function.opcode = DIAG_FECC;
      request_ps->function.funct_code |= FECC3_LOAD_CRATE_STANDARD;
      request_ps->pci_size = 0;
      request_ps->pci_addr = 0;
      request_ps->msg_size = sizeof(u32  );
      request_ps->msg_p = CAMAC_standard_p;
      /*
      ** Transmit request to PCIL and wait for execution to complete.
      */
      stat = PCIL_IO_Xmit_Wait (PCIL_IO_Device_Block_Low_ps, request_ps);
   }  /* if (request_ps != NULL) */
   /*
   ** Release the request_ts & address map.
   */
   if ( request_ps != NULL )
      PCIL_IO_Release_Request (PCIL_IO_Device_Block_Low_ps, request_ps);
   return stat;
}

/*************************************************
 ** Start driver ioctl, read and write routines
 ************************************************/

/*=============================================================================

  Abs:  I/O control

  Name: pcil_ioctl

  Args: 

  Rem:  

  Ret:  

==============================================================================*/

static int pcil_ioctl (struct inode *inode_ps, struct file  *file_ps, unsigned int cmd, 
                       unsigned long uarg)
{
   int    stat =0;             /* Status returned to user */ 
   u32    boot_ctrl_word = 0;  /* boot control word        */
   /*-------------------------------------------------------------------------*/
   if (!PCIL_Load_OK)  /* Only need driver loaded for ioctl functions */ 
      goto egress;
   switch (cmd) 
   {
      case IOCTL_PCIL_INIT:
      {
	/*************************************************************
         ** Complete initialization of PCIL & FECC, optionally loading 
         ** the pcil and fecc code memories.
         ************************************************************/
         /*-------------- Local data for this function --------------------*/
         ioctl_pcil_init_ts *kioctl_init_ps = NULL; /* Kernel buffer for user's struct */
         u32    pcil_size = 0;
         u32    fecc_size = 0;
         int1u *kpcilbuf_p = NULL;  /* Kernel buffers for pcil & fecc code. */
         int1u *kfeccbuf_p = NULL;
         u32    self_test_status;   /* PCIL self test status */
	 /*
	 ** Clear psuedo-Unibus map register array and free any user wait queues allocated.
	 */
         int    ix;
         for (ix=0; ix < PTEMAX; ix++)
	    vmap_s.pu_ps[ix] = 0;
	 vmap_s.lux = 0;
         for (ix = 0; ix < uwait_s.wux; ix++)
	   kfree (uwait_s.wqcond_paps[ix]);
	 uwait_s.wux=0;
         /*
         ** Optionally load PCIL & FECC memories.
         */
         if (uarg != 0)
         {
            kioctl_init_ps = kmalloc (sizeof(ioctl_pcil_init_ts), GFP_KERNEL);
            copy_from_user (kioctl_init_ps, (char *)uarg, sizeof(ioctl_pcil_init_ts));
            pcil_size = kioctl_init_ps->pcil_size;
            fecc_size = kioctl_init_ps->fecc_size;
            if (pcil_size != 0)
            {
              if (pcil_size != PCIL_PROM_SIZE)
              {
                  stat = -EINVAL;
                  printk (KERN_ERR "PCIL: code buffer wrong size. Is %d s/b %d\n",
                          pcil_size, PCIL_PROM_SIZE);
                  goto init_egress;
               }
               kpcilbuf_p = kmalloc (PCIL_PROM_SIZE, GFP_KERNEL);
               copy_from_user (kpcilbuf_p, kioctl_init_ps->pcil_code_p, PCIL_PROM_SIZE);
               writel (__pa(kpcilbuf_p), PCIL_IO_Code_Buffer_ap + 0);
               boot_ctrl_word |= DOWNLOAD_PCIL;
            }
            if (fecc_size != 0)
            {
               kfeccbuf_p = kmalloc (fecc_size, GFP_KERNEL);
               copy_from_user (kfeccbuf_p, kioctl_init_ps->fecc_code_p, fecc_size);
               writel (fecc_size, PCIL_IO_Code_Buffer_ap + 3);
               writel (__pa(kfeccbuf_p), PCIL_IO_Code_Buffer_ap + 2);
               boot_ctrl_word |= DOWNLOAD_FECC;
            }
         }        /* uarg != 0 */
         /*
         ** Boot the PCIL and FECC.
         */
         writel ( boot_ctrl_word, PCIL_IO_Boot_Control_p);
         /*
         ** Allocate and initialize the communications area. Overlaps with boot.
         */
         PCIL_IO_Comm_Area_ps = (commarea_ts *) kmalloc (sizeof (commarea_ts), GFP_KERNEL);
         memset (PCIL_IO_Comm_Area_ps, 0x0, sizeof (commarea_ts));
         /*
         ** Allocate and initialize the low & high priority device blocks.
         */
         PCIL_IO_Create_Device_Block (&PCIL_IO_Device_Block_Low_ps, 
                                      PCIL_IO_Doorbell_Low_p,
                                      PCIL_IO_Comm_Area_ps->command_ring_low_a,
				      PCIL_IO_Comm_Area_ps->response_ring_low_a);
         PCIL_IO_Create_Device_Block (&PCIL_IO_Device_Block_High_ps, 
                                      PCIL_IO_Doorbell_High_p,
                                      PCIL_IO_Comm_Area_ps->command_ring_high_a,
                                      PCIL_IO_Comm_Area_ps->response_ring_high_a);
         /*
         ** Wait for self test delay
         */
         interruptible_sleep_on_timeout(&sleep_queue, SELF_TEST_DELAY);
         /*
         ** Check self-test status.
         */
         if (( self_test_status = *PCIL_IO_Commun_Init_High_p ) != PORT_STEP_1 )
         {
            stat = -ENODEV;
            printk (KERN_ERR "PCIL: Failed self test with status %x in pcil_ioctl \n", 
                    self_test_status);
            goto init_egress;
         }
         /*
         ** Pass the communication area address to the PCIL and save statically.
         */
         writel (__pa(PCIL_IO_Comm_Area_ps), PCIL_IO_Commun_Init_Low_p);
         PCIL_IO_Comm_Area_a =  __pa(PCIL_IO_Comm_Area_ps);
         /*
         ** Send the host step 1 command (ring lengths and interrupt enable).
         */
         init_wait = 1;  /* Tell ISR this is init and we're waiting */
         writel ( HOST_STEP_1, PCIL_IO_Commun_Init_High_p);
         /*
         ** Wait on low priority queue. Give up if the interrupt doesn't happen quickly.
         */
         interruptible_sleep_on_timeout(&init_queue, INIT_WAIT);
         /*
         ** Check that PCIL has properly echoed the communication area address 
         ** and host step 1 command.
         */
         if ( (readl(PCIL_IO_Commun_Init_Low_p) != PCIL_IO_Comm_Area_a ) ||
              (readl(PCIL_IO_Commun_Init_High_p) != PORT_STEP_2 ) ) 
         {
            stat = -ENODEV;
            printk (KERN_ERR "PCIL: Failed to echo comm area address in pcil_ioctl.\n"); 
            goto init_egress;
         }
         /*
         ** Write pagetable address and send host step 2 command (final "go" command).
         */
         writel (__pa(vmap_s.pu_ps), PCIL_IO_Commun_Init_Low_p);
         PDEBUG ("Psuedo-Unibus Map virt addr = %x physddr = %lx\n", 
                 (u32  )vmap_s.pu_ps, __pa(vmap_s.pu_ps)); 
         writel (HOST_STEP_2, PCIL_IO_Commun_Init_High_p);
         /*
         ** Initialize response rings.
         */
         PCIL_IO_Initialize_Response_Ring (PCIL_IO_Device_Block_Low_ps);
         PCIL_IO_Initialize_Response_Ring (PCIL_IO_Device_Block_High_ps);
         /*
         ** Mark the device online.
         */
         PCIL_Init_Complete = TRUE_T;
         /**
         ** Exit this function. Free all resources.
         */
      init_egress:
         kfree(kioctl_init_ps);
         kfree(kpcilbuf_p);
         kfree(kfeccbuf_p);
         goto egress;
      }
      break;        /* End case: IOCTL_PCIL_INIT */

      /***************************************
      ** Get and possibly map physical address.
      ****************************************/
      case IOCTL_PCIL_GETADDR:
      {
         pcil_addr_ts *kioctl_getaddr_ps = NULL; /* Kernel buffer for user's struct */
         kioctl_getaddr_ps = kmalloc (sizeof(pcil_addr_ts), GFP_KERNEL);
         copy_from_user (kioctl_getaddr_ps, (char *)uarg, sizeof(pcil_addr_ts));
         if ( (stat = PCIL_GetAddr (kioctl_getaddr_ps) < 0 ))
         {
            kfree (kioctl_getaddr_ps);
            goto egress;
	 }
         PDEBUG ("Ioctl GETADDR entered. virtaddr = %x physaddr = %x\n",
                 (u32  ) kioctl_getaddr_ps->virtaddr_p,  (u32) kioctl_getaddr_ps->physaddr);
         copy_to_user ((char *)uarg, kioctl_getaddr_ps, sizeof(pcil_addr_ts));
         kfree (kioctl_getaddr_ps);
      }
      break;        /* End case IOCTL_PCIL_GETADDR */

      /*
      ** Release physical address map.
      */
      case IOCTL_PCIL_RELADDR:
      {
         pcil_addr_ts *kioctl_reladdr_ps = NULL; /* Kernel buffer for user's struct */
         kioctl_reladdr_ps = kmalloc (sizeof(pcil_addr_ts), GFP_KERNEL);
         copy_from_user (kioctl_reladdr_ps, (char *)uarg, sizeof(pcil_addr_ts));
         if ( (stat - PCIL_RelAddr (kioctl_reladdr_ps) < 0 ))
         {
            kfree (kioctl_reladdr_ps);
            goto egress;
	 }
         copy_to_user ((char *)uarg, kioctl_reladdr_ps, sizeof(pcil_addr_ts));
         kfree (kioctl_reladdr_ps);
      }
      break;        /* End case IOCTL_PCIL_RELADDR */

      /*
      ** Allocate our wait queue struct with condition, initialize it and return pointer 
      ** to caller.
      */
      case IOCTL_PCIL_NEWFECCMBX:
      {
         /*
         ** Keep track of user wait queues allocated. Release them in IOCTL_PCIL_INIT
         */
         if (down_interruptible (&uwait_s.semq))   /* Get access to uwait_s */
         {
            printk (KERN_ERR "PCIL: Signal interrupt in IOCTL_PCIL_NEWFECCMBX\n");
            stat = -ERESTARTSYS;
            goto egress;
         }
         if (uwait_s.wux < WAITMAX)
	 {
            wqcond_ts *waitq_ps = kmalloc (sizeof(wqcond_ts), GFP_KERNEL);
	    init_waitqueue_head (&waitq_ps->wq);
            waitq_ps->cond = 0;
            copy_to_user ((void *)uarg, &waitq_ps, sizeof(waitq_ps));
            uwait_s.wqcond_paps[uwait_s.wux++] = waitq_ps;
            up (&uwait_s.semq);
	 }
	 else
	 {
            printk (KERN_ERR "PCIL: Max wait queues allocated in  IOCTL_PCIL_NEWFECCMBX\n");
            stat = -EOVERFLOW;
          }
          up (&uwait_s.semq);
     }
      break;

      /*
      ** Invalid ioctl command.
      */
      default:
      {
         printk (KERN_DEBUG "PCIL: Invalid ioctl command = %x\n", cmd);
         stat = -EINVAL;
      }
      break;
   }               /* End switch */

egress:
   return stat;
}       /* End pcil_ioctl */


/*=============================================================================

  Abs:  Read from specified minor device. 

  Name: pcil_read

  Args: 

  Rem:  

  Ret:  

==============================================================================*/

static ssize_t pcil_read (struct file *file_ps, char *data_p, size_t databytes, 
                          loff_t *filepos)
{
   int     stat =-ENODEV; /* Status returned to user */ 
   mdev_te minor_e;       /* Minor device number as enum */
   /*-------------------------------------------------------------------------*/
   if (!PCIL_Init_Complete) goto egress;
   minor_e = MINOR(file_ps->f_dentry->d_inode->i_rdev);
   PDEBUG ("Read entered with minor id %d\n",minor_e);
   switch (minor_e) 
   {
      case BBUSRD:  /* Read BITbus message as hi priority */
      {
	 parm_buf_len_ts  *pbl_ps = kmalloc (sizeof(parm_buf_len_ts), GFP_KERNEL);
         copy_from_user (pbl_ps, data_p, sizeof(parm_buf_len_ts));
         stat = PCIL_IO_Receive_BITbus (pbl_ps->buf_p, pbl_ps->len, pbl_ps->parm);
	 kfree(pbl_ps);
      }    /* End case BBUSRD */
      break;

      case AMFELO:  /* Read low priority application msg from FECC */
      case AMFEHI:  /* Read high priority application msg from FECC */
      {
	 parm_buf_len_ts  *pbl_ps = kmalloc (sizeof(parm_buf_len_ts), GFP_KERNEL);
         copy_from_user (pbl_ps, data_p, sizeof(parm_buf_len_ts));
	 /*
	 ** The parm in the pbl struct is the timeout value in ticks and buf_p is
	 ** points to the wqcond_ts struct allocated via IOCTL_PCIL_NEWFECCMBX.
         ** len is ignored.
	 */
         wqcond_ts *wqcond_ps = (wqcond_ts *) pbl_ps->buf_p;
	 int timeout = pbl_ps->parm;
	 pcil_wait_event_interruptible_timeout (wqcond_ps->wq, wqcond_ps->cond, timeout);
	 /*
	 ** Check for timeout, signal or buffer overrun.
	 */
         stat = timeout;
	 PDEBUG ("appmsg read timeout = %i\n",timeout);
         if ( (timeout == 0) )
	 {
           stat = -ETIME;    /* Read timed out */
	   printk (KERN_ERR "Timeout in amfelo/amfehi.\n");
	 }
         if ((timeout > 0) && (wqcond_ps->cond > 1))
	 {
	    stat = -EXFULL;
            printk (KERN_ERR "Buffer overrun in amfelo/amfehi.\n");
	 }
         wqcond_ps->cond = 0;
	 kfree (pbl_ps);
      }    /* End case AMFELO & AMFEHI */
      break;

      case GFGCNT:  /* Read (get) fine granularity timer count from PCIL */
      {
         stat = PCIL_IO_Get_Counter ((u32   *) data_p);
      }    /* End case GFGETCNT */
      break;

      case DPCMEM:  /* Read (dump) PCIL memory */
      {
	 parm_buf_len_ts  *pbl_ps = kmalloc (sizeof(parm_buf_len_ts), GFP_KERNEL);
         copy_from_user (pbl_ps, data_p, sizeof(parm_buf_len_ts));
         stat = PCIL_IO_Dump_PCIL_Memory (pbl_ps->buf_p, pbl_ps->len, pbl_ps->parm);
	 kfree(pbl_ps);
      }    /* End case DPCMEM */
      break;

      case GPCVER:  /* Read (get) PCIL hardware/software version  */
      {
         stat = PCIL_IO_Get_PCIL_Version ((u32   *)data_p);
      }    /* End case GPCVER */
      break;

      case GPCCPU:  /* Read (get) PCIL CPU use statistics */
      {
	 parm_buf_len_ts  *pbl_ps = kmalloc (sizeof(parm_buf_len_ts), GFP_KERNEL);
         copy_from_user (pbl_ps, data_p, sizeof(parm_buf_len_ts));
         stat = PCIL_IO_Get_PCIL_CPU_Stats (pbl_ps->buf_p, pbl_ps->len, &pbl_ps->parm);
	 kfree (pbl_ps);
      }    /* End case GPCCPU */
      break;

      case GPCLMC:  /* Read (get) PCIL2 link maintenance counters */
      {
         stat = PCIL_IO_Get_PCIL2_Maintenance_Counters ((int2u *)data_p, databytes);
      }    /* End case GPCLMC */
      break;

      case DFEMEM:  /* Read (dump) FECC memory  */
      {
	 parm_buf_len_ts  *pbl_ps = kmalloc (sizeof(parm_buf_len_ts), GFP_KERNEL);
         copy_from_user (pbl_ps, data_p, sizeof(parm_buf_len_ts));
         stat = PCIL_IO_Dump_FECC_Memory (pbl_ps->buf_p, pbl_ps->len, &pbl_ps->parm);
	 kfree (pbl_ps);
      }    /* End case DFEMEM */
      break;

      case GFEVER:  /* Read (get) FECC hardware/software version */
      {
         stat = PCIL_IO_Get_FECC_Version ((u32   *)data_p);
      }    /* End case GFEVER */
      break;

      case GFECPU:  /* Read (get) FECC CPU use statistics */
      {
	 parm_buf_len_ts  *pbl_ps = kmalloc (sizeof(parm_buf_len_ts), GFP_KERNEL);
         copy_from_user (pbl_ps, data_p, sizeof(parm_buf_len_ts));
         stat = PCIL_IO_Get_FECC_CPU_Stats (pbl_ps->buf_p, pbl_ps->len, &pbl_ps->parm);
	 kfree (pbl_ps);

      }    /* End case GFECPU */
      break;

      case GFELMC:  /* Read (get) FECC3 link maintenance counters */
      {
         stat = PCIL_IO_Get_FECC3_Maintenance_Counters ((int2u *)data_p, databytes);
      }    /* End case GFELMC */
      break;

      /*
      ** Invalid read minor device number.
      */
      default:
         printk (KERN_DEBUG "PCIL: Invalid read minor device number = %x\n", minor_e);
         stat = -EINVAL;
      break;
   }      /* End switch */
egress:
   return stat;
}

/*=============================================================================

  Abs:  Write to specified minor device. 

  Name: pcil_write

  Args: 

  Rem:  

  Ret:  

==============================================================================*/

static ssize_t pcil_write (struct file *file_ps, const char *data_p, size_t databytes, 
                           loff_t *filepos)
{
   int     stat =-ENODEV; /* Status returned to user */ 
   mdev_te minor_e;       /* Minor device number as enum */
   /*-------------------------------------------------------------------------*/
   if (!PCIL_Init_Complete) goto egress;
   minor_e = MINOR(file_ps->f_dentry->d_inode->i_rdev);
   PDEBUG ("Write entered with minor id %d\n",minor_e);
   switch (minor_e) 
   {
      /*
      ** For both MBCD writes we assume that the user has already done all of the conversion
      ** to physical addresses and the physical address is passed as data_p.
      */
      case MBCDLO:  /* Write low priority MBCD message */
      {
         stat = PCIL_IO_Send_MBCD ((u32  ) data_p, databytes, PCIL_IO_Device_Block_Low_ps);
      }    /* End case MBCDLO */
      break;

      case MBCDHI:  /* Write hi priority MBCD message */
      {
         stat = PCIL_IO_Send_MBCD ((u32  ) data_p, databytes, PCIL_IO_Device_Block_High_ps);
      }    /* End case MBCDHI */
      break;

      /*
      ** For Bitbus requests the first word is the unit. We may change this later. !!??
      */
      case BBUSLO:  /* Write low priority BITbus message */
      {
 	 parm_buf_len_ts  *pbl_ps = kmalloc (databytes, GFP_KERNEL);
         copy_from_user (pbl_ps, data_p, databytes);
         stat = PCIL_IO_Send_BITbus (pbl_ps->buf_p, pbl_ps->len, pbl_ps->parm,
                                     PCIL_IO_Device_Block_Low_ps);
	 kfree (pbl_ps);
      }    /* End case BBUSLO */
      break;

      case BBUSHI:  /* Write hi priority BITbus message */
      {
 	 parm_buf_len_ts  *pbl_ps = kmalloc (databytes, GFP_KERNEL);
         copy_from_user (pbl_ps, data_p, databytes);
         stat = PCIL_IO_Send_BITbus (pbl_ps->buf_p, pbl_ps->len, pbl_ps->parm,
                              PCIL_IO_Device_Block_High_ps);
	 kfree (pbl_ps);
      }    /* End case MBCDHI */
      break;

      case AMFELO:  /* Write low priority application msg to FECC */
      {
         stat = PCIL_IO_Send_Appmsg ((void *)data_p, databytes, PCIL_IO_Device_Block_Low_ps);
      }    /* End case AMFELO */
      break;

      case AMFEHI:  /* Write high priority application msg to FECC */
      {
         stat = PCIL_IO_Send_Appmsg ((void *)data_p, databytes, PCIL_IO_Device_Block_High_ps);
      }    /* End case AMFEHI */
      break;

      case SNDPAT:  /* Write (Send) pattern */
      {
         stat = PCIL_IO_Send_Pattern ((u32   *)data_p);
      }    /* End case SNDPAT */
      break;

      case LDFECR:  /* Write (load) FECC command register */
      {
         stat = PCIL_IO_Load_FECC_Command ((u32   *)data_p);
     }    /* End case LDFECR */
      break;

      case LDLLIR:  /* Write (load) local link initialization register */
      {
         stat = PCIL_IO_Load_Local_Link_Initialization ((u32   *)data_p);
      }    /* End case LDLLIR */
      break;

      case LDRLIR:  /* Write (load) remote link initialization register */
      {
         stat = PCIL_IO_Load_Remote_Link_Initialization ((u32   *)data_p);
      }    /* End case LDRLIR */
      break;

      case LFEMEM:  /* Write (load) FECC memory */
      {
 	 parm_buf_len_ts  *pbl_ps = kmalloc (databytes, GFP_KERNEL);
         copy_from_user (pbl_ps, data_p, databytes);
         stat = PCIL_IO_Load_FECC_Memory (pbl_ps->buf_p, pbl_ps->len, pbl_ps->parm);
	 kfree (pbl_ps);
      }    /* End case LFEMEM */
      break;

      case LFECCM:  /* Write (load) FECC CAMAC crate map */
      {
         stat = PCIL_IO_Load_FECC_Crate_Map ((u32   *)data_p, databytes);
      }    /* End case LFECCM */
      break;

      case LFECSR:  /* Write (load) FECC3 CAMAC standard register */
      {
         stat = PCIL_IO_Load_FECC3_CAMAC_Standard ((u32   *)data_p);
      }    /* End case LFECSR */
      break;

      /*
      ** Invalid write minor device number.
      */
      default:
         printk (KERN_DEBUG "PCIL: Invalid write minor device number = %x in pcil_write\n", 
                 minor_e);
         stat = -EINVAL;
      break;
   }      /* End switch */
egress:
   return stat;
}

/****************************************************************************
 **
 ** Our fops struct. Declare after routines but before register_chrdev.
 **
 ***************************************************************************/

static struct file_operations pcil_fops =
{
  read:pcil_read,
  write:pcil_write,
  ioctl:pcil_ioctl,
};


/*==========================================================================

  Name:pcil_init 

  Abs: Called when module is first loaded.
  
  Args: None    
                                                       
  Rem: Perform all PCI device initialization.
                                                                                       
  Ret:  int linux error code
=============================================================================*/
static int __init pcil_init (void)
{
   int             stat = 0;             /* Status */
   u32             boot_ctrl_word = 0;   /* boot control word      */
   u32             self_test_status;     /* PCIL self test status  */
   int             regresult;            /* register_chrdev result */
   /*-------------------------------------------------------------------------*/
   PDEBUG ("Entered PCIL init_module\n");
   /*
   ** Locate the PCIL device on the PCI bus.
   */  
   if ( (pcil_device_ps = pci_find_device (PCIL_VENDOR,  PCIL2_DEVICE_ID, 
                                           pcil_device_ps)) == NULL )
   {
      stat = -ENODEV;
      printk (KERN_ERR "PCIL: No device found\n");
      goto egress;
   }
   /*
   ** Enable the device. Returns 0 (false) if OK.
   */
   if(pci_enable_device (pcil_device_ps))
   {
      stat = -ENODEV;
      printk (KERN_ERR "PCIL: Failed to enable.\n");
      goto egress;
   }
   /*
   ** We only support the PCIL2 device.
   */
   if (pcil_device_ps->device != PCIL2_DEVICE_ID)
   {
      stat = -ENODEV;
      printk (KERN_ERR "PCIL: Device not PCIL2 but is %d\n", pcil_device_ps->device);
      goto egress;
   }
   /*
   ** Allocate and map linear address of PCIL2 registers from base address 0 in config space
   ** and map to a virtual address. Assign that virtual address to a struct of registers.
   */
   request_mem_region(pci_resource_start(pcil_device_ps, 0), 
                      pci_resource_len(pcil_device_ps,0), "pcil"); 
   PCIL_IO_Registers2_ps = ioremap(pci_resource_start(pcil_device_ps, 0),
                                   pci_resource_len(pcil_device_ps, 0)); 
   /*
   ** Initialize register pointers for PCIL hardware.
   */
   PCIL_IO_Doorbell_Low_p = &PCIL_IO_Registers2_ps->doorbell_low;
   PCIL_IO_Doorbell_High_p = &PCIL_IO_Registers2_ps->doorbell_high;
   PCIL_IO_Doorbell_Pattern_p = &PCIL_IO_Registers2_ps->doorbell_pattern;
   PCIL_IO_Interrupt_Type_p = &PCIL_IO_Registers2_ps->doorbell_low;
   PCIL_IO_Acknowledge_Low_p = &PCIL_IO_Registers2_ps->acknowledge_low;
   PCIL_IO_Acknowledge_High_p = &PCIL_IO_Registers2_ps->acknowledge_high;
   PCIL_IO_Boot_Control_p = &PCIL_IO_Registers2_ps->boot_control;
   PCIL_IO_Commun_Init_Low_p = &PCIL_IO_Registers2_ps->commun_init_low;
   PCIL_IO_Commun_Init_High_p = &PCIL_IO_Registers2_ps->commun_init_high;
   PCIL_IO_Code_Buffer_ap = PCIL_IO_Registers2_ps->code_buffer;
   PCIL_IO_Pattern_ap = PCIL_IO_Registers2_ps->pattern;
   PCIL_IO_Counter_p = &PCIL_IO_Registers2_ps->acknowledge_low;
   PCIL_IO_Pattern_Length = PATTERN_LENGTH;
   /*
   ** Boot the PCIL and FECC.
   */
   writel (boot_ctrl_word, PCIL_IO_Boot_Control_p);
   /*
   ** Wait for PCIL-FECC initialization and self-test.
   */
   interruptible_sleep_on_timeout(&sleep_queue, SELF_TEST_DELAY);
   /*
   ** Check self-test status.
   */
   if ( ( self_test_status = readl(PCIL_IO_Commun_Init_High_p) ) != PORT_STEP_1 )
   {
      stat = -ENODEV;
      release_mem_region(pci_resource_start(pcil_device_ps, 0), 
                         pci_resource_len(pcil_device_ps,0)); 
      printk (KERN_ERR "PCIL: In pcil_init failed self test with status %x\n", 
              self_test_status);
      goto egress;
   }
   /*
   ** Save the hardware/software version number and clear any outstanding interrupts.
   */
   PCIL_IO_PCIL_Device_Version = readl(PCIL_IO_Interrupt_Type_p);
   writel (0, PCIL_IO_Acknowledge_Low_p);
   writel (0, PCIL_IO_Acknowledge_High_p);
   /*
   ** Get interrupt level and set up our handler.
   */
   if (request_irq (pcil_device_ps->irq, pcil_isr, SA_SHIRQ, "pcil", &dev_id ))
   {
      stat = -ENODEV;
      printk (KERN_ERR "PCIL: Failed to enable isr.\n");
      goto egress;
   }
   /*
   ** Get pages for psuedo-Unibus mapping registers and init struct to manage them.
   */
   if (!(vmap_s.pu_ps = (void *) __get_free_pages(GFP_KERNEL, PTEPWR) ))
   {
      release_mem_region(pci_resource_start(pcil_device_ps, 0), 
                         pci_resource_len(pcil_device_ps,0)); 
      printk (KERN_ERR "PCIL: Cannot allocate psuedo-Unibus mapping pages\n");
      stat = -ENODEV;
      goto egress;
   }
   memset (vmap_s.pu_ps, 0, PAGE_SIZE << PTEPWR);
   vmap_s.lux = 0;
   sema_init (&vmap_s.pusem, 1);
   /*
   ** Get array of pointers to user wait queues for appmsg and init struct to manage them.
   */
   uwait_s.wqcond_paps = kmalloc (WAITMAX * sizeof(uwait_s.wqcond_paps), GFP_KERNEL);
   uwait_s.wux = 0;
   sema_init (&uwait_s.semq, 1);
   /*
   ** Register our device to the system.
   */
   if ((regresult = register_chrdev (PCIL_MAJOR, "pcil", &pcil_fops)) < 0)
   {
      stat = regresult;
      release_mem_region(pci_resource_start(pcil_device_ps, 0), 
                         pci_resource_len(pcil_device_ps,0)); 
      printk (KERN_ERR "PCIL: Failed to register device. Error code = %d\n", regresult);
      goto egress;
   }
   PCIL_Load_OK = TRUE_T;
egress:
   return stat;
}

/*==========================================================================
                                                                                       
  Name:pcil_cleanup
                                                                                       
  Abs: Called when module is unloaded.
                                        
  Args: None

  Rem: Cleanup any resources allocated.
                                                                                       
  Ret:  None
=============================================================================*/
static void __exit pcil_cleanup(void)
{
   PDEBUG ("Unloading  PCIL driver\n");
   release_mem_region(pci_resource_start(pcil_device_ps, 0), 
                      pci_resource_len(pcil_device_ps,0)); 
   kfree (PCIL_IO_Comm_Area_ps);
   free_irq (pcil_device_ps->irq, &dev_id);
   free_pages((unsigned long)vmap_s.pu_ps, PTEPWR);
   kfree (uwait_s.wqcond_paps);

   /*
   ** Free some of the memory allocated for the device blocks. I know I don't
   ** free everything from the device block so there will be some lost memory. 
   ** Hopefully we won't be unloading the driver much once we're in production!
   */
   if ( PCIL_Init_Complete)
   {
      kfree (PCIL_IO_Device_Block_Low_ps->flow_array_ps);
      kfree (PCIL_IO_Device_Block_High_ps->flow_array_ps);
      kfree (PCIL_IO_Device_Block_Low_ps);
      kfree (PCIL_IO_Device_Block_High_ps);
   }
   unregister_chrdev (PCIL_MAJOR, "pcil");
}

/*
** Declare our init and cleanup routines.
*/
module_init (pcil_init);
module_exit (pcil_cleanup);
