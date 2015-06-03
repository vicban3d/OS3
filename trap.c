#include "types.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "mmu.h"
#include "proc.h"
#include "x86.h"
#include "traps.h"
#include "spinlock.h"

// TODO - remove the memory allocation in sbrk().
// TODO - add the allocation of new pages in the trap handler.
// TODO - move the user virtual memory allocation to the trap handler(follow sbrk() to find the code).
// TODO - only map the single page that caused the fault.
// TODO - when page allocation fails kill the process.
// TODO - allow copying non-present pages and page tables in fork().
// TODO - allow sbrk() to be called with negative values to deallocate.
// TODO - move process size limitation from growproc() to sbrk().
// TODO - perform regular handling when va > proc->sz.
// TODO - perform regular handling when va is in page below the process stack.

// TODO TIP - In copyuvm there is a panic("copyuvm: page not present")... should we delete this line and just continue copying
// TODO TIP - differentiate between task 1 and 2: you should check if it is mapped in the process or not.
// TODO TIP - if malloc returns -1.

kill the process.
extern pte_t * walkpgdir(pde_t *, void *, int);

// Interrupt descriptor table (shared by all CPUs).
struct gatedesc idt[256];
extern uint vectors[];  // in vectors.S: array of 256 entry pointers
struct spinlock tickslock;
uint ticks;

void
tvinit(void)
{
  int i;

  for(i = 0; i < 256; i++)
    SETGATE(idt[i], 0, SEG_KCODE<<3, vectors[i], 0);
  SETGATE(idt[T_SYSCALL], 1, SEG_KCODE<<3, vectors[T_SYSCALL], DPL_USER);
  
  initlock(&tickslock, "time");
}

void
idtinit(void)
{
  lidt(idt, sizeof(idt));
}

//PAGEBREAK: 41
void
trap(struct trapframe *tf)
{
  if(tf->trapno == T_SYSCALL){
    if(proc->killed)
      exit();
    proc->tf = tf;
    syscall();
    if(proc->killed)
      exit();
    return;
  }

  switch(tf->trapno){
  case T_IRQ0 + IRQ_TIMER:
    if(cpu->id == 0){
      acquire(&tickslock);
      ticks++;
      wakeup(&ticks);
      release(&tickslock);
    }
    lapiceoi();
    break;
  case T_IRQ0 + IRQ_IDE:
    ideintr();
    lapiceoi();
    break;
  case T_IRQ0 + IRQ_IDE+1:
    // Bochs generates spurious IDE1 interrupts.
    break;
  case T_IRQ0 + IRQ_KBD:
    kbdintr();
    lapiceoi();
    break;
  case T_IRQ0 + IRQ_COM1:
    uartintr();
    lapiceoi();
    break;
  case T_IRQ0 + 7:
  case T_IRQ0 + IRQ_SPURIOUS:
    cprintf("cpu%d: spurious interrupt at %x:%x\n",
            cpu->id, tf->cs, tf->eip);
    lapiceoi();
    break;

    case T_PGFLT: // Handle Page Fault Exception.
  { 
    //cprintf("[%p][%p]\n", tlb[0], tlb[1]);
    pushcli();
    uint va;              // Virtual address of the sought after page,
    pte_t * kernel_pte;     // Kernel Page Directory correspondong to va.
    pte_t * proc_pte;       // Process Page Directory correspondong to va.
    pte_t * old_kernel_pte; // the pte of the oldest saved entry.
    uint old_va;      // the va of the oldest saved entry.
    
    va = rcr2();    // va of the faulty page table is saved in cr2 register.

    // Enforce only TLBSIZE possible pages at all times using FIFO
    // tlb[0] alway containt the va of the entry to remove.

    if (cpu->tlb_pages < 2){
      cpu->tlb_pages++;
    }
    else {
      old_va = cpu->tlb[0];   // va of the next page to be removed from tlb.
    if ((old_kernel_pte = walkpgdir(cpu->kpgdir, (void *) old_va, 0)) != 0) {         
      *old_kernel_pte = 0;
    }     
    else{
      pte_t *pde;
      pde = &cpu->kpgdir[PDX(old_va)];
      char * v = p2v(PTE_ADDR(*pde));
      kfree(v);
    }
  }

    // Shift FIFO queue.
    int index;
    for (index=0; index<TLBSIZE-1; index++){
       cpu->tlb[index] = cpu->tlb[index+1];
    }

  cpu->tlb[TLBSIZE-1] = va; // Store found va at the end of the queue.

  // Get page table address in process page dir.
    if ((proc_pte = walkpgdir(proc->pgdir, (void *) va, 0)) == 0){ 
      panic("bad process memory access");
    }

    // Allocate new entry in kernel page dir.
    if ((kernel_pte = walkpgdir(cpu->kpgdir, (void *) va, 1)) == 0){ 
      panic("bad kernel memory allocation");
    }

    //cprintf("ALLOCATED AT PDX %p!\n", PDX(va));
         
    *kernel_pte = *proc_pte;

    popcli();
    break;
  }
   
  //PAGEBREAK: 13
  default:
    if(proc == 0 || (tf->cs&3) == 0){
      // In kernel, it must be our mistake.
      cprintf("unexpected trap %d from cpu %d eip %x (cr2=0x%x)\n",
              tf->trapno, cpu->id, tf->eip, rcr2());
      panic("trap");
    }
    // In user space, assume process misbehaved.
    cprintf("pid %d %s: trap %d err %d on cpu %d "
            "eip 0x%x addr 0x%x--kill proc\n",
            proc->pid, proc->name, tf->trapno, tf->err, cpu->id, tf->eip, 
            rcr2());
    proc->killed = 1;
  }

  // Force process exit if it has been killed and is in user space.
  // (If it is still executing in the kernel, let it keep running 
  // until it gets to the regular system call return.)
  if(proc && proc->killed && (tf->cs&3) == DPL_USER)
    exit();

  // Force process to give up CPU on clock tick.
  // If interrupts were on while locks held, would need to check nlock.
  if(proc && proc->state == RUNNING && tf->trapno == T_IRQ0+IRQ_TIMER)
    yield();

  // Check if the process has been killed since we yielded
  if(proc && proc->killed && (tf->cs&3) == DPL_USER)
    exit();
}
