#include "types.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "mmu.h"
#include "proc.h"
#include "x86.h"
#include "traps.h"
#include "spinlock.h"

#define TLBSIZE 2

extern pte_t * walkpgdir(pde_t *, const void *, int);

// Interrupt descriptor table (shared by all CPUs).
struct gatedesc idt[256];
extern uint vectors[];  // in vectors.S: array of 256 entry pointers
struct spinlock tickslock;
uint ticks;

pte_t * tlb[TLBSIZE];

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

    //TODO - sometimes crashed when using alot of ls.

    void * va;            // Virtual address of the sought after page,
    pte_t * kernel_pte;   // Kernel Page Directory correspondong to va.
    pte_t * proc_pte;     // Process Page Directory correspondong to va.

    va = (void*)rcr2();   // va is taken from cr2 register.

    if ((proc_pte = walkpgdir(proc->pgdir, va, 0)) == 0){ // Get page table address in process page dir.
      panic("bad memory access");
    }

    kernel_pte = walkpgdir(cpu->kpgdir, va, 1); // Get page table address in kerner page dir(or allocate).
    
    // Enforce only 2 possible pages at all times using fifo.
    if((int)tlb[0] & PTE_P){ 
        char * va = p2v(PTE_ADDR(tlb[0]));
        kfree(va);
      }
      tlb[0] = 0;

      int index;
      for (index=0; index<TLBSIZE-1; index++){
        tlb[index] = tlb[index+1];
      }
    
    tlb[TLBSIZE] = kernel_pte;

    int i;
    int c=0;
    for (i=0; i<NPDENTRIES/2; i++){
      if (cpu->kpgdir[i]) c++;
    }

    // Copy the process page over to the kernel page.
    memmove((void*)kernel_pte, (void*)proc_pte, NPDENTRIES);
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
