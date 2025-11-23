#include "types.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "mmu.h"
#include "proc.h"
#include "x86.h"
#include "traps.h"
#include "spinlock.h"


extern int page_allocator_type;
int mappages(pde_t *pgdir, void *va, uint size, uint pa, int perm);
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

// CS 3320 project 2: lazy page fault handler
if (tf->trapno == T_PGFLT) {
    uint faultva = rcr2();                // faulting virtual address
    struct proc *p = proc;

    // Only handle user-mode lazy allocation
    if (p && (tf->cs & 3) == DPL_USER && page_allocator_type == 1) {

        // Valid lazy fault range:
        //   - at or below heap break (inclusive)
        //   - above NULL page
        //   - below kernel memory
        if (faultva <= p->sz && faultva >= PGSIZE && faultva < KERNBASE) {

            void *va = (void*)PGROUNDDOWN(faultva);
            char *mem = kalloc();

            if (mem == 0) {
                cprintf("lazy alloc: out of memory\n");
                p->killed = 1;
            } else {
                memset(mem, 0, PGSIZE);

                if (mappages(p->pgdir, va, PGSIZE, V2P(mem), PTE_W|PTE_U) < 0) {
                    cprintf("lazy alloc: mappages failed\n");
                    kfree(mem);
                    p->killed = 1;
                } else {
                    switchuvm(p);
                    return;             // retry the instruction
                }
            }
        }
    }

    // Not handled lazily
    cprintf("Unhandled page fault for va:0x%x!\n", faultva);
}


  // ... rest of your switch statement remains the same ...
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

  default:
    if(proc == 0 || (tf->cs&3) == 0){
      cprintf("unexpected trap %d from cpu %d eip %x (cr2=0x%x)\n",
              tf->trapno, cpu->id, tf->eip, rcr2());
      panic("trap");
    }

    cprintf("pid %d %s: trap %d err %d on cpu %d "
            "eip 0x%x addr 0x%x--kill proc\n",
            proc->pid, proc->name, tf->trapno, tf->err,
            cpu->id, tf->eip, rcr2());
    proc->killed = 1;
  }

  if(proc && proc->killed && (tf->cs&3) == DPL_USER)
    exit();

  if(proc && proc->state == RUNNING && tf->trapno == T_IRQ0+IRQ_TIMER)
    yield();

  if(proc && proc->killed && (tf->cs&3) == DPL_USER)
    exit();
}
