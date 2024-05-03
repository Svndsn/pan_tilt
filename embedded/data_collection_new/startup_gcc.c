//*****************************************************************************
// startup_gcc.c - Startup code for use with GNU tools.
//*****************************************************************************

#include <stdint.h>

//*****************************************************************************
//
// Forward declaration of the default fault handlers.
//
//*****************************************************************************
void ResetISR(void);
static void NmiSR(void);
static void FaultISR(void);
static void IntDefaultHandler(void);

//*****************************************************************************
//
// The entry point for the application.
//
//*****************************************************************************
extern int main(void);

//*****************************************************************************
//
// Add handlers for the various interrupt sources.
//
//*****************************************************************************
extern void SysTick_handler(void);
extern void GPIOA_handler();
extern void GPIOB_handler();
extern void GPIOC_handler();
extern void GPIOD_handler();
extern void GPIOE_handler();
extern void UART0_handler();
extern void UART1_handler();
extern void SSI0_handler();
extern void I2C0_handler();
extern void PWM0_FAULT_handler();
extern void PWM0_0_handler();
extern void PWM0_1_handler();
extern void PWM0_2_handler();
extern void QEI0_handler();
extern void ADC0SS0_handler();
extern void ADC0SS1_handler();
extern void ADC0SS2_handler();
extern void ADC0SS3_handler();
extern void WATCHDOG_handler();
extern void TIMER0A_handler();
extern void TIMER0B_handler();
extern void TIMER1A_handler();
extern void TIMER1B_handler();
extern void TIMER2A_handler();
extern void TIMER2B_handler();
extern void COMP0_handler();
extern void COMP1_handler();
extern void SYSCTL_handler();
extern void FLASH_handler();
extern void GPIOF_handler();
extern void UART2_handler();
extern void SSI1_handler();
extern void TIMER3A_handler();
extern void TIMER3B_handler();
extern void I2C1_handler();
extern void QEI1_handler();
extern void CAN0_handler();
extern void CAN1_handler();
extern void HIBERNATE_handler();
extern void USB0_handler();
extern void PWM0_3_handler();
extern void UDMA_handler();
extern void UDMAERR_handler();
extern void ADC1SS0_handler();
extern void ADC1SS1_handler();
extern void ADC1SS2_handler();
extern void ADC1SS3_handler();
extern void SSI2_handler();
extern void SSI3_handler();
extern void UART3_handler();
extern void UART4_handler();
extern void UART5_handler();
extern void UART6_handler();
extern void UART7_handler();
extern void I2C2_handler();
extern void I2C3_handler();
extern void TIMER4A_handler();
extern void TIMER4B_handler();
extern void TIMER5A_handler();
extern void TIMER5B_handler();
extern void WTIMER0A_handler();
extern void WTIMER0B_handler();
extern void WTIMER1A_handler();
extern void WTIMER1B_handler();
extern void WTIMER2A_handler();
extern void WTIMER2B_handler();
extern void WTIMER3A_handler();
extern void WTIMER3B_handler();
extern void WTIMER4A_handler();
extern void WTIMER4B_handler();
extern void WTIMER5A_handler();
extern void WTIMER5B_handler();
extern void SYSEXC_handler();
extern void PWM1_0_handler();
extern void PWM1_1_handler();
extern void PWM1_2_handler();
extern void PWM1_3_handler();
extern void PWM1_FAULT();

//*****************************************************************************
//
// Reserve space for the system stack.
//
//*****************************************************************************
static unsigned long pulStack[64];

//*****************************************************************************
//
// The vector table.  Note that the proper constructs must be placed on this to
// ensure that it ends up at physical address 0x0000.0000.
//
//*****************************************************************************
__attribute__((section(".isr_vector"))) void (*const g_pfnVectors[])(void) = {
    (void (*)(void))((unsigned long)pulStack + sizeof(pulStack)),
    // The initial stack pointer
    ResetISR,           // The reset handler
    NmiSR,              // The NMI handler
    FaultISR,           // The hard fault handler
    IntDefaultHandler,  // The MPU fault handler
    IntDefaultHandler,  // The bus fault handler
    IntDefaultHandler,  // The usage fault handler
    0,                  // Reserved
    0,                  // Reserved
    0,                  // Reserved
    0,                  // Reserved
    IntDefaultHandler,  // SVCall handler
    IntDefaultHandler,  // Debug monitor handler
    0,                  // Reserved
    IntDefaultHandler,  // The PendSV handler
    SysTick_handler,    // The SysTick handler
    GPIOA_handler,      // GPIO Port A
    GPIOB_handler,      // GPIO Port B
    GPIOC_handler,      // GPIO Port C
    GPIOD_handler,      // GPIO Port D
    GPIOE_handler,      // GPIO Port E
    UART0_handler,      // UART0 Rx and Tx
    UART1_handler,      // UART1 Rx and Tx
    SSI0_handler,       // SSI0 Rx and Tx
    I2C0_handler,       // I2C0 Master and Slave
    PWM0_FAULT_handler, // PWM Fault
    PWM0_0_handler,     // PWM Generator 0
    PWM0_1_handler,     // PWM Generator 1
    PWM0_2_handler,     // PWM Generator 2
    QEI0_handler,       // Quadrature Encoder 0
    ADC0SS0_handler,    // ADC Sequence 0
    ADC0SS1_handler,    // ADC Sequence 1
    ADC0SS2_handler,    // ADC Sequence 2
    ADC0SS3_handler,    // ADC Sequence 3
    WATCHDOG_handler,   // Watchdog timer
    TIMER0A_handler,    // Timer 0 subtimer A
    TIMER0B_handler,    // Timer 0 subtimer B
    TIMER1A_handler,    // Timer 1 subtimer A
    TIMER1B_handler,    // Timer 1 subtimer B
    TIMER2A_handler,    // Timer 2 subtimer A
    TIMER2B_handler,    // Timer 2 subtimer B
    COMP0_handler,      // Analog Comparator 0
    COMP1_handler,      // Analog Comparator 1
    IntDefaultHandler,  // Analog Comparator 2
    SYSCTL_handler,     // System Control (PLL, OSC, BO)
    FLASH_handler,      // FLASH Control
    GPIOF_handler,      // GPIO Port F
    IntDefaultHandler,  // GPIO Port G
    IntDefaultHandler,  // GPIO Port H
    UART2_handler,      // UART2 Rx and Tx
    SSI1_handler,       // SSI1 Rx and Tx
    TIMER3A_handler,    // Timer 3 subtimer A
    TIMER3B_handler,    // Timer 3 subtimer B
    I2C1_handler,       // I2C1 Master and Slave
    QEI1_handler,       // Quadrature Encoder 1
    CAN0_handler,       // CAN0
    CAN1_handler,       // CAN1
    IntDefaultHandler,  // CAN2
    IntDefaultHandler,  // Ethernet
    IntDefaultHandler,  // Hibernate
    USB0_handler,       // USB0
    PWM0_3_handler,     // PWM Generator 3
    UDMA_handler,       // uDMA Software Transfer
    UDMAERR_handler,    // uDMA Error
    ADC1SS0_handler,    // ADC1 Sequence 0
    ADC1SS1_handler,    // ADC1 Sequence 1
    ADC1SS2_handler,    // ADC1 Sequence 2
    ADC1SS3_handler,    // ADC1 Sequence 3
    IntDefaultHandler,  // I2S0
    IntDefaultHandler,  // External Bus Interface 0
    IntDefaultHandler,  // GPIO Port J
    IntDefaultHandler,  // GPIO Port K
    IntDefaultHandler,  // GPIO Port L
    SSI2_handler,       // SSI2 Rx and Tx
    SSI3_handler,       // SSI3 Rx and Tx
    UART3_handler,      // UART3 Rx and Tx
    UART4_handler,      // UART4 Rx and Tx
    UART5_handler,      // UART5 Rx and Tx
    UART6_handler,      // UART6 Rx and Tx
    UART7_handler,      // UART7 Rx and Tx
    0,                  // Reserved
    0,                  // Reserved
    0,                  // Reserved
    0,                  // Reserved
    I2C2_handler,       // I2C2 Master and Slave
    I2C3_handler,       // I2C3 Master and Slave
    TIMER4A_handler,    // Timer 4 subtimer A
    TIMER4B_handler,    // Timer 4 subtimer B
    0,                  // Reserved
    0,                  // Reserved
    0,                  // Reserved
    0,                  // Reserved
    0,                  // Reserved
    0,                  // Reserved
    0,                  // Reserved
    0,                  // Reserved
    0,                  // Reserved
    0,                  // Reserved
    0,                  // Reserved
    0,                  // Reserved
    0,                  // Reserved
    0,                  // Reserved
    0,                  // Reserved
    0,                  // Reserved
    0,                  // Reserved
    0,                  // Reserved
    0,                  // Reserved
    0,                  // Reserved
    TIMER5A_handler,    // Timer 5 subtimer A
    TIMER5B_handler,    // Timer 5 subtimer B
    WTIMER0A_handler,   // Wide Timer 0 subtimer A
    WTIMER0B_handler,   // Wide Timer 0 subtimer B
    WTIMER1A_handler,   // Wide Timer 1 subtimer A
    WTIMER1B_handler,   // Wide Timer 1 subtimer B
    WTIMER2A_handler,   // Wide Timer 2 subtimer A
    WTIMER2B_handler,   // Wide Timer 2 subtimer B
    WTIMER3A_handler,   // Wide Timer 3 subtimer A
    WTIMER3B_handler,   // Wide Timer 3 subtimer B
    WTIMER4A_handler,   // Wide Timer 4 subtimer A
    WTIMER4B_handler,   // Wide Timer 4 subtimer B
    WTIMER5A_handler,   // Wide Timer 5 subtimer A
    WTIMER5B_handler,   // Wide Timer 5 subtimer B
    IntDefaultHandler,  // FPU
    IntDefaultHandler,  // PECI 0
    IntDefaultHandler,  // LPC 0
    IntDefaultHandler,  // I2C4 Master and Slave
    IntDefaultHandler,  // I2C5 Master and Slave
    IntDefaultHandler,  // GPIO Port M
    IntDefaultHandler,  // GPIO Port N
    IntDefaultHandler,  // Quadrature Encoder 2
    IntDefaultHandler,  // Fan 0
    0,                  // Reserved
    IntDefaultHandler,  // GPIO Port P (Summary or P0)
    IntDefaultHandler,  // GPIO Port P1
    IntDefaultHandler,  // GPIO Port P2
    IntDefaultHandler,  // GPIO Port P3
    IntDefaultHandler,  // GPIO Port P4
    IntDefaultHandler,  // GPIO Port P5
    IntDefaultHandler,  // GPIO Port P6
    IntDefaultHandler,  // GPIO Port P7
    IntDefaultHandler,  // GPIO Port Q (Summary or Q0)
    IntDefaultHandler,  // GPIO Port Q1
    IntDefaultHandler,  // GPIO Port Q2
    IntDefaultHandler,  // GPIO Port Q3
    IntDefaultHandler,  // GPIO Port Q4
    IntDefaultHandler,  // GPIO Port Q5
    IntDefaultHandler,  // GPIO Port Q6
    IntDefaultHandler,  // GPIO Port Q7
    IntDefaultHandler,  // GPIO Port R
    IntDefaultHandler,  // GPIO Port S
    PWM1_0_handler,     // PWM 1 Generator 0
    PWM1_1_handler,     // PWM 1 Generator 1
    PWM1_2_handler,     // PWM 1 Generator 2
    PWM1_3_handler,     // PWM 1 Generator 3
    PWM1_FAULT,         // PWM 1 Fault
};

//*****************************************************************************
//
// The following are constructs created by the linker, indicating where the
// the "data" and "bss" segments reside in memory.  The initializers for the
// for the "data" segment resides immediately following the "text" segment.
//
//*****************************************************************************
extern unsigned long _etext;
extern unsigned long _data;
extern unsigned long _edata;
extern unsigned long _bss;
extern unsigned long _ebss;

//*****************************************************************************
//
// This is the code that gets called when the processor first starts execution
// following a reset event.  Only the absolutely necessary set is performed,
// after which the application supplied entry() routine is called.  Any fancy
// actions (such as making decisions based on the reset cause register, and
// resetting the bits in that register) are left solely in the hands of the
// application.
//
//*****************************************************************************
void ResetISR(void) {
  unsigned long *pulSrc, *pulDest;

  //
  // Copy the data segment initializers from flash to SRAM.
  //
  pulSrc = &_etext;
  for (pulDest = &_data; pulDest < &_edata;) {
    *pulDest++ = *pulSrc++;
  }

  //
  // Zero fill the bss segment.
  //
  __asm("    ldr     r0, =_bss\n"
        "    ldr     r1, =_ebss\n"
        "    mov     r2, #0\n"
        "    .thumb_func\n"
        "zero_loop:\n"
        "        cmp     r0, r1\n"
        "        it      lt\n"
        "        strlt   r2, [r0], #4\n"
        "        blt     zero_loop");

  //
  // Call the application's entry point.
  //
  main();
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a NMI.  This
// simply enters an infinite loop, preserving the system state for examination
// by a debugger.
//
//*****************************************************************************
static void NmiSR(void) {
  //
  // Enter an infinite loop.
  //
  while (1) {
  }
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a fault
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void FaultISR(void) {
  //
  // Enter an infinite loop.
  //
  while (1) {
  }
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives an unexpected
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void IntDefaultHandler(void) {
  //
  // Go into an infinite loop.
  //
  while (1) {
  }
}

//*****************************************************************************
//
// Add default handlers for the following interrupts.
// Meant to be overridden by the application.
//
//*****************************************************************************
__attribute__((weak)) void SysTick_handler() {
  while (1) {
  }
}
__attribute__((weak)) void GPIOA_handler() {
  while (1) {
  }
}
__attribute__((weak)) void GPIOB_handler() {
  while (1) {
  }
}
__attribute__((weak)) void GPIOC_handler() {
  while (1) {
  }
}
__attribute__((weak)) void GPIOD_handler() {
  while (1) {
  }
}
__attribute__((weak)) void GPIOE_handler() {
  while (1) {
  }
}
__attribute__((weak)) void UART0_handler() {
  while (1) {
  }
}
__attribute__((weak)) void UART1_handler() {
  while (1) {
  }
}
__attribute__((weak)) void SSI0_handler() {
  while (1) {
  }
}
__attribute__((weak)) void I2C0_handler() {
  while (1) {
  }
}
__attribute__((weak)) void PWM0_FAULT_handler() {
  while (1) {
  }
}
__attribute__((weak)) void PWM0_0_handler() {
  while (1) {
  }
}
__attribute__((weak)) void PWM0_1_handler() {
  while (1) {
  }
}
__attribute__((weak)) void PWM0_2_handler() {
  while (1) {
  }
}
__attribute__((weak)) void QEI0_handler() {
  while (1) {
  }
}
__attribute__((weak)) void ADC0SS0_handler() {
  while (1) {
  }
}
__attribute__((weak)) void ADC0SS1_handler() {
  while (1) {
  }
}
__attribute__((weak)) void ADC0SS2_handler() {
  while (1) {
  }
}
__attribute__((weak)) void ADC0SS3_handler() {
  while (1) {
  }
}
__attribute__((weak)) void WATCHDOG_handler() {
  while (1) {
  }
}
__attribute__((weak)) void TIMER0A_handler() {
  while (1) {
  }
}
__attribute__((weak)) void TIMER0B_handler() {
  while (1) {
  }
}
__attribute__((weak)) void TIMER1A_handler() {
  while (1) {
  }
}
__attribute__((weak)) void TIMER1B_handler() {
  while (1) {
  }
}
__attribute__((weak)) void TIMER2A_handler() {
  while (1) {
  }
}
__attribute__((weak)) void TIMER2B_handler() {
  while (1) {
  }
}
__attribute__((weak)) void COMP0_handler() {
  while (1) {
  }
}
__attribute__((weak)) void COMP1_handler() {
  while (1) {
  }
}
__attribute__((weak)) void COMP2_handler() {
  while (1) {
  }
}
__attribute__((weak)) void SYSCTL_handler() {
  while (1) {
  }
}
__attribute__((weak)) void FLASH_handler() {
  while (1) {
  }
}
__attribute__((weak)) void GPIOF_handler() {
  while (1) {
  }
}
__attribute__((weak)) void UART2_handler() {
  while (1) {
  }
}
__attribute__((weak)) void SSI1_handler() {
  while (1) {
  }
}
__attribute__((weak)) void TIMER3A_handler() {
  while (1) {
  }
}
__attribute__((weak)) void TIMER3B_handler() {
  while (1) {
  }
}
__attribute__((weak)) void I2C1_handler() {
  while (1) {
  }
}
__attribute__((weak)) void QEI1_handler() {
  while (1) {
  }
}
__attribute__((weak)) void CAN0_handler() {
  while (1) {
  }
}
__attribute__((weak)) void CAN1_handler() {
  while (1) {
  }
}
__attribute__((weak)) void HIBERNATE_handler() {
  while (1) {
  }
}
__attribute__((weak)) void USB0_handler() {
  while (1) {
  }
}
__attribute__((weak)) void PWM0_3_handler() {
  while (1) {
  }
}
__attribute__((weak)) void UDMA_handler() {
  while (1) {
  }
}
__attribute__((weak)) void UDMAERR_handler() {
  while (1) {
  }
}
__attribute__((weak)) void ADC1SS0_handler() {
  while (1) {
  }
}
__attribute__((weak)) void ADC1SS1_handler() {
  while (1) {
  }
}
__attribute__((weak)) void ADC1SS2_handler() {
  while (1) {
  }
}
__attribute__((weak)) void ADC1SS3_handler() {
  while (1) {
  }
}
__attribute__((weak)) void SSI2_handler() {
  while (1) {
  }
}
__attribute__((weak)) void SSI3_handler() {
  while (1) {
  }
}
__attribute__((weak)) void UART3_handler() {
  while (1) {
  }
}
__attribute__((weak)) void UART4_handler() {
  while (1) {
  }
}
__attribute__((weak)) void UART5_handler() {
  while (1) {
  }
}
__attribute__((weak)) void UART6_handler() {
  while (1) {
  }
}
__attribute__((weak)) void UART7_handler() {
  while (1) {
  }
}
__attribute__((weak)) void I2C2_handler() {
  while (1) {
  }
}
__attribute__((weak)) void I2C3_handler() {
  while (1) {
  }
}
__attribute__((weak)) void TIMER4A_handler() {
  while (1) {
  }
}
__attribute__((weak)) void TIMER4B_handler() {
  while (1) {
  }
}
__attribute__((weak)) void TIMER5A_handler() {
  while (1) {
  }
}
__attribute__((weak)) void TIMER5B_handler() {
  while (1) {
  }
}
__attribute__((weak)) void WTIMER0A_handler() {
  while (1) {
  }
}
__attribute__((weak)) void WTIMER0B_handler() {
  while (1) {
  }
}
__attribute__((weak)) void WTIMER1A_handler() {
  while (1) {
  }
}
__attribute__((weak)) void WTIMER1B_handler() {
  while (1) {
  }
}
__attribute__((weak)) void WTIMER2A_handler() {
  while (1) {
  }
}
__attribute__((weak)) void WTIMER2B_handler() {
  while (1) {
  }
}
__attribute__((weak)) void WTIMER3A_handler() {
  while (1) {
  }
}
__attribute__((weak)) void WTIMER3B_handler() {
  while (1) {
  }
}
__attribute__((weak)) void WTIMER4A_handler() {
  while (1) {
  }
}
__attribute__((weak)) void WTIMER4B_handler() {
  while (1) {
  }
}
__attribute__((weak)) void WTIMER5A_handler() {
  while (1) {
  }
}
__attribute__((weak)) void WTIMER5B_handler() {
  while (1) {
  }
}
__attribute__((weak)) void SYSEXC_handler() {
  while (1) {
  }
}
__attribute__((weak)) void PWM1_0_handler() {
  while (1) {
  }
}
__attribute__((weak)) void PWM1_1_handler() {
  while (1) {
  }
}
__attribute__((weak)) void PWM1_2_handler() {
  while (1) {
  }
}
__attribute__((weak)) void PWM1_3_handler() {
  while (1) {
  }
}
__attribute__((weak)) void PWM1_FAULT() {
  while (1) {
  }
}
