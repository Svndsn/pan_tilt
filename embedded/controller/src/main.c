#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "tm4c123gh6pm.h"

#include "emp_type.h"
#include "spi1.h"
#include "status_led.h"
#include "taskmodel.h"
#include "uart0.h"

// UART0 queues
xQueueHandle q_debug; // Debugging queue
xQueueHandle q_uartTx;
xQueueHandle q_uartRx;
// SPI1 queues
xQueueHandle q_spi1Tx;
xQueueHandle q_spi1Rx;

void setup_hardware() {
  // Enable clock for GPIO
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; // Status LED
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3; // SPI1
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0; // UART0
  // Wait for the GPIO port F to be ready
  while (!(SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R0)) {
  }
  vStatus_Led_init();
  vUart_0_init();
  vSpi_1_init();
}

int main() {
  // Setup the hardware
  setup_hardware();

  // Create the queues
  q_debug = xQueueCreate(20, sizeof(QueueString));
  q_uartTx = xQueueCreate(20, sizeof(INT8U));
  q_uartRx = xQueueCreate(20, sizeof(INT8U));
  q_spi1Tx = xQueueCreate(20, sizeof(INT16U));
  q_spi1Rx = xQueueCreate(20, sizeof(INT16U));

  // Create the tasks
  xTaskCreate(vStatus_Led_task, "Status LED", USERTASK_STACK_SIZE, NULL,
              LOW_PRIO, NULL);
  xTaskCreate(vUart_0_task, "UART 0 Rx/Tx", USERTASK_STACK_SIZE, NULL, MED_PRIO,
              NULL);
  // Start the scheduler
  vTaskStartScheduler();
  return 0;
}
