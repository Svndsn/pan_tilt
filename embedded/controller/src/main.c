// FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

// Project
#include "pid.h"
#include "projectdefs.h"
#include "spi1.h"
#include "status_led.h"
#include "tm4c123gh6pm.h"
#include "uart0.h"

// UART0 queues
xQueueHandle q_uartDebug;
xQueueHandle q_uartAngle;    // Send current angle
xQueueHandle q_uartSetpoint; // Receive setpoint
xQueueHandle q_uartVoltage;  // Receive setpoint
xQueueHandle q_uartRawData;  // Input buffer
// UART0 Mutex
xSemaphoreHandle m_uartDebug;
xSemaphoreHandle m_uartAngle;
xSemaphoreHandle m_uartSetpoint;
xSemaphoreHandle m_uartVoltage;

// SPI1 queues
xQueueHandle q_spiDutyCycle; // Send duty cycle
xQueueHandle q_spiAngle;     // Receive angle
// SPI1 Mutex
xSemaphoreHandle m_spiDutyCycle;
xSemaphoreHandle m_spiAngle;

void setup_hardware() {
  // Enable clocks
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; // Status LED
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3; // SPI1
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0; // UART0

  // Initialize the hardware
  vStatusLedInit();
  vUart0Init();
  vSpi1Init();

  // Enable buttons
  // Unlock the GPIO commit control register.
  GPIO_PORTF_LOCK_R = 0x4C4F434B;

  // Allow changes to PF0
  GPIO_PORTF_CR_R |= 0x01;

  // Enable the GPIO pins for digital function (PF0 - PF4)
  GPIO_PORTF_DEN_R |= 0b00010001;

  // Enable internal pull-up (PF4).
  GPIO_PORTF_PUR_R |= 0b00010001;
}

int main() {
  // Setup the hardware
  setup_hardware();

  // Create the queues
  q_uartDebug    = xQueueCreate(20, sizeof(uartDebug_t));
  q_uartAngle    = xQueueCreate(20, sizeof(uartAngle_t));
  q_uartSetpoint = xQueueCreate(40, sizeof(uartAngle_t));
  q_uartVoltage  = xQueueCreate(20, sizeof(uartVoltage_t));
  q_spiDutyCycle = xQueueCreate(20, sizeof(spiDutyCycle_t));
  q_spiAngle     = xQueueCreate(20, sizeof(spiAngle_t));
  q_uartRawData  = xQueueCreate(255, sizeof(INT8U));

  // Create the mutexes
  m_uartDebug    = xSemaphoreCreateMutex();
  m_uartAngle    = xSemaphoreCreateMutex();
  m_uartSetpoint = xSemaphoreCreateMutex();
  m_uartVoltage  = xSemaphoreCreateMutex();
  m_spiDutyCycle = xSemaphoreCreateMutex();
  m_spiAngle     = xSemaphoreCreateMutex();

  // Create the tasks
  xTaskCreate(vStatusLedTask,  "Status LED",     USERTASK_STACK_SIZE, NULL, LOW,    NULL);
  xTaskCreate(vUart0Task,      "UART 0 Rx/Tx",   USERTASK_STACK_SIZE, NULL, MEDIUM, NULL);
  xTaskCreate(vSpi1Task,       "SPI 1 Rx/Tx",    USERTASK_STACK_SIZE, NULL, MEDIUM, NULL);
  xTaskCreate(vControllerTask, "PID controller", USERTASK_STACK_SIZE, NULL, HIGH,   NULL);

  // Start the scheduler
  vTaskStartScheduler();
  return 0;
}
