// freeRTOS includes
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include <string.h>

// Project includes
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "status_led.h"


// UART0 queues
extern xQueueHandle q_uartDebug;
extern xQueueHandle q_uartAngle; 
extern xQueueHandle q_uartSetpoint;
// UART0 Mutex
extern xSemaphoreHandle m_uartDebug;
extern xSemaphoreHandle m_uartAngle;
extern xSemaphoreHandle m_uartSetpoint;

void vStatusLedInit() {
  // Set the direction of the GPIO port F pin 1 to output
  GPIO_PORTF_DIR_R = 0x0E;
  // Enable the digital function of the GPIO port F pin 1
  GPIO_PORTF_DEN_R = 0x0E;
}

void vStatusLedTask(void *pvParameters) {
  // Task loop
  while (1) {
    // Toggle the red LED
    GPIO_PORTF_DATA_R ^= 0x02;
    // Wait 500 ms
    vTaskDelay(500 / portTICK_RATE_MS); // wait 500 ms.
  }

  // Delete the task if it ever breaks out of the loop above
  vTaskDelete(NULL);
}
