// freeRTOS includes
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

// Stdlib includes
#include "string.h"

// Project includes
#include "tm4c123gh6pm.h"
#include "status_led.h"
#include "uart0.h"

// Get the queues
extern xQueueHandle q_uartDebug;
extern xQueueHandle q_uartAngle;
extern xQueueHandle q_uartSetpoint;

void vStatusLedInit() {
  // Set the direction of the GPIO port F pin 1 to output
  GPIO_PORTF_DIR_R = 0x0E;
  // Enable the digital function of the GPIO port F pin 1
  GPIO_PORTF_DEN_R = 0x0E;
}

void vStatusLedTask(void *pvParameters) {
  // uartDebug_t message;
  // strcpy(message.string, "Status LED task running\n");
  while (1) {
    // xQueueSend(q_uartDebug, &message, 14);
    GPIO_PORTF_DATA_R ^= 0x08;
    vTaskDelay(500 / portTICK_RATE_MS); // wait 500 ms.
  }

  // Delete the task if it ever breaks out of the loop above
  vTaskDelete(NULL);
}
