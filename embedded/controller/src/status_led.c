#include "status_led.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tm4c123gh6pm.h"
#include "queue.h"
#include "uart0.h"

extern xQueueHandle q_debug;
extern xQueueHandle q_uartTx;
extern xQueueHandle q_uartRx;

void vStatus_Led_init() {
  // Set the direction of the GPIO port F pin 1 to output
  GPIO_PORTF_DIR_R = 0x0E;
  // Enable the digital function of the GPIO port F pin 1
  GPIO_PORTF_DEN_R = 0x0E;
}

void vStatus_Led_task(void *pvParameters) {
  QueueString message;
  strcpy(message.string, "Status LED task running\n");
  while (1) {
    xQueueSend(q_debug, &message, 14);
    GPIO_PORTF_DATA_R ^= 0x08;
    vTaskDelay(500 / portTICK_RATE_MS); // wait 500 ms.
  }
}
