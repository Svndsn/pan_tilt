#include "FreeRTOS.h"
#include "emp_type.h"
#include "task.h"
#include "tm4c123gh6pm.h"

#define USERTASK_STACK_SIZE configMINIMAL_STACK_SIZE
#define IDLE_PRIO 0
#define LOW_PRIO 1
#define MED_PRIO 2
#define HIGH_PRIO 3

void status_led_task(void *pvParameters) {
  SYSCTL_RCGCGPIO_R = SYSCTL_RCGCGPIO_R5;
  while (!(SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5)) { }
  GPIO_PORTF_DIR_R = 0x02;
  GPIO_PORTF_DEN_R = 0x02;

  while (1) {
    // Toggle status led
    GPIO_PORTF_DATA_R ^= 0x02;
    vTaskDelay(500 / portTICK_RATE_MS); // wait 500 ms.
  }
}
int main() {
  xTaskCreate(status_led_task, "Status_led", USERTASK_STACK_SIZE, NULL,
              LOW_PRIO, NULL);
  vTaskStartScheduler();
  return 0;
}
