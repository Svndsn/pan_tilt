#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
// This hook is called by FreeRTOS when an stack overflow error is detected.
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
  //
  // This function can not return, so loop forever.  Interrupts are disabled
  // on entry to this function, so no processor interrupts will interrupt
  // this loop.
  //
  while (1) {
  }
}
