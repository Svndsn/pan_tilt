#ifndef _UART0_H
#define _UART0_H
#define MAX_STRING_LENGTH 30

typedef struct {
  char string[MAX_STRING_LENGTH];
} QueueString;

extern void vUart_0_init();
extern void vUart_0_task(void *pvParameters);

#endif // _UART0_H
