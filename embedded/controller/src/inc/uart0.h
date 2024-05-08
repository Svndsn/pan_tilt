#ifndef _UART0_H
#define _UART0_H
#include "emp_type.h"
#include "projectdefs.h"

#define MAX_STRING_LENGTH 30

typedef struct {
  char string[MAX_STRING_LENGTH];
} uartDebug_t;

typedef struct {
  motorAxis_t axis;
  INT16S angle;
  // Only used for relative movement from current position
  BOOLEAN relative;
} uartAngle_t;

extern void vUart0Init();
extern void vUart0Task(void *pvParameters);

#endif // _UART0_H
