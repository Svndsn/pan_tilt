#ifndef _UART0_H
#define _UART0_H
#include "emp_type.h"
#include "projectdefs.h"

#define MAX_STRING_LENGTH 100

typedef struct {
  char string[MAX_STRING_LENGTH];
} uartDebug_t;

typedef struct {
  motorAxis_t axis;
  FP32 angle;
  // Only used for relative movement from current position
  BOOLEAN relative;
} uartAngle_t;

typedef struct {
  motorAxis_t motor;
  FP32 voltage;
} uartVoltage_t;

extern void vUart0Init();
extern void vUart0Task(void *pvParameters);

#endif // _UART0_H
