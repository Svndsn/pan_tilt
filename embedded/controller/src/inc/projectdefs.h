#ifndef _PROJECTDEFS_H
#define _PROJECTDEFS_H
#include "FreeRTOSConfig.h"

#define USERTASK_STACK_SIZE configMINIMAL_STACK_SIZE

#define CONTROLLER_PERIOD_MS 10

typedef enum { IDLE = 0, LOW, MEDIUM, HIGH } taskPriority_t;

typedef enum { TILT = 0, PAN } motorAxis_t;

typedef enum { FALSE = 0, TRUE } bool_t;

typedef enum {
  SETPOINT_PAN_ABSOLUTE,  // 4 bytes
  SETPOINT_PAN_RELATIVE,  // 4 bytes
  ANGLE_PAN,              // 4 bytes
  SETPOINT_TILT_ABSOLUTE, // 4 bytes
  SETPOINT_TILT_RELATIVE, // 4 bytes
  ANGLE_TILT,             // 4 bytes
  DEBUG_STRING,           // 1 to 255 bytes
  VOLTAGE_PAN,            // 4 bytes
  VOLTAGE_TILT,           // 4 bytes
} uartCommandType_t;

#endif // _PROJECTDEFS_H
