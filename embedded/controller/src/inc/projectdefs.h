#ifndef _PROJECTDEFS_H
#define _PROJECTDEFS_H
#include "FreeRTOSConfig.h"

#define USERTASK_STACK_SIZE configMINIMAL_STACK_SIZE

#define CONTROLLER_PERIOD_MS 10

typedef enum { IDLE = 0, LOW, MEDIUM, HIGH } taskPriority_t;

typedef enum { TILT = 0, PAN } motorAxis_t;

#endif // _PROJECTDEFS_H
