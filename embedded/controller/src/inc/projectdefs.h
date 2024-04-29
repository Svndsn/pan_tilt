#ifndef _PROJECTDEFS_H
#define _PROJECTDEFS_H
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"

#define USERTASK_STACK_SIZE configMINIMAL_STACK_SIZE

#define CONTROLLER_HZ 100
#define CONTROLLER_PERIOD_TICKS ((1000 / CONTROLLER_HZ) / portTICK_PERIOD_MS)

typedef enum { IDLE = 0, LOW, MEDIUM, HIGH } taskPriority_t;

typedef enum { TILT = 0, PAN } motorAxis_t;

#endif // _PROJECTDEFS_H
