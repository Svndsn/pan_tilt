#ifndef _SYSTICK_H
#define _SYSTICK_H
#include "tm4c123gh6pm.h"
#define SYSTICK_RELOAD_VALUE 16000 // SysTick interrupt every 1ms

extern void setup_systick();
#endif // !SYSTICK_H
