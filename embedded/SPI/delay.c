/*****************************************************************************
* University of Southern Denmark
* Robotics 4th Semester project
*
* MODULENAME.: delay.c
*
* PROJECT....: Control and Regulation Project
*
* DESCRIPTION: delay function
*
* Change Log:
******************************************************************************
* Date    Id    Change
* 240401
* --------------------
* 240401  MoH   Module created.
*
*****************************************************************************/

/***************************** Include files *******************************/
#include <stdint.h>
#include "delay.h"
#include "tm4c123gh6pm.h"

/*****************************    Defines    *******************************/

/*****************************   Constants   *******************************/

/*****************************   Functions   *******************************/

void delay_ms(int time_ms)
/* calculations are based on 16MHz system clock frequency */
{
    int i, j;
    for(i = 0 ; i < time_ms; i++)
        for(j = 0; j < 3180; j++)
            {}  /* execute NOP for 1ms */
}
/****************************** End Of Module *******************************/
