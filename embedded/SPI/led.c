/*****************************************************************************
* University of Southern Denmark
* Robotics 4th Semester project
*
* MODULENAME.: led.c
*
* PROJECT....: Control and Regulation Project
*
* DESCRIPTION: LEDS for testing
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
#include "led.h"
#include "tm4c123gh6pm.h"
/*****************************    Defines    *******************************/

/*****************************   Constants   *******************************/

/*****************************   Functions   *******************************/
void LED_init(){
    // Initialize LED pin
    GPIO_PORTF_DIR_R |= (1<<1);    // Set LED pin as output
    GPIO_PORTF_DEN_R |= (1<<1);    // Enable digital functionality
}

void ControlLED(unsigned char led_on){
    if (led_on) {
        GPIO_PORTF_DATA_R |= (1<<1);  // Turn LED on
    } else {
        GPIO_PORTF_DATA_R &= ~(1<<1); // Turn LED off
    }
}
/****************************** End Of Module *******************************/
