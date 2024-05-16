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
#include "led.h"
#include "tm4c123gh6pm.h"
#include <stdint.h>

/*****************************    Defines    *******************************/

/*****************************   Constants   *******************************/

/*****************************   Functions   *******************************/
void LED_init() {
  // Initialize LED pin
  //
  // Unlock the GPIO commit control register.
  GPIO_PORTF_LOCK_R = 0x4C4F434B;

  // Allow changes to PF0
  GPIO_PORTF_CR_R |= 0x01;

  GPIO_PORTF_DIR_R |= (0b111 << 1); // Set LED pin as output
  GPIO_PORTF_PUR_R |= (1 << 4)| (1 << 0); // Enable pull-up resistor for SW1 and SW2
  GPIO_PORTF_DEN_R |= (0b111 << 1) | (1 << 4) | (1 << 0); // Enable digital functionality
}

void ControlLED(unsigned char led_on) {
  GPIO_PORTF_DATA_R &= ~(0b111 << 1);           // Turn off all LEDs
  GPIO_PORTF_DATA_R |= ((led_on & 0b111) << 1); // Turn LED off
}
/****************************** End Of Module *******************************/
