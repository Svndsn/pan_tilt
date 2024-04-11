/*****************************************************************************
 * University of Southern Denmark
 * Robotics 4th Semester project
 *
 * MODULENAME.: main.c
 *
 * PROJECT....: Control and Regulation Project
 *
 * DESCRIPTION: main file for TIVA C TM4C123GH6PM
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
#include "SPI1.h"
#include "delay.h"
#include "led.h"
#include "tm4c123gh6pm.h"

/*****************************    Defines    *******************************/
/*****************************   Constants   *******************************/
/*****************************   Variables   *******************************/
unsigned char received_data;
unsigned char led_on;
/*****************************   Functions   *******************************/

int main(void) {
  SPI1_init();
  LED_init();
  unsigned char val1 = 'U'; // 0b01010101

  while (1) {

    if ((GPIO_PORTF_DATA_R & (1 << 4)) == 0) // Check if SW1 is pressed
    {
      // Turn on red led
      GPIO_PORTF_DATA_R |= (1 << 1);
      SPI1_Write(val1);
      while ((GPIO_PORTF_DATA_R & (1 << 4)) == 0)
        ;// Only send one value pr. press
    } else {
      // Turn off red led
      GPIO_PORTF_DATA_R &= ~(1 << 1);
    }
  }
}
