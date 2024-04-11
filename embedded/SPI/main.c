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
  unsigned char val1 = 'U';

  while (1) {

    if ((GPIO_PORTF_DATA_R & (1 << 4)) == 0) // Check if SW1 is pressed
    {
      GPIO_PORTF_DATA_R |= (1 << 1);
      SPI1_Write(val1);
      // delay_ms(1000);
      while((GPIO_PORTF_DATA_R & (1 << 4)) == 0);
    } else {
      GPIO_PORTF_DATA_R &= ~(1 << 1);
    }
  }
}
