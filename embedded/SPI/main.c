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
#include "uart.h"
#include "emp_type.h"

/*****************************    Defines    *******************************/
/*****************************   Constants   *******************************/
/*****************************   Variables   *******************************/
unsigned char received_data;
unsigned char led_on;
/*****************************   Functions   *******************************/

void delay(INT32U count) {
  while (count--) {
    for (int i = 0; i < 1600; i++) {
    }
  }
}
void send_char(INT8U chr) {
  while (UART0_FR_R & (1 << 5))
    ;
  UART0_DR_R = chr;
}

void send_string(char *str) {
  while (*str) {
    send_char(*str);
    str++;
  }
}
int main(void) {
  SPI1_init();
  LED_init();
  setup_uart0();
  unsigned char val1 = 'U'; // 0b01010101

  while (1) {

    if ((GPIO_PORTF_DATA_R & (1 << 4)) == 0) // Check if SW1 is pressed
    {
      // Turn on red led
      GPIO_PORTF_DATA_R |= (1 << 1);
      // SPI1_Write(val1);
      send_string("Sending data to SPI1\n");
      while ((GPIO_PORTF_DATA_R & (1 << 4)) == 0)
        ;// Only send one value pr. press
    } else {
      // Turn off red led
      GPIO_PORTF_DATA_R &= ~(1 << 1);
    }
  }
}
