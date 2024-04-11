/***********************************************
 * Univeristy of Southern Denmark
 * Embedded Programming (EMP)
 *
 * MODULENAME: main.c
 * PROJECT: template
 * DESCRIPTION: Empty project template
 * Change log:
 ***********************************************
 * Date of Change
 * YYMMDD
 * ----------------
 * 160224 MB Module created.
 **********************************************/

/***************** Header *********************/
/***************** Include files **************/
#include "emp_type.h"
#include "gpio.h"
#include "tm4c123gh6pm.h"

/***************** Defines ********************/
/***************** Constants ******************/
/***************** Variables ******************/
/***************** Functions ******************/
void delay(INT32U count) {
  while (count--) {
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

/***************** End of module **************/

int main(void) {
  INT8U uart_data = 0;
  setup_gpio();
  setup_uart0();
  // INT16S count;

  // Loop forever.
  while (1) {
    if ((GPIO_PORTF_DATA_R & (1 << 4)) == 0) {
      set_led_color(GREEN);
      GPIO_PORTA_DATA_R |=
          0b10100000; // Enable the motor and set the direction to forward
    } else if ((GPIO_PORTF_DATA_R & (1 << 0)) == 0) {
      set_led_color(BLUE);
      GPIO_PORTA_DATA_R |=
          0b11000000; // Enable the motor and set the direction to reverse
    } else {
      GPIO_PORTA_DATA_R &= ~0b11100000; // Disable the motor
      set_led_color(OFF);
    }
  }
  return (0);
}
