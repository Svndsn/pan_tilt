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
void send_count(INT16U count) {
  send_char(count / 100 + '0');
  send_char((count % 100) / 10 + '0');
  send_char(count % 10 + '0');
  send_string("\n");
}

/***************** End of module **************/
volatile INT16U count = 0;

int main(void) {
  setup_gpio();
  setup_uart0();

set_led_color(GREEN);
  // Loop forever.
  while (1) {
  }
  return (0);
}

void GPIOA_handler(void) {
  if ((GPIO_PORTA_DATA_R & 0b01000000) == 0) {
    set_led_color(RED);
    count++;
    if (count > 450) {
      count = 0;
    }
    send_count(count);
  } else {
    set_led_color(GREEN);
    count--;
    if (count > 450) {
      count = 449;
    }
    send_count(count);
  }
  // clear interrupt flag for PA6
  GPIO_PORTA_ICR_R = 0b10000000;
}
