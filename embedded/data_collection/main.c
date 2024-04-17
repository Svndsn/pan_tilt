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
#include "systick.h"
#include "tm4c123gh6pm.h"
#include "uart.h"

/***************** Defines ********************/

/***************** Constants ******************/
volatile INT16U panCount = 0;
volatile INT16U tiltCount = 0;
volatile INT16U ticks = 0;
/***************** Variables ******************/
/***************** Functions ******************/

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
void send_count(INT16U count) {
  send_char(count / 10000 + '0');
  count = count % 10000;
  send_char(count / 1000 + '0');
  count = count % 1000;
  send_char(count / 100 + '0');
  count = count % 100;
  send_char(count / 10 + '0');
  count = count % 10;
  send_char(count + '0');
}

/***************** End of module **************/

int main(void) {
  setup_gpio();
  setup_uart0();
  setup_systick();
  // setup_pwm();

  // Loop forever.
  while (1) {
    if (get_port(SW1) == 0) {
      set_port(ENA, 1);
      set_port(IN1A, 1);
      ticks = 0;
      while (get_port(SW1) == 0) {
      }
    } else if (get_port(SW2) == 0) {
      set_port(ENA, 1);
      set_port(IN2A, 1);
      ticks = 0;
      while (get_port(SW2) == 0) {
      }
    } else {
      set_port(ENA, 0);
      set_port(IN1A, 0);
      set_port(IN2A, 0);
    }
    if(get_port(Index1)){
      set_led_color(RED);
    } else{
      set_led_color(BLUE);
    }
  }
  return (0);
}

void GPIOA_handler(void) {
  // Index hall sensor
  if (GPIO_PORTA_MIS_R & 0b00000100) {
    // Pan hall sensor
    panCount = 32768;
    GPIO_PORTA_ICR_R |= 0b00000100;
    send_string("RP\n");
  } else if (GPIO_PORTA_MIS_R & 0b00001000) {
    // Tilt hall sensor
    tiltCount = 32768;
    GPIO_PORTA_ICR_R |= 0b00001000;
    send_string("RT\n");
  }
}

void GPIOC_handler(void) {
  if (GPIO_PORTC_MIS_R & 0b00010000) {
    if (get_port(Sensor1B) == 0) {
      panCount++;
    } else {
      panCount--;
    }
    send_char('P');
    send_char(' ');
    send_count(panCount);
    send_char(' ');
    send_count(ticks);
    send_char('\n');
    GPIO_PORTC_ICR_R |= 0b00010000;
  } else if (GPIO_PORTC_MIS_R & 0b01000000) {
    if (get_port(Sensor2B) == 0) {
      tiltCount++;
    } else {
      tiltCount--;
    }
    send_char('T');
    send_char(' ');
    send_count(tiltCount);
    send_char(' ');
    send_count(ticks);
    send_char('\n');
    GPIO_PORTC_ICR_R |= 0b01000000;
  }
}

void SysTick_handler(void) { ticks++; }
