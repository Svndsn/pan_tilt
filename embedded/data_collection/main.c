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
#include "pwm.h"
#include "systick.h"
#include "tm4c123gh6pm.h"
#include "uart.h"

/***************** Defines ********************/

/***************** Constants ******************/
volatile INT16U panCount = 32768; // (2^16)/2
volatile INT16U tiltCount = 32768; // (2^16)/2
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
  while (UART0_FR_R & (1 << 5));
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
INT16U speed = 5;
void crement_pwm(INT16U *pwm_val, INT8U *dir) {
  static INT8U mode = 0;
  if (*dir == 1) {
    set_port(IN1A, 1);
    set_port(IN2A, 0);
    if (mode == 0) {
      (*pwm_val) += speed;
      if (*pwm_val >= 12000) {
        mode = 1;
      }
    } else if (*pwm_val > 0 && mode == 1) {
      (*pwm_val) -= speed;
    } else {
      *dir = 0;
    }
  } else {
    set_port(IN1A, 0);
    set_port(IN2A, 1);
    if (mode == 1) {
      (*pwm_val) += speed;
      if (*pwm_val >= 12000) {
        mode = 0;
      }
    } else if (*pwm_val > 0 && mode == 0) {
      (*pwm_val) -= speed;
    } else {
      *dir = 1;
    }
  }
}

int main(void) {
  setup_gpio();
  setup_uart0();
  setup_systick();
  setup_pwm();
  //
  INT16U temp_pan = 0;
  INT16U temp_tilt = 0;
  INT16U temp_ticks = 0;
  INT16U temp_last_ticks = -1;

  INT8U dir = 1;
  INT16U pwm_val = 0;
  while (1) {
    if (get_port(SW1) == 0) {
      set_led_color(RED);
      ticks = 0;
      while (1) {
        temp_ticks = ticks;
        if (temp_ticks % 4 == 0 && temp_last_ticks != temp_ticks) {
          crement_pwm(&pwm_val, &dir);
          PWM0_0_CMPA_R = pwm_val;
          temp_last_ticks = temp_ticks;
          temp_tilt = tiltCount;
          send_char('T');
          send_char(',');
          if(!dir){
            send_char('-');
          }
          send_count(pwm_val);
          send_char(',');
          send_count(temp_tilt);
          send_char(',');
          send_count(temp_ticks);
          send_char('\n');
        }
      }
    } else {
      dir = 1;
      pwm_val = 0;
      set_port(IN1A, 0);
      set_port(IN2A, 0);
    }
    if (get_port(Index1)) {
      set_led_color(RED);
    } else {
      set_led_color(BLUE);
    }
  }
  return (0);
}

void GPIOA_handler(void) {
  // Index hall sensor
  if (GPIO_PORTA_MIS_R & 0b00000100) {
    // Pan hall sensor
    // panCount = 32768;
    GPIO_PORTA_ICR_R |= 0b00000100;
  } else if (GPIO_PORTA_MIS_R & 0b00001000) {
    // Tilt hall sensor
    // tiltCount = 32768;
    GPIO_PORTA_ICR_R |= 0b00001000;
  }
}

void GPIOC_handler(void) {
  if (GPIO_PORTC_MIS_R & 0b01000000) {
    if (get_port(Sensor2B) == 0) {
      tiltCount++;
      set_led_color(RED);
    } else {
      tiltCount--;
      set_led_color(BLUE);
    }
    // send_count(tiltCount);
    GPIO_PORTC_ICR_R |= 0b01000000;
  }
}

void SysTick_handler(void) { ticks++; }
