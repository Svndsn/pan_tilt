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
typedef enum { TILT = 0, PAN = 1 } Angle;
/***************** Constants ******************/
/***************** Variables ******************/
/***************** Functions ******************/
void delay(int n) {
  int i;
  for (i = 0; i < n; i++) {
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

void send_angle_txt(Angle type, INT16S angle) {
  send_string(type == PAN ? "Pan: " : "Tilt: ");
  if (angle < 0) {
    send_char('-');
    angle = -angle;
  }
  send_char('0' + (angle) / 1000);
  send_char('0' + (angle % 1000) / 100);
  send_char('0' + (angle % 100) / 10);
  send_char('0' + (angle % 10));
  send_char('\n');
}

void send_angle(Angle type, INT16S angle) {
  INT8U lowByte = 0b00000000;
  INT8U highByte = 0b01000000;

  // Set angle bit
  if (type == PAN) {
    lowByte |= 0b10000000;
    highByte |= 0b10000000;
  }

  // Set sign bit
  if (angle < 0) {
    angle = -angle;
    highByte |= 0b00010000;
  }

  // Set data bits
  lowByte |= angle & 0b00111111;
  highByte |= (angle >> 6) & 0b00001111;

  // Send data
  send_char(lowByte);
  send_char(highByte);
}

void uart_data_handler(INT8U data, Angle type, INT16S *angle) {
  // Format:
  // ABRSDDDD
  // A: Angle (1 = Pan, 0 = Tilt)
  // B: Bits  (1 = High bits, 0 = Low bits)
  // R: Relative (1 = Relative, 0 = Absolute)
  // S: Sign  (Data if B=0, Sign if B=1)
  // D: Data  (Always)
  // If B=1 and R=1, then it is a request and data is ignored

  if (data & (1 << 6)) {   // High bits
    if (data & (1 << 5)) { // Request data
      send_angle(type, *angle);
      // send_angle_txt(type, *angle);
    } else { // No request (Data and sign)
      if (*angle < 0) {
        *angle = -*angle;
      }
      *angle = ((data & 0x0F) << 6) | (*angle & 0x3F);
      if (data & (1 << 4)) { // Negative
        *angle = -(*angle);
      }
    }
  } else { // Low bits
    if (*angle < 0) {
      *angle = -((data & 0x3F) | ((-*angle) & 0xFFC0));
    } else {
      *angle = (data & 0x3F) | (*angle & 0xFFC0);
    }
  }
}

/***************** End of module **************/

int main(void) {
  INT16S pan_angle = 0;
  INT16S tilt_angle = 0;
  INT8U uart_data = 0;
  setup_gpio();

  // Loop forever.
  while (1) {
    if (UART0_FR_R & (1 << 4)) { // Check if there is data
      delay(10000);
      set_led_color(RED);
    } else {
      uart_data = UART0_DR_R; // Read the data
      set_led_color(BLUE);
      if (uart_data & (1 << 7)) {
        uart_data_handler(uart_data, PAN, &pan_angle);
      } else {
        uart_data_handler(uart_data, TILT, &tilt_angle);
      }
      delay(1000);
    }
  }
  return (0);
}
