/***********************************************
 * Univeristy of Southern Denmark
 * Embedded Programming (EMP)
 *
 * MODULENAME: module.c
 * PROJECT: template c-file
 * DESCRIPTION: Empty module template
 * Change log:
 ***********************************************
 * Date of Change
 * YYMMDD
 * ----------------
 * 160224 MB Module created.
 ************************************************/

/***************** Header *********************/
/***************** Include files **************/
#include "gpio.h"
#include "emp_type.h"
#include "tm4c123gh6pm.h"
/***************** Defines ********************/
/***************** Constants ******************/
/***************** Variables ******************/
/***************** Functions ******************/
void set_led_color(enum LED_Color color) { GPIO_PORTF_DATA_R = color << 1; }

void setup_gpio(void) {
  // Enable the GPIO port that is used for the on-board LEDs and switches.
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; // GPIO Port F
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4; // GPIO Port E
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0; // GPIO Port A

  // Set the direction as output (PF1 - PF3).
  GPIO_PORTF_DIR_R |= 0b00001110;

  // Enable the GPIO pins for digital function (PF1 - PF4)
  GPIO_PORTF_DEN_R |= 0b00011110;

  // Enable internal pull-up (PF4).
  GPIO_PORTF_PUR_R |= (1 << 4);

  // Setup the keyboard
  // Set Y as input and X as output
  GPIO_PORTE_DIR_R &= ~(0b00001111);
  GPIO_PORTE_PDR_R |= 0b00001111;
  GPIO_PORTE_DEN_R |= 0b00001111;

  GPIO_PORTA_DIR_R |= 0b00011100;
  GPIO_PORTA_DEN_R |= 0b00011100;

  // Setup the UART0
  // Enable the UART0 clock
  SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
  // Set the alternate function for PA0 and PA1
  GPIO_PORTA_AFSEL_R |= 0b00000011;
  // Enable the alternate function for PA0 and PA1
  GPIO_PORTA_PCTL_R |= 0x11;
  // Enable the GPIO pins for digital function (PA0 and PA1)
  GPIO_PORTA_DEN_R |= 0b00000011;
  // Handle direction of the pins
  GPIO_PORTA_DIR_R |= 0b00000010;

  // Disable UART0
  UART0_CTL_R &= ~(UART_CTL_UARTEN);

  INT32U baud_rate = 115200;
  INT32U brd;

  // Set the baud rate
  // IBRD = int(16,000,000 / (16 * 9600)) = int(104.1667)
  // Set the fractional part of the baud rate
  // FBRD = 0.1667 * 64 + 0.5 = int(11,16688)
  brd = 64000000 / baud_rate; // X-sys*64/(16*baudrate) = 16M*4/baudrate
  UART0_IBRD_R = brd / 64;
  UART0_FBRD_R = brd & 0x0000003F;

  // Set the transmission parameters
  UART0_LCRH_R = 0b01100000;  // 8-bit word length
  UART0_LCRH_R |= 0b00000110; // even parity
  UART0_LCRH_R |= (1 << 4);   // Enable FIFO

  // Set the clock source
  UART0_CC_R = 0;
  // Enable the UART0
  UART0_CTL_R |=
      (UART_CTL_RXE | UART_CTL_TXE | UART_CTL_UARTEN); // Enable UART0
}
/***************** End of module **************/
