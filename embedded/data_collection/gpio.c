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
void setup_gpio(void) {
  // Enable the GPIO ports
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; // GPIO Port F // LED and buttons
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4; // GPIO Port E // H-bridge
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2; // GPIO Port C // Protection board
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0; // GPIO Port A // H-bridge, Protection board and uart0

  // PORTF
  // Unlock the GPIO commit control register.
  GPIO_PORTF_LOCK_R = 0x4C4F434B;

  // Allow changes to PF0
  GPIO_PORTF_CR_R |= 0x01;

  // Set the direction as output (PF1 - PF3).
  GPIO_PORTF_DIR_R |= 0b00001110;

  // Enable the GPIO pins for digital function (PF1 - PF4)
  GPIO_PORTF_DEN_R |= 0b00011111;

  // Enable internal pull-up (PF4).
  GPIO_PORTF_PUR_R |= 0b00010001;

  // PORTE
  // Set the direction as output (PF1 - PF3).
  GPIO_PORTE_DIR_R |= 0b00001110;

  // Enable the GPIO pins for digital function (PF1 - PF4)
  GPIO_PORTE_DEN_R |= 0b00001110;


  // PORTA
  // Set the direction as output (PA5 - PA7).
  GPIO_PORTA_DIR_R |= 0b11100000;
  GPIO_PORTA_PUR_R |= 0b00001100;
  GPIO_PORTA_DEN_R |= 0b11101100;

  // Set the interrupt type for GPIOF PF4 (SW1)
  GPIO_PORTA_IS_R &= ~(0b00001100);  // PF4 is edge-sensitive
  GPIO_PORTA_IBE_R &= ~(0b00001100); // PF4 not both edges
  GPIO_PORTA_IEV_R &= ~0b00001100;     // PF4 falling edge
  GPIO_PORTA_IM_R = 0b00001100;      // Interrupt on PF4
  GPIO_PORTA_ICR_R = 0b00001100;     // clear flag4

  // Enable interrupt for GPIO Port F (INT30)
  NVIC_EN0_R |= (1 << (INT_GPIOA - 16)); // Enable interrupt 0 in NVIC
  // Set priority to 5 (0-7)
  // NVIC_PRI0_R = (NVIC_PRI0_R & 0xFF00FFFF) | (0b101 << 21);

  // PORTC
  // Set the direction as input (PA5 - PA7).
  GPIO_PORTC_DIR_R &= ~0b11110000;
  GPIO_PORTC_PUR_R |= 0b11110000;
  GPIO_PORTC_DEN_R |= 0b11110000;

  // Set the interrupt type for GPIOF PF4 (SW1)
  GPIO_PORTC_IS_R &= ~(0b01010000);  // PF4 is edge-sensitive
  GPIO_PORTC_IBE_R &= ~(0b01010000); // PF4 not both edges
  GPIO_PORTC_IEV_R &= ~0b01010000;     // PF4 falling edge
  GPIO_PORTC_IM_R = 0b01010000;      // Interrupt on PF4
  GPIO_PORTC_ICR_R = 0b01010000;     // clear flag4

  // Enable interrupt for GPIO Port F (INT30)
  NVIC_EN0_R |= (1 << (INT_GPIOC - 16)); // Enable interrupt 0 in NVIC
  // Set priority to 5 (0-7)
  // NVIC_PRI0_R = (NVIC_PRI0_R & 0xFF00FFFF) | (0b101 << 21);
}

void setup_uart0(void) {
  // Setup the UART0
  // Enable the UART0 clock
  SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
  // Set the alternate function for PA0 and PA1
  GPIO_PORTA_AFSEL_R |= 0b00000011;
  // Enable the alternate function for PA0 and PA1
  GPIO_PORTA_PCTL_R |= 0x00000011;
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

void set_led_color(LED_Color color) { GPIO_PORTF_DATA_R = color << 1; }

void set_port(output_port port, BOOLEAN value) {
  switch (port) {
  case ENA:
    if (value) {
      GPIO_PORTE_DATA_R |= 0b00000010;
    } else {
      GPIO_PORTE_DATA_R &= ~0b00000010;
    }
    break;
  case IN1A:
    if (value) {
      GPIO_PORTE_DATA_R |= 0b00000100;
    } else {
      GPIO_PORTE_DATA_R &= ~0b00000100;
    }
    break;
  case IN2A:
    if (value) {
      GPIO_PORTE_DATA_R |= 0b00001000;
    } else {
      GPIO_PORTE_DATA_R &= ~0b00001000;
    }
    break;
  case ENB:
    if (value) {
      GPIO_PORTA_DATA_R |= 0b00100000;
    } else {
      GPIO_PORTA_DATA_R &= ~0b00100000;
    }
    break;
  case IN1B:
    if (value) {
      GPIO_PORTA_DATA_R |= 0b01000000;
    } else {
      GPIO_PORTA_DATA_R &= ~0b01000000;
    }
    break;
  case IN2B:
    if (value) {
      GPIO_PORTA_DATA_R |= 0b10000000;
    } else {
      GPIO_PORTA_DATA_R &= ~0b10000000;
    }
    break;
  default:
    break;
  }
}

BOOLEAN get_port(input_port port) {
  BOOLEAN result = FALSE;
  switch (port) {
  case Index0:
    result = (GPIO_PORTA_DATA_R & 0b00000100) != 0;
    break;
  case Index1:
    result = (GPIO_PORTA_DATA_R & 0b00001000) != 0;
    break;
  case Sensor1A:
    result = (GPIO_PORTC_DATA_R & 0b00010000) != 0;
    break;
  case Sensor1B:
    result = (GPIO_PORTC_DATA_R & 0b00100000) != 0;
    break;
  case Sensor2A:
    result = (GPIO_PORTC_DATA_R & 0b01000000) != 0;
    break;
  case Sensor2B:
    result = (GPIO_PORTC_DATA_R & 0b10000000) != 0;
    break;
  case SW1:
    result = (GPIO_PORTF_DATA_R & 0b00010000) != 0;
    break;
  case SW2:
    result = (GPIO_PORTF_DATA_R & 0b00000001) != 0;
    break;
  default:
    break;
  }
  return result;
}

/***************** End of module **************/
