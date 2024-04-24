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
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3; // GPIO Port D PWM
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2; // GPIO Port C // Protection board
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1; // GPIO Port B // Protection board
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

  GPIO_PORTA_IS_R &= ~(0b00001100);  // PF4 is edge-sensitive
  GPIO_PORTA_IBE_R &= ~(0b00001100); // PF4 not both edges
  GPIO_PORTA_IEV_R &= ~0b00001100;     // PF4 falling edge
  GPIO_PORTA_IM_R = 0b00001100;      // Interrupt on PF4
  GPIO_PORTA_ICR_R = 0b00001100;     // clear flag4

  // Enable interrupt for GPIO Port A (INT0)
  NVIC_EN0_R |= (1 << (INT_GPIOA - 16)); // Enable interrupt 0 in NVIC
  // Set priority to 5 (0-7)
  // NVIC_PRI0_R = (NVIC_PRI0_R & 0xFF00FFFF) | (0b101 << 21);

  // PORTC
  // Set the direction as input (PA5 - PA7).
  GPIO_PORTC_DIR_R &= ~0b11110000;
  GPIO_PORTC_PUR_R |= 0b11110000;
  GPIO_PORTC_DEN_R |= 0b11110000;

  // Set the interrupt type for GPIOC
  GPIO_PORTC_IS_R &= ~(0b01010000);  // PF4 is edge-sensitive
  GPIO_PORTC_IBE_R &= ~(0b01010000); // PF4 not both edges
  GPIO_PORTC_IEV_R &= ~0b01010000;     // PF4 falling edge
  GPIO_PORTC_IM_R = 0b01010000;      // Interrupt on PF4
  GPIO_PORTC_ICR_R = 0b01010000;     // clear flag4

  // Enable interrupt for GPIO Port C
  NVIC_EN0_R |= (1 << (INT_GPIOC - 16)); // Enable interrupt 0 in NVIC
  // Set priority to 5 (0-7)
  // NVIC_PRI0_R = (NVIC_PRI0_R & 0xFF00FFFF) | (0b101 << 21);
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
