#include "pwm.h"
#include "tm4c123gh6pm.h"

void setup_pwm() {
  // GPIO PORT B already setup in gpio.c
  // Enable the PWM module 0
  SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
  // Set the system clock as the clock source for the PWM module
  SYSCTL_RCC_R &= ~(SYSCTL_RCC_USEPWMDIV);

  // Setup the pins for the PWM module
  // Enable the alternate function for the PWM pins
  GPIO_PORTB_AFSEL_R |= 0b01000000;
  GPIO_PORTB_AMSEL_R &= ~0b01000000;
  // Set the alternate function for the PWM pins
  GPIO_PORTB_PCTL_R &= ~0x0F000000;
  GPIO_PORTB_PCTL_R |= 0x04000000;
  // Enable the digital function for the PWM pins
  GPIO_PORTB_DEN_R |= 0b01000000;

  // PWM configuration for 10 kHz frequency
  // Load value for the PWM generator
  // 16.000.000 / 16.000 = 1000
  // For PB6 (A) and PF3 (B)
  // Disable the PWM generator (gen 0)
  PWM0_0_CTL_R &= ~PWM_0_CTL_ENABLE;
  // Select the down-count mode
  PWM0_0_CTL_R &= ~PWM_0_CTL_MODE;
  PWM0_0_GENA_R |= 0x000000C8; // Set the action to drive the output high
  // Set the PWM load value
  PWM0_0_LOAD_R = 12000;
  PWM0_0_CMPA_R = 0;
  // Enable the PWM generator (gen 0)
  PWM0_0_CTL_R |= PWM_0_CTL_ENABLE;

  // Enable the PWM outputs
  PWM0_ENABLE_R |= PWM_ENABLE_PWM0EN;
}
