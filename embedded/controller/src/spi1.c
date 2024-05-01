#include "spi1.h"
#include "FreeRTOS.h"
#include "emp_type.h"
#include "queue.h"
#include "task.h"
#include "tm4c123gh6pm.h"

// SPI1 queues
extern xQueueHandle q_spiDutyCycle;
extern xQueueHandle q_spiAngle;

void vSpi1Init() {
  // Enable clock to SPI1, GPIOD and GPIOF
  SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R1; // set clock enabling bit for SPI1

  // Initialize PD3 and PD0 for SPI1 alternate function
  GPIO_PORTD_AMSEL_R &= ~0b00001111; // disable analog functionality RD0 to RD3
  GPIO_PORTD_DEN_R   |= 0b00001111;    // Set RD0 to RD3 as digital pin
  GPIO_PORTD_DIR_R   |= 0b00001011;
  GPIO_PORTD_AFSEL_R |= 0b00001111; // enable alternate function of RD0 to RD3
  GPIO_PORTD_PCTL_R  &= ~0x0000FFFF; // assign RD0 and RD3 pins to SPI1
  GPIO_PORTD_PCTL_R  |= 0x00002222;  // assign RD0 and RD3 pins to SPI1

  // Select SPI1 as a Master, POL = 0, PHA = 0, clock = 8 MHz, 8 bit data
  SSI1_CR1_R  = 0;       // disable SPI1 and configure it as a Master
  SSI1_CC_R   = 0;        // Enable System clock Option
  SSI1_CPSR_R = 2;      // Select prescaler value of 2 .i.e 16MHz/2 = 8MHz
  SSI1_CR0_R  = 0x0000F; // 4MHz SPI1 clock, SPI mode, 16 bit data
  SSI1_CR1_R  |= 2;      // enable SPI1
}

BOOLEAN SPI1_Read(INT16U *data) {
  if (SSI1_SR_R & (1 << 2)) { // Wait for Rx-FIFO not empty
    *data = SSI1_DR_R;        // Read received data
    return 1;
  } else {
    return 0;
  }
}

void SPI1_Write(INT16U data) {
  while ((SSI1_SR_R & (1 << 1)) == 0){} // wait until Tx FIFO is not full
    SSI1_DR_R = data; // transmit byte over SSI1Tx line
}

// q_spiDutyCycle = xQueueCreate(20, sizeof(spiDutyCycle_t));
// q_spiAngle     = xQueueCreate(20, sizeof(spiAngle_t));
void vSpi1Task(void *pvParameters) {
  spiDutyCycle_t dutyCycle;
  spiAngle_t angle;

  while (1) {
    // Send duty cycle
    while(xQueueReceive(q_spiDutyCycle, &dutyCycle, 0)) {
      // TODO: Implement data encoding
      INT16U txData;
      SPI1_Write(txData);
    }

    // Receive angle
    while(SSI1_SR_R & (1 << 2)) { // Check for FIFO not empty
      // Receive data
      INT16U rxData = SSI1_DR_R;
      // TODO: Implement data decoding
      xQueueSendToBack(q_spiAngle, &angle, 0);
    }

    vTaskDelay(5 / portTICK_PERIOD_MS);
  }

  // Delete the task if it ever breaks out of the loop above
  vTaskDelete(NULL);
}
