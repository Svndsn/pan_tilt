#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "tm4c123gh6pm.h"
#include "emp_type.h"
#include "uart0.h"
#include "spi1.h"

#define ANGLE_COUNT_CENTER 512
// 360 / 450
#define ANGLE_COUNT_RATIO_TILT 0.8f 
// 360 / 360
#define ANGLE_COUNT_RATIO_PAN 1.4f 

// SPI1 queues
extern xQueueHandle q_spiDutyCycle; // Send duty cycle
extern xQueueHandle q_spiAngle; // Receive angle
// SPI1 Mutex
extern xSemaphoreHandle m_spiDutyCycle;
extern xSemaphoreHandle m_spiAngle;

// Debug
extern xQueueHandle q_uartDebug;
extern xSemaphoreHandle m_uartDebug;


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

BOOLEAN Spi1Read(INT16U *data) {
  if (SSI1_SR_R & (1 << 2)) { // Rx-FIFO not empty
    *data = SSI1_DR_R;        // Read received data
    return 1;
  } else {
    return 0;
  }
}

void Spi1Write(INT16U data) {
  while ((SSI1_SR_R & (1 << 1)) == 0){} // wait until Tx FIFO is not full

  // transmit byte over SSI1Tx line
  SSI1_DR_R = data; 
}

void vSendDutyCycleData(spiDutyCycle_t dutyCycle){
  INT16U data = 0;

  // Set axis
  if(dutyCycle.axis == PAN){
    data |= (1 << 15);
  }

  // Set the sign
  if(dutyCycle.dutyCycle < 0){
    dutyCycle.dutyCycle = -dutyCycle.dutyCycle;
    data |= (1 << 14);
  }

  // Set the data
  data |= dutyCycle.dutyCycle & 0x3FF;

  // Send the data
  Spi1Write(data);
}

// Send angle
void vSendDutyCyclesSpi() {
  // Return if there are no messages in the queue
  if (!uxQueueMessagesWaiting(q_spiDutyCycle)) { return; }

  // Get the mutex
  if (xSemaphoreTake(m_spiDutyCycle, 1)) {
    spiDutyCycle_t dutyCycle;
    static spiDutyCycle_t dutyCyclePan = {PAN, 0};
    static spiDutyCycle_t dutyCycleTilt = {TILT, 0};
    // Send duty cycle
    while (xQueueReceive(q_spiDutyCycle, &dutyCycle, 0)) {
      if (dutyCycle.axis == PAN) {
        dutyCyclePan = dutyCycle;
      } else {
        dutyCycleTilt = dutyCycle;
      }
    }
    // Send the duty cycle
    vSendDutyCycleData(dutyCyclePan);
    vSendDutyCycleData(dutyCycleTilt);

    // Release the mutex
    xSemaphoreGive(m_spiDutyCycle);
  }
}

void vDataToAngle(INT16U data, spiAngle_t *angle){
  INT16S count = data & 0x3FF;
  FP32 angleValue;
  // Translate the count to an angle
  // Angle 0 is at 512
  if(angle->axis == PAN){
    angleValue = (count - ANGLE_COUNT_CENTER) * ANGLE_COUNT_RATIO_PAN;
  } else {
    angleValue = (count - ANGLE_COUNT_CENTER) * ANGLE_COUNT_RATIO_TILT;
  }

  // Set the angle
  angle->angle = angleValue;
}

// Receive setpoint
void vReceiveCountsSpi(){
  if (!(SSI1_SR_R & (1 << 2))) { return; } // Return if there is no data

  // Receive angle
  // The data packets is constructed of 16 bits:
  // ASXX XXDD DDDD DDDD
  // A: Axis (1 = Pan, 0 = Tilt)
  // S: Sign (1 = Negative, 0 = Positive)
  // D: Data
  // X: Unused

  // Get the mutex
  if (xSemaphoreTake(m_spiAngle, 1)) {
    static spiAngle_t panAngle = {PAN, 0};
    static spiAngle_t tiltAngle = {TILT, 0};
    INT16U rxData;

    while (Spi1Read(&rxData)) {
      // if ((rxData & 0x3FF) > 980 || (rxData & 0x3FF) < 40) {
      if (rxData == 0) {//65535
        // Should never happen
        continue;
      }

      // Check if the data is for pan or tilt
      if (rxData & (1 << 15)) {
        // uartDebug_t buffer;
        // itoa(rxData, buffer.string, 10);
        // xQueueSendToBack(q_uartDebug, &buffer, 0);
        vDataToAngle(rxData, &panAngle);
        xQueueSendToBack(q_spiAngle, &panAngle, 0);
      } else {
        vDataToAngle(rxData, &tiltAngle);
        xQueueSendToBack(q_spiAngle, &tiltAngle, 0);
      }
    }

    // Release the mutex
    xSemaphoreGive(m_spiAngle);
  }
}

void vSpi1Task(void *pvParameters) {
  while (1) {
    // Send the duty cycle for the motors
    vSendDutyCyclesSpi();

    // Receive the angle count
    vReceiveCountsSpi();

    // Delay task
    vTaskDelay(2 / portTICK_PERIOD_MS);
  }

  // Delete the task if it ever breaks out of the loop above
  vTaskDelete(NULL);
}
