#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "emp_type.h"
#include "projectdefs.h"

// UART0 queues
extern xQueueHandle q_uartDebug;
extern xQueueHandle q_uartAngle; // Send current angle
extern xQueueHandle q_uartSetpoint; // Receive setpoint
// UART0 Mutex
extern xSemaphoreHandle uart0RxMutex;
extern xSemaphoreHandle uart0TxMutex;

void vUart0Init() {
  // Setup UART0
  // Enable the UART0 clock
  SYSCTL_RCGCUART_R  |= SYSCTL_RCGCUART_R0;
  // Set the alternate function for PA0 and PA1
  GPIO_PORTA_AFSEL_R |= 0b00000011;
  // Enable the alternate function for PA0 and PA1
  GPIO_PORTA_PCTL_R  |= 0x00000011;
  // Enable the GPIO pins for digital function (PA0 and PA1)
  GPIO_PORTA_DEN_R   |= 0b00000011;
  // Handle direction of the pins
  GPIO_PORTA_DIR_R   |= 0b00000010;

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

void vSendCharUart(INT8U chr) {
  while (UART0_FR_R & (1 << 5)){}
  UART0_DR_R = chr;
}

void vSendStringUart(const char *str) {
  while (*str) {
    vSendCharUart(*str);
    str++;
  }
}

void vSendDebugUart() {
  if(uxQueueMessagesWaiting(q_uartDebug) == 0) { return; }

  // Send debug data
  uartDebug_t debug_data;
  while(xQueueReceive(q_uartDebug, &debug_data, 0)) {
    vSendStringUart(debug_data.string);
  }
}

// Format of angle data:
// MBRS DDDD
// A: Axis (1 = Pan, 0 = Tilt)
// B: Bits  (1 = High bits, 0 = Low bits)
// R: Relative (1 = Relative, 0 = Absolute)
// S: Sign  (Data if B=0, Sign if B=1)
// D: Data  (Always)

void vSendAngleData(uartAngle_t angle_data) {
  INT8U lowByte  = 0;
  INT8U highByte = (1 << 6);

  // Set axis bit
  if (angle_data.axis == PAN) {
    lowByte  |= (1 << 7);
    highByte |= (1 << 7);
  }

  // Check if negative
  if (angle_data.angle < 0) {
    angle_data.angle = -angle_data.angle;
    // Set sign bit
    highByte |= (1 << 4);
  }

  // Set data bits
  lowByte  |= angle_data.angle & 0b00011111;
  highByte |= (angle_data.angle >> 5) & 0b00001111;

  // Relative bit is ignored (Always absolute)

  // Send data
  vSendCharUart(lowByte);
  vSendCharUart(highByte);
}

void vSendAnglesUart(){
  // Check if there are any messages in the queue
  if(!uxQueueMessagesWaiting(q_uartAngle)) { return; }

  if (xSemaphoreTake(uart0TxMutex, 1)) {
    // Send angle data (pan and tilt
    uartAngle_t angle_data;
    // Empty the queue to get the latest angles
    while (xQueueReceive(q_uartAngle, &angle_data, 0)) {
      vSendAngleData(angle_data);
    }

    // Release the mutex
    xSemaphoreGive(uart0TxMutex);
  }
}

void vDataToSetpoint(INT8U data, uartAngle_t *setpoint){
  // *setpoint is already determined to be either pan or tilt

  // Check if the data is negative
  BOOLEAN isNegative = setpoint->angle < 0 ? TRUE : FALSE;
  // Make positive for bit manipulation
  if (isNegative){
    setpoint->angle = -setpoint->angle;
  }

  // High bits
  if (data & (1 << 6)) {
    // Remove old high bits
    setpoint->angle &= 0x1F;
    // Set new high bits
    setpoint->angle |= (data & 0x0F) << 5;
    // Set sign
    isNegative = data & (1 << 4) ? TRUE : FALSE;
  } else {
    // Low bits
    // Remove old low bits
    setpoint->angle &= 0xFFE0;
    // Set new low bits
    setpoint->angle |= data & 0x1F;
  }

  if (isNegative){
    setpoint->angle = -setpoint->angle;
  }

  if (data & (1 << 5)) {
    // Set relative
    setpoint->relative = TRUE;
  } else {
    // Set absolute
    setpoint->relative = FALSE;
  }
}

void vReceiveSetpointsUart(){
  // Return if there is no data
  if(UART0_FR_R & (1 << 4)){ return; }

  if (xSemaphoreTake(uart0RxMutex, 1)) {
    // Initialize setpoints
    static uartAngle_t pan_setpoint = {PAN, 0, FALSE};
    static uartAngle_t tilt_setpoint = {TILT, 0, FALSE};
    INT8U rxData;
    while (!(UART0_FR_R & (1 << 4))) { // Receive 4 bytes
        rxData = UART0_DR_R;
        if (rxData & (1 << 7)) {
          vDataToSetpoint(rxData, &pan_setpoint);
        } else {
          vDataToSetpoint(rxData, &tilt_setpoint);
        }
    }

    xQueueSendToBack(q_uartSetpoint, &pan_setpoint, 0);
    xQueueSendToBack(q_uartSetpoint, &tilt_setpoint, 0);

    // Release the mutex
    xSemaphoreGive(uart0RxMutex);
  }
}

void vUart0Task(void *pvParameters) {
  // Task loop
  while (1) {
    // Send debug
    vSendDebugUart();

    // Send angle
    vSendAnglesUart();

    // Receive setpoint
    vReceiveSetpointsUart();

    // Delay task
    vTaskDelay(50 / portTICK_RATE_MS); 
  }

  // Delete the task if it ever breaks out of the loop above
  vTaskDelete(NULL);
}
