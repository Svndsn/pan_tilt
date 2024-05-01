#include "uart0.h"
#include "FreeRTOS.h"
#include "emp_type.h"
#include "portmacro.h"
#include "queue.h"
#include "task.h"
#include "tm4c123gh6pm.h"

extern xQueueHandle q_uartDebug;
extern xQueueHandle q_uartAngle;
extern xQueueHandle q_uartSetpoint;

void send_char(INT8U chr) {
  while (UART0_FR_R & (1 << 5)){}
  UART0_DR_R = chr;
}

void send_string(const char *str) {
  while (*str) {
    send_char(*str);
    str++;
  }
}


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

// Format of angle data:
// ABRSDDDD
// A: Angle (1 = Pan, 0 = Tilt)
// B: Bits  (1 = High bits, 0 = Low bits)
// R: Relative (1 = Relative, 0 = Absolute)
// S: Sign  (Data if B=0, Sign if B=1)
// D: Data  (Always)
void vSendAngleData(uartAngle_t angle_data) {
  INT8U lowByte  = 0;
  INT8U highByte = (1 << 6);

  // Set angle bit
  if (angle_data.axis == PAN) {
    lowByte  |= (1 << 7);
    highByte |= (1 << 7);
  }

  // Set sign bit
  if (angle_data.angle < 0) {
    angle_data.angle = -angle_data.angle;
    highByte |= (1 << 4);
  }

  // Set data bits
  lowByte  |= angle_data.angle & 0b00011111;
  highByte |= (angle_data.angle >> 5) & 0b00001111;

  // Send data
  send_char(lowByte);
  send_char(highByte);
}

void vSendAngles(){
  // Check if there are any messages in the queue
  if(uxQueueMessagesWaiting(q_uartAngle) == 0) {
    // No messages in the queue
    return;
  }

  uartAngle_t angle_data;
  uartAngle_t pan_angle;
  uartAngle_t tilt_angle;
  // Empty the queue to get the latest angles
  while(xQueueReceive(q_uartAngle, &angle_data, 0)) {
    if(angle_data.axis == PAN){
      pan_angle = angle_data;
    } else {
      tilt_angle = angle_data;
    }
  }
  vSendAngleData(pan_angle);
  vSendAngleData(tilt_angle);
}

void vDataToSetpoint(INT8U data, uartAngle_t *setpoint){

}

void vReceiveSetpoint(){
  // Return if there is no data
  if(UART0_FR_R & (1 << 4)){
    return;
  }

  uartAngle_t pan_setpoint;
  uartAngle_t tilt_setpoint;
  INT8U rxData;
  while(!(UART0_FR_R & (1 << 4))) { // Check for FIFO not empty
    rxData = UART0_DR_R;
    if(rxData & (1 << 7)){
      vDataToSetpoint(rxData, &pan_setpoint);
    }else{
      vDataToSetpoint(rxData, &tilt_setpoint);
    }
  }
  xQueueSendToBack(q_uartAngle, &pan_setpoint, 0);
  xQueueSendToBack(q_uartAngle, &tilt_setpoint, 0);
  // xQueueSendToBack(q_uartSetpoint, &setpoint_data, 0);
}

void vSendDebug() {
  if(uxQueueMessagesWaiting(q_uartDebug) == 0) {
    return;
  }

  uartDebug_t debug_data;
  while(xQueueReceive(q_uartDebug, &debug_data, 0)) {
    send_string(debug_data.string);
  }
}

void vUart0Task(void *pvParameters) {
  // Task loop
  while (1) {
    // Send debug
    vSendDebug();

    // Send angle
    vSendAngles();

    // Receive setpoint
    vReceiveSetpoint();

    // Delay task
    vTaskDelay(50 / portTICK_RATE_MS); 
  }

  // Delete the task if it ever breaks out of the loop above
  vTaskDelete(NULL);
}
