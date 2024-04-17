#ifndef _MODULE_H
#define _MODULE_H
/***********************************************
 * Univeristy of Southern Denmark
 * Embedded Programming (EMP)
 *
 * MODULENAME: module.h
 * PROJECT: template h-file
 * DESCRIPTION: Empty header template
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
/***************** Defines ********************/
#define FALSE 0
#define TRUE 1

typedef enum {
  Index0,
  Index1,
  Sensor1A,
  Sensor1B,
  Sensor2A,
  Sensor2B,
  SW1,
  SW2
} input_port;

typedef enum { ENA, IN1A, IN2A, ENB, IN1B, IN2B } output_port;

typedef enum {
  OFF = 0b000,
  GREEN = 0b100,
  BLUE = 0b010,
  CYAN = 0b110,
  RED = 0b001,
  YELLOW = 0b101,
  MAGENTA = 0b011,
  WHITE = 0b111
} LED_Color;
/***************** Variables ******************/
/***************** Functions ******************/
extern void set_led_color( LED_Color color);
extern void setup_gpio(void);
extern void set_port(output_port port, INT8U value);
extern INT8U get_port(input_port port);
/***********************************************
 * Input:
 * Output:
 * Function:
 **********************************************/
/***************** End of module **************/

#endif // _MODULE_H