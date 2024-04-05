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
enum LED_Color {
  OFF = 0b000,
  GREEN = 0b100,
  BLUE = 0b010,
  CYAN = 0b110,
  RED = 0b001,
  YELLOW = 0b101,
  MAGENTA = 0b011,
  WHITE = 0b111
};
/***************** Variables ******************/
/***************** Functions ******************/
extern void set_led_color(enum LED_Color color);
extern void setup_gpio(void);
/***********************************************
 * Input:
 * Output:
 * Function:
 **********************************************/
/***************** End of module **************/

#endif // _MODULE_H
