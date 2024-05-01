#ifndef _SPI1_H
#define _SPI1_H
#include "emp_type.h"
#include "projectdefs.h"

typedef struct {
  motorAxis_t axis;
  INT8S angle;
} spiAngle_t;

typedef struct {
  motorAxis_t axis;
  INT16S dutyCycle;
} spiDutyCycle_t;

extern void vSpi1Init();
extern void vSpi1Task(void *pvParameters);

#endif // _SPI1_H
