#ifndef _PID_H
#define _PID_H
typedef struct{
  // PID controller constants
  float Kp, Ki, Kd;
  // Sampling time
  float T;
  // Motor voltage limits
  float maxLim, minLim;
  // 
  float integrator;
} PID;

void vController_task();
void vController_init();

#endif // _PID_H
