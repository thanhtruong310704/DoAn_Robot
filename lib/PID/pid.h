#ifndef PID_H
#define PID_H
#include <Arduino.h>
class PID
{
private:
  float tp;
  float vp;
  unsigned long last;
  float e_old;
 // float out;

public:
  PID();
  float Kp = 2;
  float Ki = 0.0005;
  float Kd = 0.001;
  long out;
  //  int32_t output_pid(float dt,float setpoint,float goc_thuc_te,float kp,float ki,float kd);
  int32_t output_pid(float setpoint, float goc_thuc_te);
  long outputPID(float dt, float setpoint, float goc_thuc_te, float kp, float ki, float kd);
  void reset();
};

#endif
