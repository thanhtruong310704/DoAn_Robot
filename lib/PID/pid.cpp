#include<pid.h>

// dung kp ki kd ngoai
/*int32_t PID::output_pid(float dt,float setpoint,float goc_thuc_te,float kp,float ki,float kd){
    float e=setpoint-goc_thuc_te;
    tp += e*dt;
    vp = (e-e_old)/dt;
    out =kp*e+ki*tp+kd*vp;
    out =constrain(out,-100,100);
    out=map(out,-100,100,-200,200);
    e_old=e;
  return out;
}*/
// float PID::outputPID(float dt, float setpoint, float goc_thuc_te, float kp, float ki, float kd) { 
//   float e = setpoint - goc_thuc_te;
//   tp += e * dt;
//   float vp = (e - e_old) / dt;
//   out = kp * e + ki * tp + kd * vp;
//   e_old = e;
//   return out;
// }
PID :: PID () : tp(0),vp(0){}
long PID::outputPID(float dt, float setpoint, float goc_thuc_te, float kp, float ki, float kd) { 

  float e = setpoint - goc_thuc_te; 
  tp += e * dt;

  float max_i = 3000.0; 
  if (tp > max_i) tp = max_i;
  else if (tp < -max_i) tp = -max_i;

  vp = (e - e_old) / dt;
  out = kp * e + ki * tp + kd * vp;
  e_old = e;
  return out;
}
int32_t PID::output_pid(float setpoint,float goc_thuc_te){
  unsigned long now = millis();
  if (now-last>=10)
  {
    float dt =(now-last)/1000.0;
    last = now;
    float e = setpoint - goc_thuc_te;
    tp += e * dt;
    vp = (e - e_old) / dt;
    out = Kp * e + Ki* tp + Kd * vp;
    out = constrain(out, -40, 40);
    e_old=e;
  }
  return out;
}
void PID::reset(){
  tp = 0;       
  e_old = 0;   
  out = 0;
}