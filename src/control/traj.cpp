//#include <Arduino.h>
#include "control/traj.h"
#include <math.h>
void TrajectoryRobot ::quinticTrajectory(float qi, float qf, float vi, float vf,float ai, float af, float t) {
  a0 = qi;
  a1 = vi;
  a2 = ai / 2;
  a3 = (20 * (qf - qi) - (8 * vf + 12 * vi) * t - (3 * ai - af) * t * t) / (2 * t * t*t);
  a4 = (30 * (qi - qf) + (14 * vf + 16 * vi) * t + (3 * ai - 2 * af) * t * t) / (2 * t * t*t*t);
  a5 = (12 * (qf - qi) - 6 * (vf + vi) * t - (ai - af) * t * t) / (2 *  t * t*t*t*t);
  T = t;
}

void TrajectoryRobot ::cubicTrajectory(float qi, float qf, float vi, float vf, float t){
  a0 = qi;
  a1 = vi;
  a2 = (3 * (qf - qi) / (t * t)) - (2 * vi + vf) / t;
  a3 = (2 * (qi - qf) / (t * t * t)) + (vi + vf) / (t * t);
  T = t;
}

float TrajectoryRobot ::getPosition3(float t) {
  if (t > T) t = T;
  return a0 + a1 * t + a2 * t * t + a3 * t * t * t;
}
float TrajectoryRobot::getPosition5(float t) {
  if (t > T) t = T;
  return a0 + a1 * t + a2 * t * t + a3 * t * t * t + a4 * t * t * t * t + a5 * t * t * t * t * t;
}


float TrajectoryRobot :: getTime(float qi, float qf,unsigned int vmax,int K){
  if (fabs(qf - qi)== 0) return 0;
  return  fabs(K*(qf - qi)/ vmax);
}
