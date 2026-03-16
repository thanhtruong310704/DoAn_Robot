#ifndef TRAJ_H
#define TRAJ_H

class TrajectoryRobot {
 public:
  float T;
  void quinticTrajectory(float qi, float qf, float vi, float vf,float ai, float af, float t);
  void cubicTrajectory(float qi, float qf, float vi, float vf,float t);
  float getPosition3(float t);
  float getPosition5(float t);
  float getTime(float qi, float qf,  unsigned int vmax, int K);
 private:
  float a0=0, a1=0, a2=0, a3=0, a4=0, a5=0;
};
#endif