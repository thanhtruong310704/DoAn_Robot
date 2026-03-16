#ifndef KINEMATIC_H
#define KINEMATIC_H

class Kinematic {
 public:
  void FK(float& x, float& y, float& z, float q1, float q2, float q3);
  void IK(float& x, float& y, float& z, float& q1, float& q2, float& q3);

 private:
  float q1_rad = 0;
  float q2_rad = 0;
  float q3_rad = 0;
};
#endif