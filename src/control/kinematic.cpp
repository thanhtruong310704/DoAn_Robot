#include <Arduino.h>
#include "control/kinematic.h"

void Kinematic :: FK(float& x, float& y, float& z, float q1, float q2, float q3) {
  float l1 = 155.2, l2 = 140, l3 = 190;
  q1_rad = radians(q1);
  q2_rad = radians(q2);
  q3_rad = radians(q3);
  x = cos(q1_rad) * (l2 * cos(q2_rad) + l3 * cos(q2_rad + q3_rad));
  y = sin(q1_rad) * (l2 * cos(q2_rad) + l3 * cos(q2_rad + q3_rad));
  z = l1 - (l2 * sin(q2_rad) + l3 * sin(q2_rad + q3_rad));
}

void Kinematic :: IK(float& x, float& y, float& z, float& q1, float& q2, float& q3) {
  float l1 = 155.2, l2 = 140, l3 = 190;  // cũ 135.2
  q1 = atan2(y, x) * 180 / PI;
  float c3 = ((x * cos(radians(q1)) + y * sin(radians(q1))) * (x * cos(radians(q1)) + y * sin(radians(q1))) + (l1 - z) * (l1 - z) - l3 * l3 - l2 * l2) / (2 * l2 * l3);
  if (c3 > 1) c3 = 1;
  if (c3 < -1) c3 = -1;
  float s3 = sqrt(1 - c3 * c3);
  q3 = atan2(s3, c3) * 180.0 / PI;
  float s2 = ((l1 - z) * (l3 * c3 + l2) - (x * cos(radians(q1)) + y * sin(radians(q1))) * (l3 * s3)) /
             (l3 * l3 + l2 * l2 + 2 * l2 * l3 * c3);
  float c2 = ((l1 - z) * (l3 * s3) + (x * cos(radians(q1)) + y * sin(radians(q1))) * (l3 * c3 + l2)) /
             (l3 * l3 + l2 * l2 + 2 * l2 * l3 * c3);
  q2 = atan2(s2, c2) * 180.0 / PI;
}
