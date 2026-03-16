#include "control/controlrobot.h"

MotorStep motor1(54, 55, 38, 4.5, 0); // step dir en gear channel of TCA4895A
MotorStep motor2(60, 61, 56, 4.5, 2);
MotorStep motor3(46, 48, 62, 4.5, 1);

unsigned long last = 0;
const unsigned long stepTime = 5000;
float delta_t = 0;

