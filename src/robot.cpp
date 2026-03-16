#include "debug.h"
#include "config.h"
ControlRobot robot(motor1, motor2, motor3);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setWireTimeout(25000, true);
  robot.init();
}

void loop() {
  robot.getDatafrCV2();
  unsigned long now = micros();
  if (now - last >= stepTime) {
    delta_t = (now - last) / 1000000.0f;
    last = now;
    robot.update(delta_t);
  }
  robot.Run();
  #ifdef DEBUG_ANGLE
    Serial.print(robot.step1.goc);
    Serial.print(",");
    Serial.print(robot.step1.setpoint); 
    Serial.print(",");

    // Trục 2
    Serial.print(robot.step2.goc);
    Serial.print(",");
    Serial.print(robot.step2.setpoint);
    Serial.print(",");

    // Trục 3 (Lưu ý cái cuối cùng dùng println)
    Serial.print(robot.step3.goc);
    Serial.print(",");
    Serial.println(robot.step3.setpoint);
  #endif  
  
}
