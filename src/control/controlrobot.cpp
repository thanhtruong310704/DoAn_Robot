#include "control/controlrobot.h"
#include "debug.h"

ControlRobot ::ControlRobot(MotorStep& m1, MotorStep& m2, MotorStep& m3)
    : step1(m1), step2(m2), step3(m3), state(WAIT), t_traj(0), test(false) {}

void ControlRobot ::init() {
  step1.as5600begin();
  step1.begin();
  step2.as5600begin();
  step2.begin();
  step3.as5600begin();
  step3.begin();
  configSetpoint();
  pinMode(ReLay, OUTPUT);
}

void ControlRobot ::update(float denta_t) {
  step1.goc = step1.readAngle();
  step2.goc = step2.readAngle() - 90;
  step3.goc = -step3.readAngle() - step2.goc;
  step1.output = step1.pid.outputPID(denta_t, step1.setpoint, step1.goc, 9, 0.001, 0.05);  // 4.7, 0.001, 0.7
  step2.output = step2.pid.outputPID(denta_t, step2.setpoint, step2.goc, 9, 0.001, 0.05);
  step3.output = step3.pid.outputPID(denta_t, step3.setpoint, step3.goc, 9, 0.001, 0.01);
  if (checkState == true && isReached()) { // đã đến nơi gửi lên python làm tiếp theo
      Serial.println("ARRIVED"); 
      checkState = false;    
  }
}
void ControlRobot ::getDatafrCV2() {
  getData();
  //planTrajectory();
}

void ControlRobot ::Run() {
  step1.output = constrain(step1.output, -step1.step.maxSpeed(), step1.step.maxSpeed());
  step1.step.setSpeed(-step1.output*1.5);
  step2.output = constrain(step2.output, -step2.step.maxSpeed(), step2.step.maxSpeed());
  step2.step.setSpeed(-step2.output*1.5);
  step3.output = constrain(step3.output, -step3.step.maxSpeed(), step3.step.maxSpeed());
  step3.step.setSpeed(step3.output*1.5);
    step1.step.runSpeed();
    step2.step.runSpeed();
    step3.step.runSpeed();
  
  #ifdef DEBUG_OUT
    Serial.print("q1: ");
    Serial.print(step1.output);
    Serial.print(" q2: ");
    Serial.print(step2.output);
    Serial.print(" q3: ");
    Serial.println(step3.output);
  #endif
}

void ControlRobot ::configSetpoint() {
  step1.setpoint = step1.readAngle();
  step2.setpoint = step2.readAngle() - 90;
  step3.setpoint = -step3.readAngle() - step2.setpoint;
}



// check da den noi chua
bool ControlRobot::isReached() {
  if (running_traj) return false;
  float e1 = abs(step1.setpoint - step1.goc);
  float e2 = abs(step2.setpoint - step2.goc);
  float e3 = abs(step3.setpoint - step3.goc);

  return (e1 < ERROR_THRESHOLD && e2 < ERROR_THRESHOLD && e3 < ERROR_THRESHOLD);
}

/* void ControlRobot ::planTrajectory() {
  if (test) startTrajectory();
  if (!running_traj) return;
    unsigned long now_time = millis();
    t_traj = (now_time - traj_start_time) / 1000.0;
    if (t_traj > TIME_TRAJ) {
      t_traj =  TIME_TRAJ;
      running_traj = false;
  }
  step1.setpoint = trajX.getPosition3(t_traj);
  step2.setpoint = trajY.getPosition3(t_traj);
  step3.setpoint = trajZ.getPosition3(t_traj);

}
void ControlRobot::startTrajectory() {
  test = false;
  step1.goc = step1.readAngle();
  step2.goc = step2.readAngle() - 90;
  step3.goc = -step3.readAngle() - step2.goc;
  trajX.cubicTrajectory(step1.goc, step1.qcur, step1.v0, step1.vf,  TIME_TRAJ);
  trajY.cubicTrajectory(step2.goc, step2.qcur, step2.v0, step2.vf,  TIME_TRAJ);
  trajZ.cubicTrajectory(step3.goc, step3.qcur, step3.v0, step3.vf,  TIME_TRAJ);
  traj_start_time = millis();
  running_traj = true;
} */


void ControlRobot::getData() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      data[bufferIndex] = '\0'; 

      if (strcmp(data, "C") == 0) {
        checkState = true;
      }
      else if (strcmp(data, "P1") == 0) {
        digitalWrite(ReLay, HIGH);
      }
      else if (strcmp(data, "P0") == 0) {
        digitalWrite(ReLay, LOW);
      }
      else if (strcmp(data, "A") == 0) {
        step1.Burn_Angle_Command();
        step2.Burn_Angle_Command();
        step3.Burn_Angle_Command();
      }
      else if (strcmp(data, "G") == 0) {
        Serial.print("A:");
        Serial.print(step1.goc); Serial.print(",");
        Serial.print(step2.goc); Serial.print(",");
        Serial.println(step3.goc);
      }
      else if (strchr(data, ',') != NULL) {
        char* ptr = strtok(data, ",");
        if (ptr) {
          float q1 = atof(ptr); 
          ptr = strtok(NULL, ",");
          if (ptr) {
            float q2 = atof(ptr);
            ptr = strtok(NULL, ",");
            if (ptr) {
              float q3 = atof(ptr);
              
              step1.setpoint = q1;
              step2.setpoint = q2;
              step3.setpoint = q3;
              
              // Báo lại cho Python biết đã nhận OK
              Serial.println("OK"); 
              test = true;
              traj_start_time = 0;
            }
          }
        }
      }
      
      bufferIndex = 0;
    }
    else {
      if (bufferIndex < 63) {
        data[bufferIndex] = c;
        bufferIndex++;
      }
      else {
        bufferIndex = 0; 
      }
    }
  }
}

