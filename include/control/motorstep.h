#ifndef MOTORSTEP_H
#define MOTORSTEP_H


#include <Arduino.h>
#include <AccelStepper.h>
#include <pid.h>
#include <Adafruit_AS5600.h>
#include <Wire.h>


class MotorStep {
 public:
  AccelStepper step;
  PID pid;
  int16_t startAngle = 0;
  float output = 0.0;
  float setpoint = 0;
  float goc = 0;
  float Kp = 4.7; // 4.7,0.001, 0.7
  float Ki = 0.001;
  float Kd = 0.7;
  int dir;
  int en;
  int stepp;
  float qcur = 0;
  int ch;
  float tst = 0;
  int16_t total_round = 0;
  int v0 = 0, vf = 0;
  int a0 = 0, af = 0;
  int time = 0;
  float qf=0.0;
  MotorStep(int stepPin, int dirPin, int enPin, float gear, int channel);
  float readAngle();
  void begin();
  void as5600begin();
  bool Burn_Angle_Command();
  bool Burn_Setting_Command();
  void readBurn();
  void setZero();
private:
  Adafruit_AS5600 as5600 = Adafruit_AS5600();
  bool state = true;
  bool start = 0;
  float dt = 0;
  float lastAngle = 0;
  uint32_t last=0;
  void setChannel(uint8_t channel);
};
#endif