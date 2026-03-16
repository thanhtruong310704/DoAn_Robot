#ifndef CONTROLROBOT_H
#define CONTROLROBOT_H

#include "control/bien.h"
#include "control/kinematic.h"
#include "control/motorstep.h"
#include "control/traj.h"

#define TIME_PICK 750
#define ERROR_THRESHOLD 0.5
class ControlRobot {
 public:
  MotorStep& step1;
  MotorStep& step2;
  MotorStep& step3;
  ControlRobot(MotorStep& m1, MotorStep& m2, MotorStep& m3);
  void init();
  void update(float dt);
  void getDatafrCV2();
  void Run();
  String serialBuffer = "";
 private:
  TrajectoryRobot trajX,trajY,trajZ;
  int bufferIndex=0;
  char data[64];
  StateRobot state;
  bool cAngle = false;
  bool checkState = false;
  float t_traj = 0;
  bool running_traj=false;
  bool test;
  float traj_start_time = 0;
  float dt = 0;
  float TIME_TRAJ = 3.0;
  float actionTime = 0;
  uint8_t serialIndex;
  void getData();
  void PickandPlan();
  void configSetpoint();
  bool isReached();
  bool isBufferAvailable();
  void planTrajectory();
  void startTrajectory();
  void handleCommand(String &cmd);
};
#endif