#include "control/motorstep.h"

MotorStep ::MotorStep(int stepPin, int dirPin, int enPin, float gear, int channel)
  : step(AccelStepper::DRIVER, stepPin, dirPin, enPin) {
  dir = dirPin;
  stepp = stepPin;
  en = enPin;
  ch = channel;
  tst = gear;
}

void MotorStep ::begin() {
  pinMode(dir, OUTPUT);
  pinMode(en, OUTPUT);
  pinMode(stepp, OUTPUT);
  step.setMaxSpeed(3000);
  step.setAcceleration(1000);
}

float MotorStep ::readAngle() {
  setChannel(ch);

  float sobuoc = as5600.getRawAngle();
  float goc = sobuoc * 360.0 / 4096;

  if (state) {
    startAngle = goc;
    lastAngle = goc;
    state = false;
  }
  float tam = goc - lastAngle;

  // if (tam <0)    tam += 360.0;
  // else if (tam<-180) tam+=360;
  if (tam > 180) total_round++;
  if (tam < -180) total_round--;

  float total_angle = (total_round * 360.0) +  (- goc + startAngle);
  lastAngle = goc;
  return total_angle / tst;
}

void MotorStep ::as5600begin() {

  setChannel(ch);
  as5600.begin();

  if (!as5600.begin()) {
    Serial.println("fail kenh ");
    Serial.println(ch);
  } else {
    Serial.print("ket noi ok kenh ");
    Serial.println(ch);
  }

}

// void MotorStep ::caclu_pid(float dt) {
//   output = pid.outputPID(dt, setpoint, goc, Kp, Ki, Kd);
// }

// chon kenh doc as5600
void MotorStep ::setChannel(uint8_t channel) {
  if (channel > 7)
    return;
  else {
    Wire.beginTransmission(0x70);
    Wire.write(1 << channel);
    Wire.endTransmission();
  }
}
  


// dùng đc 3 lan trong thanh ghi BURN
// nếu dùng cái này sẽ ko dung đc BURN setting
// Burn_Angle Command (ZPOS, MPOS)
/*  The host microcontroller can perform a permanent
programming of ZPOS and MPOS with a BURN_ANGLE
command. To perform a BURN_ANGLE command, write the
value 0x80 into register 0xFF. The BURN_ANGLE command can
be executed up to 3 times. ZMCO shows how many times ZPOS
and MPOS have been permanently written.
This command may only be executed if the presence of the
magnet is detected (MD = 1).*/

bool MotorStep ::Burn_Angle_Command() {
  
  setChannel(ch);

  Wire.beginTransmission(AS5600_DEFAULT_ADDR);
  Wire.write(AS5600_REG_BURN);
  Wire.write(0x80);
  delayMicroseconds(20);
  uint8_t error = Wire.endTransmission();
  if (error == 0) {
    delay(50); 
    return true; 
    } else {
    return false; 
  }
}
/*  The host microcontroller can perform a permanent writing of
MANG and CONFIG with a BURN_SETTING command. To
perform a BURN_SETTING command, write the value 0x40 into
register 0xFF.
MANG can be written only if ZPOS and MPOS have never been
permanently written (ZMCO = 00). The BURN_ SETTING
command can be performed only one time*/

bool MotorStep ::Burn_Setting_Command() {

  setChannel(ch);

  Wire.beginTransmission(AS5600_DEFAULT_ADDR);
  Wire.write(AS5600_REG_BURN);
  Wire.write(0x40);
  delayMicroseconds(20);
  Wire.endTransmission();
  
  uint8_t error = Wire.endTransmission();
  if (error == 0) {
    delay(10); 
    return true; 
    } else {
    return false; 
  }
}

// kiem tra so lan da burn
void MotorStep :: readBurn() {

  setChannel(ch);

  Wire.beginTransmission(AS5600_DEFAULT_ADDR);
  Wire.write(AS5600_REG_ZMCO);
  Wire.endTransmission();

  Wire.requestFrom(AS5600_DEFAULT_ADDR, 1);

  int value = Wire.read();
  if (value <= 3) {
    Serial.print("so lan da ghi:  ");
    Serial.println(value);
  } else
    Serial.print("da het so lan ghi");
}

void MotorStep :: setZero(){

  uint16_t sobuoc = as5600.getRawAngle();
  as5600.setZPosition(sobuoc);

 }