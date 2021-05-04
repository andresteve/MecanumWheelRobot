#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_PWMServoDriver.h>
#include "Encoder.h"

#define MOTOR_MIN_U           -12.0
#define MOTOR_MAX_U            12.0
#define MOTOR_MAX_SPEED_RPM    120

class Motor {
  
  public:
    Motor(Adafruit_MotorShield* AFMS, uint8_t motorPin, uint8_t encoderPinChA, uint8_t encoderPinChB, uint8_t periodMs, uint16_t wheelDiameter);
    void setMotorSpeedPwm(uint8_t motorSpeedPWM);
    void setPID(float Kp, float Ki, float Kd);
    void setMotorSpeedPIDRpm(float desiredSpeedRpm);
    void motorRun(uint8_t motorSpinDirection);
    void calculateMotorSpeedRpm();
    void calculateMotorPosDeg();
    float getMotorSpeedRpm();
    float getMotorSpeedRads();
    float getMotorPosDeg();
    Adafruit_DCMotor* motor;
    Encoder* encoder;
    
  private:
     typedef union {
      float floatingPoint;
      byte binary[4];
    }binaryFloat;
    
    Adafruit_MotorShield* _AFMS;
    uint8_t _motorPin;
    uint8_t _periodMs;
    uint8_t _pwmValue;
    uint16_t _wheelDiameter;
    float _rpmConst,_degConst, _rpmToRadsConst;
    float _speedRpm, _speedRpmFilter, _posDeg;
    float _speedRpm1, _speedRpm2, _speedRpm3;
    float _Ref, _Kp, _Ki, _Kd;
    float _Ek, _EkOld, _Ik, _IkOld, _Dk, _Uk;
    binaryFloat _EkBin, _DkBin, _IkBin, _RefBin, _speedRpmBin, _speedRpmFilterBin, _UkBin, _pwmValueBin;
};

#endif
