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
    void setMotorSpeedPID(float desiredSpeed);
    void motorRun(uint8_t motorSpinDirection);
    void calculateMotorSpeed();
    float getMotorSpeed();
    Adafruit_DCMotor* motor;
    Encoder* encoder;
    
  private:
    Adafruit_MotorShield* _AFMS;
    uint8_t _motorPin;
    uint8_t _periodMs;
    uint8_t _pwmValue;
    uint16_t _wheelDiameter;
    float _radsConst;
    float _speed, _speedFilter, _posDeg;
    float _speed1, _speed2, _speed3;
    float _Ref, _Kp, _Ki, _Kd;
    float _Ek, _EkOld, _Ik, _IkOld, _Dk, _Uk;
};

#endif
