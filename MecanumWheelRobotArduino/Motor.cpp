#include "Motor.h"


/*
 * Constructor: Motor
 * ----------------------------
 * Class constructor.
 * This class control a motor and its encoder.
 *   param:
 *    @AFMS: pointer to Adafruit Motorshield
 *    @motorPin: shield motor pin (1,2,3,4)
 *    @encoderPinChA: arduino pin where is connected the encoder (ChA)
 *    @encoderPinChB: arduino pin where is connected the encoder (ChB)
 *    @periodMs: cycle period in milliseconds
 *    @wheelDiameter: wheel diameter in mm
 */
Motor::Motor(Adafruit_MotorShield* AFMS, uint8_t motorPin, uint8_t encoderPinChA, uint8_t encoderPinChB, uint8_t periodMs, uint16_t wheelDiameter){
  _AFMS = AFMS;
  motor = _AFMS->getMotor(motorPin);
  encoder = new Encoder(encoderPinChA, encoderPinChB);
  _wheelDiameter = wheelDiameter;
  _periodMs = periodMs;
  _motorPin = motorPin;
  _rpmConst = (float) (60000.0/_periodMs) / (encoder->getEncoderResolution()); 
  _degConst = (float) 360.0 / (encoder->getEncoderResolution());
  _rpmToRadsConst = (float) (2*3.14) / 60.0;
  _speedRpm = 0; _speedRpm1 = 0; _speedRpm2 = 0; _speedRpm3 = 0;
  _Ref=0;_Ek=0; _Ik=0;_IkOld = 0; _EkOld = 0;
}

/*
 * Function: setMotorSpeedPwm
 * ----------------------------
 * Function to control motor speed using pwm
 *   param:
 *    @motorSpeedPwm: pwm speed value (0-255).
 */
void Motor::setMotorSpeedPwm(uint8_t motorSpeedPwm){
  if(motorSpeedPwm > 255)       motorSpeedPwm = 255;
  else if(motorSpeedPwm < 0)    motorSpeedPwm = 0;
  motor->setSpeed(motorSpeedPwm);
}


/*
 * Function: motorRun
 * ----------------------------
 * Function to control motor direction
 *   param:
 *    @motorSpinDirection: FORWARD, BACKWARD, RELEASE
 */
void Motor::motorRun(uint8_t motorSpinDirection){
  motor->run(motorSpinDirection);
}


/*
 * Function: calculateMotorSpeedRpm
 * ----------------------------
 * Function to calculate motor speed in rpm.
 * The function use the encoder value associated with the motor.
 */
void Motor::calculateMotorSpeedRpm(){
  long currentEncValue = encoder->getEncoderValue();
  long oldEncValue = encoder->getEncoderValueOld();
  _speedRpm = (currentEncValue - oldEncValue) * _rpmConst;
  encoder->setEncoderValueOld(currentEncValue);
  _speedRpmFilter = 0.2*_speedRpm + 0.2*_speedRpm1 + 0.3*_speedRpm2 + 0.3*_speedRpm3;
  _speedRpm3 = _speedRpm2; _speedRpm2 = _speedRpm1; _speedRpm1 = _speedRpm;
}


/*
 * Function: calculateMotorPosDeg
 * ----------------------------
 * Function to calculate position in degree.
 * The function use the encoder value associated with the motor.
 */
void Motor::calculateMotorPosDeg(){
  _posDeg = ((int)(encoder->getEncoderValue() * _degConst)) % 360;
}


/*
 * Function: setMotorSpeedPID
 * ----------------------------
 * Function to set and maintain with a PID controller the desired speed of the wheel.
 *    param:
 *    @desiredSpeedRpm: Desired speed of the wheel in rpm.
 */
void Motor::setMotorSpeedPIDRpm(float desiredSpeedRpm){
    
    _Ref = desiredSpeedRpm;                         // Speed reference
    _Ek = _Ref - _speedRpmFilter;                   // Calculate error
    _Ik = _Ek * (_periodMs/1000.0) + _IkOld;        // Calculate integral accion
    if(_Ik > MOTOR_MAX_U)       _Ik = MOTOR_MAX_U;  // Check and limit integral saturation
    else if(_Ik < MOTOR_MIN_U)  _Ik = MOTOR_MIN_U;  
    _Dk = (_Ek - _EkOld) / _periodMs;               // Calculate derivative accion                                         
    _Uk = _Ek * _Kp + _Ik * _Ki + _Dk * _Kd;        // Calculate total accion  
    if(_Uk < MOTOR_MIN_U)       _Uk = MOTOR_MIN_U;  // Check and limit total accion saturation
    else if(_Uk > MOTOR_MAX_U)  _Uk = MOTOR_MAX_U;  
    _IkOld = _Ik; _EkOld = _Ek;                     // Store old value
    

    _pwmValue = (uint8_t) (abs(_Uk)*255/MOTOR_MAX_U);
    
    if(desiredSpeedRpm == 0){  motorRun(RELEASE);  _pwmValue = 0; _IkOld = 0;}
    else if(_Uk > 0)           motorRun(FORWARD);          
    else if(_Uk < 0)           motorRun(BACKWARD);                   

    setMotorSpeedPwm(_pwmValue);
}


/*
 * Function: setPID
 * ----------------------------
 * Function to set PID constants values.
 *    param:
 *    @Kp: proporcional gain.
 *    @Ki: integral gain.
 *    @Kd: derivative gain.
 */
void Motor::setPID(float Kp, float Ki, float Kd){
  _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;
}



/*************  GET FUNTIONS ***********/
/***************************************/
float Motor::getMotorSpeedRpm(){
  return _speedRpm;
}

float Motor::getMotorSpeedRads(){
  return _speedRpm * _rpmToRadsConst;
} 

float Motor::getMotorPosDeg(){
  return _posDeg;
}
