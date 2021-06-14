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
  _radsConst = (float) (2*3.14159 * 1000 / _periodMs) / (encoder->getEncoderResolution());
  _speed = 0; _speed1 = 0; _speed2 = 0; _speed3 = 0;
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
 * Function: calculateMotorSpeed
 * ----------------------------
 * Function to calculate motor speed.
 * The function use the encoder value associated with the motor.
 */
void Motor::calculateMotorSpeed(){
  long currentEncValue = encoder->getEncoderValue();
  long oldEncValue = encoder->getEncoderValueOld();
  _speed = (currentEncValue - oldEncValue) * _radsConst;
  encoder->setEncoderValueOld(currentEncValue);
  _speedFilter = 0.2*_speed + 0.2*_speed1 + 0.3*_speed2 + 0.3*_speed3;
  _speed3 = _speed2; _speed2 = _speed1; _speed1 = _speed;
}


/*
 * Function: setMotorSpeedPID
 * ----------------------------
 * Function to set and maintain with a PID controller the desired speed of the wheel.
 *    param:
 *    @desiredSpeed: Desired speed of the wheel in rad/s.
 */
void Motor::setMotorSpeedPID(float desiredSpeed){
    
    _Ref = desiredSpeed;                            // Speed reference
    _Ek = _Ref - _speedFilter;                      // Calculate error
    _Ik = _Ek * (_periodMs/1000.0) + _IkOld;        // Calculate integral accion
    if(_Ik > MOTOR_MAX_U)       _Ik = MOTOR_MAX_U;  // Check and limit integral saturation
    else if(_Ik < MOTOR_MIN_U)  _Ik = MOTOR_MIN_U;  
    _Dk = (_Ek - _EkOld) / _periodMs;               // Calculate derivative accion                                         
    _Uk = _Ek * _Kp + _Ik * _Ki + _Dk * _Kd;        // Calculate total accion  
    if(_Uk < MOTOR_MIN_U)       _Uk = MOTOR_MIN_U;  // Check and limit total accion saturation
    else if(_Uk > MOTOR_MAX_U)  _Uk = MOTOR_MAX_U;  
    _IkOld = _Ik; _EkOld = _Ek;                     // Store old value
    

    _pwmValue = (uint8_t) (abs(_Uk)*255/MOTOR_MAX_U);
    
    if(desiredSpeed == 0){      motorRun(RELEASE);  _pwmValue = 0; _IkOld = 0;}
    else if(_Uk > 0)            motorRun(FORWARD);          
    else if(_Uk < 0)            motorRun(BACKWARD);                   

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

float Motor::getMotorSpeed(){
  return _speedFilter;
} 
