#include "Robot.h"


/*
   Constructor: Robot
   ----------------------------
   Class constructor.
   This class control a 4 mechanum wheel robot.
   FR: Front Right
   FL: Front Left
   RR: Rear Right
   RL: Rear Left
     param:
      @motorPinFR: Adafruit MotorShield pin connected to FR motor (1,2,3,4)
      @motorPinFL: Adafruit MotorShield pin connected to FL motor (1,2,3,4)
      @motorPinRR: Adafruit MotorShield pin connected to RR motor (1,2,3,4)
      @motorPinRL: Adafruit MotorShield pin connected to RL motor (1,2,3,4)
      @encoderFRChA: pin connected to the encoder of FR motor (ChA)
      @encoderFRChB: pin connected to the encoder of FR motor (ChB)
      @encoderFLChA: pin connected to the encoder of FL motor (ChA)
      @encoderFLChB: pin connected to the encoder of FL motor (ChB)
      @encoderRRChA: pin connected to the encoder of RR motor (ChA)
      @encoderRRChB: pin connected to the encoder of RR motor (ChB)
      @encoderRLChA: pin connected to the encoder of RL motor (ChA)
      @encoderRLChB: pin connected to the encoder of RL motor (ChB)
      @periodMs: cycle period in milliseconds
      @wheelToCenterX: length in mm (X axis) between center of the wheel and center of the robot
      @wheelToCenterY: length in mm(Y axis) between center of the wheel and center of the robot
      @wheelDiameter: wheel diameter in mm
*/
Robot::Robot(uint8_t motorPinFR, uint8_t motorPinFL, uint8_t motorPinRR, uint8_t motorPinRL,
             uint8_t encoderFRChA, uint8_t encoderFRChB, uint8_t encoderFLChA, uint8_t encoderFLChB,
             uint8_t encoderRRChA, uint8_t encoderRRChB, uint8_t encoderRLChA, uint8_t encoderRLChB,
             uint8_t periodMs, float wheelToCenterX, float wheelToCenterY, uint16_t wheelDiameter) {

  AFMS = new Adafruit_MotorShield();
  motorFR = new Motor(AFMS, motorPinFR, encoderFRChA, encoderFRChB, periodMs, wheelDiameter);
  motorFL = new Motor(AFMS, motorPinFL, encoderFLChA, encoderFLChB, periodMs, wheelDiameter);
  motorRR = new Motor(AFMS, motorPinRR, encoderRRChA, encoderRRChB, periodMs, wheelDiameter);
  motorRL = new Motor(AFMS, motorPinRL, encoderRLChA, encoderRLChB, periodMs, wheelDiameter);
  imu = new IMU(periodMs);


  _periodS = (float) periodMs / 1000.0;
  _wheelDiameter_Meter =  (float) wheelDiameter / 1000.0;
  _wheelToCenterX_Meter = (float) wheelToCenterX / 1000.0;
  _wheelToCenterY_Meter = (float) wheelToCenterY / 1000.0;
  _radsToRpmConst = (float) 60.0 / (2 * 3.14);
  _rpmToMsConst = (float) (1 / _radsToRpmConst) * (0.5 * _wheelDiameter_Meter);
  _msToRpmConst = (float) 1 / _rpmToMsConst;
  _posX = 0; _posY = 0; _thetaZ = 0;
  _posXInit = _posX; _posYInit = _posY;
  _cmdModeROS = ROS_CMD_MODE_MAN;
  _cmdManualROS = ROS_CMD_STOP;
  _speedProfileROS = SPEED_PROFILE_S_CURVE;
  _robotMaxSpeedMs = (float) MOTOR_MAX_SPEED_RPM * ((2 * 3.14) / 60.0) * (0.5 * _wheelDiameter_Meter);

  _IkOrientOld = 0;
}


/*
   Function: init
   ----------------------------
   Function to initialize the object of the class.
*/
void Robot::init() {
  AFMS->begin();
  imu->init();
  setAllMotorStop();
}

/*
   Function: setAllMotorStop
   ----------------------------
   Change the motor speed to zero.
*/
void Robot::setAllMotorStop() {
  _speedMotorFR = 0;
  _speedMotorFL = 0;
  _speedMotorRR = 0;
  _speedMotorRL = 0;
}

/*
   Function: runForward
   ----------------------------
   Change the motor speed to go forward.
   The speed is in rpm and its provided via ROS.
*/
void Robot::runForward() {
  _speedMotorFR = _cmdSpeedROSRpm;
  _speedMotorFL = _cmdSpeedROSRpm;
  _speedMotorRR = _cmdSpeedROSRpm;
  _speedMotorRL = _cmdSpeedROSRpm;
}

/*
   Function: runBackward
   ----------------------------
   Change the motor speed to go backward.
   The speed is in rpm and its provided via ROS.
*/
void Robot::runBackward() {
  _speedMotorFR = (-1) * _cmdSpeedROSRpm;
  _speedMotorFL = (-1) * _cmdSpeedROSRpm;
  _speedMotorRR = (-1) * _cmdSpeedROSRpm;
  _speedMotorRL = (-1) * _cmdSpeedROSRpm;
}


/*
   Function: runRight
   ----------------------------
   Change the motor speed to go right.
   The speed is in rpm and its provided via ROS.
*/
void Robot::runRight() {
  _speedMotorFR = (-1) * _cmdSpeedROSRpm;
  _speedMotorFL =  _cmdSpeedROSRpm;
  _speedMotorRR =  _cmdSpeedROSRpm;
  _speedMotorRL = (-1) * _cmdSpeedROSRpm;
}

/*
   Function: runLeft
   ----------------------------
   Change the motor speed to go left.
   The speed is in rpm and its provided via ROS.
*/
void Robot::runLeft() {
  _speedMotorFR = _cmdSpeedROSRpm;
  _speedMotorFL = (-1) * _cmdSpeedROSRpm;
  _speedMotorRR = (-1) * _cmdSpeedROSRpm;
  _speedMotorRL = _cmdSpeedROSRpm;
}

/*
   Function: runForwardRight
   ----------------------------
   Change the motor speed to go diagonal forward-right.
   The speed is in rpm and its provided via ROS.
*/
void Robot::runForwardRight() {
  _speedMotorFR = 0;
  _speedMotorFL = _cmdSpeedROSRpm;
  _speedMotorRR = _cmdSpeedROSRpm;
  _speedMotorRL = 0;
}

/*
   Function: runForwardLeft
   ----------------------------
   Change the motor speed to go diagonal forward-left.
   The speed is in rpm and its provided via ROS.
*/
void Robot::runForwardLeft() {
  _speedMotorFR = _cmdSpeedROSRpm;
  _speedMotorFL = 0;
  _speedMotorRR = 0;
  _speedMotorRL = _cmdSpeedROSRpm;
}

/*
   Function: runBackwardRight
   ----------------------------
   Change the motor speed to go diagonal backward-right.
   The speed is in rpm and its provided via ROS.
*/
void Robot::runBackwardRight() {
  _speedMotorFR = (-1) * _cmdSpeedROSRpm;
  _speedMotorFL = 0;
  _speedMotorRR = 0;
  _speedMotorRL = (-1) * _cmdSpeedROSRpm;
}

/*
   Function: runBackwardLeft
   ----------------------------
   Change the motor speed to go diagonal backward-left.
   The speed is in rpm and its provided via ROS.
*/
void Robot::runBackwardLeft() {
  _speedMotorFR = 0;
  _speedMotorFL = (-1) * _cmdSpeedROSRpm;
  _speedMotorRR = (-1) * _cmdSpeedROSRpm;
  _speedMotorRL = 0;
}

/*
   Function: runRotateCW
   ----------------------------
   Change the motor speed to rotate clockwise.
   The speed is in rpm and its provided via ROS.
*/
void Robot::runRotateCW() {
  _speedMotorFR = (-1) * _cmdSpeedROSRpm;
  _speedMotorFL = _cmdSpeedROSRpm;
  _speedMotorRR = (-1) * _cmdSpeedROSRpm;
  _speedMotorRL = _cmdSpeedROSRpm;
}

/*
   Function: runRotateCCW
   ----------------------------
   Change the motor speed to rotate counter clockwise.
   The speed is in rpm and its provided via ROS.
*/
void Robot::runRotateCCW() {
  _speedMotorFR = _cmdSpeedROSRpm;
  _speedMotorFL = (-1) * _cmdSpeedROSRpm;
  _speedMotorRR = _cmdSpeedROSRpm;
  _speedMotorRL = (-1) * _cmdSpeedROSRpm;
}


/*
   Function: ROSControl
   ----------------------------
   This function switch between two possible control of the robot:
    1. COMMAND MODE: The robot receive commands (uint8_t) via ROS and moves in manual mode
    2. POSITION MODE: The robot receive position (pack of uint8_t) via ROS and moves in automatic mode
*/
void Robot::ROSControl() {
  switch (_cmdModeROS) {
    case ROS_CMD_MODE_MAN:  commandModeManual();     break;
    case ROS_CMD_MODE_AUT:  commandModeAutomatic();  break;
    default: break;
  }
}

/*
   Function: commandMode
   ----------------------------
   This function process the received ROS command.
*/
void Robot::commandModeManual() {
  switch (_cmdManualROS) {
    case ROS_CMD_STOP:            setAllMotorStop();        break;
    case ROS_CMD_FWD:             runForward();             break;
    case ROS_CMD_BCK:             runBackward();            break;
    case ROS_CMD_RIGHT:           runRight();               break;
    case ROS_CMD_LEFT:            runLeft();                break;
    case ROS_CMD_FWD_RIGHT:       runForwardRight();        break;
    case ROS_CMD_BCK_RIGHT:       runBackwardRight();       break;
    case ROS_CMD_BCK_LEFT:        runBackwardLeft();        break;
    case ROS_CMD_FWD_LEFT:        runForwardLeft();         break;
    case ROS_CMD_CW:              runRotateCW();            break;
    case ROS_CMD_CCW:             runRotateCCW();           break;
    case ROS_CMD_IMU_RST:         imu->calculateOffset();   break;
    case ROS_CMD_ODO_RST:         setOdometryPosition(0, 0, 0); break;
    case ROS_CMD_SPEED_PROF_SQ:   _speedProfileROS = SPEED_PROFILE_SQUARE;          break;
    case ROS_CMD_SPEED_PROF_TR:   _speedProfileROS = SPEED_PROFILE_TRAPEZOIDAL;     break;
    case ROS_CMD_SPEED_PROF_SC:   _speedProfileROS = SPEED_PROFILE_S_CURVE;         break;
    case ROS_CMD_IMU_CORR_ON:     _imuCorrection = 1;       break;
    case ROS_CMD_IMU_CORR_OFF:    _imuCorrection = 0;       break;
    case ROS_CMD_LIDAR_CORR_ON:   _lidarCorrection = 1;     break;
    case ROS_CMD_LIDAR_CORR_OFF:  _lidarCorrection = 0;     break;
    default:  break;
  }
}

/*
   Function: positionMode
   ----------------------------
   This function process the received ROS position.
   The function provide different speed profiles:
    0. Square profile
    1. Trapezoidal profile
    2. S-Curve profile
   The Inverse Kinematic equations provide the wheel speed
*/
void Robot::commandModeAutomatic() {

  float maxSpeed =  _cmdSpeedROSMs;
  float targetZone = 0.01;


  /* Calculate _desiredSpeedX and _desiredSpeedY depending by selected SPEED PROFILE */
  switch (_speedProfileROS) {
    case SPEED_PROFILE_SQUARE:
      squareSpeedProfile(maxSpeed, targetZone);
      break;
    case SPEED_PROFILE_TRAPEZOIDAL:
      trapezoidalSpeedProfile(maxSpeed, targetZone);
      break;
    case SPEED_PROFILE_S_CURVE:
      sCurveSpeedProfile(_pointTargetSpeed, maxSpeed, targetZone);
      break;
    default:
      trapezoidalSpeedProfile(maxSpeed, targetZone);
  }


  _vXToTarget = _desiredSpeedX * cos(_alphaToTarget);
  _vYToTarget = _desiredSpeedY * sin(_alphaToTarget);
  _wZToTarget = 0;  //Fixed orientation

  /* Orientation PID */
  if (_imuCorrection) {
    float maxUk = 1;
    float minUk = -1;
    float error = _wZToTarget - imu->getOrientationX();

    float Ik = error * _periodS + _IkOrientOld;   // Calculate integral accion

    if (Ik > maxUk)       Ik = maxUk;             // Check and limit integral saturation
    else if (Ik < minUk)  Ik = minUk;
    float Uk = error * 0.05 + Ik * 0.002;         // Calculate total accion
    if (Uk < minUk)         Uk = minUk;           // Check and limit total accion saturation
    else if (Uk > maxUk)    Uk = maxUk;
    if (abs(error) < 0.05)  Uk = 0;
    _IkOrientOld = Ik;

    _wZToTarget = Uk;
  }

  /* Check Limits */
  if (_vXToTarget > _robotMaxSpeedMs)                  _vXToTarget = _robotMaxSpeedMs;
  else if (_vXToTarget < (-1)*_robotMaxSpeedMs)        _vXToTarget = (-1) * _robotMaxSpeedMs;
  if (_vYToTarget > _robotMaxSpeedMs)                  _vYToTarget = _robotMaxSpeedMs;
  else if (_vYToTarget < (-1)*_robotMaxSpeedMs)        _vYToTarget = (-1) * _robotMaxSpeedMs;

  /* Inverse Kinematic Ecuations */
  float w1 = (_vYToTarget - _vXToTarget + _wZToTarget * (_wheelToCenterX_Meter + _wheelToCenterY_Meter)) / (0.5 * _wheelDiameter_Meter);
  float w2 = (_vYToTarget + _vXToTarget - _wZToTarget * (_wheelToCenterX_Meter + _wheelToCenterY_Meter)) / (0.5 * _wheelDiameter_Meter);
  float w3 = (_vYToTarget + _vXToTarget + _wZToTarget * (_wheelToCenterX_Meter + _wheelToCenterY_Meter)) / (0.5 * _wheelDiameter_Meter);
  float w4 = (_vYToTarget - _vXToTarget - _wZToTarget * (_wheelToCenterX_Meter + _wheelToCenterY_Meter)) / (0.5 * _wheelDiameter_Meter);
  _speedMotorFR = w1 * _radsToRpmConst;
  _speedMotorFL = w2 * _radsToRpmConst;
  _speedMotorRR = w3 * _radsToRpmConst;
  _speedMotorRL = w4 * _radsToRpmConst;
}



/*
   Function: updateSpeed
   ----------------------------
   Control the wheel speed with a PID control.
*/
void Robot::updateSpeed() {
  motorFR->calculateMotorSpeedRpm();
  motorFR->setMotorSpeedPIDRpm(_speedMotorFR);
  motorFL->calculateMotorSpeedRpm();
  motorFL->setMotorSpeedPIDRpm(_speedMotorFL);
  motorRL->calculateMotorSpeedRpm();
  motorRL->setMotorSpeedPIDRpm(_speedMotorRL);
  motorRR->calculateMotorSpeedRpm();
  motorRR->setMotorSpeedPIDRpm(_speedMotorRR);

  calculateSpeedMs();
}


/*
   Function: updatePos
   ----------------------------
   Calculate position of the robot.
*/
void Robot::updatePos() {
  calculatePos();
  imu->calculateOrientation();
}

/*
   Function: calculateSpeedMs
   ----------------------------
   Calculate the robot speed in m/s.
*/
void Robot::calculateSpeedMs() {
  float w1 = motorFR->getMotorSpeedRads();
  float w2 = motorFL->getMotorSpeedRads();
  float w3 = motorRR->getMotorSpeedRads();
  float w4 = motorRL->getMotorSpeedRads();

  _vX = (-w1 + w2 + w3 - w4) * _wheelDiameter_Meter / 8.0;
  _vY = (+w1 + w2 + w3 + w4) * _wheelDiameter_Meter / 8.0;
  _wZ = (+w1 - w2 + w3 - w4) * _wheelDiameter_Meter / (8.0 * (_wheelToCenterX_Meter + _wheelToCenterY_Meter));
}

/*
   Function: calculatePos
   ----------------------------
   Calculate robot position [m] and orientation [rad].
*/
void Robot::calculatePos() {
  _posX = _posX + _vX * _periodS;
  _posY = _posY + _vY * _periodS;
  _thetaZ = _thetaZ + _wZ * _periodS;
}


/*
   Function: setOdometryPosition
   ----------------------------
   This function set the robot position.
*/
bool Robot::setOdometryPosition(float x, float y, float theta) {
  if (_cmdManualROS == ROS_CMD_ODO_RST) {
    _posX = x; _posXInit = x; _posXTarget = x;
    _posY = y; _posYInit = y; _posYTarget = y;
    _thetaZ = theta;  
    return true;
  }
  else
    return false;
}


void Robot::sCurveSpeedProfile(float endSpeed, float maxSpeed, float targetZone) {
  float positioningSpeed = 0.03;
  float tSpeed = maxSpeed - positioningSpeed;
  float a = 30;
  float a1 = -a;

  /* Define acceleration and deceleration zone */
  _trayDataX.accDecZone = abs(_posXTarget - _posXInit) / 8.0;
  _trayDataY.accDecZone = abs(_posYTarget - _posYInit) / 8.0;
  _trayDataX.accDecZone1 = abs(_posXTarget - _posXInit) - _trayDataX.accDecZone;
  _trayDataY.accDecZone1 = abs(_posYTarget - _posYInit) - _trayDataY.accDecZone;


  /* Calculate distance to target and from strarting point */
  _trayDataX.distToTarget = abs(_posXTarget - _posX);
  _trayDataY.distToTarget = abs(_posYTarget - _posY);
  _trayDataX.distToInit = abs(_posX - _posXInit);
  _trayDataY.distToInit = abs(_posY - _posYInit);

  /* Calculate trayectoy for X-Y */
  trayData* trayDataArray[] = {&_trayDataX, &_trayDataY};
  for (uint8_t i = 0; i < 2; i++) {
    trayData* trayPtr = trayDataArray[i];
    float dInit = trayPtr->distToInit;
    float dTarget = trayPtr->distToTarget;
    float c = trayPtr->accDecZone;
    float c1 = trayPtr->accDecZone1;
    float ySpeed = 0;
    
    if (dInit <= 2 * c)                           ySpeed = 1 / (1 + exp(-a * (dInit - c)));
    else if (dInit > 2 * c && dTarget > 2 * c)    ySpeed = 1;
    else if (dTarget <= 2 * c )                   ySpeed = 1 / (1 + exp(-a1 * (dInit - c1)));
    trayPtr->desiredSpeed = ySpeed * tSpeed + positioningSpeed;

    if (trayPtr->desiredSpeed >  maxSpeed)        trayPtr->desiredSpeed = maxSpeed;
    if (dTarget <=  targetZone) {
      if (endSpeed == 0)  trayPtr->desiredSpeed = 0;
      else                trayPtr->desiredSpeed = positioningSpeed;
    }
  }
  _desiredSpeedX = _trayDataX.desiredSpeed;
  _desiredSpeedY = _trayDataY.desiredSpeed;
}

void Robot::trapezoidalSpeedProfile(float targetSpeed, float targetZone) {

  float positioningSpeed = 0.2 * targetSpeed;

  /* Define acceleration and deceleration zone */
  _trayDataX.accDecZone = 0.25 * abs(_posXTarget - _posXInit);
  _trayDataY.accDecZone = 0.25 * abs(_posYTarget - _posYInit);

  /* Calculate distance to target and from strarting point */
  _trayDataX.distToTarget = abs(_posXTarget - _posX);
  _trayDataY.distToTarget = abs(_posYTarget - _posY);
  _trayDataX.distToInit = abs(_posX - _posXInit);
  _trayDataY.distToInit = abs(_posY - _posYInit);

  /* Calculate trayectoy for X-Y */
  trayData* trayDataArray[] = {&_trayDataX, &_trayDataY};
  for (uint8_t i = 0; i < 2; i++) {
    trayData* trayPtr = trayDataArray[i];
    float dInit = trayPtr->distToInit;
    float dTarget = trayPtr->distToTarget;
    float accDecZone = trayPtr->accDecZone;
    if (dInit < accDecZone) {
      trayPtr->desiredSpeed = positioningSpeed + targetSpeed * (dInit / accDecZone);
    }
    else if (dInit >= accDecZone && dTarget >= accDecZone) {
      trayPtr->desiredSpeed = targetSpeed;
    }
    if (dTarget < accDecZone) {
      trayPtr->desiredSpeed = positioningSpeed + targetSpeed * (dTarget / accDecZone);
    }
    if (trayPtr->desiredSpeed >  targetSpeed)   trayPtr->desiredSpeed = targetSpeed;
    if (dTarget <  targetZone)                  trayPtr->desiredSpeed = 0;
  }
  _desiredSpeedX = _trayDataX.desiredSpeed;
  _desiredSpeedY = _trayDataY.desiredSpeed;

}

void Robot::squareSpeedProfile(float targetSpeed, float targetZone) {
  float nearTargetZone = 10 * targetZone;
  float positioningSpeed = 0.2 * targetSpeed;

  /* Calcultae distance and speed to target */
  _trayDataX.distToTarget = abs(_posXTarget - _posX);
  _trayDataY.distToTarget = abs(_posYTarget - _posY);

  trayData* trayDataArray[] = {&_trayDataX, &_trayDataY};
  for (uint8_t i = 0; i < 2; i++) {
    trayData* trayPtr = trayDataArray[i];
    if (trayPtr->distToTarget < targetZone)               trayPtr->desiredSpeed = 0;
    else if (trayPtr->distToTarget < nearTargetZone)      trayPtr->desiredSpeed = positioningSpeed;
    else                                                  trayPtr->desiredSpeed = targetSpeed;
  }
  _desiredSpeedX = _trayDataX.desiredSpeed;
  _desiredSpeedY = _trayDataY.desiredSpeed;

}


void Robot::setCmdModeFromROS(uint8_t data) {
  _cmdModeROS = data;
}

void Robot::setCmdManualFromROS(uint8_t data) {
  _cmdManualROS = data;
}

void Robot::setCmdSpeedFromROS(float data) {
  _cmdSpeedROSMs = data;
  _cmdSpeedROSRpm = _cmdSpeedROSMs * _msToRpmConst;
}

void Robot::setCmdTrayFromROS(float x, float y, float pointTargetSpeed) {
  _posXTarget = x;
  _posYTarget = y;
  _pointTargetSpeedOld = _pointTargetSpeed;
  _pointTargetSpeed = pointTargetSpeed;
  _posXInit = _posX; _posYInit = _posY;
  _alphaToTarget = atan2(_posYTarget - _posY, _posXTarget - _posX);
}

void Robot::setLidarPosFromROS(float x, float y) {
  _lidarPosX = x;
  _lidarPosY = y;
}


/*
   Function: lidarOdometryCorrection
   ----------------------------
   This function replace robot position with lidar absolute position.
   Note: The lidar coordinates is not the same as robot one.
         RobotX = lidarY
         RobotY = - lidarX
*/
bool Robot::lidarOdometryCorrection() {
  if (_lidarCorrection && _cmdModeROS == ROS_CMD_MODE_AUT) {
    _posX = _lidarPosY;
    _posY = (-1)*_lidarPosX;
    return true;
  }
  else  return false;
}

void Robot::getDataSendToROS(geometry_msgs::Point32* pose, geometry_msgs::Point32* s, geometry_msgs::Point32* im) {

  // Robot position
  pose->x = _posX;
  pose->y = _posY;
  pose->z = _thetaZ ;

  // Robot speed
  s->x = _vX;
  s->y = _vY;
  s->z = _wZ;

  //Robot IMU
  im->x = imu->getOrientationX();
  im->y = 0;
  im->z = 0;
}


void Robot::serialDebug() {
  Serial.print("MODE  "); Serial.print(_cmdModeROS); Serial.print(" CMD MAN  "); Serial.print(_cmdManualROS);
  Serial.println("");
  Serial.print("SPEED RPM  "); Serial.print(_cmdSpeedROSRpm); Serial.print(" SPEED M/S  "); Serial.print(_cmdSpeedROSMs); Serial.print(" Profile  "); Serial.print(_speedProfileROS);
  Serial.println("");
  Serial.print("WHEEL SPEED RPM  FR"); Serial.print(_speedMotorFR); Serial.print(" FL "); Serial.print(_speedMotorFL); Serial.print(" RL "); Serial.print(_speedMotorRL); Serial.print(" RR "); Serial.print(_speedMotorRR);
  Serial.println("");
  Serial.print("POS TARGET "); Serial.print(_posXTarget); Serial.print(" "); Serial.print(_posYTarget); Serial.print(" start speed "); Serial.print(_pointTargetSpeedOld); Serial.print(" end speed "); Serial.print(_pointTargetSpeedOld);
  Serial.println("");
  Serial.print("POS REAL "); Serial.print(_posX); Serial.print(" "); Serial.print(_posY); Serial.print(" ");
  Serial.println("");
  if(_lidarCorrection)  Serial.println(" Lidar correction active ");
  if(_imuCorrection)    Serial.println(" IMU correction active ");
}
