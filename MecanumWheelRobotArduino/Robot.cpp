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
  _periodMs = periodMs;
  _periodS = (float) periodMs / 1000.0;
  _wheelDiameter_Meter =  (float) wheelDiameter / 1000.0;
  _wheelToCenterX_Meter = (float) wheelToCenterX / 1000.0;
  _wheelToCenterY_Meter = (float) wheelToCenterY / 1000.0;
  _cmdModeROS = ROS_CMD_MODE_MAN;
  _cmdManualROS = ROS_CMD_STOP;
  _speedProfileROS = SPEED_PROFILE_S_CURVE;
  _robotMaxSpeedMs = (float) MOTOR_MAX_SPEED_RPM * ((2 * 3.14159) / 60.0) * (0.5 * _wheelDiameter_Meter);
  
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

 /* Implement the Kalman filter corresponding to the linear problem
 *    x_k = F*x_{k-1} + B*u_k + q_k   (evolution model)
 *    y_k = H*x_k + r_k               (measure)
 *
 * with the matrices and vectors
 *    x [output] [size=Nstate]          Estimated state vector
 *    F [input]  [size=(Nstate,Nstate)] Free evolution of the state vector
 *    B [input]  [size=(Nstate,Ncom)]   [optional] Command vector acting on state
 *    Q [input]  [size=(Nstate,Nstate)] Model covariance acting as (1/inertia)
 *    y [input]  [size=Nobs]            Observed (measured) data from sensors
 *    H [input]  [size=(Nobs,Nstate)]   Observation matrix
 *    R [input]  [size=(Nobs,Nobs)]     Measurement noise covariance matrix
 *
 * Requires:
 *  BasicLinearAlgebra  https://github.com/tomstewart89/BasicLinearAlgebra
 */
  float dT = _periodS;
  float r4 = _wheelDiameter_Meter / 8.0;  // r/4
  float r4l = r4 / (_wheelToCenterX_Meter + _wheelToCenterY_Meter);  // r/(4*(lx+ly))

  // Model evolution matrix
  K.F = {1.0, 0.0, 0.0,  dT, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0,  dT, 0.0,
         0.0, 0.0, 1.0, 0.0, 0.0,  dT,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // Model control matrix
  K.B = {0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0,
         -r4,  r4,  r4, -r4,
          r4,  r4,  r4,  r4,
         r4l,-r4l, r4l,-r4l};

  // Model covariance matrix
  K.Q = {m1*m1,   0.0,   0.0,   0.0,   0.0,   0.0,
           0.0, m2*m2,   0.0,   0.0,   0.0,   0.0,
           0.0,   0.0, m3*m3,   0.0,   0.0,   0.0,
           0.0,   0.0,   0.0, m4*m4,   0.0,   0.0,
           0.0,   0.0,   0.0,   0.0, m5*m5,   0.0,
           0.0,   0.0,   0.0,   0.0,   0.0, m6*m6};
         
  // Measurament matrix
  K.H = {0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
         1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0, 0.0, 0.0};

  // Measurement covariance matrix
  K.R = {n1*n1,   0.0,   0.0,   0.0,
           0.0, n2*n2,   0.0,   0.0,
           0.0,   0.0, n3*n3,   0.0,
           0.0,   0.0,   0.0, n4*n4};

  initKalmanFilter();
}


/*
   Function: ROSControl
   ----------------------------
   This function switch between two possible control of the robot:
    1. COMMAND MODE:  The robot receive commands (uint8_t) via ROS and moves in manual mode
    2. POSITION MODE: The robot receive a (x,y,z) = (x,y=position, z = target speed) via ROS and moves in automatic mode
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
    case ROS_CMD_ODO_RST:         resetPosition();          break;
    case ROS_CMD_SPEED_PROF_SQ:   _speedProfileROS = SPEED_PROFILE_SQUARE;          break;
    case ROS_CMD_SPEED_PROF_TR:   _speedProfileROS = SPEED_PROFILE_TRAPEZOIDAL;     break;
    case ROS_CMD_SPEED_PROF_SC:   _speedProfileROS = SPEED_PROFILE_S_CURVE;         break;
    case ROS_CMD_FIXED_ORIENTATION_ON:   _fixedOrientation = 1;       break;
    case ROS_CMD_FIXED_ORIENTATION_OFF:  _fixedOrientation = 0;       break;
    case ROS_CMD_KALMAN_ON:   _useKalmanFilter = 1;     break;
    case ROS_CMD_KALMAN_OFF:  _useKalmanFilter = 0;     break;
    default:  break;
  }
}

/*
   Function: commandModeAutomatic
   ----------------------------
   This function process the received ROS position.
   The function provide different speed profiles:
    0. Square profile
    1. Trapezoidal profile
    2. S-Curve profile
   The Inverse Kinematic equations calculate the wheel speed
*/
void Robot::commandModeAutomatic() {

  _trayMaxSpeed = _cmdRobotSpeedROS;
  _trayNearTarget = 0.01;
  _positioningSpeed = 0.035;

  /* If Kalman filter is active use the Kalman position as robot position. Use odometry by default */
  if(_useKalmanFilter){
    _robotPosX = _kalmanPosX;
    _robotPosY = _kalmanPosY;
    _robotThetaZ = _kalmanThetaZ;
  }
  else{
    _robotPosX = _odomPosX;
    _robotPosY = _odomPosY;
    _robotThetaZ = imu->getOrientationX();
  }

  /* Calculate _desiredSpeedX and _desiredSpeedY depending by selected SPEED PROFILE */
  switch (_speedProfileROS) {
    case SPEED_PROFILE_SQUARE:
      squareSpeedProfile(_pointTargetSpeed, _trayMaxSpeed, _trayNearTarget);
      break;
    case SPEED_PROFILE_TRAPEZOIDAL:
      trapezoidalSpeedProfile(_pointTargetSpeed, _trayMaxSpeed, _trayNearTarget);
      break;
    case SPEED_PROFILE_S_CURVE:
      sCurveSpeedProfile(_pointTargetSpeed, _trayMaxSpeed, _trayNearTarget);
      break;
    default:
      sCurveSpeedProfile(_pointTargetSpeed, _trayMaxSpeed, _trayNearTarget);
  }

  /* Calculate the angle from actual and target position. 
     The transformation between gloabal and robot system reference is not considered */
  _alphaToTarget = atan2(_robotPosYTarget - _robotPosY, _robotPosXTarget - _robotPosX);
  _wZToTarget = 0;  //Fixed orientation
  _vXToTarget = _desiredSpeedX * cos(_alphaToTarget);
  _vYToTarget = _desiredSpeedY * sin(_alphaToTarget);

  /* Orientation PID if active*/
  if (_fixedOrientation) {
    float maxUk = 1.5;
    float minUk = -1.5;
    float error = _wZToTarget - _robotThetaZ;

    float Ik = error * _periodS + _IkOrientOld;   // Calculate integral action

    if (Ik > maxUk)       Ik = maxUk;             // Check and limit integral saturation
    else if (Ik < minUk)  Ik = minUk;
    float Uk = error * 0.5 + Ik * 0.02;         // Calculate total action
    if (Uk < minUk)         Uk = minUk;           // Check and limit total action saturation
    else if (Uk > maxUk)    Uk = maxUk;
    if (abs(error) < 0.005)  Uk = 0;
    _IkOrientOld = Ik;

    _wZToTarget = Uk;
  }

  /* Check Limits */
  if (_vXToTarget > _robotMaxSpeedMs)                  _vXToTarget = _robotMaxSpeedMs;
  else if (_vXToTarget < (-1)*_robotMaxSpeedMs)        _vXToTarget = (-1) * _robotMaxSpeedMs;
  if (_vYToTarget > _robotMaxSpeedMs)                  _vYToTarget = _robotMaxSpeedMs;
  else if (_vYToTarget < (-1)*_robotMaxSpeedMs)        _vYToTarget = (-1) * _robotMaxSpeedMs;

  /* Inverse Kinematic Ecuations */
  _speedMotorFR = (_vYToTarget - _vXToTarget + _wZToTarget * (_wheelToCenterX_Meter + _wheelToCenterY_Meter)) / (0.5 * _wheelDiameter_Meter);
  _speedMotorFL = (_vYToTarget + _vXToTarget - _wZToTarget * (_wheelToCenterX_Meter + _wheelToCenterY_Meter)) / (0.5 * _wheelDiameter_Meter);
  _speedMotorRR = (_vYToTarget + _vXToTarget + _wZToTarget * (_wheelToCenterX_Meter + _wheelToCenterY_Meter)) / (0.5 * _wheelDiameter_Meter);
  _speedMotorRL = (_vYToTarget - _vXToTarget - _wZToTarget * (_wheelToCenterX_Meter + _wheelToCenterY_Meter)) / (0.5 * _wheelDiameter_Meter);

}



/*
   Function: updateSpeed
   ----------------------------
   Control the wheel speed with a PID control.
*/
void Robot::updateSpeed() {
  motorFR->calculateMotorSpeed();
  motorFR->setMotorSpeedPID(_speedMotorFR);
  motorFL->calculateMotorSpeed();
  motorFL->setMotorSpeedPID(_speedMotorFL);
  motorRL->calculateMotorSpeed();
  motorRL->setMotorSpeedPID(_speedMotorRL);
  motorRR->calculateMotorSpeed();
  motorRR->setMotorSpeedPID(_speedMotorRR);

  calculateSpeedMs();
}


/*
   Function: updatePos
   ----------------------------
   Calculate odometry position of the robot.
   Orientation angle between [-pi, pi] rad.
*/
void Robot::updatePos() {
  _odomPosX = _odomPosX + _vX * _periodS;
  _odomPosY = _odomPosY + _vY * _periodS;
  _odomThetaZ = _odomThetaZ + _wZ * _periodS;
  if(abs(_odomThetaZ) >= 3.14159)  _odomThetaZ = (-1) * _odomThetaZ;
  imu->calculateOrientation();
}

/*
   Function: calculateSpeedMs
   ----------------------------
   Calculate the robot speed in m/s.
*/
void Robot::calculateSpeedMs() {

  /* Get wheel speed [rad/s] */
  _w1 = motorFR->getMotorSpeed();
  _w2 = motorFL->getMotorSpeed();
  _w3 = motorRR->getMotorSpeed();
  _w4 = motorRL->getMotorSpeed();

  /* Direct Kinematic */
  _vX = (-_w1 + _w2 + _w3 - _w4) * _wheelDiameter_Meter / 8.0;
  _vY = (+_w1 + _w2 + _w3 + _w4) * _wheelDiameter_Meter / 8.0;
  _wZ = (+_w1 - _w2 + _w3 - _w4) * _wheelDiameter_Meter / (8.0 * (_wheelToCenterX_Meter + _wheelToCenterY_Meter));
}


/*
   Function: isPositionResetted
   ----------------------------
   Return if a reset position command has been received.
*/
bool Robot::isPositionResetted() {
  if (_cmdManualROS == ROS_CMD_ODO_RST)   return true;
  else                                    return false;
}


/*
   Function: resetPosition
   ----------------------------
   Reset position and orientation to (x,y)=(0,0). Note that Lidar position canìt be resetted.
*/
void Robot::resetPosition(){
  _odomPosX = 0; _robotPosX = 0; _robotPosXInit = 0; _robotPosXTarget = 0;
  _odomPosY = 0; _robotPosY = 0; _robotPosYInit = 0; _robotPosYTarget = 0;
  _odomThetaZ = 0; _robotThetaZ = 0;
  imu->calculateOffset();
  initKalmanFilter();
}


/*
   Function: sCurveSpeedProfile
   ----------------------------
   S-Curve speed curve.
   param:
    @endSpeed [m/s]: if zero the robot stop when the target is reached. If not zero the speed will be set to positioningSpeed
    @trayMaxSpeed [m/s]: max speed during the trajectory.
    @targetZone [m]: use to stop or reduce speed of the robot when is near the target.
*/
void Robot::sCurveSpeedProfile(float endSpeed, float trayMaxSpeed, float targetZone) {
  float tSpeed = trayMaxSpeed - _positioningSpeed;
  float a = 30;
  float a1 = -a;

  /* Define acceleration and deceleration zone */
  _trayDataX.accDecZone = abs(_robotPosXTarget - _robotPosXInit) / 8.0;
  _trayDataY.accDecZone = abs(_robotPosYTarget - _robotPosYInit) / 8.0;
  _trayDataX.accDecZone1 = abs(_robotPosXTarget - _robotPosXInit) - _trayDataX.accDecZone;
  _trayDataY.accDecZone1 = abs(_robotPosYTarget - _robotPosYInit) - _trayDataY.accDecZone;


  /* Calculate distance to target and from strarting point */
  _trayDataX.distToTarget = abs(_robotPosXTarget - _robotPosX);
  _trayDataY.distToTarget = abs(_robotPosYTarget - _robotPosY);
  _trayDataX.distToInit = abs(_robotPosX - _robotPosXInit);
  _trayDataY.distToInit = abs(_robotPosY - _robotPosYInit);

  /* Calculate trayectoy for X-Y */
  trayData* trayDataArray[] = {&_trayDataX, &_trayDataY};
  for (uint8_t i = 0; i < 2; i++) {
    trayData* trayPtr = trayDataArray[i];
    float dInit = trayPtr->distToInit;
    float dTarget = trayPtr->distToTarget;
    float c = trayPtr->accDecZone;
    float c1 = trayPtr->accDecZone1;
    float ySpeed = 0;
    
    if (dInit <= 2*c)                         ySpeed = 1 / (1 + exp(-a * (dInit - c)));
    else if (dInit > 2*c && dTarget > 2*c)    ySpeed = 1;
    else if (dTarget <= 2*c )                 ySpeed = 1 / (1 + exp(-a1 * (dInit - c1)));
    trayPtr->desiredSpeed = ySpeed * tSpeed + _positioningSpeed;

    if (trayPtr->desiredSpeed >  trayMaxSpeed)        trayPtr->desiredSpeed = trayMaxSpeed;
    if (dTarget <=  targetZone) {
      if (endSpeed == 0)  trayPtr->desiredSpeed = 0;
      else                trayPtr->desiredSpeed = _positioningSpeed;
    }
  }
  _desiredSpeedX = _trayDataX.desiredSpeed;
  _desiredSpeedY = _trayDataY.desiredSpeed;
}


/*
   Function: trapezoidalSpeedProfile
   ----------------------------
   Trapezoidal speed curve.
   param:
    @endSpeed [m/s]: if zero the robot stop when the target is reached. If not zero the speed will be set to positioningSpeed
    @trayMaxSpeed [m/s]: max speed during the trajectory.
    @targetZone [m]: use to stop or reduce speed of the robot when is near the target.
*/
void Robot::trapezoidalSpeedProfile(float endSpeed, float trayMaxSpeed, float targetZone) {

  /* Define acceleration and deceleration zone */
  _trayDataX.accDecZone = 0.25 * abs(_robotPosXTarget - _robotPosXInit);
  _trayDataY.accDecZone = 0.25 * abs(_robotPosYTarget - _robotPosYInit);

  /* Calculate distance to target and from strarting point */
  _trayDataX.distToTarget = abs(_robotPosXTarget - _robotPosX);
  _trayDataY.distToTarget = abs(_robotPosYTarget - _robotPosY);
  _trayDataX.distToInit = abs(_robotPosX - _robotPosXInit);
  _trayDataY.distToInit = abs(_robotPosY - _robotPosYInit);

  /* Calculate trayectoy for X-Y */
  trayData* trayDataArray[] = {&_trayDataX, &_trayDataY};
  for (uint8_t i = 0; i < 2; i++) {
    trayData* trayPtr = trayDataArray[i];
    float dInit = trayPtr->distToInit;
    float dTarget = trayPtr->distToTarget;
    float accDecZone = trayPtr->accDecZone;
    if (dInit < accDecZone) {
      trayPtr->desiredSpeed = _positioningSpeed + trayMaxSpeed * (dInit / accDecZone);
    }
    else if (dInit >= accDecZone && dTarget >= accDecZone) {
      trayPtr->desiredSpeed = trayMaxSpeed;
    }
    if (dTarget < accDecZone) {
      trayPtr->desiredSpeed = _positioningSpeed + trayMaxSpeed * (dTarget / accDecZone);
    }
    if (trayPtr->desiredSpeed >  trayMaxSpeed)   trayPtr->desiredSpeed = trayMaxSpeed;
    if (dTarget <=  targetZone) {
      if (endSpeed == 0)  trayPtr->desiredSpeed = 0;
      else                trayPtr->desiredSpeed = _positioningSpeed;
    }
  }
  _desiredSpeedX = _trayDataX.desiredSpeed;
  _desiredSpeedY = _trayDataY.desiredSpeed;

}


/*
   Function: squareSpeedProfile
   ----------------------------
   Changes in speed without ramp.
   param:
    @endSpeed [m/s]: if zero the robot stop when the target is reached. If not zero the speed will be set to positioningSpeed
    @trayMaxSpeed [m/s]: max speed during the trajectory.
    @targetZone [m]: use to stop or reduce speed of the robot when is near the target.
*/
void Robot::squareSpeedProfile(float endSpeed, float trayMaxSpeed, float targetZone) {
  float nearTargetZone = 10 * targetZone;

  /* Calcultae distance and speed to target */
  _trayDataX.distToTarget = abs(_robotPosXTarget - _robotPosX);
  _trayDataY.distToTarget = abs(_robotPosYTarget - _robotPosY);

  trayData* trayDataArray[] = {&_trayDataX, &_trayDataY};
  for (uint8_t i = 0; i < 2; i++) {
    trayData* trayPtr = trayDataArray[i];
    if (trayPtr->distToTarget  <=  targetZone) {
      if (endSpeed == 0)  trayPtr->desiredSpeed = 0;
      else                trayPtr->desiredSpeed = _positioningSpeed;
    }
    else if (trayPtr->distToTarget < nearTargetZone)      trayPtr->desiredSpeed = _positioningSpeed;
    else                                                  trayPtr->desiredSpeed = trayMaxSpeed;
  }
  _desiredSpeedX = _trayDataX.desiredSpeed;
  _desiredSpeedY = _trayDataY.desiredSpeed;

}


/*
   Function: setCmdModeFromROS
   ----------------------------
   This function read the command mode ( 0:Manual 1:Automatic) receved via ROS topic.
*/
void Robot::setCmdModeFromROS(uint8_t data) {
  _cmdModeROS = data;
}

/*
   Function: setCmdManualFromROS
   ----------------------------
   This function read the manual command receved via ROS topic.
*/
void Robot::setCmdManualFromROS(uint8_t data) {
  _cmdManualROS = data;
}

/*
   Function: setCmdSpeedFromROS
   ----------------------------
   This function read the robot speed [m/s] receved via ROS topic.
*/
void Robot::setCmdSpeedFromROS(float data) {
  _cmdRobotSpeedROS = data;
  _cmdWheelSpeedROS = data / (0.5 * _wheelDiameter_Meter);
}


/*
   Function: setCmdTrayFromROS
   ----------------------------
   This function read the position receved via ROS topic.
   msg.x = x position
   msg.y = y position
   msg.z = target speed. If zero the robot stops when the position is reached.
*/
void Robot::setCmdTrayFromROS(float x, float y, float pointTargetSpeed) {
  _pointTargetSpeed = pointTargetSpeed;
  _robotPosXTarget = x; _robotPosYTarget = y;
  _robotPosXInit = _robotPosX; _robotPosYInit = _robotPosY;
}


/*
   Function: setLidarPosFromROS
   ----------------------------
   This function save lidar position receved via ROS topic.
   Note: The lidar coordinates is not the same as robot one.
         RobotX = lidarY - offset Y
         RobotY = - lidarX - offsetX
*/
void Robot::setLidarPosFromROS(float x, float y, float qx, float qy, float qz, float qw, float lidarOfstX, float lidarOfstY) {
  _lidarPosX = y - lidarOfstY;
  _lidarPosY =(-1)*x - lidarOfstX;

  // yaw (z-axis rotation) quaternion to euler
  double siny_cosp = 2 * (qw * qz + qx * qy);
  double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
  _lidarThetaZ = atan2(siny_cosp, cosy_cosp);
}



/*
   Function: getDataSendToROS
   ----------------------------
   This function gets all the data to be send via ROS.
*/
void Robot::getDataSendToROS(geometry_msgs::Pose* pose, geometry_msgs::Pose* s, geometry_msgs::Point32* im, geometry_msgs::Point32* li, geometry_msgs::Point32* kf){

  // Robot odometry position
  pose->position.x = _odomPosX;
  pose->position.y = _odomPosY;
  pose->position.z = _odomThetaZ;

  // Robot wheel speed
  pose->orientation.x = _speedMotorFR;
  pose->orientation.y = _speedMotorFL;
  pose->orientation.z = _speedMotorRR;
  pose->orientation.w = _speedMotorRL;

  // Robot speed
  s->position.x = _vX;
  s->position.y = _vY;
  s->position.z = _wZ;
  s->orientation.x = _w1;
  s->orientation.y = _w2;
  s->orientation.z = _w3;
  s->orientation.w = _w4;

  //Robot IMU 
  im->x = imu->getOrientationX();
  im->y = 0;
  im->z = 0;

  // Robot lidar
  li->x = _lidarPosX;
  li->y = _lidarPosY;
  li->z = _lidarThetaZ;

  // Robot Kalman Filter
  kf->x = _kalmanPosX;
  kf->y = _kalmanPosY;
  kf->z = _kalmanThetaZ;
}


/*
   Function: initKalmanFilter
   ----------------------------
   This function initialize Kalman Filter.
*/
void Robot::initKalmanFilter(){
  state.Fill(0.0);
  obs.Fill(0.0);
  u.Fill(0.0);
}


/*
   Function: updateKalmanFilter
   ----------------------------
   This function update the Kalman Filter.
   Is divided in two step:
      1. Prediction based on odometry and kinematic
      2. Update (correction) when lidar data ara available.
         The Lidar measurament is fixed by adding at the measurament the estimated distance during the delay.
         Calculated as the avarage speed during the delay multiplied by the delay. 
*/
void Robot::updateKalmanFilter(){
           
    // Input vector
    u(0) = _w1;
    u(1) = _w2;
    u(2) = _w3;
    u(3) = _w4;

    float lidarDelay = 960; // Lidar delay in milliseconds
    
    if(_updateLidarCounter >= (LIDAR_UPDATE_PERIOD / _periodMs)){
      obs(0) = imu->getOrientationX();
      obs(1) = _lidarPosX + (_vXSum / _updateLidarCounter) * (lidarDelay / 1000.0);  // Fix the error caused by lidar and SLAM delay.
      obs(2) = _lidarPosY + (_vYSum / _updateLidarCounter) * (lidarDelay / 1000.0);
      obs(3) = _lidarThetaZ + (_wZSum / _updateLidarCounter) * (lidarDelay / 1000.0);
      K.update(obs,u);
      _updateLidarCounter = 0;  
      _vXSum = 0; _vYSum = 0; _wZSum = 0;
    }
    else{
      K.predict(u);
      _vXSum = _vXSum + _vX;
      _vYSum = _vYSum + _vY;
      _wZSum = _wZSum + _wZ;
      _updateLidarCounter ++;
    }
 
    _kalmanPosX = K.x(0); 
    _kalmanPosY = K.x(1);
    _kalmanThetaZ = K.x(2);

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
  _speedMotorFR = _cmdWheelSpeedROS;
  _speedMotorFL = _cmdWheelSpeedROS;
  _speedMotorRR = _cmdWheelSpeedROS;
  _speedMotorRL = _cmdWheelSpeedROS;
}

/*
   Function: runBackward
   ----------------------------
   Change the motor speed to go backward.
   The speed is in rpm and its provided via ROS.
*/
void Robot::runBackward() {
  _speedMotorFR = (-1) * _cmdWheelSpeedROS;
  _speedMotorFL = (-1) * _cmdWheelSpeedROS;
  _speedMotorRR = (-1) * _cmdWheelSpeedROS;
  _speedMotorRL = (-1) * _cmdWheelSpeedROS;
}


/*
   Function: runRight
   ----------------------------
   Change the motor speed to go right.
   The speed is in rpm and its provided via ROS.
*/
void Robot::runRight() {
  _speedMotorFR = (-1) * _cmdWheelSpeedROS;
  _speedMotorFL =  _cmdWheelSpeedROS;
  _speedMotorRR =  _cmdWheelSpeedROS;
  _speedMotorRL = (-1) * _cmdWheelSpeedROS;
}

/*
   Function: runLeft
   ----------------------------
   Change the motor speed to go left.
   The speed is in rpm and its provided via ROS.
*/
void Robot::runLeft() {
  _speedMotorFR = _cmdWheelSpeedROS;
  _speedMotorFL = (-1) * _cmdWheelSpeedROS;
  _speedMotorRR = (-1) * _cmdWheelSpeedROS;
  _speedMotorRL = _cmdWheelSpeedROS;
}

/*
   Function: runForwardRight
   ----------------------------
   Change the motor speed to go diagonal forward-right.
   The speed is in rpm and its provided via ROS.
*/
void Robot::runForwardRight() {
  _speedMotorFR = 0;
  _speedMotorFL = _cmdWheelSpeedROS;
  _speedMotorRR = _cmdWheelSpeedROS;
  _speedMotorRL = 0;
}

/*
   Function: runForwardLeft
   ----------------------------
   Change the motor speed to go diagonal forward-left.
   The speed is in rpm and its provided via ROS.
*/
void Robot::runForwardLeft() {
  _speedMotorFR = _cmdWheelSpeedROS;
  _speedMotorFL = 0;
  _speedMotorRR = 0;
  _speedMotorRL = _cmdWheelSpeedROS;
}

/*
   Function: runBackwardRight
   ----------------------------
   Change the motor speed to go diagonal backward-right.
   The speed is in rpm and its provided via ROS.
*/
void Robot::runBackwardRight() {
  _speedMotorFR = (-1) * _cmdWheelSpeedROS;
  _speedMotorFL = 0;
  _speedMotorRR = 0;
  _speedMotorRL = (-1) * _cmdWheelSpeedROS;
}

/*
   Function: runBackwardLeft
   ----------------------------
   Change the motor speed to go diagonal backward-left.
   The speed is in rpm and its provided via ROS.
*/
void Robot::runBackwardLeft() {
  _speedMotorFR = 0;
  _speedMotorFL = (-1) * _cmdWheelSpeedROS;
  _speedMotorRR = (-1) * _cmdWheelSpeedROS;
  _speedMotorRL = 0;
}

/*
   Function: runRotateCW
   ----------------------------
   Change the motor speed to rotate clockwise.
   The speed is in rpm and its provided via ROS.
*/
void Robot::runRotateCW() {
  _speedMotorFR = (-1) * _cmdWheelSpeedROS;
  _speedMotorFL = _cmdWheelSpeedROS;
  _speedMotorRR = (-1) * _cmdWheelSpeedROS;
  _speedMotorRL = _cmdWheelSpeedROS;
}

/*
   Function: runRotateCCW
   ----------------------------
   Change the motor speed to rotate counter clockwise.
   The speed is in rpm and its provided via ROS.
*/
void Robot::runRotateCCW() {
  _speedMotorFR = _cmdWheelSpeedROS;
  _speedMotorFL = (-1) * _cmdWheelSpeedROS;
  _speedMotorRR = _cmdWheelSpeedROS;
  _speedMotorRL = (-1) * _cmdWheelSpeedROS;
}


void Robot::serialDebug() {
  Serial.print("MODE  "); Serial.print(_cmdModeROS); Serial.print(" CMD MAN  "); Serial.print(_cmdManualROS);
  Serial.println("");
  Serial.print("REF WHEEL SPEED rad/s  "); Serial.print(_cmdWheelSpeedROS); Serial.print(" Profile  "); Serial.print(_speedProfileROS);
  Serial.println("");
  Serial.print("WHEEL SPEED rad/s  FR"); Serial.print(_w1); Serial.print(" FL "); Serial.print(_w2); Serial.print(" RL "); Serial.print(_w3); Serial.print(" RR "); Serial.print(_w4);
  Serial.println("");
  Serial.print("POS TARGET "); Serial.print(_robotPosXTarget); Serial.print(" "); Serial.print(_robotPosYTarget);  Serial.print(" end speed "); Serial.print(_pointTargetSpeed);
  Serial.println("");
  Serial.print("POS REAL "); Serial.print(_robotPosX); Serial.print(" "); Serial.print(_robotPosY); Serial.print(" ");
  Serial.println("");
  Serial.print("POS ESTIMATED "); Serial.print(_kalmanPosX); Serial.print(" "); Serial.print(_kalmanPosY); Serial.print(" "); Serial.print(_kalmanThetaZ); Serial.print(" ");
  Serial.println("");
  if(_useKalmanFilter)  Serial.println(" Kalman Filter correction active ");
  if(_fixedOrientation)    Serial.println(" IMU correction active ");
}
