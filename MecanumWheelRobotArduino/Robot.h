#ifndef ROBOT_H
#define ROBOT_H

#include <Adafruit_MotorShield.h>
#include <Adafruit_PWMServoDriver.h>
#include "Motor.h"
#include "IMU.h"
#include "ros.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Pose.h"
#include "Kalman.h"


using namespace BLA;
#define Nstate 6       // Number of the system state 
#define Nobs 4         // Number of the observable (mesurable) states
#define Ncom 4         // Number of input states

// measurement covariance (to be characterized from your sensors)
#define n1  0.05  // IMU orientation z
#define n2  0.01  // Lidar position x
#define n3  0.01  // Lidar position y
#define n4  0.01  // Lidar orientation z

// model std (~1/inertia). Freedom you give to relieve your evolution equation
#define m1 0.001  // Odometry covariance: position X
#define m2 0.001  // Odometry covariance: position Y
#define m3 0.8    // Odometry covariance: orientation Z
#define m4 0.001  // Kinematic covariance: linear speed X
#define m5 0.001  // Kinematic covariance: linear speed Y
#define m6 0.8    // Kinematic covariance: angular speed Z

// Update kalman filter depending on slam frecuency rate
#define LIDAR_UPDATE_PERIOD 100

/****************** ROS COMMANDS  ***********************/

#define SPEED_PROFILE_SQUARE       0
#define SPEED_PROFILE_TRAPEZOIDAL  1
#define SPEED_PROFILE_S_CURVE      2

#define ROS_CMD_MODE_MAN      0
#define ROS_CMD_MODE_AUT      1

#define ROS_CMD_STOP         0
#define ROS_CMD_FWD          1
#define ROS_CMD_BCK          2
#define ROS_CMD_RIGHT        3
#define ROS_CMD_LEFT         4
#define ROS_CMD_FWD_RIGHT    5
#define ROS_CMD_BCK_RIGHT    6
#define ROS_CMD_BCK_LEFT     7
#define ROS_CMD_FWD_LEFT     8
#define ROS_CMD_CW           9
#define ROS_CMD_CCW          10
#define ROS_CMD_IMU_RST      11
#define ROS_CMD_ODO_RST      12
#define ROS_CMD_SPEED_PROF_SQ   13
#define ROS_CMD_SPEED_PROF_TR   14
#define ROS_CMD_SPEED_PROF_SC   15
#define ROS_CMD_FIXED_ORIENTATION_ON   16
#define ROS_CMD_FIXED_ORIENTATION_OFF  17
#define ROS_CMD_KALMAN_ON   18
#define ROS_CMD_KALMAN_OFF  19


class Robot {
  public:
    Robot(uint8_t motorPinFR, uint8_t motorPinFL, uint8_t motorPinRR, uint8_t motorPinRL,
          uint8_t encoderFRChA, uint8_t encoderFRChB, uint8_t encoderFLChA, uint8_t encoderFLChB,
          uint8_t encoderRRChA, uint8_t encoderRRChB, uint8_t encoderRLChA, uint8_t encoderRLChB,
          uint8_t periodMs, float wheelToCenterX, float wheelToCenterY, uint16_t wheelDiameter);
    void init();
    void setAllMotorStop();
    void runForward();
    void runBackward();
    void runRight();
    void runLeft();
    void runForwardRight();
    void runForwardLeft();
    void runBackwardRight();
    void runBackwardLeft();
    void runRotateCW();
    void runRotateCCW();
    void getROSData();
    void ROSControl();
    void commandModeManual();
    void commandModeAutomatic();
    void trapezoidalSpeedProfile(float endSpeed, float trayMaxSpeed, float targetZone);
    void squareSpeedProfile(float endSpeed, float trayMaxSpeed, float targetZone);
    void sCurveSpeedProfile(float endSpeed, float trayMaxSpeed, float targetZone);
    void updateSpeed();
    void updatePos();
    void calculateSpeedMs();
    void serialDebug();
    void resetPosition();
    bool isPositionResetted();
    void updateKalmanFilter();
    void initKalmanFilter();

    //From ROS subscriber
    void setCmdModeFromROS(uint8_t data);
    void setCmdManualFromROS(uint8_t data);
    void setCmdSpeedFromROS(float data);
    void setCmdTrayFromROS(float x, float y, float pointTargetSpeed);
    void setLidarPosFromROS(float x, float y, float qx, float qy, float qz, float qw, float lidarOfstX, float lidarOfstY);
    void getDataSendToROS(geometry_msgs::Pose* pose, geometry_msgs::Pose* s, geometry_msgs::Point32* im, geometry_msgs::Point32* li, geometry_msgs::Point32* kf);

    Motor* motorFR;  //FrontRight
    Motor* motorFL;  //FrontLeft
    Motor* motorRR;  //RearRight
    Motor* motorRL;  //RearLeft
    IMU *imu;        //IMU

  private:
    Adafruit_MotorShield* AFMS;
    float _wheelDiameter_Meter, _robotMaxSpeedMs;
    float _wheelToCenterX_Meter, _wheelToCenterY_Meter, _periodS, _periodMs;
    float _robotPosX, _robotPosY, _robotThetaZ;
    float  _robotPosXInit, _robotPosYInit, _robotPosXTarget, _robotPosYTarget, _pointTargetSpeed;
    float _globalPosXTarget, _globalPosYTarget;
    float _vX, _vY, _wZ, _w1, _w2, _w3, _w4;
    float _speedMotorFR, _speedMotorFL, _speedMotorRR, _speedMotorRL;
    float _IkOrientOld;

    //Odometry data
    float _odomPosX, _odomPosY, _odomThetaZ;

    //Lidar data
    float _lidarPosX, _lidarPosY, _lidarThetaZ;
    
    //ROS communication data
    uint8_t _cmdModeROS, _cmdManualROS, _speedProfileROS;
    float _cmdRobotSpeedROS, _cmdWheelSpeedROS;

    //Kalman Filter
    KALMAN<Nstate,Nobs,Ncom> K;   //Kalman filter
    BLA::Matrix<Nstate> state;    // State vector
    BLA::Matrix<Nobs> obs;        // Observation vector
    BLA::Matrix<Ncom> u;          // Input vector
    float _kalmanPosX, _kalmanPosY, _kalmanThetaZ;
    float _vXSum, _vYSum, _wZSum;
    uint8_t _updateLidarCounter, _updateIMUCounter;

    //Positioning possible correction
    bool _useKalmanFilter, _fixedOrientation;

    //Trajectory data
    typedef struct {
      float accDecZone;
      float accDecZone1;
      float distToInit;
      float distToTarget;
      float desiredSpeed;
    } trayData;
    float _desiredSpeedX, _desiredSpeedY, _alphaToTarget;
    float  _vXToTarget, _vYToTarget, _wZToTarget;
    trayData _trayDataX, _trayDataY;
    float _positioningSpeed, _trayMaxSpeed, _trayNearTarget; 

};

#endif
