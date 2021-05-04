#ifndef ROBOT_H
#define ROBOT_H

#include <Adafruit_MotorShield.h>
#include <Adafruit_PWMServoDriver.h>
#include "Motor.h"
#include "IMU.h"
#include "ros.h"
#include "geometry_msgs/Point32.h"


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
#define ROS_CMD_IMU_CORR_ON     16
#define ROS_CMD_IMU_CORR_OFF    17
#define ROS_CMD_LIDAR_CORR_ON   18
#define ROS_CMD_LIDAR_CORR_OFF  19


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
    void trapezoidalSpeedProfile(float targetSpeed, float targetZone);
    void squareSpeedProfile(float targetSpeed, float targetZone);
    void sCurveSpeedProfile(float endSpeed, float maxSpeed, float targetZone);
    void updateSpeed();
    void updatePos();
    void calculateSpeedMs();
    void calculatePos();
    void serialDebug();
    bool setOdometryPosition(float x, float y, float theta);
    bool lidarOdometryCorrection();

    //From ROS subscriber
    void setCmdModeFromROS(uint8_t data);
    void setCmdManualFromROS(uint8_t data);
    void setCmdSpeedFromROS(float data);
    void setCmdTrayFromROS(float x, float y, float pointTargetSpeed);
    void setLidarPosFromROS(float x, float y);
    void getDataSendToROS(geometry_msgs::Point32* pose, geometry_msgs::Point32* s, geometry_msgs::Point32* im);

    Motor* motorFR;  //FrontRight
    Motor* motorFL;  //FrontLeft
    Motor* motorRR;  //RearRight
    Motor* motorRL;  //RearLeft
    IMU *imu;        //IMU
    
   private:

    typedef struct{
      float accDecZone;
      float accDecZone1;
      float distToInit;
      float distToTarget;
      float desiredSpeed;
    }trayData;
    
    Adafruit_MotorShield* AFMS; 
    float _wheelDiameter_Meter, _robotMaxSpeedMs, _radsToRpmConst, _rpmToMsConst, _msToRpmConst;
    float _wheelToCenterX_Meter, _wheelToCenterY_Meter, _periodS;
    float _posX, _posY;
    double _thetaX, _thetaY, _thetaZ;
    float _vX, _vY, _vR, _wZ;
    float _speedMotorFR,_speedMotorFL, _speedMotorRR,_speedMotorRL;
    float  _posXInit, _posYInit, _posXTarget, _posYTarget, _pointTargetSpeed, _pointTargetSpeedOld, _alphaToTarget;
    float _desiredSpeedX, _desiredSpeedY;
    float  _vXToTarget, _vYToTarget, _wZToTarget;
    trayData _trayDataX, _trayDataY;
    float _IkOrientOld;

    // ROS communication data
    uint8_t _cmdModeROS, _cmdManualROS, _speedProfileROS;
    float _cmdSpeedROSMs, _cmdSpeedROSRpm;

    // Odometry correction 
    bool _lidarCorrection, _imuCorrection;
    float _lidarPosX, _lidarPosY;
    
};

#endif
