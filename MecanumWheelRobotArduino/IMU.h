#ifndef IMU_H
#define IMU_H


#include <Arduino.h>
#include <Adafruit_BNO055.h>
#define IMU_ADDRESS 0x29

class IMU {
  public:
    IMU(uint8_t periodMs);
    void init();
    void calculatePos();
    void calculateOrientation();
    void calculateOffset();
    bool getCalibration();
    double getPosX();
    double getPosY();
    double getOrientationX();
    double getOrientationY();
    double getOrientationZ();
    double getVelX();
    double getVelY();
    double getAngVelZ();
        
  private:
    Adafruit_BNO055 _bno;
    float _periodS;
    sensors_event_t _orientationData, _linearAccelData, _angVelocityData ;
    double _posX, _posY,_thetaX, _thetaY, _thetaZ, _vX, _vY, _wZ;
    double _thetaXOfst, _thetaYOfst, _thetaZOfst;
    double _accPosConst, _degToRadConst;
};

#endif
