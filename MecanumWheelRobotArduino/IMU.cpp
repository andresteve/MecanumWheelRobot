#include "IMU.h"

IMU::IMU(uint8_t periodMs){
  _bno = Adafruit_BNO055(55,IMU_ADDRESS,&Wire1);
  _periodS = (float) periodMs / 1000.0;
  _accPosConst = (double) _periodS * _periodS;
  _degToRadConst = 0.01745329252;
  _posX = 0; _posY = 0; _thetaZ; _vX = 0, _vY=0, _wZ=0;
  _thetaXOfst = 0; _thetaYOfst = 0; _thetaZOfst = 0;
}

void IMU::init(){
   if(!_bno.begin()){
     Serial.println("IMU not found");
   }
   _bno.setExtCrystalUse(true);
}

bool IMU::getCalibration(){
   uint8_t system, gyro, accel, mag;
   system = gyro = accel = mag = 0;
   _bno.getCalibration(&system, &gyro, &accel, &mag);
   if(gyro >= 2 || mag >= 1){
      return true;
   }
   else
     return false;
}

void IMU::calculatePos(){
  if(getCalibration()){
    _bno.getEvent(&_orientationData,Adafruit_BNO055::VECTOR_EULER);
    _bno.getEvent(&_linearAccelData,Adafruit_BNO055::VECTOR_LINEARACCEL);
    _bno.getEvent(&_angVelocityData,Adafruit_BNO055::VECTOR_GYROSCOPE);
    _posX = _posX + _accPosConst * _linearAccelData.acceleration.x;
    _posY = _posY + _accPosConst * _linearAccelData.acceleration.y;
    _vX = _vX + _linearAccelData.acceleration.x * _periodS;
    _vY = _vY + _linearAccelData.acceleration.y * _periodS;
    _wZ = _wZ +_angVelocityData.gyro.z;
  }
}

double IMU::getPosX(){
  return _posX;
}

double IMU::getPosY(){
  return _posY;
}

double IMU::getVelX(){
  return _vX;
}

double IMU::getVelY(){
  return _vY;
}

double IMU::getAngVelZ(){
  return _wZ;
}

void IMU::calculateOrientation(){
  if(getCalibration()){
    sensors_event_t event; 
    _bno.getEvent(&event);
    _thetaX = (-1) * (event.orientation.x - _thetaXOfst);
    _thetaY = event.orientation.y - _thetaYOfst;
    _thetaZ = event.orientation.z - _thetaZOfst;
  }
}

void IMU::calculateOffset(){
  uint8_t i, nSamples = 10;
  double xSum=0, ySum=0, zSum=0;
  sensors_event_t event; 
  if(getCalibration()){
    for (i=0;i<nSamples;i++){
       _bno.getEvent(&event);
       xSum += event.orientation.x;
       ySum += event.orientation.y;
       zSum += event.orientation.z;
    }
    _thetaXOfst = (double) xSum / nSamples;
    _thetaYOfst = (double) ySum / nSamples;
    _thetaZOfst = (double) zSum / nSamples;
  }
}

double IMU::getOrientationX(){
  return _thetaX * _degToRadConst;
}


double IMU::getOrientationY(){
  return _thetaY * _degToRadConst;
}


double IMU::getOrientationZ(){
  return _thetaZ * _degToRadConst;
}
