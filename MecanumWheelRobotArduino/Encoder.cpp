#include "Encoder.h"

Encoder::Encoder(uint8_t pinChA, uint8_t pinChB){
  _pinChA = pinChA;
  _pinChB = pinChB;
  pinMode(_pinChA,INPUT);
  pinMode(_pinChB,INPUT);
  _encoderValue = 0;
  _encoderValueOld = _encoderValue;
}

  
void Encoder::updateEncoderChA(){
  if(digitalRead(_pinChA) == digitalRead(_pinChB))  _encoderValue--; 
  else                                              _encoderValue++;
}

void Encoder::updateEncoderChB(){
  if(digitalRead(_pinChA) == digitalRead(_pinChB))   _encoderValue++; 
  else                                               _encoderValue--;
}

long Encoder::getEncoderValue(){
  return  _encoderValue;
}

long Encoder::getEncoderValueOld(){
  return  _encoderValueOld;
}

uint16_t Encoder::getEncoderResolution(){
  return GEAR_RATIO*ENCODER_CPR ;
}

void Encoder::setEncoderValueOld(long encValue){
  _encoderValueOld = encValue;
}
