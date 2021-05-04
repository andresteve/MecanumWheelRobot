#ifndef ENCODER_H
#define ENCODER_H


#include <Arduino.h>
#define GEAR_RATIO  70
#define ENCODER_CPR 64

class Encoder {
  public:
    Encoder(uint8_t pinChA, uint8_t pinChB);
    long getEncoderValue();
    long getEncoderValueOld();
    uint16_t getEncoderResolution();
    void setEncoderValueOld(long encValue);
    void updateEncoderChA();
    void updateEncoderChB();
    
  private:
    long _encoderValue;
    long _encoderValueOld;
    uint8_t _pinChA;
    uint8_t _pinChB;
};

#endif
