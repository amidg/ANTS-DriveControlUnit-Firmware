/*
  Encoder.h - library created to support motor operation by providing feedback. Created for iANTS by Dmitrii Gusev, Electronic Systems Engineer
*/

#ifndef EncoderANTS_h
#define EncoderANTS_h

#include "Arduino.h"

class EncoderANTS
{
    public:
    EncoderANTS(int controlA, int controlB);
    int getFeedback();

    protected:
    int encoderPosition; 
    int encoderPinALast; 

    private:
    int pinA;
    int pinB;
};

#endif