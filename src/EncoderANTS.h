/*
  Encoder.h - library created to support motor operation by providing feedback. Created for iANTS by Dmitrii Gusev, Electronic Systems Engineer
*/

#ifndef EncoderANTS_h
#define EncoderANTS_h

#include "Adafruit_MCP23017.h"
#include "Arduino.h"

class EncoderANTS
{
    public:
    EncoderANTS(int controlA, int controlB);
    void begin(Adafruit_MCP23017 *encControl);
    void getFeedback(Adafruit_MCP23017 *encControl);
    int readCurrentPosition();
    void setCurrentPosition(int resetPosition);

    protected:
    int encoderPosition; 
    int encoderPinALast; 

    private:
    int pinA;
    int pinB;
};

#endif