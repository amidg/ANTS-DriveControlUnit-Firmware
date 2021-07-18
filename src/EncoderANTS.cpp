/*
  Encoder.h - library created to support motor operation by providing feedback. Created for iANTS by Dmitrii Gusev, Electronic Systems Engineer
*/

#include "Arduino.h"
#include "EncoderANTS.h"

EncoderANTS::EncoderANTS(int controlA, int controlB)
{
    pinMode(controlA, INPUT);
    pinMode(controlB, INPUT);

    pinA = controlA;
    pinB = controlB;
}

int EncoderANTS::getFeedback() 
{
    int n = digitalRead(pinA);
    
    if ((encoderPinALast == LOW) && (n == HIGH)) {
        if (digitalRead(pinB) == LOW) {
            encoderPosition--;
        } else {
            encoderPosition++;
        }
    }
    encoderPinALast = n;
    
    return encoderPosition;
}