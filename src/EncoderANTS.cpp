/*
  Encoder.h - library created to support motor operation by providing feedback. Created for iANTS by Dmitrii Gusev, Electronic Systems Engineer
*/

#include "Arduino.h"
#include "EncoderANTS.h"
#include "Adafruit_MCP23017.h"

EncoderANTS::EncoderANTS(int controlA, int controlB) {
    pinA = controlA;
    pinB = controlB;
}

int EncoderANTS::readCurrentPosition() {
    return encoderPosition;
}

void EncoderANTS::setCurrentPosition(int resetPosition) {
    encoderPosition = resetPosition;
}

void EncoderANTS::begin(Adafruit_MCP23017 *encControl) {
    //function to begin the encoder instance
    encControl->pinMode(pinA, INPUT);
    encControl->pullUp(pinA, 0);
    encControl->setupInterruptPin(pinA, CHANGE);

    encControl->pinMode(pinB, INPUT);
    encControl->pullUp(pinB, 0);
    encControl->setupInterruptPin(pinB, CHANGE);
}

void EncoderANTS::getFeedback(Adafruit_MCP23017 *encControl) {
    int n = encControl->digitalRead(pinA);
    
    if ((encoderPinALast == LOW) && (n == HIGH)) {
        if (encControl->digitalRead(pinB) == LOW) {
            encoderPosition--;
        } else {
            encoderPosition++;
        }
    }
    encoderPinALast = n;
}