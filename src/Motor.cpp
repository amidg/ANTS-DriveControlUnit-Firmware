/*
  Motor.h - library created to support motor operation. Created for iANTS by Dmitrii Gusev, Electronic Systems Engineer
*/

#include "Arduino.h"
#include "Motor.h"
#include "analogWrite.h"
#include "Adafruit_MCP23017.h"

Motor::Motor(int control1, int control2, int pwmPin)
{
  IN1 = control1;
  IN2 = control2;
  PWM = pwmPin;
}

void Motor::begin(Adafruit_MCP23017 *control) {
  control->pinMode(IN1, OUTPUT);
  control->pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);
}

void Motor::go(Adafruit_MCP23017 *control, int directionAndPower)
{
  int power = abs(directionAndPower);
  currentPWM = power;

  //controls side
  if(directionAndPower >= 0) {
    //if positive, go forward
    control->digitalWrite(IN1, HIGH);
    control->digitalWrite(IN2, LOW);
  } else if (directionAndPower < 0) {
    //if negative, go backwards
    control->digitalWrite(IN1, LOW);
    control->digitalWrite(IN2, HIGH);
  }

  for (int i = 0; i < power; i++) { //soft start to avoid BJT back current
    analogWrite(PWM, i);
    delay(10);
  }
}

void Motor::stop(Adafruit_MCP23017 *control)
{
  control->digitalWrite(IN1, LOW);
  control->digitalWrite(IN2, LOW); 

  for (int i = currentPWM; i >= 0; i--) { //soft slowdown to avoid BJT damage
    analogWrite(PWM, i);
    delay(10);
  }
}