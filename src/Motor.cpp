/*
  Motor.h - library created to support motor operation. Created for iANTS by Dmitrii Gusev, Electronic Systems Engineer
*/

#include "Arduino.h"
#include "Motor.h"

Motor::Motor(int control1, int control2, int pwmPin)
{
  pinMode(control1, OUTPUT);
  pinMode(control2, OUTPUT);
  pinMode(pwmPin, OUTPUT);

  IN1 = control1;
  IN2 = control2;
  PWM = pwmPin;
}

void Motor::go(int directionAndPower)
{
  int power = abs(directionAndPower);
  //analogWrite(PWM, power);

  //controls side
  if(directionAndPower >= 0) {
    //if positive, go forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (directionAndPower < 0) {
    //if negative, go backwards
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
}

void Motor::stop()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW); 
  //analogWrite(PWM, 0);
}