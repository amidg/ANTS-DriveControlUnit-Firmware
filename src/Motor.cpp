/*
  Motor.h - library created to support motor operation. Created for iANTS by Dmitrii Gusev, Electronic Systems Engineer
*/

#include "Arduino.h"
#include "Motor.h"
#include "analogWrite.h"
#include "Adafruit_MCP23017.h"

Motor::Motor(int control1, int control2, int pwmPin) {
  IN1 = control1;
  IN2 = control2;

  IN3 = IN1; //placeholders to ensure no erros in single channel operation
  IN4 = IN1;

  PWM1 = pwmPin;
  PWM2 = PWM1; //placeholders to ensure no erros in single channel operation

  pinMode(PWM1, OUTPUT);
}

Motor::Motor(int control1, int control2, int control3, int control4, int pwmPin1, int pwmPin2) {
  //2ch Motor constructor
  IN1 = control1;
  IN2 = control2;
  IN3 = control3;
  IN4 = control4;

  PWM1 = pwmPin1;
  PWM2 = pwmPin2; //placeholders to ensure no erros in single channel operation

  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
}

void Motor::begin(Adafruit_MCP23017 *control) {
  //same function for both 2ch and 1ch operation. 
  //If single channel is used, this function will just work in dummy mode for IN3 and IN4
  control->pinMode(IN1, OUTPUT);
  control->pullUp(IN1, 0);
  control->pinMode(IN2, OUTPUT);
  control->pullUp(IN2, 0);
  control->pinMode(IN3, OUTPUT);
  control->pullUp(IN3, 0);
  control->pinMode(IN4, OUTPUT);
  control->pullUp(IN4, 0);
} 

void Motor::go(Adafruit_MCP23017 *control, int directionAndPower) {
  int power = abs(directionAndPower);
  
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

  if (currentPWM <= power) {
    for (int i = currentPWM; i < power; i++) {
      analogWrite(PWM1, i);
      delay(50);
    }
    
  } else if (currentPWM > power) {
    for (int i = currentPWM; i > power; i--) {
      analogWrite(PWM1, i);
      delay(50);
    }
  }

  currentPWM = power; //set current PWM, needed for prevent soft start cycling
}

void Motor::go2ch(Adafruit_MCP23017 *control, int directionAndPower) {
  //move motors in 2ch operation, this function requires hardware verification to ensure no damage to DCU
  int power = abs(directionAndPower);
  
  //controls side
  if(directionAndPower >= 0) {
    //if positive, go forward
    control->digitalWrite(IN1, HIGH);
    control->digitalWrite(IN2, LOW);
    control->digitalWrite(IN3, HIGH);
    control->digitalWrite(IN4, LOW);
  } else if (directionAndPower < 0) {
    //if negative, go backwards
    control->digitalWrite(IN1, LOW);
    control->digitalWrite(IN2, HIGH);
    control->digitalWrite(IN3, LOW);
    control->digitalWrite(IN4, HIGH);
  }

  if (currentPWM <= power) {
    for (int i = currentPWM; i < power; i++) {
      analogWrite(PWM1, i);
      analogWrite(PWM2, i);
      delay(50);
    }
    
  } else if (currentPWM > power) {
    for (int i = currentPWM; i > power; i--) {
      analogWrite(PWM1, i);
      analogWrite(PWM2, i);
      delay(50);
    }
  }

  currentPWM = power; //set current PWM, needed for prevent soft start cycling
}

void Motor::stop(Adafruit_MCP23017 *control) {
  for (int i = currentPWM; i >= 0; i--) { //soft slowdown to avoid BJT damage
    analogWrite(PWM1, i);
    delay(10);
  }

  control->digitalWrite(IN1, LOW); //fully turn off motors
  control->digitalWrite(IN2, LOW); 

  currentPWM = 0;
}

void Motor::stop2ch(Adafruit_MCP23017 *control) {
  //stopping in two channel mode
  for (int i = currentPWM; i >= 0; i--) { //soft slowdown to avoid BJT damage
    analogWrite(PWM1, i);
    analogWrite(PWM2, i);
    delay(10);
  }

  control->digitalWrite(IN1, LOW); //fully turn off motors
  control->digitalWrite(IN2, LOW); 

  currentPWM = 0;
}