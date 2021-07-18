#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_MCP23017.h" //please run pio lib install "adafruit/Adafruit BusIO@^1.8.2" before proceeding
#include "analogWrite.h"

//pin definitions:
#define GIGAVACENABLE 14
#define MOTOR1PWM 26

/*
  MCP23017 L298P PIN DEFINITIONS:
  GPA0 -> M1-IN1 (pins 1 and 4) ---> USE PIN ID 0
  GPA1 -> M1-IN2 (pins 2 and 3)

  GPA2 -> M2-IN1 (pins 1 and 4)
  GPA3 -> M2-IN2 (pins 2 and 3)

  GPA4 -> M3-IN1 (pins 1 and 4)
  GPA5 -> M3-IN2 (pins 2 and 3)

  GPA6 -> M4-IN1 (pins 1 and 4)
  GPA7 -> M4-IN2 (pins 2 and 3) ----> USE PIN ID 7

  addr 0 = A2 low , A1 low , A0 low  000 --> motor control MCP23017
  addr 1 = A2 low , A1 low , A0 high 001
  addr 2 = A2 low , A1 high , A0 low  010 --> encoder MCP23017
  addr 3 = A2 low , A1 high , A0 high  011
  addr 4 = A2 high , A1 low , A0 low  100
  addr 5 = A2 high , A1 low , A0 high  101
  addr 6 = A2 high , A1 high , A0 low  110
  addr 7 = A2 high, A1 high, A0 high 111
*/

Adafruit_MCP23017 motorControl;

void setup()
{
  Serial.begin (9600);  
  Wire.begin();
  motorControl.begin(0, &Wire); //specified custom address
  pinMode(GIGAVACENABLE, OUTPUT);
  pinMode(MOTOR1PWM, OUTPUT);
  motorControl.pinMode(0, OUTPUT); //motor1, input 1/4
  motorControl.pinMode(1, OUTPUT); //motor1, input 2/3 
  motorControl.pinMode(2, OUTPUT); //motor2, input 1/4
  motorControl.pinMode(3, OUTPUT); //motor2, input 2/3
  motorControl.pinMode(4, OUTPUT); //motor3, input 1/4
  motorControl.pinMode(5, OUTPUT); //motor3, input 2/3
  motorControl.pinMode(6, OUTPUT); //motor4, input 1/4
  motorControl.pinMode(7, OUTPUT); //motor4, input 2/3
}

void loop()
{
  digitalWrite(GIGAVACENABLE, HIGH);

  //TEST MOTOR 1
  motorControl.digitalWrite(0, HIGH);
  motorControl.digitalWrite(1, LOW);
  analogWrite(MOTOR1PWM, 255);
}