#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_MCP23017.h" //please run pio lib install "adafruit/Adafruit BusIO@^1.8.2" before proceeding

//pin definitions:
#define GIGAVACENABLE 14

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
}

void loop()
{
  digitalWrite(GIGAVACENABLE, HIGH);
  delay(3000);
  digitalWrite(GIGAVACENABLE, LOW);
  delay(3000);
}