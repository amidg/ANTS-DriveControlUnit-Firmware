#include "Arduino.h"
#include <Wire.h>

//pin definitions:
#define GIGAVACENABLE 14

void setup()
{
  Serial.begin (9600);  
  pinMode(GIGAVACENABLE, OUTPUT);
}

void loop()
{
  digitalWrite(GIGAVACENABLE, HIGH);
  delay(3000);
  digitalWrite(GIGAVACENABLE, LOW);
  delay(3000);
}