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

  MCP23017 ENCODER PIN DEFINITIONS:
  GPA0 -> ENC1-A ---> USE PIN ID 0 --> MOTOR 1 (FRONT RIGHT)
  GPA1 -> ENC1-B ---> USE PIN ID 1

  GPA2 -> ENC2-A ---> USE PIN ID 2 --> MOTOR 2 (FRONT LEFT)
  GPA3 -> ENC2-B ---> USE PIN ID 3

  GPA4 -> ENC3-A ---> USE PIN ID 4 --> MOTOR 3 (REAR LEFT)
  GPA5 -> ENC3-B ---> USE PIN ID 5

  GPA6 -> ENC4-A ---> USE PIN ID 6 --> MOTOR 4 (REAR RIGHT)
  GPA7 -> ENC4-B ---> USE PIN ID 7
*/

#include "Arduino.h"

#define GIGAVACENABLE 14
#define ENCODERINTERRUPT 13 //interrupt pin from MCP23017 encoder circuit

//MCP23017 CIRCUIT
#define MOTOR1PWM 26
#define MOTOR1IN1 0
#define MOTOR1IN2 1

#define MOTOR2PWM 27
#define MOTOR2IN1 2
#define MOTOR2IN2 3

#define MOTOR3PWM 32
#define MOTOR3IN1 4
#define MOTOR3IN2 5

#define MOTOR4PWM 33
#define MOTOR4IN1 6
#define MOTOR4IN2 7

//DIRECT ESP32 CONTROL
// #define MOTOR1PWM 26
// #define MOTOR1IN1 18 //SCK

// #define MOTOR2PWM 27
// #define MOTOR2IN1 19 //MISO

// #define MOTOR3PWM 32
// #define MOTOR3IN1 23 //MOSI

// #define MOTOR4PWM 33
// #define MOTOR4IN1 25 //CS
