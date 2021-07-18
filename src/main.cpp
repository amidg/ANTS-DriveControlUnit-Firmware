#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_MCP23017.h" //please run pio lib install "adafruit/Adafruit BusIO@^1.8.2" before proceeding
#include "analogWrite.h"
#include "Motor.h"

//pin definitions:
#define GIGAVACENABLE 14

//motor definitions
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

//assumed direction when motherboard ethernet side facing rear of the robot
Motor FrontLeftMotor = Motor(MOTOR2IN1, MOTOR2IN2, MOTOR2PWM); //FL, motor 2
Motor FrontRightMotor = Motor(MOTOR1IN1, MOTOR1IN2, MOTOR1PWM); //FR, motor 1
Motor RearLeftMotor = Motor(MOTOR3IN1, MOTOR3IN2, MOTOR3PWM); //RL, motor 3
Motor RearRightMotor = Motor(MOTOR4IN1, MOTOR4IN2, MOTOR4PWM); //RR, motor 4

//supporting functions
void motorGo(Motor *motorToRun, int PWMvalue);

void setup()
{
  Serial.begin (9600);  
  Wire.begin();
  motorControl.begin(0, &Wire); //specified custom address

  //enable pins motor
  // motorControl.pinMode(MOTOR1IN1, OUTPUT);
  // motorControl.pinMode(MOTOR1IN2, OUTPUT);
  // motorControl.pinMode(MOTOR2IN1, OUTPUT);
  // motorControl.pinMode(MOTOR2IN2, OUTPUT);
  // motorControl.pinMode(MOTOR3IN1, OUTPUT);
  // motorControl.pinMode(MOTOR3IN2, OUTPUT);
  // motorControl.pinMode(MOTOR4IN1, OUTPUT);
  // motorControl.pinMode(MOTOR4IN2, OUTPUT);
  FrontRightMotor.begin(&motorControl); //motor 1
  FrontLeftMotor.begin(&motorControl); //motor 2
  RearRightMotor.begin(&motorControl); //motor 4
  RearLeftMotor.begin(&motorControl); //motor 3

  pinMode(GIGAVACENABLE, OUTPUT);
}

void loop()
{
  digitalWrite(GIGAVACENABLE, HIGH);

  //TEST MOTOR 
  FrontRightMotor.go(&motorControl, 255);
}

//functions
void motorGo(Motor *motorToRun, int PWMvalue) {
  
  int power = abs(PWMvalue);
  //controls side
  if(PWMvalue >= 0) {
    //if positive, go forward
    motorControl.digitalWrite(motorToRun->IN1, HIGH);
    motorControl.digitalWrite(motorToRun->IN2, LOW);
  } else if (PWMvalue < 0) {
    //if negative, go backwards
    motorControl.digitalWrite(motorToRun->IN1, LOW);
    motorControl.digitalWrite(motorToRun->IN2, HIGH);
  }

  //motorToRun->go(power);
}