#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_MCP23017.h"
#include "analogWrite.h"
#include "Motor.h"
#include <Rotary.h>
#include <RotaryEncOverMCP.h>

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

//MOTOR CONTROL
Adafruit_MCP23017 motorControl;

//assumed direction when motherboard ethernet side facing rear of the robot
Motor FrontRightMotor = Motor(MOTOR1IN1, MOTOR1IN2, MOTOR1PWM); //FR, motor 1
Motor FrontLeftMotor = Motor(MOTOR2IN1, MOTOR2IN2, MOTOR2PWM); //FL, motor 2
Motor RearLeftMotor = Motor(MOTOR3IN1, MOTOR3IN2, MOTOR3PWM); //RL, motor 3
Motor RearRightMotor = Motor(MOTOR4IN1, MOTOR4IN2, MOTOR4PWM); //RR, motor 4

//ENCODER CONTROL
#define ENCODERINTERRUPT 13 //interrupt pin from MCP23017 encoder circuit
Adafruit_MCP23017 encoderControl;
//void RotaryEncoderChanged(bool clockwise, int id); //callback function
int encoderValue[4]; //all motor encoders
int encoder1Alast = LOW;
void encoderHandler(); //interrupt function that
void calculateEncoders();
bool isInterruptEnabledonEncoder = 0;

/* Array of all rotary encoders and their pins */
// RotaryEncOverMCP FrontRightEncoder = RotaryEncOverMCP(&encoderControl, 0, 1, &RotaryEncoderChanged, 1); //motor 1 encoder
// RotaryEncOverMCP FrontLeftEncoder = RotaryEncOverMCP(&encoderControl, 2, 3, &RotaryEncoderChanged, 2); //motor 2 encoder
// RotaryEncOverMCP RearLeftEncoder = RotaryEncOverMCP(&encoderControl, 4, 5, &RotaryEncoderChanged, 3); //motor 3 encoder
// RotaryEncOverMCP RearRightEncoder = RotaryEncOverMCP(&encoderControl, 6, 7, &RotaryEncoderChanged, 4); //motor 4 encoder

// RotaryEncOverMCP encoders[] = {
//   FrontRightEncoder, FrontLeftEncoder, RearLeftEncoder, RearRightEncoder
// }; 

//MAIN FUNCTION
void setup()
{
  Serial.begin (9600);  
  Wire.begin();

  //motor control
  motorControl.begin(0, &Wire); //specified custom address

  //enable pins motor
  FrontRightMotor.begin(&motorControl); //motor 1
  FrontLeftMotor.begin(&motorControl); //motor 2
  RearRightMotor.begin(&motorControl); //motor 4
  RearLeftMotor.begin(&motorControl); //motor 3

  pinMode(GIGAVACENABLE, OUTPUT); //gigavac control relay

  //encoder control
  encoderControl.begin(2, &Wire); //specified custom address for encoders
  pinMode(ENCODERINTERRUPT, INPUT); //encoder interrupt pin

  //Setup interrupts, OR INTA, INTB together on both ports.
  //thus we will receive an interrupt if something happened on
  //port A or B with only a single INT connection.
  encoderControl.setupInterrupts(true,false,LOW);
  encoderControl.pinMode(0, INPUT);
  encoderControl.pullUp(0, 0);
  encoderControl.setupInterruptPin(0, CHANGE);
  encoderControl.pinMode(1, INPUT);
  encoderControl.pullUp(1, 0);
  encoderControl.setupInterruptPin(1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODERINTERRUPT), encoderHandler, FALLING); //configure interrupt

  //initialize encoders
  //FrontRightEncoder.init();
}

void loop()
{
  digitalWrite(GIGAVACENABLE, HIGH);

  //TEST MOTOR
  //for (int i = 0; i < 255; i = i + 5) { //go forward
    FrontRightMotor.go(&motorControl, 100);
    //delay(100);
    //calculateEncoders();
  //} 

  // FrontRightMotor.stop(&motorControl); //go full stop
  // delay(10000);

  // for (int i = 0; i > -255; i--) { //go reverse
  //   FrontRightMotor.go(&motorControl, i);
  //   delay(100);
  // }

  // FrontRightMotor.stop(&motorControl);
  // delay(10000);
}


//encoder callback function
// void RotaryEncoderChanged(bool clockwise, int id) {
//     Serial.println("Encoder " + String(id) + ": " + (clockwise ? String("clockwise") : String("counter-clock-wise")));
// }

void IRAM_ATTR encoderHandler() {
  /*
      Interrupt Service routine disables timing:
      - no serial
      - no internla i2c

      also do not use GIGAVAC pin because it causes relay knocking 
  */
  detachInterrupt(digitalPinToInterrupt(ENCODERINTERRUPT));
  isInterruptEnabledonEncoder = 1;
  attachInterrupt(digitalPinToInterrupt(ENCODERINTERRUPT), encoderHandler, FALLING);
}

void calculateEncoders() {
  //if (isInterruptEnabledonEncoder) {
    Serial.println(isInterruptEnabledonEncoder);
    if ( (encoder1Alast == LOW ) && (encoderControl.digitalRead(0) == HIGH) ) {
      if (encoderControl.digitalRead(1) == LOW)
        encoderValue[1] = encoderValue[1] - 1;
      else 
        encoderValue[1] = encoderValue[1] + 1;
    }
    Serial.println(encoderValue[1]);
    isInterruptEnabledonEncoder = 0;
  //}
}