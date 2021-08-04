#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_MCP23017.h"
#include "analogWrite.h"
#include "Motor.h"
#include "EncoderANTS.h"
#include "ANTS_ROS.h"
#include "ANTShardwareDescription.h"
#include "BluetoothSerial.h" //adds Bluetooth support to existing ESP32

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define IGNOREDEBUG 0 //must be set to 0 to enable fully working
#define USEBLUETOOTH 1 //must be 1 to use Blueooth for debugging
#define MAXPOWER 0.2 //MAX POWER IN %/100

BluetoothSerial SerialBT;
TaskHandle_t BluetoothDataTransfer;
void BluetoothROS(void * parameter);

// I/O expander constructorsl;
Adafruit_MCP23017 motorControl;
Adafruit_MCP23017 encoderControl;

//Motor constructor for the polulu -> direct control VS MCP23017 should be changed thru hardware description
Motor FrontRightMotor = Motor(MOTOR1IN1, MOTOR1PWM); //FR, motor 1 -> polulu
Motor FrontLeftMotor = Motor(MOTOR2IN1, MOTOR2PWM); //FL, motor2 -> polulu
Motor RearLeftMotor = Motor(MOTOR3IN1, MOTOR3PWM); //RL, motor3 -> polulu
Motor RearRightMotor = Motor(MOTOR4IN1, MOTOR4PWM); //RR, motor4 -> polulu

float Motor1DataFromROS;
float Motor2DataFromROS;
float Motor3DataFromROS;
float Motor4DataFromROS;
int16_t contactorEnabled;

long TimeSinceStart;

void testMotorsSeparately();

EncoderANTS FrontRightEncoder = EncoderANTS(0, 1);
EncoderANTS FrontLeftEncoder = EncoderANTS(2, 3);
EncoderANTS RearLeftEncoder = EncoderANTS(4, 5);
EncoderANTS RearRightEncoder = EncoderANTS(6, 7);

//hardware interfaice
TwoWire motorInterface = TwoWire(0);
TwoWire encoderInterface = TwoWire(1);

//MAIN FUNCTION ===============================================================================]
void setup()
{
    if (USEBLUETOOTH) { 
      SerialBT.begin("ANTS_DCU"); 

      xTaskCreatePinnedToCore(
        BluetoothROS,              /* Function to implement the task */
        "Transfer ROS data over Bluetooth", /* Name of the task */
        10000,                              /* Stack size in words */
        NULL,                               /* Task input parameter */
        0,                                  /* Priority of the task */
        &BluetoothDataTransfer,                             /* Task handle. */
        0);                                 /* Core where the task should run */

      if (SerialBT.available()) {
        SerialBT.println("Connected");
      }
    }

    pinMode(GIGAVACENABLE, OUTPUT); //gigavac control relay

    //start wi-fi and ROS node
    if (!IGNOREDEBUG) {
      DCU1.initNode();
      SerialBT.println("Initializing ROS topics");

      //motor subs -> read DCU power from ROS and apply to motors
      DCU1.subscribe(FrontRightSpeed); //motor 1
      DCU1.subscribe(FrontLeftSpeed); //motor 2 -> included in motor 1
      DCU1.subscribe(RearLeftSpeed); //motor 3 -> included in motor 4
      DCU1.subscribe(RearRightSpeed); //motor 4

      //motor publishing -> read encoders on DCU side and publish them to ROS


      //contactor ROS -> subscribe to unlock, publish to let PC know gigavac is engaged
      DCU1.subscribe(PowerLock); //contactor subscriber
    }


    //start hardware interface for motor and encoders
    motorInterface.begin(21, 22);
    encoderInterface.begin(16,17);

    //MOTOR CONTROL RUNS ON CORE 1 (MAIN)
    motorControl.begin(0, &motorInterface); //specified custom address

    FrontRightMotor.begin(&motorControl); //motor 1
    FrontLeftMotor.begin(&motorControl); //motor 2 -> included in motor 1
    RearLeftMotor.begin(&motorControl); //motor 3 -> included in motor 4
    RearRightMotor.begin(&motorControl); //motor 4
}

// LOOP FUNCTION ====================================================================================
void loop()
{
  if (!IGNOREDEBUG) {
    while(true) {
      DCU1.spinOnce();
      delayMicroseconds(20);
    }
  } else if (IGNOREDEBUG) {
    testMotorsSeparately();
  }
}

/*
  ROS FUNCTIONS BELOW -> DO NOT TOUCH UNLESS NOT WORKING
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////
// ADDITIONAL FUNCTIONS ================================================================================
void FrontRightROS(const std_msgs::Float32& msg1) { //motor 1 data from ROS to motor control
  Motor1DataFromROS = msg1.data;
  FrontRightMotor1speed = (-1)*255*MAXPOWER*(Motor1DataFromROS); //-1 is required because of FET polarity VS BJS polarity
  FrontRightMotor.go(&motorControl, FrontRightMotor1speed);
}

void FrontLeftROS(const std_msgs::Float32& msg2) { //motor 2 data from ROS to motor control
  Motor2DataFromROS = msg2.data;
  FrontLeftMotor2speed = (-1)*255*MAXPOWER*(Motor2DataFromROS);
  FrontLeftMotor.go(&motorControl, FrontLeftMotor2speed);
}
void RearLeftROS(const std_msgs::Float32& msg3) { //motor 3 data from ROS to motor control
  Motor3DataFromROS = msg3.data;
  RearLeftMotor3speed = (-1)*255*MAXPOWER*(Motor3DataFromROS);
  RearLeftMotor.go(&motorControl, RearLeftMotor3speed);
} 

void RearRightROS(const std_msgs::Float32& msg4) { //motor 4 data from ROS to motor control
  Motor4DataFromROS = msg4.data;
  RearRightMotor4speed = (-1)*255*MAXPOWER*(Motor4DataFromROS);
  RearRightMotor.go(&motorControl, RearRightMotor4speed);
}

void unlockPowerToMotors(const std_msgs::Int16& msg5) {
  //unlock power to motors based on ROS command
  contactorEnabled = msg5.data;

  if(contactorEnabled == 0) { //turn off GIGAVAC
    digitalWrite(GIGAVACENABLE, LOW); //LOW turns it off
  }

  else if (contactorEnabled == 1) { //turn on GIGAVAC
    digitalWrite(GIGAVACENABLE, HIGH); //HIGH turns it on
  }
}

void BluetoothROS(void * parameter) {
  while(1) {
    if (USEBLUETOOTH) { 
      //publish bluetooth
      SerialBT.print("ROS node status: "); SerialBT.println(DCU1.connected());
      SerialBT.print("Contactor Status: "); SerialBT.println(contactorEnabled);
      SerialBT.print("Motor 1 speed ROS/PWM: "); SerialBT.print(Motor1DataFromROS); SerialBT.print("/"); SerialBT.println(FrontRightMotor1speed);
      SerialBT.print("Motor 2 speed ROS/PWM: "); SerialBT.print(Motor2DataFromROS); SerialBT.print("/"); SerialBT.println(FrontLeftMotor2speed);
      SerialBT.print("Motor 3 speed ROS/PWM: "); SerialBT.print(Motor3DataFromROS); SerialBT.print("/"); SerialBT.println(RearLeftMotor3speed);
      SerialBT.print("Motor 4 speed ROS/PWM: "); SerialBT.print(Motor4DataFromROS); SerialBT.print("/"); SerialBT.println(RearRightMotor4speed);
      // SerialBT.println("------------------------------------");
      SerialBT.flush();
      vTaskDelay(10);
    }

    //place encoder stuff here
    
    
  }
}

void testMotorsSeparately() {
  FrontRightMotor.go(&motorControl, 0.2*255);
  delay(2000);
  FrontRightMotor.stop(&motorControl);

  FrontLeftMotor.go(&motorControl, 0.2*255);
  delay(2000);
  FrontLeftMotor.stop(&motorControl);

  RearLeftMotor.go(&motorControl, 0.2*255);
  delay(2000);
  RearLeftMotor.stop(&motorControl);

  RearRightMotor.go(&motorControl, 0.2*255);
  delay(2000);
  RearRightMotor.stop(&motorControl);
}