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

//BluetoothSerial //SerialBT;

#define IGNOREDEBUG 0 //must be set to 0 to enable fully working
#define MAXPOWER 0.25 //MAX POWER IN %/100

// I/O expander constructorsl;[]]]]]
Adafruit_MCP23017 motorControl;
Adafruit_MCP23017 encoderControl;

//Motor constructor for the polulu -> direct control VS MCP23017 should be changed thru hardware description
Motor FrontRightMotor = Motor(MOTOR1IN1, MOTOR1PWM); //FR, motor 1 -> polulu
Motor FrontLeftMotor = Motor(MOTOR2IN1, MOTOR2PWM); //FL, motor2 -> polulu
Motor RearLeftMotor = Motor(MOTOR3IN1, MOTOR3PWM); //RL, motor3 -> polulu
Motor RearRightMotor = Motor(MOTOR4IN1, MOTOR4PWM); //RR, motor4 -> polulu

void moveMotorsBasedOnROS();
void testMotorsSeparately();

EncoderANTS FrontRightEncoder = EncoderANTS(0, 1);
EncoderANTS FrontLeftEncoder = EncoderANTS(2, 3);
EncoderANTS RearLeftEncoder = EncoderANTS(4, 5);
EncoderANTS RearRightEncoder = EncoderANTS(6, 7);

//hardware interfaice
TwoWire motorInterface = TwoWire(0);
TwoWire encoderInterface = TwoWire(1);

//WI-FI DEFINITIONS: ============================================================================
#define ESP32
const char* ssid     = "autobot_F07B";
const char* password = "mse2021cap";
// IPAddress ip(192, 168, 1, 3);
IPAddress server(25,2,117,165);
const uint16_t serverPort = 11411;

//MAIN FUNCTION ===============================================================================]
void setup()
{
    Serial.begin(9600);  
    //SerialBT.begin("DCU1");

    pinMode(GIGAVACENABLE, OUTPUT); //gigavac control relay

    //start wi-fi and ROS node
    if (!IGNOREDEBUG) {
        WiFi.begin(ssid, password);
        delay(1000);
        //SerialBT.println("Connecting to WiFi");
        
        while (WiFi.status() != WL_CONNECTED && !IGNOREDEBUG) {
          delay(500);
          Serial.print(".");
          //SerialBT.print(".");
        }

        Serial.println("");
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());

        //SerialBT.println("");
        //SerialBT.println("WiFi connected");
        //SerialBT.println("IP address: ");
        //SerialBT.println(WiFi.localIP());

        // Set the connection to rosserial socket server
        DCU1.getHardware()->setConnection(server, serverPort);
        DCU1.initNode();

        // Another way to get IP
        Serial.print("IP = ");
        //SerialBT.print("IP = ");
        Serial.println(DCU1.getHardware()->getLocalIP());

        //motor subs -> read DCU power from ROS and apply to motors
        DCU1.subscribe(FrontRightSpeed); //motor 1
        DCU1.subscribe(FrontLeftSpeed); //motor 2 -> included in motor 1
        DCU1.subscribe(RearLeftSpeed); //motor 3 -> included in motor 4
        DCU1.subscribe(RearRightSpeed); //motor 4

        // //motor publishing -> read encoders on DCU side and publish them to ROS


        // //contactor ROS -> subscribe to unlock, publish to let PC know gigavac is engaged
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

    //direct control
    // FrontRightMotor.begin(); //motor 1
    // FrontLeftMotor.begin(); //motor 2 -> included in motor 1
    // RearLeftMotor.begin(); //motor 3 -> included in motor 4
    // RearRightMotor.begin(); //motor 4
}

// LOOP FUNCTION ====================================================================================
void loop()
{
    //first of all check DCU connection to ROS -> do not start program if no ROS node
    if(millis() - last_time >= period && !IGNOREDEBUG)
    {
        last_time = millis();
        if (DCU1.connected()) {
            Serial.println("Connected");
            //SerialBT.println("Connected");
            
            //run motors based on ROS -> single DCU 4ch operation
            Serial.print("Motor 1 speed: "); Serial.println(FrontRightMotor1speed);
            Serial.print("Motor 2 speed: "); Serial.println(FrontLeftMotor2speed);
            Serial.print("Motor 3 speed: "); Serial.println(RearLeftMotor3speed);
            Serial.print("Motor 4 speed: "); Serial.println(RearRightMotor4speed);
            moveMotorsBasedOnROS(); 
        } else {
            Serial.println("Not Connected");
            //SerialBT.println("Not Connected");
        }
    } 

    //DEBUG ONLY
    //testMotorsSeparately();

    DCU1.spinOnce();
    delay(1);
}

/*
  ROS FUNCTIONS BELOW -> DO NOT TOUCH UNLESS NOT WORKING
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////
// ADDITIONAL FUNCTIONS ================================================================================
void FrontRightROS(const std_msgs::Float32& msg1) { //motor 1 data from ROS to motor control
    FrontRightMotor1speed = (-1)*255*MAXPOWER*(msg1.data); //-1 is required because of FET polarity VS BJS polarity
}

void FrontLeftROS(const std_msgs::Float32& msg2) { //motor 2 data from ROS to motor control
    FrontLeftMotor2speed = (-1)*255*MAXPOWER*(msg2.data);
}
void RearLeftROS(const std_msgs::Float32& msg3) { //motor 3 data from ROS to motor control
    RearLeftMotor3speed = (-1)*255*MAXPOWER*(msg3.data);
} 

void RearRightROS(const std_msgs::Float32& msg4) { //motor 4 data from ROS to motor control
    RearRightMotor4speed = (-1)*255*MAXPOWER*(msg4.data);
}

void moveMotorsBasedOnROS() {
  //make sure to stop motors if there is 0 velocity command from ROS
  if (FrontRightMotor1speed == 0) { 
    FrontRightMotor.stop(&motorControl);
  } else {
    FrontRightMotor.go(&motorControl, FrontRightMotor1speed);
  }

  if (FrontLeftMotor2speed == 0) {
    FrontLeftMotor.stop(&motorControl);
  } else {
    FrontLeftMotor.go(&motorControl, FrontLeftMotor2speed);
  }

  if (RearLeftMotor3speed == 0) {
    RearLeftMotor.stop(&motorControl);
  } else {
    RearLeftMotor.go(&motorControl, RearLeftMotor3speed);
  }

  if (RearRightMotor4speed == 0) { 
    RearRightMotor.stop(&motorControl);
  } else {
    RearRightMotor.go(&motorControl, RearRightMotor4speed);
  }
}

void unlockPowerToMotors(const std_msgs::Int16& msg5) {
  //unlock power to motors based on ROS command
  int16_t contactorEnabled = msg5.data;

  if(contactorEnabled == 0) { //turn off GIGAVAC
    digitalWrite(GIGAVACENABLE, LOW); //LOW turns it off
  }

  else if (contactorEnabled == 1) { //turn on GIGAVAC
    digitalWrite(GIGAVACENABLE, HIGH); //HIGH turns it on
  }
}

// void testTopicSub(const std_msgs::String& msg6) {
//     Serial.println(msg6.data);
// }

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