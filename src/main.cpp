#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_MCP23017.h"
#include "analogWrite.h"
#include "Motor.h"
#include <Rotary.h>
#include <RotaryEncOverMCP.h>
#include "EncoderANTS.h"
#include "ANTS_ROS.h"
#include "ANTShardwareDescription.h"

#define IGNOREDEBUG 0 //must be set to 0 to enable fully working

//assumed direction when motherboard ethernet side facing rear of the robot
Motor FrontRightMotor = Motor(MOTOR1IN1, MOTOR1PWM); //FR, motor 1 -> use Motor constructor for polulu
Motor FrontLeftMotor = Motor(MOTOR2IN1, MOTOR2PWM); //FL, motor2 -> polulu
Motor RearLeftMotor = Motor(MOTOR3IN1, MOTOR3PWM); //RL, motor3 -> polulu
Motor RearRightMotor = Motor(MOTOR4IN1, MOTOR4PWM); //RR, motor4 -> polulu

// //hardware interfaice
// TwoWire motorInterface = TwoWire(0);
// TwoWire encoderInterface = TwoWire(1);

//WI-FI DEFINITIONS: ============================================================================
#define ESP32
const char* ssid     = "autobot_F07B";
const char* password = "mse2021cap";
// IPAddress ip(192, 168, 1, 3);
IPAddress server(25,2,117,165);
const uint16_t serverPort = 11411;

//MAIN FUNCTION ===============================================================================
void setup()
{
    Serial.begin(9600);  

    pinMode(GIGAVACENABLE, OUTPUT); //gigavac control relay

    //start wi-fi and ROS node
    if (!IGNOREDEBUG) {
        WiFi.begin(ssid, password);

        while (WiFi.status() != WL_CONNECTED && !IGNOREDEBUG) {
        delay(500);
        Serial.print(".");
        }

        Serial.println("");
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());

        // Set the connection to rosserial socket server
        DCU1.getHardware()->setConnection(server, serverPort);
        DCU1.initNode();

        // Another way to get IP
        Serial.print("IP = ");
        Serial.println(DCU1.getHardware()->getLocalIP());

        //motor subs -> read DCU power from ROS and apply to motors
        // DCU1.subscribe(FrontRightSpeed); //motor 1
        // DCU1.subscribe(FrontLeftSpeed); //motor 2 -> included in motor 1
        // DCU1.subscribe(RearLeftSpeed); //motor 3 -> included in motor 4
        // DCU1.subscribe(RearRightSpeed); //motor 4

        // //motor publishing -> read encoders on DCU side and publish them to ROS

        // //contactor ROS -> subscribe to unlock, publish to let PC know gigavac is engaged
        DCU1.subscribe(PowerLock); //contactor subscriber
        //DCU1.subscribe(ChatterSubTest);
        // DCU1.advertise(motor_power); //contactor feedback publisher
    }


    //start hardware interface for motor and encoders
    //motorInterface.begin(21, 22);
    //encoderInterface.begin(16,17);

    //MOTOR CONTROL RUNS ON CORE 1 (MAIN)
    //motorControl.begin(0, &motorInterface); //specified custom address

    // FrontRightMotor.begin(&motorControl); //motor 1
    // FrontLeftMotor.begin(&motorControl); //motor 2 -> included in motor 1
    // RearLeftMotor.begin(&motorControl); //motor 3 -> included in motor 4
    // RearRightMotor.begin(&motorControl); //motor 4

    // FrontRightMotor.beginNoMCP(); //motor 1
    // FrontLeftMotor.beginNoMCP(); //motor 2 -> included in motor 1
    // RearLeftMotor.beginNoMCP(); //motor 3 -> included in motor 4
    // RearRightMotor.beginNoMCP(); //motor 4

}

// LOOP FUNCTION ====================================================================================
void loop()
{
    //first of all check DCU connection to ROS -> do not start program if no ROS node
    // while (!DCU1.connected() && !IGNOREDEBUG) {
    //     Serial.println("ERROR: NO ROS CONNECTION");
    // }

    if(millis() - last_time >= period)
    {
        last_time = millis();
        if (DCU1.connected()) {
            Serial.println("Connected");
            // Say hello
            // str_msg.data = hello;
            // chatter.publish( &str_msg );
        } else {
            Serial.println("Not Connected");
        }
    }

    //enable GIGAVAC
    //motor_power.publish( &powerLocker_msg ); //power locker, str_msg.data is generated based on the input from the ROS power locker 

    //run motors based on ROS -> single DCU 4ch operation
    //moveMotorsBasedOnROS(); 

    DCU1.spinOnce();
    delay(1);
}

/*
  ROS FUNCTIONS BELOW -> DO NOT TOUCH UNLESS NOT WORKING
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////
// ADDITIONAL FUNCTIONS ================================================================================
// void FrontRightROS(const std_msgs::Int16& msg1) { //motor 1 data from ROS to motor control
//   FrontRightMotor1speed = msg1.data;
// }

// void FrontLeftROS(const std_msgs::Int16& msg2) { //motor 2 data from ROS to motor control
//   FrontLeftMotor2speed = msg2.data;
// }
// void RearLeftROS(const std_msgs::Int16& msg3) { //motor 3 data from ROS to motor control
//   RearLeftMotor3speed = msg3.data;
// } 

// void RearRightROS(const std_msgs::Int16& msg4) { //motor 4 data from ROS to motor control
//   RearRightMotor4speed = msg4.data;
// }

// void moveMotorsBasedOnROS() {
//   //make sure to stop motors if there is 0 velocity command from ROS
//   digitalWrite(GIGAVACENABLE, HIGH);

//   if (FrontRightMotor1speed == 0) { // --> IGNORE IN DCU2
//     //FrontRightMotor.stop(&motorControl);
//     FrontRightMotor.go(0);
//   } else {
//     FrontRightMotor.go(FrontRightMotor1speed);
//     //FrontRightMotor.go(&motorControl, FrontRightMotor1speed);
//   }

//   if (FrontLeftMotor2speed == 0) { // --> IGNORE IN DCU1
//     //FrontLeftMotor.stop(&motorControl);
//     FrontLeftMotor.go(0);
//   } else {
//     FrontLeftMotor.go(FrontLeftMotor2speed);
//     //FrontLeftMotor.go(&motorControl, FrontLeftMotor2speed);
//   }

//   if (RearLeftMotor3speed == 0) {
//     //RearLeftMotor.stop(&motorControl);
//     RearLeftMotor.go(0);
//   } else {
//     RearLeftMotor.go(RearLeftMotor3speed);
//     //RearLeftMotor.go(&motorControl, RearLeftMotor3speed);
//   }

//   if (RearRightMotor4speed == 0) { //--> IGNORE IN DCU2
//     //RearRightMotor.stop(&motorControl);
//     RearRightMotor.go(0);
//   } else {
//     RearRightMotor.go(RearRightMotor4speed);
//     //RearRightMotor.go(&motorControl, RearRightMotor4speed);
//   }
// }

void unlockPowerToMotors(const std_msgs::Int16& msg5) {
  //unlock power to motors based on ROS command
  int16_t contactorEnabled = msg5.data;

  if(contactorEnabled == 0) { //turn off GIGAVAC
    digitalWrite(GIGAVACENABLE, LOW); //HIGH turns it off
  }

  else if (contactorEnabled == 1) { //turn on GIGAVAC
    digitalWrite(GIGAVACENABLE, HIGH); //LOW turns it on
  }
}

// void testTopicSub(const std_msgs::String& msg6) {
//     Serial.println(msg6.data);
// }