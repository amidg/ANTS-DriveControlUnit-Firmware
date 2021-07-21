#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_MCP23017.h"
#include "analogWrite.h"
#include "Motor.h"
#include <Rotary.h>
#include <RotaryEncOverMCP.h>
#include "EncoderANTS.h"
#include "ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"

#define IGRNOREDEBUG 1 //must be set to 0 to enable fully working 

//MOTOR CONTROL
Adafruit_MCP23017 motorControl;

#define GIGAVACENABLE 14

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

//assumed direction when motherboard ethernet side facing rear of the robot
Motor FrontRightMotor = Motor(MOTOR1IN1, MOTOR1IN2, MOTOR1PWM); //FR, motor 1
Motor FrontLeftMotor = Motor(MOTOR2IN1, MOTOR2IN2, MOTOR2PWM); //FL, motor 2
Motor RearLeftMotor = Motor(MOTOR3IN1, MOTOR3IN2, MOTOR3PWM); //RL, motor 3
Motor RearRightMotor = Motor(MOTOR4IN1, MOTOR4IN2, MOTOR4PWM); //RR, motor 4

void moveMotorsBasedOnROS();
void moveDualDCUmotorsBasedOnROS();

//ENCODER CONTROL =============================================================================
TaskHandle_t encoderCalculator;
#define ENCODERINTERRUPT 13 //interrupt pin from MCP23017 encoder circuit
Adafruit_MCP23017 encoderControl;
//void RotaryEncoderChanged(bool clockwise, int id); //callback function
int encoderValue[4]; //all motor encoders
void encoderHandler(); //interrupt function that
void calculateEncoders(void * pvParameters);
bool isInterruptEnabledonEncoder;

EncoderANTS FrontRightEncoder = EncoderANTS(0, 1);
EncoderANTS FrontLeftEncoder = EncoderANTS(2, 3);
EncoderANTS RearLeftEncoder = EncoderANTS(4, 5);
EncoderANTS RearRightEncoder = EncoderANTS(6, 7);

TwoWire motorInterface = TwoWire(0);
TwoWire encoderInterface = TwoWire(1);

//WI-FI DEFINITIONS: ============================================================================
#define ESP32
const char* ssid     = "AutoBot1_2G";
const char* password = "mse2021cap";
IPAddress ip(192, 168, 1, 3);
IPAddress server(192,168,100,100);
const uint16_t serverPort = 11411;


//ROS DEFINITIONS =============================================================================
ros::NodeHandle DCU1;
// Make a chatter publisher
std_msgs::String str_msg;
ros::Publisher chatter("dcu_test", &str_msg);

// Be polite and say hello
char hello[13] = "DCU test";
uint16_t period = 20;
uint32_t last_time = 0;

//ROS motor control
void FrontRightROS(const std_msgs::Int16& msg1); //motor 1
void FrontLeftROS(const std_msgs::Int16& msg2); //motor 2
void RearLeftROS(const std_msgs::Int16& msg3); //motor 3
void RearRightROS(const std_msgs::Int16& msg4); //motor 4
void unlockPowerToMotors(const std_msgs::Int16& msg5); //GIGAVAC contactor

int FrontRightMotor1speed;
int FrontLeftMotor2speed;
int RearLeftMotor3speed;
int RearRightMotor4speed; 

ros::Subscriber<std_msgs::Int16> FrontRightSpeed("/dcu1/motor1/cmd", FrontRightROS); //Front Right wheel, motor 1
ros::Subscriber<std_msgs::Int16> FrontLeftSpeed("/dcu1/motor2/cmd", FrontLeftROS); //Front Left Wheel, motor 2
ros::Subscriber<std_msgs::Int16> RearLeftSpeed("/dcu1/motor3/cmd", RearLeftROS); //Rear Left wheel, motor 3
ros::Subscriber<std_msgs::Int16> RearRightSpeed("/dcu1/motor4/cmd", RearRightROS); //Rear Right Wheel, motor 4

//contactor power
ros::Subscriber<std_msgs::Int16> PowerLock("/dcu1/contactor", unlockPowerToMotors);
std_msgs::String powerLocker_msg;
ros::Publisher motor_power("gigavac_feedback", &powerLocker_msg);

//ROS Encoder Control:
std_msgs::Int32 FrontRightEncMsg; //Front Right, motor 1 encoder message to ROS
ros::Publisher FrontRightEncPublish("/dcu1/motor1/enc", &FrontRightEncMsg);

std_msgs::Int32 FrontLeftEncMsg; //Front Left, motor 2 encoder message to ROS
ros::Publisher FrontLeftEncPublish("/dcu1/motor2/enc", &FrontLeftEncMsg);

std_msgs::Int32 RearLeftEncMsg; //Rear Left, motor 3 encoder message to ROS
ros::Publisher RearLeftEncPublish("/dcu1/motor3/enc", &RearLeftEncMsg);

std_msgs::Int32 RearRightEncMsg; //Rear Right, motor 4 encoder message to ROS
ros::Publisher RearRightEncPublish("/dcu1/motor4/enc", &RearRightEncMsg);

//MAIN FUNCTION ===============================================================================
void setup()
{
  Serial.begin(9600);  

  motorInterface.begin(21, 22, 100000);
  encoderInterface.begin(21, 22, 100000);

  //MOTOR CONTROL RUNS ON CORE 1 (MAIN)
  motorControl.begin(0, &motorInterface); //specified custom address

  FrontRightMotor.begin(&motorControl); //motor 1
  FrontLeftMotor.begin(&motorControl); //motor 2
  RearLeftMotor.begin(&motorControl); //motor 3
  RearRightMotor.begin(&motorControl); //motor 4

  pinMode(GIGAVACENABLE, OUTPUT); //gigavac control relay

  //ENCODER CONTROL RUNS ON CORE 0 (ADDITIONAL)
  encoderControl.begin(2, &encoderInterface); //specified custom address for encoders
  pinMode(ENCODERINTERRUPT, INPUT); //encoder interrupt pin

  FrontRightEncoder.begin(&encoderControl);
  FrontLeftEncoder.begin(&encoderControl);
  RearLeftEncoder.begin(&encoderControl);
  RearRightEncoder.begin(&encoderControl);

  xTaskCreatePinnedToCore(
                    calculateEncoders,          // Task function.
                    "Calculate Encoder values", // name of task.
                    100000,                      // Stack size of task
                    NULL,                       // parameter of the task
                    0,                          // priority of the task
                    &encoderCalculator,         // Task handle to keep track of created task 
                    0);                         // pin task to core 0

  delay(500); 

  //Setup interrupts, OR INTA, INTB together on both ports.
  //thus we will receive an interrupt if something happened on
  //port A or B with only a single INT connection.
  attachInterrupt(digitalPinToInterrupt(ENCODERINTERRUPT), encoderHandler, FALLING); //configure interrupt

  //ROS PART --------------------------------------------------------------------------------------------------
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
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
  DCU1.subscribe(FrontRightSpeed); //motor 1
  DCU1.subscribe(FrontLeftSpeed); //motor 2
  DCU1.subscribe(RearLeftSpeed); //motor 3
  DCU1.subscribe(RearRightSpeed); //motor 4

  //motor publishing -> read encoders on DCU side and publish them to ROS

  //contactor ROS -> subscribe to unlock, publish to let PC know gigavac is engaged
  DCU1.subscribe(PowerLock); //contactor subscriber
  DCU1.advertise(motor_power); //contactor feedback publisher


  //Start the ROS node on DCUx
  DCU1.advertise(chatter);
}

// LOOP FUNCTION ====================================================================================
void loop()
{
  //first of all check DCU connection to ROS -> do not start program if no ROS node
  while (!DCU1.connected() && !IGNOREDEBUG) {
    Serial.println("ERROR: NO ROS CONNECTION");
  }

  // topic publishers
  motor_power.publish( &powerLocker_msg ); //power locker, str_msg.data is generated based on the input from the ROS power locker 

  //enable GIGAVAC
  unlockPowerToMotors(const std_msgs::Int16& msg5);

  //run motors based on ROS -> single DCU 4ch operation
  moveMotorsBasedOnROS(); 

  //run motors based on ROS -> dual DCU 4to2ch operation
  //moveDualDCUmotorsBasedOnROS();

  // //TEST MOTOR
  // FrontRightMotor.go(&motorControl, 100);
  // Serial.println(FrontRightEncoder.readCurrentPosition());

  // FrontRightMotor.stop(&motorControl); //go full stop
  // Serial.println(FrontRightEncoder.readCurrentPosition());
  // delay(2000);

  // FrontRightMotor.go(&motorControl, -100);
  // Serial.println(FrontRightEncoder.readCurrentPosition());

  // FrontRightMotor.stop(&motorControl);
  // Serial.println(FrontRightEncoder.readCurrentPosition());
  // delay(2000);

  DCU1.spinOnce();
  delay(1);
}

// INTERRUPT FUNCTION ==================================================================================
void IRAM_ATTR encoderHandler() {
  /*
      Interrupt Service routine disables timing:
      - no serial
      - no internal i2c

      also do not use GIGAVAC pin because it causes relay knocking 
  */
  detachInterrupt(digitalPinToInterrupt(ENCODERINTERRUPT));
  isInterruptEnabledonEncoder = 1;
  attachInterrupt(digitalPinToInterrupt(ENCODERINTERRUPT), encoderHandler, FALLING);
}

// 2ND CORE TASK ================================================================================
void calculateEncoders(void * pvParameters) {
  while(1) {
    vTaskDelay(1);
    FrontRightEncoder.getFeedback(&encoderControl);
    FrontLeftEncoder.getFeedback(&encoderControl);
    RearLeftEncoder.getFeedback(&encoderControl);
    RearRightEncoder.getFeedback(&encoderControl);
  }
}

// ADDITIONAL FUNCTIONS ================================================================================
void FrontRightROS(const std_msgs::Int16& msg1) { //motor 1 data from ROS to motor control
  FrontRightMotor1speed = msg1.data;
}

void FrontLeftROS(const std_msgs::Int16& msg2) { //motor 2 data from ROS to motor control
  FrontLeftMotor2speed = msg2.data;
}
void RearLeftROS(const std_msgs::Int16& msg3) { //motor 3 data from ROS to motor control
  RearLeftMotor3speed = msg3.data;
} 

void RearRightROS(const std_msgs::Int16& msg4) { //motor 4 data from ROS to motor control
  RearRightMotor4speed = msg4.data;
}

void moveMotorsBasedOnROS() {
  //make sure to stop motors if there is 0 velocity command from ROS
  digitalWrite(GIGAVACENABLE, HIGH);

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
  if(msg5.data == 0) {
    digitalWrite(GIGAVACENABLE, HIGH); //HIGH turns it off
    powerLocker_msg.data = "POWER IS OFF";
  }

  else if (msg5.data == 1) {
    digitalWrite(GIGAVACENABLE, LOW); //LOW turns it on
    powerLocker_msg.data = "POWER IS ON";
  }
}