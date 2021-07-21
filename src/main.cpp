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

//ROS DEFINITIONS: ============================================================================
#define ESP32
const char* ssid     = "AutoBot1_2G";
const char* password = "mse2021cap";
IPAddress ip(192, 168, 1, 3);
IPAddress server(192,168,100,100);
const uint16_t serverPort = 11411;
ros::NodeHandle nh;
// Make a chatter publisher
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

// Be polite and say hello
char hello[13] = "hello world!";
uint16_t period = 20;
uint32_t last_time = 0;

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
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Another way to get IP
  Serial.print("IP = ");
  Serial.println(nh.getHardware()->getLocalIP());

  // Start to be polite
  nh.advertise(chatter);
}

// LOOP FUNCTION ====================================================================================
void loop()
{
  // digitalWrite(GIGAVACENABLE, HIGH);

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

  if(millis() - last_time >= period)
  {
    last_time = millis();
    if (nh.connected())
    {
      Serial.println("Connected");
      // Say hello
      str_msg.data = hello;
      chatter.publish( &str_msg );
    } else {
      Serial.println("Not Connected");
    }
  }
  nh.spinOnce();
  delay(1);
}

// INTERRUPT FUNCTION ==================================================================================
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