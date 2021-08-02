//This header takes care of all ROS functions:
#include "ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"

//ROS DEFINITIONS =============================================================================
ros::NodeHandle DCU1;

// Make a default chatter publisher and subscriber
std_msgs::String str_msg;
// ros::Publisher chatter("chatter", &str_msg);

// void testTopicSub(const std_msgs::String& msg6);
// ros::Subscriber<std_msgs::String> ChatterSubTest("/dcu1/contactor", testTopicSub);

// Be polite and say hello
char hello[13] = "DCU1 IS HERE";
uint16_t period = 20;
uint32_t last_time = 0;

// //ROS motor control
void FrontRightROS(const std_msgs::Float32& msg1); //motor 1
void FrontLeftROS(const std_msgs::Float32& msg2); //motor 2
void RearLeftROS(const std_msgs::Float32& msg3); //motor 3
void RearRightROS(const std_msgs::Float32& msg4); //motor 4
void unlockPowerToMotors(const std_msgs::Int16& msg5); //GIGAVAC contactor

int FrontRightMotor1speed;
int FrontLeftMotor2speed;
int RearLeftMotor3speed;
int RearRightMotor4speed; 

ros::Subscriber<std_msgs::Float32> FrontRightSpeed("/dcu1/motor1/cmd", FrontRightROS); //Front Right wheel, motor 1
ros::Subscriber<std_msgs::Float32> FrontLeftSpeed("/dcu1/motor2/cmd", FrontLeftROS); //Front Left Wheel, motor 2
ros::Subscriber<std_msgs::Float32> RearLeftSpeed("/dcu1/motor3/cmd", RearLeftROS); //Rear Left wheel, motor 3
ros::Subscriber<std_msgs::Float32> RearRightSpeed("/dcu1/motor4/cmd", RearRightROS); //Rear Right Wheel, motor 4

//contactor power
ros::Subscriber<std_msgs::Int16> PowerLock("/dcu1/contactor", unlockPowerToMotors);

//ROS Encoder Control:
std_msgs::Int32 FrontRightEncMsg; //Front Right, motor 1 encoder message to ROS
ros::Publisher FrontRightEncPublish("/dcu1/motor1/enc", &FrontRightEncMsg);

std_msgs::Int32 FrontLeftEncMsg; //Front Left, motor 2 encoder message to ROS
ros::Publisher FrontLeftEncPublish("/dcu1/motor2/enc", &FrontLeftEncMsg);

std_msgs::Int32 RearLeftEncMsg; //Rear Left, motor 3 encoder message to ROS
ros::Publisher RearLeftEncPublish("/dcu1/motor3/enc", &RearLeftEncMsg);

std_msgs::Int32 RearRightEncMsg; //Rear Right, motor 4 encoder message to ROS
ros::Publisher RearRightEncPublish("/dcu1/motor4/enc", &RearRightEncMsg);

