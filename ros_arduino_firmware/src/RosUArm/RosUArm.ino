#include <ArduinoHardware.h>

#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
//#include <uarm/Joints.h>

#include <Servo.h>
#include <EEPROM.h>
#include <UF_uArm.h>

#define JOINTS_NUM 7
UF_uArm uarm_robot;
/*
void jointCallback(const uarm::Joints &command) {
  uarm_robot.setPosition(command.stretch, command.height,
                         command.arm_rot, command.hand_rot);
  delay(500);
}

void gripperCallback(const std_msgs::Bool &command) {
  if (command.data) {
    uarm_robot.gripperCatch();
  } else {
    uarm_robot.gripperRelease();
  }
}

*/
ros::NodeHandle nh;

float positions[JOINTS_NUM] = {0.0};
/*
const char* names[] = {"uarm_base_body_joint", 
                        "uarm_body_upper_arm_joint",
                        "uarm_upper_arm_forearm_joint", 
                        "uarm_forearm_wrist_joint", 
                        "wrist_palm_joint", 
                        "palm_left_finger_joint",
                        "palm_right_finger_joint"};
*/
//ros::Subscriber<std_msgs::Bool> gripper_sub("gripper", gripperCallback);
//ros::Subscriber<uarm::Joints> joint_sub("joint_commands", jointCallback);
                                                   
sensor_msgs::JointState joint_state_msg;
ros::Publisher joint_state_pub("joint_states", &joint_state_msg);
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[13] = "hello world!";

void setup() 
{
  uarm_robot.init();          // initialize the uArm position
//	joint_state_msg.name.resize(JOINTS_NUM);
    joint_state_msg.position_length = JOINTS_NUM;
    joint_state_msg.name_length = JOINTS_NUM;
    joint_state_msg.velocity_length = JOINTS_NUM;
    joint_state_msg.effort_length = JOINTS_NUM;

	joint_state_msg.name[0] = "uarm_base_body_joint";
	joint_state_msg.name[1] = "uarm_body_upper_arm_joint";
	joint_state_msg.name[2] = "uarm_upper_arm_forearm_joint";
	joint_state_msg.name[3] = "uarm_forearm_wrist_joint";
	joint_state_msg.name[4] = "wrist_palm_joint";
	joint_state_msg.name[5] = "palm_left_finger_joint";
	joint_state_msg.name[6] = "palm_right_finger_joint";
//	joint_state_msg.position.resize(JOINTS_NUM);
//  for(int i=0; i< JOINTS_NUM; i++)
//    joint_state_msg.position[i] = 0.0;
 
//	joint_state_msg.velocity.resize(JOINTS_NUM);
//	joint_state_msg.effort.resize(JOINTS_NUM);
//  joint_state_msg.position_length = JOINTS_NUM;
//  for(int i=0; i< JOINTS_NUM; i++){
//    joint_state_msg.position[i] = (float)uarm_robot.getJointRot(i);
//
  joint_state_msg.position = positions;
//  joint_state_msg.name = names;
  nh.initNode();
//  nh.advertise(joint_state_pub);
    nh.advertise(chatter);
//  nh.subscribe(gripper_sub);
//  nh.subscribe(joint_sub);
  delay(500);
}

void loop()
{
  //#define JOINT_BASE_BODY             0    //
//#define JOINT_BODY_UPPER_ARM        1    //
//#define JOINT_UPPER_ARM_FOREARM     2    //
//#define JOINT_FOREARM_WRIST         3    //
//#define JOINT_WRIST_PALM            4     //
//#define JOINT_PALM_RIGHT_FINGER     5     //
//#define JOINT_PALM_LEFT_FINGER      6     //

for(int i=0; i< JOINTS_NUM; i++)
  joint_state_msg.position[i] = 0.0; //(float)uarm_robot.getJointRot(i);

//  joint_state_pub.publish(&joint_state_msg);
  
    str_msg.data = hello;
  chatter.publish( &str_msg );

  nh.spinOnce();
  delay(10);
//  uarm_robot.gripperDetach();  
}
