#include <ArduinoHardware.h>
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
//#include <uarm/Joints.h>
#include <string.h>
#include <Servo.h>
#include <EEPROM.h>
//#include <UF_uArm.h>


//UF_uArm uarm_robot;
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
float positions[7] = {0.0};
char* joints[7] = {"base_body_joint",
"body_upper_arm_joint",
"upper_arm_forearm_joint",
"forearm_wrist_joint",
"wrist_palm_joint",
"palm_left_finger_joint",
"palm_right_finger_joint"
};

//ros::Subscriber<std_msgs::Bool> gripper_sub("gripper", gripperCallback);
//ros::Subscriber<uarm::Joints> joint_sub("joint_commands", jointCallback);
                                                   
sensor_msgs::JointState joint_state_msg;
ros::Publisher joint_state_pub("joint_states", &joint_state_msg);


void setup()
{
//  uarm_robot.init(); // initialize the uArm position
  joint_state_msg.position_length = 7;
  joint_state_msg.name_length = 7;
  joint_state_msg.position = positions;
  joint_state_msg.name = joints;

  nh.initNode();
  nh.advertise(joint_state_pub);
//  nh.subscribe(gripper_sub);
//  nh.subscribe(joint_sub);
  delay(500);
}

void loop()
{
  for(int i=0; i<7;i++)  joint_state_msg.position[i] = (float)0.0;
  joint_state_pub.publish(&joint_state_msg);
  nh.spinOnce();
  delay(100);
//  uarm_robot.gripperDetach();
}
