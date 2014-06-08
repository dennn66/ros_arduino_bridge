/* 
 * rosserial Planar Odometry Example
 */

#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

ros::NodeHandle  nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;
sensor_msgs::JointState joint_state_msg;
//ros::Publisher joint_state_pub("joint_states", &joint_state_msg);

double x = 1.0;
double y = 0.0;
double theta = 1.57;

char base_link[] = "/base_link";
char odom[] = "/odom";
#define JOINTS_NUM 7
float positions[JOINTS_NUM] = {0.0};

void setup()
{
//  joint_state_msg.position = positions;
  nh.initNode();
  broadcaster.init(nh);
//  nh.advertise(joint_state_pub);
}

void loop()
{  
  // drive in a circle
  double dx = 0.2;
  double dtheta = 0.18;
  x += cos(theta)*dx*0.1;
  y += sin(theta)*dx*0.1;
  theta += dtheta*0.1;
  if(theta > 3.14)
    theta=-3.14;
    
  // tf odom->base_link
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  
  t.transform.rotation = tf::createQuaternionFromYaw(theta);
  t.header.stamp = nh.now();
  //joint_state_pub.publish(&joint_state_msg);
  broadcaster.sendTransform(t);
  nh.spinOnce();
  
  delay(10);
}
