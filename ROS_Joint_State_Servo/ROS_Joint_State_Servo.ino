#include <ros.h>
//Library to describe message datatypes for joint_state topic
#include <sensor_msgs/JointState.h> 
#include <Servo.h>

using namespace ros;

NodeHandle nh;
Servo servo1;

//Callback Function: writes joint_state angle to the servo
void cb( const sensor_msgs::JointState& msg) {
  //Calculate angle in deg. from ROS message
  int angle = (int) (msg.position[0]*(180/3.14));
  servo1.write(angle); //Writes between 0 and 180
}

Subscriber <sensor_msgs::JointState> sub("joint_states", cb);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  servo1.attach(9);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
