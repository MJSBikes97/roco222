#include <ros.h>
//Library to describe message datatypes for joint_state topic
#include <sensor_msgs/JointState.h>
#include <Servo.h>

using namespace ros;

NodeHandle nh;
Servo servo1;
Servo servo2;

//Callback Function: writes joint_state angle to the servo
void cb( const sensor_msgs::JointState& msg) {
  int angle[2];
  //Calculate angle in deg. from ROS message
  for (int n = 0; n <= 1; n++) {
    angle[n] = (int) (msg.position[n] * (180 / 3.14));
    switch (n) {
      case 0:
        servo1.write(angle[n]); //Writes between 0 and 180
        break;
      case 1:
        servo2.write(angle[n]);
        break;
    }
  }
}

Subscriber <sensor_msgs::JointState> sub("joint_states", cb);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  servo1.attach(9);
  servo2.attach(10);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
