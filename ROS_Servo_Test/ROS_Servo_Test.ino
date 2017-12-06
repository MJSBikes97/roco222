#include <ros.h>
#include <std_msgs/UInt16.h>
#include <Servo.h>

using namespace ros;

NodeHandle nh;
Servo servo1;

void cb( const std_msgs::UInt16& msg) {
  servo1.write(msg.data); //Writes between 0 and 180
}

Subscriber <std_msgs::UInt16> sub("servo", cb);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  servo1.attach(9);
}

void loop() {
  nh.spinOnce();
  delay(1);
}
