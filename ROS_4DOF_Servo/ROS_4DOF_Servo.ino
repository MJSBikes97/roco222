#include <ros.h>
//Library to describe message datatypes for joint_state topic
#include <sensor_msgs/JointState.h>
#include <Servo.h>

using namespace ros;
#define stepper_step 5
#define stepper_direction 4
#define stepper_enable 8
NodeHandle nh;
Servo servo1;
Servo servo2;
int count = 0;
int target_count = 0;
//int sample;

//Callback Function: writes joint_state angle to the servo
void cb( const sensor_msgs::JointState& msg) {
  int angle[3];
  //Calculate angle in deg. from ROS message
  
    angle[0] = map((msg.position[0]*(180 / 3.14)), -180, 180, -1600, 1600);
    angle[1] = (msg.position[1]*(180/3.14));
    angle[2] = map((msg.position[2]*(180/3.14)), -90, 90, 0, 180);
    if(target_count > count) {
      digitalWrite(stepper_direction, HIGH);
    }
    if(target_count < count) {
      digitalWrite(stepper_direction, LOW);
    }
    if (angle[0] != target_count) {
      target_count = angle[0];
    }
    servo1.write(angle[1]); //Writes between 0 and 180
    servo2.write(angle[2]);
}

Subscriber <sensor_msgs::JointState> sub("joint_states", cb);

void setup() {
  //Serial.begin(9600);
  pinMode(stepper_step, OUTPUT);
  pinMode(stepper_direction, OUTPUT);
  noInterrupts();
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 60000;
  TCCR3B |= (1<<CS10);
  TIMSK3 |= (1<<TOIE1);
  interrupts();
  
  nh.initNode();
  nh.subscribe(sub);
  servo1.attach(6);
  servo2.attach(7);
}

ISR(TIMER3_OVF_vect) {
  TCNT3 = 60000;
  if (count != target_count) {
    digitalWrite(stepper_enable, LOW);
    if(digitalRead(stepper_step) == LOW) {
      digitalWrite(stepper_step,HIGH);
      if (digitalRead(stepper_direction) == HIGH) {
        count++;
      } else {
        count--;
      }
    } else {
      digitalWrite(stepper_step, LOW);
    }
  } else {
    digitalWrite(stepper_enable, HIGH);  
  }
}

void loop() {
  nh.spinOnce();
  delay(1);
}
