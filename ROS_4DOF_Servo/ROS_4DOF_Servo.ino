#include <ros.h>
//Library to describe message datatypes for joint_state topic
#include <sensor_msgs/JointState.h>
#include <Servo.h>

using namespace ros;
#define stepper_step 5
#define stepper_direction 4
#define stepper_enable 3
NodeHandle nh;
Servo servo1;
Servo servo2;
Servo servo3;
int count = 0;
int target_count = 0;
//int sample;

//Callback Function: writes joint_state angle to the servo
void cb( const sensor_msgs::JointState& msg) {
  int angle[4];
  //Calculate target microstep from ROS message
  angle[0] = map((msg.position[0]*(180 / 3.14)), -180, 180, -1600, 1600);
  //Calculate joint angles in deg. from ROS message
  angle[1] = (msg.position[1]*(180/3.14));
  angle[2] = map((msg.position[2]*(180/3.14)), -90, 90, 0, 180);
  angle[3] = (msg.position[3]*(180/3.14));
  //If the target angle is changed, update it for the ISR
  if (angle[0] != target_count) {
    target_count = angle[0];
  }
  //Compare current position with target to set stepper direction
  if(target_count > count) {
    digitalWrite(stepper_direction, HIGH);
  }
  if(target_count < count) {
    digitalWrite(stepper_direction, LOW);
  }
 //Write to servos
  servo1.write(angle[1]); //Writes between 0 and 180
  servo2.write(angle[2]);
  servo3.write(angle[3]);
}
//Setup ROS Subscriber
Subscriber <sensor_msgs::JointState> sub("joint_states", cb);

void setup() {
  nh.getHardware()->setBaud(115200); //Increase baud rate due to message size
  pinMode(stepper_step, OUTPUT); //Config stepper driver pins
  pinMode(stepper_direction, OUTPUT);
  pinMode(stepper_enable, OUTPUT);
  noInterrupts(); //Disable all interrupts whilst setting up hardware timer
  TCCR3A = 0; //Count-Compare registers set to 0
  TCCR3B = 0;
  TCNT3 = 60000; //Set count value to 60000, max is 65535, hence 5535 clock edges between interrupts
  TCCR3B |= (1<<CS10); //Clock divded by 8; gives a clock speed for the timer of 2MHz
  TIMSK3 |= (1<<TOIE1); //Enable overflow interrupt, when timer count exceeds 65535, interrupt occurs
  interrupts(); //Enable interrupts
  
  nh.initNode();
  nh.subscribe(sub);
  servo1.attach(6); //Attach servos to appropriate pins
  servo2.attach(7);
  servo3.attach(8);
}
/*### Timer Interrupt Service Routine ###*/
ISR(TIMER3_OVF_vect) {
  TCNT3 = 60000; //Set count to start value 
  if (count != target_count) { //Compare count value to value output by callback
    digitalWrite(stepper_enable, LOW); //Enable stepper drivere
    if(digitalRead(stepper_step) == LOW) { //Invert step pin and increment count based on direction pin
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
    digitalWrite(stepper_enable, HIGH);  //If the count is equal to target, disable stepper driver
  }
}

void loop() {
  nh.spinOnce();
  delay(1);
}
