# ROCO222 Lab Journal
Matthew Shaw - 10548340
## Introduction

This journal is written using the markdown syntax.
Markdown allows formatting of rich text using syntax in a plain text editor.
As seen below, hashtags are used to create various sizes of header (more hastags gives a smaller header) and asterisks are used to create bullet-lists.

## Command line 101

### $ ls
* lists contents of current directory (by default) or specified directory.
* Can be used to sort contents by size, file type etc.

### $ cd /tmp
* used to change current directory
* in this case it sets the current directory to tmp

### $ cd $HOME
* sets current directory to HOME

### $ mkdir
* makes a new directory
* this command requires further input to actually create a directory, i.e a name

### $ echo "Hello" > hello.md
* creates a markdown file containing the word "Hello" in the current directory

### $ cat hello.md
* returns the contents of the file hello.md

### $ cp hello.md hello-again.md
* copies the contents of the file hello.md to the file hello-again.md; creating the latter in the process
* can be used to copy multiple files to a directory

### $ mv hello-again.md hello-hello.md
* renames 'hello-again.md' to 'hello-hello.md'

### $ rm hello.md
* deletes the file hello.md

### rm -rf
* 

### cat /proc/cpuinfo
* returns info abou the machine's CPU

## Uploading to GitHub Repository

## Hacking into the Robot
Knowing that the robot's name was Chapman, we used the terminal commang "ping chapman" to retrieve the IP adress for the robot.

With the IP address, using the command "ssh nao@192.168.0.184" we established an SSH connection to the robot using the password "nao"

Using the GNU nano text editor we copied the python program to the robot, saving it as a .py file to run on the robot.

# Building the DC Motor

## The initial motor design
I began by attempting to build a motor using the provided materials. As a slight improvement to those shown in the lab notes, I decided to add a second coil to reduce cogging of the motor. 

I wound each coil with the minimum 60 turns and recorded equal coil resistances of 5 ohms per coil.

![1st Armature](https://github.com/MJSBikes97/roco222/blob/master/lab-journal/matt/IMG-20171005-WA0001.jpg)

Unfortunately, the armature I had constructed had a fatal flaw, as the cork used had holes for the motor shaft which were not concentric; resulting in violent eccentric motion when spun. I therefore opted to abandon this design and move to an improved method of construction.

## Motor v2 - Lego
In order to rapidly prototype a more reliable motor, I decided to use lego technic components to construct my armature. The shaft parts are easily dissasembled and very 'tweakable', making them ideal for finely adjusting the motor. 

Also, unlike a 3D-Printed prototype, a lego design is not limited by the long print times; The protoype could be completely rebuilt in minutes.

As a first prototype, I produced a single coil armature with only 60 turns and an average diameter of 35mm. This was connected to a separate commutator. The coil resistance was again measured at 5 ohms.

![Motor V2](https://github.com/MJSBikes97/roco222/blob/master/lab-journal/matt/IMG-20171012-WA0000.jpg)

The brushes are a spring-arm design that have copper tape contacts on the end with connecting leads soldered to them. I found that during extended running periods that these contacts wear out very quickly; a design change was needed.

![Brushes](https://github.com/MJSBikes97/roco222/blob/master/lab-journal/matt/IMG-20171012-WA0002.jpg)

I found that with some light lubrication, this motor runs at a very stable RPM for extended periods, however, I feel that greater reliability and performance may be gained from adding a second coil. The motor can be seen [here](https://youtu.be/CTOZp_LgISQ) running with a 12V, 4A DC supply

The main issue with this motor was the cogging effect due to the small period where force is applied to the armature (and hence torque generated) during each rotation of the motor shaft.

The effect can be seen in [this](https://youtu.be/0WgSKoiiXSg) slow-motion clip.

## Motor v3 - Further Improvements

After the structural limits of the lego motor were accidentally realised (in other words, I dropped it) I decided to implement some further improvements over the original design. These included an additional 60-turn coil at 90-degrees to the existing one and brushes made from exposed wire-ends rather than copper tape.

I also repositioned the brushes so they both trail in the direction of rotation to reduce drag on the commumtator and improve efficiency.

The remainder of the construction was broadly the same as before.

The additional coils reduced the cogging effect when the motor was operating, however the motor was not self starting. This was because, when powered up, the first coil aligned horizontally with the magnetic field would provide force to move the armature the first 45-degrees, but would then have moved out of the magnetic field. This would cause the motor to stall with both coils at 45-degree angles from the magnetic field due to a lack of armature momentum. The armature would stall before the second coil was connected to the supply by the commutator, hence a manual start was always required by pushing the flywheel connected to the armature.
## Motor v4 - Self-Starting and Improved Brushes

To improve the  motor performance I decided to add a third coil. This allowed the motor to be self starting as a certain percentage of the maximum force applied to the coils is always applied to 2 of the 3 coils. This means the motor cannot be stalled at startup as the force on the armature is always imbalanced.
![3-Coil Armature](https://github.com/MJSBikes97/roco222/blob/master/lab-journal/matt/IMG_20180109_122936.jpg)
![Armature Windings](https://github.com/MJSBikes97/roco222/blob/master/lab-journal/matt/IMG_20180109_122952.jpg)
The arrangement of the coils is such that two coils are connected in series between the poles of the commutator. The position of the magnets was also adjusted due to the reduction in the OD of the coil rotor. Each coil has a turn count of 80 and are separated at 120 degree increments.

All 3 Coils of the armature are in fact connected together in series. The pads on the commutator are arranged such that each one is connected as a center tap between two of the coils. This arrangement gave a coil resistance of 3.3 ohms between each pad, as I used a thicker gauge of enameled wire which has a lower resistivity. This gave a greater coil current for the same supply voltage.

As the torque of the motor is directly proportional to the product of current and number of turns, the increase in both of these parameters in the v4 design, the motor ran at a significantly higher RPM compared to to the 2-coil design as the mechanical losses in the motor were very similar.
![v4 Motor Internals](https://github.com/MJSBikes97/roco222/blob/master/lab-journal/matt/IMG_20180109_123156.jpg)
This motor also uses exposed multicore wire brushes instead of strips of copper tape as this was found to give greater reliability. The copper tape would wear through very quickly and  cause the motor to fail. 
![v4 Brushes](https://github.com/MJSBikes97/roco222/blob/master/lab-journal/matt/IMG_20180109_123205.jpg)
Testing showed that this motor had significantly improved performance compared to the 2-Coil motor. 


## Adding the Incremental Encoder

I soldered the encoder board as per the Lab Sheet and tested its output using an oscilloscope. The encoder gave a satisfactory output, so I began mounting the encoder to the motor chassis and made a disc with a cutout to give input to the encoder.
```
const byte ledPin = 13;
const byte interruptPin = 2;
volatile byte state = LOW;

void setup() {
pinMode(ledPin, OUTPUT);
pinMode(interruptPin, INPUT);

/*### config interrupt callback for every lo-hi transition ###*/
attachInterrupt(digitalPinToInterrupt(interruptPin), blink,
RISING);
}

void loop() {
digitalWrite(ledPin, state);

}

void blink() {
  state = !state;
}
```
This basic script inverts the LED every time a rising edge is detected from the encoder, hence the LED blinks at half the frequency of rotation.
## Speed Measurement
To measure the RPM of the motor, I used 10 pulse edges of the encoder and the Arduino millis() function to time the period of 10 rotations. I used the interrupt service routine to increment the rotation counter as well as inverting the LED state as per the previous script.

I then used the main loop to calculate and write the rpm value out on the serial monitor. This would allow closed loop control of the motor later.
```
/*#### Encoder RPM Measurement ####*/

const byte ledPin = 13;
const byte interruptPin = 2;

unsigned long start_time;
unsigned long stop_time;
int n = 0;
unsigned long period;
unsigned long rpm;
volatile byte state = LOW;

void setup() {
pinMode(ledPin, OUTPUT);
pinMode(interruptPin, INPUT);
Serial.begin(9600);
/*### config interrupt callback for every lo-hi transition ###*/
attachInterrupt(digitalPinToInterrupt(interruptPin), timer,
RISING);
}

void loop() {
  start_time = millis();
  while (n<=10) {
    digitalWrite(ledPin, state);
  }
  stop_time = millis();
  period = ((stop_time - start_time)/10);
  rpm = (60000/period);
  Serial.println(rpm);
  n = 0;
}

void timer() {
 n++; 
 state = !state;
}
```
This [link](https://youtu.be/K2EGBEzusm4) is to a video of the encoder and speed measurement script all working, including the oscilloscope trace of the output from the incremental encoder.

The motor's speed for a 12V, 3A input current averaged 2700rpm according to the encoder measurement. The motor would operate stably down to speeds of around 900rpm before the mechanical losses would cause unreliability and potentially stall the motor. At this speed, the motor would require a coil current of approx. 1.5A at 12V.

## Speed Control with the Arduino

The next step was to use these rpm measurements to develop some sort of closed loop control for the DC motor. This required the use of a L298P motor driver shield for the Arduino.

# Stepper Motors

## Stepper Motor Control

Having wired the stepper motor to the Arduino motor shield as per the Lab Sheet I programmed the Arduino with a code similar to the example given in lectures but with a few functions included to compact the main loop and simplify the later tasks.
![Stepper Motor Wiring](https://github.com/MJSBikes97/roco222/blob/master/lab-journal/matt/IMG_20171102_112640.jpg)
![Full Stepper Motor Arrangement](https://github.com/MJSBikes97/roco222/blob/master/lab-journal/matt/IMG_20171102_112632.jpg)
The motor operated successfully and I was able to adjust its speed by varying the step delay constant in the program.
```
/*### Stepper Motor Control ###*/

/*## Definitions ##*/

#define DIR_A 12
#define DIR_B 13
#define PWM_A 10
#define PWM_B 11

#define Fwd HIGH
#define Rev LOW
/*## Step Delay ##*/

const int stepDelay_ms = 5;

void setup() {
/*## PinMode Config ##*/
pinMode(DIR_A, OUTPUT);
pinMode(DIR_B, OUTPUT);
// Turn off Braking for both Channels
pinMode(8, OUTPUT); digitalWrite(8, LOW);
pinMode(9, OUTPUT); digitalWrite(9, LOW);

}

void loop() {
  setDirection(Fwd, Fwd);
  ch_A_ON();
  ch_B_OFF();
  delay(stepDelay_ms);

  ch_A_OFF();
  ch_B_ON();
  delay(stepDelay_ms);

  setDirection(Rev, Rev);
  ch_A_ON();
  ch_B_OFF();
  delay(stepDelay_ms);

  ch_A_OFF();
  ch_B_ON();
  delay(stepDelay_ms);

}
/*## Directional Control Function ##*/
void setDirection(int A, int B) {
  digitalWrite(DIR_A, A);
  digitalWrite(DIR_B, B);

}
/*## CH A Control Functions ##*/
void ch_A_ON() {
  analogWrite(PWM_A, 255);
}

void ch_A_OFF() {
  analogWrite(PWM_A, 0);
}
/*## CH B Control Functions ##*/
void ch_B_ON() {
  analogWrite(PWM_B, 255);
}

void ch_B_OFF() {
  analogWrite(PWM_B, 0);
}
```
## Multi-Mode Operation
I produced a separate function for each mode (apart from microstepping) which used the same functions as the main loop for the previous code. I then tested each function with the motor to observe the physical differences between the modes.
The functions for the modes are shown below; setup code is broadly the same as the basic test script above.
```
/*## Mode Functions ##*/
void fullStep() {
    setDirection(Fwd, Fwd);
  ch_A_ON();
  ch_B_OFF();
  delay(stepDelay_ms);

  ch_A_OFF();
  ch_B_ON();
  delay(stepDelay_ms);

  setDirection(Rev, Rev);
  ch_A_ON();
  ch_B_OFF();
  delay(stepDelay_ms);

  ch_A_OFF();
  ch_B_ON();
  delay(stepDelay_ms);
}

void doubStep() {
  setDirection(Fwd,Fwd);
  ch_A_ON();
  ch_B_ON();
  delay(stepDelay_ms);

  setDirection(Rev, Fwd);
  delay(stepDelay_ms);

  setDirection(Rev,Rev);
  delay(stepDelay_ms);

  setDirection(Fwd, Rev);
  delay(stepDelay_ms);
}

void halfStep() {
  //1
  setDirection(Fwd, Fwd);
  ch_A_ON();
  ch_B_OFF();
  delay(stepDelay_ms);
  //2
  ch_A_ON();
  ch_B_ON();
  delay(stepDelay_ms);
  //3
  ch_A_OFF();
  ch_B_ON();
  delay(stepDelay_ms);
  //4
  setDirection(Rev, Fwd);
  ch_A_ON();
  ch_B_ON();
  delay(stepDelay_ms);
  //5
  ch_A_ON();
  ch_B_OFF();
  delay(stepDelay_ms);
  //6
  setDirection(Rev, Rev);
  ch_A_ON();
  ch_B_ON();
  delay(stepDelay_ms);
  //7
  ch_A_OFF();
  ch_B_ON();
  delay(stepDelay_ms);
  //8
  setDirection(Fwd, Rev);
  ch_A_ON();
  ch_A_OFF();
}
```

## Microstepping
My Microstepping code generates tables of sine and cosine PWM values that can be used in the analogWrite() function in Arduino. The tables are stored as arrays. The code then writes the corresponding positions from the arrays to the two coils of the stepper, putting each coil voltage 90 degrees out of phase with each other. By increasing the number of data points in the arrays, the resolution of the microsteps-per-step increases. The code snippet below shows the microstepping routine:
```
/*## Definitions ##*/

#define DIR_A 12
#define DIR_B 13
#define PWM_A 10
#define PWM_B 11
#define ZERO_SWITCH 5

#define Fwd HIGH
#define Rev LOW
int usteps = 360;
float amp = 127;
int pulseDelay_us = 40;
int a[360];
int b[360];
int idxG = 0;

void writeMotor(int value, int PWM_PIN, int DIR_PIN);
/*## Step Delay ##*/

//const int stepDelay_ms = 50;

void setup() {
/*## Sin/Cos Tables ##*/
for (int i=0; i<usteps; i++) {
  a[i] = amp*sin(i*2*PI/usteps);
  b[i] = amp*cos(i*2*PI/usteps);
}
/*## PinMode Config ##*/
pinMode(DIR_A, OUTPUT);
pinMode(DIR_B, OUTPUT);
pinMode(ZERO_SWITCH, INPUT);
// Turn off Braking for both Channels
pinMode(8, OUTPUT); digitalWrite(8, LOW);
pinMode(9, OUTPUT); digitalWrite(9, LOW);

}

void loop() {
  while (digitalRead(ZERO_SWITCH) == HIGH) {
    writeMotor(a[idxG], PWM_A, DIR_A);
    writeMotor(b[idxG], PWM_B, DIR_B);
  
    idxG++;
    if (idxG == usteps) {
      idxG = 0;
    }
    delayMicroseconds(pulseDelay_us);
  }
  delay(100);
  analogWrite(PWM_A, 0);
  analogWrite(PWM_B, 0);
  delay(2000);
}

void writeMotor(int value, int PWM_PIN, int DIR_PIN) {
  int absPWM = abs(value);
  analogWrite(PWM_PIN, absPWM);
  if (value > 0) {
    digitalWrite(DIR_PIN, Fwd);
  } else {
    digitalWrite(DIR_PIN, Rev);
  }
}
```
In this case the microstep resolution is 360 per step. This gives a total of 72000 steps per rotation as the motor has 200 full steps.
# Servo Arm Project

## First Steps - Interfacing Servos to the Arduino
### Initial testing

The first test code I wrote to control the Servo Motors from the Arduino used the Arduino Servo library and two 'for' loops to iterate the angle output to the servos between 0 and 180, then from 180 back to 0, allowing the servos to sweep through their full operating range. A simple blocking delay was added in the loops to change the rate of the sweep.

```
/*### 2-Servo Control Test ###*/
//Servo Library Definitions
#include <Servo.h>
Servo actuator_1;
Servo actuator_2;
#define actuator_1_pin 9
#define actuator_2_pin 10

//Standard Delay Time
int loop_delay_ms = 10;

void setup() {
  actuator_1.attach(actuator_1_pin,700,2000);
  actuator_2.attach(actuator_2_pin,700,2000);

}

void loop() {
  for (int n=0; n<=180; n++) {
    actuator_1.write(n);
    actuator_2.write(180-n);
    delay(loop_delay_ms);
  }
  for(int n=180; n>=0; n--) {
    actuator_1.write(n);
    actuator_2.write(180-n);
    delay(loop_delay_ms);
  }
}

```
The servos were wired up using a breadboard to allow use of common power rails for both servos.
![Servo Setup on Breadboard](https://github.com/MJSBikes97/roco222/blob/master/lab-journal/matt/IMG_20171109_113049.jpg)
This code does not give a smooth sinusoidal movement of the servo shaft, however it does allow the servos to be quickly tested.

### Controlling the servos with potentiometers

To control the servos with potentiometers I once again used the Arduino servo library and the Arduino 'map' function. This allowed me to map the ADC inputs of 0 to 1023 to the 0 to 180 degree values of the 'servo.write' function. I also set up the USB serial monitor to confirm the ADC values were being recieved correctly.
A blocking delay of 1ms was added to the loop to allow the values on the monitor to be read more easily.

```
/*### Potentiometer 2-Servo Control ###*/
//Servo Library Definitions
#include <Servo.h>
Servo actuator_1;
Servo actuator_2;
#define actuator_1_pin 9
#define actuator_2_pin 10

//Analog In Pin Definitions
#define act_1_ctrl A0
#define act_2_ctrl A1
//Standard Delay Length
int loop_delay_ms = 1;
//Serial Definitions
#define PC_baud 9600

/*## Setup ## */
void setup() {
//Attach Servos
actuator_1.attach(actuator_1_pin,700,2000);
actuator_2.attach(actuator_2_pin,700,2000);
Serial.begin(PC_baud);
}
/*## Main Loop ##*/
void loop() {
  actuator_1.write(map(analogRead(act_1_ctrl), 0, 1023, 0, 180));
  Serial.println(analogRead(act_1_ctrl));
  actuator_2.write(map(analogRead(act_2_ctrl), 0, 1023, 0, 180));
  Serial.println(analogRead(act_2_ctrl));
  delay(loop_delay_ms);

}
```
I set up the 2 Potentiometers on a breadboard as shown below for testing the code:
![Dual Potentiometer Control](https://github.com/MJSBikes97/roco222/blob/master/lab-journal/matt/IMG_20171109_124655.jpg)
![Potentiometer Wiring](https://github.com/MJSBikes97/roco222/blob/master/lab-journal/matt/IMG_20171109_124645.jpg)

## Controlling the Robot Arm with ROS

### ROS Node for the Arduino

Having successfully installed ROS and tested the /test topic echo using the terminal I loaded the Arduino Script to the UNO board and tested the Servo topic.
The Arduino script is as follows:

```
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
```
The function defined at line 10 (void cb) is a function to describe what the ROS Topic does with the message it recieves from the the rosserial topic.
The object 'Subscriber' subscribes the Arduino to the topic "servo" with the message cb. Hence when a message is published on the topic servo, the Arduino updates the servo to reflect the contents of the message. 

### Using RViz and URDF

![Screenshot of RViz and the Joint State Publisher running](https://github.com/MJSBikes97/roco222/blob/master/lab-journal/matt/Screenshot%20from%202017-12-07%2011-38-44.png)

Shown in this screenshot is a single-joint URDF model running in RViz with the Joint state Publisher node UI. I used this to test the operation of these ROS components using the example URDF file before creating my own URDF for my arm design.

My arm was intended to have 4 degrees of freedom; 2 revolute joints for the arm segments, 1 for the grabber at the end of the arm and a single continuous turret joint with a stepper motor. This requires the URDF model to have 5 segments (including base_link).

Initially, I tested a 2 segment arm with a fixed base to ensure the URDF and servos were behaving as intended when running with RViz and the Arduino node. The URDF for this arm is shown here:
```
<?xml version="1.0"?>
<robot name="roco_arm">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.06" radius="0.1"/>
      </geometry>
    </visual>
  </link>

  <link name="first_segment">
    <visual>
      <geometry>
        <box size="0.6 0.05 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="-0.3 0 0" />
    </visual>
  </link>

  <link name="second_segment">
    <visual>
      <geometry>
        <box size="0.6 0.05 0.1"/>
      </geometry>
      <origin rpy="0 0 0" xyz="-0.3 0 0" />
    </visual>
  </link>

 

  <joint name="base_to_first" type="revolute">
    <axis xyz="0 1 0" />
    <limit effort="1000" lower="0" upper="3.14" velocity="0.5" />
    <parent link="base_link"/>
    <child link="first_segment"/>
    <origin xyz="0 0 0.03" />
  </joint>

<joint name="first_to_second" type="revolute">
    <axis xyz="0 1 0" />
    <limit effort="1000" lower="0" upper="3.14" velocity="0.5" />
    <parent link="first_segment"/>
    <child link="second_segment"/>
    <origin xyz="-0.6 0 0" />
  </joint>

</robot>

```

It quickly became apparent that having a ROS launch file for all the required nodes would be extremely helpful and would streamline the development process. Hence I produced a launch file including all nodes apart from the rosserial_python node; reason being that I was regularly updating the Arduino code and wanted to be able to stop and start this node separately.
```
<launch>
	<arg name="model" />
	<arg name="gui" default = "true" />
	
	<param name="robot_description" textfile="$(arg model)" />
	<param name="use_gui" value="$(arg gui)" />

	<node name="joint_state_publisher" pkg="joint_state_publisher"
		type="joint_state_publisher" />
	<node name="robot_state_publisher" pkg="robot_state_publisher"
		type="state_publisher" />
	<node name="rviz" pkg="rviz" type="rviz" required="true" />
</launch>
	
```

#### The First Arm design - Lego Again
In order to quickly begin prototyping the arm and its movement I 3D-Printed a set of 3 lego adapters for the 9g servos, consisting of a brick and a shaft adapter, for which I modelled the spline of the servo.

The adapters allowed me to attach lego gears to the servo directly in order to move parts of the lego arm. Pictures of the lego arm are shown below:
![The Lego Arm Design](https://github.com/MJSBikes97/roco222/blob/master/lab-journal/matt/IMG_20171215_181922.jpg)
![Lego Servo Mounts](https://github.com/MJSBikes97/roco222/blob/master/lab-journal/matt/IMG_20171215_181929.jpg)
![Turret Stepper](https://github.com/MJSBikes97/roco222/blob/master/lab-journal/matt/IMG_20171215_181931.jpg)
Issues with the mass of the arm segments quickly became apparent, with the servos struggling to make fine positional adjustments due to insufficient torque. Despite this, I was able to produce an accurate URDF for the first 2 segments and the base stepper of this arm. This allowed me to test the ROS Node on the Arduino with the servos in position and begin programming the stepper motor functionality.
```
<?xml version="1.0"?>
<robot name="roco_arm">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.045" radius="0.085"/>
      </geometry>
    </visual>
  </link>

  <link name="arm_turret">
    <visual>
      <geometry>
        <cylinder length="0.025" radius="0.068"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0.0175" />
    </visual>
  </link>

  <link name="first_segment">
    <visual>
      <geometry>
        <box size="0.095 0.032 0.032"/>
      </geometry>
      <origin rpy="0 0 0" xyz="-0.06 0 0" />
    </visual>
  </link>

  <link name="second_segment">
    <visual>
      <geometry>
        <box size="0.075 0.032 0.032"/>
      </geometry>
      <origin rpy="0 0 0" xyz="-0.034 0 0" />
    </visual>
  </link>

 <joint name="turret_swivel" type="continuous">
    <axis xyz="0 0 1" />
    <parent link="base_link"/>
    <child link="arm_turret"/>
    <origin xyz="0 0 0.03" />
  </joint>

  <joint name="turret_to_first" type="revolute">
    <axis xyz="0 1 0" />
    <limit effort="1000" lower="0" upper="3.14" velocity="0.3" />
    <parent link="arm_turret"/>
    <child link="first_segment"/>
    <origin xyz="0 0 0.0255" />
  </joint>

<joint name="first_to_second" type="revolute">
    <axis xyz="0 1 0" />
    <limit effort="1000" lower="-1.57" upper="1.57" velocity="0.3" />
    <parent link="first_segment"/>
    <child link="second_segment"/>
    <origin xyz="-0.12 0 0" />
  </joint>

</robot>
```

#### The 3D-Printed Arm - A better solution
To resolve the issues of lack of torque in the servos I decided to produce a more lightweight design that could be 3d printed. This design would also allow me to add the 3rd servo for the grabber attached to the end of the arm.

A few reprints of parts for the design were required. The main issues initially encountered were the attachment of the servo to the turret, where the shaft on the arm segment could not be positioned due to a lack of clearance. This issue was solved by splitting the saddle where the shaft sits and adding a cap to keep the shaft in position. Also the teeth on the grabber gears were far too fine for the 3D-Printer and as a result would not engage and operate properly. I reprinted these with a coarser tooth pattern and a 20-degree helix angle to keep the gears engaged and prevent slippage.
#### Programming for Stepper Motor Joint
Adding additional servos to the Arduino was a relatively simple process thanks to the Servo library; simply writing a different value from the Joint State Publisher message array onto the PWM pin for the servo. The stepper motor, however, was a considerably more involved to implement.

Initially, I opted to use a basic motor driver shield that I had for my Arduino, which used and L298p H-bridge like the official Arduino shields. The board was the same as that which I had used for the microstepping development task. This presented a problem due to the rate at which the stepper motor must be written to whilst microstepping in order to operate correctly. The number of writes and delays between the writes was slowing down the ROS callback function to the extent that the servo response to the joint state publisher would be very delayed and jerky. I had to find an alternative solution.
![The L298P Driver Board](https://github.com/MJSBikes97/roco222/blob/master/lab-journal/matt/IMG_20171215_181936.jpg)

As I was using an Arduino ATmega2560-based development board I decided to use some of its additional hardware timers that are not used by the Servo.h library to generate a timer interrupt for stepping the motor. To make the interrupt rountine simpler I also switched to a different motor driver board specifically for driving stepper motors. It has two input pins from the Arduino; a direction pin and a step pin. The step pin will respond to a rising edge of a digital write.

Using the timer interrupt, the stepper could be moved towards a target position whilst the servos were being moved independently, the target being updated by each ROS callback function. Hence the issue with the stepper preventing the servos from updated was resolved.

The A4988 Stepper Driver used in this board has 3 mode select pins and a current limiting potentiometer. For the purpose of this arm I set the current limit to 500mA and connected all the mode pins high so that a 16 microstep-per-step resolution was set. This required mapping of the Joint_State message value to between -1600 and 1600 as the motor has 200 steps-per-revolution. The Arduino sketch for the node is shown below:
```
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
```
#### Adding the Grabber Joint and Issues with Serial connection speed
Having successfully added the stepper motor with the initial 2 servos I ran into a new problem. When adding a 4th joint to the URDF to actuate the grabber servo, the arm became unresponsive. The default baud rate for the rosserial_python node is 57600bps. This may mean that the Joint messages are not being sent fast enough to transfer all the data before the other nodes update. My solution was to increase the baud rate to 115200bps, which seemed to resolve the issue.

I created a 4-DOF URDF with the basic dimensions required to test the arm, but not yet fully accurate.
```
<?xml version="1.0"?>
<robot name="roco_arm">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.045" radius="0.085"/>
      </geometry>
    </visual>
  </link>

  <link name="arm_turret">
    <visual>
      <geometry>
        <cylinder length="0.025" radius="0.068"/>
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0" />
    </visual>
  </link>

  <link name="first_segment">
    <visual>
      <geometry>
        <box size="0.095 0.032 0.032"/>
      </geometry>
      <origin rpy="0 0 0" xyz="-0.0475 0 0" />
    </visual>
  </link>

  <link name="second_segment">
    <visual>
      <geometry>
        <box size="0.075 0.032 0.032"/>
      </geometry>
      <origin rpy="0 0 0" xyz="-0.0375 0 0" />
    </visual>
  </link>

  <link name="grabber_1">
     <visual>
       <geometry>
         <box size="0.045 0.005 0.005"/>
       </geometry>
       <origin rpy="0 0 0" xyz="-0.0175 0 0" />
     </visual>
  </link>

 <joint name="turret_swivel" type="continuous">
    <axis xyz="0 0 1" />
    <parent link="base_link"/>
    <child link="arm_turret"/>
    <origin xyz="0 0 0.0175" />
  </joint>

  <joint name="turret_to_first" type="revolute">
    <axis xyz="0 1 0" />
    <limit effort="1000" lower="0" upper="3.14" velocity="0.1" />
    <parent link="arm_turret"/>
    <child link="first_segment"/>
    <origin xyz="0 0 0.025" />
  </joint>

  <joint name="first_to_second" type="revolute">
     <axis xyz="0 1 0" />
     <limit effort="1000" lower="-1.57" upper="1.57" velocity="0.1" />
     <parent link="first_segment"/>
     <child link="second_segment"/>
     <origin xyz="-0.095 0 0" />
  </joint>

  <joint name="second_to_grabber" type="revolute">
     <axis xyz="0 1 0" />
     <limit effort="1000" lower="0" upper="3.14" velocity="0.1" />
     <parent link="second_segment"/>
     <child link="grabber_1"/>
     <origin xyz="-0.075 0 0" />
  </joint>

</robot>
``` 



