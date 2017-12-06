# My First Lab Journal (25/09)
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

As a first prototype, I produced a single coil armature with only 60 turns and an average diameter of 35mm. This was connected to a separate commutator.

![Motor V2](https://github.com/MJSBikes97/roco222/blob/master/lab-journal/matt/IMG-20171012-WA0000.jpg)

The brushes are a spring-arm design that have copper tape contacts on the end with connecting leads soldered to them. I have found during extended running periods that these contacts wear out very quickly; a design change will be needed.

![Brushes](https://github.com/MJSBikes97/roco222/blob/master/lab-journal/matt/IMG-20171012-WA0002.jpg)

I found that with some light lubrication, this motor runs at a very stable RPM for extended periods, however, I feel that greater reliability and performance may be gained from adding a second coil.

## Motor v3 - Further Improvements

After the structural limits of the lego motor were accidentally realised (in other words, I dropped it) I decided to implement some further improvements over the original design. These included an additional 60-turn coil a 90-degrees to the existing one and brushes made from exposed wire-ends rather than copper tape.

I also repositioned the brushes so they both trail in the direction of rotation to reduce drag on the commumtator and improve efficiency.

The remainder of the construction was broadly the same as before.

## Motor v4 - Self-Starting and Improved Brushes

To improve the  motor performance I decided to add a third coil. This allowed the motor to be self starting as a certain percentage of the maximum force applied to the coils is always applied to 2 of the 3 coils.

The arrangement of the coils is such that two coils are connected in series between the poles of the commutator. The position of the magnets was also adjusted due to the reduction in the OD of the coil rotor. Each coil has a turn count of 80 and are separated at 120 degree increments.

This motor also uses exposed multicore wire 

Testing showed that this motor had significantly improved performance compared to the 2-Coil motor. 


## Adding the Incremental Encoder

I soldered the encoder board as per the Lab Sheet and tested its output using an oscilloscope. The encoder gave a satisfactory output, so I began mounting the encoder to the motor chassis and made a disc to give input to the encoder.

## Speed Measurement

# Stepper Motor Control

Having wired the stepper motor to the Arduino motor shield as per the Lab Sheet I programmed the Arduino with a code similar to the example given in lectures but with a few functions included to compact the main loop and simplify the later tasks.
![Stepper Motor Wiring](https://github.com/MJSBikes97/roco222/blob/master/lab-journal/matt/IMG_20171102_112640.jpg)
![Full Stepper Motor Arrangement](https://github.com/MJSBikes97/roco222/blob/master/lab-journal/matt/IMG_20171102_112632.jpg)
The motor operated successfully and I was able to adjust its speed by varying the step delay constant in the program.

## Multi-Mode Operation
I produced a separate function for each mode (apart from microstepping) which used the same functions as the main loop for the previous code. I then tested each function with the motor to observe the physical differences between the modes.

## Microstepping


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


