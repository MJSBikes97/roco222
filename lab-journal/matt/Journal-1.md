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

## Adding the Incremental Encoder

I soldered the encoder board as per the Lab Sheet and tested its output using an oscilloscope. The encoder gave a satisfactory output, so I began mounting the encoder to the motor chassis and made a disc to give input to the encoder.

# Stepper Motor Control

Having wired the stepper motor to the Arduino motor shield as per the Lab Sheet I programmed the Arduino with a code similar to the example given in lectures but with a few functions included to compact the main loop and simplify the later tasks.

The motor operated successfully and I was able to adjust its speed by varying the step delay constant in the program.

## Multi-Mode Operation
I produced a separate function for each mode (apart from microstepping) which used the same functions as the main loop for the previous code. I then tested each function with the motor to observe the physical differences between the modes.

## Microstepping

