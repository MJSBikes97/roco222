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
