# Autonomous Navigation using QLearning
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![Build Status](https://travis-ci.org/harish1696/QLearning-Navigation-.svg?branch=master)](https://travis-ci.org/harish1696/QLearning-Navigation-)

## Overview

The module will be able to train a robot to navigate autonomously in a completely unknown environment by avoiding obstacles using Q-learning technique. A model world will be simulated in Gazebo and the robot will be allowed to navigate in it without any knowledge about the world. A positive reward is given to states clear of obstacles while negative reward is given to states that is proportional to the proximity of an obstacle. The robot is allowed to navigate the world repeatedly until Q value table is updated to achieve an optimal policy. With the help of the Q table generated, the robot can navigate in any other random world/environment by avoiding obstacles.

## License
BSD License

Copyright (c) 2017 Harish Sampathkumar

```
Redistribution and use in source and binary forms, with or without  
modification, are permitted provided that the following conditions are 
met:
 
1. Redistributions of source code must retain the above copyright notice, 
this list of conditions and the following disclaimer.
 
2. Redistributions in binary form must reproduce the above copyright 
notice, this list of conditions and the following disclaimer in the   
documentation and/or other materials provided with the distribution.
 
3. Neither the name of the copyright holder nor the names of its 
contributors may be used to endorse or promote products derived from this 
software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS 
IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
CONTRIBUTORS BE 
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
THE POSSIBILITY OF SUCH DAMAGE.
```

## SIP Process
SIP Process is followed to develop the module. It is detailed in this [link](https://docs.google.com/spreadsheets/d/1iwXafoxuYP-64WJcZ8xOhcCnD-_6G_7DA12rQynheLY/edit#gid=0).

## Sprint Planning
The sprint planning notes can be accesed in this [link](https://docs.google.com/document/d/1guVZCdS4A_2YL14LjqNll8VrVJQzgvoqjZ6OoDONSNw/edit).

## Youtube Video
Presentation of the project can be found in this [link](https://youtu.be/WA5o0XrypwI)

## Installing Dependencies
This program works on a device running Ubuntu 16.04 and ROS Kinetic Kame.

To install ROS Kinetic Kame in Ubuntu 16.04, follow the steps in this [link](http://wiki.ros.org/kinetic/Installation/Ubuntu).

To install catkin, follow the installation steps in this [link](http://wiki.ros.org/catkin).

To install turtlebot_gazebo package can be installed using the following command.

```
sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```
## How to build
Open a terminal window and run the following commands

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
cd src
git clone https://github.com/harish1696/QLearning-Navigation-
cd ..
catkin_make
```

## How to run demo

Open a terminal window and run the following command to launch the learner node

```
roslaunch QLearning-Navigation- learner.launch
```
To test it in a new environment

```
roslaunch QLearning-Navigation-testrun.launch
```
## How to record bag files with launch command
The following command can be used to generate a rosbag file in the Results directory with launch command

```
roslaunch QLearning-Navigation- learner.launch record:=enable
```

The generated bag file contains recorded messages published to all topics which can be played back later.

## How to inspect rosbag file
To get more information about the generated rosbag file, go to the Results directory and run the following command

```
rosbag info pub.bag
```

## How to play a rosbag file while listener node is running 
The generated rosbag file can also be used to play the recorded messages in all topics. 

```
rosbag play pub.bag
```
 
## How to run tests
To run the tests implemented using gtest and rostest for the first time, run the following command

```
catkin_make run_tests
```

Later, the following command can be used to run the tests

```
source devel/setup.bash
rostest Qlearning-Navigation- testQlearnNav.launch
```
## Known Issues
In training phase, if you prematurely close the program, press Ctrl+C and you will displayed some random numbers. Should be fixed.
Also, the real time factor affects the performance the turtlebot both in training and testing phase.

