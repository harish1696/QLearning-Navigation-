# Autonomous Navigation using QLearning
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![Build Status](https://travis-ci.org/harish1696/QLearning-Navigation-.svg?branch=master)](https://travis-ci.org/harish1696/QLearning-Navigation-)

## Overview

The module will be able to train a robot to navigate autonomously in a completely unknown environment by avoiding obstacles using Q-learning technique. A model world will be simulated in Gazebo and the robot will be allowed to navigate in it without any knowledge about the world. A positive reward is given to states clear of obstacles while negative reward is given to states that is proportional to the proximity of an obstacle. The robot is allowed to navigate the world repeatedly until Q value table is updated to achieve an optimal policy. With the help of the Q table generated, the robot can navigate in any other random world/environment by avoiding obstacles.

## License
The BSD license definition for this project can be viewed [here](https://opensource.org/licenses/BSD-3-Clause)

## SIP Process
SIP Process is followed to develop the module. It is detailed in this [link](https://docs.google.com/spreadsheets/d/1iwXafoxuYP-64WJcZ8xOhcCnD-_6G_7DA12rQynheLY/edit#gid=0).

## Sprint Planning
The sprint planning notes can be accesed in this [link](https://docs.google.com/document/d/1guVZCdS4A_2YL14LjqNll8VrVJQzgvoqjZ6OoDONSNw/edit).

## Youtube Video
Presentation of the project can be found in this [link](https://youtu.be/WA5o0XrypwI)

## Installing Dependencies
This program works on a device running Ubuntu 16.04 and ROS Kinetic Kame.

To install ROS Kinetic Kame in Ubuntu 16.04, follow the steps in this [link](http://wiki.ros.org/kinetic/Installation/Ubuntu).

To install catkin, follow the installation steps in this [link](http://wiki.ros.org/catkin)

TO install Gazebo, follow the installation steps in this [link](http://gazebosim.org/tutorials?tut=install_ubuntu&ver=7.0)

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
Before launching the learner and testrun nodes which performs the learning and testing respectively, the current version needs the user to go the QLearning-Navigation package and make the following changes so that the Qtable gets stored at the desired directory and loaded from the desired directory the user wants.

```
cd ~/catkin_ws/src/QLearning-Navigation-/src
gedit Qtable.cpp
```
In that file go to line 71 and change the path to where you want to store the trained computer in your Qtable.

Close the gedit file.

```
gedit Test.cpp
```
In that file go to line 69 and change the path to where you want to load the Qtable from.
Note: Absolute path should be mentioned as shown.

Once the necessary changes are made run the following commands to launch the learner node.
   
Open a terminal window and run the following command to launch the learner node

```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch QLearning-Navigation- learner.launch
```
To view the simulation in Gazebo, open a new terminal window and run the following command

```
gzclient
```

This starts the learner node and it starts updating the Qtable. You can the reward obtained for each episode in the terminal window.
Also, Closing gazebo doesn't close the node.
Note:You can close the node by pressing Ctrl+C.

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
THe following issues can be fixed in the following versions
1. In training phase, if you prematurely close the program using Ctrl+C you will displayed some random numbers.
2. This version requires you to manually change the path in the source file to store and load the Qtable into the nodes. 

The following issue is due to system graphic processing power at the time of running simulation.
1. The real time factor affects the performance the turtlebot both in training and testing phase. For example, it is possible to change the real_time_update_rate in the .world file to speed up the simulation. But this backfires when the simulation speeds up like (6-12x). The .world file are now to set with real_time_update_rate value of 100 which has been tested in different systems.

## How to generate Doxygen documentation

```
doxygen ./Doxygen
```
## About Me
Harish Sampathkumar
Major: Robotics, UMD
Mail: hsampath@umd.edu
