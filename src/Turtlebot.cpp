/**
 * BSD 3-Clause LICENSE
 *
 * Copyright <2017> <HARISH SAMPATHKUMAR>
 *
 * Redistribution and use in source and binary forms, with or without  
 * modification, are permitted provided that the following conditions are 
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright 
 * notice, this list of conditions and the following disclaimer in the   
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its 
 * contributors may be used to endorse or promote products derived from this 
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS 
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  @file    Turtlebot.hpp
 *  @author  Harish Sampathkumar
 *  @copyright BSD License
 *
 *  @brief Implementing class Turtlebot
 *
 *  @section DESCRIPTION
 *
 *  Defines the data variables and data members of  
 *  class Turtlebot which is used to collect scanner
 *  data from the turtlebot and perform the chosen
 *  action in the envirinment.
 *  
 */

#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <vector>
#include "geometry_msgs/Twist.h"
#include "Turtlebot.hpp"

// constructor of walker object
Turtlebot::Turtlebot() {
  state = {0, 0, 0, 0};
  done = false;
  velocity = n.advertise<geometry_msgs::Twist>
                             ("/mobile_base/commands/velocity", 1000);
}

// destructor of walker object
Turtlebot::~Turtlebot() {}

void Turtlebot::getSensorData(const sensor_msgs::LaserScan::ConstPtr& msg) {
  int mod = msg->ranges.size()/4;
  int k = 0;
  for (int i = 0; i < msg->ranges.size(); ++i) {
    if (msg->ranges[i] < 0.6) {
      done = true;
      return;
    }
    if (i%mod == 0) {
      // ROS_INFO_STREAM(msg->ranges[i]);
      if (msg->ranges[i] >= 5) {
        state[k] = 5;
      } else if (std::isnan(msg->ranges[i])) {
        state[k] = 0;
      } else {
        state[k] = round(msg->ranges[i]);
      }
      k += 1;
    }
  }
  done = false;
  return;
}

std::vector<int> Turtlebot::getQstate() {
  std::vector<int> Qstate = state;
  return Qstate;
}

// callback function of subscriber
void Turtlebot::publishVelocity(int action) {
    geometry_msgs::Twist msg;
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;

    if (action == 0) {
      msg.linear.x = 0.2;
    }
    if (action == 1) {
      msg.linear.x = 0.05;
      msg.angular.z = 0.3;
    }
    if (action == 2) {
      msg.linear.x = 0.05;
      msg.angular.z = -0.3;
    }

  velocity.publish(msg);
}

bool Turtlebot::isDone() {
  return done;
}

void Turtlebot::setDone() {
  done = false;
}

