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
 *  @file    Environment.hpp
 *  @author  Harish Sampathkumar
 *  @copyright BSD License
 *
 *  @brief Implementing class Environment
 *
 *  @section DESCRIPTION
 *
 *  Defines the data variables and data members of  
 *  class Environment which is used to reset simulation,
 *  find the state of turtlebot and use it to make the
 *  turtlebot learn eventually
 */

#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <vector>
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "Turtlebot.hpp"
#include "Environment.hpp"

// Constructs a object
Environment::Environment() {
  botSensor = n.subscribe<sensor_msgs::
          LaserScan>("/scan", 50, &Turtlebot::getSensorData, &learnerBot);
}

// Destroys a object
Environment::~Environment() {}

std::vector<int> Environment::resetEnvironment() {
  learnerBot.setDone();

  std_srvs::Empty reset, pause, unpause;

  ros::service::call("/gazebo/reset_world", reset);

  ros::service::call("/gazebo/unpause_physics", unpause);

  auto data = ros::topic::waitForMessage<sensor_msgs::LaserScan>
              ("/scan", n, ros::Duration(10));

  ros::service::call("/gazebo/pause_physics", pause);

  std::vector<int> state = learnerBot.getQstate();

  return state;
}

std::vector<int> Environment::performAct(int action) {
  std_srvs::Empty pause, unpause;

  ros::service::call("/gazebo/unpause_physics", unpause);

  learnerBot.publishVelocity(action);

  auto data = ros::topic::waitForMessage<sensor_msgs::LaserScan>
              ("/scan", n, ros::Duration(10));

  ros::service::call("/gazebo/pause_physics", pause);

  std::vector<int> state = learnerBot.getQstate();

  return state;
}

int Environment::rewardAct(int action) {
  int reward;
  if (learnerBot.isDone() == false) {
    if (action == 0) {
      reward = 5;
    } else {
      reward = 1;
    }
  } else {
      reward = -200;
  }
  return reward;
}


