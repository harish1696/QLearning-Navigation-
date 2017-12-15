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
 *  @brief Defining class Turtlebot
 *
 *  @section DESCRIPTION
 *
 *  Defines the data variables and data members of  
 *  class Turtlebot which is used to collect scanner
 *  data from the turtlebot and perform the chosen
 *  action in the envirinment.
 *  
 */
#pragma once

#include <iostream>
#include <vector>
#include "ros/ros.h"

class Turtlebot {
 public:
  Turtlebot();
  ~Turtlebot();
  void getSensorData(const sensor_msgs::LaserScan::ConstPtr& msg);
  void publishVelocity(int action);
  std::vector<int> getQstate();
  bool isDone();
  void setDone();

 private:
  std::vector<int> state;
  bool done;
  ros::NodeHandle n;
  ros::Publisher velocity;
};
