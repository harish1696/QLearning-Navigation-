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

#ifndef INCLUDE_TURTLEBOT_HPP_
#define INCLUDE_TURTLEBOT_HPP_

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
#include <iostream>
#include <vector>
#include "ros/ros.h"

class Turtlebot {
 public:
/**
 * @brief  Constructs a object
 */
  Turtlebot();

/**
 * @brief Destroys a object
 */
  ~Turtlebot();

/**
 * @brief Callback function for topic /scan
 * @param LaserScan data
 * @return none
 */
  void getSensorData(const sensor_msgs::LaserScan::ConstPtr& msg);

/**
 * @brief Publishes velocity to the robot 
 * @param action to be performed
 * @return none
 */
  void publishVelocity(int action);

/**
 * @brief obtains the state of the robot
 * @param none
 * @return state in vector form
 */
  std::vector<int> getQstate();

/**
 * @brief checks if the robot crashes
 * @param none
 * @return true if robot crashes, otherwise false
 */
  bool isDone();

/**
 * @brief sets the Done variable to False
 * @param none
 * @return none
 */
  void setDone();

 private:
  std::vector<int> state = {0, 0, 0, 0};  // holds state of the robot
  bool done;  // true if robot crashes else false
  ros::NodeHandle n;
  ros::Publisher velocity;  // publish velocity to robot
};

#endif  // INCLUDE_TURTLEBOT_HPP_
