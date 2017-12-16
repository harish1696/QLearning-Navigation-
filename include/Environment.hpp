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

#ifndef INCLUDE_ENVIRONMENT_HPP_
#define INCLUDE_ENVIRONMENT_HPP_

/**
 *  @file    Environment.hpp
 *  @author  Harish Sampathkumar
 *  @copyright BSD License
 *
 *  @brief Defining class Environment
 *
 *  @section DESCRIPTION
 *
 *  Defines the data variables and data members of  
 *  class Environment which is used to reset simulation,
 *  find the state of turtlebot and use it to make the
 *  turtlebot learn eventually
 */
#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "Turtlebot.hpp"

class Environment {
 public:
/**
  * @brief  Constructs a object
  */
  Environment();

/**
 * @brief Destroys a object
 */
  ~Environment();

/**
 * @brief Resets the simulation each time robot crashes
 * @param none
 * @return initial state of the robot after it respawns
 */
  std::vector<int> resetEnvironment();

/**
 * @brief Performs the chosen action on the robot 
 * @param action to be performed
 * @return  state of the robot after it performs action
 */
  std::vector<int> performAct(int action);

/**
 * @brief Finds the reward for the performed act
 * @param action performed
 * @return reward obtained
 */
  int rewardAct(int action);

  Turtlebot learnerBot;  // robot in the environemnt
  std::vector<int> currState;  // state of the robot before action
  std::vector<int> nextState;  // state of the robot after action

 private:
  ros::NodeHandle n;
  ros::Subscriber botSensor;  // collects the scan data
};

#endif  // INCLUDE_ENVIRONMENT_HPP_
