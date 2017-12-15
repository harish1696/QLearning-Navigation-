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
 *  @file    test.cpp
 *  @author  Harish Sampathkumar
 *  @copyright BSD License
 *
 *  @brief Implementing test cases 
 *
 *  @section DESCRIPTION
 *
 *  This program establishes test cases
 *  
 *
 */
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include "Environment.hpp"
#include "Turtlebot.hpp"

/**
 * @brief      testing if the rewardAct works properly
 *
 * @param[in]     TESTSuite
 * @param[in]     testrewardAct
 *
 * @return     none
 */
TEST(TESTSuite, testrewardAct) {
  Environment agent;
  int action = 0;
  int reward = agent.rewardAct(action);
  EXPECT_EQ(5, reward);
  action = 1;
  reward = agent.rewardAct(action);
  EXPECT_EQ(1, reward);
  action = 2;
  reward = agent.rewardAct(action);
  EXPECT_EQ(1, reward);
  agent.learnerBot.setDone();
  action = 0;
  reward = agent.rewardAct(action);
  EXPECT_NE(-200, reward);
}
