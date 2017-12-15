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
 *  @file    Learn.cpp
 *  @author  Harish Sampathkumar
 *  @copyright BSD License
 *
 *  @brief Implementing the module to learn navigation
 *
 *  @section DESCRIPTION
 *
 *  Defines the data variables and data members of  
 *  class Environment which is used to reset simulation,
 *  find the state of turtlebot and use it to make the
 *  turtlebot learn eventually
 */

#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "Environment.hpp"
#include "Qlearn.hpp"
#include "Turtlebot.hpp"
#include "Qtable.hpp"

int main(int argc, char **argv) {
  // initializing a ros node called learner
  ros::init(argc, argv, "learner");

  Environment agent;

  Qlearn learn(0.9, 0.2, 0.8);
  double discount = 0.9986;

  Qtable table;

  int maxEpisodes = 3000;
  int maxIterations = 1000;

  ros::Rate loop_rate(10);
  
  while(ros::ok()) {
  for (int i = 0; i < maxEpisodes; i++) {
    int cumulativeReward = 0;
    std::vector<int> currState = agent.resetEnvironment();

    if (learn.epsilon > 0.05) {
      learn.epsilon *= discount;
    }

    int currstate = table.vecToint(currState);
    int j = 0;
    for (j = 0; j < maxIterations; j++) {
      int action = learn.chooseAction(currstate);

      std::vector<int> nextState = agent.performAct(action);

      int nextstate = table.vecToint(nextState);

      int reward = agent.rewardAct(action);
      cumulativeReward += reward;

      learn.updateQvalue(currstate, action, nextstate, reward);

      if (reward >=0) {
        currstate = nextstate;
      } else {
          break;
      }
      ros::spinOnce();
      if (i%100 == 0) {
        table.convertTocsv(learn.Q);
      }
    }
    std::cout << "E: " << learn.epsilon <<  "Episodes: " << i << " "
              << "Iterations: " << j << " " << "Cumulative Reward: "
              << cumulativeReward << std::endl;
  }
  table.convertTocsv(learn.Q);
  return 0;
  }
}



