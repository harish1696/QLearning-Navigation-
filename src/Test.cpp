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
 *  @file    Test.cpp
 *  @author  Harish Sampathkumar
 *  @copyright BSD License
 *
 *  @brief Implementing the module to test the Lookuptable
 *
 *  @section DESCRIPTION
 *
 *    
 *  
 *  
 *  
 */

#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "Environment.hpp"
#include "Qlearn.hpp"
#include "Turtlebot.hpp"
#include "Qtable.hpp"

int main(int argc, char **argv) {
  // initializing a ros node called testrun
  ros::init(argc, argv, "testrun");

  std::ifstream lookuptable;
  lookuptable.open("/home/harish/catkin_final/final.csv");
  std::vector<std::vector<double>> Q;
  std::string row, column;
  while (std::getline(lookuptable, row)) {
    std::vector<double> temp;
    std::stringstream buffer(row);
    while (std::getline(buffer, column, ',')) {
      std::stringstream temp1(column);
      double val;
      temp1 >> val;
      std::cout << val << " ";
      temp.push_back(val);
    }
    for (unsigned int i = 0; i < temp.size(); i++) {
      std::cout << temp[i] << " ";
    }
    std::cout << std::endl;
    Q.push_back(temp);
  }

  Environment agent;
  Qlearn test(Q);
  Qtable table;

  ros::Rate loop_rate(10);
  std::vector<int> currState = agent.resetEnvironment();

  int currstate = table.vecToint(currState);

  while (ros::ok()) {
    currstate = table.vecToint(currState);

    int action = test.chooseNextAct(currstate);

    int reward = agent.rewardAct(action);

    std::vector<int> nextState = agent.performAct(action);

    int nextstate = table.vecToint(nextState);

    if (reward >=0) {
        currstate = nextstate;
    } else {
        currState = agent.resetEnvironment();
        currstate = table.vecToint(currState);
    }

    ros::spinOnce();
  }
  table.convertTocsv(test.Q);
  return 0;
}



