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
 *  @file    Qlearn.cpp
 *  @author  Harish Sampathkumar
 *  @copyright BSD License
 *
 *  @brief Implementing class Qlearn
 *
 *  @section DESCRIPTION
 *
 *   
 *  Implements class Qlearn which is used to choose action of
 *  the turtlebot, update the Qtable, get Qvalue for 
 *  given state and action and choose the best action
 *  for the resultant state.
 *  
 */

#include <ros/ros.h>
#include <random>
#include <vector>
#include "Qlearn.hpp"

// Constructs a object
Qlearn::Qlearn(float e, float a, float d) {
  epsilon = e;
  alpha = a;
  discount = d;
  for (int i = 0; i < 1296; i++) {
    std::vector<double> temp(3, 0.0);
    Q.push_back(temp);
  }
}

// Destroys a object
Qlearn::~Qlearn() {}

int Qlearn::chooseAction(int state) {
  std::default_random_engine eng((std::random_device())());
  std::uniform_real_distribution<double> ddis(0, 1.0);
  std::uniform_int_distribution<int> idis(0, 2);
  float e = ddis(eng);

  if (e < epsilon) {
    int act =  idis(eng);
    return act;
  } else {
    int act = 0;
    int temp = Q[state][0];
    for (int i = 1; i < 2; i++) {
      if (temp < Q[state][i]) {
        act = i;
      }
    }
    return act;
  }
}

void Qlearn::updateQvalue(int currState, int action,
              int nextState, int reward) {
  double Qcurr = getQvalue(currState, action);
  int nextAction = chooseNextAct(nextState);
  double Qnext = getQvalue(nextState, nextAction);

  double change = alpha * (reward + discount * Qnext - Qcurr);
  Q[currState][action] += change;
}

double Qlearn::getQvalue(int state, int action) {
  double Qvalue = Q[state][action];
  return Qvalue;
}

int Qlearn::chooseNextAct(int nextState) {
  int act = 0;
  double temp = Q[nextState][0];
  for (int i = 1; i < 2; i++) {
    if (temp < Q[nextState][i]) {
      act = i;
    }
  }
  return act;
}
