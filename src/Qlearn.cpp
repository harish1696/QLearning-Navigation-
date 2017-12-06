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

#include <random>
#include "Qlearn.hpp"

using  std::cout;
using  std::endl;

//Constructs a object
Qlearn::Qlearn() {
  epsilon = 0.0;
  alpha = 0.0;
  discount = 0.0; 
}

//Destroys a object
Qlearn::~Qlearn() {}

int Qlearn::chooseAction(vector<int> state) {
  std::default_random_engine eng((std::random_device())());
  std::uniform_real_distribution<double> ddis(0, 1.0);
  std::uniform_int_distribution<int> idis(1, 3);
  float e = ddis(eng);
  
  if(e < epsilon) {
    act =  idis(eng);
    return act;
  } else {
    act = 1;
    int temp = Q(s,0);
    for(int i =1; i<2; i++) {
      if temp < Q(state,i) {
        act = i + 1;
      }
    }
    return act;
  }  
}
    
void Qlearn::updateQvalue(vector<int> currState, int action,vector<int> nextState, int reward) {
  Qcurr = getQvalue(currState,action);
  nextAction = chooseNextAct(nextstate);
  Qnext = getQvalue(nextState,nextAction);  

  Q[state, action] += alpha * (reward + discount * Qnext - Qcurr);
}

double Qlearn::getQvalue(vector<int> state, int action) {
  int Qvalue = Q(state, action);
  return Qvalue;
}

int Qlearn::chooseNextAct(vector<int> nextstate) {
  act = 1;
  int temp = Q(s,0);
  for(int i =1; i<2; i++) {
    if temp < Q(nextstate,i) {
      act = i + 1;
    }
  }
  return act; 
}
