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


#ifndef INCLUDE_QLEARN_HPP_
#define INCLUDE_QLEARN_HPP_

/**
 *  @file    Qlearn.hpp
 *  @author  Harish Sampathkumar
 *  @copyright BSD License
 *
 *  @brief Defining class Qlearn
 *
 *  @section DESCRIPTION
 *
 *  Defines the data variables and data members of  
 *  class Qlearn which is used to choose action of
 *  the turtlebot, update the Qtable, get Qvalue for 
 *  given state and action and choose the best action
 *  for the resultant state.
 */
#include <iostream>
#include <vector>

class Qlearn {
 public:
/**
 * @brief  Constructs a object
 */
  Qlearn(float e, float a, float d);

/**
 * @brief Overloaded Constructor for loading trained Qtable
 */
  explicit Qlearn(std::vector<std::vector<double>> Qin);

/**
 * @brief Destroys a object
 */
  ~Qlearn();

/**
 * @brief Finds the action that the bot can perform
 * @param current state of the robot
 * @return action
 */
  int chooseAction(int state);


/**
 * @brief Fetches the best possible action for the resultant state
 * @param resultant state of the turtlebot after a action is performed
 * @return action
 */ 
  int chooseNextAct(int nextstate);

/**
 * @brief Updates the Qvalue
 * @param current state of the robot, action performed, 
 * resultant state of the robot and the reward for the 
 * action taken
 * @return none
 */  
  void updateQvalue(int currstate, int action, int nextstate, int reward);

  double epsilon;
  double alpha;
  double discount;
  std::vector<std::vector<double>> Q; //Qtable

 private:
/**
 * @brief Fetches the Qvalue for given state and action
 * @param current state of the robot, action performed, 
 * @return Qvalue
 */ 
  double getQvalue(int state, int action);
};

#endif  // INCLUDE_QLEARN_HPP_
