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

#ifndef INCLUDE_QTABLE_HPP_
#define INCLUDE_QTABLE_HPP_

/**
 *  @file    Qtable.hpp
 *  @author  Harish Sampathkumar
 *  @copyright BSD License
 *
 *  @brief Defining class Qtable
 *
 *  @section DESCRIPTION
 *
 *  Defines the data variables and data members of  
 *  class Qlearn which is used to store the Qvalues
 *  found in the form of a table so that it can be used
 *  by the turtlebot in other unknown environments.
 *  
 */
#include <iostream>
#include <vector>

class Qtable {
 public:
/**
 * @brief  Constructs a object
 */
  Qtable();

/**
 * @brief Destroys a object
 */
  ~Qtable();

/**
 * @brief Converts state from vector form to integer
 * @param Current state of the robot
 * @return Integer equivalent
 */
  int vecToint(std::vector<int> state);

/**
 * @brief Stores the trained Qtable in .csv format
 * @param Qtable
 * @return none
 */
  void convertTocsv(std::vector<std::vector<double>> Q);

 private:
  int index;  // index of a state
};

#endif  // INCLUDE_QTABLE_HPP_
