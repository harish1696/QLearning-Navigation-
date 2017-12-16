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
 *  @file    Qtable.cpp
 *  @author  Harish Sampathkumar
 *  @copyright BSD License
 *
 *  @brief Implements class Qtable
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
#include <fstream>
#include <vector>
#include "Qtable.hpp"

Qtable::Qtable() {
  index = 0;
}

Qtable::~Qtable() {}

int Qtable::vecToint(std::vector<int>state) {
  index = 216 * (state[3]) + 36  * (state[2])
              + 6 * (state[1]) + (state[0] + 1);
  return index;
}

void Qtable::convertTocsv(std::vector<std::vector<double>> Q) {
  typedef std::vector<int>::size_type vec_size;
  std::ofstream output;
  // replace the path below to where you want the Qtable stored
  output.open("/home/harish/catkin_final/Lookuptable.csv");
  vec_size Qsize = Q.size();
  for (unsigned int i = 0; i < Qsize; i++) {
      output << Q[i][0] << "," << Q[i][1]
                  << "," << Q[i][2] << std::endl;
  }
}


