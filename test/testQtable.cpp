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
#include <vector>
#include "Qtable.hpp"

/**
 * @brief      testing if the vectToint works properly
 *
 * @param[in]     TESTSuite
 * @param[in]     testvecToint
 *
 * @return     none
 */
TEST(TESTSuite, testvecToint) {
  Qtable table;
  std::vector<int> state = {0, 0, 0, 0};
  int index = table.vecToint(state);
  EXPECT_EQ(1, index);
  state = {1, 3, 3, 3};
  index = table.vecToint(state);
  EXPECT_EQ(776, index);
  state = {5, 4, 1, 1};
  index = table.vecToint(state);
  EXPECT_EQ(282, index);
  state = {5, 5, 5, 5};
  index = table.vecToint(state);
  EXPECT_EQ(1296, index);
}
