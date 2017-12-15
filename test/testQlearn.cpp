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
#include "Qlearn.hpp"

/**
 * @brief      testing if the Qlearn works properly
 *
 * @param[in]     TESTSuite
 * @param[in]     testQlearn
 *
 * @return     none
 */
TEST(TESTSuite, testQlearn) {
  Qlearn learn(0.3, 0.7, 0.1);
  EXPECT_NEAR(0.3, learn.epsilon, 0.01);
  EXPECT_NEAR(0.7, learn.alpha, 0.01);
  EXPECT_NEAR(0.1, learn.discount, 0.01);
  EXPECT_EQ(0, learn.Q[11][1]);
  EXPECT_NE(3, learn.Q[1000][0]);
}

/**
 * @brief      testing if the choosAction works properly
 *
 * @param[in]     TESTSuite
 * @param[in]     testchooseAction
 *
 * @return     none
 */
TEST(TESTSuite, testchooseAction) {
  Qlearn learn(0, 0, 0);
  learn.Q[154][0] = 1.5;
  learn.Q[154][1] = 3.5;
  learn.Q[154][2] = 0;
  int action = learn.chooseAction(154);
  EXPECT_EQ(1, action);
  learn.Q[1010][0] = 0;
  learn.Q[1010][1] = 0;
  learn.Q[1010][2] = 0;
  action = learn.chooseAction(1010);
  EXPECT_EQ(0, action);
}

/**
 * @brief      testing if the choosNextAct works properly
 *
 * @param[in]     TESTSuite
 * @param[in]     testchooseNextAct
 *
 * @return     none
 */
TEST(TESTSuite, testchooseNextAct) {
  Qlearn learn(0, 0, 0);
  learn.Q[154][0] = 1.5;
  learn.Q[154][1] = 3.5;
  learn.Q[154][2] = 0;
  int action = learn.chooseNextAct(154);
  EXPECT_EQ(1, action);
  learn.Q[1010][0] = 0;
  learn.Q[1010][1] = 0;
  learn.Q[1010][2] = 0;
  action = learn.chooseNextAct(1010);
  EXPECT_EQ(0, action);
}


/**
 * @brief      testing if the updateQlvalue works properly
 *
 * @param[in]     TESTSuite
 * @param[in]     testupdateQvalue
 *
 * @return     none
 */
TEST(TESTSuite, testupdateQvalue) {
  Qlearn learn(0.9, 0.2, 0.8);
  learn.Q[10][0] = 10;
  learn.Q[100][0] = 4;
  learn.Q[100][1] = 6;
  learn.Q[100][2] = 2;
  learn.updateQvalue(10, 0, 100, 5);
  EXPECT_NEAR(9.96, learn.Q[10][0], 0.01);
}
