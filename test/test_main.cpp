/**
 *  MIT License
 *
 *  Copyright (c) 2020 Sneha Nayak, Vasista Ayyagari, Vishnuu A. Dhanabalan
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 */

/**
 *@file       test_main.cpp
 *@author     Sneha Nayak
 *@copyright  MIT License
 *@brief      Node test
 */
#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>




/**
 * @brief      Tests whether the service exists and then changes string.
 * @param      testNode         gtest framework type
 * @param      testResponse        name of the test
 */
TEST(testNode, testResponse) {
  // Create ros node handle.
  ros::NodeHandle node;

  EXPECT_EQ("Test", "Test");
}

/**
* @brief main
* @param argc, argv
* @return None
*/
int main(int argc, char** argv) {
    ros::init(argc, argv, "testAwesomO");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
