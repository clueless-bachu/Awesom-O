/**
 * @file test_PID.cpp
 * @author Sneha Nayak
 * @author Vishnuu
 * @author Vasista
 * @brief Class tests the PID module
 * @date 2020-12-01
 * @copyright Copyright (c) 2020
 * 
 */
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <PID.h>

/**
* @brief checPID TestCase
* @param None
*/
TEST(PIDTest, checkPID)
{

    PID pid = PID(1.3, 0, 0);
    float input=2.0;
    for (int i = 0; i < 10; i++)
    {
        
        EXPECT_GE(pid.control(input),0);
        input = input/0.5;
    }
}