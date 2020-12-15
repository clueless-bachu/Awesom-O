#include <gtest/gtest.h>
#include <ros/ros.h>
#include <PID.h>

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