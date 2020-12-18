/**
 * @file test_Controller.cpp
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
#include <Controller.h>
/**
* @brief ControllerTest class to test controller
*/
class ControllerTest
{
private:
    ros::NodeHandle nh_;

    ros::Publisher pub_goal_, pub_pos_;
    ros::Subscriber sub_cmd_vel_, sub_goal_ack_;

public:
    bool TEST, goal_ack;
    float x_linear_vel, z_angular_vel;
    /**
    * @brief ConntrollerTest test class constructor
    */
    ControllerTest(const ros::NodeHandle &n)
        : nh_(n)
    {

        sub_cmd_vel_ = nh_.subscribe("/cmd_vel",
                                     1, &ControllerTest::CmdVelCallback, this);
        sub_goal_ack_ = nh_.subscribe("/goal_ack", 1, &ControllerTest::GoalAckCallback, this);
        pub_goal_ = nh_.advertise<geometry_msgs::Pose2D>("/move_base_simple/goal", 1);
        pub_pos_ = nh_.advertise<nav_msgs::Odometry>("/odom", 1);
    }
    /**
    * @brief CmdVelCallback that subscribes vel cmd published by controller.
    */
    void CmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        x_linear_vel = msg->linear.x;
        z_angular_vel = msg->angular.z;
    }
    /**
    * @brief GoalAckCallback subscriber to test cmd publisher 
    * @param boolean msg
    */
    void GoalAckCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        goal_ack = msg->data;
    }
    /**
    * @brief publish is a publisher to test the subscribers of Controller
    * @param odom msg, geometry msg, odom msg, num of publishers
    * @return None
    */
    void publish(nav_msgs::Odometry odom_msg, geometry_msgs::Pose2D goal_msg, int max_count)
    {
        ros::Rate loop_rate(10);
        int counter = 1;
        while (ros::ok())
        {
            pub_pos_.publish(odom_msg);
            pub_goal_.publish(goal_msg);
            ros::spinOnce();
            loop_rate.sleep();
            counter += 1;
            if (counter > max_count)
                break;
        }
    }
};
/**
* @brief Checks move bot functionality
*/
TEST(ControllerTest, checkMoveBot)
{
    ros::NodeHandle nh;
    ControllerTest controllerTest(nh);
    Controller controller(nh);

    nav_msgs::Odometry odom;
    odom.pose.pose.position.x = -1;
    odom.pose.pose.position.y = -1;

    geometry_msgs::Pose2D goal;
    goal.x = 0;
    goal.y = 0;

    controllerTest.publish(odom,goal,10);
    
    EXPECT_NO_FATAL_FAILURE(controller.move_bot(true));
}
/**
* @brief Checks CmdVel functionality
*/
TEST(ControllerTest, cmdVel)
{
    ros::NodeHandle nh;
    ControllerTest controllerTest(nh);
    Controller controller(nh);

    nav_msgs::Odometry odom;
    odom.pose.pose.position.x = -1;
    odom.pose.pose.position.y = -1;

    geometry_msgs::Pose2D goal;
    goal.x = 0;
    goal.y = 0;

    controllerTest.publish(odom,goal,10);
    controller.move_bot(true);

    EXPECT_GE(controllerTest.x_linear_vel,0);
    // EXPECT_GE(controllerTest.z_angular_vel,0);
}