/**
 * @file test_PID.cpp
 * @author Sneha Nayak
 * @author Vishnuu
 * @author Vasista
 * @brief Class tests the PID module
 * @date 2020-12-01
 * @copyright Copyright (c) 2020
 */
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <threatGen.h>
/**
* @brief ThreatGenTest is class defined to help with testing
*       of planner module
*/
class ThreatGenTest
{
private:
    ros::NodeHandle nh_;

    ros::Publisher pub_pos_;
    ros::Subscriber sub_target_;

public:
    geometry_msgs::Pose2D set_point;
    int count = 0;
    /**
    * @brief ThreatGenTest is class defined to help with testing
    *       of threatGen module
    */
    ThreatGenTest(const ros::NodeHandle &n)
        : nh_(n)
    {
        sub_target_ = nh_.subscribe("/threat",
                                  1, &ThreatGenTest::ThreatCallback, this);
        pub_pos_ = nh_.advertise<nav_msgs::Odometry>("/odom", 1);
        
    }
    /**
    * @brief ThreatCallback is class defined to help with testing
    *       of planner module
    */
    void ThreatCallback(const geometry_msgs::Pose2D::ConstPtr &data)
    {
        count++;
        set_point.x = data->x;
        set_point.y = data->y;
    }
    /**
    * @brief publish is a publisher to test the subscribers of threatGen
    * @param odom msg, ar_markers_msg, ar_pose_msg, max_count
    * @return None
    */
    void publish(nav_msgs::Odometry odom_msg, int max_count)
    {
        ros::Rate loop_rate(10);
        int counter = 1;
        while (ros::ok())
        {
            pub_pos_.publish(odom_msg);
            ros::spinOnce();
            loop_rate.sleep();
            counter += 1;
            if (counter > max_count)
                break;
        }
    }

};
/**
* @brief checkGenerate test function
* @param None
*/
TEST(ThreatGenTests, checkGenerate)
{
    ros::NodeHandle nh_;
    
    ThreatGenTest threatTest(nh_);
    ThreatGen threat(nh_, 3, 10, true);

    threat.generateTargets(true);
    nav_msgs::Odometry odom;
    odom.pose.pose.position.x = -1;
    odom.pose.pose.position.y = -1;
    threatTest.publish(odom,3);
    ros::spinOnce();
    ros::Duration(5).sleep();
    EXPECT_GT(threatTest.count,0);
}