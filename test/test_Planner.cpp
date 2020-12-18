/**
 * @file test_Planner.cpp
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
#include <Planner.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

/**
* @brief PlannerTest is class defined to help with testing
*       of planner module
*/
class PlannerTest {
private:
    ros::NodeHandle nh_;

    ros::Publisher pub_ar_pos_, pub_pos_;
    ros::Subscriber sub_goal_;
    bool useVision_;

public:
    bool TEST, goal_ack;
    geometry_msgs::Pose2D set_point;
    /**
    * @brief PlannerTest constructor
    * @param Node handle object
    * @param useVision flag
    */
    PlannerTest (const ros::NodeHandle &n, bool useVision)
        : nh_(n) {
        useVision_ = useVision;
        sub_goal_ = nh_.subscribe("/move_base_simple/goal",
                                  1, &PlannerTest::GoalCallback, this);
        pub_pos_ = nh_.advertise<nav_msgs::Odometry>("/odom", 1);
        if (!useVision)
        {
            pub_ar_pos_ = nh_.advertise<geometry_msgs::Pose2D>("/threat", 1);
        }
        else
        {
            pub_ar_pos_ = nh_.advertise<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker", 1);
        }
    }

    /**
    * @brief GoalCallBack function is a subscriber callback to goal sent
    *  by planner
    * @param callback msg
    */
    void GoalCallback(const geometry_msgs::Pose2D::ConstPtr &data)
    {

        set_point.x = data->x;
        set_point.y = data->y;
    }
    /**
    * @brief publish is a publisher to test the subscribers of Planner
    * @param odom msg, ar_markers_msg, ar_pose_msg, max_count
    * @return None
    */
    void publish(nav_msgs::Odometry odom_msg, ar_track_alvar_msgs::AlvarMarkers ar_marker_msg, geometry_msgs::Pose2D ar_pose_msg, int max_count)
    {
        ros::Rate loop_rate(10);
        int counter = 1;
        while (ros::ok())
        {
            pub_pos_.publish(odom_msg);

            if (useVision_)
                pub_ar_pos_.publish(ar_marker_msg);
            else
                pub_ar_pos_.publish(ar_pose_msg);

            ros::spinOnce();
            loop_rate.sleep();
            counter += 1;
            if (counter > max_count)
                break;
        }
    }

};

/**
* @brief checkUseVision Test case that tests planner 
* while using vision based detection 
* @param checkUseVision flag
*/
TEST(PlannerTest, chechUseVision)
{
    ros::NodeHandle nh;
    PlannerTest plannerTest(nh, true);
    Planner planner(nh, true);
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "odom";
    static_transformStamped.child_frame_id = "ar_marker_1";
    static_transformStamped.transform.translation.x = 1;
    static_transformStamped.transform.translation.y = 1;
    static_transformStamped.transform.translation.z = 1;
    tf2::Quaternion quat;
    quat.setRPY(2, 2, 2);
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();
    static_broadcaster.sendTransform(static_transformStamped);

    ar_track_alvar_msgs::AlvarMarkers ar_marker_msg;
    ar_track_alvar_msgs::AlvarMarker ar_marker_msg_single;
    ar_marker_msg_single.id = 1;
    ar_marker_msg_single.pose.pose.position.x = 2.0;
    ar_marker_msg_single.pose.pose.position.y = -2.0;
    ar_marker_msg.markers.push_back(ar_marker_msg_single);

    nav_msgs::Odometry odom;
    odom.pose.pose.position.x = -1;
    odom.pose.pose.position.y = -1;

    geometry_msgs::Pose2D ar_pose_msg;

    plannerTest.publish(odom, ar_marker_msg, ar_pose_msg, 10);

    EXPECT_NO_FATAL_FAILURE(planner.run(true));
}
/**
* @brief checkWithoutVision Test case that tests planner 
* while not using vision based detection 
* @param checkUseVision flag
*/
TEST(PlannerTest, chechWithoutVision)
{
    ros::NodeHandle nh;
    PlannerTest plannerTest(nh, false);
    Planner planner(nh, false);
    ar_track_alvar_msgs::AlvarMarkers ar_marker_msg;
    ar_track_alvar_msgs::AlvarMarker ar_marker_msg_single;
    ar_marker_msg.markers.push_back(ar_marker_msg_single);

    nav_msgs::Odometry odom;
    odom.pose.pose.position.x = -1;
    odom.pose.pose.position.y = -1;

    geometry_msgs::Pose2D ar_pose_msg;
    ar_pose_msg.x = 0;
    ar_pose_msg.y = 0;

    plannerTest.publish(odom, ar_marker_msg, ar_pose_msg, 10);

    EXPECT_NO_FATAL_FAILURE(planner.run(true));
}
