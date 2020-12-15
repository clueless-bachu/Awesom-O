/**
 * @file Planner.h
 * @author Sneha Nayak
 * @author Vishnuu
 * @author Vasista
 * @brief
 * @date 2020-12-01
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef INCLUDE_PLANNER_H
#define INCLUDE_PLANNER_H

#include <ros/ros.h>
#include <iostream>
#include <queue>
#include <set>
#include <vector>
#include <math.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <tf/tf.h> 
#include <tf/transform_datatypes.h>

class Planner {
private:

    // Ros Node handle
    ros::NodeHandle nh_;
    
    // Subscribers
    
    // Subscribes to 
    ros::Subscriber sub_ar_pos_;
    ros::Subscriber sub_pos_;
    ros::Subscriber sub_reached_flag_;

    // Publishers
    // Publishes goal for controller node
    ros::Publisher pub_goal_;
    ros::Publisher pub_cmd_vel_;
    
    //variables
    int wayPointCount_;
    std::queue<geometry_msgs::Pose2D> targets_queue_;
    std::set<int> detected_targets_;
    geometry_msgs::PoseStamped curr_pose_;
    tf::TransformListener listener_;
    geometry_msgs::Point robot_position_;
    std::queue<geometry_msgs::Pose2D> waypoints_;
    bool reached_target_flag_;
    bool mission_complete_;
    bool useVision;
    int num_targets_;

    float x_pos, y_pos;

public:
    /*
    *   @brief Default constructor for Planner object
    *   @param n -  Ros node handle reference
    */
    Planner(const ros::NodeHandle&, bool);
    /*
    *   @brief Default destructor
    */
    ~Planner();
    /*
    *   @brief 
    *   @param msg - 
    */
    void aRCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);
    /*
    *   @brief 
    *   @param msg - 
    */
    void threatCallback(const geometry_msgs::Pose2D::ConstPtr &data);
    /*
    *   @brief 
    *   @param msg - 
    */
    void run(bool TEST=false);
    /*
    *   @brief 
    *   @param msg - 
    */
    void flagCallBack(const nav_msgs::Odometry::ConstPtr &data);
};
#endif // INCLUDE_PLANNER_H