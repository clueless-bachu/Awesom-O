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
#include <math.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/LaserScan.h>

class Planner {
private:

    // Ros Node handle
    ros::NodeHandle nh_;
    
    // Subscribers
    
    // Subscribes to 
    ros::Subscriber sub_ar_pos_;
    ros::Subscriber sub_pos_;

    // Publishers
    // Publishes goal for controller node
    ros::Publisher pub_goal_;
    ros::Publisher pub_cmd_vel_;
    
    //variables
    int wayPointCount_;

public:
    /*
    *   @brief Default constructor for Planner object
    *   @param n -  Ros node handle reference
    */
    Planner(const ros::NodeHandle& n);
    /*
    *   @brief Default destructor
    */
    ~Planner();
    /*
    *   @brief 
    *   @param msg - 
    */
    void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &data);
    /*
    *   @brief 
    *   @param msg - 
    */
    void ARCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg);
    /*
    *   @brief 
    *   @param msg - 
    */
    void getNextPoint(const std::vector<std::vector<float>> &points);
    
};
#endif // INCLUDE_PLANNER_H