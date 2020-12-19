/**
 * @file threatGen.h
 * @author Sneha Nayak
 * @author Vishnuu
 * @author Vasista
 * @brief
 * @date 2020-12-01
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef INCLUDE_THREAT_GEN_H
#define INCLUDE_THREAT_GEN_H

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
#include <stdio.h>
#include <stdlib.h>

/**
* @brief Class ThreatGen to generate targets randomly
*/
class ThreatGen {
 private:

    ros::NodeHandle nh;

    ros::Subscriber sub_pos;
    ros::Publisher pub_threat;
    int n;
    double threshold;
    bool useDetector;
    std::vector<std::vector<double>> threats;
 public:
/**
* @brief ThreatGen constructor.
* @param Node handle, int value for number of targets, double threshold
* bool flag
*/      
    ThreatGen(const ros::NodeHandle&, int, double, bool);
/**
* @brief ThreatGen destructor
* @param None
*/      

    ~ThreatGen();
/**
* @brief generateTargets spawns multiple targets into gazebo world
* @param bool for Test cases
*/  
    void generateTargets(bool TEST=false);
/**
* @brief poseCallBack is a call back function for ros subscriber.
* @param nav_msgs::Odometry
*/  
    void poseCallback(const nav_msgs::Odometry::ConstPtr &data);


};
#endif // INCLUDE_THREAT_GEN_H
