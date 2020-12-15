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
 	ThreatGen(const ros::NodeHandle&, int, double, bool);
 	~ThreatGen();
 	void generateTargets(bool TEST=false);
 	void poseCallback(const nav_msgs::Odometry::ConstPtr &data);


};
#endif // INCLUDE_THREAT_GEN_H
