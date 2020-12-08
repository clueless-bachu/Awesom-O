/**
 * @file Robot.h
 * @author Sneha Nayak
 * @brief
 * @date 2020-12-01
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef INCLUDE_ROBOT_H
#define INCLUDE_ROBOT_H

#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <sensor_msgs/LaserScan.h>

class Robot {
private:

    // Ros Node handle
    ros::NodeHandle nh_;
    
    // Subscribers
    
    // Subscribes to 
	ros::Subscriber sub_laser_;

	// Publishers
    // Publishes 
	ros::Publisher pub_repel_sig_;
	


public:
	/*
	*	@brief Default constructor for Robot object
	*	@param n -  Ros node handle reference
	*/
	Robot(const ros::NodeHandle& n);
	/*
	*	@brief Default destructor
	*/
	~Robot();
	/*
	*	@brief 
	*	@param msg - 
	*/
	void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
	
};
#endif // INCLUDE_ROBOT_H