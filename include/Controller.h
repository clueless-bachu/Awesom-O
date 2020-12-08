/**
 * @file Controller.h
 * @author Sneha Nayak
 * @brief
 * @date 2020-12-01
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef INCLUDE_CONTROLLER_H
#define INCLUDE_CONTROLLER_H

#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

class Controller {
private:

    // Ros Node handle
    ros::NodeHandle nh_;
    
    // Subscribers
    
    // Subscribes to 
	ros::Subscriber sub_goal_;
	ros::Subscriber sub_pos_;

	// Publishers
    // Publishes 
	ros::Publisher pub_cmd_vel_;
	
	float kp_;            // P gain
    float ki_;            // I gain
    float kd_;            // D gain
    float prev_error_;      // Previous error
    double P_term_;          // Proportional term
    double I_term_;          // Integral term
    double D_term_;          // Differential term
    int Integrator, Derivator, Integrator_max, Integrator_min;

public:
	/*
	*	@brief Default constructor for Controller object
	*	@param n -  Ros node handle reference
	*/
	Controller(const ros::NodeHandle& n);
	/*
	*	@brief Default destructor
	*/
	~Controller();
	
	/*
	*	@brief 
	*	@param msg - 
	*/
	void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &data);
	/*
	*	@brief 
	*	@param msg - 
	*/
	void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void setGains(float kp, float ki, float kd);
	float control(float input);
};
#endif // INCLUDE_CONTROLLER_H