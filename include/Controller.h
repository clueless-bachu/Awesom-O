/**
 * @file Controller.h
 * @author Sneha Nayak
 * @author Vishnuu
 * @author Vasista
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
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <PID.h>

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
    ros::Publisher pub_goal_ack_;
    

    bool set_point_received_ = false;
    geometry_msgs::Pose2D pose_, set_point_;
    PID pid_x_ = PID(0.7,0,0);
    PID pid_yaw_ = PID(1.4,0,0);
    double theta_;


public:
    /*
    *   @brief Default constructor for Controller object
    *   @param n -  Ros node handle reference
    */
    Controller(const ros::NodeHandle& n);
    /*
    *   @brief Default destructor
    */
    ~Controller();
    /*
    *   @brief 
    *   @param msg - 
    */
    void PoseCallback(const nav_msgs::Odometry::ConstPtr &data);
    /**
    * @brief GoalCallback method
    * @param PoseStamped msg
    * @return None
    */
    void GoalCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
    /**
    * @brief euclidean_distance calculator
    * @param goal_pose - geometry_msgs
    * @return double
    */
    double euclidean_distance(geometry_msgs::Pose2D goal_pose);
    /**
    * @brief steering_angle calculator
    * @param goal_pose - geometry_msgs
    * @return double
    */
    double steering_angle(geometry_msgs::Pose2D goal_pose);
    /**
    * @brief angular_vel calculator
    * @param goal_pose - geometry_msgs
    * @return double
    */
    double angular_vel(geometry_msgs::Pose2D goal_pose);
    /**
    * @brief angular_controller 
    * @param goal_pose - geometry_msgs
    * @return None
    */
    void angular_controller(geometry_msgs::Pose2D goal_pose);
    /**
    * @brief distance_controller 
    * @param goal_pose - geometry_msgs
    * @return None
    */
    void distance_controller(geometry_msgs::Pose2D goal_pose);
    /**
    * @brief move_bot
    * @param bool for 
    * @return None
    */
    void move_bot(bool TEST=false);
    
};
#endif // INCLUDE_CONTROLLER_H