/**
 * @file TargetGen.h
 * @author Sneha Nayak
 * @author Vishnuu
 * @author Vasista
 * @brief
 * @date 2020-12-01
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef INCLUDE_TARGETGEN_H

#define INCLUDE_TARGETGEN_H

#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <awesomo/boxes.h>

class TargetGen {
private:

    // Ros Node handle
    ros::NodeHandle nh_;
    
    // Publishers
    // Publishes 
    ros::Publisher pub_poses_;
    


public:
    /*
    *   @brief Default constructor for TargetGen object
    *   @param n -  Ros node handle reference
    */
    TargetGen(const ros::NodeHandle& n);
    /*
    *   @brief Default destructor
    */
    ~TargetGen();
    /*
    *   @brief 
    *   @param msg - 
    */
static bool getPositions(awesomo::boxes::Request &req,
        awesomo::boxes::Response &res);
    
};
#endif // INCLUDE_TARGETGEN_H