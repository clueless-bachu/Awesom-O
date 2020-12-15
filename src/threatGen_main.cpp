/**
 * @file Planner.cpp
 * @author Sneha Nayak
 * @author Vishnuu
 * @author Vasista
 * @brief
 * @date 2020-12-01
 * @copyright Copyright (c) 2020
 * 
 */
#include "threatGen.h"



int main(int argc, char** argv) {
	ros::init(argc, argv, "threatGenerator");
    ros::NodeHandle nh;
    ROS_INFO("threatGenerator node initilizing");
    bool useDetector;
    std::string arg(argv[1]);
    if(arg == "true") {
        useDetector= false;
    } else {
        useDetector = true;
    }
    ThreatGen threatGen(nh, 3, 1, useDetector);
    threatGen.generateTargets();
    ros::spin();
    return 0;
}