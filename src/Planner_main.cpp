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
#include "Planner.h"


/**
* @brief main function
* @param argc, argv
* @return None
*/

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner");
    bool useVision;
    std::string arg(argv[1]);
    if(arg == "true") {
        useVision= true;
    } else {
        useVision = false;
    }
    ros::NodeHandle nh;
    ROS_INFO("Planner node initilizing");
    Planner planner(nh, useVision);
    planner.run();
    return 0;
}
