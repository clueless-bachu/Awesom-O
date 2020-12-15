/**
 * @file Controller.cpp
 * @author Sneha Nayak
 * @author Vishnuu
 * @author Vasista
 * @brief
 * @date 2020-12-01
 * @copyright Copyright (c) 2020
 * 
 */
#include "Controller.h"


/**
* @brief main
* @param argc, argv
* @return 0
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;
    Controller controller(nh);
    controller.move_bot();
    ROS_INFO("Controller node initiated");
    ros::spin();
    return 0;
}
