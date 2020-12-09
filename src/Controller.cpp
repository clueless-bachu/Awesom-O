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
* @brief Constructor
* @param NodeHandle function
* @return None
*/
Controller::Controller(const ros::NodeHandle& n):
    nh_(n) {
    sub_goal_ = nh_.subscribe("/move_base_simple/goal",
     1, &Controller::GoalCallback, this);
    sub_pos_ = nh_.subscribe("/odom", 1, &Controller::PoseCallback, this);
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

/**
* @brief Destructor
* @param None
* @return None
*/
Controller::~Controller() {
}

/**
* @brief PoseCallback method
* @param PoseStamped msg
* @return None
*/
void Controller::PoseCallback
    (const geometry_msgs::PoseStamped::ConstPtr &data) {
}
/**
* @brief GoalCallback method
* @param PoseStamped msg
* @return None
*/
void Controller::GoalCallback
    (const geometry_msgs::PoseStamped::ConstPtr &data) {
}
/**
* @brief setGains method
* @param float values
* @return None
*/
void Controller::setGains(float kp, float ki, float kd) {
}
/**
* @brief control
* @param float
* @return float
*/
float Controller::control(float input) {
    return 0.0;
}


/**
* @brief main
* @param argc, argv
* @return 0
*/
int main(int argc, char** argv) {
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;
    Controller controller(nh);
    ROS_INFO("Controller node initiated");
    ros::spin();
    return 0;
}
