/**
 * @file TargetGen.cpp
 * @author Sneha Nayak
 * @author Vishnuu
 * @author Vasista
 * @brief
 * @date 2020-12-01
 * @copyright Copyright (c) 2020
 * 
 */
#include "TargetGen.h"

bool TargetGen::getPositions(awesomo::boxes::Request &req,
    awesomo::boxes::Response &res) {
    int num = req.numBoxes;
    // vector<float> positions;
    for (auto i = 0; i < 2*num; i++) {
        res.positions.push_back
                ((static_cast<float>(std::rand())/(RAND_MAX))*(5) - 2.5);
    }
    // res.positions = positions;
    return true;
}

TargetGen::TargetGen(const ros::NodeHandle& n):
    nh_(n) {
    pub_poses_ = nh_.advertise<geometry_msgs::PoseStamped>
                                            ("/targets", 1);
    ros::ServiceServer service = nh_.advertiseService
                        ("getPositions", this->getPositions);
    ROS_INFO("Inside constructor");
}

TargetGen::~TargetGen() {
}
/**
* @brief Main file for targetGeneration
* @param argc, argv
* @return int
*/
int main(int argc, char** argv) {
    ros::init(argc, argv, "targetgen");
    ros::NodeHandle nh;
    TargetGen targetGen(nh);
    ROS_INFO("TargetGen node initiated");
    ROS_INFO("TargetGen Service active");
    ros::spin();
    return 0;
}
