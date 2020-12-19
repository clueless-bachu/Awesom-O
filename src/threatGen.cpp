/**
 * @file threatGen.cpp
 * @author Sneha Nayak
 * @author Vishnuu
 * @author Vasista
 * @brief
 * @date 2020-12-01
 * @copyright Copyright (c) 2020
 * 
 */
#include "threatGen.h"

ThreatGen::ThreatGen(const ros::NodeHandle &node,
 int n, double threshold, bool useDetector) : nh(node) {
    this->n = n;
    this->threshold = threshold;
    // this->generateTargets();
    this->useDetector = useDetector;
    if (this->useDetector) {
        pub_threat = nh.advertise<geometry_msgs::Pose2D>("/threat", 1);
        sub_pos = nh.subscribe("/odom", 1, &ThreatGen::poseCallback, this);
    }
}

ThreatGen::~ThreatGen() {
}

void ThreatGen::generateTargets(bool TEST) {
    for (int i = 0; i < this->n; ++i) {
        std::vector<double> rand_point =
         {(double)(rand() % 6) - 2.5, (double)(rand() % 6) - 2.5};
        threats.push_back(rand_point);
        if (TEST == false) {
            std::string command =
                "rosrun gazebo_ros spawn_model -urdf -file \
                 $(rospack find awesomo)/urdf/ar_tag" + \
                  std::to_string(i + 1) + \
                ".urdf -x " + std::to_string(rand_point[0]) + \
                 " -y " + std::to_string(rand_point[1]) + \
                  " -z 0.25 -model ar_tag" + std::to_string(i + 1);
            system(command.c_str());
        }
    }
}

void ThreatGen::poseCallback(const nav_msgs::Odometry::ConstPtr &data) {
    double xpos = data->pose.pose.position.x;
    double ypos = data->pose.pose.position.y;
    double dist = 100;
    int obs_id = -1;
    for (int i = 0; i < this->threats.size(); ++i) {
        double cur_dist = sqrt(pow((xpos - this->threats[i][0]), 2) +
                               pow((ypos - this->threats[i][1]), 2));
        if (cur_dist < this->threshold && cur_dist < dist) {
            dist = cur_dist;
            obs_id = i;
        }
    }

    if (obs_id != -1) {
        geometry_msgs::Pose2D pos;
        pos.x = this->threats[obs_id][0];
        pos.y = this->threats[obs_id][1];
        pos.theta = obs_id;
        pub_threat.publish(pos);
    }
    // ros::spinOnce();
}
