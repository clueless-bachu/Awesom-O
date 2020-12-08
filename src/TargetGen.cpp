#include "TargetGen.h"
TargetGen::TargetGen(const ros::NodeHandle& n):
	nh_(n)
{
    pub_poses_ = nh_.advertise<geometry_msgs::PoseStamped>("/targets", 1);
}

TargetGen::~TargetGen() {
}

std::vector<std::vector<float>> TargetGen::getPositions(int locations_count)
{
	
}



int main( int argc, char** argv ) {
	ros::init(argc, argv, "targetgen");
    ros::NodeHandle nh;
    TargetGen targetGen(nh);
    ROS_INFO("TargetGen node initiated");
    ros::spin();
    return 0;
}