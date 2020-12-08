#include "Robot.h"
Robot::Robot(const ros::NodeHandle& n):
	nh_(n)
{
    

	// sub_mavros_rcin_ = nh_.subscribe<mavros_msgs::RCIn>("/mavros/rc/in", 10, &Robot::rcInCallback, this);  
	// sub_laser_ = nh_.subscribe<sensor_msgs::LaserScan>("/cat_dog/corrected_lidar_scan", 10, &Robot::laserCallback, this);
	// pub_repel_sig_ = nh_.advertise<cat_dog::HighLevelCommands>("/cat_dog/obstacle_avoid_signal", 10);
	// pub_debug = nh_.advertise<sensor_msgs::LaserScan>("/cat_dog/avoid_debug", 10);
}

Robot::~Robot() {
}

void Robot::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
	
}



int main( int argc, char** argv ) {
	ros::init(argc, argv, "robot");
    ros::NodeHandle nh;
    Robot robot(nh);
    ROS_INFO("Robot node initiated");
    ros::spin();
    return 0;
}