#include "Planner.h"
Planner::Planner(const ros::NodeHandle& n):
	nh_(n)
{
    sub_ar_pos_ = nh_.subscribe("/ar_pose_marker", 1, &Planner::ARCallback, this);
    sub_pos_ = nh_.subscribe("/odom", 1, &Planner::PoseCallback, this);
    
	pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
}

Planner::~Planner() {
}

void Planner::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &data)
{
    
}
void Planner::ARCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
    
}
void Planner::getNextPoint(const std::vector<std::vector<float>> &points)
{

}

int main( int argc, char** argv ) {
	ros::init(argc, argv, "planner");
    ros::NodeHandle nh;
    Planner planner(nh);
    ROS_INFO("Planner node initiated");
    ros::spin();
    return 0;
}