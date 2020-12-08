#include "Controller.h"
Controller::Controller(const ros::NodeHandle& n):
	nh_(n)
{
	sub_goal_ = nh_.subscribe("/move_base_simple/goal", 1, &Controller::GoalCallback, this);
    sub_pos_ = nh_.subscribe("/odom", 1, &Controller::PoseCallback, this);
    
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

Controller::~Controller() {
}

void Controller::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &data)
{
    
}
void Controller::GoalCallback(const geometry_msgs::PoseStamped::ConstPtr &data)
{
    
}

void Controller::setGains(float kp, float ki, float kd)
{

}
float Controller::control(float input) {
	return 0.0;
};


int main( int argc, char** argv ) {
	ros::init(argc, argv, "controller");
    ros::NodeHandle nh;
    Controller controller(nh);
    ROS_INFO("Controller node initiated");
    ros::spin();
    return 0;
}