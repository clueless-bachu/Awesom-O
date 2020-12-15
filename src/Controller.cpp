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
Controller::Controller(const ros::NodeHandle &n) : nh_(n)
{
    set_point_received_ = false;
    sub_goal_ = nh_.subscribe("/move_base_simple/goal",
                              1, &Controller::GoalCallback, this);
    sub_pos_ = nh_.subscribe("/odom", 1, &Controller::PoseCallback, this);
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_goal_ack_ = nh_.advertise<std_msgs::Bool>("/goal_ack", 1);

}

/**
* @brief Destructor
* @param None
* @return None
*/
Controller::~Controller()
{
}
double Controller::euclidean_distance(geometry_msgs::Pose2D goal_pose)
{
    return sqrt(pow((goal_pose.x - pose_.x), 2) +
                pow((goal_pose.y - pose_.y), 2));
}
double Controller::steering_angle(geometry_msgs::Pose2D goal_pose)
{
    return atan2(goal_pose.y - pose_.y, goal_pose.x - pose_.x);
}

double Controller::angular_vel(geometry_msgs::Pose2D goal_pose)
{
    int constant = 1;
    double delta = (steering_angle(goal_pose) - theta_);
    if (delta > M_PI)
        delta -= 2 * M_PI;
    else if (delta <= -M_PI)
        delta += 2 * M_PI;
    return constant * delta;
}

void Controller::angular_controller(geometry_msgs::Pose2D goal_pose)
{
    geometry_msgs::Twist msg;

    while (abs(angular_vel(goal_pose)) > 0.3 && ros::ok())
    {
        ROS_INFO_STREAM("In ANGULAR CONTROLLER");

        double alpha = angular_vel(goal_pose);
        ROS_INFO_STREAM("Alpha :" << alpha);
        float PID_angle = pid_yaw_.control(alpha);

        msg.angular.z = PID_angle;
        msg.linear.x = 0.1;
        pub_cmd_vel_.publish(msg);
        // ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
    ROS_INFO_STREAM("Outside ANGULAR CONTROLLER");
    ROS_INFO_STREAM("angle :" << (abs(angular_vel(goal_pose))));
}
void Controller::distance_controller(geometry_msgs::Pose2D goal_pose)
{
    geometry_msgs::Twist msg;
    while (euclidean_distance(goal_pose) >= 0.5 && ros::ok())
    {
        angular_controller(goal_pose);
        double distance = euclidean_distance(goal_pose);

        float PID_distance = pid_x_.control(distance);
        msg.linear.x = 0.1;//PID_distance;
        pub_cmd_vel_.publish(msg);
        std_msgs::Bool finish;
        finish.data = false;
        pub_goal_ack_.publish(finish);
        // ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
    if (euclidean_distance(goal_pose) <= 0.5)
    {
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        pub_cmd_vel_.publish(msg);

        std_msgs::Bool finish;
        finish.data = true;
        pub_goal_ack_.publish(finish);
        set_point_received_ = false;
        ros::spinOnce();
    }
    ROS_INFO_STREAM("OUTTTT DISTANCE CONTROLLER");
    ROS_INFO_STREAM(euclidean_distance(goal_pose));
}
void Controller::move_bot(bool TEST)
{
    int counter = 0; 
    while (ros::ok())
    {
    
    // ROS_INFO("Inside Move bot initilized!");
    // ros::topic::waitForMessage<nav_msgs::Odometry>("/odom");

    counter++;

    if(TEST == true && counter < 10)
        return;
    if (set_point_received_)
    {
        geometry_msgs::Pose2D goal_pose;
        goal_pose.x = set_point_.x;
        goal_pose.y = set_point_.y;

        angular_controller(goal_pose);
        distance_controller(goal_pose);
    }

    ros::spinOnce();
    ros::Duration(0.1).sleep();
    }
}
/**
* @brief PoseCallback method
* @param PoseStamped msg
* @return None
*/
void Controller::PoseCallback(const nav_msgs::Odometry::ConstPtr &data)
{
    double roll, pitch;

    pose_.x = data->pose.pose.position.x;
    pose_.y = data->pose.pose.position.y;
    tf::Quaternion q(
        data->pose.pose.orientation.x,
        data->pose.pose.orientation.y,
        data->pose.pose.orientation.z,
        data->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, theta_);
}
/**
* @brief GoalCallback method
* @param PoseStamped msg
* @return None
*/
void Controller::GoalCallback(const geometry_msgs::Pose2D::ConstPtr &data)
{

    set_point_received_ = true;
    set_point_.x = data->x;
    set_point_.y = data->y;
}


